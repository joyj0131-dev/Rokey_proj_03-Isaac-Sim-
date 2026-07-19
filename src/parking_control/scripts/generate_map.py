#!/usr/bin/env python3
"""주차장 환경 파라미터 → config/parking_map.yaml 생성기.

좌표의 유일한 원천. 주차장 레이아웃이 바뀌면 여기 파라미터만 고치고
다시 실행한다 — 다른 어떤 코드에도 좌표를 하드코딩하지 않는다.

기본값은 Isaac_envo의 build_parking_environment.py 상수 스냅샷(2026-07-19)이며,
레이아웃 변경 가능성이 있으므로 전부 CLI 인자로 덮어쓸 수 있다.

좌표 규약: ROS map 프레임 (x, y). USD(Y-up)와의 변환은 ros_x = usd_x,
ros_y = -usd_z (Isaac Sim ROS2 브리지 기본 규약, 이식 후 실측 검증 필요).

실행:
    python3 generate_map.py                      # 기본 파라미터로 생성
    python3 generate_map.py --space-width 3.2    # 레이아웃 변경 반영 예시
"""

import argparse
import math
from pathlib import Path

import yaml

DEFAULT_OUTPUT = Path(__file__).resolve().parent.parent / "config" / "parking_map.yaml"


def build_map(space_count=10, parking_start=1, parking_end=8,
              space_width=3.40, space_length=6.60, aisle_width=9.00,
              border_margin=1.10):
    """환경 파라미터에서 노드/엣지/존을 계산한다."""
    half_w = space_count * space_width * 0.5
    # 슬롯 중심의 USD z (통로 중심선 기준 거리) → ROS y = -usd_z
    row_center = aisle_width * 0.5 + space_length * 0.5

    nodes = {}
    edges = []
    zones = []

    # 중앙 통로 분기점: 슬롯 열 경계마다 하나 (존 경계와 1:1 대응)
    for k in range(space_count + 1):
        nodes[f"J{k}"] = dict(x=round(-half_w + k * space_width, 3), y=0.0,
                              kind="junction")
    for k in range(space_count):
        zone_id = f"Z{k + 1:02d}"
        zones.append(zone_id)
        edges.append(dict(u=f"J{k}", v=f"J{k + 1}", zone=zone_id))

    # 차량 출입구 (서쪽 벽 중앙)
    nodes["entrance"] = dict(x=round(-half_w - border_margin, 3), y=0.0,
                             kind="entrance")
    edges.append(dict(u="entrance", v="J0", zone="Z_ENTRANCE"))
    zones.append("Z_ENTRANCE")

    # 주차 슬롯 + 로봇 대기/충전 도크. A행 usd z=+row_center → ros y=-row_center.
    special = {0: ("dock_wait", "waiting"),
               space_count - 1: ("dock_charge", "charging")}
    for row_name, usd_z_sign in (("A", 1.0), ("B", -1.0)):
        y = round(-usd_z_sign * row_center, 3)
        for index in range(space_count):
            x = round(-half_w + (index + 0.5) * space_width, 3)
            if index in special:
                prefix, role = special[index]
                node_id = f"{prefix}_{row_name}"
                nodes[node_id] = dict(x=x, y=y, kind="dock", role=role)
            elif parking_start <= index <= parking_end:
                node_id = f"{row_name}{index}"
                nodes[node_id] = dict(x=x, y=y, kind="slot",
                                      accessible=node_id in ("A1", "A2"))
            else:
                continue
            # 슬롯/도크는 양옆 분기점 두 곳에 연결한다. 통로 엣지를 쪼개지
            # 않아야 존(통로 구간)과 엣지의 1:1 대응이 유지된다.
            edges.append(dict(u=node_id, v=f"J{index}"))
            edges.append(dict(u=node_id, v=f"J{index + 1}"))

    return dict(
        meta=dict(
            generated_by="generate_map.py",
            frame="ros_map",
            usd_to_ros="ros_x = usd_x, ros_y = -usd_z (이식 후 실측 검증 필요)",
            params=dict(space_count=space_count, parking_start=parking_start,
                        parking_end=parking_end, space_width=space_width,
                        space_length=space_length, aisle_width=aisle_width,
                        border_margin=border_margin),
        ),
        nodes=nodes,
        edges=edges,
        zones=sorted(zones),
    )


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--space-count", type=int, default=10)
    parser.add_argument("--space-width", type=float, default=3.40)
    parser.add_argument("--space-length", type=float, default=6.60)
    parser.add_argument("--aisle-width", type=float, default=9.00)
    parser.add_argument("--border-margin", type=float, default=1.10)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    args = parser.parse_args()

    data = build_map(space_count=args.space_count, space_width=args.space_width,
                     space_length=args.space_length, aisle_width=args.aisle_width,
                     border_margin=args.border_margin)

    args.output.parent.mkdir(parents=True, exist_ok=True)
    with open(args.output, "w") as f:
        yaml.safe_dump(data, f, allow_unicode=True, sort_keys=False)

    slots = {k: v for k, v in data["nodes"].items() if v["kind"] == "slot"}
    print(f"생성 완료: {args.output}")
    print(f"노드 {len(data['nodes'])}개, 엣지 {len(data['edges'])}개, "
          f"존 {len(data['zones'])}개, 슬롯 {len(slots)}개")
    for name in sorted(slots):
        print(f"  {name}: ({slots[name]['x']:+.1f}, {slots[name]['y']:+.1f})")


if __name__ == "__main__":
    main()
