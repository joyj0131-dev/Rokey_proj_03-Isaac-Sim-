#!/usr/bin/env python3
"""주차장 환경(v4, 3슬롯 A1/A2/A3) → config/parking_map.yaml 생성기.

좌표의 유일한 원천. 주차장 레이아웃이 바뀌면 여기 상수만 고치고 다시
실행한다 — 다른 어떤 코드에도 좌표를 하드코딩하지 않는다.

2026-07-25: 좌표 원천을 ~/Downloads/parking_environment_v3.usd(대략치)에서
실제 배포에 쓰이는 isaacpjt/Isaac_envo/parking/parking_environment_v4.usd의
ArucoMarkerPreview 스코프 실측값으로 교체했다. 그리고 **ENTRY/EXIT를 반대로
잘못 배정했던 버그를 고쳤다**: v4 에셋의 aruco:note는 z 음수 쪽을
"입차"로, z 양수 쪽을 "출차"로 표기하지만, 이는 에셋 제작자의 라벨일 뿐이고
우리 팀이 실제로 채택한 규약은 정반대다(src/parkbot_aruco/parkbot_aruco/
site_map_v4.py 참고: "사용자 규약: 입차(ENTRY) = z 양수, 출차(EXIT) = z 음수").
이전 버전은 에셋 note 텍스트만 보고 z 음수 쪽(GATE_IN/W_IN/D_IN_*/XS)을
그대로 ENTRY로 옮겨 적어 전체가 뒤집혀 있었다 — site_map_v4.py를
유일한 변환 기준점으로 삼아 다시 옮겼다.

핵심 구조: 입차와 출차가 완전히 분리된 차로를 쓴다.
  - 입차(z 양수, GATE_OUT/W_OUT/D_OUT_*/XN 마커 기반): 인계베이 → 입차 게이트
    → 입차 차로(로봇이 슬롯 열 쪽 진입 분기점 A1'/A2'/A3'까지 이동) → 슬롯
    (A1/A2/A3, z가 음수인 슬롯 열까지 마지막 한 구간을 가로질러 진입)
  - 출차(z 음수, GATE_IN/W_IN/D_IN_*/XS 마커 기반): 슬롯(A1/A2/A3 마커 자체가
    "슬롯 기준점 + 통로 차선"을 겸함) → 출차 차로 → 출차 게이트 → 인계베이
  각 차로에 로봇 대기 도크가 2개씩 있다(입차 전용 2대 / 출차 전용 2대에 대응,
  로봇 ID는 site_map_v4.ROBOTS: entry_lead/entry_follow/exit_lead/exit_follow).

좌표 규약: ROS map 프레임 (x, y). USD(Y-up)와의 변환은 ros_x = usd_x,
ros_y = -usd_z (Isaac Sim ROS2 브리지 기본 규약) — v4.usd의
ArucoMarkerPreview 스코프 aruco:position을 이 규약으로 변환해 아래 상수에
그대로 옮겼다.

실행:
    python3 generate_map.py
"""

import math
from pathlib import Path

import yaml

PKG_ROOT = Path(__file__).resolve().parent.parent
DEFAULT_OUTPUT = PKG_ROOT / "config" / "parking_map.yaml"
DEFAULT_SEED_SQL = PKG_ROOT / "db" / "002_seed.sql"


def _usd_to_map(x_usd, z_usd):
    return round(x_usd, 3), round(-z_usd, 3)


# ---- v4.usd 실측 좌표 (USD x,z) — ArucoMarkerPreview 스코프, aruco:position 기준 ----
# 슬롯 실제 위치는 아루코 마커(A1/A2/A3, z=-6.875)가 아니라 World/Spaces 스코프의
# 진짜 슬롯 지오메트리(BlueFloor 큐브, parking:center)를 써야 한다 — 2026-07-25
# 확인: parking:center=(2.8/6.2/9.6, 0, 0), parking:length=6.6, parking:width=3.4.
# 아루코 A1/A2/A3 마커는 슬롯 근처의 기준점일 뿐 슬롯 중심이 아니다(z=-6.875는
# 마커 자체 위치이지 슬롯 중심 z=0이 아님) — 실제 스크린샷과 대조해 확인했다
# (마커 z 그대로 쓰면 슬롯이 출차 쪽 끝에 붙어버리는데, 실제로는 입/출차 사이
# 정중앙에 있다).
SLOTS_USD = {"A1": (2.8, 0.0), "A2": (6.2, 0.0), "A3": (9.6, 0.0)}
# 입차 차로 진입 분기점(A1'/A2'/A3' 마커, z 양수) — 슬롯과 같은 x, 반대쪽 z.
# 로봇이 여기서 슬롯(z=0, World/Spaces 실제 중심)까지 마지막 구간을 곧장 들어간다.
ENTRY_SLOT_JUNCTION_USD = {"A1": (2.8, 6.875), "A2": (6.2, 6.875), "A3": (9.6, 6.875)}
# 출차 차로 진입 분기점 — XS(crossing_exit, x=-2.5)와 같은 z(=CROSSING_EXIT_USD의 z)에서
# 슬롯 x로 먼저 이동한 뒤 슬롯(z=0)으로 곧장 들어간다. v4.usd에 이 지점을 위한 별도
# 아루코 마커는 없다(A1/A2/A3 마커 자체가 "통로 차선"을 겸한다고 note에 적혀 있었지만,
# 슬롯 실제 중심이 z=0으로 확인된 이상 그 마커 z=-6.875를 차로 좌표로 재사용한다 —
# crossing_exit와 같은 z라 두 점이 일직선이 되어 입차 쪽과 대칭인 ㄱ자 경로가 나온다.
EXIT_SLOT_JUNCTION_USD = {"A1": (2.8, -6.875), "A2": (6.2, -6.875), "A3": (9.6, -6.875)}
ENTRY_GATE_USD = (-12.55, 7.075)     # GATE_OUT 마커 (z 양수 = 입차 규약)
ENTRY_WAIT_USD = (-8.5, 7.075)       # W_OUT 마커 — 인계(핸드오프) 지점
ENTRY_OUTER_USD = (-21.0, 7.075)     # 게이트 밖(건물 밖) — 정밀 마커 없어 게이트 연장
EXIT_GATE_USD = (-12.55, -7.075)     # GATE_IN 마커 (z 음수 = 출차 규약)
EXIT_WAIT_USD = (-8.5, -7.075)       # W_IN 마커 — 인계(핸드오프) 지점
EXIT_OUTER_USD = (-21.0, -7.075)     # 게이트 밖(건물 밖) — 정밀 마커 없어 게이트 연장
# 입차 전용 로봇 대기 도크 2개(D_OUT_1/2), 출차 전용 2개(D_IN_1/2).
DOCK_ENTRY_1_USD = (-3.2, 2.2)
DOCK_ENTRY_2_USD = (-1.2, 2.2)
DOCK_EXIT_1_USD = (-3.2, -2.2)
DOCK_EXIT_2_USD = (-1.2, -2.2)
# XN/XS(아루코 "통로 분기점") — 입/출차 차로에서 슬롯 열/도크 쪽으로 갈라지는 지점.
CROSSING_ENTRY_USD = (-2.5, 6.875)    # XN 마커 (z 양수 = 입차 규약)
CROSSING_EXIT_USD = (-2.5, -6.875)    # XS 마커 (z 음수 = 출차 규약)


def build_map():
    """v3.usd 실측 좌표에서 노드/엣지/존을 만든다(파라미터 없음 — 손으로 설계된 특정 배치)."""
    nodes = {}
    edges = []
    zones = []

    def add_node(node_id, x_usd, z_usd, **attrs):
        x, y = _usd_to_map(x_usd, z_usd)
        nodes[node_id] = dict(x=x, y=y, **attrs)

    def add_edge(u, v, zone_id=None):
        if zone_id:
            zones.append(zone_id)
        edges.append(dict(u=u, v=v, zone=zone_id))

    # ---- 슬롯 3개(단일 열, A1/A2/A3) — 물리적으로 z 음수(출차 규약) 쪽에 있다 ----
    for slot_id, (x_usd, z_usd) in SLOTS_USD.items():
        add_node(slot_id, x_usd, z_usd, kind="slot")

    # ---- 입차 차로(z 양수, GATE_OUT/W_OUT/XN 마커): entry_outer → 게이트 → 대기
    # → crossing_entry(XN) → 슬롯별 진입 분기점(A1'/A2'/A3') → 슬롯(z를 가로질러 진입) ----
    entry_chain = ["entry_outer", "entry_gate", "entry_wait", "crossing_entry"]
    entry_coords = [ENTRY_OUTER_USD, ENTRY_GATE_USD, ENTRY_WAIT_USD, CROSSING_ENTRY_USD]
    for node_id, (x_usd, z_usd) in zip(entry_chain, entry_coords):
        add_node(node_id, x_usd, z_usd, kind="junction")
    for i in range(len(entry_chain) - 1):
        add_edge(entry_chain[i], entry_chain[i + 1], f"ZIN{i + 1:02d}")
    # crossing_entry(XN)에서 슬롯 열마다 갈라지는 진입 분기점(A1'/A2'/A3') → 슬롯.
    for slot_id, (x_usd, z_usd) in ENTRY_SLOT_JUNCTION_USD.items():
        junction_id = f"entry_{slot_id.lower()}"
        add_node(junction_id, x_usd, z_usd, kind="junction")
        add_edge("crossing_entry", junction_id, f"ZIN_{slot_id}")
        add_edge(junction_id, slot_id, f"ZIN_{slot_id}_dock")

    # ---- 출차 차로(z 음수, GATE_IN/W_IN/XS 마커): 슬롯(A1/A2/A3 자체가 통로 차선을
    # 겸함) → crossing_exit(XS) → 대기 → 게이트 → exit_outer ----
    exit_chain = ["crossing_exit", "exit_wait", "exit_gate", "exit_outer"]
    exit_coords = [CROSSING_EXIT_USD, EXIT_WAIT_USD, EXIT_GATE_USD, EXIT_OUTER_USD]
    for node_id, (x_usd, z_usd) in zip(exit_chain, exit_coords):
        add_node(node_id, x_usd, z_usd, kind="junction")
    for i in range(len(exit_chain) - 1):
        add_edge(exit_chain[i], exit_chain[i + 1], f"ZOUT{i + 1:02d}")
    # crossing_exit(XS)에서 슬롯 열마다 갈라지는 진출 분기점 → 슬롯 (입차 쪽과 대칭:
    # 먼저 slot x로 수평 이동한 뒤 슬롯 z로 수직 진입 — 대각선 대신 ㄱ자 경로).
    for slot_id, (x_usd, z_usd) in EXIT_SLOT_JUNCTION_USD.items():
        junction_id = f"exit_{slot_id.lower()}"
        add_node(junction_id, x_usd, z_usd, kind="junction")
        add_edge("crossing_exit", junction_id, f"ZOUT_{slot_id}")
        add_edge(junction_id, slot_id, f"ZOUT_{slot_id}_dock")

    # ---- 로봇 대기 도크 4개(입차 전용 2: entry_lead/entry_follow, 출차 전용 2:
    # exit_lead/exit_follow) — crossing_entry/crossing_exit는 위 차로 체인에서
    # 이미 노드로 추가했으므로 여기서는 도크만 추가해 연결한다 ----
    add_node("dock_entry_1", *DOCK_ENTRY_1_USD, kind="dock", role="entry")
    add_node("dock_entry_2", *DOCK_ENTRY_2_USD, kind="dock", role="entry")
    add_edge("crossing_entry", "dock_entry_1")
    add_edge("crossing_entry", "dock_entry_2")

    add_node("dock_exit_1", *DOCK_EXIT_1_USD, kind="dock", role="exit")
    add_node("dock_exit_2", *DOCK_EXIT_2_USD, kind="dock", role="exit")
    add_edge("crossing_exit", "dock_exit_1")
    add_edge("crossing_exit", "dock_exit_2")

    return dict(
        meta=dict(
            generated_by="generate_map.py",
            frame="ros_map",
            usd_to_ros="ros_x = usd_x, ros_y = -usd_z (v4.usd 실측 기반, 2026-07-25)",
            source="isaacpjt/Isaac_envo/parking/parking_environment_v4.usd "
                   "(ArucoMarkerPreview 스코프) + src/parkbot_aruco/parkbot_aruco/"
                   "site_map_v4.py(ENTRY/EXIT 변환 기준)",
            # space_length/space_width: v3.usd Spaces 스코프의 parking:length/width
            # (A1/A2/A3 전부 동일, 6.6x3.4 — v4에서도 슬롯 규격 자체는 안 바뀜,
            # v4.usd에는 이 스코프가 없어 v3 실측값을 그대로 유지).
            # aisle_width: v2 값(9.0)을 그대로 유지. scripts/lidar/
            # visualize_lidar.py, generate_sample_pointcloud.py가 여전히 쓴다.
            # (2026-07-27: 이 값을 "모든 통로는 y=0 수평선"이라는 전제로 쓰던
            # core/obstacle_detector.py의 zone_boxes()는 그 전제가 v3/v4 좌표와
            # 안 맞아 삭제됐다 — safety_monitor_node.py 상단 주석 참고.)
            # half_w_m: 더 이상 안 쓰임(2026-07-24) — LiDAR가 서/동 2대에서
            # 1대로 통합되면서 safety_monitor_node.py가
            # core/lidar_frame_transform.sensor_offset()(인자 없음, v3.usd 실측
            # 고정값)을 직접 쓰도록 바뀌었다. 이 키는 죽은 값이라 지워도 되지만
            # 당장 해는 없어 남겨둠.
            params=dict(slot_count=3, layout="v4", space_length=6.6,
                        space_width=3.4, aisle_width=9.0, half_w_m=17.0),
        ),
        nodes=nodes,
        edges=edges,
        zones=sorted(set(zones)),
    )


def write_seed_sql(data, path):
    """지도 데이터 → DB 시드 SQL. YAML과 같은 원천이므로 어긋날 수 없다."""
    lines = [
        "-- generate_map.py가 자동 생성한 시드. 손으로 편집하지 말 것.",
        "-- 적용: mysql -u parking -p parking < 002_seed.sql",
        "",
    ]
    for zone_id in data["zones"]:
        lines.append(
            f"INSERT INTO zones (zone_id) VALUES ('{zone_id}')"
            " ON DUPLICATE KEY UPDATE zone_id = zone_id;"
        )
    lines.append("")
    for node_id, attrs in data["nodes"].items():
        if attrs["kind"] != "slot":
            continue
        lines.append(
            "INSERT INTO parking_slots (slot_id, x, y, is_accessible)"
            f" VALUES ('{node_id}', {attrs['x']}, {attrs['y']}, FALSE)"
            " ON DUPLICATE KEY UPDATE x = VALUES(x), y = VALUES(y),"
            " is_accessible = VALUES(is_accessible);"
        )
    lines.append("")
    nodes = data["nodes"]
    for edge in data["edges"]:
        u, v = sorted((edge["u"], edge["v"]))
        dist = round(math.hypot(nodes[u]["x"] - nodes[v]["x"],
                                nodes[u]["y"] - nodes[v]["y"]), 3)
        zone = f"'{edge['zone']}'" if edge.get("zone") else "NULL"
        lines.append(
            "INSERT INTO parking_lot_edges (u, v, dist_m, zone_id)"
            f" VALUES ('{u}', '{v}', {dist}, {zone})"
            " ON DUPLICATE KEY UPDATE dist_m = VALUES(dist_m),"
            " zone_id = VALUES(zone_id);"
        )
    Path(path).write_text("\n".join(lines) + "\n")


def main():
    data = build_map()

    DEFAULT_OUTPUT.parent.mkdir(parents=True, exist_ok=True)
    with open(DEFAULT_OUTPUT, "w") as f:
        yaml.safe_dump(data, f, allow_unicode=True, sort_keys=False)

    write_seed_sql(data, DEFAULT_SEED_SQL)

    slots = {k: v for k, v in data["nodes"].items() if v["kind"] == "slot"}
    print(f"생성 완료: {DEFAULT_OUTPUT}")
    print(f"시드 SQL: {DEFAULT_SEED_SQL}")
    print(f"노드 {len(data['nodes'])}개, 엣지 {len(data['edges'])}개, "
          f"존 {len(data['zones'])}개, 슬롯 {len(slots)}개")
    for name in sorted(slots):
        print(f"  {name}: ({slots[name]['x']:+.1f}, {slots[name]['y']:+.1f})")


if __name__ == "__main__":
    main()
