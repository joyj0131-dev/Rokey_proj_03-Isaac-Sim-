#!/usr/bin/env python3
"""주차장 환경(v3, 3슬롯 A1/A2/A3) → config/parking_map.yaml 생성기.

좌표의 유일한 원천. 주차장 레이아웃이 바뀌면 여기 상수만 고치고 다시
실행한다 — 다른 어떤 코드에도 좌표를 하드코딩하지 않는다.

2026-07-24: 이전(v2) 16슬롯(A1-A8/B1-B8, 인계장 1개, 통로 공유) 레이아웃을
버리고, 실제 확정된 물리 배치(~/Downloads/parking_environment_v3.usd)로
전면 재작성했다. v3는 손으로 설계된 특정 배치라 v2처럼 --space-count 같은
파라미터로 일반화하지 않는다 — 레이아웃이 다시 바뀌면 아래 상수를 v3.usd에서
다시 실측해 고친다.

핵심 변경: 입차와 출차가 완전히 분리된 차로를 쓴다(v2는 통로 하나를 공유).
  - 입차: 인계베이 → 입차 게이트 → 입차 차로(y=+5.3~5.5) → 슬롯(A1/A2/A3, 북쪽에서 진입)
  - 출차: 슬롯(남쪽으로 진출) → 출차 차로(y=-5.3~-5.5) → 출차 게이트 → 인계베이
  각 차로에 로봇 대기 도크가 2개씩 있다(입차 전용 2대 / 출차 전용 2대에 대응).

좌표 규약: ROS map 프레임 (x, y). USD(Y-up)와의 변환은 ros_x = usd_x,
ros_y = -usd_z (Isaac Sim ROS2 브리지 기본 규약, generate_map.py 기존 관례
그대로 유지) — v3.usd의 Navigation/RobotServiceArea/ArucoMarkerPreview
스코프에 있는 USD 좌표를 이 규약으로 변환해 아래 상수에 그대로 옮겼다.

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


# ---- v3.usd 실측 좌표 (USD x,z) — Navigation/RobotServiceArea/ArucoMarkerPreview 스코프 ----
SLOTS_USD = {"A1": (2.8, 0.0), "A2": (6.2, 0.0), "A3": (9.6, 0.0)}
# route:inbound (v3.usd Navigation 스코프) — 입차 게이트 밖(-21)에서 슬롯 열까지.
ROUTE_INBOUND_USD = [(-21, -5.5), (-13, -5.5), (-8.5, -5.5), (-4.2, -5.3)]
# route:outbound — 슬롯 열에서 출차 게이트 밖(-21)까지.
ROUTE_OUTBOUND_USD = [(-4.2, 5.3), (-8.5, 5.5), (-13, 5.5), (-21, 5.5)]
LANE_INBOUND_Z = -5.3    # v3.usd lane:inboundZ — 슬롯 열 옆 입차 차로
LANE_OUTBOUND_Z = 5.3    # v3.usd lane:outboundZ — 슬롯 열 옆 출차 차로
ENTRY_GATE_USD = (-13, -5.5)     # vehicle:entryGate
ENTRY_WAIT_USD = (-8.5, -5.5)    # vehicle:entryWait — 인계(핸드오프) 지점
EXIT_GATE_USD = (-13, 5.5)       # vehicle:exitGate
EXIT_WAIT_USD = (-8.5, 5.5)      # vehicle:exitWait — 인계(핸드오프) 지점
# 입차 전용 로봇 대기 도크 2개(dock_ROBOT_IN/IN_2), 출차 전용 2개(dock_ROBOT_OUT/OUT_2).
DOCK_ENTRY_1_USD = (-3.2, -2.2)
DOCK_ENTRY_2_USD = (-1.2, -2.2)
DOCK_EXIT_1_USD = (-3.2, 2.2)
DOCK_EXIT_2_USD = (-1.2, 2.2)
# crossing_S/N(아루코 "통로 분기점") — 입/출차 차로에서 도크 쪽으로 갈라지는 지점.
CROSSING_ENTRY_USD = (-2.5, -6.875)
CROSSING_EXIT_USD = (-2.5, 6.875)


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

    # ---- 슬롯 3개(단일 열, A1/A2/A3) ----
    for slot_id, (x_usd, z_usd) in SLOTS_USD.items():
        add_node(slot_id, x_usd, z_usd, kind="slot")

    # ---- 입차 차로: entrance(=인계베이 인접 게이트) → 슬롯 열 ----
    entry_chain = ["entry_gate", "entry_wait", "EJ0", "EJ1"]
    entry_coords = [ENTRY_GATE_USD, ENTRY_WAIT_USD, *ROUTE_INBOUND_USD[2:]]
    # ROUTE_INBOUND_USD = [(-21,-5.5), (-13,-5.5)=게이트, (-8.5,-5.5)=대기, (-4.2,-5.3)]
    # 게이트/대기는 위에서 이미 별도 이름으로 넣었으므로 나머지(-21과 -4.2)만 체인에 쓴다.
    entry_chain = ["entry_outer", "entry_gate", "entry_wait", "entry_j0"]
    entry_coords = [ROUTE_INBOUND_USD[0], ENTRY_GATE_USD, ENTRY_WAIT_USD,
                    ROUTE_INBOUND_USD[3]]
    for node_id, (x_usd, z_usd) in zip(entry_chain, entry_coords):
        add_node(node_id, x_usd, z_usd, kind="junction")
    for i in range(len(entry_chain) - 1):
        add_edge(entry_chain[i], entry_chain[i + 1], f"ZIN{i + 1:02d}")
    # entry_j0(-4.2,-5.3)에서 슬롯 열마다 갈라지는 진입 분기점 → 각 슬롯.
    for slot_id, (slot_x, _) in SLOTS_USD.items():
        junction_id = f"entry_{slot_id.lower()}"
        add_node(junction_id, slot_x, LANE_INBOUND_Z, kind="junction")
        add_edge("entry_j0", junction_id, f"ZIN_{slot_id}")
        add_edge(junction_id, slot_id, f"ZIN_{slot_id}_dock")

    # ---- 출차 차로: 슬롯 열 → exit(=인계베이 인접 게이트) ----
    exit_chain = ["exit_j0", "exit_wait", "exit_gate", "exit_outer"]
    exit_coords = [ROUTE_OUTBOUND_USD[0], EXIT_WAIT_USD, EXIT_GATE_USD,
                   ROUTE_OUTBOUND_USD[3]]
    for node_id, (x_usd, z_usd) in zip(exit_chain, exit_coords):
        add_node(node_id, x_usd, z_usd, kind="junction")
    for i in range(len(exit_chain) - 1):
        add_edge(exit_chain[i], exit_chain[i + 1], f"ZOUT{i + 1:02d}")
    for slot_id, (slot_x, _) in SLOTS_USD.items():
        junction_id = f"exit_{slot_id.lower()}"
        add_node(junction_id, slot_x, LANE_OUTBOUND_Z, kind="junction")
        add_edge("exit_j0", junction_id, f"ZOUT_{slot_id}")
        add_edge(junction_id, slot_id, f"ZOUT_{slot_id}_dock")

    # ---- 로봇 대기 도크 4개(입차 전용 2, 출차 전용 2) — 통로 분기점(crossing)을 통해 연결 ----
    add_node("crossing_entry", *CROSSING_ENTRY_USD, kind="junction")
    add_edge("entry_wait", "crossing_entry", "ZIN_CROSS")
    add_node("dock_entry_1", *DOCK_ENTRY_1_USD, kind="dock", role="entry")
    add_node("dock_entry_2", *DOCK_ENTRY_2_USD, kind="dock", role="entry")
    add_edge("crossing_entry", "dock_entry_1")
    add_edge("crossing_entry", "dock_entry_2")

    add_node("crossing_exit", *CROSSING_EXIT_USD, kind="junction")
    add_edge("exit_wait", "crossing_exit", "ZOUT_CROSS")
    add_node("dock_exit_1", *DOCK_EXIT_1_USD, kind="dock", role="exit")
    add_node("dock_exit_2", *DOCK_EXIT_2_USD, kind="dock", role="exit")
    add_edge("crossing_exit", "dock_exit_1")
    add_edge("crossing_exit", "dock_exit_2")

    return dict(
        meta=dict(
            generated_by="generate_map.py",
            frame="ros_map",
            usd_to_ros="ros_x = usd_x, ros_y = -usd_z (v3.usd 실측 기반, 2026-07-24)",
            source="~/Downloads/parking_environment_v3.usd (Navigation/"
                   "RobotServiceArea/ArucoMarkerPreview 스코프)",
            # space_length/space_width: v3.usd Spaces 스코프의 parking:length/width
            # (A1/A2/A3 전부 동일, 6.6x3.4 — v2와 슬롯 규격 자체는 안 바뀜).
            # aisle_width: v2 값(9.0)을 임시로 유지 — v3는 통로가 y=0 수평선이
            # 아니라 core/obstacle_detector.py의 zone_boxes()가 이 값을 쓰는
            # "모든 통로가 수평"이라는 전제 자체가 안 맞는다(2026-07-24 확인,
            # 아래 obstacle_detector.py 주석 참고). 로봇 물리 경로 재설계(B) 때
            # zone_boxes도 같이 다시 설계해야 한다.
            # half_w_m: 더 이상 안 쓰임(2026-07-24) — LiDAR가 서/동 2대에서
            # 1대로 통합되면서 safety_monitor_node.py가
            # core/lidar_frame_transform.sensor_offset()(인자 없음, v3.usd 실측
            # 고정값)을 직접 쓰도록 바뀌었다. 이 키는 죽은 값이라 지워도 되지만
            # 당장 해는 없어 남겨둠.
            params=dict(slot_count=3, layout="v3", space_length=6.6,
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
