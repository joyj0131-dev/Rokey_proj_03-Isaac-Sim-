"""P1 게이트: 그래프+경로탐색 검증 (Isaac Sim·ROS 불필요).

v3 레이아웃(3슬롯 A1/A2/A3, 입/출차 차로 분리, 2026-07-24) 기준:
  1) 인계 지점(entry_wait)→A1 최단 경로가 손계산 거리와 일치
  2) 엣지 차단 시 도달 불가로 정확히 판정, 해제 시 원래 경로 복귀
     (이 레이아웃은 슬롯당 진입 경로가 하나뿐인 트리 구조라 "우회"는 없다 —
     v2처럼 슬롯이 통로 양옆 분기점에 이중 연결되던 구조가 아니다)
  3) 도달 불가(없는 노드) 시 명시적 실패(None)
  4) 좌표 회귀 — 슬롯 좌표가 환경 스냅샷과 일치 (레이아웃 변경 감지)
"""

import math
from pathlib import Path

import pytest

from parking_control.core.graph import ParkingMap
from parking_control.core.pathfinder import PathFinder

MAP_YAML = Path(__file__).resolve().parent.parent / "config" / "parking_map.yaml"

# 현재 환경 스냅샷(v3.usd) 기준 손계산 값. entry_wait(-8.5,5.5) -> entry_j0(-4.2,5.3)
# -> entry_a1(2.8,5.3) -> A1(2.8,0).
ENTRY_WAIT_TO_J0 = math.hypot(4.3, 0.2)
J0_TO_A1_JUNCTION = 7.0
A1_JUNCTION_TO_A1 = 5.3


@pytest.fixture
def pf():
    return PathFinder(ParkingMap.load(MAP_YAML))


def test_shortest_path_entry_wait_to_a1(pf):
    result = pf.find_path("entry_wait", "A1")
    assert result is not None
    expected = ENTRY_WAIT_TO_J0 + J0_TO_A1_JUNCTION + A1_JUNCTION_TO_A1
    assert result.length == pytest.approx(expected, abs=1e-6)
    assert result.nodes[0] == "entry_wait" and result.nodes[-1] == "A1"
    assert len(result.waypoints) == len(result.nodes)
    assert result.zones == ["ZIN03", "ZIN_A1", "ZIN_A1_dock"]


def test_blocked_edge_reroutes_via_exit_lane_then_restores(pf):
    """그래프가 무방향이라 A1은 입차 진입로(entry_a1)와 출차 진입로(exit_a1)
    양쪽으로 연결돼 있다 — 하나만 막으면 물리적으로는 실제 없는 경로(다른
    슬롯·차로를 거쳐 도달)까지 "닿을 수 있는" 것으로 나온다. 이건 운영상
    입차팀이 출차 차로를 넘나들지 않는다는 정책(task_dispatcher의 role-fixed
    로봇쌍)이 그래프 밖에서 강제되기 때문에 실제로는 안 일어나지만, 그래프
    자체의 연결성은 정직하게 이렇다는 걸 보여주는 테스트다."""
    base = pf.find_path("entry_wait", "A1")
    assert base is not None
    pf.block_edge("entry_j0", "entry_a1")
    detour = pf.find_path("entry_wait", "A1")
    assert detour is not None
    assert detour.length > base.length
    assert "exit_a1" in detour.nodes   # 출차 차로를 거쳐 우회
    pf.unblock_edge("entry_j0", "entry_a1")
    restored = pf.find_path("entry_wait", "A1")
    assert restored.length == pytest.approx(base.length, abs=1e-9)
    assert restored.nodes == base.nodes


def test_unreachable_returns_none(pf):
    # A1의 두 연결(입차 진입로, 출차 진입로)을 모두 끊어야 진짜 도달 불가다.
    pf.block_edge("A1", "entry_a1")
    pf.block_edge("A1", "exit_a1")
    assert pf.find_path("entry_wait", "A1") is None
    # 없는 노드도 None (예외를 밖으로 던지지 않음)
    assert pf.find_path("entry_wait", "Z9") is None


def test_slot_coordinates_regression(pf):
    """환경 파라미터가 바뀌면 여기서 걸린다 — 스냅샷 갱신 필요 신호."""
    expected = {"A1": (2.8, 0.0), "A2": (6.2, 0.0), "A3": (9.6, 0.0)}
    for slot_id, xy in expected.items():
        assert pf.map.node_pos(slot_id) == xy, f"{slot_id} 좌표 불일치"
    assert len(pf.map.nodes_of_kind("slot")) == 3


def test_exit_route_reaches_gate(pf):
    """출차 경로(슬롯 -> 출차 게이트)도 입차와 별도 차로로 성립해야 한다."""
    result = pf.find_path("A1", "exit_gate")
    assert result is not None
    assert result.nodes[0] == "A1" and result.nodes[-1] == "exit_gate"
    # 입차 차로(ZIN*) 존을 거치지 않는다 — 입/출차 차로가 물리적으로 분리돼 있다.
    assert not any(z.startswith("ZIN") for z in result.zones)
