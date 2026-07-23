"""P1 게이트: 그래프+경로탐색 검증 (Isaac Sim·ROS 불필요).

계획서 3절의 4개 케이스:
  1) 입구→A5 최단 경로가 손계산 거리와 일치
  2) 엣지 차단 시 우회, 해제 시 원래 경로 복귀
  3) 도달 불가 시 명시적 실패(None)
  4) 좌표 회귀 — 슬롯 좌표가 환경 스냅샷과 일치 (레이아웃 변경 감지)
"""

import math
from pathlib import Path

import pytest

from parking_control.core.graph import ParkingMap
from parking_control.core.pathfinder import PathFinder

MAP_YAML = Path(__file__).resolve().parent.parent / "config" / "parking_map.yaml"

# 현재 환경 스냅샷 기준 손계산 값
SPACE_WIDTH = 3.40
SPUR = math.hypot(1.7, 7.8)          # 슬롯 중심 ↔ 인접 분기점
ENTRANCE_TO_J0 = 1.10                # border_margin


@pytest.fixture
def pf():
    return PathFinder(ParkingMap.load(MAP_YAML))


def test_shortest_path_entrance_to_a5(pf):
    result = pf.find_path("entrance", "A5")
    assert result is not None
    # entrance → J0 → J1..J5 (통로 5구간) → A5 스퍼
    expected = ENTRANCE_TO_J0 + 5 * SPACE_WIDTH + SPUR
    assert result.length == pytest.approx(expected, abs=1e-6)
    assert result.nodes[0] == "entrance" and result.nodes[-1] == "A5"
    assert len(result.waypoints) == len(result.nodes)
    # 존 매핑: 출입구 존 + 통로 Z01~Z05
    assert result.zones == ["Z_ENTRANCE", "Z01", "Z02", "Z03", "Z04", "Z05"]


def test_blocked_edge_reroutes_and_unblock_restores(pf):
    base = pf.find_path("entrance", "A5")
    pf.block_edge("J4", "J5")
    detour = pf.find_path("entrance", "A5")
    assert detour is not None
    assert detour.length > base.length
    assert ("J4", "J5") not in zip(detour.nodes, detour.nodes[1:])
    assert ("J5", "J4") not in zip(detour.nodes, detour.nodes[1:])
    pf.unblock_edge("J4", "J5")
    restored = pf.find_path("entrance", "A5")
    assert restored.length == pytest.approx(base.length, abs=1e-9)
    assert restored.nodes == base.nodes


def test_unreachable_returns_none(pf):
    # A5로 들어가는 스퍼 2개를 모두 차단하면 도달 불가
    pf.block_edge("A5", "J5")
    pf.block_edge("A5", "J6")
    assert pf.find_path("entrance", "A5") is None
    # 없는 노드도 None (예외를 밖으로 던지지 않음)
    assert pf.find_path("entrance", "Z9") is None


def test_slot_coordinates_regression(pf):
    """환경 파라미터가 바뀌면 여기서 걸린다 — 스냅샷 갱신 필요 신호."""
    half_w = 17.0
    for row, y in (("A", -7.8), ("B", 7.8)):
        for i in range(1, 9):
            x = round(-half_w + (i + 0.5) * SPACE_WIDTH, 3)
            assert pf.map.node_pos(f"{row}{i}") == (x, y), f"{row}{i} 좌표 불일치"
    assert len(pf.map.nodes_of_kind("slot")) == 16
    assert pf.map.graph.nodes["A1"]["accessible"] is True
    assert pf.map.graph.nodes["A3"]["accessible"] is False


def test_slot_axis_matches_layout(pf):
    """통로가 X축 일렬이고 A/B 두 행 다 슬롯이 Y방향으로 파여있으므로,
    모든 슬롯이 같은 축(pi/2)을 요구해야 한다."""
    for slot_id in pf.map.nodes_of_kind("slot"):
        assert pf.map.slot_axis_rad(slot_id) == pytest.approx(math.pi / 2)


def test_handoff_bay_route_reaches_indoor_slot():
    """인계 베이(H_B) → 실내 슬롯(B3) 경로가 성립해야 E2E 미션 배차가 가능하다."""
    pf = PathFinder(ParkingMap.load(MAP_YAML))
    result = pf.find_path("H_B", "B3")
    assert result is not None
    assert result.nodes[0] == "H_B"
    assert "entrance" in result.nodes          # 서측 개구부를 지난다
    assert "ZH_GATE" in result.zones           # 인계장 게이트 존
    assert any(z.startswith("ZH") and z != "ZH_GATE" for z in result.zones)
    assert 20.0 < result.length < 60.0         # H_B(-29.6,+7.8)→B3 대략 30m대
