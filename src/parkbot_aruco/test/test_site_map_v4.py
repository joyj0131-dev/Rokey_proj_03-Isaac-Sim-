"""site_map_v4 단위 테스트 — z 반전 규약이 무너지면 여기서 잡힌다."""
import pytest

from parkbot_aruco import site_map_v4 as sm


# v4 USD 실측값 (parking_environment_v4.usd 의 aruco:serves / aruco:position.z)
V4_MARKERS = [
    {"serves": "GATE_IN", "z": -7.075}, {"serves": "GATE_OUT", "z": 7.075},
    {"serves": "W_IN", "z": -7.075},    {"serves": "W_OUT", "z": 7.075},
    {"serves": "D_IN_1", "z": -2.2},    {"serves": "D_OUT_1", "z": 2.2},
    {"serves": "D_IN_2", "z": -2.2},    {"serves": "D_OUT_2", "z": 2.2},
    {"serves": "XS", "z": -6.875},      {"serves": "XN", "z": 6.875},
    {"serves": "A1", "z": -6.875},      {"serves": "A1'", "z": 6.875},
    {"serves": "A2", "z": -6.875},      {"serves": "A2'", "z": 6.875},
    {"serves": "A3", "z": -6.875},      {"serves": "A3'", "z": 6.875},
]


def test_entry_is_positive_z_side():
    """입차는 z 양수. 에셋이 OUT 이라 부르는 것이 우리 ENTRY 다."""
    assert sm.role_of("W_OUT") == sm.ENTRY
    assert sm.role_of("D_OUT_1") == sm.ENTRY
    assert sm.expected_z_sign(sm.ENTRY) == 1


def test_exit_is_negative_z_side():
    assert sm.role_of("W_IN") == sm.EXIT
    assert sm.role_of("A1") == sm.EXIT
    assert sm.expected_z_sign(sm.EXIT) == -1


def test_all_16_v4_markers_have_a_role():
    for m in V4_MARKERS:
        assert sm.role_of(m["serves"]) in (sm.ENTRY, sm.EXIT)


def test_v4_markers_match_convention():
    """실측 z 부호가 배정한 역할과 일치해야 한다."""
    assert sm.validate_markers(V4_MARKERS) == []


def test_validate_detects_flipped_marker():
    bad = [{"serves": "W_OUT", "z": -7.075}]     # ENTRY 인데 z 음수
    problems = sm.validate_markers(bad)
    assert len(problems) == 1
    assert "W_OUT" in problems[0]


def test_unknown_serves_raises():
    with pytest.raises(KeyError):
        sm.role_of("NOPE")


def test_four_robots_map_to_distinct_dock_markers():
    assert len(sm.ROBOTS) == 4
    docks = [sm.ROBOT_DOCK_MARKER[r] for r in sm.ROBOTS]
    assert len(set(docks)) == 4
    for d in docks:
        assert d in sm.SERVES_ROLE


def test_team_assignment():
    assert sm.team_of("entry_lead") == sm.ENTRY
    assert sm.team_of("entry_follow") == sm.ENTRY
    assert sm.team_of("exit_lead") == sm.EXIT
    assert sm.team_of("exit_follow") == sm.EXIT
