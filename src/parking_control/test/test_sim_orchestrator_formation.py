import math

import pytest

from parking_control.sim_orchestrator_node import formation_positions


def test_two_robot_formation_moves_both_members_along_horizontal_route():
    leader, follower = formation_positions((0.0, 0.0), (4.0, 0.0))

    assert leader[0] > 4.0
    assert follower[0] < 4.0
    assert leader[1] == follower[1] == 0.0
    assert math.dist(leader, follower) == pytest.approx(2.9)


def test_two_robot_formation_rotates_with_vertical_route():
    leader, follower = formation_positions((2.0, -3.0), (2.0, 1.0))

    assert leader[0] == follower[0] == 2.0
    assert leader[1] > 1.0
    assert follower[1] < 1.0
    assert math.dist(leader, follower) == pytest.approx(2.9)


def test_single_robot_fallback_stays_at_route_center():
    assert formation_positions((0.0, 0.0), (3.0, 2.0), robot_count=1) == [
        (3.0, 2.0)
    ]
