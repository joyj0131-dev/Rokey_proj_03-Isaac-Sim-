import math
import sys
from pathlib import Path

from parking_robot_system.formation_motion import (
    BAY_CLEAR_Z, FormationMotion,
)

RUNNER_DIR = Path(__file__).resolve().parents[3] / "isaacpjt" / "Isaac_envo"
sys.path.insert(0, str(RUNNER_DIR))
from parking_v4_runner import vehicle_mission_role  # noqa: E402


class _Logger:
    def info(self, _message):
        pass

    def warn(self, _message):
        pass


class _Node:
    def get_logger(self):
        return _Logger()


def _bare_motion():
    motion = object.__new__(FormationMotion)
    motion.node = _Node()
    motion.rear_id = "exit_lead"
    motion.front_id = "exit_follow"
    motion.robots = (motion.rear_id, motion.front_id)
    motion._settle = lambda *_args, **_kwargs: None
    motion._stop_all = lambda: None
    return motion


def test_vehicle_role_depends_on_start_location_not_vehicle_model():
    assert vehicle_mission_role("marker:W_OUT") == "entry"
    assert vehicle_mission_role("slot:A3") == "exit"
    assert vehicle_mission_role("slot:B7") == "exit"


def test_exit_transport_leaves_slot_then_rotates_then_moves_to_bay():
    motion = _bare_motion()
    motion.veh_x = 8.5
    motion.veh_z = 0.0
    motion.lane_z = -6.875
    heading = 0.25
    rotated_heading = heading + math.pi / 2
    calls = []

    motion.carry_heading = lambda: (
        heading if not any(c[0] == "rotate" for c in calls)
        else rotated_heading)
    motion.carry_to = lambda x, z, heading_ref=None: (
        calls.append(("carry", x, z, heading_ref)) or True)
    motion.carry_rotate_to = lambda yaw: (
        calls.append(("rotate", yaw)) or True)

    assert motion.carry_to_bay(-8.5, -7.075) is True
    assert calls == [
        ("carry", 8.5, -6.875, heading),
        ("rotate", rotated_heading),
        ("carry", -8.5, -7.075, rotated_heading),
    ]


def test_exit_transport_does_not_rotate_when_slot_exit_fails():
    motion = _bare_motion()
    motion.veh_x = 8.5
    motion.veh_z = 0.0
    motion.lane_z = -6.875
    motion.carry_heading = lambda: 0.0
    motion.carry_to = lambda *_args, **_kwargs: False
    motion.carry_rotate_to = lambda _yaw: (_ for _ in ()).throw(
        AssertionError("회전하면 안 됨"))

    assert motion.carry_to_bay(-8.5, -7.075) is False


def test_return_backout_uses_current_robot_axis_after_rotation():
    motion = _bare_motion()
    motion.pose = {
        motion.rear_id: (0.0, 2.0, 0.0),
        motion.front_id: (2.0, 2.0, 0.0),
    }
    captured = []
    motion.approach_parallel = lambda routes: (
        captured.append(routes) or False)

    assert motion.return_from_bay() is False
    assert captured[0] == {
        motion.rear_id: [(1.0 - BAY_CLEAR_Z, 2.0)],
        motion.front_id: [(1.0 + BAY_CLEAR_Z, 2.0)],
    }


def test_slot_pickup_propagates_rotation_failure():
    motion = _bare_motion()
    motion.wait_data = lambda: True
    motion.pose = {
        motion.rear_id: (0.0, 0.0, 0.0),
        motion.front_id: (0.0, 0.0, 0.0),
    }
    motion.rear_axle = -1.93
    motion.front_axle = 1.66
    motion.approach_parallel = lambda _routes: True
    motion.rotate_parallel = lambda _targets: False
    motion.ingress_parallel = lambda _targets: (_ for _ in ()).throw(
        AssertionError("회전 실패 뒤 진입하면 안 됨"))

    ok, message = motion.pickup_at_slot(8.5, 0.0)
    assert ok is False
    assert "회전 실패" in message
