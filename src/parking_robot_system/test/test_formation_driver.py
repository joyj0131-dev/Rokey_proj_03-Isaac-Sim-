import math
from parking_robot_system.formation_driver import body_twist_from_world_error, wrap


def test_wrap():
    assert abs(wrap(math.pi * 3)) - math.pi < 1e-9
    assert wrap(0.0) == 0.0


def test_forward_when_facing_plus_x():
    # yaw=0: forward_world=(cos0,-sin0)=(+x). world +x 오차 → 순수 전진.
    fwd, left = body_twist_from_world_error(1.0, 0.0, 0.0)
    assert fwd > 0.9 and abs(left) < 1e-9


def test_strafe_axis():
    # yaw=0: +vy(left) = world -z. world -z 오차(ez=-1) → +left.
    fwd, left = body_twist_from_world_error(0.0, -1.0, 0.0)
    assert abs(fwd) < 1e-9 and left > 0.9
