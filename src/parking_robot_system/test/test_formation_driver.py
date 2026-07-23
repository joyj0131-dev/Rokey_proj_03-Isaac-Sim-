import math
from parking_robot_system.formation_driver import (
    body_twist_from_world_error, formation_heading, heading_hold_omega,
    rigid_body_world_velocity, wrap,
)


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


def test_formation_heading_matches_pickup_yaw_convention():
    rear = (-29.6, -1.93, 0.0)
    front = (-29.6, 1.66, 0.0)
    assert math.isclose(formation_heading(rear, front), math.pi / 2, abs_tol=1e-9)


def test_rigid_rotation_keeps_center_velocity_zero():
    rear_v = rigid_body_world_velocity(0.0, 0.0, 0.1, 0.0, -1.8)
    front_v = rigid_body_world_velocity(0.0, 0.0, 0.1, 0.0, 1.8)
    assert math.isclose(rear_v[0], -0.18, abs_tol=1e-12)
    assert math.isclose(front_v[0], 0.18, abs_tol=1e-12)
    assert math.isclose((rear_v[0] + front_v[0]) / 2, 0.0, abs_tol=1e-12)


def test_positive_rigid_rotation_increases_project_heading():
    rear = [0.0, -1.8]
    front = [0.0, 1.8]
    before = formation_heading(rear, front)
    rear_v = rigid_body_world_velocity(0.0, 0.0, 0.1, rear[0], rear[1])
    front_v = rigid_body_world_velocity(0.0, 0.0, 0.1, front[0], front[1])
    dt = 0.01
    rear_after = [rear[0] + rear_v[0] * dt, rear[1] + rear_v[1] * dt]
    front_after = [front[0] + front_v[0] * dt, front[1] + front_v[1] * dt]
    assert wrap(formation_heading(rear_after, front_after) - before) > 0.0


def test_heading_hold_command_sign_deadband_and_saturation():
    assert heading_hold_omega(1.0, 0.9, 0.8, 0.2) > 0.0
    assert heading_hold_omega(0.9, 1.0, 0.8, 0.2) < 0.0
    assert heading_hold_omega(1.0, 0.999, 0.8, 0.2, deadband=0.01) == 0.0
    assert heading_hold_omega(1.0, 0.0, 1.0, 0.2) == 0.2
