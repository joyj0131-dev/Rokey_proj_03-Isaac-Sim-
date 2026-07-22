"""pivot-rotate 컨트롤러 단위 테스트. ROS/Isaac Sim 없이 순수 로직만 검증."""

import math

from parking_control.core.pivot_rotate_controller import (
    PivotRotateController, Pose2D, formation_center, group_yaw_progress,
)


def test_formation_center_is_midpoint():
    a = Pose2D(x=-1.45, y=0.0, yaw=0.0)
    b = Pose2D(x=1.45, y=0.0, yaw=0.0)
    cx, cy = formation_center(a, b)
    assert math.isclose(cx, 0.0, abs_tol=1e-9)
    assert math.isclose(cy, 0.0, abs_tol=1e-9)


def test_group_yaw_progress_zero_before_moving():
    start = Pose2D(x=0.0, y=0.0, yaw=0.3)
    partner_start = Pose2D(x=1.0, y=0.0, yaw=0.1)
    progress = group_yaw_progress(
        start.yaw, partner_start.yaw, start.yaw, partner_start.yaw)
    assert math.isclose(progress, 0.0, abs_tol=1e-9)


def test_group_yaw_progress_averages_both_robots():
    # 한쪽만 0.4rad, 다른 쪽은 0.6rad 돌았으면 그룹 진행은 평균인 0.5rad.
    progress = group_yaw_progress(
        own_yaw=0.4, partner_yaw=0.6, start_own_yaw=0.0, start_partner_yaw=0.0)
    assert math.isclose(progress, 0.5, abs_tol=1e-9)


def test_pure_rotation_gives_opposite_linear_directions_same_omega():
    """제자리 피벗: 중심을 사이에 둔 두 로봇은 선속도는 반대, 자기 회전
    속도(각속도)는 항상 같아야 한다 — 회전목마 반대편 말과 같은 원리."""
    controller = PivotRotateController(
        target_angle_rad=math.pi / 2, k_omega=1.0, max_omega=10.0, max_linear=10.0)
    rear = Pose2D(x=-1.45, y=0.0, yaw=0.0)
    front = Pose2D(x=1.45, y=0.0, yaw=0.0)

    cmd_rear = controller.compute(rear, front, rear, front)
    cmd_front = controller.compute(front, rear, front, rear)

    assert math.isclose(cmd_rear.angular_z, cmd_front.angular_z, abs_tol=1e-9)
    assert cmd_rear.angular_z != 0.0
    # 선속도(이 시점엔 둘 다 yaw=0이라 body == world)는 부호가 반대여야 한다.
    assert cmd_rear.linear_y * cmd_front.linear_y < 0.0
    assert math.isclose(abs(cmd_rear.linear_y), abs(cmd_front.linear_y), abs_tol=1e-9)


def test_omega_clamped_to_slip_safety_limit():
    """제자리 회전은 롤러 미끄러짐 지배적이라 안전 상한 밖으로 못 나간다."""
    controller = PivotRotateController(
        target_angle_rad=math.pi, k_omega=100.0, max_omega=0.15, max_linear=10.0)
    rear = Pose2D(x=-1.45, y=0.0, yaw=0.0)
    front = Pose2D(x=1.45, y=0.0, yaw=0.0)
    cmd = controller.compute(rear, front, rear, front)
    assert abs(cmd.angular_z) <= 0.15 + 1e-9


def test_settled_when_target_angle_reached():
    controller = PivotRotateController(target_angle_rad=math.pi / 2)
    start_rear = Pose2D(x=-1.45, y=0.0, yaw=0.0)
    start_front = Pose2D(x=1.45, y=0.0, yaw=0.0)
    # 두 로봇 다 정확히 90도 돈 상태.
    rear = Pose2D(x=0.0, y=-1.45, yaw=math.pi / 2)
    front = Pose2D(x=0.0, y=1.45, yaw=math.pi / 2)
    assert controller.is_settled(rear, front, start_rear, start_front, tol_rad=0.05)


def test_not_settled_before_reaching_target():
    controller = PivotRotateController(target_angle_rad=math.pi / 2)
    start_rear = Pose2D(x=-1.45, y=0.0, yaw=0.0)
    start_front = Pose2D(x=1.45, y=0.0, yaw=0.0)
    rear = Pose2D(x=-1.45, y=0.0, yaw=0.1)   # 아직 0.1rad밖에 안 돎
    front = Pose2D(x=1.45, y=0.0, yaw=0.1)
    assert not controller.is_settled(rear, front, start_rear, start_front, tol_rad=0.05)


def test_omega_direction_flips_with_target_sign():
    """목표각이 음수(시계 방향)면 각속도 부호도 반대가 되어야 한다."""
    rear = Pose2D(x=-1.45, y=0.0, yaw=0.0)
    front = Pose2D(x=1.45, y=0.0, yaw=0.0)
    ccw = PivotRotateController(target_angle_rad=math.pi / 2, max_omega=10.0)
    cw = PivotRotateController(target_angle_rad=-math.pi / 2, max_omega=10.0)
    cmd_ccw = ccw.compute(rear, front, rear, front)
    cmd_cw = cw.compute(rear, front, rear, front)
    assert cmd_ccw.angular_z > 0.0
    assert cmd_cw.angular_z < 0.0
