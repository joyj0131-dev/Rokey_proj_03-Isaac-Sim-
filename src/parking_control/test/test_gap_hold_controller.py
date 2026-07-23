"""gap-hold 컨트롤러 단위 테스트. ROS/Isaac Sim 없이 순수 로직만 검증."""

import math

from parking_control.core.gap_hold_controller import (
    GapHoldController, Pose2D, follower_target, yaw_from_quaternion,
)


def test_follower_target_leader_facing_plus_x():
    leader = Pose2D(x=0.0, y=0.0, yaw=0.0)
    target = follower_target(leader, gap_m=1.5)
    assert target.x == -1.5
    assert target.y == 0.0


def test_follower_target_leader_facing_plus_y():
    leader = Pose2D(x=0.0, y=0.0, yaw=math.pi / 2)
    target = follower_target(leader, gap_m=2.0)
    assert math.isclose(target.x, 0.0, abs_tol=1e-9)
    assert math.isclose(target.y, -2.0, abs_tol=1e-9)


def test_follower_target_reverses_with_leader_direction():
    """리더가 방향을 반대로 틀면(후진) 목표점도 그대로 뒤집힌다 — role을
    바꾸지 않고도 전진/후진 둘 다 같은 식으로 대응한다는 설계의 핵심."""
    forward = follower_target(Pose2D(0.0, 0.0, 0.0), gap_m=1.0)
    reversed_ = follower_target(Pose2D(0.0, 0.0, math.pi), gap_m=1.0)
    assert math.isclose(forward.x, -reversed_.x, abs_tol=1e-9)


def test_controller_stops_when_already_at_target():
    controller = GapHoldController(gap_m=1.0)
    leader = Pose2D(x=1.0, y=0.0, yaw=0.0)
    follower = follower_target(leader, gap_m=1.0)  # 이미 목표 지점
    cmd = controller.compute(follower, leader)
    assert math.isclose(cmd.linear_x, 0.0, abs_tol=1e-9)
    assert math.isclose(cmd.angular_z, 0.0, abs_tol=1e-9)


def test_controller_drives_forward_when_behind_target():
    controller = GapHoldController(gap_m=1.0, max_linear=10.0)
    leader = Pose2D(x=5.0, y=0.0, yaw=0.0)
    follower = Pose2D(x=0.0, y=0.0, yaw=0.0)  # target은 x=4.0, 훨씬 뒤처짐
    cmd = controller.compute(follower, leader)
    assert cmd.linear_x > 0.0
    assert math.isclose(cmd.angular_z, 0.0, abs_tol=1e-9)


def test_controller_clamps_to_max_linear():
    controller = GapHoldController(gap_m=1.0, k_linear=100.0, max_linear=0.5)
    leader = Pose2D(x=100.0, y=0.0, yaw=0.0)
    follower = Pose2D(x=0.0, y=0.0, yaw=0.0)
    cmd = controller.compute(follower, leader)
    assert cmd.linear_x == 0.5


def test_yaw_from_quaternion_identity_is_zero():
    assert math.isclose(yaw_from_quaternion(0.0, 0.0, 0.0, 1.0), 0.0, abs_tol=1e-9)


def test_yaw_from_quaternion_90_degrees():
    half = math.sin(math.pi / 4)
    yaw = yaw_from_quaternion(0.0, 0.0, half, math.cos(math.pi / 4))
    assert math.isclose(yaw, math.pi / 2, abs_tol=1e-6)


def test_controller_turns_in_place_when_target_behind():
    """목표가 follower 뒤쪽(heading_error > 90도)이면 전진 없이 회전만."""
    controller = GapHoldController(gap_m=1.0, max_linear=10.0)
    leader = Pose2D(x=-5.0, y=0.0, yaw=0.0)   # target은 x=-6.0, follower 뒤쪽
    follower = Pose2D(x=0.0, y=0.0, yaw=0.0)  # follower는 +X를 보고 있음
    cmd = controller.compute(follower, leader)
    assert cmd.linear_x == 0.0
    assert cmd.angular_z != 0.0


def test_holonomic_uses_lateral_not_rotation():
    """홀로노믹 모드: 목표가 옆(follower 좌측)에 있으면 회전이 아니라 linear_y로 붙는다."""
    from parking_control.core.gap_hold_controller import (
        GapHoldController, Pose2D)
    ctrl = GapHoldController(gap_m=0.0, holonomic=True, max_yaw_hold=0.15)
    # follower 동향(yaw=0), leader가 follower 정좌측(+y)에 있음 → 목표도 +y
    follower = Pose2D(x=0.0, y=0.0, yaw=0.0)
    leader = Pose2D(x=0.0, y=2.0, yaw=0.0)
    cmd = ctrl.compute(follower, leader)
    assert cmd.linear_y > 0.1          # 좌측으로 횡이동
    assert abs(cmd.linear_x) < 1e-9    # 전진 성분 없음
    assert abs(cmd.angular_z) < 1e-9   # yaw 정렬돼 있으니 회전 없음


def test_holonomic_yaw_hold_bounded():
    """리더와 방위가 어긋나면 회전으로 정렬하되 max_yaw_hold로 제한된다."""
    from parking_control.core.gap_hold_controller import (
        GapHoldController, Pose2D)
    ctrl = GapHoldController(gap_m=0.0, holonomic=True,
                             k_yaw_hold=2.0, max_yaw_hold=0.15)
    follower = Pose2D(x=0.0, y=0.0, yaw=0.0)
    leader = Pose2D(x=0.0, y=0.0, yaw=1.0)   # 큰 방위 오차
    cmd = ctrl.compute(follower, leader)
    assert abs(cmd.angular_z) <= 0.15 + 1e-9
    assert cmd.angular_z > 0
