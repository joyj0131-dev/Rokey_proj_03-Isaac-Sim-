"""ingress_control(R5a) 순수 함수/클래스 단위테스트 — ROS 무의존.

이 모듈은 rclpy 를 전혀 import 하지 않으므로(순수 파이썬), test_pose_controller.py/
test_axle_center.py 와 같은 패턴으로 직접 부른다.
"""
import pathlib
import sys

import pytest

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))
from parkbot_motion.ingress_control import (
    IngressController, lateral_centring_vy, pick_target_axle, return_phase_vx)


# --- lateral_centring_vy ----------------------------------------------------

def test_lateral_centring_zero_when_balanced():
    assert lateral_centring_vy(1.0, 1.0) == 0.0


def test_lateral_centring_within_deadband_is_zero():
    # err=0.005 < deadband(0.01)
    assert lateral_centring_vy(1.005, 1.0) == 0.0


def test_lateral_centring_positive_error_drives_positive_vy():
    # left > right -> err>0 -> vy>0 (kp=1.2 기본)
    vy = lateral_centring_vy(1.10, 1.00)
    assert vy == pytest.approx(1.2 * 0.10)


def test_lateral_centring_negative_error_drives_negative_vy():
    vy = lateral_centring_vy(1.00, 1.10)
    assert vy == pytest.approx(-1.2 * 0.10)


def test_lateral_centring_clips_to_vy_max():
    vy = lateral_centring_vy(2.0, 0.0)  # err=2.0, kp*err=2.4 >> vy_max=0.15
    assert vy == pytest.approx(0.15)
    vy2 = lateral_centring_vy(0.0, 2.0)
    assert vy2 == pytest.approx(-0.15)


def test_lateral_centring_left_only_is_zero():
    # taskC2fix §2.2 핵심 안전장치: 한쪽만 유효하면 강제로 밀지 않는다.
    assert lateral_centring_vy(0.30, None) == 0.0


def test_lateral_centring_right_only_is_zero():
    assert lateral_centring_vy(None, 0.30) == 0.0


def test_lateral_centring_both_none_is_zero():
    assert lateral_centring_vy(None, None) == 0.0


# --- return_phase_vx --------------------------------------------------------

def test_return_phase_vx_target_ahead_of_travel_is_negative():
    # remaining = target - travel > 0 -> vx = clip(-remaining) < 0 (러너 부호 그대로)
    assert return_phase_vx(remaining=0.05, return_speed=0.15) == pytest.approx(-0.05)


def test_return_phase_vx_target_behind_travel_is_positive():
    assert return_phase_vx(remaining=-0.05, return_speed=0.15) == pytest.approx(0.05)


def test_return_phase_vx_clips_to_return_speed():
    assert return_phase_vx(remaining=10.0, return_speed=0.15) == pytest.approx(-0.15)
    assert return_phase_vx(remaining=-10.0, return_speed=0.15) == pytest.approx(0.15)


# --- pick_target_axle (stop-target selection) -------------------------------

def test_pick_target_axle_none_until_enough_troughs():
    assert pick_target_axle([], 0) is None
    assert pick_target_axle([-6.569], 1) is None


def test_pick_target_axle_selects_first_trough():
    assert pick_target_axle([-6.569], 0) == pytest.approx(-6.569)


def test_pick_target_axle_selects_second_trough_not_first():
    axle_centers = [-6.569, -10.163]
    assert pick_target_axle(axle_centers, 1) == pytest.approx(-10.163)
    assert pick_target_axle(axle_centers, 0) == pytest.approx(-6.569)


def test_pick_target_axle_rejects_negative_index():
    with pytest.raises(ValueError):
        pick_target_axle([-6.569], -1)


# --- IngressController: 상태기계/정지목표 선택 통합 테스트 --------------------

def test_controller_rejects_negative_trough_index():
    with pytest.raises(ValueError):
        IngressController(-1)


def test_controller_stays_seek_until_target_trough_count_reached():
    # trough_index=1(둘째축) -- 트로프가 1개만 있을 땐 아직 SEEK 이어야 한다.
    ctrl = IngressController(trough_index=1, settle_frames=3)
    axle_centers = []
    ctrl.step(0.0, None, None, axle_centers, 0.05)
    assert ctrl.phase == IngressController.PHASE_SEEK

    axle_centers.append(-6.569)
    ctrl.step(0.0, None, None, axle_centers, 0.05)
    assert ctrl.phase == IngressController.PHASE_SEEK  # 아직 1개뿐, index=1 은 2개 필요

    axle_centers.append(-10.163)
    ctrl.step(-7.3, None, None, axle_centers, 0.05)
    assert ctrl.phase == IngressController.PHASE_RETURN
    assert ctrl.target_x == pytest.approx(-10.163)


def test_controller_full_ingress_reaches_and_settles_at_first_axle():
    """SEEK(전진) -> 트로프 확정(이미 지나친 뒤) -> RETURN(후진) -> SETTLING -> DONE.

    로컬 forward(+vx) 가 주행좌표(x)를 감소시키는 이 미션의 부호 관례(러너
    APPROACH_YAW=-90°, ingress_node.py docstring 참고)를 그대로 재현하는 1차원
    합성 플랜트: x_next = x - vx*dt. 값은 taskC2fix-report.md §4 실측(rear axle
    center=-6.569, 트로프 exit≈-7.217)을 그대로 썼다.
    """
    ctrl = IngressController(trough_index=0, forward_speed=0.4, return_speed=0.15,
                              pos_tol=0.02, settle_frames=5)
    x = -4.5
    dt = 0.05
    axle_centers = []
    true_target = -6.569
    confirm_at_x = -7.217  # 트로프 exit 실측 근사치 -- 이 지점에서 axle_detector_node 가 완료 발행

    steps = 0
    last_cmd = None
    while not ctrl.done and steps < 5000:
        steps += 1
        vx, vy, wz = ctrl.step(x, 1.0, 1.0, axle_centers, dt)  # left==right -> vy=0 항상
        last_cmd = (vx, vy, wz)
        x -= vx * dt
        if not axle_centers and x <= confirm_at_x:
            axle_centers.append(true_target)

    assert ctrl.done, f'{5000}스텝 안에 완료되지 않음(마지막 phase={ctrl.phase})'
    assert ctrl.target_x == pytest.approx(true_target)
    assert ctrl.final_stop_x == pytest.approx(true_target, abs=0.03)
    assert last_cmd == (0.0, 0.0, 0.0)
    assert vy == 0.0 and wz == 0.0


def test_controller_lateral_offset_produces_nonzero_vy_during_seek():
    ctrl = IngressController(trough_index=0, settle_frames=3)
    vx, vy, wz = ctrl.step(0.0, 1.20, 1.00, [], 0.05)  # left 가 더 멀다(여유) -> +vy
    assert vx == pytest.approx(ctrl.forward_speed)
    assert vy > 0.0
    assert wz == 0.0
    assert ctrl.max_lat_dev_est > 0.0


def test_controller_done_is_idempotent_zero_twist():
    ctrl = IngressController(trough_index=0, pos_tol=0.02, settle_frames=1)
    axle_centers = [0.0]
    # 이미 target 근방(x=target=0.0) + settle_frames=1 -> SEEK->RETURN->SETTLING->
    # DONE 이 같은 틱에 모두 cascade 된다(IngressController.step 의 fallthrough).
    vx, vy, wz = ctrl.step(0.0, None, None, axle_centers, 0.05)
    assert (vx, vy, wz) == (0.0, 0.0, 0.0)
    assert ctrl.done
    # 완료 후 다시 불러도 계속 0 twist(멱등).
    vx, vy, wz = ctrl.step(0.0, None, None, axle_centers, 0.05)
    assert (vx, vy, wz) == (0.0, 0.0, 0.0)
