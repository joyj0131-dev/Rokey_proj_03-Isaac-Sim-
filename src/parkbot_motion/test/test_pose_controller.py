"""PoseController(parkbot_motion.pose_controller) 단위테스트 — Isaac 없이.

R3a(ROS2 노드 구조 이행 설계서 3절, docs/superpowers/specs/
2026-07-25-ros2-node-refactor-design.md): parking_v4_runner.py 의 main() 안에
중첩돼 있던 drive_to_pose/rotate_in_place 의 "결정" 로직(래치·settle 표본·
median 스무딩·reached 판정)을 뺀 클래스를 여기서 검증한다. 합성 플랜트로는
기존 test_mission_control.py 와 같은 패턴(WheelOdometry 로 body twist 를
적분)을 쓴다 — Isaac 이 전혀 필요 없다.
"""
import math
import sys
import pathlib

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))
from parkbot_motion.pose_controller import PoseController
from parkbot_motion.wheel_odometry import WheelOdometry


def test_converges_to_target_and_latches_done():
    # 합성 플랜트(WheelOdometry)에 ctrl.step 의 명령 twist 를 그대로 먹이면
    # 오차가 줄고, 결국 도달래치(ctrl.done)가 걸려야 한다.
    target = (1.0, 0.5, 90.0)
    ctrl = PoseController(target, pos_tol=0.03, yaw_tol=0.5)
    od = WheelOdometry(x=0.0, z=0.0, yaw=math.radians(0.0))
    d0 = math.hypot(target[0] - od.x, target[1] - od.z)
    dt = 0.05
    reached_done = False
    for _ in range(600):
        fp = (od.x, od.z, math.degrees(od.yaw))
        vx, vy, wz = ctrl.step(fp, dt)
        od.update(vx, vy, wz, dt)
        if ctrl.done:
            reached_done = True
            break
    assert reached_done, "600 스텝 안에 도달래치가 걸리지 않음"
    d1 = math.hypot(target[0] - od.x, target[1] - od.z)
    assert d1 < d0 * 0.2


def test_stopping_latch_holds_zero_twist_once_engaged():
    # 목표에 있다고 먹여 래치를 걸고 나면, 이후 잡음으로 tol 밖 관측이 들어와도
    # 래치가 풀리지 않고 계속 0 twist 를 내야 한다(원본 drive_to_pose 의
    # `stopping` 래치가 다시 안 풀리는 동작 보존).
    target = (0.0, 1.0, 0.0)
    ctrl = PoseController(target, pos_tol=0.03, yaw_tol=0.5)
    dt = 0.05
    at_target = (0.0, 1.0, 0.0)
    for _ in range(100):
        ctrl.step(at_target, dt)
        if ctrl.done:
            break
    assert ctrl.done

    # 마커 잡음으로 갑자기 멀리 있는 관측이 들어와도(예: 오검출) 래치는 안 풀린다.
    vx, vy, wz = ctrl.step((5.0, 5.0, 90.0), dt)
    assert (vx, vy, wz) == (0.0, 0.0, 0.0)
    assert ctrl.done


def test_settle_median_rejects_single_outlier():
    # taskBharden Item3 회귀 재현: 정지한 로봇 주위에서 마커 fix 한 프레임의
    # 잡음이 filt 를 흔들어도(=settle 표본 중 단 하나가 outlier), 강건 중앙값은
    # 그 outlier 를 무시해야 한다. (평균이었다면 outlier 쪽으로 끌려갔을 것.)
    target = (0.0, 0.0, 0.0)
    ctrl = PoseController(target, pos_tol=0.03, yaw_tol=0.5)
    good = (0.0, 0.0, 0.0)
    # yaw 는 good 과 같게 둔다 — 이 테스트는 x/z 중앙값이 outlier 를 씹어내는지만
    # 본다(yaw 는 원본도 median 이 아니라 원형평균이라 단일 outlier 에 그대로
    # 끌린다 — 그건 이 함수의 기존 동작이지 이 테스트의 관심사가 아니다).
    outlier = (5.0, 5.0, 0.0)
    for s in [good] * 9 + [outlier]:
        ctrl.settle_sample(s)

    final_pose, reached = ctrl.finish(fallback_pose=good)
    assert final_pose == (0.0, 0.0, 0.0)
    assert reached
    # 대조: 단순 평균이었다면 outlier 에 끌려가 tol(3cm) 밖으로 튀었을 것.
    mean_x = sum(p[0] for p in [good] * 9 + [outlier]) / 10.0
    assert mean_x > 0.03


def test_reached_false_when_final_smoothed_pose_outside_tolerance():
    # settle 표본들이 일관되게 tol 밖이면(=노이즈가 아니라 실제로 못 미쳤음),
    # median 스무딩을 거쳐도 reached 는 False 여야 한다.
    target = (0.0, 0.0, 0.0)
    ctrl = PoseController(target, pos_tol=0.03, yaw_tol=0.5)
    off_target = (0.10, 0.10, 0.0)   # pos_tol=3cm 인데 일관되게 ~14cm 벗어남
    for _ in range(5):
        ctrl.settle_sample(off_target)

    final_pose, reached = ctrl.finish(fallback_pose=off_target)
    assert not reached
    assert final_pose == off_target


def test_finish_uses_fallback_when_no_settle_samples():
    # 마커가 한 번도 안 잡힌 순수오도 세그먼트(예: ROTCHK) 또는 max_steps 소진으로
    # settle 을 못 밟은 경우: 원본은 filt 를 안 건드렸다 — 여기서는 finish 가
    # fallback_pose 를 그대로 최종 자세로 써야 한다(median 을 만들어내지 않음).
    target = (0.0, 0.0, 45.0)
    ctrl = PoseController(target, pos_tol=0.03, yaw_tol=0.5)
    fallback = (0.001, -0.002, 44.9)
    final_pose, reached = ctrl.finish(fallback_pose=fallback)
    assert final_pose == fallback
    assert reached


def test_rotate_in_place_style_goal_holds_position_only_changes_yaw():
    # rotate_in_place 의 목표 구성(현재 x,z + 새 yaw)을 그대로 재현: 위치는
    # 거의 안 움직이고 yaw 만 목표로 수렴해야 한다.
    start = (2.0, -1.0, 0.0)
    target = (start[0], start[1], 90.0)
    ctrl = PoseController(target, pos_tol=0.03, yaw_tol=0.5)
    od = WheelOdometry(x=start[0], z=start[1], yaw=math.radians(start[2]))
    dt = 0.05
    for _ in range(600):
        fp = (od.x, od.z, math.degrees(od.yaw))
        vx, vy, wz = ctrl.step(fp, dt)
        od.update(vx, vy, wz, dt)
        if ctrl.done:
            break
    assert ctrl.done
    assert math.isclose(od.x, start[0], abs_tol=0.05)
    assert math.isclose(od.z, start[1], abs_tol=0.05)
    yaw_err = abs(((math.degrees(od.yaw) - 90.0 + 180.0) % 360.0) - 180.0)
    assert yaw_err < 1.0
