"""폐루프 위치제어 정책 — Isaac/rclpy 비의존, 순수 Python. 단위테스트 대상.

R3a(ROS2 노드 구조 이행 설계서, docs/superpowers/specs/2026-07-25-ros2-node-refactor-design.md
3절): ``isaacpjt/Isaac_envo/parking_v4_runner.py`` 의 ``main()`` 안에 중첩돼 있던
``drive_to_pose``/``rotate_in_place`` 의 "무엇을 할지" 결정 로직(정책)만 여기로
추출한다. "어떻게 관측/구동할지"(휠 관절 속도 읽기, ``filt.predict_body``,
``detect_current``/``localize_pose``/``_apply_fix``, ``art.set_joint_velocity_targets``,
``app.update()``)는 여전히 Isaac 에 묶인 배관(plumbing)이라 러너에 남는다 —
이 클래스는 그 배관이 매 스텝 ``(fused_pose, dt)`` 를 먹여주면 다음에 명령할
바디 twist ``(vx, vy, wz)`` 를 뱉을 뿐, 마커·관절·앱을 전혀 모른다.

행동 보존: 게인/허용오차/가감속/settle 프레임 수(30) 기본값은 러너의 기존
``drive_to_pose`` 시그니처·``LINEAR_ACCEL``/``LINEAR_DECEL``/``ANGULAR_ACCEL``
상수와 동일한 값이다(숫자를 바꾸지 않았다 — 이 파일이 새 진실원이 아니라,
러너가 여전히 진실원인 상수를 명시적으로 넘겨준다는 점에 유의. 기본값은
독립 사용/테스트 편의를 위한 것일 뿐이다).
"""
import math

from parkbot_motion.mecanum_kinematics import slew_twist
from parkbot_motion.mission_control import body_twist_toward

# 러너의 LINEAR_ACCEL/LINEAR_DECEL/ANGULAR_ACCEL(parking_v4_runner.py 88-90행)과
# 동일한 값 — 그쪽이 여전히 진실원이며, 실제 호출자(러너)는 자신의 상수를
# 명시적으로 넘겨야 한다(이 기본값에 암묵적으로 기대지 말 것). 여기 있는 이유는
# 오직 단위테스트/독립 사용에서 인자를 안 넘겨도 되게 하기 위함이다.
_DEFAULT_LINEAR_ACCEL = 0.5
_DEFAULT_LINEAR_DECEL = 0.8
_DEFAULT_ANGULAR_ACCEL = 0.8
_DEFAULT_SETTLE_FRAMES = 30

# 2026-07-28 라이브 실측 버그: 잔차가 tol 바로 위(yaw 0.5~0.56°, tol=0.5°)로
# settle 될 때 body_twist_toward 의 P 제어 출력이 너무 작아(yaw_gain=1.2 ×
# 0.0087rad ≈ 0.01rad/s) 휠 정지마찰(stiction)을 못 이겨 로봇이 실제로는
# 전혀 안 움직였다(GT 로 58초 동안 yaw 0.500°에 고정 확인). 그 결과 done 이
# 다시는 True 가 안 돼 settle 재제어(§ pose_controller_node.py)가 소진될
# 때까지 무한 대기 — 목표 도달과 무관하게 "명령 자체가 물리적으로 무의미"한
# 경우다. 아직 done 이 아닌 스텝에서만(=계속 몰아야 하는 상황에서만) 최소
# 명령 크기를 강제해 이 교착을 막는다 — done 이면 목표가 그대로 0 이라
# 이 바닥값의 영향을 받지 않는다.
_MIN_ANG_CMD = 0.05    # rad/s
_MIN_LIN_CMD = 0.03    # m/s


def _median_settle_pose(poses):
    """settle 창에서 모은 (x,z,yaw_deg) 표본들의 중앙값(x,z)+원형평균(yaw).

    parking_v4_runner.py 의 drive_to_pose 가 하던 것과 완전히 동일한 계산
    (Task 3b-harden Item3): 정지한 로봇 주위에서 마커 fix 자체의 프레임간
    잡음이 filt 를 미세하게 흔드는 것을, 창 전체의 강건 중앙값으로 눌러
    마지막 한 표본의 잡음이 reached 판정을 뒤집지 않게 한다.
    """
    xs = sorted(p[0] for p in poses)
    zs = sorted(p[1] for p in poses)
    n = len(xs)
    mid = n // 2
    med_x = xs[mid] if n % 2 else 0.5 * (xs[mid - 1] + xs[mid])
    med_z = zs[mid] if n % 2 else 0.5 * (zs[mid - 1] + zs[mid])
    sy = sum(math.sin(math.radians(p[2])) for p in poses)
    cy = sum(math.cos(math.radians(p[2])) for p in poses)
    med_yaw = math.degrees(math.atan2(sy, cy))
    return (med_x, med_z, med_yaw)


class PoseController:
    """목표 (x,z,yaw_deg) 로의 폐루프 주행 정책 한 세그먼트(=drive_to_pose 한 번 호출).

    사용법(러너의 새 thin drive_to_pose 가 하는 일):
      1. 매 제어 스텝마다 ``vx, vy, wz = ctrl.step(filt.pose(), dt)`` 를 호출해
         이번 스텝에 명령할 바디 twist 를 얻고, 그걸로 휠 속도를 계산해 구동한다.
      2. ``ctrl.done`` 이 True 가 되면(도달래치 + 감속이 실제로 0 에 도달) settle
         창을 돈다: 매 프레임 마커 fix 를 (여전히 러너가) filt 에 반영한 뒤
         ``ctrl.settle_sample(filt.pose())`` 로 표본을 넘긴다. 프레임 수는
         ``ctrl.settle_frames``.
      3. 루프 종료 후(위 settle 을 다 돌았든, max_steps 로 안전상한에 걸려
         못 돌았든) ``final_pose, reached = ctrl.finish(filt.pose())`` 를 부르고,
         ``final_pose`` 가 None 이 아니면 ``filt.set_pose(*final_pose)`` 로 되써서
         이후(다음 세그먼트·리포팅)에도 중앙값이 반영되게 한다.

    래치·settle·median 의미는 원본 drive_to_pose 와 동일하게 보존된다:
    - ``done`` 이 한 번 True 가 되면(=stopping 래치) 그 뒤로 fused_pose 가 다시
      허용오차 밖으로 흔들려도 계속 0 twist 를 명령한다(래치는 절대 풀리지 않음).
    - fused_pose 가 None(아직 시딩도 fix 도 없음)이면 안전하게 0 twist 로 감속.
    - settle 표본이 하나도 없으면(마커가 한 번도 안 잡힌 순수오도 세그먼트, 또는
      max_steps 소진으로 settle 자체를 못 밟은 경우) ``finish()`` 는 median 대신
      호출자가 넘긴 ``fallback_pose`` 를 그대로 최종 자세로 쓴다 — 원본이 이
      경우 filt 를 안 건드리던 것과 동일하다.
    """

    def __init__(self, target_xzyaw, *, pos_gain=0.8, yaw_gain=1.2,
                 max_lin=0.25, max_ang=0.6, pos_tol=0.03, yaw_tol=0.5,
                 yaw_min_cmd=0.0,
                 linear_accel=_DEFAULT_LINEAR_ACCEL,
                 linear_decel=_DEFAULT_LINEAR_DECEL,
                 angular_accel=_DEFAULT_ANGULAR_ACCEL,
                 settle_frames=_DEFAULT_SETTLE_FRAMES):
        self.target = target_xzyaw
        self.pos_gain = pos_gain
        self.yaw_gain = yaw_gain
        self.max_lin = max_lin
        self.max_ang = max_ang
        self.pos_tol = pos_tol
        self.yaw_tol = yaw_tol
        self.yaw_min_cmd = yaw_min_cmd
        self.linear_accel = linear_accel
        self.linear_decel = linear_decel
        self.angular_accel = angular_accel
        self.settle_frames = settle_frames

        self.steps = 0
        self._stopping = False
        self._cur_tw = (0.0, 0.0, 0.0)
        self._settle_poses = []

    def step(self, fused_pose, dt):
        """한 제어 스텝: fused_pose(x,z,yaw_deg 또는 None), dt[s] -> (vx,vy,wz).

        원본 drive_to_pose 루프 본문의 "제어" 절과 동일한 분기·순서:
        fused_pose 가 None -> 0 목표; 아직 안 멈췄으면 body_twist_toward 로 목표
        twist 산출, done 이면 래치를 걸고 목표를 0 으로 덮어씀; 이미 멈췄으면
        계속 0 목표. 그 목표를 매 스텝 slew_twist 로 가감속 제한한다(fused_pose
        가 None 이거나 이미 멈춘 스텝도 예외 없이 slew 를 거친다 — 원본과 동일).
        """
        self.steps += 1
        if fused_pose is None:
            target_tw = (0.0, 0.0, 0.0)
        elif not self._stopping:
            tvx, tvy, twz, done = body_twist_toward(
                fused_pose, self.target, pos_gain=self.pos_gain, yaw_gain=self.yaw_gain,
                max_lin=self.max_lin, max_ang=self.max_ang,
                pos_tol=self.pos_tol, yaw_tol=self.yaw_tol, yaw_min_cmd=self.yaw_min_cmd)
            if done:
                self._stopping = True
                target_tw = (0.0, 0.0, 0.0)
            else:
                # § 위 _MIN_ANG_CMD/_MIN_LIN_CMD: 아직 도달 전인데 P 출력이
                # 정지마찰 문턱보다 작으면 부호를 보존한 채 문턱까지 올린다.
                if 0.0 < abs(twz) < _MIN_ANG_CMD:
                    twz = math.copysign(_MIN_ANG_CMD, twz)
                if 0.0 < abs(tvx) < _MIN_LIN_CMD:
                    tvx = math.copysign(_MIN_LIN_CMD, tvx)
                if 0.0 < abs(tvy) < _MIN_LIN_CMD:
                    tvy = math.copysign(_MIN_LIN_CMD, tvy)
                target_tw = (tvx, tvy, twz)
        else:
            target_tw = (0.0, 0.0, 0.0)

        self._cur_tw = slew_twist(self._cur_tw, target_tw, dt,
                                   linear_accel=self.linear_accel,
                                   linear_decel=self.linear_decel,
                                   angular_accel=self.angular_accel)
        return self._cur_tw

    @property
    def done(self):
        """도달래치가 걸렸고 감속된 twist 가 실제로 (0,0,0) 에 도달했는가.

        원본의 루프 종료 조건 ``stopping and cur_tw == (0.0, 0.0, 0.0)`` 과 동일.
        """
        return self._stopping and self._cur_tw == (0.0, 0.0, 0.0)

    def resume(self):
        """settle 결과가 허용오차 밖일 때 **재제어**를 위해 도달래치를 푼다.

        원래 래치(``_stopping``)는 절대 안 풀리는 설계였지만, 그러면 회전 중
        노이즈 낀 yaw 추정이 한 프레임 tol 안에 들어와 래치→정지→settle 중앙값이
        tol 밖이면 **재제어 없이 실패**한다(실측 follow 91.34°/목표90°). 노드가
        settle 후 reached=False 면 이걸 불러 정지래치·현재twist·settle표본을 리셋,
        다시 DRIVING 으로 돌려 목표로 계속 몬다. steps(총 워치독)는 보존한다."""
        self._stopping = False
        self._cur_tw = (0.0, 0.0, 0.0)
        self._settle_poses = []

    def settle_sample(self, fused_pose):
        """settle 창 한 프레임의 관측(fused_pose 또는 None)을 표본에 추가.

        None 은 버린다(원본 ``if fp_settle is not None: settle_poses.append(...)``
        와 동일) — 마커 fix 가 그 프레임에 없어도 filt 가 이미 시딩돼 있으면
        fused_pose 는 보통 None 이 아니다(직전 값 유지).
        """
        if fused_pose is not None:
            self._settle_poses.append(fused_pose)

    def finish(self, fallback_pose):
        """세그먼트 종료 처리: (최종 자세, reached) 를 반환.

        settle 표본이 있으면 중앙값(x,z)+원형평균(yaw)으로 스무딩한 자세를,
        없으면 ``fallback_pose``(호출 시점의 filt.pose())를 그대로 최종 자세로
        쓴다. reached 는 그 최종 자세를 target 에 다시 견줘 재판정한다(루프
        중간에 래치한 done 값을 쓰지 않음 — 원본과 동일).

        호출자는 반환된 final_pose 가 None 이 아니면 ``filt.set_pose(*final_pose)``
        로 되써야 한다(median 스무딩이 filt 상태에도 반영되도록 — 원본의
        ``filt.set_pose(med_x, med_z, med_yaw)`` 와 동일한 부작용을 얻기 위함).
        """
        if self._settle_poses:
            final_pose = _median_settle_pose(self._settle_poses)
        else:
            final_pose = fallback_pose

        if final_pose is None:
            reached = False
        else:
            _, _, _, reached = body_twist_toward(
                final_pose, self.target, pos_gain=self.pos_gain, yaw_gain=self.yaw_gain,
                max_lin=self.max_lin, max_ang=self.max_ang,
                pos_tol=self.pos_tol, yaw_tol=self.yaw_tol)
        return final_pose, reached
