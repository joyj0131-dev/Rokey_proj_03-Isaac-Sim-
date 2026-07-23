"""편대 모션 엔진 — dock_lift_handoff_mission.HandoffMission의 검증된 폐루프를
navigate_action_server/align_action_server가 공유해 재사용하기 위한 이식.

원본: isaacpjt/Isaac_envo/dock_lift_handoff_mission.py (class HandoffMission).

## 로직 변경 없이 이식(Task 10b 지시) — 원본 라인과 1:1 대조
    FormationMotion._odom             <- HandoffMission._odom              (L86-89)
    FormationMotion._veh              <- HandoffMission._veh               (L91-94)
    FormationMotion._pub              <- HandoffMission._pub               (L96-99)
    FormationMotion._stop_all         <- HandoffMission._stop_all          (L101-103)
    FormationMotion._settle           <- HandoffMission._settle            (L105-111)
    FormationMotion.wait_data         <- HandoffMission._wait_data         (L116-121)
    FormationMotion._omni_step        <- HandoffMission._omni_step         (L123-138)
    FormationMotion.goto_xz           <- HandoffMission._goto_xz           (L140-148)
    FormationMotion.approach_parallel <- HandoffMission._approach_parallel (L150-170)
    FormationMotion.rotate_to         <- HandoffMission._rotate_to         (L172-184)
    FormationMotion.ingress_to        <- HandoffMission._ingress_to        (L186-205)
    FormationMotion.pickup_sequence   <- HandoffMission._on_dock_lift 의 접근+진입부만
                                          (L257-286; 파지/리프트(_grip_lift)·운반(_omni_carry)
                                          이후 단계는 제외 — lift_action_server(Task 10a에서
                                          이미 이식)·carry_to/carry_rotate_to(아래)의 몫)

(_omni_step/ingress_to 안의 world→body 변환식은 formation_driver.body_twist_from_world_error로
위임한다 — Task 9에서 원본과 수치적으로 동일함이 단위테스트로 검증된 순수 함수라 값 변경 없음.
K_LIN 등 게인·톨러런스 상수도 formation_driver에서 이미 대조된 값을 그대로 임포트해 재사용한다.)

(_call_arms/_grip_lift는 이 파일의 이식 대상이 아니다 — Task 10a에서 lift_action_server.py로
이미 이식되었고, 이번 태스크 지시("lift/detect 등 다른 파일 건드리지 마세요")가 그 파일을
범위 밖으로 명시했다.)

## 신규(원본에 없음, best-effort) — carry_to / carry_rotate_to
원본 `_omni_carry`(L226-252)는 "파지 후 전/후/옆 각 1m" 데모 운반만 한다. 실제 P1 플로우는
인계베이(x≈-29.6)에서 목표 주차 슬롯까지(개구부 재통과 포함 ~22m) 편대를 운반해야 하는데,
이 장거리 구간은 원본에 전례가 없다. 아래 두 메서드는 원본의 제어 패턴(같은 body 지령을
두 로봇에 동시에 내려 편대를 유지)만 재사용해 새로 작성한 것이며, 단순 직선/웨이포인트
best-effort다 — 개구부 재통과·장애물 회피 등 실제 경로 튜닝은 Task 12(Isaac GUI)에서
사람이 검증해야 한다(각 메서드 docstring에 TODO 명시).
    FormationMotion.carry_to        — 원본 _omni_carry 안의 move() 헬퍼(고정 vx/vy로 거리
                                       기준 정지)를 "/vehicle/pose 를 목표 (tx,tz)로" 폐루프
                                       추종하도록 일반화.
    FormationMotion.carry_rotate_to — 원본에 아예 없던 "파지 후 편대 회전". rotate_to와 동일한
                                       게인(K_YAW/MAX_YAW/YAW_TOL)으로 두 로봇을 병렬 폐루프
                                       (같은 tick에 동시 명령)로 돌린다 — 강체로 잡은 차량이
                                       한쪽만 돌고 한쪽은 멈춰 있으면 서로 밀고 당기게 되므로
                                       순차 호출(rotate_to 두 번)이 아니라 동시 명령을 택했다.

## ★동시성 패턴(필수)
이 클래스의 메서드는 최대 수분까지 블로킹되는 폐루프다. 이 클래스를 사용하는 노드가 odom/vehicle
구독과 액션서버 콜백을 모두 같은 ReentrantCallbackGroup에 두고 MultiThreadedExecutor로 스핀하지
않으면, 블로킹 루프 도중 odom 콜백이 스케줄되지 않아 self.pose가 갱신되지 않고 폐루프가 수렴하지
못한다(치명적 — 원본 HandoffMission과 동일한 요구사항, 원본 L75/L303-304 대조).
"""
import math
import time

from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.callback_groups import ReentrantCallbackGroup

from parking_robot_system.formation_driver import (
    CONTROL_HZ, INGRESS_SPEED, K_LIN, K_STRAFE, K_YAW, MAX_LIN, MAX_YAW,
    POS_TOL, YAW_TOL, body_twist_from_world_error, clamp, wrap,
)

ROBOTS = ("robot_rear", "robot_front")

# 아래 상수는 dock_lift_handoff_mission.py L28-55 그대로 옮긴 값이다(원본과 대조 완료 —
# 값이 바뀌면 모션이 달라지므로 원본이 바뀌면 함께 갱신할 것). K_LIN 등 게인류는 이미
# formation_driver에 추출·대조돼 있어 그쪽에서 임포트해 재사용한다(위 import 참고).
STEP_TIMEOUT = 90.0        # L28
CORNER_TOL = 0.40          # L30 — 중간 웨이포인트 통과 반경(정지 없이 코너를 돎)
FACE_MZ = math.pi / 2      # L32 — -z 를 향하는 yaw(odom 규약: atan2(-fwd_z, fwd_x))
LANE_Z_REAR = -1.5         # L46 — 개구부 통과 통로(남쪽, rear 전용)
LANE_Z_FRONT = 1.5         # L47 — 개구부 통과 통로(북쪽, front 전용)
NORTH_STAGE_Z = 4.0        # L48 — Pickup 차체 북쪽 끝 밖 북쪽 스테이징 지점
WALL_CLEAR_X = -20.0       # L51 — 서쪽 벽(-18.1) 서쪽, 인계장 바닥 안
DOCK_X = -15.3             # L52 — West 도크 x (robot:dockPose)
APPROACH_TIMEOUT = 300.0   # L55

# 신규(원본에 없음) — carry_to 전용 속도/타임아웃.
# 파지 후 운반은 차량을 강체로 들고 직선(L자) 이동이라 픽업 진입(정밀·저속)보다 빠르게 가도
# 된다. 원본 CARRY_SPEED(0.30) 지령은 롤러 슬립으로 실측 ~0.09m/s에 그쳐, L자 경로
# (~29m)에서 300s 타임아웃을 넘겨 abort(status=6)가 났다(사용자 실측). 그래서 운반 구간만
# 지령 속도를 올린다. 두 로봇 동기가 흐트러지면(차 뒤틀림) 값을 낮춰 조정.
CARRY_SPEED_FAST = 0.9    # 운반 지령 속도. 1.2배(0.90→1.08). 원본 0.30.
CARRY_TO_TIMEOUT = 420.0   # 상향 속도로도 안전하도록 타임아웃 여유 확대(구간별 개별 적용).

# --- 초기 대기 도크(USD robot:dockPose 실측) — 복귀 목표 ---
# West_A_WaitingDock=(-15.3,0,7.8)=front, West_B_WaitingDock=(-15.3,0,-7.8)=rear.
# (이전엔 개구부 lane ±1.5로 잘못 복귀했음 — 실제 초기 위치는 ±7.8.)
DOCK_Z_FRONT = 7.8         # front(West_A) 초기 대기 z
DOCK_Z_REAR = -7.8         # rear(West_B) 초기 대기 z

# 출차 하차(인계베이) 후 복귀 시 차 길이축(z)으로 빠져나올 거리. 차체 반길이 ≈2.9 + 로봇
# 반길이 + 여유. 사용자 실측: 3.5는 바퀴에 살짝 닿아 4.0으로 상향(좀 더 나온 뒤 이동).
BAY_CLEAR_Z = 4.0

# 신규 — 축(axle) 정밀 진입용 정지 허용오차. 원본 ingress_to는 POS_TOL(0.10)에서 멈춰
# 최대 10cm 오차를 허용했는데, 사용자 보고("앞바퀴 리프트 위치가 약간 안 맞음")에 따라
# 픽업 진입만 더 조인다. 폐루프 P제어라 더 가까이 수렴하며, 못 맞춰도 기존처럼 근처에서 정지한다.
INGRESS_TOL = 0.05
# 진입 후 안정(settle) 시간 — 차량이 흔들림 없이 멎도록. 속도↑ 요청 반영해 1.5→1.0으로 단축.
INGRESS_SETTLE = 1.0


class FormationMotion:
    """dock_lift_handoff_mission.HandoffMission의 편대 모션 폐루프를 재사용 가능한 형태로 이식.

    navigate_action_server/align_action_server가 각자 자신의 Node(+ReentrantCallbackGroup)를
    넘겨 `FormationMotion(self, callback_group=grp)`로 생성한다. world 좌표는 모두 USD(XZ,
    +Y상방) 프레임 그대로다 — map 프레임 변환은 호출부(액션서버)의 책임
    (parking_robot_system.frame_transform 참고).
    """

    def __init__(self, node, *, center_x=-29.6, rear_axle_z=-1.93, front_axle_z=1.66,
                 callback_group=None):
        # center_x/rear_axle_z/front_axle_z 기본값은 원본 HandoffMission.__init__의 ROS
        # 파라미터 기본값(L68: center_x=-29.6, rear_axle_z=-1.93, front_axle_z=1.66)과 동일.
        self.node = node
        self.cx = center_x
        self.rear_axle = rear_axle_z
        self.front_axle = front_axle_z
        self.pose = {r: None for r in ROBOTS}   # rid -> (x, z, yaw), USD
        self.veh_x = self.veh_y = self.veh_z = None
        grp = callback_group or ReentrantCallbackGroup()
        for r in ROBOTS:
            node.create_subscription(
                Odometry, f"/{r}/odom",
                lambda m, rid=r: self._odom(rid, m), 10, callback_group=grp)
        node.create_subscription(
            PoseStamped, "/vehicle/pose", self._veh, 10, callback_group=grp)
        self.cmd = {r: node.create_publisher(Twist, f"/{r}/cmd_vel", 10) for r in ROBOTS}

    # ---- 구독 콜백 (원본 L86-94 그대로) ----
    def _odom(self, rid, m):
        q = m.pose.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        self.pose[rid] = (m.pose.pose.position.x, m.pose.pose.position.z, yaw)

    def _veh(self, m):
        self.veh_x = m.pose.position.x
        self.veh_y = m.pose.position.y
        self.veh_z = m.pose.position.z

    # ---- 발행/정지 헬퍼 (원본 L96-111 그대로) ----
    def _pub(self, rid, vx, vy=0.0, wz=0.0):
        t = Twist()
        t.linear.x, t.linear.y, t.angular.z = float(vx), float(vy), float(wz)
        self.cmd[rid].publish(t)

    def _stop_all(self):
        for r in ROBOTS:
            self._pub(r, 0.0)

    def _settle(self, secs=0.5):
        """단계 경계에서 정지 후 잠깐 멈춤 → 관성 흡수(원본 L105-111 그대로)."""
        self._stop_all()
        end = time.time() + secs
        while time.time() < end:
            self._stop_all()
            time.sleep(1.0 / CONTROL_HZ)

    def wait_data(self, timeout=15.0):
        """두 로봇 odom + 차량 pose 수신 대기(원본 _wait_data, L116-121 그대로)."""
        end = time.time() + timeout
        while time.time() < end and (any(self.pose[r] is None for r in ROBOTS)
                                     or self.veh_z is None):
            time.sleep(0.1)
        return all(self.pose[r] is not None for r in ROBOTS) and self.veh_z is not None

    # ---- 원본 _omni_step (L123-138), 로직 변경 없음 ----
    def _omni_step(self, rid, tx, tz, tol=POS_TOL):
        """world (tx,tz)로 향하는 옴니 지령 한 틱 발행. tol 반경 도달 시 True."""
        x, z, yaw = self.pose[rid]
        ex, ez = tx - x, tz - z
        if math.hypot(ex, ez) < tol:
            return True
        fwd, left = body_twist_from_world_error(ex, ez, yaw)
        self._pub(rid, clamp(K_LIN * fwd, MAX_LIN), clamp(K_STRAFE * left, MAX_LIN), 0.0)
        return False

    # ---- 원본 _goto_xz (L140-148), 로직 변경 없음 ----
    def goto_xz(self, rid, tx, tz, timeout=STEP_TIMEOUT):
        """현재 yaw 유지한 채 world (tx,tz)로 옴니 이동(vx,vy). 회전 없음."""
        end = time.time() + timeout
        while time.time() < end:
            if self._omni_step(rid, tx, tz):
                break
            time.sleep(1.0 / CONTROL_HZ)
        self._pub(rid, 0.0)
        return math.hypot(tx - self.pose[rid][0], tz - self.pose[rid][1]) < POS_TOL * 2

    # ---- 원본 _approach_parallel (L150-170), 로직 변경 없음 ----
    def approach_parallel(self, routes, timeout=APPROACH_TIMEOUT):
        """routes: {rid: [(x,z),...]} 두 로봇을 동시에 웨이포인트 체인 따라 옴니 이동.
        중간 웨이포인트는 CORNER_TOL 반경에서 통과(정지 없이 코너를 돌아 부드럽게),
        마지막 웨이포인트만 POS_TOL 로 정밀 정지."""
        idx = {rid: 0 for rid in routes}
        end = time.time() + timeout
        while time.time() < end:
            for rid, wps in routes.items():
                if idx[rid] >= len(wps):
                    self._pub(rid, 0.0)
                    continue
                tx, tz = wps[idx[rid]]
                last = idx[rid] == len(wps) - 1
                if self._omni_step(rid, tx, tz, POS_TOL if last else CORNER_TOL):
                    idx[rid] += 1
            if all(idx[rid] >= len(routes[rid]) for rid in routes):
                break
            time.sleep(1.0 / CONTROL_HZ)
        for rid in routes:
            self._pub(rid, 0.0)
        return all(idx[rid] >= len(routes[rid]) for rid in routes)

    # ---- 원본 _rotate_to (L172-184), 로직 변경 없음 ----
    def rotate_to(self, rid, target_yaw, timeout=90.0):
        """GT yaw 폐루프 회전(느림). 회전 방향 비신뢰라 작은 wz로 수렴.
        인플레이스 회전은 롤러 슬립이 커 느리므로 타임아웃 넉넉히."""
        end = time.time() + timeout
        while time.time() < end:
            yaw = self.pose[rid][2]
            e = wrap(target_yaw - yaw)
            if abs(e) < YAW_TOL:
                break
            self._pub(rid, 0.0, 0.0, clamp(K_YAW * e, MAX_YAW))
            time.sleep(1.0 / CONTROL_HZ)
        self._pub(rid, 0.0)
        return abs(wrap(target_yaw - self.pose[rid][2])) < YAW_TOL * 3

    # ---- 원본 _ingress_to (L186-205), 로직 변경 없음 ----
    def ingress_to(self, rid, target_z, face_yaw, timeout=STEP_TIMEOUT, tol=POS_TOL, cx=None):
        """차 밑으로 진입하며 축(target_z)에 정렬. 중심선(x=cx)과 방위(face_yaw)를
        폐루프 유지 → 진입 중 드리프트로 바퀴에 걸리는 것을 방지. 진입 방향은 target_z
        부호가 알아서 결정(옴니).

        cx: 중심선 x. None이면 self.cx(인계베이 중심). 출차(슬롯 픽업)는 슬롯 x를 넘긴다.
        tol: 정지 허용오차(기본 POS_TOL=0.10). 픽업 정밀 진입은 INGRESS_TOL(0.05)로 조여 호출.

        주의: 아래 yaw 보정 게인(0.6)과 clamp 상한(0.10)은 원본이 K_YAW/MAX_YAW가 아닌
        별도 하드코딩 값을 쓴다(원본 L202 주석: "완만한 방위 유지") — 그대로 유지.
        """
        cx = self.cx if cx is None else cx
        end = time.time() + timeout
        while time.time() < end:
            x, z, yaw = self.pose[rid]
            if abs(z - target_z) < tol and abs(x - cx) < tol * 2:
                break
            ex, ez = cx - x, target_z - z
            fwd, left = body_twist_from_world_error(ex, ez, yaw)
            eyaw = wrap(face_yaw - yaw)
            self._pub(rid, clamp(K_LIN * fwd, INGRESS_SPEED),
                      clamp(K_STRAFE * left, INGRESS_SPEED),  # 중심선 보정도 젠틀히
                      clamp(0.6 * eyaw, 0.10))                # 완만한 방위 유지(원본 그대로)
            time.sleep(1.0 / CONTROL_HZ)
        self._pub(rid, 0.0)
        return abs(self.pose[rid][1] - target_z) < max(POS_TOL, tol) * 3

    # ---- 원본 _on_dock_lift(L254-286)의 접근+양 로봇 진입부만 이식 ----
    def pickup_sequence(self):
        """검증된 미션 픽업 시퀀스(게이트 통과 → rear 진입 → front 진입) 그대로.

        원본 HandoffMission._on_dock_lift L257-286 로직 그대로 — 유일한 차이는 std_srvs
        서비스 응답(resp.success/resp.message)을 (ok, message) 튜플로 반환하는 것뿐(그 밖의
        분기·타임아웃·좌표·settle 위치는 전부 동일). 파지·리프트(_grip_lift)와 운반
        (_omni_carry)은 원본에서도 이 시퀀스 *다음* 단계라 여기 포함하지 않는다.
        """
        if not self.wait_data():
            return False, "데이터 미수신"
        # 게이트 통과는 병렬(rear 남쪽 통로 -1.5, front 북쪽 통로 +1.5 로 분리 → 벽 서쪽).
        gate = {
            "robot_rear":  [(DOCK_X, LANE_Z_REAR), (WALL_CLEAR_X, LANE_Z_REAR)],
            "robot_front": [(DOCK_X, LANE_Z_FRONT), (WALL_CLEAR_X, LANE_Z_FRONT)],
        }
        self.node.get_logger().info("접근: 게이트 통과(병렬)")
        if not self.approach_parallel(gate):
            self._stop_all()
            return False, "게이트 통과 타임아웃"
        self._settle()
        # rear 가 먼저 북쪽 스테이징으로 → -z 회전 → 뒷축까지 깊이 진입.
        self.node.get_logger().info("뒷축 로봇: 북쪽 정렬 → 진입(깊이)")
        self.goto_xz("robot_rear", self.cx, NORTH_STAGE_Z)
        self._settle()
        if not self.rotate_to("robot_rear", FACE_MZ):
            self._stop_all()
            return False, "rear 회전 실패"
        self._settle()
        self.ingress_to("robot_rear", self.rear_axle, FACE_MZ, timeout=140.0, tol=INGRESS_TOL)
        self._settle(INGRESS_SETTLE)
        # front 가 같은 북쪽 스테이징으로(rear 는 이미 깊이 들어가 비어 있음) → 앞축 진입.
        self.node.get_logger().info("앞축 로봇: 북쪽 정렬 → 진입")
        self.goto_xz("robot_front", self.cx, NORTH_STAGE_Z)
        self._settle()
        if not self.rotate_to("robot_front", FACE_MZ):
            self._stop_all()
            return False, "front 회전 실패"
        self._settle()
        self.ingress_to("robot_front", self.front_axle, FACE_MZ, tol=INGRESS_TOL)
        self._settle(INGRESS_SETTLE)
        return True, "픽업 시퀀스 완료"

    # ---- 신규(원본에 없음, best-effort) ----
    def carry_to(self, tx_usd, tz_usd, timeout=CARRY_TO_TIMEOUT, tol=POS_TOL):
        """파지 후 편대(둘 다 동일 body 지령)를 /vehicle/pose 기준 world (tx_usd,tz_usd)로 이동.

        원본 _omni_carry 의 move() 헬퍼(L231-241: 고정 vx/vy를 두 로봇에 동시 발행하고
        거리(getter() 델타)가 dist에 도달하면 정지)를 "임의 목표 (tx,tz)까지 폐루프 추종"으로
        일반화한 것 — 원본에는 없던 신규 코드다.

        TODO(Task 12, best-effort): 인계베이(x≈-29.6)→목표 슬롯까지 개구부 재통과 포함
        ~22m 장거리는 원본에 전례가 없다. 여기서는 단순 직선(웨이포인트 없음) 폐루프만
        구현했다 — 개구부(서쪽 벽 x≈-18.1, z∈[-4.5,4.5]) 재통과·다른 로봇/장애물 회피 등
        실제 경로는 Isaac GUI에서 사람이 관찰하며 튜닝해야 한다(예: 호출부에서 carry_to를
        여러 웨이포인트로 나눠 순차 호출). 편대 대표 yaw는 robot_rear의 실측 yaw를 쓴다
        (두 로봇이 같은 방향을 향한다는 원본의 편대 가정 — L229 "두 로봇 다 -z 향함" — 을
        일반 yaw로 확장한 것; rear 데이터가 아직 없으면 FACE_MZ로 대체).
        """
        if self.veh_x is None or self.veh_z is None:
            return False
        # 로봇별 '자기 yaw'로 지령을 계산한다(둘 다 같은 world 방향 (ex,ez)로 이동). 두 로봇
        # yaw가 크게 달라질 때 대비한 올바른 일반화이긴 하나, headless A/B 실측(2026-07-23,
        # 물리 120)에서는 공유yaw(3.1°) vs 로봇별yaw(4.1°)로 삐뚤이 유의미하게 줄지 않았다
        # — 이 씬에선 두 로봇 yaw 차가 작아 프레임 선택의 효과가 미미. 실제 삐뚤의 지배 원인은
        # 아래 wz=0.0, 즉 '차 방향(heading) 피드백 부재'다: 두 로봇이 동일 지령을 밀기만 하고
        # 파지·마찰 비대칭이 만든 차 회전을 아무도 보정하지 않아 운반 중 heading이 단조로
        # 90°→94° 드리프트한다(DEBUG_LOG 참고). 진짜 해법은 가상중심 협조제어(설계노트 a37a1fc)
        # — carry_to에 차 heading P제어(차분 스트래이프)를 넣는 것. 여기선 아직 미적용.
        end = time.time() + timeout
        while time.time() < end:
            ex, ez = tx_usd - self.veh_x, tz_usd - self.veh_z
            if math.hypot(ex, ez) < tol:
                break
            for r in ROBOTS:
                rp = self.pose.get(r)
                ryaw = rp[2] if rp is not None else FACE_MZ
                fwd, left = body_twist_from_world_error(ex, ez, ryaw)
                self._pub(r, clamp(K_LIN * fwd, CARRY_SPEED_FAST),
                          clamp(K_STRAFE * left, CARRY_SPEED_FAST), 0.0)
            time.sleep(1.0 / CONTROL_HZ)
        self._stop_all()
        return math.hypot(tx_usd - self.veh_x, tz_usd - self.veh_z) < tol * 3

    def return_both_to_docks(self):
        """주차·하차 후 두 로봇을 동시에 초기 대기 도크로 복귀(사용자 요구: 동시 + 초기위치).

        초기 위치(USD robot:dockPose 실측): front=West_A(-15.3,+7.8), rear=West_B(-15.3,-7.8).
        차량을 주차하면 슬롯 안쪽(뒤)은 공간이 없으므로 반드시 앞(통로 쪽)으로 나온다:
          각 로봇 웨이포인트 체인 ── ① 현재 x 유지한 채 자기 통로 차로(rear −1.5 / front +1.5)로
          앞(통로)으로 빠져나옴 → ② 도크 x(-15.3)까지 서진 → ③ 자기 도크 z(rear −7.8 / front +7.8)로.
        approach_parallel로 두 로봇을 '동시에' 웨이포인트 체인 따라 이동한다. rear/front가 서로
        다른 차로(±1.5)로 이동하고, rear가 처음부터 front보다 남쪽에 있어(픽업 진입 순서상)
        z 순서가 유지되므로 통로에서 겹치지 않는다.
        """
        routes = {}
        for rid, lane_z, dock_z in (("robot_rear", LANE_Z_REAR, DOCK_Z_REAR),
                                    ("robot_front", LANE_Z_FRONT, DOCK_Z_FRONT)):
            cur = self.pose.get(rid)
            if cur is None:
                return False
            cur_x = cur[0]
            routes[rid] = [(cur_x, lane_z), (DOCK_X, lane_z), (DOCK_X, dock_z)]
        self.node.get_logger().info("복귀(동시): 두 로봇 앞으로→통로→초기 도크")
        ok = self.approach_parallel(routes)
        self._stop_all()
        return ok

    def return_from_bay(self):
        """출차 하차(인계베이) 후 도크 복귀 — 두 단계(사용자 실측 반영):

        ① 백아웃(정밀): 차 밑에서 차 길이축(z)으로 '완전히' 빠져나온다. rear 남(-z)/front 북(+z)
           으로 |z|=BAY_CLEAR_Z까지(단일 웨이포인트 → CORNER_TOL로 안 자르고 정밀 정지). 이렇게
           확실히 나온 뒤에 이동해야 좌우 바퀴를 안 스친다(진입 역방향).
        ② 일직선 전진 후 개구부 통과: 백아웃 위치(x≈-29.6, z=±4.0)에서 곧바로 통로 차로로
           대각선 이동하면 차 폭 안에 있는 동안 z가 중심으로 당겨져 바퀴를 친다(사용자 실측).
           그래서 먼저 z를 유지한 채 '일직선으로 동진'해 차 밖(WALL_CLEAR_X)으로 나간 뒤,
           비로소 통로 차로(rear −1.5 / front +1.5)로 정렬하고 개구부(x≈-18.1, z∈[-4.5,4.5])를
           지나 각자 초기 도크로 간다. rear/front가 서로 다른 방향/차로라 겹치지 않는다.
        """
        # ① 정밀 백아웃 — 차 밖으로 완전히
        backout = {}
        for rid, clear_z in (("robot_rear", -BAY_CLEAR_Z), ("robot_front", BAY_CLEAR_Z)):
            cur = self.pose.get(rid)
            if cur is None:
                return False
            backout[rid] = [(cur[0], clear_z)]   # 단일 웨이포인트 → 정밀 정지(안 자름)
        self.node.get_logger().info("출차 복귀①: 차 길이축으로 완전히 빠져나옴")
        if not self.approach_parallel(backout):
            self._stop_all()
            return False
        self._settle()
        # ② 일직선 동진(z 유지, 차 밖으로) → 통로 차로 정렬 → 개구부 통과 → 초기 도크
        transit = {
            "robot_rear":  [(WALL_CLEAR_X, -BAY_CLEAR_Z), (WALL_CLEAR_X, LANE_Z_REAR),
                            (DOCK_X, LANE_Z_REAR), (DOCK_X, DOCK_Z_REAR)],
            "robot_front": [(WALL_CLEAR_X, BAY_CLEAR_Z), (WALL_CLEAR_X, LANE_Z_FRONT),
                            (DOCK_X, LANE_Z_FRONT), (DOCK_X, DOCK_Z_FRONT)],
        }
        self.node.get_logger().info("출차 복귀②: 일직선 동진 → 개구부 통로 지나 초기 도크로")
        ok = self.approach_parallel(transit)
        self._stop_all()
        return ok

    # ---- 출차(EXIT) 신규(best-effort) — 슬롯 픽업 / 베이 운반 ----
    def ingress_parallel(self, targets, timeout=140.0, tol=INGRESS_TOL):
        """여러 로봇을 '동시에' 각자의 축으로 진입시킨다(ingress_to의 병렬판).

        targets: {rid: (cx, target_z, face_yaw)}. 매 tick 각 로봇에 ingress_to와 동일한
        폐루프(중심선 cx·방위 face_yaw 유지 + 축 target_z로 옴니 진입) 지령을 동시에 낸다.
        두 로봇이 서로 다른 차로에서 시작하고 축 순서가 유지되면 겹치지 않는다."""
        done = {rid: False for rid in targets}
        end = time.time() + timeout
        while time.time() < end:
            for rid, (cx, target_z, face_yaw) in targets.items():
                if done[rid]:
                    self._pub(rid, 0.0)
                    continue
                x, z, yaw = self.pose[rid]
                if abs(z - target_z) < tol and abs(x - cx) < tol * 2:
                    done[rid] = True
                    self._pub(rid, 0.0)
                    continue
                ex, ez = cx - x, target_z - z
                fwd, left = body_twist_from_world_error(ex, ez, yaw)
                eyaw = wrap(face_yaw - yaw)
                self._pub(rid, clamp(K_LIN * fwd, INGRESS_SPEED),
                          clamp(K_STRAFE * left, INGRESS_SPEED),
                          clamp(0.6 * eyaw, 0.10))
            if all(done.values()):
                break
            time.sleep(1.0 / CONTROL_HZ)
        self._stop_all()
        return all(abs(self.pose[r][1] - targets[r][1]) < max(POS_TOL, tol) * 3 for r in targets)

    def pickup_at_slot(self, slot_x, slot_z):
        """출차: 슬롯에 주차된 차 밑으로 '두 로봇이 동시에' 진입(사용자 요구, best-effort).

        입차는 방향 유지한 채 차를 넣으므로 축 오프셋은 인계베이와 동일:
        rear축=slot_z+rear_axle, front축=slot_z+front_axle. 두 로봇을 서로 다른 통로 차로
        (rear −1.5 / front +1.5)에 세운 뒤 FACE_MZ로 정렬하고, 각자 축으로 '동시' 진입한다.
        차로 배정상 깊은 축으로 가는 로봇이 항상 그 방향 바깥 차로에서 출발하므로(A/B열 모두)
        진입 중 z 순서가 유지되어 서로 막지 않는다 — 순차 진입에서 생기던 엉킴을 없앤다.

        TODO(사용자 sim 튜닝): 통로 차로·진입 속도/정밀도는 실측 관찰로 미세조정.
        """
        if not self.wait_data():
            return False, "데이터 미수신"
        rear_t = slot_z + self.rear_axle    # 슬롯 rear축 z
        front_t = slot_z + self.front_axle  # 슬롯 front축 z
        # ① 접근: 두 로봇을 슬롯 열의 통로 차로로(병렬, 서로 다른 차로라 안 겹침)
        approach = {}
        for rid, lane in (("robot_rear", LANE_Z_REAR), ("robot_front", LANE_Z_FRONT)):
            cur = self.pose.get(rid)
            if cur is None:
                return False, "pose 없음"
            approach[rid] = [(cur[0], lane), (slot_x, lane)]
        self.node.get_logger().info(f"출차 접근: 슬롯 {slot_x:.1f} 열 통로로")
        if not self.approach_parallel(approach):
            self._stop_all()
            return False, "슬롯 접근 타임아웃"
        self._settle()
        # ② 두 로봇 동시에 FACE_MZ로 정렬(빈 몸이라 제자리 회전 무방)
        self.carry_rotate_to(FACE_MZ)
        self._settle()
        # ③ 두 로봇 동시에 각자 축으로 진입
        self.node.get_logger().info("출차: 두 로봇 동시 진입")
        self.ingress_parallel({
            "robot_front": (slot_x, front_t, FACE_MZ),
            "robot_rear": (slot_x, rear_t, FACE_MZ),
        })
        self._settle(INGRESS_SETTLE)
        return True, "슬롯 픽업 완료(동시)"

    def carry_to_bay(self, bay_x, bay_z):
        """출차 운반: 슬롯에서 통로로 나와 통로 따라 인계베이로(입차 carry의 역방향, L자).
          ① carry_to(현재 veh_x, 0) → 슬롯 밖 통로로,
          ② carry_to(bay_x, bay_z) → 통로 따라 베이 한가운데로.
        """
        if self.veh_x is None or self.veh_z is None:
            return False
        self.node.get_logger().info("출차 운반: 슬롯→통로")
        ok = self.carry_to(self.veh_x, 0.0)
        if ok:
            self.node.get_logger().info(f"출차 운반: 통로→인계베이({bay_x:.1f},{bay_z:.1f})")
            ok = self.carry_to(bay_x, bay_z)
        return ok

    def carry_rotate_to(self, target_yaw, timeout=90.0):
        """파지 후 두 로봇을 target_yaw로 회전(강체로 잡은 차량이 함께 회전) — best-effort.

        원본에는 파지 후 회전 시퀀스가 아예 없다(원본은 전/후/옆 직선 운반만 함, _omni_carry
        L226-252). rotate_to와 동일한 게인(K_YAW/MAX_YAW/YAW_TOL)을 재사용하되, 두 로봇을
        순차 호출(rotate_to 두 번)이 아니라 같은 tick에 동시 명령한다 — 차량을 강체로 잡고
        있다면 한쪽만 돌고 한쪽이 정지해 있을 때 서로 밀고 당기게 되기 때문이다.

        TODO(Task 12, best-effort): 실제로 두 로봇의 각속도가 충분히 동기화되는지, 그립에
        유격이 있어 순차 회전이 오히려 더 매끄러울 수 있는지는 Isaac GUI 관찰로만 확인
        가능하다 — 여기서는 "동시 명령"이 더 물리적으로 타당하다는 판단만으로 구현했다.
        """
        done = {r: False for r in ROBOTS}
        end = time.time() + timeout
        while time.time() < end:
            for rid in ROBOTS:
                if done[rid]:
                    continue
                yaw = self.pose[rid][2]
                e = wrap(target_yaw - yaw)
                if abs(e) < YAW_TOL:
                    done[rid] = True
                    self._pub(rid, 0.0)
                    continue
                self._pub(rid, 0.0, 0.0, clamp(K_YAW * e, MAX_YAW))
            if all(done.values()):
                break
            time.sleep(1.0 / CONTROL_HZ)
        self._stop_all()
        return all(abs(wrap(target_yaw - self.pose[r][2])) < YAW_TOL * 3 for r in ROBOTS)
