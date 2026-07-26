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

`pickup_sequence`는 최초에는 HandoffMission._on_dock_lift의 순차 진입부를 이식했지만,
현재는 rear가 차량 남쪽에서 뒷축으로, front가 북쪽에서 앞축으로 동시에 진입하도록 변경됐다.
파지·리프트는 이 파일이 아니라 lift_action_server가 담당한다.

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
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from parking_robot_interfaces.msg import FormationStop, ObstacleAlert, SafetyState
from parking_control.core.obstacle_scope import (
    obstacle_affects_team,
    robot_team_role,
)

from parking_robot_system.formation_driver import (
    CONTROL_HZ, INGRESS_SPEED, K_LIN, K_STRAFE, K_YAW, MAX_LIN, MAX_YAW,
    POS_TOL, YAW_TOL, body_twist_from_world_error, clamp, formation_heading,
    heading_hold_omega, rigid_body_world_velocity, wrap,
)
# carry_rotate_to(차를 든 채 제자리 회전)의 강체(ω×r) 계산 — 2026-07-24 배선.
# 예전엔 이 컨트롤러가 테스트만 통과하고 실제 로봇 코드엔 연결된 적이 없었다
# (두 로봇이 각자 같은 목표각으로 "따로" 도는 단순한 방식만 실사용됐음). 이제
# 실제로 여기서 가져다 쓴다 — 원리는 pivot_rotate_controller.py docstring 참고.
from parking_control.core.pivot_rotate_controller import PivotRotateController, Pose2D

# 아래 상수는 dock_lift_handoff_mission.py L28-55 그대로 옮긴 값이다(원본과 대조 완료 —
# 값이 바뀌면 모션이 달라지므로 원본이 바뀌면 함께 갱신할 것). K_LIN 등 게인류는 이미
# formation_driver에 추출·대조돼 있어 그쪽에서 임포트해 재사용한다(위 import 참고).
#
# 2026-07-24: DOCK_X/WALL_CLEAR_X/NORTH_STAGE_Z/SOUTH_STAGE_Z/DOCK_Z_FRONT/DOCK_Z_REAR는
# 예전 "인계베이 하나(center_x=-29.6)를 입/출차가 공유" 레이아웃 전용 상수였다. v3
# 레이아웃(입차 전용 인계지점/도크, 출차 전용 인계지점/도크 물리 분리, 로봇 4대 배치
# 확정)으로 바뀌면서 이 값들은 FormationMotion 생성자 파라미터(handoff_x/handoff_z/
# gate_x/dock_rear/dock_front)로 옮겼다 — 아래 STAGE_OFFSET류는 인계지점 "중심 기준
# 상대 오프셋"이라 위치가 바뀌어도 그대로 재사용한다(차량 규격 자체는 안 바뀌므로).
STEP_TIMEOUT = 90.0        # L28
CORNER_TOL = 0.40          # L30 — 중간 웨이포인트 통과 반경(정지 없이 코너를 돎)
FACE_MZ = math.pi / 2      # L32 — -z 를 향하는 yaw(odom 규약: atan2(-fwd_z, fwd_x))
FACE_PZ = -math.pi / 2     # +z 를 향하는 yaw — rear가 차량 남쪽에서 뒷축으로 진입할 때 사용
LANE_Z_REAR = -1.5         # L46 — 로봇 개별 차로(남쪽, rear 전용) — 인계지점 중심 기준 상대값
LANE_Z_FRONT = 1.5         # L47 — 로봇 개별 차로(북쪽, front 전용) — 인계지점 중심 기준 상대값
NORTH_STAGE_OFFSET = 4.0   # L48 — 차체 북쪽 끝 밖 북쪽 스테이징(인계지점 중심 기준 상대값)
SOUTH_STAGE_OFFSET = -4.0  # 차체 남쪽 끝 밖 — rear 전용 스테이징(인계지점 중심 기준 상대값)
APPROACH_TIMEOUT = 300.0   # L55

# 신규(원본에 없음) — carry_to 전용 속도/타임아웃.
# 파지 후 운반은 차량을 강체로 들고 직선(L자) 이동이라 픽업 진입(정밀·저속)보다 빠르게 가도
# 된다. 원본 CARRY_SPEED(0.30) 지령은 롤러 슬립으로 실측 ~0.09m/s에 그쳐, L자 경로
# (~29m)에서 300s 타임아웃을 넘겨 abort(status=6)가 났다(사용자 실측). 그래서 운반 구간만
# 지령 속도를 올린다. 두 로봇 동기가 흐트러지면(차 뒤틀림) 값을 낮춰 조정.
CARRY_SPEED_FAST = 0.9    # 운반 지령 속도. 1.2배(0.90→1.08). 원본 0.30.
CARRY_TO_TIMEOUT = 420.0   # 상향 속도로도 안전하도록 타임아웃 여유 확대(구간별 개별 적용).

# 운반 heading hold — 두 로봇+차량을 하나의 가상 강체로 제어한다.
CARRY_HEADING_KP = 0.8
CARRY_HEADING_MAX_OMEGA = 0.10
CARRY_HEADING_DEADBAND = math.radians(0.10)
CARRY_HEADING_TOL = math.radians(0.50)

# 인계 후(하차/승차) 복귀 시 차 길이축(z)으로 빠져나올 거리. 차체 반길이 ≈2.9 + 로봇
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

    def __init__(self, node, *, rear_id="entry_lead", front_id="entry_follow",
                 handoff_x=-8.5, handoff_z=7.075, gate_x=-12.55,
                 dock_rear=(-3.2, 2.2), dock_front=(-1.2, 2.2), lane_z=6.875,
                 rear_axle_z=-1.93, front_axle_z=1.66,
                 callback_group=None):
        # rear_id/front_id: 이 편대가 실제로 제어할 물리 로봇 이름(토픽/서비스 네임스페이스에
        # 그대로 쓰인다) — 입차/출차 전용 로봇쌍을 분리하려면 액션서버 생성 시 다른 이름을
        # 넘기면 된다(하드코딩 대신 파라미터화, 2026-07-24). 실제 로봇 ID는
        # src/parkbot_aruco/parkbot_aruco/site_map_v4.ROBOTS =
        # (entry_lead, entry_follow, exit_lead, exit_follow) — 아래 기본값은 그중
        # entry_lead/entry_follow(2026-07-25, 이전 robot_rear/robot_front 오기 수정).
        #
        # handoff_x/handoff_z/gate_x/dock_rear/dock_front/lane_z: 2026-07-25 v4
        # 레이아웃 기본값(입차 전용). parking_environment_v4.usd ArucoMarkerPreview
        # 실측 좌표(USD 프레임, 이 클래스가 쓰는 것과 동일 좌표계) 기반이다. ★주의:
        # v4 에셋의 aruco:note는 이 좌표를 "출차"라고 표기하지만, 그건 에셋 제작자
        # 라벨일 뿐이고 우리 팀 규약(site_map_v4.py: 입차=z 양수)에서는 ENTRY다 —
        # 이전 버전은 이 반전을 놓쳐 입/출차 좌표가 통째로 뒤바뀌어 있었다.
        #   handoff = W_OUT 마커(-8.5, 7.075) — 차량이 로봇에게 실제 인계되는 지점
        #   gate_x  = GATE_OUT 마커 x(-12.55) — 인계지점 진입 전 통과하는 게이트
        #   dock_rear/dock_front = D_OUT_1/D_OUT_2(-3.2,2.2)/(-1.2,2.2) — 로봇 대기 도크
        #   lane_z  = XN/A1' 마커 계열(6.875) — carry 구간 1단계(핸드오프→슬롯 x열)에서
        #             쓰는 입차 전용 차로. 슬롯 자체는 반대쪽(z=-6.875)에 있어 carry의
        #             2단계가 그 경계를 가로질러 슬롯까지 마저 들어간다(v4 실측 반영,
        #             2026-07-25). 출차 세트는 대칭값(-6.875 등, 슬롯과 같은 차로)을
        #             launch 파라미터로 넘긴다(parking_robot_system.launch.py 참고) —
        #             입/출차가 물리적으로 다른 차로를 쓰게 돼 있어야 두 로봇쌍이
        #             동시에 움직여도 안 부딪힌다.
        # rear_axle_z/front_axle_z(차체 축 오프셋)는 차량 규격이라 위치와 무관 — 그대로 유지.
        self.node = node
        self.rear_id = rear_id
        self.front_id = front_id
        self.robots = (rear_id, front_id)
        self.handoff_x = handoff_x
        self.handoff_z = handoff_z
        self.gate_x = gate_x
        self.dock_rear = dock_rear
        self.dock_front = dock_front
        self.lane_z = lane_z
        self.cx = handoff_x   # pickup_at_slot 등 "중심선 x" 의미로 재사용(이름 유지)
        self.rear_axle = rear_axle_z
        self.front_axle = front_axle_z
        self.pose = {r: None for r in self.robots}   # rid -> (x, z, yaw), USD
        self.veh_x = self.veh_y = self.veh_z = None
        self.veh_yaw = None
        # 중앙 안전 상태를 받기 전에는 fail-safe 정지.
        self._emergency_stop = True
        self._team_role = robot_team_role(self.rear_id)
        self._obstacle_paused = False
        self._last_safety_state_at = None
        # 비상정지가 한 번이라도 들어온 현재 액션은 운영 복귀 승인 뒤에도
        # 이어서 실행하면 안 된다. 새 액션 콜백이 begin_operation()을 호출할
        # 때만 이 래치를 해제한다.
        self._operation_cancelled = True
        grp = callback_group or ReentrantCallbackGroup()
        for r in self.robots:
            # 토픽 접두사 "/robot_{id}/..."는 isaacpjt/Isaac_envo/parking_v4_runner.py가
            # 실제로 발행하는 이름(odom_pub, attach_camera_graph 참고) — "/{id}/..."가
            # 아니다(2026-07-25 확인·수정: 이전엔 접두사 없이 구독해서 odom이 전혀 안
            # 들어왔다). cmd_vel은 현재 v4 runner 쪽에 구독자가 없어(주행 루프가 아직
            # probe 전용) 실제로는 아무도 안 받지만, 나중에 붙을 때 같은 네임스페이스
            # 관례를 따르도록 미리 맞춰둔다.
            node.create_subscription(
                Odometry, f"/robot_{r}/odom",
                lambda m, rid=r: self._odom(rid, m), 10, callback_group=grp)
        node.create_subscription(
            PoseStamped, "/vehicle/pose", self._veh, 10, callback_group=grp)
        node.create_subscription(
            FormationStop, "/formation_stop", self._on_emergency_stop,
            10, callback_group=grp)
        node.create_subscription(
            ObstacleAlert, "/obstacle_alert", self._on_obstacle_alert,
            10, callback_group=grp)
        safety_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        node.create_subscription(
            SafetyState, "/safety/state", self._on_safety_state,
            safety_qos, callback_group=grp)
        self.cmd = {r: node.create_publisher(Twist, f"/robot_{r}/cmd_vel", 10) for r in self.robots}

    # ---- 구독 콜백 (원본 L86-94 그대로) ----
    def _odom(self, rid, m):
        q = m.pose.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        self.pose[rid] = (m.pose.pose.position.x, m.pose.pose.position.z, yaw)

    def _veh(self, m):
        self.veh_x = m.pose.position.x
        self.veh_y = m.pose.position.y
        self.veh_z = m.pose.position.z
        q = m.pose.orientation
        # 과거 runner는 orientation을 채우지 않아 (0,0,0,0)을 발행했다.
        if q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w > 0.5:
            self.veh_yaw = math.atan2(
                2 * (q.w * q.z + q.x * q.y),
                1 - 2 * (q.y * q.y + q.z * q.z))

    def _on_emergency_stop(self, msg):
        """관제탑의 전역 비상정지는 프로세스 재기동 전까지 래치한다."""
        if msg.stop and msg.source_robot_id == "control_tower":
            first_stop = not self._emergency_stop
            self._emergency_stop = True
            self._operation_cancelled = True
            if first_stop:
                self.node.get_logger().error(
                    f"관제탑 비상정지 수신: {msg.reason or '사유 없음'}"
                )

    def _on_obstacle_alert(self, msg):
        previous = self._obstacle_paused
        self._obstacle_paused = bool(
            msg.obstacle_detected
            and obstacle_affects_team(
                self._team_role,
                msg.description,
                msg.location.y,
            )
        )
        if self._obstacle_paused:
            self._publish_zero_now()
        if previous != self._obstacle_paused:
            log = (
                self.node.get_logger().warn
                if self._obstacle_paused
                else self.node.get_logger().info
            )
            log(
                "지역 장애물 감지 · 편대 일시정지"
                if self._obstacle_paused
                else "지역 장애물 해소 · 편대 모션 재개"
            )

    def _on_safety_state(self, msg):
        """중앙 상태만 정지 래치를 해제할 수 있다.

        READY_FOR_OPERATION도 motion_allowed=False라서 점검 승인만으로
        로봇이 움직이지 않는다.
        """
        was_stopped = self._emergency_stop
        self._last_safety_state_at = time.monotonic()
        self._emergency_stop = not msg.motion_allowed
        if self._emergency_stop:
            self._operation_cancelled = True
        if was_stopped != self._emergency_stop:
            log = (
                self.node.get_logger().warn
                if self._emergency_stop
                else self.node.get_logger().info
            )
            log(
                f"중앙 안전 상태={msg.state}, "
                f"motion_allowed={msg.motion_allowed}"
            )

    def _motion_blocked(self):
        while self._obstacle_paused:
            if self._central_motion_blocked():
                return True
            self._publish_zero_now()
            time.sleep(1.0 / CONTROL_HZ)
        return self._central_motion_blocked()

    def _central_motion_blocked(self):
        heartbeat_stale = (
            self._last_safety_state_at is None
            or time.monotonic() - self._last_safety_state_at > 3.0
        )
        return (
            self._emergency_stop
            or self._operation_cancelled
            or heartbeat_stale
        )

    def begin_operation(self):
        """새 액션 시작 시에만 이전 작업 취소 래치를 해제한다.

        STOPPED_LATCHED/READY_FOR_OPERATION/heartbeat 두절 상태에서는
        새 액션 자체를 시작하지 않는다. 이미 실행 중이던 액션은 비상정지로
        _operation_cancelled=True가 된 뒤 NORMAL이 와도 계속 취소 상태다.
        """
        heartbeat_stale = (
            self._last_safety_state_at is None
            or time.monotonic() - self._last_safety_state_at > 3.0
        )
        if self._emergency_stop or heartbeat_stale or self._obstacle_paused:
            if self._emergency_stop or heartbeat_stale:
                self._operation_cancelled = True
            return False
        self._operation_cancelled = False
        return True

    def carry_heading(self):
        """차량 직접 heading을 우선하고, 없으면 로봇 baseline으로 폴백한다."""
        if self.veh_yaw is not None:
            return self.veh_yaw
        rear, front = self.pose.get(self.rear_id), self.pose.get(self.front_id)
        if rear is None or front is None:
            return None
        return formation_heading(rear, front)

    # ---- 발행/정지 헬퍼 (원본 L96-111 그대로) ----
    def _publish_zero_now(self):
        """장애물 콜백에서도 블로킹 검사 없이 두 로봇에 즉시 0속도를 보낸다."""
        for robot_id in self.robots:
            twist = Twist()
            self.cmd[robot_id].publish(twist)

    def _pub(self, rid, vx, vy=0.0, wz=0.0):
        if self._motion_blocked():
            vx = vy = wz = 0.0
        t = Twist()
        t.linear.x, t.linear.y, t.angular.z = float(vx), float(vy), float(wz)
        self.cmd[rid].publish(t)

    def _stop_all(self):
        for r in self.robots:
            self._pub(r, 0.0)

    def _settle(self, secs=0.5):
        """단계 경계에서 정지 후 잠깐 멈춤 → 관성 흡수(원본 L105-111 그대로)."""
        self._stop_all()
        end = time.time() + secs
        while time.time() < end:
            if self._motion_blocked():
                self._stop_all()
                return False
            self._stop_all()
            time.sleep(1.0 / CONTROL_HZ)
        return True

    def wait_data(self, timeout=15.0):
        """두 로봇 odom + 차량 pose 수신 대기(원본 _wait_data, L116-121 그대로)."""
        end = time.time() + timeout
        while time.time() < end and (any(self.pose[r] is None for r in self.robots)
                                     or self.veh_z is None):
            if self._motion_blocked():
                return False
            time.sleep(0.1)
        return (
            not self._motion_blocked()
            and all(self.pose[r] is not None for r in self.robots)
            and self.veh_z is not None
        )

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
            if self._motion_blocked():
                self._stop_all()
                return False
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
            if self._motion_blocked():
                self._stop_all()
                return False
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
            if self._motion_blocked():
                self._stop_all()
                return False
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
            if self._motion_blocked():
                self._stop_all()
                return False
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

    def rotate_parallel(self, targets, timeout=90.0):
        """빈 로봇 여러 대를 각자의 목표 yaw로 동시에 회전한다.

        targets: ``{rid: target_yaw}``. 각 로봇은 독립 yaw 폐루프를 사용하지만 같은 제어
        tick에서 명령을 발행한다. 인계장 픽업에서는 rear가 +z, front가 -z를 바라보므로
        두 로봇에 서로 반대 방향의 목표 yaw를 줄 수 있어야 한다.
        """
        done = {rid: False for rid in targets}
        end = time.time() + timeout
        while time.time() < end:
            if self._motion_blocked():
                self._stop_all()
                return False
            for rid, target_yaw in targets.items():
                if done[rid]:
                    self._pub(rid, 0.0)
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
        return all(abs(wrap(targets[r] - self.pose[r][2])) < YAW_TOL * 3 for r in targets)

    # ---- 인계장 픽업: 남·북 분리 접근 후 양 로봇 동시 진입 ----
    def pickup_sequence(self):
        """두 로봇이 차량의 앞·뒤에서 각 축으로 동시에 진입한다.

        게이트 통과 후 rear는 차량 남쪽에서 +z 방향으로 뒷축을, front는 차량 북쪽에서
        -z 방향으로 앞축을 향한다. 서로 반대쪽에서 접근하므로 진입 경로가 겹치지 않으며,
        회전과 축 진입을 모두 병렬로 수행한다.
        """
        if not self.wait_data():
            return False, "데이터 미수신"
        # 게이트 통과는 병렬(rear 남쪽 차로, front 북쪽 차로로 분리) — 각자 도크에서
        # 인계지점 앞 게이트(gate_x)까지, 인계지점 중심 기준 상대 차로(LANE_Z_REAR/FRONT)로.
        gate = {
            self.rear_id:  [(self.gate_x, self.handoff_z + LANE_Z_REAR)],
            self.front_id: [(self.gate_x, self.handoff_z + LANE_Z_FRONT)],
        }
        self.node.get_logger().info("접근: 게이트 통과(병렬)")
        if not self.approach_parallel(gate):
            self._stop_all()
            return False, "게이트 통과 타임아웃"
        self._settle()

        # 차량 양끝 바깥으로 동시에 이동. rear와 front의 진입 경로를 물리적으로 분리한다.
        staging = {
            self.rear_id: [(self.handoff_x, self.handoff_z + SOUTH_STAGE_OFFSET)],
            self.front_id: [(self.handoff_x, self.handoff_z + NORTH_STAGE_OFFSET)],
        }
        self.node.get_logger().info("인계장 접근: rear 남쪽·front 북쪽 동시 정렬")
        if not self.approach_parallel(staging):
            self._stop_all()
            return False, "남·북 스테이징 접근 타임아웃"
        self._settle()

        self.node.get_logger().info("인계장 회전: rear +z·front -z 동시 정렬")
        if not self.rotate_parallel({
                self.rear_id: FACE_PZ,
                self.front_id: FACE_MZ,
        }):
            self._stop_all()
            return False, "동시 회전 실패"
        self._settle()

        self.node.get_logger().info("인계장 픽업: rear 뒷축·front 앞축 동시 진입")
        if not self.ingress_parallel({
                self.rear_id: (self.handoff_x, self.handoff_z + self.rear_axle, FACE_PZ),
                self.front_id: (self.handoff_x, self.handoff_z + self.front_axle, FACE_MZ),
        }, timeout=140.0, tol=INGRESS_TOL):
            self._stop_all()
            return False, "동시 축 진입 실패"
        self._settle(INGRESS_SETTLE)
        return True, "인계장 앞·뒤축 동시 픽업 완료"

    # ---- 신규(원본에 없음, best-effort) ----
    def carry_to(self, tx_usd, tz_usd, timeout=CARRY_TO_TIMEOUT, tol=POS_TOL,
                 heading_ref=None):
        """파지 후 가상강체를 목표 위치로 heading을 유지하며 이동.

        위치는 기존 /vehicle/pose P제어를 유지한다. 방향은 차량 quaternion(없으면 두 로봇
        baseline)을 피드백으로 삼아 P제어한 omega를 구하고, 각 로봇에
        ``V_center + omega x r_i`` 접선속도와 같은 omega를 동시에 분배한다. 따라서 각 로봇이
        자기 중심으로만 돌아 그립을 비트는 대신 두 로봇+차량 전체가 하나의 강체처럼 움직인다.

        호출부는 인계베이→통로→슬롯 L자 경로의 모든 leg에 같은 heading_ref를 전달한다.
        장애물 회피는 여전히 별도 경로계획 과제지만, 240Hz A2 E2E에서 두 번 연속 차량 최대
        heading 오차 0.61deg 이하, 최종 오차 0.13deg 이하로 검증했다(2026-07-23).
        """
        if self.veh_x is None or self.veh_z is None:
            return False
        if heading_ref is None:
            heading_ref = self.carry_heading()
        if heading_ref is None:
            return False

        max_heading_error = 0.0
        end = time.time() + timeout
        while time.time() < end:
            if self._motion_blocked():
                self._stop_all()
                return False
            ex, ez = tx_usd - self.veh_x, tz_usd - self.veh_z
            heading = self.carry_heading()
            if heading is None:
                self._stop_all()
                return False
            heading_error = wrap(heading_ref - heading)
            max_heading_error = max(max_heading_error, abs(heading_error))
            if math.hypot(ex, ez) < tol and abs(heading_error) < CARRY_HEADING_TOL:
                break

            omega = heading_hold_omega(
                heading_ref, heading, CARRY_HEADING_KP,
                CARRY_HEADING_MAX_OMEGA, CARRY_HEADING_DEADBAND)
            rear, front = self.pose.get(self.rear_id), self.pose.get(self.front_id)
            if rear is None or front is None:
                self._stop_all()
                return False
            center_x = 0.5 * (rear[0] + front[0])
            center_z = 0.5 * (rear[1] + front[1])

            for r in self.robots:
                rp = self.pose.get(r)
                if rp is None:
                    self._stop_all()
                    return False
                rx, rz, ryaw = rp[0] - center_x, rp[1] - center_z, rp[2]

                # 기존 위치 P제어에 가상중심 회전의 접선속도를 합성한다.
                base_fwd, base_left = body_twist_from_world_error(ex, ez, ryaw)
                base_fwd = clamp(K_LIN * base_fwd, CARRY_SPEED_FAST)
                base_left = clamp(K_STRAFE * base_left, CARRY_SPEED_FAST)
                tangent_x, tangent_z = rigid_body_world_velocity(
                    0.0, 0.0, omega, rx, rz)
                turn_fwd, turn_left = body_twist_from_world_error(
                    tangent_x, tangent_z, ryaw)
                self._pub(r, clamp(base_fwd + turn_fwd, MAX_LIN),
                          clamp(base_left + turn_left, MAX_LIN), omega)
            time.sleep(1.0 / CONTROL_HZ)
        self._stop_all()
        final_heading_error = wrap(heading_ref - self.carry_heading())
        self.node.get_logger().info(
            f"운반 heading: max={math.degrees(max_heading_error):.3f}deg "
            f"final={math.degrees(final_heading_error):+.3f}deg")
        return (math.hypot(tx_usd - self.veh_x, tz_usd - self.veh_z) < tol * 3
                and abs(final_heading_error) < math.radians(1.0))

    def return_both_to_docks(self):
        """주차·하차 후 두 로봇을 동시에 초기 대기 도크로 복귀(사용자 요구: 동시 + 초기위치).

        2026-07-24 v3 레이아웃: 각 로봇 웨이포인트 체인 ── ① 현재 x 유지한 채 자기
        차로(lane_z + LANE_Z_REAR/FRONT)로 통로 쪽으로 빠져나옴 → ② 자기 도크 x로 이동
        → ③ 자기 도크 z로. approach_parallel로 두 로봇을 '동시에' 이동시킨다. rear/front가
        서로 다른 차로로 이동해 통로에서 겹치지 않는다.
        """
        routes = {}
        for rid, offset, dock in ((self.rear_id, LANE_Z_REAR, self.dock_rear),
                                  (self.front_id, LANE_Z_FRONT, self.dock_front)):
            cur = self.pose.get(rid)
            if cur is None:
                return False
            cur_x = cur[0]
            dock_x, dock_z = dock
            lane_z = self.lane_z + offset
            routes[rid] = [(cur_x, lane_z), (dock_x, lane_z), (dock_x, dock_z)]
        self.node.get_logger().info("복귀(동시): 두 로봇 앞으로→통로→초기 도크")
        ok = self.approach_parallel(routes)
        self._stop_all()
        return ok

    def return_from_bay(self):
        """출차 하차(인계지점) 후 도크 복귀 — 두 단계(사용자 실측 반영):

        ① 백아웃(정밀): 차 밑에서 차 길이축(z)으로 '완전히' 빠져나온다. rear 남(-z)/front 북(+z)
           으로 인계지점 중심에서 BAY_CLEAR_Z만큼(단일 웨이포인트 → CORNER_TOL로 안 자르고
           정밀 정지). 이렇게 확실히 나온 뒤에 이동해야 좌우 바퀴를 안 스친다(진입 역방향).
        ② 통로 차로로 복귀: rear/front가 서로 다른 차로(lane_z + LANE_Z_REAR/FRONT)로 각자
           도크까지 이동한다(return_both_to_docks와 동일 패턴).
        """
        # ① 정밀 백아웃 — 차 밖으로 완전히
        backout = {}
        for rid, offset in ((self.rear_id, -BAY_CLEAR_Z), (self.front_id, BAY_CLEAR_Z)):
            cur = self.pose.get(rid)
            if cur is None:
                return False
            backout[rid] = [(cur[0], self.handoff_z + offset)]   # 단일 웨이포인트 → 정밀 정지
        self.node.get_logger().info("출차 복귀①: 차 길이축으로 완전히 빠져나옴")
        if not self.approach_parallel(backout):
            self._stop_all()
            return False
        self._settle()
        # ② 통로 차로 정렬 → 각자 도크로
        transit = {}
        for rid, offset, dock in ((self.rear_id, LANE_Z_REAR, self.dock_rear),
                                  (self.front_id, LANE_Z_FRONT, self.dock_front)):
            cur = self.pose.get(rid)
            if cur is None:
                return False
            dock_x, dock_z = dock
            lane_z = self.lane_z + offset
            transit[rid] = [(cur[0], lane_z), (dock_x, lane_z), (dock_x, dock_z)]
        self.node.get_logger().info("출차 복귀②: 통로 차로 지나 초기 도크로")
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
            if self._motion_blocked():
                self._stop_all()
                return False
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
        for rid, lane in ((self.rear_id, LANE_Z_REAR), (self.front_id, LANE_Z_FRONT)):
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
            self.front_id: (slot_x, front_t, FACE_MZ),
            self.rear_id: (slot_x, rear_t, FACE_MZ),
        })
        self._settle(INGRESS_SETTLE)
        return True, "슬롯 픽업 완료(동시)"

    def carry_to_bay(self, bay_x, bay_z):
        """출차 운반: 슬롯에서 통로로 나와 통로 따라 인계지점으로(입차 carry의 역방향, L자).
          ① carry_to(현재 veh_x, 이 세트 전용 차로) → 슬롯 밖 통로로,
          ② carry_to(bay_x, bay_z) → 통로 따라 인계지점으로.
        """
        if self.veh_x is None or self.veh_z is None:
            return False
        heading_ref = self.carry_heading()
        self.node.get_logger().info("출차 운반: 슬롯→통로")
        ok = self.carry_to(self.veh_x, self.lane_z, heading_ref=heading_ref)
        if ok:
            self.node.get_logger().info(f"출차 운반: 통로→인계지점({bay_x:.1f},{bay_z:.1f})")
            ok = self.carry_to(bay_x, bay_z, heading_ref=heading_ref)
        return ok

    def carry_rotate_to(self, target_yaw, timeout=90.0, tol_rad=None):
        """파지 후 두 로봇을 target_yaw로 회전(강체로 잡은 차량이 함께 회전).

        2026-07-24: PivotRotateController(core/pivot_rotate_controller.py)로 배선했다.
        예전 방식(rotate_parallel — 두 로봇이 각자 독립적으로 같은 목표각까지 도는 것)은
        "차량 중심"이 아니라 "각자 자기 자신"을 축으로 도는 셈이라, 강체로 잡은 차량이
        있으면 실제로는 원을 그리며 서로 밀고 당겨야 하는데 그 계산이 없었다(테스트만
        통과하고 실사용된 적 없는 구현이었음 — 이번에 실제로 연결).

        PivotRotateController는 "두 로봇 위치의 중점"(≈ 차량 중심)을 회전축으로 삼아
        접선속도(ω×r)를 계산한다 — 중심에서 반대편에 있는 두 로봇은 이 식 하나로 자동으로
        반대 방향 속도가 나오고, 각속도는 항상 동일하다(회전목마 원리). 롤러 미끄러짐을
        감안해 매 tick 실측 yaw로 "지금까지 실제로 돈 각도"를 피드백한다.
        """
        rear_p, front_p = self.pose.get(self.rear_id), self.pose.get(self.front_id)
        if rear_p is None or front_p is None:
            return False
        start_rear = Pose2D(x=rear_p[0], y=rear_p[1], yaw=rear_p[2])
        start_front = Pose2D(x=front_p[0], y=front_p[1], yaw=front_p[2])
        current_heading = self.carry_heading()
        if current_heading is None:
            return False
        # PivotRotateController.target_angle_rad는 "지금부터 얼마나 돌아야 하는가"
        # (상대량)다 — carry_rotate_to의 target_yaw(절대각)를 여기서 변환한다.
        relative_angle = wrap(target_yaw - current_heading)
        tol = YAW_TOL if tol_rad is None else tol_rad
        controller = PivotRotateController(
            target_angle_rad=relative_angle, k_omega=K_YAW,
            max_omega=MAX_YAW, max_linear=MAX_LIN)

        end = time.time() + timeout
        while time.time() < end:
            if self._motion_blocked():
                self._stop_all()
                return False
            rear_p, front_p = self.pose.get(self.rear_id), self.pose.get(self.front_id)
            if rear_p is None or front_p is None:
                self._stop_all()
                return False
            rear_now = Pose2D(x=rear_p[0], y=rear_p[1], yaw=rear_p[2])
            front_now = Pose2D(x=front_p[0], y=front_p[1], yaw=front_p[2])
            if controller.is_settled(rear_now, front_now, start_rear, start_front, tol):
                break
            rear_cmd = controller.compute(rear_now, front_now, start_rear, start_front)
            front_cmd = controller.compute(front_now, rear_now, start_front, start_rear)
            self._pub(self.rear_id, rear_cmd.linear_x, rear_cmd.linear_y, rear_cmd.angular_z)
            self._pub(self.front_id, front_cmd.linear_x, front_cmd.linear_y, front_cmd.angular_z)
            time.sleep(1.0 / CONTROL_HZ)
        self._stop_all()

        rear_p, front_p = self.pose.get(self.rear_id), self.pose.get(self.front_id)
        if rear_p is None or front_p is None:
            return False
        rear_now = Pose2D(x=rear_p[0], y=rear_p[1], yaw=rear_p[2])
        front_now = Pose2D(x=front_p[0], y=front_p[1], yaw=front_p[2])
        return controller.is_settled(rear_now, front_now, start_rear, start_front, tol * 3)
