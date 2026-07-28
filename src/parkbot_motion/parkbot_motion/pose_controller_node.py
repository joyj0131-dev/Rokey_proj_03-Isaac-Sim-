#!/usr/bin/env python3
"""pose_controller_node — R3b: PoseController 를 NavigateToPose 액션 서버로.

설계서 docs/superpowers/specs/2026-07-25-ros2-node-refactor-design.md §3(R3):
시뮬레이터(Isaac)가 아니라 이 ROS2 노드가 "무엇을 명령할지" 결정한다. R2 브리지
(``parking_v4_runner.sh --bridge``, taskR2-report.md)가 발행하는
``/robot_<id>/odom`` 을 구독해 R3a 의 순수 정책 클래스
(``parkbot_motion.pose_controller.PoseController``) 한 스텝을 돌리고
``/robot_<id>/cmd_vel`` 로 명령한다.

범위(정직하게, R3b 지시사항 그대로):
- R3b 원안: 이 노드는 ``filt``(PoseFilter, 마커 융합)를 모른다 — ``/odom`` 이
  이미 실어온 자세(기본은 R2 브리지의 휠 오도, ``--odom=gt`` 로 바꾸지 않는 한)를
  그대로 ``PoseController`` 에 먹인다.
- **R3c 갱신**: 그 한계를 닫았다. ``pose_topic``/``pose_msg_type`` 파라미터로
  자세 소스를 ``nav_msgs/Odometry``(R2 브리지의 ``/robot_<id>/odom``, 기본값 —
  하위호환) 또는 ``geometry_msgs/PoseStamped``(``marker_localizer_node`` 가
  ``fuse:=true`` 로 낸 마커+오도 융합 자세, 보통 ``/robot_<id>/pose``) 중
  아무거나 구독할 수 있다. 이 노드 자신은 여전히 ``PoseFilter`` 를 모른다 —
  그저 두 메시지 타입 중 하나에서 ``(x, z, yaw_deg)`` 를 뽑아낼 뿐이고, 마커
  융합 자체는 ``marker_localizer_node`` 가 이미 끝내놓은 결과를 받는다(§ 아래
  ``odom_quat_to_yaw_deg``/``_on_pose_stamped`` 근처 상세).
- ``NavigateToPose`` 목표의 ``PoseStamped`` 는 ``map`` 이 아니라 **USD 월드
  프레임 그대로** 해석한다(``position.x/z`` 직접 사용, yaw 는 월드 Y축 회전으로
  해석). ``parking_robot_system.frame_transform`` 의 map<->USD 변환은 존재하지만
  아직 배선하지 않았다 — 배선은 후속 스테이지(보고서에 기록).
- ``src/parking_robot_system/parking_robot_system/navigate_action_server.py`` 는
  이 스테이지에서 건드리지 않는다(레거시 ``FormationMotion`` 백엔드, 별개 액션
  이름 ``navigate_to_pose`` 미절대경로). 두 서버를 어떻게 합칠지는 후속 단계
  판단.
"""
import math
import threading
import time

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from nav2_msgs.action import NavigateToPose

from parkbot_motion.pose_controller import PoseController


def odom_quat_to_yaw_deg(x, y, z, w):
    """``/robot_<id>/odom`` 의 ``orientation`` -> yaw_deg(이 프로젝트 규약).

    R2 브리지 계약(taskR2-report.md §2.2, ``parking_v4_runner.py`` 의
    ``publish_odom()``): ``orientation`` 은 표준 ROS ``map``(z-up) 쿼터니언이
    아니라, 이 프로젝트의 yaw(``wheel_odometry``/``gt_pose_xz_yaw`` 규약:
    yaw=0 -> 전방=+Z, 전방=(sinψ, cosψ))를 메시지의 z축 회전 슬롯에 그대로
    인코딩한 것이다: ``orientation.x=orientation.y=0``,
    ``orientation.z=sin(yaw/2)``, ``orientation.w=cos(yaw/2)``.

    표준 "z축 요(yaw)" 공식 ``atan2(2(wz+xy), 1-2(y²+z²))`` 을 그대로 적용하면
    (x=y=0 이므로) ``atan2(2wz, 1-2z²) = atan2(sin yaw, cos yaw) = yaw`` 로
    정확히 역산된다 — 인코딩 공식의 항등식 역변환이다(마커/카메라 쿼터니언이
    아니라 순수하게 이 프로젝트가 만든 슬롯이므로 x,y 항이 실제로 0 이라는
    전제가 성립한다).

    R3c: ``marker_localizer_node`` 가 ``frame="usd"``(기본값)로 내는
    ``geometry_msgs/PoseStamped`` 도 **같은 인코딩**을 쓴다(taskR3c 수정 —
    이전에는 이 분기가 orientation 을 아예 채우지 않는 버그가 있었다). 그래서
    이 함수 하나를 ``Odometry``/``PoseStamped`` 양쪽에 그대로 재사용한다 —
    아래 ``_on_odom``/``_on_pose_stamped`` 참고.
    """
    return math.degrees(math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z)))


def resolve_pose_msg_type(declared, topic, topic_names_and_types):
    """``pose_msg_type`` 파라미터 -> 실제 구독할 메시지 타입('odometry'|
    'posestamped') + 판정 근거 문자열. 순수 함수(rclpy 노드 불필요) — 호출부가
    ``Node.get_topic_names_and_types()`` 결과를 그대로 넘겨준다.

    - 명시값('odometry'/'posestamped')이면 그대로 쓴다.
    - 'auto' 면: (1) ``topic`` 을 이미 발행 중인 퍼블리셔가 있으면
      ``topic_names_and_types``(``[(name, [type_str, ...]), ...]``)에서 실제
      타입을 판정한다(마커 로컬라이저/브리지를 먼저 띄우는 이 스테이지의 통상
      실행 순서에서 유효). (2) 아무도 아직 안 냈으면(예: 이 노드를 먼저 띄운
      경우, 또는 DDS 디스커버리가 아직 안 끝난 경우) 토픽명 휴리스틱으로
      추정한다: 이름에 'odom' 이 들어 있으면 Odometry, 그 외(예: 마커
      로컬라이저의 기본 '/robot_pose' 류)는 PoseStamped 로 본다.
    """
    if declared in ('odometry', 'posestamped'):
        return declared, 'explicit'
    if declared != 'auto':
        raise ValueError(
            f"pose_msg_type 은 'odometry'|'posestamped'|'auto' 여야 합니다: {declared!r}")
    for name, types in topic_names_and_types:
        if name == topic:
            if 'nav_msgs/msg/Odometry' in types:
                return 'odometry', 'auto-detected(발행중)'
            if 'geometry_msgs/msg/PoseStamped' in types:
                return 'posestamped', 'auto-detected(발행중)'
    guess = 'odometry' if 'odom' in topic.lower() else 'posestamped'
    return guess, 'auto-heuristic(토픽 미발행, 이름으로 추정)'


def travel_axis_value(x, z, axis):
    """자세의 (x, z) -> 축감지/진입(``axle_detector_node``/``ingress_node``)이
    쓰는 1차원 "주행좌표". 두 노드 모두 원래 world **x** 만 지원했다(entry_lead/
    entry_follow가 world -x 로 진입하는 배치 하나만 있었으므로, 각 노드 docstring
    "주행좌표" 절 참고). 2026-07-27 Phase X(출차, exit_lead/exit_follow가 북향
    +z 로 A3 트럭 밑에 진입)에서 처음으로 z 축 진입이 생겨 일반화한다.

    ``axis``: 'x'|'-x'|'z'|'-z'. 부호는 "SEEK 진행 방향으로 이 좌표가 감소해야
    한다"는 ``ingress_control``의 내장 가정(RETURN 보정식이 그렇게 유도돼 있다,
    ``ingress_control.return_phase_vx`` docstring 참고)을 axis 별로 맞추기 위함이다
    — Phase X는 북향으로 전진할수록 world z가 **증가**하므로 '-z'(=-z가 감소)를
    쓴다. 기존 x 배치는 부호 반전 없이 'x' 그대로(회귀 없음).
    """
    if axis == 'x':
        return float(x)
    if axis == '-x':
        return -float(x)
    if axis == 'z':
        return float(z)
    if axis == '-z':
        return -float(z)
    raise ValueError(f"travel_axis 는 'x'|'-x'|'z'|'-z' 여야 합니다: {axis!r}")


def goal_quat_to_yaw_deg(x, y, z, w):
    """``NavigateToPose`` 목표 ``PoseStamped`` 의 쿼터니언 -> yaw_deg(월드 Y축 기준).

    이 스테이지는 ``goal.pose`` 를 ``map`` 이 아니라 USD 월드 프레임(Y-up)
    그대로 해석한다(R3b 지시사항). USD 월드에서 이 프로젝트의 yaw 는 "로컬
    +Z 축을 월드 Y축 둘레로 회전"한 각이다(``wheel_odometry.py`` 규약: yaw=0
    -> 전방=+Z, 전방=(sinψ,cosψ)). 로컬 +Z=(0,0,1) 을 쿼터니언(x,y,z,w)이
    나타내는 회전행렬의 3번째 열(=로컬 z축의 월드 상 방향)로 보내면::

        world_fwd = (2(xz+wy), 2(yz-wx), 1-2(x²+y²))

    이고, xz-평면에 투영한 yaw 는::

        yaw = atan2(world_fwd_x, world_fwd_z) = atan2(2(xz+wy), 1-2(x²+y²))

    순수 Y축 회전 쿼터니언(x=0, z=0, y=sin(θ/2), w=cos(θ/2))을 넣으면 정확히
    θ 가 나온다(단위테스트 ``test_pose_quat_conversion.py`` 로 확인). 일반적인
    (비-순수-Y) 쿼터니언에 대해서는 로컬 +Z 축의 xz-평면 투영 방향일 뿐이라
    피치/롤 성분은 버려진다 — 이 스테이지에서 클라이언트는 순수 Y회전만
    보낸다고 가정한다.
    """
    return math.degrees(math.atan2(2.0 * (x * z + w * y), 1.0 - 2.0 * (x * x + y * y)))


class PoseControllerNode(Node):
    """``/robot_<id>/odom`` 구독 + ``PoseController`` + ``/robot_<id>/cmd_vel``
    발행을 ``NavigateToPose`` 액션 뒤에 감싼 노드.

    설계(콜백 vs 타이머): **포즈 콜백 구동**을 선택했다. 이유:
      1. R3a 원본(``drive_to_pose``)은 "매 관측(app.update 스텝)마다 정확히
         한 번" 제어를 결정했다 — 포즈 콜백 구동이 그 구조를 가장 가깝게
         보존한다(별도 타이머가 새 관측 없이 재구동하거나, 반대로 두 개의
         관측 사이에 아무 것도 안 하는 낭비/지연을 만들지 않는다).
      2. dt 를 "가장 최근 관측 간격"으로 자연스럽게 정의할 수 있다(아래 dt
         절 참고) — 타이머 구동이면 dt(제어 주기)와 관측 최신성이 분리되어
         버린다(오래된 포즈로 여러 번 스텝하거나, 새 포즈가 여러 번 와도
         한 번만 반영하는 등 혼동의 소지).
      3. 단점(정직하게): 포즈 발행 주기가 요동치면(DDS 지터, 시뮬 RTF<1 등)
         제어 주기도 같이 요동친다 — 원본이 튜닝된 "동기 60Hz" 가정이 이미
         깨졌으므로(설계서 5절), 타이머로 바꿔도 이 위험 자체는 없어지지
         않는다. 아래 ``max_dt`` 클램프로 최악의 경우만 막는다.

    dt: ``/odom`` 메시지의 ``header.stamp``(R2 브리지가
    ``ros_node.get_clock().now()`` 로 채움, wall/system 시계) 간격을 쓴다.
    첫 메시지는 이전 표본이 없어 스킵(스텝하지 않음), ``dt<=0``(시계 역행·
    중복 스탬프)도 스킵, 그 외에는 ``max_dt`` 파라미터로 상한 클램프한다.
    **재튜닝 위험(R3a 보고서 #2 그대로 적용)**: 원본은 시뮬 시간 60Hz(dt≈
    0.0167s, 거의 고정)로 튜닝됐다. 여기서는 DDS/시뮬 RTF 에 따라 dt 가 훨씬
    크고 들쭉날쭉할 수 있어(R2 실측: 헤드리스 4로봇 RTF≈0.5) ``pos_gain``/
    ``linear_accel`` 등의 체감 응답이 달라질 수 있다.
    """

    def __init__(self):
        super().__init__('pose_controller_node')

        self.declare_parameter('robot_id', 'entry_lead')
        robot_id = self.get_parameter('robot_id').value

        self.declare_parameter('pose_topic', f'/robot_{robot_id}/odom')
        self.declare_parameter('cmd_vel_topic', f'/robot_{robot_id}/cmd_vel')
        self.declare_parameter('action_name', f'/robot_{robot_id}/navigate_to_pose')
        # R3c: 자세 소스 메시지 타입. 'odometry'|'posestamped'|'auto'(기본).
        # 'auto' 는 (1) pose_topic 이 이미 발행 중이면 실제 타입으로 판정하고,
        # (2) 아직 아무도 안 냈으면 토픽명 휴리스틱('odom' 포함 여부)으로
        # 추정한다. 기본 pose_topic(=/robot_<id>/odom, 위)과 'auto' 조합은
        # 항상 Odometry 로 판정돼 R3b 와 동일하게 동작한다(하위호환) —
        # marker_localizer_node 의 융합 PoseStamped 를 쓰려면 pose_topic 을
        # 그 토픽(예: /robot_<id>/pose)으로 바꾸면 된다(§ 클래스 docstring).
        self.declare_parameter('pose_msg_type', 'auto')

        # PoseController 게인/허용오차 — "미션이 오늘 쓰는 기본값"과 동일
        # (isaacpjt/Isaac_envo/parking_v4_runner.py: drive_to_pose 시그니처
        # 1166-1168행 = pos_gain/yaw_gain/max_lin/max_ang/pos_tol/yaw_tol,
        # LINEAR_ACCEL/LINEAR_DECEL/ANGULAR_ACCEL 88-90행, settle_frames=30
        # 은 PoseController/pose_controller.py 자체의 기본값과 동일).
        self.declare_parameter('pos_gain', 0.8)
        self.declare_parameter('yaw_gain', 1.2)
        self.declare_parameter('max_lin', 0.25)
        self.declare_parameter('max_ang', 0.6)
        self.declare_parameter('pos_tol', 0.03)
        self.declare_parameter('yaw_tol', 0.5)
        # 메카넘 회전 데드밴드 보정[rad/s]. 요 오차가 tol 밖인데 비례 wz 가 이 값보다
        # 작으면(데드밴드~0.012 아래) 이 최소 속도로 깔아 마지막 <1° 를 인칭한다.
        # 0=끔(하위호환). 진입 정렬처럼 tight yaw_tol 을 실제로 달성해야 할 때 켠다.
        self.declare_parameter('yaw_min_cmd', 0.0)
        self.declare_parameter('linear_accel', 0.5)
        self.declare_parameter('linear_decel', 0.8)
        self.declare_parameter('angular_accel', 0.8)
        self.declare_parameter('settle_frames', 30)
        # settle 결과가 허용오차 밖이면 재제어(도달래치 풀고 다시 DRIVING)하는 최대 횟수.
        # 회전 중 노이즈로 조기 래치→오차밖 정지 시, 포기(abort) 대신 목표로 다시 몬다.
        self.declare_parameter('settle_retries', 6)

        # 이 ROS2 노드에서만 필요한 파라미터(러너에는 대응 없음) — dt 가드 +
        # 액션 타임아웃/워치독. 러너의 max_steps=2000 은 안전상한이었다;
        # 여기서는 wall-clock 타임아웃과 "포즈가 끊겼다" 워치독으로 대체한다.
        self.declare_parameter('max_dt', 0.5)
        self.declare_parameter('goal_timeout_sec', 90.0)
        self.declare_parameter('pose_stale_timeout_sec', 5.0)
        # R3c: 코스팅 방지 워치독(실측으로 발견 — taskR3c-report.md). cmd_vel 은
        # 포즈 콜백(_handle_pose)에서만 발행되므로, 포즈가 끊기면(예: 단일
        # 전방캠 융합에서 회전 중 마커가 FOV 밖으로 나가는 경우) 마지막
        # 발행값이 그대로 유지된 채 로봇이 "관성 주행"한다 — pose_stale_timeout_sec
        # (기본 5.0s) 이 끝나야 비로소 0 이 발행된다. 실측: 회전 중 마커를
        # 놓친 뒤 5초간 미보정 회전이 이어져 GT 가 목표(90°)를 훨씬 넘어
        # ~85°(우연히 근사값)까지 무통제로 쓸었다. cmd_watchdog_sec(기본 0.5s,
        # pose_stale_timeout_sec 보다 훨씬 짧게)를 넘겨 포즈가 안 오면 즉시
        # (0,0,0) 을 발행해 코스팅을 최소화한다 — 목표 자체는 여전히
        # pose_stale_timeout_sec 까지 기다렸다가(포즈가 복구되면 계속 진행)
        # 그래도 안 오면 abort 한다(기존 동작 유지, 이 워치독은 그 사이의
        # "무엇을 하고 있나"만 안전하게 바꾼다).
        self.declare_parameter('cmd_watchdog_sec', 0.5)

        self.robot_id = robot_id
        gp = self.get_parameter
        self.pose_topic = gp('pose_topic').value
        pose_msg_type_param = str(gp('pose_msg_type').value)
        self.pose_msg_type, pose_msg_type_how = resolve_pose_msg_type(
            pose_msg_type_param, self.pose_topic, self.get_topic_names_and_types())
        self.cmd_vel_topic = gp('cmd_vel_topic').value
        self.action_name = gp('action_name').value
        self.pos_gain = float(gp('pos_gain').value)
        self.yaw_gain = float(gp('yaw_gain').value)
        self.max_lin = float(gp('max_lin').value)
        self.max_ang = float(gp('max_ang').value)
        self.pos_tol = float(gp('pos_tol').value)
        self.yaw_tol = float(gp('yaw_tol').value)
        self.yaw_min_cmd = float(gp('yaw_min_cmd').value)
        self.linear_accel = float(gp('linear_accel').value)
        self.linear_decel = float(gp('linear_decel').value)
        self.angular_accel = float(gp('angular_accel').value)
        self.settle_frames = int(gp('settle_frames').value)
        self.settle_retries = int(gp('settle_retries').value)
        self.max_dt = float(gp('max_dt').value)
        self.goal_timeout_sec = float(gp('goal_timeout_sec').value)
        self.pose_stale_timeout_sec = float(gp('pose_stale_timeout_sec').value)
        self.cmd_watchdog_sec = float(gp('cmd_watchdog_sec').value)

        self._last_stamp_sec = None
        self._last_pose_wall_time = None
        # self._active 는 활성 목표 하나(dict) 또는 None. _on_pose(구독 콜백
        # 스레드)와 _execute(액션 실행 콜백 스레드)가 함께 건드리므로 잠근다.
        self._active = None
        self._active_lock = threading.Lock()

        cbg = ReentrantCallbackGroup()
        self._cmd_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        if self.pose_msg_type == 'odometry':
            self.create_subscription(Odometry, self.pose_topic, self._on_odom, 10,
                                      callback_group=cbg)
        else:
            self.create_subscription(PoseStamped, self.pose_topic, self._on_pose_stamped, 10,
                                      callback_group=cbg)
        # ★동시성 패턴(navigate_action_server.py 와 동일 교정 패턴): _execute
        # 콜백은 목표 완료까지 폴링 대기하며 블로킹된다(수십 초 단위) — 구독
        # 콜백(_on_odom/_on_pose_stamped)이 같은 그룹에서 계속 돌아야 그 사이
        # self._active 를 갱신할 수 있다. MultiThreadedExecutor 로 스핀해야
        # 한다(main() 참고).
        self._action_server = ActionServer(
            self, NavigateToPose, self.action_name, self._execute,
            goal_callback=self._on_goal, cancel_callback=self._on_cancel,
            callback_group=cbg)

        self.get_logger().info(
            f'pose_controller_node 시작: robot_id={robot_id} pose_topic={self.pose_topic} '
            f'pose_msg_type={self.pose_msg_type}({pose_msg_type_how}) '
            f'cmd_vel_topic={self.cmd_vel_topic} action={self.action_name} '
            f'gains=(pos={self.pos_gain},yaw={self.yaw_gain}) '
            f'tol=(pos={self.pos_tol},yaw={self.yaw_tol})')

    # ---- 구독 콜백: 매 포즈 관측마다 정확히 한 번 제어 스텝 ----
    #
    # R3c: 자세 소스 두 종류를 지원한다. 메시지 레이아웃 차이(핵심 차이 하나뿐):
    #   - ``nav_msgs/Odometry``: position/orientation 이 ``msg.pose.pose`` 아래
    #     (즉 pose 필드가 한 번 더 감싸여 있다 — nav_msgs 의 ``PoseWithCovariance``
    #     때문).
    #   - ``geometry_msgs/PoseStamped``: position/orientation 이 바로 ``msg.pose``
    #     아래(한 겹만).
    #   둘 다 ``header.stamp`` 는 최상위에 있고, orientation 인코딩은 R3c 에서
    #   맞춰 놓은 덕에 동일(§ ``odom_quat_to_yaw_deg`` docstring) — 그래서 얇은
    #   두 콜백이 필드 경로만 다르게 뽑아 공용 ``_handle_pose`` 로 넘긴다.

    def _on_odom(self, msg):
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        self._handle_pose(p.x, p.z, q.x, q.y, q.z, q.w, msg.header.stamp)

    def _on_pose_stamped(self, msg):
        p, q = msg.pose.position, msg.pose.orientation
        self._handle_pose(p.x, p.z, q.x, q.y, q.z, q.w, msg.header.stamp)

    def _handle_pose(self, x, z, qx, qy, qz, qw, stamp):
        x = float(x)
        z = float(z)
        yaw_deg = odom_quat_to_yaw_deg(float(qx), float(qy), float(qz), float(qw))
        pose = (x, z, yaw_deg)

        now_wall = time.monotonic()
        self._last_pose_wall_time = now_wall

        stamp_sec = stamp.sec + stamp.nanosec * 1e-9
        if self._last_stamp_sec is None:
            # 첫 메시지: 이전 표본이 없어 dt 를 낼 수 없다 — 이번 콜백은
            # 스텝하지 않고 다음 콜백부터 dt 를 갖는다(원본 drive_to_pose 의
            # `prev = timeline.get_current_time()` 초기화와 동일 취지).
            self._last_stamp_sec = stamp_sec
            return
        dt = stamp_sec - self._last_stamp_sec
        self._last_stamp_sec = stamp_sec
        if dt <= 0.0:
            # 시계 역행 또는 중복 스탬프 — 방어적으로 이번 콜백은 건너뛴다.
            return
        dt = min(dt, self.max_dt)

        with self._active_lock:
            active = self._active
            if active is None:
                return
            ctrl = active['ctrl']
            phase = active['phase']
            if phase == 'DRIVING':
                vx, vy, wz = ctrl.step(pose, dt)
                self._publish_twist(vx, vy, wz)
                self._publish_feedback(active, pose)
                if ctrl.done:
                    active['phase'] = 'SETTLING'
                    active['settle_count'] = 0
            elif phase == 'SETTLING':
                ctrl.settle_sample(pose)
                active['settle_count'] += 1
                self._publish_twist(0.0, 0.0, 0.0)
                if active['settle_count'] >= ctrl.settle_frames:
                    final_pose, reached = ctrl.finish(pose)
                    if not reached and active['settle_retries'] < self.settle_retries:
                        # 오차 밖 → 포기 말고 재제어. 도달래치 풀고 다시 DRIVING 으로.
                        active['settle_retries'] += 1
                        ctrl.resume()
                        active['phase'] = 'DRIVING'
                        active['settle_count'] = 0
                        self.get_logger().info(
                            f"navigate_to_pose: settle 오차밖 → 재제어 "
                            f"{active['settle_retries']}/{self.settle_retries} "
                            f"pose={final_pose}")
                    else:
                        active['final_pose'] = final_pose
                        active['reached'] = reached
                        active['phase'] = 'DONE'
                        active['done_event'].set()
            # phase == 'DONE': _execute 가 곧 self._active 를 지운다 — 여기선
            # 아무 것도 하지 않는다(중복 명령 방지).

    def _publish_twist(self, vx, vy, wz):
        msg = Twist()
        msg.linear.x = float(vx)
        msg.linear.y = float(vy)
        msg.angular.z = float(wz)
        self._cmd_pub.publish(msg)

    def _publish_feedback(self, active, pose):
        tx, tz, _ = active['target']
        x, z, _ = pose
        fb = NavigateToPose.Feedback()
        fb.distance_remaining = float(math.hypot(tx - x, tz - z))
        try:
            active['goal_handle'].publish_feedback(fb)
        except Exception:  # noqa: BLE001 — 목표가 막 취소/종료된 경합은 무시
            pass

    # ---- 액션 서버 콜백 ----

    def _on_goal(self, goal_request):
        with self._active_lock:
            if self._active is not None:
                self.get_logger().warn(
                    'navigate_to_pose: 이미 활성 목표가 있어 새 목표를 거부합니다'
                    '(선점 미지원, R4/R5 과제)')
                return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _on_cancel(self, goal_handle):
        return CancelResponse.ACCEPT

    def _execute(self, goal_handle):
        goal = goal_handle.request
        p = goal.pose.pose.position
        q = goal.pose.pose.orientation
        # 지시사항: goal.pose 를 USD 월드 프레임 그대로 해석(map 변환 미배선).
        tx, tz = float(p.x), float(p.z)
        tyaw_deg = goal_quat_to_yaw_deg(float(q.x), float(q.y), float(q.z), float(q.w))
        target = (tx, tz, tyaw_deg)

        # per-goal yaw_tol override: 미사용 behavior_tree 필드에 "yaw_tol=<deg>" 를 실어
        # 스텝별 완화(예: dock_check 는 요 정밀 불필요 — final_align 1° 는 노드기본 유지).
        yaw_tol = self.yaw_tol
        bt = str(goal.behavior_tree or '')
        if 'yaw_tol=' in bt:
            try:
                yaw_tol = float(bt.split('yaw_tol=', 1)[1].split()[0].strip(',;'))
            except (ValueError, IndexError):
                pass

        ctrl = PoseController(
            target, pos_gain=self.pos_gain, yaw_gain=self.yaw_gain,
            max_lin=self.max_lin, max_ang=self.max_ang,
            pos_tol=self.pos_tol, yaw_tol=yaw_tol, yaw_min_cmd=self.yaw_min_cmd,
            linear_accel=self.linear_accel, linear_decel=self.linear_decel,
            angular_accel=self.angular_accel, settle_frames=self.settle_frames)

        done_event = threading.Event()
        active = {
            'ctrl': ctrl, 'goal_handle': goal_handle, 'target': target,
            'phase': 'DRIVING', 'settle_count': 0, 'done_event': done_event,
            'reached': None, 'final_pose': None, 'settle_retries': 0,
        }
        with self._active_lock:
            self._active = active

        self.get_logger().info(
            f'navigate_to_pose: 목표 수락 target=(x={tx:.3f},z={tz:.3f},yaw={tyaw_deg:.2f}deg) '
            f'yaw_tol={yaw_tol:.1f}')

        start_wall = time.monotonic()
        outcome = 'timeout'
        try:
            while rclpy.ok():
                if goal_handle.is_cancel_requested:
                    outcome = 'canceled'
                    break
                if done_event.wait(timeout=0.1):
                    outcome = 'done'
                    break
                elapsed = time.monotonic() - start_wall
                if elapsed > self.goal_timeout_sec:
                    outcome = 'timeout'
                    break
                if self._last_pose_wall_time is None:
                    pose_age = math.inf
                else:
                    pose_age = time.monotonic() - self._last_pose_wall_time
                if pose_age > self.cmd_watchdog_sec:
                    # 코스팅 방지(위 cmd_watchdog_sec 선언부 설명 참고): 포즈가
                    # 잠깐이라도 끊기면 마지막 명령이 무기한 유지되지 않도록
                    # 즉시 0 을 발행한다. 포즈가 회복되면 _handle_pose 가 다시
                    # 정상 명령을 낸다 — 여기서는 반복 발행해도 무해(멱등)하다.
                    self._publish_twist(0.0, 0.0, 0.0)
                if pose_age > self.pose_stale_timeout_sec:
                    outcome = 'stale_pose'
                    break
        finally:
            with self._active_lock:
                self._active = None
            self._publish_twist(0.0, 0.0, 0.0)

        result = NavigateToPose.Result()
        if outcome == 'canceled':
            self.get_logger().info('navigate_to_pose: 취소됨')
            goal_handle.canceled()
        elif outcome == 'done' and active['reached']:
            self.get_logger().info(
                f"navigate_to_pose: 도달 성공 final_pose={active['final_pose']}")
            goal_handle.succeed()
        else:
            if outcome == 'done':
                self.get_logger().warn(
                    f"navigate_to_pose: 정지했으나 허용오차 밖(reached=False) "
                    f"final_pose={active['final_pose']} target={target}")
            else:
                self.get_logger().warn(f'navigate_to_pose: {outcome} 로 중단')
            goal_handle.abort()
        return result


def main(args=None):
    rclpy.init(args=args)
    node = PoseControllerNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
