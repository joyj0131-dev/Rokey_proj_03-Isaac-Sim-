#!/usr/bin/env python3
"""ingress_node — R5a: 차 밑 진입(측면 뎁스 중앙유지 + 축 중간값 정지) 액션 서버.

설계서 ``docs/superpowers/specs/2026-07-25-ros2-node-refactor-design.md`` §3~4(R5a).
전제: R4 ``axle_detector_node``(측면 뎁스 -> 트로프 완료 발행, taskR4-report.md),
R3c 자세 배선(``pose_topic``/``pose_msg_type``, taskR3c-report.md).

## 무엇을 재구현하지 않았는가 — 트로프 검출은 ``axle_detector_node`` 에 위임한다

브리프가 "축감지를 ``axle_detector_node`` 출력을 구독하거나, 자체
``TroughTracker`` 를 돌리거나 — 하나를 고르고, 두 진실원을 만들지 말 것"을
요구한다. 이 노드는 **전자**를 골랐다: ``/robot_<id>/axle_center``
(``geometry_msgs/PointStamped``)/``axle_index``(``std_msgs/Int32``) 를 구독해
"트로프가 몇 개 확정됐고 각각 어디였는가"의 유일한 진실원으로 삼는다. 이유:

1. ``axle_detector_node`` 는 이미 R4 에서 실측 검증됐다(후축 오차 1mm, 전축
   오차 2.12cm, taskR4-report.md §2.3). 같은 ``TroughTracker``/ROI-min 계산을
   이 노드에서 다시 돌리면, 두 노드가 서로 다른 좌/우 뎁스 콜백 타이밍(다른
   구독자 큐, 다른 콜백 스레드 스케줄)으로 인해 **미세하게 다른 트로프
   경계**를 낼 수 있다 — 그 어긋남이 "이 노드가 도는 트로프"와 "저 노드가
   보고하는 트로프"를 서로 다른 값으로 갈라놓으면(둘 다 각자는 정직하게
   동작해도) 어느 쪽이 맞는지 디버깅하기 어려운 이중 진실원 문제가 생긴다.
2. 반면 **횡(vy) 중앙유지는 매 틱(포즈 콜백)마다 필요한 고빈도 연속 제어
   법칙**이라 ``axle_detector_node`` 가 노출하지 않는다(그 노드는 트로프
   "완료" 이벤트만 발행한다, 연속 ROI-min 값은 캐시에만 있고 토픽으로 내지
   않는다) — 그래서 이 노드는 좌/우 뎁스 원시 토픽을 **직접** 구독해 중앙유지
   법칙(``ingress_control.lateral_centring_vy``)을 스스로 계산한다. 이건 "같은
   목적의 로직을 두 곳에 만드는" 것이 아니다 — ROI-min 추출 자체는
   ``depth_stop_detector.roi_min_depth``/``axle_detector_node.
   depth_image_to_array`` **순수 함수를 그대로 재사용**(복붙 아님, import)하고,
   그 위에서 하는 일(연속 중앙유지 vs 트로프 이벤트 검출)이 서로 다른
   목적이라 "축감지 로직 이중화"에 해당하지 않는다고 판단했다.

## 자세는 이 노드의 "틱" — ``axle_detector_node``/``pose_controller_node`` 와 동일 설계

포즈 콜백이 도착할 때마다 (1) 캐시된 좌/우 뎁스로 vy 를 계산하고, (2) 지금까지
쌓인 ``axle_centers`` 스냅샷과 함께 ``IngressController.step()`` 을 한 번 불러
이번 틱에 낼 twist 를 얻는다. 뎁스/축감지 토픽은 각자 콜백에서 최신값만
캐시한다(비동기 다중 입력, ``axle_detector_node`` 와 동일한 이유).

## 안전장치 — 두 개의 독립된 워치독

1. **포즈 코스팅 방지**(``cmd_watchdog_sec``, 기본 0.5s): ``pose_controller_node``
   가 R3c 에서 실측으로 찾은 버그(포즈 유실 시 마지막 명령이 무기한 유지돼
   최대 5초간 ~81° 무통제 회전)와 같은 클래스의 위험이 이 노드에도 있다 —
   ``cmd_vel`` 은 포즈 콜백 안에서만 발행되므로, 포즈가 끊기면 마지막 값이
   그대로 유지된다. ``_execute`` 의 폴링 루프(포즈 콜백과 별개 스레드, 포즈가
   안 와도 계속 돈다)가 마지막 포즈 이후 경과시간을 감시해 이 값을 넘기면
   즉시 (0,0,0)을 발행한다(같은 패턴을 ``pose_controller_node._execute`` 에서
   그대로 가져옴). ``pose_stale_timeout_sec``(기본 5.0s) 를 넘기면 목표 자체를
   ``pose_stale`` 로 중단한다.
2. **뎁스 완전 유실 시 전체 정지**(브리프 명시 지시): 좌/우 뎁스 콜백이 각자
   마지막 수신 시각을 기록한다. 포즈 틱마다 "각 채널이 ``depth_stale_sec``
   (기본 1.0s) 이내에 수신됐는가"를 판정해, 신선하지 않으면 그 채널값을
   ``None`` 으로 저하시켜 ``IngressController`` 에 넘긴다(중앙유지는 자연히
   vy=0 으로 열화 — ``lateral_centring_vy`` 의 "둘 다 유효해야 보정" 규칙).
   **양쪽 다** 신선하지 않은 상태가 ``depth_loss_timeout_sec``(기본 2.0s) 이상
   지속되면 그건 단순 열화가 아니라 "뎁스 신호가 아예 없다"는 뜻이라 판단해
   즉시 (0,0,0) 을 발행하고 목표를 ``depth_lost`` 로 중단한다 — 트럭 밑을
   뎁스 없이 맹목 주행하는 것은 R4 가 실측으로 보여준 요-드리프트 충돌 위험을
   그대로 안고 가는 것이므로(taskR4-report.md §2.4).

## 주행좌표 부호 가정(``axle_detector_node``/러너와 동일, 문서화된 한계)

``ingress_control.return_phase_vx`` 의 부호는 "로봇 로컬 forward(+vx) 가 세계
좌표 x 를 **감소**시킨다"는 이 미션의 실측 관례(진입선에서 yaw=-90°, 즉 -x 를
보고 전진)에서만 성립한다. 다른 헤딩으로 진입하는 배치가 생기면 이 가정을
재검증해야 한다(``axle_detector_node`` 의 "주행좌표=world x" 가정과 같은
종류의, 문서화된 단일-미션 한계).

## 액션 선택 근거

기존 ``parking_robot_interfaces``엔 이 계약(트로프 인덱스 목표 -> 정지좌표
결과)에 맞는 액션이 없었다(``AlignVehicle`` 은 ``geometry_msgs/Pose`` 목표라
"몇 번째 축" 같은 이산값을 표현할 수 없고, 완전히 다른 용도인 SR-04 정밀정렬
프로토콜이다). R4 가 ``ControlLift`` 를 새로 만든 것과 같은 판단 근거로,
``IngressUnderTruck`` 액션을 새로 정의했다(플레인 서비스보다 액션을 고른 이유:
이 안무는 수십 초 걸리는 장시간 작업이고, ``pose_controller_node``/
``lift_action_server`` 모두 이미 액션으로 노출돼 있어 R5b 오케스트레이터가
일관된 방식으로 세 노드를 조합할 수 있다 — 취소/피드백/타임아웃이 서비스보다
액션에서 자연스럽다).
"""
import math
import threading
import time

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from geometry_msgs.msg import PointStamped, PoseStamped, Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
from std_msgs.msg import Int32

from parking_robot_interfaces.action import IngressUnderTruck
from parkbot_motion.axle_detector_node import DEPTH_ROI_FRAC, depth_image_to_array
from parkbot_motion.depth_stop_detector import roi_min_depth
from parkbot_motion.ingress_control import (
    DEFAULT_FORWARD_SPEED, DEFAULT_LAT_DEADBAND, DEFAULT_LAT_KP, DEFAULT_LAT_VY_MAX,
    DEFAULT_POS_TOL, DEFAULT_RETURN_SPEED, DEFAULT_SETTLE_FRAMES, IngressController)
from parkbot_motion.pose_controller_node import resolve_pose_msg_type


class IngressNode(Node):

    def __init__(self):
        super().__init__('ingress_node')

        self.declare_parameter('robot_id', 'entry_lead')
        robot_id = self.get_parameter('robot_id').value

        self.declare_parameter('left_depth_topic', f'/robot_{robot_id}/left/depth')
        self.declare_parameter('right_depth_topic', f'/robot_{robot_id}/right/depth')
        self.declare_parameter('pose_topic', f'/robot_{robot_id}/odom')
        self.declare_parameter('pose_msg_type', 'auto')
        self.declare_parameter('axle_center_topic', f'/robot_{robot_id}/axle_center')
        self.declare_parameter('axle_index_topic', f'/robot_{robot_id}/axle_index')
        self.declare_parameter('cmd_vel_topic', f'/robot_{robot_id}/cmd_vel')
        self.declare_parameter('action_name', f'/robot_{robot_id}/ingress_under_truck')

        self.declare_parameter('roi_frac', list(DEPTH_ROI_FRAC))
        self.declare_parameter('forward_speed', DEFAULT_FORWARD_SPEED)
        self.declare_parameter('return_speed', DEFAULT_RETURN_SPEED)
        self.declare_parameter('lat_kp', DEFAULT_LAT_KP)
        self.declare_parameter('lat_vy_max', DEFAULT_LAT_VY_MAX)
        self.declare_parameter('lat_deadband', DEFAULT_LAT_DEADBAND)
        self.declare_parameter('pos_tol', DEFAULT_POS_TOL)
        self.declare_parameter('settle_frames', DEFAULT_SETTLE_FRAMES)

        self.declare_parameter('max_dt', 0.5)
        self.declare_parameter('goal_timeout_sec', 120.0)
        self.declare_parameter('pose_stale_timeout_sec', 5.0)
        self.declare_parameter('cmd_watchdog_sec', 0.5)
        # 뎁스 워치독(브리프 지시 "stop on loss of depth data") — depth_stale_sec
        # 이상 못 받은 채널은 그 값을 무효(None)로 저하시키고(중앙유지가 자연히
        # vy=0 로 열화), 양쪽 다 depth_loss_timeout_sec 이상 무효 상태면 전체
        # 정지 + 목표 중단(depth_lost). 클래스 docstring §"안전장치" 참고.
        self.declare_parameter('depth_stale_sec', 1.0)
        self.declare_parameter('depth_loss_timeout_sec', 2.0)

        gp = self.get_parameter
        self.robot_id = robot_id
        self.left_depth_topic = gp('left_depth_topic').value
        self.right_depth_topic = gp('right_depth_topic').value
        self.pose_topic = gp('pose_topic').value
        pose_msg_type_param = str(gp('pose_msg_type').value)
        self.pose_msg_type, pose_msg_type_how = resolve_pose_msg_type(
            pose_msg_type_param, self.pose_topic, self.get_topic_names_and_types())
        self.axle_center_topic = gp('axle_center_topic').value
        self.axle_index_topic = gp('axle_index_topic').value
        self.cmd_vel_topic = gp('cmd_vel_topic').value
        self.action_name = gp('action_name').value

        self.roi_frac = tuple(float(v) for v in gp('roi_frac').value)
        self.forward_speed = float(gp('forward_speed').value)
        self.return_speed = float(gp('return_speed').value)
        self.lat_kp = float(gp('lat_kp').value)
        self.lat_vy_max = float(gp('lat_vy_max').value)
        self.lat_deadband = float(gp('lat_deadband').value)
        self.pos_tol = float(gp('pos_tol').value)
        self.settle_frames = int(gp('settle_frames').value)

        self.max_dt = float(gp('max_dt').value)
        self.goal_timeout_sec = float(gp('goal_timeout_sec').value)
        self.pose_stale_timeout_sec = float(gp('pose_stale_timeout_sec').value)
        self.cmd_watchdog_sec = float(gp('cmd_watchdog_sec').value)
        self.depth_stale_sec = float(gp('depth_stale_sec').value)
        self.depth_loss_timeout_sec = float(gp('depth_loss_timeout_sec').value)

        # ---- 상태(여러 콜백 스레드가 건드림 -> self._state_lock 으로 보호) ----
        self._state_lock = threading.Lock()
        self._axle_centers = []          # axle_center 콜백이 순서대로 append
        self._active = None              # 활성 목표 dict 또는 None

        # 뎁스 캐시(락 불필요 -- 단순 스칼라 대입, axle_detector_node 와 동일 관례)
        self._left_min = None
        self._right_min = None
        self._last_left_wall = None
        self._last_right_wall = None
        self._depth_loss_since = None

        self._last_seen_axle_index = None
        self._last_stamp_sec = None
        self._last_pose_wall_time = None
        self._last_travel_x = None

        cbg = ReentrantCallbackGroup()
        self._cmd_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)

        self.create_subscription(Image, self.left_depth_topic, self._on_left_depth,
                                  qos_profile_sensor_data, callback_group=cbg)
        self.create_subscription(Image, self.right_depth_topic, self._on_right_depth,
                                  qos_profile_sensor_data, callback_group=cbg)
        self.create_subscription(PointStamped, self.axle_center_topic, self._on_axle_center,
                                  10, callback_group=cbg)
        self.create_subscription(Int32, self.axle_index_topic, self._on_axle_index,
                                  10, callback_group=cbg)
        if self.pose_msg_type == 'odometry':
            self.create_subscription(Odometry, self.pose_topic, self._on_odom, 10,
                                      callback_group=cbg)
        else:
            self.create_subscription(PoseStamped, self.pose_topic, self._on_pose_stamped, 10,
                                      callback_group=cbg)

        # ★동시성 패턴(pose_controller_node.py 와 동일): _execute 는 목표 완료까지
        # 블로킹 폴링한다 -- 구독 콜백들이 같은 그룹에서 계속 돌아야 그 사이
        # self._active/self._axle_centers 를 갱신할 수 있다. MultiThreadedExecutor
        # 로 스핀해야 한다(main() 참고).
        self._action_server = ActionServer(
            self, IngressUnderTruck, self.action_name, self._execute,
            goal_callback=self._on_goal, cancel_callback=self._on_cancel,
            callback_group=cbg)

        self.get_logger().info(
            f'ingress_node 시작: robot_id={robot_id} '
            f'left={self.left_depth_topic} right={self.right_depth_topic} '
            f'pose_topic={self.pose_topic} pose_msg_type={self.pose_msg_type}'
            f'({pose_msg_type_how}) axle_center={self.axle_center_topic} '
            f'cmd_vel={self.cmd_vel_topic} action={self.action_name} '
            f'forward_speed={self.forward_speed} return_speed={self.return_speed} '
            f'roi_frac={self.roi_frac}')

    # ---- 뎁스 구독 콜백: 최신 ROI-min + 수신시각 캐시만 갱신 ----

    def _on_left_depth(self, msg):
        self._left_min = self._roi_min(msg)
        self._last_left_wall = time.monotonic()

    def _on_right_depth(self, msg):
        self._right_min = self._roi_min(msg)
        self._last_right_wall = time.monotonic()

    def _roi_min(self, msg):
        try:
            grid = depth_image_to_array(msg.height, msg.width, msg.step, msg.data,
                                         encoding=msg.encoding)
        except ValueError as exc:
            self.get_logger().warn(f'뎁스 프레임 파싱 실패, 이 프레임 스킵: {exc}')
            return None
        value = roi_min_depth(grid, roi_frac=self.roi_frac)
        return value if math.isfinite(value) else None

    # ---- 축감지 구독 콜백: axle_detector_node 를 유일한 진실원으로 삼는다 ----

    def _on_axle_center(self, msg):
        with self._state_lock:
            self._axle_centers.append(float(msg.point.x))
            n = len(self._axle_centers)
        last_idx = self._last_seen_axle_index
        if last_idx is not None and last_idx != n - 1:
            self.get_logger().warn(
                f'axle_center/axle_index 페어링 불일치 의심: '
                f'axle_index={last_idx} but axle_centers 길이={n}')
        self.get_logger().info(f'axle_center 수신 #{n - 1}: x={msg.point.x:.4f}')

    def _on_axle_index(self, msg):
        self._last_seen_axle_index = int(msg.data)

    # ---- 자세 구독 콜백: 이 노드의 "틱" ----

    def _on_odom(self, msg):
        p = msg.pose.pose.position
        self._handle_pose(p.x, msg.header.stamp)

    def _on_pose_stamped(self, msg):
        p = msg.pose.position
        self._handle_pose(p.x, msg.header.stamp)

    def _handle_pose(self, x, stamp):
        x = float(x)
        self._last_travel_x = x
        self._last_pose_wall_time = time.monotonic()

        stamp_sec = stamp.sec + stamp.nanosec * 1e-9
        if self._last_stamp_sec is None:
            # 첫 메시지 -- dt 를 낼 이전 표본이 없다(pose_controller_node 와 동일).
            self._last_stamp_sec = stamp_sec
            return
        dt = stamp_sec - self._last_stamp_sec
        self._last_stamp_sec = stamp_sec
        if dt <= 0.0:
            return
        dt = min(dt, self.max_dt)

        with self._state_lock:
            active = self._active
            if active is None:
                return
            ctrl = active['ctrl']

            now = time.monotonic()
            left_fresh = (self._last_left_wall is not None
                          and (now - self._last_left_wall) <= self.depth_stale_sec)
            right_fresh = (self._last_right_wall is not None
                           and (now - self._last_right_wall) <= self.depth_stale_sec)
            eff_left = self._left_min if left_fresh else None
            eff_right = self._right_min if right_fresh else None

            if not left_fresh and not right_fresh:
                if self._depth_loss_since is None:
                    self._depth_loss_since = now
                depth_loss_age = now - self._depth_loss_since
            else:
                self._depth_loss_since = None
                depth_loss_age = 0.0

            if depth_loss_age > self.depth_loss_timeout_sec:
                self.get_logger().warn(
                    f'ingress_under_truck: 뎁스 신호 완전 유실 '
                    f'{depth_loss_age:.1f}s -> 정지/목표 중단')
                active['outcome'] = 'depth_lost'
                self._publish_twist(0.0, 0.0, 0.0)
                active['done_event'].set()
                return

            axle_centers_snapshot = list(self._axle_centers)
            vx, vy, wz = ctrl.step(x, eff_left, eff_right, axle_centers_snapshot, dt)
            self._publish_twist(vx, vy, wz)
            self._publish_feedback(active, ctrl, x, vy, len(axle_centers_snapshot))
            if ctrl.done:
                active['outcome'] = 'midpoint_reached'
                active['done_event'].set()

    def _publish_twist(self, vx, vy, wz):
        msg = Twist()
        msg.linear.x = float(vx)
        msg.linear.y = float(vy)
        msg.angular.z = float(wz)
        self._cmd_pub.publish(msg)

    def _publish_feedback(self, active, ctrl, x, vy, troughs_seen):
        fb = IngressUnderTruck.Feedback()
        fb.phase = ctrl.phase
        fb.current_x = float(x)
        fb.troughs_seen = int(troughs_seen)
        fb.vy_cmd = float(vy)
        try:
            active['goal_handle'].publish_feedback(fb)
        except Exception:  # noqa: BLE001 -- 목표가 막 취소/종료된 경합은 무시
            pass

    # ---- 액션 서버 콜백 ----

    def _on_goal(self, goal_request):
        with self._state_lock:
            if self._active is not None:
                self.get_logger().warn(
                    'ingress_under_truck: 이미 활성 목표가 있어 새 목표를 거부합니다'
                    '(선점 미지원)')
                return GoalResponse.REJECT
        if int(goal_request.trough_index) < 0:
            self.get_logger().warn(
                f'ingress_under_truck: trough_index<0 거부: {goal_request.trough_index}')
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _on_cancel(self, goal_handle):
        return CancelResponse.ACCEPT

    def _execute(self, goal_handle):
        goal = goal_handle.request
        trough_index = int(goal.trough_index)
        forward_speed = float(goal.forward_speed) if goal.forward_speed > 0.0 else self.forward_speed
        return_speed = float(goal.return_speed) if goal.return_speed > 0.0 else self.return_speed

        ctrl = IngressController(
            trough_index, forward_speed=forward_speed, return_speed=return_speed,
            lat_kp=self.lat_kp, lat_vy_max=self.lat_vy_max, lat_deadband=self.lat_deadband,
            pos_tol=self.pos_tol, settle_frames=self.settle_frames)

        done_event = threading.Event()
        active = {'ctrl': ctrl, 'goal_handle': goal_handle, 'done_event': done_event,
                  'outcome': None}
        with self._state_lock:
            self._active = active

        self.get_logger().info(
            f'ingress_under_truck: 목표 수락 trough_index={trough_index} '
            f'forward_speed={forward_speed} return_speed={return_speed}')

        start_wall = time.monotonic()
        outcome = 'timeout'
        try:
            while rclpy.ok():
                if goal_handle.is_cancel_requested:
                    outcome = 'canceled'
                    break
                if done_event.wait(timeout=0.1):
                    outcome = active['outcome'] or 'midpoint_reached'
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
                    # 코스팅 방지(pose_controller_node._execute 와 동일 패턴):
                    # 포즈가 끊기면 마지막 명령이 무기한 유지되지 않도록 즉시
                    # 0 을 발행한다. 포즈가 회복되면 _handle_pose 가 다시 정상
                    # 명령을 낸다 -- 여기서 반복 발행해도 무해(멱등).
                    self._publish_twist(0.0, 0.0, 0.0)
                if pose_age > self.pose_stale_timeout_sec:
                    outcome = 'pose_stale'
                    break
        finally:
            with self._state_lock:
                self._active = None
            self._publish_twist(0.0, 0.0, 0.0)

        result = IngressUnderTruck.Result()
        result.stop_x = float(ctrl.final_stop_x if ctrl.final_stop_x is not None
                               else (self._last_travel_x if self._last_travel_x is not None
                                     else float('nan')))
        result.target_axle_x = float(ctrl.target_x if ctrl.target_x is not None else float('nan'))
        result.est_max_lateral_dev_m = float(ctrl.max_lat_dev_est)

        if outcome == 'canceled':
            self.get_logger().info('ingress_under_truck: 취소됨')
            result.success = False
            result.stop_reason = 'canceled'
            goal_handle.canceled()
        elif outcome == 'midpoint_reached':
            self.get_logger().info(
                f'ingress_under_truck: 완료 stop_x={result.stop_x:.4f} '
                f'target_axle_x={result.target_axle_x:.4f} '
                f'est_max_lateral_dev_m={result.est_max_lateral_dev_m:.4f}')
            result.success = True
            result.stop_reason = 'midpoint_reached'
            goal_handle.succeed()
        else:
            self.get_logger().warn(f'ingress_under_truck: {outcome} 로 중단')
            result.success = False
            result.stop_reason = outcome
            goal_handle.abort()
        return result


def main(args=None):
    rclpy.init(args=args)
    node = IngressNode()
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
