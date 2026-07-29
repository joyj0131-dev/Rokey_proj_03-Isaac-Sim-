#!/usr/bin/env python3
"""lift_action_server (v4 브리지 대응) — R4: ``ControlLift`` 액션 백엔드.

설계서 ``docs/superpowers/specs/2026-07-25-ros2-node-refactor-design.md`` §3/4(R4).
``parking_robot_interfaces/action/ControlLift``(이미 정의됨: ``string command``
UP/DOWN -> ``bool success, string support_state``, feedback ``string status``)
를 서빙해 R2 브리지(``parking_v4_runner.sh --bridge``, taskR2-report.md §2.3)의
``/robot_<id>/lift_cmd``(``std_msgs/Float32``, 0.0~1.0 팔 전개 스케일, 1.0/s
램프)를 명령한다. 인프로세스 러너의 ``deploy_arms``/리프트 시퀀스를 대체한다.

## 배치 — 왜 ``parking_robot_system`` 이 아니라 ``parkbot_motion`` 인가

``src/parking_robot_system/parking_robot_system/lift_action_server.py`` 에
이미 같은 이름의 파일이 있다. **재사용하지 않고 이 패키지에 새로 만든
이유**: 그 파일은 완전히 다른(구) 아키텍처용이다 —
  - 로봇 페어를 ``robot_rear``/``robot_front`` **고정 상수**로 다룬다
    (``ROBOTS = ("robot_rear", "robot_front")``, 이 프로젝트의 v4 러너가 쓰는
    ``robot_<id>``(entry_lead/entry_follow/exit_lead/exit_follow) 네이밍과
    안 맞는다).
  - 리프트 명령이 ``std_srvs/SetBool`` **서비스**(``/{robot}/arm_control``)다 —
    R2 브리지의 계약은 ``std_msgs/Float32`` **토픽**(``lift_cmd``, 램프는
    브리지가 이미 다 함)이라 완전히 다른 프로토콜.
  - 완료 판정을 ``/vehicle/pose``(GT 트럭 Y좌표) **구독**으로 한다 — R2 브리지는
    GT 를 콘솔에만 찍고 어떤 토픽으로도 발행하지 않는다(설계서 전제,
    taskR2-report.md §2.4: "GT는 콘솔에만 노출하고 제어 경로에는 절대 넣지
    않았다"). 그 토픽 자체가 v4 브리지엔 존재하지 않는다.

  즉 그 파일은 v4 브리지와 **메시지 계약이 통째로 다른** 레거시(v2 계열)
  백엔드다. 억지로 두 프로토콜을 한 노드에 욱여넣는 것보다, R2/R3 가 이미
  이 패키지(``parkbot_motion``)에 v4 브리지 전용 노드들을 모아온 관례
  (``pose_controller_node`` 등)를 그대로 따라 새 파일로 분리하는 게 더
  안전하다(레거시 노드를 건드리지 않아 그쪽 회귀 위험이 없고, 액션 이름도
  로봇별로 분리돼(§ 아래) 여러 로봇을 동시에 띄워도 서로 안 밟는다). 레거시
  파일의 v4 전환 여부는 이 작업 범위 밖(후속 판단).

## 완료 판정 — 왜 시간 기반 대기인가(정직하게, 대안 없음을 확인함)

R2 브리지는 램프를 **시뮬레이션 시간**(``now_sim - prev_sim``, ``timeline.
get_current_time()`` 기준, taskR2-report.md 3956-3989행)으로 적분한다.
반면 이 노드가 구독 가능한 모든 ROS2 메시지(``/robot_<id>/odom``,
``/joint_states`` 등)의 ``header.stamp`` 는 **벽시계**(``ros_node.get_clock().
now()``, taskR3c-report.md §4 명시)로 채워진다 — 즉 이 프로세스는 "시뮬
시간이 실제로 얼마나 흘렀는지" 알 방법이 ROS2 경로에 전혀 없다(브리지가
그 정보를 어떤 토픽으로도 노출하지 않는다 — GT 와 마찬가지로 설계 전제).
그리고 이 머신의 헤드리스 RTF(=sim_s/wall_s)는 실측상 **~0.5~1.0 사이에서
변동**한다(taskR2-report.md §4.4: RTF≈0.5 실측, 카메라 대수·부하에 따라
달라짐) — "sim 1.0초 램프"가 벽시계로 몇 초 걸릴지 고정값이 아니다.

트럭 높이(``/vehicle/pose`` 류)도 R2 브리지엔 없다(위 배치 이유 절 참고) —
그래서 (구) ``lift_action_server`` 가 쓰던 "물리적으로 실제 상승했는지"
피드백 판정도 이 노드에선 못 쓴다. **남은 선택지는 벽시계 타임아웃뿐**이라
브리프 지시("ramping is already handled bridge-side; return success when the
ramp has had time to complete")를 그대로 따른다: ``lift_cmd`` 를 발행한 뒤
``ramp_wait_sec`` 만큼 대기하고 성공을 반환한다.

``ramp_wait_sec`` 기본값(파라미터로 오버라이드 가능)은 taskR4-report.md 의
라이브 스모크로 보정한 값이다(그 보고서 §라이브 스모크 참고 — RTF 최악
관측치 기준으로 sim 1.0초 램프 + 물리적 정착(트럭이 실제로 올라오는 시간)
여유를 더한 값). **정직한 한계**: RTF 가 이번 스모크보다 더 나쁜 조건(카메라
더 많이 켜짐 등)에서 돌면 이 고정값이 부족할 수 있다 — R5 가 이 노드를
쓸 때 자기 환경의 RTF 를 감안해 ``ramp_wait_sec`` 을 다시 재보정해야 한다.
"""
import time

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import Float32

from parking_robot_interfaces.action import ControlLift

# 브리지 램프율(parking_v4_runner.py --bridge, LIFT_RAMP_RATE=1.0, sim 초당
# 0..1) 미러 — 이 노드는 sim 시간을 못 읽으므로 값 자체를 제어에 쓰진 않지만
# ramp_wait_sec 기본값 산출 근거로 문서에 남긴다(위 docstring).
BRIDGE_LIFT_RAMP_RATE = 1.0  # 0..1 스케일 / sim 초


def normalize_lift_command(command):
    """``ControlLift.Goal.command`` -> ``'UP'``|``'DOWN'`` (대소문자 무시).

    순수 함수(rclpy 불필요) — ``_on_goal``/``_execute`` 양쪽이 재사용해 판정
    로직이 갈라지지 않게 한다. 알 수 없는 값은 ``ValueError``.
    """
    normalized = str(command).strip().upper()
    if normalized not in ('UP', 'DOWN'):
        raise ValueError(f"command 는 'UP'|'DOWN' 이어야 합니다: {command!r}")
    return normalized


class LiftActionServerNode(Node):

    def __init__(self):
        super().__init__('lift_action_server')

        self.declare_parameter('robot_id', 'entry_lead')
        robot_id = self.get_parameter('robot_id').value

        self.declare_parameter('lift_cmd_topic', f'/robot_{robot_id}/lift_cmd')
        self.declare_parameter('action_name', f'/robot_{robot_id}/control_lift')
        # 기본 3.0s = sim 1.0초 램프 / RTF 하한 실측(~0.5) 여유 2배 + 물리 정착
        # 여유. taskR4-report.md 라이브 스모크로 실측 검증(§ 위 docstring).
        self.declare_parameter('ramp_wait_sec', 3.0)
        # 목표 취소를 즉시 반영하기 위한 대기 폴링 주기.
        self.declare_parameter('poll_sec', 0.1)

        gp = self.get_parameter
        self.robot_id = robot_id
        self.lift_cmd_topic = gp('lift_cmd_topic').value
        self.action_name = gp('action_name').value
        self.ramp_wait_sec = float(gp('ramp_wait_sec').value)
        self.poll_sec = float(gp('poll_sec').value)

        cbg = ReentrantCallbackGroup()
        self._lift_pub = self.create_publisher(Float32, self.lift_cmd_topic, 10)
        self._action_server = ActionServer(
            self, ControlLift, self.action_name, self._execute,
            goal_callback=self._on_goal, cancel_callback=self._on_cancel,
            callback_group=cbg)

        self.get_logger().info(
            f'lift_action_server 시작: robot_id={robot_id} lift_cmd_topic={self.lift_cmd_topic} '
            f'action={self.action_name} ramp_wait_sec={self.ramp_wait_sec}')

    def _on_goal(self, goal_request):
        try:
            normalize_lift_command(goal_request.command)
        except ValueError:
            self.get_logger().warn(
                f'control_lift: 알 수 없는 command={goal_request.command!r}, 거부')
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _on_cancel(self, goal_handle):
        return CancelResponse.ACCEPT

    def _publish_scale(self, scale):
        msg = Float32()
        msg.data = float(scale)
        self._lift_pub.publish(msg)

    def _execute(self, goal_handle):
        command = normalize_lift_command(goal_handle.request.command)
        up = (command == 'UP')
        scale = 1.0 if up else 0.0

        self.get_logger().info(
            f'control_lift: {command} 수락 -> {self.lift_cmd_topic}={scale}')

        feedback = ControlLift.Feedback()
        feedback.status = 'ramping'

        # 초반 discovery 경합(구독자가 아직 매칭 안 된 순간의 유실)에 대비해
        # 대기 구간 내내 반복 발행한다 — 멱등(같은 값 반복 발행은 무해)이고,
        # 브리지 쪽 lift_cmd 콜백은 매 수신마다 그냥 최신값을 덮어쓸 뿐이라
        # 안전하다(taskR2-report.md §2.3).
        deadline = time.monotonic() + self.ramp_wait_sec
        settled_feedback_sent = False
        while True:
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                result = ControlLift.Result()
                result.success = False
                result.support_state = 'UNKNOWN'
                self.get_logger().info('control_lift: 취소됨')
                return result
            self._publish_scale(scale)
            now = time.monotonic()
            if now >= deadline:
                break
            if not settled_feedback_sent and (deadline - now) <= self.ramp_wait_sec * 0.3:
                feedback.status = 'settling'
                settled_feedback_sent = True
            try:
                goal_handle.publish_feedback(feedback)
            except Exception:  # noqa: BLE001 — 목표가 막 종료된 경합은 무시
                pass
            time.sleep(min(self.poll_sec, max(0.0, deadline - now)))

        result = ControlLift.Result()
        result.success = True
        result.support_state = 'SUPPORTED' if up else 'RELEASED'
        goal_handle.succeed()
        self.get_logger().info(
            f'control_lift: {command} 완료(대기 {self.ramp_wait_sec}s 경과) '
            f'support_state={result.support_state}')
        return result


def main(args=None):
    rclpy.init(args=args)
    node = LiftActionServerNode()
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
