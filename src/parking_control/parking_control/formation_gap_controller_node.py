#!/usr/bin/env python3
"""formation_gap_controller: 로봇 2대(leader/follower) 간격 유지 + 공동 정지.

로봇 1대마다 이 노드를 하나씩 상시 띄워둔다(실제 로봇의 온보드 소프트웨어
처럼 — task_dispatcher가 필요할 때마다 껐다 켰다 하는 프로세스가 아니다).
평소엔 idle 상태로 아무것도 발행하지 않고 대기하다가, task_dispatcher가
formation_assignment 토픽으로 "너는 지금부터 task T의 leader/follower다"를
보내면 그때부터만 아래 로직이 켜진다(_on_assignment). 작업이 끝나면
active=false 배정이 다시 와서 idle로 되돌아간다.

2026-07-21 논의로 확정한 제어 설계:

  - follower는 leader의 odom을 구독해서 "leader 로컬 좌표계 기준 뒤로
    gap_m 떨어진 지점"을 목표로 추종한다 (core/gap_hold_controller.py).
    전진/후진 어느 방향이든 같은 식이 그대로 적용되므로 role을 방향에
    따라 바꿀 필요가 없다.
  - "한쪽만 멈춤"이 가장 위험하므로(차가 뒤틀림), 아래 중 하나라도
    해당하면 두 로봇 다 즉시 정지한다 (core/formation_costop.py):
      1) 파트너 odom이 watchdog 시간 안에 안 옴 (통신 두절)
      2) 파트너가 formation_stop 토픽으로 "나 멈춰야 해"를 방송함
      3) 자기 자신에게 이상이 생김 (지금은 TODO 훅만 있음 — 힘 센서 등
         실제 하드웨어 신호가 붙기 전까지는 항상 False)
    formation_stop은 공용 토픽(네임스페이스 없음)이라 task_id로 우리
    팀 메시지인지 걸러낸다.

알려진 한계 (다음 단계):
  - leader 역할일 때 이 노드는 주행 명령을 만들지 않는다(정지 게이트만
    담당) — 실제 주행은 아직 스켈레톤인 navigate_action_server 쪽 몫이다.
    그 노드가 실제로 붙으면 cmd_vel 발행 주체를 조율해야 한다.
  - watchdog 두절로 인한 정지는 파트너 odom이 다시 들어오면 자동으로
    풀린다(통신이 잠깐 끊겼다 돌아온 거라 모호함이 해소됐다고 봄). 하지만
    파트너가 명시적으로 "나 이상 있어"(FormationStop stop=True)를 보낸
    경우는 자동으로 안 풀린다 — 그 원인(힘 센서 이상 등)이 실제로
    해결됐는지 이 노드는 알 방법이 없으므로, 사람이 개입해서 해제해야
    한다(자동 복구 로직은 TODO).
  - leader 로봇의 파트너 odom 토픽은 "/<partner_robot_id>/odom" 컨벤션을
    그대로 가정한다 — 실제 로봇 네임스페이스 규칙이 이것과 다르게 정해지면
    여기도 맞춰 바꿔야 한다.
"""

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node

from parking_robot_interfaces.msg import FormationAssignment, FormationStop

from parking_control.core.formation_costop import is_stale, should_stop
from parking_control.core.gap_hold_controller import (
    GapHoldController, Pose2D, yaw_from_quaternion,
)


def _pose_from_odom(msg: Odometry) -> Pose2D:
    p = msg.pose.pose.position
    q = msg.pose.pose.orientation
    return Pose2D(x=p.x, y=p.y, yaw=yaw_from_quaternion(q.x, q.y, q.z, q.w))


class FormationGapControllerNode(Node):

    def __init__(self):
        super().__init__("formation_gap_controller")

        self.declare_parameter("robot_id", "")   # 이 노드가 대표하는 로봇의 고정 식별자
        self.declare_parameter("gap_m", 2.9)
        self.declare_parameter("watchdog_timeout_sec", 0.5)
        self.declare_parameter("control_rate_hz", 10.0)
        self.declare_parameter("k_linear", 1.0)
        self.declare_parameter("k_angular", 2.0)
        self.declare_parameter("max_linear", 0.5)
        self.declare_parameter("max_angular", 1.0)

        p = self.get_parameter
        self._robot_id = p("robot_id").value
        self._watchdog_timeout = float(p("watchdog_timeout_sec").value)
        self._controller = GapHoldController(
            gap_m=float(p("gap_m").value),
            k_linear=float(p("k_linear").value),
            k_angular=float(p("k_angular").value),
            max_linear=float(p("max_linear").value),
            max_angular=float(p("max_angular").value))

        # 아래는 formation_assignment로 배정이 올 때까지는 전부 빈 상태(idle).
        self._active = False
        self._task_id = ""
        self._role = None
        self._partner_robot_id = ""
        self._partner_sub = None
        self._own_pose = None
        self._partner_pose = None
        self._last_partner_odom_at = None
        self._peer_requested_stop = False
        self._self_fault = False   # TODO: 실제 하드웨어 이상 감지 연결
        self._was_stopped = False  # 정지 전환 시점만 로그로 남기기 위한 상태

        self._cmd_pub = self.create_publisher(Twist, "cmd_vel", 10)
        self._stop_pub = self.create_publisher(FormationStop, "formation_stop", 10)
        self.create_subscription(Odometry, "odom", self._on_own_odom, 10)
        self.create_subscription(
            FormationStop, "formation_stop", self._on_formation_stop, 10)
        self.create_subscription(
            FormationAssignment, "formation_assignment", self._on_assignment, 10)

        rate = float(p("control_rate_hz").value)
        self.create_timer(1.0 / rate, self._on_tick)

        self.get_logger().info(
            f"formation_gap_controller 시작 (robot_id={self._robot_id}) — "
            "배정 대기 중(idle)")

    # ---- 구독 콜백 ----

    def _on_own_odom(self, msg):
        self._own_pose = _pose_from_odom(msg)

    def _on_partner_odom(self, msg):
        self._partner_pose = _pose_from_odom(msg)
        self._last_partner_odom_at = self._now_sec()

    def _on_assignment(self, msg):
        if msg.robot_id != self._robot_id:
            return

        if self._partner_sub is not None:
            self.destroy_subscription(self._partner_sub)
            self._partner_sub = None

        if not msg.active:
            if self._active:
                self.get_logger().info(
                    f"배정 해제(task {self._task_id[:8]}) — idle로 복귀")
            self._active = False
            self._task_id = ""
            self._role = None
            self._partner_robot_id = ""
            return

        # 새 작업 시작 — 이전 작업의 흔적(공동정지 플래그 등)을 들고 가지 않는다.
        self._task_id = msg.task_id
        self._role = msg.role
        self._partner_robot_id = msg.partner_robot_id
        self._partner_pose = None
        self._last_partner_odom_at = None
        self._peer_requested_stop = False
        self._was_stopped = False
        self._active = True

        partner_topic = f"/{msg.partner_robot_id}/odom"
        self._partner_sub = self.create_subscription(
            Odometry, partner_topic, self._on_partner_odom, 10)
        self.get_logger().info(
            f"배정 수신: task {msg.task_id[:8]} role={msg.role} "
            f"partner={msg.partner_robot_id}({partner_topic})")

    def _on_formation_stop(self, msg):
        if (not self._active or msg.task_id != self._task_id
                or msg.source_robot_id == self._robot_id):
            return
        if msg.stop:
            self._peer_requested_stop = True
            self.get_logger().warn(
                f"파트너({msg.source_robot_id})가 공동 정지 요청: {msg.reason}")

    # ---- 제어 루프 ----

    def _now_sec(self):
        return self.get_clock().now().nanoseconds / 1e9

    def _on_tick(self):
        if not self._active:
            return   # 배정 전에는 아무것도 발행하지 않는다

        now = self._now_sec()
        peer_stale = is_stale(now, self._last_partner_odom_at, self._watchdog_timeout)
        stop_now = should_stop(self._self_fault, peer_stale, self._peer_requested_stop)

        if stop_now:
            self._cmd_pub.publish(Twist())   # 전부 0 — 즉시 정지
            reason = ("파트너 신호 두절" if peer_stale else
                      "파트너 정지 요청" if self._peer_requested_stop else
                      "자기 이상 감지")
            self._stop_pub.publish(FormationStop(
                task_id=self._task_id, source_robot_id=self._robot_id,
                stop=True, reason=reason))
            if not self._was_stopped:
                self.get_logger().warn(f"공동 정지 발동: {reason}")
                self._was_stopped = True
            return

        if self._was_stopped:
            self.get_logger().info("공동 정지 해제 조건 충족 — 재개")
            self._was_stopped = False

        if self._role == "follower" and self._own_pose and self._partner_pose:
            cmd = self._controller.compute(self._own_pose, self._partner_pose)
            twist = Twist()
            twist.linear.x = cmd.linear_x
            twist.angular.z = cmd.angular_z
            self._cmd_pub.publish(twist)
        # role == "leader": 이 노드는 정지 게이트만 담당, 주행 명령은
        # 만들지 않는다 (navigate_action_server 쪽 몫 — 위 docstring 참고).


def main(args=None):
    rclpy.init(args=args)
    node = FormationGapControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
