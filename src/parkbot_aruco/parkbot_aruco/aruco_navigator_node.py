#!/usr/bin/env python3
"""aruco_navigator: NavigateToPose 실물 (팀원 navigate_action_server 스켈레톤의 대역).

pose 소스는 GT가 아니라 마커 fix(robot_pose) + 휠 twist(wheel_twist) 융합이다.
목표 하나 = 웨이포인트 하나. 경로/존 관리는 상위(미션/orchestrator) 몫.
초기 pose 는 파라미터로 받는다(도크 좌표 — 실제 서비스에선 첫 마커가 잡아줌).

주의: ActionServer 실행 콜백 안에서 spin_once 를 돌리는 단일 스레드 + 액션
1개 전용 구조다(데모 용도로 충분. 멀티 goal 은 범위 밖).
"""
import math

import rclpy
from geometry_msgs.msg import PoseStamped, Twist, TwistStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionServer, CancelResponse
from rclpy.node import Node

from parkbot_aruco.pose_estimator import PoseEstimator


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def _wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


class ArucoNavigatorNode(Node):
    def __init__(self):
        super().__init__("aruco_navigator")
        p = self.declare_parameter
        p("initial_x", -15.3); p("initial_y", -7.8); p("initial_yaw", 0.0)
        p("pos_tol", 0.20); p("yaw_tol_deg", 10.0)
        p("k_lin", 0.8); p("k_yaw", 1.2); p("max_lin", 0.35); p("max_yaw", 0.5)
        g = lambda k: self.get_parameter(k).value
        self.est = PoseEstimator(g("initial_x"), g("initial_y"), g("initial_yaw"))
        self.pos_tol, self.yaw_tol = g("pos_tol"), math.radians(g("yaw_tol_deg"))
        self.k_lin, self.k_yaw = g("k_lin"), g("k_yaw")
        self.max_lin, self.max_yaw = g("max_lin"), g("max_yaw")
        self._last_twist_stamp = None
        self.fix_count = 0

        self.cmd_pub = self.create_publisher(Twist, "cmd_vel", 10)
        self.diag_pub = self.create_publisher(PoseStamped, "nav_pose", 10)
        self.create_subscription(TwistStamped, "wheel_twist", self._on_twist, 50)
        self.create_subscription(PoseStamped, "robot_pose", self._on_fix, 10)
        self._server = ActionServer(
            self, NavigateToPose, "navigate_to_pose", self._execute,
            cancel_callback=lambda _: CancelResponse.ACCEPT)
        self.get_logger().info("aruco_navigator 시작 (마커+데드레커닝 융합)")

    def _on_twist(self, m):
        t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        if self._last_twist_stamp is not None:
            dt = max(0.0, min(0.1, t - self._last_twist_stamp))
            self.est.predict(m.twist.linear.x, m.twist.linear.y,
                             m.twist.angular.z, dt)
        self._last_twist_stamp = t

    def _on_fix(self, m):
        self.est.correct(m.pose.position.x, m.pose.position.y,
                         yaw_of(m.pose.orientation))
        self.fix_count += 1

    def _publish_cmd(self, vx, vy, wz):
        t = Twist()
        t.linear.x, t.linear.y, t.angular.z = float(vx), float(vy), float(wz)
        self.cmd_pub.publish(t)

    def _execute(self, goal_handle):
        gp = goal_handle.request.pose.pose
        gx, gy, gyaw = gp.position.x, gp.position.y, yaw_of(gp.orientation)
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.05)
            if goal_handle.is_cancel_requested:
                self._publish_cmd(0, 0, 0)
                goal_handle.canceled()
                return NavigateToPose.Result()
            x, y, yaw = self.est.pose
            ex, ey = gx - x, gy - y
            eyaw = _wrap(gyaw - yaw)
            if math.hypot(ex, ey) < self.pos_tol and abs(eyaw) < self.yaw_tol:
                break
            # 월드 오차를 로봇 로컬로 회전 → 홀로노믹 P제어 (회전 최소화)
            c, s = math.cos(yaw), math.sin(yaw)
            lx, ly = ex * c + ey * s, -ex * s + ey * c
            clamp = lambda v, m: max(-m, min(m, v))
            self._publish_cmd(clamp(self.k_lin * lx, self.max_lin),
                              clamp(self.k_lin * ly, self.max_lin),
                              clamp(self.k_yaw * eyaw, self.max_yaw))
            d = PoseStamped()
            d.header.frame_id = "map"
            d.pose.position.x, d.pose.position.y = x, y
            d.pose.orientation.z = math.sin(yaw / 2)
            d.pose.orientation.w = math.cos(yaw / 2)
            self.diag_pub.publish(d)
        self._publish_cmd(0, 0, 0)
        goal_handle.succeed()
        self.get_logger().info(
            f"도착 ({gx:+.2f},{gy:+.2f}) — 누적 마커 보정 {self.fix_count}회")
        return NavigateToPose.Result()


def main(args=None):
    rclpy.init(args=args)
    node = ArucoNavigatorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
