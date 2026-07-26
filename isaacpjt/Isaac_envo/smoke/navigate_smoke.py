#!/usr/bin/env python3
"""R3b navigate_to_pose 스모크 테스트 (순수 ROS2, 시스템 rclpy).

parking_v4_runner.py --bridge 와 pose_controller_node(run_pose_controller_node.sh)
가 둘 다 떠 있다고 가정하고, /robot_<id>/navigate_to_pose (nav2_msgs/NavigateToPose)
로 목표 둘을 순차로 보낸다. ros2 CLI 가 이 머신에서 세그폴트하므로(taskB0 기록)
`ros2 action send_goal` 대신 이 스크립트를 쓴다.

  Test A(이동): /robot_<id>/odom 에서 현재 자세를 읽어, 그 자세의 전방(project
    yaw 규약: fwd=(sinψ,cosψ))으로 ~1m 앞을 목표(같은 yaw)로 보낸다.
  Test B(회전): Test A 종료 시점의 자세에서 위치는 그대로, yaw 만 +90° 인
    목표를 보낸다 — R2 가 분석적으로만 검증한 angular.z 부호를 실측으로 닫는다.

목표 쿼터니언은 pose_controller_node.goal_quat_to_yaw_deg 의 역(순수 월드 Y축
회전: x=0,z=0,y=sin(yaw/2),w=cos(yaw/2))으로 만든다 — 서버 문서화 계약을 클라
이언트 쪽에서 독립적으로 재구현해 계약이 실제로 지켜지는지 검증한다.
"""
import math
import sys
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from nav2_msgs.action import NavigateToPose

ROBOT = sys.argv[1] if len(sys.argv) > 1 else "entry_lead"
GOAL_WAIT_SEC = 60.0
DISCOVERY_SEC = 5.0


def _yaw_to_goal_quat(yaw_deg):
    yr = math.radians(yaw_deg)
    return (0.0, math.sin(yr * 0.5), 0.0, math.cos(yr * 0.5))


def _odom_yaw_deg(q):
    # publish_odom 인코딩(x=y=0, z=sin(yaw/2), w=cos(yaw/2))의 역.
    return math.degrees(math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                                    1.0 - 2.0 * (q.y * q.y + q.z * q.z)))


class NavigateSmoke(Node):
    def __init__(self):
        super().__init__("navigate_smoke")
        self.latest = None  # (x, z, yaw_deg)
        self.create_subscription(Odometry, f"/robot_{ROBOT}/odom", self._on_odom, 10)
        self.client = ActionClient(self, NavigateToPose, f"/robot_{ROBOT}/navigate_to_pose")
        self.get_logger().info(f"navigate_smoke 시작: robot={ROBOT}")

    def _on_odom(self, msg):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self.latest = (p.x, p.z, _odom_yaw_deg(q))

    def _on_feedback(self, feedback_msg):
        dist = feedback_msg.feedback.distance_remaining
        self.get_logger().info(f"feedback distance_remaining={dist:.3f}m")

    def wait_for_odom(self, timeout_sec):
        t0 = time.monotonic()
        while time.monotonic() - t0 < timeout_sec:
            rclpy.spin_once(self, timeout_sec=0.2)
            if self.latest is not None:
                return True
        return False

    def send_goal_and_wait(self, target_xzyaw, label):
        tx, tz, tyaw = target_xzyaw
        self.get_logger().info(f"{label}: 서버 대기…")
        if not self.client.wait_for_server(timeout_sec=15.0):
            self.get_logger().error(f"{label}: 액션 서버를 찾지 못함")
            return None

        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = "usd_world"
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(tx)
        goal.pose.pose.position.z = float(tz)
        qx, qy, qz, qw = _yaw_to_goal_quat(tyaw)
        goal.pose.pose.orientation.x = qx
        goal.pose.pose.orientation.y = qy
        goal.pose.pose.orientation.z = qz
        goal.pose.pose.orientation.w = qw

        self.get_logger().info(
            f"{label}: 목표 전송 target=(x={tx:.3f},z={tz:.3f},yaw={tyaw:.2f}deg)")
        send_future = self.client.send_goal_async(goal, feedback_callback=self._on_feedback)
        t0 = time.monotonic()
        while not send_future.done() and time.monotonic() - t0 < 15.0:
            rclpy.spin_once(self, timeout_sec=0.1)
        if not send_future.done():
            self.get_logger().error(f"{label}: 목표 전송이 응답하지 않음")
            return None
        goal_handle = send_future.result()
        if not goal_handle.accepted:
            self.get_logger().error(f"{label}: 목표가 거부됨")
            return None

        result_future = goal_handle.get_result_async()
        t0 = time.monotonic()
        while not result_future.done() and time.monotonic() - t0 < GOAL_WAIT_SEC:
            rclpy.spin_once(self, timeout_sec=0.1)
        if not result_future.done():
            self.get_logger().error(f"{label}: 결과 대기 타임아웃({GOAL_WAIT_SEC}s)")
            return None
        result = result_future.result()
        status = result.status  # GoalStatus: 4=SUCCEEDED, 5=CANCELED, 6=ABORTED
        odom_final = self.latest
        self.get_logger().info(
            f"=== {label}_RESULT status={status} odom_final={odom_final} "
            f"target=(x={tx:.3f},z={tz:.3f},yaw={tyaw:.2f}) ===")
        return status, odom_final


def main():
    rclpy.init()
    node = NavigateSmoke()
    try:
        if not node.wait_for_odom(DISCOVERY_SEC + 10.0):
            node.get_logger().error("odom 을 받지 못함 — 브리지가 --bridge 로 떠 있는지 확인")
            return
        x0, z0, yaw0 = node.latest
        node.get_logger().info(f"시작 odom pose=({x0:.3f},{z0:.3f},{yaw0:.2f}deg)")

        # ---- Test A: 전방으로 ~1m ----
        yr = math.radians(yaw0)
        fwd_x, fwd_z = math.sin(yr), math.cos(yr)
        target_a = (x0 + fwd_x * 1.0, z0 + fwd_z * 1.0, yaw0)
        node.send_goal_and_wait(target_a, "TEST_A")

        time.sleep(1.0)
        rclpy.spin_once(node, timeout_sec=0.2)

        # ---- Test B: 그 자리에서 yaw 만 +90 ----
        xb, zb, yawb = node.latest
        target_b = (xb, zb, yawb + 90.0)
        node.send_goal_and_wait(target_b, "TEST_B")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
