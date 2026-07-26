#!/usr/bin/env python3
"""R3c 마커융합 자세소스 navigate_to_pose 스모크 테스트 (순수 ROS2, 시스템 rclpy).

R3b 의 `navigate_smoke.py` 와 같은 절차(Test A: 전방 N미터, Test B: 그 자리에서
+90° 회전)지만 두 가지가 다르다:

  1. 현재 자세를 `/robot_<id>/odom`(Odometry) 이 아니라 `marker_localizer_node`
     가 `fuse:=true` 로 내는 `/robot_<id>/pose`(PoseStamped, 융합 자세)에서
     읽는다 — `pose_controller_node` 가 실제로 먹는 것과 같은 소스로 목표를
     계산해야 검증이 의미 있다.
  2. Test A 이동거리를 CLI 인자로 받는다(기본 1.0m, R3b 와 동일). 마커의 실측
     신뢰 검출창(taskR3c-report.md §5.1: 정적 배치 기준 대략 [1.1, 2.0]m, 이
     마커/카메라 조합 한정)을 벗어나지 않도록 줄여서 쓸 수 있다 — 그 외
     절차/로직은 R3b 와 동일하게 유지해 직접 비교 가능하다.

`parking_v4_runner.py --bridge`(전방캠 부착, `--bridge-cameras=`),
`marker_localizer_node`(`fuse:=true`), `pose_controller_node`
(`pose_msg_type:=posestamped`, `pose_topic:=/robot_<id>/pose`) 가 모두 떠 있다고
가정한다. `ros2 CLI` 가 이 머신에서 세그폴트하므로(taskB0 기록) 이 스크립트를
쓴다.

목표 쿼터니언은 `pose_controller_node.goal_quat_to_yaw_deg` 의 역(순수 월드 Y축
회전)으로 만든다 — R3b `navigate_smoke.py` 와 동일 관례.
"""
import math
import sys
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose

ROBOT = sys.argv[1] if len(sys.argv) > 1 else "entry_lead"
DIST_M = float(sys.argv[2]) if len(sys.argv) > 2 else 1.0
GOAL_WAIT_SEC = 60.0
DISCOVERY_SEC = 5.0


def _yaw_to_goal_quat(yaw_deg):
    yr = math.radians(yaw_deg)
    return (0.0, math.sin(yr * 0.5), 0.0, math.cos(yr * 0.5))


def _pose_yaw_deg(q):
    # marker_localizer_node usd 프레임 인코딩(R3c 수정, taskR3c-report.md §3):
    # /odom 과 동일한 z축 회전 슬롯 인코딩(x=y=0, z=sin(yaw/2), w=cos(yaw/2)).
    return math.degrees(math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                                    1.0 - 2.0 * (q.y * q.y + q.z * q.z)))


class NavigateSmokeFused(Node):
    def __init__(self):
        super().__init__("navigate_smoke_fused")
        self.latest = None  # (x, z, yaw_deg)
        self.create_subscription(PoseStamped, f"/robot_{ROBOT}/pose", self._on_pose, 10)
        self.client = ActionClient(self, NavigateToPose, f"/robot_{ROBOT}/navigate_to_pose")
        self.get_logger().info(f"navigate_smoke_fused 시작: robot={ROBOT} dist={DIST_M}m")

    def _on_pose(self, msg):
        p = msg.pose.position
        q = msg.pose.orientation
        self.latest = (p.x, p.z, _pose_yaw_deg(q))

    def _on_feedback(self, feedback_msg):
        dist = feedback_msg.feedback.distance_remaining
        self.get_logger().info(f"feedback distance_remaining={dist:.3f}m")

    def wait_for_pose(self, timeout_sec):
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
        pose_final = self.latest
        self.get_logger().info(
            f"=== {label}_RESULT status={status} pose_final={pose_final} "
            f"target=(x={tx:.3f},z={tz:.3f},yaw={tyaw:.2f}) ===")
        return status, pose_final


def main():
    rclpy.init()
    node = NavigateSmokeFused()
    try:
        if not node.wait_for_pose(DISCOVERY_SEC + 10.0):
            node.get_logger().error(
                "fused pose 를 받지 못함 — marker_localizer_node 가 fuse:=true 로 떠 있고 "
                "마커가 보이는 위치인지 확인(taskR3c-report.md §5.1)")
            return
        x0, z0, yaw0 = node.latest
        node.get_logger().info(f"시작 fused pose=({x0:.3f},{z0:.3f},{yaw0:.2f}deg)")

        # ---- Test A: 전방으로 ~DIST_M ----
        yr = math.radians(yaw0)
        fwd_x, fwd_z = math.sin(yr), math.cos(yr)
        target_a = (x0 + fwd_x * DIST_M, z0 + fwd_z * DIST_M, yaw0)
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
