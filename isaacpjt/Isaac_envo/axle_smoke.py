#!/usr/bin/env python3
"""R4 축 감지 스모크 (순수 ROS2, 시스템 rclpy).

parking_v4_runner.py --bridge --bridge-depth-pose=<ROBOT> (트럭 진입선 배치)
가 떠 있고, axle_detector_node(run_axle_detector_node.sh, robot_id=<ROBOT>,
pose_topic=/robot_<ROBOT>/odom)가 떠 있다고 가정한다. /robot_<ROBOT>/cmd_vel
로 전진(body-forward, 진입선 배치 후 world -X 방향과 같다 — --probe=DEPTH
와 동일 관례)을 계속 명령하면서 /robot_<ROBOT>/axle_center,
/robot_<ROBOT>/axle_index, /robot_<ROBOT>/odom 을 구독해 검출된 축 개수·
좌표를 리포트한다. 두 축(트로프 2개)이 검출되거나 타임아웃/이동한계에
도달하면 정지하고 결과를 찍는다.

**1차 시도에서 실측으로 발견**: 순수 linear.x 만 명령하면(피드백 보정 없음)
로봇이 물리적으로 요(yaw) 드리프트한다(실측: -90.0deg -> -81.3deg 까지, z 도
7.075 -> 7.22 로 어긋남) — 러너 인프로세스 --probe=DEPTH(taskC2fix-report.md
§2.2)가 "vy strafe 중앙유지"를 필수로 넣은 것과 같은 이유다. 이 스모크는
축감지 노드 자체(R4 산출물)를 시험하는 게 목적이지 안무 재현이 목적이
아니므로(R5 몫), 딱 필요한 최소한의 자세 유지만 재현한다:
  - **yaw-hold**: /odom yaw 를 목표(-90°, 텔레포트 그대로)로 잡는 간단한 P
    제어(cmd_vel.angular.z).
  - **lateral centering**: --probe=DEPTH 와 동일한 좌우 ROI-min 차이 기반 vy
    비례제어(LAT_KP/LAT_VY_MAX/LAT_DEADBAND 값도 동일 — taskC2fix-report.md
    §2.2 그대로 재사용, 짐작 아님).
이 두 보정이 없으면 로봇이 트럭 하부구조에 스치며 두 트로프가 하나로 뭉개진다
(실측: enter=-6.42 exit=-10.24 하나짜리 트로프 — 1차 시도 기록, taskR4-report.md
§실패 재현 참고).

ros2 CLI 가 이 머신에서 세그폴트하므로(taskB0 기록) 이 스크립트로 대신한다.
실행: run_axle_smoke.sh <ROBOT> [--timeout=SEC] [--speed=MPS] [--x-end=X]
"""
import math
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist, PointStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
from std_msgs.msg import Int32

sys.path.insert(0, "/home/rokey/p3/cobot_ws/src/parkbot_motion")
from parkbot_motion.axle_detector_node import (  # noqa: E402
    DEPTH_ROI_FRAC, combine_side_depths, depth_image_to_array)
from parkbot_motion.depth_stop_detector import roi_min_depth  # noqa: E402
from parkbot_motion.pose_controller_node import odom_quat_to_yaw_deg  # noqa: E402

# --probe=DEPTH 실측값 그대로(taskC2fix-report.md §2.2) — 짐작 아님.
LAT_KP = 1.2
LAT_VY_MAX = 0.15
LAT_DEADBAND = 0.01
YAW_KP = 0.03   # deg 오차 -> rad/s, 보수적(과보정 방지) — 이 스모크 전용, 실측 튜닝 안 함
YAW_WZ_MAX = 0.3


class AxleSmoke(Node):
    def __init__(self, robot, speed, x_end, timeout, target_yaw_deg):
        super().__init__("axle_smoke")
        self.robot = robot
        self.speed = speed
        self.x_end = x_end
        self.timeout = timeout
        self.target_yaw_deg = target_yaw_deg
        self.cmd_pub = self.create_publisher(Twist, f"/robot_{robot}/cmd_vel", 10)
        self.create_subscription(Odometry, f"/robot_{robot}/odom", self._on_odom, 10)
        self.create_subscription(PointStamped, f"/robot_{robot}/axle_center",
                                  self._on_center, 10)
        self.create_subscription(Int32, f"/robot_{robot}/axle_index", self._on_index, 10)
        self.create_subscription(Image, f"/robot_{robot}/left/depth", self._make_depth_cb("left"),
                                  qos_profile_sensor_data)
        self.create_subscription(Image, f"/robot_{robot}/right/depth", self._make_depth_cb("right"),
                                  qos_profile_sensor_data)
        self.x = None
        self.yaw_deg = None
        self.odom_msgs = 0
        self.left_v = None
        self.right_v = None
        self.centers = []
        self.indices = []
        self.get_logger().info(
            f"axle_smoke 시작: robot={robot} speed={speed} x_end={x_end} "
            f"timeout={timeout}s target_yaw={target_yaw_deg}. "
            f"/robot_{robot}/odom, axle_center, axle_index 대기…")

    def _on_odom(self, msg):
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        self.x = p.x
        self.yaw_deg = odom_quat_to_yaw_deg(q.x, q.y, q.z, q.w)
        self.odom_msgs += 1

    def _on_center(self, msg):
        self.centers.append(msg.point.x)
        self.get_logger().info(f"AXLE_CENTER #{len(self.centers)} x={msg.point.x:.4f}")

    def _on_index(self, msg):
        self.indices.append(int(msg.data))
        self.get_logger().info(f"AXLE_INDEX #{len(self.indices)} index={msg.data}")

    def _make_depth_cb(self, side):
        def cb(msg):
            grid = depth_image_to_array(msg.height, msg.width, msg.step, msg.data, msg.encoding)
            v = roi_min_depth(grid, roi_frac=DEPTH_ROI_FRAC)
            val = v if math.isfinite(v) else None
            if side == "left":
                self.left_v = val
            else:
                self.right_v = val
        return cb

    def control_twist(self):
        vy = 0.0
        if self.left_v is not None and self.right_v is not None:
            err = self.left_v - self.right_v
            if abs(err) >= LAT_DEADBAND:
                vy = max(-LAT_VY_MAX, min(LAT_VY_MAX, LAT_KP * err))
        wz = 0.0
        if self.yaw_deg is not None:
            dyaw = (self.target_yaw_deg - self.yaw_deg + 180.0) % 360.0 - 180.0
            wz = max(-YAW_WZ_MAX, min(YAW_WZ_MAX, YAW_KP * dyaw))
        return vy, wz


def main():
    robot = "entry_follow"
    speed = 0.4
    x_end = -11.8
    timeout = 90.0
    target_yaw_deg = -90.0
    for a in sys.argv[1:]:
        if a.startswith("--robot="):
            robot = a.split("=", 1)[1]
        elif a.startswith("--speed="):
            speed = float(a.split("=", 1)[1])
        elif a.startswith("--x-end="):
            x_end = float(a.split("=", 1)[1])
        elif a.startswith("--timeout="):
            timeout = float(a.split("=", 1)[1])
        elif a.startswith("--target-yaw="):
            target_yaw_deg = float(a.split("=", 1)[1])

    rclpy.init()
    node = AxleSmoke(robot, speed, x_end, timeout, target_yaw_deg)
    try:
        t0 = time.monotonic()
        while time.monotonic() - t0 < 5.0:
            rclpy.spin_once(node, timeout_sec=0.2)
        node.get_logger().info(
            f"discovery 창 종료: odom {node.odom_msgs}개 수신, x={node.x} yaw={node.yaw_deg}")

        t_start = time.monotonic()
        stop_reason = "timeout"
        last_log = 0.0
        while time.monotonic() - t_start < timeout:
            vy, wz = node.control_twist()
            tw = Twist()
            tw.linear.x = speed
            tw.linear.y = vy
            tw.angular.z = wz
            node.cmd_pub.publish(tw)
            rclpy.spin_once(node, timeout_sec=0.05)
            now = time.monotonic()
            if now - last_log > 3.0:
                last_log = now
                node.get_logger().info(
                    f"driving x={node.x} yaw={node.yaw_deg} vy={vy:.3f} wz={wz:.3f} "
                    f"left={node.left_v} right={node.right_v} "
                    f"n_centers={len(node.centers)}")
            if len(node.centers) >= 2:
                stop_reason = "two_troughs"
                break
            if node.x is not None and node.x <= x_end:
                stop_reason = "x_end"
                break

        t_stop = time.monotonic()
        while time.monotonic() - t_stop < 2.0:
            tw = Twist()
            node.cmd_pub.publish(tw)
            rclpy.spin_once(node, timeout_sec=0.05)

        node.get_logger().info(
            f"=== AXLE_SMOKE_RESULT stop_reason={stop_reason} odom_msgs={node.odom_msgs} "
            f"final_x={node.x} final_yaw={node.yaw_deg} n_centers={len(node.centers)} "
            f"centers={node.centers} indices={node.indices} ===")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
