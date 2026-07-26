#!/usr/bin/env python3
"""R4 측면 뎁스 토픽 스모크 (순수 ROS2, 시스템 rclpy).

parking_v4_runner.py --bridge --bridge-cameras=<ROBOT> 가 떠 있다고 가정하고,
/robot_<ROBOT>/left/depth, /robot_<ROBOT>/right/depth (sensor_msgs/Image,
encoding 32FC1) 를 구독해 메시지 크기·encoding·ROI 최소값 몇 개를 찍는다.
depth_stop_detector.roi_min_depth 를 그대로 써서(별도 파싱 로직 중복 없음)
축 감지 노드가 실제로 보게 될 값과 동일한 값을 확인한다.

실행: run_depth_smoke.sh <ROBOT> [--count=N]
"""
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[2] / "src" / "parkbot_motion"))

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

from parkbot_motion.axle_detector_node import DEPTH_ROI_FRAC, depth_image_to_array
from parkbot_motion.depth_stop_detector import roi_min_depth


class DepthSmoke(Node):
    def __init__(self, robot, count):
        super().__init__("depth_smoke")
        self.robot = robot
        self.count = count
        self.n_left = 0
        self.n_right = 0
        self.create_subscription(Image, f"/robot_{robot}/left/depth", self._make_cb("left"),
                                  qos_profile_sensor_data)
        self.create_subscription(Image, f"/robot_{robot}/right/depth", self._make_cb("right"),
                                  qos_profile_sensor_data)
        self.get_logger().info(
            f"depth_smoke 시작: robot={robot} count={count}. "
            f"/robot_{robot}/{{left,right}}/depth 대기…")

    def _make_cb(self, side):
        def cb(msg):
            n = self.n_left if side == "left" else self.n_right
            n += 1
            if side == "left":
                self.n_left = n
            else:
                self.n_right = n
            if n <= self.count:
                grid = depth_image_to_array(msg.height, msg.width, msg.step, msg.data,
                                             encoding=msg.encoding)
                roi_min = roi_min_depth(grid, roi_frac=DEPTH_ROI_FRAC)
                self.get_logger().info(
                    f"DEPTH_{side.upper()} #{n} encoding={msg.encoding} "
                    f"size=({msg.height}x{msg.width}) step={msg.step} "
                    f"frame_min={float(grid.min()):.4f} frame_max={float(grid.max()):.4f} "
                    f"roi_min={roi_min:.4f}")
        return cb


def main():
    robot = "entry_lead"
    count = 5
    for a in sys.argv[1:]:
        if a.startswith("--robot="):
            robot = a.split("=", 1)[1]
        elif a.startswith("--count="):
            count = int(a.split("=", 1)[1])

    rclpy.init()
    node = DepthSmoke(robot, count)
    try:
        import time
        t0 = time.monotonic()
        while time.monotonic() - t0 < 20.0 and (node.n_left < count or node.n_right < count):
            rclpy.spin_once(node, timeout_sec=0.2)
        node.get_logger().info(
            f"=== DEPTH_SMOKE_RESULT n_left={node.n_left} n_right={node.n_right} ===")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
