#!/usr/bin/env python3
"""Isaac의 USD Y-up PointCloud2 두 개를 ROS map Z-up cloud 하나로 합친다."""

import numpy as np
import rclpy
from rclpy._rclpy_pybind11 import RCLError
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header


RAW_TOPICS = (
    "/parking/lidar/ceiling_01/points_usd",
    "/parking/lidar/ceiling_02/points_usd",
)
WORLD_TOPIC = "/parking/lidar/points_world"


class WorldCloudRelay(Node):
    def __init__(self):
        super().__init__("parking_lidar_world_relay")
        self._latest = {topic: np.empty((0, 3), dtype=np.float32)
                        for topic in RAW_TOPICS}
        output_qos = QoSProfile(
            depth=5,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self._publisher = self.create_publisher(PointCloud2, WORLD_TOPIC, output_qos)
        for topic in RAW_TOPICS:
            self.create_subscription(
                PointCloud2,
                topic,
                lambda message, source=topic: self._receive(message, source),
                qos_profile_sensor_data,
            )
        self.create_timer(0.05, self._publish)  # RViz에는 20 Hz면 충분하다.
        self.get_logger().info(
            f"USD Y-up → ROS Z-up 변환: {', '.join(RAW_TOPICS)} → {WORLD_TOPIC}")

    def _receive(self, message: PointCloud2, source: str) -> None:
        cloud = point_cloud2.read_points(
            message, field_names=("x", "y", "z"), skip_nans=True)
        if cloud.size == 0:
            self._latest[source] = np.empty((0, 3), dtype=np.float32)
            return
        usd = np.column_stack((cloud["x"], cloud["y"], cloud["z"]))
        # 프로젝트 공통 좌표 규약: ROS x=USD x, ROS y=-USD z, ROS z=USD y.
        self._latest[source] = np.ascontiguousarray(
            np.column_stack((usd[:, 0], -usd[:, 2], usd[:, 1])),
            dtype=np.float32,
        )

    def _publish(self) -> None:
        available = [points for points in self._latest.values() if len(points)]
        if not available:
            return
        points = np.concatenate(available, axis=0)
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = "map"
        self._publisher.publish(point_cloud2.create_cloud_xyz32(header, points))


def main() -> None:
    rclpy.init()
    node = WorldCloudRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except rclpy.executors.ExternalShutdownException:
        pass
    except RCLError:
        if rclpy.ok():
            raise
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
