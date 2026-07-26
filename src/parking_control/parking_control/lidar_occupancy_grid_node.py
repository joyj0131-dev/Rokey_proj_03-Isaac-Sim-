#!/usr/bin/env python3
"""천장 LiDAR 월드 PointCloud2를 RViz용 2D OccupancyGrid로 변환한다."""

import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2

from parking_control.core.occupancy_grid import GridSpec, rasterize_points


class LidarOccupancyGridNode(Node):

    def __init__(self):
        super().__init__("parking_lidar_occupancy_grid")
        self.declare_parameter("input_topic", "/parking/lidar/points_world")
        self.declare_parameter("output_topic", "/parking/occupancy_grid")
        self.declare_parameter("resolution", 0.1)
        self.declare_parameter("origin_x", -24.0)
        self.declare_parameter("origin_y", -12.0)
        self.declare_parameter("width", 400)
        self.declare_parameter("height", 240)
        self.declare_parameter("min_height", 0.15)
        self.declare_parameter("max_height", 3.0)
        self.declare_parameter("min_points_per_cell", 2)
        self.declare_parameter("inflate_cells", 1)
        self.declare_parameter("publish_hz", 5.0)

        value = lambda name: self.get_parameter(name).value
        self._spec = GridSpec(
            resolution=float(value("resolution")),
            origin_x=float(value("origin_x")),
            origin_y=float(value("origin_y")),
            width=int(value("width")),
            height=int(value("height")),
        )
        self._min_height = float(value("min_height"))
        self._max_height = float(value("max_height"))
        self._min_points = int(value("min_points_per_cell"))
        self._inflate_cells = int(value("inflate_cells"))
        self._latest_grid = None

        output_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._publisher = self.create_publisher(
            OccupancyGrid, str(value("output_topic")), output_qos)
        self.create_subscription(
            PointCloud2,
            str(value("input_topic")),
            self._on_cloud,
            qos_profile_sensor_data,
        )
        publish_hz = float(value("publish_hz"))
        if publish_hz <= 0:
            raise ValueError("publish_hz는 0보다 커야 합니다")
        self.create_timer(1.0 / publish_hz, self._publish)
        self.get_logger().info(
            f"{value('input_topic')} → {value('output_topic')} "
            f"({self._spec.width}x{self._spec.height}, "
            f"{self._spec.resolution:.2f} m/cell, frame=map)")

    def _on_cloud(self, message):
        cloud = point_cloud2.read_points(
            message, field_names=("x", "y", "z"), skip_nans=True)
        if cloud.size == 0:
            return
        points = np.column_stack((cloud["x"], cloud["y"], cloud["z"]))
        self._latest_grid = rasterize_points(
            points,
            self._spec,
            min_height=self._min_height,
            max_height=self._max_height,
            min_points_per_cell=self._min_points,
            inflate_cells=self._inflate_cells,
        )

    def _publish(self):
        if self._latest_grid is None:
            return
        message = OccupancyGrid()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = "map"
        message.info.map_load_time = message.header.stamp
        message.info.resolution = self._spec.resolution
        message.info.width = self._spec.width
        message.info.height = self._spec.height
        message.info.origin.position.x = self._spec.origin_x
        message.info.origin.position.y = self._spec.origin_y
        message.info.origin.orientation.w = 1.0
        message.data = self._latest_grid.reshape(-1).tolist()
        self._publisher.publish(message)


def main(args=None):
    rclpy.init(args=args)
    node = LidarOccupancyGridNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
