#!/usr/bin/env python3
"""vehicle_detection_node (인식): 위치·크기 인식."""

import rclpy
from rclpy.node import Node


class VehicleDetectionNode(Node):

    def __init__(self):
        super().__init__('vehicle_detection_node')
        self.get_logger().info('vehicle_detection_node started')

        # TODO(SR-01): 카메라/LiDAR로 차량 위치·방향·크기 인식
        # TODO(SR-04): 차량 하부 진입 위치 계산 지원


def main(args=None):
    rclpy.init(args=args)
    node = VehicleDetectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
