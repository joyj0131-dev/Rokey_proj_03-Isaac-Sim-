#!/usr/bin/env python3
"""vehicle_detection_node (인식): 위치·크기 인식.

[초안] 인터페이스: detect_vehicle(액션 서버).
"""

import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node

from parking_robot_interfaces.action import DetectVehicle


class VehicleDetectionNode(Node):

    def __init__(self):
        super().__init__('vehicle_detection_node')

        self._action_server = ActionServer(
            self, DetectVehicle, 'detect_vehicle', self._on_detect_vehicle)

        self.get_logger().info('vehicle_detection_node started')

    def _on_detect_vehicle(self, goal_handle):
        # TODO(SR-01, SR-04): 카메라/LiDAR로 차량 위치·방향·크기 인식
        goal_handle.succeed()
        result = DetectVehicle.Result()
        result.success = False
        return result


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
