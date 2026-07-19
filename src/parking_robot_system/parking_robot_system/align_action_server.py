#!/usr/bin/env python3
"""align_action_server (정렬): 정렬 액션 서버.

[초안] 인터페이스: align_vehicle(액션 서버).
"""

import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node

from parking_robot_interfaces.action import AlignVehicle


class AlignActionServerNode(Node):

    def __init__(self):
        super().__init__('align_action_server')

        self._action_server = ActionServer(
            self, AlignVehicle, 'align_vehicle', self._on_align_vehicle)

        self.get_logger().info('align_action_server started')

    def _on_align_vehicle(self, goal_handle):
        # TODO(SR-04): 차량 하부 중심 위치로 정밀 정렬, 오차 초과 시 재정렬 수행
        goal_handle.succeed()
        result = AlignVehicle.Result()
        result.success = False
        result.final_error = 0.0
        return result


def main(args=None):
    rclpy.init(args=args)
    node = AlignActionServerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
