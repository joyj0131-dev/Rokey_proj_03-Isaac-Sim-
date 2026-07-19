#!/usr/bin/env python3
"""lift_action_server (리프트): 리프트 액션 서버."""

import rclpy
from rclpy.node import Node


class LiftActionServerNode(Node):

    def __init__(self):
        super().__init__('lift_action_server')
        self.get_logger().info('lift_action_server started')

        # TODO(SR-05, SR-07, SR-08): 차량 리프트 승강 제어
        # TODO: 리프트 동작 전 안전 확인, 완료 후 지지 상태 확인


def main(args=None):
    rclpy.init(args=args)
    node = LiftActionServerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
