#!/usr/bin/env python3
"""navigate_action_server (이동): Nav2 액션 서버."""

import rclpy
from rclpy.node import Node


class NavigateActionServerNode(Node):

    def __init__(self):
        super().__init__('navigate_action_server')
        self.get_logger().info('navigate_action_server started')

        # TODO(SR-06): Nav2 NavigateToPose 래핑
        # TODO: 목표 위치까지 경로계획 및 이동
        # TODO: 장애물 시 정지 또는 재경로 계산


def main(args=None):
    rclpy.init(args=args)
    node = NavigateActionServerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
