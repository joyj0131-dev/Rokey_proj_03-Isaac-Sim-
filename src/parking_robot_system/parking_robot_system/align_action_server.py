#!/usr/bin/env python3
"""align_action_server (정렬): 정렬 액션 서버."""

import rclpy
from rclpy.node import Node


class AlignActionServerNode(Node):

    def __init__(self):
        super().__init__('align_action_server')
        self.get_logger().info('align_action_server started')

        # TODO(SR-04): 차량 하부 중심 위치로 정밀 정렬
        # TODO: 정렬 오차가 허용 범위 초과 시 재정렬 수행


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
