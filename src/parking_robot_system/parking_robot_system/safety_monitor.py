#!/usr/bin/env python3
"""safety_monitor (안전): 장애물 감지, 정지."""

import rclpy
from rclpy.node import Node


class SafetyMonitorNode(Node):

    def __init__(self):
        super().__init__('safety_monitor')
        self.get_logger().info('safety_monitor node started')

        # TODO(SR-10): 사람/차량/장애물 감지 시 즉시 정지
        # TODO: obstacle_alert 토픽으로 orchestrator에 긴급정지 신호 발행
        # TODO: 장애물 해소 시 작업 재개 신호 전달


def main(args=None):
    rclpy.init(args=args)
    node = SafetyMonitorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
