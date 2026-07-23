#!/usr/bin/env python3
"""safety_monitor (안전): 장애물 감지, 정지.

[초안] 인터페이스: obstacle_alert(발행 토픽).
"""

import rclpy
from rclpy.node import Node

from parking_robot_interfaces.msg import ObstacleAlert


class SafetyMonitorNode(Node):

    def __init__(self):
        super().__init__('safety_monitor')

        self._alert_pub = self.create_publisher(ObstacleAlert, 'obstacle_alert', 10)

        # TODO(SR-10): 센서 구독 및 실제 감지 로직으로 아래 타이머 대체
        self._timer = self.create_timer(1.0, self._check_obstacles)

        self.get_logger().info('safety_monitor node started')

    def _check_obstacles(self):
        # TODO: 사람/차량/장애물 감지 로직. 지금은 항상 안전 상태로 발행.
        msg = ObstacleAlert()
        msg.obstacle_detected = False
        msg.description = ''
        self._alert_pub.publish(msg)


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
