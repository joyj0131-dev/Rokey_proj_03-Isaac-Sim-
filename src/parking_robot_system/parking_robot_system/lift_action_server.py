#!/usr/bin/env python3
"""lift_action_server (리프트): 리프트 액션 서버.

[초안] 인터페이스: control_lift(액션 서버).
"""

import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node

from parking_robot_interfaces.action import ControlLift


class LiftActionServerNode(Node):

    def __init__(self):
        super().__init__('lift_action_server')

        self._action_server = ActionServer(
            self, ControlLift, 'control_lift', self._on_control_lift)

        self.get_logger().info('lift_action_server started')

    def _on_control_lift(self, goal_handle):
        # TODO(SR-05, SR-07, SR-08): 리프트 동작 전 안전 확인, 완료 후 지지 상태 확인
        goal_handle.succeed()
        result = ControlLift.Result()
        result.success = False
        result.support_state = ''
        return result


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
