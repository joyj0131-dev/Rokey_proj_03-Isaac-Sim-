#!/usr/bin/env python3
"""navigate_action_server (이동): Nav2 액션 서버.

[초안] 인터페이스: navigate_to_pose(액션 서버, nav2_msgs/action/NavigateToPose 재사용).
"""

import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from nav2_msgs.action import NavigateToPose


class NavigateActionServerNode(Node):

    def __init__(self):
        super().__init__('navigate_action_server')

        self._action_server = ActionServer(
            self, NavigateToPose, 'navigate_to_pose', self._on_navigate_to_pose)

        self.get_logger().info('navigate_action_server started')

    def _on_navigate_to_pose(self, goal_handle):
        # TODO(SR-06): 실제 Nav2 NavigateToPose 클라이언트로 위임, 장애물 시 재경로 계산
        goal_handle.succeed()
        return NavigateToPose.Result()


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
