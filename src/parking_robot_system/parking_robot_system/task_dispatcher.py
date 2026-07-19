#!/usr/bin/env python3
"""task_dispatcher (제어): 가용 로봇 선택, 작업 할당."""

import rclpy
from rclpy.node import Node


class TaskDispatcherNode(Node):

    def __init__(self):
        super().__init__('task_dispatcher')
        self.get_logger().info('task_dispatcher node started')

        # TODO(SR-03): 가용 로봇 선택, 작업 할당
        # TODO: 동일 로봇 중복 할당 방지
        # TODO(SR-09): 다중 로봇 위치·상태 관리
        # TODO: 경로 충돌 예상 시 우선순위 조정


def main(args=None):
    rclpy.init(args=args)
    node = TaskDispatcherNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
