#!/usr/bin/env python3
"""robot_task_orchestrator (제어): SR-11 상태머신 실행."""

import rclpy
from rclpy.node import Node


class RobotTaskOrchestratorNode(Node):

    def __init__(self):
        super().__init__('robot_task_orchestrator')
        self.get_logger().info('robot_task_orchestrator node started')

        # TODO(SR-11): 상태머신 순차 실행 (로봇 1대당 1인스턴스)
        # TODO: 인식·이동·정렬·리프트 4개 액션서버 순차 호출
        # TODO: 진행 상황을 task_state 토픽으로 발행
        # TODO: safety_monitor의 obstacle_alert 수신 시 긴급정지 처리


def main(args=None):
    rclpy.init(args=args)
    node = RobotTaskOrchestratorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
