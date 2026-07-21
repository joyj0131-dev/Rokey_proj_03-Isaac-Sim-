#!/usr/bin/env python3
"""robot_task_orchestrator (제어): SR-11 상태머신 실행.

[초안] 인터페이스:
- execute_parking_task(액션 서버, task_dispatcher가 호출)
- detect_vehicle/navigate_to_pose/align_vehicle/control_lift(액션 클라이언트, 4개 액션서버 순차 호출)
- task_state(발행 토픽), obstacle_alert(구독 토픽)
"""

import rclpy
from rclpy.action import ActionClient, ActionServer
from rclpy.node import Node
from nav2_msgs.action import NavigateToPose

from parking_robot_interfaces.action import (
    AlignVehicle, ControlLift, DetectVehicle, ExecuteParkingTask,
)
from parking_robot_interfaces.msg import ObstacleAlert, TaskState


class RobotTaskOrchestratorNode(Node):

    def __init__(self):
        super().__init__('robot_task_orchestrator')

        self._task_state_pub = self.create_publisher(TaskState, 'task_state', 10)
        self._obstacle_alert_sub = self.create_subscription(
            ObstacleAlert, 'obstacle_alert', self._on_obstacle_alert, 10)

        self._execute_task_server = ActionServer(
            self, ExecuteParkingTask, 'execute_parking_task', self._on_execute_parking_task)

        self._detect_client = ActionClient(self, DetectVehicle, 'detect_vehicle')
        self._navigate_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self._align_client = ActionClient(self, AlignVehicle, 'align_vehicle')
        self._lift_client = ActionClient(self, ControlLift, 'control_lift')

        self._emergency_stop = False

        self.get_logger().info('robot_task_orchestrator node started')

    def _on_obstacle_alert(self, msg):
        # TODO(SR-10): 긴급정지 상태 반영, 장애물 해소 시 작업 재개 신호 처리
        self._emergency_stop = msg.obstacle_detected

    def _on_execute_parking_task(self, goal_handle):
        # TODO(SR-11): 인식 -> 이동 -> 정렬 -> 리프트 순차 상태머신 실행 (로봇 1대당 1인스턴스)
        # TODO: 각 단계 진행 시 task_state 발행, obstacle_alert 수신 시 중단
        goal_handle.succeed()
        result = ExecuteParkingTask.Result()
        result.success = False
        result.message = 'TODO: 상태머신 미구현'
        return result


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
