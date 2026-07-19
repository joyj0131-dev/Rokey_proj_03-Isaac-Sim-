#!/usr/bin/env python3
"""task_dispatcher (제어): 가용 로봇 선택, 작업 할당.

[초안] 인터페이스: find_empty_slot(서비스 클라이언트), execute_parking_task(액션 클라이언트).
"""

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from parking_robot_interfaces.action import ExecuteParkingTask
from parking_robot_interfaces.srv import FindEmptySlot


class TaskDispatcherNode(Node):

    def __init__(self):
        super().__init__('task_dispatcher')

        self._robot_states = {}  # robot_id -> state (TODO(SR-09): 다중 로봇 위치·상태 관리)

        self._find_slot_client = self.create_client(FindEmptySlot, 'find_empty_slot')
        self._execute_task_client = ActionClient(
            self, ExecuteParkingTask, 'execute_parking_task')

        self.get_logger().info('task_dispatcher node started')

    def dispatch_task(self, task_id, request_type, vehicle_id):
        """TODO(SR-03): 가용 로봇 선택 및 중복 할당 방지 로직 후 아래 흐름 호출."""
        # TODO(SR-02): find_empty_slot 서비스로 빈 슬롯 확보 후 goal에 반영
        goal = ExecuteParkingTask.Goal()
        goal.task_id = task_id
        goal.request_type = request_type
        goal.vehicle_id = vehicle_id
        self._execute_task_client.wait_for_server()
        return self._execute_task_client.send_goal_async(goal)


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
