#!/usr/bin/env python3
"""task_dispatcher: /dispatch/park_in_slot 검증 후 execute_parking_task 전송."""
import uuid

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from parking_robot_interfaces.action import ExecuteParkingTask
from parking_robot_interfaces.srv import GetSlotInfo, ParkInSlot


def decide(info, data_ready):
    """(accepted, message). info: get_slot_info 결과 dict 또는 None."""
    if not data_ready:
        return (False, "관제 데이터 없음(재시도)")
    if info is None or not info.get("exists", False):
        return (False, "존재하지 않는 구역")
    if info.get("occupied", False):
        return (False, "해당 구역에 차량이 있어 주차 불가")
    return (True, "접수됨")


class TaskDispatcherNode(Node):
    def __init__(self):
        super().__init__('task_dispatcher')
        self._slot_client = self.create_client(GetSlotInfo, 'get_slot_info')
        self._exec_client = ActionClient(self, ExecuteParkingTask, 'execute_parking_task')
        self.create_service(ParkInSlot, '/dispatch/park_in_slot', self._on_dispatch)
        self.get_logger().info('task_dispatcher node started')

    def _on_dispatch(self, request, response):
        if not self._slot_client.wait_for_service(timeout_sec=3.0):
            response.accepted, response.message = False, "관제 데이터 없음(재시도)"
            return response
        fut = self._slot_client.call_async(GetSlotInfo.Request(slot_id=request.slot_id))
        rclpy.spin_until_future_complete(self, fut, timeout_sec=5.0)
        res = fut.result()
        info = None
        if res is not None and res.exists:
            info = {"exists": True, "occupied": res.occupied,
                    "is_accessible": res.is_accessible, "pose": res.pose}
        data_ready = res is not None and res.data_ready
        accepted, message = decide(info, data_ready)
        response.accepted, response.message = accepted, message
        if accepted:
            task_id = str(uuid.uuid4())
            response.task_id = task_id
            goal = ExecuteParkingTask.Goal()
            goal.task_id, goal.request_type, goal.vehicle_id = task_id, "ENTRY", "Pickup"
            goal.slot_id, goal.slot_pose = request.slot_id, res.pose
            goal.leader_robot_id, goal.follower_robot_id = "robot_rear", "robot_front"
            self._exec_client.wait_for_server()
            self._exec_client.send_goal_async(goal)
        return response


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
