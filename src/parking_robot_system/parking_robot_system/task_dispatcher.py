#!/usr/bin/env python3
"""task_dispatcher: /dispatch/park_in_slot 검증 후 execute_parking_task 전송."""
import time
import uuid

import rclpy
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from parking_robot_interfaces.action import ExecuteParkingTask
from parking_robot_interfaces.srv import GetSlotInfo, ParkInSlot


def decide(info, data_ready):
    """입차(ENTRY) 판정 (accepted, message). 비어있어야 접수, 차 있으면 거부."""
    if not data_ready:
        return (False, "관제 데이터 없음(재시도)")
    if info is None or not info.get("exists", False):
        return (False, "존재하지 않는 구역")
    if info.get("occupied", False):
        return (False, "해당 구역에 차량이 있어 주차 불가")
    return (True, "접수됨")


def decide_exit(info, data_ready):
    """출차(EXIT) 판정 (accepted, message). 차가 있어야 접수, 비어있으면 거부."""
    if not data_ready:
        return (False, "관제 데이터 없음(재시도)")
    if info is None or not info.get("exists", False):
        return (False, "존재하지 않는 구역")
    if not info.get("occupied", False):
        return (False, "해당 구역에 차량이 없어 출차 불가")
    return (True, "접수됨")


class TaskDispatcherNode(Node):
    def __init__(self):
        super().__init__('task_dispatcher')
        self._cbg = ReentrantCallbackGroup()
        self._slot_client = self.create_client(
            GetSlotInfo, 'get_slot_info', callback_group=self._cbg)
        self._exec_client = ActionClient(
            self, ExecuteParkingTask, 'execute_parking_task', callback_group=self._cbg)
        self.create_service(
            ParkInSlot, '/dispatch/park_in_slot', self._on_park, callback_group=self._cbg)
        self.create_service(
            ParkInSlot, '/dispatch/exit_slot', self._on_exit, callback_group=self._cbg)
        self.get_logger().info('task_dispatcher node started')

    def _on_park(self, request, response):
        return self._dispatch(request, response, "ENTRY", decide)

    def _on_exit(self, request, response):
        return self._dispatch(request, response, "EXIT", decide_exit)

    def _dispatch(self, request, response, request_type, decider):
        """입차/출차 공통: get_slot_info 조회 → decider로 판정 → 접수 시 execute_parking_task
        전송(request_type로 ENTRY/EXIT 구분). decider만 다르다(입차=비어야, 출차=차 있어야)."""
        if not self._slot_client.wait_for_service(timeout_sec=3.0):
            response.accepted, response.message = False, "관제 데이터 없음(재시도)"
            return response
        fut = self._slot_client.call_async(GetSlotInfo.Request(slot_id=request.slot_id))
        deadline = time.time() + 5.0
        while not fut.done() and time.time() < deadline:
            time.sleep(0.02)
        res = fut.result() if fut.done() else None
        info = None
        if res is not None and res.exists:
            info = {"exists": True, "occupied": res.occupied,
                    "is_accessible": res.is_accessible, "pose": res.pose}
        data_ready = res is not None and res.data_ready
        accepted, message = decider(info, data_ready)
        response.accepted, response.message = accepted, message
        if accepted:
            if not self._exec_client.wait_for_server(timeout_sec=5.0):
                response.accepted, response.message = False, "실행 서버(orchestrator) 미기동"
                return response
            task_id = str(uuid.uuid4())
            response.task_id = task_id
            goal = ExecuteParkingTask.Goal()
            goal.task_id, goal.request_type, goal.vehicle_id = task_id, request_type, "Pickup"
            goal.slot_id, goal.slot_pose = request.slot_id, res.pose
            goal.leader_robot_id, goal.follower_robot_id = "robot_rear", "robot_front"
            send_fut = self._exec_client.send_goal_async(goal)
            send_fut.add_done_callback(
                lambda f: self.get_logger().info(f"{request_type} goal 전송됨: {request.slot_id}"))
        return response


def main(args=None):
    rclpy.init(args=args)
    node = TaskDispatcherNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
