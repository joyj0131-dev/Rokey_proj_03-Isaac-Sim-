#!/usr/bin/env python3
"""user_request_gateway: 사용자 대면 서비스 프록시.
  /park_in_slot → /dispatch/park_in_slot (입차)
  /exit_slot    → /dispatch/exit_slot    (출차)
둘 다 ParkInSlot.srv(slot_id → accepted/task_id/message)를 재사용한다."""
import time

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from parking_robot_interfaces.srv import ParkInSlot


def normalize_slot_id(raw):
    return (raw or "").strip().upper()


class UserRequestGatewayNode(Node):
    def __init__(self):
        super().__init__('user_request_gateway')
        self._cbg = ReentrantCallbackGroup()
        self._park_client = self.create_client(
            ParkInSlot, '/dispatch/park_in_slot', callback_group=self._cbg)
        self._exit_client = self.create_client(
            ParkInSlot, '/dispatch/exit_slot', callback_group=self._cbg)
        self.create_service(
            ParkInSlot, '/park_in_slot', self._on_park, callback_group=self._cbg)
        self.create_service(
            ParkInSlot, '/exit_slot', self._on_exit, callback_group=self._cbg)
        self.get_logger().info('user_request_gateway node started')

    def _on_park(self, request, response):
        return self._proxy(self._park_client, request, response)

    def _on_exit(self, request, response):
        return self._proxy(self._exit_client, request, response)

    def _proxy(self, client, request, response):
        slot_id = normalize_slot_id(request.slot_id)
        if not client.wait_for_service(timeout_sec=3.0):
            response.accepted, response.message = False, "관제(dispatcher) 미기동"
            return response
        fut = client.call_async(ParkInSlot.Request(slot_id=slot_id))
        deadline = time.monotonic() + 10.0
        while not fut.done() and time.monotonic() < deadline:
            time.sleep(0.02)
        res = fut.result() if fut.done() else None
        if res is None:
            response.accepted, response.message = False, "dispatcher 응답 없음"
        else:
            response.accepted, response.task_id, response.message = res.accepted, res.task_id, res.message
        return response


def main(args=None):
    rclpy.init(args=args)
    node = UserRequestGatewayNode()
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
