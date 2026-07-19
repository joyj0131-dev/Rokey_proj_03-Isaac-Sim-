#!/usr/bin/env python3
"""user_request_gateway (제어): 입고/출차 요청 접수.

[초안] 인터페이스: request_parking_task(서비스), get_task_status(서비스).
실제 요청 처리 로직은 task_dispatcher에 위임하는 얇은 API 계층으로 의도됨.
"""

import uuid

import rclpy
from rclpy.node import Node

from parking_robot_interfaces.srv import GetTaskStatus, RequestParkingTask


class UserRequestGatewayNode(Node):

    def __init__(self):
        super().__init__('user_request_gateway')

        self._task_states = {}  # task_id -> state (TODO: task_dispatcher와 동기화)

        self._request_srv = self.create_service(
            RequestParkingTask, 'request_parking_task', self._on_request_parking_task)
        self._status_srv = self.create_service(
            GetTaskStatus, 'get_task_status', self._on_get_task_status)

        self.get_logger().info('user_request_gateway node started')

    def _on_request_parking_task(self, request, response):
        # TODO(SR-12): 실제 접수 로직은 task_dispatcher에 위임
        task_id = str(uuid.uuid4())
        self._task_states[task_id] = 'WAITING'
        response.accepted = True
        response.task_id = task_id
        response.message = f'{request.request_type} 요청이 접수되었습니다.'
        return response

    def _on_get_task_status(self, request, response):
        # TODO: task_dispatcher/orchestrator로부터 실제 상태 조회
        response.state = self._task_states.get(request.task_id, 'FAILED')
        response.eta_seconds = 0
        response.message = 'TODO: 예상 완료 시간 계산 미구현'
        return response


def main(args=None):
    rclpy.init(args=args)
    node = UserRequestGatewayNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
