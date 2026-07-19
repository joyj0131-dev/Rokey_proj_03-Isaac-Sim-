#!/usr/bin/env python3
"""user_request_gateway (제어): 입고/출차 요청 접수."""

import rclpy
from rclpy.node import Node


class UserRequestGatewayNode(Node):

    def __init__(self):
        super().__init__('user_request_gateway')
        self.get_logger().info('user_request_gateway node started')

        # TODO(SR-12): 입고/출차 요청 접수 서비스 서버
        # TODO: 요청 상태를 대기중/처리중/완료/실패로 구분해 관리
        # TODO: 예상 완료 시간 조회 응답 서비스
        # TODO: 얇은 API 계층으로 두고 실제 로직은 task_dispatcher에 위임


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
