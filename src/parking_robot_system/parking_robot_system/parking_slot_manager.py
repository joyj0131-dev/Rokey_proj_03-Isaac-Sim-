#!/usr/bin/env python3
"""parking_slot_manager (탐색): 빈 구역 탐색.

[초안] 인터페이스: find_empty_slot(서비스 서버).
"""

import rclpy
from rclpy.node import Node

from parking_robot_interfaces.srv import FindEmptySlot


class ParkingSlotManagerNode(Node):

    def __init__(self):
        super().__init__('parking_slot_manager')

        self._slots = {}  # slot_id -> occupied(bool) (TODO(SR-02): 전체 주차면 관리)

        self._srv = self.create_service(
            FindEmptySlot, 'find_empty_slot', self._on_find_empty_slot)

        self.get_logger().info('parking_slot_manager node started')

    def _on_find_empty_slot(self, request, response):
        # TODO: 차량 크기·이동거리 기준 최적 슬롯 탐색
        response.success = False
        response.slot_id = ''
        return response


def main(args=None):
    rclpy.init(args=args)
    node = ParkingSlotManagerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
