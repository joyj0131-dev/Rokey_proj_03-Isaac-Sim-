#!/usr/bin/env python3
"""parking_slot_manager (탐색): 빈 구역 탐색."""

import rclpy
from rclpy.node import Node


class ParkingSlotManagerNode(Node):

    def __init__(self):
        super().__init__('parking_slot_manager')
        self.get_logger().info('parking_slot_manager node started')

        # TODO(SR-02): 전체 주차면 사용 여부 관리
        # TODO: 차량 크기·이동거리 기준 최적 슬롯 탐색
        # TODO: find_empty_slot 서비스 제공


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
