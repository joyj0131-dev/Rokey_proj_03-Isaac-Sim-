#!/usr/bin/env python3
"""parking_slot_manager: /parking_slots 구독 캐시 + get_slot_info 서비스."""
import json

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Pose
import math

from parking_robot_interfaces.srv import GetSlotInfo


class SlotCache:
    def __init__(self):
        self._slots = {}

    @property
    def ready(self):
        return len(self._slots) > 0

    def update_from_json(self, s):
        arr = json.loads(s)
        if not isinstance(arr, list):
            return
        new_slots = {}
        for d in arr:
            if not isinstance(d, dict) or "slot_id" not in d:
                return
            new_slots[d["slot_id"]] = d
        self._slots = new_slots

    def query(self, slot_id):
        return self._slots.get(slot_id)


def _yaw_deg_to_pose(x, y, yaw_deg):
    p = Pose()
    p.position.x, p.position.y = float(x), float(y)
    half = math.radians(yaw_deg) / 2.0
    p.orientation.z, p.orientation.w = math.sin(half), math.cos(half)
    return p


class ParkingSlotManagerNode(Node):
    def __init__(self):
        super().__init__('parking_slot_manager')
        self._cache = SlotCache()
        self.create_subscription(String, '/parking_slots', self._on_slots, 10)
        self.create_service(GetSlotInfo, 'get_slot_info', self._on_get_slot_info)
        self.get_logger().info('parking_slot_manager node started')

    def _on_slots(self, msg):
        try:
            self._cache.update_from_json(msg.data)
        except (ValueError, KeyError, TypeError) as e:
            self.get_logger().warn(f'/parking_slots 파싱 실패: {e}')

    def _on_get_slot_info(self, request, response):
        response.data_ready = self._cache.ready
        info = self._cache.query(request.slot_id)
        response.exists = info is not None
        if info is not None:
            response.occupied = bool(info["occupied"])
            response.is_accessible = bool(info["is_accessible"])
            response.pose = _yaw_deg_to_pose(info["x"], info["y"], info["yaw_deg"])
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
