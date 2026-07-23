#!/usr/bin/env python3
"""parking_slot_manager: find_empty_slot 서비스 제공.

DB에서 빈 슬롯 후보를 뽑고, 그래프 최단거리(pathfinder) 기준으로
입구에서 가장 가까운 슬롯을 골라 반환한다. 로직은 전부 core/에 있고
이 노드는 ROS 서비스 껍데기만 담당한다.
"""

from pathlib import Path

import rclpy
from rclpy.node import Node

from parking_robot_interfaces.srv import FindEmptySlot

from parking_control.core.db import ParkingDB
from parking_control.core.graph import ParkingMap
from parking_control.core.pathfinder import PathFinder


def _default_map_yaml() -> str:
    try:
        from ament_index_python.packages import get_package_share_directory
        share = Path(get_package_share_directory("parking_control"))
        candidate = share / "config" / "parking_map.yaml"
        if candidate.exists():
            return str(candidate)
    except Exception:
        pass
    # 소스 트리에서 직접 실행하는 경우
    return str(Path(__file__).resolve().parent.parent
               / "config" / "parking_map.yaml")


class ParkingSlotManagerNode(Node):

    def __init__(self):
        super().__init__("parking_slot_manager")

        self.declare_parameter("db_host", "localhost")
        self.declare_parameter("db_user", "parking")
        self.declare_parameter("db_password", "parking1234")
        self.declare_parameter("db_name", "parking")
        self.declare_parameter("map_yaml", _default_map_yaml())
        self.declare_parameter("origin_node", "entrance")
        self.declare_parameter("fit_margin_m", 0.3)

        p = self.get_parameter
        self._db = ParkingDB(
            host=p("db_host").value, user=p("db_user").value,
            password=p("db_password").value, database=p("db_name").value)
        self._map = ParkingMap.load(p("map_yaml").value)
        self._pathfinder = PathFinder(self._map)

        params = self._map.meta["params"]
        self._slot_length = params["space_length"]
        self._slot_width = params["space_width"]

        self._service = self.create_service(
            FindEmptySlot, "find_empty_slot", self._handle_find_empty_slot)
        self.get_logger().info(
            f"parking_slot_manager 시작 (지도: {p('map_yaml').value})")

    def _handle_find_empty_slot(self, request, response):
        response.success = False
        response.slot_id = ""

        margin = self.get_parameter("fit_margin_m").value
        if (request.vehicle_length + margin > self._slot_length
                or request.vehicle_width + margin > self._slot_width):
            self.get_logger().warn(
                f"차량({request.vehicle_length:.2f}x{request.vehicle_width:.2f}m)이 "
                f"슬롯({self._slot_length}x{self._slot_width}m)에 안 들어감")
            return response

        origin = self.get_parameter("origin_node").value
        # 요청마다 다른 값(웹 UI 체크박스 → task_dispatcher가 실어 보냄) — 배려석과
        # 일반 슬롯을 완전히 분리 배정한다(섞어서 "가까운 곳" 고르지 않음).
        candidates = self._db.find_empty_slots(request.accessible)
        if not candidates:
            self.get_logger().warn("빈 슬롯 없음")
            return response

        # 그래프 최단거리 기준 정렬. 경로가 막힌(도달 불가) 슬롯은 제외.
        best = None
        for slot in candidates:
            path = self._pathfinder.find_path(origin, slot["slot_id"])
            if path is not None and (best is None or path.length < best[1]):
                best = (slot, path.length)
        if best is None:
            self.get_logger().warn("모든 빈 슬롯이 도달 불가")
            return response

        slot, dist = best
        x, y = self._map.node_pos(slot["slot_id"])
        response.success = True
        response.slot_id = slot["slot_id"]
        response.slot_pose.position.x = float(x)
        response.slot_pose.position.y = float(y)
        response.slot_pose.orientation.w = 1.0
        self.get_logger().info(
            f"슬롯 배정: {slot['slot_id']} (경로거리 {dist:.2f}m)")
        return response

    def destroy_node(self):
        self._db.close()
        super().destroy_node()


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


if __name__ == "__main__":
    main()
