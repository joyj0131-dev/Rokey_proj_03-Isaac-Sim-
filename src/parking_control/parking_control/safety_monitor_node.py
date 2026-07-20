#!/usr/bin/env python3
"""safety_monitor: LiDAR 포인트클라우드 하나로 두 가지를 감시한다.

  ① 통로 장애물 감지 → obstacle_alert 토픽 발행 (parking_robot_system의
     safety_monitor 스켈레톤과 같은 인터페이스)
  ② 주차 슬롯 점유 판정 → parking_slots.status를 실시간으로 갱신

같은 LiDAR 토픽을 보는 감시 기능이라 노드 하나로 합쳤다(구독·DB 연결을
두 번 만들 이유가 없음). 담당자가 아직 미정인 팀 공유 스켈레톤
(parking_robot_system)은 건드리지 않고, 이 노드가 그 자리를 대신할 수
있는 독립 구현이다(sim_orchestrator와 같은 패턴 — 필요하면 팀 합의 후 교체).

무엇이 막았는지/점유했는지(사람/차량/기타)는 구분하지 않는다 —
ObstacleAlert.msg가 불리언 하나뿐이고, 주차 목적에도 있다/없다면
충분하기 때문이다.

주의: LiDAR가 실제로 ROS2 브릿지로 연결되기 전까지는 입력 토픽에 아무
데이터도 안 들어와서 이 노드는 그냥 조용히 대기만 한다 (에러는 안 남).
"""

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2

from parking_robot_interfaces.msg import ObstacleAlert

from parking_control.core.db import ParkingDB
from parking_control.core.graph import ParkingMap
from parking_control.core.obstacle_detector import detect_blocked_zones, zone_boxes
from parking_control.core.slot_occupancy_detector import detect as detect_slot_occupancy
from parking_control.parking_slot_manager_node import _default_map_yaml


class SafetyMonitorNode(Node):

    def __init__(self):
        super().__init__("safety_monitor")

        self.declare_parameter("db_host", "localhost")
        self.declare_parameter("db_user", "parking")
        self.declare_parameter("db_password", "parking1234")
        self.declare_parameter("db_name", "parking")
        self.declare_parameter("map_yaml", _default_map_yaml())
        self.declare_parameter("lidar_topic", "lidar_points")

        p = self.get_parameter
        self._db = ParkingDB(
            host=p("db_host").value, user=p("db_user").value,
            password=p("db_password").value, database=p("db_name").value)
        self._map = ParkingMap.load(p("map_yaml").value)
        self._zone_boxes = zone_boxes(self._map)
        self._last_slot_status = {}   # slot_id -> 마지막으로 DB에 쓴 상태 (중복 쓰기 방지)

        self._alert_pub = self.create_publisher(ObstacleAlert, "obstacle_alert", 10)
        self.create_subscription(
            PointCloud2, p("lidar_topic").value, self._on_pointcloud, 10)

        slot_count = len(self._map.nodes_of_kind("slot"))
        self.get_logger().info(
            f"safety_monitor 시작 (lidar_topic={p('lidar_topic').value}, "
            f"통로 {len(self._zone_boxes)}개 + 슬롯 {slot_count}개 감시) — "
            "LiDAR가 이 토픽에 연결되기 전까지는 대기만 합니다")

    def _on_pointcloud(self, msg):
        # read_points()는 구조화 배열(필드별 named dtype)을 반환하므로
        # np.array(list(...), dtype=float64)로 바로 캐스팅하면 에러가 난다.
        # 필드를 각각 뽑아서 일반 (N,3) 배열로 조립해야 한다.
        cloud = point_cloud2.read_points(
            msg, field_names=("x", "y", "z"), skip_nans=True)
        if cloud.size == 0:
            return
        points = np.column_stack(
            [cloud["x"], cloud["y"], cloud["z"]]).astype(np.float64)

        self._check_obstacles(points)
        self._update_slot_occupancy(points)

    def _check_obstacles(self, points):
        robot_positions = self._db.all_robot_positions()
        blocked = detect_blocked_zones(points, self._zone_boxes, robot_positions)
        blocked_zones = sorted(zid for zid, is_blocked in blocked.items() if is_blocked)

        alert = ObstacleAlert()
        alert.obstacle_detected = bool(blocked_zones)
        if blocked_zones:
            x0, x1, y0, y1 = self._zone_boxes[blocked_zones[0]]
            alert.description = f"통로 막힘: {', '.join(blocked_zones)}"
            alert.location.x = (x0 + x1) / 2
            alert.location.y = (y0 + y1) / 2
            self.get_logger().warn(alert.description)
        self._alert_pub.publish(alert)

    def _update_slot_occupancy(self, points):
        # 로봇/차량 구분 없이 판정한다 — 로봇이 슬롯 위에 있다는 것 자체가
        # 지금 그 칸에 뭔가(차든 로봇이든) 있다는 뜻이라 제외할 이유가 없다
        # (통로 장애물 감지와 달리 여기서는 robot_positions을 빼지 않는다).
        results = detect_slot_occupancy(points, self._map)
        for slot_id, r in results.items():
            new_status = "OCCUPIED" if r["occupied"] else "EMPTY"
            if new_status != self._last_slot_status.get(slot_id):
                self._db.set_slot_status(slot_id, new_status)
                self._last_slot_status[slot_id] = new_status
                self.get_logger().info(f"슬롯 {slot_id}: {new_status} (LiDAR 판정)")

    def destroy_node(self):
        self._db.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SafetyMonitorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
