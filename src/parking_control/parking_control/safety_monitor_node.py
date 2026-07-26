#!/usr/bin/env python3
"""safety_monitor: LiDAR 월드 포인트클라우드 하나로 두 가지를 한다.

  ① 통로 장애물 감지 → obstacle_alert 토픽 발행 (parking_robot_system의
     safety_monitor 스켈레톤과 같은 인터페이스)
  ② 주차 슬롯 점유 판정 → parking_slots.status를 실시간으로 갱신
  둘의 판정 결과는 parking_status_markers(MarkerArray — 슬롯은 초록/빨강
  박스, 막힌 통로는 빨강 반투명 박스)로도 발행한다. RViz2에서
  config/lidar_live.rviz를 열고 Marker Array 디스플레이(토픽:
  parking_status_markers)만 하나 추가하면 실시간으로 주차 현황·장애물이
  눈에 보인다.

같은 LiDAR 데이터를 보는 감시 기능이라 노드 하나로 합쳤다(구독·DB 연결을
두 번 만들 이유가 없음). 담당자가 아직 미정인 팀 공유 스켈레톤
(parking_robot_system)은 건드리지 않고, 이 노드가 그 자리를 대신할 수
있는 독립 구현이다(sim_orchestrator와 같은 패턴 — 필요하면 팀 합의 후 교체).

무엇이 막았는지/점유했는지(사람/차량/기타)는 구분하지 않는다 —
ObstacleAlert.msg가 불리언 하나뿐이고, 주차 목적에도 있다/없다면
충분하기 때문이다.

2026-07-25: 이전 버전은 자체 lidar_topic 파라미터(추정 토픽명)로 raw
센서 좌표를 구독한 뒤 core/lidar_frame_transform.py(검증 안 된 센서
오프셋 계산)로 직접 월드 좌표 변환을 했었다. 그 토픽명은 실제로 존재하지
않았고(실측 결과 진짜 발행 토픽은 /parking/lidar/ceiling_01/points_usd,
scripts/lidar/capture_lidar.py --live가 발행), 변환도 이미 검증된
scripts/lidar/ros_pointcloud_world_relay.py가 같은 일을 하고 있었다 —
raw 토픽들을 구독해 USD Y-up → ROS map 변환까지 끝낸 뒤
/parking/lidar/points_world로 재발행한다(scripts/lidar/run_live_rviz.sh가
캡처+릴레이+RViz를 한 번에 띄우는 스크립트). 그래서 이 노드는 이제 자체
변환 없이 그 결과 토픽을 바로 구독한다 — 이 노드가 다시 같은 이름으로
points_world를 발행하면 릴레이와 퍼블리셔가 중복되므로 더 이상 발행하지
않는다(RViz는 릴레이가 내는 걸 직접 본다).

주의: 브릿지/캡처/릴레이가 안 떠 있으면 토픽에 데이터가 안 들어와서 이
노드는 그냥 조용히 대기만 한다 (에러는 안 남).
"""

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Bool
from visualization_msgs.msg import Marker, MarkerArray

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
        # 2026-07-25: scripts/lidar/ros_pointcloud_world_relay.py가 이미 raw
        # 센서 토픽들을 구독해 ROS map(월드) 좌표로 변환·병합해 이 토픽으로 낸다
        # (scripts/lidar/run_live_rviz.sh가 캡처+릴레이+RViz를 한 번에 띄움).
        self.declare_parameter("lidar_world_topic", "/parking/lidar/points_world")

        p = self.get_parameter
        self._db = ParkingDB(
            host=p("db_host").value, user=p("db_user").value,
            password=p("db_password").value, database=p("db_name").value)
        self._map = ParkingMap.load(p("map_yaml").value)
        self._zone_boxes = zone_boxes(self._map)
        self._last_slot_status = {}   # slot_id -> 마지막으로 DB에 쓴 상태 (중복 쓰기 방지)

        self._alert_pub = self.create_publisher(
            ObstacleAlert, "/obstacle_alert", 10)
        # Isaac 러너는 커스텀 인터페이스 설치 여부와 관계없이 받을 수 있도록
        # 표준 Bool 긴급정지 토픽도 함께 사용한다.
        self._emergency_pub = self.create_publisher(
            Bool, "/emergency_stop", 10)
        self._marker_pub = self.create_publisher(
            MarkerArray, "parking_status_markers", 10)
        self.create_subscription(
            PointCloud2, p("lidar_world_topic").value, self._on_pointcloud, 10)

        slot_count = len(self._map.nodes_of_kind("slot"))
        self.get_logger().info(
            f"safety_monitor 시작 (lidar_world_topic={p('lidar_world_topic').value}, "
            f"통로 {len(self._zone_boxes)}개 + 슬롯 {slot_count}개 감시) — "
            "캡처+릴레이(scripts/lidar/run_live_rviz.sh)가 연결되기 전까지는 "
            "대기만 합니다.")

    def _on_pointcloud(self, msg):
        # ros_pointcloud_world_relay.py가 이미 ROS map(월드) 좌표로 변환해 내보내므로
        # 여기서는 추가 변환 없이 그대로 쓴다. read_points()는 구조화 배열(필드별
        # named dtype)을 반환하므로 np.array(list(...), dtype=float64)로 바로
        # 캐스팅하면 에러가 난다 — 필드를 각각 뽑아서 일반 (N,3) 배열로 조립한다.
        cloud = point_cloud2.read_points(
            msg, field_names=("x", "y", "z"), skip_nans=True)
        if cloud.size == 0:
            return
        points = np.column_stack(
            [cloud["x"], cloud["y"], cloud["z"]]).astype(np.float64)

        blocked = self._check_obstacles(points)
        slot_results = self._update_slot_occupancy(points)
        self._publish_markers(blocked, slot_results)

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
        self._emergency_pub.publish(
            Bool(data=bool(alert.obstacle_detected)))
        return blocked

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
        return results

    def _publish_markers(self, blocked, slot_results):
        """슬롯 점유/통로 막힘 판정을 RViz2 MarkerArray로 시각화(2026-07-23).

        슬롯: 초록(빈칸)/빨강(점유) 박스 + 텍스트 라벨.
        통로: 막힌 구간만 빨강 반투명 박스로 표시(평소엔 안 그림 — 통로 18개를
        늘 다 그리면 화면이 지저분해지고, "막힘"이야말로 실시간으로 눈에 띄어야
        하는 정보라 그것만 그린다)."""
        space_w = self._map.meta["params"]["space_width"]
        space_l = self._map.meta["params"]["space_length"]
        markers = MarkerArray()
        now = self.get_clock().now().to_msg()
        idx = 0

        for slot_id, r in slot_results.items():
            m = Marker()
            m.header.frame_id = "map"
            m.header.stamp = now
            m.ns = "slots"
            m.id = idx
            idx += 1
            m.type = Marker.CUBE
            m.action = Marker.ADD
            m.pose.position.x = r["x"]
            m.pose.position.y = r["y"]
            m.pose.position.z = 0.05
            m.pose.orientation.w = 1.0
            m.scale.x = space_w * 0.9
            m.scale.y = space_l * 0.9
            m.scale.z = 0.05
            m.color.a = 0.6
            if r["occupied"]:
                m.color.r, m.color.g, m.color.b = 1.0, 0.15, 0.15
            else:
                m.color.r, m.color.g, m.color.b = 0.15, 0.85, 0.15
            markers.markers.append(m)

        for zone_id, is_blocked in blocked.items():
            if not is_blocked:
                continue
            x0, x1, y0, y1 = self._zone_boxes[zone_id]
            m = Marker()
            m.header.frame_id = "map"
            m.header.stamp = now
            m.ns = "blocked_zones"
            m.id = idx
            idx += 1
            m.type = Marker.CUBE
            m.action = Marker.ADD
            m.pose.position.x = (x0 + x1) / 2
            m.pose.position.y = (y0 + y1) / 2
            m.pose.position.z = 0.1
            m.pose.orientation.w = 1.0
            m.scale.x = x1 - x0
            m.scale.y = y1 - y0
            m.scale.z = 0.2
            m.color.r, m.color.g, m.color.b, m.color.a = 1.0, 0.0, 0.0, 0.35
            markers.markers.append(m)

        self._marker_pub.publish(markers)

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
