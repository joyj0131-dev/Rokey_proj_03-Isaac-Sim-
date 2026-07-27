#!/usr/bin/env python3
"""LiDAR 장애물 감시와 슬롯 점유 검증 결과를 함께 발행한다.

입력 PointCloud2는 map 좌표로 변환한 뒤 한 번만 판정한다. 같은 결과에서
RViz2용 필터 cloud/marker와 웹 UI용 SlotOccupancyArray를 만들기 때문에 두
화면의 슬롯별 포인트 수와 상태가 어긋나지 않는다.

LiDAR 판정은 DB의 운영 상태를 덮어쓰지 않는다. DB는 작업·예약 정책의
기준이고, LiDAR는 웹에서 불일치 경고를 만드는 독립 검증 값이다.
"""

import numpy as np
import rclpy
import time
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from rclpy.time import Time
from geometry_msgs.msg import Point
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

from parking_robot_interfaces.msg import (
    ObstacleAlert,
    SlotOccupancy,
    SlotOccupancyArray,
)

from parking_control.core.db import ParkingDB
from parking_control.core.graph import ParkingMap
from parking_control.core.obstacle_detector import detect_blocked_zones, zone_boxes
from parking_control.core.slot_occupancy_detector import (
    HEIGHT_THRESHOLD_M,
    POINT_THRESHOLD,
    STABILIZATION_FRAMES,
    STATUS_EMPTY,
    STATUS_OCCUPIED,
    STATUS_UNCERTAIN,
    SlotDecisionStabilizer,
    detect_with_masks,
)
from parking_control.parking_slot_manager_node import _default_map_yaml


_STATUS_TO_MSG = {
    "WAITING": SlotOccupancy.STATUS_WAITING,
    STATUS_EMPTY: SlotOccupancy.STATUS_EMPTY,
    STATUS_OCCUPIED: SlotOccupancy.STATUS_OCCUPIED,
    STATUS_UNCERTAIN: SlotOccupancy.STATUS_UNCERTAIN,
}


class SafetyMonitorNode(Node):

    def __init__(self):
        super().__init__("safety_monitor")

        self.declare_parameter("db_host", "localhost")
        self.declare_parameter("db_user", "parking")
        self.declare_parameter("db_password", "parking1234")
        self.declare_parameter("db_name", "parking")
        self.declare_parameter("map_yaml", _default_map_yaml())
        self.declare_parameter("lidar_world_topic", "/parking/lidar/points_world")
        self.declare_parameter(
            "filtered_topic", "/parking/lidar/points_filtered"
        )
        self.declare_parameter(
            "slot_points_topic", "/parking/lidar/points_in_slots"
        )
        self.declare_parameter(
            "slot_marker_topic", "/parking/slot_markers"
        )
        self.declare_parameter(
            "slot_occupancy_topic", "/parking/slot_occupancy"
        )
        self.declare_parameter("target_frame", "map")
        self.declare_parameter("height_threshold_m", HEIGHT_THRESHOLD_M)
        self.declare_parameter("point_threshold", POINT_THRESHOLD)
        self.declare_parameter(
            "stabilization_frames", STABILIZATION_FRAMES
        )
        self.declare_parameter("visual_publish_hz", 5.0)
        # parking_environment_v4.usd의 CeilingLidarCenter 실측 위치.
        self.declare_parameter("sensor_id", "L1")
        self.declare_parameter("sensor_x", 0.5)
        self.declare_parameter("sensor_y", 0.0)
        self.declare_parameter("sensor_z", 5.12)

        p = self.get_parameter
        self._input_topic = str(p("lidar_world_topic").value)
        self._target_frame = str(p("target_frame").value)
        self._height_threshold = float(p("height_threshold_m").value)
        self._point_threshold = int(p("point_threshold").value)
        self._stabilization_frames = int(p("stabilization_frames").value)
        visual_publish_hz = float(p("visual_publish_hz").value)
        if visual_publish_hz <= 0:
            raise ValueError("visual_publish_hz는 0보다 커야 합니다")
        self._visual_publish_period = 1.0 / visual_publish_hz
        self._sensor_id = str(p("sensor_id").value)
        self._sensor_position = (
            float(p("sensor_x").value),
            float(p("sensor_y").value),
            float(p("sensor_z").value),
        )
        if self._point_threshold < 1:
            raise ValueError("point_threshold는 1 이상이어야 합니다")

        self._db = ParkingDB(
            host=p("db_host").value,
            user=p("db_user").value,
            password=p("db_password").value,
            database=p("db_name").value,
        )
        self._map = ParkingMap.load(p("map_yaml").value)
        self._zone_boxes = zone_boxes(self._map)
        slot_ids = self._map.nodes_of_kind("slot")
        self._stabilizer = SlotDecisionStabilizer(
            slot_ids, frames=self._stabilization_frames
        )
        self._last_reported_status = {}
        self._last_cloud_at = None
        self._last_visual_publish_at = None
        self._offline_state_published = False

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        latched_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        cloud_qos = QoSProfile(
            depth=5,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self._alert_pub = self.create_publisher(
            ObstacleAlert, "/obstacle_alert", 10
        )
        self._filtered_pub = self.create_publisher(
            PointCloud2,
            str(p("filtered_topic").value),
            cloud_qos,
        )
        self._slot_points_pub = self.create_publisher(
            PointCloud2,
            str(p("slot_points_topic").value),
            cloud_qos,
        )
        self._occupancy_pub = self.create_publisher(
            SlotOccupancyArray,
            str(p("slot_occupancy_topic").value),
            latched_qos,
        )
        self._marker_pub = self.create_publisher(
            MarkerArray,
            str(p("slot_marker_topic").value),
            latched_qos,
        )
        # 이전 RViz/스크립트 호환. 신규 설정은 /parking/slot_markers를 쓴다.
        self._legacy_marker_pub = self.create_publisher(
            MarkerArray, "parking_status_markers", latched_qos
        )
        self.create_subscription(
            PointCloud2,
            self._input_topic,
            self._on_pointcloud,
            qos_profile_sensor_data,
        )
        self.create_timer(0.5, self._publish_offline_state_if_needed)

        self.get_logger().info(
            "safety_monitor 시작: "
            f"{self._input_topic} → filtered/slot points/occupancy/markers "
            f"(frame={self._target_frame}, z>{self._height_threshold:.2f}m, "
            f"points>={self._point_threshold}, 안정화={self._stabilization_frames}프레임)"
        )

    @staticmethod
    def _cloud_to_array(message):
        cloud = point_cloud2.read_points(
            message, field_names=("x", "y", "z"), skip_nans=True
        )
        if cloud.size == 0:
            return np.empty((0, 3), dtype=np.float64)
        return np.column_stack(
            [cloud["x"], cloud["y"], cloud["z"]]
        ).astype(np.float64)

    @staticmethod
    def _apply_transform(points, transform):
        """geometry_msgs/Transform을 (N,3) 점에 적용한다."""
        if points.size == 0:
            return points
        q = transform.rotation
        norm = np.sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w)
        if norm <= 1e-12:
            raise ValueError("TF quaternion의 크기가 0입니다")
        x, y, z, w = q.x / norm, q.y / norm, q.z / norm, q.w / norm
        rotation = np.array([
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ])
        translation = np.array([
            transform.translation.x,
            transform.translation.y,
            transform.translation.z,
        ])
        return points @ rotation.T + translation

    def _to_target_frame(self, points, message):
        source_frame = message.header.frame_id.strip()
        if not source_frame:
            raise TransformException("PointCloud2 header.frame_id가 비어 있습니다")
        if source_frame == self._target_frame:
            return points
        stamped = self._tf_buffer.lookup_transform(
            self._target_frame,
            source_frame,
            Time.from_msg(message.header.stamp),
            timeout=Duration(seconds=0.1),
        )
        return self._apply_transform(points, stamped.transform)

    def _on_pointcloud(self, message):
        now = time.monotonic()
        self._last_cloud_at = now
        self._offline_state_published = False
        received = self._cloud_to_array(message)
        try:
            points = self._to_target_frame(received, message)
        except (TransformException, ValueError) as error:
            self.get_logger().error(
                f"LiDAR TF 변환 실패 ({message.header.frame_id or 'empty'}"
                f" → {self._target_frame}): {error}"
            )
            self._stabilizer.reset()
            self._publish_measurement_error(
                message,
                received_count=len(received),
                detail=str(error),
            )
            return

        frame_results, height_mask, slot_mask = detect_with_masks(
            points,
            self._map,
            height_threshold=self._height_threshold,
            point_threshold=self._point_threshold,
        )
        stable_results = self._stabilizer.update(frame_results)
        filtered = np.ascontiguousarray(points[height_mask], dtype=np.float32)
        in_slots = np.ascontiguousarray(points[slot_mask], dtype=np.float32)

        header = Header()
        header.stamp = message.header.stamp
        header.frame_id = self._target_frame
        blocked = self._check_obstacles(points)
        occupancy = self._build_occupancy_message(
            header,
            stable_results,
            received_count=len(received),
            filtered_count=len(filtered),
            slot_count=len(in_slots),
        )
        self._occupancy_pub.publish(occupancy)
        if (
            self._last_visual_publish_at is None
            or now - self._last_visual_publish_at >= self._visual_publish_period
        ):
            self._last_visual_publish_at = now
            self._filtered_pub.publish(
                point_cloud2.create_cloud_xyz32(header, filtered)
            )
            self._slot_points_pub.publish(
                point_cloud2.create_cloud_xyz32(header, in_slots)
            )
            markers = self._build_markers(header, blocked, stable_results)
            self._marker_pub.publish(markers)
            self._legacy_marker_pub.publish(markers)
        self._log_status_changes(stable_results)

    def _check_obstacles(self, points):
        robot_positions = self._db.all_robot_positions()
        blocked = detect_blocked_zones(
            points, self._zone_boxes, robot_positions
        )
        blocked_zones = sorted(
            zone_id for zone_id, value in blocked.items() if value
        )
        alert = ObstacleAlert()
        alert.obstacle_detected = bool(blocked_zones)
        if blocked_zones:
            x0, x1, y0, y1 = self._zone_boxes[blocked_zones[0]]
            alert.description = f"통로 막힘: {', '.join(blocked_zones)}"
            alert.location.x = (x0 + x1) / 2
            alert.location.y = (y0 + y1) / 2
        self._alert_pub.publish(alert)
        return blocked

    def _build_occupancy_message(
        self,
        header,
        results,
        *,
        received_count,
        filtered_count,
        slot_count,
    ):
        message = SlotOccupancyArray()
        message.header = header
        message.sensor_id = self._sensor_id
        message.source_topic = self._input_topic
        message.measurement_status = (
            SlotOccupancyArray.MEASUREMENT_NO_VALID_POINTS
            if filtered_count == 0
            else SlotOccupancyArray.MEASUREMENT_OK
        )
        message.status_message = (
            "PointCloud2는 수신됐지만 높이 필터를 통과한 포인트가 없습니다."
            if filtered_count == 0
            else "정상 수신"
        )
        message.received_point_count = received_count
        message.filtered_point_count = filtered_count
        message.slot_point_count = slot_count
        message.stabilization_frames = self._stabilization_frames
        width = float(self._map.meta["params"]["space_width"])
        length = float(self._map.meta["params"]["space_length"])
        for slot_id, result in results.items():
            slot = SlotOccupancy()
            slot.slot_id = slot_id
            slot.status = _STATUS_TO_MSG[result["status"]]
            slot.point_count = result["point_count"]
            slot.point_threshold = self._point_threshold
            slot.height_threshold_m = self._height_threshold
            slot.center.x = float(result["x"])
            slot.center.y = float(result["y"])
            slot.width = width
            slot.length = length
            message.slots.append(slot)
        return message

    def _publish_measurement_error(self, source, *, received_count, detail):
        waiting_results = self._waiting_results()
        message = SlotOccupancyArray()
        message.header.stamp = source.header.stamp
        message.header.frame_id = self._target_frame
        message.sensor_id = self._sensor_id
        message.source_topic = self._input_topic
        message.measurement_status = SlotOccupancyArray.MEASUREMENT_TF_ERROR
        message.status_message = f"map 좌표 변환 실패: {detail}"
        message.received_point_count = received_count
        message.stabilization_frames = self._stabilization_frames
        for slot_id, result in waiting_results.items():
            slot = SlotOccupancy()
            slot.slot_id = slot_id
            slot.status = SlotOccupancy.STATUS_WAITING
            slot.point_threshold = self._point_threshold
            slot.height_threshold_m = self._height_threshold
            slot.center.x = float(result["x"])
            slot.center.y = float(result["y"])
            slot.width = float(result["width"])
            slot.length = float(result["length"])
            message.slots.append(slot)
        self._occupancy_pub.publish(message)
        header = message.header
        empty = np.empty((0, 3), dtype=np.float32)
        self._filtered_pub.publish(
            point_cloud2.create_cloud_xyz32(header, empty)
        )
        self._slot_points_pub.publish(
            point_cloud2.create_cloud_xyz32(header, empty)
        )
        markers = self._build_markers(header, {}, waiting_results)
        self._marker_pub.publish(markers)
        self._legacy_marker_pub.publish(markers)

    def _publish_offline_state_if_needed(self):
        age = (
            None
            if self._last_cloud_at is None
            else time.monotonic() - self._last_cloud_at
        )
        if age is not None and age <= 3.0:
            return
        if self._offline_state_published:
            return
        self._offline_state_published = True
        self._stabilizer.reset()
        message = SlotOccupancyArray()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = self._target_frame
        message.sensor_id = self._sensor_id
        message.source_topic = self._input_topic
        message.measurement_status = SlotOccupancyArray.MEASUREMENT_WAITING
        message.status_message = "PointCloud2 메시지 수신 대기"
        message.stabilization_frames = self._stabilization_frames
        waiting_results = self._waiting_results()
        for slot_id, result in waiting_results.items():
            slot = SlotOccupancy()
            slot.slot_id = slot_id
            slot.status = SlotOccupancy.STATUS_WAITING
            slot.point_threshold = self._point_threshold
            slot.height_threshold_m = self._height_threshold
            slot.center.x = float(result["x"])
            slot.center.y = float(result["y"])
            slot.width = float(result["width"])
            slot.length = float(result["length"])
            message.slots.append(slot)
        self._occupancy_pub.publish(message)
        empty = np.empty((0, 3), dtype=np.float32)
        self._filtered_pub.publish(
            point_cloud2.create_cloud_xyz32(message.header, empty)
        )
        self._slot_points_pub.publish(
            point_cloud2.create_cloud_xyz32(message.header, empty)
        )
        markers = self._build_markers(message.header, {}, waiting_results)
        self._marker_pub.publish(markers)
        self._legacy_marker_pub.publish(markers)

    def _waiting_results(self):
        width = float(self._map.meta["params"]["space_width"])
        length = float(self._map.meta["params"]["space_length"])
        return {
            slot_id: {
                "x": self._map.node_pos(slot_id)[0],
                "y": self._map.node_pos(slot_id)[1],
                "width": width,
                "length": length,
                "status": "WAITING",
                "point_count": 0,
            }
            for slot_id in self._map.nodes_of_kind("slot")
        }

    def _build_markers(self, header, blocked, results):
        markers = MarkerArray()
        clear = Marker()
        clear.action = Marker.DELETEALL
        markers.markers.append(clear)
        marker_id = 1
        width = float(self._map.meta["params"]["space_width"])
        length = float(self._map.meta["params"]["space_length"])

        colors = {
            STATUS_EMPTY: (0.1, 0.9, 0.2),
            STATUS_OCCUPIED: (1.0, 0.1, 0.1),
            STATUS_UNCERTAIN: (1.0, 0.55, 0.05),
            "WAITING": (0.55, 0.58, 0.62),
        }
        labels = {
            STATUS_EMPTY: "공석",
            STATUS_OCCUPIED: "점유",
            STATUS_UNCERTAIN: "불확실",
            "WAITING": "판정 대기",
        }
        for slot_id, result in results.items():
            color = colors[result["status"]]
            fill = self._marker(header, "slot_fill", marker_id, Marker.CUBE)
            marker_id += 1
            fill.pose.position.x = float(result["x"])
            fill.pose.position.y = float(result["y"])
            fill.pose.position.z = 0.03
            fill.scale.x, fill.scale.y, fill.scale.z = width, length, 0.05
            fill.color.r, fill.color.g, fill.color.b = color
            fill.color.a = 0.28
            markers.markers.append(fill)

            border = self._marker(
                header, "slot_boundary", marker_id, Marker.LINE_STRIP
            )
            marker_id += 1
            border.scale.x = 0.08
            border.color.r, border.color.g, border.color.b = color
            border.color.a = 1.0
            for x, y in (
                (result["x"] - width / 2, result["y"] - length / 2),
                (result["x"] + width / 2, result["y"] - length / 2),
                (result["x"] + width / 2, result["y"] + length / 2),
                (result["x"] - width / 2, result["y"] + length / 2),
                (result["x"] - width / 2, result["y"] - length / 2),
            ):
                point = Point()
                point.x, point.y, point.z = float(x), float(y), 0.08
                border.points.append(point)
            markers.markers.append(border)

            text = self._marker(header, "slot_text", marker_id, Marker.TEXT_VIEW_FACING)
            marker_id += 1
            text.pose.position.x = float(result["x"])
            text.pose.position.y = float(result["y"])
            text.pose.position.z = 0.65
            text.scale.z = 0.42
            text.color.r = text.color.g = text.color.b = 1.0
            text.color.a = 1.0
            text.text = (
                f"{slot_id}\n{labels[result['status']]}\n"
                f"유효 {result['point_count']} / 기준 {self._point_threshold}"
            )
            markers.markers.append(text)

        for zone_id, is_blocked in blocked.items():
            if not is_blocked:
                continue
            x0, x1, y0, y1 = self._zone_boxes[zone_id]
            marker = self._marker(
                header, "blocked_zones", marker_id, Marker.CUBE
            )
            marker_id += 1
            marker.pose.position.x = (x0 + x1) / 2
            marker.pose.position.y = (y0 + y1) / 2
            marker.pose.position.z = 0.1
            marker.scale.x = x1 - x0
            marker.scale.y = y1 - y0
            marker.scale.z = 0.2
            marker.color.r, marker.color.a = 1.0, 0.35
            markers.markers.append(marker)

        sx, sy, sz = self._sensor_position
        sensor = self._marker(header, "lidar_sensor", marker_id, Marker.SPHERE)
        marker_id += 1
        sensor.pose.position.x, sensor.pose.position.y = sx, sy
        sensor.pose.position.z = sz
        sensor.scale.x = sensor.scale.y = sensor.scale.z = 0.35
        sensor.color.r, sensor.color.g, sensor.color.b, sensor.color.a = (
            0.1, 0.55, 1.0, 1.0
        )
        markers.markers.append(sensor)
        sensor_text = self._marker(
            header, "lidar_sensor", marker_id, Marker.TEXT_VIEW_FACING
        )
        sensor_text.pose.position.x, sensor_text.pose.position.y = sx, sy
        sensor_text.pose.position.z = sz + 0.45
        sensor_text.scale.z = 0.35
        sensor_text.color.r = sensor_text.color.g = sensor_text.color.b = 1.0
        sensor_text.color.a = 1.0
        sensor_text.text = self._sensor_id
        markers.markers.append(sensor_text)
        return markers

    @staticmethod
    def _marker(header, namespace, marker_id, marker_type):
        marker = Marker()
        marker.header = header
        marker.ns = namespace
        marker.id = marker_id
        marker.type = marker_type
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        return marker

    def _log_status_changes(self, results):
        for slot_id, result in results.items():
            status = result["status"]
            if self._last_reported_status.get(slot_id) == status:
                continue
            self._last_reported_status[slot_id] = status
            self.get_logger().info(
                f"LiDAR 검증 {slot_id}: {status} "
                f"({result['point_count']} / {self._point_threshold} points)"
            )

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
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
