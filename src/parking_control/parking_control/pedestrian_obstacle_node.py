#!/usr/bin/env python3
"""animation/pedestrians_v4.usda 보행자 통로의 사람/비사람 LiDAR 기하 분류.

safety_monitor는 슬롯 점유만 판정하고(통로 장애물 판정은 로봇이 차량을
들고 지나갈 때 오히려 스스로를 막힘으로 오인해 2026-07-27에 제거됐다 —
safety_monitor_node.py 상단 주석 참고) 무엇이 있는지는 구분하지 않는다.
이 노드는 그 옆에서 별도 목적으로 돈다: 보행자 대기 차로(X=-8.5,
animation/pedestrians_v4.usda의 EntryPedestrian/ExitPedestrian
appear/vanishPose 기준)에서 사람과 차량/로봇/기타를 키(top_height)·
풋프린트(width/length)로 구분해 사람 유무·위치·분류 근거를 별도 토픽으로
낸다.

scripts/lidar/ros_pointcloud_world_relay.py가 이미 두 LiDAR raw 토픽을
USD Y-up → ROS map(Z-up)으로 합쳐 /parking/lidar/points_world로 내므로
이 노드는 추가 좌표 변환 없이 그 결과를 그대로 구독한다.

카메라 영상이나 보행자 애니메이션의 정답 좌표는 쓰지 않는다 — 판정은
오직 병합된 점군의 기하만으로 이뤄진다.
"""

import json

import numpy as np
import rclpy
from geometry_msgs.msg import Pose, PoseArray
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Bool, String
from visualization_msgs.msg import Marker, MarkerArray

from parking_control.core.pedestrian_presence import (
    PersonClassifierConfig,
    Region,
    classify_person_clusters,
    detect_region_presence,
)

# 보행자 대기 차로 ROI(ROS map 좌표). animation/pedestrians_v4.usda의
# EntryPedestrian(appearPose (-8.5,0,7.075) -> vanishPose (-8.5,0,2.5))과
# ExitPedestrian(appearPose (-8.5,0,-2.5) -> vanishPose (-8.5,0,-7.075))이
# 지나는 X=-8.5 고정 차로다. usd_to_ros: ros_x=usd_x, ros_y=-usd_z
# (config/parking_map.yaml meta 참고). 사람 캡슐 콜라이더 반지름(0.32 m)에
# 여유를 더해 폭 1.0 m, 진행 방향으로는 appear/vanish 지점 바깥쪽에
# 각각 0.4 m 여유를 뒀다.
REGIONS = (
    Region("entry_wait_lane", -9.0, -8.0, -7.5, -2.1),
    Region("exit_wait_lane", -9.0, -8.0, 2.1, 7.5),
)

POINTS_TOPIC = "/parking/lidar/points_world"


class PedestrianObstacleNode(Node):

    def __init__(self):
        super().__init__("pedestrian_obstacle")
        self.declare_parameter("min_points", 20)
        self.declare_parameter("min_cluster_points", 8)
        self.declare_parameter("cluster_cell_size", 0.3)
        self.declare_parameter("min_person_height", 1.0)
        self.declare_parameter("max_person_height", 2.3)
        self.declare_parameter("max_person_width", 1.2)
        self.declare_parameter("max_person_length", 1.6)
        self.declare_parameter("detection_hold_sec", 1.0)

        value = lambda name: self.get_parameter(name).value
        self._min_points = int(value("min_points"))
        self._classifier_config = PersonClassifierConfig(
            cluster_cell_size=float(value("cluster_cell_size")),
            min_cluster_points=int(value("min_cluster_points")),
            min_top_height=float(value("min_person_height")),
            max_top_height=float(value("max_person_height")),
            max_width=float(value("max_person_width")),
            max_length=float(value("max_person_length")),
        )
        self._hold_ns = int(float(value("detection_hold_sec")) * 1e9)

        self._pedestrian_pub = self.create_publisher(
            Bool, "/parking/pedestrian_obstacle", 10)
        self._pedestrian_detail_pub = self.create_publisher(
            String, "/parking/pedestrian_obstacle_detail", 10)
        self._person_pub = self.create_publisher(
            Bool, "/parking/person_detected", 10)
        self._classification_pub = self.create_publisher(
            String, "/parking/person_classification", 10)
        self._candidates_pub = self.create_publisher(
            PoseArray, "/parking/person_candidates", 10)
        self._markers_pub = self.create_publisher(
            MarkerArray, "/parking/person_classification_markers", 10)

        self._last_person_stamp_ns = None
        self._last_person_clusters = []

        self.create_subscription(
            PointCloud2, POINTS_TOPIC, self._on_points, qos_profile_sensor_data)
        self.get_logger().info(
            f"pedestrian_obstacle 시작 ({POINTS_TOPIC} 구독, "
            f"ROI={[r.name for r in REGIONS]})")

    def _on_points(self, message):
        cloud = point_cloud2.read_points(
            message, field_names=("x", "y", "z"), skip_nans=True)
        if cloud.size == 0:
            points = np.empty((0, 3), dtype=np.float64)
        else:
            points = np.column_stack(
                (cloud["x"], cloud["y"], cloud["z"])).astype(np.float64)

        presence = detect_region_presence(
            points, REGIONS, min_points=self._min_points)
        obstacle_regions = [
            name for name, info in presence.items() if info["detected"]]
        self._pedestrian_pub.publish(Bool(data=bool(obstacle_regions)))
        self._pedestrian_detail_pub.publish(String(data=json.dumps(presence)))

        clusters = classify_person_clusters(
            points, REGIONS, config=self._classifier_config)
        person_clusters = [c for c in clusters if c["label"] == "PERSON"]

        now_ns = self.get_clock().now().nanoseconds
        if person_clusters:
            self._last_person_stamp_ns = now_ns
            self._last_person_clusters = person_clusters
        held = (
            self._last_person_stamp_ns is not None
            and now_ns - self._last_person_stamp_ns <= self._hold_ns
        )
        active_clusters = (
            person_clusters if person_clusters
            else (self._last_person_clusters if held else [])
        )

        self._person_pub.publish(Bool(data=bool(held)))
        payload = {
            "person_detected": bool(held),
            "person_count": len(active_clusters),
            "clusters": clusters,
        }
        self._classification_pub.publish(String(data=json.dumps(payload)))

        pose_array = PoseArray()
        pose_array.header.stamp = self.get_clock().now().to_msg()
        pose_array.header.frame_id = "map"
        for cluster in active_clusters:
            pose = Pose()
            (pose.position.x, pose.position.y, pose.position.z) = (
                cluster["centroid"])
            pose.orientation.w = 1.0
            pose_array.poses.append(pose)
        self._candidates_pub.publish(pose_array)

        self._markers_pub.publish(self._build_markers(clusters))

    def _build_markers(self, clusters):
        markers = MarkerArray()
        stamp = self.get_clock().now().to_msg()
        for index, cluster in enumerate(clusters):
            is_person = cluster["label"] == "PERSON"
            color = (
                (0.1, 1.0, 0.2, 0.55) if is_person else (1.0, 0.25, 0.1, 0.55))
            cx, cy, _ = cluster["centroid"]
            center_z = (cluster["bottom_height"] + cluster["top_height"]) / 2.0

            box = Marker()
            box.header.frame_id = "map"
            box.header.stamp = stamp
            box.ns = "person_classification"
            box.id = index * 2
            box.type = Marker.CUBE
            box.action = Marker.ADD
            box.pose.position.x = cx
            box.pose.position.y = cy
            box.pose.position.z = center_z
            box.pose.orientation.w = 1.0
            box.scale.x = max(cluster["width"], 0.05)
            box.scale.y = max(cluster["length"], 0.05)
            box.scale.z = max(cluster["observed_height"], 0.05)
            box.color.r, box.color.g, box.color.b, box.color.a = color
            markers.markers.append(box)

            label = Marker()
            label.header.frame_id = "map"
            label.header.stamp = stamp
            label.ns = "person_classification"
            label.id = index * 2 + 1
            label.type = Marker.TEXT_VIEW_FACING
            label.action = Marker.ADD
            label.pose.position.x = cx
            label.pose.position.y = cy
            label.pose.position.z = cluster["top_height"] + 0.25
            label.pose.orientation.w = 1.0
            label.scale.z = 0.28
            label.color.r = 1.0
            label.color.g = 1.0
            label.color.b = 1.0
            label.color.a = 0.9
            label.text = (
                f"{cluster['label']} {cluster['confidence']:.2f}\n"
                f"{cluster['width']:.2f}x{cluster['length']:.2f}m "
                f"top={cluster['top_height']:.2f}m"
            )
            markers.markers.append(label)
        return markers


def main(args=None):
    rclpy.init(args=args)
    node = PedestrianObstacleNode()
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
