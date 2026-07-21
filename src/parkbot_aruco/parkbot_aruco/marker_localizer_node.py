"""M3 측위 노드 — Isaac 이 발행하는 카메라 토픽을 받아 마커를 검출하고
로봇의 **월드 좌표**를 계산해 마커 ID·좌표를 로그로 찍는다. (순수 ROS 2)

이 노드가 사용자가 그린 흐름의 "순수 ROS 쪽"이다:
  [Isaac] 영상 토픽 발행  →  [이 노드] 받아서 검출+측위, 마커 ID+좌표 로그

M2(검출, aruco_pose)와 M3(월드 측위, marker_localizer)를 한 노드에서 돈다 —
별도 aruco_detector 노드로 쪼갤 수도 있지만(계획서 구조), 데모에서는 두 토픽을
인덱스로 짝짓는 취약함 없이 한 노드로 image→좌표까지 가는 게 견고하다.

카메라 장착 T_base_cam 은 파라미터(기본값 = 깊이캠 에셋 전방 카메라 실측). 실제
로봇에선 TF 로 받는 게 정석이나, 고정 마운트라 파라미터로 충분하다.

실행 (시스템 ROS 2, cv2 4.5.4):
    ros2 run parkbot_aruco marker_localizer_node
    ros2 run parkbot_aruco marker_localizer_node --ros-args -p fuse:=true
"""

import os

import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image

from parkbot_aruco import aruco_pose as AP
from parkbot_aruco import marker_localizer as ML

os.environ.setdefault("ROS_DOMAIN_ID", "50")

# 깊이캠 에셋 전방 카메라의 base_link→광학 변환(실측). base_link 기준 카메라 마운트.
# 실제 로봇에선 TF(cam_optical→base_link)로 대체 가능. 행 우선 16개.
_DEFAULT_T_BASE_CAM = [
    0.0, -0.5, 0.866025, 0.924,
    -1.0, 0.0, 0.0, 0.0,
    0.0, -0.866025, -0.5, 0.09,
    0.0, 0.0, 0.0, 1.0,
]


class MarkerLocalizerNode(Node):
    def __init__(self):
        super().__init__("marker_localizer_node")

        self.declare_parameter("image_topic", "/image_raw")
        self.declare_parameter("camera_info_topic", "/camera_info")
        self.declare_parameter("marker_map", "")           # 빈 값 → 패키지 표준 지도
        self.declare_parameter("t_base_cam", _DEFAULT_T_BASE_CAM)
        self.declare_parameter("max_reproj_px", 3.0)
        self.declare_parameter("fuse", False)              # 오도메트리 융합(구독 필요)
        self.declare_parameter("log_every", 1)             # 같은 마커 N프레임마다 로그

        image_topic = self.get_parameter("image_topic").value
        info_topic = self.get_parameter("camera_info_topic").value
        map_param = self.get_parameter("marker_map").value
        self.max_reproj = float(self.get_parameter("max_reproj_px").value)
        self.log_every = max(1, int(self.get_parameter("log_every").value))
        self.T_base_cam = np.array(
            self.get_parameter("t_base_cam").value, dtype=np.float64).reshape(4, 4)

        import json
        from pathlib import Path
        map_file = (Path(map_param) if map_param
                    else ML.default_marker_map_path())
        raw = json.loads(map_file.read_text(encoding="utf-8"))
        self.marker_map = ML.MarkerMap.from_json(raw, align_yaw_deg=0.0)  # yaw 포함
        self.code_size = float(raw["code_size_m"])
        self.detector = AP.make_detector(raw["dictionary"])
        self.bridge = CvBridge()
        self.K = None
        self.dist = None
        self._seen_count = {}

        self.create_subscription(
            CameraInfo, info_topic, self._on_info, qos_profile_sensor_data)
        self.create_subscription(
            Image, image_topic, self._on_image, qos_profile_sensor_data)
        self.pub_pose = self.create_publisher(PoseStamped, "/robot_pose", 10)

        self.get_logger().info(
            f"marker_localizer_node 시작 | image={image_topic} info={info_topic} "
            f"| 지도 {len(self.marker_map.by_id)}개 마커 | 카메라 마운트 파라미터 로드")

    def _on_info(self, msg: CameraInfo):
        self.K = np.array(msg.k, dtype=np.float64).reshape(3, 3)
        self.dist = (np.array(msg.d, dtype=np.float64).reshape(-1, 1)
                     if len(msg.d) else np.zeros((5, 1)))

    def _on_image(self, msg: Image):
        if self.K is None:
            self.get_logger().warn("camera_info 대기 중 — 아직 K 없음", once=True)
            return
        import cv2
        img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        poses = AP.detect_and_estimate(
            gray, self.detector, self.code_size, self.K, self.dist)

        for p in poses:
            if p.reproj_err_px > self.max_reproj:
                continue
            if not self.marker_map.has(p.marker_id):
                continue
            T_cm = ML.rvec_tvec_to_T(p.rvec, p.tvec)
            fix = ML.robot_pose_from_marker(
                p.marker_id, T_cm, self.T_base_cam, self.marker_map)
            if fix is None:
                continue

            n = self._seen_count.get(p.marker_id, 0) + 1
            self._seen_count[p.marker_id] = n
            if n % self.log_every == 0:
                m = self.marker_map.by_id[p.marker_id]
                self.get_logger().info(
                    f"[측위] 마커 ID {p.marker_id} ({m['label']})  "
                    f"월드좌표=({m['x']:+.2f}, {m['z']:+.2f})  →  "
                    f"로봇 위치 x={fix.x:+.3f} z={fix.z:+.3f} yaw={fix.yaw_deg:+.1f}°  "
                    f"(재투영 {p.reproj_err_px:.2f}px)")

            ps = PoseStamped()
            ps.header = msg.header
            ps.pose.position.x = fix.x
            ps.pose.position.y = 0.0
            ps.pose.position.z = fix.z
            self.pub_pose.publish(ps)


def main(args=None):
    rclpy.init(args=args)
    node = MarkerLocalizerNode()
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
