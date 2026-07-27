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

import json
import math
import os

import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String

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


def odom_world_delta(prev, cur):
    """직전/현재 odom (x, z, yaw_deg) 로부터 월드 증분 (dx, dz, dyaw_deg) 를 낸다.
    yaw 증분은 (-180, 180] 로 접는다. prev 가 None 이면 None(첫 표본)."""
    if prev is None:
        return None
    lx, lz, lyaw = prev
    x, z, yaw = cur
    dyaw = (yaw - lyaw + 180.0) % 360.0 - 180.0
    return (x - lx, z - lz, dyaw)


class MarkerLocalizerNode(Node):
    def __init__(self):
        super().__init__("marker_localizer_node")

        self.declare_parameter("image_topic", "/image_raw")
        self.declare_parameter("camera_info_topic", "/camera_info")
        self.declare_parameter("marker_map", "")           # 빈 값 → 패키지 표준 지도
        self.declare_parameter("t_base_cam", _DEFAULT_T_BASE_CAM)
        self.declare_parameter("max_reproj_px", 3.0)
        self.declare_parameter("fuse", False)              # 오도메트리 융합(구독 필요)
        self.declare_parameter("odom_topic", "/robot_entry_lead/odom")  # fuse=True 일 때 구독
        self.declare_parameter("log_every", 1)             # 같은 마커 N프레임마다 로그
        self.declare_parameter("frame", "usd")             # usd(기존 호환) | ros_map
        self.declare_parameter("robot_id", "entry_lead")
        self.declare_parameter("pose_topic", "/robot_pose")
        self.declare_parameter("diagnostics_topic", "")

        image_topic = self.get_parameter("image_topic").value
        info_topic = self.get_parameter("camera_info_topic").value
        map_param = self.get_parameter("marker_map").value
        self.max_reproj = float(self.get_parameter("max_reproj_px").value)
        self.fuse = bool(self.get_parameter("fuse").value)
        self.log_every = max(1, int(self.get_parameter("log_every").value))
        self.frame = self.get_parameter("frame").value
        self.robot_id = str(self.get_parameter("robot_id").value)
        self.pose_topic = str(self.get_parameter("pose_topic").value)
        diagnostics_topic = str(
            self.get_parameter("diagnostics_topic").value
        ).strip()
        self.diagnostics_topic = (
            diagnostics_topic
            or f"/robot_{self.robot_id}/vision/alignment"
        )
        self.T_base_cam = np.array(
            self.get_parameter("t_base_cam").value, dtype=np.float64).reshape(4, 4)

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
        self.pub_pose = self.create_publisher(PoseStamped, self.pose_topic, 10)
        self.pub_diagnostics = self.create_publisher(
            String, self.diagnostics_topic, 10
        )

        # fuse=True 면 상보 필터를 만들고 오도메트리를 구독해 예측에 쓴다.
        # fuse=False 면 self.filt 가 None 으로 남아 기존 마커 단독 경로를 그대로 탄다.
        self.filt = None
        self._last_odom = None
        if self.fuse:
            from parkbot_aruco.marker_localizer import PoseFilter
            self.filt = PoseFilter()
            self.create_subscription(Odometry, self.get_parameter("odom_topic").value,
                                     self._on_odom, qos_profile_sensor_data)

        self.get_logger().info(
            f"marker_localizer_node 시작 | image={image_topic} info={info_topic} "
            f"| pose={self.pose_topic} diagnostics={self.diagnostics_topic} "
            f"| 지도 {len(self.marker_map.by_id)}개 마커 | 카메라 마운트 파라미터 로드")

    def _on_info(self, msg: CameraInfo):
        self.K = np.array(msg.k, dtype=np.float64).reshape(3, 3)
        self.dist = (np.array(msg.d, dtype=np.float64).reshape(-1, 1)
                     if len(msg.d) else np.zeros((5, 1)))

    def _on_odom(self, msg):
        # 러너 odom 은 위치를 (x, ·, z), yaw 를 z/w 쿼터니언으로 담는다(XZ 평면).
        x = msg.pose.pose.position.x
        z = msg.pose.pose.position.z
        qz, qw = msg.pose.pose.orientation.z, msg.pose.pose.orientation.w
        cur = (x, z, math.degrees(2.0 * math.atan2(qz, qw)))
        delta = odom_world_delta(self._last_odom, cur)
        if delta is not None and self.filt is not None and self.filt.x is not None:
            self.filt.predict(*delta)
        self._last_odom = cur

    def _on_image(self, msg: Image):
        if self.K is None:
            self.get_logger().warn("camera_info 대기 중 — 아직 K 없음", once=True)
            self._publish_diagnostics(msg, None)
            return
        import cv2
        img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        poses = AP.detect_and_estimate(
            gray, self.detector, self.code_size, self.K, self.dist)

        valid_diagnostics = []
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
            valid_diagnostics.append((p.reproj_err_px, p, fix))

            n = self._seen_count.get(p.marker_id, 0) + 1
            self._seen_count[p.marker_id] = n
            if n % self.log_every == 0:
                m = self.marker_map.by_id[p.marker_id]
                # v4 지도는 "label" 대신 role/serves 스키마라 키가 없다 — 기존
                # MarkerMap.label() 폴백(없으면 id 문자열)을 그대로 재사용한다.
                self.get_logger().info(
                    f"[측위] 마커 ID {p.marker_id} ({self.marker_map.label(p.marker_id)})  "
                    f"월드좌표=({m['x']:+.2f}, {m['z']:+.2f})  →  "
                    f"로봇 위치 x={fix.x:+.3f} z={fix.z:+.3f} yaw={fix.yaw_deg:+.1f}°  "
                    f"(재투영 {p.reproj_err_px:.2f}px)")

            if self.fuse and self.filt is not None:
                if self.filt.x is None:
                    self.filt.set_pose(fix.x, fix.z, fix.yaw_deg)
                else:
                    self.filt.update(fix)
                px, pz, pyaw = self.filt.pose()
            else:
                px, pz, pyaw = fix.x, fix.z, fix.yaw_deg
            # 이후 px,pz,pyaw 로 PoseStamped 발행(기존 x,z,yaw 자리 대체)

            ps = PoseStamped()
            ps.header = msg.header
            if self.frame == "ros_map":
                # 확정 규약: ros_x=usd_x, ros_y=-usd_z, ros_yaw=psi-pi/2.
                # psi=atan2(fwd_x,fwd_z), 부호는 Isaac GT 대조 실측으로 확정했다.
                ps.header.frame_id = "map"
                ps.pose.position.x = px
                ps.pose.position.y = -pz
                ros_yaw = math.radians(pyaw) - math.pi / 2.0
                ps.pose.orientation.z = math.sin(ros_yaw / 2.0)
                ps.pose.orientation.w = math.cos(ros_yaw / 2.0)
            else:
                ps.pose.position.x = px
                ps.pose.position.y = 0.0
                ps.pose.position.z = pz
            self.pub_pose.publish(ps)
        best = min(valid_diagnostics, key=lambda item: item[0], default=None)
        self._publish_diagnostics(msg, best)

    def _publish_diagnostics(self, image_msg, detection) -> None:
        """웹 관제용 최소 ArUco 결과를 표준 String(JSON)으로 발행한다.

        목표점 오차는 이 노드가 임의로 만들지 않는다. 마커 코너·거리·측위값만
        제공하고, 목표를 아는 상위 제어/웹 계층이 필요한 오차를 계산한다.
        """
        payload = {
            "robot_id": self.robot_id,
            "camera": "front",
            "connected": True,
            "detected": detection is not None,
            "frame_width": int(image_msg.width),
            "frame_height": int(image_msg.height),
            "source": "ARUCO_FUSED" if self.fuse else "ARUCO",
        }
        if detection is not None:
            _score, pose, fix = detection
            width = max(1.0, float(image_msg.width))
            height = max(1.0, float(image_msg.height))
            payload.update(
                marker_id=int(pose.marker_id),
                distance_m=float(pose.distance_m),
                reprojection_error_px=float(pose.reproj_err_px),
                ambiguity=float(pose.ambiguity),
                marker_corners=[
                    [float(point[0]) / width, float(point[1]) / height]
                    for point in pose.corners
                ],
                fix_x=float(fix.x),
                fix_z=float(fix.z),
                fix_yaw_deg=float(fix.yaw_deg),
            )
        message = String()
        message.data = json.dumps(payload, ensure_ascii=False)
        self.pub_diagnostics.publish(message)


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
