"""ArUco 바닥 마커 검출 노드 (M2).

Isaac 의 ROS2 브리지가 발행하는 카메라 토픽을 받아 마커를 검출하고 마커별
자세 T_cam_marker 를 발행한다. 실행 환경은 시스템 ROS 2(Humble, python 3.10,
cv2 4.5.4) — Isaac 전용 python 이 아니다. 두 터미널 브리지 구성:

  [터미널 A: Isaac] Isaac python 으로 카메라 퍼블리셔(OmniGraph ROS2CameraHelper)
     export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
     export FASTRTPS_DEFAULT_PROFILES_FILE=$HOME/.ros/fastdds_whitelist.xml
     export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:.../isaacsim.ros2.bridge/humble/lib
  [터미널 B: 이 노드] source /opt/ros/humble/setup.bash + 같은 RMW/프로파일
     ros2 run parkbot_aruco aruco_detector

CV 로직은 parkbot_aruco.aruco_pose 에 있고 여기서는 ROS 배관만 한다.
그 분리 덕에 자세 복원 검증(test_aruco_pose)이 ROS 없이 돌아간다.

출력 메시지: vision_msgs 가 있으면 Detection3DArray(각 검출에 id+자세),
없으면 geometry_msgs/PoseArray + std_msgs/Int32MultiArray(같은 순서의 id) 로
폴백한다. vision_msgs 는 `sudo apt install ros-humble-vision-msgs` 로 설치.
"""

import json
import os
from pathlib import Path

import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Header, Int32MultiArray
from geometry_msgs.msg import Pose, PoseArray

from parkbot_aruco import aruco_pose as AP
from parkbot_aruco.marker_localizer import default_marker_map_path

try:
    from vision_msgs.msg import (
        Detection3D, Detection3DArray, ObjectHypothesisWithPose)
    _HAVE_VISION_MSGS = True
except ImportError:
    _HAVE_VISION_MSGS = False

# marker_map.json 기본 위치 = 이 패키지의 표준 지도(share/ → 소스 data/).
# 지도는 ROS 패키지가 소유한다(실제 배포엔 Isaac 이 없다). 파라미터로 덮어쓸 수 있다.
_DEFAULT_MAP = default_marker_map_path()

os.environ.setdefault("ROS_DOMAIN_ID", "50")


def _rvec_to_quat(rvec):
    """로드리게스 rvec → 쿼터니언 (x, y, z, w)."""
    import cv2
    R, _ = cv2.Rodrigues(np.asarray(rvec, dtype=np.float64))
    t = np.trace(R)
    if t > 0:
        s = 0.5 / np.sqrt(t + 1.0)
        w = 0.25 / s
        x = (R[2, 1] - R[1, 2]) * s
        y = (R[0, 2] - R[2, 0]) * s
        z = (R[1, 0] - R[0, 1]) * s
    else:
        i = int(np.argmax([R[0, 0], R[1, 1], R[2, 2]]))
        if i == 0:
            s = 2.0 * np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
            w = (R[2, 1] - R[1, 2]) / s
            x = 0.25 * s
            y = (R[0, 1] + R[1, 0]) / s
            z = (R[0, 2] + R[2, 0]) / s
        elif i == 1:
            s = 2.0 * np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
            w = (R[0, 2] - R[2, 0]) / s
            x = (R[0, 1] + R[1, 0]) / s
            y = 0.25 * s
            z = (R[1, 2] + R[2, 1]) / s
        else:
            s = 2.0 * np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
            w = (R[1, 0] - R[0, 1]) / s
            x = (R[0, 2] + R[2, 0]) / s
            y = (R[1, 2] + R[2, 1]) / s
            z = 0.25 * s
    return float(x), float(y), float(z), float(w)


class ArucoDetector(Node):
    def __init__(self):
        super().__init__("aruco_detector")

        self.declare_parameter("image_topic", "/image_raw")
        self.declare_parameter("camera_info_topic", "/camera_info")
        self.declare_parameter("marker_map", str(_DEFAULT_MAP))
        self.declare_parameter("max_reproj_px", 3.0)     # 이보다 크면 버린다
        self.declare_parameter("max_ambiguity", 0.6)     # IPPE 2등/1등 비 게이트

        image_topic = self.get_parameter("image_topic").value
        info_topic = self.get_parameter("camera_info_topic").value
        map_path = Path(self.get_parameter("marker_map").value)
        self.max_reproj = float(self.get_parameter("max_reproj_px").value)
        self.max_ambiguity = float(self.get_parameter("max_ambiguity").value)

        mm = json.loads(map_path.read_text(encoding="utf-8"))
        self.code_size = float(mm["code_size_m"])
        self.detector = AP.make_detector(mm["dictionary"])
        self.bridge = CvBridge()
        self.K = None
        self.dist = None
        self.info_frame = "camera"

        self.create_subscription(
            CameraInfo, info_topic, self._on_info, qos_profile_sensor_data)
        self.create_subscription(
            Image, image_topic, self._on_image, qos_profile_sensor_data)

        if _HAVE_VISION_MSGS:
            self.pub = self.create_publisher(
                Detection3DArray, "/aruco/detections", 10)
        else:
            self.get_logger().warn(
                "vision_msgs 없음 → PoseArray+Int32MultiArray 로 폴백합니다. "
                "정식 메시지는 `sudo apt install ros-humble-vision-msgs` 후 재빌드.")
            self.pub = self.create_publisher(PoseArray, "/aruco/poses", 10)
            self.pub_ids = self.create_publisher(
                Int32MultiArray, "/aruco/ids", 10)

        self.get_logger().info(
            f"aruco_detector 시작 | dict={mm['dictionary']} "
            f"code={self.code_size:.4f}m | image={image_topic} info={info_topic} "
            f"| 메시지={'vision_msgs' if _HAVE_VISION_MSGS else 'PoseArray 폴백'}")

    def _on_info(self, msg: CameraInfo):
        self.K = np.array(msg.k, dtype=np.float64).reshape(3, 3)
        self.dist = np.array(msg.d, dtype=np.float64).reshape(-1, 1) \
            if len(msg.d) else np.zeros((5, 1))
        self.info_frame = msg.header.frame_id or self.info_frame

    def _on_image(self, msg: Image):
        if self.K is None:
            self.get_logger().warn("camera_info 대기 중 — 아직 K 없음", once=True)
            return
        import cv2
        img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        poses = AP.detect_and_estimate(
            gray, self.detector, self.code_size, self.K, self.dist)

        header = Header()
        header.stamp = msg.header.stamp
        header.frame_id = msg.header.frame_id or self.info_frame

        kept = []
        for p in poses:
            if p.reproj_err_px > self.max_reproj:
                continue
            # ambiguity 는 IPPE 2등/1등 재투영 비(0~1, 클수록 두 해가 비슷=모호).
            # -1 은 ITERATIVE 폴백(모호성 미상), 0 은 단일해 → 둘 다 통과.
            if p.ambiguity > self.max_ambiguity:
                continue
            kept.append(p)

        self._publish(header, kept)

    def _publish(self, header, poses):
        if _HAVE_VISION_MSGS:
            arr = Detection3DArray(header=header)
            for p in poses:
                det = Detection3D(header=header)
                hyp = ObjectHypothesisWithPose()
                hyp.hypothesis.class_id = str(p.marker_id)
                hyp.hypothesis.score = 1.0
                self._fill_pose(hyp.pose.pose, p)
                det.results.append(hyp)
                self._fill_pose(det.bbox.center, p)
                arr.detections.append(det)
            self.pub.publish(arr)
        else:
            pa = PoseArray(header=header)
            ids = Int32MultiArray()
            for p in poses:
                pose = Pose()
                self._fill_pose(pose, p)
                pa.poses.append(pose)
                ids.data.append(int(p.marker_id))
            self.pub.publish(pa)
            self.pub_ids.publish(ids)

    def _fill_pose(self, pose, p):
        t = p.tvec.flatten()
        pose.position.x, pose.position.y, pose.position.z = \
            float(t[0]), float(t[1]), float(t[2])
        qx, qy, qz, qw = _rvec_to_quat(p.rvec)
        pose.orientation.x, pose.orientation.y = qx, qy
        pose.orientation.z, pose.orientation.w = qz, qw


def main(args=None):
    rclpy.init(args=args)
    node = ArucoDetector()
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
