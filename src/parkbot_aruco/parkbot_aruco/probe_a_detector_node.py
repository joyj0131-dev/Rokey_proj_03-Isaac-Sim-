#!/usr/bin/env python3
"""Probe A 외부 검출 노드 (순수 ROS 2, 시스템 cv2).

Isaac 러너(parking_v4_runner.py --probe=A)가 발행하는 카메라 영상을 받아 ArUco 마커를
검출하고, /probe_a/state 로 전달되는 "현재 거리 + 대상 마커 id"에 맞춰 각 거리에서
마커를 인식했는지 보고한다.

핵심: 검출을 Isaac **밖에서** 한다. 실제 배포에는 Isaac 이 없고, 외부 노드는 시스템
python 3.10 + 시스템 cv2(예: 4.5.4)로 돈다. 그 경로를 그대로 시험한다. cv2 4.5.4 의
IPPE_SQUARE 버그는 aruco_pose 가 ITERATIVE 폴백으로 흡수한다.

실행: run_probe_a_detector.sh   (도메인 126 + fastdds 화이트리스트가 세팅된다)
Isaac 러너: bash parking_v4_runner.sh --gui --probe=A --cam-height=0.15
"""
import json

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import String

from parkbot_aruco import aruco_pose
from parkbot_aruco.marker_localizer import default_marker_map_path


def image_to_gray(msg: Image) -> np.ndarray:
    """sensor_msgs/Image -> HxW uint8 그레이. rgb8/bgr8/mono8 지원."""
    h, w = msg.height, msg.width
    buf = np.frombuffer(bytes(msg.data), dtype=np.uint8)
    if msg.encoding == "mono8":
        return np.ascontiguousarray(buf.reshape(h, w))
    img = buf.reshape(h, w, -1)[:, :, :3]
    code = cv2.COLOR_RGB2GRAY if msg.encoding == "rgb8" else cv2.COLOR_BGR2GRAY
    return cv2.cvtColor(np.ascontiguousarray(img), code)


class ProbeADetector(Node):
    def __init__(self):
        super().__init__("probe_a_detector")
        self.declare_parameter("image_topic", "/robot_entry_lead/image_raw")
        self.declare_parameter("camera_info_topic", "/robot_entry_lead/camera_info")
        self.declare_parameter("state_topic", "/probe_a/state")
        img_t = self.get_parameter("image_topic").value
        info_t = self.get_parameter("camera_info_topic").value
        state_t = self.get_parameter("state_topic").value

        mm = json.loads(default_marker_map_path().read_text(encoding="utf-8"))
        self.dict_name = mm["dictionary"]
        self.code_size = float(mm["code_size_m"])
        self.detector = aruco_pose.make_detector(self.dict_name)
        self.K = None
        self.dist = np.zeros((5, 1), dtype=np.float64)
        self.state = None
        self.results = {}          # distance_m -> {"detected", "reproj", "ids"}
        self._last_logged_d = None

        self.create_subscription(CameraInfo, info_t, self._on_info, 10)
        self.create_subscription(String, state_t, self._on_state, 10)
        self.create_subscription(Image, img_t, self._on_image, 10)
        self.get_logger().info(
            f"probe_a_detector 시작: image={img_t} info={info_t} state={state_t}. "
            f"사전={self.dict_name} code_size={self.code_size:.4f}m. "
            f"camera_info 와 /probe_a/state 를 기다립니다…")

    def _on_info(self, msg: CameraInfo):
        if self.K is None:
            self.K = np.array(msg.k, dtype=np.float64).reshape(3, 3)
            self.get_logger().info("camera_info 수신 — 검출을 시작합니다.")

    def _on_state(self, msg: String):
        try:
            self.state = json.loads(msg.data)
        except (json.JSONDecodeError, TypeError):
            self.state = None

    def _on_image(self, msg: Image):
        if self.K is None or self.state is None:
            return
        if self.state.get("phase") != "sweep":
            return
        target_id = int(self.state["marker_id"])
        d = round(float(self.state["distance_m"]), 3)

        gray = image_to_gray(msg)
        det = aruco_pose.detect_and_estimate(gray, self.detector, self.code_size,
                                             self.K, self.dist)
        hit = [p for p in det if int(p.marker_id) == target_id]
        ok = len(hit) > 0
        reproj = min((p.reproj_err_px for p in hit), default=float("nan"))
        ids = sorted(int(p.marker_id) for p in det)

        # 한 거리에서 한 번이라도 인식하면 '검출'로 승격한다(단발 미검출에 흔들리지 않게).
        prev = self.results.get(d)
        if prev is None or (ok and not prev["detected"]):
            self.results[d] = {"detected": ok, "reproj": reproj, "ids": ids}

        if self._last_logged_d != d:
            self._last_logged_d = d
            mark = "O" if ok else "X"
            extra = f"  reproj={reproj:.2f}px" if ok and reproj == reproj else ""
            self.get_logger().info(
                f"거리 {d:.2f}m → 마커 {target_id} 인식 {mark}  "
                f"(프레임 내 검출 id={ids}){extra}")

    def summary(self):
        hits = sorted(d for d, r in self.results.items() if r["detected"])
        if not hits:
            self.get_logger().warn(
                "인식된 거리가 없습니다. 러너가 --probe=A 로 영상을 발행 중인지, "
                "도메인/화이트리스트가 맞는지 확인하세요.")
            return
        d_min, d_max = min(hits), max(hits)
        self.get_logger().info(
            f"=== 검출 창: d_min={d_min:.2f}m  d_max={d_max:.2f}m  "
            f"window={d_max - d_min:.2f}m  "
            f"(인식 {len(hits)}/{len(self.results)} 지점) ===")


def main():
    try:
        from rclpy.executors import ExternalShutdownException
    except ImportError:                       # 구버전 호환
        ExternalShutdownException = ()
    rclpy.init()
    node = ProbeADetector()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.summary()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
