"""Wrist camera의 /rgb 영상에서 파랑/초록을 판별해 /color_id로 발행한다."""

import os

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge, CvBridgeError
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import Int32


BLUE_ID = 1
GREEN_ID = 2
COLOR_NAMES = {BLUE_ID: "파랑", GREEN_ID: "초록"}

# 사용자가 별도로 지정하지 않은 경우 시나리오의 공통 domain을 사용한다.
os.environ.setdefault("ROS_DOMAIN_ID", "50")


class M0609ColorDetector(Node):
    """HSV 픽셀 수와 연속 프레임을 사용해 잘못된 단발 검출을 줄인다."""

    def __init__(self):
        super().__init__("m0609_color_detector")

        self.declare_parameter("image_topic", "/rgb")
        self.declare_parameter("color_topic", "/color_id")
        self.declare_parameter("roi_scale", 0.55)
        self.declare_parameter("min_pixels", 350)
        self.declare_parameter("stable_frames", 3)

        image_topic = self.get_parameter("image_topic").value
        color_topic = self.get_parameter("color_topic").value
        self.roi_scale = float(self.get_parameter("roi_scale").value)
        self.min_pixels = int(self.get_parameter("min_pixels").value)
        self.stable_frames = int(self.get_parameter("stable_frames").value)

        if not 0.1 <= self.roi_scale <= 1.0:
            raise ValueError("roi_scale은 0.1~1.0 범위여야 합니다.")
        if self.min_pixels < 1 or self.stable_frames < 1:
            raise ValueError("min_pixels와 stable_frames는 1 이상이어야 합니다.")

        self.bridge = CvBridge()
        self.publisher = self.create_publisher(Int32, color_topic, 10)
        self.subscription = self.create_subscription(
            Image,
            image_topic,
            self._image_callback,
            qos_profile_sensor_data,
        )
        self._candidate = None
        self._candidate_count = 0
        self._last_published = None

        self.get_logger().info(
            f"색상 판별 시작: {image_topic} -> {color_topic} "
            f"(파랑={BLUE_ID}, 초록={GREEN_ID})"
        )

    def _center_roi(self, image):
        height, width = image.shape[:2]
        roi_width = max(1, int(width * self.roi_scale))
        roi_height = max(1, int(height * self.roi_scale))
        x0 = (width - roi_width) // 2
        y0 = (height - roi_height) // 2
        return image[y0:y0 + roi_height, x0:x0 + roi_width]

    def _detect(self, bgr_image):
        roi = self._center_roi(bgr_image)
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

        # Isaac Sim 기본 재질의 파랑/초록을 조명 변화에 견디도록 넓게 잡는다.
        blue_mask = cv2.inRange(
            hsv,
            np.array([90, 80, 55], dtype=np.uint8),
            np.array([135, 255, 255], dtype=np.uint8),
        )
        green_mask = cv2.inRange(
            hsv,
            np.array([35, 70, 45], dtype=np.uint8),
            np.array([85, 255, 255], dtype=np.uint8),
        )

        kernel = np.ones((3, 3), dtype=np.uint8)
        blue_mask = cv2.morphologyEx(blue_mask, cv2.MORPH_OPEN, kernel)
        green_mask = cv2.morphologyEx(green_mask, cv2.MORPH_OPEN, kernel)
        counts = {
            BLUE_ID: int(cv2.countNonZero(blue_mask)),
            GREEN_ID: int(cv2.countNonZero(green_mask)),
        }
        color_id = max(counts, key=counts.get)
        if counts[color_id] < self.min_pixels:
            return None, counts
        return color_id, counts

    def _image_callback(self, msg):
        try:
            bgr_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except CvBridgeError as exc:
            self.get_logger().error(f"/rgb 변환 실패: {exc}")
            return

        color_id, counts = self._detect(bgr_image)
        if color_id is None:
            self._candidate = None
            self._candidate_count = 0
            return

        if color_id == self._candidate:
            self._candidate_count += 1
        else:
            self._candidate = color_id
            self._candidate_count = 1

        if self._candidate_count < self.stable_frames:
            return

        self.publisher.publish(Int32(data=color_id))
        if color_id != self._last_published:
            self.get_logger().info(
                f"감지: {COLOR_NAMES[color_id]}({color_id}), "
                f"blue_pixels={counts[BLUE_ID]}, green_pixels={counts[GREEN_ID]}"
            )
            self._last_published = color_id


def main(args=None):
    rclpy.init(args=args)
    node = M0609ColorDetector()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
