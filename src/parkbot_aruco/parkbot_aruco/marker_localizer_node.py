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

import math
import os

import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import ParameterDescriptor, SetParametersResult
from rclpy.node import Node
from rclpy.parameter import Parameter
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


def filter_detections_by_ref(dets, ref_ids):
    """검출 리스트를 ref_ids 로 하드필터한다.

    러너 fuse_camera_setup 의 `hit = [p for p in det if int(p.marker_id) ==
    ctx["ref_id"]]`(parking_v4_runner.py 960·974행)와 동일한 필터를, 단일 id
    가 아니라 **집합**(여러 id 허용)으로 일반화한다. ref_ids 가 빈 리스트면
    필터를 걸지 않고 dets 를 그대로 돌려준다(하위호환 — 기본값).
    """
    if not ref_ids:
        return dets
    ref_set = {int(r) for r in ref_ids}
    return [d for d in dets if int(d.marker_id) in ref_set]


def apply_fix(filt, fix, correct_yaw):
    """마커 관측 fix 로 filt 를 보정한다.

    correct_yaw=True 면 기존 PoseFilter.update(fix)(위치+yaw 블렌드).
    correct_yaw=False 면 PoseFilter.update_position_only(fix) — 위치만
    pos_gain 으로 블렌드하고 yaw 는 오도(predict) 값을 그대로 지킨다(러너
    drive_to_pose 의 `_apply_fix` correct_yaw=False 분기와 동일).
    """
    if correct_yaw:
        filt.update(fix)
    else:
        filt.update_position_only(fix)


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
        # R3c: 다중 로봇이 각자 노드 인스턴스를 띄울 때 서로 다른 /robot_pose 를
        # 내야 하므로 토픽명을 파라미터화한다(기본값은 기존 하드코딩 값 그대로 —
        # 기존 호출부와 하위호환). 예: -p pose_topic:=/robot_entry_lead/pose
        self.declare_parameter("pose_topic", "/robot_pose")
        # Phase B(도크→XN 융합주행): 러너 ctx["ref_id"] 하드필터의 ROS2 이식.
        # 빈 리스트=필터 없음(전체 검출 사용, 기존 동작과 동일=하위호환). T3
        # 오케스트레이터가 set_parameters 로 도크(21/23)→XN(31) 런타임 전환.
        # dynamic_typing=True 필수: 빈 리스트 기본값을 그냥 선언하면 rclpy 가
        # 타입을 BYTE_ARRAY 로 굳혀버려서(빈 배열에서 타입 추론), 이후 T3 가
        # 정수 배열로 set_parameters 하면 우리 콜백에 닿기도 전에 rclpy 자체
        # 타입검사에서 거부된다 — dynamic_typing 으로 그 고정을 막는다.
        self.declare_parameter(
            "ref_ids", [], ParameterDescriptor(dynamic_typing=True))
        # 러너 drive_to_pose `_apply_fix` correct_yaw=False(위치전용 보정)의
        # ROS2 이식. 기본 True=기존 filt.update(fix) 그대로(하위호환).
        self.declare_parameter("correct_yaw", True)

        image_topic = self.get_parameter("image_topic").value
        info_topic = self.get_parameter("camera_info_topic").value
        map_param = self.get_parameter("marker_map").value
        self.max_reproj = float(self.get_parameter("max_reproj_px").value)
        self.fuse = bool(self.get_parameter("fuse").value)
        self.log_every = max(1, int(self.get_parameter("log_every").value))
        self.frame = self.get_parameter("frame").value
        self.T_base_cam = np.array(
            self.get_parameter("t_base_cam").value, dtype=np.float64).reshape(4, 4)
        self.ref_ids = list(self.get_parameter("ref_ids").value)
        self.correct_yaw = bool(self.get_parameter("correct_yaw").value)
        self.add_on_set_parameters_callback(self._on_set_parameters)

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
        self.pose_topic = self.get_parameter("pose_topic").value
        self.pub_pose = self.create_publisher(PoseStamped, self.pose_topic, 10)

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
            f"pose_topic={self.pose_topic} | 지도 {len(self.marker_map.by_id)}개 마커 "
            f"| 카메라 마운트 파라미터 로드")

    def _on_info(self, msg: CameraInfo):
        self.K = np.array(msg.k, dtype=np.float64).reshape(3, 3)
        self.dist = (np.array(msg.d, dtype=np.float64).reshape(-1, 1)
                     if len(msg.d) else np.zeros((5, 1)))

    def _on_set_parameters(self, params):
        """`ref_ids`/`correct_yaw` 런타임 갱신(T3 오케스트레이터의 set_parameters 진입점).

        타입이 어긋나거나(문자열·실수 등) 값이 int/bool 로 못 바뀌면 거부하고
        기존 self.ref_ids/self.correct_yaw 는 그대로 둔다 — 잘못된 값이 들어와도
        노드가 죽거나 필터가 조용히 깨지지 않게 한다.
        """
        for p in params:
            if p.name == "ref_ids":
                if p.type_ != Parameter.Type.INTEGER_ARRAY:
                    return SetParametersResult(
                        successful=False, reason="ref_ids must be an integer array")
                try:
                    new_ref_ids = [int(v) for v in p.value]
                except (TypeError, ValueError):
                    return SetParametersResult(
                        successful=False, reason="ref_ids must contain integers")
                self.ref_ids = new_ref_ids
            elif p.name == "correct_yaw":
                if p.type_ != Parameter.Type.BOOL:
                    return SetParametersResult(
                        successful=False, reason="correct_yaw must be a bool")
                self.correct_yaw = bool(p.value)
        return SetParametersResult(successful=True)

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
            return
        import cv2
        img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        poses = AP.detect_and_estimate(
            gray, self.detector, self.code_size, self.K, self.dist)
        poses = filter_detections_by_ref(poses, self.ref_ids)

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
                    apply_fix(self.filt, fix, self.correct_yaw)
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
                # R3c 수정: 이전에는 이 usd 분기가 orientation 을 전혀 채우지
                # 않아(geometry_msgs 기본값 x=y=z=0,w=1) yaw 가 항상 0 으로
                # 발행됐다(pose_controller_node 같은 소비자가 회전 피드백을
                # 아예 못 받는 버그). /robot_<id>/odom 과 같은 규약(R2 계약,
                # parking_v4_runner.publish_odom): 이 프로젝트 yaw(yaw=0→+Z)를
                # 메시지의 z축 회전 슬롯에 그대로 인코딩한다 — 그래야
                # pose_controller_node.odom_quat_to_yaw_deg 를 Odometry/
                # PoseStamped 양쪽에 그대로 재사용할 수 있다(같은 인코딩).
                yaw_rad = math.radians(pyaw)
                ps.pose.orientation.x = 0.0
                ps.pose.orientation.y = 0.0
                ps.pose.orientation.z = math.sin(yaw_rad * 0.5)
                ps.pose.orientation.w = math.cos(yaw_rad * 0.5)
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
