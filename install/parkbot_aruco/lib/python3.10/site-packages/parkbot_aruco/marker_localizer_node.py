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
from std_msgs.msg import Float32
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


def rear_t_base_cam_from_front(front_t_base_cam):
    """전방 T_base_cam(행우선 16개 또는 4x4)에서 후방 마운트 기본값을 유도한다.

    저작 근거(`isaacpjt/hwia_parking_robot_final_caster_package/build_rear_camera.py`
    헤더): "cam_rear 의 마운트는 base_link 수직축(Z) 180° 회전 미러 —
    T_rear = Rz(180°)·T_front" (에셋의 **네이티브 Z-up** 로컬 프레임 기준, 링크
    변환에 왼쪽곱으로 적용).

    그런데 `marker_localizer.py`(이 T_base_cam 이 실제 쓰이는 수학 프레임)는
    "스테이지 up=+Y, 회전축 월드 +Y"(모듈 독스트링) 규약이다 — 로봇을 Z-up
    에셋에서 Y-up 스테이지에 배치할 때 쓰는 고정 축변환(`parking_v4_runner.py`
    628행 `AddRotateXOp(-90)`, 즉 X축 둘레 -90°)이 이미 T_base_cam 값 자체에
    녹아 있다. X축 둘레 회전은 회전각을 보존하면서 네이티브 Z축을 정확히 이
    프레임의 Y축으로 보내므로(검산: Rx(-90) 하에서 Z_native→Y_new), 저작
    스크립트의 Rz(180)(네이티브 프레임)는 이 T_base_cam 프레임에서는 Ry(180)
    에 대응한다.

    T_rear = Ry(180) @ T_front (왼쪽곱 = base_link 원점·수직축 둘레로 마운트
    전체를 뒤집는다): 위치 (x,z) 는 부호반전, 높이(y) 는 불변, 카메라가
    바라보는 방향도 180° 돌아 후방을 향한다. 기본값 실측으로 검산: 전방
    t=(0.924, 0, 0.09) → 후방 t=(-0.924, 0, -0.09)(y=0 유지), 전방 광학 Z축
    (0.866,0,-0.5)(+X쪽 약간 하향) → 후방 (-0.866,0,0.5)(-X쪽, 같은 하향각) —
    "뒤를 보되 같은 틸트" 라는 물리적 기대와 일치한다.

    확신도: 축 대응(Z_native→Y_new)은 기하학적으로 검산했으나, 실제 로봇/에셋
    좌표계가 문서와 정확히 일치하는지는 T5 라이브 검증 대상이다 — 어긋나면
    `rear_t_base_cam` 파라미터로 오버라이드한다.
    """
    front = np.asarray(front_t_base_cam, dtype=np.float64).reshape(4, 4)
    ry180 = np.eye(4)
    ry180[:3, :3] = ML.rot_y(180.0)
    rear = ry180 @ front
    return rear.reshape(-1).tolist()


_DEFAULT_REAR_T_BASE_CAM = rear_t_base_cam_from_front(_DEFAULT_T_BASE_CAM)


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
        # seed_pose: fuse=True 필터의 초기 자세 [x, z, yaw_deg]. 비면 시딩 안 함
        # (첫 마커 fix 로 자기시딩=기존 동작, 하위호환). Phase B 처럼 도크 스폰에서
        # 마커를 아직 못 본 상태로 융합주행을 시작해야 할 때, 여기에 스폰 GT 자세를
        # 주면 odom 예측이 첫 fix 전에도 자세를 실어나른다(러너 _mission_setup 이
        # 필터를 도크 좌표로 직접 시딩하던 것의 ROS2 등가물). 마커 fix 가 들어오면
        # correct_yaw 규약대로 보정된다.
        self.declare_parameter(
            "seed_pose", [], ParameterDescriptor(dynamic_typing=True))
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
        # carry 정지·회전 트리거용: ref_marker_dist 를 **이 마커에만** 발행한다.
        # -1(기본)=검출한 아무 ref 마커까지 거리(하위호환). 오케스트레이터가 carry
        # 진입 전 선택 슬롯의 레인마커(A1'=3/A2'=4/A3'=5)로 런타임 설정 → 두 로봇이
        # 그 마커를 등거리로 볼 때만 carry 가 정지·회전한다(엉뚱한 레인마커 오발 방지).
        self.declare_parameter("mdist_marker_id", -1)
        # Task 3b: 이중카메라(전방+후방)-단일필터. rear_image_topic 이 빈
        # 문자열(기본)이면 후방 구독을 아예 만들지 않는다 — 기존 단일카메라
        # 동작이 완전히 그대로 유지된다(하위호환). 채워지면 인프로세스 러너
        # _run_entry_lead_b(같은 filt 을 rear_ctx/front_ctx 로 번갈아 먹임)를
        # ROS2 로 그대로 이식: 후방/전방 두 콜백이 같은 self.filt 를 공유한다.
        self.declare_parameter("rear_image_topic", "")
        self.declare_parameter("rear_camera_info_topic", "")
        self.declare_parameter("rear_t_base_cam", _DEFAULT_REAR_T_BASE_CAM)

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
        self.mdist_marker_id = int(self.get_parameter("mdist_marker_id").value)
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
        # 검출한 ref 마커까지의 거리[m] — carry 가 "양 로봇 같은 거리면 정지·회전" 판정에 씀.
        # 토픽: <robot ns>/ref_marker_dist (pose_topic 에서 유도).
        _ns = self.pose_topic.rsplit('/', 1)[0]
        self.pub_mdist = self.create_publisher(Float32, f'{_ns}/ref_marker_dist', 10)

        # Task 3b: rear_image_topic 이 비어있으면(기본) 후방 구독을 아예 만들지
        # 않는다 — self.rear_enabled=False 로 남고, 아래 K_rear/dist_rear/T_rear
        # 도 없다. 채워지면 후방 CameraInfo/Image 를 추가로 구독해 같은
        # self.filt(전방과 동일 객체) 를 보정한다(_process_frame 공유).
        rear_image_topic = self.get_parameter("rear_image_topic").value
        rear_info_topic = self.get_parameter("rear_camera_info_topic").value
        self.rear_enabled = bool(rear_image_topic)
        self.K_rear = None
        self.dist_rear = None
        if self.rear_enabled:
            self.T_rear = np.array(
                self.get_parameter("rear_t_base_cam").value,
                dtype=np.float64).reshape(4, 4)
            self.create_subscription(
                CameraInfo, rear_info_topic, self._on_info_rear, qos_profile_sensor_data)
            self.create_subscription(
                Image, rear_image_topic, self._on_image_rear, qos_profile_sensor_data)

        # fuse=True 면 상보 필터를 만들고 오도메트리를 구독해 예측에 쓴다.
        # fuse=False 면 self.filt 가 None 으로 남아 기존 마커 단독 경로를 그대로 탄다.
        self.filt = None
        self._last_odom = None
        if self.fuse:
            from parkbot_aruco.marker_localizer import PoseFilter
            self.filt = PoseFilter()
            seed = list(self.get_parameter("seed_pose").value or [])
            if len(seed) == 3:
                self.filt.set_pose(float(seed[0]), float(seed[1]), float(seed[2]))
                self.get_logger().info(
                    f"seed_pose 로 필터 초기화: x={seed[0]:.3f} z={seed[1]:.3f} "
                    f"yaw={seed[2]:.1f}° (첫 fix 전 odom 예측 활성)")
            elif seed:
                self.get_logger().warn(
                    f"seed_pose 는 [x,z,yaw_deg] 3개여야 합니다(받음 {len(seed)}개) — 무시")
            self.create_subscription(Odometry, self.get_parameter("odom_topic").value,
                                     self._on_odom, qos_profile_sensor_data)

        self.get_logger().info(
            f"marker_localizer_node 시작 | image={image_topic} info={info_topic} "
            f"pose_topic={self.pose_topic} | 지도 {len(self.marker_map.by_id)}개 마커 "
            f"| 카메라 마운트 파라미터 로드"
            + (f" | rear_image={rear_image_topic} rear_info={rear_info_topic}"
               if self.rear_enabled else " | 후방캠 비활성(단일카메라)"))

    @staticmethod
    def _parse_camera_info(msg: CameraInfo):
        """CameraInfo → (K 3x3, dist Nx1). 전방/후방 공용(중복 제거)."""
        K = np.array(msg.k, dtype=np.float64).reshape(3, 3)
        dist = (np.array(msg.d, dtype=np.float64).reshape(-1, 1)
                if len(msg.d) else np.zeros((5, 1)))
        return K, dist

    def _on_info(self, msg: CameraInfo):
        self.K, self.dist = self._parse_camera_info(msg)

    def _on_info_rear(self, msg: CameraInfo):
        self.K_rear, self.dist_rear = self._parse_camera_info(msg)

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
            elif p.name == "mdist_marker_id":
                if p.type_ != Parameter.Type.INTEGER:
                    return SetParametersResult(
                        successful=False, reason="mdist_marker_id must be an integer")
                self.mdist_marker_id = int(p.value)
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
        # 융합 자세를 odom 콜백에서도 발행한다(필터 시딩 후). /odom 은 브리지가 매
        # sim 스텝 발행하는 가장 안정적인 신호라(카메라 렌더와 무관), 저RTF 에서
        # 카메라 프레임이 드물어져도 /pose 피드가 끊기지 않는다 — 소비자
        # (pose_controller)의 pose_stale_timeout(자세 공백 시 goal abort)을 막는다.
        # 마커 보정은 여전히 _process_frame 에서 일어나고, 여기선 그 최신 필터
        # 자세(마커보정+odom예측)를 재발행할 뿐이다.
        if self.filt is not None and self.filt.x is not None:
            px, pz, pyaw = self.filt.pose()
            self._publish_pose(px, pz, pyaw, msg.header)

    def _on_image(self, msg: Image):
        if self.K is None:
            self.get_logger().warn("camera_info 대기 중 — 아직 K 없음", once=True)
            return
        self._process_frame(msg, self.K, self.dist, self.T_base_cam, cam='front')

    def _on_image_rear(self, msg: Image):
        if self.K_rear is None:
            self.get_logger().warn(
                "후방 camera_info 대기 중 — 아직 K_rear 없음", once=True)
            return
        self._process_frame(msg, self.K_rear, self.dist_rear, self.T_rear, cam='rear')

    def _process_frame(self, msg: Image, K, dist, t_base_cam, cam='front'):
        """전방/후방 공용 파이프라인: 검출→ref 필터→마커별 fix→filt 공유 보정→발행.
        ``cam``('front'/'rear')은 어느 카메라가 이 프레임을 냈는지 로그 표시용.

        Task 3b: 인프로세스 러너 `_run_entry_lead_b` 가 하나의 `filt` 을
        `rear_ctx`/`front_ctx` 로 번갈아 먹이는 것의 ROS2 이식 — `_on_image`/
        `_on_image_rear` 둘 다 이 함수를 호출하고, 둘 다 같은 `self.filt` 을
        보정한다(카메라별로 다른 건 인자로 받는 K/dist/t_base_cam 뿐).
        T1 의 `filter_detections_by_ref`/`apply_fix`(ref_ids, correct_yaw) 를
        그대로 재사용하므로 후방에도 동일하게 적용된다.
        """
        import cv2
        img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        poses = AP.detect_and_estimate(
            gray, self.detector, self.code_size, K, dist)
        poses = filter_detections_by_ref(poses, self.ref_ids)

        for p in poses:
            if p.reproj_err_px > self.max_reproj:
                continue
            if not self.marker_map.has(p.marker_id):
                continue
            T_cm = ML.rvec_tvec_to_T(p.rvec, p.tvec)
            fix = ML.robot_pose_from_marker(
                p.marker_id, T_cm, t_base_cam, self.marker_map)
            if fix is None:
                continue

            # 카메라→마커 거리(tvec 노름). carry 의 "양 로봇 같은 거리" 정지판정용.
            # mdist_marker_id 가 지정되면 그 타깃 마커에만 발행(엉뚱한 레인마커 오발 방지).
            if self.mdist_marker_id < 0 or int(p.marker_id) == self.mdist_marker_id:
                self.pub_mdist.publish(Float32(data=float(np.linalg.norm(p.tvec))))

            n = self._seen_count.get(p.marker_id, 0) + 1
            self._seen_count[p.marker_id] = n
            if n % self.log_every == 0:
                m = self.marker_map.by_id[p.marker_id]
                # v4 지도는 "label" 대신 role/serves 스키마라 키가 없다 — 기존
                # MarkerMap.label() 폴백(없으면 id 문자열)을 그대로 재사용한다.
                self.get_logger().info(
                    f"[측위][{cam}캠] 마커 ID {p.marker_id} ({self.marker_map.label(p.marker_id)})  "
                    f"월드좌표=({m['x']:+.2f}, {m['z']:+.2f})  →  "
                    f"로봇 위치 x={fix.x:+.3f} z={fix.z:+.3f} yaw={fix.yaw_deg:+.1f}°  "
                    f"거리 {float(np.linalg.norm(p.tvec)):.2f}m (재투영 {p.reproj_err_px:.2f}px)")

            if self.fuse and self.filt is not None:
                # 융합: 이 프레임의 마커로 공유 필터를 보정한다(발행은 루프 밖에서
                # 프레임당 1회 — 마커 사각 구간에서도 끊김 없이 자세를 내보내기 위함).
                if self.filt.x is None:
                    self.filt.set_pose(fix.x, fix.z, fix.yaw_deg)
                else:
                    apply_fix(self.filt, fix, self.correct_yaw)
            else:
                # 마커 단독(비융합): 마커별 raw fix 를 그대로 발행(기존 동작 유지).
                self._publish_pose(fix.x, fix.z, fix.yaw_deg, msg.header)

        # 융합 모드: 이 프레임에 매칭 마커를 봤든(위에서 보정) 아니든, 필터가
        # 시딩돼 있으면 현재 자세(odom 예측 + 그간의 마커 보정)를 프레임마다
        # 발행한다. 소비자(pose_controller)가 도크/XN 마커가 아직 안 보이는
        # 초입에서도 끊김 없는 피드백을 받아 stale_pose 로 중단되지 않게 한다
        # (seed_pose 로 시딩한 Phase B dock_check 초입 stale_pose 버그 수정).
        if self.fuse and self.filt is not None and self.filt.x is not None:
            px, pz, pyaw = self.filt.pose()
            self._publish_pose(px, pz, pyaw, msg.header)

    def _publish_pose(self, px, pz, pyaw, header):
        """(x, z, yaw_deg) 를 frame 규약대로 PoseStamped 로 발행(전방/후방/융합 공용)."""
        ps = PoseStamped()
        ps.header = header
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
            # 이 프로젝트 yaw(yaw=0→+Z)를 메시지의 z축 회전 슬롯에 그대로 인코딩
            # (R2/R3c 규약 — pose_controller_node.odom_quat_to_yaw_deg 와 동일
            # 인코딩이라 Odometry/PoseStamped 양쪽에 그대로 재사용된다).
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
