#!/usr/bin/env python3
"""axle_detector_node — R4: 측면 뎁스캠(ROS2) -> 축(axle) 중심 검출.

설계서 ``docs/superpowers/specs/2026-07-25-ros2-node-refactor-design.md`` §3/4(R4).
R2 브리지(``parking_v4_runner.sh --bridge``, taskR2-report.md)가
``attach_camera_graph(..., image_type="depth")``(Task R4)로 OmniGraph 를 통해
발행하는 좌/우 측면 뎁스 이미지(``/robot_<id>/left/depth``,
``/robot_<id>/right/depth`` — ``sensor_msgs/Image``, encoding ``32FC1``, 픽셀당
미터)를 구독해, 러너의 인프로세스 ``_ingress_axle``(3327행 부근)가 쓰던 것과
**같은 알고리즘**(ROI-min 결합 -> ``TroughTracker``)을 ROS2 노드로 옮긴 것이다.

## 아키텍처 — 왜 포즈 콜백이 아니라 "최신값 캐시" 인가

``pose_controller_node``(R3b/c)는 포즈 콜백 하나가 "관측·결정·명령"을 전부
담당했다(포즈가 유일한 입력이었으므로). 이 노드는 입력이 **셋**이다(좌뎁스,
우뎁스, 자세) — 세 토픽이 서로 다른 주기/지연으로 도착한다(좌우는 각자
OmniGraph 렌더 파이프라인, 자세는 오도/마커융합 노드). 인프로세스 원본은
단일 프로세스 동기 루프(``app.update()`` 한 번에 셋 다 같이 최신화)였지만
ROS2 에선 그 동기성이 없다 — 그래서 좌/우 뎁스는 각자의 구독 콜백이 "최신
ROI-min" 캐시(``self._left_min``/``self._right_min``)만 갱신하고, **자세
콜백이 도착할 때마다** 그 시점의 최신 캐시값을 ``combine_side_depths()`` 로
합쳐 ``TroughTracker.update(travel_pos, combined)`` 를 한 번 호출한다. 자세
토픽을 "틱"으로 고른 이유: 트로프 검출은 결국 "이 위치에서 뎁스가 얼마였나"
쌍이 필요하고, 위치가 있어야 트로프 중간점이 의미가 있다 — 뎁스만 오고
위치가 없으면 애초에 아무것도 못 한다. (참고: ``pose_controller_node`` 도
동일한 "가장 정보가 필요한 토픽을 틱으로 쓴다" 설계를 이미 썼다.)

## 좌우 결합 — 러너 ``_ingress_axle``(3373-3385행)와 동일 규칙

둘 다 유효하면 평균(``0.5*(left+right)``), 한쪽만 유효하면 그 값, 둘 다
무효면 ``math.inf``(신호 없음, ``TroughTracker`` 가 baseline 학습에서
스킵하고 트로프 판정에도 걸리지 않는다). ``combine_side_depths()`` 로 뽑아
단위테스트했다.

## 주행좌표 — 이 노드는 "world x" 를 쓴다(가정, 문서화)

러너의 실측 벤치마크(taskC2fix-report.md §4: rear axle x≈-6.569, front axle
x≈-10.163)와 인프로세스 ``_ingress_axle`` 모두 트럭 통과 주행축을 world
**x** 로 다룬다(entry_lead/entry_follow 가 도크 아래를 지나가는 방향).
그래서 이 노드는 자세 메시지의 ``position.x``(``odom``/``PoseStamped`` 공용
필드 경로는 ``pose_controller_node`` 의 ``_on_odom``/``_on_pose_stamped`` 와
동일)를 travel_pos 로 쓴다. 다른 주행축(z)을 쓰는 배치가 생기면 이 가정을
파라미터화해야 한다 — 지금은 이 미션 하나만 지원한다(정직하게 기록).

## 자세 소스 — odom(기본) 또는 마커융합 PoseStamped

``pose_topic``/``pose_msg_type`` 파라미터는 ``pose_controller_node`` 와 같은
계약(``resolve_pose_msg_type``, ``odom_quat_to_yaw_deg`` 재사용)이다. 기본값은
``/robot_<id>/odom``(휠 오도, YAW_ODOM_SCALE 보정 미적용 상태 그대로 브리지가
발행 — 트럭 밑에는 도크 마커가 없어 마커융합 자체가 사실상 순수 오도와
같다, 러너 ``_ingress_axle`` 주석과 동일한 이유) — 마커가 실제로 보이는
구간(도크 인근)에서는 ``/robot_<id>/pose``(마커융합) 로 바꿔 쓸 수 있다.

## 하드 요구사항(브리프 그대로) — 트로프 완료 시점 = 이미 축을 지나친 뒤

``TroughTracker``(axle_center.py)는 "진입 위치와 이탈 위치의 중간값"으로
축 중심을 낸다 — **이탈이 확정돼야 트로프가 완료**되므로, 이 노드가
``/robot_<id>/axle_center`` 를 발행하는 시점에는 로봇이 이미 그 축을
지나쳐 있다(설계서 C3 지식, R4 브리프가 재확인하라고 명시). 이 노드는
"돌아가서 정지"하지 않는다 — 그건 소비자(R5, 오케스트레이터/제어 노드)의
몫이다. 이 노드의 책임은 오직 "축 중심 x 좌표를 정확히 보고"까지다.

## 메시지 계약 (설계 그대로 문서화)

- ``/robot_<id>/axle_center`` (``geometry_msgs/PointStamped``): 트로프가
  완료될 때마다 1개씩 발행. ``point.x`` = 축 중심의 world x[m](§ 위 "주행좌표"
  가정). ``point.y``/``point.z`` 는 **쓰지 않음**(항상 0.0 — 이 노드는 좌우
  방향(z) 이나 높이 정보를 갖고 있지 않다, 있는 값을 억지로 채우지 않는다).
  ``header.frame_id="map"``(R2 odom 계약과 동일 규약).
- ``/robot_<id>/axle_index`` (``std_msgs/Int32``): 같은 콜백에서 **바로 뒤에**
  발행하는 0-기반 완료 순번(``len(tracker.troughs)-1`` 이 트로프 완료
  당시 값). 헤더가 없는 메시지라 별도 상관 필드가 없다 — 소비자는 "이
  토픽의 k 번째 수신 메시지"와 "``axle_center`` 토픽의 k 번째 수신 메시지"를
  같은 트로프로 짝짓는다(둘 다 이 노드의 같은 콜백 안에서, 항상 index 먼저,
  center 나중에, 순서를 바꾸지 않고 발행 — DDS 는 같은 발행자·구독자 쌍
  안에서 발행 순서를 보존한다).
"""
import math

import numpy as np
import rclpy
from geometry_msgs.msg import PointStamped, PoseStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import Int32

from parkbot_motion.axle_center import TroughTracker
from parkbot_motion.depth_stop_detector import roi_min_depth
from parkbot_motion.pose_controller_node import odom_quat_to_yaw_deg, resolve_pose_msg_type

# 러너의 실측 작동값(taskC2fix-report.md §2.3 DEPTH_ROI_FRAC 주석) — 이 카메라
# 자산(수직FOV 65°, 지상고 0.16m)에서 바닥을 피하려면 depth_stop_detector 의
# 일반 기본값(DEFAULT_ROI_FRAC=(0.30,0.70,0.50,1.00))이 아니라 이 좁은 행
# 범위가 필요하다(브리프 지시: "ROI fraction(default = the runner's working
# DEPTH_ROI_FRAC)"). depth_stop_detector.DEFAULT_ROI_FRAC 자체는 건드리지
# 않는다(다른 자산/용도에 쓰일 수 있는 범용 기본값이므로) — 이 노드의 파라미터
# 기본값만 러너 실측값으로 덮어쓴다.
DEPTH_ROI_FRAC = (0.30, 0.70, 0.48, 0.58)


def depth_image_to_array(height, width, step, data, encoding="32FC1"):
    """``sensor_msgs/Image``(단채널 float32, encoding "32FC1") 필드 -> (h,w) ndarray.

    순수 함수(메시지 객체가 아니라 낱개 필드를 받는다) — 노드 콜백은
    ``depth_image_to_array(msg.height, msg.width, msg.step, msg.data)`` 로
    호출한다. ROI-min 계산(``depth_stop_detector.roi_min_depth``)이 기대하는
    (height, width) 2D 배열을 낸다.

    ``step``(행 바이트 스트라이드)이 ``width*4`` 보다 크면(정렬 패딩) 여분
    바이트를 버린다 — 패딩이 있는 스트라이드를 무시하고 그냥 통짜로
    reshape 하면 프레임이 행마다 옆으로 밀려 잘못된 배열이 나온다.
    """
    if encoding != "32FC1":
        raise ValueError(f"depth_image_to_array 는 encoding='32FC1' 만 지원합니다: {encoding!r}")
    flat = np.frombuffer(bytes(data), dtype=np.float32)
    cols_per_row = step // 4
    expected = cols_per_row * height
    if flat.size < expected:
        raise ValueError(
            f"depth 데이터 길이 부족: got {flat.size} floats, need {expected} "
            f"(height={height}, step={step})")
    grid = flat[:expected].reshape((height, cols_per_row))
    return grid[:, :width]


def combine_side_depths(left, right):
    """좌/우 ROI-min 뎁스를 하나의 신호로 결합.

    러너 ``_ingress_axle``(parking_v4_runner.py, 3373-3385행)와 동일 규칙:
    둘 다 유효(``None`` 아님)하면 평균, 한쪽만 유효하면 그 값, 둘 다 없으면
    ``math.inf``(신호 없음 — baseline 학습 스킵, 트로프 판정도 걸리지 않음).
    """
    if left is not None and right is not None:
        return 0.5 * (left + right)
    if left is not None:
        return left
    if right is not None:
        return right
    return math.inf


class AxleDetectorNode(Node):
    def __init__(self):
        super().__init__('axle_detector_node')

        self.declare_parameter('robot_id', 'entry_lead')
        robot_id = self.get_parameter('robot_id').value

        self.declare_parameter('left_depth_topic', f'/robot_{robot_id}/left/depth')
        self.declare_parameter('right_depth_topic', f'/robot_{robot_id}/right/depth')
        self.declare_parameter('pose_topic', f'/robot_{robot_id}/odom')
        self.declare_parameter('pose_msg_type', 'auto')
        self.declare_parameter('axle_center_topic', f'/robot_{robot_id}/axle_center')
        self.declare_parameter('axle_index_topic', f'/robot_{robot_id}/axle_index')

        # TroughTracker 파라미터 — 기본값은 러너 인프로세스 _ingress_axle 이 쓰는
        # 실측 검증값 그대로(3342행: baseline_frames=30, drop_margin=0.05,
        # confirm_frames=3).
        self.declare_parameter('roi_frac', list(DEPTH_ROI_FRAC))
        self.declare_parameter('baseline_frames', 30)
        self.declare_parameter('drop_margin', 0.05)
        self.declare_parameter('confirm_frames', 3)

        gp = self.get_parameter
        self.robot_id = robot_id
        self.left_depth_topic = gp('left_depth_topic').value
        self.right_depth_topic = gp('right_depth_topic').value
        self.pose_topic = gp('pose_topic').value
        pose_msg_type_param = str(gp('pose_msg_type').value)
        self.pose_msg_type, pose_msg_type_how = resolve_pose_msg_type(
            pose_msg_type_param, self.pose_topic, self.get_topic_names_and_types())
        self.axle_center_topic = gp('axle_center_topic').value
        self.axle_index_topic = gp('axle_index_topic').value
        self.roi_frac = tuple(float(v) for v in gp('roi_frac').value)
        baseline_frames = int(gp('baseline_frames').value)
        drop_margin = float(gp('drop_margin').value)
        confirm_frames = int(gp('confirm_frames').value)

        self._left_min = None
        self._right_min = None
        self.tracker = TroughTracker(baseline_frames=baseline_frames, drop_margin=drop_margin,
                                      confirm_frames=confirm_frames)

        self.create_subscription(Image, self.left_depth_topic, self._on_left_depth,
                                  qos_profile_sensor_data)
        self.create_subscription(Image, self.right_depth_topic, self._on_right_depth,
                                  qos_profile_sensor_data)
        if self.pose_msg_type == 'odometry':
            self.create_subscription(Odometry, self.pose_topic, self._on_odom, 10)
        else:
            self.create_subscription(PoseStamped, self.pose_topic, self._on_pose_stamped, 10)

        self._center_pub = self.create_publisher(PointStamped, self.axle_center_topic, 10)
        self._index_pub = self.create_publisher(Int32, self.axle_index_topic, 10)

        self.get_logger().info(
            f'axle_detector_node 시작: robot_id={robot_id} '
            f'left={self.left_depth_topic} right={self.right_depth_topic} '
            f'pose_topic={self.pose_topic} pose_msg_type={self.pose_msg_type}'
            f'({pose_msg_type_how}) roi_frac={self.roi_frac} '
            f'baseline_frames={baseline_frames} drop_margin={drop_margin} '
            f'confirm_frames={confirm_frames} -> '
            f'{self.axle_center_topic} / {self.axle_index_topic}')

    # ---- 뎁스 구독 콜백: 최신 ROI-min 캐시만 갱신 ----

    def _on_left_depth(self, msg):
        self._left_min = self._roi_min(msg)

    def _on_right_depth(self, msg):
        self._right_min = self._roi_min(msg)

    def _roi_min(self, msg):
        try:
            grid = depth_image_to_array(msg.height, msg.width, msg.step, msg.data,
                                         encoding=msg.encoding)
        except ValueError as exc:
            self.get_logger().warn(f'뎁스 프레임 파싱 실패, 이 프레임 스킵: {exc}')
            return None
        value = roi_min_depth(grid, roi_frac=self.roi_frac)
        return value if math.isfinite(value) else None

    # ---- 자세 구독 콜백: 이 노드의 "틱" — 캐시된 좌/우 뎁스를 결합해 트래커 갱신 ----

    def _on_odom(self, msg):
        p = msg.pose.pose.position
        self._handle_pose(p.x)

    def _on_pose_stamped(self, msg):
        p = msg.pose.position
        self._handle_pose(p.x)

    def _handle_pose(self, x):
        travel_pos = float(x)
        combined = combine_side_depths(self._left_min, self._right_min)
        n_before = len(self.tracker.troughs)
        self.tracker.update(travel_pos, combined)
        for i in range(n_before, len(self.tracker.troughs)):
            self._publish_trough(i)

    def _publish_trough(self, index):
        center = self.tracker.troughs[index]['center']
        idx_msg = Int32()
        idx_msg.data = int(index)
        self._index_pub.publish(idx_msg)

        pt = PointStamped()
        pt.header.stamp = self.get_clock().now().to_msg()
        pt.header.frame_id = 'map'
        pt.point.x = float(center)
        pt.point.y = 0.0
        pt.point.z = 0.0
        self._center_pub.publish(pt)

        self.get_logger().info(
            f'axle_center: index={index} center_x={center:.4f} '
            f"(enter={self.tracker.troughs[index]['enter']:.4f}, "
            f"exit={self.tracker.troughs[index]['exit']:.4f})")


def main(args=None):
    rclpy.init(args=args)
    node = AxleDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
