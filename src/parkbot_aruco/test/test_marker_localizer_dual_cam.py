"""marker_localizer_node 이중카메라(전방+후방)-단일필터 회귀 테스트. (Task 3b)

인프로세스 러너 `_run_entry_lead_b`(parking_v4_runner.py)는 하나의 공유
`filt` 을 후방캠(도크 마커 검출)과 전방캠(XN 마커 검출)이 번갈아 먹인다
(`drive_to_pose` 에 같은 `filt` 을 `rear_ctx`/`front_ctx` 로 넘김). 이 테스트는
ROS2 `MarkerLocalizerNode` 가 그 구조를 충실히 이식했는지 확인한다:

  1. `rear_t_base_cam_from_front` — 전방 마운트에서 후방 마운트 기본값을
     유도하는 순수 헬퍼(180° 미러).
  2. `rear_image_topic=""`(기본) 이면 후방 구독/상태가 전혀 생기지 않는다
     (하위호환 — 기존 단일카메라 동작 불변).
  3. `rear_image_topic` 을 채우면 후방 콜백(`_on_image_rear`)이 검출한 ref
     마커가 전방과 **같은 `self.filt` 객체**를 실제로 보정한다(ref_ids 필터도
     후방에 그대로 적용됨을 함께 확인).

Isaac 실행 없음 — rclpy 는 쓰지만(노드 인스턴스 필요) 카메라/이미지는 전부
합성 더미(monkeypatch)다. 실행: `python -m pytest src/parkbot_aruco/test/ -p no:anyio -v`
"""
import sys
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

REPO = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(REPO / "src" / "parkbot_aruco"))

import rclpy  # noqa: E402
from sensor_msgs.msg import CameraInfo, Image  # noqa: E402

from parkbot_aruco import marker_localizer_node as MLN  # noqa: E402
from parkbot_aruco.marker_localizer_node import (  # noqa: E402
    MarkerLocalizerNode, rear_t_base_cam_from_front, _DEFAULT_T_BASE_CAM,
    _DEFAULT_REAR_T_BASE_CAM,
)


# ---------- 1. rear_t_base_cam_from_front (순수 헬퍼) ----------

def test_rear_mirror_flips_position_xz_keeps_height_y():
    """build_rear_camera.py 규약(base_link 수직축 180° 회전 미러)의 검산:
    이 T_base_cam 프레임에서 수직축은 Y — 미러는 위치의 x,z 만 부호반전하고
    높이(y) 는 그대로 지킨다."""
    rear = np.array(rear_t_base_cam_from_front(_DEFAULT_T_BASE_CAM)).reshape(4, 4)
    front = np.array(_DEFAULT_T_BASE_CAM).reshape(4, 4)
    assert rear[0, 3] == pytest.approx(-front[0, 3])   # x 부호반전
    assert rear[1, 3] == pytest.approx(front[1, 3])    # y(높이) 불변
    assert rear[2, 3] == pytest.approx(-front[2, 3])   # z 부호반전


def test_rear_mirror_is_180_degree_rotation_about_local_y():
    """미러 변환 자체가 정확히 Y축 둘레 180° 회전의 왼쪽곱인지 확인한다
    (다른 각도·다른 축이면 아래 등식이 깨진다)."""
    front = np.array(_DEFAULT_T_BASE_CAM).reshape(4, 4)
    rear = np.array(rear_t_base_cam_from_front(_DEFAULT_T_BASE_CAM)).reshape(4, 4)
    ry180 = np.diag([-1.0, 1.0, -1.0, 1.0])
    assert np.allclose(rear, ry180 @ front, atol=1e-9)


def test_rear_mirror_applied_twice_returns_to_front():
    """180° 를 두 번 적용하면 원래 전방으로 돌아와야 한다(자기역원)."""
    once = rear_t_base_cam_from_front(_DEFAULT_T_BASE_CAM)
    twice = rear_t_base_cam_from_front(once)
    assert np.allclose(np.array(twice), np.array(_DEFAULT_T_BASE_CAM), atol=1e-9)


def test_rear_mirror_generalizes_to_arbitrary_front_matrix():
    """기본 전방 마운트뿐 아니라 임의의(회전 포함) 4x4 에도 같은 180°/Y 규칙이
    적용되는지 — 오버라이드 상황(T5 실측값 대입) 대비."""
    rng = np.random.default_rng(0)
    # 임의의 직교(회전) 행렬 하나 + 임의 평행이동으로 합성 전방 마운트를 만든다.
    q, _ = np.linalg.qr(rng.normal(size=(3, 3)))
    if np.linalg.det(q) < 0:
        q[:, 0] *= -1  # 고유회전(det=+1) 보장
    front = np.eye(4)
    front[:3, :3] = q
    front[:3, 3] = [0.5, 0.2, -0.7]

    rear = np.array(rear_t_base_cam_from_front(front.reshape(-1).tolist())).reshape(4, 4)
    ry180 = np.diag([-1.0, 1.0, -1.0, 1.0])
    assert np.allclose(rear, ry180 @ front, atol=1e-9)


def test_default_rear_t_base_cam_matches_helper_on_default_front():
    """노드 모듈 상수 _DEFAULT_REAR_T_BASE_CAM 이 헬퍼 결과와 일치(배선 검산)."""
    assert _DEFAULT_REAR_T_BASE_CAM == rear_t_base_cam_from_front(_DEFAULT_T_BASE_CAM)


# ---------- 2/3. 노드 배선: 하위호환 + 이중카메라 공유필터 ----------

def _fake_image(w=4, h=4):
    img = Image()
    img.height = h
    img.width = w
    img.encoding = "bgr8"
    img.step = w * 3
    img.data = bytes(h * img.step)
    return img


def _fake_camera_info():
    info = CameraInfo()
    info.k = [500.0, 0.0, 320.0, 0.0, 500.0, 240.0, 0.0, 0.0, 1.0]
    info.d = []
    return info


def _fake_detection(marker_id, reproj_err_px=0.5):
    """AP.detect_and_estimate 결과(MarkerPose)를 흉내내는 더미. _process_frame
    은 marker_id/reproj_err_px/rvec/tvec 만 읽으므로 그만큼만 갖춘다."""
    return SimpleNamespace(
        marker_id=marker_id, reproj_err_px=reproj_err_px,
        rvec=np.array([0.0, 0.0, 0.0]), tvec=np.array([0.0, 0.0, 1.0]))


def _make_node(overrides=None):
    """노드 하나를 독립된 rclpy 컨텍스트에서 만든다.

    파라미터 오버라이드는 CLI 스타일(`-p name:=value`, YAML 파싱)로 넘긴다 —
    `MarkerLocalizerNode.__init__` 이 인자를 받지 않아(항상 `Node.__init__(self,
    "marker_localizer_node")` 만 호출) `parameter_overrides=` 를 직접 통과시킬
    방법이 없고, `rclpy.init(args=[...])` 로 준 전역 `-p` 오버라이드는 이후
    생성되는 노드의 같은 이름 파라미터에 그대로 반영된다(ROS 2 표준 동작).
    """
    if rclpy.ok():
        rclpy.shutdown()
    args = ["--ros-args"]
    for name, value in (overrides or {}).items():
        args += ["-p", f"{name}:={value}"]
    rclpy.init(args=args)
    return MarkerLocalizerNode()


def test_rear_disabled_by_default_no_rear_subscriptions():
    """rear_image_topic 기본값("")이면 후방 상태가 전혀 만들어지지 않는다 —
    기존 단일카메라 동작 완전 불변(하위호환)."""
    node = _make_node()
    try:
        assert node.get_parameter("rear_image_topic").value == ""
        assert node.rear_enabled is False
        assert node.K_rear is None
        assert node.dist_rear is None
        assert not hasattr(node, "T_rear")   # 후방 마운트조차 계산되지 않는다
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_rear_disabled_on_empty_string_even_if_info_topic_set():
    """rear_camera_info_topic 만 채우고 rear_image_topic 은 비워두면(오배선
    방지) 여전히 후방 비활성이어야 한다 — 활성화 스위치는 image_topic 뿐."""
    node = _make_node({"rear_camera_info_topic": "/robot_x/rear/camera_info"})
    try:
        assert node.rear_enabled is False
        assert node.K_rear is None
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_rear_enabled_creates_rear_mount_and_state():
    node = _make_node({
        "rear_image_topic": "/robot_x/rear/image_raw",
        "rear_camera_info_topic": "/robot_x/rear/camera_info",
    })
    try:
        assert node.rear_enabled is True
        assert hasattr(node, "T_rear")
        assert node.T_rear.shape == (4, 4)
        # 오버라이드 없으면 전방에서 유도한 기본값 그대로.
        assert np.allclose(node.T_rear.reshape(-1),
                            np.array(_DEFAULT_REAR_T_BASE_CAM))
        assert node.K_rear is None   # CameraInfo 아직 안 옴
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_dual_camera_updates_shared_filter_and_applies_ref_ids(monkeypatch):
    """후방 프레임의 ref 마커가 전방과 **같은 self.filt** 을 실제로 보정하고,
    ref_ids 하드필터가 후방에도 그대로 적용되는지 확인한다."""
    node = _make_node({
        "fuse": "true",
        "rear_image_topic": "/robot_x/rear/image_raw",
        "rear_camera_info_topic": "/robot_x/rear/camera_info",
        "ref_ids": "[21]",
    })
    try:
        # 전방에서 이미 도크로 시딩됐다고 가정(러너의 filt 시딩과 동일 상황).
        node.filt.set_pose(0.0, 0.0, 0.0)
        shared_filt = node.filt

        node._on_info_rear(_fake_camera_info())
        assert node.K_rear is not None

        # ref_ids=[21] 이므로 99 는 걸러지고 21 만 filt 을 보정해야 한다.
        monkeypatch.setattr(
            MLN.AP, "detect_and_estimate",
            lambda *a, **k: [_fake_detection(99), _fake_detection(21)])

        node._on_image_rear(_fake_image())

        assert node.filt is shared_filt            # 새 필터가 아니라 같은 객체
        assert node.filt.n_fix == 1                 # 21 하나만 반영(99 는 필터링)
        assert node.filt.pose() != (0.0, 0.0, 0.0)   # 실제로 보정되어 움직였다
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_front_and_rear_callbacks_accumulate_on_same_filter(monkeypatch):
    """전방(_on_image)과 후방(_on_image_rear)을 번갈아 호출해도(러너
    _run_entry_lead_b 의 rear_ctx/front_ctx 교대 패턴) 같은 filt 에 계속
    누적된다 — n_fix 가 두 콜백 합산으로 증가한다."""
    node = _make_node({
        "fuse": "true",
        "rear_image_topic": "/robot_x/rear/image_raw",
        "rear_camera_info_topic": "/robot_x/rear/camera_info",
    })
    try:
        node._on_info(_fake_camera_info())
        node._on_info_rear(_fake_camera_info())

        monkeypatch.setattr(
            MLN.AP, "detect_and_estimate",
            lambda *a, **k: [_fake_detection(21)])

        node._on_image(_fake_image())        # 전방: filt 없으면 set_pose 로 시딩
        assert node.filt.x is not None
        assert node.filt.n_fix == 0            # 첫 관측은 set_pose(시딩), n_fix 아직 0

        node._on_image_rear(_fake_image())    # 후방: 같은 filt 을 보정(update)
        assert node.filt.n_fix == 1
    finally:
        node.destroy_node()
        rclpy.shutdown()
