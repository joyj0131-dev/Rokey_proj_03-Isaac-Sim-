"""marker_localizer_node ref-마커 하드필터 + 위치전용 보정 회귀 테스트.

Phase B(도크→XN 융합주행)의 진실의 원천은 러너 parking_v4_runner.py:
  - fuse_camera_setup 의 `hit = [p for p in det if int(p.marker_id) == ctx["ref_id"]]`
    (960, 974행) — ref 마커 하드필터.
  - drive_to_pose 의 `_apply_fix` correct_yaw=False 분기 — 위치만 pos_gain 으로
    블렌드하고 yaw 는 오도(predict) 값을 그대로 둔다.

이 테스트는 노드/rclpy 스핀 없이 두 순수함수(filter_detections_by_ref, apply_fix)를
검증한다. detection/fix 모의는 marker_id·x·z·yaw_deg 속성만 있으면 되므로 간단한
더미 객체(SimpleNamespace)로 대체한다.
"""
import sys
from pathlib import Path
from types import SimpleNamespace

REPO = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(REPO / "src" / "parkbot_aruco"))


def _fake(marker_id):
    """detect_and_estimate 결과(PoseEstimate)를 흉내내는 더미 — marker_id 만 쓴다."""
    return SimpleNamespace(marker_id=marker_id)


def _fix(x, z, yaw_deg):
    """RobotFix 를 흉내내는 더미 — apply_fix 가 쓰는 x·z·yaw_deg 만 있으면 된다."""
    return SimpleNamespace(x=x, z=z, yaw_deg=yaw_deg)


def test_ref_ids_filters_detections():
    """ref_ids=[31] 이면 id 21/23 검출은 무시하고 31만 측위에 쓴다."""
    from parkbot_aruco.marker_localizer_node import filter_detections_by_ref
    dets = [_fake(21), _fake(31), _fake(23)]
    kept = filter_detections_by_ref(dets, [31])
    assert [d.marker_id for d in kept] == [31]
    assert filter_detections_by_ref(dets, []) == dets  # 빈=전체(하위호환)


def test_ref_ids_filters_multiple_ids():
    """ref_ids 는 집합 — 여러 id 를 동시에 허용한다(도크 21/23 동시 필터 등)."""
    from parkbot_aruco.marker_localizer_node import filter_detections_by_ref
    dets = [_fake(21), _fake(31), _fake(23)]
    kept = filter_detections_by_ref(dets, [21, 23])
    assert [d.marker_id for d in kept] == [21, 23]


def test_ref_ids_no_match_returns_empty():
    """ref_ids 에 없는 id 만 검출됐으면 빈 리스트(필터 없음이 아니라 진짜 매치 없음)."""
    from parkbot_aruco.marker_localizer_node import filter_detections_by_ref
    dets = [_fake(21), _fake(23)]
    assert filter_detections_by_ref(dets, [31]) == []


def test_position_only_correction_keeps_odom_yaw():
    """correct_yaw=False 면 마커 fix 의 위치만 블렌드하고 yaw 는 오도 값을 지킨다."""
    from parkbot_aruco.marker_localizer import PoseFilter
    f = PoseFilter()
    f.set_pose(0.0, 0.0, 90.0)
    f.predict(0.0, 0.0, 5.0)          # 오도가 yaw 를 95°로
    from parkbot_aruco.marker_localizer_node import apply_fix
    apply_fix(f, _fix(x=0.1, z=0.1, yaw_deg=45.0), correct_yaw=False)
    assert abs(f.pose()[2] - 95.0) < 1e-6   # yaw 는 마커(45)에 안 끌려감
    assert f.pose()[0] > 0.0                 # 위치는 마커 쪽으로 이동


def test_correct_yaw_true_uses_existing_update():
    """correct_yaw=True 는 기존 filt.update(fix) 와 동일하게 yaw 도 블렌드한다."""
    from parkbot_aruco.marker_localizer import PoseFilter
    f_direct = PoseFilter()
    f_direct.set_pose(0.0, 0.0, 90.0)
    f_direct.update(_fix(x=0.1, z=0.1, yaw_deg=45.0))

    f_via_helper = PoseFilter()
    f_via_helper.set_pose(0.0, 0.0, 90.0)
    from parkbot_aruco.marker_localizer_node import apply_fix
    apply_fix(f_via_helper, _fix(x=0.1, z=0.1, yaw_deg=45.0), correct_yaw=True)

    assert f_direct.pose() == f_via_helper.pose()
