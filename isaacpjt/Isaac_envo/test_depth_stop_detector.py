#!/usr/bin/env python3
"""depth_stop_detector 단위 테스트 (Isaac/ROS 불필요).

실행:
    python3 test_depth_stop_detector.py    # 직접 실행, PASS/FAIL 출력
    pytest test_depth_stop_detector.py     # 동일 검증을 assert 로
"""

import math

import numpy as np

from depth_stop_detector import DepthStopDetector, roi_min_depth


def test_roi_min_depth_extracts_bottom_center_region():
    depth = np.full((100, 200), 5.0, dtype=np.float32)
    # 세로 하단 50%(50~100행), 가로 중앙 40%(80~120열)에만 낮은 값을 심는다.
    depth[70:80, 90:110] = 0.42
    value = roi_min_depth(depth, roi_frac=(0.30, 0.70, 0.50, 1.00))
    assert abs(value - 0.42) < 1e-6

    # 같은 낮은 값을 ROI 밖(상단)에 심으면 잡히면 안 된다.
    depth2 = np.full((100, 200), 5.0, dtype=np.float32)
    depth2[0:10, 90:110] = 0.42
    value2 = roi_min_depth(depth2, roi_frac=(0.30, 0.70, 0.50, 1.00))
    assert abs(value2 - 5.0) < 1e-6


def test_roi_min_depth_ignores_non_finite():
    depth = np.full((100, 200), np.inf, dtype=np.float32)
    depth[60:90, 80:120] = np.nan
    depth[65, 95] = 0.9
    value = roi_min_depth(depth, roi_frac=(0.30, 0.70, 0.50, 1.00))
    assert abs(value - 0.9) < 1e-6


def test_roi_min_depth_all_non_finite_returns_inf():
    depth = np.full((100, 200), np.inf, dtype=np.float32)
    value = roi_min_depth(depth, roi_frac=(0.30, 0.70, 0.50, 1.00))
    assert math.isinf(value)


def test_detector_does_not_trigger_before_calibration():
    det = DepthStopDetector(baseline_frames=5, drop_margin=0.05, confirm_frames=3)
    # 첫 4프레임은 낮은 값이 섞여도(잡음) 베이스라인이 아직 안 잡혔으므로 트리거 금지.
    for step, value in enumerate([2.0, 2.0, 0.1, 2.0], start=1):
        assert det.update(step, value) is False
    assert det.baseline is None


def test_detector_ignores_single_frame_noise():
    det = DepthStopDetector(baseline_frames=3, drop_margin=0.05, confirm_frames=3)
    for step, value in enumerate([2.0, 2.0, 2.0], start=1):
        det.update(step, value)
    assert det.baseline == 2.0
    # 한 프레임만 뚝 떨어지고 바로 회복 -> 트리거되면 안 된다.
    assert det.update(4, 1.0) is False
    assert det.update(5, 2.0) is False
    assert det.triggered is False


def test_detector_triggers_after_confirm_frames():
    det = DepthStopDetector(baseline_frames=3, drop_margin=0.05, confirm_frames=3)
    for step, value in enumerate([2.0, 2.0, 2.0], start=1):
        det.update(step, value)
    assert det.update(4, 1.0) is False
    assert det.update(5, 1.0) is False
    assert det.update(6, 1.0) is True
    assert det.triggered is True
    assert det.trigger_step == 6
    assert det.trigger_value == 1.0


def main():
    tests = [
        test_roi_min_depth_extracts_bottom_center_region,
        test_roi_min_depth_ignores_non_finite,
        test_roi_min_depth_all_non_finite_returns_inf,
        test_detector_does_not_trigger_before_calibration,
        test_detector_ignores_single_frame_noise,
        test_detector_triggers_after_confirm_frames,
    ]
    failed = 0
    for test in tests:
        try:
            test()
            print(f"PASS {test.__name__}")
        except AssertionError as exc:
            failed += 1
            print(f"FAIL {test.__name__}: {exc}")
    print(f"DETECTOR_TEST_OK={failed == 0}")
    if failed:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
