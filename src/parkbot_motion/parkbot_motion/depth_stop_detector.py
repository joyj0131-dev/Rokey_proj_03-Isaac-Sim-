#!/usr/bin/env python3
"""뎁스캠 프레임에서 정지 시점을 판단하는 순수 파이썬 로직.

Isaac/ROS 의존성 없음(numpy만 사용) — depth_stop_lift_test.py와
probe_depth_cam_stop.py가 이 모듈을 가져다 쓴다.

정지 판단은 절대 임계값이 아니라 **런타임 캘리브레이션**이다: 진입 초반
`baseline_frames` 개 프레임 동안 ROI 최소 뎁스를 모아 중앙값을 베이스라인으로
잡고, 그보다 `drop_margin` 이상 낮은 값이 `confirm_frames` 연속으로 나오면
트리거한다. 단일 프레임 잡음으로 오정지하지 않기 위한 디바운스다.
"""

import math

import numpy as np

DEFAULT_ROI_FRAC = (0.30, 0.70, 0.50, 1.00)  # (col_lo, col_hi, row_lo, row_hi), 0~1 비율


def roi_min_depth(depth_hw, roi_frac=DEFAULT_ROI_FRAC):
    """depth_hw: (height, width) 2차원 배열. ROI 내 유효(finite) 최소값을 반환.

    유효 픽셀이 하나도 없으면 math.inf (신호 없음).
    """
    depth_hw = np.asarray(depth_hw)
    if depth_hw.ndim != 2:
        raise ValueError(f"depth_hw must be 2D (height, width), got shape {depth_hw.shape}")
    h, w = depth_hw.shape
    col_lo_f, col_hi_f, row_lo_f, row_hi_f = roi_frac
    row_lo, row_hi = int(h * row_lo_f), max(int(h * row_hi_f), int(h * row_lo_f) + 1)
    col_lo, col_hi = int(w * col_lo_f), max(int(w * col_hi_f), int(w * col_lo_f) + 1)
    patch = np.asarray(depth_hw[row_lo:row_hi, col_lo:col_hi], dtype=np.float64)
    finite = patch[np.isfinite(patch)]
    if finite.size == 0:
        return math.inf
    return float(finite.min())


class DepthStopDetector:
    """진입 구동 중 프레임을 하나씩 넣어 정지 시점을 판단한다."""

    def __init__(self, baseline_frames=30, drop_margin=0.05, confirm_frames=3):
        self.baseline_frames = baseline_frames
        self.drop_margin = drop_margin
        self.confirm_frames = confirm_frames
        self._baseline_samples = []
        self.baseline = None
        self.threshold = None
        self._consecutive = 0
        self.triggered = False
        self.trigger_step = None
        self.trigger_value = None

    def update(self, step, roi_value):
        """한 프레임의 ROI 최소뎁스를 넣는다. 이번 프레임에 트리거되면 True."""
        if self.triggered:
            return True

        if self.baseline is None:
            if math.isfinite(roi_value):
                self._baseline_samples.append(roi_value)
            if len(self._baseline_samples) >= self.baseline_frames:
                self.baseline = float(np.median(self._baseline_samples))
                self.threshold = self.baseline - self.drop_margin
            return False

        if math.isfinite(roi_value) and roi_value < self.threshold:
            self._consecutive += 1
        else:
            self._consecutive = 0

        if self._consecutive >= self.confirm_frames:
            self.triggered = True
            self.trigger_step = step
            self.trigger_value = roi_value
            return True
        return False
