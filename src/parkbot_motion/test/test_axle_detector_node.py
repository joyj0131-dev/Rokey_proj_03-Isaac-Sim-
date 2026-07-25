"""axle_detector_node 의 순수 함수(ROS 무의존) 단위테스트.

rclpy 는 이 모듈이 import-time 에 필요로 하지만(Node 서브클래스 정의,
sensor_msgs/geometry_msgs/nav_msgs 등 메시지 타입 임포트), 이 테스트가 실제로
부르는 두 함수(depth_image_to_array, combine_side_depths)는 rclpy.init()/
노드 생성/스핀이 전혀 필요 없는 평범한 파이썬 함수다(test_pose_controller_node_quat.py
와 동일한 취지 — 이 개발 머신엔 시스템 ROS2 Humble 이 설치돼 있어 import 자체는
항상 성공한다).
"""
import math
import pathlib
import sys

import numpy as np
import pytest

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))
from parkbot_motion.axle_detector_node import combine_side_depths, depth_image_to_array


# --- depth_image_to_array -------------------------------------------------

def test_depth_image_to_array_no_padding():
    h, w = 3, 4
    grid = np.arange(h * w, dtype=np.float32).reshape(h, w)
    data = grid.tobytes()
    out = depth_image_to_array(h, w, w * 4, data)
    assert out.shape == (h, w)
    np.testing.assert_array_equal(out, grid)


def test_depth_image_to_array_with_row_padding():
    # step 이 width*4 보다 큰 경우(정렬 패딩) — 여분 컬럼을 버려야 한다.
    h, w = 2, 3
    cols_per_row = 5  # 패딩 2컬럼
    step = cols_per_row * 4
    raw = np.zeros((h, cols_per_row), dtype=np.float32)
    raw[0, :w] = [1.0, 2.0, 3.0]
    raw[1, :w] = [4.0, 5.0, 6.0]
    raw[:, w:] = -999.0  # 패딩 영역(버려져야 함)
    out = depth_image_to_array(h, w, step, raw.tobytes())
    assert out.shape == (h, w)
    np.testing.assert_array_equal(out, [[1.0, 2.0, 3.0], [4.0, 5.0, 6.0]])


def test_depth_image_to_array_rejects_non_32fc1():
    with pytest.raises(ValueError):
        depth_image_to_array(2, 2, 8, b"\x00" * 16, encoding="rgb8")


def test_depth_image_to_array_rejects_short_buffer():
    with pytest.raises(ValueError):
        depth_image_to_array(4, 4, 16, b"\x00" * 8)  # 4x4 float32 에 부족한 길이


# --- combine_side_depths ---------------------------------------------------

def test_combine_side_depths_both_valid_averages():
    assert combine_side_depths(0.2, 0.4) == pytest.approx(0.3)


def test_combine_side_depths_left_only():
    assert combine_side_depths(0.25, None) == pytest.approx(0.25)


def test_combine_side_depths_right_only():
    assert combine_side_depths(None, 0.33) == pytest.approx(0.33)


def test_combine_side_depths_neither_is_inf():
    assert math.isinf(combine_side_depths(None, None))
