"""LiDAR 센서 로컬→월드 변환 검증.

주의: 이 테스트는 "계산이 스스로 일관되는지"만 확인한다 — Isaac Sim
ROS2 브릿지가 실제로 이 축 규약을 쓰는지는 실측으로만 확인 가능하다
(core/lidar_frame_transform.py 상단 경고 참고).
"""

import numpy as np
import pytest

from parking_control.core.lidar_frame_transform import (
    sensor_offsets, transform_to_world, verify_with_known_point,
)

HALF_W = 17.0  # parking_map.yaml 기준 (space_count=10, space_width=3.4)


def test_sensor_offsets_match_known_mount_positions():
    west_x, east_x, height = sensor_offsets(HALF_W)
    assert west_x == pytest.approx(-7.82, abs=0.01)
    assert east_x == pytest.approx(7.82, abs=0.01)
    assert height == pytest.approx(5.12, abs=0.01)


def test_sensor_origin_maps_to_its_own_mount_position():
    """센서 로컬 원점(0,0,0)은 정의상 그 센서의 설치 위치 자체여야 한다."""
    west_x, east_x, height = sensor_offsets(HALF_W)
    origin = np.array([[0.0, 0.0, 0.0]])

    world = transform_to_world(origin, west_x, height)
    assert world[0, 0] == pytest.approx(west_x)   # ros_x
    assert world[0, 2] == pytest.approx(height)    # ros_z(높이)

    world_e = transform_to_world(origin, east_x, height)
    assert world_e[0, 0] == pytest.approx(east_x)


def test_local_forward_axis_moves_toward_floor():
    """로컬 +Z(가정) 방향으로 나아갈수록 월드 높이(ros_z)가 낮아져야
    한다 — 천장 센서가 아래를 보고 있다는 물리적 사실의 최소 검증."""
    west_x, _, height = sensor_offsets(HALF_W)
    near = transform_to_world(np.array([[0.0, 0.0, 0.5]]), west_x, height)
    far = transform_to_world(np.array([[0.0, 0.0, 3.0]]), west_x, height)
    assert far[0, 2] < near[0, 2] < height


def test_empty_input_returns_empty():
    west_x, _, height = sensor_offsets(HALF_W)
    empty = np.empty((0, 3))
    assert transform_to_world(empty, west_x, height).size == 0


def test_verify_with_known_point_helper():
    pts = np.array([[-18.0, 0.2, 0.0], [5.0, 5.0, 0.0]])
    assert verify_with_known_point(pts, (-18.1, 0.0), tolerance_m=1.0) is True
    assert verify_with_known_point(pts, (100.0, 100.0), tolerance_m=1.0) is False
