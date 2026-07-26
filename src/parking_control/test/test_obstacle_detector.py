"""v4 통로 장애물 감지 검증 (Isaac Sim·ROS 불필요, 가짜 좌표만 사용)."""

from pathlib import Path

import numpy as np
import pytest

from parking_control.core.graph import ParkingMap
from parking_control.core.obstacle_detector import (
    HEIGHT_THRESHOLD_M, detect_blocked_zones, zone_boxes,
)

MAP_YAML = Path(__file__).resolve().parent.parent / "config" / "parking_map.yaml"


@pytest.fixture
def boxes():
    return zone_boxes(ParkingMap.load(MAP_YAML))


def _cluster(cx, cy, n=40, z_lo=0.3, z_hi=1.6):
    rng = np.random.default_rng(1)
    return np.column_stack([
        rng.uniform(cx - 0.2, cx + 0.2, n),
        rng.uniform(cy - 0.2, cy + 0.2, n),
        rng.uniform(z_lo, z_hi, n),
    ])


def test_all_zones_clear_when_no_points(boxes):
    empty = np.empty((0, 3))
    results = detect_blocked_zones(empty, boxes)
    assert all(not blocked for blocked in results.values())
    assert set(results) == set(boxes)


def test_person_blocks_only_the_zone_they_stand_in(boxes):
    # 입차 외곽 차로 ZIN01의 가운데. 출차 차로와 y축으로 분리돼야 한다.
    points = _cluster(-16.8, -7.075)
    results = detect_blocked_zones(points, boxes)
    assert results["ZIN01"] is True
    assert not any(
        blocked for zid, blocked in results.items() if zid.startswith("ZOUT")
    )


def test_entry_and_exit_lanes_keep_their_real_y_coordinates(boxes):
    zin = boxes["ZIN01"]
    zout = boxes["ZOUT03"]
    assert (zin[2] + zin[3]) / 2 == pytest.approx(-7.075)
    assert (zout[2] + zout[3]) / 2 == pytest.approx(7.075)
    assert zin[3] < zout[2]


def test_vertical_slot_connector_is_centered_on_slot(boxes):
    x0, x1, y0, y1 = boxes["ZIN_A1_dock"]
    assert (x0 + x1) / 2 == pytest.approx(2.8)
    assert y0 == pytest.approx(-6.875)
    assert y1 == pytest.approx(0.0)
    # A1 진입로가 A2 중심(6.2m)까지 침범하면 안 된다.
    assert x1 < 6.2


def test_floor_noise_alone_does_not_trigger(boxes):
    rng = np.random.default_rng(2)
    floor = np.column_stack([
        rng.uniform(-20, 20, 500), rng.uniform(-12, 12, 500),
        rng.uniform(0, HEIGHT_THRESHOLD_M * 0.6, 500),
    ])
    results = detect_blocked_zones(floor, boxes)
    assert all(not blocked for blocked in results.values())


def test_robot_at_own_position_is_excluded():
    from parking_control.core.obstacle_detector import ROBOT_EXCLUDE_RADIUS_M
    parking_map = ParkingMap.load(MAP_YAML)
    boxes_ = zone_boxes(parking_map)
    robot_xy = (-16.8, -7.075)
    points = _cluster(*robot_xy)  # 로봇 자신의 몸체가 만드는 점들

    without_exclusion = detect_blocked_zones(points, boxes_)
    assert without_exclusion["ZIN01"] is True  # 제외 안 하면 스스로를 장애물로 봄

    with_exclusion = detect_blocked_zones(points, boxes_, robot_positions=[robot_xy])
    assert with_exclusion["ZIN01"] is False  # 제외하면 정상적으로 clear

    assert ROBOT_EXCLUDE_RADIUS_M > 0  # 반경 상수가 실제로 쓰이고 있다는 방증


def test_two_people_block_two_different_zones(boxes):
    points = np.vstack([
        _cluster(-16.8, -7.075),
        _cluster(-16.8, 7.075),
    ])
    results = detect_blocked_zones(points, boxes)
    assert results["ZIN01"] is True
    assert results["ZOUT03"] is True
