"""통로 장애물 감지 검증 (Isaac Sim·ROS 불필요, 가짜 좌표만 사용)."""

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
    # Z03 구간(J2~J3, x=-10.2~-6.8)의 한가운데(x=-8.5, y=0)에 사람 하나
    points = _cluster(-8.5, 0.0)
    results = detect_blocked_zones(points, boxes)
    assert results["Z03"] is True
    assert all(not blocked for zid, blocked in results.items() if zid != "Z03")


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
    robot_xy = (-8.5, 0.0)
    points = _cluster(*robot_xy)  # 로봇 자신의 몸체가 만드는 점들

    without_exclusion = detect_blocked_zones(points, boxes_)
    assert without_exclusion["Z03"] is True  # 제외 안 하면 스스로를 장애물로 봄

    with_exclusion = detect_blocked_zones(points, boxes_, robot_positions=[robot_xy])
    assert with_exclusion["Z03"] is False  # 제외하면 정상적으로 clear

    assert ROBOT_EXCLUDE_RADIUS_M > 0  # 반경 상수가 실제로 쓰이고 있다는 방증


def test_two_people_block_two_different_zones(boxes):
    points = np.vstack([_cluster(-8.5, 0.0), _cluster(5.1, 0.0)])
    results = detect_blocked_zones(points, boxes)
    blocked = {zid for zid, v in results.items() if v}
    assert blocked == {"Z03", "Z07"}
