"""사람/비사람 LiDAR 기하 분류 검증 (Isaac Sim·ROS 불필요, 가짜 좌표만 사용)."""

import numpy as np

from parking_control.core.occupancy_grid import GridSpec, rasterize_points
from parking_control.core.pedestrian_presence import (
    PersonClassifierConfig,
    Region,
    classify_person_clusters,
    detect_region_presence,
)

TEST_REGION = Region("test_region", -5.0, 5.0, -5.0, 5.0)
CONFIG = PersonClassifierConfig()


def _box_points(rng, *, center, x_range, y_range, z_range, count=300):
    cx, cy = center
    x = rng.uniform(cx - x_range / 2, cx + x_range / 2, count)
    y = rng.uniform(cy - y_range / 2, cy + y_range / 2, count)
    z = rng.uniform(z_range[0], z_range[1], count)
    return np.column_stack((x, y, z))


def test_floor_points_are_not_obstacle_or_person():
    rng = np.random.default_rng(1)
    floor = _box_points(
        rng, center=(0.0, 0.0), x_range=3.0, y_range=3.0, z_range=(0.0, 0.02))
    presence = detect_region_presence(floor, (TEST_REGION,))
    assert presence["test_region"]["detected"] is False
    assert classify_person_clusters(floor, (TEST_REGION,), config=CONFIG) == []


def test_points_outside_roi_are_ignored():
    rng = np.random.default_rng(2)
    outside = _box_points(
        rng, center=(100.0, 100.0), x_range=0.6, y_range=0.6, z_range=(0.1, 1.8))
    presence = detect_region_presence(outside, (TEST_REGION,))
    assert presence["test_region"]["detected"] is False
    assert classify_person_clusters(outside, (TEST_REGION,), config=CONFIG) == []


def test_dense_person_sized_cluster_is_classified_as_person():
    rng = np.random.default_rng(3)
    person = _box_points(
        rng, center=(0.0, 0.0), x_range=0.55, y_range=0.70, z_range=(0.05, 1.83))
    clusters = classify_person_clusters(person, (TEST_REGION,), config=CONFIG)
    assert len(clusters) == 1
    cluster = clusters[0]
    assert cluster["label"] == "PERSON"
    assert 1.0 <= cluster["top_height"] <= 2.3
    assert cluster["width"] <= 1.2
    assert cluster["length"] <= 1.6


def test_vehicle_sized_cluster_is_non_person():
    rng = np.random.default_rng(4)
    vehicle = _box_points(
        rng, center=(0.0, 0.0), x_range=4.0, y_range=2.0, z_range=(0.05, 1.5),
        count=800)
    clusters = classify_person_clusters(vehicle, (TEST_REGION,), config=CONFIG)
    assert len(clusters) == 1
    assert clusters[0]["label"] == "NON_PERSON"


def test_low_robot_cluster_is_non_person():
    rng = np.random.default_rng(5)
    robot = _box_points(
        rng, center=(0.0, 0.0), x_range=0.4, y_range=0.5, z_range=(0.05, 0.28))
    clusters = classify_person_clusters(robot, (TEST_REGION,), config=CONFIG)
    assert len(clusters) == 1
    assert clusters[0]["label"] == "NON_PERSON"
    assert clusters[0]["top_height"] < CONFIG.min_top_height


def test_two_separated_person_clusters_are_both_detected():
    rng = np.random.default_rng(6)
    person_a = _box_points(
        rng, center=(-2.0, 0.0), x_range=0.55, y_range=0.70, z_range=(0.05, 1.83))
    person_b = _box_points(
        rng, center=(2.0, 0.0), x_range=0.55, y_range=0.70, z_range=(0.05, 1.83))
    assert np.linalg.norm(np.array([-2.0, 0.0]) - np.array([2.0, 0.0])) >= 2.4

    clusters = classify_person_clusters(
        np.concatenate([person_a, person_b]), (TEST_REGION,), config=CONFIG)
    person_clusters = [c for c in clusters if c["label"] == "PERSON"]
    assert len(person_clusters) == 2


def test_occupancy_grid_height_filter_min_points_and_inflation():
    # parking_control.core.occupancy_grid(기존 점유지도 모듈)에도 같은 성질이
    # 있는지 확인한다 — pedestrian_obstacle_node가 그 옆에서 같은 points_world를
    # 보므로 두 소비자가 같은 전제(높이 필터/최소 점/팽창)로 동작해야 한다.
    spec = GridSpec(
        resolution=1.0, origin_x=0.0, origin_y=0.0, width=5, height=5)
    points = np.array([
        [2.5, 2.5, 0.02],  # 높이 필터 밖(min_height=0.05 미만) -> 무시
        [2.5, 2.5, 1.0],
        [2.5, 2.5, 1.0],   # 셀 (row=2, col=2)에 유효 점 2개 -> 점유
        [0.5, 0.5, 1.0],   # 셀 (row=0, col=0)에 점 1개뿐 -> 최소 점 개수 미달
    ])
    grid = rasterize_points(
        points, spec, min_height=0.05, max_height=2.0,
        min_points_per_cell=2, inflate_cells=1)

    assert grid[2, 2] == 100
    assert grid[0, 0] == 0
    assert grid[1, 2] == 100
    assert grid[3, 2] == 100
    assert grid[2, 1] == 100
    assert grid[2, 3] == 100
    assert grid[4, 4] == 0
