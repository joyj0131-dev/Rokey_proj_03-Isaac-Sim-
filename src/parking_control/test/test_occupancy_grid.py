import numpy as np
import pytest

from parking_control.core.occupancy_grid import GridSpec, rasterize_points


def test_projects_world_xy_into_grid_and_filters_floor_and_ceiling():
    spec = GridSpec(resolution=1.0, origin_x=-2.0, origin_y=-1.0, width=5, height=4)
    points = np.array([
        [0.2, 1.2, 0.5],
        [0.4, 1.4, 0.8],
        [1.2, 1.2, 0.01],   # 바닥
        [1.2, 1.2, 4.0],    # 천장
        [99.0, 99.0, 1.0],  # 지도 밖
    ])

    grid = rasterize_points(
        points, spec, min_points_per_cell=2, inflate_cells=0)

    assert grid.shape == (4, 5)
    assert grid[2, 2] == 100
    assert np.count_nonzero(grid) == 1


def test_requires_configured_number_of_points_per_cell():
    spec = GridSpec(resolution=1.0, origin_x=0.0, origin_y=0.0, width=2, height=2)
    point = np.array([[0.2, 0.2, 1.0]])

    grid = rasterize_points(point, spec, min_points_per_cell=2, inflate_cells=0)

    assert np.count_nonzero(grid) == 0


def test_inflates_occupied_cell_for_map_visibility():
    spec = GridSpec(resolution=1.0, origin_x=0.0, origin_y=0.0, width=5, height=5)
    points = np.array([[2.2, 2.2, 1.0]])

    grid = rasterize_points(
        points, spec, min_points_per_cell=1, inflate_cells=1)

    assert np.all(grid[1:4, 1:4] == 100)
    assert np.count_nonzero(grid) == 9


def test_rejects_invalid_point_shape():
    with pytest.raises(ValueError, match="Nx3"):
        rasterize_points(np.array([1.0, 2.0, 3.0]), GridSpec())
