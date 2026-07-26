"""월드 좌표 PointCloud를 2D 점유 격자로 투영한다. ROS 의존성 없음."""

from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class GridSpec:
    resolution: float = 0.1
    origin_x: float = -24.0
    origin_y: float = -12.0
    width: int = 400
    height: int = 240

    def __post_init__(self):
        if self.resolution <= 0:
            raise ValueError("resolution은 0보다 커야 합니다")
        if self.width <= 0 or self.height <= 0:
            raise ValueError("width와 height는 0보다 커야 합니다")


def _inflate(mask: np.ndarray, radius: int) -> np.ndarray:
    """점유 셀을 정사각형 반경만큼 확장한다."""
    if radius <= 0 or not mask.any():
        return mask
    inflated = np.zeros_like(mask)
    height, width = mask.shape
    for dy in range(-radius, radius + 1):
        src_y0 = max(0, -dy)
        src_y1 = min(height, height - dy)
        dst_y0 = max(0, dy)
        dst_y1 = min(height, height + dy)
        for dx in range(-radius, radius + 1):
            src_x0 = max(0, -dx)
            src_x1 = min(width, width - dx)
            dst_x0 = max(0, dx)
            dst_x1 = min(width, width + dx)
            inflated[dst_y0:dst_y1, dst_x0:dst_x1] |= \
                mask[src_y0:src_y1, src_x0:src_x1]
    return inflated


def rasterize_points(
    points,
    spec: GridSpec,
    *,
    min_height: float = 0.15,
    max_height: float = 3.0,
    min_points_per_cell: int = 2,
    inflate_cells: int = 1,
) -> np.ndarray:
    """Nx3 ``(x, y, z)`` 포인트를 ``(height, width)`` int8 격자로 만든다.

    바닥과 천장을 제외한 점만 장애물(100)로 표시하고 나머지는 자유 공간(0)으로
    둔다. 입력 포인트는 이미 ``map`` 프레임이어야 한다.
    """
    cloud = np.asarray(points)
    if cloud.ndim != 2 or cloud.shape[1] < 3:
        raise ValueError("points는 Nx3 이상의 배열이어야 합니다")
    if min_points_per_cell <= 0:
        raise ValueError("min_points_per_cell은 0보다 커야 합니다")

    grid = np.zeros((spec.height, spec.width), dtype=np.int8)
    if cloud.size == 0:
        return grid

    finite = np.isfinite(cloud[:, :3]).all(axis=1)
    height_ok = (
        (cloud[:, 2] >= min_height) &
        (cloud[:, 2] <= max_height)
    )
    selected = cloud[finite & height_ok]
    if not len(selected):
        return grid

    gx = np.floor((selected[:, 0] - spec.origin_x) / spec.resolution).astype(np.int64)
    gy = np.floor((selected[:, 1] - spec.origin_y) / spec.resolution).astype(np.int64)
    inside = (gx >= 0) & (gx < spec.width) & (gy >= 0) & (gy < spec.height)
    if not inside.any():
        return grid

    counts = np.zeros((spec.height, spec.width), dtype=np.uint32)
    np.add.at(counts, (gy[inside], gx[inside]), 1)
    occupied = counts >= min_points_per_cell
    occupied = _inflate(occupied, inflate_cells)
    grid[occupied] = 100
    return grid
