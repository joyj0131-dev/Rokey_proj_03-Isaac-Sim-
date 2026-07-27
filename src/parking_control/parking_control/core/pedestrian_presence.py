"""LiDAR 군집 기하로 사람/비사람을 구분한다 (ROS import 금지).

slot_occupancy_detector.py의 "막혔다/안 막혔다"류 판정과 달리, 여기서는
XY 군집화 후 각 군집의 높이(top_height)와 풋프린트(width/length)로
사람인지 차량/로봇/기타인지를 가른다. 천장 LiDAR는 사람의 전신보다
머리·어깨 최고점을 안정적으로 보므로 observed_height가 아니라
top_height를 주 높이 특징으로 쓴다.

animation/pedestrians_v4.usda의 보행자 경로(X=-8.5 고정, USD Z -> ROS
y=-USD z)를 대상으로 하는 pedestrian_obstacle_node.py에서 쓴다.
"""

from collections import defaultdict, deque
from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class Region:
    name: str
    x_min: float
    x_max: float
    y_min: float
    y_max: float


@dataclass(frozen=True)
class PersonClassifierConfig:
    cluster_cell_size: float = 0.3
    min_cluster_points: int = 8
    min_top_height: float = 1.0
    max_top_height: float = 2.3
    min_footprint: float = 0.08
    max_width: float = 1.2
    max_length: float = 1.6


def _points_in_region(points, region):
    return points[
        (points[:, 0] >= region.x_min)
        & (points[:, 0] <= region.x_max)
        & (points[:, 1] >= region.y_min)
        & (points[:, 1] <= region.y_max)
    ]


def _cluster_xy(points, cell_size):
    """XY 해시 격자의 8-이웃 연결 성분을 점 인덱스로 반환한다."""
    if not len(points):
        return []
    cells = np.floor(points[:, :2] / cell_size).astype(np.int64)
    by_cell = defaultdict(list)
    for index, cell in enumerate(cells):
        by_cell[(int(cell[0]), int(cell[1]))].append(index)

    clusters = []
    pending = set(by_cell)
    while pending:
        start = pending.pop()
        queue = deque([start])
        member_cells = [start]
        while queue:
            cx, cy = queue.popleft()
            for dx in (-1, 0, 1):
                for dy in (-1, 0, 1):
                    neighbour = (cx + dx, cy + dy)
                    if neighbour in pending:
                        pending.remove(neighbour)
                        queue.append(neighbour)
                        member_cells.append(neighbour)
        clusters.append(np.fromiter(
            (
                index
                for cell in member_cells
                for index in by_cell[cell]
            ),
            dtype=np.int64,
        ))
    return clusters


def _cluster_features(points):
    # 극단값에 덜 민감하도록 2~98 백분위를 사용한다.
    low = np.percentile(points[:, :3], 2, axis=0)
    high = np.percentile(points[:, :3], 98, axis=0)
    size = high - low
    footprint = sorted((float(size[0]), float(size[1])))
    centroid = np.median(points[:, :3], axis=0)
    return {
        "centroid": [float(v) for v in centroid],
        "width": footprint[0],
        "length": footprint[1],
        "bottom_height": float(low[2]),
        "top_height": float(high[2]),
        "observed_height": float(size[2]),
        "point_count": int(len(points)),
    }


def classify_person_clusters(
    points, regions, *, config=PersonClassifierConfig()
):
    cloud = np.asarray(points, dtype=float)
    if cloud.ndim != 2 or cloud.shape[1] < 3:
        raise ValueError("points는 Nx3 이상의 배열이어야 합니다")
    clean = cloud[
        np.isfinite(cloud[:, :3]).all(axis=1)
        & (cloud[:, 2] >= 0.05)
        & (cloud[:, 2] <= config.max_top_height + 0.5)
    ]

    classified = []
    for region in regions:
        region_points = _points_in_region(clean, region)
        for indices in _cluster_xy(
            region_points, config.cluster_cell_size
        ):
            cluster = region_points[indices]
            if len(cluster) < config.min_cluster_points:
                continue
            features = _cluster_features(cluster)
            top_ok = (
                config.min_top_height
                <= features["top_height"]
                <= config.max_top_height
            )
            footprint_ok = (
                features["width"] >= config.min_footprint
                and features["width"] <= config.max_width
                and features["length"] <= config.max_length
            )
            is_person = bool(top_ok and footprint_ok)

            target_height = 1.7
            height_half_range = max(
                target_height - config.min_top_height,
                config.max_top_height - target_height,
            )
            height_score = max(
                0.0,
                1.0
                - abs(features["top_height"] - target_height)
                / height_half_range,
            )
            footprint_score = max(
                0.0,
                1.0 - features["length"] / config.max_length,
            )
            density_score = min(
                1.0,
                features["point_count"]
                / (config.min_cluster_points * 3.0),
            )
            confidence = (
                0.55 * height_score
                + 0.30 * footprint_score
                + 0.15 * density_score
            )
            if not is_person:
                confidence = min(confidence, 0.49)
            classified.append({
                "region": region.name,
                "label": "PERSON" if is_person else "NON_PERSON",
                "confidence": round(float(confidence), 3),
                **features,
            })
    return classified


def detect_region_presence(
    points,
    regions,
    *,
    min_height=0.15,
    max_height=2.2,
    min_points=20,
):
    cloud = np.asarray(points)
    clean = cloud[
        np.isfinite(cloud[:, :3]).all(axis=1)
        & (cloud[:, 2] >= min_height)
        & (cloud[:, 2] <= max_height)
    ]
    result = {}
    for region in regions:
        count = int(len(_points_in_region(clean, region)))
        result[region.name] = {
            "detected": count >= min_points,
            "point_count": count,
        }
    return result
