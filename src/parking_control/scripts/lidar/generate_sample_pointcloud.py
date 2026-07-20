#!/usr/bin/env python3
"""LiDAR 포인트클라우드 샘플 데이터 생성기 (테스트용 — 실제 센서 데이터 아님).

detect_occupancy.py / visualize_lidar.py를 Isaac Sim 없이 미리 검증하기
위한 가짜 데이터. 주차장 치수는 parking_map.yaml(같은 원천)에서 가져오므로
레이아웃이 바뀌어도 이 스크립트를 다시 돌리면 그대로 반영된다.

실행:
    python3 generate_sample_pointcloud.py
    → config/sample_lidar_pointcloud.npy 생성 (shape (N,3): x,y,z 미터)
"""
import sys
from pathlib import Path

import numpy as np

PKG_ROOT = Path(__file__).resolve().parent.parent.parent
sys.path.insert(0, str(PKG_ROOT))

from parking_control.core.graph import ParkingMap  # noqa: E402

MAP = ParkingMap.load(PKG_ROOT / "config" / "parking_map.yaml")
P = MAP.meta["params"]
RNG = np.random.default_rng(7)
WALL_HEIGHT = 5.6

# 데모용으로 점유시킬 슬롯 (실측 아님, 임의 지정 — occupancy 판정 검증용 정답지 역할)
OCCUPIED_SLOTS = {"A3", "A4", "A6", "B2", "B3", "B5", "B7"}


def _box(cx, cy, z0, sx, sy, sz, n):
    return np.column_stack([
        RNG.uniform(cx - sx / 2, cx + sx / 2, n),
        RNG.uniform(cy - sy / 2, cy + sy / 2, n),
        RNG.uniform(z0, z0 + sz, n),
    ])


def build():
    half_w = P["space_count"] * P["space_width"] * 0.5
    half_d = P["aisle_width"] * 0.5 + P["space_length"]
    points = []

    # 바닥 (희소, 노이즈용)
    n_floor = 4000
    points.append(np.column_stack([
        RNG.uniform(-half_w - 1, half_w + 1, n_floor),
        RNG.uniform(-half_d - 1, half_d + 1, n_floor),
        RNG.uniform(0, 0.03, n_floor),
    ]))

    # 벽 4면 (테두리를 따라 흩뿌린 점)
    for x in np.linspace(-half_w - 1, half_w + 1, 250):
        points.append(_box(x, half_d + 1, 0, 0.05, 0.05, WALL_HEIGHT, 3))
        points.append(_box(x, -half_d - 1, 0, 0.05, 0.05, WALL_HEIGHT, 3))
    for y in np.linspace(-half_d - 1, half_d + 1, 180):
        points.append(_box(-half_w - 1, y, 0, 0.05, 0.05, WALL_HEIGHT, 3))
        points.append(_box(half_w + 1, y, 0, 0.05, 0.05, WALL_HEIGHT, 3))

    # 기둥 12개 (build_parking_environment.py의 배치와 동일한 규칙)
    for row_z in (half_d - 0.24, -(half_d - 0.24)):
        for k in range(6):
            x = -half_w + k * 2 * P["space_width"] if k < 5 else half_w
            points.append(_box(x, row_z, 0, 0.48, 0.48, WALL_HEIGHT, 40))

    # 주차 슬롯: 점유면 차량 크기 점군, 빈 면은 바닥만
    for node_id, attrs in MAP.graph.nodes(data=True):
        if attrs.get("kind") != "slot":
            continue
        cx, cy = attrs["x"], attrs["y"]
        if node_id in OCCUPIED_SLOTS:
            points.append(_box(cx, cy, 0.30, 1.8, 4.3, 1.1, 900))  # 차체

    # 사람 2명 (통로에 서 있는 것처럼)
    for px, py in [(-6.0, 0.0), (4.0, 0.0)]:
        points.append(_box(px, py, 0.0, 0.35, 0.35, 1.7, 60))

    return np.vstack(points).astype(np.float32)


if __name__ == "__main__":
    pts = build()
    out = PKG_ROOT / "config" / "sample_lidar_pointcloud.npy"
    np.save(out, pts)
    print(f"샘플 포인트클라우드 생성: {out} ({len(pts):,}개 점)")
    print(f"점유로 지정한 슬롯(정답지): {sorted(OCCUPIED_SLOTS)}")
