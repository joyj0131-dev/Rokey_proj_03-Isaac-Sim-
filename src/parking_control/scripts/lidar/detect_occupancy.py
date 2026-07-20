#!/usr/bin/env python3
"""LiDAR 포인트클라우드 → 슬롯별 점유 판단 (실시간 운영용 경량 로직).

폴리시드 분석 이미지(visualize_lidar.py)와 목적이 다르다 — 이건 계산이
가벼워서 반복 실행에 적합하다. 실제 운영에서는 이 판정 결과(dict)만
저희 관제 시스템(parking_slots.status)에 전달하면 된다. 이미지는 이
스크립트를 사람이 눈으로 검증할 때만 필요한 진단용 부산물이다.

판정 방법: 각 슬롯의 XY 사각형 범위 안에 있으면서, 바닥 노이즈를 걸러내는
높이 임계값(HEIGHT_THRESHOLD_M)보다 높은 점이 POINT_THRESHOLD개 이상이면
점유로 판단한다. (실제 데이터로 바꾸면 이 두 임계값은 재보정이 필요할 수 있음)

실행:
    python3 detect_occupancy.py ../../config/sample_lidar_pointcloud.npy
    → 판정 결과 JSON을 stdout에 출력 + occupancy_check.png 저장 (진단용)
"""
import argparse
import json
import sys
from pathlib import Path

import numpy as np

PKG_ROOT = Path(__file__).resolve().parent.parent.parent
sys.path.insert(0, str(PKG_ROOT))

from parking_control.core.graph import ParkingMap  # noqa: E402

HEIGHT_THRESHOLD_M = 0.15   # 이보다 낮으면 바닥/차선 노이즈로 간주하고 무시
POINT_THRESHOLD = 30        # 이 개수 이상이면 점유로 판단

EMPTY_COLOR = "#008300"
OCCUPIED_COLOR = "#e34948"


def detect(points, parking_map):
    """슬롯 id -> {occupied, point_count, x, y} 딕셔너리 반환."""
    W = parking_map.meta["params"]["space_width"]
    L = parking_map.meta["params"]["space_length"]
    results = {}
    for slot_id in parking_map.nodes_of_kind("slot"):
        cx, cy = parking_map.node_pos(slot_id)
        in_box = (
            (points[:, 0] > cx - W / 2) & (points[:, 0] < cx + W / 2) &
            (points[:, 1] > cy - L / 2) & (points[:, 1] < cy + L / 2) &
            (points[:, 2] > HEIGHT_THRESHOLD_M)
        )
        count = int(in_box.sum())
        results[slot_id] = dict(
            occupied=count >= POINT_THRESHOLD, point_count=count, x=cx, y=cy)
    return results


def render(points, results, parking_map, out_path):
    import matplotlib.pyplot as plt
    from matplotlib.patches import Rectangle
    plt.rcParams["font.family"] = "NanumGothic"
    plt.rcParams["axes.unicode_minus"] = False

    W = parking_map.meta["params"]["space_width"]
    L = parking_map.meta["params"]["space_length"]

    fig, ax = plt.subplots(figsize=(11, 6.5), facecolor="#1a1a19")
    ax.set_facecolor("#1a1a19")

    floor = points[points[:, 2] < HEIGHT_THRESHOLD_M]
    above = points[points[:, 2] >= HEIGHT_THRESHOLD_M]
    ax.scatter(floor[:, 0], floor[:, 1], s=1, c="#3a3a37", alpha=0.4,
               label="바닥(높이 임계값 이하, 무시됨)")
    ax.scatter(above[:, 0], above[:, 1], s=1.5, c="#c3c2b7", alpha=0.6,
               label="판정에 사용된 점")

    for slot_id, r in results.items():
        color = OCCUPIED_COLOR if r["occupied"] else EMPTY_COLOR
        ax.add_patch(Rectangle(
            (r["x"] - W / 2, r["y"] - L / 2), W, L,
            fill=True, facecolor=color, alpha=0.35,
            edgecolor=color, linewidth=1.8))
        label = "점유" if r["occupied"] else "공석"
        ax.text(r["x"], r["y"] + 0.4, slot_id, ha="center",
               color="#ffffff", fontsize=10, fontweight="bold")
        ax.text(r["x"], r["y"] - 0.6, f"{label} ({r['point_count']}pt)",
               ha="center", color="#ffffff", fontsize=8)

    ax.set_aspect("equal")
    ax.set_xlabel("x (m)", color="#c3c2b7")
    ax.set_ylabel("y (m)", color="#c3c2b7")
    ax.tick_params(colors="#c3c2b7")
    for spine in ax.spines.values():
        spine.set_color("#3a3a37")
    ax.legend(loc="upper right", fontsize=8, facecolor="#242422",
              labelcolor="#c3c2b7", edgecolor="#3a3a37")

    n_occ = sum(1 for r in results.values() if r["occupied"])
    ax.set_title(
        f"LiDAR 기반 슬롯 점유 판단 — {n_occ}/{len(results)}대 주차 중"
        f"  (높이 임계값 {HEIGHT_THRESHOLD_M}m · 포인트 임계값 {POINT_THRESHOLD}개)",
        color="#ffffff", fontsize=12)
    fig.tight_layout()
    fig.savefig(out_path, dpi=150, facecolor=fig.get_facecolor())
    print(f"진단 이미지 저장: {out_path}")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("pointcloud", type=Path)
    parser.add_argument("--map-yaml", type=Path,
                        default=PKG_ROOT / "config" / "parking_map.yaml")
    parser.add_argument("-o", "--output", type=Path,
                        default=Path("occupancy_check.png"))
    args = parser.parse_args()

    points = np.load(args.pointcloud)
    parking_map = ParkingMap.load(args.map_yaml)
    results = detect(points, parking_map)

    print(json.dumps(
        {k: dict(occupied=v["occupied"], point_count=v["point_count"])
         for k, v in results.items()}, indent=2, ensure_ascii=False))

    render(points, results, parking_map, args.output)


if __name__ == "__main__":
    main()
