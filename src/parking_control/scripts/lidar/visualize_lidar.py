#!/usr/bin/env python3
"""LiDAR 포인트클라우드 → 4패널 분석 이미지 (3D+Top+Front+Side, 높이 색상).

보고서/PPT 공유용 스냅샷 생성기. Isaac Sim 불필요 — 포인트클라우드
파일(.npy, shape (N,3))만 있으면 이 컴퓨터에서도 그대로 동작한다.
슬롯별 점유 판단은 이 이미지의 목적이 아니다 (그건 detect_occupancy.py의
역할 — 이 스크립트는 그 결과를 재사용해 요약 텍스트만 곁들인다).

실행:
    python3 visualize_lidar.py sample_lidar_pointcloud.npy -o lidar_analysis.png
"""
import _fix_mpl3d  # noqa: F401  (반드시 matplotlib import보다 먼저)

import argparse
import sys
from pathlib import Path

import numpy as np

PKG_ROOT = Path(__file__).resolve().parent.parent.parent
sys.path.insert(0, str(PKG_ROOT))

from parking_control.core.graph import ParkingMap  # noqa: E402
from detect_occupancy import detect, HEIGHT_THRESHOLD_M  # noqa: E402

HEIGHT_MIN, HEIGHT_MAX = 0.0, 5.0
BG = "#0d0d10"
PANEL_BG = "#141418"
INK = "#e8e7e2"
INK2 = "#8b8a84"
GRID = "#2a2a2e"
CMAP = "turbo"


def _style(ax, title):
    ax.set_facecolor(PANEL_BG)
    ax.set_title(title, color=INK, fontsize=11, fontweight="bold", loc="left")
    ax.tick_params(colors=INK2, labelsize=8)
    for spine in ax.spines.values():
        spine.set_color(GRID)


def render(points, results, lidar_positions, out_path, parking_map):
    import matplotlib.pyplot as plt
    from matplotlib.gridspec import GridSpec, GridSpecFromSubplotSpec

    plt.rcParams["font.family"] = "NanumGothic"
    plt.rcParams["axes.unicode_minus"] = False

    x, y, z = points[:, 0], points[:, 1], points[:, 2]
    c = np.clip((z - HEIGHT_MIN) / (HEIGHT_MAX - HEIGHT_MIN), 0, 1)

    fig = plt.figure(figsize=(15, 11.5), facecolor=BG)
    gs = GridSpec(3, 3, height_ratios=[2.1, 1.5, 0.62], hspace=0.4, wspace=0.28,
                 figure=fig, top=0.94, bottom=0.05, left=0.05, right=0.97)

    # 1) 3D 포인트클라우드
    ax3d = fig.add_subplot(gs[0, :], projection="3d")
    ax3d.set_facecolor(BG)
    ax3d.scatter(x, y, z, c=c, cmap=CMAP, s=1.3, vmin=0, vmax=1, linewidths=0)
    ax3d.set_title("LiDAR 설치 후 포인트 클라우드 (3D)", color=INK,
                   fontsize=14, fontweight="bold", loc="left")
    ax3d.text2D(0.0, 0.94, "천장 중앙에 LiDAR 설치 기준", transform=ax3d.transAxes,
               color=INK2, fontsize=9)
    ax3d.view_init(elev=26, azim=-58)
    ax3d.set_box_aspect((1.6, 1, 0.35))
    for axis in (ax3d.xaxis, ax3d.yaxis, ax3d.zaxis):
        axis.set_pane_color((0, 0, 0, 0))
        axis.line.set_color(GRID)
    ax3d.tick_params(colors=INK2, labelsize=7)
    ax3d.set_axis_off()

    # 2) Top View
    axt = fig.add_subplot(gs[1, 0:2])
    _style(axt, "위에서 본 LiDAR 뷰 (Top View)")
    axt.scatter(x, y, c=c, cmap=CMAP, s=2.2, vmin=0, vmax=1, linewidths=0)
    for lx, ly in lidar_positions:
        axt.scatter([lx], [ly], c="#ff3b3b", s=70, marker="o", zorder=5,
                   edgecolors="white", linewidths=1.2)
        for r in (4, 8, 12, 16):
            axt.add_patch(plt.Circle((lx, ly), r, fill=False,
                                     color="#ff3b3b", alpha=0.22, linewidth=1))
        axt.annotate("LiDAR 위치", (lx, ly), color="#ff5c5c", fontsize=9,
                    fontweight="bold", xytext=(8, -14), textcoords="offset points")
    axt.set_aspect("equal")
    axt.set_xlabel("x (m)", color=INK2, fontsize=8)
    axt.set_ylabel("y (m)", color=INK2, fontsize=8)

    # 3) Front / Side (같은 셀을 위아래로 분할)
    sub = GridSpecFromSubplotSpec(2, 1, subplot_spec=gs[1, 2], hspace=0.55)
    axf = fig.add_subplot(sub[0, 0])
    _style(axf, "정면 단면 뷰 (Front View)")
    axf.scatter(x, z, c=c, cmap=CMAP, s=1.3, vmin=0, vmax=1, linewidths=0)
    axf.set_ylim(0, HEIGHT_MAX)
    axf.set_xlabel("x (m)", color=INK2, fontsize=7)
    axf.set_ylabel("높이 (m)", color=INK2, fontsize=7)

    axs = fig.add_subplot(sub[1, 0])
    _style(axs, "측면 단면 뷰 (Side View)")
    axs.scatter(y, z, c=c, cmap=CMAP, s=1.3, vmin=0, vmax=1, linewidths=0)
    axs.set_ylim(0, HEIGHT_MAX)
    axs.set_xlabel("y (m)", color=INK2, fontsize=7)
    axs.set_ylabel("높이 (m)", color=INK2, fontsize=7)

    # 4) 하단 3박스: 컬러 범례 / 상황 요약 / 활용 예시
    axc = fig.add_subplot(gs[2, 0])
    axc.set_facecolor(PANEL_BG)
    axc.set_title("Point Color (높이)", color=INK, fontsize=10,
                  fontweight="bold", loc="left")
    gradient = np.linspace(0, 1, 256).reshape(1, -1)
    axc.imshow(gradient, aspect="auto", cmap=CMAP,
              extent=[HEIGHT_MIN, HEIGHT_MAX, 0, 1])
    axc.set_yticks([])
    axc.set_xticks([0.0, 1.5, 3.0, 5.0])
    axc.set_xticklabels(["0.0m", "1.5m", "3.0m", "5.0m"], color=INK2, fontsize=8)
    for spine in axc.spines.values():
        spine.set_visible(False)

    n_occupied = sum(1 for r in results.values() if r["occupied"])
    n_total = len(results)
    # 사람 수 추정(거친 휴리스틱): 주차 슬롯도 벽도 아닌 통로 영역에서, 바닥
    # 노이즈를 뺀 점 개수를 사람 1명당 평균 포인트 수로 나눈다. 슬롯 박스
    # 안(차량)과 동서 벽 부근은 제외해야 오염되지 않는다 — 처음엔 이걸
    # 빼먹어서 사람 2명이 29명으로 잡히는 버그가 있었다. 실제 데이터로
    # 바꾸면 이 어림수(POINTS_PER_PERSON)도 재보정이 필요할 가능성이 높다.
    half_w = parking_map.meta["params"]["space_count"] * \
        parking_map.meta["params"]["space_width"] / 2
    aisle_half = parking_map.meta["params"]["aisle_width"] / 2
    POINTS_PER_PERSON = 55
    in_aisle = (np.abs(y) < aisle_half) & (np.abs(x) < half_w) & \
        (z > HEIGHT_THRESHOLD_M)
    n_people_est = max(0, round(in_aisle.sum() / POINTS_PER_PERSON))
    summary_lines = [
        f"- 차량: 총 {n_occupied}대 주차 ({n_occupied}/{n_total}면)",
        f"- 사람(추정): {n_people_est}명",
        "- 기둥 및 벽체: 포인트 클라우드로 인식",
        f"- 빈 주차면: {n_total - n_occupied}면 (포인트가 거의 없음)",
    ]
    axsum = fig.add_subplot(gs[2, 1])
    axsum.axis("off")
    axsum.set_facecolor(PANEL_BG)
    axsum.add_patch(plt.Rectangle((0, 0), 1, 1, transform=axsum.transAxes,
                                  facecolor=PANEL_BG, edgecolor=GRID))
    axsum.text(0.04, 0.88, "현재 상황 요약", color=INK, fontsize=10,
              fontweight="bold", transform=axsum.transAxes, va="top")
    axsum.text(0.04, 0.68, "\n".join(summary_lines), color=INK2, fontsize=8.5,
              transform=axsum.transAxes, va="top", linespacing=1.7)

    usage_lines = [
        "- 주차면 점유 여부 판단",
        "- 사람 및 장애물 감지",
        "- 로봇 이동 경로 계획",
        "- 충돌 방지 및 안전 모니터링",
    ]
    axuse = fig.add_subplot(gs[2, 2])
    axuse.axis("off")
    axuse.set_facecolor(PANEL_BG)
    axuse.add_patch(plt.Rectangle((0, 0), 1, 1, transform=axuse.transAxes,
                                  facecolor="#1c1830", edgecolor="#4a3aa7"))
    axuse.text(0.04, 0.88, "활용 예시", color="#c7bfff", fontsize=10,
              fontweight="bold", transform=axuse.transAxes, va="top")
    axuse.text(0.04, 0.68, "\n".join(usage_lines), color="#a79ee0", fontsize=8.5,
              transform=axuse.transAxes, va="top", linespacing=1.7)

    fig.savefig(out_path, dpi=150, facecolor=fig.get_facecolor())
    print(f"분석 이미지 저장: {out_path}")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("pointcloud", type=Path)
    parser.add_argument("--map-yaml", type=Path,
                        default=PKG_ROOT / "config" / "parking_map.yaml")
    parser.add_argument("--lidar-pos", type=str, default="0,0",
                        help="LiDAR 좌표들, 'x1,y1;x2,y2' 형식 (기본: 중앙 1대)")
    parser.add_argument("-o", "--output", type=Path,
                        default=Path("lidar_analysis.png"))
    args = parser.parse_args()

    points = np.load(args.pointcloud)
    parking_map = ParkingMap.load(args.map_yaml)
    results = detect(points, parking_map)
    lidar_positions = [
        tuple(map(float, pair.split(",")))
        for pair in args.lidar_pos.split(";")
    ]

    render(points, results, lidar_positions, args.output, parking_map)


if __name__ == "__main__":
    main()
