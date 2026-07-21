#!/usr/bin/env python3
"""마커 배치 평면도(SVG). 의존성 없음 — 시스템 python3로 바로 돈다.

배치는 marker_layout.py 하나만 본다(USD 빌더와 동일 정의).
"""

from pathlib import Path

import marker_layout as ML

OUT_SVG = Path(__file__).resolve().parent / "marker_layout_plan.svg"

# 실내 + 서측 인계장을 한 화면에 담는다.
X_MIN, X_MAX = ML.HANDOFF_MIN_X - 1.0, ML.FLOOR_W * 0.5 + 1.0
Z_MIN, Z_MAX = -ML.FLOOR_D * 0.5 - 1.0, ML.FLOOR_D * 0.5 + 1.0

PPM = 17.0
PAD = 70
W = (X_MAX - X_MIN) * PPM + PAD * 2
H = (Z_MAX - Z_MIN) * PPM + PAD * 2

COLORS = {
    "slot": "#22c55e", "dock": "#3b82f6", "crossing": "#f97316",
    "gateway": "#f43f5e", "handoff_bay": "#eab308", "handoff_lane": "#8b5cf6",
}
LEGEND = [
    ("slot", "슬롯 마커 — 슬롯 입구에서 2 m 앞, 3.40 m 간격. 이것만으로 통로 차선이 된다"),
    ("dock", "도크 마커 — 로봇 대기/충전 구역"),
    ("crossing", "횡단 마커 — A열↔B열 전환 (위치 미확정)"),
    ("gateway", "게이트 마커 — 실내↔인계장 경계 (서측 개구부 9 m)"),
    ("handoff_bay", "인계 베이 기준점 — 베이 길이축이 x라 로봇이 x에서 진입"),
    ("handoff_lane", "인계장 우회 차선 — 베이 열 사이가 0.70 m라 통과 불가 (라우팅 미확정)"),
]


def sx(x):
    return PAD + (x - X_MIN) * PPM


def sy(z):
    return PAD + (Z_MAX - z) * PPM     # +z가 화면 위


def rect(x, z, w, d, **kw):
    a = " ".join(f'{k.replace("_", "-")}="{v}"' for k, v in kw.items())
    return (f'<rect x="{sx(x - w / 2):.1f}" y="{sy(z + d / 2):.1f}" '
            f'width="{w * PPM:.1f}" height="{d * PPM:.1f}" {a}/>')


def text(x, z, s, size=10, fill="#94a3b8", anchor="middle", weight="400", dy=0):
    return (f'<text x="{sx(x):.1f}" y="{sy(z) + dy:.1f}" font-size="{size}" fill="{fill}" '
            f'text-anchor="{anchor}" font-weight="{weight}" '
            f'font-family="ui-monospace,Menlo,monospace">{s}</text>')


def main():
    p = [f'<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 {W:.0f} {H:.0f}" '
         f'width="{W:.0f}" height="{H:.0f}">',
         f'<rect width="{W:.0f}" height="{H:.0f}" fill="#0b1220"/>']

    # 인계장 바닥
    hx = ML.WEST_X - ML.HANDOFF_LENGTH * 0.5
    p.append(rect(hx, 0, ML.HANDOFF_LENGTH, ML.HANDOFF_WIDTH,
                  fill="#131d2c", stroke="#3f6212", stroke_width=2))
    p.append(text(hx, ML.HANDOFF_WIDTH * 0.5 - 0.7, "DRIVER DROP-OFF / HANDOFF",
                  11, "#65a30d", weight="600"))
    for idx, label in enumerate(ML.HANDOFF_VEHICLE_LABELS):
        bx = ML.HANDOFF_COLUMNS[idx // 2]
        bz = ML.HANDOFF_LANE_Z if idx % 2 == 0 else -ML.HANDOFF_LANE_Z
        p.append(rect(bx, bz, ML.HANDOFF_BAY_LENGTH, ML.HANDOFF_BAY_WIDTH,
                      fill="none", stroke="#e2e8f0", stroke_width=1.0,
                      stroke_dasharray="4 3"))
        p.append(text(bx, bz, label, 10, "#cbd5e1", dy=4))

    # 실내 바닥 / 통로
    p.append(rect(0, 0, ML.FLOOR_W, ML.FLOOR_D, fill="#141c2e",
                  stroke="#334155", stroke_width=2))
    p.append(rect(0, 0, ML.FLOOR_W, ML.AISLE_WIDTH, fill="#18233a"))

    # 서측 개구부(벽이 끊긴 9 m 구간)
    p.append(f'<line x1="{sx(ML.WEST_X):.1f}" y1="{sy(ML.AISLE_WIDTH / 2):.1f}" '
             f'x2="{sx(ML.WEST_X):.1f}" y2="{sy(-ML.AISLE_WIDTH / 2):.1f}" '
             f'stroke="#f43f5e" stroke-width="3" stroke-dasharray="6 4"/>')

    # 주차 구획
    for row, sign in (("A", 1.0), ("B", -1.0)):
        zc = sign * (ML.AISLE_WIDTH * 0.5 + ML.SPACE_LENGTH * 0.5)
        for x, i in ML.slot_columns():
            is_slot = i in ML.PARKING_INDICES
            p.append(rect(x, zc, ML.SPACE_WIDTH - 0.11, ML.SPACE_LENGTH, fill="none",
                          stroke="#e2e8f0" if is_slot else "#eab308", stroke_width=1.0,
                          stroke_dasharray="" if is_slot else "5 4"))
            p.append(text(x, zc, f"{row}{i}" if is_slot else ("대기" if i == 0 else "충전"),
                          11, "#e2e8f0" if is_slot else "#eab308", weight="600", dy=4))

    # 기둥
    for sign in (1.0, -1.0):
        for x in (-ML.HALF_W, -ML.HALF_W + 2 * ML.SPACE_WIDTH,
                  -ML.HALF_W + 4 * ML.SPACE_WIDTH, -ML.HALF_W + 6 * ML.SPACE_WIDTH,
                  -ML.HALF_W + 8 * ML.SPACE_WIDTH, ML.HALF_W):
            p.append(rect(x, sign * (ML.HALF_D - 0.24), 0.48, 0.48, fill="#64748b"))

    # 차선(마커 열)
    for sign in (1.0, -1.0):
        z = sign * ML.LANE_Z
        p.append(f'<line x1="{sx(-15.3):.1f}" y1="{sy(z):.1f}" x2="{sx(15.3):.1f}" '
                 f'y2="{sy(z):.1f}" stroke="#22c55e" stroke-width="1.3" '
                 f'stroke-dasharray="7 6" opacity="0.6"/>')
        zb = sign * ML.HANDOFF_BYPASS_Z
        p.append(f'<line x1="{sx(ML.HANDOFF_MIN_X + 1.5):.1f}" y1="{sy(zb):.1f}" '
                 f'x2="{sx(ML.WEST_X - 2.0):.1f}" y2="{sy(zb):.1f}" '
                 f'stroke="#8b5cf6" stroke-width="1.3" stroke-dasharray="7 6" opacity="0.6"/>')

    # 마커
    for kind, label, x, z, _note in ML.markers():
        c = COLORS[kind]
        p.append(f'<circle cx="{sx(x):.1f}" cy="{sy(z):.1f}" r="7" fill="{c}" opacity="0.18"/>')
        p.append(rect(x, z, ML.MARKER_TILE, ML.MARKER_TILE, fill=c,
                      stroke="#0b1220", stroke_width="0.5"))

    p.append(text(0, ML.FLOOR_D * 0.5 - 0.6,
                  f"실내 {ML.FLOOR_W:.1f} x {ML.FLOOR_D:.1f} m", 11, "#64748b"))
    p.append(text(-ML.HALF_W + 1.0, ML.LANE_Z + 0.8,
                  f"차선 z=+{ML.LANE_Z:.1f}", 10, "#22c55e", anchor="start"))
    p.append(text(-ML.HALF_W + 1.0, -ML.LANE_Z - 0.4,
                  f"차선 z=-{ML.LANE_Z:.1f}", 10, "#22c55e", anchor="start"))

    rows, kinds = ML.summary()
    for i, (kind, desc) in enumerate(LEGEND):
        y = H - PAD + 10 + i * 16
        p.append(f'<rect x="{PAD}" y="{y - 8}" width="9" height="9" fill="{COLORS[kind]}"/>')
        p.append(f'<text x="{PAD + 16}" y="{y}" font-size="11" fill="#94a3b8" '
                 f'font-family="ui-monospace,Menlo,monospace">{desc} ({kinds[kind]}장)</text>')
    p.append(f'<text x="{W - PAD:.0f}" y="{PAD - 26}" font-size="12" fill="#64748b" '
             f'text-anchor="end" font-family="ui-monospace,Menlo,monospace">'
             f'마커 타일 {ML.MARKER_TILE:.2f} m · 총 {len(rows)}장</text>')
    p.append("</svg>")

    OUT_SVG.write_text("\n".join(p), encoding="utf-8")
    print(f"[plan] {OUT_SVG}")
    print(f"[plan] 총 {len(rows)}장 — " + ", ".join(f"{k} {v}" for k, v in kinds.items()))


if __name__ == "__main__":
    main()
