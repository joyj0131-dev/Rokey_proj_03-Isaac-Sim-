#!/usr/bin/env python3
"""ArUco 바닥 마커 배치 정의 (순수 파이썬, 의존성 없음).

USD 빌더(build_marker_layout.py)와 평면도(plot_marker_layout.py)가 이 모듈을 함께 쓴다.
따라서 둘의 결과는 항상 일치한다.

좌표는 전부 팀원 주차장 에셋(parking/build_parking_environment.py) 실측값에서 유도한다.
상수를 손으로 베끼지 않고 같은 치수식을 다시 쓴다.

배치 근거 (2026-07-20 논의):
  - 슬롯이 x축 3.40 m 등간격으로 붙어 있으므로, 슬롯 앞에 하나씩만 놓아도
    통로를 따라 3.40 m 간격의 마커 열(= 주행 차선)이 자동으로 생긴다.
  - 슬롯 입구(z=±4.50)에서 통로 쪽으로 2.0 m 앞(z=±2.50)에 둔다. 1.0 m면 리드 로봇이
    이 열 위를 주행할 때 적재 차량 앞끝이 옆칸 주차 차량(z=4.50)과 5 cm 간섭한다.
    2.0 m면 Sedan(축거 2.951 실측) 기준 1.02 m, 최장 Pickup(3.594) 기준 0.88 m 여유.
  - 3.40 m 간격이 필요한 이유: 메카넘 오도메트리 오차가 자체 측정으로 3~12%(YAW_SCALE
    보정 12%, strafe 드리프트 2.7%). 통로 36.2 m를 무보정으로 가면 1.09~4.34 m가 밀려
    슬롯 폭 3.40 m를 넘는다. 3.40 m마다 보정하면 0.10~0.41 m로 떨어진다.
"""

# --- 팀원 주차장 에셋과 일치해야 하는 치수 ---
SPACE_COUNT = 10
SPACE_WIDTH = 3.40
SPACE_LENGTH = 6.60
AISLE_WIDTH = 9.00
BORDER_MARGIN = 1.10
PARKING_INDICES = tuple(range(1, 9))     # 1~8만 주차면. 0/9는 로봇 대기/충전 도크
# 인계장 — 2026-07-21 재설계(parking_environment_with_markers.usd 기준).
# 주차구역과 같은 단면: 중앙 통로(z=0) 양쪽 z=±7.8 에 대기 베이. 차량은 세로(길이축 z).
# 이전 6베이·3열 배치(HANDOFF_COLUMNS)와 우회 차선(BYPASS)은 폐기됐다.
HANDOFF_LENGTH = 23.0                    # x 방향 (WEST_X-23 ~ WEST_X)
HANDOFF_WIDTH = 24.0                     # z 방향. 07-21 확대(11.4 → 24)

HALF_W = SPACE_COUNT * SPACE_WIDTH * 0.5          # 17.0
HALF_D = AISLE_WIDTH * 0.5 + SPACE_LENGTH         # 11.1
FLOOR_W = HALF_W * 2 + BORDER_MARGIN * 2          # 36.2
FLOOR_D = HALF_D * 2 + BORDER_MARGIN * 2          # 24.4
WEST_X = -HALF_W - BORDER_MARGIN                  # -18.1 (서측 벽 = 인계장 경계)
SLOT_MOUTH_Z = AISLE_WIDTH * 0.5                  # 4.5
HANDOFF_MIN_X = WEST_X - HANDOFF_LENGTH           # -41.1

# --- 마커 파라미터 ---
MARKER_AHEAD = 2.00        # 슬롯 입구에서 통로 쪽으로
LANE_Z = SLOT_MOUTH_Z - MARKER_AHEAD              # ±2.5

# --- ArUco 사전 ---
# 4X4가 아니라 5X5를 쓴다. 44장이면 4X4_50/100으로도 수량은 되지만, 4X4는
# inter-marker Hamming 거리가 작아 최대 1비트만 정정된다(5X5는 3비트).
# 바닥 마커는 타이어 자국·반사·모션블러로 칸이 깨지기 쉬운 조건이고,
# 오검출은 로봇을 엉뚱한 좌표로 순간이동시키는 치명적 실패라 여유가 필요하다.
# 픽셀 예산은 남아돈다(640x480 hFOV90, 0.5m 거리에서 17.7 px/cell. 검출 하한 3~4).
ARUCO_DICT = "DICT_5X5_100"
DICT_BITS = 5
BORDER_CELLS = 1           # ArUco 자체 검은 테두리
QUIET_CELLS = 1            # 흰 여백. 없으면 검출 자체가 안 된다
CODE_CELLS = DICT_BITS + 2 * BORDER_CELLS      # 7
TILE_CELLS = CODE_CELLS + 2 * QUIET_CELLS      # 9

# 타일 크기와 코드 크기를 반드시 구분할 것.
#   MARKER_TILE      = USD 쿼드 한 변 (흰 여백 포함)
#   MARKER_CODE_SIZE = solvePnP 에 넘길 값 (흰 여백 제외)
# 여기서 타일 값을 solvePnP에 주면 거리가 정확히 TILE/CODE = 9/7 = 1.286배,
# 즉 28.6% 어긋난다. 거리 오차가 일정 비율로 나오면 이 줄을 먼저 의심할 것.
MARKER_TILE = 0.25
MARKER_CELL = MARKER_TILE / TILE_CELLS
MARKER_CODE_SIZE = CODE_CELLS * MARKER_CELL    # 0.194444...

# 바닥 도색면(build_parking_environment.PAINT_SURFACE_Y = 0.0008) 바로 위.
# 페인트처럼 보이게 하고 z-fighting은 피하는 높이.
MARKER_Y = 0.0012

# --- ID 체계 ---
# kind별 블록으로 나눈다. 한 kind의 마커 수가 바뀌어도 다른 kind의 ID가 밀리지 않는다.
# (레이아웃을 고칠 때마다 전체 ID가 재배치되면 이미 바닥에 칠한 마커를 다시 칠해야 한다)
ID_BLOCKS = {
    "slot": 0, "dock": 20, "crossing": 30,
    "gateway": 40, "handoff_bay": 50, "handoff_lane": 60,
}
ID_BLOCK_CAP = {
    "slot": 20, "dock": 10, "crossing": 10,
    "gateway": 10, "handoff_bay": 10, "handoff_lane": 20,
}

# 인계장 기하 (07-21 재설계). 베이 z가 실내 슬롯 행(±7.8)과 같아 마커 규칙도 동일하다.
HANDOFF_CENTER_X = WEST_X - HANDOFF_LENGTH * 0.5             # -29.6 (베이 열 x)
HANDOFF_BAY_Z = AISLE_WIDTH * 0.5 + SPACE_LENGTH * 0.5       # 7.8
HANDOFF_BAY_LABELS = ("H_A", "H_B")                          # +z(A쪽) / -z(B쪽)


def slot_columns():
    """10개 구획의 x 중심. index 0/9는 로봇 도크."""
    return [(-HALF_W + (i + 0.5) * SPACE_WIDTH, i) for i in range(SPACE_COUNT)]


def markers():
    """(kind, label, x, z, note) 목록을 돌려준다.

    kind: slot | dock | crossing | gateway | handoff_bay | handoff_lane
    """
    out = []

    # 1) 실내 차선 = 슬롯/도크 마커. 슬롯이 붙어 있어 이것만으로 차선이 된다.
    for row, sign in (("A", 1.0), ("B", -1.0)):
        z = sign * LANE_Z
        for x, i in slot_columns():
            if i in PARKING_INDICES:
                out.append(("slot", f"{row}{i}", x, z, "슬롯 기준점 + 통로 차선"))
            else:
                side = "W" if i == 0 else "E"
                role = "대기" if i == 0 else "충전"
                out.append(("dock", f"{row}·{side}", x, z, f"로봇 {role} 도크"))

    # 2) A열↔B열 전환용 횡단 마커.
    #    슬롯 열과 x가 정확히 맞아야 그래프에서 차선과 연결된다("같은 x" 엣지 규칙).
    #    슬롯 열은 ±1.7, ±5.1, ±8.5, ±11.9, ±15.3 이므로 x=0.0 같은 값은 어느 열과도
    #    안 맞아 고립 노드가 된다. 실제로 그렇게 만들었다가 경로탐색에서 확인되어 제거했다.
    for tag, x in (
        ("W", -15.3),      # 서측 도크 열
        ("2", -8.5),       # A2/B2 사이
        ("7", 8.5),        # A7/B7 사이
        ("E", 15.3),       # 동측 도크 열
    ):
        out.append(("crossing", tag, x, 0.0, "A↔B 전환"))

    # 3) 서측 개구부(폭 9 m, z ∈ [-4.5, 4.5])를 지나는 실내↔실외 경계 마커.
    for sign in (1.0, -1.0):
        out.append(("gateway", "GW" if sign > 0 else "GW'", WEST_X, sign * LANE_Z,
                    "실내↔인계장 경계"))

    # 4) 인계 베이 기준점 (07-21 재설계). 베이가 실내 슬롯과 같은 단면(z=±7.8,
    #    길이축 z)이므로 슬롯 마커와 같은 규칙을 쓴다: 베이 입구(z=±4.5)에서
    #    통로 쪽 2.0 m 앞(z=±2.5), x는 베이 열 중심. 로봇 2대는 실내 슬롯과
    #    동일하게 통로에서 z 방향으로 줄지어 하부 진입한다.
    for label, sign in ((HANDOFF_BAY_LABELS[0], 1.0), (HANDOFF_BAY_LABELS[1], -1.0)):
        out.append((
            "handoff_bay", label,
            HANDOFF_CENTER_X, sign * LANE_Z,
            "인계 베이 기준점 + 하부 진입 정렬",
        ))

    # 5) 인계장 통로 차선. 실내 차선(z=±2.5)을 게이트 마커 너머 서쪽으로 연장한다.
    #    x 격자는 베이 열(HANDOFF_CENTER_X)에 앵커를 두고 3.40 m 간격 —
    #    베이 기준점(4번)이 격자의 중심 열이 되어 차선과 자연히 이어진다.
    #    동쪽 끝(-19.4)과 게이트(-18.1) 사이는 1.3 m로 자투리, 서쪽 끝(-39.8)은
    #    서벽(-41.1)에서 1.3 m 여유.
    lane_x = [HANDOFF_CENTER_X + k * SPACE_WIDTH for k in (3, 2, 1, -1, -2, -3)]
    for sign in (1.0, -1.0):
        for x in lane_x:
            k = round((x - HANDOFF_CENTER_X) / SPACE_WIDTH)
            tag = f"H{'E' if k > 0 else 'W'}{abs(k)}"
            out.append((
                "handoff_lane", tag if sign > 0 else tag + "'",
                x, sign * LANE_Z,
                "인계장 통로 차선",
            ))

    # 6) 인계장 A↔B 전환 마커. 실내 crossing과 같은 규칙(차선 열과 같은 x).
    #    베이 열 양옆 2칸(±6.8) 열에 둔다. 순서상 실내 crossing(ID 30~33) 뒤에
    #    붙으므로 실내 ID는 밀리지 않는다.
    for tag, x in (("HE", HANDOFF_CENTER_X + 2 * SPACE_WIDTH),
                   ("HW", HANDOFF_CENTER_X - 2 * SPACE_WIDTH)):
        out.append(("crossing", tag, x, 0.0, "인계장 A↔B 전환"))

    return out


# 마커의 평면내 방향(yaw, 도). 바닥에 그려진 아루코 텍스처가 기준 방향(canonical)에서
# 얼마나 돌아가 있는지를 나타낸다. **측위(marker_localizer)·평면도(build_marker_layout)·
# 지도(build_marker_textures)가 이 한 값을 공유**해야 json 과 실제 바닥이 어긋나지 않는다.
#
# 지금은 모든 마커를 같은 방향으로 깐다 → 전부 0. 특정 마커(예: 어느 교차점)를 돌려
# 깔고 싶으면 여기서 kind/label 별로 지정한다. 0 이 아닌 값을 쓰면 그 부호가 USD
# 배치(AddRotateYOp)와 측위(rot_y)에서 일치하는지 GT 로 한 번 확인할 것
# (align_yaw 캘리브와 같은 방식 — m3_localize_demo).
MARKER_YAW_BY_KIND = {
    "slot": 0.0, "dock": 0.0, "crossing": 0.0,
    "gateway": 0.0, "handoff_bay": 0.0, "handoff_lane": 0.0,
}


def marker_yaw(kind, label, x, z):
    """마커의 평면내 방향[도]. 기본은 kind 별 값(현재 전부 0)."""
    return MARKER_YAW_BY_KIND.get(kind, 0.0)


def assign_ids():
    """(marker_id, kind, label, x, z, yaw, note) 목록.

    markers()의 순서에 결정적으로 종속된다. 같은 레이아웃이면 항상 같은 ID가 나온다.
    yaw 는 마커의 평면내 방향[도] (marker_yaw 참고).
    """
    seen = {}
    out = []
    for kind, label, x, z, note in markers():
        index = seen.get(kind, 0)
        seen[kind] = index + 1
        if index >= ID_BLOCK_CAP[kind]:
            raise ValueError(
                f"{kind} ID 블록 초과: {index + 1}개째인데 상한이 {ID_BLOCK_CAP[kind]}. "
                "ID_BLOCKS/ID_BLOCK_CAP을 조정할 것(다른 kind와 겹치지 않게)."
            )
        yaw = marker_yaw(kind, label, x, z)
        out.append((ID_BLOCKS[kind] + index, kind, label, x, z, yaw, note))

    ids = [r[0] for r in out]
    if len(set(ids)) != len(ids):
        raise ValueError("marker ID 중복")
    if max(ids) >= 100:
        raise ValueError(f"ID {max(ids)} 가 {ARUCO_DICT} 범위(0~99)를 넘음")
    return out


def summary():
    rows = markers()
    kinds = {}
    for kind, *_ in rows:
        kinds[kind] = kinds.get(kind, 0) + 1
    return rows, kinds


if __name__ == "__main__":
    rows, kinds = summary()
    tagged = assign_ids()
    print(f"실내 바닥 {FLOOR_W:.1f} x {FLOOR_D:.1f} m, 인계장 {HANDOFF_LENGTH:.1f} x {HANDOFF_WIDTH:.1f} m")
    print(f"차선 z = ±{LANE_Z:.2f} m (슬롯 입구 ±{SLOT_MOUTH_Z:.2f}에서 {MARKER_AHEAD:.1f} m 앞)")
    print(f"사전 {ARUCO_DICT}, 타일 {MARKER_TILE:.3f} m / 코드 {MARKER_CODE_SIZE:.6f} m "
          f"({TILE_CELLS}칸 중 {CODE_CELLS}칸)")
    print()
    print(f"  {'ID':>4} {'종류':<14}{'대상':<14}{'x':>9}{'z':>9}{'yaw':>6}")
    for mid, kind, label, x, z, yaw, _note in tagged:
        print(f"  {mid:>4} {kind:<14}{label:<14}{x:>9.2f}{z:>9.2f}{yaw:>6.0f}")
    print()
    print("  합계:", ", ".join(f"{k} {v}" for k, v in kinds.items()), f"= {len(rows)}장")
    print(f"  ID 범위: {min(r[0] for r in tagged)} ~ {max(r[0] for r in tagged)}, 중복 없음")
