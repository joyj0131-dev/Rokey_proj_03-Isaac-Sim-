#!/usr/bin/env python3
"""v4 주차장 사이트 맵 — 에셋 라벨과 우리 프로세스 용어 사이의 **유일한** 변환 지점.

사용자 규약: 입차(ENTRY) = z 양수, 출차(EXIT) = z 음수.

v4 에셋은 정반대로 라벨링돼 있다. `EntryVehicleWait` 에
`parking:center = (-8.5, 0, -5.5)`, `parking:direction = "inbound"`,
`parking:displayLabel = "입차 차량 대기 구역"` 이 박혀 있어 에셋은 "입차 = z 음수"
라고 말한다. 마커 serves 값(W_IN, D_IN_1 ...)도 같은 규약이다.

우리는 사용자 규약을 채택하므로 이 모듈이 반전을 흡수한다. **다른 코드는 ENTRY/EXIT
만 쓰고 z 부호나 에셋 라벨을 직접 참조하지 않는다.** 팀원이 하루 사이 v2->v3->v4 를
냈으므로 앞으로도 에셋 라벨은 계속 우리와 반대로 말할 것이다.
"""

ENTRY = "ENTRY"
EXIT = "EXIT"

# 에셋 serves -> 우리 프로세스. 에셋의 OUT 계열이 우리 ENTRY(입차)다.
SERVES_ROLE = {
    # z 양수 = 입차
    "GATE_OUT": ENTRY, "W_OUT": ENTRY,
    "D_OUT_1": ENTRY, "D_OUT_2": ENTRY,
    "XN": ENTRY, "A1'": ENTRY, "A2'": ENTRY, "A3'": ENTRY,
    # 진입 회랑 드리프트 보정 마커(z=+7.075 라인, 유일 ID 61~64).
    "LANE_1": ENTRY, "LANE_2": ENTRY, "LANE_3": ENTRY, "LANE_4": ENTRY,
    # 주차 슬롯 진입 마커(2026-07-27, 유일 ID 65~70). _F=주차칸 앞(z≈3.5),
    # _C=슬롯 가운데 최종정지(z≈0). 세 슬롯 A1/A2/A3.
    "A1_F": ENTRY, "A1_C": ENTRY, "A2_F": ENTRY, "A2_C": ENTRY,
    "A3_F": ENTRY, "A3_C": ENTRY,
    # 출차(남쪽) 대칭 슬롯 앞 마커(유일 ID 71~73, z≈-3.5). lane 은 기존 id 0/1/2(A1/A2/A3)
    # 를 z=-7.075 로 옮겨 대칭, 가운데(z=0)는 입차와 공유.
    "A1_FX": EXIT, "A2_FX": EXIT, "A3_FX": EXIT,
    # 진입 회랑 드리프트 보정 마커의 출차측 대칭(2026-07-27, 유일 ID 74~77, z=-7.075).
    # LANE_1~4(z=+7.075)는 입차 통로에만 있고 출차 통로엔 대응 마커가 없어 비대칭이었다
    # — LANE_1~4 와 같은 x, z만 반전해 대칭 추가.
    "LANE_1X": EXIT, "LANE_2X": EXIT, "LANE_3X": EXIT, "LANE_4X": EXIT,
    # 인계 베이 입구(Mission Phase C, Task C4 — parking_v4_runner.spawn_bay_marker
    # 가 런타임 스폰. z=+7.075 로 이미 ENTRY 부호 규약을 따른다).
    "BAY_OUT_ENTRY": ENTRY,
    # z 음수 = 출차
    "GATE_IN": EXIT, "W_IN": EXIT,
    "D_IN_1": EXIT, "D_IN_2": EXIT,
    "XS": EXIT, "A1": EXIT, "A2": EXIT, "A3": EXIT,
}

ROBOTS = ("entry_lead", "entry_follow", "exit_lead", "exit_follow")

# 로봇 -> 대기 도크 마커. 도크 마커는 도크 중심과 같은 좌표에 있어,
# 러너가 마커 위치에서 스폰 좌표를 얻는다(좌표 손 입력 금지).
ROBOT_DOCK_MARKER = {
    "entry_lead": "D_OUT_1",
    "entry_follow": "D_OUT_2",
    "exit_lead": "D_IN_1",
    "exit_follow": "D_IN_2",
}


def role_of(serves):
    """마커 serves -> ENTRY | EXIT."""
    try:
        return SERVES_ROLE[serves]
    except KeyError:
        raise KeyError(
            f"알 수 없는 마커 serves={serves!r}. v4 에셋이 바뀌었다면 "
            f"SERVES_ROLE 을 갱신할 것. 알려진 값: {sorted(SERVES_ROLE)}")


def expected_z_sign(role):
    """역할 -> 기대되는 z 부호(+1 / -1)."""
    if role == ENTRY:
        return 1
    if role == EXIT:
        return -1
    raise ValueError(f"알 수 없는 role={role!r}")


def team_of(robot_id):
    """로봇 id -> 소속 팀(ENTRY | EXIT)."""
    try:
        return role_of(ROBOT_DOCK_MARKER[robot_id])
    except KeyError:
        raise KeyError(f"알 수 없는 robot_id={robot_id!r}. 알려진 값: {list(ROBOTS)}")


def validate_markers(markers):
    """마커 실측 z 부호가 배정된 역할과 맞는지 검사한다.

    markers: [{"serves": str, "z": float}, ...]
    반환: 문제 설명 문자열 리스트(빈 리스트면 정상).

    에셋이 갱신돼 좌표 규약이 바뀌면 여기서 큰 소리로 잡힌다.
    """
    problems = []
    for m in markers:
        serves = m["serves"]
        z = float(m["z"])
        role = role_of(serves)
        want = expected_z_sign(role)
        if z == 0.0 or (1 if z > 0 else -1) != want:
            problems.append(
                f"마커 {serves}: 역할 {role} 은 z 부호 {want:+d} 를 기대하는데 "
                f"실제 z={z:+.3f}. 에셋 규약이 바뀌었는지 확인할 것.")
    return problems
