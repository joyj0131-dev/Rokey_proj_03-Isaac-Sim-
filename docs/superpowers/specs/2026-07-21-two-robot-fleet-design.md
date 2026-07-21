# 2로봇 동시 제어 (분리 주행 + 합류 + 결합 운반) — 설계

> 작성: 2026-07-21. 관련: `HANDOFF.md` 5절(서비스 설계 결정), `ARUCO_PLAN.md`(측위 트랙),
> `origin/feature/parking-control`의 `src/parking_control`(팀 정식 관제 — 본 설계의 정렬 대상).

## 배경

지금까지 2로봇이 동시에 움직인 것은 `two_robot_carry_demo.py`(Isaac 내부 파이썬이
Articulation 2개에 직접 명령)뿐이고, ROS 2 제어는 로봇 1대(전역 `/cmd_vel` 하드코딩)까지만
구현되어 있다. 이 설계는 **로봇 2대가 ROS 2 노드 구조에서 각자 자율 주행하되 충돌 없이
움직이는 것**을 만든다.

핵심 개념 (사용자와 합의된 계층 구조):

```
관제/교통 계층: "누가 언제 어느 구역을 쓰나"  ← 구역 통행권(존 락)
주행 계층:      "받은 목표까지 어떻게 가나"    ← 웨이포인트 폐루프 (Phase 2에서 nav2로 교체)
```

- 주행은 로봇마다 독립 (1로봇 자율주행 × 2). 조율은 **구역 통행권 중재** 하나뿐이다.
- 로봇은 서로를 감지하지 않는다 — 같은 구역에 동시에 존재하는 상황 자체를 차단한다.
- 팀 정식 관제(`parking_control`)의 존 락과 동일 개념. 프로토콜 시맨틱을 맞춰 두고,
  이번 데모는 DB 없이 인메모리 장부로 시작한다 (후술).

## 범위

**포함** (사용자가 만들 두 코드):
1. **잡으러 이동** — 두 로봇이 각자 출발해 교차 구간을 양보로 통과하고 대상 차량의
   앞/뒤 대기점에 도착 (분리 주행 + 합류)
2. **잡고 이동** — 파지 완료 후 두 로봇을 캐리어 1대로 묶어 운반 주행 (결합 주행)

**제외**:
- 차량 파지/하차 동작 — **팀원이 별도 함수로 제작**. 본 설계는 미션 상태기계에
  훅(서비스 호출 자리)만 둔다. 임시로 기존 `/arm_control`(SetBool) 사용.
- ArUco 실측위 — 측위는 GT(`/robotN/odom`)로 시작, M6 완성 시 동일 토픽·프레임
  계약으로 백엔드만 교체.
- nav2 — Phase 2 주행 백엔드. 목표 인터페이스를 NavigateToPose 호환으로 유지해
  교체 경로를 보존한다 (HANDOFF 4-A 원칙).
- 풀 ENTRY/EXIT 작업 파이프라인, 2로봇 페어의 관제 스키마 모델링 — parking_control
  통합 시 별도 작업.

**대기 항목**: 주차장 디자인이 팀원 수정 중. 확정되면 `marker_layout.py` 상수를 갱신해
마커 배치와 존 그래프를 **같은 소스에서 재파생**한다(재계산·재실행만, 노드 코드 불변).

## 노드 구조

```
┌─ Isaac Sim ─ two_robot_bringup.py (신규) ────────────────────────┐
│  주차장 + 로봇 2대 (서/동 도크 스폰)                                │
│  로봇별 C++ OmniGraph 브리지:                                     │
│   · ROS2SubscribeTwist ← /robot1/cmd_vel, /robot2/cmd_vel        │
│   · odom 발행 → /robot1/odom, /robot2/odom  (GT)                 │
│  내부 rclpy: /robotN/arm_control 서비스 (파지 함수 연결 자리)       │
└──────────────────────────────────────────────────────────────────┘
                      ↕ DDS (ROS_DOMAIN_ID=126, 기존 env 그대로)
┌─ 외부 ROS 2 Humble ─ src/parkbot_fleet (신규 패키지) ─────────────┐
│ ① fleet_coordinator_node ×1 — 통행권 장부                         │
│ ② robot_navigator_node ×2 (/robot1, /robot2) — 잡으러 이동        │
│ ③ carrier_controller_node ×1 — 잡고 이동                          │
│ ④ demo_mission_node ×1 — 시나리오 상태기계 + 리포트                │
│    (순수 로직 모듈: zone_ledger.py, lane_zones.py,                │
│     waypoint_follower.py — ROS 불의존, 단위테스트 대상)            │
└──────────────────────────────────────────────────────────────────┘
```

### ① fleet_coordinator_node

- 서비스: `/fleet/acquire_zone` (robot_id, zone_id → granted: bool),
  `/fleet/release_zone` (robot_id, zone_id).
- 장부는 `zone_ledger.py`(순수 파이썬 dict) — 불변식: 존당 점유자 최대 1,
  release는 소유자만 가능(아니면 무시+경고).
- **parking_control 호환 전략**: 시맨틱을 `acquire_zones/release_zones`(MySQL PK
  INSERT 원자성)와 동일하게 유지. 통합 시 이 노드가 해당 서비스로 위임하는
  어댑터가 되거나 제거된다. 로봇 쪽(②③)은 변경 없음.
- 로봇 pose 구독(감시): 점유하지 않은 존에 로봇이 들어와 있으면 경고 로그
  (데모에서는 로그만, 서비스에서는 전체 정지 사유).

### ② robot_navigator_node (로봇마다 1개, 네임스페이스 분리)

- 구독 `/robotN/odom`(절대 pose로 취급 — 지금 GT, M6 후 ArUco 융합 측위),
  발행 `/robotN/cmd_vel`, 액션 서버 `/robotN/navigate_to_pose`.
- 목표 인터페이스는 **NavigateToPose 호환**: `nav2_msgs` 설치 확인 후 사용,
  미설치면 동형 자체 액션으로 시작하고 이름·필드를 맞춰 둔다 (확인 항목).
- 내부 루프 (~50 Hz):
  1. pose 갱신 (odom 구독 최신값)
  2. 현재 웨이포인트 대비 오차(x, z, yaw) → P제어 → cmd_vel
     (기존 검증된 hold 보정 게인 재사용, `m3_drive_localize_test.py` 계열)
  3. **존 게이트**: 다음 존 경계 1.0 m 전에 acquire 선요청.
     허가 → 감속 없이 통과 / 거절 → 경계 0.5 m 앞 정지, 1 s 간격 재시도
  4. 존을 완전히 벗어나면(로봇 중심이 경계+마진 통과) release
- 규칙: **다음 존을 획득한 뒤에만 현재 존을 release** (자기 위치는 항상 점유 중).
- 경로(웨이포인트+존 열)는 `lane_zones.py`가 `marker_layout.py`에서 파생:
  차선 2줄(z=±2.5), 존 = 마커 간격 3.40 m 세그먼트 + 합류 구간 존.

### ③ carrier_controller_node

- 서비스 `/carrier/engage` (robot1+robot2를 캐리어로 등록, facing 부호 포함),
  `/carrier/disengage`.
- 결합 중: 캐리어 목표(액션 `/carrier/navigate_to_pose`) → 같은 웨이포인트
  추종 루프를 캐리어 기준점(두 로봇 중점)으로 돌리고, 산출 twist를
  **`/robot1/cmd_vel`·`/robot2/cmd_vel`에 동시 발행** — facing 부호 반영
  (마주 보는 배치, `two_robot_carry_demo.py`의 ±facing과 동일), **wz=0 강제**
  (차량 회전 금지 결정. wz≠0이면 각 로봇이 자기 중심 회전 → 파지 파손).
- 존 허가는 캐리어 명의 1세트 (`robot_id="carrier"`). 캐리어 footprint는
  차량 길이를 포함하므로 진입 판정 마진을 차량 전장(최대 Pickup 기준)으로 확대.
- 결합 중 ②의 두 navigator는 cmd_vel 발행을 중지한 대기 상태(액션 목표 없음).
  발행 주체 충돌 방지를 위해 engage 시 ②에 대기 확인(활성 goal이 있으면 거절).

### ④ demo_mission_node

상태기계 (승인된 데모 시나리오):

```
DISPATCH → APPROACH: ②robot1에 "차량 앞 대기점", ②robot2에 "뒤 대기점" goal
        → 두 액션 완료 대기 (교차 구간 양보는 ①②가 자동 처리)
→ GRIP:   [팀원 파지 함수 훅 — 지금은 /robotN/arm_control(true) 순차 호출]
→ ENGAGE: /carrier/engage
→ CARRY:  ③에 목표 슬롯 앞 경유점 goal
→ RELEASE:[팀원 하차 함수 훅 — 지금은 /arm_control(false)] → disengage
→ DONE:   리포트 JSON 저장
```

- 각 단계 타임아웃 보유. 실패 시 즉시 양 로봇 cmd_vel=0 + 사유 기록 후 종료.
- 리포트: 두 로봇 전체 궤적, 로봇 간 최소 거리, 존 점유 타임라인(acquire/release
  이벤트 로그), 단계별 소요 시간, 실패 사유.

## 좌표/프레임

- 작업 프레임은 **기존 검증 규약 유지**: 월드 XZ 평면, Y-up,
  `ψ = atan2(fwd_x, fwd_z)` (ARUCO_PLAN 0절 — 새로 정의하지 않는다).
- `parking_control`의 `ros_map`(ros_x=usd_x, ros_y=−usd_z) 변환은 통합 시점에
  어댑터에서 1회 적용 (그 변환의 실측 검증도 그때 — 팀원 코드 ★경고 항목).

## 데드락 방지

존 락만으로는 "r1이 Z1 들고 Z2 대기, r2가 Z2 들고 Z1 대기" 사이클이 가능하다.
2대 데모 규칙: **경로 배정 시점에 반대 방향으로 같은 존 열을 공유하는 경로를 감지하면
후순위 로봇을 공유 구간 진입 전 노드에서 대기**시킨다(선순위가 전 구간 반납할 때까지).
기본 시나리오는 차선이 분리되어(robot1: z=+2.5 동진, robot2: z=−2.5 서진) 공유 존이
합류 구간뿐이므로, 이 규칙으로 사이클이 원천 차단된다. N대 일반화는 parking_control
통합 후 과제.

## 에러 처리 (실패는 전부 정지 방향으로)

| 상황 | 동작 |
|---|---|
| acquire 거절 지속 | 경계 앞 정지 유지, 미션 타임아웃이 최종 판정 |
| coordinator 무응답 | 다음 존 진입 불가 → 현재 존에서 정지 (통신 두절 = 정지) |
| odom 끊김 (> 0.5 s) | 해당 로봇 즉시 cmd_vel=0 (측위 모르면 경계 판정 불가) |
| 비점유 존에서 로봇 감지 | 경고 로그 (서비스에서는 전체 정지 사유로 승격 예정) |
| 미션 단계 타임아웃 | 양 로봇 정지 + 리포트에 사유 기록 후 종료 |

ArUco 교체 후에는 "마커 무보정 거리 > 임계"도 정지 조건에 추가된다 (GT에는 해당 없음).

## 테스트 계획

1. **단위 (ROS/Isaac 불필요)**: `zone_ledger.py`(동시 acquire 1승자, 소유자 release,
   재획득), `lane_zones.py`(존 파생이 marker_layout과 정합, 경로→존 열 변환),
   `waypoint_follower.py`(오차→twist 부호, 도착 판정). 기존 관례대로
   `test_*.py` assert 기반, `python3` 직접 실행 + pytest 겸용.
2. **통합 헤드리스**: 브링업 + 노드 4종 + 데모 미션 전체 실행.
   PASS 기준: 두 로봇 대기점 도착(오차 < 0.15 m), **로봇 간 최소 중심거리 ≥ 1.5 m**
   (합류 양보가 실제로 일어났다는 증거로 존 이벤트 로그에 거절→재획득 1회 이상),
   존 동시 점유 위반 0, CARRY 단계 캐리어 변위 목표 도달. 리포트 JSON 저장.
3. **GUI**: 같은 시나리오를 `--gui`로 육안 확인 (기존 스크립트 관례).

검증 수치 근거: 존 3.40 m, 로봇 폭 0.73 m, GT 측위(오차 0), 속도 0.4 m/s →
경계 마진 0.5 m로 존 내 2대 공존 불가 기하가 성립. ArUco 교체 시 최악 드리프트
0.4 m를 마진에 가산해도 성립함을 통합 단계에서 재검증.

## 리스크 / 열린 항목

- `nav2_msgs` 설치 여부 미확인 → 액션 타입 결정이 구현 첫 단계.
- Isaac odom 발행 방식(OmniGraph ComputeOdometry vs 내부 rclpy pose 발행) 실측 선택 필요
  — 원칙은 C++ 브리지 우선.
- 주차장 디자인 미확정 — 존/마커/도크·대기점 좌표가 바뀔 수 있음. 전부
  `marker_layout.py` 파생으로 격리했으므로 확정 후 상수 갱신+재실행.
- 결합(CARRY) 중 개루프 복제 명령의 로봇 간 상대오차 누적 — 단거리(데모)는 차체가
  기계적으로 흡수(검증됨). 장거리(30 m 통로)는 상대자세 폐루프 추가 후보 (범위 밖).
- `parking_robot_interfaces` 패키지가 어느 브랜치에도 없음 — parking_control 통합
  시점의 선결 확보 항목 (본 데모는 의존하지 않음).
- 파지/하차 함수 인터페이스 미정 (팀원 제작 중) — 훅은 서비스 호출 1개 자리로
  격리했으므로 시그니처 확정 시 교체.
