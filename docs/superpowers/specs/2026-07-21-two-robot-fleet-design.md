# 2로봇 동시 제어 — Team A 관제와의 융합 설계 (v2)

> 작성: 2026-07-21. v1은 Team A 문서 수령 전 독자 설계였고, 팀원(youngjin)의
> 1~11단계 보고서·작업계획·인터페이스 수정제안을 반영해 전면 개정.
> 관련: `HANDOFF.md` 5절, `ARUCO_PLAN.md`, `origin/feature/parking-control`.

## 배경 — 반쪽씩 완성돼 있다

- **Team A (팀원, 완료·E2E 검증)**: dispatcher가 작업당 로봇 2대(리더+팔로워) 자동
  배정(9단계), `formation_assignment` 토픽으로 역할 방송(11단계), 팔로워
  gap-hold + co-stop 컨트롤러(10단계), 존 락 이중 소유권(통로=robot_id 개별,
  슬롯=task_id 팀 명의, 8단계), MySQL 원장 + 웹 대시보드.
- **비어 있는 자리 (= 이 머신에서 우리가 채울 것)**: 리더 실주행
  `navigate_action_server`(빈 스켈레톤), 실제 `/robotN/odom` 발행(현재
  sim_orchestrator는 DB 좌표만 갱신하는 가짜), Isaac Sim 2로봇 ROS2 통신
  ("rokey 머신 필요, 보류 중"이라 명시된 그 항목).

v1에서 설계한 fleet_coordinator(관제 대역)·carrier_controller(twist 복제)·
demo_mission(배정 역할)은 팀원 실물이 있으므로 **폐기**한다.

## 존 락 호출 규약 (v1의 leg-batch 정정)

팀원 인터페이스_수정제안 Q2의 권장은 **"구간마다"**다: 다음 통로 구간 진입
직전 `acquire_zones` 호출(락 보유 시간 최소화), 거부되면 정지 후
`retry_after_sec` 뒤 재시도, 구간을 벗어난 직후 `release_zones`. 슬롯 존은
로봇이 잡지 않는다 — dispatcher가 task_id 명의로 선점(8단계). 오름차순·
전부-아니면-무 규칙은 한 요청에 여러 존을 담을 때의 계약으로 준수한다.

## 노드 구조 (융합 후)

```
[Team A — 그대로 실행 (가져와서 이 머신에 셋업)]
  task_dispatcher (allocator=nearest|hungarian, zone_lock_mode=db)
  parking_slot_manager, MySQL(스키마 001~004), 웹 대시보드
  → formation_assignment 방송, ExecuteParkingTask 발행, 존 락 서비스 제공

[Team B 실물 — 이번 작업의 본체, src/parkbot_robot(신규 or parking_robot_system 확장)]
  navigate_action_server ×2 (/robot1, /robot2)
    · v1의 robot_navigator가 이 이름/자리로 들어간다
    · 구독 /robotN/odom, 발행 /robotN/cmd_vel
    · 내부 루프(~50Hz): 웨이포인트 P제어(검증된 hold 보정 게인)
      + 구간 존 게이트(진입 직전 acquire, 거부 시 경계 정지·재시도, 이탈 직후 release)
  formation_gap_controller (팀원 노드 재사용 + 메카넘 확장 3종 — 아래)
  (임시) mission trigger: RequestParkingTask 서비스 호출 스크립트
    · B/C의 진짜 robot_task_orchestrator가 오면 대체. sim_orchestrator와
      동시 기동 금지(같은 액션 이름) — 팀원 문서 주의사항 준수.

[Isaac — two_robot_bringup.py (신규)]
  주차장 + 로봇 2대(서/동 도크), 로봇별 C++ 브리지:
  /robotN/cmd_vel 구독, /robotN/odom 발행(GT, ros_map 프레임 변환 후)
  /robotN/arm_control 서비스(파지 함수 훅 자리 — 팀원 C 함수로 교체 예정)
```

## formation_gap_controller 메카넘 확장 3종 (팀원과 합의할 것)

결합 운반은 v1의 twist 복제(개루프)가 아니라 **팀원의 리더-팔로워 gap-hold
폐루프를 기본으로 채택**한다(상대오차 자가 보정 + co-stop 안전장치).
단, 메카넘/차량 제약에서 나온 다음 3개를 추가해야 한다:

1. **결합 중 wz=0 강제** — 회전 명령은 각 로봇이 자기 중심 회전해 파지 파손
   (서비스 결정: 차량 회전 금지. 무적재 시에는 회전 자유).
2. **횡방향 오차항** — 현 gap 제어는 진행축 1차원. 운반 기본 모드가
   게걸음(횡이동)이므로 리더 프레임 좌우 오프셋도 `linear.y`로 폐루프.
3. **facing 부호** — 두 로봇이 마주 보고 파지하므로 같은 월드 방향 이동이
   로봇 좌표로는 부호 반대(`two_robot_carry_demo.py`의 ±facing 검증값 재사용).

## 지도/차선 — 팀 합의 필요한 구조 이슈

팀원 그래프는 중앙 통로 1줄(J0~J10, 존 Z01~Z10이 통로 전폭). 이 구조에서
반대 방향 2대는 구간별 개별 락으로 **정면 교착이 성립**한다(r1이 Z3 쥐고 Z4
대기, r2가 Z4 쥐고 Z3 대기 — 요청 단위 오름차순으로는 안 막힘. P7 "2대 단순
교차" 시나리오에서 드러날 것). 제안:

- `generate_map.py`를 **통로 차선 2줄**(y=∓2.5, 방향별 차선) 노드/존을 뽑도록
  확장. 존 경계 리듬(x=−17+k×3.4)이 ArUco 마커 간격 3.40 m와 동일하므로
  마커 배치와 1:1 정렬된다 (`marker_layout.py`와 파라미터 공유).
- 채택 전까지의 안전책: dispatcher 경로 배정 시 반대 방향 동일 통로 동시
  배정 금지(후순위 대기).

## 좌표/프레임

- ROS 쪽 전부 `ros_map` (ros_x=usd_x, ros_y=−usd_z) — 팀원 규약 그대로.
- 변환은 Isaac 브링업의 odom 발행 지점 1곳. **알려진 지점(도크·슬롯 중심)
  대조 실측 검증이 구현 1순위** — 팀원이 "이사 첫날 할 일 1순위"로 명시한 항목.
- 로봇 내부 yaw 규약(ψ=atan2(fwd_x, fwd_z))과의 변환도 같은 지점에서 처리.

## 측위 (v3에서 변경 — ArUco를 지금 투입)

- **접근·도킹·하차 구간: 진짜 ArUco 측위 폐루프** — M-시리즈(M6)를 이번 미션에서
  완성한다. GT-먼저 방침(v2)은 폐기.
- **운반(결합) 구간만 1차 완성본에서 GT 보조 허용** (2026-07-21 사용자 결정) —
  차를 들면 전방 카메라가 차 하부에 가려 마커 가시성이 제한되기 때문.
  2차에서 마커(z=0 횡단 열)+오도메트리 융합으로 교체한다. 보조/진짜 전환은
  `/robot_N/odom` 발행부 한 곳에만 존재해야 한다(소비자는 출처를 모름).
- 네임스페이스는 팀 확정 규칙 **`/robot_1`, `/robot_2`**(언더스코어)를 따른다.

## 에러 처리 (계층별, 전부 정지 방향)

| 계층 | 장치 |
|---|---|
| 교통(존) | acquire 거부 → 경계 정지·재시도. 관제 무응답 → 다음 구간 진입 불가 |
| 편대 | 팀원 co-stop: 파트너 odom 0.5 s 두절 → 정지, 명시적 정지 방송 → 동반 정지 |
| 로봇 개별 | odom 두절 > 0.5 s → cmd_vel=0. (ArUco 전환 후) 무보정 거리 초과 → 정지 |
| 시설 | safety_monitor obstacle_alert → block_edge (락과 무관하게 즉시 정지 계층) |
| 미션 | 단계 타임아웃 → 양 로봇 정지 + 리포트 기록 |

## 테스트 계획

1. **단위**: 웨이포인트 추종 오차→twist 부호, 존 게이트 상태기계(acquire 거부
   시 정지/재시도/재개), ros_map 변환 왕복. 기존 관례(assert 기반, pytest 겸용).
2. **통합 1 — 통신**: two_robot_bringup + 외부에서 /robotN/cmd_vel 수동 발행
   → 두 로봇 독립 구동 + /robotN/odom 수신 확인 (팀원 "Isaac 2로봇 ROS2 통신
   검증" 항목 해소).
3. **통합 2 — 데모**: Team A 스택(디스패처+슬롯+DB) 실기동 + RequestParkingTask
   1건 → 2대 배정 → 분리 주행(구간 락 경쟁 1회 이상 로그) → 대기점 도착.
   PASS: 도착 오차 < 0.15 m, 존 동시 점유 위반 0, 로봇 간 최소 거리 ≥ 1.5 m.
4. **통합 3 — 결합**: 파지 훅 후 리더 주행 + 팔로워 gap-hold로 운반 구간 이동.
   PASS: 상대거리 오차 유지, co-stop 강제 시나리오(odom 차단) 동작.
5. 리포트 JSON(궤적, 존 이벤트 타임라인, 최소 거리) — 기존 관례.

## v3 — E2E 데모 미션 확정 (2026-07-21 저녁)

> 목표: **"팀원 조율 스택으로 로봇 2대를 제어해, 인계장 아래쪽(H_B) 차량을
> 집어 ArUco 마커를 읽으며 이동, 빈 주차면에 넣는다"** (사용자 정의).

### 미션 단계

| 단계 | 내용 | 재료 |
|---|---|---|
| 0 | MySQL+스키마(001~004), 지도에 인계장 노드/존 확장(marker_layout 파생), Isaac 필드(마커 환경+뎁스캠 로봇 2대), **odom y부호/yaw 실측 확정** | 팀원 generate_map + 우리 마커 좌표 |
| 1 | RequestParkingTask(ENTRY) → 리더+팔로워 배정, find_empty_slot, formation 방송 | 팀원 dispatcher (그대로) |
| 2 | 접근: 리더 ArUco 폐루프 웨이포인트 주행(navigate 자리), 팔로워 gap-hold 호송, 존 락 통과 | 팀원 formation + 우리 M-시리즈 |
| 3 | 도킹: 베이 마커(ID 51) 정렬 → 뎁스 스톱 하부 진입·정지 (리더=먼 축 순차 진입) | 우리 depth-stop 트랙 |
| 4 | 파지: 팔 전개(검증된 ARM_TARGETS; 팀원 C 함수 훅) | 기존 arm 제어 |
| 5 | 운반: 결합 게걸음, task 명의 팀 존 락, **1차는 GT 보조 측위** | 팀원 co-stop + 메카넘 확장 |
| 6 | 하차: 슬롯 마커 정렬 → z 진입 → 팔 접기 → 이탈 → DONE | 2~4 역순 |

### 코드 정독(07-21)으로 확정된 사실

- 팀원 스택에 formation_gap_controller(간격 2.9 m 추종 + co-stop)와 2로봇
  배정 dispatcher가 **실구현·단위테스트 완료** 상태로 존재. `parking_robot_system`
  6개 노드는 전부 TODO 스켈레톤 — **리더 주행기(navigate)가 우리가 채울 자리.**
- Isaac 쪽 2로봇 스크립트(`mecanum_ros2_drive_dual.py`, `run_dual_robot_ros2_field.py`,
  `build_dual_robot_parking_field.py`)를 팀원이 미검증 상태로 만들어 둠 —
  신규 브링업 작성 대신 **이를 이 머신에서 검증·수정해 재사용**한다.
- ⚠️ **odom y부호 모순 발견**: dual 드라이버는 `ros_y=+usd_z`, 필드 러너·공식
  규약은 `ros_y=−usd_z`. 실측으로 확정 후 한쪽을 고쳐야 한다 (0단계 최우선).
- gap-hold는 diff-drive 모델(linear.x+angular.z) — 메카넘 확장 3종(wz=0 결합 강제,
  linear.y 횡오차항, facing 부호)이 코드 수준에서도 필요함이 확인됨.
- 팀원 지도는 실내만 커버(entrance까지) — 인계장 노드/존/마커 열은 우리
  marker_layout(베이 H_A/H_B, 차선 HE/HW 열, 횡단 34·35)에서 파생해 확장한다.

## 선결/합의 항목 (팀원에게)

1. `parking_robot_interfaces`·`parking_robot_system` 패키지 push (아직 미push —
   .srv/.action 원본과 orchestrator 스켈레톤이 이 안에 있음).
2. 이 머신(rokey)에 MySQL + 스키마 001~004 셋업 (004 적용 누락 사례 주의 —
   11단계 문서의 교훈).
3. formation_gap_controller 메카넘 확장 3종 반영 방식 (우리가 PR? 팀원이 수정?).
4. 통로 차선 2줄 존 분할 채택 여부 (generate_map.py 확장).
5. AcquireZones 호출 위치 "구간마다" 확정 (팀원 권장안에 동의 회신).
6. 슬롯 존 task_id 선점 시점(배정 직후 vs 도착 직전) — 8단계 설계 확인.
