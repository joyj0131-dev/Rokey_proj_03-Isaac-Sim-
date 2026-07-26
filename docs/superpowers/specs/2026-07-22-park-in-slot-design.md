# 서비스 기반 지정 구역 주차 (P1: end-to-end 수직 관통) — 설계

> 작성: 2026-07-22. 관련: `dock_lift_handoff_mission.py`/`dock_lift_handoff_runner.py`(동작하는
> Isaac 운반 구현), `src/parking_robot_system`(ROS2 관제 스텁 9노드), `src/parking_control`
> (관제 DB 스키마·`parking_map.yaml`). 이 문서는 5단계 로드맵의 **P1**만 다룬다.

## 배경

현재 `dock_lift_handoff_mission.py`는 로봇 2대가 차량(Pickup)을 팔 4개로 파지·리프트한 뒤
**전/후/옆 1m씩 데모 운반(`_omni_carry`)만** 하고 끝난다. 진입은 `/dock_lift`(std_srvs/Trigger,
입력 없음) 서비스 한 번으로 전체 시퀀스를 동기 실행한다.

한편 `src/parking_robot_system`에는 정식 관제 아키텍처(gateway → dispatcher → slot_manager →
orchestrator → navigate/align/lift 액션서버 + safety_monitor + vehicle_detection, 총 9노드)가
**전부 빈 스텁(TODO)**으로 잡혀 있고, `src/parking_control`에는 MySQL 관제 DB 스키마와
좌표 맵(`parking_map.yaml`)이 있다. 실제 로봇을 움직이는 로직은 Isaac 쪽에만 있고, ROS
관제 파이프라인과 연결돼 있지 않다.

## 목표 (사용자 요구)

서비스로 **주차 구역(slot_id)을 입력**하면:
- 해당 구역이 **비어 있으면** 그 구역에 **방향을 맞춰**(같은 열의 기존 주차 차량과 동일 방향) 주차한다.
- 해당 구역에 **차량이 있으면** "해당 구역에 차량이 있어 주차 불가"라고 알려주고 동작하지 않는다.

이를 **전체 ROS 파이프라인을 관통**하는 방식으로 구현한다(단, 아래 "범위"의 P1 한정).

## 로드맵과 범위

| 단계 | 내용 |
|---|---|
| **P1 (이 문서)** | 지정 구역 주차 end-to-end 수직 관통. 단일 요청·단일 팀(로봇 2대). |
| P2 | MySQL 관제 DB 연동(점유 Isaac↔DB 동기화, 작업 원장). |
| P3 | 다중 로봇 + 존 락(AcquireZones/ReleaseZones, zone_locks). |
| P4 | safety_monitor 긴급정지 + 편대 제어 정식화(FormationAssignment/Stop). |
| P5 | 출차(EXIT) + 카메라 차량인식(vehicle_detection, feat/camera 연계). |

### P1 범위
- 커스텀 srv 신규 추가로 slot_id 입력.
- gateway → dispatcher → slot_manager → orchestrator → navigate/align/lift 액션서버를 **실제 연결**.
- 점유 진실원본 = **Isaac 스테이지**(runner가 `/parking_slots`로 발행). MySQL 미사용.
- 동작하는 `dock_lift_handoff` 모션 프리미티브를 공유 모듈로 추출해 액션서버가 구동.
- 주차 완료 후 로봇 2대는 **West 대기 도크로 복귀**.

### P1 범위 밖 (후속 단계)
- MySQL DB, 다중 로봇 동시성/존 락, 안전 긴급정지, 편대 메시지 정식화, 출차, 카메라 인식.
- detect_vehicle는 P1에서 **알려진 Pickup 좌표를 반환하는 얇은 스텁**(카메라 인식은 P5).

## 좌표계와 변환

- **Isaac/USD**: XZ 지면, +Y 상방. 차량 길이축 = z. 주차면 중심
  `x = -17 + (index+0.5)·3.4`, `z = +7.8`(A열) / `-7.8`(B열).
- **주차 방향(핵심)**: 기존 주차 차량은 A열 yaw=**180°**, B열 yaw=**0°**
  (`build_parking_environment.py:716`). 목표 구역과 같은 열의 방향으로 안착시킨다.
- **ROS map 프레임**: `x_map = x_usd`, `y_map = -z_usd` (`parking_map.yaml`·DB 시드 규약).
  A2 = USD(-8.5, +7.8) = map(-8.5, -7.8).
- 모든 ROS 메시지(geometry_msgs/Pose 등)는 **map 프레임**으로 표현하고, USD↔map 변환은
  **Isaac 브리지 한 곳**(공유 모듈 `frame_transform`)에서만 수행한다. 변환 함수와 역함수에
  단위 테스트를 둔다.

## 아키텍처

### 노드 토폴로지 & 메시지 흐름
```
ros2 service call /park_in_slot  (slot_id="A2")
        │
        ▼
[user_request_gateway]  ParkInSlot.srv (사용자 대면, 얇은 프록시)
        │  1) dispatcher 내부 서비스 /dispatch/park_in_slot 호출
        │     (ParkInSlot.srv 재사용 — 원 스텁에 없던 seam을 이 서비스로 확정)
        ▼
[task_dispatcher]
        │  2) GetSlotInfo(slot_id) ─────────▶ [parking_slot_manager]
        │       exists / occupied / pose(map) ◀──  (/parking_slots 구독 캐시)
        │  3) 검증:
        │      · 없음        → accepted=false, "존재하지 않는 구역"
        │      · 점유         → accepted=false, "해당 구역에 차량이 있어 주차 불가"
        │      · 비어있음     → ExecuteParkingTask.action goal(slot_id, slot_pose, 로봇편성) 전송
        ▼
[robot_task_orchestrator]  상태머신, task_state 발행
        │  navigate_to_pose / align_vehicle / control_lift 액션 순차 호출
        ▼
[navigate/align/lift 액션서버]  ── 공유 모듈 formation_driver ──▶ Isaac
        │  /robot_{rear,front}/cmd_vel (발행), /robot_*/odom·/vehicle/pose (구독),
        │  /robot_*/arm_control (SetBool)
        ▼
[dock_lift_handoff_runner (Isaac)]  물리 구동 + /parking_slots 점유 발행
```

`/park_in_slot` 응답은 **즉시 검증 결과**만 돌려준다(없음/점유 → 거부, 비어있음 → accepted+task_id).
실제 주차 진행·완료는 `task_state` 토픽(SEARCHING…PARKED…DONE) 및 `GetTaskStatus`로 관찰한다.

### 점유(occupancy) 모델
- runner가 **모든 차량 prim의 실제 위치 vs 각 주차면 중심**을 매 발행 틱 비교해 점유를 산출한다
  (`parking:occupied` 초기값과 일치하며, 로봇이 A2에 차를 놓으면 **다음 발행에 자동 반영**).
- 발행 토픽 `/parking_slots` = `std_msgs/String`(JSON 배열):
  `[{"slot_id":"A2","occupied":false,"is_accessible":true,"x":-8.5,"y":-7.8,"yaw_deg":180.0}, …]`.
  (map 프레임 좌표 + 목표 yaw. 내부 rclpy 제약 회피 위해 **커스텀 인터페이스 대신 std_msgs/String**.)
- `parking_slot_manager`가 이 토픽을 구독·캐시하고 `GetSlotInfo(slot_id)` 서비스로 응답한다.

### 액션서버 = 기존 Isaac 모션 브리지
동작하는 `dock_lift_handoff_mission`의 프리미티브(`_omni_step`, `_goto_xz`,
`_approach_parallel`, `_rotate_to`, `_ingress_to`, `_call_arms`, `_grip_lift`)를 **공유 모듈**
(`formation_driver.py`)로 추출한다. 액션서버는 그 위의 얇은 래퍼:
- **navigate_to_pose** (nav2 `NavigateToPose` 재사용): 편대 world 이동. 픽업 전=로봇 개별 접근
  (`_approach_parallel`), 픽업 후=차량을 실은 편대 운반(both robots 동일 body twist).
- **align_vehicle** (`AlignVehicle.action`): `_ingress_to`로 차량 축(axle) 정밀 정렬.
- **control_lift** (`ControlLift.action`): `_call_arms(True)`=UP(리프트), `_call_arms(False)`=DOWN(안착).
- **detect_vehicle** (`DetectVehicle.action`): P1은 알려진 Pickup 좌표를 반환하는 스텁.

> 리스크 완화: 기존 `dock_lift_handoff_mission.py`의 검증된 제어 로직을 **로직 변경 없이 이동**시키고,
> 기존 파일은 회귀 비교용으로 보존한다. runner는 P1에서 점유 발행만 추가(모션 로직 불변).

### 오케스트레이터 상태머신
`TaskState.state` 전이(모두 `task_state` 토픽 발행):
```
SEARCHING   detect_vehicle → Pickup 좌표 확보
APPROACHING navigate(개별) → 차량 하부 진입 위치, align_vehicle → 축 정렬
PICKED_UP   control_lift(UP) → 리프트 확인(Δy ≥ 임계)
MOVING      navigate(편대) → 목표 슬롯 근처로 운반
ARRIVED     슬롯 진입 정렬 + 편대 회전으로 목표 yaw(A=180/B=0) 수렴
PARKED      control_lift(DOWN) → 차량 안착, 파지 해제
RETURNING   navigate(개별) → West 대기 도크 복귀
DONE        결과 success=true 반환
FAILED      임의 단계 실패 시 message와 함께 종료(로봇 안전 정지)
```

## 인터페이스 (parking_robot_interfaces)

신규 커스텀:
```
# ParkInSlot.srv  (외부 진입: 사용자 → user_request_gateway)
string slot_id
---
bool accepted
string task_id
string message

# GetSlotInfo.srv  (task_dispatcher → parking_slot_manager)
string slot_id
---
bool data_ready              # false = /parking_slots 캐시 비어있음(runner 미기동)
bool exists
bool occupied
bool is_accessible
geometry_msgs/Pose pose      # map 프레임, orientation.z/w = 목표 yaw
```
재사용: `ExecuteParkingTask.action`(slot_id·slot_pose·leader/follower 이미 정의됨),
`TaskState.msg`, nav2 `NavigateToPose`, `AlignVehicle.action`, `ControlLift.action`,
`DetectVehicle.action`, std_srvs/SetBool(arm_control).

## 요구 동작 매트릭스

| 입력 slot_id | 조건 | `/park_in_slot` 응답 | 로봇 동작 |
|---|---|---|---|
| 유효·비어있음 | is_accessible 무관 | accepted=true, task_id | 픽업→운반→방향 맞춰 주차→도크 복귀 |
| 유효·점유됨 | `parking:occupied` | accepted=false, "해당 구역에 차량이 있어 주차 불가" | 없음 |
| 존재하지 않음 | 라벨 미매칭 | accepted=false, "존재하지 않는 구역" | 없음 |

## 오류 처리
- runner 미기동/`/parking_slots` 미수신 → slot_manager는 캐시가 비어 `GetSlotInfo`를
  `data_ready=false`로 표시하고, dispatcher는 "관제 데이터 없음(재시도)" 메시지로 거부한다
  (로봇 구동 안 함). ※ `GetSlotInfo.srv`에 `bool data_ready` 필드를 포함한다.
- 액션 단계 실패(리프트 Δy 미달, 정렬 타임아웃 등) → 상태 FAILED, `_stop_all`, task_state에 사유.
- 서비스 콜 터미널 ROS 환경(도메인 126, FastRTPS 프로파일 unset)은 런북에 명시.

## 테스트 전략 (TDD)
Isaac 없이 단위 테스트 가능한 순수 로직부터:
- `frame_transform`: USD↔map 왕복, A/B열 yaw 매핑.
- 점유 판정: 차량 위치 → 슬롯 점유 산출(경계·공차).
- slot_manager `GetSlotInfo`: 캐시 → exists/occupied/pose 응답.
- dispatcher 검증 분기: 없음/점유/비어있음 3케이스가 올바른 응답·액션 호출로 이어짐.
- 오케스트레이터 상태 전이(액션 클라이언트 모킹).
모션(편대 운반·회전·안착)은 `dock_lift_handoff_runner.sh --headless-test` 스모크로 검증.

## 열린 질문 / 가정
- **편대 회전 제어 법칙**: 목표 yaw 수렴을 (a) 슬롯 진입 전 편대 전체 인플레이스 회전 vs
  (b) 접근 경로 설계로 자연 정렬 중 어느 쪽으로 할지는 구현 계획(플랜)에서 TDD로 결정.
- A2 등 accessible 구역에 일반 차량 주차 허용 여부: P1은 **허용**(점유만 판정, 접근성 제한 없음).
- 완료 통지: P1은 task_state 관찰로 충분. 동기 블로킹 응답이 필요하면 후속에서 옵션 추가.
