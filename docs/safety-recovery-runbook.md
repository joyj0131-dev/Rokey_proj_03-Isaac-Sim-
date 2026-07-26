# 관제 비상정지·복구 운영 가이드

이 문서는 관제 담당자와 후속 개발자가 비상정지 복구 구조를 검토하고 실제
장비 정책을 반영할 때 사용하는 기준 문서다.

> 현재 기능은 ROS2 소프트웨어 안전 정지다. 실제 장비에 적용할 때는 안전
> PLC, 구동 전원 차단, 물리 비상정지 입력 등 안전 등급 하드웨어와 반드시
> 연동해야 한다. 웹 UI가 물리 비상정지 회로를 대신하지 않는다.

## 설계 원칙

- 안전 상태의 단일 권위는 `safety_supervisor`다.
- 비상정지 상태는 MySQL `safety_state`에 저장되어 프로세스 재기동 후에도 유지된다.
- `/safety/state`는 transient-local QoS로 발행되어 늦게 실행된 노드도 최신 상태를 받는다.
- 3초 동안 supervisor heartbeat가 없으면 웹·디스패처·모션 계층이 fail-safe 정지한다.
- 비상정지 해제와 운전 재개를 분리한다.
- 점검 승인 후에도 `motion_allowed=false`다.
- 기존 작업은 자동 재개하지 않는다. 중단된 작업은 `FAILED/CANCELLED`로 남긴다.
- `NORMAL` 전환은 이동 명령이 아니다. 새 작업 접수 권한만 다시 연다.

## 상태 전이

```text
NORMAL
  │ 비상정지
  ▼
STOPPED_LATCHED
  │ 현장 점검 + 관제 해제 요청 승인
  ▼
READY_FOR_OPERATION   (계속 motion_allowed=false)
  │ 별도 운영 복귀 승인
  ▼
NORMAL                (기존 작업 재개 없음)
```

`UNKNOWN`은 웹 또는 디스패처가 `safety_supervisor`의 상태를 아직 받지 못한
fail-safe 상태다. 이 상태에서는 새 작업을 접수하지 않는다.

## 관제 담당자 절차

### 1. 비상정지 직후

1. 로봇과 차량이 실제로 정지했는지 확인한다.
2. 작업 구역 접근을 통제한다.
3. 차량이 리프트에 걸려 있거나 불안정하게 지지되는지 확인한다.
4. 중단된 작업 ID와 정지 원인을 기록한다.

### 2. `점검 완료 · 해제 요청`

UI의 `안전 복구 절차`에서 다음 항목을 모두 확인한다.

- 작업 구역에 사람이 없고 안전함
- 전체 로봇 정지 상태
- 차량·리프트 지지 상태
- 센서·통신 상태 또는 계획된 센서 비활성 상태

담당자 ID와 점검 결과를 입력한다. 이 단계가 승인되어도 로봇은 움직이지 않는다.

### 3. `운영 복귀 승인`

다른 관제 담당자 또는 승인 권한자가 다음을 확인한다.

- 미종결 작업이 없음
- 로봇 상태가 `IDLE` 또는 `CHARGING`
- 현재 로봇·차량 위치를 기준으로 새 작업을 계획할 수 있음

승인 후 시스템은 `NORMAL`이 된다. 중단 작업의 재개 버튼은 제공하지 않는다.
필요하면 현재 위치를 기준으로 새로운 복구 작업 또는 입·출차 요청을 생성한다.

## ROS2 계약

| 종류 | 이름 | 역할 |
|---|---|---|
| Topic | `/safety/state` (`SafetyState`) | 영속 안전 상태 방송 |
| Service | `/safety/activate_emergency_stop` | 비상정지 요청 |
| Service | `/safety/request_reset` | 현장 점검 결과 제출 |
| Service | `/safety/approve_operation` | 별도 운영 복귀 승인 |
| Topic | `/formation_stop` (`FormationStop`) | 기존 모션 노드 호환용 즉시 정지 방송 |

`task_dispatcher`, `sim_orchestrator`, `FormationMotion`,
`robot_task_orchestrator`, `lift_action_server`,
`formation_gap_controller`가 중앙 안전 상태를 구독한다.

## 웹 API

```text
POST /api/emergency-stop
POST /api/safety/reset-request
POST /api/safety/approve-operation
GET  /api/system
GET  /api/dashboard
```

`GET /api/system`과 대시보드의 `system.safety`에서 현재 상태, 정지 세대
(`stop_epoch`), 담당자, 점검 기록, 차단 사유를 확인할 수 있다.

## DB와 감사 이력

마이그레이션:

```bash
mysql -u parking -p parking \
  < src/parking_control/db/005_safety_supervisor.sql
```

현재 상태:

```sql
SELECT * FROM safety_state WHERE singleton_id = 1;
```

승인 이력:

```sql
SELECT event_id, stop_epoch, event_type, state, operator_id, note, created_at
FROM safety_events
ORDER BY event_id DESC;
```

`run_test_stack.sh`도 안전 상태를 초기화하지 않는다. 테스트 스택을 종료했다가
다시 실행해도 `STOPPED_LATCHED` 또는 `READY_FOR_OPERATION` 상태가 유지되며,
관제 UI의 정상 복구 절차를 완료해야 `NORMAL`로 돌아간다. 작업·차량·주차면
테스트 데이터 초기화와 안전 상태 초기화는 서로 분리한다.

## 후속 실제 장비 연동 체크리스트

- [ ] 물리 비상정지 입력과 safety PLC 상태를 `safety_supervisor`에 연결
- [ ] 로봇별 정지 완료 ACK와 구동 전원 상태 수집
- [ ] 차량 지지/리프트 위치 센서의 자동 확인
- [ ] 작업 구역 사람 감지 또는 출입문 인터록 확인
- [ ] 관제 사용자 인증과 `operator_id` 자동 주입
- [ ] 2인 승인 정책이 필요하면 reset 담당자와 approve 담당자 동일 ID 거부
- [ ] 중단 지점별 전용 복구 Action 설계
- [ ] 안전 요구 수준 및 위험성 평가 후 하드웨어 정지 성능 검증

현재 체크 항목은 관제 담당자의 명시적 확인과 DB 상태 검증을 결합한 구조다.
실제 장비 센서가 연결되면 서비스 요청의 boolean 값을 신뢰하기보다
`safety_supervisor`가 각 안전 입력과 로봇 ACK를 직접 확인하도록 교체해야 한다.
