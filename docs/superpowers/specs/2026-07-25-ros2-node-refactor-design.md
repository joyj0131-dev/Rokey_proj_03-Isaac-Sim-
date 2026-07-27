# ROS2 노드 구조 이행 설계 (미션 코드 탈-러너)

> 작성: 2026-07-25. 계기: 사용자 지적 — 미션 기능이 ROS2 노드가 아니라 단일 파일
> `parking_v4_runner.py`(3,782줄)에 누적되고 있었다. Phase D 착수 전에 구조를 바로잡는다.

## 1. 문제

| 사실 | 수치 |
|---|---|
| 러너 단일 파일 | **3,782줄** |
| 분리된 순수 모듈 | 523줄 |
| 미션이 쓰는 ROS2 노드 | **0개** (패키지는 4개 존재) |
| 제어 계층 위치 | `drive_to_pose`(1157), `rotate_in_place`(1344) — **`main()` 내부 중첩 함수** |

실제 로봇으로 이식돼야 할 코드(제어·안무·감지)가 시뮬레이터 함수 안에 갇혀 있어 **재사용·단위테스트·하드웨어 이식이 불가능**하다.

**원인(정직하게)**: Isaac은 Python 3.11, ROS2 Humble은 3.10이라 한 프로세스에 못 올린다 →
인프로세스가 당장 빨랐다. 그리고 이 저장소의 Phase A~C 계획서가 전부 "러너에 추가"로 쓰여 있었고
그대로 실행됐다(계획을 쓴 것도 나다).

## 2. 이미 존재하는 자산 (재발명 금지)

`src/parking_robot_interfaces` 에 인터페이스가 **이미 정의**돼 있고, 미션 로직과 1:1 대응한다.

| 인터페이스 / 노드 | 대응하는 현재 러너 코드 |
|---|---|
| `nav2_msgs/NavigateToPose` ← `navigate_action_server` | `drive_to_pose` / `rotate_in_place` |
| `AlignVehicle.action` ← `align_action_server` | 마커 기반 정밀 정렬(XN·베이마커) |
| `ControlLift.action` (UP/DOWN) | `deploy_arms` 리프트 |
| `ExecuteParkingTask.action` (**leader/follower robot_id**) ← `robot_task_orchestrator` | 미션 B/C 2로봇 안무 |
| `marker_localizer_node` (parkbot_aruco) | 인프로세스 `PoseFilter` 융합 |

따라서 이 작업은 **새 구조 설계가 아니라, 이미 있는 액션들의 백엔드를 미션 코드로 채우고
러너를 얇은 시뮬 브리지로 되돌리는 것**이다.

## 3. 목표 구조

```
┌─ Isaac 프로세스 (py3.11) ──────────────┐      ┌─ ROS2 프로세스 (py3.10) ─────────────┐
│ parking_v4_runner.py = 시뮬 브리지     │      │ marker_localizer_node  (측위 융합)   │
│  · 씬/로봇/차량/마커 스폰              │ pub  │ axle_detector_node     (뎁스 축감지) │
│  · 카메라 RGB/Depth 발행(OmniGraph)    │─────▶│ navigate_action_server (주행 제어)   │
│  · /odom, /joint_states 발행           │      │ align_action_server    (정밀 정렬)   │
│  · /cmd_vel, 리프트 명령 구독          │◀─────│ lift_action_server     (리프트)      │
│  · 프로브 11종(시뮬 전용 진단) 유지    │ sub  │ robot_task_orchestrator(2로봇 안무)  │
└────────────────────────────────────────┘      └──────────────────────────────────────┘
                     공유 순수 로직(ROS2 패키지): body_twist_toward · 메카넘 기구학 ·
                     휠오도 · TroughTracker · DepthStopDetector · marker_localizer
```

**원칙**
- 시뮬레이터에만 있어야 하는 것: 씬 저작, 물리, 렌더, GT 채점, 진단 프로브.
- 실로봇에도 가야 하는 것: 측위·제어·감지·안무 → 전부 ROS2 패키지.
- 순수 알고리즘은 패키지에 두고 **양쪽이 import** 한다(러너는 이미 `sys.path.insert` 로 소스 직접 import 중이라 빌드 없이도 동작).

## 4. 이행 단계 (각 단계마다 검증 가능, 기존 미션을 깨지 않음)

| 단계 | 내용 | 검증 |
|---|---|---|
| **R1** | 순수 로직(`mission_control`, `axle_center`, `depth_stop_detector`, 메카넘 순수 기구학, 휠오도)을 ROS2 패키지로 이전. 러너는 거기서 import | pytest 12개 + `--mission=C` 결과 불변 |
| **R2** | 시뮬 브리지: `/robot_<id>/odom`·`/joint_states` 발행 + `/cmd_vel`·리프트 명령 구독 | 외부 ROS2 노드가 cmd_vel 로 로봇을 실제로 움직임 |
| **R3** | 제어 노드: `drive_to_pose`/`rotate_in_place` 를 `main()` 밖 클래스로 추출 → `navigate_action_server`(NavigateToPose) 백엔드로. 자세는 `marker_localizer_node` 구독 | 액션 호출로 목표 자세 도달, 정확도 인프로세스 수준 |
| **R4** | `axle_detector_node`(측면 뎁스 → 축 중심) + `lift_action_server`(ControlLift) | 노드가 축 검출, 액션으로 트럭 들림 |
| **R5** | `robot_task_orchestrator` 가 ExecuteParkingTask(leader/follower)로 Phase B+C 안무 수행 | ROS2 경로 전체 미션 = `--mission=C` 결과와 동등 |
| **R6** | 러너에서 미션 코드 제거(프로브·브리지만 잔존), 문서 갱신 | 러너 줄 수 대폭 감소, 회귀 없음 |

## 5. 위험과 대응

- **타이밍**: 인프로세스 동기 루프(60Hz)에서 비동기 DDS로 바뀌면, 공들여 튜닝한 값들
  (`YAW_ODOM_SCALE=1.1677`, settle 중앙값, 뎁스 중앙유지 게인)이 **재튜닝이 필요할 수 있다.**
  → 단계마다 GT 대비 정확도를 기존 수치와 비교하고, 나빠지면 그 단계에서 재튜닝·기록.
- **양쪽 파이썬 버전**: 브리지는 Isaac 내장 rclpy(레거시 러너가 쓰던 경로)를 쓴다 — 이미 검증된 방식.
- **되돌릴 수 있게**: 각 단계는 별도 커밋. 미션이 깨지면 이전 단계로 되돌아간다.
- **이 머신의 `ros2` CLI 세그폴트**(Task B0 기록): CLI 도구 문제이고 노드 간 DDS 통신은
  Phase A 2-터미널 검증에서 정상이었다. 검증은 CLI 대신 노드/로그로 한다.

## 6. 수용 기준

- 미션 제어·안무·감지가 **ROS2 노드**로 동작하고, `--mission=C` 와 **동등한 결과**(축 오차 ≤5cm, 리프트 성공)를 ROS2 경로로 재현한다.
- 러너는 시뮬 브리지 + 진단 프로브만 남는다.
- 기존 테스트·프로브 회귀 없음. GUI 실행 유지.

## 7. 범위 밖

- Phase D(운반·주차) 기능 추가 — 리팩터 완료 후 ROS2 구조 위에서 진행.
- 실하드웨어 배포, nav2 스택 전면 도입.
