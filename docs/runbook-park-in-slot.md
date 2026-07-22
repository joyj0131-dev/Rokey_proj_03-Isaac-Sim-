# 런북: 지정 구역 주차(P1) End-to-End 실행·검증

`ros2 service call /park_in_slot ... slot_id: 'A2'` 한 번으로 사용자 요청이
`user_request_gateway → task_dispatcher → parking_slot_manager → robot_task_orchestrator →
navigate/align/lift 액션서버 → Isaac(dock_lift_handoff_runner)` 전체를 관통해 로봇이 실제로
차량을 주차면까지 옮기는지 사람이 Isaac Sim GUI로 직접 확인하기 위한 절차다. 설계 배경은
`docs/superpowers/specs/2026-07-22-park-in-slot-design.md` 참고.

## ★ 반드시 지킬 것 3가지

이 셋 중 하나라도 어기면 "동작해야 하는데 안 됨"처럼 보이는 상황이 발생한다(실제로는 환경
설정 문제). 아래 실행 순서에도 각 지점마다 다시 표시해 둔다.

1. **Isaac runner는 `--gui`로 띄운다. `--headless-test`는 절대 쓰지 않는다.**
   `--headless-test`는 물리 안정성만 확인하고 `rclpy.init()`/`/parking_slots` 발행 루프에
   도달하기 **전에 조기 종료**한다(`dock_lift_handoff_runner.py`의 `if "--headless-test" ...:
   ... app.close(); return` 분기가 `import rclpy`보다 앞에 있음). 즉 `--headless-test`로는
   ROS 그래프에 아무 토픽도 뜨지 않는다 — 이 런북의 검증에는 무용하다.
2. **모든 ROS 터미널(B/C/D/E)이 동일 환경이어야 한다**: `ROS_DOMAIN_ID=126`,
   `RMW_IMPLEMENTATION=rmw_fastrtps_cpp`, `FASTRTPS_DEFAULT_PROFILES_FILE`/
   `FASTDDS_DEFAULT_PROFILES_FILE` **unset**. Isaac runner(`.sh`)는 내부적으로 이 값을
   스스로 설정하므로, 시스템 ROS 2로 뜨는 노드 쪽이 여기에 맞춰야 서로 디스커버리된다.
   **`ros2 launch`를 실행하는 터미널(B)도 예외 없이 이 환경이 필요하다** — launch 파일
   자체는 이 env를 설정하지 않는다(아래 "launch 파일 주석" 참고). 잊기 쉬운 포인트.
3. **`carry_to`/`carry_rotate_to`(인계 베이 → 슬롯, ~22m)와 RETURNING 도크 좌표는
   best-effort다.** 원본 미션에 전례가 없던 신규 코드로, 실제 경로 튜닝·충돌 회피가 아직
   검증되지 않았다. 이 구간은 **GUI로 직접 관찰**하면서 막히거나(타임아웃) 이상하게
   움직이면(벽 스침, 개구부 오정렬 등) 튜닝이 필요하다고 기록해야 한다 — "실패"가 곧
   "런북이 틀렸다"는 뜻은 아니다.

## 사전 준비 (최초 1회 또는 코드 변경 후)

```bash
cd /home/rokey/p3/Rokey_proj_03-Isaac-Sim-
source /opt/ros/humble/setup.bash
colcon build --packages-select parking_robot_interfaces parking_robot_system
```
`Summary: 2 packages finished`면 OK(기존 setuptools dash-separated-option 경고는 무해,
무시). 이후 모든 시스템-ROS 터미널에서 `source install/setup.bash`가 필요하다.

## ROS 환경 한 줄 요약 (터미널 B/C/D/E 전부에서 실행)

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=126
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
unset FASTRTPS_DEFAULT_PROFILES_FILE FASTDDS_DEFAULT_PROFILES_FILE
```
(작업 디렉터리는 리포 루트 `/home/rokey/p3/Rokey_proj_03-Isaac-Sim-` 기준.)

---

## 실행 순서 (터미널 A → B → C → D)

### 터미널 A — Isaac runner (GUI)

**이 터미널에서는 `/opt/ros/humble/setup.bash`를 source하지 않는다.** 스크립트가
`ROS_DOMAIN_ID`(기본 126)/`RMW_IMPLEMENTATION`/`LD_LIBRARY_PATH`를 자체적으로 설정하고
`FASTRTPS_DEFAULT_PROFILES_FILE`/`FASTDDS_DEFAULT_PROFILES_FILE`를 unset하며, 시스템
Humble(3.10)이 섞이면 내부 rclpy(3.11, Isaac 브리지)와 ABI가 충돌한다. 새 터미널을 쓴다.

```bash
cd /home/rokey/p3/Rokey_proj_03-Isaac-Sim-/isaacpjt/Isaac_envo
bash dock_lift_handoff_runner.sh --gui
```

Isaac Sim GUI 창이 뜨고 씬(주차장 + 인계 베이 Pickup + 로봇 2대)이 로드된다(부팅에 수십 초
~ 수 분 소요될 수 있음, 하드웨어 의존). 콘솔에 아래 줄이 뜨면 **/parking_slots 발행 시작**
신호이므로 이후 터미널로 진행:
```
DOCK_LIFT_HANDOFF_READY robots=['robot_rear','robot_front'] domain=126
```

### 터미널 B — ROS 2 파이프라인 launch

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=126
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
unset FASTRTPS_DEFAULT_PROFILES_FILE FASTDDS_DEFAULT_PROFILES_FILE
cd /home/rokey/p3/Rokey_proj_03-Isaac-Sim-
ros2 launch parking_robot_system parking_robot_system.launch.py
```
노드 9개(`user_request_gateway, task_dispatcher, parking_slot_manager,
robot_task_orchestrator, safety_monitor, vehicle_detection_node, navigate_action_server,
align_action_server, lift_action_server`)가 `output='screen'`으로 각자 `... node started`
로그를 찍는다. 에러 없이 유지되면 OK(이 터미널은 계속 켜 둔다).

**(선택, 권장) ROS 그래프 사전 점검** — 다른 터미널에서 같은 환경(위 "ROS 환경 한 줄
요약") 소싱 후:
```bash
ros2 node list      # 9개 노드 모두 보여야 함
ros2 topic list      # /parking_slots, /task_state, /robot_rear/odom, /robot_front/odom,
                      # /vehicle/pose, /robot_rear/cmd_vel, /robot_front/cmd_vel, /obstacle_alert 등
ros2 service list    # /park_in_slot, /dispatch/park_in_slot, /get_slot_info,
                      # /robot_rear/arm_control, /robot_front/arm_control 등
ros2 action list     # /execute_parking_task, /detect_vehicle, /navigate_to_pose,
                      # /align_vehicle, /control_lift
```
여기서 무언가 비어 있으면(특히 `/parking_slots`가 `ros2 topic list`에 없음) 터미널 A가
아직 READY 로그를 안 찍었거나, 터미널 B의 환경(도메인/RMW/프로파일)이 A와 어긋난 것이다 —
아래 "문제 해결" 참고.

### 터미널 C — 초기 점유 확인

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=126
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
unset FASTRTPS_DEFAULT_PROFILES_FILE FASTDDS_DEFAULT_PROFILES_FILE
ros2 topic echo --once /parking_slots
```
한 번 메시지를 받으면 자동 종료한다(`std_msgs/String`의 `data`에 JSON 배열 문자열). 기대값:
**`A3`/`A5`/`A6`/`B3`는 `"occupied": true`, `A1`/`A2`는 `"occupied": false`**(씬에 미리
배치된 차량 기준 — Task 5에서 확정된 값). 예:
```yaml
data: '[{"slot_id": "A1", "occupied": false, "is_accessible": true, "x": -14.2, "y": -7.8,
  "yaw_deg": 180.0}, ..., {"slot_id": "A3", "occupied": true, "is_accessible": false,
  "x": -5.1, "y": -7.8, "yaw_deg": 180.0}, ...]'
---
```
A2가 `false`(아래 검증 케이스 3에서 사용)이고 A3가 `true`(케이스 1에서 사용)인지만 최소
확인하면 된다. 이후 이 터미널은 재사용 가능(같은 명령을 반복 실행해 점유 변화를 재확인).

---

## 검증 케이스 (터미널 D, 필요시 E)

셋 다 `ros2 service call /park_in_slot parking_robot_interfaces/srv/ParkInSlot "{slot_id:
'<값>'}"` 형태다. 매번 위 "ROS 환경 한 줄 요약"을 먼저 실행한다(새 터미널이면 필수, 같은
터미널 재사용이면 이미 되어 있으면 생략 가능).

### 케이스 1 — 점유 구역 거부 (A3)

```bash
ros2 service call /park_in_slot parking_robot_interfaces/srv/ParkInSlot "{slot_id: 'A3'}"
```
**기대 출력:**
```
response:
parking_robot_interfaces.srv.ParkInSlot_Response(accepted=False, task_id='', message='해당 구역에 차량이 있어 주차 불가')
```
**사람이 GUI로 확인:** Isaac 창에서 로봇 2대가 **전혀 움직이지 않아야 한다**(거부는
dispatcher 단계에서 끝나고 orchestrator에 goal이 전달되지 않음).

### 케이스 2 — 존재하지 않는 구역 거부 (C1)

```bash
ros2 service call /park_in_slot parking_robot_interfaces/srv/ParkInSlot "{slot_id: 'C1'}"
```
**기대 출력:**
```
response:
parking_robot_interfaces.srv.ParkInSlot_Response(accepted=False, task_id='', message='존재하지 않는 구역')
```
(`C1`은 A/B열 8칸 체계에 없는 라벨 — `parking_slot_manager` 캐시에 키가 없어 `exists=False`.)
**사람이 GUI로 확인:** 케이스 1과 동일하게 로봇 무동작.

### 케이스 3 — 빈 구역 주차 (A2, 전체 파이프라인 관통)

먼저 **터미널 E**를 열어 진행 상황 관찰을 시작해 둔다(최초 `SEARCHING` 메시지를 놓치지
않기 위해 서비스콜보다 먼저 실행 권장):
```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=126
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
unset FASTRTPS_DEFAULT_PROFILES_FILE FASTDDS_DEFAULT_PROFILES_FILE
ros2 topic echo /task_state
```

그 다음 터미널 D에서:
```bash
ros2 service call /park_in_slot parking_robot_interfaces/srv/ParkInSlot "{slot_id: 'A2'}"
```
**기대 출력 (즉시):**
```
response:
parking_robot_interfaces.srv.ParkInSlot_Response(accepted=True, task_id='<uuid4 문자열>', message='접수됨')
```
`ros2 service call`은 여기서 바로 반환한다(`/park_in_slot`은 접수 여부만 즉답 — 실제 주차
진행은 `task_state`로 관찰). task_id는 매번 랜덤 UUID.

**터미널 E(`task_state`)에서 기대되는 진행(`state` 필드, 실패 없으면 이 순서 그대로):**

| state | current_step (사람이 읽는 문장) | 비고 |
|---|---|---|
| SEARCHING | 차량 탐색 중 | 즉시(스텁, 카메라 인식 아님) |
| APPROACHING | 차량 하부 진입 중(픽업 정렬) | `pickup_sequence` — 검증된 원본 이식, 신뢰도 높음 |
| PICKED_UP | 리프트 상승 완료(픽업) | 팔 전개 + 리프트, 수 초~수십 초 |
| MOVING | 슬롯으로 이동 중 | ★`carry_to` best-effort(위 핵심주의 3) — 여기가 가장 튜닝 가능성 높은 구간 |
| ARRIVED | 슬롯 방향 정렬 중 | ★`carry_rotate_to` best-effort — 편대가 A열 방향(180°)으로 제자리 회전 |
| PARKED | 안착 완료(리프트 하강) | 리프트 하강 + 파지 해제 |
| RETURNING | 대기 도크로 복귀 중 | ★도크 좌표 best-effort — rear 먼저, front 다음(순차) |
| DONE | (message="주차 완료") | 정상 종료 |

실패 시에는 `state="FAILED"`로 바로 전이하고 `current_step`에 구체 사유가 담긴다(예:
`"MOVING: navigate_to_pose[carry] 실패(status=6)"`, `"APPROACHING: align_vehicle
실패(final_error=0.421)"`). 어느 단계에서 `FAILED`가 났는지가 곧 "어디를 튜닝해야 하는지"다.

**소요 시간:** 각 단계의 내부 안전망 타임아웃은 상한일 뿐 실제 소요와는 다르다(정상이면
훨씬 빨리 끝남). 참고용 상한(`robot_task_orchestrator.py` 상수, 근거는 파일 내 주석):
`PICKED_UP/PARKED` 최대 20s, `MOVING`(carry_to) 최대 300s, `ARRIVED`(회전) 최대 90s,
`RETURNING`(rear+front 순차) 최대 660s. 즉 케이스 3 전체가 최악의 경우 수 분~십수 분
걸릴 수 있다 — 멈춘 게 아니라 정상 범위일 수 있으니 GUI에서 실제 로봇이 움직이는지로
판단한다(완전히 멈춰 있는데 상태도 안 바뀌면 진짜 hang — 아래 "알려진 한계" 참고).

**사람이 GUI로 확인 (체크리스트):**
- [ ] 로봇 2대(`robot_rear`/`robot_front`)가 인계 베이(Pickup, USD x≈-29.6)에서 차량 밑으로
      진입(APPROACHING)
- [ ] 팔 4개 전개 + 리프트 상승 — 차량이 살짝 들림(PICKED_UP)
- [ ] 편대가 서쪽 벽 개구부를 재통과해 A2까지 이동(MOVING) — **직선 경로, 장애물 회피
      없음**. 벽에 스치거나 개구부에서 걸리면 튜닝 필요 지점으로 기록
- [ ] A2 부근에서 편대가 제자리 회전해 **A열 방향(180°)**으로 정렬(ARRIVED)
- [ ] 리프트 하강 + 팔 접힘으로 A2에 안착(PARKED)
- [ ] 로봇 2대가 각자 West 대기 도크(`West_A_WaitingDock`/`West_B_WaitingDock`)로 복귀
      (RETURNING) — 정확한 도크 정위치인지, 경로가 자연스러운지 관찰
- [ ] `task_state`가 `DONE`으로 종료

**최종 확인 — 점유 갱신 (터미널 C 재사용):**
```bash
ros2 topic echo --once /parking_slots
```
**기대:** `A2`가 이제 `"occupied": true`(로봇이 실제로 차를 놓은 world 좌표가 A2 슬롯
반경 안에 들어왔다는 뜻 — runner가 매 ~2Hz 재계산하므로 DONE 이후 곧바로 반영됨).

---

## 정리(종료) 순서

1. 터미널 D/E: 서비스콜은 1회성이라 별도 종료 불필요, `task_state echo`는 Ctrl+C.
2. 터미널 B: Ctrl+C로 `ros2 launch` 종료(9개 노드 함께 내려감).
3. 터미널 A: Isaac Sim GUI 창을 닫거나 Ctrl+C(정리에 시간이 걸릴 수 있음, 강제 종료 지양).

---

## 알려진 한계 — best-effort 구간 (튜닝 필요 가능성 높음)

아래는 **원본 `dock_lift_handoff_mission.py`에 전례가 없어 이번 P1에서 새로 작성된** 코드다
(원본 검증 로직 이식분인 `pickup_sequence`/`goto_xz`/`rotate_to`/`ingress_to`는 신뢰도가
높음 — 대조·회귀 테스트 완료). 이번 태스크(P1 관통)에서는 순수 로직/가짜 액션서버로만
검증했고 **실제 Isaac 물리로는 이 런북이 최초 검증**이므로, 다음 중 하나라도 GUI에서
관찰되면 "버그"가 아니라 "예상된 후속 튜닝 대상"으로 기록한다:

- **`FormationMotion.carry_to`** (`formation_motion.py`) — 인계 베이(x≈-29.6)에서 목표
  슬롯까지 ~22m를 **중간 웨이포인트 없는 단순 직선 폐루프 하나**로 이동한다. 서쪽 벽
  개구부 재통과, 다른 차량/장애물 회피 로직이 없다. `CARRY_TO_TIMEOUT=300s`도 이론상
  근접치(≈240s)라 여유가 크지 않다. 막히거나 벽에 걸리면: 웨이포인트 분할 호출로 전환
  검토.
- **`FormationMotion.carry_rotate_to`** — 파지 후 편대 회전은 원본에 전례가 전혀 없다.
  두 로봇을 **동시(같은 tick)**에 회전시키도록 설계했는데(순차가 아님 — 강체로 잡은
  차량이 한쪽만 돌면 서로 밀고 당길 것이라는 추론), 실제 그립 유격에 따라 순차가 더
  매끄러울 수도 있다. GUI 관찰 없이는 어느 쪽이 맞는지 확인 불가.
- **RETURNING 도크 좌표** (`robot_task_orchestrator.py`의 `DOCK_X_MAP`/`DOCK_Y_REAR_MAP`/
  `DOCK_Y_FRONT_MAP`) — 좌표 변환의 왕복 상쇄(map↔USD)는 수식·통합스모크로 검증했지만,
  실제 도크 "정위치"(안착 자세, 장애물 없는 진입)까지는 보장하지 않는다.

이 셋 중 무언가 조정이 필요하다고 판단되면 각각 `formation_motion.py`(`carry_to`/
`carry_rotate_to` 메서드 docstring에 TODO 명시됨) 또는 `robot_task_orchestrator.py`의
`## RETURNING` 주석 블록을 참고해 후속 태스크로 진행한다.

---

## 문제 해결(트러블슈팅)

| 증상 (`ros2 service call` 응답 `message`) | 원인 | 조치 |
|---|---|---|
| `"관제 데이터 없음(재시도)"` | `parking_slot_manager`가 아직 `/parking_slots`를 한 번도 못 받음 | 터미널 A가 `DOCK_LIFT_HANDOFF_READY`를 찍었는지, 터미널 B/C/D 환경(도메인 126/RMW/프로파일 unset)이 A와 일치하는지 확인. `ros2 topic hz /parking_slots`로 발행 여부 직접 확인 |
| `"관제(dispatcher) 미기동"` / `"dispatcher 응답 없음"` | 터미널 B(`ros2 launch`)가 안 떠 있거나 `task_dispatcher`가 크래시 | 터미널 B 로그 확인, `ros2 node list`에 `task_dispatcher` 있는지 |
| `"실행 서버(orchestrator) 미기동"` | `robot_task_orchestrator` 미기동 | launch에 포함돼 있으므로 정상 상황이면 발생 안 함 — 터미널 B 로그에서 크래시 확인 |
| `ros2 topic list`에 `/parking_slots`가 안 보임 | 터미널 A/B 도메인 불일치, 또는 A가 아직 READY 전 | `echo $ROS_DOMAIN_ID`가 양쪽 다 126인지, `unset FASTRTPS_DEFAULT_PROFILES_FILE FASTDDS_DEFAULT_PROFILES_FILE` 했는지 재확인 |
| `task_state`가 특정 상태에서 몇 분째 안 바뀜 | 위 "알려진 한계" best-effort 구간에서 실제로 멎었을 가능성 | GUI에서 로봇이 물리적으로 움직이는지 관찰. 완전 정지면 해당 단계 안전망 타임아웃(최대 5~11분, 단계별로 다름 — 위 표 참고) 후 `FAILED`로 자동 전이할 때까지 대기하거나, 원인 파악 후 후속 튜닝 |
| Isaac 콘솔에 `Could not import system rclpy` 등 | 터미널 A에서 실수로 `/opt/ros/humble/setup.bash`를 source함 | 새 터미널로 다시 시작(A는 절대 시스템 ROS를 source하지 않음) |

---

## 참고: launch 파일

`src/parking_robot_system/launch/parking_robot_system.launch.py`는 노드 9개를 띄우기만
할 뿐 `ROS_DOMAIN_ID`/`RMW_IMPLEMENTATION` 등 환경변수를 스스로 설정하지 않는다(파일
상단에 이 런북을 가리키는 주석을 추가해 둠) — **`ros2 launch`를 실행하는 터미널 자체가**
위 "ROS 환경 한 줄 요약"을 미리 소싱해 둔 상태여야 한다.
