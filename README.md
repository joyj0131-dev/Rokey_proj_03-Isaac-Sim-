# 발렛파킹 통합 시스템 — 멀티 PC 배치 가이드

2대 협동로봇이 트럭을 들어 옮기는 발렛파킹 시스템. **입차(entry)**·**출차(exit)** 로봇쌍이
각각 다른 PC에서 제어되고, **관제탑(control tower)**이 UI 요청을 받아 두 쪽으로 라우팅한다.
Isaac Sim이 4대 로봇의 가상 하드웨어(odom·카메라·cmd_vel)를 DDS로 뿌린다.

> 실제 하드웨어로 가면 Isaac 자리는 진짜 로봇 4대가 대신한다 — 나머지 구조는 동일.

---

## 1. 시스템 구성

로봇 4대(`site_map_v4.py:43`): `entry_lead`, `entry_follow`, `exit_lead`, `exit_follow`

| PC 역할 | 워크스페이스 | 실행 | 담당 |
|---|---|---|---|
| **관제(Control Tower)** | featureUI | `ros2 launch parking_control control_tower.launch.py` | UI 접수·라우팅(task_dispatcher), DB, 슬롯관리, 안전(safety_supervisor/monitor), 로봇위치→DB |
| **입차(Entry)** | cobot_ws | `ros2 launch parkbot_motion nodes.launch.py` | `entry_lead/follow` 제어 스택 |
| **출차(Exit)** | featureUI | `ros2 launch parkbot_motion exit_nodes.launch.py` | `exit_lead/follow` 제어 스택 |
| **Isaac** | (GPU 있는 곳) | `./isaacpjt/Isaac_envo/sim_bridge.sh` | 가상 세계 1개 + 로봇 4대 스폰 |

Isaac은 관제 PC에 얹거나 별도 PC 어디든 상관없다 — **단 1개**여야 한다(§4 참고).

---

## 2. 왜 동시에 띄워도 안 부딪히나 (ROS 그래프 격리)

입차/출차가 이름 공간을 완전히 나눠 써서 한 DDS 도메인(126)에서 공존한다.

| 축 | 입차 | 출차 |
|---|---|---|
| 로봇 네임스페이스 | `/robot_entry_*` | `/robot_exit_*` |
| 미션 액션 | `/entry/execute_parking_task`, `/entry/carry_to_slot` | `/exit/execute_parking_task`, `/exit/carry_to_slot` |
| per-robot 액션/토픽 | `/robot_entry_*/navigate_to_pose(_fused)`, `/…/odom`, `/…/pose`, `/…/cmd_vel` | `/robot_exit_*/…` |
| 노드명 | `pose_controller_*_entry_*`, `ingress_entry_*`, `carry_action_server`, `pickup_orchestrator_node` | `*_exit_*`, `exit_carry_action_server`, `exit_pickup_orchestrator_node` |

`ExecuteParkingTask.action` / `CarryToSlot.action` 정의는 두 워크스페이스가 **바이트 동일**이라
관제→각 orchestrator 크로스머신 액션 호출이 붙는다.

---

## 3. 요청 흐름

```
   [웹 UI]
      │  RequestParkingTask (request_type = ENTRY | EXIT)
      ▼
  dispatch_parking_task ── task_dispatcher (관제 PC)
      │                         │  빈 슬롯 조회 / DB 작업 생성 / 안전 게이트 확인
      │        request_type 로 분기
      ├─ ENTRY ─▶ /entry/execute_parking_task ─▶ pickup_orchestrator_node       (입차 PC)
      └─ EXIT  ─▶ /exit/execute_parking_task  ─▶ exit_pickup_orchestrator_node  (출차 PC)
```

라우팅 근거: `task_dispatcher_node.py:90-95`(ENTRY/EXIT 액션 클라이언트),
`nodes.launch.py:36`(입차 서빙 이름), `exit_nodes.launch.py:213`(출차 서빙 이름).

---

## 4. 실행 순서

모든 셸에서 **먼저** DDS 환경을 맞춘다(브리지·노드가 같은 프로파일이어야 데이터가 흐른다):

```bash
export ROS_DOMAIN_ID=126
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE=~/.ros/fastdds_whitelist.xml
export FASTDDS_DEFAULT_PROFILES_FILE=~/.ros/fastdds_whitelist.xml
# 화이트리스트 XML 안에 네 PC(관제/입차/출차/Isaac)의 IP(10.10.0.x)가 모두 들어있어야 함
```

1. **Isaac PC** — 세계 + 로봇 4대 스폰
   ```bash
   cd ~/p3/cobot_ws && ./isaacpjt/Isaac_envo/sim_bridge.sh
   ```
2. **관제 PC** — 반드시 먼저(안전 게이트 때문에, §5-2)
   ```bash
   cd <featureUI_ws> && source install/setup.bash
   ros2 launch parking_control control_tower.launch.py
   ```
3. **입차 PC**
   ```bash
   cd ~/p3/cobot_ws && source install/setup.bash
   ros2 launch parkbot_motion nodes.launch.py
   ```
4. **출차 PC**
   ```bash
   cd <featureUI_ws> && source install/setup.bash
   ros2 launch parkbot_motion exit_nodes.launch.py   # auto_start:=false 기본(UI 대기)
   ```
5. **웹 UI**에서 입차/출차 요청 → 각 PC 로봇이 자동으로 움직인다.

> **Isaac은 1개만.** 4대 로봇이 같은 트럭·같은 주차장을 공유해야 서로 충돌회피·핸드오프가
> 성립한다. entry용/exit용 Isaac을 따로 쪼개면 트럭이 두 개인 별세계가 돼 통합 운영이 깨진다.

---

## 5. 필수 전제조건 (안 지키면 "UI 눌러도 로봇이 안 움직임")

1. **도메인 126 + fastdds 화이트리스트** — 네 PC 모두 동일 프로파일, 화이트리스트에 서로의 IP 등록.
   빠지면 토픽은 discovery로 보여도 데이터가 안 흐른다(publisher count 0).
2. **`safety_supervisor`가 살아서 `/safety/state`를 발행**해야 한다.
   task_dispatcher는 fail-safe라 안전상태를 못 받으면(`UNKNOWN`) **모든 요청을 막는다**
   (`task_dispatcher_node.py:78-80,123`). → 관제 `control_tower.launch`를 통째로 띄우면 해결.
   **가장 흔한 함정**: 안전노드 빼고 dispatcher만 띄우면 조용히 전부 거부됨.
3. **양쪽 orchestrator가 떠서 액션 서버 대기 중**일 것.
4. **UI가 중앙 `dispatch_parking_task`(RequestParkingTask)를 호출**할 것. 출차 스택이 자기
   게이트웨이(`dispatch_parking_task_exit`)도 띄우는데, 중앙 경로를 쓰면 그건 안 쓰이는 잉여다.
   UI가 어느 서비스를 부르는지만 맞추면 된다.

---

## 6. 하지 말 것 (충돌·오작동 유발)

- **입차 PC에서 `user_request_gateway_node`를 수동 실행하지 말 것.**
  기본 서비스명이 `dispatch_parking_task`라 관제 task_dispatcher와 **같은 이름 서버 2대**가 되고,
  UI 요청이 랜덤 라우팅돼 로봇이 안 움직인다. `nodes.launch.py`는 의도적으로 안 띄운다 —
  건드리지만 않으면 안전. (원천봉쇄하려면 `user_request_gateway_node.py`를 삭제)
- **한 PC에 cobot_ws와 featureUI 워크스페이스를 동시에 소스하지 말 것.**
  둘 다 `parking_robot_interfaces`라는 같은 패키지명인데 내용이 서로 다르다(cobot_ws는
  `IngressUnderTruck`에 egress 필드 O·안전 msg 없음 / featureUI는 그 반대). 어느 쪽도 상위집합이
  아니라 한쪽이 반드시 깨진다. **PC를 나눠 각자 자기 워크스페이스만 소스**하면 문제없다.
- **`sim_orchestrator`(가짜 로봇 데모)를 추가로 띄우지 말 것.** DB에 가짜 로봇 위치가 섞여
  `robot_position_bridge`의 실측과 충돌한다(`control_tower.launch.py` 주석에도 명시).

---

## 7. 단독 테스트 (통합 배치 없이)

- **입차만** 이 PC에서 검증: Isaac + `nodes.launch.py`만 띄우면 깨끗하게 돈다(§6 함정 다 무관).
  솔로 격리하려면 `ROS_DOMAIN_ID=130` 등으로 도메인을 갈라 다른 PC와 안 섞이게 한다.
  ```bash
  ros2 action send_goal /entry/execute_parking_task \
    parking_robot_interfaces/action/ExecuteParkingTask "{slot_id: 'A1'}"
  ```
- **복귀(RETURN) 반복검증**: `RETURN_TEST=1`로 sim_bridge·nodes.launch를 띄우면 주차완료
  상태로 스폰돼 복귀 시퀀스만 바로 테스트한다.
  ```bash
  RETURN_TEST=1 ./isaacpjt/Isaac_envo/sim_bridge.sh          # Isaac
  RETURN_TEST=1 ros2 launch parkbot_motion nodes.launch.py    # 노드
  ```

---

## 참고

- 로봇/도크/슬롯 좌표의 유일 출처: `src/parkbot_aruco/parkbot_aruco/site_map_v4.py`
- 입차 제어 상세: `src/parkbot_motion/parkbot_motion/pickup_orchestrator_node.py`
- 마커 측위(전후방 이중카메라 융합): `src/parkbot_aruco/parkbot_aruco/marker_localizer_node.py`
