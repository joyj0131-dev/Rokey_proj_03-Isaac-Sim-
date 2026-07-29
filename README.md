# 발렛파킹 통합 시스템 — 2로봇 협동 · Isaac Sim + ROS2

협동로봇 **2대가 한 팀**이 되어 트럭을 들어 옮기는 자율 발렛파킹 시스템이다.
**입차(entry)** 팀과 **출차(exit)** 팀이 각각 다른 PC에서 제어되고,
**관제(control tower)** 가 웹 UI 요청을 받아 두 팀으로 라우팅한다.
Isaac Sim이 로봇 4대의 가상 하드웨어(odom·카메라·`cmd_vel`)를 DDS로 뿌린다.

> 실제 하드웨어로 가면 Isaac 자리에 진짜 로봇 4대가 들어온다 — 나머지 구조는 그대로.

---

## 저장소 구조

```
p3/
├─ entry/        입차 PC용 워크스페이스 (구 cobot_ws)
│  ├─ src/       parkbot_motion · parkbot_aruco · parking_robot_interfaces
│  └─ isaacpjt/  Isaac Sim 러너(sim_bridge.sh)
├─ exit_ws/      출차 PC용 워크스페이스
│  ├─ src/       parkbot_motion · parkbot_aruco · parking_robot_interfaces
│  ├─ isaacpjt/  Isaac Sim 러너
│  └─ parking/ source/ *.usd  Isaac 에셋·빌더
├─ UI/           관제 PC용
│  └─ ui_ws/
│     ├─ src/parking_control        task_dispatcher·안전·슬롯·DB 브리지
│     └─ parking_control_mvp/       웹 UI (FastAPI)
└─ README.md     (이 파일)
```

로봇 4대(좌표 유일 출처 `src/parkbot_aruco/parkbot_aruco/site_map_v4.py`):
`entry_lead`, `entry_follow`, `exit_lead`, `exit_follow`

| 역할 | 폴더 | 실행 | 담당 |
|---|---|---|---|
| **관제** | `UI/ui_ws` | `ros2 launch parking_control control_tower.launch.py` + 웹UI | UI 접수·라우팅, MySQL, 슬롯관리, 안전, 로봇위치→DB |
| **입차** | `entry` | `ros2 launch parkbot_motion nodes.launch.py` | `entry_lead/follow` 제어 |
| **출차** | `exit_ws` | `ros2 launch parkbot_motion exit_nodes.launch.py` | `exit_lead/follow` 제어 |
| **Isaac** | (GPU 있는 곳) | `./isaacpjt/Isaac_envo/sim_bridge.sh` | 가상 세계 1개 + 로봇 4대 스폰 |

---

## 0. 준비물 (Prerequisites)

- **Ubuntu 22.04 + ROS 2 Humble** (`source /opt/ros/humble/setup.bash`)
- **Isaac Sim** — `sim_bridge.sh`가 `~/dev_ws/isaac_sim/isaacsim/_build/linux-x86_64/release` 경로를
  기대한다. 다른 곳에 설치했으면 `sim_bridge.sh`의 `REL=` 한 줄을 고쳐라.
- **MySQL (또는 MariaDB)** — 관제 DB. (입차/출차 PC엔 불필요, 관제 PC만)
- **Python 3 + venv** — 웹 UI(FastAPI) 실행용
- (멀티 PC로 돌릴 때만) 4대가 같은 서브넷 **`10.10.0.x`** 로 묶인 폐쇄망

> ⚠️ 1대 PC에서 전부 돌려도 된다(아래 §3). 단 **터미널마다 워크스페이스를 하나만 source** 해야 한다 —
> 이유는 §6 첫 항목.

---

## 1. 최초 1회 세팅

### 1-1. DDS 화이트리스트 (⭐ 필수 — 안 하면 "토픽은 보이는데 데이터가 안 흐름")

Isaac 번들 FastDDS와 시스템 Humble FastDDS는 **공유메모리(SHM) 전송이 크로스버전 비호환**이다.
UDP를 강제하는 화이트리스트를 걸어야 브리지↔노드 데이터가 흐른다. 아래를
`~/.ros/fastdds_whitelist.xml` 로 저장:

```xml
<?xml version="1.0" encoding="UTF-8" ?>
<dds xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles">
  <profiles>
    <transport_descriptors>
      <transport_descriptor>
        <transport_id>closed_net_udp</transport_id>
        <type>UDPv4</type>
        <interfaceWhiteList>
          <address>127.0.0.1</address>   <!-- 1대 PC면 이 줄만 있어도 됨 -->
          <address>10.10.0.1</address>   <!-- 멀티 PC면 각 PC의 IP를 -->
          <address>10.10.0.2</address>   <!-- 전부 나열 (관제/입차/출차/Isaac) -->
          <address>10.10.0.3</address>
          <address>10.10.0.4</address>
          <address>10.10.0.5</address>
        </interfaceWhiteList>
      </transport_descriptor>
    </transport_descriptors>
    <participant profile_name="closed_net" is_default_profile="true">
      <rtps>
        <userTransports><transport_id>closed_net_udp</transport_id></userTransports>
        <useBuiltinTransports>false</useBuiltinTransports>
      </rtps>
    </participant>
  </profiles>
</dds>
```

### 1-2. 공통 DDS 환경변수 (⭐ Isaac·노드 **모든** 터미널에서 먼저)

```bash
export ROS_DOMAIN_ID=126
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE=~/.ros/fastdds_whitelist.xml
export FASTDDS_DEFAULT_PROFILES_FILE=~/.ros/fastdds_whitelist.xml
```
(`~/.bashrc`에 넣어두면 편함)

### 1-3. MySQL 세팅 (관제 PC만)

접속 정보는 코드 기본값(`UI/ui_ws/src/parking_control/parking_control/core/db.py`)과 맞춰야 한다:
`host=localhost, user=parking, password=parking1234, database=parking`.

```bash
sudo mysql -e "CREATE DATABASE IF NOT EXISTS parking;
  CREATE USER IF NOT EXISTS 'parking'@'localhost' IDENTIFIED BY 'parking1234';
  GRANT ALL PRIVILEGES ON parking.* TO 'parking'@'localhost'; FLUSH PRIVILEGES;"

# 스키마·시드 순서대로 로드
for f in UI/ui_ws/src/parking_control/db/0*.sql; do
  echo "load $f"; mysql -u parking -pparking1234 parking < "$f"
done
```

### 1-4. 빌드

각 워크스페이스를 **각자** 빌드한다(서로 독립).

```bash
# 관제
cd UI/ui_ws        && colcon build && source install/setup.bash
# 입차
cd entry           && colcon build && source install/setup.bash
# 출차
cd exit_ws         && colcon build && source install/setup.bash
```

웹 UI(FastAPI)는 venv에 따로:
```bash
cd UI/ui_ws/parking_control_mvp
python3 -m venv .venv && source .venv/bin/activate
pip install -r requirements.txt
```

---

## 2. 실행 (통합 — 터미널 여러 개)

> 각 터미널은 **1-2 DDS 환경변수 → 자기 워크스페이스 source** 를 먼저 한 상태여야 한다.

**① Isaac** (세계 + 로봇 4대 스폰). 통합 운영이면 **딱 1개**만, 4대 카메라 전부 렌더:
```bash
cd entry     # 또는 exit_ws — 같은 씬을 띄운다
BRIDGE_CAMERAS=all ./isaacpjt/Isaac_envo/sim_bridge.sh
```
> `BRIDGE_CAMERAS`로 렌더할 로봇 카메라를 고른다. 기본값 `entry_lead,entry_follow`(입차만 —
> GPU 렌더부하↓, RTF↑). 통합 데모는 `all` 필수. 카메라를 줄일수록 RTF가 크게 오른다.

**② 관제** (⭐ 반드시 입·출차보다 **먼저** — §6-2 안전 게이트):
```bash
cd UI/ui_ws && ros2 launch parking_control control_tower.launch.py
```

**③ 입차**:
```bash
cd entry && ros2 launch parkbot_motion nodes.launch.py
```

**④ 출차** (`auto_start:=false` 기본 — UI 요청 대기):
```bash
cd exit_ws && ros2 launch parkbot_motion exit_nodes.launch.py
```

**⑤ 웹 UI**:
```bash
cd UI/ui_ws/parking_control_mvp && source .venv/bin/activate
python3 -m uvicorn main:app --host 0.0.0.0 --port 8000
# 브라우저 http://127.0.0.1:8000  (API 문서: /docs)
```
UI에서 입차/출차를 요청하면 해당 PC 로봇이 자동으로 움직인다.

> **Isaac은 통합 시 1개.** 4대가 같은 트럭·같은 주차장을 공유해야 충돌회피·핸드오프가 성립한다.
> 입차용/출차용 Isaac을 따로 쪼개면 트럭이 둘인 별세계가 돼 통합이 깨진다.
> (각 워크스페이스에 `isaacpjt`가 있는 건 **단독 개발**용이다 — §3)

---

## 3. 단독 / 1-PC 테스트 (통합 배치 없이)

- **입차만** 검증: Isaac + `entry`의 `nodes.launch.py`만 띄우면 깨끗하게 돈다(§6 함정 무관).
  다른 PC와 안 섞이게 하려면 `ROS_DOMAIN_ID=130` 등으로 도메인을 갈라라.
  ```bash
  ros2 action send_goal /entry/execute_parking_task \
    parking_robot_interfaces/action/ExecuteParkingTask "{slot_id: 'A1'}"
  ```
- **복귀(RETURN) 반복검증**: `RETURN_TEST=1`을 sim_bridge·nodes.launch 양쪽에 주면
  주차완료 상태로 스폰돼 복귀 시퀀스만 바로 테스트한다.
  ```bash
  RETURN_TEST=1 ./isaacpjt/Isaac_envo/sim_bridge.sh          # Isaac
  RETURN_TEST=1 ros2 launch parkbot_motion nodes.launch.py    # 노드
  ```

---

## 4. 왜 동시에 띄워도 안 부딪히나 (ROS 그래프 격리)

입차/출차가 이름공간을 완전히 나눠 써서 한 DDS 도메인(126)에서 공존한다.

| 축 | 입차 | 출차 |
|---|---|---|
| 로봇 네임스페이스 | `/robot_entry_*` | `/robot_exit_*` |
| 미션 액션 | `/entry/execute_parking_task`, `/entry/carry_to_slot` | `/exit/execute_parking_task`, `/exit/carry_to_slot` |
| per-robot 토픽 | `/robot_entry_*/odom`, `/…/pose`, `/…/cmd_vel` | `/robot_exit_*/…` |
| 노드명 | `pickup_orchestrator_node`, `carry_action_server`, `ingress_entry_*` | `exit_pickup_orchestrator_node`, `exit_carry_action_server`, `*_exit_*` |

`ExecuteParkingTask.action` / `CarryToSlot.action` 정의가 세 워크스페이스에서 동일해야
관제→각 orchestrator 크로스머신 액션 호출이 붙는다.

---

## 5. 요청 흐름

```
   [웹 UI]  (UI/ui_ws/parking_control_mvp, :8000)
      │  RequestParkingTask (request_type = ENTRY | EXIT)
      ▼
  dispatch_parking_task ── task_dispatcher (관제 PC)
      │                       빈 슬롯 조회 / DB 작업 생성 / 안전 게이트 확인
      │   request_type 로 분기
      ├─ ENTRY ─▶ /entry/execute_parking_task ─▶ pickup_orchestrator_node       (입차 PC)
      └─ EXIT  ─▶ /exit/execute_parking_task  ─▶ exit_pickup_orchestrator_node  (출차 PC)
```

---

## 6. 함정 (안 지키면 "UI 눌러도 로봇이 안 움직임")

1. **한 터미널에서 워크스페이스 2개를 동시에 source 하지 말 것.**
   `entry`와 `UI(또는 exit_ws)`의 `parking_robot_interfaces`는 **같은 패키지명인데 내용이 다르다**
   (`entry`쪽 `IngressUnderTruck`엔 egress 필드 O·안전 msg X / `UI·exit_ws`쪽은 그 반대).
   어느 쪽도 상위집합이 아니라 섞으면 한쪽이 깨진다. **터미널마다 자기 워크스페이스만 source**.
2. **관제를 먼저 띄울 것.** `task_dispatcher`는 fail-safe라 `safety_supervisor`의 `/safety/state`를
   못 받으면(`UNKNOWN`) **모든 요청을 조용히 거부**한다. `control_tower.launch`를 통째로 띄우면 해결.
3. **DDS 환경(§1-1, §1-2)을 모든 터미널에 동일하게.** 빠지면 discovery로 토픽은 보여도
   데이터가 안 흐른다(publisher count 0).
4. **입차 PC에서 `user_request_gateway_node`를 수동 실행하지 말 것.** 기본 서비스명이
   `dispatch_parking_task`라 관제 task_dispatcher와 **같은 이름 서버 2대**가 돼 요청이 랜덤
   라우팅된다. `nodes.launch.py`는 의도적으로 안 띄운다.
5. **`sim_orchestrator`(가짜 로봇 데모)를 추가로 띄우지 말 것.** DB에 가짜 위치가 섞여
   `robot_position_bridge`의 실측과 충돌한다.

---

## 참고 파일맵

- 좌표(로봇/도크/슬롯)의 유일 출처: `*/src/parkbot_aruco/parkbot_aruco/site_map_v4.py`
- 입차 미션 오케스트레이션: `entry/src/parkbot_motion/parkbot_motion/pickup_orchestrator_node.py`
- 마커 측위(전후방 이중카메라 융합): `*/src/parkbot_aruco/parkbot_aruco/marker_localizer_node.py`
- carry(가상중심 강체제어) 로직: `entry/src/parkbot_motion/parkbot_motion/carry_action_server.py` + `formation.py`
- 라우팅 로직: `UI/ui_ws/src/parking_control/parking_control/task_dispatcher_node.py`
- 관제 DB 스키마: `UI/ui_ws/src/parking_control/db/0*.sql`
