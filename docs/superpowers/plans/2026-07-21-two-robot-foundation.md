# 2로봇 기반 구축 (E2E Plan 1/4) Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 팀원 관제 스택(dispatcher+존락+formation)을 이 머신에서 실기동하고, Isaac 안의
로봇 2대가 올바른 좌표 규약으로 `/robot_N/cmd_vel`·`/robot_N/odom` 통신하며, 관제 배정으로
팔로워가 리더를 실제로 추종하는 것까지 검증한다.

**Architecture:** 팀원 브랜치(`origin/feature/parking-control`)의 ROS 패키지 3종과 Isaac
듀얼 스크립트를 이 워크스페이스로 가져와, 지도를 인계장까지 확장하고, odom 좌표 부호를
실측으로 확정한 뒤, 관제→Isaac 실로봇 연결을 처음으로 성립시킨다. E2E 미션의 2~6단계
(ArUco 주행, 도킹, 운반)는 Plan 2~4에서 이 기반 위에 쌓는다.

**Tech Stack:** ROS 2 Humble(Python 3.10, 외부) + Isaac Sim 5.1(Python 3.11, 내부 rclpy) +
MySQL 8 + networkx + colcon/pytest.

## Global Constraints

- 이 머신은 GPU PhysX (RTX 5080). Isaac 스크립트 실행 전후 좀비 프로세스 확인·정리:
  `ps aux | grep -iE "isaac" | grep -v grep` 후 필요 시 `kill`.
- USD에 절대경로 금지, 원본 에셋 비파괴(서브레이어/참조로만 합성).
- 좌표 규약: ROS map 프레임, `ros_x = usd_x`, `ros_y = -usd_z` (Task 5에서 실측 확정).
- 네임스페이스: `/robot_1`, `/robot_2` (언더스코어 — 팀 확정).
- 존 락 계약: zone_ids 오름차순, 전부-아니면-무, robot_id/task_id 중 정확히 하나.
- 외부 ROS 터미널 공통 env (이하 "**외부 env**"라 칭함):
  ```bash
  source /opt/ros/humble/setup.bash
  source /home/rokey/cobot3_ws/install/setup.bash
  export ROS_DOMAIN_ID=126 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
  export FASTRTPS_DEFAULT_PROFILES_FILE=$HOME/.ros/fastdds_whitelist.xml
  ```
- Isaac 터미널은 `/opt/ros`를 소싱하지 않는다(내부 Humble libs 사용). 같은
  DOMAIN/RMW/whitelist env만 export.
- 팀원 원본 파일 수정 시 커밋 메시지에 무엇을 왜 바꿨는지 명시(추후 팀 합의/PR 소재).

## File Structure

- 가져옴(신규): `src/parking_control/`, `src/parking_robot_interfaces/`,
  `src/parking_robot_system/` — 팀원 패키지 3종 (원본 유지, 최소 수정)
- 가져옴(신규): `isaacpjt/Isaac_envo/parking/isaac_runtime.py`,
  `build_dual_robot_parking_field.py`, `run_dual_robot_ros2_field.py`,
  `isaacpjt/Isaac_envo/mecanum_ros2_drive_dual.py` — 팀원 Isaac 듀얼 스크립트
- Modify: `src/parking_control/scripts/generate_map.py` — 인계장 노드/존 확장
- Modify: `src/parking_control/test/test_pathfinder.py` — 인계장 경로 테스트 추가
- Modify: `isaacpjt/Isaac_envo/parking/build_dual_robot_parking_field.py` — 마커 환경 사용
- Modify: `isaacpjt/Isaac_envo/mecanum_ros2_drive_dual.py` — odom y부호 정정
- Create: `isaacpjt/Isaac_envo/verify_dual_odom.py` — 좌표 부호 실측 검증 노드(외부 ROS)
- Create: `isaacpjt/Isaac_envo/foundation_formation_demo.py` — 편대 추종 데모 감독(외부 ROS)

---

### Task 1: 팀원 패키지 가져오기 + 빌드 + 기존 테스트 통과

**Files:**
- Create(checkout): `src/parking_control/`, `src/parking_robot_interfaces/`, `src/parking_robot_system/`
- Create(checkout): `isaacpjt/Isaac_envo/parking/isaac_runtime.py`,
  `isaacpjt/Isaac_envo/parking/build_dual_robot_parking_field.py`,
  `isaacpjt/Isaac_envo/parking/run_dual_robot_ros2_field.py`,
  `isaacpjt/Isaac_envo/parking/run_dual_robot_ros2_field.sh`,
  `isaacpjt/Isaac_envo/mecanum_ros2_drive_dual.py`

**Interfaces:**
- Produces: colcon 빌드된 `parking_robot_interfaces`(FormationAssignment/FormationStop msg,
  AcquireZones/ReleaseZones srv, ExecuteParkingTask action), `parking_control` 노드 4종.
  이후 모든 Task가 `source install/setup.bash`로 이를 사용한다.

- [ ] **Step 1: 원격 최신화 + 파일 체크아웃**

```bash
cd /home/rokey/cobot3_ws
git fetch origin feature/parking-control
git checkout origin/feature/parking-control -- \
  src/parking_control src/parking_robot_interfaces src/parking_robot_system \
  isaacpjt/Isaac_envo/parking/isaac_runtime.py \
  isaacpjt/Isaac_envo/parking/build_dual_robot_parking_field.py \
  isaacpjt/Isaac_envo/parking/run_dual_robot_ros2_field.py \
  isaacpjt/Isaac_envo/parking/run_dual_robot_ros2_field.sh \
  isaacpjt/Isaac_envo/mecanum_ros2_drive_dual.py
git status --short | head   # src/와 Isaac 파일 5개가 staged로 보이면 정상
```

- [ ] **Step 2: 파이썬 의존성 확인/설치**

```bash
python3 -c "import networkx, yaml, mysql.connector" 2>&1
```
Expected: 출력 없음(전부 있음). `mysql.connector` ModuleNotFoundError면:
```bash
pip3 install mysql-connector-python
```
(networkx/yaml이 없으면 `pip3 install networkx pyyaml`)

- [ ] **Step 3: colcon 빌드**

```bash
cd /home/rokey/cobot3_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select parking_robot_interfaces parking_control parking_robot_system
```
Expected: `Finished <<< parking_control` 등 3개 패키지 성공, 에러 0.

- [ ] **Step 4: 팀원 기존 단위테스트 통과 확인**

```bash
cd /home/rokey/cobot3_ws/src/parking_control
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest test/ -v 2>&1 | tail -5
```
Expected: `38 passed` (팀원 문서 기준 38건. 몇 건 늘었을 수 있음 — 전부 passed면 OK)

- [ ] **Step 5: 인터페이스 msg 생성 확인**

```bash
cd /home/rokey/cobot3_ws && source install/setup.bash
python3 -c "
from parking_robot_interfaces.msg import FormationAssignment, FormationStop
print(FormationAssignment.get_fields_and_field_types())"
```
Expected: `{'robot_id': 'string', 'task_id': 'string', 'role': 'string', 'partner_robot_id': 'string', 'active': 'boolean'}`

- [ ] **Step 6: Commit**

```bash
cd /home/rokey/cobot3_ws
git add src isaacpjt/Isaac_envo/parking/isaac_runtime.py \
  isaacpjt/Isaac_envo/parking/build_dual_robot_parking_field.py \
  isaacpjt/Isaac_envo/parking/run_dual_robot_ros2_field.py \
  isaacpjt/Isaac_envo/parking/run_dual_robot_ros2_field.sh \
  isaacpjt/Isaac_envo/mecanum_ros2_drive_dual.py
git commit -m "feat: import teammate parking-control stack + Isaac dual-robot scripts"
```

---

### Task 2: MySQL 셋업 + 스키마/시드 + 로봇 2대 등록

**Files:** 없음 (머신 상태 구성 — 커밋 없음)

**Interfaces:**
- Produces: `parking` DB (테이블 7종 + 슬롯 16 + 존 + robot_1/robot_2 IDLE 행).
  dispatcher/slot_manager/dashboard가 이 DB를 읽는다.

- [ ] **Step 1: MySQL 설치 확인/설치**

```bash
mysql --version || sudo apt install -y mysql-server
sudo systemctl start mysql 2>/dev/null; systemctl is-active mysql
```
Expected: `active`

- [ ] **Step 2: DB/계정 생성**

```bash
sudo mysql -e "
CREATE DATABASE IF NOT EXISTS parking;
CREATE USER IF NOT EXISTS 'parking'@'localhost' IDENTIFIED BY 'parking1234';
GRANT ALL PRIVILEGES ON parking.* TO 'parking'@'localhost';
FLUSH PRIVILEGES;"
mysql -u parking -pparking1234 -e "SELECT 1;"
```
Expected: `1` 출력 (접속 성공)

- [ ] **Step 3: 스키마 적용 (001이 이미 dual-robot 갱신본인지 확인 후 004 조건부)**

```bash
cd /home/rokey/cobot3_ws/src/parking_control
mysql -u parking -pparking1234 parking < db/001_schema.sql
mysql -u parking -pparking1234 parking < db/003_add_robot_target.sql 2>/dev/null || true
grep -q follower_robot_id db/001_schema.sql \
  && echo "001에 follower 포함 — 004 불필요" \
  || mysql -u parking -pparking1234 parking < db/004_dual_robot_zone_owner.sql
```
Expected: 에러 없이 종료. (`Duplicate column` 에러가 나면 이미 적용된 것 — 무시)

- [ ] **Step 4: 시드는 Task 3의 지도 재생성 후 적용하므로 여기서는 로봇만 등록**

```bash
mysql -u parking -pparking1234 parking -e "
INSERT INTO robots (robot_id, status, x, y, battery_percent)
VALUES ('robot_1','IDLE',-15.3,-7.8,100.0),('robot_2','IDLE',-15.3,7.8,100.0)
ON DUPLICATE KEY UPDATE status='IDLE', x=VALUES(x), y=VALUES(y);
SELECT robot_id, status, x, y FROM robots;"
```
Expected: robot_1 (-15.3, -7.8) / robot_2 (-15.3, 7.8) 두 행 IDLE.
(좌표 = 서측 대기 도크. A행 usd z=+7.8→ros y=-7.8 이 규약이며 Isaac 스폰 도크와 일치)

- [ ] **Step 5: 검증**

```bash
mysql -u parking -pparking1234 parking -e "SHOW TABLES; SELECT COUNT(*) AS locks FROM zone_locks;"
```
Expected: 테이블 7종(parking_slots, robots, vehicles, tasks, zones, zone_locks, parking_lot_edges), locks=0

---

### Task 3: 지도 인계장 확장 + 경로 단위테스트

**Files:**
- Modify: `src/parking_control/scripts/generate_map.py`
- Modify: `src/parking_control/test/test_pathfinder.py`
- 재생성: `src/parking_control/config/parking_map.yaml`, `src/parking_control/db/002_seed.sql`

**Interfaces:**
- Consumes: 팀원 `build_map()` (기존 실내 노드/존 생성부)
- Produces: 노드 `HJ0`~`HJ6`(인계장 정션, 동→서), `H_A`/`H_B`(인계 베이, kind=`handoff_bay`),
  존 `ZH_GATE`, `ZH01`~`ZH06`. `PathFinder.find_path("H_B", "<슬롯>")`이 유효 경로 반환.

- [ ] **Step 1: 실패하는 테스트 작성** — `test/test_pathfinder.py` 끝에 추가:

```python
def test_handoff_bay_route_reaches_indoor_slot():
    """인계 베이(H_B) → 실내 슬롯(B3) 경로가 성립해야 E2E 미션 배차가 가능하다."""
    pf = PathFinder(ParkingMap.load(CONFIG_YAML))
    result = pf.find_path("H_B", "B3")
    assert result is not None
    assert result.nodes[0] == "H_B"
    assert "entrance" in result.nodes          # 서측 개구부를 지난다
    assert "ZH_GATE" in result.zones           # 인계장 게이트 존
    assert any(z.startswith("ZH") for z in result.zones)
    assert 20.0 < result.length < 60.0         # H_B(-29.6,+7.8)→B3 대략 30m대
```
(파일 상단의 기존 import/CONFIG_YAML 상수를 그대로 사용. 이름이 다르면 파일 내
기존 테스트가 쓰는 로더 호출을 복사해 맞춘다.)

- [ ] **Step 2: 실패 확인**

```bash
cd /home/rokey/cobot3_ws/src/parking_control
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest test/test_pathfinder.py -v 2>&1 | tail -3
```
Expected: 신규 테스트 FAIL (`H_B` 노드 없음 → find_path가 None 또는 KeyError)

- [ ] **Step 3: generate_map.py 확장** — `build_map()`의 `# 차량 출입구` 블록 바로 뒤에 추가,
시그니처에 `handoff_length=23.0` 파라미터 추가:

```python
    # 인계장(서측 실외) — 2026-07-21 재설계: 실내와 같은 단면(중앙 통로+양쪽 베이).
    # 정션 x는 ArUco 인계장 차선 마커 열(중심 ±k*3.4)과 동일 — 마커가 곧 보정점.
    handoff_center_x = -half_w - border_margin - handoff_length * 0.5   # -29.6
    hj_xs = [round(handoff_center_x + k * space_width, 3)
             for k in range(3, -4, -1)]                                 # -19.4 … -39.8
    for i, x in enumerate(hj_xs):
        nodes[f"HJ{i}"] = dict(x=x, y=0.0, kind="junction")
    edges.append(dict(u="entrance", v="HJ0", zone="ZH_GATE"))
    zones.append("ZH_GATE")
    for i in range(len(hj_xs) - 1):
        zone_id = f"ZH{i + 1:02d}"
        zones.append(zone_id)
        edges.append(dict(u=f"HJ{i}", v=f"HJ{i + 1}", zone=zone_id))
    # 인계 베이 2개. H_A: usd z=+7.8(A쪽)→ros y=-7.8 / H_B: 반대.
    for name, usd_z_sign in (("H_A", 1.0), ("H_B", -1.0)):
        nodes[name] = dict(x=round(handoff_center_x, 3),
                           y=round(-usd_z_sign * row_center, 3),
                           kind="handoff_bay")
        edges.append(dict(u=name, v="HJ3"))   # HJ3 = 베이 열 정션(-29.6)
```
`main()`의 argparse에 `parser.add_argument("--handoff-length", type=float, default=23.0)`
추가하고 `build_map(...)` 호출에 `handoff_length=args.handoff_length` 전달.

- [ ] **Step 4: 지도/시드 재생성 + DB 시드 적용**

```bash
cd /home/rokey/cobot3_ws/src/parking_control
python3 scripts/generate_map.py
mysql -u parking -pparking1234 parking < db/002_seed.sql
mysql -u parking -pparking1234 parking -e "SELECT COUNT(*) FROM parking_slots; SELECT zone_id FROM zones ORDER BY zone_id;" | tail -20
```
Expected: 슬롯 16, 존 목록에 `ZH01`~`ZH06`, `ZH_GATE` 포함(총 18존)

- [ ] **Step 5: 테스트 통과 확인 (기존 회귀 포함 전체)**

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest test/ -v 2>&1 | tail -4
```
Expected: 전부 PASS (기존 좌표 회귀 테스트 포함 — 실내 좌표는 변하지 않는다)

- [ ] **Step 6: Commit**

```bash
cd /home/rokey/cobot3_ws
git add src/parking_control/scripts/generate_map.py src/parking_control/config/parking_map.yaml \
  src/parking_control/db/002_seed.sql src/parking_control/test/test_pathfinder.py
git commit -m "feat: extend parking map with handoff area nodes/zones (HJ*, H_A/H_B, ZH*)"
```

---

### Task 4: 마커 환경 기반 듀얼 필드 USD

**Files:**
- Modify: `isaacpjt/Isaac_envo/parking/build_dual_robot_parking_field.py`
- 생성물: `isaacpjt/Isaac_envo/parking/parking_robot_field_dual_markers.usd`

**Interfaces:**
- Consumes: `parking_environment_with_markers.usd` (마커 42장 + 새 인계장 + 차량 8대, flatten)
- Produces: `/World/Robots/robot_1`, `/World/Robots/robot_2` (뎁스캠 메카넘 로봇, 서측 도크),
  Task 5·7의 러너가 이 USD를 로드.

- [ ] **Step 1: 빌더가 마커 환경을 쓰도록 수정** — 상수 2줄 변경:

```python
PARKING_USD = ROOT / "parking_environment_with_markers.usd"
OUTPUT_USD = ROOT / "parking_robot_field_dual_markers.usd"
```
서브레이어 경로도 함께:
```python
    stage.GetRootLayer().subLayerPaths.append("./parking_environment_with_markers.usd")
```
(로봇 에셋은 원래부터 `hwia_depth_cam_mecha_roller.usd` — 그대로 둔다)

- [ ] **Step 2: 헤드리스 물리 검증 실행**

```bash
ps aux | grep -iE "isaac" | grep -v grep   # 좀비 확인, 있으면 kill <pid>
cd /home/rokey/cobot3_ws/isaacpjt/Isaac_envo/parking
export ROS_DOMAIN_ID=126 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
python3 build_dual_robot_parking_field.py --headless-test
```
Expected: 로봇 2대·차량(주차 6+인계 2)·180프레임 변위 ≤0.35m PASS 메시지.
(차량 수 검증이 6+6을 기대해 FAIL하면 — 새 환경은 인계 대기 2대이므로 검증부의
기대값을 6+2로 수정하고 그 사실을 커밋 메시지에 남긴다)

- [ ] **Step 3: 러너의 FIELD_USD 경로 갱신** — `run_dual_robot_ros2_field.py`:

```python
FIELD_USD = ROOT / "parking_robot_field_dual_markers.usd"
```
그리고 새 인계장에서는 대기 차량이 미션 대상이므로 기본값을 유지로 변경:
```python
KEEP_HANDOFF_VEHICLES = "--hide-handoff-vehicles" not in sys.argv[1:]
```

- [ ] **Step 4: Commit**

```bash
cd /home/rokey/cobot3_ws
git add isaacpjt/Isaac_envo/parking/build_dual_robot_parking_field.py \
  isaacpjt/Isaac_envo/parking/run_dual_robot_ros2_field.py
git commit -m "feat: dual robot field on marker environment (with_markers, keep handoff vehicles)"
```
(USD 생성물은 재생성 가능하므로 빌더 실행으로 만들고, 파일 자체도 add해도 무방 —
기존 관례대로 커밋에 포함)

---

### Task 5: odom 좌표 부호 실측 확정

**Files:**
- Create: `isaacpjt/Isaac_envo/verify_dual_odom.py`
- Modify(필요시): `isaacpjt/Isaac_envo/parking/run_dual_robot_ros2_field.py` (`_isaac_pose_to_ros`)
- Modify: `isaacpjt/Isaac_envo/mecanum_ros2_drive_dual.py` (`isaac_pose_to_ros2d` — y부호 모순 정정)

**Interfaces:**
- Produces: 실측으로 확정된 변환 1벌 (`ros_x=usd_x, ros_y=-usd_z, yaw=atan2(-fwd_z, fwd_x)`
  또는 실측이 가리키는 수정본). 이후 모든 Plan이 이 변환을 신뢰한다.

- [ ] **Step 1: 검증 노드 작성** — `isaacpjt/Isaac_envo/verify_dual_odom.py`:

```python
#!/usr/bin/env python3
"""Isaac 듀얼 필드의 odom 좌표 규약을 실측 검증한다 (외부 ROS 터미널에서 실행).

로봇별로 3상: +x 전진 → odom x 증가 / +y 좌 strafe → odom y 증가(도크에서 yaw≈0 기준)
/ +wz → yaw 증가(CCW). 각 상 후 원위치 복귀는 하지 않는다(소변위, 빈 슬롯 위라 안전).
"""
import math
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node

ROBOTS = ("robot_1", "robot_2")
PUSH_SEC = 3.0
SPEED = 0.3
TURN = 0.4


def yaw_of(msg):
    q = msg.pose.pose.orientation
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class Verify(Node):
    def __init__(self):
        super().__init__("verify_dual_odom")
        self.odom = {}
        for r in ROBOTS:
            self.create_subscription(
                Odometry, f"/{r}/odom",
                lambda m, rid=r: self.odom.__setitem__(rid, m), 10)
        self.pubs = {r: self.create_publisher(Twist, f"/{r}/cmd_vel", 10) for r in ROBOTS}

    def snap(self, r):
        m = self.odom[r]
        p = m.pose.pose.position
        return p.x, p.y, yaw_of(m)

    def push(self, r, vx=0.0, vy=0.0, wz=0.0, sec=PUSH_SEC):
        t = Twist()
        t.linear.x, t.linear.y, t.angular.z = float(vx), float(vy), float(wz)
        end = time.time() + sec
        while time.time() < end:
            self.pubs[r].publish(t)
            rclpy.spin_once(self, timeout_sec=0.05)
        self.pubs[r].publish(Twist())  # 정지
        for _ in range(10):
            rclpy.spin_once(self, timeout_sec=0.1)


def main():
    rclpy.init()
    node = Verify()
    deadline = time.time() + 20
    while time.time() < deadline and len(node.odom) < len(ROBOTS):
        rclpy.spin_once(node, timeout_sec=0.2)
    if len(node.odom) < len(ROBOTS):
        print(f"VERIFY_RESULT=FAIL 이유=odom 미수신 ({list(node.odom)})")
        sys.exit(1)

    ok = True
    for r in ROBOTS:
        x0, y0, w0 = node.snap(r)
        node.push(r, vx=SPEED)
        x1, y1, w1 = node.snap(r)
        fwd = (x1 - x0) * math.cos(w0) + (y1 - y0) * math.sin(w0)
        f_ok = fwd > 0.4
        node.push(r, vy=SPEED)
        x2, y2, _ = node.snap(r)
        left = -(x2 - x1) * math.sin(w1) + (y2 - y1) * math.cos(w1)
        l_ok = left > 0.4
        node.push(r, wz=TURN, sec=2.0)
        _, _, w2 = node.snap(r)
        dyaw = (w2 - w1 + math.pi) % (2 * math.pi) - math.pi
        y_ok = dyaw > 0.3
        print(f"{r}: forward={fwd:+.2f}({'OK' if f_ok else 'BAD'}) "
              f"left={left:+.2f}({'OK' if l_ok else 'BAD'}) "
              f"dyaw={math.degrees(dyaw):+.1f}deg({'OK' if y_ok else 'BAD'})")
        ok = ok and f_ok and l_ok and y_ok
    print(f"VERIFY_RESULT={'PASS' if ok else 'FAIL'}")
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
```

- [ ] **Step 2: 필드 러너 기동 (Isaac 터미널, 백그라운드)**

```bash
ps aux | grep -iE "isaac" | grep -v grep   # 좀비 정리
cd /home/rokey/cobot3_ws/isaacpjt/Isaac_envo/parking
export ROS_DOMAIN_ID=126 RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
  FASTRTPS_DEFAULT_PROFILES_FILE=$HOME/.ros/fastdds_whitelist.xml
python3 run_dual_robot_ros2_field.py --headless --seconds 180 &
```
Expected: 로그에 `ROS2_DUAL_FIELD_READY robots=['robot_1', 'robot_2'] domain=126`

- [ ] **Step 3: 검증 노드 실행 (외부 env 터미널)**

```bash
cd /home/rokey/cobot3_ws && # 외부 env 적용 (Global Constraints 참고)
python3 isaacpjt/Isaac_envo/verify_dual_odom.py
```
Expected: `VERIFY_RESULT=PASS`.
- `left`가 음수(BAD)면: `run_dual_robot_ros2_field.py`의 `_isaac_pose_to_ros`에서
  `return x_world, -z_world, yaw`의 부호와 yaw 식(`atan2(-forward_z, forward_x)`)을
  실측 방향에 맞게 수정 후 러너 재기동·재검증. 수정 결과가 곧 "확정 규약"이다.
- `dyaw`가 음수면 yaw 식의 `-forward_z` 부호만 뒤집는다.

- [ ] **Step 4: dual 드라이버의 y부호 모순 정정** — `mecanum_ros2_drive_dual.py`의
`isaac_pose_to_ros2d`를 Step 3에서 확정된 변환과 동일하게 수정 (현재 `ros_y=+isaac_z`로
공식 규약과 반대. 스크립트는 남겨두되 규약만 통일):

```python
    ros_x = float(position[0])
    ros_y = -float(position[2])   # 확정 규약: ros_y = -usd_z (실측 <날짜> PASS)
```
(yaw 부분도 러너와 같은 식으로 통일)

- [ ] **Step 5: 러너 종료 + 좀비 확인 + Commit**

```bash
ps aux | grep -iE "isaac" | grep -v grep   # 남았으면 kill
cd /home/rokey/cobot3_ws
git add isaacpjt/Isaac_envo/verify_dual_odom.py \
  isaacpjt/Isaac_envo/mecanum_ros2_drive_dual.py \
  isaacpjt/Isaac_envo/parking/run_dual_robot_ros2_field.py
git commit -m "feat: empirically verify odom frame convention, fix y-sign inconsistency in dual driver"
```

---

### Task 6: 관제 스택 실기동 스모크 (배정까지)

**Files:** 없음 (실행 검증 — 문제 발견 시에만 수정 커밋)

**Interfaces:**
- Consumes: Task 2 DB, Task 3 지도, Task 1 노드들
- Produces: "접수→2대 배정→formation_assignment 방송→sim 완료→해제" 실동작 확인

- [ ] **Step 1: 관제 3노드 기동 (외부 env 터미널 3개, 각각)**

```bash
ros2 run parking_control parking_slot_manager
ros2 run parking_control task_dispatcher
ros2 run parking_control sim_orchestrator
```
Expected: 각 노드 시작 로그. dispatcher는 `allocator=nearest, zone_lock_mode=stub`

- [ ] **Step 2: 배정 관찰 준비 (외부 env 터미널)**

```bash
ros2 topic echo /formation_assignment &
ros2 service call /dispatch_parking_task parking_robot_interfaces/srv/RequestParkingTask \
  "{request_type: ENTRY, vehicle_id: CAR_FOUNDATION}"
```
Expected: 응답 `accepted=True`, message에 리더 배정. echo에
`robot_1(leader, partner=robot_2)` / `robot_2(follower, partner=robot_1)` 두 건 active=true,
sim_orchestrator 완료 후 두 건 active=false.

- [ ] **Step 3: DB 원장 확인**

```bash
mysql -u parking -pparking1234 parking -e \
  "SELECT task_id, state, robot_id, follower_robot_id, slot_id FROM tasks ORDER BY created_at DESC LIMIT 1;
   SELECT robot_id, status FROM robots;"
```
Expected: 최신 task에 robot_id=robot_1, follower_robot_id=robot_2, state=DONE,
robots 둘 다 IDLE 복귀.

- [ ] **Step 4: 노드 종료 (Ctrl-C) 후 기록** — 이상 발견 시 수정·커밋, 정상이면 커밋 없음.

---

### Task 7: 편대 추종 첫 실증 — 관제 배정으로 Isaac 팔로워가 리더를 따라온다

**Files:**
- Create: `isaacpjt/Isaac_envo/foundation_formation_demo.py`

**Interfaces:**
- Consumes: Task 4 필드, Task 5 확정 변환, `formation_gap_controller`(팀원, 파라미터
  `robot_id`, 네임스페이스 remap), `FormationAssignment` msg
- Produces: `FORMATION_DEMO=PASS` — 팔로워가 리더 2.9m 뒤로 수렴 유지. Plan 2의
  접근(호송) 주행이 이 위에 선다.

- [ ] **Step 1: 데모 감독 스크립트 작성** — `isaacpjt/Isaac_envo/foundation_formation_demo.py`:

```python
#!/usr/bin/env python3
"""편대 추종 첫 실증 (외부 ROS 터미널).

절차: ① robot_2에 follower 배정 방송 → ② robot_1을 스크립트가 직접 조종해
도크→통로(y≈0)→동쪽 6m 주행 → ③ robot_2가 gap-hold로 리더 2.9m 뒤에
수렴·유지하는지 측정. gap 오차 < 0.45m가 3초 유지되면 PASS.

주의: 리더 조종은 임시(Plan 2에서 ArUco navigate로 교체). 배정 방송은 본래
task_dispatcher 몫이지만, 여기서는 편대 계층만 떼어 검증하므로 직접 쏜다.
"""
import math
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node

from parking_robot_interfaces.msg import FormationAssignment

GAP = 2.9
TOL = 0.45
HOLD_SEC = 3.0
TIMEOUT = 120.0


def yaw_of(m):
    q = m.pose.pose.orientation
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class Demo(Node):
    def __init__(self):
        super().__init__("foundation_formation_demo")
        self.odom = {}
        for r in ("robot_1", "robot_2"):
            self.create_subscription(
                Odometry, f"/{r}/odom",
                lambda m, rid=r: self.odom.__setitem__(rid, m), 10)
        self.cmd1 = self.create_publisher(Twist, "/robot_1/cmd_vel", 10)
        self.assign = self.create_publisher(FormationAssignment, "/formation_assignment", 10)

    def pose(self, r):
        m = self.odom[r]
        p = m.pose.pose.position
        return p.x, p.y, yaw_of(m)

    def wait_odom(self):
        end = time.time() + 20
        while time.time() < end and len(self.odom) < 2:
            rclpy.spin_once(self, timeout_sec=0.2)
        return len(self.odom) == 2

    def broadcast(self, active):
        for rid, role, partner in (("robot_2", "follower", "robot_1"),
                                   ("robot_1", "leader", "robot_2")):
            self.assign.publish(FormationAssignment(
                robot_id=rid, task_id="foundation-demo" if active else "",
                role=role, partner_robot_id=partner, active=active))

    def drive_leader(self, vx, vy, until, timeout):
        t = Twist()
        t.linear.x, t.linear.y = float(vx), float(vy)
        end = time.time() + timeout
        while time.time() < end and not until():
            self.cmd1.publish(t)
            rclpy.spin_once(self, timeout_sec=0.05)
        self.cmd1.publish(Twist())


def main():
    rclpy.init()
    node = Demo()
    if not node.wait_odom():
        print("FORMATION_DEMO=FAIL 이유=odom 미수신"); sys.exit(1)

    node.broadcast(True)
    # 리더: 도크(y=-7.8)에서 통로 중심(y≈-0.3)으로 좌 strafe (A1/A2 빈 슬롯 위 통과)
    node.drive_leader(0.0, 0.45, lambda: node.pose("robot_1")[1] > -0.3, 30)
    # 리더: 통로를 동쪽으로 6m (팔로워가 이 사이 수렴해야 함)
    x_start = node.pose("robot_1")[0]
    hold_since, verdict = None, "FAIL"
    end = time.time() + TIMEOUT

    def gap_error():
        lx, ly, lw = node.pose("robot_1")
        fx, fy, _ = node.pose("robot_2")
        tx, ty = lx - GAP * math.cos(lw), ly - GAP * math.sin(lw)
        return math.hypot(tx - fx, ty - fy)

    t = Twist(); t.linear.x = 0.3
    while time.time() < end:
        node.cmd1.publish(t)
        rclpy.spin_once(node, timeout_sec=0.05)
        err = gap_error()
        if err < TOL:
            hold_since = hold_since or time.time()
            if time.time() - hold_since >= HOLD_SEC:
                verdict = "PASS"; break
        else:
            hold_since = None
        if node.pose("robot_1")[0] - x_start > 6.0:
            t.linear.x = 0.0   # 리더 도착 — 정지 상태 수렴도 인정
    node.cmd1.publish(Twist())
    node.broadcast(False)
    print(f"FORMATION_DEMO={verdict} 최종 gap 오차={gap_error():.2f}m")
    sys.exit(0 if verdict == "PASS" else 1)


if __name__ == "__main__":
    main()
```

- [ ] **Step 2: 필드 러너 기동 (Isaac 터미널)** — Task 5 Step 2와 동일, `--seconds 300`

- [ ] **Step 3: 팀원 formation 컨트롤러 2개 기동 (외부 env 터미널 2개)**

```bash
ros2 run parking_control formation_gap_controller --ros-args \
  -r __ns:=/robot_1 -p robot_id:=robot_1
ros2 run parking_control formation_gap_controller --ros-args \
  -r __ns:=/robot_2 -p robot_id:=robot_2
```
Expected: 각각 `배정 대기 중(idle)` 로그. (setup.py에 실행자 이름이 없으면
`python3 src/parking_control/parking_control/formation_gap_controller_node.py --ros-args ...`로 실행)

- [ ] **Step 4: 데모 실행 (외부 env 터미널)**

```bash
cd /home/rokey/cobot3_ws
python3 isaacpjt/Isaac_envo/foundation_formation_demo.py
```
Expected: `FORMATION_DEMO=PASS`. robot_2 컨트롤러 로그에 `배정 수신: role=follower`.
FAIL이면 gap 오차 추이를 robot_2 cmd_vel echo와 함께 관찰 — gap-hold는 diff-drive
모델이라 회전으로 방향을 맞추며 따라온다(회전 자체는 무적재라 허용). 수렴이 아예
안 되면 Task 5의 yaw 부호부터 재의심할 것.

- [ ] **Step 5: 전체 종료 + 좀비 정리 + Commit**

```bash
ps aux | grep -iE "isaac" | grep -v grep   # 정리
cd /home/rokey/cobot3_ws
git add isaacpjt/Isaac_envo/foundation_formation_demo.py
git commit -m "feat: first live formation-following demo (dispatcher-style assignment drives Isaac follower)"
```

---

## 완료 기준 (Plan 1 전체)

1. 팀원 패키지 3종 빌드 + 기존 pytest 전부 통과
2. `H_B → 실내 슬롯` 경로가 pathfinder에서 성립 (신규 테스트 포함 전체 PASS)
3. `VERIFY_RESULT=PASS` — odom 규약 실측 확정, dual 드라이버 부호 모순 해소
4. 관제 접수 → 2대 배정 → 방송 → 해제 실동작 (DB 원장 정합)
5. `FORMATION_DEMO=PASS` — Isaac 실로봇 팔로워가 리더 2.9m 뒤 수렴

## 다음 계획 (별도 문서로 작성 예정)

- **Plan 2**: ArUco 폐루프 navigate(M6) — 카메라 브리지 2대분, 리더 주행 백엔드,
  베이 마커 접근까지
- **Plan 3**: 도킹·파지 — 뎁스 스톱 통합, 순차 하부 진입, 팔 전개(팀원 C 함수 훅)
- **Plan 4**: 운반·하차 — 결합 게걸음(wz=0·횡오차항·facing 확장), task 존 락,
  슬롯 하차, 풀 E2E 리포트
