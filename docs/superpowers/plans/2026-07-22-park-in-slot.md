# 서비스 기반 지정 구역 주차 (P1) 구현 플랜

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 서비스로 주차 구역(slot_id)을 입력하면, 비어 있으면 같은 열의 기존 차량과 같은 방향으로 주차하고, 차량이 있으면 "주차 불가"를 반환하는 동작을 ROS2 파이프라인 전체를 관통해 구현한다.

**Architecture:** Isaac runner가 점유를 발행(`/parking_slots`)하고, ROS2 파이프라인(gateway→dispatcher→slot_manager→orchestrator→navigate/align/lift 액션서버)이 이를 검증·구동한다. 실제 모션은 동작하는 `dock_lift_handoff_mission`의 프리미티브를 공유 모듈 `formation_driver`로 추출해 액션서버가 사용한다.

**Tech Stack:** ROS2 Humble(rclpy, rosidl), Isaac Sim 5.1(omni.usd, isaacsim), Python 3.10, pytest.

## Global Constraints

- 좌표 규약: `x_map = x_usd`, `y_map = -z_usd` (`parking_map.yaml`·DB 시드와 일치).
- 주차 방향: A열 yaw=180°, B열 yaw=0° (`build_parking_environment.py:716`). 180°/0°는 `y=-z` 반사에 대해 불변이라 map/USD 동일.
- 슬롯 중심(USD): `x = -17.0 + (index+0.5)·3.4`, `z = +7.8`(A) / `-7.8`(B). `SPACE_WIDTH=3.4`, `SPACE_LENGTH=6.6`, `half_w=17.0`, `row_center=7.8`, `PARKING_INDICES=1..8`.
- ROS 환경: 모든 노드·서비스콜은 `ROS_DOMAIN_ID=126`, `RMW_IMPLEMENTATION=rmw_fastrtps_cpp`, `FASTRTPS_DEFAULT_PROFILES_FILE`/`FASTDDS_DEFAULT_PROFILES_FILE` unset. (Isaac 내부 rclpy와 발견되기 위한 실측 조건.)
- 커스텀 인터페이스는 `parking_robot_interfaces`에만 둔다. Isaac runner(내부 rclpy)는 커스텀 인터페이스를 쓰지 않고 `std_msgs/String`만 발행한다.
- 로봇 이름: `robot_rear`, `robot_front`. 토픽: `/robot_{name}/cmd_vel`(Twist), `/robot_{name}/odom`(Odometry), `/robot_{name}/arm_control`(std_srvs/SetBool), `/vehicle/pose`(PoseStamped).
- TDD·frequent commits. DRY. YAGNI. 원본 에셋/동작 코드는 비파괴(추출은 복사 후 위임).

---

## 파일 구조

**신규 (parking_robot_interfaces):**
- `srv/ParkInSlot.srv`, `srv/GetSlotInfo.srv` — 커스텀 서비스
- `CMakeLists.txt` (수정) — 위 2개 등록

**신규 (parking_robot_system/parking_robot_system/):**
- `frame_transform.py` — USD↔map 좌표/yaw 변환 (순수 함수)
- `slot_geometry.py` — slot_id → USD 슬롯 중심·목표 yaw, 라벨 검증 (순수 함수)
- `occupancy.py` — 차량 위치 목록 → 점유 슬롯 집합 (순수 함수)
- `formation_driver.py` — 모션 프리미티브 클래스 (dock_lift_handoff에서 추출)

**수정 (parking_robot_system/parking_robot_system/):**
- `parking_slot_manager.py`, `task_dispatcher.py`, `user_request_gateway.py`,
  `robot_task_orchestrator.py`, `navigate_action_server.py`, `align_action_server.py`,
  `lift_action_server.py`, `vehicle_detection_node.py` — 스텁 → 구현
- `setup.py` — 필요시 신규 실행 스크립트 없음(기존 entry point 유지)

**수정 (isaacpjt/Isaac_envo/):**
- `dock_lift_handoff_runner.py` — `/parking_slots` 점유 발행 추가 (모션 로직 불변)

**테스트 (parking_robot_system/test/):**
- `test_frame_transform.py`, `test_slot_geometry.py`, `test_occupancy.py`,
  `test_formation_driver.py`, `test_slot_manager.py`, `test_dispatcher.py`,
  `test_gateway.py`, `test_orchestrator.py`

---

## Task 1: 커스텀 서비스 인터페이스

**Files:**
- Create: `src/parking_robot_interfaces/srv/ParkInSlot.srv`
- Create: `src/parking_robot_interfaces/srv/GetSlotInfo.srv`
- Modify: `src/parking_robot_interfaces/CMakeLists.txt:9-24`

**Interfaces:**
- Produces: `parking_robot_interfaces/srv/ParkInSlot` (req: `string slot_id`; resp: `bool accepted, string task_id, string message`), `parking_robot_interfaces/srv/GetSlotInfo` (req: `string slot_id`; resp: `bool data_ready, bool exists, bool occupied, bool is_accessible, geometry_msgs/Pose pose`).

- [ ] **Step 1: ParkInSlot.srv 작성**
```
# 사용자 → user_request_gateway. 지정 구역 주차 요청(P1).
string slot_id
---
bool accepted
string task_id
string message
```

- [ ] **Step 2: GetSlotInfo.srv 작성**
```
# task_dispatcher → parking_slot_manager. 특정 구역 상태·좌표 조회(P1).
string slot_id
---
bool data_ready          # false = /parking_slots 캐시 비어있음(runner 미기동)
bool exists
bool occupied
bool is_accessible
geometry_msgs/Pose pose  # map 프레임, orientation = 목표 yaw(quaternion z/w)
```

- [ ] **Step 3: CMakeLists.txt에 등록** — `rosidl_generate_interfaces` 목록의 `"srv/FindEmptySlot.srv"` 다음 줄들에 추가:
```cmake
  "srv/ParkInSlot.srv"
  "srv/GetSlotInfo.srv"
```

- [ ] **Step 4: 빌드하여 생성 확인**

Run:
```bash
cd /home/rokey/p3/Rokey_proj_03-Isaac-Sim- && \
source /opt/ros/humble/setup.bash && \
colcon build --packages-select parking_robot_interfaces && \
source install/setup.bash && \
python3 -c "from parking_robot_interfaces.srv import ParkInSlot, GetSlotInfo; print('OK')"
```
Expected: `OK` (빌드 성공 + import 성공)

- [ ] **Step 5: Commit**
```bash
git add src/parking_robot_interfaces/srv/ParkInSlot.srv src/parking_robot_interfaces/srv/GetSlotInfo.srv src/parking_robot_interfaces/CMakeLists.txt
git commit -m "feat(interfaces): ParkInSlot/GetSlotInfo srv 추가 (P1)"
```

---

## Task 2: frame_transform (USD↔map 좌표/yaw)

**Files:**
- Create: `src/parking_robot_system/parking_robot_system/frame_transform.py`
- Test: `src/parking_robot_system/test/test_frame_transform.py`

**Interfaces:**
- Produces: `usd_to_map(x_usd, z_usd) -> (x_map, y_map)`, `map_to_usd(x_map, y_map) -> (x_usd, z_usd)`, `usd_yaw_to_map_deg(yaw_usd_deg) -> yaw_map_deg`, `map_to_usd_yaw_deg(yaw_map_deg) -> yaw_usd_deg`.

- [ ] **Step 1: 실패 테스트 작성** — `test/test_frame_transform.py`:
```python
from parking_robot_system.frame_transform import (
    usd_to_map, map_to_usd, usd_yaw_to_map_deg, map_to_usd_yaw_deg)


def test_position_reflection():
    assert usd_to_map(-8.5, 7.8) == (-8.5, -7.8)     # A2
    assert map_to_usd(-8.5, -7.8) == (-8.5, 7.8)


def test_position_roundtrip():
    for x, z in [(-8.5, 7.8), (1.7, -7.8), (0.0, 0.0)]:
        assert map_to_usd(*usd_to_map(x, z)) == (x, z)


def test_yaw_reflection_and_slots():
    # y=-z 반사 → 회전 부호 반전. 180/0 은 불변.
    assert usd_yaw_to_map_deg(180.0) % 360 == 180.0
    assert usd_yaw_to_map_deg(0.0) % 360 == 0.0
    assert usd_yaw_to_map_deg(90.0) % 360 == 270.0
    assert map_to_usd_yaw_deg(usd_yaw_to_map_deg(37.0)) % 360 == 37.0 % 360
```

- [ ] **Step 2: 실패 확인**

Run: `cd src/parking_robot_system && python3 -m pytest test/test_frame_transform.py -v`
Expected: FAIL (ModuleNotFoundError: frame_transform)

- [ ] **Step 3: 구현** — `parking_robot_system/frame_transform.py`:
```python
"""USD(XZ,+Y상방) ↔ ROS map(XY) 좌표/yaw 변환. 규약: x_map=x_usd, y_map=-z_usd."""


def usd_to_map(x_usd, z_usd):
    return (x_usd, -z_usd)


def map_to_usd(x_map, y_map):
    return (x_map, -y_map)


def usd_yaw_to_map_deg(yaw_usd_deg):
    # y축 반사는 회전 방향을 뒤집는다 → yaw 부호 반전.
    return -yaw_usd_deg


def map_to_usd_yaw_deg(yaw_map_deg):
    return -yaw_map_deg
```

- [ ] **Step 4: 통과 확인**

Run: `python3 -m pytest test/test_frame_transform.py -v`
Expected: PASS (4개)

- [ ] **Step 5: Commit**
```bash
git add src/parking_robot_system/parking_robot_system/frame_transform.py src/parking_robot_system/test/test_frame_transform.py
git commit -m "feat: frame_transform USD↔map 변환 (P1)"
```

---

## Task 3: slot_geometry (slot_id → 슬롯 중심·목표 yaw)

**Files:**
- Create: `src/parking_robot_system/parking_robot_system/slot_geometry.py`
- Test: `src/parking_robot_system/test/test_slot_geometry.py`

**Interfaces:**
- Consumes: (없음)
- Produces: `parse_slot(slot_id) -> (row, index) | None`, `slot_center_usd(slot_id) -> (x_usd, z_usd) | None`, `slot_target_yaw_usd_deg(slot_id) -> float | None`, `is_accessible(slot_id) -> bool`. 상수 `HALF_W=17.0`, `SPACE_WIDTH=3.4`, `ROW_CENTER=7.8`, `PARKING_INDICES=range(1,9)`, `ACCESSIBLE={'A1','A2'}`.

- [ ] **Step 1: 실패 테스트 작성** — `test/test_slot_geometry.py`:
```python
from parking_robot_system.slot_geometry import (
    parse_slot, slot_center_usd, slot_target_yaw_usd_deg, is_accessible)


def test_parse_valid():
    assert parse_slot("A2") == ("A", 2)
    assert parse_slot("B8") == ("B", 8)


def test_parse_invalid():
    for bad in ["A0", "A9", "C1", "A", "", "AA", "b3", "A2 "]:
        assert parse_slot(bad) is None


def test_center_matches_build_script():
    # A2: x=-17+2.5*3.4=-8.5, z(A)=+7.8
    assert slot_center_usd("A2") == (-8.5, 7.8)
    # B3: x=-17+3.5*3.4=-5.1, z(B)=-7.8
    assert slot_center_usd("B3") == (-5.1, -7.8)


def test_target_yaw():
    assert slot_target_yaw_usd_deg("A2") == 180.0
    assert slot_target_yaw_usd_deg("B3") == 0.0


def test_accessible():
    assert is_accessible("A1") and is_accessible("A2")
    assert not is_accessible("A3") and not is_accessible("B1")
```

- [ ] **Step 2: 실패 확인**

Run: `python3 -m pytest test/test_slot_geometry.py -v`
Expected: FAIL (ModuleNotFoundError)

- [ ] **Step 3: 구현** — `parking_robot_system/slot_geometry.py`:
```python
"""slot_id('A1'~'B8') → USD 슬롯 중심·목표 yaw. build_parking_environment.py 규약과 동일."""

HALF_W = 17.0
SPACE_WIDTH = 3.4
ROW_CENTER = 7.8
PARKING_INDICES = range(1, 9)
ACCESSIBLE = {"A1", "A2"}


def parse_slot(slot_id):
    if not isinstance(slot_id, str) or len(slot_id) < 2:
        return None
    row, num = slot_id[0], slot_id[1:]
    if row not in ("A", "B") or not num.isdigit():
        return None
    index = int(num)
    if index not in PARKING_INDICES:
        return None
    return (row, index)


def slot_center_usd(slot_id):
    parsed = parse_slot(slot_id)
    if parsed is None:
        return None
    row, index = parsed
    x = -HALF_W + (index + 0.5) * SPACE_WIDTH
    z = ROW_CENTER if row == "A" else -ROW_CENTER
    return (round(x, 3), round(z, 3))


def slot_target_yaw_usd_deg(slot_id):
    parsed = parse_slot(slot_id)
    if parsed is None:
        return None
    return 180.0 if parsed[0] == "A" else 0.0


def is_accessible(slot_id):
    return slot_id in ACCESSIBLE
```

- [ ] **Step 4: 통과 확인**

Run: `python3 -m pytest test/test_slot_geometry.py -v`
Expected: PASS (5개)

- [ ] **Step 5: Commit**
```bash
git add src/parking_robot_system/parking_robot_system/slot_geometry.py src/parking_robot_system/test/test_slot_geometry.py
git commit -m "feat: slot_geometry slot_id→중심·목표yaw (P1)"
```

---

## Task 4: occupancy (차량 위치 → 점유 슬롯)

**Files:**
- Create: `src/parking_robot_system/parking_robot_system/occupancy.py`
- Test: `src/parking_robot_system/test/test_occupancy.py`

**Interfaces:**
- Consumes: `slot_geometry.slot_center_usd`
- Produces: `slot_occupied(slot_id, vehicle_positions_usd, *, exclude_xz=None) -> bool` — `vehicle_positions_usd`는 `[(x,z), ...]`. 차량 중심이 슬롯 중심의 ±(SPACE_WIDTH/2, SPACE_LENGTH/2) 박스 안이면 점유. `exclude_xz`는 운반 중인 차량(제외) 좌표.

- [ ] **Step 1: 실패 테스트 작성** — `test/test_occupancy.py`:
```python
from parking_robot_system.occupancy import slot_occupied


def test_vehicle_at_center_occupies():
    assert slot_occupied("A2", [(-8.5, 7.8)]) is True


def test_empty_slot():
    assert slot_occupied("A2", [(5.1, 7.8)]) is False  # A6 위치 차량


def test_tolerance_box():
    # 슬롯 반폭 1.7, 반길이 3.3 안/밖 경계
    assert slot_occupied("A2", [(-8.5 + 1.6, 7.8 + 3.2)]) is True
    assert slot_occupied("A2", [(-8.5 + 1.8, 7.8)]) is False


def test_exclude_carried_vehicle():
    assert slot_occupied("A2", [(-8.5, 7.8)], exclude_xz=(-8.5, 7.8)) is False


def test_unknown_slot_is_false():
    assert slot_occupied("Z9", [(-8.5, 7.8)]) is False
```

- [ ] **Step 2: 실패 확인**

Run: `python3 -m pytest test/test_occupancy.py -v`
Expected: FAIL (ModuleNotFoundError)

- [ ] **Step 3: 구현** — `parking_robot_system/occupancy.py`:
```python
"""차량 실제 위치(USD XZ)로 슬롯 점유를 기하학적으로 판정."""

from parking_robot_system.slot_geometry import slot_center_usd, SPACE_WIDTH

HALF_LEN = 6.6 / 2.0
HALF_WID = SPACE_WIDTH / 2.0


def slot_occupied(slot_id, vehicle_positions_usd, *, exclude_xz=None):
    center = slot_center_usd(slot_id)
    if center is None:
        return False
    cx, cz = center
    for vx, vz in vehicle_positions_usd:
        if exclude_xz is not None and abs(vx - exclude_xz[0]) < 1e-6 and abs(vz - exclude_xz[1]) < 1e-6:
            continue
        if abs(vx - cx) <= HALF_WID and abs(vz - cz) <= HALF_LEN:
            return True
    return False
```

- [ ] **Step 4: 통과 확인**

Run: `python3 -m pytest test/test_occupancy.py -v`
Expected: PASS (5개)

- [ ] **Step 5: Commit**
```bash
git add src/parking_robot_system/parking_robot_system/occupancy.py src/parking_robot_system/test/test_occupancy.py
git commit -m "feat: occupancy 차량위치→슬롯점유 판정 (P1)"
```

---

## Task 5: Isaac runner — /parking_slots 점유 발행

**Files:**
- Modify: `isaacpjt/Isaac_envo/dock_lift_handoff_runner.py` (import부 + 노드 셋업부 + 발행 루프)

**Interfaces:**
- Consumes: (스테이지 prim 위치)
- Produces: 토픽 `/parking_slots` (`std_msgs/String`, JSON 배열). 각 원소:
  `{"slot_id","occupied","is_accessible","x","y","yaw_deg"}` (x,y=map 프레임, yaw_deg=목표 방향).

> **주의:** runner는 Isaac 내부 rclpy. 순수 로직은 Task 3/4에 있으나 runner는 그 모듈을
> import할 수 없다(다른 파이썬 환경). 따라서 runner 안에 **동일 상수·공식을 자립적으로** 둔다
> (Global Constraints의 값과 일치해야 함 — 불일치 시 점유 오판).

- [ ] **Step 1: 발행 헬퍼 추가** — runner 상단(상수부 근처)에 슬롯 테이블과 점유 계산 함수를 자립적으로 정의:
```python
# --- /parking_slots 발행용 (parking_robot_system.slot_geometry/occupancy와 값 동일) ---
_HALF_W, _SPACE_W, _ROW_C = 17.0, 3.4, 7.8
_HALF_LEN, _HALF_WID = 3.3, 1.7
_ACCESSIBLE = {"A1", "A2"}


def _all_slots_usd():
    slots = {}
    for row, zc in (("A", _ROW_C), ("B", -_ROW_C)):
        for i in range(1, 9):
            sid = f"{row}{i}"
            slots[sid] = (-_HALF_W + (i + 0.5) * _SPACE_W, zc, 180.0 if row == "A" else 0.0)
    return slots


def _vehicle_world_positions(stage):
    """모든 주차 차량 + 운반 대상(Pickup)의 world (x,z)."""
    import omni.usd
    from pxr import UsdGeom
    positions = []
    for root in ("/World/ParkingVehicles", "/World/VehicleAsset"):
        prim = stage.GetPrimAtPath(root)
        if not prim or not prim.IsValid():
            continue
        for child in prim.GetAllChildren():
            for v in ([child] + list(child.GetAllChildren())):
                if not v.IsA(UsdGeom.Xformable):
                    continue
                m = UsdGeom.Xformable(v).ComputeLocalToWorldTransform(0)
                t = m.ExtractTranslation()
                positions.append((float(t[0]), float(t[2])))
    return positions
```

- [ ] **Step 2: 발행자 생성** — runner의 ROS 노드 셋업부(`node.create_publisher(...odom...)` 인근)에 추가:
```python
from std_msgs.msg import String as RosString
slots_pub = node.create_publisher(RosString, "/parking_slots", 10)
```

- [ ] **Step 3: 주기 발행** — runner의 메인 루프(`app.update()`가 도는 스텝 루프, odom 발행 지점 인근)에 ~2Hz로 추가:
```python
import json
_SLOT_TABLE = _all_slots_usd()
# (루프 안, 매 N틱)
positions = _vehicle_world_positions(stage)
arr = []
for sid, (sx, sz, yaw) in _SLOT_TABLE.items():
    occ = any(abs(vx - sx) <= _HALF_WID and abs(vz - sz) <= _HALF_LEN for vx, vz in positions)
    arr.append({"slot_id": sid, "occupied": occ, "is_accessible": sid in _ACCESSIBLE,
                "x": round(sx, 3), "y": round(-sz, 3), "yaw_deg": yaw})
msg = RosString(); msg.data = json.dumps(arr); slots_pub.publish(msg)
```

- [ ] **Step 4: 스모크 검증** — runner를 헤드리스로 띄우고 토픽 확인.

Run (터미널 A):
```bash
cd /home/rokey/p3/Rokey_proj_03-Isaac-Sim-/isaacpjt/Isaac_envo && bash dock_lift_handoff_runner.sh --headless-test
```
Run (터미널 B, ROS 환경):
```bash
source /opt/ros/humble/setup.bash; export ROS_DOMAIN_ID=126
unset FASTRTPS_DEFAULT_PROFILES_FILE FASTDDS_DEFAULT_PROFILES_FILE
ros2 topic echo --once /parking_slots
```
Expected: JSON 배열 출력. `A3/A5/A6/B3`는 `occupied:true`, `A1/A2`는 `occupied:false`.

- [ ] **Step 5: Commit**
```bash
git add isaacpjt/Isaac_envo/dock_lift_handoff_runner.py
git commit -m "feat(runner): /parking_slots 점유 발행 (P1)"
```

---

## Task 6: parking_slot_manager — GetSlotInfo 서비스

**Files:**
- Modify: `src/parking_robot_system/parking_robot_system/parking_slot_manager.py`
- Test: `src/parking_robot_system/test/test_slot_manager.py`

**Interfaces:**
- Consumes: 토픽 `/parking_slots`(std_msgs/String JSON), `slot_geometry.parse_slot`
- Produces: 서비스 `get_slot_info`(GetSlotInfo). 캐시 헬퍼 `SlotCache.update_from_json(s)`, `SlotCache.query(slot_id) -> dict|None`, `SlotCache.ready -> bool` (테스트가 rclpy 없이 검증).

- [ ] **Step 1: 실패 테스트 작성 (순수 캐시 로직)** — `test/test_slot_manager.py`:
```python
import json
from parking_robot_system.parking_slot_manager import SlotCache


def test_empty_cache_not_ready():
    c = SlotCache()
    assert c.ready is False
    assert c.query("A2") is None


def test_update_and_query():
    c = SlotCache()
    c.update_from_json(json.dumps([
        {"slot_id": "A2", "occupied": False, "is_accessible": True, "x": -8.5, "y": -7.8, "yaw_deg": 180.0},
        {"slot_id": "A3", "occupied": True, "is_accessible": False, "x": -5.1, "y": -7.8, "yaw_deg": 180.0},
    ]))
    assert c.ready is True
    assert c.query("A2")["occupied"] is False
    assert c.query("A3")["occupied"] is True
    assert c.query("B9") is None
```

- [ ] **Step 2: 실패 확인**

Run: `python3 -m pytest test/test_slot_manager.py -v`
Expected: FAIL (ImportError: SlotCache)

- [ ] **Step 3: 구현** — `parking_slot_manager.py` 전체 교체:
```python
#!/usr/bin/env python3
"""parking_slot_manager: /parking_slots 구독 캐시 + get_slot_info 서비스."""
import json

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Pose
import math

from parking_robot_interfaces.srv import GetSlotInfo


class SlotCache:
    def __init__(self):
        self._slots = {}

    @property
    def ready(self):
        return len(self._slots) > 0

    def update_from_json(self, s):
        arr = json.loads(s)
        self._slots = {d["slot_id"]: d for d in arr}

    def query(self, slot_id):
        return self._slots.get(slot_id)


def _yaw_deg_to_pose(x, y, yaw_deg):
    p = Pose()
    p.position.x, p.position.y = float(x), float(y)
    half = math.radians(yaw_deg) / 2.0
    p.orientation.z, p.orientation.w = math.sin(half), math.cos(half)
    return p


class ParkingSlotManagerNode(Node):
    def __init__(self):
        super().__init__('parking_slot_manager')
        self._cache = SlotCache()
        self.create_subscription(String, '/parking_slots', self._on_slots, 10)
        self.create_service(GetSlotInfo, 'get_slot_info', self._on_get_slot_info)
        self.get_logger().info('parking_slot_manager node started')

    def _on_slots(self, msg):
        try:
            self._cache.update_from_json(msg.data)
        except (ValueError, KeyError) as e:
            self.get_logger().warn(f'/parking_slots 파싱 실패: {e}')

    def _on_get_slot_info(self, request, response):
        response.data_ready = self._cache.ready
        info = self._cache.query(request.slot_id)
        response.exists = info is not None
        if info is not None:
            response.occupied = bool(info["occupied"])
            response.is_accessible = bool(info["is_accessible"])
            response.pose = _yaw_deg_to_pose(info["x"], info["y"], info["yaw_deg"])
        return response


def main(args=None):
    rclpy.init(args=args)
    node = ParkingSlotManagerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

- [ ] **Step 4: 통과 확인**

Run: `python3 -m pytest test/test_slot_manager.py -v`
Expected: PASS (2개)

- [ ] **Step 5: Commit**
```bash
git add src/parking_robot_system/parking_robot_system/parking_slot_manager.py src/parking_robot_system/test/test_slot_manager.py
git commit -m "feat(slot_manager): /parking_slots 캐시 + get_slot_info (P1)"
```

---

## Task 7: task_dispatcher — 검증·분기·작업 전송

**Files:**
- Modify: `src/parking_robot_system/parking_robot_system/task_dispatcher.py`
- Test: `src/parking_robot_system/test/test_dispatcher.py`

**Interfaces:**
- Consumes: `get_slot_info`(GetSlotInfo 클라이언트), `execute_parking_task`(ExecuteParkingTask 액션 클라이언트)
- Produces: 서비스 `/dispatch/park_in_slot`(ParkInSlot). 순수 결정 함수 `decide(info: dict|None, data_ready: bool) -> (accepted: bool, message: str)` (테스트 대상).

- [ ] **Step 1: 실패 테스트 작성 (순수 결정 로직)** — `test/test_dispatcher.py`:
```python
from parking_robot_system.task_dispatcher import decide


def test_no_data():
    assert decide(None, data_ready=False) == (False, "관제 데이터 없음(재시도)")


def test_nonexistent():
    assert decide(None, data_ready=True) == (False, "존재하지 않는 구역")


def test_occupied():
    info = {"exists": True, "occupied": True}
    assert decide(info, data_ready=True) == (False, "해당 구역에 차량이 있어 주차 불가")


def test_empty_accepted():
    info = {"exists": True, "occupied": False}
    accepted, msg = decide(info, data_ready=True)
    assert accepted is True
```

- [ ] **Step 2: 실패 확인**

Run: `python3 -m pytest test/test_dispatcher.py -v`
Expected: FAIL (ImportError: decide)

- [ ] **Step 3: 구현** — `task_dispatcher.py` 전체 교체:
```python
#!/usr/bin/env python3
"""task_dispatcher: /dispatch/park_in_slot 검증 후 execute_parking_task 전송."""
import uuid

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from parking_robot_interfaces.action import ExecuteParkingTask
from parking_robot_interfaces.srv import GetSlotInfo, ParkInSlot


def decide(info, data_ready):
    """(accepted, message). info: get_slot_info 결과 dict 또는 None."""
    if not data_ready:
        return (False, "관제 데이터 없음(재시도)")
    if info is None or not info.get("exists", False):
        return (False, "존재하지 않는 구역")
    if info.get("occupied", False):
        return (False, "해당 구역에 차량이 있어 주차 불가")
    return (True, "접수됨")


class TaskDispatcherNode(Node):
    def __init__(self):
        super().__init__('task_dispatcher')
        self._slot_client = self.create_client(GetSlotInfo, 'get_slot_info')
        self._exec_client = ActionClient(self, ExecuteParkingTask, 'execute_parking_task')
        self.create_service(ParkInSlot, '/dispatch/park_in_slot', self._on_dispatch)
        self.get_logger().info('task_dispatcher node started')

    def _on_dispatch(self, request, response):
        if not self._slot_client.wait_for_service(timeout_sec=3.0):
            response.accepted, response.message = False, "관제 데이터 없음(재시도)"
            return response
        fut = self._slot_client.call_async(GetSlotInfo.Request(slot_id=request.slot_id))
        rclpy.spin_until_future_complete(self, fut, timeout_sec=5.0)
        res = fut.result()
        info = None
        if res is not None and res.exists:
            info = {"exists": True, "occupied": res.occupied,
                    "is_accessible": res.is_accessible, "pose": res.pose}
        data_ready = res is not None and res.data_ready
        accepted, message = decide(info, data_ready)
        response.accepted, response.message = accepted, message
        if accepted:
            task_id = str(uuid.uuid4())
            response.task_id = task_id
            goal = ExecuteParkingTask.Goal()
            goal.task_id, goal.request_type, goal.vehicle_id = task_id, "ENTRY", "Pickup"
            goal.slot_id, goal.slot_pose = request.slot_id, res.pose
            goal.leader_robot_id, goal.follower_robot_id = "robot_rear", "robot_front"
            self._exec_client.wait_for_server()
            self._exec_client.send_goal_async(goal)
        return response


def main(args=None):
    rclpy.init(args=args)
    node = TaskDispatcherNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

- [ ] **Step 4: 통과 확인**

Run: `python3 -m pytest test/test_dispatcher.py -v`
Expected: PASS (4개)

- [ ] **Step 5: Commit**
```bash
git add src/parking_robot_system/parking_robot_system/task_dispatcher.py src/parking_robot_system/test/test_dispatcher.py
git commit -m "feat(dispatcher): 검증·분기 + execute_parking_task 전송 (P1)"
```

---

## Task 8: user_request_gateway — /park_in_slot 프록시

**Files:**
- Modify: `src/parking_robot_system/parking_robot_system/user_request_gateway.py`
- Test: `src/parking_robot_system/test/test_gateway.py`

**Interfaces:**
- Consumes: `/dispatch/park_in_slot`(ParkInSlot 클라이언트)
- Produces: 서비스 `/park_in_slot`(ParkInSlot). 순수 헬퍼 `normalize_slot_id(raw) -> str` (공백 제거·대문자화).

- [ ] **Step 1: 실패 테스트 작성** — `test/test_gateway.py`:
```python
from parking_robot_system.user_request_gateway import normalize_slot_id


def test_normalize():
    assert normalize_slot_id(" a2 ") == "A2"
    assert normalize_slot_id("b8") == "B8"
    assert normalize_slot_id("A2") == "A2"
```

- [ ] **Step 2: 실패 확인**

Run: `python3 -m pytest test/test_gateway.py -v`
Expected: FAIL (ImportError)

- [ ] **Step 3: 구현** — `user_request_gateway.py` 전체 교체:
```python
#!/usr/bin/env python3
"""user_request_gateway: /park_in_slot 사용자 대면 → /dispatch/park_in_slot 프록시."""
import rclpy
from rclpy.node import Node

from parking_robot_interfaces.srv import ParkInSlot


def normalize_slot_id(raw):
    return (raw or "").strip().upper()


class UserRequestGatewayNode(Node):
    def __init__(self):
        super().__init__('user_request_gateway')
        self._dispatch = self.create_client(ParkInSlot, '/dispatch/park_in_slot')
        self.create_service(ParkInSlot, '/park_in_slot', self._on_park)
        self.get_logger().info('user_request_gateway node started')

    def _on_park(self, request, response):
        slot_id = normalize_slot_id(request.slot_id)
        if not self._dispatch.wait_for_service(timeout_sec=3.0):
            response.accepted, response.message = False, "관제(dispatcher) 미기동"
            return response
        fut = self._dispatch.call_async(ParkInSlot.Request(slot_id=slot_id))
        rclpy.spin_until_future_complete(self, fut, timeout_sec=10.0)
        res = fut.result()
        if res is None:
            response.accepted, response.message = False, "dispatcher 응답 없음"
        else:
            response.accepted, response.task_id, response.message = res.accepted, res.task_id, res.message
        return response


def main(args=None):
    rclpy.init(args=args)
    node = UserRequestGatewayNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

- [ ] **Step 4: 통과 확인**

Run: `python3 -m pytest test/test_gateway.py -v`
Expected: PASS

- [ ] **Step 5: Commit**
```bash
git add src/parking_robot_system/parking_robot_system/user_request_gateway.py src/parking_robot_system/test/test_gateway.py
git commit -m "feat(gateway): /park_in_slot 프록시 (P1)"
```

---

## Task 9: formation_driver — 모션 프리미티브 추출

**Files:**
- Create: `src/parking_robot_system/parking_robot_system/formation_driver.py`
- Test: `src/parking_robot_system/test/test_formation_driver.py`

**Interfaces:**
- Consumes: `dock_lift_handoff_mission.py`의 제어 상수·수식(복사)
- Produces: 순수 함수 `body_twist_from_world_error(ex, ez, yaw) -> (fwd, left)` (odom 규약 역행렬), `wrap(a) -> float`. 클래스 `FormationDriver`(pose provider + cmd publisher 주입)로 `goto_xz`, `rotate_to`, `ingress_to`, `carry_step` 제공 — 단, **순수 기하 함수만 단위 테스트**하고 폐루프는 Isaac 스모크로 검증.

> 원본 `dock_lift_handoff_mission.py`는 보존한다. 여기서는 `_omni_step`의 body twist 변환과
> `wrap`을 순수 함수로 분리해 재사용·테스트한다. 폐루프 메서드는 이 순수 함수를 호출한다.

- [ ] **Step 1: 실패 테스트 작성** — `test/test_formation_driver.py`:
```python
import math
from parking_robot_system.formation_driver import body_twist_from_world_error, wrap


def test_wrap():
    assert abs(wrap(math.pi * 3)) - math.pi < 1e-9
    assert wrap(0.0) == 0.0


def test_forward_when_facing_plus_x():
    # yaw=0: forward_world=(cos0,-sin0)=(+x). world +x 오차 → 순수 전진.
    fwd, left = body_twist_from_world_error(1.0, 0.0, 0.0)
    assert fwd > 0.9 and abs(left) < 1e-9


def test_strafe_axis():
    # yaw=0: +vy(left) = world -z. world -z 오차(ez=-1) → +left.
    fwd, left = body_twist_from_world_error(0.0, -1.0, 0.0)
    assert abs(fwd) < 1e-9 and left > 0.9
```

- [ ] **Step 2: 실패 확인**

Run: `python3 -m pytest test/test_formation_driver.py -v`
Expected: FAIL (ImportError)

- [ ] **Step 3: 구현** — `formation_driver.py` (순수 함수 + 폐루프 스켈레톤). 제어 상수는
  `dock_lift_handoff_mission.py` 상단에서 **그대로 복사**(`K_LIN, K_STRAFE, K_YAW, MAX_LIN,
  MAX_YAW, INGRESS_SPEED, CONTROL_HZ, POS_TOL, YAW_TOL`):
```python
"""편대 모션 프리미티브. dock_lift_handoff_mission.py에서 순수 기하부를 추출."""
import math

# dock_lift_handoff_mission.py 와 동일 값 (실측 대조 완료).
K_LIN, MAX_LIN = 0.8, 0.6
K_STRAFE = 0.8
K_YAW, MAX_YAW = 0.5, 0.15   # 회전 느리게(발레 방지)
INGRESS_SPEED = 0.30
CARRY_SPEED = 0.30
CONTROL_HZ = 20.0
POS_TOL = 0.10
YAW_TOL = math.radians(4.0)


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def body_twist_from_world_error(ex, ez, yaw):
    """world 오차(ex,ez) → body (fwd=vx, left=vy). odom 규약 역행렬."""
    c, s = math.cos(yaw), math.sin(yaw)
    fwd = ex * c - ez * s
    left = -(ex * s + ez * c)
    return (fwd, left)


def clamp(v, m):
    return max(-m, min(m, v))
```
> **주의:** 위 상수는 `dock_lift_handoff_mission.py:36-40`과 대조 완료한 실제 값이다. 원본이 바뀌면 함께 갱신할 것(값이 다르면 모션이 달라진다).

- [ ] **Step 4: 통과 확인**

Run: `python3 -m pytest test/test_formation_driver.py -v`
Expected: PASS (3개)

- [ ] **Step 5: Commit**
```bash
git add src/parking_robot_system/parking_robot_system/formation_driver.py src/parking_robot_system/test/test_formation_driver.py
git commit -m "feat: formation_driver 순수 기하 프리미티브 추출 (P1)"
```

---

## Task 10: 액션서버 3종 — Isaac 모션 브리지

**Files:**
- Modify: `navigate_action_server.py`, `align_action_server.py`, `lift_action_server.py`, `vehicle_detection_node.py`

**Interfaces:**
- Consumes: `formation_driver`, `/robot_*/odom`, `/vehicle/pose`, `/robot_*/cmd_vel`, `/robot_*/arm_control`
- Produces: 액션서버 `navigate_to_pose`(NavigateToPose), `align_vehicle`(AlignVehicle), `control_lift`(ControlLift), `detect_vehicle`(DetectVehicle). 각 서버는 odom/vehicle 구독 + cmd_vel/arm 발행을 자체 보유.

> 각 서버는 `dock_lift_handoff_mission.py`의 해당 폐루프(navigate=`_approach_parallel`/편대 운반,
> align=`_ingress_to`, lift=`_call_arms`+`_grip_lift`)를 이식한다. **로직 변경 없이 이식**하고
> goal 파라미터(목표 pose/command)만 외부에서 주입되도록 바꾼다. 아래는 lift 예시(가장 짧음);
> navigate/align은 같은 패턴으로 각 폐루프를 감싼다.

- [ ] **Step 1: control_lift 구현** — `lift_action_server.py` `_on_control_lift` 교체:
```python
# command: "UP" → arm_control(True) + 리프트 확인; "DOWN" → arm_control(False)
def _on_control_lift(self, goal_handle):
    opening = goal_handle.request.command == "UP"
    ok = self._call_arms(opening)   # 이식: dock_lift_handoff_mission._call_arms
    result = ControlLift.Result()
    result.success = ok
    result.support_state = "SUPPORTED" if (opening and ok) else "RELEASED"
    goal_handle.succeed()
    return result
```
(노드 `__init__`에 `self.arm = {r: self.create_client(SetBool, f"/{r}/arm_control") for r in ("robot_rear","robot_front")}` 추가, `_call_arms`는 원본에서 이식.)

- [ ] **Step 2: navigate_to_pose 구현** — `navigate_action_server.py`: goal의 `pose`(map)를
  `map_to_usd`로 변환 후 `formation_driver`로 두 로봇을 목표로 편대 이동. 픽업 전/후 모드는 goal의
  `behavior_tree` 문자열 필드로 구분("approach"|"carry")하거나 별도 파라미터로 전달.
  (상세 폐루프는 `_approach_parallel`/편대 운반 이식.)

- [ ] **Step 3: align_vehicle 구현** — `align_action_server.py`: goal `target_pose`(차량 축)로
  `_ingress_to` 이식 실행, `final_error` 반환.

- [ ] **Step 4: detect_vehicle 스텁** — `vehicle_detection_node.py`: `_on_detect`가 알려진 Pickup
  좌표(`VehicleInfo.pose`)를 채워 `success=True` 반환.

- [ ] **Step 5: 빌드 + import 스모크**

Run:
```bash
cd /home/rokey/p3/Rokey_proj_03-Isaac-Sim- && source /opt/ros/humble/setup.bash && \
colcon build --packages-select parking_robot_system parking_robot_interfaces && \
source install/setup.bash && \
ros2 pkg executables parking_robot_system
```
Expected: 9개 실행 스크립트 목록 출력(에러 없음).

- [ ] **Step 6: Commit**
```bash
git add src/parking_robot_system/parking_robot_system/*.py
git commit -m "feat(action-servers): navigate/align/lift/detect Isaac 브리지 (P1)"
```

---

## Task 11: robot_task_orchestrator — 주차 상태머신

**Files:**
- Modify: `src/parking_robot_system/parking_robot_system/robot_task_orchestrator.py`
- Test: `src/parking_robot_system/test/test_orchestrator.py`

**Interfaces:**
- Consumes: `detect_vehicle`, `navigate_to_pose`, `align_vehicle`, `control_lift` 액션 클라이언트, goal의 `slot_id`/`slot_pose`
- Produces: 액션서버 `execute_parking_task`, 토픽 `task_state`. 순수 함수 `next_state(current) -> str` (전이 테이블), `plan_steps(slot_pose) -> list[str]`.

- [ ] **Step 1: 실패 테스트 작성 (전이 테이블)** — `test/test_orchestrator.py`:
```python
from parking_robot_system.robot_task_orchestrator import next_state, TRANSITIONS


def test_full_sequence():
    seq = ["SEARCHING", "APPROACHING", "PICKED_UP", "MOVING", "ARRIVED",
           "PARKED", "RETURNING", "DONE"]
    for a, b in zip(seq, seq[1:]):
        assert next_state(a) == b


def test_terminal():
    assert next_state("DONE") == "DONE"
```

- [ ] **Step 2: 실패 확인**

Run: `python3 -m pytest test/test_orchestrator.py -v`
Expected: FAIL (ImportError)

- [ ] **Step 3: 구현** — `robot_task_orchestrator.py`에 전이 테이블 + 상태머신 실행부.
  전이 테이블(순수):
```python
TRANSITIONS = {
    "SEARCHING": "APPROACHING", "APPROACHING": "PICKED_UP", "PICKED_UP": "MOVING",
    "MOVING": "ARRIVED", "ARRIVED": "PARKED", "PARKED": "RETURNING",
    "RETURNING": "DONE", "DONE": "DONE",
}


def next_state(current):
    return TRANSITIONS.get(current, "FAILED")
```
  `_on_execute_parking_task`는 각 상태에서 해당 액션 클라이언트를 호출하고 성공 시 `next_state`로
  진행하며 매 전이마다 `task_state` 발행. ARRIVED→PARKED 사이에 goal `slot_pose`의 목표 yaw로
  편대를 정렬(navigate의 회전 모드). 실패 시 상태 FAILED + `result.success=False`.

- [ ] **Step 4: 통과 확인**

Run: `python3 -m pytest test/test_orchestrator.py -v`
Expected: PASS (2개)

- [ ] **Step 5: Commit**
```bash
git add src/parking_robot_system/parking_robot_system/robot_task_orchestrator.py src/parking_robot_system/test/test_orchestrator.py
git commit -m "feat(orchestrator): 주차 상태머신 + task_state (P1)"
```

---

## Task 12: 통합 — end-to-end 주차 동작 검증

**Files:**
- Modify: `src/parking_robot_system/launch/parking_robot_system.launch.py` (필요시 파라미터)
- Create: `docs/runbook-park-in-slot.md` (실행 순서)

**Interfaces:**
- Consumes: 전체 노드 + Isaac runner

- [ ] **Step 1: 런북 작성** — `docs/runbook-park-in-slot.md`: 4터미널 순서
  (① runner `--gui` → ② `ros2 launch parking_robot_system parking_robot_system.launch.py`
  → ③ 점유 확인 `ros2 topic echo --once /parking_slots` → ④ `ros2 service call /park_in_slot ...`).
  모든 터미널 ROS 환경(도메인 126, 프로파일 unset) 명시.

- [ ] **Step 2: 점유 구역 거부 검증**

Run (터미널 4):
```bash
source /opt/ros/humble/setup.bash; source install/setup.bash; export ROS_DOMAIN_ID=126
unset FASTRTPS_DEFAULT_PROFILES_FILE FASTDDS_DEFAULT_PROFILES_FILE
ros2 service call /park_in_slot parking_robot_interfaces/srv/ParkInSlot "{slot_id: 'A3'}"
```
Expected: `accepted: false, message: "해당 구역에 차량이 있어 주차 불가"` (로봇 미동작).

- [ ] **Step 3: 없는 구역 거부 검증**

Run: `ros2 service call /park_in_slot parking_robot_interfaces/srv/ParkInSlot "{slot_id: 'C1'}"`
Expected: `accepted: false, message: "존재하지 않는 구역"`.

- [ ] **Step 4: 빈 구역 주차 검증 (A2)**

Run: `ros2 service call /park_in_slot parking_robot_interfaces/srv/ParkInSlot "{slot_id: 'A2'}"`
그리고 별도 터미널: `ros2 topic echo /task_state`
Expected: `accepted: true` + task_state가 SEARCHING…PARKED…DONE 진행. GUI에서 로봇이 차량을
A2로 운반해 **A열 방향(180°)으로 안착** 후 West 도크 복귀. 이어 `/parking_slots`에서 A2 `occupied:true`.

- [ ] **Step 5: Commit**
```bash
git add docs/runbook-park-in-slot.md src/parking_robot_system/launch/parking_robot_system.launch.py
git commit -m "docs+test: end-to-end 지정구역 주차 런북·검증 (P1)"
```

---

## Self-Review (작성자 체크)

**Spec coverage:**
- 서비스 slot_id 입력 → Task 1(srv), 8(gateway). ✓
- 점유 진실원본=Isaac → Task 5(runner 발행), 6(캐시). ✓
- 없음/점유/비어있음 분기 → Task 7(decide). ✓
- 방향(A180/B0) → Task 3(target yaw), 5(발행 yaw), 11(정렬). ✓
- 파이프라인 관통 → Task 6~11. ✓
- 도크 복귀 → Task 11(RETURNING). ✓
- 좌표 변환 단일화 → Task 2. ✓

**Placeholder scan:** Task 9의 `K_LIN` 등 4상수는 "원본에서 실제 값 복사" 명시(자리표시 아님, 대조 필수). Task 10 navigate/align의 폐루프 이식은 원본 메서드를 지목(코드 위치 명시). 그 외 순수 로직은 완전 코드.

**Type consistency:** `decide(info, data_ready)`·`SlotCache.query`·`GetSlotInfo` 응답 필드(`data_ready/exists/occupied/is_accessible/pose`) 전 태스크 일치. `next_state`/`TRANSITIONS` 상태명은 `TaskState.msg` 값과 일치.

**주의(구현 시):** Task 9~11의 폐루프 모션은 Isaac 물리라 순수 단위테스트 불가 → Task 12 스모크로 검증. 편대 회전 제어 법칙(스펙 열린 질문)은 Task 11 구현 중 GUI 관찰로 (a)인플레이스 회전을 우선 시도, 미흡 시 (b)접근 경로 정렬로 전환.
