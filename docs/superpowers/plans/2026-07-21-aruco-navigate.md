# ArUco 폐루프 주행 (E2E Plan 2/4) Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 리더(robot_1)가 GT 없이 **전방 카메라의 ArUco 마커 측위 + 휠 데드레커닝**만으로
도크→인계 베이(H_B) 접근점까지 폐루프 주행하고, 존 통행권을 구간마다 실사용하며,
팔로워는 편대 호송한다. GT는 채점(검증)용으로만 쓴다.

**Architecture:** Plan 1의 듀얼 필드 러너에 카메라 브리지(C++ OmniGraph)와 휠 FK twist를
추가하고, 기존 `marker_localizer_node`(M4 검증)를 ros_map 프레임+yaw 출력으로 확장,
신규 `aruco_navigator_node`가 NavigateToPose 액션 서버로서 마커 fix + 데드레커닝 융합
pose 위에서 웨이포인트 P제어를 돈다(팀원 navigate 스켈레톤의 실물 대역 — sim_orchestrator
패턴). 임시 leg-mission 스크립트가 PathFinder 경로를 구간별 존 락과 함께 순차 전송한다.

**Tech Stack:** Plan 1과 동일 + `parkbot_aruco`(cv2 4.5.4), `nav2_msgs`(액션 정의).

## Global Constraints

- Plan 1의 Global Constraints 전부 계승 (GPU PhysX·좀비 정리·비파괴·robot_N 네임스페이스).
- **화이트리스트 금지**: 내부 rclpy 러너와 통신하는 모든 터미널에서
  `unset FASTRTPS_DEFAULT_PROFILES_FILE` (Plan 1 실측).
- formation 컨트롤러는 `__ns` 금지, per-topic remap만 (Plan 1 실측).
- 확정 좌표 규약: `ros_x=usd_x, ros_y=-usd_z`, cmd `angular.z`는 러너 경계에서 반전.
- 마커 좌표계(usd x,z; ψ=atan2(fwd_x,fwd_z)) → ros_map 변환: `ros_yaw = ψ − π/2`.
  **종이 유도이므로 Task 3에서 GT 대조 실측으로 확정** — 틀리면 그 한 곳만 고친다.
- "**외부 env**" = `source /opt/ros/humble/setup.bash; source install/setup.bash;
  export ROS_DOMAIN_ID=126 RMW_IMPLEMENTATION=rmw_fastrtps_cpp; unset FASTRTPS_DEFAULT_PROFILES_FILE`

## File Structure

- Modify: `isaacpjt/Isaac_envo/parking/run_dual_robot_ros2_field.py` — ① robot_1 전방
  카메라 브리지(OmniGraph) ② `/robot_N/wheel_twist` 발행(휠 FK)
- Modify: `isaacpjt/Isaac_envo/mecanum_drive.py` — `cmd_vel_from_wheel_velocities`(FK) 추가
- Modify: `src/parkbot_aruco/parkbot_aruco/marker_localizer_node.py` — `frame:=ros_map`
  모드 + orientation(yaw) 발행
- Create: `src/parkbot_aruco/parkbot_aruco/aruco_navigator_node.py` — NavigateToPose 서버
- Create: `src/parkbot_aruco/parkbot_aruco/pose_estimator.py` — 마커 fix + twist 데드레커닝
  융합 (순수 파이썬, 단위테스트 대상)
- Create: `src/parkbot_aruco/test/test_pose_estimator.py`
- Create: `isaacpjt/Isaac_envo/verify_leader_localization.py` — 정지/저속 측위 GT 대조
- Create: `isaacpjt/Isaac_envo/aruco_leg_mission.py` — 임시 미션: 경로 산출 + 구간 존 락
  + NavigateToPose 순차 전송 + 편대 배정
- Modify: `src/parkbot_aruco/setup.py` — `aruco_navigator` console_script 등록

---

### Task 0: 선결 — nav2_msgs 설치 (사용자 sudo 1회)

- [ ] **Step 1:** 사용자에게 요청:
```bash
sudo apt install -y ros-humble-nav2-msgs
```
- [ ] **Step 2: 확인**
```bash
source /opt/ros/humble/setup.bash && python3 -c "from nav2_msgs.action import NavigateToPose; print('OK')"
```
Expected: `OK`. (설치 불가 시 STOP — 동형 자체 액션으로 우회할지 사용자와 결정)

---

### Task 1: 러너에 robot_1 전방 카메라 브리지

**Files:**
- Modify: `isaacpjt/Isaac_envo/parking/run_dual_robot_ros2_field.py`

**Interfaces:**
- Produces: `/robot_1/front_cam/image_raw`(sensor_msgs/Image, rgb),
  `/robot_1/front_cam/camera_info` — Task 3의 localizer가 소비.

- [ ] **Step 1: 카메라 그래프 함수 추가** — 러너의 `_add_virtual_target` 아래에:

```python
CAM_RES = (640, 480)


def _add_front_camera_bridge(robot_id: str) -> None:
    """robot_N 전방 카메라를 C++ OmniGraph로 ROS 토픽에 발행한다.

    aruco_sim_bringup.py 검증 패턴 그대로: CameraHelper(rgb) + CameraInfoHelper
    (camera_info 는 CameraHelper 의 type 이 아니라 별도 노드 — 기존 함정).
    """
    import omni.graph.core as og
    import omni.replicator.core as rep
    import omni.usd

    cam_prim = (f"/World/Robots/{robot_id}/cam_front_link/depth_cam_front"
                "/Camera_Pseudo_Depth_Front")
    stage = omni.usd.get_context().get_stage()
    if not stage.GetPrimAtPath(cam_prim):
        raise RuntimeError(f"전방 카메라 프림이 없습니다: {cam_prim}")
    rp = rep.create.render_product(cam_prim, CAM_RES)
    graph_path = f"/World/CamGraph_{robot_id}"
    og.Controller.edit(
        {"graph_path": graph_path, "evaluator_name": "execution"},
        {
            og.Controller.Keys.CREATE_NODES: [
                ("OnTick", "omni.graph.action.OnPlaybackTick"),
                ("CamRgb", "isaacsim.ros2.bridge.ROS2CameraHelper"),
                ("CamInfo", "isaacsim.ros2.bridge.ROS2CameraInfoHelper"),
            ],
            og.Controller.Keys.SET_VALUES: [
                ("CamRgb.inputs:renderProductPath", rp.path),
                ("CamRgb.inputs:topicName", f"/{robot_id}/front_cam/image_raw"),
                ("CamRgb.inputs:type", "rgb"),
                ("CamRgb.inputs:frameId", f"{robot_id}/front_cam"),
                ("CamInfo.inputs:renderProductPath", rp.path),
                ("CamInfo.inputs:topicName", f"/{robot_id}/front_cam/camera_info"),
                ("CamInfo.inputs:frameId", f"{robot_id}/front_cam"),
            ],
            og.Controller.Keys.CONNECT: [
                ("OnTick.outputs:tick", "CamRgb.inputs:execIn"),
                ("OnTick.outputs:tick", "CamInfo.inputs:execIn"),
            ],
        },
    )
```

- [ ] **Step 2: 호출 삽입** — `enable_extension("isaacsim.ros2.bridge")` 이후,
`timeline.play()` **이전**에 (렌더 프로덕트는 스테이지 로드 후):

```python
        _add_front_camera_bridge("robot_1")
```

- [ ] **Step 3: 기동 + 토픽 검증** (러너 재기동 후 외부 env 터미널)

```bash
bash isaacpjt/Isaac_envo/parking/run_dual_robot_ros2_field.sh --headless --seconds 120 &
# READY 후:
timeout 10 ros2 topic hz /robot_1/front_cam/image_raw 2>&1 | tail -2
timeout 5 ros2 topic echo --once /robot_1/front_cam/camera_info --field k
```
Expected: hz ≥ 10, K 행렬 9개 값(0 아님).

- [ ] **Step 4: Commit**
```bash
git add isaacpjt/Isaac_envo/parking/run_dual_robot_ros2_field.py
git commit -m "feat: robot_1 front camera ROS bridge in dual field runner"
```

---

### Task 2: 휠 FK — /robot_N/wheel_twist 발행

**Files:**
- Modify: `isaacpjt/Isaac_envo/mecanum_drive.py`
- Modify: `isaacpjt/Isaac_envo/parking/run_dual_robot_ros2_field.py`

**Interfaces:**
- Produces: `mecanum_drive.cmd_vel_from_wheel_velocities(omegas: dict) -> (vx, vy, wz)`
  (로봇 로컬, IK의 최소자승 역), `/robot_N/wheel_twist`(geometry_msgs/TwistStamped,
  로봇 로컬 프레임) — Task 4 데드레커닝의 입력.

- [ ] **Step 1: FK 단위테스트 작성** — `isaacpjt/Isaac_envo/test_mecanum_fk.py`:

```python
#!/usr/bin/env python3
"""IK→FK 왕복이 항등이어야 한다 (순수 파이썬, Isaac 불필요)."""
from mecanum_drive import wheel_velocities_from_cmd_vel, cmd_vel_from_wheel_velocities


def test_fk_roundtrip():
    for cmd in ((0.4, 0.0, 0.0), (0.0, 0.3, 0.0), (0.0, 0.0, 0.5), (0.2, -0.1, 0.3)):
        omegas = wheel_velocities_from_cmd_vel(*cmd)
        back = cmd_vel_from_wheel_velocities(omegas)
        for a, b in zip(cmd, back):
            assert abs(a - b) < 1e-9, f"{cmd} -> {back}"


if __name__ == "__main__":
    test_fk_roundtrip()
    print("FK_ROUNDTRIP=PASS")
```

- [ ] **Step 2: 실패 확인** — `cd isaacpjt/Isaac_envo && python3 test_mecanum_fk.py`
Expected: ImportError (`cmd_vel_from_wheel_velocities` 없음)

- [ ] **Step 3: FK 구현** — `mecanum_drive.py`의 `wheel_velocities_from_cmd_vel` 아래에:

```python
def cmd_vel_from_wheel_velocities(omegas):
    """IK의 최소자승 역: 휠 각속도 dict -> (vx, vy, wz) 로봇 로컬 twist.

    IK가 선형이므로 4x3 행렬의 pseudo-inverse 로 정확히 복원된다. 계수는
    IK 함수에서 수치적으로 추출한다(상수 중복 금지 — IK 가 바뀌면 FK 도 따라간다).
    """
    import numpy as np

    wheels = list(WHEEL_JOINTS)
    basis = [(1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0)]
    A = np.array([[wheel_velocities_from_cmd_vel(*b)[w] for b in basis]
                  for w in wheels])                       # (4, 3)
    vec = np.array([float(omegas[w]) for w in wheels])    # (4,)
    vx, vy, wz = np.linalg.lstsq(A, vec, rcond=None)[0]
    return float(vx), float(vy), float(wz)
```

- [ ] **Step 4: 테스트 통과 확인** — `python3 test_mecanum_fk.py` → `FK_ROUNDTRIP=PASS`

- [ ] **Step 5: 러너에서 발행** — 러너 import에 `cmd_vel_from_wheel_velocities` 추가,
`TwistStamped` import, 로봇별 publisher 생성:

```python
        from geometry_msgs.msg import TwistStamped
        wheel_twist_publishers = {
            robot_id: node.create_publisher(
                TwistStamped, f"/{robot_id}/wheel_twist", 10)
            for robot_id in ROBOT_IDS
        }
```
메인 루프의 odom 발행 옆에 (조인트 속도 → FK):

```python
                joint_vel = robot["articulation"].get_joint_velocities()
                row = joint_vel[0] if joint_vel.ndim == 2 else joint_vel
                omegas = {w: float(row[robot["wheel_indices"][w]])
                          for w in robot["wheel_indices"]}
                vx, vy, wz = cmd_vel_from_wheel_velocities(omegas)
                tw = TwistStamped()
                tw.header.stamp = node.get_clock().now().to_msg()
                tw.header.frame_id = f"{robot_id}/base_link"
                tw.twist.linear.x = vx
                tw.twist.linear.y = vy
                tw.twist.angular.z = -wz   # odom 프레임과 같은 REP-103 부호로
                wheel_twist_publishers[robot_id].publish(tw)
```

- [ ] **Step 6: 실측 검증** — 러너 기동 후 외부 env에서:

```bash
timeout 6 ros2 topic pub -r 10 /robot_1/cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.3}}' >/dev/null &
timeout 5 ros2 topic echo /robot_1/wheel_twist --field twist.linear | head -6
```
Expected: `x:` 값이 0.25~0.35 부근 (명령 0.3 추종. 부호가 음수면 FK 부호 재점검)

- [ ] **Step 7: Commit**
```bash
git add isaacpjt/Isaac_envo/mecanum_drive.py isaacpjt/Isaac_envo/test_mecanum_fk.py \
  isaacpjt/Isaac_envo/parking/run_dual_robot_ros2_field.py
git commit -m "feat: mecanum FK (least-squares inverse of IK) + /robot_N/wheel_twist"
```

---

### Task 3: 측위 노드 ros_map 모드 + 정지 측위 GT 대조

**Files:**
- Modify: `src/parkbot_aruco/parkbot_aruco/marker_localizer_node.py`
- Create: `isaacpjt/Isaac_envo/verify_leader_localization.py`

**Interfaces:**
- Produces: `frame:=ros_map` 파라미터 시 `/robot_pose`(PoseStamped)가
  ros_map 좌표(x, y)+orientation(z/w 쿼터니언, ros_yaw)을 담는다.
  remap으로 `/robot_1/robot_pose` 사용.

- [ ] **Step 1: 노드 확장** — `declare_parameter` 블록에 추가:

```python
        self.declare_parameter("frame", "usd")   # usd(기존 호환) | ros_map
```
`__init__`에서 읽기: `self.frame = self.get_parameter("frame").value`
발행부(`ps = PoseStamped()` 블록)를 다음으로 교체:

```python
            ps = PoseStamped()
            ps.header = msg.header
            if self.frame == "ros_map":
                # 확정 규약: ros_x=usd_x, ros_y=-usd_z, ros_yaw=psi-pi/2
                # (psi=atan2(fwd_x,fwd_z), Task 3 에서 GT 대조 실측으로 확정)
                ps.header.frame_id = "map"
                ps.pose.position.x = fix.x
                ps.pose.position.y = -fix.z
                ros_yaw = math.radians(fix.yaw_deg) - math.pi / 2.0
                ps.pose.orientation.z = math.sin(ros_yaw / 2.0)
                ps.pose.orientation.w = math.cos(ros_yaw / 2.0)
            else:
                ps.pose.position.x = fix.x
                ps.pose.position.y = 0.0
                ps.pose.position.z = fix.z
            self.pub_pose.publish(ps)
```
파일 상단에 `import math` 추가(없으면). 기존 usd 모드는 그대로 — M4 사용처 호환.

- [ ] **Step 2: 재빌드** — `colcon build --packages-select parkbot_aruco && source install/setup.bash`

- [ ] **Step 3: GT 대조 검증 스크립트** — `isaacpjt/Isaac_envo/verify_leader_localization.py`:

```python
#!/usr/bin/env python3
"""robot_1 마커 측위 vs GT odom 대조 (외부 ROS).

robot_1을 A차선(ros y=-2.5)으로 이동시킨 뒤 천천히 전진, 마커 fix 를 GT와 비교.
위치 오차 <0.15m & yaw 오차 <8도 fix 가 3회 이상이면 PASS.
"""
import math
import sys
import time

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class Check(Node):
    def __init__(self):
        super().__init__("verify_leader_localization")
        self.gt = None
        self.hits = []
        self.create_subscription(Odometry, "/robot_1/odom",
                                 lambda m: setattr(self, "gt", m), 10)
        self.create_subscription(PoseStamped, "/robot_1/robot_pose", self.on_fix, 10)
        self.cmd = self.create_publisher(Twist, "/robot_1/cmd_vel", 10)

    def on_fix(self, m):
        if self.gt is None:
            return
        g = self.gt.pose.pose
        dp = math.hypot(m.pose.position.x - g.position.x,
                        m.pose.position.y - g.position.y)
        dyaw = abs((yaw_of(m.pose.orientation) - yaw_of(g.orientation)
                    + math.pi) % (2 * math.pi) - math.pi)
        self.hits.append((dp, math.degrees(dyaw)))
        print(f"fix#{len(self.hits)}: 위치오차={dp:.3f}m yaw오차={math.degrees(dyaw):.1f}deg")

    def drive(self, vx, vy, sec):
        t = Twist(); t.linear.x, t.linear.y = float(vx), float(vy)
        end = time.time() + sec
        while time.time() < end:
            self.cmd.publish(t)
            rclpy.spin_once(self, timeout_sec=0.05)
        self.cmd.publish(Twist())


def main():
    rclpy.init()
    n = Check()
    end = time.time() + 15
    while time.time() < end and n.gt is None:
        rclpy.spin_once(n, timeout_sec=0.2)
    if n.gt is None:
        print("LOCALIZE_VERIFY=FAIL 이유=GT odom 미수신"); sys.exit(1)
    # 도크(y=-7.8) → A차선(y=-2.5): 좌 strafe 5.3m, 이후 저속 전진 8m (마커 열 통과)
    n.drive(0.0, 0.45, 13.0)
    n.drive(0.25, 0.0, 32.0)
    good = [h for h in n.hits if h[0] < 0.15 and h[1] < 8.0]
    verdict = "PASS" if len(good) >= 3 else "FAIL"
    print(f"LOCALIZE_VERIFY={verdict} fixes={len(n.hits)} good={len(good)}")
    sys.exit(0 if verdict == "PASS" else 1)


if __name__ == "__main__":
    main()
```

- [ ] **Step 4: 실행** — 러너(카메라 포함) 기동 + localizer 기동(외부 env 별도 터미널):

```bash
ros2 run parkbot_aruco marker_localizer_node --ros-args \
  -p image_topic:=/robot_1/front_cam/image_raw \
  -p camera_info_topic:=/robot_1/front_cam/camera_info \
  -p frame:=ros_map -r /robot_pose:=/robot_1/robot_pose
# 다른 터미널:
python3 isaacpjt/Isaac_envo/verify_leader_localization.py
```
Expected: `LOCALIZE_VERIFY=PASS`. yaw 오차만 크게(≈90/180도) 나오면 Global
Constraints의 `ros_yaw = ψ − π/2` 를 실측 방향으로 수정 후 재실행 — 그 값이 확정 규약.

- [ ] **Step 5: Commit**
```bash
git add src/parkbot_aruco/parkbot_aruco/marker_localizer_node.py \
  isaacpjt/Isaac_envo/verify_leader_localization.py
git commit -m "feat: marker localizer ros_map frame + yaw, verified against GT"
```

---

### Task 4: pose_estimator(융합) + aruco_navigator_node (NavigateToPose 실물)

**Files:**
- Create: `src/parkbot_aruco/parkbot_aruco/pose_estimator.py`
- Create: `src/parkbot_aruco/test/test_pose_estimator.py`
- Create: `src/parkbot_aruco/parkbot_aruco/aruco_navigator_node.py`
- Modify: `src/parkbot_aruco/setup.py` (console_scripts에
  `'aruco_navigator = parkbot_aruco.aruco_navigator_node:main'` 추가)

**Interfaces:**
- Produces: `PoseEstimator.predict(vx, vy, wz, dt)`, `.correct(x, y, yaw, gain=0.9)`,
  `.pose -> (x, y, yaw)` (전부 ros_map). 액션 서버 `/robot_1/navigate_to_pose`
  (nav2_msgs/NavigateToPose): goal.pose 도달(위치 0.20m, yaw 10도) 시 succeed.
  `/robot_1/nav_pose`(PoseStamped) — 추정 pose 진단 발행.

- [ ] **Step 1: 융합 추정기 단위테스트** — `src/parkbot_aruco/test/test_pose_estimator.py`:

```python
"""데드레커닝 + 마커 보정 융합 (ROS 불필요)."""
import math

from parkbot_aruco.pose_estimator import PoseEstimator


def test_predict_integrates_in_heading_frame():
    est = PoseEstimator(x=0.0, y=0.0, yaw=math.pi / 2)   # +y를 바라봄
    est.predict(vx=1.0, vy=0.0, wz=0.0, dt=0.5)          # 로컬 전진 0.5m
    x, y, yaw = est.pose
    assert abs(x) < 1e-9 and abs(y - 0.5) < 1e-9


def test_correct_pulls_toward_fix():
    est = PoseEstimator(x=1.0, y=0.0, yaw=0.0)
    est.correct(x=2.0, y=0.0, yaw=0.0, gain=0.5)
    assert abs(est.pose[0] - 1.5) < 1e-9


def test_yaw_correct_wraps():
    est = PoseEstimator(x=0.0, y=0.0, yaw=math.pi - 0.05)
    est.correct(x=0.0, y=0.0, yaw=-math.pi + 0.05, gain=1.0)
    assert abs(abs(est.pose[2]) - math.pi) < 0.06
```

- [ ] **Step 2: 실패 확인**
```bash
cd src/parkbot_aruco && PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest test/test_pose_estimator.py -v 2>&1 | tail -2
```
Expected: ImportError

- [ ] **Step 3: 구현** — `src/parkbot_aruco/parkbot_aruco/pose_estimator.py`:

```python
"""마커 fix + 휠 twist 데드레커닝 융합 (순수 파이썬, ros_map 프레임).

M3의 융합 규약을 따른다: 마커가 보이면 게인 0.9로 절대 보정, 안 보이는 구간은
로봇 로컬 twist 를 heading 프레임으로 회전해 적분한다.
"""
import math


def _wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


class PoseEstimator:
    def __init__(self, x=0.0, y=0.0, yaw=0.0):
        self.x, self.y, self.yaw = float(x), float(y), float(yaw)

    @property
    def pose(self):
        return self.x, self.y, self.yaw

    def predict(self, vx, vy, wz, dt):
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        self.x += (vx * c - vy * s) * dt
        self.y += (vx * s + vy * c) * dt
        self.yaw = _wrap(self.yaw + wz * dt)

    def correct(self, x, y, yaw, gain=0.9):
        self.x += gain * (x - self.x)
        self.y += gain * (y - self.y)
        self.yaw = _wrap(self.yaw + gain * _wrap(yaw - self.yaw))
```

- [ ] **Step 4: 테스트 통과 확인** — Step 2 명령 재실행 → 3 passed

- [ ] **Step 5: 네비게이터 노드** — `src/parkbot_aruco/parkbot_aruco/aruco_navigator_node.py`:

```python
#!/usr/bin/env python3
"""aruco_navigator: NavigateToPose 실물 (팀원 navigate_action_server 스켈레톤의 대역).

pose 소스는 GT가 아니라 마커 fix(/robot_pose) + 휠 twist(/wheel_twist) 융합이다.
목표 하나 = 웨이포인트 하나. 경로/존 관리는 상위(미션/orchestrator) 몫.
초기 pose 는 파라미터로 받는다(도크 좌표 — 실제 서비스에선 첫 마커가 잡아줌).
"""
import math

import rclpy
from geometry_msgs.msg import PoseStamped, Twist, TwistStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionServer, CancelResponse
from rclpy.node import Node

from parkbot_aruco.pose_estimator import PoseEstimator


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def _wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


class ArucoNavigatorNode(Node):
    def __init__(self):
        super().__init__("aruco_navigator")
        p = self.declare_parameter
        p("initial_x", -15.3); p("initial_y", -7.8); p("initial_yaw", 0.0)
        p("pos_tol", 0.20); p("yaw_tol_deg", 10.0)
        p("k_lin", 0.8); p("k_yaw", 1.2); p("max_lin", 0.35); p("max_yaw", 0.5)
        g = lambda k: self.get_parameter(k).value
        self.est = PoseEstimator(g("initial_x"), g("initial_y"), g("initial_yaw"))
        self.pos_tol, self.yaw_tol = g("pos_tol"), math.radians(g("yaw_tol_deg"))
        self.k_lin, self.k_yaw = g("k_lin"), g("k_yaw")
        self.max_lin, self.max_yaw = g("max_lin"), g("max_yaw")
        self._last_twist_stamp = None
        self.fix_count = 0

        self.cmd_pub = self.create_publisher(Twist, "cmd_vel", 10)
        self.diag_pub = self.create_publisher(PoseStamped, "nav_pose", 10)
        self.create_subscription(TwistStamped, "wheel_twist", self._on_twist, 50)
        self.create_subscription(PoseStamped, "robot_pose", self._on_fix, 10)
        self._server = ActionServer(
            self, NavigateToPose, "navigate_to_pose", self._execute,
            cancel_callback=lambda _: CancelResponse.ACCEPT)
        self.get_logger().info("aruco_navigator 시작 (마커+데드레커닝 융합)")

    def _on_twist(self, m):
        t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        if self._last_twist_stamp is not None:
            dt = max(0.0, min(0.1, t - self._last_twist_stamp))
            self.est.predict(m.twist.linear.x, m.twist.linear.y,
                             m.twist.angular.z, dt)
        self._last_twist_stamp = t

    def _on_fix(self, m):
        self.est.correct(m.pose.position.x, m.pose.position.y,
                         yaw_of(m.pose.orientation))
        self.fix_count += 1

    def _publish_cmd(self, vx, vy, wz):
        t = Twist()
        t.linear.x, t.linear.y, t.angular.z = float(vx), float(vy), float(wz)
        self.cmd_pub.publish(t)

    def _execute(self, goal_handle):
        gp = goal_handle.request.pose.pose
        gx, gy, gyaw = gp.position.x, gp.position.y, yaw_of(gp.orientation)
        rate_dt = 0.05
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=rate_dt)
            if goal_handle.is_cancel_requested:
                self._publish_cmd(0, 0, 0)
                goal_handle.canceled()
                return NavigateToPose.Result()
            x, y, yaw = self.est.pose
            ex, ey = gx - x, gy - y
            eyaw = _wrap(gyaw - yaw)
            if math.hypot(ex, ey) < self.pos_tol and abs(eyaw) < self.yaw_tol:
                break
            # 월드 오차를 로봇 로컬로 회전 → 홀로노믹 P제어 (회전 최소화)
            c, s = math.cos(yaw), math.sin(yaw)
            lx, ly = ex * c + ey * s, -ex * s + ey * c
            clamp = lambda v, m: max(-m, min(m, v))
            self._publish_cmd(clamp(self.k_lin * lx, self.max_lin),
                              clamp(self.k_lin * ly, self.max_lin),
                              clamp(self.k_yaw * eyaw, self.max_yaw))
            d = PoseStamped()
            d.header.frame_id = "map"
            d.pose.position.x, d.pose.position.y = x, y
            d.pose.orientation.z = math.sin(yaw / 2)
            d.pose.orientation.w = math.cos(yaw / 2)
            self.diag_pub.publish(d)
        self._publish_cmd(0, 0, 0)
        goal_handle.succeed()
        self.get_logger().info(
            f"도착 ({gx:+.2f},{gy:+.2f}) — 누적 마커 보정 {self.fix_count}회")
        return NavigateToPose.Result()


def main(args=None):
    rclpy.init(args=args)
    node = ArucoNavigatorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
```
주의: ActionServer 실행 콜백 안에서 `rclpy.spin_once`를 돌리므로 이 노드는
**단일 스레드 + 액션 1개 전용**이다(데모 용도로 충분. 멀티 goal은 범위 밖).

- [ ] **Step 6: setup.py 등록 + 빌드**
```bash
# setup.py console_scripts에 'aruco_navigator = parkbot_aruco.aruco_navigator_node:main' 추가 후
colcon build --packages-select parkbot_aruco && source install/setup.bash
ros2 run parkbot_aruco aruco_navigator --ros-args -r __ns:=/robot_1 -p initial_x:=-15.3 &
sleep 4 && ros2 action list | grep navigate && kill %1
```
Expected: `/robot_1/navigate_to_pose` 출력. (navigator는 자기 네임스페이스 토픽만
쓰므로 `__ns` 사용 가능 — formation 컨트롤러와 다른 점)

- [ ] **Step 7: Commit**
```bash
git add src/parkbot_aruco
git commit -m "feat: aruco_navigator (NavigateToPose on marker+dead-reckoning fusion)"
```

---

### Task 5: leg 미션 — 도크→H_B 접근점 ArUco 폐루프 + 존 락 + 편대

**Files:**
- Create: `isaacpjt/Isaac_envo/aruco_leg_mission.py`

**Interfaces:**
- Consumes: PathFinder(H_B 경로), `/acquire_zones`·`/release_zones`(dispatcher,
  `zone_lock_mode:=db`), `/robot_1/navigate_to_pose`, formation_assignment
- Produces: `ARUCO_LEG=PASS` — GT 무개입 주행으로 H_B 접근점 도달

- [ ] **Step 1: 미션 스크립트** — `isaacpjt/Isaac_envo/aruco_leg_mission.py`:

```python
#!/usr/bin/env python3
"""임시 미션: robot_1 이 ArUco 폐루프로 도크→H_B 접근점(HJ3, 베이 2m 앞)까지.

- 경로: PathFinder(dock_wait_A → H_B). 마지막 노드는 베이 중심 대신 접근점
  (HJ3 x, 베이 y-2.5·부호 보정)으로 치환 — 도킹은 Plan 3 몫.
- 존: 구간마다 acquire_zones(robot_1) → 통과 → release (팀원 권장 규약).
- 편대: robot_2 팔로워 배정(호송). GT는 최종 채점(도착 오차)만.
"""
import math
import sys
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node

from parking_control.core.graph import ParkingMap
from parking_control.core.pathfinder import PathFinder
from parking_robot_interfaces.msg import FormationAssignment
from parking_robot_interfaces.srv import AcquireZones, ReleaseZones

MAP_YAML = "/home/rokey/cobot3_ws/src/parking_control/config/parking_map.yaml"
GOAL_TOL_GT = 0.35   # 최종 채점(GT 기준) 허용 오차


class LegMission(Node):
    def __init__(self):
        super().__init__("aruco_leg_mission")
        self.gt = None
        self.create_subscription(Odometry, "/robot_1/odom",
                                 lambda m: setattr(self, "gt", m), 10)
        self.nav = ActionClient(self, NavigateToPose, "/robot_1/navigate_to_pose")
        self.acquire = self.create_client(AcquireZones, "/acquire_zones")
        self.release = self.create_client(ReleaseZones, "/release_zones")
        self.assign = self.create_publisher(FormationAssignment, "/formation_assignment", 10)

    def call(self, client, req, timeout=5.0):
        fut = client.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=timeout)
        return fut.result()

    def acquire_zone(self, zone):
        while True:
            r = self.call(self.acquire, AcquireZones.Request(
                robot_id="robot_1", task_id="", zone_ids=[zone]))
            if r and r.granted:
                self.get_logger().info(f"존 획득: {zone}")
                return
            wait = (r.retry_after_sec if r else 1.0) or 1.0
            self.get_logger().info(f"존 대기: {zone} ({wait:.0f}s)")
            time.sleep(wait)

    def release_zone(self, zone):
        self.call(self.release, ReleaseZones.Request(
            robot_id="robot_1", task_id="", zone_ids=[zone]))

    def goto(self, x, y):
        goal = NavigateToPose.Goal()
        goal.pose.pose.position.x = float(x)
        goal.pose.pose.position.y = float(y)
        goal.pose.pose.orientation.w = 1.0   # yaw 0 (동쪽) 유지
        self.nav.wait_for_server()
        send = self.nav.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send, timeout_sec=10)
        handle = send.result()
        if handle is None or not handle.accepted:
            return False
        res = handle.get_result_async()
        rclpy.spin_until_future_complete(self, res, timeout_sec=180)
        return res.done()

    def broadcast(self, active):
        for rid, role, partner in (("robot_2", "follower", "robot_1"),
                                   ("robot_1", "leader", "robot_2")):
            self.assign.publish(FormationAssignment(
                robot_id=rid, task_id="aruco-leg" if active else "",
                role=role, partner_robot_id=partner, active=active))


def main():
    rclpy.init()
    n = LegMission()
    pf = PathFinder(ParkingMap.load(MAP_YAML))
    result = pf.find_path("dock_wait_A", "H_B")
    nodes, waypoints = result.nodes, list(result.waypoints)
    # 마지막(H_B 베이 중심 (-29.6,+7.8)) → 접근점 (-29.6,+2.5+? ) : 통로쪽 2m 앞
    waypoints[-1] = (-29.6, 2.5)
    # 엣지별 존: pathfinder 결과의 zones 는 통과 순서 (엣지 수보다 적을 수 있음)
    print(f"경로: {' → '.join(nodes)} / 존: {result.zones}")
    n.broadcast(True)
    held = None
    zone_iter = iter(result.zones)
    try:
        for i, (x, y) in enumerate(waypoints):
            nxt = next(zone_iter, None)
            if nxt:
                n.acquire_zone(nxt)
                if held:
                    n.release_zone(held)
                held = nxt
            ok = n.goto(x, y)
            if not ok:
                print("ARUCO_LEG=FAIL 이유=goal 실패"); sys.exit(1)
    finally:
        if held:
            n.release_zone(held)
        n.broadcast(False)
    # GT 채점
    end = time.time() + 5
    while time.time() < end:
        rclpy.spin_once(n, timeout_sec=0.2)
    g = n.gt.pose.pose.position
    err = math.hypot(g.x - waypoints[-1][0], g.y - waypoints[-1][1])
    verdict = "PASS" if err < GOAL_TOL_GT else "FAIL"
    print(f"ARUCO_LEG={verdict} GT기준 도착 오차={err:.2f}m")
    sys.exit(0 if verdict == "PASS" else 1)


if __name__ == "__main__":
    main()
```

- [ ] **Step 2: 전체 스택 기동** (터미널: 러너 / dispatcher(db) / slot_manager /
localizer / navigator / formation×2)

```bash
bash isaacpjt/Isaac_envo/parking/run_dual_robot_ros2_field.sh --headless &
ros2 run parking_control task_dispatcher --ros-args -p zone_lock_mode:=db &
ros2 run parking_control parking_slot_manager &
ros2 run parkbot_aruco marker_localizer_node --ros-args \
  -p image_topic:=/robot_1/front_cam/image_raw \
  -p camera_info_topic:=/robot_1/front_cam/camera_info \
  -p frame:=ros_map -r /robot_pose:=/robot_1/robot_pose &
ros2 run parkbot_aruco aruco_navigator --ros-args -r __ns:=/robot_1 &
ros2 run parking_control formation_gap_controller --ros-args -p robot_id:=robot_2 \
  -r cmd_vel:=/robot_2/cmd_vel -r odom:=/robot_2/odom -r __node:=fgc_r2 &
```

- [ ] **Step 3: 미션 실행**
```bash
python3 isaacpjt/Isaac_envo/aruco_leg_mission.py
```
Expected: 존 획득/반납 로그가 경로 순서대로, navigator "도착 … 마커 보정 N회"(N≥5),
`ARUCO_LEG=PASS GT기준 도착 오차<0.35m`.
FAIL 시: `/robot_1/nav_pose` vs `/robot_1/odom` 궤적을 비교해 추정 발산 지점 확인
(마커 무보정 구간이 길면 waypoint 간격/속도 조정).

- [ ] **Step 4: 존 원장 확인**
```bash
mysql -u parking -pparking1234 parking -e "SELECT * FROM zone_locks;"
```
Expected: 0행 (전부 반납됨)

- [ ] **Step 5: Commit**
```bash
git add isaacpjt/Isaac_envo/aruco_leg_mission.py
git commit -m "feat: ArUco closed-loop leg mission dock->H_B approach with per-zone locks"
```

---

## 완료 기준 (Plan 2 전체)

1. `/robot_1/front_cam/*` 발행 (≥10Hz) + 휠 FK 왕복 단위테스트 PASS
2. `LOCALIZE_VERIFY=PASS` — 마커 측위 ros_map 변환이 GT 대조로 확정
3. pose_estimator 단위테스트 3건 + navigator 액션 서버 기동
4. **`ARUCO_LEG=PASS`** — GT 무개입 폐루프로 도크→H_B 접근점, 존 락 원장 정합,
   팔로워 호송 동반

## 리스크

- 카메라 검출창이 로봇 앞 1.1~1.4m로 좁음(기존 실측) — waypoint 3.4m 간격과
  데드레커닝으로 메꾸는 구조라, 휠 FK 품질이 나쁘면(복합운동 ~0.5x 과소) 보정
  전 이탈 가능. 대응: 직선 구간 위주 경로 + 저속(0.35) + 필요시 마커 열 위 정지 재보정.
- 인계장(HJ 열)은 마커가 차선(y=±2.5)에 있는데 경로 waypoint 는 중앙(y=0) —
  전방 카메라가 마커를 못 볼 수 있음. 미션에서 HJ 구간 waypoint 를 y=-2.5 차선으로
  치환하는 조정이 필요할 수 있다(실측 후 결정).
- GUI 실행 시 렌더+카메라 2개로 GPU 부하 — 데모는 headless 우선, GUI는 최종 시연.
