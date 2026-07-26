# ROS2 촉발 도킹·리프트·운반 (Plan 3) Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 외부 ROS2에서 `/dock_lift` 서비스를 한 번 호출하면, 로봇 2대가 순차로 A5_Coupe 하부에
진입해(실제 메카넘 구동, 바퀴감지 없이 하드코딩 위치 정지) 팔로 바퀴를 파지·리프트하고 편대로
운반하는 것까지 자동 진행한다.

**Architecture:** Isaac 러너(`dock_lift_runner.py`)는 차량+로봇2 씬을 만들고 ROS2 브리지
(로봇별 `cmd_vel` 구독 / `odom` 발행 / `arm_control` 서비스)만 노출한 채 대기한다. 외부 ROS2
오케스트레이터(`dock_lift_mission.py`)가 `/dock_lift` 요청을 받아 순수 상태기계
(`dock_lift_state.py`)로 진입→파지→운반 순서를 구동한다. carry demo의 검증된 물리(축 좌표
산출·`ARM_TARGETS`·그립 마찰재·리프트/운반)를 그대로 이식하되 제어 주체만 ROS2로 올린다.

**Tech Stack:** Isaac Sim 5.1(Python 3.11 내부 rclpy) + 외부 ROS2 Humble(3.10) + `pxr`(USD) +
`isaacsim.core.prims.Articulation` + `std_srvs/SetBool`·`std_srvs/Trigger` + numpy.

## Global Constraints

- 이 머신은 GPU PhysX(RTX 5080). Isaac 스크립트 실행 전후 좀비 프로세스 확인·정리:
  `ps aux | grep -iE "isaac" | grep -v grep` 후 필요 시 `kill`.
- 원본 USD 비파괴 — 차량 콜라이더/그립 재질은 **런타임 stage override**로만. 원본
  `fab_vehicles.usd`·로봇 에셋 수정 금지.
- 로봇 에셋은 낮춘 바퀴 변형 `hwia_depth_cam_mecha_roller_lowered.usd`(지상고 +3cm) 사용.
- 좌표: 월드 XZ 평면, Y-up. 차량 길이축 = z. 로봇 진입 = 직진(vx>0)만, 제자리 회전 금지.
- ROS2 env(외부 터미널 공통 = "**외부 env**"):
  ```bash
  source /opt/ros/humble/setup.bash
  source /home/rokey/cobot3_ws/install/setup.bash
  export ROS_DOMAIN_ID=126 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
  unset FASTRTPS_DEFAULT_PROFILES_FILE
  ```
- Isaac 러너는 내부 Humble libs 사용(`run_dual_robot_ros2_field.sh`와 동일 env: 도메인 126,
  RMW fastrtps, **화이트리스트 unset**, LD_LIBRARY_PATH=브리지 humble lib). `/opt/ros` 소싱 금지.
- cmd_vel `angular.z`는 러너 경계에서 부호 반전(REP-103 정합, Plan 2 실측). 진입/운반은 직진이라
  wz=0이지만 규약은 유지.
- 검증된 물리 상수(ARM_TARGETS, 그립 마찰 static 1.2/dynamic 1.0, 램프 스텝)는 바꾸지 않는다.

## 재사용 상수 (carry demo·arm service에서 이식, 여러 파일이 공유)

```python
ARM_TARGETS = {
    "arm_left_front_joint": 90.0, "arm_left_rear_joint": -90.0,
    "arm_right_front_joint": -90.0, "arm_right_rear_joint": 90.0,
}
VEHICLE_WHEELS = ("FrontLeftWheel", "FrontRightWheel", "RearLeftWheel", "RearRightWheel")
ROBOT_APPROACH_GAP_M = 1.75   # 각 축에서 이만큼 떨어진 곳에서 진입 시작
CARRY_SPEED = 0.35            # m/s per robot, 운반 중 world +Z 방향
CARRY_DISTANCE_M = 1.0
INGRESS_SPEED = 0.30          # m/s, 하부 진입 구동 속도
POS_TOL_M = 0.05             # 진입 목표 z 도달 허용
```

## File Structure

- Create: `isaacpjt/Isaac_envo/dock_lift_runner.py` — Isaac 러너: 씬(A5_Coupe + 콜라이더 수정 +
  그립 재질 + 로봇2 접근 스폰) + ROS2 브리지(cmd_vel/odom/arm_control). SimulationApp 필요.
- Create: `isaacpjt/Isaac_envo/dock_lift_runner.sh` — 러너 실행 래퍼(내부 humble env).
- Create: `src/parkbot_aruco/parkbot_aruco/dock_lift_state.py` — 순수 상태기계(전이·게이트·실패).
  ROS/Isaac 불의존.
- Create: `src/parkbot_aruco/test/test_dock_lift_state.py` — 상태기계 단위 테스트.
- Create: `isaacpjt/Isaac_envo/dock_lift_mission.py` — 외부 오케스트레이터: `/dock_lift` 서비스 →
  순차 진입(cmd_vel) + 파지(arm_control) + 운반(cmd_vel). 상태기계 사용.
- Modify: `src/parkbot_aruco/setup.py` — `dock_lift_mission`은 Isaac_envo 스크립트라 등록 불필요;
  상태기계만 패키지에 포함(별도 entry 없음).

---

### Task 1: dock_lift_runner 씬 — 차량 + 콜라이더 수정 + 그립 재질 + 로봇2

**Files:**
- Create: `isaacpjt/Isaac_envo/dock_lift_runner.py` (씬 빌드 + 헤드리스 물리 검증까지)

**Interfaces:**
- Produces: `build_stage()` — A5_Coupe(`/World/Vehicle`) + 로봇2(`/World/Robots/robot_rear`,
  `/World/Robots/robot_front`)를 접근 위치에 배치한 stage. 축 좌표(`AXLE`)를 실제 휠에서 산출해
  모듈 전역에 기록: `AXLE = {"rear_z": float, "front_z": float, "center_x": float}`,
  로봇별 `start_z`/`target_z`. `--headless-test`로 물리 안정성 확인.

- [ ] **Step 1: 재실행 + 상수 + 경로 헤더 작성** — 파일 상단:

```python
#!/usr/bin/env python3
"""ROS2 촉발 도킹·리프트·운반 Isaac 러너.

씬: A5_Coupe(콜라이더 수정+그립 재질) + 낮춘 바퀴 로봇 2대(축 앞 접근 위치).
ROS2: /robot_N/cmd_vel 구독, /robot_N/odom 발행, /robot_N/arm_control 서비스.
프로그램은 켜두고 대기 — 외부 오케스트레이터가 구동한다.

실행: dock_lift_runner.sh [--gui] [--headless-test]
"""
import math
import os
import sys
from pathlib import Path

WORK_DIR = Path(__file__).resolve().parent
ROBOT_USD = (WORK_DIR.parent / "hwia_parking_robot_final_caster_package"
             / "hwia_depth_cam_mecha_roller_lowered.usd")
VEHICLES_USD = WORK_DIR / "fab_vehicles.usd"
ISAAC_PYTHON = Path("/home/rokey/dev_ws/isaac_sim/isaacsim/_build/linux-x86_64/release/python.sh")
BRIDGE_RCLPY = Path("/home/rokey/dev_ws/isaac_sim/isaacsim/_build/linux-x86_64/release"
                    "/exts/isaacsim.ros2.bridge/humble/rclpy")

TARGET_VEHICLE = "Coupe"           # fab_vehicles 내 프림 이름
ARM_TARGETS = {
    "arm_left_front_joint": 90.0, "arm_left_rear_joint": -90.0,
    "arm_right_front_joint": -90.0, "arm_right_rear_joint": 90.0,
}
VEHICLE_WHEELS = ("FrontLeftWheel", "FrontRightWheel", "RearLeftWheel", "RearRightWheel")
ROBOT_APPROACH_GAP_M = 1.75

ROBOTS = {
    "rear":  {"xform": "/World/Robots/robot_rear",  "facing": +1},
    "front": {"xform": "/World/Robots/robot_front", "facing": -1},
}
AXLE = {}   # build_stage()가 채운다


def _restart_with_isaac_python():
    if os.environ.get("CARB_APP_PATH"):
        return
    os.execv(str(ISAAC_PYTHON), [str(ISAAC_PYTHON), str(Path(__file__).resolve()), *sys.argv[1:]])
```

- [ ] **Step 2: 콜라이더 수정 + 그립 재질 헬퍼** — 아래 함수 추가:

```python
def _fix_vehicle_colliders(stage, vehicle_path):
    """대상 차량 바퀴 Collision.height(축 X=횡폭)를 시각 폭으로 축소(override).

    조사 실측: Coupe 휠 콜라이더가 시각 대비 1.61배 넓어 로봇 진입이 빡빡하다.
    원본 비파괴 — 런타임 stage 에서만 height 를 시각 실린더 반경*2 로 낮춘다.
    구동계 raycast 서스펜션은 콜라이더 형상을 안 쓰므로 물리 무관.
    """
    from pxr import UsdGeom
    fixed = 0
    for wheel_name in VEHICLE_WHEELS:
        wheel = stage.GetPrimAtPath(f"{vehicle_path}/{wheel_name}")
        if not wheel or not wheel.IsValid():
            continue
        col = None
        vis_r = None
        for d in wheel.GetChildren():
            p = str(d.GetPath()).lower()
            if "collision" in p and d.IsA(UsdGeom.Cylinder):
                col = UsdGeom.Cylinder(d)
            if "collision" not in p and d.IsA(UsdGeom.Cylinder):
                vis_r = d.GetAttribute("radius").Get()
        if col is not None and vis_r is not None:
            col.GetHeightAttr().Set(float(vis_r) * 2.0)   # 시각 폭 = 반경*2
            fixed += 1
    return fixed


def _grip_material(stage):
    """파지 마찰재(static 1.2 / dynamic 1.0). carry demo 검증값."""
    from pxr import UsdShade, UsdPhysics
    mat = UsdShade.Material.Define(stage, "/World/Looks/Grip")
    api = UsdPhysics.MaterialAPI.Apply(mat.GetPrim())
    api.CreateStaticFrictionAttr(1.2)
    api.CreateDynamicFrictionAttr(1.0)
    api.CreateRestitutionAttr(0.0)
    return mat
```

(주: 실제 wheel 하위 collision 프림 이름은 Task 1 Step 5 검증에서 확인해 필요시 매칭을 조정.
`_fix_vehicle_colliders` 반환이 0이면 매칭 실패 — 그때 프림 트리를 출력해 이름을 맞춘다.)

- [ ] **Step 3: build_stage() — 차량·로봇 배치 + 축 좌표 산출**:

```python
def build_stage(app):
    from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics, UsdShade
    import omni.usd

    ctx = omni.usd.get_context()
    ctx.new_stage()
    stage = ctx.get_stage()
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.y)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    world = UsdGeom.Xform.Define(stage, "/World")
    stage.SetDefaultPrim(world.GetPrim())

    # GPU PhysX 씬 (검증된 설정)
    scene = UsdPhysics.Scene.Define(stage, "/World/PhysicsScene")
    scene.CreateGravityDirectionAttr(Gf.Vec3f(0, -1, 0))
    scene.CreateGravityMagnitudeAttr(9.81)
    from pxr import PhysxSchema
    px = PhysxSchema.PhysxSceneAPI.Apply(scene.GetPrim())
    px.CreateBroadphaseTypeAttr("GPU")
    px.CreateEnableGPUDynamicsAttr(True)
    px.CreateSolverTypeAttr("TGS")

    # 바닥
    ground = UsdGeom.Cube.Define(stage, "/World/Ground")
    ground.CreateSizeAttr(1.0)
    UsdGeom.Xformable(ground).AddTranslateOp().Set(Gf.Vec3d(0, -0.05, 0))
    UsdGeom.Xformable(ground).AddScaleOp().Set(Gf.Vec3f(40, 0.1, 40))
    UsdPhysics.CollisionAPI.Apply(ground.GetPrim())

    _grip_material(stage)

    # 대상 차량 참조
    veh = stage.DefinePrim("/World/Vehicle", "Xform")
    veh.GetReferences().AddReference(f"./{VEHICLES_USD.name}", f"/World/{TARGET_VEHICLE}")
    UsdGeom.Xformable(veh).AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, 0.0))
    for _ in range(30):
        app.update()

    _fix_vehicle_colliders(stage, "/World/Vehicle")

    # 축 좌표 = 실제 휠 world z (carry demo 방식)
    cache = UsdGeom.XformCache()
    centers = {}
    for wn in VEHICLE_WHEELS:
        w = stage.GetPrimAtPath(f"/World/Vehicle/{wn}")
        if not w.IsValid():
            raise RuntimeError(f"휠 없음: /World/Vehicle/{wn}")
        centers[wn] = cache.GetLocalToWorldTransform(w).ExtractTranslation()
    front_z = (centers["FrontLeftWheel"][2] + centers["FrontRightWheel"][2]) * 0.5
    rear_z = (centers["RearLeftWheel"][2] + centers["RearRightWheel"][2]) * 0.5
    center_x = sum(c[0] for c in centers.values()) / 4.0
    low_z, high_z = sorted((front_z, rear_z))
    AXLE.update(rear_z=low_z, front_z=high_z, center_x=center_x)
    ROBOTS["rear"]["target_z"] = low_z
    ROBOTS["rear"]["start_z"] = low_z - ROBOT_APPROACH_GAP_M
    ROBOTS["front"]["target_z"] = high_z
    ROBOTS["front"]["start_z"] = high_z + ROBOT_APPROACH_GAP_M

    # 로봇 2대 배치 (진입 방향을 향하도록: rear=+Z 향함, front=-Z 향함)
    for key, cfg in ROBOTS.items():
        r = stage.DefinePrim(cfg["xform"], "Xform")
        r.GetReferences().AddReference(
            f"../hwia_parking_robot_final_caster_package/{ROBOT_USD.name}")
        m = _robot_matrix(Gf, cfg["facing"], center_x, cfg["start_z"])
        _set_matrix(UsdGeom.Xformable(r), m)
    for _ in range(30):
        app.update()
    print(f"DOCK_STAGE_READY axle rear_z={low_z:.3f} front_z={high_z:.3f} "
          f"wheelbase={abs(high_z-low_z):.3f} center_x={center_x:.3f}", flush=True)
    return stage


def _robot_matrix(Gf, facing, tx, tz):
    """로봇 로컬 +X를 world ±Z로 향하게 하는 4x4. facing=+1 -> +Z, -1 -> -Z."""
    import math as _m
    yaw = _m.pi / 2 if facing > 0 else -_m.pi / 2   # +X -> +Z / -Z
    c, s = _m.cos(yaw), _m.sin(yaw)
    # world Y축 회전 (XZ 평면). row-major USD Matrix4d.
    return Gf.Matrix4d(
        c, 0, -s, 0,
        0, 1, 0, 0,
        s, 0, c, 0,
        tx, 0.03, tz, 1)   # y=0.03: 낮춘 바퀴 지상고 만큼 살짝 띄워 스폰


def _set_matrix(xformable, matrix):
    from pxr import UsdGeom
    xformable.ClearXformOpOrder()
    xformable.AddTransformOp().Set(matrix)
```

(주: `_robot_matrix`의 yaw 부호와 y 스폰 높이는 Step 5 물리 검증에서 로봇이 안정 착지하는지로
확인. 로봇이 뒤집히거나 튀면 yaw 부호/스폰 y를 조정 — Plan 1·2에서 로봇 스폰은 y≈0 근처에서
안정했음.)

- [ ] **Step 4: main() — 헤드리스 물리 검증 경로**:

```python
def main():
    _restart_with_isaac_python()
    from isaacsim import SimulationApp
    headless = "--gui" not in sys.argv[1:]
    app = SimulationApp({"headless": headless, "width": 1280, "height": 800})
    try:
        from isaacsim.core.utils.extensions import enable_extension
        enable_extension("isaacsim.ros2.bridge")
        for _ in range(12):
            app.update()
        import numpy as np
        import omni.timeline
        from isaacsim.core.prims import Articulation

        stage = build_stage(app)
        timeline = omni.timeline.get_timeline_interface()
        timeline.play()
        for _ in range(30):
            app.update()

        arts = {}
        for key, cfg in ROBOTS.items():
            art = Articulation(f"{cfg['xform']}/base_link")
            art.initialize()
            arts[key] = art

        if "--headless-test" in sys.argv[1:]:
            p0 = {k: np.asarray(a.get_world_poses()[0]).reshape(-1)[:3] for k, a in arts.items()}
            for _ in range(180):
                app.update()
            p1 = {k: np.asarray(a.get_world_poses()[0]).reshape(-1)[:3] for k, a in arts.items()}
            disp = {k: float(np.linalg.norm(p1[k] - p0[k])) for k in arts}
            ok = all(d < 0.35 for d in disp.values())
            print(f"DOCK_PHYSICS_TEST={'PASS' if ok else 'FAIL'} disp={disp}", flush=True)
            app.close()
            return
        # (Task 2~3에서 ROS2 루프 추가)
        app.close()
    finally:
        pass


if __name__ == "__main__":
    main()
```

- [ ] **Step 5: 헤드리스 물리 검증 실행**

```bash
ps aux | grep -iE "isaac" | grep -v grep   # 좀비 정리
cd /home/rokey/cobot3_ws/isaacpjt/Isaac_envo
python3 dock_lift_runner.py --headless-test 2>&1 | grep -E "DOCK_STAGE_READY|DOCK_PHYSICS_TEST|Error|RuntimeError"
```
Expected: `DOCK_STAGE_READY axle ...`(축거 ≈ 2.715 Coupe) 그리고 `DOCK_PHYSICS_TEST=PASS`.
- 콜라이더 매칭 실패(`_fix_vehicle_colliders` 0) 또는 휠 프림 못 찾으면: 차량 프림 트리를
  출력해(`for p in stage.Traverse(): print(p.GetPath())` 임시 삽입) 실제 이름으로 맞춘다.
- 로봇이 튀면(disp 큼) `_robot_matrix` yaw 부호/스폰 y 조정.

- [ ] **Step 6: Commit**

```bash
cd /home/rokey/cobot3_ws
git add isaacpjt/Isaac_envo/dock_lift_runner.py
git commit -m "feat: dock_lift_runner scene (A5_Coupe collider-fixed + grip material + 2 lowered robots)"
```

---

### Task 2: ROS2 cmd_vel/odom 브리지 (러너)

**Files:**
- Modify: `isaacpjt/Isaac_envo/dock_lift_runner.py` (main 루프에 ROS2 브리지 추가)
- Create: `isaacpjt/Isaac_envo/dock_lift_runner.sh`

**Interfaces:**
- Consumes: Task 1의 `arts`(로봇별 Articulation), `ROBOTS`
- Produces: `/robot_rear/cmd_vel`·`/robot_front/cmd_vel` 구독(메카넘 IK 구동),
  `/robot_rear/odom`·`/robot_front/odom` 발행(world pose, z 성분=world z). `mecanum_drive`의
  `wheel_velocities_from_cmd_vel`·`WHEEL_JOINTS`·`configure_hub_drives` 재사용.

- [ ] **Step 1: 실행 래퍼 작성** — `dock_lift_runner.sh` (run_dual_robot_ros2_field.sh 복제):

```bash
#!/bin/bash
set -u
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REL=$HOME/dev_ws/isaac_sim/isaacsim/_build/linux-x86_64/release
unset PYTHONPATH AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH
unset FASTRTPS_DEFAULT_PROFILES_FILE FASTDDS_DEFAULT_PROFILES_FILE
export ROS_DISTRO=humble
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-126}"
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export LD_LIBRARY_PATH="$REL/exts/isaacsim.ros2.bridge/humble/lib"
exec "$REL/python.sh" "$SCRIPT_DIR/dock_lift_runner.py" "$@"
```

- [ ] **Step 2: main()에 hub drive 설정 + cmd_vel 구동 함수 추가** — `arts` 초기화 직후:

```python
        sys.path.insert(0, str(WORK_DIR))
        from mecanum_drive import (WHEEL_JOINTS, configure_hub_drives,
                                   wheel_velocities_from_cmd_vel)
        for key, cfg in ROBOTS.items():
            configure_hub_drives(stage, f"{cfg['xform']}/joints")
        wheel_idx = {k: {w: arts[k].dof_names.index(j) for w, j in WHEEL_JOINTS.items()}
                     for k in arts}
        vel_buf = {k: np.zeros(arts[k].get_joint_positions().shape, dtype=np.float32)
                   for k in arts}

        def drive(key, vx, vy, wz):
            omegas = wheel_velocities_from_cmd_vel(vx, vy, wz)
            buf = vel_buf[key]; buf[...] = 0.0
            for w, om in omegas.items():
                i = wheel_idx[key][w]
                if buf.ndim == 2: buf[0, i] = om
                else: buf[i] = om
            arts[key].set_joint_velocity_targets(buf)
```

- [ ] **Step 3: ROS2 노드 + cmd_vel 구독 / odom 발행** — `--headless-test` 분기 뒤(대기 루프):

```python
        if str(BRIDGE_RCLPY) not in sys.path:
            sys.path.insert(0, str(BRIDGE_RCLPY))
        import rclpy
        from geometry_msgs.msg import Twist
        from nav_msgs.msg import Odometry
        rclpy.init()
        node = rclpy.create_node("dock_lift_runner_bridge")
        last_cmd_at = {k: 0.0 for k in arts}

        def make_cb(key):
            def cb(msg):
                last_cmd_at[key] = 1.0
                drive(key, msg.linear.x, msg.linear.y, -msg.angular.z)
            return cb
        odom_pub = {}
        for key, cfg in ROBOTS.items():
            node.create_subscription(Twist, f"/{ 'robot_'+key }/cmd_vel", make_cb(key), 10)
            odom_pub[key] = node.create_publisher(Odometry, f"/robot_{key}/odom", 10)

        print("DOCK_LIFT_RUNNER_READY robots=['robot_rear','robot_front'] "
              f"domain={os.environ.get('ROS_DOMAIN_ID','0')}", flush=True)
        import time as _t
        while app.is_running():
            app.update()
            rclpy.spin_once(node, timeout_sec=0.0)
            for key, cfg in ROBOTS.items():
                pos, orn = arts[key].get_world_poses()
                pos = np.asarray(pos).reshape(-1)[:3]
                orn = np.asarray(orn).reshape(-1)[:4]
                w, x, y, z = (float(v) for v in orn)
                fwd_x = 1.0 - 2.0 * (y*y + z*z)
                fwd_z = 2.0 * (x*z - w*y)
                yaw = math.atan2(-fwd_z, fwd_x)
                od = Odometry()
                od.header.stamp = node.get_clock().now().to_msg()
                od.header.frame_id = "map"
                od.child_frame_id = f"robot_{key}/base_link"
                od.pose.pose.position.x = float(pos[0])
                od.pose.pose.position.y = float(pos[1])
                od.pose.pose.position.z = float(pos[2])   # world z (진입 축)
                od.pose.pose.orientation.z = math.sin(yaw*0.5)
                od.pose.pose.orientation.w = math.cos(yaw*0.5)
                odom_pub[key].publish(od)
        app.close()
```

- [ ] **Step 4: 기동 + cmd_vel/odom 검증**

```bash
ps aux | grep -iE "isaac" | grep -v grep
bash /home/rokey/cobot3_ws/isaacpjt/Isaac_envo/dock_lift_runner.sh --headless &
# READY 후 (외부 env 터미널):
timeout 6 ros2 topic echo --once /robot_rear/odom --field pose.pose.position
timeout 6 ros2 topic pub -r 10 /robot_rear/cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.3}}' >/dev/null &
sleep 3; timeout 4 ros2 topic echo --once /robot_rear/odom --field pose.pose.position.z
```
Expected: odom z가 접근 시작값에서 +방향으로 증가(rear는 +Z로 진입). robot_rear가 차량 쪽으로
움직임. (안 움직이면 hub drive/휠 인덱스 확인. 방향 반대면 facing/스폰 yaw 재확인.)

- [ ] **Step 5: Commit**

```bash
cd /home/rokey/cobot3_ws
git add isaacpjt/Isaac_envo/dock_lift_runner.py isaacpjt/Isaac_envo/dock_lift_runner.sh
git commit -m "feat: ROS2 cmd_vel/odom bridge in dock_lift_runner (per-robot mecanum drive)"
```

---

### Task 3: /robot_N/arm_control 서비스 (러너)

**Files:**
- Modify: `isaacpjt/Isaac_envo/dock_lift_runner.py`

**Interfaces:**
- Consumes: `arts`, `ARM_TARGETS`, ROS2 `node`
- Produces: `/robot_rear/arm_control`·`/robot_front/arm_control` (std_srvs/SetBool):
  data=true → 팔 ARM_TARGETS로 전개(파지/리프트), 다 전개 후 success. false → 접기.

- [ ] **Step 1: 팔 구동 헬퍼 + 서비스 등록** — cmd_vel 구독 등록 뒤, 대기 루프 앞:

```python
        from std_srvs.srv import SetBool
        ARM_TOL_DEG = 3.0
        arm_idx = {k: {n: arts[k].dof_names.index(n) for n in ARM_TARGETS} for k in arts}
        arm_cmd = {k: 0.0 for k in arts}   # 목표 scale (0=접힘,1=전개). 매 틱 램프 적용.
        arm_applied = {k: 0.0 for k in arts}

        def apply_arms(key):
            """매 틱: arm_applied 를 arm_cmd 로 0.02씩 램프하며 조인트 타깃 설정."""
            tgt = arm_cmd[key]; cur = arm_applied[key]
            if abs(tgt - cur) > 1e-4:
                cur += max(-0.02, min(0.02, tgt - cur))
                arm_applied[key] = cur
            pos = np.array(arts[key].get_joint_positions(), dtype=np.float32, copy=True)
            for n, deg in ARM_TARGETS.items():
                v = math.radians(deg * cur)
                if pos.ndim == 2: pos[0, arm_idx[key][n]] = v
                else: pos[arm_idx[key][n]] = v
            arts[key].set_joint_position_targets(pos)

        def arms_reached(key, scale):
            cur = arts[key].get_joint_positions()
            cur = cur[0] if getattr(cur, "ndim", 1) == 2 else cur
            return all(abs(math.degrees(float(cur[arm_idx[key][n]])) - deg*scale) < ARM_TOL_DEG
                       for n, deg in ARM_TARGETS.items())

        def make_arm_cb(key):
            def cb(req, resp):
                arm_cmd[key] = 1.0 if req.data else 0.0
                # 서비스는 즉시 목표만 설정하고, 도달은 오케스트레이터가 재확인.
                resp.success = True
                resp.message = "arm target set: " + ("open" if req.data else "fold")
                print(f"ARM_CMD robot_{key} -> {'open' if req.data else 'fold'}", flush=True)
                return resp
            return cb
        for key in arts:
            node.create_service(SetBool, f"/robot_{key}/arm_control", make_arm_cb(key))
```

- [ ] **Step 2: 대기 루프에서 매 틱 팔 램프 적용** — `while app.is_running():` 안, odom 발행과
같은 for 루프에 추가:

```python
                apply_arms(key)
```

- [ ] **Step 3: 기동 + 팔 서비스 검증**

```bash
ps aux | grep -iE "isaac" | grep -v grep
bash /home/rokey/cobot3_ws/isaacpjt/Isaac_envo/dock_lift_runner.sh --gui &
# READY 후 (외부 env):
ros2 service call /robot_rear/arm_control std_srvs/srv/SetBool "{data: true}"
```
Expected: 응답 success=true, GUI에서 robot_rear 팔 4개가 ±90°로 펼쳐짐. `{data: false}`로 접힘.

- [ ] **Step 4: Commit**

```bash
cd /home/rokey/cobot3_ws
git add isaacpjt/Isaac_envo/dock_lift_runner.py
git commit -m "feat: per-robot arm_control service (SetBool, ARM_TARGETS ramp) in dock_lift_runner"
```

---

### Task 4: dock_lift_state 순수 상태기계 + 단위 테스트

**Files:**
- Create: `src/parkbot_aruco/parkbot_aruco/dock_lift_state.py`
- Test: `src/parkbot_aruco/test/test_dock_lift_state.py`

**Interfaces:**
- Produces: `class DockLiftPlan` — 순차 진입·파지·운반 전이를 결정하는 순수 로직.
  - `DockLiftPlan(rear_target_z, front_target_z, center_x, carry_distance)`
  - `.ingress_cmd(phase, rear_z, front_z) -> dict` : 현 단계에서 각 로봇의 (vx) 명령.
    반환 `{"robot_rear": vx, "robot_front": vx}` (로봇 로컬 forward, +면 차량 쪽).
  - `.rear_arrived(rear_z) -> bool`, `.front_arrived(front_z) -> bool` : POS_TOL 이내.
  - `.next_phase(phase, rear_z, front_z, car_lift_m, carried_z) -> str` : 상태 전이.
    단계: `"ingress_rear" -> "ingress_front" -> "grip" -> "carry" -> "done"` / `"fail"`.

- [ ] **Step 1: 실패하는 테스트 작성** — `test/test_dock_lift_state.py`:

```python
"""순차 도킹 상태기계 (ROS/Isaac 불필요)."""
from parkbot_aruco.dock_lift_state import DockLiftPlan

def _plan():
    # rear 축 z=-1.36, front 축 z=+1.36 (Coupe 축거 2.715 근사), center_x=0
    return DockLiftPlan(rear_target_z=-1.36, front_target_z=1.36,
                        center_x=0.0, carry_distance=1.0)

def test_ingress_rear_drives_rear_only():
    p = _plan()
    cmd = p.ingress_cmd("ingress_rear", rear_z=-3.11, front_z=3.11)
    assert cmd["robot_rear"] > 0     # rear 전진(차량 쪽)
    assert cmd["robot_front"] == 0   # front 는 순차 — 아직 대기

def test_rear_arrival_advances_to_front():
    p = _plan()
    # rear 목표(-1.36) 도달 → ingress_front 로
    nxt = p.next_phase("ingress_rear", rear_z=-1.35, front_z=3.11,
                       car_lift_m=0.0, carried_z=0.0)
    assert nxt == "ingress_front"

def test_both_arrived_advances_to_grip():
    p = _plan()
    nxt = p.next_phase("ingress_front", rear_z=-1.35, front_z=1.35,
                       car_lift_m=0.0, carried_z=0.0)
    assert nxt == "grip"

def test_grip_lift_advances_to_carry():
    p = _plan()
    nxt = p.next_phase("grip", rear_z=-1.36, front_z=1.36,
                       car_lift_m=0.03, carried_z=0.0)   # 실제 상승
    assert nxt == "carry"

def test_grip_no_lift_fails():
    p = _plan()
    nxt = p.next_phase("grip", rear_z=-1.36, front_z=1.36,
                       car_lift_m=0.001, carried_z=0.0)   # 안 들림
    assert nxt == "fail"

def test_carry_distance_reached_done():
    p = _plan()
    nxt = p.next_phase("carry", rear_z=-1.36, front_z=1.36,
                       car_lift_m=0.03, carried_z=1.05)   # 목표 1.0 초과
    assert nxt == "done"
```

- [ ] **Step 2: 실패 확인**

```bash
cd /home/rokey/cobot3_ws/src/parkbot_aruco
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest test/test_dock_lift_state.py -q 2>&1 | tail -2
```
Expected: ImportError (`dock_lift_state` 없음)

- [ ] **Step 3: 구현** — `parkbot_aruco/dock_lift_state.py`:

```python
"""ROS2 촉발 도킹·리프트·운반 순수 상태기계 (numpy·표준 라이브러리만).

단계: ingress_rear -> ingress_front -> grip -> carry -> done / fail.
좌표: 차량 길이축 = world z. 로봇은 자기 target_z 를 향해 로컬 forward(vx>0) 직진.
rear 로봇은 -z 쪽에서 +z 로, front 로봇은 +z 쪽에서 -z 로 진입한다.
"""
POS_TOL_M = 0.05
INGRESS_SPEED = 0.30
CARRY_SPEED = 0.35
LIFT_MIN_M = 0.025


class DockLiftPlan:
    def __init__(self, rear_target_z, front_target_z, center_x, carry_distance):
        self.rear_tz = float(rear_target_z)
        self.front_tz = float(front_target_z)
        self.center_x = float(center_x)
        self.carry_distance = float(carry_distance)

    def rear_arrived(self, rear_z):
        return abs(rear_z - self.rear_tz) <= POS_TOL_M

    def front_arrived(self, front_z):
        return abs(front_z - self.front_tz) <= POS_TOL_M

    def ingress_cmd(self, phase, rear_z, front_z):
        cmd = {"robot_rear": 0.0, "robot_front": 0.0}
        if phase == "ingress_rear" and not self.rear_arrived(rear_z):
            cmd["robot_rear"] = INGRESS_SPEED    # 로컬 forward(+X->+Z) = 차량 쪽
        elif phase == "ingress_front":
            if not self.rear_arrived(rear_z):
                cmd["robot_rear"] = INGRESS_SPEED   # 미세 보정 유지
            if not self.front_arrived(front_z):
                cmd["robot_front"] = INGRESS_SPEED  # 로컬 forward(-X->-Z) = 차량 쪽
        return cmd

    def next_phase(self, phase, rear_z, front_z, car_lift_m, carried_z):
        if phase == "ingress_rear":
            return "ingress_front" if self.rear_arrived(rear_z) else "ingress_rear"
        if phase == "ingress_front":
            if self.rear_arrived(rear_z) and self.front_arrived(front_z):
                return "grip"
            return "ingress_front"
        if phase == "grip":
            return "carry" if car_lift_m >= LIFT_MIN_M else "fail"
        if phase == "carry":
            return "done" if carried_z >= self.carry_distance else "carry"
        return phase
```

- [ ] **Step 4: 통과 확인**

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest test/test_dock_lift_state.py -q 2>&1 | tail -2
```
Expected: `6 passed`

- [ ] **Step 5: Commit**

```bash
cd /home/rokey/cobot3_ws
git add src/parkbot_aruco/parkbot_aruco/dock_lift_state.py src/parkbot_aruco/test/test_dock_lift_state.py
git commit -m "feat: dock_lift_state sequential docking state machine (pure logic, 6 tests)"
```

---

### Task 5: dock_lift_mission 오케스트레이터 — /dock_lift 서비스 + 순차 진입

**Files:**
- Create: `isaacpjt/Isaac_envo/dock_lift_mission.py`

**Interfaces:**
- Consumes: `DockLiftPlan`(Task 4), `/robot_N/odom`·`/robot_N/cmd_vel`(Task 2),
  `/robot_N/arm_control`(Task 3), 러너 콘솔의 `DOCK_STAGE_READY`(축 좌표)
- Produces: `/dock_lift`(std_srvs/Trigger) 서비스. 호출 시 진입까지 구동(파지·운반은 Task 6).
  축 좌표는 파라미터로 받는다(러너 로그에서 읽어 전달): `rear_target_z`, `front_target_z`, `center_x`.

- [ ] **Step 1: 오케스트레이터 노드(진입 단계까지)** — `dock_lift_mission.py`:

```python
#!/usr/bin/env python3
"""ROS2 촉발 도킹·리프트·운반 오케스트레이터 (외부 ROS2 Humble).

/dock_lift(Trigger) 요청 하나로 순차 진입 → 파지 → 운반. Isaac 러너의
cmd_vel/odom/arm_control 만 쓴다. 축 좌표는 파라미터(러너 DOCK_STAGE_READY 값).
"""
import math
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_srvs.srv import Trigger, SetBool
from rclpy.node import Node

sys.path.insert(0, "/home/rokey/cobot3_ws/src/parkbot_aruco")
from parkbot_aruco.dock_lift_state import DockLiftPlan, CARRY_SPEED

ROBOTS = ("robot_rear", "robot_front")
FACING = {"robot_rear": +1, "robot_front": -1}   # 로컬 forward -> world ±Z
CONTROL_HZ = 20.0
STEP_TIMEOUT = 60.0


class DockLiftMission(Node):
    def __init__(self):
        super().__init__("dock_lift_mission")
        self.declare_parameter("rear_target_z", -1.36)
        self.declare_parameter("front_target_z", 1.36)
        self.declare_parameter("center_x", 0.0)
        g = lambda k: self.get_parameter(k).value
        self.plan = DockLiftPlan(g("rear_target_z"), g("front_target_z"),
                                 g("center_x"), carry_distance=1.0)
        self.z = {r: None for r in ROBOTS}
        for r in ROBOTS:
            self.create_subscription(Odometry, f"/{r}/odom",
                                     lambda m, rid=r: self._odom(rid, m), 10)
        self.cmd = {r: self.create_publisher(Twist, f"/{r}/cmd_vel", 10) for r in ROBOTS}
        self.arm = {r: self.create_client(SetBool, f"/{r}/arm_control") for r in ROBOTS}
        self.create_service(Trigger, "/dock_lift", self._on_dock_lift)
        self.get_logger().info("dock_lift_mission 준비 — /dock_lift 대기")

    def _odom(self, rid, m):
        self.z[rid] = m.pose.pose.position.z

    def _pub(self, rid, vx):
        t = Twist(); t.linear.x = float(vx)   # 로컬 forward; 러너가 메카넘 IK 처리
        self.cmd[rid].publish(t)

    def _stop_all(self):
        for r in ROBOTS:
            self._pub(r, 0.0)

    def _wait_odom(self, timeout=15.0):
        end = time.time() + timeout
        while time.time() < end and any(self.z[r] is None for r in ROBOTS):
            rclpy.spin_once(self, timeout_sec=0.2)
        return all(self.z[r] is not None for r in ROBOTS)

    def _run_ingress(self):
        """순차 진입: ingress_rear -> ingress_front. 상태기계로 명령 결정."""
        phase = "ingress_rear"
        end = time.time() + STEP_TIMEOUT
        while phase in ("ingress_rear", "ingress_front") and time.time() < end:
            cmd = self.plan.ingress_cmd(phase, self.z["robot_rear"], self.z["robot_front"])
            for r in ROBOTS:
                self._pub(r, cmd[r])
            rclpy.spin_once(self, timeout_sec=1.0 / CONTROL_HZ)
            phase = self.plan.next_phase(phase, self.z["robot_rear"],
                                         self.z["robot_front"], 0.0, 0.0)
        self._stop_all()
        return phase == "grip"

    def _on_dock_lift(self, req, resp):
        if not self._wait_odom():
            resp.success = False; resp.message = "odom 미수신"; return resp
        self.get_logger().info("순차 진입 시작")
        if not self._run_ingress():
            self._stop_all()
            resp.success = False; resp.message = "진입 타임아웃"; return resp
        # (Task 6: 파지 + 운반)
        resp.success = True
        resp.message = f"진입 완료 rear_z={self.z['robot_rear']:.2f} front_z={self.z['robot_front']:.2f}"
        return resp


def main():
    rclpy.init()
    node = DockLiftMission()
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

- [ ] **Step 2: 진입 실동작 검증** — 러너 GUI 기동 후 축 좌표를 로그에서 읽어 전달:

```bash
ps aux | grep -iE "isaac" | grep -v grep
bash /home/rokey/cobot3_ws/isaacpjt/Isaac_envo/dock_lift_runner.sh --gui > /tmp/dock_runner.log 2>&1 &
# DOCK_STAGE_READY 줄에서 rear_z/front_z/center_x 읽기:
grep DOCK_STAGE_READY /tmp/dock_runner.log
# 외부 env 터미널에서 (읽은 값 대입):
python3 isaacpjt/Isaac_envo/dock_lift_mission.py --ros-args \
  -p rear_target_z:=<rear_z> -p front_target_z:=<front_z> -p center_x:=<center_x> &
sleep 3
ros2 service call /dock_lift std_srvs/srv/Trigger {}
```
Expected: 응답 `success=true, "진입 완료 ..."`. GUI에서 두 로봇이 **순차로**(rear 먼저, 그다음
front) 차량 하부로 직진 진입해 축 아래 정지. 회전 없음.
- 로봇이 차량에 부딪혀 못 들어가면: 콜라이더 수정 확인(Task 1) + 진입 정지 위치(target_z) 미세 조정.
- 방향 반대로 가면: `_robot_matrix` yaw 부호 / cmd_vel forward 매핑 재확인.

- [ ] **Step 3: Commit**

```bash
cd /home/rokey/cobot3_ws
git add isaacpjt/Isaac_envo/dock_lift_mission.py
git commit -m "feat: dock_lift_mission orchestrator — /dock_lift service + sequential cmd_vel ingress"
```

---

### Task 6: 파지 + 리프트 + 운반 + 전체 E2E 검증

**Files:**
- Modify: `isaacpjt/Isaac_envo/dock_lift_mission.py` (`_on_dock_lift`에 grip·carry 추가)
- Modify: `isaacpjt/Isaac_envo/dock_lift_runner.py` (차량 world z 발행 → 리프트/운반 판정용)

**Interfaces:**
- Consumes: `/robot_N/arm_control`(Task 3), `DockLiftPlan.next_phase`의 grip/carry 전이(Task 4)
- Produces: `/dock_lift` 호출 하나로 진입→파지→리프트→운반 전 과정 + `success` 응답.
  러너가 `/vehicle/pose`(PoseStamped, 차량 world 위치) 발행 → 오케스트레이터가 리프트/운반 판정.

- [ ] **Step 1: 러너가 차량 pose 발행** — `dock_lift_runner.py`의 odom 발행 루프에 추가
(차량 리지드바디 world pose). ROS2 노드 생성부에 publisher 추가:

```python
        from geometry_msgs.msg import PoseStamped
        veh_pub = node.create_publisher(PoseStamped, "/vehicle/pose", 10)
        from isaacsim.core.prims import RigidPrim
        veh_rb = RigidPrim("/World/Vehicle")
```
대기 루프 안(로봇 for 루프 뒤):

```python
            vp = np.asarray(veh_rb.get_world_poses()[0]).reshape(-1)[:3]
            ps = PoseStamped()
            ps.header.stamp = node.get_clock().now().to_msg()
            ps.header.frame_id = "map"
            ps.pose.position.x = float(vp[0]); ps.pose.position.y = float(vp[1])
            ps.pose.position.z = float(vp[2])
            veh_pub.publish(ps)
```

- [ ] **Step 2: 오케스트레이터에 차량 pose 구독 + grip/carry 단계** —
`dock_lift_mission.py`의 `__init__`에 구독 추가:

```python
        from geometry_msgs.msg import PoseStamped
        self.veh_y = None; self.veh_z = None
        self.create_subscription(PoseStamped, "/vehicle/pose", self._veh, 10)
```
콜백 + grip/carry 메서드:

```python
    def _veh(self, m):
        self.veh_y = m.pose.position.y   # world Y = 세로 높이(리프트)
        self.veh_z = m.pose.position.z   # world Z = 운반 진행축

    def _call_arms(self, opening):
        for r in ROBOTS:
            if not self.arm[r].wait_for_service(timeout_sec=5.0):
                return False
        futs = []
        for r in ROBOTS:
            req = SetBool.Request(); req.data = opening
            futs.append(self.arm[r].call_async(req))
        for f in futs:
            rclpy.spin_until_future_complete(self, f, timeout_sec=5.0)
        return all(f.result() and f.result().success for f in futs)

    def _grip_and_check(self):
        """팔 전개 서비스 호출 후, 팔 램프+리프트가 일어날 시간을 준 뒤 상승량 측정."""
        y0 = self.veh_y
        if not self._call_arms(True):
            return 0.0
        end = time.time() + 12.0   # 램프(0.02/틱)+정착 대기
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.05)
        return (self.veh_y - y0) if (self.veh_y is not None and y0 is not None) else 0.0

    def _carry(self):
        """편대 직진 운반: 두 로봇 로컬 forward 로 차량을 world +Z 로 밀어 이동."""
        z0 = self.veh_z
        end = time.time() + STEP_TIMEOUT
        while time.time() < end:
            # rear facing +1 -> forward=+Z, front facing -1 -> forward=+Z 되려면 vx 부호 반대
            self._pub("robot_rear", CARRY_SPEED)
            self._pub("robot_front", -CARRY_SPEED)   # facing -1 이라 forward 반대
            rclpy.spin_once(self, timeout_sec=1.0 / CONTROL_HZ)
            carried = (self.veh_z - z0) if (self.veh_z is not None and z0 is not None) else 0.0
            if carried >= self.plan.carry_distance:
                break
        self._stop_all()
        return (self.veh_z - z0) if (self.veh_z is not None and z0 is not None) else 0.0
```

- [ ] **Step 3: `_on_dock_lift`에 grip/carry 연결** — 진입 성공 뒤:

```python
        car_lift = self._grip_and_check()
        if self.plan.next_phase("grip", self.z["robot_rear"], self.z["robot_front"],
                                car_lift, 0.0) != "carry":
            self._stop_all()
            resp.success = False
            resp.message = f"리프트 실패 car_lift={car_lift:.4f}m"
            return resp
        carried = self._carry()
        resp.success = carried >= self.plan.carry_distance * 0.8
        resp.message = (f"완료: 리프트 {car_lift:.3f}m, 운반 {carried:.3f}m")
        self.get_logger().info(resp.message)
        return resp
```
(진입 성공 응답을 리턴하던 Task 5의 마지막 `resp.success=True` 블록을 위 코드로 교체.)

- [ ] **Step 4: 전체 E2E — GUI 시연 + 모션품질 검증**

```bash
ps aux | grep -iE "isaac" | grep -v grep
bash /home/rokey/cobot3_ws/isaacpjt/Isaac_envo/dock_lift_runner.sh --gui > /tmp/dock_runner.log 2>&1 &
grep DOCK_STAGE_READY /tmp/dock_runner.log   # 축 좌표 확인
# 외부 env: 오케스트레이터 + 모션품질 프로브 + 요청
python3 isaacpjt/Isaac_envo/dock_lift_mission.py --ros-args \
  -p rear_target_z:=<rear_z> -p front_target_z:=<front_z> -p center_x:=<center_x> &
sleep 3
# 모션품질(로봇 토픽 이름이 robot_rear/robot_front 라 프로브 ROBOTS 상수 조정 필요 —
# motion_quality_probe.py 를 복사해 ROBOTS=("robot_rear","robot_front") 로 바꿔 사용)
ros2 service call /dock_lift std_srvs/srv/Trigger {}
```
Expected: 응답 `success=true, "완료: 리프트 ≥0.025m, 운반 ≥0.8m"`. GUI에서 순차 진입 → 팔 파지 →
차량 상승 → 두 로봇이 차 든 채 이동. **모션품질 PASS**(진입·운반 중 두 로봇 발레·점프 없음).
- 리프트 안 되면(car_lift 낮음): 진입 정지 위치 정밀도(target_z)·낮춘 바퀴 영향 점검. 팔 롤러가
  타이어에 정확히 닿는 z 인지 GUI로 확인 후 target_z 미세 조정.
- 운반 중 차가 안 따라오면: facing 부호(carry vx)·파지 마찰 확인.

- [ ] **Step 5: Commit**

```bash
cd /home/rokey/cobot3_ws
git add isaacpjt/Isaac_envo/dock_lift_mission.py isaacpjt/Isaac_envo/dock_lift_runner.py
git commit -m "feat: dock_lift grip+lift+carry — single /dock_lift request runs full flow (verified lift+carry)"
```

---

## 완료 기준 (Plan 3 전체)

1. `dock_lift_runner` 헤드리스 물리 안정 + cmd_vel/odom/arm_control ROS2 노출
2. `dock_lift_state` 단위 테스트 6건 통과
3. **`ros2 service call /dock_lift` 한 번 → 순차 진입 → 파지 → 리프트(≥0.025m) → 운반(≥0.8m)**
   전 과정 자동 + success 응답 (핵심 성공 기준 = 사용자 요구)
4. 모션품질 PASS — 진입·운반 중 두 로봇 발레·점프·이탈 없음

## 리스크 / 튜닝 포인트 (실측 조정 예상)

- **진입 정지 위치 정밀도**가 리프트 성패를 가른다. 하드코딩 target_z + P제어로 맞추되, 팔
  롤러가 타이어에 닿는 정확한 z는 GUI 실측으로 미세 조정(Plan 2처럼 1~2회 반복 예상).
- 낮춘 바퀴(+3cm)로 팔-타이어 접촉 높이가 carry demo와 달라졌을 수 있음 — 리프트 재현 재확인.
- 순차 진입 타임아웃/게이트 값은 실측 조정.
- 차량 콜라이더 수정 프림 이름 매칭 실패 시 트리 출력로 이름 확정.
