#!/usr/bin/env python3
"""인계장 전체 환경 ROS2 촉발 도킹·리프트·오미 운반 Isaac 러너.

씬: 주차장 전체 환경(parking_environment_with_markers.usd) + 인계 베이의
Pickup(콜라이더 수정·그립 재질) + 낮춘 바퀴 로봇 2대(West 대기 도크).
프로그램 켜두고 대기 — 외부 오케스트레이터(dock_lift_handoff_mission.py)가 구동.

ROS2: /robot_N/cmd_vel 구독, /robot_N/odom 발행(x,y=높이,z,yaw),
      /robot_N/arm_control 서비스, /vehicle/pose 발행.
      /sim_reset(Trigger): 씬 재로딩 없이 도크 초기 상태로 순간이동 리셋.
      /sim_checkpoint_staged(Trigger): 로봇을 게이트 통과 직후 대기 위치로 순간이동
        — 느린 게이트 통과 구간만 건너뛰고 회전·진입·파지·운반은 실제로 재생
        (차 밑까지 직접 텔레포트하면 차체와 겹쳐 물리가 튕겨나가는 게 확인돼 이렇게
        바꿈). mission의 /dock_lift_from_staged 와 짝.

실행: dock_lift_handoff_runner.sh [--gui] [--headless-test]
"""
import math
import os
import sys
from pathlib import Path

WORK_DIR = Path(__file__).resolve().parent
PARKING_USD = WORK_DIR / "parking" / "parking_environment_with_markers.usd"
ROBOT_USD = (WORK_DIR.parent / "hwia_parking_robot_final_caster_package"
             / "hwia_depth_cam_mecha_roller_lowered.usd")
VEHICLES_USD = WORK_DIR / "fab_vehicles.usd"
ISAAC_PYTHON = Path("/home/rokey/dev_ws/isaac_sim/isaacsim/_build/linux-x86_64/release/python.sh")
BRIDGE_RCLPY = Path("/home/rokey/dev_ws/isaac_sim/isaacsim/_build/linux-x86_64/release"
                    "/exts/isaacsim.ros2.bridge/humble/rclpy")

TARGET_VEHICLE = "Pickup"
VEHICLE_PATH = f"/World/VehicleAsset/Vehicles/{TARGET_VEHICLE}"
FAB_VEHICLE_TYPES = ("Compact", "Coupe", "Hatchback", "Minivan", "Offroad",
                     "Pickup", "Sedan", "Sport", "SUV", "Wagon")
TARGET_COLLIDER_WIDTH = 0.30
# Coupe 위치: 인계장 입구(서쪽 개구부 z∈[-4.5,4.5]) 정면 z=0, 인계장 중앙.
# 로봇이 개구부에서 서진하면 Pickup이 정면. 양끝(북=앞축, 남=뒷축) 접근에 공간 충분.
# (로봇 높이 0.18m > 차체 언더바디 0.163m 라 차 밑 장거리 통과 불가 → 각 축을 바깥
#  끝에서 짧게(~1.3m, 휠웰 간격) 진입해야 함. 차량을 중앙에 둬야 남/북 공간 확보.)
VEHICLE_POS = (-29.6, 0.035, 0.0)
HANDOFF_WAGON = "/World/ParkingVehicles/HandoffQueue/H2_Wagon"

ARM_TARGETS = {
    "arm_left_front_joint": 90.0, "arm_left_rear_joint": -90.0,
    "arm_right_front_joint": -90.0, "arm_right_rear_joint": 90.0,
}
VEHICLE_WHEELS = ("FrontLeftWheel", "FrontRightWheel", "RearLeftWheel", "RearRightWheel")

# 로봇: robot_rear -> West_B 도크, robot_front -> West_A 도크. 초기엔 +X 향함(도크 기본).
ROBOTS = {
    "rear":  {"xform": "/World/Robots/robot_rear",
              "dock": "/World/ParkingEnvironment/RobotServiceArea/West_B_WaitingDock"},
    "front": {"xform": "/World/Robots/robot_front",
              "dock": "/World/ParkingEnvironment/RobotServiceArea/West_A_WaitingDock"},
}
AXLE = {}


def _restart_with_isaac_python():
    if os.environ.get("CARB_APP_PATH"):
        return
    os.execv(str(ISAAC_PYTHON), [str(ISAAC_PYTHON), str(Path(__file__).resolve()), *sys.argv[1:]])


def _fix_vehicle_colliders(stage, vehicle_path):
    from pxr import UsdGeom
    fixed = 0
    for wheel_name in VEHICLE_WHEELS:
        col = stage.GetPrimAtPath(f"{vehicle_path}/{wheel_name}/Collision")
        if col and col.IsValid() and col.IsA(UsdGeom.Cylinder):
            UsdGeom.Cylinder(col).GetHeightAttr().Set(float(TARGET_COLLIDER_WIDTH))
            fixed += 1
    return fixed


def _grip_material(stage):
    from pxr import UsdShade, UsdPhysics
    mat = UsdShade.Material.Define(stage, "/World/Looks/GripDock")
    api = UsdPhysics.MaterialAPI.Apply(mat.GetPrim())
    api.CreateStaticFrictionAttr(1.2)
    api.CreateDynamicFrictionAttr(1.0)
    api.CreateRestitutionAttr(0.0)
    return mat


def _dock_position(stage, dock_path):
    prim = stage.GetPrimAtPath(dock_path)
    if not prim or not prim.IsValid():
        raise RuntimeError(f"도크 없음: {dock_path}")
    v = prim.GetAttribute("robot:dockPose").Get()
    if v is None:
        raise RuntimeError(f"dockPose 없음: {dock_path}")
    return tuple(float(c) for c in v)


def _place_robot_dock(UsdGeom, Gf, prim, pos):
    """도크에 로봇을 세운다(Z-up->Y-up). 초기 방향은 +X(도크 기본, 회전은 미션이)."""
    xf = UsdGeom.Xformable(prim)
    xf.ClearXformOpOrder()
    xf.AddTranslateOp().Set(Gf.Vec3d(pos[0], 0.06, pos[2]))
    xf.AddRotateXOp().Set(-90.0)


def _apply_vehicle_context(stage):
    """주차장 씬의 PhysicsScene에 PhysX Vehicle 컨텍스트 + 리프트 안정화 설정을 붙인다.

    주차장 에셋의 차량(주차칸·인계장)은 전부 PhysX Vehicle 프림인데 에셋의
    PhysicsScene 에는 VehicleContext 가 없어(applied schemas=[]) play 시 구동계가
    기본값으로 오작동해 폭발한다(=사용자가 본 '차가 벽으로 떨어짐'). Plan 3 러너와
    동일 설정을 세션 레이어에서 덮어 적용한다.
    """
    from pxr import UsdPhysics
    from pxr import PhysxSchema
    sc = stage.GetPrimAtPath("/World/PhysicsScene")
    if not sc or not sc.IsValid():
        raise RuntimeError("주차장 PhysicsScene 없음 — vehicle context 적용 불가")
    px = PhysxSchema.PhysxSceneAPI.Apply(sc)
    px.CreateBroadphaseTypeAttr("GPU")
    px.CreateSolverTypeAttr("TGS")
    px.CreateEnableCCDAttr(True)
    px.CreateEnableStabilizationAttr(True)
    px.CreateEnableGPUDynamicsAttr(True)
    px.CreateTimeStepsPerSecondAttr(240)
    vctx = PhysxSchema.PhysxVehicleContextAPI.Apply(sc)
    vctx.CreateUpdateModeAttr(PhysxSchema.Tokens.velocityChange)
    vctx.CreateVerticalAxisAttr(PhysxSchema.Tokens.posY)
    vctx.CreateLongitudinalAxisAttr(PhysxSchema.Tokens.posZ)


def build_stage(app):
    from pxr import Gf, UsdGeom, UsdPhysics
    import omni.usd

    ctx = omni.usd.get_context()
    ctx.new_stage()
    stage = ctx.get_stage()
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.y)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    # 주차장 전체 환경을 서브레이어로(물리씬·인계장·차량 vehicle context 포함).
    # 런타임 익명 stage라 절대경로(상대경로는 CWD에 의존해 깨짐).
    stage.GetRootLayer().subLayerPaths.append(str(PARKING_USD))
    world = stage.GetPrimAtPath("/World")
    if not world or not world.IsValid():
        raise RuntimeError("서브레이어에서 /World를 찾지 못했습니다.")
    stage.SetDefaultPrim(world)
    # 주차장 PhysicsScene 에 vehicle context 보강(차량 폭발 방지).
    _apply_vehicle_context(stage)
    for _ in range(30):
        app.update()

    # 인계 베이의 Wagon을 끄고 그 자리에 Pickup.
    wagon = stage.GetPrimAtPath(HANDOFF_WAGON)
    if wagon and wagon.IsValid():
        wagon.SetActive(False)

    _grip_material(stage)

    # fab 전체 참조 후 Pickup만 인계 베이에 재배치(재질·차량물리 바인딩 유지).
    asset = stage.DefinePrim("/World/VehicleAsset", "Xform")
    asset.GetReferences().AddReference(str(VEHICLES_USD))
    for _ in range(20):
        app.update()
    for name in ("PhysicsScene", "DriveGround", "FabLighting", "Cylinder001"):
        p = stage.GetPrimAtPath(f"/World/VehicleAsset/{name}")
        if p and p.IsValid():
            p.SetActive(False)
    for vt in FAB_VEHICLE_TYPES:
        if vt == TARGET_VEHICLE:
            continue
        p = stage.GetPrimAtPath(f"/World/VehicleAsset/Vehicles/{vt}")
        if p and p.IsValid():
            p.SetActive(False)

    coupe = stage.GetPrimAtPath(VEHICLE_PATH)
    if not coupe or not coupe.IsValid():
        raise RuntimeError(f"Pickup 없음: {VEHICLE_PATH}")
    xf = UsdGeom.Xformable(coupe)
    xf.ClearXformOpOrder()
    m = Gf.Matrix4d(1.0)
    m.SetTranslateOnly(Gf.Vec3d(*VEHICLE_POS))   # 인계 베이, yaw=0(길이축 z)
    xf.AddTransformOp().Set(m)
    for _ in range(20):
        app.update()

    n_fixed = _fix_vehicle_colliders(stage, VEHICLE_PATH)

    cache = UsdGeom.XformCache()
    centers = {}
    for wn in VEHICLE_WHEELS:
        w = stage.GetPrimAtPath(f"{VEHICLE_PATH}/{wn}")
        if not w.IsValid():
            raise RuntimeError(f"휠 없음: {VEHICLE_PATH}/{wn}")
        centers[wn] = cache.GetLocalToWorldTransform(w).ExtractTranslation()
    front_z = (centers["FrontLeftWheel"][2] + centers["FrontRightWheel"][2]) * 0.5
    rear_z = (centers["RearLeftWheel"][2] + centers["RearRightWheel"][2]) * 0.5
    center_x = sum(c[0] for c in centers.values()) / 4.0
    AXLE.update(rear_z=min(front_z, rear_z), front_z=max(front_z, rear_z),
                center_x=center_x)

    # 로봇 2대 도크 배치
    UsdGeom.Xform.Define(stage, "/World/Robots")
    for key, cfg in ROBOTS.items():
        pos = _dock_position(stage, cfg["dock"])
        r = stage.DefinePrim(cfg["xform"], "Xform")
        r.GetReferences().AddReference(str(ROBOT_USD))
        _place_robot_dock(UsdGeom, Gf, r, pos)
    for _ in range(30):
        app.update()
    print(f"DOCK_STAGE_READY vehicle_pos={VEHICLE_POS} axle rear_z={AXLE['rear_z']:.3f} "
          f"front_z={AXLE['front_z']:.3f} center_x={center_x:.3f} colliders_fixed={n_fixed}",
          flush=True)
    return stage


def main():
    _restart_with_isaac_python()
    from isaacsim import SimulationApp
    headless = "--gui" not in sys.argv[1:]
    app = SimulationApp({"headless": headless, "width": 1280, "height": 800})

    # 실시간 동기화(대기) 스위치만 끈다 — 물리 스텝 크기(useFixedTimeStepping)는
    # 그대로 둬서 정확도는 안 건드리고, "다음 스텝까지 실제 시간을 기다리는"
    # 부분만 없앤다. GUI(--gui)에서도 그대로 적용됨(headless 전용 아님).
    import carb.settings
    settings = carb.settings.get_settings()
    settings.set_bool("/app/runLoops/main/manualModeEnabled", False)
    settings.set_bool("/exts/isaacsim.core.throttling/enable_manualmode", False)

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

        # 초기(도크) 포즈 스냅샷 — 재시작(씬 리로드) 없이 리셋할 때 되돌아갈 기준점.
        DOCK_SNAPSHOT = {k: tuple(np.array(v, copy=True) for v in a.get_world_poses())
                         for k, a in arts.items()}

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
                if buf.ndim == 2:
                    buf[0, i] = om
                else:
                    buf[i] = om
            arts[key].set_joint_velocity_targets(buf)

        if "--headless-test" in sys.argv[1:]:
            from isaacsim.core.prims import RigidPrim
            def _p(a):
                return np.asarray(a.get_world_poses()[0]).reshape(-1)[:3]
            coupe_rb = RigidPrim(VEHICLE_PATH)
            def _coupe_p():
                return np.asarray(coupe_rb.get_world_poses()[0]).reshape(-1)[:3]
            p0 = {k: _p(a) for k, a in arts.items()}
            c0 = _coupe_p()
            for _ in range(180):
                app.update()
            p1 = {k: _p(a) for k, a in arts.items()}
            c1 = _coupe_p()
            disp = {k: float(np.linalg.norm(p1[k] - p0[k])) for k in arts}
            coupe_disp = float(np.linalg.norm(c1 - c0))
            # 차량 폭발 판정: vehicle context 없으면 play 시 수 m 튐 → coupe_disp 큼.
            robots_ok = all(d < 0.35 for d in disp.values())
            coupe_ok = coupe_disp < 0.30
            ok = robots_ok and coupe_ok
            print(f"DOCK_PHYSICS_TEST={'PASS' if ok else 'FAIL'} "
                  f"robot_disp={ {k: round(v,4) for k,v in disp.items()} } "
                  f"coupe_disp={coupe_disp:.4f} coupe_pos0={ [round(float(v),2) for v in c0] } "
                  f"coupe_pos1={ [round(float(v),2) for v in c1] }", flush=True)
            app.close()
            return

        if str(BRIDGE_RCLPY) not in sys.path:
            sys.path.insert(0, str(BRIDGE_RCLPY))
        import rclpy
        from geometry_msgs.msg import Twist, PoseStamped
        from nav_msgs.msg import Odometry
        from std_srvs.srv import SetBool, Trigger
        from isaacsim.core.prims import RigidPrim
        rclpy.init()
        node = rclpy.create_node("dock_lift_handoff_bridge")
        veh_pub = node.create_publisher(PoseStamped, "/vehicle/pose", 10)
        veh_rb = RigidPrim(VEHICLE_PATH)
        VEH_SNAPSHOT = tuple(np.array(v, copy=True) for v in veh_rb.get_world_poses())

        def make_cb(key):
            def cb(msg):
                drive(key, msg.linear.x, msg.linear.y, -msg.angular.z)
            return cb
        odom_pub = {}
        for key in ROBOTS:
            node.create_subscription(Twist, f"/robot_{key}/cmd_vel", make_cb(key), 10)
            odom_pub[key] = node.create_publisher(Odometry, f"/robot_{key}/odom", 10)

        arm_idx = {k: {n: arts[k].dof_names.index(n) for n in ARM_TARGETS} for k in arts}
        arm_cmd = {k: 0.0 for k in arts}
        arm_applied = {k: 0.0 for k in arts}

        def apply_arms(key):
            tgt, cur = arm_cmd[key], arm_applied[key]
            if abs(tgt - cur) > 1e-4:
                # 시연용으로 0.02→0.035→0.05까지 올렸다가 리프트 중 차량이 살짝
                # 들리는 흔들림이 실측 확인돼 원래 검증된 값(dock_motion_check.py
                # PASS 기준)으로 되돌림. 속도보다 정확도 우선.
                cur += max(-0.02, min(0.02, tgt - cur))
                arm_applied[key] = cur
            pos = np.array(arts[key].get_joint_positions(), dtype=np.float32, copy=True)
            for n, deg in ARM_TARGETS.items():
                v = math.radians(deg * cur)
                if pos.ndim == 2:
                    pos[0, arm_idx[key][n]] = v
                else:
                    pos[arm_idx[key][n]] = v
            arts[key].set_joint_position_targets(pos)

        def make_arm_cb(key):
            def cb(req, resp):
                arm_cmd[key] = 1.0 if req.data else 0.0
                resp.success = True
                resp.message = "arm target set: " + ("open" if req.data else "fold")
                return resp
            return cb
        for key in arts:
            node.create_service(SetBool, f"/robot_{key}/arm_control", make_arm_cb(key))

        # --- 테스트 반복용 리셋/체크포인트: 씬 재로딩 없이 순간이동으로 되돌린다. ---
        # mission.py 의 게이트/스테이징 상수와 반드시 일치해야 함(별도 프로세스라 공유 불가).
        WALL_CLEAR_X = -20.0
        LANE_Z_FRONT = 1.5
        NORTH_STAGE_Z = 4.0

        def _teleport_xz(art, snapshot, x, z):
            """snapshot(캡처된 정상 포즈)의 y·회전은 그대로 두고 x,z만 바꿔 순간이동.

            차 밑(축 정렬 위치)으로 직접 텔레포트하면 차체와 겹쳐(interpenetration)
            물리가 튕겨나가는 게 실측 확인됨 — 그래서 차량과 안 겹치는 "진입 직전
            대기 지점"까지만 순간이동시키고, 차 밑 진입은 항상 실제 주행으로 한다.
            높이/회전을 새로 만들지 않고 캡처된 값을 그대로 쓰는 것도 같은 이유
            (임의로 만든 회전값이 실제와 안 맞아 넘어지는 문제도 같이 방지)."""
            pos0, orn = snapshot
            pos = np.array(pos0, dtype=np.float32, copy=True)
            if pos.ndim == 2:
                pos[0, 0], pos[0, 2] = x, z
            else:
                pos[0], pos[2] = x, z
            art.set_world_poses(pos, np.array(orn, dtype=np.float32))
            try:
                art.set_velocities(np.zeros((1, 6), dtype=np.float32))
            except Exception as e:
                node.get_logger().warn(f"속도 초기화 실패(무시): {e}")

        def _on_reset(req, resp):
            """씬을 처음(도크) 상태로 되돌린다 — Isaac 재시작 없이 반복 테스트용."""
            for key, art in arts.items():
                pos, orn = DOCK_SNAPSHOT[key]
                art.set_world_poses(np.array(pos, dtype=np.float32),
                                    np.array(orn, dtype=np.float32))
                try:
                    art.set_velocities(np.zeros((1, 6), dtype=np.float32))
                except Exception as e:
                    node.get_logger().warn(f"속도 초기화 실패(무시): {e}")
                arm_cmd[key] = 0.0
                arm_applied[key] = 0.0
            pos, orn = VEH_SNAPSHOT
            veh_rb.set_world_poses(np.array(pos, dtype=np.float32),
                                   np.array(orn, dtype=np.float32))
            try:
                veh_rb.set_velocities(np.zeros((1, 6), dtype=np.float32))
            except Exception as e:
                node.get_logger().warn(f"속도 초기화 실패(무시): {e}")
            resp.success = True
            resp.message = "리셋 완료(도크 위치)"
            node.get_logger().info(resp.message)
            return resp

        def _on_checkpoint_staged(req, resp):
            """로봇을 "게이트 통과 직후" 상태로 순간이동 — 뒷축 로봇은 북쪽 스테이징
            (center_x, NORTH_STAGE_Z), 앞축 로봇은 서쪽 벽 대기 위치(WALL_CLEAR_X,
            LANE_Z_FRONT). 둘 다 차량과 안 겹치는 open floor라 순간이동 안전함.
            여기서부터 회전·진입·파지·운반은 mission이 실제로(진짜 주행으로) 한다 —
            건너뛰는 건 시간이 오래 걸리는 게이트 통과 구간뿐."""
            if not AXLE:
                resp.success = False; resp.message = "AXLE 미계산"; return resp
            _teleport_xz(arts["rear"], DOCK_SNAPSHOT["rear"], AXLE["center_x"], NORTH_STAGE_Z)
            _teleport_xz(arts["front"], DOCK_SNAPSHOT["front"], WALL_CLEAR_X, LANE_Z_FRONT)
            for key in arts:
                arm_cmd[key] = 0.0
                arm_applied[key] = 0.0
            pos, orn = VEH_SNAPSHOT
            veh_rb.set_world_poses(np.array(pos, dtype=np.float32),
                                   np.array(orn, dtype=np.float32))
            try:
                veh_rb.set_velocities(np.zeros((1, 6), dtype=np.float32))
            except Exception as e:
                node.get_logger().warn(f"속도 초기화 실패(무시): {e}")
            resp.success = True
            resp.message = "체크포인트 완료(스테이징) — /dock_lift_from_staged 로 이어서"
            node.get_logger().info(resp.message)
            return resp

        node.create_service(Trigger, "/sim_reset", _on_reset)
        node.create_service(Trigger, "/sim_checkpoint_staged", _on_checkpoint_staged)

        print(f"DOCK_LIFT_HANDOFF_READY robots=['robot_rear','robot_front'] "
              f"domain={os.environ.get('ROS_DOMAIN_ID','0')}", flush=True)
        while app.is_running():
            app.update()
            rclpy.spin_once(node, timeout_sec=0.0)
            for key, cfg in ROBOTS.items():
                apply_arms(key)
                pos, orn = arts[key].get_world_poses()
                pos = np.asarray(pos).reshape(-1)[:3]
                orn = np.asarray(orn).reshape(-1)[:4]
                w, x, y, z = (float(v) for v in orn)
                fwd_x = 1.0 - 2.0 * (y * y + z * z)
                fwd_z = 2.0 * (x * z - w * y)
                yaw = math.atan2(-fwd_z, fwd_x)
                od = Odometry()
                od.header.stamp = node.get_clock().now().to_msg()
                od.header.frame_id = "map"
                od.child_frame_id = f"robot_{key}/base_link"
                od.pose.pose.position.x = float(pos[0])
                od.pose.pose.position.y = float(pos[1])
                od.pose.pose.position.z = float(pos[2])
                od.pose.pose.orientation.z = math.sin(yaw * 0.5)
                od.pose.pose.orientation.w = math.cos(yaw * 0.5)
                odom_pub[key].publish(od)
            veh_pos, veh_orn = veh_rb.get_world_poses()
            vp = np.asarray(veh_pos).reshape(-1)[:3]
            vo = np.asarray(veh_orn).reshape(-1)[:4]
            vw, vx, vy, vz = (float(v) for v in vo)
            veh_fwd_x = 1.0 - 2.0 * (vy * vy + vz * vz)
            veh_fwd_z = 2.0 * (vx * vz - vw * vy)
            veh_yaw = math.atan2(-veh_fwd_z, veh_fwd_x)
            ps = PoseStamped()
            ps.header.stamp = node.get_clock().now().to_msg()
            ps.header.frame_id = "map"
            ps.pose.position.x = float(vp[0])
            ps.pose.position.y = float(vp[1])
            ps.pose.position.z = float(vp[2])
            ps.pose.orientation.z = math.sin(veh_yaw * 0.5)
            ps.pose.orientation.w = math.cos(veh_yaw * 0.5)
            veh_pub.publish(ps)
        app.close()
    finally:
        pass


if __name__ == "__main__":
    main()
