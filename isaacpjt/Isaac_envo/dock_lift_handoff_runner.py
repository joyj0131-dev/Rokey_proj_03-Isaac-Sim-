#!/usr/bin/env python3
"""인계장 전체 환경 ROS2 촉발 도킹·리프트·오미 운반 Isaac 러너.

씬: 주차장 전체 환경(parking_environment_with_markers.usd) + 인계 베이의
Pickup(콜라이더 수정·그립 재질) + 낮춘 바퀴 로봇 2대(West 대기 도크).
프로그램 켜두고 대기 — 외부 오케스트레이터(dock_lift_handoff_mission.py)가 구동.

ROS2: /robot_N/cmd_vel 구독, /robot_N/odom 발행(x,y=높이,z,yaw),
      /robot_N/arm_control 서비스, /vehicle/pose 발행.

실행: dock_lift_handoff_runner.sh [--gui] [--headless-test]
"""
import json
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
        for child in prim.GetChildren():
            for v in ([child] + list(child.GetChildren())):
                if not v.IsActive():
                    continue
                if not v.IsA(UsdGeom.Xformable):
                    continue
                m = UsdGeom.Xformable(v).ComputeLocalToWorldTransform(0)
                t = m.ExtractTranslation()
                positions.append((float(t[0]), float(t[2])))
    return positions


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
    # 물리 스텝 240→120: 벽시계 체감속도(RTF) 약 2배. 리프트 시 차량이 튀거나
    # 흔들리면 240으로 되돌릴 것(240+CCD가 리프트 안정성을 잡아주던 값).
    px.CreateTimeStepsPerSecondAttr(120)
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
        from std_srvs.srv import SetBool
        from std_msgs.msg import String as RosString
        from isaacsim.core.prims import RigidPrim
        rclpy.init()
        node = rclpy.create_node("dock_lift_handoff_bridge")
        veh_pub = node.create_publisher(PoseStamped, "/vehicle/pose", 10)
        veh_rb = RigidPrim(VEHICLE_PATH)

        def make_cb(key):
            def cb(msg):
                drive(key, msg.linear.x, msg.linear.y, -msg.angular.z)
            return cb
        odom_pub = {}
        for key in ROBOTS:
            node.create_subscription(Twist, f"/robot_{key}/cmd_vel", make_cb(key), 10)
            odom_pub[key] = node.create_publisher(Odometry, f"/robot_{key}/odom", 10)
        slots_pub = node.create_publisher(RosString, "/parking_slots", 10)

        arm_idx = {k: {n: arts[k].dof_names.index(n) for n in ARM_TARGETS} for k in arts}
        arm_cmd = {k: 0.0 for k in arts}
        arm_applied = {k: 0.0 for k in arts}

        def apply_arms(key):
            tgt, cur = arm_cmd[key], arm_applied[key]
            if abs(tgt - cur) > 1e-4:
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

        print(f"DOCK_LIFT_HANDOFF_READY robots=['robot_rear','robot_front'] "
              f"domain={os.environ.get('ROS_DOMAIN_ID','0')}", flush=True)
        _SLOT_TABLE = _all_slots_usd()
        _slot_tick = 0
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
            vp = np.asarray(veh_rb.get_world_poses()[0]).reshape(-1)[:3]
            ps = PoseStamped()
            ps.header.stamp = node.get_clock().now().to_msg()
            ps.header.frame_id = "map"
            ps.pose.position.x = float(vp[0])
            ps.pose.position.y = float(vp[1])
            ps.pose.position.z = float(vp[2])
            veh_pub.publish(ps)
            # /parking_slots: ~2Hz (app.update()가 30틱마다 1회 발행, 60Hz 가정)
            _slot_tick += 1
            if _slot_tick % 30 == 0:
                positions = _vehicle_world_positions(stage)
                arr = []
                for sid, (sx, sz, yaw) in _SLOT_TABLE.items():
                    occ = any(abs(vx - sx) <= _HALF_WID and abs(vz - sz) <= _HALF_LEN
                              for vx, vz in positions)
                    arr.append({"slot_id": sid, "occupied": occ, "is_accessible": sid in _ACCESSIBLE,
                                "x": round(sx, 3), "y": round(-sz, 3), "yaw_deg": yaw})
                msg = RosString(); msg.data = json.dumps(arr); slots_pub.publish(msg)
        app.close()
    finally:
        pass


if __name__ == "__main__":
    main()
