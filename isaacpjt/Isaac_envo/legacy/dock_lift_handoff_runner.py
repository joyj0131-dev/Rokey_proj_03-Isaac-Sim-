#!/usr/bin/env python3
"""인계장 전체 환경 ROS2 촉발 도킹·리프트·오미 운반 Isaac 러너.

씬: 주차장 전체 환경(parking_environment_v2.usd) + 출차 인계 구역의
Pickup + A3의 Offroad + 낮춘 바퀴 로봇 2대(입·출차 로봇 대기 도크).
프로그램 켜두고 대기 — parking_robot_system의 액션 서버·오케스트레이터가 구동.

ROS2: /robot_N/cmd_vel 구독, /robot_N/odom 발행(x,y=높이,z,yaw),
      /robot_N/arm_control 서비스, /vehicle/pose 발행.

실행: dock_lift_handoff_runner.sh [--gui] [--headless-test]
"""
import json
import math
import os
import sys
import time
from pathlib import Path

WORK_DIR = Path(__file__).resolve().parent
PARKING_USD = WORK_DIR / "parking" / "parking_environment_v2.usd"
ROBOT_USD = (WORK_DIR.parent / "hwia_parking_robot_final_caster_package"
             / "hwia_depth_cam_mecha_roller_lowered.usd")
VEHICLES_USD = WORK_DIR / "fab_vehicles.usd"
ISAAC_PYTHON = Path("/home/rokey/dev_ws/isaac_sim/isaacsim/_build/linux-x86_64/release/python.sh")
BRIDGE_RCLPY = Path("/home/rokey/dev_ws/isaac_sim/isaacsim/_build/linux-x86_64/release"
                    "/exts/isaacsim.ros2.bridge/humble/rclpy")

TARGET_VEHICLE = "Pickup"
VEHICLE_PATH = f"/World/VehicleAsset/Vehicles/{TARGET_VEHICLE}"
PARKED_VEHICLE = "Offroad"
PARKED_VEHICLE_PATH = f"/World/VehicleAsset/Vehicles/{PARKED_VEHICLE}"
ACTIVE_VEHICLES = (TARGET_VEHICLE, PARKED_VEHICLE)
FAB_VEHICLE_TYPES = ("Compact", "Coupe", "Hatchback", "Minivan", "Offroad",
                     "Pickup", "Sedan", "Sport", "SUV", "Wagon")
TARGET_COLLIDER_WIDTH = 0.30
# ROS odom/vehicle pose 발행이 app.update()에 묶여 있어 30Hz로 낮추면 운반 중 wall-time
# 피드백률이 7Hz 아래로 떨어지고 차량 heading 최대 오차가 1.22deg까지 증가했다.
# 제어 안정성을 위해 timeline은 60Hz를 유지하고, 해상도와 headless viewport만 줄인다.
RENDER_HZ = 60.0
RENDER_WIDTH = 640
RENDER_HEIGHT = 400
PHYSICS_HZ = 120.0
LINEAR_ACCEL = 0.5       # m/s^2, body X/Y 벡터 가속도
LINEAR_DECEL = 0.8       # m/s^2, 감속·반전은 조금 더 빠르게
ANGULAR_ACCEL = 0.8      # rad/s^2
VEHICLE_SPAWN_Y = 0.035
PICKUP_ANCHOR = "/World/ParkingEnvironment/VehicleWaitAreas/ExitVehicleWait"
OFFROAD_ANCHOR = "/World/ParkingEnvironment/Spaces/A3"
PLACEMENT = {}
ARM_TARGETS = {
    "arm_left_front_joint": 90.0, "arm_left_rear_joint": -90.0,
    "arm_right_front_joint": -90.0, "arm_right_rear_joint": 90.0,
}
VEHICLE_WHEELS = ("FrontLeftWheel", "FrontRightWheel", "RearLeftWheel", "RearRightWheel")

# 입차 로봇은 -Z 도크, 출차 로봇은 +Z 도크. 둘 다 도크 메타데이터의 +X 방향을 본다.
ROBOTS = {
    "rear":  {"xform": "/World/Robots/robot_rear",
              "dock": "/World/ParkingEnvironment/RobotServiceArea/EntryRobotDock"},
    "front": {"xform": "/World/Robots/robot_front",
              "dock": "/World/ParkingEnvironment/RobotServiceArea/ExitRobotDock"},
}
AXLE = {}

# --- /parking_slots 발행용 ---
_HALF_LEN, _HALF_WID = 3.3, 1.7
_ACCESSIBLE = set()


def _all_slots_usd(stage):
    """v2 USD의 슬롯 메타데이터에서 중심 좌표를 읽는다."""
    slots = {}
    spaces = stage.GetPrimAtPath("/World/ParkingEnvironment/Spaces")
    if not spaces or not spaces.IsValid():
        raise RuntimeError("주차 슬롯 메타데이터 없음: /World/ParkingEnvironment/Spaces")
    for prim in spaces.GetChildren():
        center = prim.GetAttribute("parking:center").Get()
        if center is None:
            continue
        slots[prim.GetName()] = (float(center[0]), float(center[2]), 0.0)
    if not slots:
        raise RuntimeError("parking:center가 정의된 주차 슬롯이 없습니다.")
    return slots


def _vehicle_world_positions(stage):
    """활성 차량(Pickup/Offroad)의 world (x,z)."""
    from pxr import UsdGeom
    positions = []
    for name in ACTIVE_VEHICLES:
        prim = stage.GetPrimAtPath(f"/World/VehicleAsset/Vehicles/{name}")
        if not prim or not prim.IsValid() or not prim.IsActive():
            continue
        m = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(0)
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


def _layout_vehicle_pose(stage, anchor_path, *, heading_default=0.0):
    """레이아웃 프림의 parking:center/heading을 차량 월드 pose로 변환한다."""
    prim = stage.GetPrimAtPath(anchor_path)
    if not prim or not prim.IsValid():
        raise RuntimeError(f"차량 배치 기준 프림 없음: {anchor_path}")
    center = prim.GetAttribute("parking:center").Get()
    if center is None:
        raise RuntimeError(f"parking:center 없음: {anchor_path}")
    heading = prim.GetAttribute("parking:heading").Get()
    if heading is None:
        heading = heading_default
    return (float(center[0]), VEHICLE_SPAWN_Y, float(center[2])), float(heading)


def _place_vehicle(UsdGeom, Gf, prim, pos, heading_deg):
    """로컬 +Z가 차량 전방인 fab 차량을 v2 레이아웃 heading으로 배치한다."""
    xf = UsdGeom.Xformable(prim)
    xf.ClearXformOpOrder()
    m = Gf.Matrix4d(1.0)
    m.SetRotate(Gf.Rotation(Gf.Vec3d(0.0, 1.0, 0.0), heading_deg))
    m.SetTranslateOnly(Gf.Vec3d(*pos))
    xf.AddTransformOp().Set(m)


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
    # 120Hz를 유지하되 cmd_vel 가속도 제한으로 롤러 접촉에 들어가는 토크 충격을 줄인다.
    px.CreateTimeStepsPerSecondAttr(PHYSICS_HZ)
    vctx = PhysxSchema.PhysxVehicleContextAPI.Apply(sc)
    vctx.CreateUpdateModeAttr(PhysxSchema.Tokens.velocityChange)
    vctx.CreateVerticalAxisAttr(PhysxSchema.Tokens.posY)
    vctx.CreateLongitudinalAxisAttr(PhysxSchema.Tokens.posZ)


def _disable_unused_scene(stage):
    """미션에 필요 없는 차량·센서·차량 충돌 그룹을 세션 레이어에서 비활성화한다.

    원본 주차장 USD는 수정하지 않는다. v2는 차량을 포함하지 않지만 이전 USD와 함께
    사용할 때 남아 있을 수 있는 `/World/ParkingVehicles`도 방어적으로 비활성화한다.
    Pickup과 Offroad는 별도 `/World/VehicleAsset` 아래에 추가되므로 영향을 받지 않는다.
    """
    vehicle_count = 0
    parking_vehicles = stage.GetPrimAtPath("/World/ParkingVehicles")
    if parking_vehicles and parking_vehicles.IsValid():
        for container_name in ("Parked", "HandoffQueue"):
            container = stage.GetPrimAtPath(f"/World/ParkingVehicles/{container_name}")
            if container and container.IsValid():
                vehicle_count += len(container.GetChildren())
        parking_vehicles.SetActive(False)

    sensors = stage.GetPrimAtPath("/World/Sensors")
    sensor_count = 0
    if sensors and sensors.IsValid():
        sensor_count = sum(
            "Lidar" in child.GetName() and not child.GetName().endswith("Mount")
            for child in sensors.GetChildren())
        sensors.SetActive(False)

    # 제거된 주차 차량만 사용하던 collision group 메타데이터도 제외한다.
    vehicle_physics = stage.GetPrimAtPath("/World/VehiclePhysics")
    if vehicle_physics and vehicle_physics.IsValid():
        vehicle_physics.SetActive(False)

    return vehicle_count, sensor_count


def build_stage(app):
    from pxr import Gf, UsdGeom, UsdPhysics
    import omni.usd

    ctx = omni.usd.get_context()
    ctx.new_stage()
    stage = ctx.get_stage()
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.y)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    stage.SetTimeCodesPerSecond(RENDER_HZ)
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

    # 기존 주차·대기 차량과 천장 RTX LiDAR를 끈 뒤 요청한 두 차량만 별도로 추가한다.
    disabled_vehicles, disabled_sensors = _disable_unused_scene(stage)

    _grip_material(stage)

    # fab 전체 참조 후 Pickup과 Offroad만 활성화(재질·차량물리 바인딩 유지).
    asset = stage.DefinePrim("/World/VehicleAsset", "Xform")
    asset.GetReferences().AddReference(str(VEHICLES_USD))
    for _ in range(20):
        app.update()
    for name in ("PhysicsScene", "DriveGround", "FabLighting", "Cylinder001"):
        p = stage.GetPrimAtPath(f"/World/VehicleAsset/{name}")
        if p and p.IsValid():
            p.SetActive(False)
    for vt in FAB_VEHICLE_TYPES:
        if vt in ACTIVE_VEHICLES:
            continue
        p = stage.GetPrimAtPath(f"/World/VehicleAsset/Vehicles/{vt}")
        if p and p.IsValid():
            p.SetActive(False)

    pickup_pos, pickup_heading = _layout_vehicle_pose(stage, PICKUP_ANCHOR)
    offroad_pos, offroad_heading = _layout_vehicle_pose(stage, OFFROAD_ANCHOR)
    PLACEMENT.clear()
    PLACEMENT.update(
        Pickup={"pos": pickup_pos, "heading": pickup_heading},
        Offroad={"pos": offroad_pos, "heading": offroad_heading},
    )
    for name, path in ((TARGET_VEHICLE, VEHICLE_PATH),
                       (PARKED_VEHICLE, PARKED_VEHICLE_PATH)):
        prim = stage.GetPrimAtPath(path)
        if not prim or not prim.IsValid():
            raise RuntimeError(f"{name} 없음: {path}")
        spec = PLACEMENT[name]
        _place_vehicle(UsdGeom, Gf, prim, spec["pos"], spec["heading"])
    for _ in range(20):
        app.update()

    n_fixed = sum(_fix_vehicle_colliders(stage, path)
                  for path in (VEHICLE_PATH, PARKED_VEHICLE_PATH))

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
    print(
        "DOCK_STAGE_READY "
        f"parking={PARKING_USD.name} "
        f"pickup_pos={pickup_pos} pickup_heading={pickup_heading:.1f}deg "
        f"offroad_pos={offroad_pos} offroad_heading={offroad_heading:.1f}deg "
        f"axle rear_z={AXLE['rear_z']:.3f} front_z={AXLE['front_z']:.3f} "
        f"center_x={center_x:.3f} colliders_fixed={n_fixed}",
        flush=True)
    print(
        f"SCENE_OPTIMIZED disabled_vehicles={disabled_vehicles} "
        f"disabled_sensors={disabled_sensors} "
        f"render={RENDER_WIDTH}x{RENDER_HEIGHT}@{RENDER_HZ:.0f}Hz "
        f"physics={PHYSICS_HZ:.0f}Hz",
        flush=True)
    print(
        f"MOTION_SMOOTHING linear_accel={LINEAR_ACCEL:.2f}m/s2 "
        f"linear_decel={LINEAR_DECEL:.2f}m/s2 "
        f"angular_accel={ANGULAR_ACCEL:.2f}rad/s2",
        flush=True)
    return stage


def main():
    _restart_with_isaac_python()
    from isaacsim import SimulationApp
    headless = "--gui" not in sys.argv[1:]
    app = SimulationApp({
        "headless": headless,
        "width": RENDER_WIDTH,
        "height": RENDER_HEIGHT,
        # 자동시험은 화면을 소비하지 않으므로 렌더 프레임 갱신 자체를 생략한다.
        # --gui에서는 False라 640x400 viewport를 그대로 볼 수 있다.
        "disable_viewport_updates": headless,
    })
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
        from mecanum_drive import (WHEEL_JOINTS, configure_hub_drives, slew_twist,
                                   wheel_velocities_from_cmd_vel)
        for key, cfg in ROBOTS.items():
            configure_hub_drives(stage, f"{cfg['xform']}/joints")
        wheel_idx = {k: {w: arts[k].dof_names.index(j) for w, j in WHEEL_JOINTS.items()}
                     for k in arts}
        vel_buf = {k: np.zeros(arts[k].get_joint_positions().shape, dtype=np.float32)
                   for k in arts}
        target_twist = {k: (0.0, 0.0, 0.0) for k in arts}
        applied_twist = {k: (0.0, 0.0, 0.0) for k in arts}

        def apply_wheel_velocity(key, vx, vy, wz):
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

            def _vehicle_pose(rb):
                pos, orn = rb.get_world_poses()
                p = np.asarray(pos).reshape(-1)[:3]
                q = np.asarray(orn).reshape(-1)[:4]
                w, x, y, z = (float(v) for v in q)
                axis_x = 2.0 * (x * z + w * y)
                axis_z = 1.0 - 2.0 * (x * x + y * y)
                return p, math.degrees(math.atan2(axis_x, axis_z))

            vehicle_rb = {
                TARGET_VEHICLE: RigidPrim(VEHICLE_PATH),
                PARKED_VEHICLE: RigidPrim(PARKED_VEHICLE_PATH),
            }
            p0 = {k: _p(a) for k, a in arts.items()}
            v0 = {name: _vehicle_pose(rb)[0] for name, rb in vehicle_rb.items()}
            for _ in range(180):
                app.update()
            p1 = {k: _p(a) for k, a in arts.items()}
            v1 = {name: _vehicle_pose(rb) for name, rb in vehicle_rb.items()}
            disp = {k: float(np.linalg.norm(p1[k] - p0[k])) for k in arts}
            vehicle_disp = {
                name: float(np.linalg.norm(v1[name][0] - v0[name]))
                for name in vehicle_rb
            }
            robot_xz_error = {}
            for key, cfg in ROBOTS.items():
                dock = _dock_position(stage, cfg["dock"])
                robot_xz_error[key] = float(np.linalg.norm(
                    p1[key][[0, 2]] - np.asarray([dock[0], dock[2]])))
            vehicle_xz_error = {}
            vehicle_yaw_error = {}
            for name, (pos, yaw_deg) in v1.items():
                expected = PLACEMENT[name]
                vehicle_xz_error[name] = float(np.linalg.norm(
                    pos[[0, 2]] - np.asarray([expected["pos"][0], expected["pos"][2]])))
                vehicle_yaw_error[name] = abs(
                    (yaw_deg - expected["heading"] + 180.0) % 360.0 - 180.0)
            slot_table = _all_slots_usd(stage)
            slots_ok = (
                set(slot_table) == {"A1", "A2", "A3"}
                and np.linalg.norm(
                    np.asarray(slot_table["A3"][:2])
                    - np.asarray([PLACEMENT[PARKED_VEHICLE]["pos"][0],
                                  PLACEMENT[PARKED_VEHICLE]["pos"][2]])) < 1e-4
            )

            # vehicle context가 없으면 play 시 수 m 튄다. 위치·방향도 요청한 레이아웃
            # 메타데이터에서 벗어나지 않는지 함께 검사한다.
            robots_ok = all(d < 0.35 for d in disp.values())
            vehicle_stable = all(d < 0.30 for d in vehicle_disp.values())
            placement_ok = (
                all(e < 0.20 for e in robot_xz_error.values())
                and all(e < 0.20 for e in vehicle_xz_error.values())
                and all(e < 1.0 for e in vehicle_yaw_error.values())
                and slots_ok
            )
            ok = robots_ok and vehicle_stable and placement_ok
            print(f"DOCK_PHYSICS_TEST={'PASS' if ok else 'FAIL'} "
                   f"robot_disp={ {k: round(v,4) for k,v in disp.items()} } "
                   f"vehicle_disp={ {k: round(v,4) for k,v in vehicle_disp.items()} } "
                   f"robot_xz_error={ {k: round(v,4) for k,v in robot_xz_error.items()} } "
                   f"vehicle_xz_error={ {k: round(v,4) for k,v in vehicle_xz_error.items()} } "
                   f"vehicle_yaw_error_deg="
                   f"{ {k: round(v,3) for k,v in vehicle_yaw_error.items()} } "
                   f"slots={sorted(slot_table)} slots_ok={slots_ok}",
                   flush=True)
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
        # ROS2 Bridge가 환경에 따라 내부 context를 먼저 초기화할 수 있다.
        # 이미 활성인 context에 init()을 다시 호출하면 Isaac 기동 직후 종료된다.
        if not rclpy.ok():
            rclpy.init()
        node = rclpy.create_node("dock_lift_handoff_bridge")
        veh_pub = node.create_publisher(PoseStamped, "/vehicle/pose", 10)
        veh_rb = RigidPrim(VEHICLE_PATH)

        def make_cb(key):
            def cb(msg):
                # 콜백에서는 목표만 갱신한다. 실제 휠 목표는 시뮬레이션 시간 기준
                # slew-rate limiter를 거쳐 출발·정지·반전 충격을 줄인다.
                target_twist[key] = (
                    float(msg.linear.x), float(msg.linear.y), -float(msg.angular.z))
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
        _SLOT_TABLE = _all_slots_usd(stage)
        _slot_tick = 0
        _rtf_wall = time.monotonic()
        _rtf_sim = timeline.get_current_time()
        _drive_sim = _rtf_sim
        while app.is_running():
            app.update()
            now_sim = timeline.get_current_time()
            if now_sim - _rtf_sim >= 5.0:
                now_wall = time.monotonic()
                sim_dt = now_sim - _rtf_sim
                wall_dt = now_wall - _rtf_wall
                print(
                    f"RUNTIME_RTF={sim_dt / max(wall_dt, 1e-6):.3f} "
                    f"sim_dt={sim_dt:.2f}s wall_dt={wall_dt:.2f}s",
                    flush=True)
                _rtf_sim, _rtf_wall = now_sim, now_wall
            rclpy.spin_once(node, timeout_sec=0.0)
            drive_dt = min(0.1, max(0.0, now_sim - _drive_sim))
            _drive_sim = now_sim
            for key, cfg in ROBOTS.items():
                applied_twist[key] = slew_twist(
                    applied_twist[key], target_twist[key], drive_dt,
                    linear_accel=LINEAR_ACCEL,
                    linear_decel=LINEAR_DECEL,
                    angular_accel=ANGULAR_ACCEL,
                )
                apply_wheel_velocity(key, *applied_twist[key])
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
            vq = np.asarray(veh_orn).reshape(-1)[:4]
            vw, vx, vy, vz = (float(v) for v in vq)
            # 차량의 길이축은 로컬 +Z다. 월드 XZ 평면으로 투영한 길이축의
            # yaw(0=월드 +Z)를 ROS식 z/w quaternion으로 담는다. Isaac의 원래
            # Y-up quaternion을 그대로 넣으면 ROS 소비자가 Z-up yaw로 오해한다.
            vehicle_axis_x = 2.0 * (vx * vz + vw * vy)
            vehicle_axis_z = 1.0 - 2.0 * (vx * vx + vy * vy)
            vehicle_yaw = math.atan2(vehicle_axis_x, vehicle_axis_z)
            ps = PoseStamped()
            ps.header.stamp = node.get_clock().now().to_msg()
            ps.header.frame_id = "map"
            ps.pose.position.x = float(vp[0])
            ps.pose.position.y = float(vp[1])
            ps.pose.position.z = float(vp[2])
            ps.pose.orientation.z = math.sin(vehicle_yaw * 0.5)
            ps.pose.orientation.w = math.cos(vehicle_yaw * 0.5)
            veh_pub.publish(ps)
            # /parking_slots: 시뮬레이션 시간 기준 약 2Hz.
            _slot_tick += 1
            if _slot_tick % max(1, int(RENDER_HZ / 2.0)) == 0:
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
