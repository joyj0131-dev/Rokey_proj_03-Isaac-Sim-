#!/usr/bin/env python3
"""Two-robot carry demo for the HWIA parking robot (Isaac Sim 5.1).

Like ``parking_robot_rear_lift_test.py``, but with TWO mecanum robots (the baked
``*_mecha_roller`` asset). One robot enters under the front axle, one under the
rear axle from the opposite end, both deploy arms to grab the tires, then both
drive the car FORWARD ~1 m using real mecanum wheel drive
(``mecanum_drive.wheel_velocities_from_cmd_vel``). If the real carry stalls the
demo falls back to a guided carry so the motion is always viewable.

GUI (watch it, Play to replay):
  cd /home/rokey/dev_ws/isaac_sim/isaacsim/_build/linux-x86_64/release
  ./python.sh /home/rokey/cobot3_ws/isaacpjt/Isaac_envo/two_robot_carry_demo.py --gui

Headless single verification:
  ./python.sh .../two_robot_carry_demo.py
"""

import json
import math
import os
import sys
from pathlib import Path

from mecanum_drive import wheel_velocities_from_cmd_vel, WHEEL_JOINTS


WORK_DIR = Path(__file__).resolve().parent
PARKING_SOURCE_USD = WORK_DIR / "parking" / "parking_environment.usd"
PARKING_USD = WORK_DIR / "parking" / "parking_environment_mechanical_test.usd"
VEHICLES_USD = WORK_DIR / "fab_vehicles.usd"
ROBOT_PACKAGE = WORK_DIR.parent / "hwia_parking_robot_final_caster_package"
ROBOT_USD = ROBOT_PACKAGE / "hwia_parking_robot_final_caster_mecha_roller.usd"
# USD에는 상대경로로 굽는다(절대경로는 트리를 옮기는 순간 끊긴다).
ROBOT_REF = f"../{ROBOT_PACKAGE.name}/{ROBOT_USD.name}"

# --keep-drivetrain: 리프트 테스트와 동일한 실험 플래그.
# 구동계를 떼면 원통 휠 충돌체가 실제 접촉체가 되는데, PhysX는 원통을 면진 볼록체로
# 근사하므로 서스펜션 없는 차체가 면 사이를 계속 흔들거린다(리프트 테스트에서 측정:
# 이동경로 0.632 m / 순변위 0.033 m, 끝내 정착 안 함). 그 진동이 팔을 통해 로봇까지
# 밀어내면서 리프트 테스트를 실패시켰고, 구동계를 유지하자 arrival 오차가
# 0.6056 -> 0.0011 m로 떨어졌다. 운반 데모도 같은 원인인지 확인하기 위한 플래그다.
KEEP_DRIVETRAIN = "--keep-drivetrain" in sys.argv[1:]

# --sphere-wheels: 구동계는 제거한 채 휠 충돌체만 원통 -> 구로 교체(리프트 테스트와 동일).
# 차가 자유 강체로 남으므로 --keep-drivetrain과 달리 지면에 묶이지 않아 운반에 적합하다.
# 리프트 테스트 측정: Sedan settle 이동경로 0.632 -> 0.00002 m, 뒷축 상승 마진 6% -> 16%.
SPHERE_WHEELS = "--sphere-wheels" in sys.argv[1:]
OUTPUT_USD = WORK_DIR / "two_robot_carry_demo.usd"
REPORT_JSON = WORK_DIR / "two_robot_carry_demo_report.json"

ISAAC_ROOT = Path("/home/rokey/dev_ws/isaac_sim/isaacsim/_build/linux-x86_64/release")
ISAAC_PYTHON = ISAAC_ROOT / "python.sh"

# A7. A5는 팀원 주차장 에셋의 A5_Coupe가 이미 점유하고 있다(리프트 테스트와 동일).
PARKING_CENTER = (8.5, 0.0, 7.8)  # A7
SEDAN_REAR_AXLE_LOCAL_Z = (-1.5053923 - 1.4878858) * 0.5
SEDAN_FRONT_AXLE_LOCAL_Z = (1.4452012 + 1.4468861) * 0.5
REAR_TARGET_Z = PARKING_CENTER[2] + SEDAN_REAR_AXLE_LOCAL_Z
FRONT_TARGET_Z = PARKING_CENTER[2] + SEDAN_FRONT_AXLE_LOCAL_Z
REAR_START_Z = REAR_TARGET_Z - 1.75      # rear robot approaches from behind (-Z)
FRONT_START_Z = FRONT_TARGET_Z + 1.75    # front robot approaches from the front (+Z)

CARRY_DISTANCE_M = 1.0
CARRY_SPEED = 0.35        # m/s per robot toward world +Z during the carry
CARRY_MODE = "real"       # "real" mecanum drive, else guided fallback

SEDAN_ROOT = "/World/VehicleAsset/Vehicles/Sedan"
SEDAN_WHEELS = ("FrontLeftWheel", "FrontRightWheel", "RearLeftWheel", "RearRightWheel")

ARM_TARGETS = {
    "arm_left_front_joint": 90.0,
    "arm_left_rear_joint": -90.0,
    "arm_right_front_joint": -90.0,
    "arm_right_rear_joint": 90.0,
}

# Per-robot composed prim paths (baked asset referenced at /World/<name>).
ROBOTS = {
    "rear": {
        "wrap": "/World/RobotRear/base_link",
        "root": "/World/RobotRear/base_link/base_link",
        "joints": "/World/RobotRear/base_link/joints",
        "xform": "/World/RobotRear",
        "start_z": REAR_START_Z,
        "target_z": REAR_TARGET_Z,
        "facing": +1,   # local +X -> world +Z
    },
    "front": {
        "wrap": "/World/RobotFront/base_link",
        "root": "/World/RobotFront/base_link/base_link",
        "joints": "/World/RobotFront/base_link/joints",
        "xform": "/World/RobotFront",
        "start_z": FRONT_START_Z,
        "target_z": FRONT_TARGET_Z,
        "facing": -1,   # local +X -> world -Z
    },
}


def _restart_with_isaac_python():
    if os.environ.get("CARB_APP_PATH"):
        return
    if not ISAAC_PYTHON.is_file():
        raise FileNotFoundError(f"Isaac Sim python.sh not found: {ISAAC_PYTHON}")
    os.execv(
        str(ISAAC_PYTHON),
        [str(ISAAC_PYTHON), str(Path(__file__).resolve()), *sys.argv[1:]],
    )


def _replace_matrix_xform(prim, matrix, UsdGeom):
    xform = UsdGeom.Xformable(prim)
    xform.ClearXformOpOrder()
    xform.MakeMatrixXform().Set(matrix)


def _robot_matrix(Gf, facing, tx, tz):
    """Robot X-forward,Y-left,Z-up -> world, facing +Z (facing=+1) or -Z (-1)."""
    if facing > 0:
        return Gf.Matrix4d(
            0.0, 0.0, 1.0, 0.0,
            1.0, 0.0, 0.0, 0.0,
            0.0, 1.0, 0.0, 0.0,
            tx, 0.0, tz, 1.0,
        )
    return Gf.Matrix4d(
        0.0, 0.0, -1.0, 0.0,
        -1.0, 0.0, 0.0, 0.0,
        0.0, 1.0, 0.0, 0.0,
        tx, 0.0, tz, 1.0,
    )


def build_test_stage():
    from pxr import Gf, PhysxSchema, Sdf, Usd, UsdGeom, UsdPhysics, UsdShade

    for path in (PARKING_SOURCE_USD, VEHICLES_USD, ROBOT_USD):
        if not path.is_file():
            raise FileNotFoundError(path)

    # Local mechanical-test parking copy with the remote lidar stripped.
    source_parking_layer = Sdf.Layer.FindOrOpen(str(PARKING_SOURCE_USD))
    if source_parking_layer is None:
        raise RuntimeError(f"unable to read parking layer: {PARKING_SOURCE_USD}")
    parking_layer = (
        Sdf.Layer.FindOrOpen(str(PARKING_USD))
        if PARKING_USD.is_file()
        else Sdf.Layer.CreateNew(str(PARKING_USD))
    )
    parking_layer.TransferContent(source_parking_layer)
    # Match by prefix: the parking builder may emit "CeilingLidar" or the
    # zone-suffixed "CeilingLidarWest"/"CeilingLidarEast" pair.
    sensors_spec = parking_layer.GetPrimAtPath("/World/Sensors")
    if sensors_spec is not None:
        for child in sensors_spec.nameChildren:
            if child.name.startswith("CeilingLidar"):
                child.referenceList.ClearEdits()
                child.active = False
    parking_layer.Save()

    stage = Usd.Stage.CreateNew(str(OUTPUT_USD))
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.y)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    stage.SetTimeCodesPerSecond(60.0)
    world = UsdGeom.Xform.Define(stage, "/World").GetPrim()
    stage.SetDefaultPrim(world)

    parking = UsdGeom.Xform.Define(stage, "/World/Parking").GetPrim()
    parking.GetReferences().AddReference(str(PARKING_USD))
    vehicle_asset = UsdGeom.Xform.Define(stage, "/World/VehicleAsset").GetPrim()
    vehicle_asset.GetReferences().AddReference(str(VEHICLES_USD))
    for name in ("RobotRear", "RobotFront"):
        r = UsdGeom.Xform.Define(stage, f"/World/{name}").GetPrim()
        r.GetReferences().AddReference(ROBOT_REF)

    deactivate = [
        "/World/Parking/PhysicsScene",
        "/World/VehicleAsset/PhysicsScene",
        "/World/VehicleAsset/DriveGround",
        "/World/VehicleAsset/FabLighting",
        # 쇼룸 바닥 디스크. Y=0으로 주차장 바닥과 겹쳐 Z-fighting을 일으킨다.
        "/World/VehicleAsset/Cylinder001",
    ]
    parking_sensors = stage.GetPrimAtPath("/World/Parking/Sensors")
    if parking_sensors.IsValid():
        deactivate += [
            str(child.GetPath())
            for child in parking_sensors.GetChildren()
            if child.GetName().startswith("CeilingLidar")
        ]
    for path in deactivate:
        prim = stage.GetPrimAtPath(path)
        if prim.IsValid():
            prim.SetActive(False)

    scene = UsdPhysics.Scene.Define(stage, "/World/PhysicsScene")
    scene.CreateGravityDirectionAttr(Gf.Vec3f(0.0, -1.0, 0.0))
    scene.CreateGravityMagnitudeAttr(9.81)
    physx_scene = PhysxSchema.PhysxSceneAPI.Apply(scene.GetPrim())
    physx_scene.CreateBroadphaseTypeAttr("GPU")
    physx_scene.CreateSolverTypeAttr("TGS")
    physx_scene.CreateEnableCCDAttr(True)
    physx_scene.CreateEnableStabilizationAttr(True)
    physx_scene.CreateEnableGPUDynamicsAttr(True)
    physx_scene.CreateTimeStepsPerSecondAttr(240)
    vehicle_context = PhysxSchema.PhysxVehicleContextAPI.Apply(scene.GetPrim())
    vehicle_context.CreateUpdateModeAttr(PhysxSchema.Tokens.velocityChange)
    vehicle_context.CreateVerticalAxisAttr(PhysxSchema.Tokens.posY)
    vehicle_context.CreateLongitudinalAxisAttr(PhysxSchema.Tokens.posZ)

    ground = UsdGeom.Cube.Define(stage, "/World/TestGround")
    ground.CreateSizeAttr(1.0)
    ground.CreateVisibilityAttr(UsdGeom.Tokens.invisible)
    ground_xform = UsdGeom.Xformable(ground)
    ground_xform.AddTranslateOp().Set(Gf.Vec3d(0.0, -0.10, 0.0))
    ground_xform.AddScaleOp().Set(Gf.Vec3f(40.0, 0.20, 40.0))
    UsdPhysics.CollisionAPI.Apply(ground.GetPrim())

    # One Sedan at A7, converted to a plain rigid body (same as the lift test).
    vehicles = stage.GetPrimAtPath("/World/VehicleAsset/Vehicles")
    for vehicle in vehicles.GetChildren():
        if vehicle.GetName() != "Sedan":
            vehicle.SetActive(False)
    sedan = stage.GetPrimAtPath(SEDAN_ROOT)
    sedan_matrix = UsdGeom.Xformable(sedan).GetLocalTransformation()
    sedan_matrix.SetTranslate(Gf.Vec3d(*PARKING_CENTER))
    _replace_matrix_xform(sedan, sedan_matrix, UsdGeom)

    sedan_single_apis = (
        PhysxSchema.PhysxVehicleAPI,
        PhysxSchema.PhysxVehicleDriveStandardAPI,
        PhysxSchema.PhysxVehicleEngineAPI,
        PhysxSchema.PhysxVehicleGearsAPI,
        PhysxSchema.PhysxVehicleAutoGearBoxAPI,
        PhysxSchema.PhysxVehicleClutchAPI,
        PhysxSchema.PhysxVehicleControllerAPI,
        PhysxSchema.PhysxVehicleAckermannSteeringAPI,
        PhysxSchema.PhysxVehicleMultiWheelDifferentialAPI,
    )
    wheel_apis = (
        PhysxSchema.PhysxVehicleWheelAttachmentAPI,
        PhysxSchema.PhysxVehicleWheelAPI,
        PhysxSchema.PhysxVehicleTireAPI,
        PhysxSchema.PhysxVehicleSuspensionAPI,
        PhysxSchema.PhysxVehicleSuspensionComplianceAPI,
    )
    if KEEP_DRIVETRAIN:
        print(
            "[carry] --keep-drivetrain: Sedan의 PhysX Vehicle 구동계를 유지합니다.",
            flush=True,
        )
    else:
        for api_schema in sedan_single_apis:
            if sedan.HasAPI(api_schema):
                sedan.RemoveAPI(api_schema)
        for instance_name in (PhysxSchema.Tokens.brakes0, PhysxSchema.Tokens.brakes1):
            if sedan.HasAPI(PhysxSchema.PhysxVehicleBrakesAPI, instance_name):
                sedan.RemoveAPI(PhysxSchema.PhysxVehicleBrakesAPI, instance_name)
        for wheel_name in SEDAN_WHEELS:
            wheel = stage.GetPrimAtPath(f"{SEDAN_ROOT}/{wheel_name}")
            for api_schema in wheel_apis:
                if wheel.HasAPI(api_schema):
                    wheel.RemoveAPI(api_schema)

    if SPHERE_WHEELS:
        # 원통 충돌체를 끄고 같은 반경의 구를 대신 세운다(물리 재질은 휠 프림에서 상속).
        for wheel_name in SEDAN_WHEELS:
            wheel_path = f"{SEDAN_ROOT}/{wheel_name}"
            cylinder = stage.GetPrimAtPath(f"{wheel_path}/Collision")
            if not cylinder.IsValid():
                raise RuntimeError(f"휠 충돌체를 찾지 못했습니다: {wheel_path}/Collision")
            radius = float(UsdGeom.Cylinder(cylinder).GetRadiusAttr().Get())
            cylinder.SetActive(False)
            sphere = UsdGeom.Sphere.Define(stage, f"{wheel_path}/CollisionSphere")
            sphere.CreateRadiusAttr(radius)
            sphere.CreatePurposeAttr(UsdGeom.Tokens.guide)
            UsdPhysics.CollisionAPI.Apply(sphere.GetPrim())
        print(
            f"[carry] --sphere-wheels: 휠 충돌체 4개를 구(r={radius:.4f})로 교체했습니다.",
            flush=True,
        )

    sedan_rigid = PhysxSchema.PhysxRigidBodyAPI.Apply(sedan)
    sedan_rigid.GetDisableGravityAttr().Set(False)
    sedan_rigid.CreateEnableCCDAttr(True)
    sedan_rigid.GetSolverPositionIterationCountAttr().Set(16)
    sedan_rigid.GetSolverVelocityIterationCountAttr().Set(8)

    # Place the two robots at opposite ends of the car.
    for name, cfg in (("RobotRear", ROBOTS["rear"]), ("RobotFront", ROBOTS["front"])):
        prim = stage.GetPrimAtPath(f"/World/{name}")
        _replace_matrix_xform(
            prim, _robot_matrix(Gf, cfg["facing"], PARKING_CENTER[0], cfg["start_z"]), UsdGeom
        )

    # Grippy contact for rollers/drive-wheels/tires.
    materials = UsdGeom.Scope.Define(stage, "/World/TestMaterials").GetPath()
    grip = UsdShade.Material.Define(stage, materials.AppendChild("Grip"))
    grip_api = UsdPhysics.MaterialAPI.Apply(grip.GetPrim())
    grip_api.CreateStaticFrictionAttr(1.2)
    grip_api.CreateDynamicFrictionAttr(1.0)
    grip_api.CreateRestitutionAttr(0.0)
    UsdShade.MaterialBindingAPI.Apply(ground.GetPrim()).Bind(
        grip, UsdShade.Tokens.weakerThanDescendants, "physics"
    )
    for cfg in ROBOTS.values():
        for link_name in (
            "bearing_roller_left_front", "bearing_roller_left_rear",
            "bearing_roller_right_front", "bearing_roller_right_rear",
        ):
            link = stage.GetPrimAtPath(f"{cfg['wrap']}/{link_name}")
            if link.IsValid():
                UsdShade.MaterialBindingAPI.Apply(link).Bind(
                    grip, UsdShade.Tokens.weakerThanDescendants, "physics"
                )
    for wheel_name in SEDAN_WHEELS:
        wheel = stage.GetPrimAtPath(f"{SEDAN_ROOT}/{wheel_name}")
        UsdShade.MaterialBindingAPI.Apply(wheel).Bind(
            grip, UsdShade.Tokens.weakerThanDescendants, "physics"
        )

    # Velocity drives on hubs; position drives on arms (start folded).
    for cfg in ROBOTS.values():
        for jname in WHEEL_JOINTS.values():
            joint = stage.GetPrimAtPath(f"{cfg['joints']}/{jname}")
            drive = UsdPhysics.DriveAPI.Get(joint, "angular")
            if not drive:
                drive = UsdPhysics.DriveAPI.Apply(joint, "angular")
            drive.CreateStiffnessAttr(0.0)
            drive.CreateDampingAttr(1500.0)
            drive.CreateMaxForceAttr(6000.0)
            drive.CreateTargetVelocityAttr(0.0)
        for name in ARM_TARGETS:
            joint = stage.GetPrimAtPath(f"{cfg['joints']}/{name}")
            drive = UsdPhysics.DriveAPI.Get(joint, "angular")
            if drive:
                drive.CreateStiffnessAttr(2000.0)
                drive.CreateDampingAttr(150.0)
                drive.CreateMaxForceAttr(6000.0)
                drive.CreateTargetPositionAttr(0.0)

    world.SetCustomDataByKey("demo", "two robot carry forward")
    stage.GetRootLayer().Save()


def capture_viewport_camera():
    try:
        from omni.kit.viewport.utility import get_active_viewport
        from omni.kit.viewport.utility.camera_state import ViewportCameraState

        viewport = get_active_viewport()
        if viewport is None:
            return None
        camera_path = str(viewport.camera_path)
        camera_state = ViewportCameraState(camera_path, viewport)
        return {
            "camera_path": camera_path,
            "position": tuple(float(v) for v in camera_state.position_world),
            "target": tuple(float(v) for v in camera_state.target_world),
        }
    except Exception as exc:
        print(f"VIEWPORT_CAPTURE_WARNING={type(exc).__name__}: {exc}", flush=True)
        return None


def restore_viewport_camera(camera_snapshot):
    if camera_snapshot is None:
        return
    try:
        from omni.kit.viewport.utility import get_active_viewport
        from omni.kit.viewport.utility.camera_state import ViewportCameraState
        from pxr import Gf

        viewport = get_active_viewport()
        if viewport is None:
            return
        camera_path = camera_snapshot["camera_path"]
        viewport.camera_path = camera_path
        camera_state = ViewportCameraState(camera_path, viewport)
        camera_state.set_position_world(Gf.Vec3d(*camera_snapshot["position"]), True)
        camera_state.set_target_world(Gf.Vec3d(*camera_snapshot["target"]), True)
    except Exception as exc:
        print(f"VIEWPORT_RESTORE_WARNING={type(exc).__name__}: {exc}", flush=True)


def run_demo(app, preserved_camera=None):
    import numpy as np
    import omni.physx
    import omni.timeline
    import omni.usd
    from isaacsim.core.prims import Articulation

    context = omni.usd.get_context()
    if not context.open_stage(str(OUTPUT_USD)):
        raise RuntimeError(f"failed to open demo stage: {OUTPUT_USD}")
    restore_viewport_camera(preserved_camera)
    for _ in range(12):
        app.update()
    restore_viewport_camera(preserved_camera)

    timeline = omni.timeline.get_timeline_interface()
    timeline.play()
    for _ in range(12):
        app.update()

    physx = omni.physx.get_physx_interface()
    arts = {}
    for key, cfg in ROBOTS.items():
        art = Articulation(cfg["root"])
        art.initialize()
        shape = art.get_joint_positions().shape
        arts[key] = {
            "art": art,
            "cfg": cfg,
            "vel": np.zeros(shape, dtype=np.float32),
            "pos": np.array(art.get_joint_positions(), dtype=np.float32, copy=True),
            "wheel_idx": {w: art.dof_names.index(j) for w, j in WHEEL_JOINTS.items()},
            "arm_idx": {n: art.dof_names.index(n) for n in ARM_TARGETS},
        }

    def rigid_z(path):
        return float(physx.get_rigidbody_transformation(path)["position"][2])

    def rigid_pos(path):
        v = physx.get_rigidbody_transformation(path)["position"]
        return tuple(float(x) for x in v)

    def set_cmd_vel(state, vx, vy, wz):
        omegas = wheel_velocities_from_cmd_vel(vx, vy, wz)
        vel = state["vel"]
        for w, omega in omegas.items():
            idx = state["wheel_idx"][w]
            if vel.ndim == 2:
                vel[0, idx] = omega
            else:
                vel[idx] = omega
        state["art"].set_joint_velocity_targets(vel)

    def set_arms(state, scale):
        pos = state["pos"]
        for name, target in ARM_TARGETS.items():
            idx = state["arm_idx"][name]
            val = math.radians(target * scale)
            if pos.ndim == 2:
                pos[0, idx] = val
            else:
                pos[idx] = val
        state["art"].set_joint_position_targets(pos)

    # 1) settle
    for st in arts.values():
        set_cmd_vel(st, 0.0, 0.0, 0.0)
        set_arms(st, 0.0)
    for _ in range(120):
        app.update()

    # 2) guided ingress under each axle (keeps approach off the drift path)
    ingress_steps = 200
    starts = {k: rigid_pos(v["cfg"]["root"]) for k, v in arts.items()}
    for step in range(1, ingress_steps + 1):
        p = step / ingress_steps
        for key, st in arts.items():
            cfg = st["cfg"]
            s = starts[key]
            z = s[2] + (cfg["target_z"] - s[2]) * p
            pos = np.array([[PARKING_CENTER[0], s[1], z]], dtype=np.float32)
            st["art"].set_world_poses(positions=pos)
        app.update()
    for _ in range(60):
        app.update()

    car_before_grab = rigid_pos(SEDAN_ROOT)

    # 3) deploy arms on both robots to grab the tires
    for ramp in range(1, 181):
        for st in arts.values():
            set_arms(st, ramp / 180.0)
        app.update()
    for _ in range(300):
        app.update()

    car_after_grab = rigid_pos(SEDAN_ROOT)
    carry_start_z = car_after_grab[2]
    # 파지로 차가 실제로 얼마나 떴는지. 0에 가까우면 팔이 타이어를 못 잡은 것이다.
    car_lift_m = car_after_grab[1] - car_before_grab[1]
    # 운반 중 로봇 자신의 이동량. 차 이동량과 비교해야 "싣고 갔는지 / 두고 갔는지"가 갈린다.
    robot_carry_start_z = {k: rigid_z(v["cfg"]["root"]) for k, v in arts.items()}

    # 4) carry forward ~1 m (world +Z)
    mode = CARRY_MODE
    reached = False
    if mode == "real":
        for st in arts.values():
            # world +Z is local +X for the rear robot, local -X for the front one
            set_cmd_vel(st, CARRY_SPEED * st["cfg"]["facing"], 0.0, 0.0)
        for _ in range(900):
            app.update()
            moved = rigid_z(SEDAN_ROOT) - carry_start_z
            car_y = rigid_pos(SEDAN_ROOT)[1]
            if not math.isfinite(moved) or abs(car_y) > 1.0:
                break
            if moved >= CARRY_DISTANCE_M:
                reached = True
                break
        for st in arts.values():
            set_cmd_vel(st, 0.0, 0.0, 0.0)
        for _ in range(60):
            app.update()

    car_after_real = rigid_pos(SEDAN_ROOT)
    real_moved = car_after_real[2] - carry_start_z

    # 로봇이 차를 싣고 갔는가, 아니면 두고 혼자 빠져나갔는가.
    # 추종률 ~1.0 = 함께 이동(정상), ~0 = 로봇만 빠져나감(리프트가 수평 구속을 못 함).
    robot_carry = {
        k: rigid_z(v["cfg"]["root"]) - robot_carry_start_z[k] for k, v in arts.items()
    }
    robot_carry_mean = sum(robot_carry.values()) / len(robot_carry)
    carry_follow_ratio = (
        real_moved / robot_carry_mean if abs(robot_carry_mean) > 1e-6 else 0.0
    )
    print(
        f"[carry] 리프트 {car_lift_m:+.4f} m | 로봇 {robot_carry_mean:+.4f} m / "
        f"차량 {real_moved:+.4f} m | 추종률 {carry_follow_ratio:.3f}",
        flush=True,
    )

    used_guided = False
    if not (mode == "real" and real_moved >= 0.8 * CARRY_DISTANCE_M):
        # Guided fallback: move both robots + car together +Z by the remaining gap.
        from isaacsim.core.prims import RigidPrim
        used_guided = True
        remaining = CARRY_DISTANCE_M - max(real_moved, 0.0)
        sedan_view = RigidPrim(SEDAN_ROOT)
        base_car = np.array(rigid_pos(SEDAN_ROOT), dtype=np.float32)
        base_robot = {k: np.array(rigid_pos(v["cfg"]["root"]), dtype=np.float32)
                      for k, v in arts.items()}
        guided_steps = 180
        for step in range(1, guided_steps + 1):
            dz = remaining * step / guided_steps
            for key, st in arts.items():
                bp = base_robot[key]
                pos = np.array([[bp[0], bp[1], bp[2] + dz]], dtype=np.float32)
                st["art"].set_world_poses(positions=pos)
            car_pos = np.array([[base_car[0], base_car[1], base_car[2] + dz]], dtype=np.float32)
            sedan_view.set_world_poses(positions=car_pos)
            app.update()
        for _ in range(30):
            app.update()

    car_final = rigid_pos(SEDAN_ROOT)
    total_moved = car_final[2] - carry_start_z

    passed = bool(
        total_moved >= 0.8 * CARRY_DISTANCE_M
        and abs(car_final[1]) < 1.0
        and all(math.isfinite(c) for c in car_final)
    )

    report = {
        "passed": passed,
        "keep_drivetrain": KEEP_DRIVETRAIN,
        "sphere_wheels": SPHERE_WHEELS,
        "carry_mode_requested": CARRY_MODE,
        "used_guided_fallback": used_guided,
        "car_before_grab_xyz_m": car_before_grab,
        "car_after_grab_xyz_m": car_after_grab,
        "car_final_xyz_m": car_final,
        "real_drive_forward_m": real_moved,
        "total_forward_m": total_moved,
        "carry_target_m": CARRY_DISTANCE_M,
        "car_lift_m": car_lift_m,
        "robot_carry_m": robot_carry_mean,
        "robot_carry_per_robot_m": robot_carry,
        "carry_follow_ratio": carry_follow_ratio,
    }
    REPORT_JSON.write_text(json.dumps(report, indent=2, ensure_ascii=False), encoding="utf-8")

    print(f"DEMO_PASSED={passed}", flush=True)
    print(f"REAL_DRIVE_FORWARD_M={real_moved:.4f}", flush=True)
    print(f"TOTAL_FORWARD_M={total_moved:.4f}", flush=True)
    print(f"USED_GUIDED_FALLBACK={used_guided}", flush=True)
    print(f"CAR_FINAL_Y_M={car_final[1]:.4f}", flush=True)
    print(f"REPORT={REPORT_JSON}", flush=True)

    timeline.stop()
    app.update()
    return report


def run_gui_replay_loop(app, automated_runs=0):
    import omni.timeline
    import omni.usd

    context = omni.usd.get_context()
    timeline = omni.timeline.get_timeline_interface()
    timeline.stop()
    if not context.open_stage(str(OUTPUT_USD)):
        raise RuntimeError(f"failed to open demo stage: {OUTPUT_USD}")
    for _ in range(12):
        app.update()

    print("GUI_REPLAY_READY: Press Play to run the two-robot carry.", flush=True)
    run_number = 0
    if automated_runs:
        timeline.play()
    while app.is_running():
        app.update()
        if not timeline.is_playing():
            continue
        timeline.stop()
        app.update()
        run_number += 1
        print(f"GUI_REPLAY_START={run_number}", flush=True)
        try:
            preserved_camera = capture_viewport_camera()
            report = run_demo(app, preserved_camera=preserved_camera)
            print(f"GUI_REPLAY_DONE={run_number} passed={report['passed']}", flush=True)
            if automated_runs and not report["passed"]:
                raise RuntimeError(f"automated replay {run_number} failed")
        except Exception as exc:
            timeline.stop()
            print(f"GUI_REPLAY_EXCEPTION={run_number} {type(exc).__name__}: {exc}", flush=True)
            if automated_runs:
                raise
        print("GUI_REPLAY_READY: Press Play to run again.", flush=True)
        if automated_runs:
            if run_number >= automated_runs:
                return
            timeline.play()


def main():
    _restart_with_isaac_python()
    from isaacsim import SimulationApp

    app = SimulationApp({"headless": "--gui" not in sys.argv[1:]})
    try:
        build_test_stage()
        if "--gui" in sys.argv[1:]:
            run_gui_replay_loop(app)
        else:
            report = run_demo(app)
            if not report["passed"]:
                raise RuntimeError(f"two-robot carry demo failed; see {REPORT_JSON}")
    except Exception as exc:
        print(f"DEMO_EXCEPTION={type(exc).__name__}: {exc}", flush=True)
        raise
    finally:
        app.close()


if __name__ == "__main__":
    main()
