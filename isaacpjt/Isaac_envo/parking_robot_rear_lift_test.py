#!/usr/bin/env python3
"""Parking-robot rear-wheel lift integration test for Isaac Sim 5.1.

The test composes the existing parking environment, one Sedan from the FAB
vehicle pack, and the HWIA parking robot without modifying any source asset.
It then drives the robot under the rear axle, deploys all four swing arms, and
records the rear/front wheel-height change in a JSON report.
"""

import json
import math
import os
import sys
from pathlib import Path


WORK_DIR = Path(__file__).resolve().parent
PARKING_SOURCE_USD = WORK_DIR / "parking" / "parking_environment.usd"
PARKING_USD = WORK_DIR / "parking" / "parking_environment_mechanical_test.usd"
VEHICLES_USD = WORK_DIR / "fab_vehicles.usd"
ROBOT_PACKAGE = WORK_DIR.parent / "hwia_parking_robot_final_caster_package"
# 이 프로젝트의 최종 로봇은 메카넘이다. 베이스 에셋(_final_caster.usd)은 축 Y 고정
# 일반 휠이라 횡이동이 불가능하고, 리프트 중 차체를 지지하는 접지 모델도 실제와 다르다.
ROBOT_USD = ROBOT_PACKAGE / "hwia_parking_robot_final_caster_mecha_roller.usd"
# USD에는 상대경로로 굽는다(절대경로는 트리를 옮기는 순간 끊긴다).
# OUTPUT_USD가 Isaac_envo/ 에 있으므로 한 단계만 올라간다.
ROBOT_REF = f"../{ROBOT_PACKAGE.name}/{ROBOT_USD.name}"

# --keep-drivetrain: 테스트 Sedan의 PhysX Vehicle 구동계를 제거하지 않고 그대로 둔다.
#
# 기본값(구동계 제거)에서는 원통 휠 충돌체가 실제 접촉체로 승격되는데, PhysX는 원통을
# 면진 볼록체로 근사하므로 서스펜션 없는 1600 kg 강체가 면 사이를 계속 흔들거린다.
# 측정: 정착 120프레임 동안 이동경로 0.632 m / 순변위 0.033 m (떨림비 19), 끝내 정착 안 함.
# 반면 구동계를 유지한 팀원 차량 12대는 레이캐스트 서스펜션으로 지지되어 정확히 0.00000으로 멎는다.
#
# 이 플래그는 그 차이가 원인인지 확인하는 실험용이다. 확인할 것:
#   (a) 떨림이 사라지는가          (b) 그래도 뒷바퀴가 들리는가
# (b)가 실패하면 구동계 제거가 옳았던 것이고, 원통 접촉 문제는 다른 방법으로 풀어야 한다.
KEEP_DRIVETRAIN = "--keep-drivetrain" in sys.argv[1:]

# --sphere-wheels: 구동계는 제거한 채(차가 자유 강체로 남아 들리고 실려야 하므로),
# 휠 충돌체만 원통 -> 구로 교체한다. 로봇이 이미 쓰는 방식과 같다:
#   구동휠     visual cylinder r=0.060 len=0.048  /  collision sphere r=0.060  (폭 2.5배)
#   베어링롤러 visual cylinder len=0.18           /  collision sphere 3개      (폭 1.17배)
# 구는 PhysX 네이티브 프리미티브라 면진 근사가 없어 원통 rocking이 원천적으로 없다.
# 폭이 부풀지만(반경 0.343 -> 폭 0.686 vs 타이어 0.256) 최악값은 휠 중심 높이에서이고,
# 팔 롤러가 실제 작업하는 y=0.045 높이에서는 0.340 m로 원통 대비 1.33배에 그친다.
SPHERE_WHEELS = "--sphere-wheels" in sys.argv[1:]
OUTPUT_USD = WORK_DIR / "parking_robot_rear_lift_test.usd"
REPORT_JSON = WORK_DIR / "parking_robot_rear_lift_test_report.json"

ISAAC_ROOT = Path(
    "/home/rokey/dev_ws/isaac_sim/isaacsim/_build/linux-x86_64/release"
)
ISAAC_PYTHON = ISAAC_ROOT / "python.sh"

# A7. 팀원 주차장 에셋이 A5에 A5_Coupe를 미리 주차시켜 두므로 A5를 쓰면 차량이
# 겹쳐 스폰되어 PhysX가 상호침투를 해소하려다 폭발한다. A7은 비어 있고 슬롯 기하가
# A5와 동일(3.40 x 6.60, 같은 A열)해서 기존 리프트 검증 수치를 그대로 비교할 수 있다.
PARKING_CENTER = (8.5, 0.0, 7.8)  # A7
SEDAN_REAR_AXLE_LOCAL_Z = (-1.5053923 - 1.4878858) * 0.5
SEDAN_FRONT_AXLE_LOCAL_Z = (1.4452012 + 1.4468861) * 0.5
ROBOT_START_Z = 4.55
ROBOT_TARGET_Z = PARKING_CENTER[2] + SEDAN_REAR_AXLE_LOCAL_Z

ROBOT_ROOT = "/World/Robot/base_link/base_link"
ROBOT_JOINTS = "/World/Robot/base_link/joints"
SEDAN_ROOT = "/World/VehicleAsset/Vehicles/Sedan"

ARM_TARGETS = {
    "arm_left_front_joint": 90.0,
    "arm_left_rear_joint": -90.0,
    "arm_right_front_joint": -90.0,
    "arm_right_rear_joint": 90.0,
}
WHEEL_JOINTS = (
    "wheel_fl_joint",
    "wheel_fr_joint",
    "wheel_rl_joint",
    "wheel_rr_joint",
)
SEDAN_WHEELS = (
    "FrontLeftWheel",
    "FrontRightWheel",
    "RearLeftWheel",
    "RearRightWheel",
)


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


def build_test_stage():
    from pxr import Gf, PhysxSchema, Sdf, Usd, UsdGeom, UsdPhysics, UsdShade

    for path in (PARKING_SOURCE_USD, VEHICLES_USD, ROBOT_USD):
        if not path.is_file():
            raise FileNotFoundError(path)

    # Make a local mechanical-test copy at the raw layer level and strip the
    # one remote lidar prim before USD composition. This preserves the parking
    # geometry but avoids a network timeout on offline/headless machines.
    source_parking_layer = Sdf.Layer.FindOrOpen(str(PARKING_SOURCE_USD))
    if source_parking_layer is None:
        raise RuntimeError(f"unable to read parking layer: {PARKING_SOURCE_USD}")
    parking_layer = (
        Sdf.Layer.FindOrOpen(str(PARKING_USD))
        if PARKING_USD.is_file()
        else Sdf.Layer.CreateNew(str(PARKING_USD))
    )
    parking_layer.TransferContent(source_parking_layer)
    # Ceiling lidar prim names are not fixed: the parking builder emits one
    # "CeilingLidar" or zone-suffixed ones ("CeilingLidarWest"/"CeilingLidarEast").
    # Match by prefix so a renamed or added lidar is never silently left online.
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
    robot = UsdGeom.Xform.Define(stage, "/World/Robot").GetPrim()
    robot.GetReferences().AddReference(ROBOT_REF)

    # Use one shared Y-up physics scene. Source scenes and the vehicle pack's
    # showroom ground would otherwise overlap the parking floor.
    deactivate = [
        "/World/Parking/PhysicsScene",
        "/World/VehicleAsset/PhysicsScene",
        "/World/VehicleAsset/DriveGround",
        "/World/VehicleAsset/FabLighting",
        # 차량 팩의 쇼룸 바닥 디스크(반경 10.84 m, 두께 0). 충돌체는 없지만
        # 주차장 바닥 상면과 정확히 같은 Y=0 평면이라 Z-fighting으로 바닥이 깜빡인다.
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
    vehicle_context = PhysxSchema.PhysxVehicleContextAPI.Apply(scene.GetPrim())
    vehicle_context.CreateUpdateModeAttr(PhysxSchema.Tokens.velocityChange)
    vehicle_context.CreateVerticalAxisAttr(PhysxSchema.Tokens.posY)
    vehicle_context.CreateLongitudinalAxisAttr(PhysxSchema.Tokens.posZ)

    # The supplied parking floor is rendered geometry without a usable PhysX
    # collider. Add an invisible slab exactly below y=0 so the integration test
    # uses the same visible environment while rigid bodies have a support plane.
    ground = UsdGeom.Cube.Define(stage, "/World/TestGround")
    ground.CreateSizeAttr(1.0)
    ground.CreateVisibilityAttr(UsdGeom.Tokens.invisible)
    ground_xform = UsdGeom.Xformable(ground)
    ground_xform.AddTranslateOp().Set(Gf.Vec3d(0.0, -0.10, 0.0))
    ground_xform.AddScaleOp().Set(Gf.Vec3f(30.0, 0.20, 30.0))
    UsdPhysics.CollisionAPI.Apply(ground.GetPrim())

    # Keep only one vehicle and preserve its existing local orientation while
    # moving its root to parking bay A7.
    vehicles = stage.GetPrimAtPath("/World/VehicleAsset/Vehicles")
    for vehicle in vehicles.GetChildren():
        if vehicle.GetName() != "Sedan":
            vehicle.SetActive(False)
    sedan = stage.GetPrimAtPath(SEDAN_ROOT)
    sedan_matrix = UsdGeom.Xformable(sedan).GetLocalTransformation()
    sedan_matrix.SetTranslate(Gf.Vec3d(*PARKING_CENTER))
    _replace_matrix_xform(sedan, sedan_matrix, UsdGeom)

    # Convert the selected referenced vehicle to an ordinary rigid body for
    # this lifting test. Merely setting vehicleEnabled=false still lets the
    # PhysX Vehicle subsystem force gravity off, so remove its applied APIs in
    # this stronger test layer while retaining all visual/collision geometry.
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
            "[lift] --keep-drivetrain: Sedan의 PhysX Vehicle 구동계를 유지합니다 "
            "(팀원 차량 12대와 동일 조건).",
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
        # 원통 충돌체를 끄고 같은 반경의 구를 대신 세운다. 물리 재질은 휠 프림에
        # 바인딩돼 있어 자식이 상속하므로 따로 다시 걸지 않는다.
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
            f"[lift] --sphere-wheels: 휠 충돌체 4개를 구(r={radius:.4f})로 교체했습니다.",
            flush=True,
        )

    sedan_rigid = PhysxSchema.PhysxRigidBodyAPI.Apply(sedan)
    sedan_rigid.GetDisableGravityAttr().Set(False)
    sedan_rigid.CreateEnableCCDAttr(True)
    sedan_rigid.GetSolverPositionIterationCountAttr().Set(16)
    sedan_rigid.GetSolverVelocityIterationCountAttr().Set(8)

    # Robot coordinates are X-forward, Y-left, Z-up. Map them to the parking
    # stage as world Z-forward, X-left, Y-up and start in the central aisle.
    robot_to_world = Gf.Matrix4d(
        0.0, 0.0, 1.0, 0.0,
        1.0, 0.0, 0.0, 0.0,
        0.0, 1.0, 0.0, 0.0,
        PARKING_CENTER[0], 0.0, ROBOT_START_Z, 1.0,
    )
    _replace_matrix_xform(robot, robot_to_world, UsdGeom)

    # Tire/roller and drive-wheel/floor friction for a dry concrete test.
    materials = UsdGeom.Scope.Define(stage, "/World/TestMaterials").GetPath()
    grip = UsdShade.Material.Define(stage, materials.AppendChild("RobotGrip"))
    grip_api = UsdPhysics.MaterialAPI.Apply(grip.GetPrim())
    grip_api.CreateStaticFrictionAttr(1.35)
    grip_api.CreateDynamicFrictionAttr(1.10)
    grip_api.CreateRestitutionAttr(0.01)
    UsdShade.MaterialBindingAPI.Apply(ground.GetPrim()).Bind(
        grip, UsdShade.Tokens.weakerThanDescendants, "physics"
    )
    for link_name in (
        "wheel_fl", "wheel_fr", "wheel_rl", "wheel_rr",
        "bearing_roller_left_front", "bearing_roller_left_rear",
        "bearing_roller_right_front", "bearing_roller_right_rear",
    ):
        link = stage.GetPrimAtPath(f"/World/Robot/base_link/{link_name}")
        UsdShade.MaterialBindingAPI.Apply(link).Bind(
            grip, UsdShade.Tokens.weakerThanDescendants, "physics"
        )
    for wheel_name in SEDAN_WHEELS:
        wheel = stage.GetPrimAtPath(f"{SEDAN_ROOT}/{wheel_name}")
        UsdShade.MaterialBindingAPI.Apply(wheel).Bind(
            grip, UsdShade.Tokens.weakerThanDescendants, "physics"
        )

    # Velocity drives move the robot into place. The arm drives are already
    # authored by the robot asset; targets remain zero in the saved stage.
    for name in WHEEL_JOINTS:
        joint = stage.GetPrimAtPath(f"{ROBOT_JOINTS}/{name}")
        drive = UsdPhysics.DriveAPI.Get(joint, "angular")
        drive.GetStiffnessAttr().Set(0.0)
        drive.GetDampingAttr().Set(1200.0)
        drive.GetMaxForceAttr().Set(5000.0)
        drive.GetTargetVelocityAttr().Set(0.0)
    for name in ARM_TARGETS:
        joint = stage.GetPrimAtPath(f"{ROBOT_JOINTS}/{name}")
        drive = UsdPhysics.DriveAPI.Get(joint, "angular")
        drive.GetStiffnessAttr().Set(1800.0)
        drive.GetDampingAttr().Set(140.0)
        drive.GetMaxForceAttr().Set(5000.0)
        drive.GetTargetPositionAttr().Set(0.0)

    world.SetCustomDataByKey("test", "parking robot rear-wheel lift")
    world.SetCustomDataByKey("vehicle", "Sedan")
    world.SetCustomDataByKey("parkingBay", "A7")
    world.SetCustomDataByKey("robotTargetZ", ROBOT_TARGET_Z)
    stage.GetRootLayer().Save()


def capture_viewport_camera():
    """Capture the active GUI camera without coupling headless tests to UI."""
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
            "position": tuple(float(value) for value in camera_state.position_world),
            "target": tuple(float(value) for value in camera_state.target_world),
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
        camera_state.set_position_world(
            Gf.Vec3d(*camera_snapshot["position"]), True
        )
        camera_state.set_target_world(
            Gf.Vec3d(*camera_snapshot["target"]), True
        )
    except Exception as exc:
        print(f"VIEWPORT_RESTORE_WARNING={type(exc).__name__}: {exc}", flush=True)


def run_test(app, preserved_camera=None):
    import numpy as np
    import omni.physx
    import omni.timeline
    import omni.usd
    from isaacsim.core.prims import Articulation
    from pxr import Usd, UsdGeom, UsdPhysics

    context = omni.usd.get_context()
    if not context.open_stage(str(OUTPUT_USD)):
        raise RuntimeError(f"failed to open test stage: {OUTPUT_USD}")
    # Restore before the first rendered update to avoid a visible jump to the
    # newly opened stage's default perspective.
    restore_viewport_camera(preserved_camera)
    for _ in range(12):
        app.update()
    # Some viewport builds reselect the default camera after stage attachment;
    # reapply once more before physics starts.
    restore_viewport_camera(preserved_camera)

    stage = context.get_stage()
    timeline = omni.timeline.get_timeline_interface()
    timeline.play()
    for _ in range(12):
        app.update()

    robot = Articulation(ROBOT_ROOT)
    robot.initialize()
    physx = omni.physx.get_physx_interface()

    # Articulation is a one-prim view, so runtime targets are shaped (1, dof).
    # Updating DriveAPI attributes after playback starts does not reliably
    # propagate through Fabric to PhysX; send commands through the articulation
    # controller instead.
    joint_positions = robot.get_joint_positions()
    command_shape = joint_positions.shape
    wheel_indices = [robot.dof_names.index(name) for name in WHEEL_JOINTS]
    arm_indices = {
        name: robot.dof_names.index(name) for name in ARM_TARGETS
    }
    velocity_targets = np.zeros(command_shape, dtype=np.float32)
    position_targets = np.array(joint_positions, dtype=np.float32, copy=True)

    def rigid_position(path):
        value = physx.get_rigidbody_transformation(path)
        return tuple(float(x) for x in value["position"])

    def wheel_center(name):
        prim = stage.GetPrimAtPath(f"{SEDAN_ROOT}/{name}")
        matrix = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(
            Usd.TimeCode.Default()
        )
        return tuple(float(x) for x in matrix.ExtractTranslation())

    def set_wheel_speed(deg_per_sec):
        angular_speed = math.radians(float(deg_per_sec))
        if velocity_targets.ndim == 2:
            velocity_targets[0, wheel_indices] = angular_speed
        else:
            velocity_targets[wheel_indices] = angular_speed
        robot.set_joint_velocity_targets(velocity_targets)

    def set_arm_targets(scale):
        for name, target in ARM_TARGETS.items():
            index = arm_indices[name]
            target_rad = math.radians(float(target * scale))
            if position_targets.ndim == 2:
                position_targets[0, index] = target_rad
            else:
                position_targets[index] = target_rad
        robot.set_joint_position_targets(position_targets)

    # Let the car and robot settle on the parking floor.
    set_wheel_speed(0.0)
    set_arm_targets(0.0)
    for _ in range(120):
        app.update()

    start_robot = rigid_position(ROBOT_ROOT)
    start_sedan = rigid_position(SEDAN_ROOT)
    initial_wheels = {name: wheel_center(name) for name in SEDAN_WHEELS}

    # Guide the chassis along the parking-bay centerline while the drive wheels
    # turn. Spherical wheel colliders are robust in CPU PhysX but can develop
    # lateral drift, which is outside this rear-lift integration test. Guidance
    # is removed before arm deployment, so tire lifting itself is unconstrained.
    drive_speed = 360.0
    set_wheel_speed(drive_speed)
    ingress_steps = 240
    for step in range(1, ingress_steps + 1):
        progress = step / ingress_steps
        guided_position = np.array(
            [[
                PARKING_CENTER[0],
                start_robot[1],
                start_robot[2] + (ROBOT_TARGET_Z - start_robot[2]) * progress,
            ]],
            dtype=np.float32,
        )
        robot.set_world_poses(positions=guided_position)
        app.update()
    pulse_position = rigid_position(ROBOT_ROOT)
    arrival_step = ingress_steps
    set_wheel_speed(0.0)
    for _ in range(60):
        app.update()
    parked_robot = rigid_position(ROBOT_ROOT)
    before_lift = {name: wheel_center(name) for name in SEDAN_WHEELS}

    if arrival_step is None:
        live_positions = robot.get_joint_positions()
        live_velocities = robot.get_joint_velocities()
        diagnostics = {
            "passed": False,
            "phase": "drive",
            "robot_start_xyz_m": start_robot,
            "robot_after_pulse_xyz_m": pulse_position,
            "robot_parked_xyz_m": parked_robot,
            "robot_target_z_m": ROBOT_TARGET_Z,
            "wheel_joint_positions_rad": {
                name: float(live_positions[0, index]
                            if live_positions.ndim == 2 else live_positions[index])
                for name, index in zip(WHEEL_JOINTS, wheel_indices)
            },
            "wheel_joint_velocities_rad_s": {
                name: float(live_velocities[0, index]
                            if live_velocities.ndim == 2 else live_velocities[index])
                for name, index in zip(WHEEL_JOINTS, wheel_indices)
            },
            "drive_command_rad_s": math.radians(drive_speed),
        }
        REPORT_JSON.write_text(
            json.dumps(diagnostics, indent=2, ensure_ascii=False),
            encoding="utf-8",
        )
        print(f"DRIVE_DIAGNOSTICS={json.dumps(diagnostics)}", flush=True)
        raise RuntimeError(
            f"robot failed to reach rear axle: z={parked_robot[2]:.3f}, "
            f"target={ROBOT_TARGET_Z:.3f}"
        )

    # Deploy gradually so the front/rear bearing rollers close around both
    # rear tires instead of applying an impulse at the joint limit.
    for ramp_step in range(1, 181):
        set_arm_targets(ramp_step / 180.0)
        app.update()
    for _ in range(360):
        app.update()

    after_lift = {name: wheel_center(name) for name in SEDAN_WHEELS}
    final_robot = rigid_position(ROBOT_ROOT)
    final_sedan = rigid_position(SEDAN_ROOT)

    joint_positions = robot.get_joint_positions()
    if getattr(joint_positions, "ndim", 1) == 2:
        joint_positions = joint_positions[0]
    arm_angles = {
        name: math.degrees(float(joint_positions[robot.dof_names.index(name)]))
        for name in ARM_TARGETS
    }

    rear_lifts = [
        after_lift[name][1] - before_lift[name][1]
        for name in ("RearLeftWheel", "RearRightWheel")
    ]
    front_lifts = [
        after_lift[name][1] - before_lift[name][1]
        for name in ("FrontLeftWheel", "FrontRightWheel")
    ]
    mean_rear_lift = sum(rear_lifts) * 0.5
    mean_front_lift = sum(front_lifts) * 0.5
    arrival_error = abs(final_robot[2] - ROBOT_TARGET_Z)
    arms_reached = all(
        abs(arm_angles[name] - target) < 3.0
        for name, target in ARM_TARGETS.items()
    )
    lift_pass = mean_rear_lift >= 0.025 and mean_rear_lift > mean_front_lift + 0.012
    passed = arrival_error < 0.15 and arms_reached and lift_pass

    report = {
        "passed": passed,
        "assets": {
            "parking": str(PARKING_SOURCE_USD),
            "parking_test_layer": str(PARKING_USD),
            "vehicle": str(VEHICLES_USD),
            "robot": str(ROBOT_USD),
            "test_stage": str(OUTPUT_USD),
        },
        "vehicle": "Sedan",
        "keep_drivetrain": KEEP_DRIVETRAIN,
        "sphere_wheels": SPHERE_WHEELS,
        "parking_bay": "A7",
        "robot_start_xyz_m": start_robot,
        # 유도가 끝난 직후와 자유 정착 60스텝 뒤를 모두 남긴다. 둘의 차이가 크면
        # 유도 순간이동이 남긴 잔류 속도를 휠이 제동하지 못했다는 뜻이다.
        "robot_pulse_xyz_m": pulse_position,
        "robot_parked_xyz_m": parked_robot,
        "robot_settle_drift_m": abs(parked_robot[2] - pulse_position[2]),
        "robot_final_xyz_m": final_robot,
        "robot_target_z_m": ROBOT_TARGET_Z,
        "arrival_error_m": arrival_error,
        "sedan_start_xyz_m": start_sedan,
        "sedan_final_xyz_m": final_sedan,
        "initial_wheel_centers_m": initial_wheels,
        "before_lift_wheel_centers_m": before_lift,
        "after_lift_wheel_centers_m": after_lift,
        "rear_wheel_lift_m": rear_lifts,
        "front_wheel_lift_m": front_lifts,
        "mean_rear_lift_m": mean_rear_lift,
        "mean_front_lift_m": mean_front_lift,
        "arm_angles_deg": arm_angles,
        "checks": {
            "robot_arrived": arrival_error < 0.15,
            "arms_reached_targets": arms_reached,
            "rear_wheels_lifted": lift_pass,
        },
    }
    REPORT_JSON.write_text(
        json.dumps(report, indent=2, ensure_ascii=False), encoding="utf-8"
    )

    print(f"TEST_PASSED={passed}", flush=True)
    print(f"ROBOT_ARRIVAL_ERROR_M={arrival_error:.6f}", flush=True)
    print(f"MEAN_REAR_LIFT_M={mean_rear_lift:.6f}", flush=True)
    print(f"MEAN_FRONT_LIFT_M={mean_front_lift:.6f}", flush=True)
    for name, angle in arm_angles.items():
        print(f"ARM_ANGLE {name}={angle:.3f}", flush=True)
    print(f"REPORT={REPORT_JSON}", flush=True)

    timeline.stop()
    app.update()
    return report


def write_exception_report(exc):
    existing = {}
    if REPORT_JSON.is_file():
        try:
            existing = json.loads(REPORT_JSON.read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError):
            existing = {}
    existing.update(
        {
            "passed": False,
            "exception_type": type(exc).__name__,
            "exception": str(exc),
        }
    )
    REPORT_JSON.write_text(
        json.dumps(existing, indent=2, ensure_ascii=False),
        encoding="utf-8",
    )


def run_gui_replay_loop(app, automated_runs=0):
    import omni.timeline
    import omni.usd

    context = omni.usd.get_context()
    timeline = omni.timeline.get_timeline_interface()
    timeline.stop()
    if not context.open_stage(str(OUTPUT_USD)):
        raise RuntimeError(f"failed to open test stage: {OUTPUT_USD}")
    for _ in range(12):
        app.update()

    print(
        "GUI_REPLAY_READY: Press Play to reset and run the complete test.",
        flush=True,
    )
    run_number = 0
    if automated_runs:
        timeline.play()
    while app.is_running():
        app.update()
        if not timeline.is_playing():
            continue

        # A Play transition is the replay trigger. Stop the just-started
        # timeline, then run_test reopens the pristine saved stage before it
        # starts physics, so every click begins from the same initial state.
        timeline.stop()
        app.update()
        run_number += 1
        print(f"GUI_REPLAY_START={run_number}", flush=True)
        try:
            preserved_camera = capture_viewport_camera()
            report = run_test(app, preserved_camera=preserved_camera)
            print(
                f"GUI_REPLAY_DONE={run_number} passed={report['passed']}",
                flush=True,
            )
            if automated_runs and not report["passed"]:
                raise RuntimeError(f"automated replay {run_number} failed")
        except Exception as exc:
            timeline.stop()
            write_exception_report(exc)
            print(
                f"GUI_REPLAY_EXCEPTION={run_number} "
                f"{type(exc).__name__}: {exc}",
                flush=True,
            )
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
        elif "--replay-smoke" in sys.argv[1:]:
            run_gui_replay_loop(app, automated_runs=2)
        else:
            report = run_test(app)
            if not report["passed"]:
                raise RuntimeError(
                    "rear-wheel lift integration test failed; see "
                    f"{REPORT_JSON}"
                )
    except Exception as exc:
        write_exception_report(exc)
        print(f"TEST_EXCEPTION={type(exc).__name__}: {exc}", flush=True)
        raise
    finally:
        app.close()


if __name__ == "__main__":
    main()
