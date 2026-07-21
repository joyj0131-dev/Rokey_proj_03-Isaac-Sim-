#!/usr/bin/env python3
"""뎁스캠 기반 정지 판단 + 뒷바퀴 리프트 통합 테스트 (Isaac Sim 5.1).

parking_robot_rear_lift_test.py 와 같은 주차장(A7) + Sedan 시나리오를 재사용하되,
로봇을 hwia_depth_cam_mecha_roller.usd(뎁스캠 4대 + 메카넘 롤러 내장)로 교체하고,
진입을 순간이동(set_world_poses) 대신 실제 /cmd_vel 기반 휠 구동으로 한다.
전방 뎁스캠(depth_stop_detector.DepthStopDetector)이 뒷축 접근을 감지하면 정지하고
팔을 전개한다. 원본 로봇/차량/주차장 에셋은 수정하지 않는다.

실행:
    python3 depth_stop_lift_test.py                    # 헤드리스
    python3 depth_stop_lift_test.py --gui               # GUI
    python3 depth_stop_lift_test.py --sphere-wheels      # Sedan 휠 충돌체를 구로(권장)
    python3 depth_stop_lift_test.py --keep-drivetrain    # Sedan PhysX Vehicle 구동계 유지
    python3 depth_stop_lift_test.py --drop-margin 0.08   # 뎁스 정지 마진 조절
"""

import json
import math
import os
import sys
from pathlib import Path

WORK_DIR = Path(__file__).resolve().parent
PARKING_SOURCE_USD = WORK_DIR / "parking" / "parking_environment.usd"
PARKING_USD = WORK_DIR / "parking" / "parking_environment_depth_stop_test.usd"
VEHICLES_USD = WORK_DIR / "fab_vehicles.usd"
ROBOT_PACKAGE = WORK_DIR.parent / "hwia_parking_robot_final_caster_package"
ROBOT_USD = ROBOT_PACKAGE / "hwia_depth_cam_mecha_roller.usd"
ROBOT_REF = f"../{ROBOT_PACKAGE.name}/{ROBOT_USD.name}"

KEEP_DRIVETRAIN = "--keep-drivetrain" in sys.argv[1:]
SPHERE_WHEELS = "--sphere-wheels" in sys.argv[1:]
DROP_MARGIN = 0.05
if "--drop-margin" in sys.argv[1:]:
    DROP_MARGIN = float(sys.argv[sys.argv.index("--drop-margin") + 1])

OUTPUT_USD = WORK_DIR / "depth_stop_lift_test.usd"
REPORT_JSON = WORK_DIR / "depth_stop_lift_test_report.json"

ISAAC_ROOT = Path("/home/rokey/dev_ws/isaac_sim/isaacsim/_build/linux-x86_64/release")
ISAAC_PYTHON = ISAAC_ROOT / "python.sh"

# A7 슬롯 — parking_robot_rear_lift_test.py 와 동일(A5 는 팀원 에셋이 A5_Coupe 로 점유).
PARKING_CENTER = (8.5, 0.0, 7.8)
SEDAN_REAR_AXLE_LOCAL_Z = (-1.5053923 - 1.4878858) * 0.5
ROBOT_START_Z = 4.55
ROBOT_TARGET_Z = PARKING_CENTER[2] + SEDAN_REAR_AXLE_LOCAL_Z

# 이 로봇 에셋(hwia_depth_cam_mecha_roller.usd)은 hwia_parking_robot_final_caster_mecha_roller.usd
# 보다 prim 계층이 한 단계 얕다 — wheel_fl 등이 base_link 밑이 아니라 루트 바로 밑에 있다.
# (verify_depth_cam_mecha.py 에서 이미 같은 상수를 쓴다.)
ROBOT_WRAP = "/World/Robot"
ROBOT_ROOT = "/World/Robot/base_link"
ROBOT_JOINTS = "/World/Robot/joints"
CAM_FRONT = "/World/Robot/cam_front_link/depth_cam_front/Camera_Pseudo_Depth_Front"
CAM_RES = (640, 480)

SEDAN_ROOT = "/World/VehicleAsset/Vehicles/Sedan"
ARM_TARGETS = {
    "arm_left_front_joint": 90.0,
    "arm_left_rear_joint": -90.0,
    "arm_right_front_joint": -90.0,
    "arm_right_rear_joint": 90.0,
}
SEDAN_WHEELS = (
    "FrontLeftWheel",
    "FrontRightWheel",
    "RearLeftWheel",
    "RearRightWheel",
)

DEPTH_SPEED = 0.4          # m/s, verify_depth_cam_mecha.py 에서 검증된 mecanum 전진 속도
DEPTH_BASELINE_FRAMES = 30
DEPTH_CONFIRM_FRAMES = 3
DEPTH_MAX_STEPS = 900       # 15s @ 60Hz 안전 컷오프 — 이보다 오래 걸리면 timeout 처리


def _restart_with_isaac_python():
    if os.environ.get("CARB_APP_PATH"):
        return
    if not ISAAC_PYTHON.is_file():
        raise FileNotFoundError(f"Isaac Sim python.sh not found: {ISAAC_PYTHON}")
    os.execv(str(ISAAC_PYTHON), [str(ISAAC_PYTHON), str(Path(__file__).resolve()), *sys.argv[1:]])


def _replace_matrix_xform(prim, matrix, UsdGeom):
    xform = UsdGeom.Xformable(prim)
    xform.ClearXformOpOrder()
    xform.MakeMatrixXform().Set(matrix)


def build_test_stage():
    from mecanum_drive import configure_hub_drives
    from pxr import Gf, PhysxSchema, Sdf, Usd, UsdGeom, UsdPhysics, UsdShade

    for path in (PARKING_SOURCE_USD, VEHICLES_USD, ROBOT_USD):
        if not path.is_file():
            raise FileNotFoundError(path)

    source_parking_layer = Sdf.Layer.FindOrOpen(str(PARKING_SOURCE_USD))
    if source_parking_layer is None:
        raise RuntimeError(f"unable to read parking layer: {PARKING_SOURCE_USD}")
    parking_layer = (
        Sdf.Layer.FindOrOpen(str(PARKING_USD))
        if PARKING_USD.is_file()
        else Sdf.Layer.CreateNew(str(PARKING_USD))
    )
    parking_layer.TransferContent(source_parking_layer)
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

    deactivate = [
        "/World/Parking/PhysicsScene",
        "/World/VehicleAsset/PhysicsScene",
        "/World/VehicleAsset/DriveGround",
        "/World/VehicleAsset/FabLighting",
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

    ground = UsdGeom.Cube.Define(stage, "/World/TestGround")
    ground.CreateSizeAttr(1.0)
    ground.CreateVisibilityAttr(UsdGeom.Tokens.invisible)
    ground_xform = UsdGeom.Xformable(ground)
    ground_xform.AddTranslateOp().Set(Gf.Vec3d(0.0, -0.10, 0.0))
    ground_xform.AddScaleOp().Set(Gf.Vec3f(30.0, 0.20, 30.0))
    UsdPhysics.CollisionAPI.Apply(ground.GetPrim())

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
        print("[depth-lift] --keep-drivetrain: Sedan PhysX Vehicle 구동계 유지", flush=True)
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
        radius = None
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
        print(f"[depth-lift] --sphere-wheels: 휠 충돌체 4개를 구(r={radius:.4f})로 교체", flush=True)

    sedan_rigid = PhysxSchema.PhysxRigidBodyAPI.Apply(sedan)
    sedan_rigid.GetDisableGravityAttr().Set(False)
    sedan_rigid.CreateEnableCCDAttr(True)
    sedan_rigid.GetSolverPositionIterationCountAttr().Set(16)
    sedan_rigid.GetSolverVelocityIterationCountAttr().Set(8)

    robot_to_world = Gf.Matrix4d(
        0.0, 0.0, 1.0, 0.0,
        1.0, 0.0, 0.0, 0.0,
        0.0, 1.0, 0.0, 0.0,
        PARKING_CENTER[0], 0.0, ROBOT_START_Z, 1.0,
    )
    _replace_matrix_xform(robot, robot_to_world, UsdGeom)

    materials = UsdGeom.Scope.Define(stage, "/World/TestMaterials").GetPath()
    grip = UsdShade.Material.Define(stage, materials.AppendChild("RobotGrip"))
    grip_api = UsdPhysics.MaterialAPI.Apply(grip.GetPrim())
    grip_api.CreateStaticFrictionAttr(1.35)
    grip_api.CreateDynamicFrictionAttr(1.10)
    grip_api.CreateRestitutionAttr(0.01)
    UsdShade.MaterialBindingAPI.Apply(ground.GetPrim()).Bind(
        grip, UsdShade.Tokens.weakerThanDescendants, "physics"
    )
    # 이 에셋은 wheel_fl 등이 base_link 밑이 아니라 /World/Robot 바로 밑에 있다(위 설명 참고).
    for link_name in (
        "wheel_fl", "wheel_fr", "wheel_rl", "wheel_rr",
        "bearing_roller_left_front", "bearing_roller_left_rear",
        "bearing_roller_right_front", "bearing_roller_right_rear",
    ):
        link = stage.GetPrimAtPath(f"{ROBOT_WRAP}/{link_name}")
        UsdShade.MaterialBindingAPI.Apply(link).Bind(
            grip, UsdShade.Tokens.weakerThanDescendants, "physics"
        )
    for wheel_name in SEDAN_WHEELS:
        wheel = stage.GetPrimAtPath(f"{SEDAN_ROOT}/{wheel_name}")
        UsdShade.MaterialBindingAPI.Apply(wheel).Bind(
            grip, UsdShade.Tokens.weakerThanDescendants, "physics"
        )

    configure_hub_drives(stage, ROBOT_JOINTS)
    for name in ARM_TARGETS:
        joint = stage.GetPrimAtPath(f"{ROBOT_JOINTS}/{name}")
        drive = UsdPhysics.DriveAPI.Get(joint, "angular")
        drive.GetStiffnessAttr().Set(1800.0)
        drive.GetDampingAttr().Set(140.0)
        drive.GetMaxForceAttr().Set(5000.0)
        drive.GetTargetPositionAttr().Set(0.0)

    world.SetCustomDataByKey("test", "depth-cam stop-detect + rear-wheel lift")
    world.SetCustomDataByKey("vehicle", "Sedan")
    world.SetCustomDataByKey("parkingBay", "A7")
    world.SetCustomDataByKey("robotTargetZ", ROBOT_TARGET_Z)
    stage.GetRootLayer().Save()


if __name__ == "__main__":
    _restart_with_isaac_python()
    # SimulationApp must exist before `pxr` (and other kit extension modules) are
    # importable — python.sh alone does not put them on sys.path.
    from isaacsim import SimulationApp
    app = SimulationApp({"headless": True})
    try:
        build_test_stage()
        print(f"STAGE_BUILT={OUTPUT_USD}")
    finally:
        app.close()
