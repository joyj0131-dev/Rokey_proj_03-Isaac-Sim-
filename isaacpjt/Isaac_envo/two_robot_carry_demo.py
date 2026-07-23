#!/usr/bin/env python3
"""Two-robot carry demo for the HWIA parking robot (Isaac Sim 5.1).

메카넘 로봇 2대가 주차장에 '이미 주차되어 있는' 차량을 앞뒤 축에서 집어 들고
앞쪽(+Z)으로 1 m 운반한다. 대상 차량은 PhysX Vehicle 구동계를 유지한 상태이며,
이것이 2026-07-20 실측으로 확정된 기본 구성이다.

기본 실행 (대상 A5_Coupe, 구동계 유지):
  cd /home/rokey/cobot3_ws/isaacpjt/Isaac_envo
  python3 two_robot_carry_demo.py

GUI로 보기 (Play를 누르면 시작, 다시 누르면 반복):
  python3 two_robot_carry_demo.py --gui
  python3 two_robot_carry_demo.py --gui --show-colliders   # 충돌체 와이어프레임까지

대상 차량 바꾸기:
  python3 two_robot_carry_demo.py --target B5_Pickup
  (Parked: A3_Compact A5_Coupe A6_Hatchback B3_Minivan B5_Pickup B7_Sedan
   HandoffQueue: H1_SUV H2_Wagon H3_Sport H4_Offroad H5_Hatchback H6_Minivan)

비교/진단용 플래그:
  --strip-drivetrain   구동계 제거(예전 기본). 원통 충돌체가 접촉체가 되어 차가 떤다
  --sphere-wheels      휠 충돌체를 구로 교체(--strip-drivetrain 필요)
  --convex-wheels      휠 충돌체를 Convex Decomposition으로 교체(--strip-drivetrain 필요)
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

# 기본 구성 = PhysX Vehicle 구동계를 '유지'한 채로 집어 옮긴다 (2026-07-20 실측으로 확정).
#
# 구동계를 떼면 원통 휠 충돌체가 실제 접촉체가 되는데, PhysX는 원기둥 프리미티브가 없어
# 면진 볼록체로 근사한다. 그래서 서스펜션 없는 차체가 면 사이를 계속 흔들거린다.
# 구(sphere)로 바꾸면 그 진동은 잡히지만 폭이 2.7배로 부풀고, Mesh 볼록체는 구동계가
# 살아있는 차량에서 폭발한다(변위 1600~2600 m).
#
# 반면 구동계를 그대로 두면 레이캐스트 서스펜션이 차체를 지지해 원통 접촉 자체가
# 발생하지 않는다. 팀원이 배치한 주차 차량 12대가 이 상태로 안정적이고, 실측에서도
# 가장 좋았다: A5_Coupe 리프트 0.0849 m(구동계 제거+구의 3배), 추종률 1.000.
#
# --strip-drivetrain 으로 예전 동작(구동계 제거)을 쓸 수 있다.
STRIP_DRIVETRAIN = "--strip-drivetrain" in sys.argv[1:]
KEEP_DRIVETRAIN = not STRIP_DRIVETRAIN

# --sphere-wheels: 휠 충돌체를 원통 -> 구로 교체. 면진 rocking은 사라지지만 폭이
# 2.7배로 부푼다. 구동계가 살아있는 차량에는 쓰지 말 것(--strip-drivetrain 필요).
SPHERE_WHEELS = "--sphere-wheels" in sys.argv[1:]

# --convex-wheels: 휠 시각 Mesh에 Convex Decomposition 충돌체를 건다. 타이어 폭은
# 정확해지지만 구동계가 살아있는 차량에서는 폭발한다(변위 2622 m). --strip-drivetrain 필요.
CONVEX_WHEELS = "--convex-wheels" in sys.argv[1:]

# --convex-hull: 위와 같되 조각 1개(convexHull). 폭발 원인이 '조각 개수'가 아니라
# 'Mesh 충돌체 자체'임을 확인한 실험용 플래그다(hull도 1627 m로 동일하게 폭발).
CONVEX_HULL_WHEELS = "--convex-hull" in sys.argv[1:]

# --target <이름>: 집어 옮길 차량. 차종마다 축거(2.344~3.594 m)와 타이어 반경
# (0.277~0.440 m)이 달라, 일반화 여부를 확인하려면 여러 차종으로 돌려봐야 한다.
def _arg_value(name):
    args = sys.argv[1:]
    if name in args:
        i = args.index(name)
        if i + 1 < len(args) and not args[i + 1].startswith("--"):
            return args[i + 1]
    return None


# 대상은 항상 팀원이 주차해 둔 차량이다. 자체 스폰 Sedan 경로는 제거했다 —
# Y=0.0에 놓여 구동계를 유지하면 서스펜션이 지면에 파묻힌 채 시작해 운반이 안 됐고
# (실제 구동 0.056 m, 차량 미상승), 애초에 실제 서비스 시나리오도 아니다.
DEFAULT_TARGET = "A5_Coupe"
TARGET_VEHICLE = _arg_value("--target") or DEFAULT_TARGET

# --show-colliders: standalone SimulationApp으로 GUI를 띄우면 물리 디버그 시각화가 꺼져 있어
# 뷰포트 메뉴에도 항목이 없다. carb 설정을 직접 켜야 충돌체 와이어프레임이 보인다.
SHOW_COLLIDERS = "--show-colliders" in sys.argv[1:]
OUTPUT_USD = WORK_DIR / "two_robot_carry_demo.usd"
REPORT_JSON = WORK_DIR / "two_robot_carry_demo_report.json"

ISAAC_ROOT = Path("/home/rokey/dev_ws/isaac_sim/isaacsim/_build/linux-x86_64/release")
ISAAC_PYTHON = ISAAC_ROOT / "python.sh"

# 아래 둘은 자리표시자다. build_test_stage()가 대상 차량의 실제 휠 좌표에서
# 계산해 덮어쓴다(차종마다 축거가 2.344~3.594 m로 달라 상수로 둘 수 없다).
PARKING_CENTER = (0.0, 0.0, 0.0)
SEDAN_ROOT = ""

ROBOT_APPROACH_GAP_M = 1.75   # 로봇이 각 축에서 이만큼 떨어진 곳에서 진입 시작

CARRY_DISTANCE_M = 1.0
CARRY_SPEED = 0.35        # m/s per robot toward world +Z during the carry
CARRY_MODE = "real"       # "real" mecanum drive, else guided fallback

# 휠 프림 이름은 차종과 무관하게 공통이다(fab_vehicles 10종 전부 동일).
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
        "start_z": 0.0,   # build_test_stage()에서 대상 차량 기준으로 설정
        "target_z": 0.0,
        "facing": +1,   # local +X -> world +Z
    },
    "front": {
        "wrap": "/World/RobotFront/base_link",
        "root": "/World/RobotFront/base_link/base_link",
        "joints": "/World/RobotFront/base_link/joints",
        "xform": "/World/RobotFront",
        "start_z": 0.0,   # build_test_stage()에서 대상 차량 기준으로 설정
        "target_z": 0.0,
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

    # 대상은 팀원이 배치한 주차 차량이다. /World/VehicleAsset의 쇼룸 차량 10대는
    # 재질(Looks)만 쓰고 지오메트리는 필요 없으므로 전부 끈다.
    vehicles = stage.GetPrimAtPath("/World/VehicleAsset/Vehicles")
    for vehicle in vehicles.GetChildren():
        vehicle.SetActive(False)

    target = None
    for group in ("Parked", "HandoffQueue"):
        candidate = stage.GetPrimAtPath(
            f"/World/Parking/ParkingVehicles/{group}/{TARGET_VEHICLE}"
        )
        if candidate.IsValid():
            target = candidate
            break
    if target is None:
        raise RuntimeError(
            f"대상 차량을 찾지 못했습니다: {TARGET_VEHICLE}. "
            "Parked/HandoffQueue 아래 이름이어야 합니다(예: A5_Coupe, B5_Pickup, H1_SUV)."
        )

    # 축 위치를 상수로 두지 않고 실제 휠 좌표에서 뽑는다(차종마다 축거가 2.344~3.594 m).
    cache = UsdGeom.XformCache()
    centers = {}
    for wheel_name in SEDAN_WHEELS:
        wheel = stage.GetPrimAtPath(f"{target.GetPath()}/{wheel_name}")
        if not wheel.IsValid():
            raise RuntimeError(f"휠을 찾지 못했습니다: {target.GetPath()}/{wheel_name}")
        centers[wheel_name] = cache.GetLocalToWorldTransform(wheel).ExtractTranslation()
    front_z = (centers["FrontLeftWheel"][2] + centers["FrontRightWheel"][2]) * 0.5
    rear_z = (centers["RearLeftWheel"][2] + centers["RearRightWheel"][2]) * 0.5
    center_x = sum(c[0] for c in centers.values()) / 4.0

    # yaw(nose-in / nose-out)에 무관하게, z가 작은 축을 -Z에서 진입하는 로봇이 맡는다.
    low_z, high_z = sorted((front_z, rear_z))
    globals()["SEDAN_ROOT"] = str(target.GetPath())
    globals()["PARKING_CENTER"] = (center_x, 0.0, (front_z + rear_z) * 0.5)
    ROBOTS["rear"]["target_z"] = low_z    # facing +1, -Z에서 접근
    ROBOTS["rear"]["start_z"] = low_z - ROBOT_APPROACH_GAP_M
    ROBOTS["front"]["target_z"] = high_z  # facing -1, +Z에서 접근
    ROBOTS["front"]["start_z"] = high_z + ROBOT_APPROACH_GAP_M
    print(
        f"[carry] 대상 {TARGET_VEHICLE}: {target.GetPath()} "
        f"(축 z={low_z:.3f}/{high_z:.3f}, 축거 {abs(high_z - low_z):.3f} m, x={center_x:.3f})",
        flush=True,
    )
    sedan = target

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
            f"[carry] --keep-drivetrain: {TARGET_VEHICLE or 'Sedan'}의 "
            "PhysX Vehicle 구동계를 유지합니다.",
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

    _convex_any = CONVEX_WHEELS or CONVEX_HULL_WHEELS
    _convex_approx = (
        UsdPhysics.Tokens.convexHull
        if CONVEX_HULL_WHEELS
        else UsdPhysics.Tokens.convexDecomposition
    )
    if sum([SPHERE_WHEELS, CONVEX_WHEELS, CONVEX_HULL_WHEELS]) > 1:
        raise RuntimeError(
            "--sphere-wheels / --convex-wheels / --convex-hull 은 하나만 쓸 수 있습니다."
        )

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

    if _convex_any:
        # 원통을 끄고, 바퀴의 시각 Mesh 자체에 Convex Decomposition 충돌체를 건다.
        # 시각 Mesh를 그대로 쓰므로 타이어 폭·지름이 실제와 일치한다.
        #
        # 씬 안의 '모든' 차량에 적용한다: 테스트 Sedan + 팀원이 배치한 12대.
        # 서비스가 결국 어느 차든 집어야 하므로 특정 1대만 고쳐두면 의미가 없다.
        # PhysX Vehicle은 충돌체가 wheel attachment 프림의 '직속 자식'일 것을 요구한다.
        # 시각 Mesh는 <Wheel>/Visual/<Mesh>로 한 단계 깊으므로 그 자리에 CollisionAPI를
        # 걸면 거부된다("has to be a direct child of the wheel attachment prim").
        # 따라서 지오메트리를 <Wheel>/CollisionMesh 로 복제하고 상대 변환을 그대로 얹는다.
        xf_cache = UsdGeom.XformCache()

        def _convex_wheels(vehicle_prim):
            done = 0
            for wheel_name in SEDAN_WHEELS:
                wheel_prim = stage.GetPrimAtPath(
                    f"{vehicle_prim.GetPath()}/{wheel_name}"
                )
                if not wheel_prim.IsValid():
                    continue
                # 차종마다 메시 이름이 다르므로(Wheel_A, Wheel_C002 …) 첫 Mesh를 찾는다.
                src = next(
                    (p for p in Usd.PrimRange(wheel_prim) if p.GetTypeName() == "Mesh"),
                    None,
                )
                if src is None:
                    continue
                src_geom = UsdGeom.Mesh(src)
                points = src_geom.GetPointsAttr().Get()
                counts = src_geom.GetFaceVertexCountsAttr().Get()
                indices = src_geom.GetFaceVertexIndicesAttr().Get()
                if not points or not counts or not indices:
                    continue
                relative, _ = xf_cache.ComputeRelativeTransform(src, wheel_prim)

                dst = UsdGeom.Mesh.Define(
                    stage, f"{wheel_prim.GetPath()}/CollisionMesh"
                )
                dst.CreatePointsAttr(points)
                dst.CreateFaceVertexCountsAttr(counts)
                dst.CreateFaceVertexIndicesAttr(indices)
                dst.CreatePurposeAttr(UsdGeom.Tokens.guide)
                UsdGeom.Xformable(dst).AddTransformOp().Set(relative)
                UsdPhysics.CollisionAPI.Apply(dst.GetPrim())
                UsdPhysics.MeshCollisionAPI.Apply(dst.GetPrim()).CreateApproximationAttr(
                    _convex_approx
                )

                # 원통은 새 충돌체를 세운 뒤에 끈다(중간에 충돌체 0개인 순간을 만들지 않음).
                cylinder = stage.GetPrimAtPath(f"{wheel_prim.GetPath()}/Collision")
                if cylinder.IsValid():
                    cylinder.SetActive(False)
                done += 1
            return done

        targets = []
        sedan_prim = stage.GetPrimAtPath(SEDAN_ROOT)
        if sedan_prim.IsValid():
            targets.append(sedan_prim)
        for group in ("Parked", "HandoffQueue"):
            root = stage.GetPrimAtPath(f"/World/Parking/ParkingVehicles/{group}")
            if root.IsValid():
                targets.extend(root.GetChildren())

        total_wheels = 0
        for vehicle_prim in targets:
            total_wheels += _convex_wheels(vehicle_prim)
        print(
            f"[carry] 휠 충돌체 교체: 차량 {len(targets)}대 / 휠 {total_wheels}개 "
            f"-> {_convex_approx}",
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

    # 대상 차량을 비추는 초기 카메라. --target으로 대상이 바뀌면 위치도 따라간다.
    # (없으면 GUI를 처음 열었을 때 대상이 화면 밖이라 매번 직접 찾아가야 한다.)
    cx, _, cz = PARKING_CENTER
    camera = UsdGeom.Camera.Define(stage, "/World/CarryDemoCamera")
    camera.CreateFocalLengthAttr(24.0)
    camera.AddTransformOp().Set(
        Gf.Matrix4d().SetLookAt(
            Gf.Vec3d(cx - 6.5, 3.2, cz - 6.0),   # 차량 뒤 비스듬히 위
            Gf.Vec3d(cx, 0.35, cz),              # 차량 중심
            Gf.Vec3d(0.0, 1.0, 0.0),
        ).GetInverse()
    )

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

    # 팀원이 배치한 주차 차량 12대의 시작 위치. --convex-wheels로 이들 휠 충돌체까지
    # 바꾸므로, 데모가 끝난 뒤 얼마나 밀렸는지 확인해 거동이 깨지지 않았는지 본다.
    live_stage = omni.usd.get_context().get_stage()
    parked_paths = []
    if live_stage and live_stage.GetPrimAtPath("/World/Parking/ParkingVehicles").IsValid():
        parked_paths = [
            str(v.GetPath())
            for group in ("Parked", "HandoffQueue")
            for v in live_stage.GetPrimAtPath(
                f"/World/Parking/ParkingVehicles/{group}"
            ).GetChildren()
        ]

    # 1) settle
    for st in arts.values():
        set_cmd_vel(st, 0.0, 0.0, 0.0)
        set_arms(st, 0.0)
    for _ in range(120):
        app.update()
    parked_start = {p: rigid_pos(p) for p in parked_paths}

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
    # 파지 유지 구간에서 차가 떠는지 잰다. 원통 충돌체의 rocking이 여기서 드러난다.
    # drift = 순변위, path = 프레임별 이동량 총합. path >> drift 이면 제자리 진동이다.
    grip_samples = []
    for _ in range(300):
        app.update()
        grip_samples.append(rigid_pos(SEDAN_ROOT))
    grip_drift_m = math.dist(grip_samples[0], grip_samples[-1])
    grip_path_m = sum(
        math.dist(a, b) for a, b in zip(grip_samples, grip_samples[1:])
    )
    grip_jitter_ratio = grip_path_m / max(grip_drift_m, 1e-4)

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
    print(
        f"[carry] 파지 유지 300프레임 떨림: 이동경로 {grip_path_m:.5f} m / "
        f"순변위 {grip_drift_m:.5f} m (떨림비 {grip_jitter_ratio:.1f})",
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

    # 주차 차량 12대가 제자리를 지켰는지. 크게 밀렸다면 휠 충돌체 교체가 이들의
    # 레이캐스트 서스펜션 거동을 깨뜨렸다는 뜻이다.
    # 대상 차량 자신은 옮겨진 것이 정상이므로 제외한다. 나머지가 제자리를 지켰는지만 본다.
    parked_drift = {
        p: math.dist(parked_start[p], rigid_pos(p))
        for p in parked_paths
        if p != SEDAN_ROOT
    }
    parked_max_drift_m = max(parked_drift.values()) if parked_drift else 0.0
    parked_worst = (
        max(parked_drift, key=parked_drift.get).split("/")[-1] if parked_drift else "-"
    )
    print(
        f"[carry] 주차 차량 {len(parked_drift)}대 최대 변위 "
        f"{parked_max_drift_m:.4f} m ({parked_worst})",
        flush=True,
    )

    # 합격 조건. 예전에는 total_moved만 봤는데, 실제 구동이 실패하면 유도 폴백이 차를
    # 대신 옮겨 기준을 통과시켜 버렸다(B5_Pickup: 리프트 0.4 mm·추종률 0.033인데 PASS).
    # 그래서 "폴백 없이 / 실제로 들어서 / 로봇과 함께" 갔는지를 모두 요구한다.
    checks = {
        "carried_far_enough": total_moved >= 0.8 * CARRY_DISTANCE_M,
        "car_upright": abs(car_final[1]) < 1.0,
        "finite": all(math.isfinite(c) for c in car_final),
        "real_drive_no_fallback": not used_guided,
        "car_actually_lifted": car_lift_m >= 0.02,
        "car_followed_robot": 0.8 <= carry_follow_ratio <= 1.2,
    }
    passed = bool(all(checks.values()))
    if not passed:
        failed = [k for k, v in checks.items() if not v]
        print(f"[carry] 불합격 항목: {', '.join(failed)}", flush=True)

    report = {
        "passed": passed,
        "keep_drivetrain": KEEP_DRIVETRAIN,
        "target_vehicle": TARGET_VEHICLE or "Sedan(self-spawned)",
        "target_root": SEDAN_ROOT,
        "sphere_wheels": SPHERE_WHEELS,
        "convex_wheels": CONVEX_WHEELS,
        "convex_hull_wheels": CONVEX_HULL_WHEELS,
        "carry_mode_requested": CARRY_MODE,
        "used_guided_fallback": used_guided,
        "car_before_grab_xyz_m": car_before_grab,
        "car_after_grab_xyz_m": car_after_grab,
        "car_final_xyz_m": car_final,
        "real_drive_forward_m": real_moved,
        "total_forward_m": total_moved,
        "carry_target_m": CARRY_DISTANCE_M,
        "car_lift_m": car_lift_m,
        "grip_path_m": grip_path_m,
        "grip_drift_m": grip_drift_m,
        "grip_jitter_ratio": grip_jitter_ratio,
        "parked_max_drift_m": parked_max_drift_m,
        "parked_worst_vehicle": parked_worst,
        "parked_count": len(parked_drift),
        "robot_carry_m": robot_carry_mean,
        "robot_carry_per_robot_m": robot_carry,
        "carry_follow_ratio": carry_follow_ratio,
        "checks": checks,
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

    # 대상 차량을 비추는 카메라로 시작한다. 이후 사용자가 시점을 바꾸면
    # capture/restore_viewport_camera가 그 시점을 반복 재생 내내 유지한다.
    try:
        from isaacsim.core.utils.viewports import set_active_viewport_camera

        set_active_viewport_camera("/World/CarryDemoCamera")
        for _ in range(4):
            app.update()
    except Exception as exc:  # 뷰포트가 없거나 카메라 경로가 바뀐 경우
        print(f"[carry] 초기 카메라 설정 건너뜀: {exc}", flush=True)

    print(
        f"GUI_REPLAY_READY: 대상={TARGET_VEHICLE or 'Sedan(자체 스폰)'} "
        f"{'구동계 유지' if KEEP_DRIVETRAIN else '구동계 제거'}. Play를 누르면 시작합니다.",
        flush=True,
    )
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


def _enable_collider_visualization():
    """standalone SimulationApp에서 충돌체 와이어프레임을 켠다.

    GUI를 스크립트로 띄우면 물리 디버그 시각화가 꺼진 상태로 시작하고 뷰포트
    메뉴(눈 아이콘 → Physics)에도 항목이 나타나지 않는다. carb 설정을 직접 써야 한다.
      SETTING_DISPLAY_COLLIDERS      : VisualizerMode(NONE=0 / SELECTED=1 / ALL=2)
      SETTING_VISUALIZATION_COLLISION_MESH : 쿠킹된 실제 충돌 메시를 그릴지 여부.
        Convex Decomposition 결과(볼록 조각들)를 눈으로 보려면 이게 켜져 있어야 한다.
    """
    import carb.settings
    from omni.physx.bindings import _physx as physx_bindings

    settings = carb.settings.get_settings()
    settings.set_int(
        physx_bindings.SETTING_DISPLAY_COLLIDERS, physx_bindings.VisualizerMode.ALL
    )
    settings.set_bool(physx_bindings.SETTING_VISUALIZATION_COLLISION_MESH, True)
    print(
        "[carry] --show-colliders: 충돌체 시각화 ON "
        "(모드=ALL, 쿠킹된 충돌 메시 표시). 초록 와이어프레임이 실제 물리 형상이다.",
        flush=True,
    )


def main():
    _restart_with_isaac_python()
    from isaacsim import SimulationApp

    app = SimulationApp({"headless": "--gui" not in sys.argv[1:]})
    try:
        if SHOW_COLLIDERS:
            _enable_collider_visualization()
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
