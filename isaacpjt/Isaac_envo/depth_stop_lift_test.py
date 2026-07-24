#!/usr/bin/env python3
"""뎁스캠 기반 정지 판단 + 뒷바퀴 리프트 통합 테스트 (Isaac Sim 5.1).

parking_robot_rear_lift_test.py 와 같은 주차장(A7) 시나리오를 재사용하되, 차량은
fab_vehicles.usd 의 Pickup(dock_lift_handoff_runner.py 의 인계 대상과 동일 — 언더바디
0.243m > 로봇 높이 0.18m 라 로봇이 차체 밑을 실제로 통과할 수 있음, Sedan/Coupe 는 여유가
없거나 불가능했다)으로 바꾼다. 로봇을 hwia_depth_cam_mecha_roller.usd(뎁스캠 4대 + 메카넘
롤러 내장)로 교체하고, 진입을 순간이동(set_world_poses) 대신 실제 /cmd_vel 기반 휠 구동으로
한다. 좌측 사이드 뎁스캠(depth_stop_detector.DepthStopDetector)이 뒷바퀴가 옆을 지나가는
근접 신호를 감지하면 정지하고 팔을 전개한다 — 전방 카메라는 로봇이 축에 도착하는 순간
바퀴가 정면에서 옆으로 빠져 시야 밖으로 나가버리는 구조적 문제가 있어(실측으로 확인)
로봇 길이 중심에서 바깥을 보는 사이드 카메라로 교체했다. 원본 로봇/차량/주차장 에셋은
수정하지 않는다.

실행:
    python3 depth_stop_lift_test.py                    # 헤드리스
    python3 depth_stop_lift_test.py --gui               # GUI
    python3 depth_stop_lift_test.py --sphere-wheels      # Pickup 휠 충돌체를 구로(권장)
    python3 depth_stop_lift_test.py --keep-drivetrain    # Pickup PhysX Vehicle 구동계 유지
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
# 사이드캠 중앙 ROI 배경값 0.333m. 실제 통합 실행(run_test, per-frame 해상도)에서 실측한
# 진짜 최저값은 0.2407m(diag_side_cam.py 의 0.214m 는 10스텝 간격 샘플링 오차) — margin=0.10
# (threshold=0.233)은 이 최저값보다 낮아 한 번도 안 트리거되고 timeout 났다. margin=0.08
# (threshold=0.253)로 낮춰 확실히 트리거되게 한다. 하강 구간이 급격히 떨어진 뒤 넓게
# 평평해지는 모양이라(diag_side_cam.py z=5.5~6.2 구간이 0.241~0.242 로 거의 평평) margin을
# 조금 더 키운다고 트리거 시점이 목표 축(z=5.869)에 크게 더 가까워지지는 않는다 — 이번
# 구현의 남은 과제로 depth_trace 에 남긴다.
DROP_MARGIN = 0.08
if "--drop-margin" in sys.argv[1:]:
    DROP_MARGIN = float(sys.argv[sys.argv.index("--drop-margin") + 1])

OUTPUT_USD = WORK_DIR / "depth_stop_lift_test.usd"
REPORT_JSON = WORK_DIR / "depth_stop_lift_test_report.json"

ISAAC_ROOT = Path("/home/rokey/dev_ws/isaac_sim/isaacsim/_build/linux-x86_64/release")
ISAAC_PYTHON = ISAAC_ROOT / "python.sh"

# A7 슬롯 — parking_robot_rear_lift_test.py 와 동일(A5 는 팀원 에셋이 A5_Coupe 로 점유).
PARKING_CENTER = (8.5, 0.0, 7.8)
# Pickup 뒷축 local Z — fab_vehicles.usd 실측(RearLeftWheel/RearRightWheel translate.z, 둘 다
# 동일값). dock_lift_handoff_runner.py 의 AXLE 계산과 일치(rear=-1.93, front=+1.66, 휠베이스 3.59m).
PICKUP_REAR_AXLE_LOCAL_Z = -1.9309189453124995
ROBOT_START_Z = 4.55
ROBOT_TARGET_Z = PARKING_CENTER[2] + PICKUP_REAR_AXLE_LOCAL_Z

# 이 로봇 에셋(hwia_depth_cam_mecha_roller.usd)은 hwia_parking_robot_final_caster_mecha_roller.usd
# 보다 prim 계층이 한 단계 얕다 — wheel_fl 등이 base_link 밑이 아니라 루트 바로 밑에 있다.
# (probe_depth_cam_stop.py 실측 + 직접 stage 조회로 확인됨.)
ROBOT_WRAP = "/World/Robot"
ROBOT_ROOT = "/World/Robot/base_link"
ROBOT_JOINTS = "/World/Robot/joints"
CAM_FRONT = "/World/Robot/cam_front_link/depth_cam_front/Camera_Pseudo_Depth_Front"
# 좌측 사이드 뎁스캠 — 정지 판단은 이 카메라로 한다(아래 DEPTH_SPEED 밑 설명 참고).
# 로봇 로컬 x=0(로봇 길이 중심)에 장착, 바깥쪽(왼쪽)을 향하도록 로컬에서 X축 기준 90도
# 회전돼 있다(실측, inspect_side_cams.py). local x=0 이므로 이 카메라의 world Z 위치는
# ROBOT_ROOT(rigid_position(ROBOT_ROOT)[2]) 와 사실상 같다 — 뒷바퀴가 이 카메라 옆을
# 지나가는 순간이 곧 뒷축이 로봇 중심과 나란해지는 순간이다.
CAM_LEFT = "/World/Robot/cam_side_left_link/depth_cam_left/Camera_Pseudo_Depth_Left"
CAM_RES = (640, 480)

PICKUP_ROOT = "/World/VehicleAsset/Vehicles/Pickup"
ARM_TARGETS = {
    "arm_left_front_joint": 90.0,
    "arm_left_rear_joint": -90.0,
    "arm_right_front_joint": -90.0,
    "arm_right_rear_joint": 90.0,
}
PICKUP_WHEELS = (
    "FrontLeftWheel",
    "FrontRightWheel",
    "RearLeftWheel",
    "RearRightWheel",
)

DEPTH_SPEED = 0.4          # m/s, verify_depth_cam_mecha.py 에서 검증된 mecanum 전진 속도
DEPTH_BASELINE_FRAMES = 30
DEPTH_CONFIRM_FRAMES = 3
# 전방 카메라(CAM_FRONT, 바닥을 향함)로 시도한 첫 설계는 실패했다: Pickup 언더바디 여유
# (0.243m)가 로봇+카메라 높이보다 넉넉해 바닥을 향한 빔이 차량 밑을 그냥 통과해 버려서
# ROI를 어디로 옮겨도(하단/상단) 차량과 무관한 값만 나왔다(diag_depth_profile.py,
# diag_depth_columns.py, 1200스텝 실측). 더 근본적인 문제: 전방 카메라는 로봇이 실제로
# 축 위치에 도착하는 순간 바퀴가 "정면"에서 "옆"으로 빠져 시야 밖으로 나가버린다 —
# 도착 시점을 감지해야 하는데 정확히 그 시점에 안 보이는 구조적 모순.
#
# 그래서 CAM_LEFT(측면 카메라, 로봇 길이 중심에서 바깥을 향함)로 교체했다. 이 카메라는
# 바퀴가 다가오고 멀어지는 전체 과정을 옆에서 계속 볼 수 있다. 실측(diag_side_cam.py,
# 로봇을 축보다 훨씬 뒤(z=4.0)에서 출발시켜 축을 지나 한참 더(z=6.7)까지 관찰):
# 중앙 40%x40% ROI 최소뎁스가 배경 0.333m -> 뒷축 통과 구간(z=5.7~5.9, 목표 축 정확히
# 그 사이)에서 0.214m까지 뚜렷하게 떨어졌다가 통과 후 다시 0.333m로 회복 — 차량과
# 무관하게 고정된 값을 내던 전방 카메라와 달리, 축 위치와 뚜렷하게 상관된 신호다.
DEPTH_ROI_FRAC = (0.30, 0.70, 0.30, 0.70)  # (col_lo, col_hi, row_lo, row_hi) — 사이드캠 중앙
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
        if vehicle.GetName() != "Pickup":
            vehicle.SetActive(False)
    pickup = stage.GetPrimAtPath(PICKUP_ROOT)
    pickup_matrix = UsdGeom.Xformable(pickup).GetLocalTransformation()
    pickup_matrix.SetTranslate(Gf.Vec3d(*PARKING_CENTER))
    _replace_matrix_xform(pickup, pickup_matrix, UsdGeom)

    pickup_single_apis = (
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
        print("[depth-lift] --keep-drivetrain: Pickup PhysX Vehicle 구동계 유지", flush=True)
    else:
        for api_schema in pickup_single_apis:
            if pickup.HasAPI(api_schema):
                pickup.RemoveAPI(api_schema)
        for instance_name in (PhysxSchema.Tokens.brakes0, PhysxSchema.Tokens.brakes1):
            if pickup.HasAPI(PhysxSchema.PhysxVehicleBrakesAPI, instance_name):
                pickup.RemoveAPI(PhysxSchema.PhysxVehicleBrakesAPI, instance_name)
        for wheel_name in PICKUP_WHEELS:
            wheel = stage.GetPrimAtPath(f"{PICKUP_ROOT}/{wheel_name}")
            for api_schema in wheel_apis:
                if wheel.HasAPI(api_schema):
                    wheel.RemoveAPI(api_schema)

    if SPHERE_WHEELS:
        radius = None
        for wheel_name in PICKUP_WHEELS:
            wheel_path = f"{PICKUP_ROOT}/{wheel_name}"
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

    pickup_rigid = PhysxSchema.PhysxRigidBodyAPI.Apply(pickup)
    pickup_rigid.GetDisableGravityAttr().Set(False)
    pickup_rigid.CreateEnableCCDAttr(True)
    pickup_rigid.GetSolverPositionIterationCountAttr().Set(16)
    pickup_rigid.GetSolverVelocityIterationCountAttr().Set(8)

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
    for wheel_name in PICKUP_WHEELS:
        wheel = stage.GetPrimAtPath(f"{PICKUP_ROOT}/{wheel_name}")
        UsdShade.MaterialBindingAPI.Apply(wheel).Bind(
            grip, UsdShade.Tokens.weakerThanDescendants, "physics"
        )

    # 뎁스캠 자기 오클루전 수정. 최초 실행에서 roi_min_depth가 900스텝(4m+ 주행, 차량
    # 통과 전후 포함) 내내 0.078±0.001m 로 완전히 고정되어 차량 접근에 전혀 반응하지
    # 않았다 — 세로 하단 60%·가로 10구간 전부에서 동일(diag_depth_columns.py 실측).
    # 원인을 find_occluder.py 로 카메라 기준 거리순 전수 조사해 특정: 진짜 범인은
    # RSD455 하우징(0.4~6mm, 안전하지만 부차적으로 같이 정리)이 아니라
    # base_link/visuals/front_accent/box — 로봇 전폭(0.336m)에 걸친 얇은 장식 범퍼 판이
    # 카메라 원점에서 0.124m 거리(가시 상태)에 있어 하단 시야 전체를 가린다. 원본 에셋은
    # 건드리지 않고 이 테스트 stage 참조본에서만 둘 다 invisible 로 덮어써 제외한다
    # (둘 다 순수 비주얼 메시 — 콜리전은 별도 collisions 스코프에 있으므로 물리엔 영향 없음).
    for occluder_path in (
        f"{ROBOT_WRAP}/cam_front_link/depth_cam_front/RSD455/Visual",
        f"{ROBOT_ROOT}/visuals/front_accent",
    ):
        occluder = stage.GetPrimAtPath(occluder_path)
        if occluder.IsValid():
            UsdGeom.Imageable(occluder).CreateVisibilityAttr(UsdGeom.Tokens.invisible)

    configure_hub_drives(stage, ROBOT_JOINTS)
    for name in ARM_TARGETS:
        joint = stage.GetPrimAtPath(f"{ROBOT_JOINTS}/{name}")
        drive = UsdPhysics.DriveAPI.Get(joint, "angular")
        drive.GetStiffnessAttr().Set(1800.0)
        drive.GetDampingAttr().Set(140.0)
        drive.GetMaxForceAttr().Set(5000.0)
        drive.GetTargetPositionAttr().Set(0.0)

    world.SetCustomDataByKey("test", "depth-cam stop-detect + rear-wheel lift")
    world.SetCustomDataByKey("vehicle", "Pickup")
    world.SetCustomDataByKey("parkingBay", "A7")
    world.SetCustomDataByKey("robotTargetZ", ROBOT_TARGET_Z)
    stage.GetRootLayer().Save()


def write_exception_report(exc):
    existing = {}
    if REPORT_JSON.is_file():
        try:
            existing = json.loads(REPORT_JSON.read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError):
            existing = {}
    existing.update({
        "passed": False,
        "exception_type": type(exc).__name__,
        "exception": str(exc),
    })
    REPORT_JSON.write_text(json.dumps(existing, indent=2, ensure_ascii=False), encoding="utf-8")


def _depth_to_hw(depth_raw, rgba_hw):
    """카메라 원시 뎁스 프레임을 (height, width) 2D로 정규화한다.

    probe_depth_cam_stop.py 로 실측한 축 순서(height_width)를 따른다: rgba_hw(=(height,width))와
    앞 두 축이 일치하면 그대로, 뒤집혀 있으면 전치한다.
    """
    import numpy as np

    if depth_raw is None or not getattr(depth_raw, "size", 0):
        return None
    arr = np.asarray(depth_raw, dtype=np.float64).squeeze()
    if arr.ndim != 2:
        return None
    if arr.shape == rgba_hw:
        return arr
    if arr.shape == rgba_hw[::-1]:
        return arr.T
    return arr  # 예상 밖 shape — 호출측에서 roi_min_depth 가 ValueError 로 드러낸다


def run_test(app):
    import numpy as np
    import omni.physx
    import omni.timeline
    import omni.usd
    from isaacsim.core.prims import Articulation
    from isaacsim.sensors.camera import Camera
    from pxr import Usd, UsdGeom

    from depth_stop_detector import DepthStopDetector, roi_min_depth
    from mecanum_drive import WHEEL_JOINTS, wheel_velocities_from_cmd_vel

    context = omni.usd.get_context()
    if not context.open_stage(str(OUTPUT_USD)):
        raise RuntimeError(f"failed to open test stage: {OUTPUT_USD}")
    for _ in range(12):
        app.update()

    stage = context.get_stage()
    timeline = omni.timeline.get_timeline_interface()
    timeline.play()
    for _ in range(12):
        app.update()

    robot = Articulation(ROBOT_ROOT)
    robot.initialize()
    physx = omni.physx.get_physx_interface()

    joint_positions = robot.get_joint_positions()
    command_shape = joint_positions.shape
    wheel_idx = {w: robot.dof_names.index(j) for w, j in WHEEL_JOINTS.items()}
    arm_indices = {name: robot.dof_names.index(name) for name in ARM_TARGETS}
    velocity_targets = np.zeros(command_shape, dtype=np.float32)
    position_targets = np.array(joint_positions, dtype=np.float32, copy=True)

    def rigid_position(path):
        value = physx.get_rigidbody_transformation(path)
        return tuple(float(x) for x in value["position"])

    def wheel_center(name):
        prim = stage.GetPrimAtPath(f"{PICKUP_ROOT}/{name}")
        matrix = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
        return tuple(float(x) for x in matrix.ExtractTranslation())

    def drive(vx, vy, wz):
        for wname, omega in wheel_velocities_from_cmd_vel(vx, vy, wz).items():
            i = wheel_idx[wname]
            if velocity_targets.ndim == 2:
                velocity_targets[0, i] = omega
            else:
                velocity_targets[i] = omega
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

    # 정착
    drive(0.0, 0.0, 0.0)
    set_arm_targets(0.0)
    for _ in range(120):
        app.update()

    start_robot = rigid_position(ROBOT_ROOT)
    start_pickup = rigid_position(PICKUP_ROOT)
    initial_wheels = {name: wheel_center(name) for name in PICKUP_WHEELS}

    cam = Camera(prim_path=CAM_LEFT, resolution=CAM_RES)
    cam.initialize()
    cam.add_distance_to_image_plane_to_frame()
    for _ in range(30):
        app.update()
    rgba_hw = tuple(int(v) for v in cam.get_rgba().shape[:2])

    detector = DepthStopDetector(
        baseline_frames=DEPTH_BASELINE_FRAMES,
        drop_margin=DROP_MARGIN,
        confirm_frames=DEPTH_CONFIRM_FRAMES,
    )
    depth_trace = []
    stop_reason = "timeout"
    drive(DEPTH_SPEED, 0.0, 0.0)
    for step in range(1, DEPTH_MAX_STEPS + 1):
        app.update()
        depth_hw = _depth_to_hw(cam.get_depth(), rgba_hw)
        roi_value = roi_min_depth(depth_hw, roi_frac=DEPTH_ROI_FRAC) if depth_hw is not None else math.inf
        p = rigid_position(ROBOT_ROOT)
        depth_trace.append({
            "step": step,
            "x": round(p[0], 4),
            "z": round(p[2], 4),
            "roi_min_depth_m": None if math.isinf(roi_value) else round(roi_value, 4),
        })
        if detector.update(step, roi_value):
            stop_reason = "depth_trigger"
            break
    drive(0.0, 0.0, 0.0)
    for _ in range(60):
        app.update()

    stop_position = rigid_position(ROBOT_ROOT)
    before_lift = {name: wheel_center(name) for name in PICKUP_WHEELS}

    for ramp_step in range(1, 181):
        set_arm_targets(ramp_step / 180.0)
        app.update()
    for _ in range(360):
        app.update()

    after_lift = {name: wheel_center(name) for name in PICKUP_WHEELS}
    final_robot = rigid_position(ROBOT_ROOT)
    final_pickup = rigid_position(PICKUP_ROOT)

    live_positions = robot.get_joint_positions()
    if getattr(live_positions, "ndim", 1) == 2:
        live_positions = live_positions[0]
    arm_angles = {
        name: math.degrees(float(live_positions[robot.dof_names.index(name)]))
        for name in ARM_TARGETS
    }

    rear_lifts = [after_lift[n][1] - before_lift[n][1] for n in ("RearLeftWheel", "RearRightWheel")]
    front_lifts = [after_lift[n][1] - before_lift[n][1] for n in ("FrontLeftWheel", "FrontRightWheel")]
    mean_rear_lift = sum(rear_lifts) * 0.5
    mean_front_lift = sum(front_lifts) * 0.5
    arrival_error = abs(stop_position[2] - ROBOT_TARGET_Z)
    arms_reached = all(abs(arm_angles[n] - t) < 3.0 for n, t in ARM_TARGETS.items())
    lift_pass = mean_rear_lift >= 0.025 and mean_rear_lift > mean_front_lift + 0.012
    depth_stop_ok = stop_reason == "depth_trigger"
    passed = depth_stop_ok and arrival_error < 0.15 and arms_reached and lift_pass

    report = {
        "passed": passed,
        "assets": {
            "parking": str(PARKING_SOURCE_USD),
            "vehicle": str(VEHICLES_USD),
            "robot": str(ROBOT_USD),
            "test_stage": str(OUTPUT_USD),
        },
        "vehicle": "Pickup",
        "keep_drivetrain": KEEP_DRIVETRAIN,
        "sphere_wheels": SPHERE_WHEELS,
        "parking_bay": "A7",
        "robot_start_xyz_m": start_robot,
        "robot_stop_xyz_m": stop_position,
        "robot_final_xyz_m": final_robot,
        "robot_target_z_m": ROBOT_TARGET_Z,
        "arrival_error_m": arrival_error,
        "pickup_start_xyz_m": start_pickup,
        "pickup_final_xyz_m": final_pickup,
        "initial_wheel_centers_m": initial_wheels,
        "before_lift_wheel_centers_m": before_lift,
        "after_lift_wheel_centers_m": after_lift,
        "rear_wheel_lift_m": rear_lifts,
        "front_wheel_lift_m": front_lifts,
        "mean_rear_lift_m": mean_rear_lift,
        "mean_front_lift_m": mean_front_lift,
        "arm_angles_deg": arm_angles,
        "depth_baseline_m": detector.baseline,
        "depth_threshold_m": detector.threshold,
        "depth_drop_margin_m": DROP_MARGIN,
        "depth_stop_value_m": detector.trigger_value,
        "stop_step": detector.trigger_step,
        "stop_reason": stop_reason,
        "depth_trace": depth_trace,
        "checks": {
            "depth_stop_triggered": depth_stop_ok,
            "robot_arrived": arrival_error < 0.15,
            "arms_reached_targets": arms_reached,
            "rear_wheels_lifted": lift_pass,
        },
    }
    REPORT_JSON.write_text(json.dumps(report, indent=2, ensure_ascii=False), encoding="utf-8")

    print(f"TEST_PASSED={passed}", flush=True)
    print(f"DEPTH_STOP_REASON={stop_reason}", flush=True)
    print(f"DEPTH_BASELINE_M={detector.baseline}", flush=True)
    print(f"ROBOT_ARRIVAL_ERROR_M={arrival_error:.6f}", flush=True)
    print(f"MEAN_REAR_LIFT_M={mean_rear_lift:.6f}", flush=True)
    print(f"MEAN_FRONT_LIFT_M={mean_front_lift:.6f}", flush=True)
    print(f"REPORT={REPORT_JSON}", flush=True)

    timeline.stop()
    app.update()
    return report


def main():
    _restart_with_isaac_python()
    from isaacsim import SimulationApp

    gui = "--gui" in sys.argv[1:]
    app = SimulationApp({"headless": not gui})
    try:
        build_test_stage()
        report = run_test(app)
        if gui:
            while app.is_running():
                app.update()
        elif not report["passed"]:
            raise RuntimeError(f"depth-stop lift test failed; see {REPORT_JSON}")
    except Exception as exc:
        write_exception_report(exc)
        print(f"TEST_EXCEPTION={type(exc).__name__}: {exc}", flush=True)
        raise
    finally:
        app.close()


if __name__ == "__main__":
    main()
