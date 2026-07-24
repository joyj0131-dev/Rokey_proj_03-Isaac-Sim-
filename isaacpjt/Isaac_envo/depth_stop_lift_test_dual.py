#!/usr/bin/env python3
"""로봇 2대 동시 진입 + 뎁스캠 기반 정지 + 앞/뒤축 동시 리프트 (Isaac Sim 5.1).

depth_stop_lift_test.py(로봇 1대, 뒷축만)를 검증한 뒤 이어지는 다음 단계 — 로봇 2대가
같은 쪽(차량 뒤쪽 진입로)에서 동시에 굴러 들어가, 한 대는 앞축에서, 다른 한 대는
뒷축에서 각자 좌측 사이드 뎁스캠으로 접근을 감지해 멈추고 팔을 전개한다.

두 로봇이 같은 중심선(X=PARKING_CENTER[0])을 따라 한 줄로 들어가므로, 더 먼 목표(앞축)를
맡은 로봇이 처음부터 앞서 있어야 한다 — 뒷축 로봇이 먼저 멈춘 자리를 나중에 앞축 로봇이
통과할 필요가 없도록. 그래서 RobotFront 는 ROBOT_START_Z 에서, RobotRear 는 그보다
ROBOT_GAP_M 만큼 뒤에서 동시에 출발한다(같이 있다가 동시에 진입).

정지 판단은 좌표를 전혀 쓰지 않는다: 뎁스가 기준선(baseline) 아래로 떨어지는 "진입 위치"와
다시 기준선 위로 올라오는 "이탈 위치"를 기록해 그 중간값으로 후진 정렬한다. 바퀴는 곡면
(타이어)이라 "가장 가까운 지점(최솟값)"만 쫓으면 축 중심과 다른 위치가 나와 차종별
보정값이 필요했지만(초기 버전), 진입/이탈 중간값은 대칭 기하 성질만으로 보정값 없이도
축 중심에 근접한다(실측 오차 14mm 수준).

실행:
    python3 depth_stop_lift_test_dual.py                    # 헤드리스 (기본 차량: Pickup)
    python3 depth_stop_lift_test_dual.py --gui               # GUI
    python3 depth_stop_lift_test_dual.py --sphere-wheels      # 휠 충돌체를 구로(권장)
    python3 depth_stop_lift_test_dual.py --target SUV         # 다른 차종으로

차종 바꾸기(--target): 진입/이탈 중간값 방식은 차종 전용 보정값이 필요 없으므로, 축
위치를 하드코딩하지 않고 build_test_stage() 에서 선택된 차량의 실제 휠 좌표로부터
매번 새로 계산한다(two_robot_carry_demo.py 의 기존 관례와 동일). fab_vehicles.usd 의
차량 10종(Compact/Coupe/Hatchback/Minivan/Offroad/Pickup/Sedan/Sport/SUV/Wagon) 모두
휠 prim 이름이 공통(FrontLeftWheel 등)이라 그대로 재사용된다.
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


def _arg_value(name):
    args = sys.argv[1:]
    if name in args:
        i = args.index(name)
        if i + 1 < len(args) and not args[i + 1].startswith("--"):
            return args[i + 1]
    return None


TARGET_VEHICLE = _arg_value("--target") or "Pickup"
KEEP_DRIVETRAIN = "--keep-drivetrain" in sys.argv[1:]
SPHERE_WHEELS = "--sphere-wheels" in sys.argv[1:]
# GUI 모드일 때만 로봇이 실제로 보는 뎁스 이미지 + 실시간 수치를 별도 창으로 띄운다
# (뎁스 기반 정지 판단이 실제로 작동한다는 걸 눈으로 보여달라는 요청).
SHOW_DEPTH_VIEW = "--gui" in sys.argv[1:]
# run_test() 가 GUI Play 버튼으로 재실행될 때마다 창을 새로 만들면 중복 창이 쌓인다 —
# 창/위젯은 첫 실행에서만 만들고 이후 재실행에서는 재사용한다.
_DEPTH_UI_CACHE = None
# depth_stop_lift_test.py(단일 로봇)는 0.08 로 성공했지만, dual 버전 3회 실행에서 딥 깊이가
# 매 실행 0.241/0.2407/0.273 로 흔들리는 것을 확인했다(sphere-wheels 접촉이 매번 미세하게
# 달라지는 듯) — 0.273 인 실행에서는 margin=0.08(threshold=0.253)이 dip 최저값(0.273)보다도
# 낮아 아예 트리거가 안 됐다. margin 을 낮춰 신뢰성을 우선한다(정지 위치 오차는 이미 알려진
# 과제 — depth_stop_lift_test.py 주석 참고).
DROP_MARGIN = 0.05
if "--drop-margin" in sys.argv[1:]:
    DROP_MARGIN = float(sys.argv[sys.argv.index("--drop-margin") + 1])

OUTPUT_USD = WORK_DIR / "depth_stop_lift_test_dual.usd"
REPORT_JSON = WORK_DIR / "depth_stop_lift_test_dual_report.json"

ISAAC_ROOT = Path("/home/rokey/dev_ws/isaac_sim/isaacsim/_build/linux-x86_64/release")
ISAAC_PYTHON = ISAAC_ROOT / "python.sh"

# A7 슬롯 — depth_stop_lift_test.py 와 동일.
PARKING_CENTER = (8.5, 0.0, 7.8)
# 아래 둘은 자리표시자다 — build_test_stage() 가 TARGET_VEHICLE 의 실제 휠 좌표에서
# 계산해 덮어쓴다(차종마다 축거·타이어 반경이 다르다. two_robot_carry_demo.py 와 동일 관례).
REAR_TARGET_Z = 0.0
FRONT_TARGET_Z = 0.0

ROBOT_START_Z = 4.55       # depth_stop_lift_test.py 와 동일한 진입로 시작점
# 두 로봇이 같은 중심선을 따라 한 줄로 들어간다 — 더 먼 목표(앞축)를 맡은 로봇이 앞서
# 있어야 뒷축 로봇이 멈춘 자리를 나중에 통과할 필요가 없다. 첫 시도에서 1.75m(기존
# two_robot_carry_demo.py 관례값)로 뒀다가 실측(robot_bbox_length.py)해보니 로봇 전체
# 길이(팔 포함)가 3.42m 나 되어 시작부터 1.67m 겹쳐 물리 충돌로 서로 튕겨나갔다
# (X가 최대 1.17m 옆으로 밀림, arrival_error 급증). 로봇 전체 길이보다 넉넉히 크게 잡는다.
ROBOT_GAP_M = 4.0

# 이 로봇 에셋은 wheel_fl 등이 base_link 밑이 아니라 루트 바로 밑에 있다
# (depth_stop_lift_test.py 에서 이미 확인됨).
VEHICLE_ROOT = f"/World/VehicleAsset/Vehicles/{TARGET_VEHICLE}"
ARM_TARGETS = {
    "arm_left_front_joint": 90.0,
    "arm_left_rear_joint": -90.0,
    "arm_right_front_joint": -90.0,
    "arm_right_rear_joint": 90.0,
}
VEHICLE_WHEELS = (
    "FrontLeftWheel",
    "FrontRightWheel",
    "RearLeftWheel",
    "RearRightWheel",
)

DEPTH_SPEED = 0.4
DEPTH_BASELINE_FRAMES = 30
DEPTH_CONFIRM_FRAMES = 3
# 화면 정중앙의 작은 패치만 본다. 넓은 ROI(과거 40%x40%)로 "가장 가까운 지점의 최솟값"을
# 찾으면 바퀴 곡면 때문에 축 중심과 다른 위치에서 최솟값이 나와(반지름·장착 위치에 좌우되는
# 차종별 오차, 최대 0.2~0.34m) 차종마다 보정값을 다시 재야 했다. 대신 "기준선 아래로
# 떨어지는 순간의 위치"와 "다시 올라오는 순간의 위치"의 중간값을 쓰면(아래
# EXIT_CONFIRM_FRAMES 설명) 바퀴를 대칭적으로 스쳐 지나간다는 기하학적 성질만으로 축
# 중심에 가까워진다 — 차종 전용 보정값이 필요 없다.
# 처음 10x10px(약 1.5%x2%)로 시도했으나 패치가 너무 작아 유효 픽셀이 없는 프레임(None)이
# 자주 나와 이탈 판정이 한참 뒤늦게 엉뚱한 곳에서 걸리는 문제가 실측으로 확인됐다
# (rear arrival_error 3.4m). 12%x12% 로 넓혀 안정적으로 값이 나오게 했다 — 그래도
# 원래 40%x40% 보다는 훨씬 좁아 대칭성은 유지된다.
DEPTH_ROI_FRAC = (0.44, 0.56, 0.44, 0.56)
DEPTH_MAX_STEPS = 1800      # 진입~이탈~후진 정렬 여유까지 포함
# skip_triggers 로 딥 하나를 무시한 뒤 다음 딥을 다시 찾기 전에 기다리는 스텝 수.
# 트로프 폭이 실측상 대략 130~150스텝이라(diag_side_cam.py) 확실히 벗어나도록 여유를 둔다.
SKIP_COOLDOWN_STEPS = 250

# 진입 때 쓴 것과 "같은" 임계값(baseline - DROP_MARGIN)을 이탈 판정에도 그대로 쓴다 —
# 진입/이탈에 다른 기준을 쓰면 대칭이 깨져 중간값이 축 중심에서 벗어난다. 이탈도 진입과
# 동일하게 EXIT_CONFIRM_FRAMES 연속 확인해 잡음에 의한 오탐을 막는다.
EXIT_CONFIRM_FRAMES = 3
RETURN_SPEED = 0.15   # 후진은 더 천천히(정밀 정렬)

# 좌우(X) 드리프트를 중심선(PARKING_CENTER[0])으로 계속 되돌리는 비례 게인 [1/s].
# 값이 크면 진동, 작으면 드리프트를 못 잡는다 — 6.5cm 오차를 부드럽게 잡을 정도로.
LATERAL_KP = 0.5

# 다 들어올린 뒤 두 로봇을 같이 조금 전진시켜 차량을 운반하는 거리/속도(사용자 요청).
CARRY_DISTANCE_M = 0.3
CARRY_SPEED = 0.15

# 로봇별 설정. wrap/root/joints/cam_left 는 build_test_stage() 에서 xform 기준으로 채운다.
ROBOTS = {
    "front": {
        "xform": "/World/RobotFront",
        "start_z": ROBOT_START_Z,              # 앞선 출발(더 먼 목표)
        "target_z": FRONT_TARGET_Z,
        "vehicle_wheels": ("FrontLeftWheel", "FrontRightWheel"),
        # front 로봇은 차량 밑을 지나가는 경로 순서상 뒷축을 먼저 지나친 뒤에야 앞축에
        # 도착한다 — 뎁스 딥이 두 번(뒷축, 앞축) 온다. 첫 딥(뒷축)에서 멈추면 안 되므로
        # 1번은 무시하고 재보정한 뒤 2번째 딥에서 멈춘다(첫 실행에서 실측: 뒷축 딥에서
        # 잘못 멈춰 arrival_error 1.94m 이 났다 — depth_stop_lift_test.py 단일 로봇 실행과
        # 똑같은 stop_step/trigger_value 로 확인).
        "skip_triggers": 1,
    },
    "rear": {
        "xform": "/World/RobotRear",
        "start_z": ROBOT_START_Z - ROBOT_GAP_M,  # 뒤따라 출발(더 가까운 목표)
        "target_z": REAR_TARGET_Z,
        "vehicle_wheels": ("RearLeftWheel", "RearRightWheel"),
        "skip_triggers": 0,
    },
}
for _name, _cfg in ROBOTS.items():
    _wrap = _cfg["xform"]
    _cfg["wrap"] = _wrap
    _cfg["root"] = f"{_wrap}/base_link"
    _cfg["joints"] = f"{_wrap}/joints"
    _cfg["cam_left"] = f"{_wrap}/cam_side_left_link/depth_cam_left/Camera_Pseudo_Depth_Left"

CAM_RES = (640, 480)


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

    robot_prims = {}
    for name, cfg in ROBOTS.items():
        robot = UsdGeom.Xform.Define(stage, cfg["xform"]).GetPrim()
        robot.GetReferences().AddReference(ROBOT_REF)
        robot_prims[name] = robot

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
        if vehicle.GetName() != TARGET_VEHICLE:
            vehicle.SetActive(False)
    target_vehicle = stage.GetPrimAtPath(VEHICLE_ROOT)
    if not target_vehicle.IsValid():
        raise RuntimeError(f"차량을 찾지 못했습니다: {VEHICLE_ROOT}")
    vehicle_matrix = UsdGeom.Xformable(target_vehicle).GetLocalTransformation()
    vehicle_matrix.SetTranslate(Gf.Vec3d(*PARKING_CENTER))
    _replace_matrix_xform(target_vehicle, vehicle_matrix, UsdGeom)

    # 축 위치를 상수로 두지 않고 선택된 차량의 실제 휠 좌표에서 뽑는다(차종마다 축거가
    # 다르다 — two_robot_carry_demo.py 의 기존 관례와 동일). 이동 전(local translate) 값을
    # 쓰므로 위의 PARKING_CENTER 이동과 무관하게 정확하다.
    wheel_local_z = {}
    for wn in VEHICLE_WHEELS:
        w = stage.GetPrimAtPath(f"{VEHICLE_ROOT}/{wn}")
        if not w.IsValid():
            raise RuntimeError(f"휠을 찾지 못했습니다: {VEHICLE_ROOT}/{wn}")
        wheel_local_z[wn] = float(UsdGeom.Xformable(w).GetLocalTransformation().ExtractTranslation()[2])
    rear_local_z = (wheel_local_z["RearLeftWheel"] + wheel_local_z["RearRightWheel"]) * 0.5
    front_local_z = (wheel_local_z["FrontLeftWheel"] + wheel_local_z["FrontRightWheel"]) * 0.5
    globals()["REAR_TARGET_Z"] = PARKING_CENTER[2] + rear_local_z
    globals()["FRONT_TARGET_Z"] = PARKING_CENTER[2] + front_local_z
    ROBOTS["rear"]["target_z"] = REAR_TARGET_Z
    ROBOTS["front"]["target_z"] = FRONT_TARGET_Z
    print(f"[depth-lift-dual] target={TARGET_VEHICLE} rear_z={REAR_TARGET_Z:.3f} "
          f"front_z={FRONT_TARGET_Z:.3f} wheelbase={abs(front_local_z-rear_local_z):.3f}",
          flush=True)

    vehicle_single_apis = (
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
        print("[depth-lift-dual] --keep-drivetrain: Pickup PhysX Vehicle 구동계 유지", flush=True)
    else:
        for api_schema in vehicle_single_apis:
            if target_vehicle.HasAPI(api_schema):
                target_vehicle.RemoveAPI(api_schema)
        for instance_name in (PhysxSchema.Tokens.brakes0, PhysxSchema.Tokens.brakes1):
            if target_vehicle.HasAPI(PhysxSchema.PhysxVehicleBrakesAPI, instance_name):
                target_vehicle.RemoveAPI(PhysxSchema.PhysxVehicleBrakesAPI, instance_name)
        for wheel_name in VEHICLE_WHEELS:
            wheel = stage.GetPrimAtPath(f"{VEHICLE_ROOT}/{wheel_name}")
            for api_schema in wheel_apis:
                if wheel.HasAPI(api_schema):
                    wheel.RemoveAPI(api_schema)

    if SPHERE_WHEELS:
        radius = None
        for wheel_name in VEHICLE_WHEELS:
            wheel_path = f"{VEHICLE_ROOT}/{wheel_name}"
            cylinder = stage.GetPrimAtPath(f"{wheel_path}/Collision")
            if not cylinder.IsValid():
                raise RuntimeError(f"휠 충돌체를 찾지 못했습니다: {wheel_path}/Collision")
            radius = float(UsdGeom.Cylinder(cylinder).GetRadiusAttr().Get())
            cylinder.SetActive(False)
            sphere = UsdGeom.Sphere.Define(stage, f"{wheel_path}/CollisionSphere")
            sphere.CreateRadiusAttr(radius)
            sphere.CreatePurposeAttr(UsdGeom.Tokens.guide)
            UsdPhysics.CollisionAPI.Apply(sphere.GetPrim())
        print(f"[depth-lift-dual] --sphere-wheels: 휠 충돌체 4개를 구(r={radius:.4f})로 교체", flush=True)

    vehicle_rigid = PhysxSchema.PhysxRigidBodyAPI.Apply(target_vehicle)
    vehicle_rigid.GetDisableGravityAttr().Set(False)
    vehicle_rigid.CreateEnableCCDAttr(True)
    vehicle_rigid.GetSolverPositionIterationCountAttr().Set(16)
    vehicle_rigid.GetSolverVelocityIterationCountAttr().Set(8)

    materials = UsdGeom.Scope.Define(stage, "/World/TestMaterials").GetPath()
    grip = UsdShade.Material.Define(stage, materials.AppendChild("RobotGrip"))
    grip_api = UsdPhysics.MaterialAPI.Apply(grip.GetPrim())
    grip_api.CreateStaticFrictionAttr(1.35)
    grip_api.CreateDynamicFrictionAttr(1.10)
    grip_api.CreateRestitutionAttr(0.01)
    UsdShade.MaterialBindingAPI.Apply(ground.GetPrim()).Bind(
        grip, UsdShade.Tokens.weakerThanDescendants, "physics"
    )
    for wheel_name in VEHICLE_WHEELS:
        wheel = stage.GetPrimAtPath(f"{VEHICLE_ROOT}/{wheel_name}")
        UsdShade.MaterialBindingAPI.Apply(wheel).Bind(
            grip, UsdShade.Tokens.weakerThanDescendants, "physics"
        )

    for name, cfg in ROBOTS.items():
        robot = robot_prims[name]
        robot_to_world = Gf.Matrix4d(
            0.0, 0.0, 1.0, 0.0,
            1.0, 0.0, 0.0, 0.0,
            0.0, 1.0, 0.0, 0.0,
            PARKING_CENTER[0], 0.0, cfg["start_z"], 1.0,
        )
        _replace_matrix_xform(robot, robot_to_world, UsdGeom)

        for link_name in (
            "wheel_fl", "wheel_fr", "wheel_rl", "wheel_rr",
            "bearing_roller_left_front", "bearing_roller_left_rear",
            "bearing_roller_right_front", "bearing_roller_right_rear",
        ):
            link = stage.GetPrimAtPath(f"{cfg['wrap']}/{link_name}")
            UsdShade.MaterialBindingAPI.Apply(link).Bind(
                grip, UsdShade.Tokens.weakerThanDescendants, "physics"
            )

        # 뎁스캠 자기 오클루전 수정(depth_stop_lift_test.py 에서 실측/확인된 두 지점) —
        # 로봇마다 각자 적용해야 한다.
        for occluder_path in (
            f"{cfg['wrap']}/cam_front_link/depth_cam_front/RSD455/Visual",
            f"{cfg['root']}/visuals/front_accent",
        ):
            occluder = stage.GetPrimAtPath(occluder_path)
            if occluder.IsValid():
                UsdGeom.Imageable(occluder).CreateVisibilityAttr(UsdGeom.Tokens.invisible)

        configure_hub_drives(stage, cfg["joints"])
        for name2 in ARM_TARGETS:
            joint = stage.GetPrimAtPath(f"{cfg['joints']}/{name2}")
            drive = UsdPhysics.DriveAPI.Get(joint, "angular")
            drive.GetStiffnessAttr().Set(1800.0)
            drive.GetDampingAttr().Set(140.0)
            drive.GetMaxForceAttr().Set(5000.0)
            drive.GetTargetPositionAttr().Set(0.0)

    world.SetCustomDataByKey("test", "dual-robot depth-cam stop-detect + front/rear lift")
    world.SetCustomDataByKey("vehicle", TARGET_VEHICLE)
    world.SetCustomDataByKey("parkingBay", "A7")
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
    return arr


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

    physx = omni.physx.get_physx_interface()

    def rigid_position(path):
        value = physx.get_rigidbody_transformation(path)
        return tuple(float(x) for x in value["position"])

    # Live depth-camera view window — shows the exact depth_hw array the detector
    # algorithm reads (no separate computation — what's displayed is what decides).
    # Omni UI's default font has no Hangul glyphs (renders as "???"), so this UI is
    # English-only. Normalizing per-frame min/max washed the image out to near-solid
    # white (most pixels are far background, only the ROI wheel region is near) —
    # instead clip to a fixed, calibrated range around the values that actually matter
    # here (baseline ~0.33m, trough ~0.20-0.25m) and use a blue->amber gradient instead
    # of grayscale, plus draw the ROI box so it's clear exactly what region decides.
    DEPTH_VIS_MIN = 0.15
    DEPTH_VIS_MAX = 0.45
    FAR_COLOR = np.array([25, 35, 65], dtype=np.float32)     # dark navy = far
    MID_COLOR = np.array([40, 150, 140], dtype=np.float32)   # teal = mid
    NEAR_COLOR = np.array([255, 140, 40], dtype=np.float32)  # warm amber = close

    global _DEPTH_UI_CACHE
    depth_ui = None
    if SHOW_DEPTH_VIEW:
        if _DEPTH_UI_CACHE is None:
            import omni.ui as ui

            depth_window = ui.Window("Depth Camera Live View", width=700, height=440)
            providers = {}
            status_labels = {}
            with depth_window.frame:
                with ui.VStack():
                    with ui.HStack():
                        for rname in ROBOTS:
                            with ui.VStack(width=340):
                                ui.Label(f"{rname.upper()} robot - left side depth cam",
                                          height=20, style={"font_size": 16})
                                providers[rname] = ui.ByteImageProvider()
                                ui.ImageWithProvider(providers[rname], width=320, height=240)
                                status_labels[rname] = ui.Label("-", height=60, word_wrap=True,
                                                                  style={"font_size": 14})
            _DEPTH_UI_CACHE = {
                "window": depth_window, "providers": providers, "labels": status_labels,
            }
        depth_ui = _DEPTH_UI_CACHE

    def update_depth_view(rname, depth_hw, roi_value, det, phase):
        if depth_ui is None or depth_hw is None:
            return
        small = depth_hw[::3, ::3]
        finite = np.isfinite(small)
        if not finite.any():
            return
        clipped = np.clip(small, DEPTH_VIS_MIN, DEPTH_VIS_MAX)
        norm = (clipped - DEPTH_VIS_MIN) / (DEPTH_VIS_MAX - DEPTH_VIS_MIN)  # 0=near, 1=far
        norm = np.where(finite, norm, 1.0)
        # 2-segment gradient: near->mid for norm<0.5, mid->far for norm>=0.5.
        t = np.clip(norm * 2.0, 0.0, 1.0)[..., None]
        lower = NEAR_COLOR * (1 - t) + MID_COLOR * t
        t2 = np.clip(norm * 2.0 - 1.0, 0.0, 1.0)[..., None]
        upper = MID_COLOR * (1 - t2) + FAR_COLOR * t2
        rgb = np.where((norm[..., None] < 0.5), lower, upper).astype(np.uint8)
        alpha = np.full(rgb.shape[:2] + (1,), 255, dtype=np.uint8)
        rgba = np.concatenate([rgb, alpha], axis=-1)
        # ROI 표시 — 꽉 찬 사각형 테두리 대신 카메라 뷰파인더/AF 스타일 모서리 괄호로.
        # "이 영역을 감시 중"이라는 뜻이 사각 테두리보다 더 직관적으로 읽힌다.
        h, w = rgba.shape[:2]
        col_lo, col_hi, row_lo, row_hi = DEPTH_ROI_FRAC
        r0, r1 = int(h * row_lo), min(int(h * row_hi), h - 1)
        c0, c1 = int(w * col_lo), min(int(w * col_hi), w - 1)
        mark_color = np.array([255, 225, 60, 255], dtype=np.uint8)  # 눈에 띄는 노랑
        arm = max(6, (c1 - c0) // 6)
        thickness = 3
        corners = [(r0, c0, 1, 1), (r0, c1, 1, -1), (r1, c0, -1, 1), (r1, c1, -1, -1)]
        for cr, cc, dr, dc in corners:
            r_lo, r_hi = sorted((cr, cr + dr * arm))
            c_lo, c_hi = sorted((cc, cc + dc * thickness))
            rgba[max(r_lo, 0):min(r_hi, h), max(c_lo, 0):min(c_hi, w)] = mark_color
            r_lo, r_hi = sorted((cr, cr + dr * thickness))
            c_lo, c_hi = sorted((cc, cc + dc * arm))
            rgba[max(r_lo, 0):min(r_hi, h), max(c_lo, 0):min(c_hi, w)] = mark_color
        depth_ui["providers"][rname].set_bytes_data(rgba.flatten().tolist(), [w, h])

        baseline = det.baseline
        base_txt = f"{baseline:.3f}m" if baseline is not None else "calibrating..."
        roi_txt = f"{roi_value:.3f}m" if math.isfinite(roi_value) else "N/A"
        triggered = phase in ("in_dip", "returning")
        state_txt = {
            "seeking": "watching for wheel...",
            "in_dip": "WHEEL DETECTED - waiting for exit point...",
            "returning": "WHEEL PASSED - backing up to midpoint...",
        }.get(phase, phase)
        label = depth_ui["labels"][rname]
        label.text = f"ROI min depth: {roi_txt}   baseline: {base_txt}\n{state_txt}"
        label.style = {
            "font_size": 15 if triggered else 14,
            "color": 0xFF30D0FF if triggered else 0xFFDDDDDD,
        }

    def wheel_center(name):
        prim = stage.GetPrimAtPath(f"{VEHICLE_ROOT}/{name}")
        matrix = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
        return tuple(float(x) for x in matrix.ExtractTranslation())

    robots = {}
    for name, cfg in ROBOTS.items():
        art = Articulation(cfg["root"])
        art.initialize()
        joint_positions = art.get_joint_positions()
        wheel_idx = {w: art.dof_names.index(j) for w, j in WHEEL_JOINTS.items()}
        arm_indices = {n: art.dof_names.index(n) for n in ARM_TARGETS}
        robots[name] = {
            "cfg": cfg,
            "art": art,
            "wheel_idx": wheel_idx,
            "arm_indices": arm_indices,
            "velocity_targets": np.zeros(joint_positions.shape, dtype=np.float32),
            "position_targets": np.array(joint_positions, dtype=np.float32, copy=True),
            "stopped": False,
            "stop_reason": "timeout",
            "stop_step": None,
            "depth_trace": [],
            "skip_remaining": cfg.get("skip_triggers", 0),
            "skip_events": [],
            "cooldown_remaining": 0,
            "phase": "seeking",   # seeking -> in_dip -> returning -> (stopped)
            "entry_z": None,
            "entry_step": None,
            "exit_confirm_count": 0,
            "best_z": None,       # (entry_z + exit_z) / 2 — 후진 정렬 목표
        }

    def drive(rname, vx, vy, wz):
        r = robots[rname]
        vt = r["velocity_targets"]
        for wname, omega in wheel_velocities_from_cmd_vel(vx, vy, wz).items():
            i = r["wheel_idx"][wname]
            if vt.ndim == 2:
                vt[0, i] = omega
            else:
                vt[i] = omega
        r["art"].set_joint_velocity_targets(vt)

    def set_arm_targets(rname, scale):
        r = robots[rname]
        pt = r["position_targets"]
        for aname, target in ARM_TARGETS.items():
            index = r["arm_indices"][aname]
            target_rad = math.radians(float(target * scale))
            if pt.ndim == 2:
                pt[0, index] = target_rad
            else:
                pt[index] = target_rad
        r["art"].set_joint_position_targets(pt)

    # 정착
    for name in robots:
        drive(name, 0.0, 0.0, 0.0)
        set_arm_targets(name, 0.0)
    for _ in range(120):
        app.update()

    start_positions = {name: rigid_position(r["cfg"]["root"]) for name, r in robots.items()}
    initial_wheels = {name: wheel_center(name) for name in VEHICLE_WHEELS}

    cams = {}
    rgba_hw_map = {}
    for name, r in robots.items():
        cam = Camera(prim_path=r["cfg"]["cam_left"], resolution=CAM_RES)
        cam.initialize()
        cam.add_distance_to_image_plane_to_frame()
        cams[name] = cam
    for _ in range(30):
        app.update()
    for name, cam in cams.items():
        rgba_hw_map[name] = tuple(int(v) for v in cam.get_rgba().shape[:2])

    detectors = {
        name: DepthStopDetector(
            baseline_frames=DEPTH_BASELINE_FRAMES,
            drop_margin=DROP_MARGIN,
            confirm_frames=DEPTH_CONFIRM_FRAMES,
        )
        for name in robots
    }

    for name in robots:
        drive(name, DEPTH_SPEED, 0.0, 0.0)

    for step in range(1, DEPTH_MAX_STEPS + 1):
        app.update()
        all_stopped = True
        for name, r in robots.items():
            if r["stopped"]:
                continue
            all_stopped = False
            depth_hw = _depth_to_hw(cams[name].get_depth(), rgba_hw_map[name])
            roi_value = roi_min_depth(depth_hw, roi_frac=DEPTH_ROI_FRAC) if depth_hw is not None else math.inf
            p = rigid_position(r["cfg"]["root"])
            r["depth_trace"].append({
                "step": step, "z": round(p[2], 4),
                "roi_min_depth_m": None if math.isinf(roi_value) else round(roi_value, 4),
            })
            update_depth_view(name, depth_hw, roi_value, detectors[name], r["phase"])

            # 좌우(X) 드리프트 보정 — dual 첫 실행에서 정지 x 가 8.565/8.482 로(중심 8.5
            # 대비 최대 6.5cm) 틀어져 있었고, 이게 팔 한쪽만 바퀴에 걸리고 반대쪽은 허공을
            # 잡는 원인이었다(단일 로봇 실행은 주행거리가 짧아 드리프트가 1cm 뿐이라 안 보임).
            # vy 로 계속 중심선(PARKING_CENTER[0])을 따라가도록 미세 보정한다.
            vy_correction = LATERAL_KP * (PARKING_CENTER[0] - p[0])
            vx_now = -RETURN_SPEED if r["phase"] == "returning" else DEPTH_SPEED
            drive(name, vx_now, vy_correction, 0.0)

            if r["phase"] == "returning":
                # 중간 지점(best_z)을 이미 지나쳤다 — 그 지점까지 후진해 정확히 멈춘다.
                if p[2] <= r["best_z"]:
                    drive(name, 0.0, 0.0, 0.0)
                    r["stopped"] = True
                    r["stop_reason"] = "depth_trigger"
                    r["stop_step"] = r["entry_step"]
                continue

            if r["phase"] == "in_dip":
                # 진입(entry_z)은 이미 기록해뒀다. 이제 "이탈" — 진입 때와 같은 임계값
                # (detectors[name].threshold) 위로 EXIT_CONFIRM_FRAMES 연속 다시 올라오면
                # 바퀴를 지나쳤다는 뜻이다. 대칭 가정 하에 (entry_z+exit_z)/2 가 축 중심에
                # 가깝다 — 차종별 보정값 없이 이 중간값으로 후진 정렬한다.
                threshold = detectors[name].threshold
                if math.isfinite(roi_value) and roi_value > threshold:
                    r["exit_confirm_count"] += 1
                else:
                    r["exit_confirm_count"] = 0
                if r["exit_confirm_count"] >= EXIT_CONFIRM_FRAMES:
                    exit_z = p[2]
                    r["best_z"] = (r["entry_z"] + exit_z) * 0.5
                    r["phase"] = "returning"
                    drive(name, -RETURN_SPEED, 0.0, 0.0)
                continue

            if r["cooldown_remaining"] > 0:
                # 딥(예: front 로봇이 지나친 뒷축)을 막 무시한 직후 — 아직 트로프 안이라
                # 곧장 재보정하면 baseline 이 트로프 값에 눌러앉는다(실측으로 확인된 버그:
                # 재보정 직후 baseline=0.240 로 잡혀 두번째 딥이 threshold 아래로 절대
                # 안 내려감). 트로프를 확실히 벗어날 시간(SKIP_COOLDOWN_STEPS)을 준 다음
                # 새 감지기를 만든다.
                r["cooldown_remaining"] -= 1
                if r["cooldown_remaining"] <= 0:
                    detectors[name] = DepthStopDetector(
                        baseline_frames=DEPTH_BASELINE_FRAMES,
                        drop_margin=DROP_MARGIN,
                        confirm_frames=DEPTH_CONFIRM_FRAMES,
                    )
                continue

            if detectors[name].update(step, roi_value):
                if r["skip_remaining"] > 0:
                    # 이 딥은 목표 축이 아니다 — 무시하고 쿨다운 후 재보정해 다음 딥을
                    # 기다린다. 계속 굴러가야 하므로 drive() 는 다시 부르지 않는다
                    # (이미 DEPTH_SPEED 로 구동 중).
                    r["skip_events"].append({
                        "step": step, "roi_min_depth_m": round(roi_value, 4),
                        "baseline_m": detectors[name].baseline,
                    })
                    r["skip_remaining"] -= 1
                    r["cooldown_remaining"] = SKIP_COOLDOWN_STEPS
                else:
                    # 진입 시점 기록 — 이탈을 기다리는 "in_dip" 으로.
                    r["phase"] = "in_dip"
                    r["entry_z"] = p[2]
                    r["entry_step"] = step
                    r["exit_confirm_count"] = 0
        if all_stopped:
            break

    # 아직 못 멈춘 로봇은 정지시키고 timeout 으로 기록.
    for name, r in robots.items():
        if not r["stopped"]:
            drive(name, 0.0, 0.0, 0.0)
    for _ in range(60):
        app.update()

    stop_positions = {name: rigid_position(r["cfg"]["root"]) for name, r in robots.items()}
    before_lift = {name: wheel_center(name) for name in VEHICLE_WHEELS}

    for ramp_step in range(1, 181):
        for name in robots:
            set_arm_targets(name, ramp_step / 180.0)
        app.update()
    for _ in range(360):
        app.update()

    after_lift = {name: wheel_center(name) for name in VEHICLE_WHEELS}

    # 다 들어올린 뒤 두 로봇을 같은 속도로 같이 조금 전진시켜 차량을 운반한다(사용자 요청).
    # 팔은 이미 position target 으로 고정돼 있어 계속 재설정할 필요 없다 — 좌우 드리프트
    # 보정만 계속 적용한다.
    carry_steps = int(round(CARRY_DISTANCE_M / CARRY_SPEED * 60.0))
    for _ in range(carry_steps):
        app.update()
        for name, r in robots.items():
            p = rigid_position(r["cfg"]["root"])
            vy_correction = LATERAL_KP * (PARKING_CENTER[0] - p[0])
            drive(name, CARRY_SPEED, vy_correction, 0.0)
    for name in robots:
        drive(name, 0.0, 0.0, 0.0)
    for _ in range(60):
        app.update()

    after_carry = {name: wheel_center(name) for name in VEHICLE_WHEELS}
    final_positions = {name: rigid_position(r["cfg"]["root"]) for name, r in robots.items()}

    per_robot_report = {}
    overall_passed = True
    for name, r in robots.items():
        cfg = r["cfg"]
        if detectors[name] is None:
            # 쿨다운 중 DEPTH_MAX_STEPS 에 도달한 극단적 경우 대비 안전장치.
            detectors[name] = DepthStopDetector(
                baseline_frames=DEPTH_BASELINE_FRAMES,
                drop_margin=DROP_MARGIN,
                confirm_frames=DEPTH_CONFIRM_FRAMES,
            )
        live_positions = r["art"].get_joint_positions()
        if getattr(live_positions, "ndim", 1) == 2:
            live_positions = live_positions[0]
        arm_angles = {
            an: math.degrees(float(live_positions[r["art"].dof_names.index(an)]))
            for an in ARM_TARGETS
        }
        w0, w1 = cfg["vehicle_wheels"]
        lifts = [after_lift[w0][1] - before_lift[w0][1], after_lift[w1][1] - before_lift[w1][1]]
        mean_lift = sum(lifts) * 0.5
        other_wheels = [w for w in VEHICLE_WHEELS if w not in cfg["vehicle_wheels"]]
        other_lift_mean = sum(after_lift[w][1] - before_lift[w][1] for w in other_wheels) / len(other_wheels)
        arrival_error = abs(stop_positions[name][2] - cfg["target_z"])
        arms_reached = all(abs(arm_angles[an] - t) < 3.0 for an, t in ARM_TARGETS.items())
        # 단일 로봇 판정 기준("다른 축은 안 들려야 한다")은 dual 시나리오엔 안 맞는다 —
        # 여기선 두 로봇이 동시에 각자 축을 드니 other_lift_mean 도 정상적으로 커진다.
        # 대신 이 로봇 자신의 좌우 바퀴가 "둘 다 확실히 들렸고 서로 대칭인지"를 본다
        # (원래 겪었던 버그 — 한쪽만 들리는 경우를 여기서 잡는다).
        lift_pass = min(lifts) >= 0.025 and (max(lifts) - min(lifts)) < 0.03
        depth_stop_ok = r["stop_reason"] == "depth_trigger"
        passed = depth_stop_ok and arrival_error < 0.15 and arms_reached and lift_pass
        overall_passed = overall_passed and passed
        per_robot_report[name] = {
            "vehicle_wheels": list(cfg["vehicle_wheels"]),
            "start_xyz_m": start_positions[name],
            "stop_xyz_m": stop_positions[name],
            "final_xyz_m": final_positions[name],
            "target_z_m": cfg["target_z"],
            "arrival_error_m": arrival_error,
            "wheel_lift_m": lifts,
            "mean_lift_m": mean_lift,
            "arm_angles_deg": arm_angles,
            "depth_baseline_m": detectors[name].baseline,
            "depth_threshold_m": detectors[name].threshold,
            "depth_stop_value_m": detectors[name].trigger_value,
            "stop_step": r["stop_step"],
            "stop_reason": r["stop_reason"],
            "skip_events": r["skip_events"],
            "depth_trace": r["depth_trace"],
            "checks": {
                "depth_stop_triggered": depth_stop_ok,
                "robot_arrived": arrival_error < 0.15,
                "arms_reached_targets": arms_reached,
                "wheels_lifted": lift_pass,
            },
            "passed": passed,
        }

    report = {
        "passed": overall_passed,
        "assets": {
            "parking": str(PARKING_SOURCE_USD),
            "vehicle": str(VEHICLES_USD),
            "robot": str(ROBOT_USD),
            "test_stage": str(OUTPUT_USD),
        },
        "vehicle": TARGET_VEHICLE,
        "keep_drivetrain": KEEP_DRIVETRAIN,
        "sphere_wheels": SPHERE_WHEELS,
        "parking_bay": "A7",
        "initial_wheel_centers_m": initial_wheels,
        "before_lift_wheel_centers_m": before_lift,
        "after_lift_wheel_centers_m": after_lift,
        "after_carry_wheel_centers_m": after_carry,
        "carry_distance_m": CARRY_DISTANCE_M,
        "final_robot_xyz_m": final_positions,
        "robots": per_robot_report,
    }
    REPORT_JSON.write_text(json.dumps(report, indent=2, ensure_ascii=False), encoding="utf-8")

    print(f"TEST_PASSED={overall_passed}", flush=True)
    for name, r in per_robot_report.items():
        print(f"[{name}] STOP_REASON={r['stop_reason']} ARRIVAL_ERROR_M={r['arrival_error_m']:.6f} "
              f"MEAN_LIFT_M={r['mean_lift_m']:.6f} PASSED={r['passed']}", flush=True)
    for wn in VEHICLE_WHEELS:
        moved = after_carry[wn][2] - after_lift[wn][2]
        print(f"CARRY_MOVED_Z {wn}={moved:.4f}", flush=True)
    print(f"REPORT={REPORT_JSON}", flush=True)

    timeline.stop()
    app.update()
    return report


def run_gui_replay_loop(app):
    """GUI 에서 Play 를 누를 때마다 처음부터 다시 실행한다.

    run_test() 는 매번 저장된 OUTPUT_USD 를 새로 열기 때문에(build_test_stage() 는 한 번만
    실행) 호출할 때마다 로봇/차량이 저장 시점 상태로 완전히 리셋된다 — 첫 실행은 자동으로,
    이후는 Play 버튼 누름을 재실행 트리거로 쓴다.
    """
    import omni.timeline

    timeline = omni.timeline.get_timeline_interface()
    run_number = 0

    def do_run():
        nonlocal run_number
        run_number += 1
        print(f"GUI_REPLAY_START={run_number}", flush=True)
        try:
            run_test(app)
        except Exception as exc:
            write_exception_report(exc)
            print(f"TEST_EXCEPTION={type(exc).__name__}: {exc}", flush=True)
        print("GUI_REPLAY_READY: Press Play to run again.", flush=True)

    do_run()
    while app.is_running():
        app.update()
        if not timeline.is_playing():
            continue
        timeline.stop()
        app.update()
        do_run()


def main():
    _restart_with_isaac_python()
    from isaacsim import SimulationApp

    gui = "--gui" in sys.argv[1:]
    app = SimulationApp({"headless": not gui})
    try:
        build_test_stage()
        if gui:
            run_gui_replay_loop(app)
        else:
            report = run_test(app)
            if not report["passed"]:
                raise RuntimeError(f"dual depth-stop lift test failed; see {REPORT_JSON}")
    except Exception as exc:
        write_exception_report(exc)
        print(f"TEST_EXCEPTION={type(exc).__name__}: {exc}", flush=True)
        raise
    finally:
        app.close()


if __name__ == "__main__":
    main()
