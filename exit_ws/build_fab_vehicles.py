#!/usr/bin/env python3
"""fab.fbx를 차량별로 조립하고 색칠하는 Isaac Sim 5.1 독립 실행 스크립트.

실행:
    python3 build_fab_vehicles.py
또는:
    ./build_fab_vehicles.py

일반 Python으로 실행하면 설치된 Isaac Sim의 python.sh로 자동 전환한다.
기본 실행은 결과 USD를 Isaac Sim 창에 표시하고, 창을 닫으면 종료한다.
창 없이 변환만 하려면 ``--headless`` 옵션을 사용한다.
최종 결과는 이 스크립트와 같은 폴더의 fab_vehicles.usd 이다.
"""

import asyncio
import os
import re
import sys
from pathlib import Path


# 프로젝트 폴더를 통째로 옮겨도 source/와 출력 경로가 자동으로 따라간다.
WORK_DIR = Path(__file__).resolve().parent
INPUT_FBX = WORK_DIR / "source" / "fab.fbx"
RAW_USD = WORK_DIR / "fab_converted_raw.usd"
OUTPUT_USD = WORK_DIR / "fab_vehicles.usd"

ISAAC_ROOT = Path(
    "/home/rokey/dev_ws/isaac_sim/isaacsim/_build/linux-x86_64/release"
)
ISAAC_PYTHON = ISAAC_ROOT / "python.sh"

VEHICLE_TYPES = (
    "Compact",
    "Coupe",
    "Hatchback",
    "Minivan",
    "Offroad",
    "Pickup",
    "Sedan",
    "Sport",
    "SUV",
    "Wagon",
)
BODY_COLORS = {
    "white": (0.92, 0.92, 0.92),
    "black": (0.025, 0.030, 0.038),
    "Sport": (0.72, 0.015, 0.01),
    "Offroad": (0.12, 0.16, 0.055),
}
BLACK_BODY_TYPES = {"Coupe", "Minivan", "Sedan", "Wagon"}
BLACK = (0.012, 0.014, 0.018)
GLASS_BLACK = (0.018, 0.026, 0.035)
LIGHT_WHITE = (0.85, 0.88, 0.82)


def _restart_with_isaac_python():
    """일반 Python 실행을 Isaac Sim Python 실행으로 자동 교체한다."""
    if os.environ.get("CARB_APP_PATH"):
        return
    if not ISAAC_PYTHON.is_file():
        raise FileNotFoundError(f"Isaac Sim python.sh를 찾을 수 없습니다: {ISAAC_PYTHON}")
    print(f"[fab] Isaac Sim Python으로 전환: {ISAAC_PYTHON}", flush=True)
    os.execv(
        str(ISAAC_PYTHON),
        [str(ISAAC_PYTHON), str(Path(__file__).resolve()), *sys.argv[1:]],
    )


def _normal(text):
    return re.sub(r"[^a-z0-9]", "", text.lower())


def _set_converter_option(context, names, value):
    for name in names:
        if hasattr(context, name):
            setattr(context, name, value)
            return


async def _convert_fbx(omni, carb):
    if not INPUT_FBX.is_file():
        raise FileNotFoundError(f"입력 FBX를 찾을 수 없습니다: {INPUT_FBX}")

    import omni.kit.asset_converter

    context = omni.kit.asset_converter.AssetConverterContext()
    _set_converter_option(context, ("ignore_materials", "ignore_material"), False)
    _set_converter_option(context, ("ignore_animations", "ignore_animation"), True)
    _set_converter_option(context, ("ignore_camera", "ignore_cameras"), True)
    _set_converter_option(context, ("ignore_light", "ignore_lights"), True)
    _set_converter_option(context, ("single_mesh",), False)
    _set_converter_option(context, ("smooth_normals",), True)
    _set_converter_option(
        context, ("export_preview_surface", "preview_surface"), True
    )
    _set_converter_option(context, ("use_meter_as_world_unit",), True)

    last_percent = -10

    def progress_callback(current, total):
        nonlocal last_percent
        percent = int(100 * current / total) if total else 0
        if percent >= last_percent + 10:
            last_percent = percent
            print(f"[fab] FBX 변환 {percent}%", flush=True)

    # Asset Converter는 기존 출력이 있을 때 백엔드별 동작이 다르므로 생성물만 갱신한다.
    if RAW_USD.exists():
        RAW_USD.unlink()

    converter = omni.kit.asset_converter.get_instance()
    task = converter.create_converter_task(
        str(INPUT_FBX), str(RAW_USD), progress_callback, context
    )
    success = await task.wait_until_finished()
    if not success:
        get_error = getattr(task, "get_error_message", None)
        if get_error is None:
            get_error = getattr(task, "get_detailed_error", None)
        detail = get_error() if get_error else "상세 오류 없음"
        carb.log_error(f"[fab] FBX 변환 실패: {task.get_status()} / {detail}")
        raise RuntimeError(f"FBX 변환 실패: {detail}")
    print(f"[fab] 중간 USD 생성: {RAW_USD}", flush=True)


def _vehicle_type_from_body_name(name):
    normalized = _normal(name)
    for vehicle_type in VEHICLE_TYPES:
        if _normal(vehicle_type) in normalized and "body" in normalized:
            return vehicle_type
    return None


def _world_center(prim, bbox_cache):
    world_range = bbox_cache.ComputeWorldBound(prim).ComputeAlignedRange()
    if world_range.IsEmpty():
        raise RuntimeError(f"경계 상자를 계산할 수 없습니다: {prim.GetPath()}")
    return world_range.GetMidpoint()


def _distance_squared(a, b):
    return sum((float(a[i]) - float(b[i])) ** 2 for i in range(3))


def _set_matrix_xform(prim, matrix, UsdGeom):
    """Prim의 변환을 단일 matrix op로 바꾼다."""
    xformable = UsdGeom.Xformable(prim)
    xformable.ClearXformOpOrder()
    xformable.MakeMatrixXform().Set(matrix)


def _bbox_size(prim, bbox_cache):
    size = bbox_cache.ComputeWorldBound(prim).ComputeAlignedRange().GetSize()
    return tuple(abs(float(size[i])) for i in range(3))


def _make_preview_material(
    stage, UsdShade, Sdf, Gf, path, color, roughness,
    metallic=0.0, opacity=1.0, emissive=None,
):
    material = UsdShade.Material.Define(stage, path)
    shader = UsdShade.Shader.Define(stage, path.AppendChild("Shader"))
    shader.CreateIdAttr("UsdPreviewSurface")
    shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*color))
    shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(float(roughness))
    shader.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(float(metallic))
    shader.CreateInput("opacity", Sdf.ValueTypeNames.Float).Set(float(opacity))
    shader.CreateInput("ior", Sdf.ValueTypeNames.Float).Set(1.5)
    if emissive is not None:
        shader.CreateInput("emissiveColor", Sdf.ValueTypeNames.Color3f).Set(
            Gf.Vec3f(*emissive)
        )
    shader.CreateOutput("surface", Sdf.ValueTypeNames.Token)
    material.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")
    return material


def _create_materials(stage, UsdGeom, UsdShade, Sdf, Gf):
    looks_path = stage.GetDefaultPrim().GetPath().AppendChild("Looks")
    UsdGeom.Scope.Define(stage, looks_path)
    custom_path = looks_path.AppendChild("FabColors")
    UsdGeom.Scope.Define(stage, custom_path)

    materials = {
        "BodyWhite": _make_preview_material(
            stage, UsdShade, Sdf, Gf, custom_path.AppendChild("BodyWhite"), BODY_COLORS["white"], 0.18, 0.22
        ),
        "BodyBlack": _make_preview_material(
            stage, UsdShade, Sdf, Gf, custom_path.AppendChild("BodyBlack"), BODY_COLORS["black"], 0.20, 0.20
        ),
        "BodyRed": _make_preview_material(
            stage, UsdShade, Sdf, Gf, custom_path.AppendChild("BodyRed"), BODY_COLORS["Sport"], 0.16, 0.28
        ),
        "BodyArmy": _make_preview_material(
            stage, UsdShade, Sdf, Gf, custom_path.AppendChild("BodyArmy"), BODY_COLORS["Offroad"], 0.38, 0.08
        ),
        "Black": _make_preview_material(
            stage, UsdShade, Sdf, Gf, custom_path.AppendChild("Black"), (0.025, 0.028, 0.032), 0.72, 0.0
        ),
        "GlassBlack": _make_preview_material(
            stage, UsdShade, Sdf, Gf, custom_path.AppendChild("GlassBlack"), (0.018, 0.035, 0.055), 0.08, 0.05, 0.38
        ),
        "LightWhite": _make_preview_material(
            stage, UsdShade, Sdf, Gf, custom_path.AppendChild("LightWhite"), LIGHT_WHITE, 0.12, 0.05, 1.0, (1.0, 0.92, 0.70)
        ),
    }
    return materials


def _create_showroom_lighting(stage, UsdGeom, Gf):
    """텍스처가 없는 저폴리 모델도 형태와 도장 하이라이트가 읽히는 조명."""
    from pxr import UsdLux

    root = stage.GetDefaultPrim().GetPath().AppendChild("FabLighting")
    UsdGeom.Xform.Define(stage, root)
    dome = UsdLux.DomeLight.Define(stage, root.AppendChild("SoftDome"))
    dome.CreateIntensityAttr(420.0)
    dome.CreateColorAttr(Gf.Vec3f(0.78, 0.86, 1.0))

    key = UsdLux.SphereLight.Define(stage, root.AppendChild("KeyLight"))
    key.CreateRadiusAttr(2.5)
    key.CreateIntensityAttr(32000.0)
    key.CreateColorAttr(Gf.Vec3f(1.0, 0.82, 0.68))
    key.AddTranslateOp().Set(Gf.Vec3f(-6.0, 10.0, 4.0))

    fill = UsdLux.SphereLight.Define(stage, root.AppendChild("FillLight"))
    fill.CreateRadiusAttr(3.5)
    fill.CreateIntensityAttr(18000.0)
    fill.CreateColorAttr(Gf.Vec3f(0.55, 0.72, 1.0))
    fill.AddTranslateOp().Set(Gf.Vec3f(7.0, 6.0, -5.0))


def _bind(prim, material, UsdShade):
    UsdShade.MaterialBindingAPI.Apply(prim).Bind(material)


def _color_vehicle(vehicle_prim, vehicle_type, materials, Usd, UsdGeom, UsdShade):
    if vehicle_type == "Sport":
        body_material = materials["BodyRed"]
    elif vehicle_type == "Offroad":
        body_material = materials["BodyArmy"]
    elif vehicle_type in BLACK_BODY_TYPES:
        body_material = materials["BodyBlack"]
    else:
        body_material = materials["BodyWhite"]

    colored = {"body": 0, "glass": 0, "optics": 0, "wheel": 0}
    for prim in Usd.PrimRange(vehicle_prim):
        name = _normal(prim.GetName())
        is_wheel_part = "wheel" in _normal(str(prim.GetPath()))

        # SUV 바퀴처럼 Mesh 아래에 Body/Wheel subset이 추가로 있는 경우도 모두
        # 검정으로 덮어써야 하므로 차량 차체 subset 판정보다 먼저 처리한다.
        if prim.IsA(UsdGeom.Subset) and is_wheel_part:
            _bind(prim, materials["Black"], UsdShade)
        elif prim.IsA(UsdGeom.Subset):
            if "glass" in name:
                _bind(prim, materials["GlassBlack"], UsdShade)
                colored["glass"] += 1
            elif "optic" in name or "light" in name:
                _bind(prim, materials["LightWhite"], UsdShade)
                colored["optics"] += 1
            elif "body" in name:
                _bind(prim, body_material, UsdShade)
                colored["body"] += 1
        elif prim.IsA(UsdGeom.Gprim) and is_wheel_part:
            _bind(prim, materials["Black"], UsdShade)
            colored["wheel"] += 1

    if colored["body"] < 1 or colored["wheel"] != 4:
        raise RuntimeError(
            f"{vehicle_type} 색상 적용 검증 실패: "
            f"Body subset={colored['body']}, Wheel mesh={colored['wheel']}"
        )
    return colored


VEHICLE_DYNAMICS = {
    # mass, drive, front weight bias, engine torque/rpm, steering, suspension
    # natural frequency/damping, travel and dry-road grip are deliberately
    # different so every body type has a recognizable dynamic character.
    "Compact":   dict(mass=1100.0, drive="FWD", front_bias=0.61, torque=210.0, rpm=620.0, steer=36.0, hz=1.55, damping=0.72, travel=0.30, grip=1.00, brake=3200.0),
    "Coupe":     dict(mass=1450.0, drive="RWD", front_bias=0.52, torque=390.0, rpm=680.0, steer=32.0, hz=1.85, damping=0.76, travel=0.25, grip=1.08, brake=4300.0),
    "Hatchback": dict(mass=1250.0, drive="FWD", front_bias=0.60, torque=240.0, rpm=630.0, steer=35.0, hz=1.65, damping=0.73, travel=0.29, grip=1.02, brake=3500.0),
    "Minivan":   dict(mass=1900.0, drive="FWD", front_bias=0.60, torque=330.0, rpm=540.0, steer=34.0, hz=1.30, damping=0.78, travel=0.36, grip=0.94, brake=5000.0),
    "Offroad":   dict(mass=1850.0, drive="AWD", front_bias=0.53, torque=430.0, rpm=520.0, steer=38.0, hz=1.20, damping=0.72, travel=0.55, grip=0.92, brake=4800.0),
    "Pickup":    dict(mass=2150.0, drive="AWD", front_bias=0.56, torque=520.0, rpm=500.0, steer=36.0, hz=1.25, damping=0.76, travel=0.48, grip=0.94, brake=5600.0),
    "Sedan":     dict(mass=1600.0, drive="RWD", front_bias=0.53, torque=360.0, rpm=620.0, steer=33.0, hz=1.60, damping=0.75, travel=0.29, grip=1.02, brake=4400.0),
    "Sport":     dict(mass=1380.0, drive="RWD", front_bias=0.46, torque=610.0, rpm=760.0, steer=30.0, hz=2.15, damping=0.82, travel=0.20, grip=1.16, brake=6100.0),
    "SUV":       dict(mass=2050.0, drive="AWD", front_bias=0.55, torque=470.0, rpm=560.0, steer=34.0, hz=1.40, damping=0.79, travel=0.38, grip=0.98, brake=5400.0),
    "Wagon":     dict(mass=1550.0, drive="AWD", front_bias=0.55, torque=350.0, rpm=600.0, steer=33.0, hz=1.55, damping=0.76, travel=0.31, grip=1.00, brake=4300.0),
}


def _add_vehicle_physics(stage, wheel_report, Usd, UsdGeom, UsdShade, Sdf, Gf):
    """열 대의 조립 차량에 PhysX Vehicle 구동계와 충돌계를 구성한다."""
    import math

    from pxr import PhysxSchema, UsdPhysics
    from omni.physx.scripts.physicsUtils import (
        add_collision_to_collision_group, add_physics_material_to_prim,
    )
    from omni.physx.scripts.utils import set_custom_metadata
    from omni.physx.bindings._physx import VEHICLE_AUTOMATIC_TRANSMISSION_GEAR_VALUE

    world_path = stage.GetDefaultPrim().GetPath()
    meters_per_unit = UsdGeom.GetStageMetersPerUnit(stage)
    length_scale = 1.0 / meters_per_unit
    torque_scale = length_scale * length_scale
    force_scale = length_scale

    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.y)
    UsdPhysics.SetStageKilogramsPerUnit(stage, 1.0)

    scene = UsdPhysics.Scene.Define(stage, world_path.AppendChild("PhysicsScene"))
    scene.CreateGravityDirectionAttr(Gf.Vec3f(0, -1, 0))
    scene.CreateGravityMagnitudeAttr(9.81 * length_scale)
    context = PhysxSchema.PhysxVehicleContextAPI.Apply(scene.GetPrim())
    context.CreateUpdateModeAttr(PhysxSchema.Tokens.velocityChange)
    context.CreateVerticalAxisAttr(PhysxSchema.Tokens.posY)
    context.CreateLongitudinalAxisAttr(PhysxSchema.Tokens.posZ)

    physics_path = world_path.AppendChild("VehiclePhysics")
    UsdGeom.Scope.Define(stage, physics_path)
    tarmac_path = physics_path.AppendChild("TarmacMaterial")
    tarmac = UsdShade.Material.Define(stage, tarmac_path)
    mat_api = UsdPhysics.MaterialAPI.Apply(tarmac.GetPrim())
    mat_api.CreateStaticFrictionAttr(1.05)
    mat_api.CreateDynamicFrictionAttr(0.90)
    mat_api.CreateRestitutionAttr(0.02)
    PhysxSchema.PhysxMaterialAPI.Apply(tarmac.GetPrim())

    tire_table_path = physics_path.AppendChild("RoadTireFriction")
    tire_table = PhysxSchema.PhysxVehicleTireFrictionTable.Define(
        stage, tire_table_path
    )
    tire_table.CreateGroundMaterialsRel().AddTarget(tarmac_path)
    tire_table.CreateFrictionValuesAttr([1.05])

    group_paths = {
        name: physics_path.AppendChild(name)
        for name in ("ChassisGroup", "WheelGroup", "GroundQueryGroup", "GroundGroup")
    }
    groups = {
        name: UsdPhysics.CollisionGroup.Define(stage, path)
        for name, path in group_paths.items()
    }
    groups["ChassisGroup"].CreateFilteredGroupsRel().AddTarget(group_paths["GroundQueryGroup"])
    groups["WheelGroup"].CreateFilteredGroupsRel().AddTarget(group_paths["GroundQueryGroup"])
    groups["WheelGroup"].CreateFilteredGroupsRel().AddTarget(group_paths["GroundGroup"])
    groups["GroundQueryGroup"].CreateFilteredGroupsRel().AddTarget(group_paths["ChassisGroup"])
    groups["GroundQueryGroup"].CreateFilteredGroupsRel().AddTarget(group_paths["WheelGroup"])
    groups["GroundGroup"].CreateFilteredGroupsRel().AddTarget(group_paths["GroundGroup"])
    groups["GroundGroup"].CreateFilteredGroupsRel().AddTarget(group_paths["WheelGroup"])

    ground = UsdGeom.Plane.Define(stage, world_path.AppendChild("DriveGround"))
    ground.CreateAxisAttr(UsdGeom.Tokens.y)
    ground.CreatePurposeAttr(UsdGeom.Tokens.guide)
    UsdPhysics.CollisionAPI.Apply(ground.GetPrim())
    add_collision_to_collision_group(stage, ground.GetPath(), group_paths["GroundGroup"])
    add_physics_material_to_prim(stage, ground.GetPrim(), tarmac_path)

    for vehicle_type in VEHICLE_TYPES:
        vehicle_path = world_path.AppendChild("Vehicles").AppendChild(vehicle_type)
        vehicle = stage.GetPrimAtPath(vehicle_path)
        wheels = wheel_report[vehicle_type]
        dynamics = VEHICLE_DYNAMICS[vehicle_type]
        mass = dynamics["mass"]
        positions = [item["position"] for item in wheels]
        if "--debug-layout" in sys.argv[1:]:
            compact_positions = [
                tuple(round(float(p[i]), 3) for i in range(3)) for p in positions
            ]
            print(f"[fab-debug] {vehicle_type}: {compact_positions}", flush=True)
        radii = [item["radius"] for item in wheels]
        avg_radius = sum(radii) / 4.0
        min_x, max_x = min(p[0] for p in positions), max(p[0] for p in positions)
        min_z, max_z = min(p[2] for p in positions), max(p[2] for p in positions)
        wheel_y = sum(p[1] for p in positions) / 4.0
        track = max_x - min_x
        wheelbase = max_z - min_z
        # 네 접지점의 산술 평균은 항상 그 볼록껍질 안에 있다. FBX 원점이나
        # 차축이 완전히 대칭이라는 가정을 하지 않아 PhysX sprung-mass 계산이
        # 모든 차종에서 성립한다.
        center_x = sum(float(p[0]) for p in positions) / 4.0
        axle_center_z = sum(float(p[2]) for p in positions) / 4.0
        # Positive local Z is the front axle. Moving the CoM forward produces
        # the requested static front axle weight bias.
        center_z = axle_center_z + (dynamics["front_bias"] - 0.5) * wheelbase
        chassis_dims = Gf.Vec3f(
            max(track * 0.82, avg_radius * 2.5),
            max(avg_radius * 1.45, 0.55 * length_scale),
            max(wheelbase * 0.88, avg_radius * 4.0),
        )
        chassis_center = Gf.Vec3f(
            center_x, wheel_y + avg_radius * 0.72, axle_center_z
        )

        UsdPhysics.RigidBodyAPI.Apply(vehicle)
        mass_api = UsdPhysics.MassAPI.Apply(vehicle)
        mass_api.CreateMassAttr(mass)
        # 낮은 무게중심은 전복을 줄이되 지면 아래로 내려가지 않도록 잡는다.
        com_height_factor = 0.38 if vehicle_type == "Sport" else 0.52 if vehicle_type in {"Offroad", "Pickup", "SUV"} else 0.45
        mass_api.CreateCenterOfMassAttr(Gf.Vec3f(
            center_x, wheel_y + avg_radius * com_height_factor, center_z
        ))
        dx, dy, dz = (float(chassis_dims[i]) for i in range(3))
        mass_api.CreateDiagonalInertiaAttr(Gf.Vec3f(
            mass * (dy * dy + dz * dz) / 12.0,
            mass * (dx * dx + dz * dz) / 12.0,
            mass * (dx * dx + dy * dy) / 12.0,
        ))
        mass_api.CreatePrincipalAxesAttr(Gf.Quatf(1, 0, 0, 0))
        rigid_api = PhysxSchema.PhysxRigidBodyAPI.Apply(vehicle)
        rigid_api.CreateDisableGravityAttr(True)
        rigid_api.CreateLinearDampingAttr(0.04)
        rigid_api.CreateAngularDampingAttr(0.12)
        rigid_api.CreateSleepThresholdAttr(0.001 * length_scale * length_scale)
        rigid_api.CreateStabilizationThresholdAttr(0.0005 * length_scale * length_scale)
        rigid_api.CreateSolverPositionIterationCountAttr(8)
        rigid_api.CreateSolverVelocityIterationCountAttr(4)

        vehicle_api = PhysxSchema.PhysxVehicleAPI.Apply(vehicle)
        vehicle_api.CreateVehicleEnabledAttr(True)
        vehicle_api.CreateSubStepThresholdLongitudinalSpeedAttr(5.0 * length_scale)
        vehicle_api.CreateLowForwardSpeedSubStepCountAttr(3)
        vehicle_api.CreateHighForwardSpeedSubStepCountAttr(1)
        vehicle_api.CreateMinPassiveLongitudinalSlipDenominatorAttr(4.0 * length_scale)
        vehicle_api.CreateMinActiveLongitudinalSlipDenominatorAttr(0.1 * length_scale)
        vehicle_api.CreateMinLateralSlipDenominatorAttr(1.0 * length_scale)
        vehicle_api.CreateLongitudinalStickyTireDampingAttr(200.0)
        vehicle_api.CreateLateralStickyTireDampingAttr(20.0)
        set_custom_metadata(vehicle, PhysxSchema.Tokens.referenceFrameIsCenterOfMass, False)

        brake = PhysxSchema.PhysxVehicleBrakesAPI.Apply(
            vehicle, PhysxSchema.Tokens.brakes0
        )
        brake.CreateMaxBrakeTorqueAttr(dynamics["brake"] * torque_scale)
        handbrake = PhysxSchema.PhysxVehicleBrakesAPI.Apply(
            vehicle, PhysxSchema.Tokens.brakes1
        )
        handbrake.CreateWheelsAttr([2, 3])
        handbrake.CreateMaxBrakeTorqueAttr(dynamics["brake"] * 0.82 * torque_scale)
        steering = PhysxSchema.PhysxVehicleAckermannSteeringAPI.Apply(vehicle)
        steering.CreateWheel0Attr(1)  # front right
        steering.CreateWheel1Attr(0)  # front left
        steering.CreateMaxSteerAngleAttr(math.radians(dynamics["steer"]))
        steering.CreateWheelBaseAttr(wheelbase)
        steering.CreateTrackWidthAttr(track)
        steering.CreateStrengthAttr(1.0)
        differential = PhysxSchema.PhysxVehicleMultiWheelDifferentialAPI.Apply(vehicle)
        if dynamics["drive"] == "FWD":
            driven_wheels, torque_ratios = [0, 1], [0.5, 0.5]
        elif dynamics["drive"] == "RWD":
            driven_wheels, torque_ratios = [2, 3], [0.5, 0.5]
        else:
            driven_wheels, torque_ratios = [0, 1, 2, 3], [0.25] * 4
        differential.CreateWheelsAttr(driven_wheels)
        differential.CreateTorqueRatiosAttr(torque_ratios)
        differential.CreateAverageWheelSpeedRatiosAttr(torque_ratios)

        PhysxSchema.PhysxVehicleDriveStandardAPI.Apply(vehicle)
        engine = PhysxSchema.PhysxVehicleEngineAPI.Apply(vehicle)
        engine.CreateMoiAttr((0.75 if vehicle_type == "Sport" else 1.0) * torque_scale)
        engine.CreatePeakTorqueAttr(dynamics["torque"] * torque_scale)
        engine.CreateMaxRotationSpeedAttr(dynamics["rpm"])
        engine.CreateTorqueCurveAttr([
            Gf.Vec2f(0.0, 0.75), Gf.Vec2f(0.35, 1.0), Gf.Vec2f(1.0, 0.72)
        ])
        engine.CreateDampingRateFullThrottleAttr(0.15 * torque_scale)
        engine.CreateDampingRateZeroThrottleClutchEngagedAttr(2.0 * torque_scale)
        engine.CreateDampingRateZeroThrottleClutchDisengagedAttr(0.35 * torque_scale)
        gears = PhysxSchema.PhysxVehicleGearsAPI.Apply(vehicle)
        gears.CreateRatiosAttr([-3.5, 3.6, 2.2, 1.55, 1.18, 0.95])
        gears.CreateRatioScaleAttr(4.35 if vehicle_type in {"Offroad", "Pickup"} else 3.55 if vehicle_type == "Sport" else 3.9)
        gears.CreateSwitchTimeAttr(0.22 if vehicle_type == "Sport" else 0.42 if vehicle_type in {"Offroad", "Pickup"} else 0.34)
        autobox = PhysxSchema.PhysxVehicleAutoGearBoxAPI.Apply(vehicle)
        autobox.CreateUpRatiosAttr([0.72, 0.72, 0.72, 0.72])
        autobox.CreateDownRatiosAttr([0.48, 0.48, 0.48, 0.48])
        autobox.CreateLatencyAttr(1.0)
        PhysxSchema.PhysxVehicleClutchAPI.Apply(vehicle).CreateStrengthAttr(
            12.0 * torque_scale
        )
        controller = PhysxSchema.PhysxVehicleControllerAPI.Apply(vehicle)
        controller.CreateAcceleratorAttr(0.0)
        controller.CreateBrake0Attr(0.0)
        controller.CreateBrake1Attr(0.0)
        controller.CreateSteerAttr(0.0)
        controller.CreateTargetGearAttr(VEHICLE_AUTOMATIC_TRANSMISSION_GEAR_VALUE)

        front_sprung_mass = mass * dynamics["front_bias"] / 2.0
        rear_sprung_mass = mass * (1.0 - dynamics["front_bias"]) / 2.0
        natural_omega = 2.0 * math.pi * dynamics["hz"]
        travel = max(avg_radius * dynamics["travel"], 0.09 * length_scale)
        for item in wheels:
            wheel_prim = stage.GetPrimAtPath(item["path"])
            attachment = PhysxSchema.PhysxVehicleWheelAttachmentAPI.Apply(wheel_prim)
            attachment.CreateCollisionGroupRel().AddTarget(group_paths["GroundQueryGroup"])
            attachment.CreateSuspensionTravelDirectionAttr(Gf.Vec3f(0, -1, 0))
            frame_pos = Gf.Vec3f(item["position"])
            frame_pos[1] += travel * 0.5
            attachment.CreateSuspensionFramePositionAttr(frame_pos)
            attachment.CreateIndexAttr(item["index"])

            wheel_api = PhysxSchema.PhysxVehicleWheelAPI.Apply(wheel_prim)
            wheel_api.CreateRadiusAttr(item["radius"])
            wheel_api.CreateWidthAttr(item["width"])
            wheel_mass = max(18.0, mass * 0.014)
            wheel_api.CreateMassAttr(wheel_mass)
            wheel_api.CreateMoiAttr(0.5 * wheel_mass * item["radius"] ** 2)
            wheel_api.CreateDampingRateAttr(0.25 * torque_scale)
            is_front = item["index"] < 2
            sprung_mass = front_sprung_mass if is_front else rear_sprung_mass
            spring_strength = sprung_mass * natural_omega * natural_omega
            spring_damping = (
                2.0 * dynamics["damping"]
                * math.sqrt(spring_strength * sprung_mass)
            )
            tire_load = sprung_mass * 9.81 * length_scale
            tire = PhysxSchema.PhysxVehicleTireAPI.Apply(wheel_prim)
            tire.CreateLateralStiffnessGraphAttr(Gf.Vec2f(2.0, 19.0 * tire_load))
            tire.CreateLongitudinalStiffnessAttr((5600.0 if vehicle_type == "Sport" else 5000.0) * force_scale)
            tire.CreateCamberStiffnessAttr(0.35 * tire_load)
            tire.CreateFrictionVsSlipGraphAttr([
                Gf.Vec2f(0.0, 0.88),
                Gf.Vec2f(0.10, dynamics["grip"]),
                Gf.Vec2f(1.0, dynamics["grip"] * 0.72),
            ])
            tire.CreateFrictionTableRel().AddTarget(tire_table_path)
            suspension = PhysxSchema.PhysxVehicleSuspensionAPI.Apply(wheel_prim)
            suspension.CreateSpringStrengthAttr(spring_strength)
            suspension.CreateSpringDamperRateAttr(spring_damping)
            suspension.CreateTravelDistanceAttr(travel)

            # Small, mirrored camber gain makes the loaded outside tire retain
            # a useful contact patch during body roll without a visual joint.
            compliance = PhysxSchema.PhysxVehicleSuspensionComplianceAPI.Apply(
                wheel_prim
            )
            side_sign = -1.0 if "Left" in item["slot"] else 1.0
            camber_gain = math.radians(1.2 if is_front else 0.8)
            compliance.CreateWheelCamberAngleAttr([
                Gf.Vec2f(0.0, side_sign * camber_gain),
                Gf.Vec2f(0.5, 0.0),
                Gf.Vec2f(1.0, -side_sign * camber_gain),
            ])
            compliance.CreateWheelToeAngleAttr([
                Gf.Vec2f(0.0, side_sign * math.radians(0.08)),
                Gf.Vec2f(1.0, side_sign * math.radians(0.03)),
            ])

            collision = UsdGeom.Cylinder.Define(
                stage, item["path"].AppendChild("Collision")
            )
            collision.CreatePurposeAttr(UsdGeom.Tokens.guide)
            collision.CreateAxisAttr(UsdGeom.Tokens.x)
            collision.CreateRadiusAttr(item["radius"])
            collision.CreateHeightAttr(item["width"])
            collision.CreateExtentAttr(UsdGeom.Cylinder.ComputeExtentFromPlugins(collision, 0))
            UsdPhysics.CollisionAPI.Apply(collision.GetPrim())
            add_collision_to_collision_group(stage, collision.GetPath(), group_paths["WheelGroup"])

        chassis = UsdGeom.Cube.Define(stage, vehicle_path.AppendChild("ChassisCollision"))
        chassis.CreatePurposeAttr(UsdGeom.Tokens.guide)
        chassis.AddTranslateOp().Set(chassis_center)
        chassis.AddScaleOp().Set(chassis_dims * 0.5)
        UsdPhysics.CollisionAPI.Apply(chassis.GetPrim())
        add_collision_to_collision_group(stage, chassis.GetPath(), group_paths["ChassisGroup"])

        vehicle.SetCustomDataByKey("massKg", mass)
        vehicle.SetCustomDataByKey("driveModel", f"PhysX Vehicle Drive Standard {dynamics['drive']}")
        vehicle.SetCustomDataByKey("frontWeightBias", dynamics["front_bias"])
        vehicle.SetCustomDataByKey("peakTorqueNm", dynamics["torque"])
        vehicle.SetCustomDataByKey("suspensionNaturalFrequencyHz", dynamics["hz"])
        vehicle.SetCustomDataByKey("tireGripScale", dynamics["grip"])


def _assemble_and_color(Usd, UsdGeom, UsdShade, Sdf, Gf, Kind):
    stage = Usd.Stage.Open(str(RAW_USD))
    if stage is None:
        raise RuntimeError(f"변환된 USD를 열 수 없습니다: {RAW_USD}")

    world = stage.GetDefaultPrim()
    if not world:
        raise RuntimeError("변환된 USD에 defaultPrim이 없습니다.")

    bbox_cache = UsdGeom.BBoxCache(
        Usd.TimeCode.Default(),
        [UsdGeom.Tokens.default_, UsdGeom.Tokens.render, UsdGeom.Tokens.proxy],
        useExtentsHint=True,
    )

    bodies = {}
    wheels = []
    for prim in world.GetChildren():
        vehicle_type = _vehicle_type_from_body_name(prim.GetName())
        if vehicle_type:
            bodies[vehicle_type] = prim
        elif _normal(prim.GetName()).startswith("wheel"):
            wheels.append(prim)

    missing_bodies = [name for name in VEHICLE_TYPES if name not in bodies]
    if missing_bodies:
        raise RuntimeError(f"차체 누락: {', '.join(missing_bodies)}")
    if len(wheels) != 40:
        raise RuntimeError(f"바퀴는 40개여야 하지만 {len(wheels)}개를 찾았습니다.")

    body_centers = {
        name: _world_center(prim, bbox_cache) for name, prim in bodies.items()
    }
    wheel_assignments = {name: [] for name in VEHICLE_TYPES}
    for wheel in wheels:
        center = _world_center(wheel, bbox_cache)
        vehicle_type = min(
            VEHICLE_TYPES,
            key=lambda name: _distance_squared(center, body_centers[name]),
        )
        wheel_assignments[vehicle_type].append(wheel)

    bad_wheel_counts = {
        name: len(items)
        for name, items in wheel_assignments.items()
        if len(items) != 4
    }
    if bad_wheel_counts:
        detail = ", ".join(f"{name}={count}" for name, count in bad_wheel_counts.items())
        raise RuntimeError(f"차량별 바퀴 배정 실패(각 4개 필요): {detail}")

    # Namespace를 바꾸기 전에 월드 변환과 실제 바퀴 크기를 보존한다. 차량
    # 루트가 차체의 월드 자세를 갖고, 모든 부품은 그 루트의 로컬 좌표가 된다.
    # 이 구조여야 차량을 움직였을 때 차체와 네 바퀴가 함께 이동한다.
    xform_cache = UsdGeom.XformCache(Usd.TimeCode.Default())
    body_world = {
        name: xform_cache.GetLocalToWorldTransform(bodies[name])
        for name in VEHICLE_TYPES
    }
    # FBX 모델 로컬축(X=좌우, Y=전후, Z=위)을 PhysX Vehicle 로컬축
    # (X=좌우, Y=위, Z=전후)으로 바꾼다. X 부호 반전은 proper rotation을
    # 유지하기 위한 것으로, 이후 실제 바퀴 좌표로 좌/우를 다시 판정한다.
    fbx_to_vehicle_basis = Gf.Matrix4d(
        -1, 0, 0, 0,
         0, 0, 1, 0,
         0, 1, 0, 0,
         0, 0, 0, 1,
    )
    vehicle_root_world = {
        name: fbx_to_vehicle_basis * body_world[name]
        for name in VEHICLE_TYPES
    }
    part_world = {
        str(prim.GetPath()): xform_cache.GetLocalToWorldTransform(prim)
        for prim in [*bodies.values(), *wheels]
    }
    wheel_sizes = {
        str(wheel.GetPath()): _bbox_size(wheel, bbox_cache) for wheel in wheels
    }

    # 일부 차종(Coupe 등)은 Body 원점 회전과 실제 휠베이스 방향이 다르다.
    # 네 바퀴 접지점의 주성분으로 전후축을 구해 차량 로컬 +Z에 정렬한다.
    import math
    for vehicle_type in VEHICLE_TYPES:
        preliminary = vehicle_root_world[vehicle_type]
        points = []
        for wheel in wheel_assignments[vehicle_type]:
            local = part_world[str(wheel.GetPath())] * preliminary.GetInverse()
            p = local.ExtractTranslation()
            points.append((float(p[0]), float(p[2])))
        mean_x = sum(p[0] for p in points) / 4.0
        mean_z = sum(p[1] for p in points) / 4.0
        cov_xx = sum((p[0] - mean_x) ** 2 for p in points)
        cov_zz = sum((p[1] - mean_z) ** 2 for p in points)
        cov_xz = sum(
            (p[0] - mean_x) * (p[1] - mean_z) for p in points
        )
        phi = 0.5 * math.atan2(2.0 * cov_xz, cov_xx - cov_zz)
        long_x, long_z = math.cos(phi), math.sin(phi)
        if long_z < 0:
            long_x, long_z = -long_x, -long_z
        yaw_basis = Gf.Matrix4d(
            long_z, 0, -long_x, 0,
            0,      1, 0,       0,
            long_x, 0, long_z,  0,
            0,      0, 0,       1,
        )
        vehicle_root_world[vehicle_type] = yaw_basis * preliminary

    vehicles_path = world.GetPath().AppendChild("Vehicles")
    vehicles_prim = UsdGeom.Xform.Define(stage, vehicles_path).GetPrim()
    Usd.ModelAPI(vehicles_prim).SetKind(Kind.Tokens.group)

    vehicle_paths = {}
    for vehicle_type in VEHICLE_TYPES:
        path = vehicles_path.AppendChild(vehicle_type)
        vehicle_prim = UsdGeom.Xform.Define(stage, path).GetPrim()
        Usd.ModelAPI(vehicle_prim).SetKind(Kind.Tokens.component)
        vehicle_prim.SetCustomDataByKey("sourceVehicleType", vehicle_type)
        vehicle_paths[vehicle_type] = path

    moved_wheels = {name: [] for name in VEHICLE_TYPES}
    for vehicle_type in VEHICLE_TYPES:
        parts = [bodies[vehicle_type], *wheel_assignments[vehicle_type]]
        for part in parts:
            old_path = part.GetPath()
            part_name = part.GetName()
            new_path = vehicle_paths[vehicle_type].AppendChild(part_name)
            editor = Usd.NamespaceEditor(stage)
            if not editor.MovePrimAtPath(old_path, new_path):
                raise RuntimeError(f"Prim 이동 요청 실패: {old_path} -> {new_path}")
            can_apply = editor.CanApplyEdits()
            if not can_apply:
                errors = getattr(can_apply, "errors", "알 수 없는 namespace 오류")
                raise RuntimeError(f"Prim 이동 불가: {old_path} -> {new_path}: {errors}")
            if not editor.ApplyEdits():
                raise RuntimeError(f"Prim 이동 실패: {old_path} -> {new_path}")

            moved = stage.GetPrimAtPath(new_path)
            local = (
                part_world[str(old_path)]
                * vehicle_root_world[vehicle_type].GetInverse()
            )
            _set_matrix_xform(moved, local, UsdGeom)
            if _normal(part_name).startswith("wheel"):
                moved_wheels[vehicle_type].append(
                    (moved, wheel_sizes[str(old_path)])
                )

        vehicle_prim = stage.GetPrimAtPath(vehicle_paths[vehicle_type])
        _set_matrix_xform(vehicle_prim, vehicle_root_world[vehicle_type], UsdGeom)

    # 바퀴를 FL/FR/RL/RR attachment 아래에 넣는다. attachment 자체는 차체
    # 로컬 좌표의 순수 이동만 가져 PhysX가 X축 회전/조향을 안정적으로 계산한다.
    wheel_report = {}
    for vehicle_type in VEHICLE_TYPES:
        vehicle_path = vehicle_paths[vehicle_type]
        wheel_data = []
        for wheel_prim, size in moved_wheels[vehicle_type]:
            local_matrix = UsdGeom.Xformable(wheel_prim).GetLocalTransformation()
            position = local_matrix.ExtractTranslation()
            wheel_data.append((wheel_prim, size, local_matrix, position))

        # 로컬 +Z가 전방, +X가 좌측인 FBX 원본 축을 사용한다.
        front = sorted(wheel_data, key=lambda item: float(item[3][2]), reverse=True)[:2]
        rear = [item for item in wheel_data if item not in front]
        ordered = []
        for axle_name, axle in (("Front", front), ("Rear", rear)):
            left, right = sorted(axle, key=lambda item: float(item[3][0]), reverse=True)
            ordered.extend(((axle_name + "Left", left), (axle_name + "Right", right)))

        wheel_report[vehicle_type] = []
        for index, (slot, (wheel_prim, size, old_local, position)) in enumerate(ordered):
            attachment_path = vehicle_path.AppendChild(slot + "Wheel")
            attachment = UsdGeom.Xform.Define(stage, attachment_path).GetPrim()
            UsdGeom.Xformable(attachment).AddTranslateOp().Set(position)

            old_path = wheel_prim.GetPath()
            visual_path = attachment_path.AppendChild("Visual")
            editor = Usd.NamespaceEditor(stage)
            if not editor.MovePrimAtPath(old_path, visual_path):
                raise RuntimeError(f"바퀴 visual 이동 요청 실패: {old_path}")
            can_apply = editor.CanApplyEdits()
            if not can_apply or not editor.ApplyEdits():
                raise RuntimeError(f"바퀴 visual 이동 실패: {old_path} -> {visual_path}")

            visual = stage.GetPrimAtPath(visual_path)
            translation = Gf.Matrix4d(1.0)
            translation.SetTranslate(position)
            _set_matrix_xform(visual, old_local * translation.GetInverse(), UsdGeom)

            dims = sorted(size)
            width = dims[0]
            radius = 0.25 * (dims[1] + dims[2])
            wheel_report[vehicle_type].append(
                {"path": attachment_path, "index": index, "slot": slot,
                 "position": position, "radius": radius, "width": width}
            )

    materials = _create_materials(stage, UsdGeom, UsdShade, Sdf, Gf)
    _create_showroom_lighting(stage, UsdGeom, Gf)
    color_report = {}
    for vehicle_type in VEHICLE_TYPES:
        vehicle_prim = stage.GetPrimAtPath(vehicle_paths[vehicle_type])
        color_report[vehicle_type] = _color_vehicle(
            vehicle_prim, vehicle_type, materials, Usd, UsdGeom, UsdShade
        )

    _add_vehicle_physics(
        stage, wheel_report, Usd, UsdGeom, UsdShade, Sdf, Gf
    )

    if OUTPUT_USD.exists():
        OUTPUT_USD.unlink()
    if not stage.Export(str(OUTPUT_USD)):
        raise RuntimeError(f"최종 USD 저장 실패: {OUTPUT_USD}")
    return color_report


def _verify_output(Usd, UsdGeom, UsdShade):
    from pxr import PhysxSchema, UsdPhysics
    stage = Usd.Stage.Open(str(OUTPUT_USD))
    if stage is None:
        raise RuntimeError("저장한 최종 USD를 다시 열 수 없습니다.")

    world_path = stage.GetDefaultPrim().GetPath()
    report = {}
    for vehicle_type in VEHICLE_TYPES:
        path = world_path.AppendChild("Vehicles").AppendChild(vehicle_type)
        vehicle = stage.GetPrimAtPath(path)
        if not vehicle:
            raise RuntimeError(f"최종 검증에서 차량 누락: {vehicle_type}")

        children = list(vehicle.GetChildren())
        bodies = [p for p in children if "body" in _normal(p.GetName())]
        wheels = [
            p for p in children
            if _normal(p.GetName()) in {
                "frontleftwheel", "frontrightwheel", "rearleftwheel", "rearrightwheel"
            }
        ]
        expected_body_material = (
            "BodyRed" if vehicle_type == "Sport"
            else "BodyArmy" if vehicle_type == "Offroad"
            else "BodyBlack" if vehicle_type in BLACK_BODY_TYPES
            else "BodyWhite"
        )
        body_material_ok = False
        for prim in Usd.PrimRange(bodies[0]) if bodies else []:
            if prim.IsA(UsdGeom.Subset) and "body" in _normal(prim.GetName()):
                material = UsdShade.MaterialBindingAPI(prim).ComputeBoundMaterial()[0]
                if material and material.GetPrim().GetName() == expected_body_material:
                    body_material_ok = True
        bound_wheels = 0
        for wheel in wheels:
            wheel_is_black = True
            found_render_prim = False
            for prim in Usd.PrimRange(wheel):
                if prim.IsA(UsdGeom.Gprim) or prim.IsA(UsdGeom.Subset):
                    if prim.IsA(UsdGeom.Imageable):
                        purpose = UsdGeom.Imageable(prim).ComputePurpose()
                        if purpose == UsdGeom.Tokens.guide:
                            continue
                    found_render_prim = True
                    material = UsdShade.MaterialBindingAPI(prim).ComputeBoundMaterial()[0]
                    if not material or material.GetPrim().GetName() != "Black":
                        wheel_is_black = False
            if found_render_prim and wheel_is_black:
                bound_wheels += 1
        if (
            len(bodies) != 1
            or len(wheels) != 4
            or bound_wheels != 4
            or not body_material_ok
            or not vehicle.HasAPI(UsdPhysics.RigidBodyAPI)
            or not vehicle.HasAPI(PhysxSchema.PhysxVehicleAPI)
            or any(
                not wheel.HasAPI(PhysxSchema.PhysxVehicleWheelAttachmentAPI)
                for wheel in wheels
            )
        ):
            raise RuntimeError(
                f"{vehicle_type} 최종 검증 실패: "
                f"Body={len(bodies)}, BodyColor={body_material_ok}, "
                f"Wheel={len(wheels)}, BlackWheel={bound_wheels}"
            )
        report[vehicle_type] = (len(bodies), len(wheels), bound_wheels)
    return report


def _run_physics_smoke_test(app, omni, UsdGeom):
    """생성 USD를 실제 PhysX에 로드해 짧게 진행하고 폭주/NaN을 검사한다."""
    import omni.timeline
    if not omni.usd.get_context().open_stage(str(OUTPUT_USD)):
        raise RuntimeError("물리 테스트용 USD 로드 실패")
    for _ in range(20):
        app.update()

    from pxr import PhysxSchema

    stage = omni.usd.get_context().get_stage()
    timeline = omni.timeline.get_timeline_interface()
    sport = stage.GetPrimAtPath("/World/Vehicles/Sport")
    start_position = (
        UsdGeom.Xformable(sport).ComputeLocalToWorldTransform(0).ExtractTranslation()
    )
    controller = PhysxSchema.PhysxVehicleControllerAPI(sport)
    controller.GetAcceleratorAttr().Set(0.65)
    timeline.play()
    app.update()  # Stage를 PhysX에 attach한다.
    import omni.physx
    simulation = omni.physx.get_physx_simulation_interface()
    dt = 1.0 / 60.0
    for step in range(180):
        simulation.simulate(dt, step * dt)
        simulation.fetch_results()
    controller.GetAcceleratorAttr().Set(0.0)
    end_position = omni.physx.get_physx_interface().get_rigidbody_transformation(
        "/World/Vehicles/Sport"
    )["position"]
    timeline.stop()
    app.update()

    import math
    checked = 0
    for vehicle_type in VEHICLE_TYPES:
        prim = stage.GetPrimAtPath(f"/World/Vehicles/{vehicle_type}")
        matrix = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(0)
        position = matrix.ExtractTranslation()
        if not all(math.isfinite(float(position[i])) for i in range(3)):
            raise RuntimeError(f"{vehicle_type} 물리 테스트에서 비정상 좌표 발생")
        if max(abs(float(position[i])) for i in range(3)) > 100000.0:
            raise RuntimeError(f"{vehicle_type} 물리 테스트에서 차량이 폭주함: {position}")
        checked += 1
    # Fabric/CPU direct stepping에서는 USD xform write-back이 꺼질 수 있으므로
    # PhysX 내부 강체 자세를 직접 조회한다.
    distance_stage = math.sqrt(sum(
        (float(end_position[i]) - float(start_position[i])) ** 2 for i in range(3)
    ))
    distance_m = distance_stage * UsdGeom.GetStageMetersPerUnit(stage)
    if distance_m < 0.05:
        raise RuntimeError(
            f"Sport 구동 테스트 실패: 가속 후 이동 거리 {distance_m:.3f} m"
        )
    print(
        f"[fab] PhysX 실제 진행 테스트 통과: 차량 {checked}대, "
        f"Sport 가속 이동 {distance_m:.2f} m",
        flush=True,
    )


def main():
    _restart_with_isaac_python()

    from isaacsim import SimulationApp

    headless = "--headless" in sys.argv[1:]
    app = SimulationApp({"headless": headless})
    try:
        import carb
        import omni
        from isaacsim.core.utils.extensions import enable_extension
        from pxr import Gf, Kind, Sdf, Usd, UsdGeom, UsdShade

        enable_extension("omni.kit.asset_converter")
        enable_extension("omni.physx.vehicle")
        # 확장 로딩을 마친 뒤 PhysX 스키마를 authoring 한다.
        for _ in range(5):
            app.update()
        asyncio.get_event_loop().run_until_complete(_convert_fbx(omni, carb))
        color_report = _assemble_and_color(
            Usd, UsdGeom, UsdShade, Sdf, Gf, Kind
        )
        verify_report = _verify_output(Usd, UsdGeom, UsdShade)

        if headless and "--skip-physics-test" not in sys.argv[1:]:
            _run_physics_smoke_test(app, omni, UsdGeom)

        print("\n[fab] 차량 조립·색상·저장 검증 완료", flush=True)
        for vehicle_type in VEHICLE_TYPES:
            _, wheel_count, black_wheels = verify_report[vehicle_type]
            color_name = (
                "빨강" if vehicle_type == "Sport"
                else "국방색" if vehicle_type == "Offroad"
                else "검정" if vehicle_type in BLACK_BODY_TYPES
                else "흰색"
            )
            subsets = color_report[vehicle_type]
            dynamics = VEHICLE_DYNAMICS[vehicle_type]
            print(
                f"  - {vehicle_type:10s}: 차체 {color_name}, "
                f"바퀴 {wheel_count}/4 결합·{black_wheels}/4 검정, "
                f"{dynamics['drive']}·{dynamics['mass']:.0f}kg, "
                f"Glass {subsets['glass']}개 검정",
                flush=True,
            )
        print(f"[fab] 최종 파일: {OUTPUT_USD}", flush=True)

        if not headless:
            # 생성 결과를 같은 프로세스의 Isaac Sim GUI에서 즉시 연다.
            # offline Usd.Stage로 작성했으므로 여기서 GUI USD context에 다시 로드한다.
            if not omni.usd.get_context().open_stage(str(OUTPUT_USD)):
                raise RuntimeError(f"GUI에서 최종 USD를 열지 못했습니다: {OUTPUT_USD}")

            # Stage와 viewport가 준비될 시간을 준 다음 전체 차량을 화면에 맞춘다.
            for _ in range(30):
                app.update()

            # 한 번에 한 차량만 키보드 입력을 받게 한다. 나머지 차량도 동일한
            # 물리 구성을 갖지만 정지 상태로 장면에 남는다.
            import omni.physxvehicle
            controlled_vehicle = "/World/Vehicles/Sport"
            omni.physxvehicle.get_physx_vehicle_interface().set_input_enabled(
                controlled_vehicle, True
            )

            import omni.kit.commands
            from omni.kit.viewport.utility import get_active_viewport

            viewport = get_active_viewport()
            if viewport and viewport.camera_path:
                resolution = viewport.resolution
                aspect_ratio = resolution[0] / max(1, resolution[1])
                omni.kit.commands.execute(
                    "FramePrimsCommand",
                    prim_to_move=viewport.camera_path,
                    prims_to_frame=["/World/Vehicles"],
                    time_code=Usd.TimeCode.Default(),
                    aspect_ratio=aspect_ratio,
                    zoom=0.80,
                )

            print(
                "[fab] 빨간 Sport 차량을 제어 대상으로 지정했습니다. "
                "상단 Play를 누른 뒤 방향키(↑ 가속, ↓ 제동/후진, ←/→ 조향)로 "
                "주행하세요. 창을 닫으면 프로그램이 종료됩니다.",
                flush=True,
            )
            while app.is_running():
                app.update()
    except Exception:
        # SimulationApp.close() 전에 traceback을 출력해야 Kit 종료 과정에서도
        # 실제 실패 원인이 터미널에 남는다.
        import traceback

        traceback.print_exc()
        raise
    finally:
        app.close()


if __name__ == "__main__":
    main()
