#!/usr/bin/env python3
"""차량 규격에 맞춘 Isaac Sim 주차장 USD 생성기.

실행:
    python3 build_parking_environment.py
    python3 build_parking_environment.py --headless

일반 Python에서 실행하면 Isaac Sim의 python.sh로 자동 전환한다. 기본 실행은
결과 USD를 Isaac Sim 창에 열고, --headless는 파일 생성과 검증 후 종료한다.
"""

import os
import sys
from pathlib import Path


WORK_DIR = Path(__file__).resolve().parent
OUTPUT_USD = WORK_DIR / "parking_environment.usd"
PROJECT_ROOT = (
    WORK_DIR.parent
    if (WORK_DIR.parent / "fab_vehicles.usd").is_file()
    else Path("/home/rokey/Isaac_envo")
)
VEHICLE_USD = PROJECT_ROOT / "fab_vehicles.usd"
ISAAC_ROOT = Path(
    "/home/rokey/dev_ws/isaac_sim/isaacsim/_build/linux-x86_64/release"
)
ISAAC_PYTHON = ISAAC_ROOT / "python.sh"

# fab_vehicles.usd의 최대 차량(Pickup 약 2.33 x 5.83 m)을 기준으로 여유 확보.
SPACE_COUNT = 10
PARKING_INDICES = tuple(range(1, 9))
ACCESSIBLE_SPACES = {"A1", "A2"}
SPACE_WIDTH = 3.40
SPACE_LENGTH = 6.60
AISLE_WIDTH = 9.00
LINE_WIDTH = 0.11
WALL_HEIGHT = 5.60
WALL_THICKNESS = 0.30
CEILING_HEIGHT = 5.60
CEILING_THICKNESS = 0.20
PILLAR_SIZE = 0.48
BORDER_MARGIN = 1.10
TARGET_INDEX = 5
LIDAR_CONFIG = "SICK_multiScan136"
LIDAR_X_FRACTION = 0.46
HANDOFF_LENGTH = 23.0
HANDOFF_WIDTH = 11.4
# Stage는 Y-up이다. 바닥 상면(Y=0)에서 1 mm 이내에 모든 도색을 배치한다.
PAINT_CENTER_Y = 0.0002
PAINT_THICKNESS = 0.0004
PAINT_SURFACE_Y = 0.0008

# 한 번 정한 초기 배치다. 실행할 때마다 다시 섞지 않는다.
# A3/B3는 운전자 직접 주차, 나머지 네 칸은 주차로봇 완료 차량이다.
PARKED_VEHICLES = (
    ("A3", "Compact", "white", "driver_parked"),
    ("A5", "Coupe", "black", "robot_parked"),
    ("A6", "Hatchback", "white", "robot_parked"),
    ("B3", "Minivan", "black", "driver_parked"),
    ("B5", "Pickup", "white", "robot_parked"),
    ("B7", "Sedan", "black", "robot_parked"),
)

# 총 12대를 맞추기 위한 외부 인계 대기 차량 6대. H1부터 순차 처리한다.
HANDOFF_VEHICLES = (
    ("H1", "SUV", "white"),
    ("H2", "Wagon", "black"),
    ("H3", "Sport", "red"),
    ("H4", "Offroad", "army"),
    ("H5", "Hatchback", "white"),
    ("H6", "Minivan", "black"),
)
FAB_VEHICLE_TYPES = {
    "Compact", "Coupe", "Hatchback", "Minivan", "Offroad",
    "Pickup", "Sedan", "Sport", "SUV", "Wagon",
}


def _restart_with_isaac_python():
    if os.environ.get("CARB_APP_PATH"):
        return
    if not ISAAC_PYTHON.is_file():
        raise FileNotFoundError(f"Isaac Sim python.sh를 찾을 수 없습니다: {ISAAC_PYTHON}")
    print(f"[parking] Isaac Sim Python으로 전환: {ISAAC_PYTHON}", flush=True)
    os.execv(
        str(ISAAC_PYTHON),
        [str(ISAAC_PYTHON), str(Path(__file__).resolve()), *sys.argv[1:]],
    )


def _material(stage, path, color, roughness=0.7, metallic=0.0, emissive=None):
    from pxr import Gf, Sdf, UsdShade

    material = UsdShade.Material.Define(stage, path)
    shader = UsdShade.Shader.Define(stage, path.AppendChild("Shader"))
    shader.CreateIdAttr("UsdPreviewSurface")
    shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*color))
    shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(float(roughness))
    shader.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(float(metallic))
    if emissive is not None:
        shader.CreateInput("emissiveColor", Sdf.ValueTypeNames.Color3f).Set(
            Gf.Vec3f(*emissive)
        )
    shader.CreateOutput("surface", Sdf.ValueTypeNames.Token)
    material.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")
    return material


def _cube(stage, path, position, size, material, collision=False, purpose=None):
    from pxr import Gf, UsdGeom, UsdPhysics, UsdShade

    cube = UsdGeom.Cube.Define(stage, path)
    cube.CreateSizeAttr(1.0)
    cube.AddTranslateOp().Set(Gf.Vec3d(*position))
    cube.AddScaleOp().Set(Gf.Vec3f(*size))
    cube.CreateDisplayColorAttr([material[1]])
    UsdShade.MaterialBindingAPI.Apply(cube.GetPrim()).Bind(material[0])
    if collision:
        UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
    if purpose:
        cube.CreatePurposeAttr(purpose)
    return cube


def _cylinder(stage, path, position, radius, height, material, collision=False):
    from pxr import Gf, UsdGeom, UsdPhysics, UsdShade

    cylinder = UsdGeom.Cylinder.Define(stage, path)
    cylinder.CreateAxisAttr(UsdGeom.Tokens.y)
    cylinder.CreateRadiusAttr(float(radius))
    cylinder.CreateHeightAttr(float(height))
    cylinder.AddTranslateOp().Set(Gf.Vec3d(*position))
    cylinder.CreateDisplayColorAttr([material[1]])
    UsdShade.MaterialBindingAPI.Apply(cylinder.GetPrim()).Bind(material[0])
    if collision:
        UsdPhysics.CollisionAPI.Apply(cylinder.GetPrim())
    return cylinder


def _outline(stage, root, center_x, center_z, width, length, material, thickness=0.09):
    """바닥에 충돌 없는 사각 테두리를 만든다."""
    for name, pos, size in (
        ("Left", (center_x - width * 0.5, PAINT_CENTER_Y, center_z),
         (thickness, PAINT_THICKNESS, length)),
        ("Right", (center_x + width * 0.5, PAINT_CENTER_Y, center_z),
         (thickness, PAINT_THICKNESS, length)),
        ("Front", (center_x, PAINT_CENTER_Y, center_z - length * 0.5),
         (width, PAINT_THICKNESS, thickness)),
        ("Back", (center_x, PAINT_CENTER_Y, center_z + length * 0.5),
         (width, PAINT_THICKNESS, thickness)),
    ):
        _cube(stage, root.AppendChild(name), pos, size, material)


def _paint_segment(stage, path, start, end, color, width, y=PAINT_SURFACE_Y):
    """충돌과 물리 두께가 없는 X/Z 평면 도색 선분을 만든다."""
    from pxr import Gf, UsdGeom

    x0, z0 = start
    x1, z1 = end
    dx, dz = x1 - x0, z1 - z0
    length = (dx * dx + dz * dz) ** 0.5
    if length <= 1e-7:
        return None
    px = -dz / length * width * 0.5
    pz = dx / length * width * 0.5
    mesh = UsdGeom.Mesh.Define(stage, path)
    mesh.CreatePointsAttr([
        Gf.Vec3f(x0 - px, y, z0 - pz),
        Gf.Vec3f(x0 + px, y, z0 + pz),
        Gf.Vec3f(x1 + px, y, z1 + pz),
        Gf.Vec3f(x1 - px, y, z1 - pz),
    ])
    mesh.CreateFaceVertexCountsAttr([4])
    mesh.CreateFaceVertexIndicesAttr([0, 1, 2, 3])
    mesh.CreateSubdivisionSchemeAttr(UsdGeom.Tokens.none)
    mesh.CreateDoubleSidedAttr(True)
    mesh.CreateDisplayColorAttr([Gf.Vec3f(*color)])
    return mesh


def _paint_polyline(stage, root, points, color, width, closed=False):
    """여러 평면 선분으로 도색 폴리라인을 만든다."""
    pairs = list(zip(points, points[1:]))
    if closed and points:
        pairs.append((points[-1], points[0]))
    for index, (start, end) in enumerate(pairs):
        _paint_segment(
            stage, root.AppendChild(f"Paint_{index:02d}"),
            start, end, color, width,
        )


def _paint_disc(stage, path, center_x, center_z, radius, color, segments=20):
    """교통약자 표지 머리처럼 쓰는 충돌 없는 평면 원형 도색."""
    import math
    from pxr import Gf, UsdGeom

    mesh = UsdGeom.Mesh.Define(stage, path)
    points = [Gf.Vec3f(center_x, PAINT_SURFACE_Y, center_z)]
    for index in range(segments):
        angle = 2.0 * math.pi * index / segments
        points.append(Gf.Vec3f(
            center_x + radius * math.cos(angle),
            PAINT_SURFACE_Y,
            center_z + radius * math.sin(angle),
        ))
    mesh.CreatePointsAttr(points)
    mesh.CreateFaceVertexCountsAttr([3] * segments)
    indices = []
    for index in range(segments):
        indices.extend([0, index + 1, (index + 1) % segments + 1])
    mesh.CreateFaceVertexIndicesAttr(indices)
    mesh.CreateSubdivisionSchemeAttr(UsdGeom.Tokens.none)
    mesh.CreateDoubleSidedAttr(True)
    mesh.CreateDisplayColorAttr([Gf.Vec3f(*color)])
    return mesh


def _accessible_symbol(stage, root, center_x, center_z, material):
    """교통약자 주차면에 완전한 평면 Mesh 도색 심볼을 만든다."""
    color = tuple(material[1])
    _paint_disc(
        stage, root.AppendChild("Head"),
        center_x - 0.20, center_z + 0.48, 0.13, color,
    )
    _paint_polyline(
        stage, root.AppendChild("Body"),
        [(center_x - 0.18, center_z + 0.27),
         (center_x - 0.06, center_z - 0.02),
         (center_x + 0.31, center_z - 0.02)],
        color, 0.12,
    )
    # 원형 바퀴를 12개 선분으로 표현한다.
    import math
    wheel = []
    for i in range(13):
        angle = 2.0 * math.pi * i / 12.0
        wheel.append((center_x - 0.04 + 0.42 * math.cos(angle),
                      center_z - 0.30 + 0.42 * math.sin(angle)))
    _paint_polyline(stage, root.AppendChild("Wheel"), wheel, color, 0.10)
    _paint_polyline(
        stage, root.AppendChild("ArmAndLeg"),
        [(center_x - 0.08, center_z + 0.06),
         (center_x + 0.18, center_z + 0.17),
         (center_x + 0.38, center_z - 0.31)],
        color, 0.10,
    )


def _curve(stage, path, points, color, width=0.07):
    from pxr import Gf, UsdGeom

    curve = UsdGeom.BasisCurves.Define(stage, path)
    curve.CreateTypeAttr(UsdGeom.Tokens.linear)
    curve.CreateWrapAttr(UsdGeom.Tokens.nonperiodic)
    curve.CreateCurveVertexCountsAttr([len(points)])
    curve.CreatePointsAttr([Gf.Vec3f(*p) for p in points])
    curve.CreateWidthsAttr([float(width)])
    curve.SetWidthsInterpolation(UsdGeom.Tokens.constant)
    curve.CreateDisplayColorAttr([Gf.Vec3f(*color)])
    return curve


def _dashed_route(stage, root, points, color, material, dash=0.38, gap=0.22):
    """X/Z 직선으로 구성된 경로를 바닥 위 점선 큐브로 만든다."""
    from pxr import Gf, Sdf

    index = 0
    for start, end in zip(points, points[1:]):
        sx, sz = start
        ex, ez = end
        dx, dz = ex - sx, ez - sz
        length = (dx * dx + dz * dz) ** 0.5
        if length <= 1e-6:
            continue
        ux, uz = dx / length, dz / length
        distance = 0.0
        while distance < length:
            segment = min(dash, length - distance)
            center = distance + segment * 0.5
            x, z = sx + ux * center, sz + uz * center
            # 현재 경로는 X축 또는 Z축 구간으로만 구성한다.
            size = ((segment, PAINT_THICKNESS, 0.075) if abs(dx) >= abs(dz)
                    else (0.075, PAINT_THICKNESS, segment))
            _cube(
                stage,
                root.AppendChild(f"Dash_{index:03d}"),
                (x, PAINT_CENTER_Y, z),
                size,
                material,
            )
            index += 1
            distance += dash + gap
    guide = stage.GetPrimAtPath(root)
    guide.CreateAttribute("navigation:waypoints", Sdf.ValueTypeNames.Float3Array).Set(
        [Gf.Vec3f(x, 0.0, z) for x, z in points]
    )
    guide.CreateAttribute("navigation:color", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*color))


def _label(stage, path, text, center_x, center_z, color):
    """바닥 위에 충돌 없이 평면 도색으로 표시하는 A/B + 숫자 문자."""
    from pxr import Sdf, UsdGeom

    root = UsdGeom.Xform.Define(stage, path).GetPrim()
    root.CreateAttribute("parking:label", Sdf.ValueTypeNames.String).Set(text)
    root.CreateAttribute("parking:markingType", Sdf.ValueTypeNames.Token).Set(
        "flat_floor_paint"
    )
    # Mesh에는 CollisionAPI를 적용하지 않으므로 로봇 바퀴와 하부가 이 문자를
    # 물리 형상으로 인식하지 않는다.
    s = 0.22
    stroke_width = 0.070
    strokes = []
    letter = text[0]
    if letter == "A":
        strokes.extend([
            [(-0.34, -0.22), (-0.20, 0.22), (-0.06, -0.22)],
            [(-0.28, -0.02), (-0.12, -0.02)],
        ])
    else:
        strokes.extend([
            [(-0.34, -0.22), (-0.34, 0.22), (-0.14, 0.18), (-0.14, 0.02), (-0.34, 0.0)],
            [(-0.34, 0.0), (-0.12, -0.04), (-0.12, -0.20), (-0.34, -0.22)],
        ])
    digit = int(text[1:])
    segments = {
        0: "abcedf", 1: "bc", 2: "abged", 3: "abgcd", 4: "fgbc",
        5: "afgcd", 6: "afgecd", 7: "abc", 8: "abcdefg", 9: "abfgcd",
    }[digit]
    p = {
        "a": [(0.03, 0.22), (0.25, 0.22)], "b": [(0.25, 0.22), (0.25, 0.0)],
        "c": [(0.25, 0.0), (0.25, -0.22)], "d": [(0.03, -0.22), (0.25, -0.22)],
        "e": [(0.03, 0.0), (0.03, -0.22)], "f": [(0.03, 0.22), (0.03, 0.0)],
        "g": [(0.03, 0.0), (0.25, 0.0)],
    }
    strokes.extend([p[name] for name in segments])
    segment_index = 0
    for stroke in strokes:
        points = [
            (center_x + x * s / 0.22, center_z + z * s / 0.22)
            for x, z in stroke
        ]
        for start, end in zip(points, points[1:]):
            _paint_segment(
                stage, path.AppendChild(f"Paint_{segment_index:02d}"),
                start, end, color, stroke_width,
            )
            segment_index += 1


def _vehicle_instance(
    stage, path, vehicle_type, color_name, position, yaw_degrees, workflow_state
):
    """build_fab_vehicles.py의 완성 차량을 물리/재질 포함 그대로 배치한다."""
    import re
    from pxr import Gf, PhysxSchema, Sdf, Usd, UsdGeom, UsdPhysics, UsdShade
    from omni.physx.scripts.physicsUtils import add_collision_to_collision_group

    vehicle = stage.DefinePrim(path, "Xform")
    vehicle.GetReferences().AddReference(
        str(VEHICLE_USD), f"/World/Vehicles/{vehicle_type}"
    )
    xform = UsdGeom.Xformable(vehicle)
    xform.ClearXformOpOrder()
    matrix = Gf.Matrix4d(1.0)
    matrix.SetRotate(Gf.Rotation(Gf.Vec3d(0.0, 1.0, 0.0), float(yaw_degrees)))
    matrix.SetTranslateOnly(Gf.Vec3d(*position))
    xform.AddTransformOp().Set(matrix)

    vehicle.CreateAttribute("parking:vehicleType", Sdf.ValueTypeNames.String).Set(vehicle_type)
    vehicle.CreateAttribute("parking:bodyColor", Sdf.ValueTypeNames.Token).Set(color_name)
    vehicle.CreateAttribute("parking:workflowState", Sdf.ValueTypeNames.Token).Set(workflow_state)
    vehicle.CreateAttribute("parking:initialPlacement", Sdf.ValueTypeNames.Bool).Set(True)
    vehicle.CreateAttribute("parking:sourcePrim", Sdf.ValueTypeNames.String).Set(
        f"/World/Vehicles/{vehicle_type}"
    )

    # 원본의 RigidBody, 질량/무게중심, PhysX Vehicle 구동계, 서스펜션과
    # 타이어 속성을 유지한다. 주차로봇이 들어 올릴 수 있도록 kinematic은 끈다.
    rigid = UsdPhysics.RigidBodyAPI.Apply(vehicle)
    rigid.CreateKinematicEnabledAttr().Set(False)
    rigid.CreateRigidBodyEnabledAttr().Set(True)
    if not vehicle.HasAPI(PhysxSchema.PhysxVehicleAPI):
        raise RuntimeError(f"{vehicle_type}에 원본 PhysxVehicleAPI가 없습니다.")

    # USD reference는 참조된 Prim 바깥의 절대 relationship target을 캡슐화한다.
    # 원본 FAB 재질과 물리 지원 Prim은 Stage에 함께 합성되어 있으므로, 동일한
    # target을 인스턴스 경로에서 다시 연결해 원본 결과를 손실 없이 보존한다.
    body_material_name = (
        "BodyRed" if vehicle_type == "Sport"
        else "BodyArmy" if vehicle_type == "Offroad"
        else "BodyBlack" if vehicle_type in {"Coupe", "Minivan", "Sedan", "Wagon"}
        else "BodyWhite"
    )
    fab_materials = {
        name: UsdShade.Material.Get(stage, f"/World/Looks/FabColors/{name}")
        for name in (body_material_name, "Black", "GlassBlack", "LightWhite")
    }

    # 원본 CollisionGroup의 collection에는 원래 /World/Vehicles 경로가 들어
    # 있으므로, 인스턴스 경로의 차체와 바퀴 충돌체를 공용 그룹에 추가한다.
    for prim in Usd.PrimRange(vehicle):
        prim_path = str(prim.GetPath())
        normalized = re.sub(r"[^a-z0-9]", "", prim.GetName().lower())
        is_wheel_part = "wheel" in re.sub(r"[^a-z0-9]", "", prim_path.lower())

        material = None
        if prim.IsA(UsdGeom.Subset) and is_wheel_part:
            material = fab_materials["Black"]
        elif prim.IsA(UsdGeom.Subset) and "glass" in normalized:
            material = fab_materials["GlassBlack"]
        elif prim.IsA(UsdGeom.Subset) and any(
            key in normalized for key in ("optic", "light")
        ):
            material = fab_materials["LightWhite"]
        elif prim.IsA(UsdGeom.Subset) and "body" in normalized:
            material = fab_materials[body_material_name]
        elif prim.IsA(UsdGeom.Gprim) and is_wheel_part:
            material = fab_materials["Black"]
        if material and material.GetPrim().IsValid():
            UsdShade.MaterialBindingAPI.Apply(prim).Bind(material)

        if prim.HasAPI(PhysxSchema.PhysxVehicleTireAPI):
            PhysxSchema.PhysxVehicleTireAPI(prim).CreateFrictionTableRel().SetTargets([
                Sdf.Path("/World/VehiclePhysics/RoadTireFriction")
            ])
        if prim.HasAPI(PhysxSchema.PhysxVehicleWheelAttachmentAPI):
            PhysxSchema.PhysxVehicleWheelAttachmentAPI(
                prim
            ).CreateCollisionGroupRel().SetTargets([
                Sdf.Path("/World/VehiclePhysics/GroundQueryGroup")
            ])

        if prim.HasAPI(UsdPhysics.CollisionAPI):
            if prim.GetName() == "ChassisCollision":
                group = Sdf.Path("/World/VehiclePhysics/ChassisGroup")
            elif prim.GetName() == "Collision" and "Wheel" in prim_path:
                group = Sdf.Path("/World/VehiclePhysics/WheelGroup")
            else:
                continue
            add_collision_to_collision_group(stage, prim.GetPath(), group)
    return vehicle


def build_stage():
    from pxr import Gf, Sdf, Usd, UsdGeom, UsdLux, UsdPhysics, UsdShade

    if not VEHICLE_USD.is_file():
        raise FileNotFoundError(
            f"build_fab_vehicles.py가 생성한 차량 USD가 없습니다: {VEHICLE_USD}"
        )
    if OUTPUT_USD.exists():
        OUTPUT_USD.unlink()
    stage = Usd.Stage.CreateNew(str(OUTPUT_USD))
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.y)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    stage.SetTimeCodesPerSecond(60.0)

    world = UsdGeom.Xform.Define(stage, "/World").GetPrim()
    stage.SetDefaultPrim(world)
    world.SetCustomDataByKey("generator", "build_parking_environment.py")
    world.SetCustomDataByKey("vehicleAsset", str(VEHICLE_USD))
    world.SetCustomDataByKey("parkingSpaceWidthM", SPACE_WIDTH)
    world.SetCustomDataByKey("parkingSpaceLengthM", SPACE_LENGTH)
    world.SetCustomDataByKey("centralAisleWidthM", AISLE_WIDTH)
    world.SetCustomDataByKey("activeParkingSpaces", "A1-A8, B1-B8")
    world.SetCustomDataByKey("accessibleParkingSpaces", "A1, A2")
    world.SetCustomDataByKey("ceilingLidarConfig", LIDAR_CONFIG)
    world.SetCustomDataByKey("ceilingLidarCount", 2)
    world.SetCustomDataByKey("initialVehicleCount", 12)
    world.SetCustomDataByKey("initialUniqueFabVehicleTypes", 10)
    world.SetCustomDataByKey("driverParkedVehicles", 2)
    world.SetCustomDataByKey("robotParkedVehicles", 4)

    scene = UsdPhysics.Scene.Define(stage, "/World/PhysicsScene")
    # VehicleContextAPI와 축/업데이트 모드는 차량 에셋의 설정을 그대로 합성한다.
    scene.GetPrim().GetReferences().AddReference(
        str(VEHICLE_USD), "/World/PhysicsScene"
    )
    scene.CreateGravityDirectionAttr(Gf.Vec3f(0.0, -1.0, 0.0))
    scene.CreateGravityMagnitudeAttr(9.81)

    # 타이어 마찰표와 차체/바퀴/지면 CollisionGroup도 원본 차량 에셋에서 가져온다.
    vehicle_physics = stage.DefinePrim("/World/VehiclePhysics", "Scope")
    vehicle_physics.GetReferences().AddReference(
        str(VEHICLE_USD), "/World/VehiclePhysics"
    )
    # 참조 원본의 collection은 /World/Vehicles 아래 쇼룸 차량을 가리킨다.
    # 주차장 인스턴스 경로를 아래에서 새로 등록하므로 원본 includes만 비운다.
    for group_name in ("ChassisGroup", "WheelGroup", "GroundGroup"):
        group = stage.GetPrimAtPath(f"/World/VehiclePhysics/{group_name}")
        includes = group.GetRelationship("collection:colliders:includes")
        if includes:
            includes.SetTargets([])

    looks = UsdGeom.Scope.Define(stage, "/World/Looks").GetPath()
    # 차량 개별 Prim의 원본 차체/유리/조명/타이어 재질을 함께 합성한다.
    fab_colors = stage.DefinePrim("/World/Looks/FabColors")
    fab_colors.GetReferences().AddReference(str(VEHICLE_USD), "/World/Looks/FabColors")
    colors = {
        "asphalt": Gf.Vec3f(0.115, 0.125, 0.14),
        "white": Gf.Vec3f(0.92, 0.94, 0.96),
        "yellow": Gf.Vec3f(0.98, 0.72, 0.04),
        "green": Gf.Vec3f(0.05, 0.82, 0.44),
        "orange": Gf.Vec3f(1.0, 0.25, 0.035),
        "wall": Gf.Vec3f(0.48, 0.50, 0.54),
        "ceiling": Gf.Vec3f(0.66, 0.67, 0.69),
        "pillar": Gf.Vec3f(0.56, 0.57, 0.60),
        "blue": Gf.Vec3f(0.025, 0.31, 0.78),
        "charge": Gf.Vec3f(0.04, 0.88, 0.83),
        "dark": Gf.Vec3f(0.025, 0.032, 0.045),
        "light": Gf.Vec3f(1.0, 0.94, 0.78),
    }
    mats = {}
    for name, color in colors.items():
        mat = _material(
            stage, looks.AppendChild(name.title()), tuple(color),
            0.92 if name == "asphalt" else 0.55,
            emissive=tuple(color * (2.5 if name == "light" else 0.18))
            if name in {"green", "orange", "charge", "light"} else None,
        )
        mats[name] = (mat, color)

    half_w = SPACE_COUNT * SPACE_WIDTH * 0.5
    row_center = AISLE_WIDTH * 0.5 + SPACE_LENGTH * 0.5
    half_d = AISLE_WIDTH * 0.5 + SPACE_LENGTH
    floor_w = half_w * 2 + BORDER_MARGIN * 2
    floor_d = half_d * 2 + BORDER_MARGIN * 2

    environment = UsdGeom.Xform.Define(stage, "/World/ParkingEnvironment").GetPath()
    floor = _cube(
        stage, environment.AppendChild("Floor"),
        (0.0, -0.06, 0.0), (floor_w, 0.12, floor_d), mats["asphalt"], collision=True,
    )
    physics_mat = UsdShade.Material.Define(stage, "/World/Looks/GroundPhysics")
    phys = UsdPhysics.MaterialAPI.Apply(physics_mat.GetPrim())
    phys.CreateStaticFrictionAttr(1.05)
    phys.CreateDynamicFrictionAttr(0.90)
    phys.CreateRestitutionAttr(0.02)
    UsdShade.MaterialBindingAPI.Apply(floor.GetPrim()).Bind(
        physics_mat, UsdShade.Tokens.weakerThanDescendants, "physics"
    )
    from omni.physx.scripts.physicsUtils import add_collision_to_collision_group
    add_collision_to_collision_group(
        stage, floor.GetPath(), Sdf.Path("/World/VehiclePhysics/GroundGroup")
    )

    markings = UsdGeom.Xform.Define(stage, environment.AppendChild("Markings")).GetPath()
    spaces_root = UsdGeom.Xform.Define(stage, environment.AppendChild("Spaces")).GetPath()
    for row_name, z_sign in (("A", 1.0), ("B", -1.0)):
        z_center = z_sign * row_center
        back_z = z_sign * half_d
        # 0번과 9번 자리의 영역은 로봇 대기/충전 구역으로 전환한다.
        for boundary in range(1, SPACE_COUNT):
            x = -half_w + boundary * SPACE_WIDTH
            _cube(
                stage, markings.AppendChild(f"{row_name}_Divider_{boundary:02d}"),
                (x, PAINT_CENTER_Y, z_center),
                (LINE_WIDTH, PAINT_THICKNESS, SPACE_LENGTH), mats["white"],
            )
        _cube(
            stage, markings.AppendChild(f"{row_name}_BackLine"),
            (0.0, PAINT_CENTER_Y, back_z),
            (half_w * 2 + LINE_WIDTH, PAINT_THICKNESS, LINE_WIDTH), mats["white"],
        )
        for index in PARKING_INDICES:
            x = -half_w + (index + 0.5) * SPACE_WIDTH
            label = f"{row_name}{index}"
            spot = UsdGeom.Xform.Define(stage, spaces_root.AppendChild(label)).GetPrim()
            spot.CreateAttribute("parking:center", Sdf.ValueTypeNames.Float3).Set(Gf.Vec3f(x, 0.0, z_center))
            spot.CreateAttribute("parking:width", Sdf.ValueTypeNames.Float).Set(SPACE_WIDTH)
            spot.CreateAttribute("parking:length", Sdf.ValueTypeNames.Float).Set(SPACE_LENGTH)
            spot.CreateAttribute("parking:occupied", Sdf.ValueTypeNames.Bool).Set(False)
            accessible = label in ACCESSIBLE_SPACES
            spot.CreateAttribute("parking:accessible", Sdf.ValueTypeNames.Bool).Set(accessible)
            spot.CreateAttribute("parking:designation", Sdf.ValueTypeNames.String).Set(
                "accessible" if accessible else "standard"
            )
            if accessible:
                _cube(
                    stage, spot.GetPath().AppendChild("BlueFloor"),
                    (x, PAINT_CENTER_Y, z_center),
                    (SPACE_WIDTH - 0.18, PAINT_THICKNESS, SPACE_LENGTH - 0.18),
                    mats["blue"],
                )
                _accessible_symbol(
                    stage, spot.GetPath().AppendChild("AccessibleSymbol"),
                    x, z_center, mats["white"],
                )
            # A1/A2는 교통약자 심볼 자체로 식별되므로 번호 도색을 중복 표시하지 않는다.
            if not accessible:
                _label(
                    stage, spot.GetPath().AppendChild("Label"),
                    label, x, z_center, colors["white"],
                )

    # 남은 네 끝 구획은 주차로봇의 대기 및 자동충전 도킹 스테이션이다.
    service = UsdGeom.Xform.Define(stage, environment.AppendChild("RobotServiceArea")).GetPath()
    dock_report = []
    for side_name, slot_index, role in (("West", 0, "waiting"), ("East", 9, "charging")):
        x = -half_w + (slot_index + 0.5) * SPACE_WIDTH
        for row_name, z_sign in (("A", 1.0), ("B", -1.0)):
            z_center = z_sign * row_center
            station_name = f"{side_name}_{row_name}_{role.title()}Dock"
            station = UsdGeom.Xform.Define(stage, service.AppendChild(station_name)).GetPrim()
            station.CreateAttribute("robot:serviceRole", Sdf.ValueTypeNames.String).Set(role)
            station.CreateAttribute("robot:dockPose", Sdf.ValueTypeNames.Float3).Set(
                Gf.Vec3f(x, 0.0, z_center)
            )
            _outline(
                stage, station.GetPath().AppendChild("FloorOutline"), x, z_center,
                SPACE_WIDTH - 0.25, SPACE_LENGTH - 0.25,
                mats["yellow"] if role == "waiting" else mats["charge"], 0.12,
            )
            # 로봇청소기 도크처럼 벽 쪽의 낮은 충전 타워와 접촉 전극을 둔다.
            dock_z = z_sign * (half_d - 0.38)
            _cube(
                stage, station.GetPath().AppendChild("DockTower"),
                (x, 0.30, dock_z), (1.10, 0.60, 0.34), mats["dark"], collision=True,
            )
            _cube(
                stage, station.GetPath().AppendChild("ChargeContacts"),
                (x, 0.18, dock_z - z_sign * 0.19),
                (0.64, 0.12, 0.035), mats["charge"],
            )
            _cube(
                stage, station.GetPath().AppendChild("DockPad"),
                (x, PAINT_CENTER_Y, dock_z - z_sign * 0.80),
                (2.10, PAINT_THICKNESS, 1.35),
                mats["charge"] if role == "charging" else mats["yellow"],
            )
            dock_report.append((station_name, x, z_center, role))

    # 원본 주차장의 높은 벽/기둥 구성을 복원한다. 서쪽 중앙은 차량 출입구다.
    walls = UsdGeom.Xform.Define(stage, environment.AppendChild("Walls")).GetPath()
    wall_y = WALL_HEIGHT * 0.5
    _cube(stage, walls.AppendChild("North"), (0, wall_y, half_d + BORDER_MARGIN),
          (floor_w, WALL_HEIGHT, WALL_THICKNESS), mats["wall"], collision=True)
    _cube(stage, walls.AppendChild("South"), (0, wall_y, -half_d - BORDER_MARGIN),
          (floor_w, WALL_HEIGHT, WALL_THICKNESS), mats["wall"], collision=True)
    _cube(stage, walls.AppendChild("East"), (half_w + BORDER_MARGIN, wall_y, 0),
          (WALL_THICKNESS, WALL_HEIGHT, floor_d), mats["wall"], collision=True)
    west_x = -half_w - BORDER_MARGIN
    west_segment_length = half_d + BORDER_MARGIN - AISLE_WIDTH * 0.5
    west_segment_center = (half_d + BORDER_MARGIN + AISLE_WIDTH * 0.5) * 0.5
    for name, z in (("WestNorth", west_segment_center), ("WestSouth", -west_segment_center)):
        _cube(stage, walls.AppendChild(name), (west_x, wall_y, z),
              (WALL_THICKNESS, WALL_HEIGHT, west_segment_length), mats["wall"], collision=True)
    _cube(stage, walls.AppendChild("EntranceLintel"),
          (west_x, WALL_HEIGHT - 0.38, 0.0),
          (WALL_THICKNESS, 0.76, AISLE_WIDTH), mats["wall"], collision=True)

    # 운전자가 주차장 밖에서 내리고 차량을 로봇에 인계하는 2열×3칸 대기장.
    handoff = UsdGeom.Xform.Define(stage, environment.AppendChild("VehicleHandoffArea")).GetPrim()
    handoff.CreateAttribute("parking:serviceRole", Sdf.ValueTypeNames.String).Set(
        "driver drop-off and robot vehicle handoff"
    )
    handoff.CreateAttribute("parking:capacity", Sdf.ValueTypeNames.Int).Set(6)
    handoff_center_x = west_x - HANDOFF_LENGTH * 0.5
    handoff_floor = _cube(
        stage, handoff.GetPath().AppendChild("Floor"),
        (handoff_center_x, -0.055, 0.0),
        (HANDOFF_LENGTH, 0.11, HANDOFF_WIDTH), mats["asphalt"], collision=True,
    )
    UsdShade.MaterialBindingAPI.Apply(handoff_floor.GetPrim()).Bind(
        physics_mat, UsdShade.Tokens.weakerThanDescendants, "physics"
    )
    add_collision_to_collision_group(
        stage, handoff_floor.GetPath(), Sdf.Path("/World/VehiclePhysics/GroundGroup")
    )
    _outline(
        stage, handoff.GetPath().AppendChild("Boundary"), handoff_center_x, 0.0,
        HANDOFF_LENGTH - 0.20, HANDOFF_WIDTH - 0.20, mats["green"], 0.12,
    )
    handoff.CreateAttribute("parking:displayLabel", Sdf.ValueTypeNames.String).Set(
        "DRIVER DROP-OFF / VEHICLE HANDOFF"
    )

    handoff_poses = {}
    for queue_index, (label, _vehicle_type, _color_name) in enumerate(HANDOFF_VEHICLES):
        column = queue_index // 2
        lane = queue_index % 2
        x = west_x - 3.75 - column * 7.05
        z = 2.35 if lane == 0 else -2.35
        handoff_poses[label] = (x, z)
        bay = UsdGeom.Xform.Define(stage, handoff.GetPath().AppendChild(label)).GetPrim()
        bay.CreateAttribute("parking:queueOrder", Sdf.ValueTypeNames.Int).Set(queue_index + 1)
        bay.CreateAttribute("parking:center", Sdf.ValueTypeNames.Float3).Set(Gf.Vec3f(x, 0.0, z))
        bay.CreateAttribute("parking:occupied", Sdf.ValueTypeNames.Bool).Set(True)
        _outline(
            stage, bay.GetPath().AppendChild("Outline"), x, z,
            6.35, 3.75, mats["white"], 0.09,
        )

    pillars = UsdGeom.Xform.Define(stage, environment.AppendChild("Pillars")).GetPath()
    pillar_z = half_d - PILLAR_SIZE * 0.5
    for row_name, z in (("North", pillar_z), ("South", -pillar_z)):
        for index, x in enumerate(
            (-half_w, -half_w + 2 * SPACE_WIDTH, -half_w + 4 * SPACE_WIDTH,
             -half_w + 6 * SPACE_WIDTH, -half_w + 8 * SPACE_WIDTH, half_w)
        ):
            _cube(stage, pillars.AppendChild(f"{row_name}_{index}"),
                  (x, WALL_HEIGHT * 0.5, z),
                  (PILLAR_SIZE, WALL_HEIGHT, PILLAR_SIZE), mats["pillar"], collision=True)

    vehicles_root = UsdGeom.Xform.Define(stage, "/World/ParkingVehicles").GetPath()
    parked_root = UsdGeom.Xform.Define(stage, vehicles_root.AppendChild("Parked")).GetPath()
    for label, vehicle_type, color_name, workflow_state in PARKED_VEHICLES:
        row_name, index = label[0], int(label[1:])
        x = -half_w + (index + 0.5) * SPACE_WIDTH
        z = row_center if row_name == "A" else -row_center
        yaw = 180.0 if row_name == "A" else 0.0
        vehicle = _vehicle_instance(
            stage, parked_root.AppendChild(f"{label}_{vehicle_type}"),
            vehicle_type, color_name, (x, 0.035, z), yaw,
            workflow_state,
        )
        vehicle.CreateAttribute("parking:space", Sdf.ValueTypeNames.String).Set(label)
        spot = stage.GetPrimAtPath(spaces_root.AppendChild(label))
        spot.GetAttribute("parking:occupied").Set(True)
        spot.CreateAttribute("parking:vehiclePath", Sdf.ValueTypeNames.String).Set(
            str(vehicle.GetPath())
        )
        spot.CreateAttribute("parking:arrivalMode", Sdf.ValueTypeNames.Token).Set(
            "driver" if workflow_state == "driver_parked" else "parking_robot"
        )

    waiting_root = UsdGeom.Xform.Define(stage, vehicles_root.AppendChild("HandoffQueue")).GetPath()
    for queue_index, (label, vehicle_type, color_name) in enumerate(HANDOFF_VEHICLES):
        x, z = handoff_poses[label]
        vehicle = _vehicle_instance(
            stage, waiting_root.AppendChild(f"{label}_{vehicle_type}"),
            vehicle_type, color_name, (x, 0.035, z), 90.0,
            "driver_dropoff_waiting",
        )
        vehicle.CreateAttribute("parking:handoffBay", Sdf.ValueTypeNames.String).Set(label)
        vehicle.CreateAttribute("parking:queueOrder", Sdf.ValueTypeNames.Int).Set(queue_index + 1)

    ceiling = _cube(
        stage, environment.AppendChild("Ceiling"),
        (0.0, CEILING_HEIGHT + CEILING_THICKNESS * 0.5, 0.0),
        (floor_w, CEILING_THICKNESS, floor_d), mats["ceiling"], collision=True,
    )
    ceiling.GetPrim().SetCustomDataByKey("hideForTopView", True)

    # 내비게이션 좌표는 속성으로만 남기고 기존 녹색/주황 점선, 시작 패드,
    # A5 주황 강조 테두리는 표시하지 않는다.
    guides = UsdGeom.Xform.Define(stage, "/World/Guides").GetPrim()
    guides.CreateAttribute("guide:visibleRoutes", Sdf.ValueTypeNames.Bool).Set(False)
    robot_x = -half_w + SPACE_WIDTH * 0.5
    robot_z = row_center
    pickup_x = -half_w + SPACE_WIDTH * 3.0
    target_x = -half_w + (TARGET_INDEX + 0.5) * SPACE_WIDTH
    target_z = row_center

    nav = UsdGeom.Scope.Define(stage, "/World/Navigation").GetPrim()
    nav.CreateAttribute("robot:start", Sdf.ValueTypeNames.Float3).Set(Gf.Vec3f(robot_x, 0, robot_z))
    nav.CreateAttribute("vehicle:pickup", Sdf.ValueTypeNames.Float3).Set(Gf.Vec3f(pickup_x, 0, 0))
    nav.CreateAttribute("parking:target", Sdf.ValueTypeNames.Float3).Set(Gf.Vec3f(target_x, 0, target_z))
    nav.CreateAttribute("parking:targetName", Sdf.ValueTypeNames.String).Set("A5")
    first_handoff_x, first_handoff_z = handoff_poses["H1"]
    nav.CreateAttribute("vehicle:handoff", Sdf.ValueTypeNames.Float3).Set(
        Gf.Vec3f(first_handoff_x, 0.0, first_handoff_z)
    )
    nav.CreateAttribute("vehicle:handoffQueue", Sdf.ValueTypeNames.Float3Array).Set(
        [Gf.Vec3f(handoff_poses[f"H{i}"][0], 0.0, handoff_poses[f"H{i}"][1]) for i in range(1, 7)]
    )

    lights = UsdGeom.Xform.Define(stage, "/World/Lighting").GetPath()
    dome = UsdLux.DomeLight.Define(stage, lights.AppendChild("Dome"))
    dome.CreateIntensityAttr(80.0)
    dome.CreateColorAttr(Gf.Vec3f(0.68, 0.76, 0.90))
    light_positions = [
        (-half_w * 0.62, -row_center * 0.58),
        (0.0, -row_center * 0.58),
        (half_w * 0.62, -row_center * 0.58),
        (-half_w * 0.62, row_center * 0.58),
        (0.0, row_center * 0.58),
        (half_w * 0.62, row_center * 0.58),
    ]
    for index, (x, z) in enumerate(light_positions):
        fixture_path = lights.AppendChild(f"CeilingFixture_{index}")
        _cube(stage, fixture_path.AppendChild("Housing"),
              (x, CEILING_HEIGHT - 0.09, z), (3.20, 0.12, 0.42), mats["dark"])
        _cube(stage, fixture_path.AppendChild("Panel"),
              (x, CEILING_HEIGHT - 0.16, z), (2.85, 0.035, 0.30), mats["light"])
        light = UsdLux.SphereLight.Define(stage, fixture_path.AppendChild("Light"))
        light.CreateRadiusAttr(1.8)
        light.CreateIntensityAttr(9500.0)
        light.CreateColorAttr(colors["light"])
        light.AddTranslateOp().Set(Gf.Vec3f(x, CEILING_HEIGHT - 0.28, z))

    sensors = UsdGeom.Xform.Define(stage, "/World/Sensors").GetPath()
    lidar_mount_x = half_w * LIDAR_X_FRACTION
    for zone_name, x in (("West", -lidar_mount_x), ("East", lidar_mount_x)):
        mount = UsdGeom.Xform.Define(
            stage, sensors.AppendChild(f"CeilingLidar{zone_name}Mount")
        ).GetPrim()
        mount.CreateAttribute("sensor:role", Sdf.ValueTypeNames.String).Set(
            "parking occupancy and robot tracking"
        )
        mount.CreateAttribute("sensor:coverageZone", Sdf.ValueTypeNames.Token).Set(zone_name.lower())
        _cube(stage, mount.GetPath().AppendChild("Bracket"),
              (x, CEILING_HEIGHT - 0.14, 0.0), (0.62, 0.26, 0.62), mats["dark"])
        _cylinder(stage, mount.GetPath().AppendChild("SafetyRing"),
                  (x, CEILING_HEIGHT - 0.34, 0.0), 0.31, 0.16, mats["yellow"])

    camera = UsdGeom.Camera.Define(stage, "/World/OverviewCamera")
    camera.CreateFocalLengthAttr(20.0)
    camera.AddTranslateOp().Set(Gf.Vec3d(-half_w + 1.4, 2.25, 0.0))
    camera.AddRotateXYZOp().Set(Gf.Vec3f(0.0, -90.0, 0.0))

    stage.GetRootLayer().documentation = (
        "Underground parking environment: 16 spaces (A1-A8/B1-B8), "
        "A1/A2 accessible, four robot wait/charge docks, 5.6 m walls and ceiling, "
        "two ceiling-mounted RTX Lidars, 12 initial fab vehicles and external handoff queue."
    )
    stage.GetRootLayer().Save()
    return {
        "floor": (floor_w, floor_d),
        "robot": (robot_x, robot_z),
        "pickup": (pickup_x, 0.0),
        "target": (target_x, target_z),
    }


def add_ceiling_lidars():
    """현재 Omni USD stage에 서쪽/동쪽 실제 RTX LiDAR 두 대를 생성한다."""
    import math
    import omni.kit.commands
    import omni.usd
    from pxr import Gf, Sdf

    half_w = SPACE_COUNT * SPACE_WIDTH * 0.5
    lidar_x = half_w * LIDAR_X_FRACTION
    sensor_paths = []
    for zone_name, x in (("West", -lidar_x), ("East", lidar_x)):
        path = f"/World/Sensors/CeilingLidar{zone_name}"
        _, sensor = omni.kit.commands.execute(
            "IsaacSensorCreateRtxLidar",
            path=path,
            parent=None,
            config=LIDAR_CONFIG,
            translation=Gf.Vec3d(x, CEILING_HEIGHT - 0.48, 0.0),
            # RTX LiDAR의 로컬 +Z 회전축을 천장 아래쪽(-Y)으로 향하게 한다.
            orientation=Gf.Quatd(math.cos(math.pi / 4), math.sin(math.pi / 4), 0.0, 0.0),
            visibility=True,
        )
        if sensor is None or not sensor.IsValid():
            raise RuntimeError(f"RTX LiDAR 생성 실패: {path} / {LIDAR_CONFIG}")
        sensor.CreateAttribute("sensor:mount", Sdf.ValueTypeNames.String).Set("ceiling-downward")
        sensor.CreateAttribute("sensor:coverageZone", Sdf.ValueTypeNames.Token).Set(zone_name.lower())
        sensor.CreateAttribute("sensor:purpose", Sdf.ValueTypeNames.String).Set(
            "parking occupancy and robot localization"
        )
        sensor.CreateAttribute("sensor:enabledByDefault", Sdf.ValueTypeNames.Bool).Set(True)
        sensor_paths.append(str(sensor.GetPath()))
    stage = omni.usd.get_context().get_stage()
    stage.GetRootLayer().Save()
    print(f"[parking] RTX LiDAR 2대 생성: {sensor_paths} ({LIDAR_CONFIG})", flush=True)
    return sensor_paths


def verify_stage():
    from pxr import PhysxSchema, Usd, UsdGeom, UsdPhysics, UsdShade

    stage = Usd.Stage.Open(str(OUTPUT_USD))
    if stage is None:
        raise RuntimeError("생성한 주차장 USD를 다시 열 수 없습니다.")
    if UsdGeom.GetStageUpAxis(stage) != UsdGeom.Tokens.y:
        raise RuntimeError("차량과 주차장의 up axis가 일치하지 않습니다.")
    spaces = stage.GetPrimAtPath("/World/ParkingEnvironment/Spaces")
    expected_names = {f"{row}{index}" for row in ("A", "B") for index in PARKING_INDICES}
    actual_names = {prim.GetName() for prim in spaces.GetChildren()} if spaces else set()
    if actual_names != expected_names:
        raise RuntimeError(f"주차 구획 A1-A8/B1-B8 검증 실패: {sorted(actual_names)}")
    for label in ACCESSIBLE_SPACES:
        prim = stage.GetPrimAtPath(f"/World/ParkingEnvironment/Spaces/{label}")
        if not prim.GetAttribute("parking:accessible").Get():
            raise RuntimeError(f"교통약자 주차면 속성 누락: {label}")
    for label in sorted(expected_names - ACCESSIBLE_SPACES):
        label_prim = stage.GetPrimAtPath(
            f"/World/ParkingEnvironment/Spaces/{label}/Label"
        )
        if (
            not label_prim
            or label_prim.GetAttribute("parking:markingType").Get()
            != "flat_floor_paint"
        ):
            raise RuntimeError(f"평면 바닥 도색 라벨 속성 누락: {label}")
        paint_prims = list(label_prim.GetChildren())
        if not paint_prims or any(not prim.IsA(UsdGeom.Mesh) for prim in paint_prims):
            raise RuntimeError(f"라벨이 평면 Mesh 도색으로 생성되지 않았습니다: {label}")
        if any(prim.HasAPI(UsdPhysics.CollisionAPI) for prim in paint_prims):
            raise RuntimeError(f"라벨에 불필요한 충돌체가 있습니다: {label}")
    for label in ACCESSIBLE_SPACES:
        if stage.GetPrimAtPath(
            f"/World/ParkingEnvironment/Spaces/{label}/Label"
        ).IsValid():
            raise RuntimeError(f"교통약자 주차면의 중복 번호 도색이 남아 있습니다: {label}")
    marking_tokens = (
        "/Markings/", "/Label/", "/AccessibleSymbol/", "/FloorOutline/",
        "/Boundary/", "/Outline/",
    )
    flat_markings = []
    for prim in stage.Traverse():
        prim_path = str(prim.GetPath())
        is_flat_marking = (
            any(token in prim_path for token in marking_tokens)
            or prim.GetName() in {"BlueFloor", "DockPad"}
        )
        if not is_flat_marking or not (
            prim.IsA(UsdGeom.Mesh) or prim.IsA(UsdGeom.Cube)
            or prim.IsA(UsdGeom.BasisCurves)
        ):
            continue
        flat_markings.append(prim)
        if prim.IsA(UsdGeom.BasisCurves):
            raise RuntimeError(f"입체 Curve 바닥 마킹이 남아 있습니다: {prim_path}")
        if prim.HasAPI(UsdPhysics.CollisionAPI):
            raise RuntimeError(f"바닥 마킹에 불필요한 충돌체가 있습니다: {prim_path}")
    if not flat_markings:
        raise RuntimeError("검증할 평면 바닥 마킹을 찾지 못했습니다.")
    bbox_cache = UsdGeom.BBoxCache(
        Usd.TimeCode.Default(),
        [UsdGeom.Tokens.default_, UsdGeom.Tokens.render],
    )
    for prim in flat_markings:
        max_height = bbox_cache.ComputeWorldBound(prim).ComputeAlignedRange().GetMax()[1]
        if max_height > 0.0011:
            raise RuntimeError(
                f"바닥 마킹 높이가 1.1 mm를 초과합니다: {prim.GetPath()} / {max_height:.6f} m"
            )
    floor = stage.GetPrimAtPath("/World/ParkingEnvironment/Floor")
    if not floor.HasAPI(UsdPhysics.CollisionAPI):
        raise RuntimeError("주차장 바닥 충돌체가 없습니다.")
    for prim in stage.Traverse():
        if "/Markings/" in str(prim.GetPath()) and prim.HasAPI(UsdPhysics.CollisionAPI):
            raise RuntimeError(f"차선에 불필요한 충돌체가 있습니다: {prim.GetPath()}")
    if not stage.GetPrimAtPath("/World/Navigation").IsValid():
        raise RuntimeError("내비게이션 좌표가 없습니다.")
    ceiling = stage.GetPrimAtPath("/World/ParkingEnvironment/Ceiling")
    if not ceiling or not ceiling.HasAPI(UsdPhysics.CollisionAPI):
        raise RuntimeError("지하주차장 천장 충돌체가 없습니다.")
    service = stage.GetPrimAtPath("/World/ParkingEnvironment/RobotServiceArea")
    if not service or len(list(service.GetChildren())) != 4:
        raise RuntimeError("로봇 대기/충전 도킹 스테이션 4개가 없습니다.")
    lidar_prims = [
        prim for prim in stage.Traverse()
        if prim.GetTypeName() == "OmniLidar" and str(prim.GetPath()).startswith("/World/Sensors/")
    ]
    if len(lidar_prims) != 2 or not all(
        prim.HasAPI("OmniSensorGenericLidarCoreAPI") for prim in lidar_prims
    ):
        raise RuntimeError("실제 동작 가능한 RTX LiDAR 두 대의 Prim 검증에 실패했습니다.")
    parked = stage.GetPrimAtPath("/World/ParkingVehicles/Parked")
    waiting = stage.GetPrimAtPath("/World/ParkingVehicles/HandoffQueue")
    if len(list(parked.GetChildren())) != 6 or len(list(waiting.GetChildren())) != 6:
        raise RuntimeError("초기 차량 12대(내부 6/외부 6) 배치 검증에 실패했습니다.")
    vehicles = [*parked.GetChildren(), *waiting.GetChildren()]
    placed_types = {
        str(vehicle.GetAttribute("parking:vehicleType").Get()) for vehicle in vehicles
    }
    if placed_types != FAB_VEHICLE_TYPES:
        raise RuntimeError(
            f"FAB 고유 차종 10종이 모두 배치되지 않았습니다: {sorted(placed_types)}"
        )
    for vehicle in vehicles:
        if (
            not vehicle.HasAPI(UsdPhysics.RigidBodyAPI)
            or not vehicle.HasAPI(UsdPhysics.MassAPI)
            or not vehicle.HasAPI(PhysxSchema.PhysxVehicleAPI)
        ):
            raise RuntimeError(f"FAB 차량 물리 API가 보존되지 않았습니다: {vehicle.GetPath()}")
        if bool(UsdPhysics.RigidBodyAPI(vehicle).GetKinematicEnabledAttr().Get()):
            raise RuntimeError(f"차량이 kinematic으로 고정되어 있습니다: {vehicle.GetPath()}")
        wheels = [
            child for child in vehicle.GetChildren()
            if child.HasAPI(PhysxSchema.PhysxVehicleWheelAttachmentAPI)
        ]
        if len(wheels) != 4:
            raise RuntimeError(f"FAB 차량 바퀴 물리 4개가 보존되지 않았습니다: {vehicle.GetPath()}")
        body_materials = []
        for prim in Usd.PrimRange(vehicle):
            if not prim.IsA(UsdGeom.Subset) or "body" not in prim.GetName().lower():
                continue
            material = UsdShade.MaterialBindingAPI(prim).ComputeBoundMaterial()[0]
            if material:
                body_materials.append(str(material.GetPath()))
        if not any(path.startswith("/World/Looks/FabColors/") for path in body_materials):
            raise RuntimeError(f"FAB 원본 차체 재질이 보존되지 않았습니다: {vehicle.GetPath()}")
    parked_colors = [str(prim.GetAttribute("parking:bodyColor").Get()) for prim in parked.GetChildren()]
    if parked_colors.count("white") != 3 or parked_colors.count("black") != 3:
        raise RuntimeError(f"내부 차량 흰색3/검정3 배치 오류: {parked_colors}")
    parked_modes = [str(prim.GetAttribute("parking:workflowState").Get()) for prim in parked.GetChildren()]
    if parked_modes.count("driver_parked") != 2 or parked_modes.count("robot_parked") != 4:
        raise RuntimeError(f"운전자2/로봇4 주차 상태 오류: {parked_modes}")
    occupied = {
        prim.GetName() for prim in spaces.GetChildren()
        if bool(prim.GetAttribute("parking:occupied").Get())
    }
    expected_occupied = {item[0] for item in PARKED_VEHICLES}
    if occupied != expected_occupied:
        raise RuntimeError(f"초기 점유 주차면 오류: {sorted(occupied)}")
    guides = stage.GetPrimAtPath("/World/Guides")
    forbidden_guides = {"CarryPath_Green", "ParkAndExitPath_Orange", "Target_A5"}
    if any(stage.GetPrimAtPath(guides.GetPath().AppendChild(name)) for name in forbidden_guides):
        raise RuntimeError("삭제 대상 컬러 경로 또는 A5 강조가 남아 있습니다.")
    handoff = stage.GetPrimAtPath("/World/ParkingEnvironment/VehicleHandoffArea/Floor")
    if not handoff or not handoff.HasAPI(UsdPhysics.CollisionAPI):
        raise RuntimeError("외부 차량 인계 대기장 바닥 충돌체가 없습니다.")
    return stage


def main():
    _restart_with_isaac_python()
    from isaacsim import SimulationApp

    headless = "--headless" in sys.argv[1:]
    app = SimulationApp({
        "headless": headless,
        "disable_viewport_updates": headless,
    })
    try:
        import omni.usd

        report = build_stage()
        # Isaac Sim 5.1의 동기 open_stage()는 성공해도 None을 반환한다.
        omni.usd.get_context().open_stage(str(OUTPUT_USD))
        for _ in range(8):
            app.update()
        lidar_paths = add_ceiling_lidars()
        for _ in range(8):
            app.update()
        verify_stage()
        print("\n[parking] 생성 및 검증 완료", flush=True)
        print(f"[parking] 주차면: 16개(A1-A8/B1-B8), {SPACE_WIDTH:.2f} x {SPACE_LENGTH:.2f} m", flush=True)
        print("[parking] 교통약자 주차면: A1, A2", flush=True)
        print("[parking] 로봇 대기/충전 도크: 4개", flush=True)
        print(f"[parking] 지하 구조: 벽/기둥/천장 높이 {CEILING_HEIGHT:.2f} m", flush=True)
        print(f"[parking] 천장 RTX LiDAR 2대: {lidar_paths}", flush=True)
        print(f"[parking] 중앙 통로: {AISLE_WIDTH:.2f} m", flush=True)
        print(f"[parking] 전체 바닥: {report['floor'][0]:.2f} x {report['floor'][1]:.2f} m", flush=True)
        print("[parking] 컬러 경로 및 A5 강조색: 제거", flush=True)
        print("[parking] 초기 차량: 내부 6대(흰색3/검정3), 외부 인계대기 6대", flush=True)
        print(f"[parking] 최종 파일: {OUTPUT_USD}", flush=True)

        if headless:
            # 모든 레이어 저장과 재검증이 끝난 생성 전용 프로세스다. 현재
            # Isaac Sim/Vulkan 조합의 shutdown 대기 및 VRAM 누적을 피한다.
            os._exit(0)

        if not headless:
            from isaacsim.core.utils.viewports import set_active_viewport_camera

            set_active_viewport_camera("/World/OverviewCamera")
            for _ in range(4):
                app.update()
            print("[parking] Isaac Sim 창을 닫으면 프로그램이 종료됩니다.", flush=True)
            while app.is_running():
                app.update()
    finally:
        app.close()


if __name__ == "__main__":
    main()
