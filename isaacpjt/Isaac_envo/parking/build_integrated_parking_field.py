#!/usr/bin/env python3
"""현재 주차장+차량 12대+HWIA 주차로봇을 한 Stage로 합성한다.

GUI 확인:
    python3 build_integrated_parking_field.py

자동 구성/물리 검증:
    python3 build_integrated_parking_field.py --headless-test
"""

from __future__ import annotations

import argparse
import math
from pathlib import Path

from isaac_runtime import restart_with_isaac_python


ROOT = Path(__file__).resolve().parent
PARKING_USD = ROOT / "parking_environment.usd"
# ROOT는 Isaac_envo/parking 이므로 로봇 패키지는 두 단계 위(isaacpjt)에 있다.
# 이전 코드는 한 단계만 올라가 LOCAL 경로가 항상 존재하지 않았고, 그 결과
# "로컬본 우선" 폴백이 사실상 에셋 종류를 바꿔치기하는 분기로 동작했다.
PROJECT_ROOT = ROOT.parent.parent
ROBOT_PACKAGE = PROJECT_ROOT / "hwia_parking_robot_final_caster_package"
# 이 프로젝트의 최종 로봇은 메카넘이다. 폴백을 두지 않는다 — 없으면 즉시 실패해야
# 조용히 다른 로봇으로 굴러가는 일이 없다.
ROBOT_USD = ROBOT_PACKAGE / "hwia_parking_robot_final_caster_mecha_roller.usd"
# OUTPUT_USD가 Isaac_envo/parking/ 에 있으므로 두 단계 올라간다.
ROBOT_REF = f"../../{ROBOT_PACKAGE.name}/{ROBOT_USD.name}"
OUTPUT_USD = ROOT / "parking_robot_field.usd"
def build_stage() -> tuple[float, float]:
    from pxr import Gf, Sdf, Usd, UsdGeom

    for asset in (PARKING_USD, ROBOT_USD):
        if not asset.is_file():
            raise FileNotFoundError(asset)
    if OUTPUT_USD.exists():
        OUTPUT_USD.unlink()

    stage = Usd.Stage.CreateNew(str(OUTPUT_USD))
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.y)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    stage.SetTimeCodesPerSecond(60.0)

    # 주차장, 차량 12대, 조명, PhysicsScene, 천장 LiDAR를 원형 그대로 유지한다.
    stage.GetRootLayer().subLayerPaths.append("./parking_environment.usd")
    world = stage.GetPrimAtPath("/World")
    if not world:
        raise RuntimeError("주차장 서브레이어에서 /World를 찾지 못했습니다.")
    stage.SetDefaultPrim(world)

    nav = stage.GetPrimAtPath("/World/Navigation")
    robot_start = nav.GetAttribute("robot:start").Get() if nav else None
    if robot_start is None:
        raise RuntimeError("주차장 Navigation의 robot:start 좌표가 없습니다.")
    start_x, _, start_z = (float(value) for value in robot_start)

    robot = stage.DefinePrim("/World/HwiaParkingRobot", "Xform")
    # 프로젝트 내부에서는 상대경로를 써서 Isaac_envo 폴더를 옮겨도 유지된다.
    robot.GetReferences().AddReference(ROBOT_REF)
    robot_xform = UsdGeom.Xformable(robot)
    robot_xform.ClearXformOpOrder()
    robot_xform.AddTranslateOp().Set(Gf.Vec3d(start_x, 0.0, start_z))
    # 로봇 로컬 +Z(up)를 주차장 world +Y(up)로 변환한다.
    robot_xform.AddRotateXOp().Set(-90.0)
    robot.CreateAttribute("parkingRobot:asset", Sdf.ValueTypeNames.Asset).Set(
        Sdf.AssetPath(str(ROBOT_USD))
    )
    robot.CreateAttribute("parkingRobot:axisAdapter", Sdf.ValueTypeNames.String).Set(
        "robot Z-up -> parking Y-up; rotateX=-90deg"
    )
    robot.CreateAttribute("parkingRobot:spawnArea", Sdf.ValueTypeNames.String).Set(
        "West A-row waiting bay"
    )
    robot.CreateAttribute("parkingRobot:rosRequired", Sdf.ValueTypeNames.Bool).Set(False)

    camera = UsdGeom.Camera.Define(stage, "/World/HwiaParkingFieldCamera")
    camera.CreateFocalLengthAttr(25.0)
    eye = Gf.Vec3d(start_x + 8.0, 4.15, start_z - 7.2)
    target = Gf.Vec3d(start_x, 0.12, start_z)
    camera.AddTransformOp().Set(
        Gf.Matrix4d().SetLookAt(eye, target, Gf.Vec3d(0.0, 1.0, 0.0)).GetInverse()
    )

    stage.GetRootLayer().documentation = (
        "Integrated test field: current parking_environment.usd with 12 placed "
        "vehicles, two ceiling RTX Lidars, and one HWIA final-caster parking robot."
    )
    stage.GetRootLayer().Save()
    return start_x, start_z


def find_robot_rigid_body(stage) -> str:
    from pxr import UsdPhysics

    robot_prefix = "/World/HwiaParkingRobot/"
    paths = [
        str(prim.GetPath())
        for prim in stage.Traverse()
        if str(prim.GetPath()).startswith(robot_prefix)
        and prim.HasAPI(UsdPhysics.RigidBodyAPI)
    ]
    if not paths:
        raise RuntimeError("HWIA 로봇 reference 안에서 RigidBody를 찾지 못했습니다.")
    # 이름이 base_link인 몸체를 우선 사용하고, 패키지 구조가 달라지면 첫 몸체를 쓴다.
    return next((path for path in paths if path.endswith("/base_link")), paths[0])


def verify_composition(stage) -> dict[str, int]:
    from pxr import PhysxSchema, UsdGeom, UsdPhysics

    if UsdGeom.GetStageUpAxis(stage) != UsdGeom.Tokens.y:
        raise RuntimeError("통합 Stage가 Y-up이 아닙니다.")
    parked = stage.GetPrimAtPath("/World/ParkingVehicles/Parked")
    waiting = stage.GetPrimAtPath("/World/ParkingVehicles/HandoffQueue")
    robot = stage.GetPrimAtPath("/World/HwiaParkingRobot")
    base_link = stage.GetPrimAtPath("/World/HwiaParkingRobot/base_link")
    if len(list(parked.GetChildren())) != 6:
        raise RuntimeError("내부 주차 차량 6대가 유지되지 않았습니다.")
    if len(list(waiting.GetChildren())) != 6:
        raise RuntimeError("외부 인계 대기 차량 6대가 유지되지 않았습니다.")
    vehicles = [*parked.GetChildren(), *waiting.GetChildren()]
    for vehicle in vehicles:
        if (
            not vehicle.HasAPI(UsdPhysics.RigidBodyAPI)
            or not vehicle.HasAPI(UsdPhysics.MassAPI)
            or not vehicle.HasAPI(PhysxSchema.PhysxVehicleAPI)
        ):
            raise RuntimeError(f"FAB 차량 물리 구동계가 누락됐습니다: {vehicle.GetPath()}")
        wheels = [
            child for child in vehicle.GetChildren()
            if child.HasAPI(PhysxSchema.PhysxVehicleWheelAttachmentAPI)
        ]
        if len(wheels) != 4:
            raise RuntimeError(f"FAB 차량 바퀴 물리가 4개가 아닙니다: {vehicle.GetPath()}")
    if not robot or not base_link:
        raise RuntimeError("HWIA 주차로봇 reference가 구성되지 않았습니다.")
    robot_body = find_robot_rigid_body(stage)
    lidar_count = sum(
        1
        for prim in stage.Traverse()
        if prim.GetTypeName() == "OmniLidar"
        and str(prim.GetPath()).startswith("/World/Sensors/")
    )
    if lidar_count != 2:
        raise RuntimeError(f"천장 RTX LiDAR 2대가 유지되지 않았습니다: {lidar_count}")
    if stage.GetPrimAtPath("/World/TestVehicleLibrary").IsValid():
        raise RuntimeError("별도 시험용 차량 라이브러리가 통합 필드에 남아 있습니다.")
    print(f"[field] HWIA 물리 몸체: {robot_body}", flush=True)
    return {
        "parked": 6,
        "waiting": 6,
        "fabPhysicsVehicles": len(vehicles),
        "robot": 1,
        "lidars": lidar_count,
    }


def world_position(stage, path: str) -> tuple[float, float, float]:
    from pxr import Usd, UsdGeom

    matrix = UsdGeom.Xformable(stage.GetPrimAtPath(path)).ComputeLocalToWorldTransform(
        Usd.TimeCode.Default()
    )
    return tuple(float(value) for value in matrix.ExtractTranslation())


def main() -> None:
    restart_with_isaac_python(Path(__file__))
    parser = argparse.ArgumentParser()
    parser.add_argument("--headless-test", action="store_true")
    args = parser.parse_args()

    from isaacsim import SimulationApp

    app = SimulationApp(
        {
            "headless": args.headless_test,
            "width": 1200,
            "height": 760,
            "enable_motion_bvh": True,
        }
    )
    try:
        import omni.timeline
        import omni.usd
        from isaacsim.core.api import World

        start = build_stage()
        context = omni.usd.get_context()
        # Isaac Sim 5.1의 동기 open_stage()는 성공해도 None을 반환한다.
        context.open_stage(str(OUTPUT_USD))
        for _ in range(30):
            app.update()
        stage = context.get_stage()
        report = verify_composition(stage)
        print(f"[field] 구성 검증 통과: {report}", flush=True)
        print(f"[field] HWIA 로봇 시작 좌표 X/Z: {start}", flush=True)

        if args.headless_test:
            world = World(stage_units_in_meters=1.0, set_defaults=False)
            robot_body = find_robot_rigid_body(stage)
            before = world_position(stage, robot_body)
            vehicle_paths = [
                str(vehicle.GetPath())
                for root_path in (
                    "/World/ParkingVehicles/Parked",
                    "/World/ParkingVehicles/HandoffQueue",
                )
                for vehicle in stage.GetPrimAtPath(root_path).GetChildren()
            ]
            vehicle_before = {
                path: world_position(stage, path) for path in vehicle_paths
            }
            timeline = omni.timeline.get_timeline_interface()
            timeline.play()
            world.reset()
            for _ in range(180):
                world.step(render=False)
            timeline.pause()
            after = world_position(stage, robot_body)
            if not all(math.isfinite(value) for value in after):
                raise RuntimeError(f"로봇 위치가 비정상입니다: {after}")
            displacement = math.dist(before, after)
            if displacement > 0.35:
                raise RuntimeError(
                    f"로봇 초기 물리 변위가 과도합니다: {displacement:.4f} m"
                )
            vehicle_displacements = {
                path: math.dist(vehicle_before[path], world_position(stage, path))
                for path in vehicle_paths
            }
            max_vehicle_path = max(vehicle_displacements, key=vehicle_displacements.get)
            max_vehicle_displacement = vehicle_displacements[max_vehicle_path]
            if not math.isfinite(max_vehicle_displacement) or max_vehicle_displacement > 0.75:
                raise RuntimeError(
                    "FAB 차량 초기 물리 변위가 과도합니다: "
                    f"{max_vehicle_path} / {max_vehicle_displacement:.4f} m"
                )
            print(
                f"[field] 180 frame 물리 안정성 통과: {before} -> {after} "
                f"(변위 {displacement:.4f} m)",
                flush=True,
            )
            print(
                "[field] FAB 차량 12대 물리 안정성 통과: "
                f"최대 변위 {max_vehicle_displacement:.4f} m ({max_vehicle_path})",
                flush=True,
            )
            return

        from isaacsim.core.utils.viewports import set_active_viewport_camera

        set_active_viewport_camera("/World/HwiaParkingFieldCamera")
        for _ in range(8):
            app.update()
        print("[field] Play 버튼으로 물리를 시작할 수 있습니다.", flush=True)
        print("[field] Isaac Sim 창을 닫으면 프로그램이 종료됩니다.", flush=True)
        while app.is_running():
            app.update()
    except Exception as exc:
        # SimulationApp.close()가 일부 Kit 버전에서 traceback 출력을 삼키므로
        # 종료 전에 원인을 확실히 남긴다.
        print(f"[field] ERROR: {type(exc).__name__}: {exc}", flush=True)
        raise
    finally:
        app.close(wait_for_replicator=False)


if __name__ == "__main__":
    main()
