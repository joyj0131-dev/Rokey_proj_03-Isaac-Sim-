#!/usr/bin/env python3
"""현재 주차장을 유지한 채 HWIA 주차로봇 2대를 대기 도크에 합성한다.

GUI 확인:
    python3 build_dual_robot_parking_field.py

자동 구성/물리 검증:
    python3 build_dual_robot_parking_field.py --headless-test

이 스크립트는 ROS 2 통신을 만들지 않는다. 먼저 실제 로봇 에셋 2대가 현재
주차장, 차량, LiDAR와 함께 안정적으로 로드되는지 확인하기 위한 베이스 Stage다.
"""

from __future__ import annotations

import argparse
import math
from pathlib import Path

from isaac_runtime import restart_with_isaac_python


ROOT = Path(__file__).resolve().parent
PARKING_USD = ROOT / "parking_environment.usd"
PROJECT_ROOT = ROOT.parent.parent
ROBOT_PACKAGE = PROJECT_ROOT / "hwia_parking_robot_final_caster_package"
ROBOT_USD = ROBOT_PACKAGE / "hwia_depth_cam_mecha_roller.usd"
ROBOT_REF = f"../../{ROBOT_PACKAGE.name}/{ROBOT_USD.name}"
OUTPUT_USD = ROOT / "parking_robot_field_dual.usd"

ROBOT_SPECS = {
    "robot_1": {
        "dock": "/World/ParkingEnvironment/RobotServiceArea/West_A_WaitingDock",
        "cmd_vel_topic": "/robot_1/cmd_vel",
        "odom_topic": "/robot_1/odom",
    },
    "robot_2": {
        "dock": "/World/ParkingEnvironment/RobotServiceArea/West_B_WaitingDock",
        "cmd_vel_topic": "/robot_2/cmd_vel",
        "odom_topic": "/robot_2/odom",
    },
}


def _dock_position(stage, dock_path: str):
    prim = stage.GetPrimAtPath(dock_path)
    if not prim:
        raise RuntimeError(f"로봇 대기 도크를 찾지 못했습니다: {dock_path}")
    value = prim.GetAttribute("robot:dockPose").Get()
    if value is None:
        raise RuntimeError(f"robot:dockPose 속성이 없습니다: {dock_path}")
    return tuple(float(component) for component in value)


def build_stage() -> dict[str, tuple[float, float, float]]:
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
    stage.GetRootLayer().subLayerPaths.append("./parking_environment.usd")

    world = stage.GetPrimAtPath("/World")
    if not world:
        raise RuntimeError("주차장 서브레이어에서 /World를 찾지 못했습니다.")
    stage.SetDefaultPrim(world)

    robots_scope = UsdGeom.Xform.Define(stage, "/World/Robots")
    spawn_positions = {}
    for robot_id, spec in ROBOT_SPECS.items():
        position = _dock_position(stage, spec["dock"])
        spawn_positions[robot_id] = position

        robot = stage.DefinePrim(f"/World/Robots/{robot_id}", "Xform")
        robot.GetReferences().AddReference(ROBOT_REF)
        xform = UsdGeom.Xformable(robot)
        xform.ClearXformOpOrder()
        xform.AddTranslateOp().Set(Gf.Vec3d(*position))
        # HWIA 에셋의 local Z-up을 주차장의 world Y-up으로 맞춘다.
        # 두 로봇 모두 서쪽 대기 도크에서 주차장 안쪽(+X)을 향한다.
        xform.AddRotateXOp().Set(-90.0)

        robot.CreateAttribute("parkingRobot:id", Sdf.ValueTypeNames.String).Set(robot_id)
        robot.CreateAttribute("parkingRobot:dock", Sdf.ValueTypeNames.String).Set(spec["dock"])
        robot.CreateAttribute("parkingRobot:cmdVelTopic", Sdf.ValueTypeNames.String).Set(
            spec["cmd_vel_topic"]
        )
        robot.CreateAttribute("parkingRobot:odomTopic", Sdf.ValueTypeNames.String).Set(
            spec["odom_topic"]
        )

    camera = UsdGeom.Camera.Define(stage, "/World/DualRobotFieldCamera")
    camera.CreateFocalLengthAttr(24.0)
    # 천장(약 5.6 m) 아래에서 두 서쪽 대기 도크와 중앙 통로를 함께 본다.
    eye = Gf.Vec3d(-23.0, 4.25, 15.0)
    target = Gf.Vec3d(-7.0, 0.0, 0.0)
    camera.AddTransformOp().Set(
        Gf.Matrix4d().SetLookAt(eye, target, Gf.Vec3d(0.0, 1.0, 0.0)).GetInverse()
    )

    stage.GetRootLayer().documentation = (
        "Current parking environment with two independently identified HWIA "
        "parking robots at the West A/B waiting docks."
    )
    stage.GetRootLayer().Save()
    return spawn_positions


def find_robot_rigid_body(stage, robot_id: str) -> str:
    from pxr import UsdPhysics

    prefix = f"/World/Robots/{robot_id}/"
    bodies = [
        str(prim.GetPath())
        for prim in stage.Traverse()
        if str(prim.GetPath()).startswith(prefix)
        and prim.HasAPI(UsdPhysics.RigidBodyAPI)
    ]
    if not bodies:
        raise RuntimeError(f"{robot_id} reference 안에서 RigidBody를 찾지 못했습니다.")
    return next((path for path in bodies if path.endswith("/base_link")), bodies[0])


def verify_stage(stage) -> dict[str, object]:
    from pxr import UsdGeom

    if UsdGeom.GetStageUpAxis(stage) != UsdGeom.Tokens.y:
        raise RuntimeError("통합 Stage가 Y-up이 아닙니다.")

    robot_bodies = {}
    for robot_id, spec in ROBOT_SPECS.items():
        root = stage.GetPrimAtPath(f"/World/Robots/{robot_id}")
        base = stage.GetPrimAtPath(f"/World/Robots/{robot_id}/base_link")
        if not root or not base:
            raise RuntimeError(f"HWIA 로봇 reference가 누락됐습니다: {robot_id}")
        if root.GetAttribute("parkingRobot:id").Get() != robot_id:
            raise RuntimeError(f"로봇 ID 속성이 올바르지 않습니다: {robot_id}")
        if root.GetAttribute("parkingRobot:cmdVelTopic").Get() != spec["cmd_vel_topic"]:
            raise RuntimeError(f"cmd_vel 토픽 속성이 올바르지 않습니다: {robot_id}")
        robot_bodies[robot_id] = find_robot_rigid_body(stage, robot_id)

    parked = stage.GetPrimAtPath("/World/ParkingVehicles/Parked")
    waiting = stage.GetPrimAtPath("/World/ParkingVehicles/HandoffQueue")
    parked_count = len(list(parked.GetChildren()))
    waiting_count = len(list(waiting.GetChildren()))
    lidar_count = sum(
        1
        for prim in stage.Traverse()
        if prim.GetTypeName() == "OmniLidar"
        and str(prim.GetPath()).startswith("/World/Sensors/")
    )
    if parked_count != 6 or waiting_count != 6:
        raise RuntimeError(
            f"기존 차량 구성이 달라졌습니다: parked={parked_count}, waiting={waiting_count}"
        )
    if lidar_count != 2:
        raise RuntimeError(f"천장 RTX LiDAR 구성이 달라졌습니다: {lidar_count}")
    return {
        "robots": robot_bodies,
        "parkedVehicles": parked_count,
        "handoffVehicles": waiting_count,
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
            "width": 1280,
            "height": 800,
            "enable_motion_bvh": True,
        }
    )
    try:
        import omni.timeline
        import omni.usd
        from isaacsim.core.api import World

        spawns = build_stage()
        context = omni.usd.get_context()
        context.open_stage(str(OUTPUT_USD))
        for _ in range(30):
            app.update()
        stage = context.get_stage()
        report = verify_stage(stage)
        print(f"[dual-field] 구성 검증 통과: {report}", flush=True)
        print(f"[dual-field] 로봇 시작 좌표: {spawns}", flush=True)

        if args.headless_test:
            world = World(stage_units_in_meters=1.0, set_defaults=False)
            body_paths = report["robots"]
            before = {
                robot_id: world_position(stage, path)
                for robot_id, path in body_paths.items()
            }
            timeline = omni.timeline.get_timeline_interface()
            timeline.play()
            world.reset()
            for _ in range(180):
                world.step(render=False)
            timeline.pause()
            after = {
                robot_id: world_position(stage, path)
                for robot_id, path in body_paths.items()
            }
            displacements = {
                robot_id: math.dist(before[robot_id], after[robot_id])
                for robot_id in ROBOT_SPECS
            }
            if not all(
                math.isfinite(value) and value <= 0.35
                for value in displacements.values()
            ):
                raise RuntimeError(f"로봇 초기 물리 변위가 과도합니다: {displacements}")
            print(
                f"[dual-field] 180 frame 물리 안정성 통과: {displacements}",
                flush=True,
            )
            print(f"[dual-field] 생성 완료: {OUTPUT_USD}", flush=True)
            return

        from isaacsim.core.utils.viewports import set_active_viewport_camera

        set_active_viewport_camera("/World/DualRobotFieldCamera")
        for _ in range(8):
            app.update()
        print("[dual-field] Play 버튼으로 물리를 시작할 수 있습니다.", flush=True)
        print("[dual-field] Isaac Sim 창을 닫으면 프로그램이 종료됩니다.", flush=True)
        while app.is_running():
            app.update()
    except Exception as exc:
        print(f"[dual-field] ERROR: {type(exc).__name__}: {exc}", flush=True)
        raise
    finally:
        app.close(wait_for_replicator=False)


if __name__ == "__main__":
    main()
