#!/usr/bin/env python3
"""Variant of build_integrated_parking_field.py that drops in the
depth-camera-equipped HWIA robot (see ../../../run_depth_camera_sim.py and
../../../export_camera_robot_asset.py) instead of the team's plain
mecanum-roller robot -- same parking_environment.usd (lot + 12 vehicles),
same axis-adapter/placement logic, different ROBOT_USD. Kept as a separate
script/output file rather than editing build_integrated_parking_field.py in
place, so the team's own composition is untouched.

GUI:
    python3 build_integrated_parking_field_depth_cam.py

Headless build+physics check:
    python3 build_integrated_parking_field_depth_cam.py --headless-test
"""

from __future__ import annotations

import argparse
import math
import os
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parent
PARKING_USD = ROOT / "parking_environment.usd"
# ROOT is Isaac_envo/parking; our camera-robot asset lives at the project root,
# three levels up (parking -> Isaac_envo -> isaacpjt -> project root).
PROJECT_ROOT = ROOT.parent.parent.parent
ROBOT_USD = PROJECT_ROOT / "hwia_parking_robot_final_caster_camera_mesh_depth_cam.usd"
ROBOT_REF = f"../../../{ROBOT_USD.name}"
OUTPUT_USD = ROOT / "parking_robot_field_depth_cam.usd"
ISAAC_PYTHON = Path(
    "/home/rokey/dev_ws/isaac_sim/isaacsim/_build/linux-x86_64/release/python.sh"
)


def restart_with_isaac_python() -> None:
    if os.environ.get("CARB_APP_PATH"):
        return
    if not ISAAC_PYTHON.is_file():
        raise FileNotFoundError(f"Isaac Sim python.sh를 찾을 수 없습니다: {ISAAC_PYTHON}")
    os.execv(
        str(ISAAC_PYTHON),
        [str(ISAAC_PYTHON), str(Path(__file__).resolve()), *sys.argv[1:]],
    )


def build_stage() -> tuple[float, float]:
    from pxr import Gf, Sdf, Usd, UsdGeom

    for asset in (PARKING_USD, ROBOT_USD):
        if not asset.is_file():
            raise FileNotFoundError(
                f"{asset} missing -- run export_camera_robot_asset.py first" if asset == ROBOT_USD else asset
            )
    if OUTPUT_USD.exists():
        OUTPUT_USD.unlink()

    stage = Usd.Stage.CreateNew(str(OUTPUT_USD))
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.y)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    stage.SetTimeCodesPerSecond(60.0)

    # Keep the lot, 12 vehicles, lighting, PhysicsScene, ceiling LiDAR as-is.
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
    robot.GetReferences().AddReference(ROBOT_REF)
    robot_xform = UsdGeom.Xformable(robot)
    robot_xform.ClearXformOpOrder()
    robot_xform.AddTranslateOp().Set(Gf.Vec3d(start_x, 0.0, start_z))
    # Robot local +Z(up) -> parking lot world +Y(up), same adapter as the
    # team's mecha-roller variant.
    robot_xform.AddRotateXOp().Set(-90.0)
    robot.CreateAttribute("parkingRobot:asset", Sdf.ValueTypeNames.Asset).Set(
        Sdf.AssetPath(str(ROBOT_USD))
    )
    robot.CreateAttribute("parkingRobot:axisAdapter", Sdf.ValueTypeNames.String).Set(
        "robot Z-up -> parking Y-up; rotateX=-90deg"
    )
    robot.CreateAttribute("parkingRobot:variant", Sdf.ValueTypeNames.String).Set(
        "camera_mesh_depth_cam (4x RealSense D455: left/right/front/rear)"
    )

    camera = UsdGeom.Camera.Define(stage, "/World/HwiaParkingFieldCamera")
    camera.CreateFocalLengthAttr(25.0)
    eye = Gf.Vec3d(start_x + 8.0, 4.15, start_z - 7.2)
    target = Gf.Vec3d(start_x, 0.12, start_z)
    camera.AddTransformOp().Set(
        Gf.Matrix4d().SetLookAt(eye, target, Gf.Vec3d(0.0, 1.0, 0.0)).GetInverse()
    )

    stage.GetRootLayer().documentation = (
        "Integrated test field: parking_environment.usd (12 vehicles, ceiling "
        "LiDARs) plus the depth-camera-equipped HWIA parking robot "
        "(4x RealSense D455: left/right/front/rear)."
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
    return next((path for path in paths if path.endswith("/base_link")), paths[0])


def world_position(stage, path: str) -> tuple[float, float, float]:
    from pxr import Usd, UsdGeom

    matrix = UsdGeom.Xformable(stage.GetPrimAtPath(path)).ComputeLocalToWorldTransform(
        Usd.TimeCode.Default()
    )
    return tuple(float(value) for value in matrix.ExtractTranslation())


def main() -> None:
    restart_with_isaac_python()
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

        start = build_stage()
        context = omni.usd.get_context()
        context.open_stage(str(OUTPUT_USD))
        for _ in range(30):
            app.update()
        stage = context.get_stage()
        robot_body = find_robot_rigid_body(stage)
        print(f"[field] HWIA 물리 몸체: {robot_body}", flush=True)
        print(f"[field] HWIA 로봇 시작 좌표 X/Z: {start}", flush=True)

        if args.headless_test:
            from isaacsim.core.simulation_manager import SimulationManager

            for _ in range(120):
                if SimulationManager.get_physics_sim_view() is not None:
                    break
                app.update()
            before = world_position(stage, robot_body)
            timeline = omni.timeline.get_timeline_interface()
            timeline.play()
            for _ in range(180):
                app.update()
            timeline.pause()
            after = world_position(stage, robot_body)
            if not all(math.isfinite(value) for value in after):
                raise RuntimeError(f"로봇 위치가 비정상입니다: {after}")
            displacement = math.dist(before, after)
            if displacement > 0.35:
                raise RuntimeError(f"로봇 초기 물리 변위가 과도합니다: {displacement:.4f} m")
            print(
                f"[field] 180 frame 물리 안정성 통과: {before} -> {after} "
                f"(변위 {displacement:.4f} m)",
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
        print(f"[field] ERROR: {type(exc).__name__}: {exc}", flush=True)
        raise
    finally:
        app.close(wait_for_replicator=False)


if __name__ == "__main__":
    main()
