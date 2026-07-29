#!/usr/bin/env python3
"""Mecanum-roller strafe experiment for the HWIA parking robot (Isaac Sim 5.1).

Feasibility probe: with the mecanum overlay from ``mecanum_drive`` on the four
fixed drive wheels, does the robot translate SIDEWAYS under a strafe wheel
pattern? The robot source asset is NOT modified; rollers are an override on a
throwaway test stage. Pass/fail is decided by measuring lateral vs forward
chassis displacement.

Run headless:
  cd /home/rokey/dev_ws/isaac_sim/isaacsim/_build/linux-x86_64/release
  ./python.sh /home/rokey/cobot3_ws/isaacpjt/Isaac_envo/mecanum_strafe_test.py
"""

import json
import math
import os
import sys
from pathlib import Path

from mecanum_drive import (
    add_mecanum_rollers,
    configure_hub_drives,
    WHEEL_CENTERS,
    WHEEL_JOINTS,
)


WORK_DIR = Path(__file__).resolve().parent
ROBOT_USD = (
    WORK_DIR.parent
    / "hwia_parking_robot_final_caster_package"
    / "hwia_parking_robot_final_caster.usd"
)
OUTPUT_USD = WORK_DIR / "mecanum_strafe_test.usd"
REPORT_JSON = WORK_DIR / "mecanum_strafe_test_report.json"

ISAAC_ROOT = Path("/home/rokey/dev_ws/isaac_sim/isaacsim/_build/linux-x86_64/release")
ISAAC_PYTHON = ISAAC_ROOT / "python.sh"

ROBOT_WRAP = "/World/Robot/base_link"
ROBOT_ROOT = "/World/Robot/base_link/base_link"
ROBOT_JOINTS = "/World/Robot/base_link/joints"

ARM_JOINTS = (
    "arm_left_front_joint", "arm_left_rear_joint",
    "arm_right_front_joint", "arm_right_rear_joint",
)

# Pure lateral for this X-config mecanum overlay: FL-, FR+, RL+, RR-.
STRAFE_PATTERN = {"wheel_fl": -1.0, "wheel_fr": +1.0, "wheel_rl": +1.0, "wheel_rr": -1.0}
STRAFE_SPEED_DEG = 720.0

MIN_LATERAL_M = 0.10
MAX_FORWARD_RATIO = 0.6
MAX_VERTICAL_M = 0.15


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
    from pxr import Gf, PhysxSchema, Usd, UsdGeom, UsdPhysics, UsdShade

    if not ROBOT_USD.is_file():
        raise FileNotFoundError(ROBOT_USD)

    stage = Usd.Stage.CreateNew(str(OUTPUT_USD))
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.y)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    stage.SetTimeCodesPerSecond(60.0)
    world = UsdGeom.Xform.Define(stage, "/World").GetPrim()
    stage.SetDefaultPrim(world)

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

    ground = UsdGeom.Cube.Define(stage, "/World/TestGround")
    ground.CreateSizeAttr(1.0)
    ground.CreateVisibilityAttr(UsdGeom.Tokens.invisible)
    gx = UsdGeom.Xformable(ground)
    gx.AddTranslateOp().Set(Gf.Vec3d(0.0, -0.10, 0.0))
    gx.AddScaleOp().Set(Gf.Vec3f(30.0, 0.20, 30.0))
    UsdPhysics.CollisionAPI.Apply(ground.GetPrim())

    robot = UsdGeom.Xform.Define(stage, "/World/Robot").GetPrim()
    robot.GetReferences().AddReference(str(ROBOT_USD))
    robot_to_world = Gf.Matrix4d(
        0.0, 0.0, 1.0, 0.0,
        1.0, 0.0, 0.0, 0.0,
        0.0, 1.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 1.0,
    )
    _replace_matrix_xform(robot, robot_to_world, UsdGeom)

    materials = UsdGeom.Scope.Define(stage, "/World/TestMaterials").GetPath()
    grip = UsdShade.Material.Define(stage, materials.AppendChild("RollerGrip"))
    grip_api = UsdPhysics.MaterialAPI.Apply(grip.GetPrim())
    grip_api.CreateStaticFrictionAttr(1.1)
    grip_api.CreateDynamicFrictionAttr(0.9)
    grip_api.CreateRestitutionAttr(0.0)
    UsdShade.MaterialBindingAPI.Apply(ground.GetPrim()).Bind(
        grip, UsdShade.Tokens.weakerThanDescendants, "physics"
    )

    add_mecanum_rollers(stage, ROBOT_WRAP, ROBOT_JOINTS, grip_material=grip)
    configure_hub_drives(stage, ROBOT_JOINTS)

    for name in ARM_JOINTS:
        joint = stage.GetPrimAtPath(f"{ROBOT_JOINTS}/{name}")
        drive = UsdPhysics.DriveAPI.Get(joint, "angular")
        if drive:
            drive.CreateStiffnessAttr(2000.0)
            drive.CreateDampingAttr(150.0)
            drive.CreateMaxForceAttr(5000.0)
            drive.CreateTargetPositionAttr(0.0)

    stage.GetRootLayer().Save()


def run_test(app):
    import numpy as np
    import omni.physx
    import omni.timeline
    import omni.usd
    from isaacsim.core.prims import Articulation

    context = omni.usd.get_context()
    if not context.open_stage(str(OUTPUT_USD)):
        raise RuntimeError(f"failed to open test stage: {OUTPUT_USD}")
    for _ in range(12):
        app.update()

    timeline = omni.timeline.get_timeline_interface()
    timeline.play()
    for _ in range(12):
        app.update()

    robot = Articulation(ROBOT_ROOT)
    robot.initialize()
    physx = omni.physx.get_physx_interface()

    command_shape = robot.get_joint_positions().shape
    wheel_indices = {name: robot.dof_names.index(jname) for name, jname in WHEEL_JOINTS.items()}
    velocity_targets = np.zeros(command_shape, dtype=np.float32)

    def set_wheel_pattern(scale):
        speed = math.radians(STRAFE_SPEED_DEG) * scale
        for wheel_name in WHEEL_CENTERS:
            idx = wheel_indices[wheel_name]
            val = speed * STRAFE_PATTERN[wheel_name]
            if velocity_targets.ndim == 2:
                velocity_targets[0, idx] = val
            else:
                velocity_targets[idx] = val
        robot.set_joint_velocity_targets(velocity_targets)

    def base_pos():
        v = physx.get_rigidbody_transformation(ROBOT_ROOT)["position"]
        return tuple(float(x) for x in v)

    set_wheel_pattern(0.0)
    for _ in range(120):
        app.update()
    start = base_pos()

    set_wheel_pattern(1.0)
    for _ in range(360):
        app.update()
    set_wheel_pattern(0.0)
    for _ in range(30):
        app.update()
    end = base_pos()

    lateral = end[0] - start[0]
    forward = end[2] - start[2]
    vertical = end[1] - start[1]
    finite = all(math.isfinite(c) for c in end)

    lateral_ok = abs(lateral) >= MIN_LATERAL_M
    straight_ok = abs(forward) <= MAX_FORWARD_RATIO * max(abs(lateral), 1e-6)
    upright_ok = abs(vertical) <= MAX_VERTICAL_M and finite
    passed = bool(lateral_ok and straight_ok and upright_ok)

    report = {
        "passed": passed,
        "robot": str(ROBOT_USD),
        "strafe_pattern": STRAFE_PATTERN,
        "start_xyz_m": start,
        "end_xyz_m": end,
        "lateral_world_x_m": lateral,
        "forward_world_z_m": forward,
        "vertical_world_y_m": vertical,
        "checks": {
            "lateral_moved": lateral_ok,
            "stayed_straight": straight_ok,
            "stayed_upright": upright_ok,
        },
    }
    REPORT_JSON.write_text(json.dumps(report, indent=2, ensure_ascii=False), encoding="utf-8")

    print(f"MECANUM_PASSED={passed}", flush=True)
    print(f"MECANUM_LATERAL_M={lateral:.6f}", flush=True)
    print(f"MECANUM_FORWARD_M={forward:.6f}", flush=True)
    print(f"MECANUM_VERTICAL_M={vertical:.6f}", flush=True)
    print(f"REPORT={REPORT_JSON}", flush=True)

    timeline.stop()
    app.update()
    return report


def main():
    _restart_with_isaac_python()
    from isaacsim import SimulationApp

    app = SimulationApp({"headless": "--gui" not in sys.argv[1:]})
    try:
        build_test_stage()
        report = run_test(app)
        if not report["passed"]:
            print("MECANUM_RESULT=strafe experiment did not meet thresholds", flush=True)
    except Exception as exc:
        print(f"MECANUM_EXCEPTION={type(exc).__name__}: {exc}", flush=True)
        raise
    finally:
        app.close()


if __name__ == "__main__":
    main()
