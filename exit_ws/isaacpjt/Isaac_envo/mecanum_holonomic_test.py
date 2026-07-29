#!/usr/bin/env python3
"""Holonomic /cmd_vel validation for the mecanum overlay (Isaac Sim 5.1).

Drives the robot with ``mecanum_drive.wheel_velocities_from_cmd_vel`` through
three primitives -- forward, strafe-left, rotate-CCW -- and checks that each
commanded axis produces the expected chassis motion. This is the calibration
gate for the cmd_vel sign constants in ``mecanum_drive``.

Run headless:
  cd /home/rokey/dev_ws/isaac_sim/isaacsim/_build/linux-x86_64/release
  ./python.sh /home/rokey/cobot3_ws/isaacpjt/Isaac_envo/mecanum_holonomic_test.py
"""

import json
import math
import os
import sys
from pathlib import Path

from mecanum_drive import (
    add_mecanum_rollers,
    configure_hub_drives,
    wheel_velocities_from_cmd_vel,
    WHEEL_JOINTS,
)


WORK_DIR = Path(__file__).resolve().parent
ROBOT_USD = (
    WORK_DIR.parent
    / "hwia_parking_robot_final_caster_package"
    / "hwia_parking_robot_final_caster.usd"
)
OUTPUT_USD = WORK_DIR / "mecanum_holonomic_test.usd"
REPORT_JSON = WORK_DIR / "mecanum_holonomic_test_report.json"

ISAAC_ROOT = Path("/home/rokey/dev_ws/isaac_sim/isaacsim/_build/linux-x86_64/release")
ISAAC_PYTHON = ISAAC_ROOT / "python.sh"

ROBOT_WRAP = "/World/Robot/base_link"
ROBOT_ROOT = "/World/Robot/base_link/base_link"
ROBOT_JOINTS = "/World/Robot/base_link/joints"

ARM_JOINTS = (
    "arm_left_front_joint", "arm_left_rear_joint",
    "arm_right_front_joint", "arm_right_rear_joint",
)

LIN_SPEED = 0.4      # m/s for forward / strafe segments
YAW_SPEED = 0.5      # rad/s for the rotate segment
SEG_STEPS = 180      # ~3 s per segment at 60 fps
SETTLE_STEPS = 40


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


def _rotate_vec_by_quat(q_wxyz, v):
    import numpy as np

    w, x, y, z = (float(q_wxyz[0]), float(q_wxyz[1]), float(q_wxyz[2]), float(q_wxyz[3]))
    qv = np.array([x, y, z], dtype=np.float64)
    v = np.asarray(v, dtype=np.float64)
    t = 2.0 * np.cross(qv, v)
    return v + w * t + np.cross(qv, t)


def run_test(app):
    import numpy as np
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

    command_shape = robot.get_joint_positions().shape
    wheel_indices = {name: robot.dof_names.index(jname) for name, jname in WHEEL_JOINTS.items()}
    velocity_targets = np.zeros(command_shape, dtype=np.float32)

    def set_cmd_vel(vx, vy, wz):
        omegas = wheel_velocities_from_cmd_vel(vx, vy, wz)
        for wheel_name, omega in omegas.items():
            idx = wheel_indices[wheel_name]
            if velocity_targets.ndim == 2:
                velocity_targets[0, idx] = omega
            else:
                velocity_targets[idx] = omega
        robot.set_joint_velocity_targets(velocity_targets)

    def pose():
        pos, orient = robot.get_world_poses()
        p = np.asarray(pos)[0] if np.asarray(pos).ndim == 2 else np.asarray(pos)
        q = np.asarray(orient)[0] if np.asarray(orient).ndim == 2 else np.asarray(orient)
        fwd = _rotate_vec_by_quat(q, [1.0, 0.0, 0.0])   # robot +X in world
        heading = math.degrees(math.atan2(float(fwd[0]), float(fwd[2])))
        return (float(p[0]), float(p[1]), float(p[2])), heading

    def settle():
        set_cmd_vel(0.0, 0.0, 0.0)
        for _ in range(SETTLE_STEPS):
            app.update()

    def run_segment(vx, vy, wz):
        settle()
        (p0, h0) = pose()
        set_cmd_vel(vx, vy, wz)
        for _ in range(SEG_STEPS):
            app.update()
        settle()
        (p1, h1) = pose()
        dyaw = h1 - h0
        if dyaw > 180.0:
            dyaw -= 360.0
        elif dyaw < -180.0:
            dyaw += 360.0
        return {
            "cmd": {"vx": vx, "vy": vy, "wz": wz},
            "lateral_world_x_m": p1[0] - p0[0],   # robot left (+Y) maps here
            "forward_world_z_m": p1[2] - p0[2],   # robot forward (+X) maps here
            "vertical_world_y_m": p1[1] - p0[1],
            "dyaw_deg": dyaw,
        }

    for _ in range(120):   # initial settle on the rollers
        app.update()

    seg_forward = run_segment(LIN_SPEED, 0.0, 0.0)
    seg_strafe = run_segment(0.0, LIN_SPEED, 0.0)
    seg_rotate = run_segment(0.0, 0.0, YAW_SPEED)

    # Direction checks (robot +X -> world +Z; robot +Y left -> world +X).
    fwd_ok = (seg_forward["forward_world_z_m"] > 0.15 and
              abs(seg_forward["forward_world_z_m"]) > abs(seg_forward["lateral_world_x_m"]))
    strafe_ok = (seg_strafe["lateral_world_x_m"] > 0.15 and
                 abs(seg_strafe["lateral_world_x_m"]) > abs(seg_strafe["forward_world_z_m"]))
    rotate_ok = (seg_rotate["dyaw_deg"] > 15.0 and
                 abs(seg_rotate["dyaw_deg"]) > 8.0 * max(
                     abs(seg_rotate["lateral_world_x_m"]),
                     abs(seg_rotate["forward_world_z_m"])))
    passed = bool(fwd_ok and strafe_ok and rotate_ok)

    report = {
        "passed": passed,
        "robot": str(ROBOT_USD),
        "lin_speed_m_s": LIN_SPEED,
        "yaw_speed_rad_s": YAW_SPEED,
        "forward_segment": seg_forward,
        "strafe_left_segment": seg_strafe,
        "rotate_ccw_segment": seg_rotate,
        "checks": {"forward_ok": fwd_ok, "strafe_ok": strafe_ok, "rotate_ok": rotate_ok},
    }
    REPORT_JSON.write_text(json.dumps(report, indent=2, ensure_ascii=False), encoding="utf-8")

    print(f"HOLO_PASSED={passed}", flush=True)
    for label, seg in (
        ("FORWARD", seg_forward), ("STRAFE_LEFT", seg_strafe), ("ROTATE_CCW", seg_rotate),
    ):
        print(
            f"HOLO_{label} dx={seg['lateral_world_x_m']:.3f} "
            f"dz={seg['forward_world_z_m']:.3f} dy={seg['vertical_world_y_m']:.3f} "
            f"dyaw={seg['dyaw_deg']:.2f}",
            flush=True,
        )
    print(f"HOLO_CHECKS fwd={fwd_ok} strafe={strafe_ok} rotate={rotate_ok}", flush=True)
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
            print("HOLO_RESULT=cmd_vel mapping did not meet direction checks", flush=True)
    except Exception as exc:
        print(f"HOLO_EXCEPTION={type(exc).__name__}: {exc}", flush=True)
        raise
    finally:
        app.close()


if __name__ == "__main__":
    main()
