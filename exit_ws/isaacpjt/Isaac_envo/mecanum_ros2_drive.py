#!/usr/bin/env python3
"""Drive the mecanum parking robot in Isaac Sim from ROS 2 /cmd_vel.

Design note (Python 3.11 vs 3.10):
  Isaac Sim 5.1 runs on Python 3.11; ROS 2 Humble ships for Python 3.10. We do
  NOT use rclpy inside Isaac's interpreter. Instead the C++ OmniGraph ROS 2
  bridge (isaacsim.ros2.bridge, internal Humble libs) subscribes to /cmd_vel and
  we read the twist in the sim loop, apply the validated mecanum inverse
  kinematics (mecanum_drive.wheel_velocities_from_cmd_vel), and command the hub
  velocities. The wire protocol is DDS, so any external ROS 2 (e.g. a laptop on
  Python 3.10) can publish /cmd_vel as long as ROS_DOMAIN_ID and RMW match.

Run on the Isaac machine (watch it):
  cd /home/rokey/dev_ws/isaac_sim/isaacsim/_build/linux-x86_64/release
  ./python.sh /home/rokey/cobot3_ws/isaacpjt/Isaac_envo/mecanum_ros2_drive.py --gui

Then, from any ROS 2 (Humble) machine on the same ROS_DOMAIN_ID:
  ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
    '{linear: {x: 0.4, y: 0.0, z: 0.0}, angular: {z: 0.0}}'
"""

import math
import os
import sys
import time
from pathlib import Path

from mecanum_drive import wheel_velocities_from_cmd_vel, configure_hub_drives, WHEEL_JOINTS

WORK_DIR = Path(__file__).resolve().parent
ROBOT_USD = (
    WORK_DIR.parent
    / "hwia_parking_robot_final_caster_package"
    / "hwia_parking_robot_final_caster_mecha_roller.usd"
)
ISAAC_PYTHON = Path("/home/rokey/dev_ws/isaac_sim/isaacsim/_build/linux-x86_64/release/python.sh")

ROBOT_WRAP = "/World/Robot/base_link"
ROBOT_ROOT = "/World/Robot/base_link/base_link"
ROBOT_JOINTS = "/World/Robot/base_link/joints"
GRAPH_PATH = "/World/CmdVelGraph"
CMD_VEL_TOPIC = "cmd_vel"

ARM_JOINTS = (
    "arm_left_front_joint", "arm_left_rear_joint",
    "arm_right_front_joint", "arm_right_rear_joint",
)

# Arm "open" (deploy) angles [deg]; folded = 0. Same as the lift/carry demos.
ARM_OPEN = {
    "arm_left_front_joint": 90.0,
    "arm_left_rear_joint": -90.0,
    "arm_right_front_joint": -90.0,
    "arm_right_rear_joint": 90.0,
}
ARM_TOL_DEG = 4.0
ARM_SERVICE = "arm_control"
# Isaac's internal (Python 3.11) rclpy — used only for the arm service so the
# custom "wait until fully open, then respond" logic can be plain ROS 2 code.
BRIDGE_RCLPY = Path(
    "/home/rokey/dev_ws/isaac_sim/isaacsim/_build/linux-x86_64/release"
    "/exts/isaacsim.ros2.bridge/humble/rclpy"
)

# Optional auto-exit for headless verification: --seconds N
RUN_SECONDS = None
for i, a in enumerate(sys.argv):
    if a == "--seconds" and i + 1 < len(sys.argv):
        RUN_SECONDS = float(sys.argv[i + 1])


def _restart_with_isaac_python():
    if os.environ.get("CARB_APP_PATH"):
        return
    if not ISAAC_PYTHON.is_file():
        raise FileNotFoundError(f"Isaac Sim python.sh not found: {ISAAC_PYTHON}")
    os.execv(str(ISAAC_PYTHON), [str(ISAAC_PYTHON), str(Path(__file__).resolve()), *sys.argv[1:]])


def _replace_matrix_xform(prim, matrix, UsdGeom):
    xf = UsdGeom.Xformable(prim)
    xf.ClearXformOpOrder()
    xf.MakeMatrixXform().Set(matrix)


def build_stage():
    import omni.usd
    from pxr import Gf, PhysxSchema, UsdGeom, UsdPhysics, UsdShade

    if not ROBOT_USD.is_file():
        raise FileNotFoundError(
            f"{ROBOT_USD} not found. Build it first:\n"
            "  ./python.sh .../build_mecha_roller_asset.py"
        )

    context = omni.usd.get_context()
    context.new_stage()
    stage = context.get_stage()
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.y)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    stage.SetTimeCodesPerSecond(60.0)
    UsdGeom.Xform.Define(stage, "/World")
    stage.SetDefaultPrim(stage.GetPrimAtPath("/World"))

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

    ground = UsdGeom.Cube.Define(stage, "/World/Ground")
    ground.CreateSizeAttr(1.0)
    gx = UsdGeom.Xformable(ground)
    gx.AddTranslateOp().Set(Gf.Vec3d(0.0, -0.10, 0.0))
    gx.AddScaleOp().Set(Gf.Vec3f(60.0, 0.20, 60.0))
    UsdPhysics.CollisionAPI.Apply(ground.GetPrim())
    # a faint grid-ish color so motion is visible
    UsdGeom.Gprim(ground).CreateDisplayColorAttr([Gf.Vec3f(0.30, 0.32, 0.34)])

    robot = UsdGeom.Xform.Define(stage, "/World/Robot").GetPrim()
    robot.GetReferences().AddReference(str(ROBOT_USD))
    _replace_matrix_xform(
        robot,
        Gf.Matrix4d(
            0.0, 0.0, 1.0, 0.0,
            1.0, 0.0, 0.0, 0.0,
            0.0, 1.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 1.0,
        ),
        UsdGeom,
    )

    materials = UsdGeom.Scope.Define(stage, "/World/Materials").GetPath()
    grip = UsdShade.Material.Define(stage, materials.AppendChild("Grip"))
    grip_api = UsdPhysics.MaterialAPI.Apply(grip.GetPrim())
    grip_api.CreateStaticFrictionAttr(1.1)
    grip_api.CreateDynamicFrictionAttr(0.9)
    grip_api.CreateRestitutionAttr(0.0)
    UsdShade.MaterialBindingAPI.Apply(ground.GetPrim()).Bind(
        grip, UsdShade.Tokens.weakerThanDescendants, "physics"
    )

    configure_hub_drives(stage, ROBOT_JOINTS)
    for name in ARM_JOINTS:
        joint = stage.GetPrimAtPath(f"{ROBOT_JOINTS}/{name}")
        drive = UsdPhysics.DriveAPI.Get(joint, "angular")
        if drive:
            drive.CreateStiffnessAttr(2000.0)
            drive.CreateDampingAttr(150.0)
            drive.CreateMaxForceAttr(5000.0)
            drive.CreateTargetPositionAttr(0.0)

    return stage


def build_cmd_vel_graph():
    """Minimal OmniGraph: OnPlaybackTick -> ROS2SubscribeTwist(/cmd_vel).

    The C++ subscribe node does the DDS work; we read its outputs in the loop.
    """
    import omni.graph.core as og

    og.Controller.edit(
        {"graph_path": GRAPH_PATH, "evaluator_name": "execution"},
        {
            og.Controller.Keys.CREATE_NODES: [
                ("OnTick", "omni.graph.action.OnPlaybackTick"),
                ("SubscribeTwist", "isaacsim.ros2.bridge.ROS2SubscribeTwist"),
            ],
            og.Controller.Keys.SET_VALUES: [
                ("SubscribeTwist.inputs:topicName", CMD_VEL_TOPIC),
            ],
            og.Controller.Keys.CONNECT: [
                ("OnTick.outputs:tick", "SubscribeTwist.inputs:execIn"),
            ],
        },
    )
    node = og.Controller.node(f"{GRAPH_PATH}/SubscribeTwist")
    lin = og.Controller.attribute("outputs:linearVelocity", node)
    ang = og.Controller.attribute("outputs:angularVelocity", node)
    return lin, ang


def main():
    _restart_with_isaac_python()
    from isaacsim import SimulationApp

    app = SimulationApp({"headless": "--gui" not in sys.argv[1:]})

    from isaacsim.core.utils.extensions import enable_extension
    enable_extension("isaacsim.ros2.bridge")
    for _ in range(10):
        app.update()

    import numpy as np
    import omni.timeline
    from isaacsim.core.prims import Articulation

    try:
        build_stage()
        lin_attr, ang_attr = build_cmd_vel_graph()

        timeline = omni.timeline.get_timeline_interface()
        timeline.play()
        for _ in range(20):
            app.update()

        robot = Articulation(ROBOT_ROOT)
        robot.initialize()
        shape = robot.get_joint_positions().shape
        wheel_idx = {w: robot.dof_names.index(j) for w, j in WHEEL_JOINTS.items()}
        vel = np.zeros(shape, dtype=np.float32)

        def drive(vx, vy, wz):
            omegas = wheel_velocities_from_cmd_vel(vx, vy, wz)
            for w, omega in omegas.items():
                idx = wheel_idx[w]
                if vel.ndim == 2:
                    vel[0, idx] = omega
                else:
                    vel[idx] = omega
            robot.set_joint_velocity_targets(vel)

        def read_and_drive():
            lin = lin_attr.get()
            ang = ang_attr.get()
            drive(float(lin[0]), float(lin[1]), float(ang[2]))

        def tick():
            app.update()
            read_and_drive()   # keep responding to /cmd_vel every step

        # --- arm position control (arm joints already have position drives) ---
        arm_idx = {n: robot.dof_names.index(n) for n in ARM_OPEN}
        pos = np.array(robot.get_joint_positions(), dtype=np.float32, copy=True)

        def set_arm(scale):
            for n, deg in ARM_OPEN.items():
                val = math.radians(deg * scale)
                if pos.ndim == 2:
                    pos[0, arm_idx[n]] = val
                else:
                    pos[arm_idx[n]] = val
            robot.set_joint_position_targets(pos)

        def arms_reached(scale):
            cur = robot.get_joint_positions()
            cur = cur[0] if getattr(cur, "ndim", 1) == 2 else cur
            return all(
                abs(math.degrees(float(cur[arm_idx[n]])) - deg * scale) < ARM_TOL_DEG
                for n, deg in ARM_OPEN.items()
            )

        def move_arms(scale, ramp=120, hold=300):
            for step in range(1, ramp + 1):
                set_arm(scale * step / ramp)
                tick()
            for _ in range(hold):
                tick()
                if arms_reached(scale):
                    return True
            return False

        # --- optional arm service via Isaac's internal (3.11) rclpy ---
        ros = {"ok": False, "node": None, "lib": None}
        try:
            if str(BRIDGE_RCLPY) not in sys.path:
                sys.path.insert(0, str(BRIDGE_RCLPY))
            import rclpy
            from std_srvs.srv import SetBool

            rclpy.init()
            ros["lib"] = rclpy
            ros["node"] = rclpy.create_node("mecanum_arm_service")

            def _arm_cb(request, response):
                opening = bool(request.data)
                ok = move_arms(1.0 if opening else 0.0)
                response.success = bool(ok)
                if opening:
                    response.message = "arms fully opened" if ok else "arm open timeout"
                else:
                    response.message = "arms folded" if ok else "arm fold timeout"
                print(f"ARM_CMD {'open' if opening else 'close'} -> success={ok}", flush=True)
                return response

            ros["node"].create_service(SetBool, ARM_SERVICE, _arm_cb)
            ros["ok"] = True
            print(f"ARM_SERVICE_READY /{ARM_SERVICE} (std_srvs/SetBool: data=true opens, false folds)",
                  flush=True)
        except Exception as exc:
            print(f"ARM_SERVICE_UNAVAILABLE={type(exc).__name__}: {exc} "
                  "(cmd_vel driving still works)", flush=True)

        print(f"ROS2_DRIVE_READY topic=/{CMD_VEL_TOPIC} domain={os.environ.get('ROS_DOMAIN_ID','0')} "
              f"rmw={os.environ.get('RMW_IMPLEMENTATION','default')}", flush=True)

        def base_xz():
            pose = robot.get_world_poses()[0]
            p = np.asarray(pose)[0] if np.asarray(pose).ndim == 2 else np.asarray(pose)
            return float(p[0]), float(p[2])

        start = time.monotonic()
        last_print = -1.0
        while app.is_running():
            tick()
            if ros["ok"]:
                ros["lib"].spin_once(ros["node"], timeout_sec=0.0)

            now = time.monotonic()
            if now - last_print >= 1.0:
                x, z = base_xz()
                lin = lin_attr.get()
                ang = ang_attr.get()
                print(f"CMD vx={float(lin[0]):+.2f} vy={float(lin[1]):+.2f} wz={float(ang[2]):+.2f} "
                      f"| base world x={x:+.3f} z={z:+.3f}", flush=True)
                last_print = now

            if RUN_SECONDS is not None and (now - start) >= RUN_SECONDS:
                print("ROS2_DRIVE_EXIT (--seconds reached)", flush=True)
                break

        if ros["ok"]:
            try:
                ros["node"].destroy_node()
                ros["lib"].shutdown()
            except Exception:
                pass
        timeline.stop()
    except Exception as exc:
        print(f"ROS2_DRIVE_EXCEPTION={type(exc).__name__}: {exc}", flush=True)
        raise
    finally:
        app.close()


if __name__ == "__main__":
    main()
