#!/usr/bin/env python3
"""Drive TWO mecanum parking robots in Isaac Sim, each on its own ROS 2
namespace, so the dev-machine control stack (task_dispatcher +
formation_gap_controller, see cobot3_ws/src/parking_control) can command and
watch them independently.

This is the 2-robot counterpart of mecanum_ros2_drive.py. It intentionally
drops the arm-lift demo logic from that script — this one is purely a
communication/formation test: "does each robot listen on its own /cmd_vel,
and does formation_gap_controller see believable /odom for each one".

Design notes:
  - mecanum_ros2_drive.py subscribes /cmd_vel via the C++ OmniGraph ROS2
    bridge node. This script instead uses Isaac's *internal* rclpy directly
    (same sys.path trick mecanum_ros2_drive.py already uses for its arm
    service, proven to work in this environment) for BOTH /cmd_vel subscribe
    and /odom publish, on both robots. One rclpy node, two robots, topics
    namespaced by robot_id ("/robot_1/cmd_vel", "/robot_1/odom", ...).
  - formation_gap_controller_node.py (cobot3_ws side) assumes exactly this
    "/<robot_id>/odom" topic naming convention, so no remapping is needed on
    the ROS2 side once this is running.
  - formation_gap_controller only reads pose.pose.position / .orientation
    from the Odometry message (see core/gap_hold_controller.py), so this
    script does not bother filling in twist/covariance — position+yaw only.

★ UNVERIFIED PIECE (read before trusting the numbers) ★
  This dev machine has no Isaac Sim install, so the Isaac world (X, Y-up, Z)
  -> ROS planar (x, y, yaw-about-Z) conversion below (isaac_pose_to_ros2d)
  could not be run here. The position mapping (ros_x=isaac_x, ros_y=isaac_z)
  is derived from data we already trust (parking_map.yaml / DB slot
  coordinates match Isaac Z directly, no sign flip — see project notes). The
  YAW SIGN is derived on paper from a right-hand-rule rotation-matrix
  argument, not measured. If a robot's yaw in `ros2 topic echo /robot_1/odom`
  turns the wrong way when you send a pure angular.z on /robot_1/cmd_vel,
  flip the sign in isaac_pose_to_ros2d's `ros_yaw = -isaac_yaw` line and tell
  me — that's the one line most likely to need correcting.

Run on the Isaac machine:
  cd /home/rokey/dev_ws/isaac_sim/isaacsim/_build/linux-x86_64/release
  ./python.sh /home/rokey/cobot3_ws/isaacpjt/Isaac_envo/mecanum_ros2_drive_dual.py --gui

Then, from this dev machine (or any ROS 2 Humble machine on the same
ROS_DOMAIN_ID):
  ros2 topic pub /robot_1/cmd_vel geometry_msgs/msg/Twist \
    '{linear: {x: 0.3, y: 0.0, z: 0.0}, angular: {z: 0.0}}'
  ros2 topic echo /robot_1/odom
  ros2 topic echo /robot_2/odom

Once both robots respond, task_dispatcher + formation_gap_controller
(running on the dev machine, no changes needed) can drive them for real:
robot_1 as leader, robot_2 as follower, gap_m=2.9 by default — matching the
GAP_M spawn offset below.
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
BRIDGE_RCLPY = Path(
    "/home/rokey/dev_ws/isaac_sim/isaacsim/_build/linux-x86_64/release"
    "/exts/isaacsim.ros2.bridge/humble/rclpy"
)

# robot_id -> world-Z spawn offset [m]. Matches formation_gap_controller's
# default gap_m=2.9 (core/gap_hold_controller.py) so a real dispatch task
# doesn't need to close a large initial gap on its first tick.
GAP_M = 2.9
ROBOTS = {
    "robot_1": {"z_offset": 0.0},
    "robot_2": {"z_offset": -GAP_M},
}

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
    """Ground + physics scene once, then one robot instance per ROBOTS entry.

    Local +X -> world +Z (facing=+1), same convention as
    mecanum_ros2_drive.py / two_robot_carry_demo.py. Robots are placed along
    world Z, GAP_M apart, both facing the same way (a convoy, not the
    opposite-ends carry approach two_robot_carry_demo.py uses).
    """
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
    UsdGeom.Gprim(ground).CreateDisplayColorAttr([Gf.Vec3f(0.30, 0.32, 0.34)])

    materials = UsdGeom.Scope.Define(stage, "/World/Materials").GetPath()
    grip = UsdShade.Material.Define(stage, materials.AppendChild("Grip"))
    grip_api = UsdPhysics.MaterialAPI.Apply(grip.GetPrim())
    grip_api.CreateStaticFrictionAttr(1.1)
    grip_api.CreateDynamicFrictionAttr(0.9)
    grip_api.CreateRestitutionAttr(0.0)
    UsdShade.MaterialBindingAPI.Apply(ground.GetPrim()).Bind(
        grip, UsdShade.Tokens.weakerThanDescendants, "physics"
    )

    for robot_id, spec in ROBOTS.items():
        robot = UsdGeom.Xform.Define(stage, f"/World/{robot_id}").GetPrim()
        robot.GetReferences().AddReference(str(ROBOT_USD))
        _replace_matrix_xform(
            robot,
            Gf.Matrix4d(
                0.0, 0.0, 1.0, 0.0,
                1.0, 0.0, 0.0, 0.0,
                0.0, 1.0, 0.0, 0.0,
                0.0, 0.0, spec["z_offset"], 1.0,
            ),
            UsdGeom,
        )
        joints_path = f"/World/{robot_id}/base_link/joints"
        configure_hub_drives(stage, joints_path)

    return stage


def isaac_pose_to_ros2d(position, orientation_wxyz):
    """Isaac world (X, Y-up, Z) + quaternion(w,x,y,z) -> ROS planar pose.

    ros_x = isaac_x, ros_y = isaac_z (validated against parking_map.yaml /
    DB slot coordinates elsewhere in this project — no sign flip needed for
    position). ros_yaw is derived on paper (right-hand rotation about
    Isaac's +Y maps to rotation about ROS's +Z with a sign flip under this
    x,y mapping) and is NOT empirically verified — see module docstring.
    Returns (ros_x, ros_y, ros_qz, ros_qw) — a quaternion with only
    z/w set (pure yaw-about-Z), which is all formation_gap_controller reads.
    """
    # 확정 규약 (2026-07-21 rokey 머신 실측, verify_dual_odom.py PASS):
    # ros_x = usd_x, ros_y = -usd_z, yaw = atan2(-fwd_z, fwd_x).
    # 원래 이 파일은 ros_y=+usd_z 로 공식 규약과 반대였다 — 정정.
    ros_x = float(position[0])
    ros_y = -float(position[2])
    w, x, y, z = (float(v) for v in orientation_wxyz)
    forward_x = 1.0 - 2.0 * (y * y + z * z)
    forward_z = 2.0 * (x * z - w * y)
    ros_yaw = math.atan2(-forward_z, forward_x)
    return ros_x, ros_y, math.sin(ros_yaw / 2.0), math.cos(ros_yaw / 2.0)


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

        timeline = omni.timeline.get_timeline_interface()
        timeline.play()
        for _ in range(20):
            app.update()

        robots = {}
        for robot_id in ROBOTS:
            art = Articulation(f"/World/{robot_id}/base_link/base_link")
            art.initialize()
            wheel_idx = {w: art.dof_names.index(j) for w, j in WHEEL_JOINTS.items()}
            robots[robot_id] = {
                "art": art,
                "wheel_idx": wheel_idx,
                "vel": np.zeros(art.get_joint_positions().shape, dtype=np.float32),
                "last_cmd": (0.0, 0.0, 0.0),
            }

        def drive(robot_id, vx, vy, wz):
            r = robots[robot_id]
            omegas = wheel_velocities_from_cmd_vel(vx, vy, wz)
            for w, omega in omegas.items():
                idx = r["wheel_idx"][w]
                if r["vel"].ndim == 2:
                    r["vel"][0, idx] = omega
                else:
                    r["vel"][idx] = omega
            r["art"].set_joint_velocity_targets(r["vel"])
            r["last_cmd"] = (vx, vy, wz)

        # --- Isaac's internal (Python 3.11) rclpy, same trick as the arm
        # service in mecanum_ros2_drive.py — proven to work in this env. ---
        if str(BRIDGE_RCLPY) not in sys.path:
            sys.path.insert(0, str(BRIDGE_RCLPY))
        import rclpy
        from geometry_msgs.msg import Twist
        from nav_msgs.msg import Odometry

        rclpy.init()
        node = rclpy.create_node("dual_mecanum_bridge")

        def _make_cmd_vel_cb(robot_id):
            def _cb(msg):
                # angular.z 반전: REP-103 정합 실측(run_dual_robot_ros2_field 동일).
                drive(robot_id, msg.linear.x, msg.linear.y, -msg.angular.z)
            return _cb

        odom_pubs = {}
        for robot_id in ROBOTS:
            node.create_subscription(
                Twist, f"/{robot_id}/cmd_vel", _make_cmd_vel_cb(robot_id), 10)
            odom_pubs[robot_id] = node.create_publisher(
                Odometry, f"/{robot_id}/odom", 10)

        print(f"ROS2_DUAL_DRIVE_READY robots={list(ROBOTS)} "
              f"domain={os.environ.get('ROS_DOMAIN_ID', '0')} "
              f"rmw={os.environ.get('RMW_IMPLEMENTATION', 'default')}", flush=True)

        def publish_odom():
            for robot_id, r in robots.items():
                positions, orientations = r["art"].get_world_poses()
                pos = np.asarray(positions)
                pos = pos[0] if pos.ndim == 2 else pos
                orient = np.asarray(orientations)
                orient = orient[0] if orient.ndim == 2 else orient
                ros_x, ros_y, qz, qw = isaac_pose_to_ros2d(pos, orient)

                msg = Odometry()
                msg.header.stamp = node.get_clock().now().to_msg()
                msg.header.frame_id = "odom"
                msg.child_frame_id = f"{robot_id}/base_link"
                msg.pose.pose.position.x = ros_x
                msg.pose.pose.position.y = ros_y
                msg.pose.pose.orientation.z = qz
                msg.pose.pose.orientation.w = qw
                odom_pubs[robot_id].publish(msg)

        def tick():
            app.update()
            rclpy.spin_once(node, timeout_sec=0.0)
            publish_odom()

        start = time.monotonic()
        last_print = -1.0
        while app.is_running():
            tick()

            now = time.monotonic()
            if now - last_print >= 1.0:
                for robot_id, r in robots.items():
                    positions, _ = r["art"].get_world_poses()
                    pos = np.asarray(positions)
                    pos = pos[0] if pos.ndim == 2 else pos
                    vx, vy, wz = r["last_cmd"]
                    print(f"[{robot_id}] cmd vx={vx:+.2f} vy={vy:+.2f} wz={wz:+.2f} "
                          f"| isaac world x={float(pos[0]):+.3f} z={float(pos[2]):+.3f}",
                          flush=True)
                last_print = now

            if RUN_SECONDS is not None and (now - start) >= RUN_SECONDS:
                print("ROS2_DUAL_DRIVE_EXIT (--seconds reached)", flush=True)
                break

        node.destroy_node()
        rclpy.shutdown()
        timeline.stop()
    except Exception as exc:
        print(f"ROS2_DUAL_DRIVE_EXCEPTION={type(exc).__name__}: {exc}", flush=True)
        raise
    finally:
        app.close()


if __name__ == "__main__":
    main()
