#!/usr/bin/env python3
"""현재 주차장 안 HWIA 로봇 2대를 ROS 2로 구동하는 Isaac Sim 런타임.

토픽:
  /robot_1/cmd_vel  -> robot_1 바퀴 속도
  /robot_2/cmd_vel  -> robot_2 바퀴 속도
  /robot_1/odom, /robot_2/odom -> ROS map 좌표계 pose

기본적으로 외부 인계 차량 H1~H6를 런타임에서만 숨기고 H1 위치에 가상
목표 영역을 표시한다. 원본 parking_environment.usd는 수정하지 않는다.
"""

from __future__ import annotations

import math
import os
import sys
import time
from pathlib import Path

from isaac_runtime import restart_with_isaac_python


ROOT = Path(__file__).resolve().parent
ISAAC_ENVO = ROOT.parent
FIELD_USD = ROOT / "parking_robot_field_dual_markers.usd"
BRIDGE_RCLPY = Path(
    "/home/rokey/dev_ws/isaac_sim/isaacsim/_build/linux-x86_64/release"
    "/exts/isaacsim.ros2.bridge/humble/rclpy"
)
ROBOT_IDS = ("robot_1", "robot_2")
VIRTUAL_TARGET_USD = (-21.85, 0.0, 2.35)
CMD_TIMEOUT_SEC = 0.6


def _arg_value(name: str, default=None):
    args = sys.argv[1:]
    if name in args:
        index = args.index(name)
        if index + 1 < len(args):
            return args[index + 1]
    return default


RUN_SECONDS = float(_arg_value("--seconds", "0")) or None
# 새 인계장의 대기 차량 2대는 E2E 미션의 대상이므로 기본 유지.
KEEP_HANDOFF_VEHICLES = "--hide-handoff-vehicles" not in sys.argv[1:]


def _isaac_pose_to_ros(position, orientation_wxyz):
    """Y-up USD pose를 확정된 ROS map 규약(x=usd_x, y=-usd_z)으로 변환."""
    x_world, _, z_world = (float(value) for value in position)
    w, x, y, z = (float(value) for value in orientation_wxyz)
    # quaternion으로 local +X 전방 벡터를 world로 회전한다.
    forward_x = 1.0 - 2.0 * (y * y + z * z)
    forward_z = 2.0 * (x * z - w * y)
    yaw = math.atan2(-forward_z, forward_x)
    return x_world, -z_world, yaw


def _add_virtual_target(stage):
    from pxr import Gf, UsdGeom

    marker = UsdGeom.Cube.Define(stage, "/World/VirtualHandoffTarget")
    marker.CreateSizeAttr(1.0)
    marker.CreateDisplayColorAttr([Gf.Vec3f(0.05, 0.85, 0.35)])
    marker.CreateDisplayOpacityAttr([0.32])
    xform = UsdGeom.Xformable(marker)
    xform.AddTranslateOp().Set(Gf.Vec3d(*VIRTUAL_TARGET_USD))
    xform.AddScaleOp().Set(Gf.Vec3f(3.1, 0.025, 1.75))


def main() -> None:
    restart_with_isaac_python(Path(__file__))
    from isaacsim import SimulationApp

    app = SimulationApp(
        {
            "headless": "--headless" in sys.argv[1:],
            "width": 1280,
            "height": 800,
            "enable_motion_bvh": True,
        }
    )
    try:
        sys.path.insert(0, str(ISAAC_ENVO))
        from mecanum_drive import (
            WHEEL_JOINTS,
            configure_hub_drives,
            wheel_velocities_from_cmd_vel,
        )

        from isaacsim.core.utils.extensions import enable_extension

        enable_extension("isaacsim.ros2.bridge")
        for _ in range(12):
            app.update()

        import numpy as np
        import omni.timeline
        import omni.usd
        from isaacsim.core.prims import Articulation

        if not FIELD_USD.is_file():
            raise FileNotFoundError(
                f"{FIELD_USD}가 없습니다. build_dual_robot_parking_field.py를 먼저 실행하세요."
            )
        context = omni.usd.get_context()
        context.open_stage(str(FIELD_USD))
        for _ in range(30):
            app.update()
        stage = context.get_stage()

        if not KEEP_HANDOFF_VEHICLES:
            handoff = stage.GetPrimAtPath("/World/ParkingVehicles/HandoffQueue")
            if not handoff:
                raise RuntimeError("HandoffQueue를 찾지 못했습니다.")
            handoff.SetActive(False)
        _add_virtual_target(stage)

        for robot_id in ROBOT_IDS:
            configure_hub_drives(
                stage, f"/World/Robots/{robot_id}/joints"
            )

        timeline = omni.timeline.get_timeline_interface()
        timeline.play()
        for _ in range(25):
            app.update()

        robots = {}
        for robot_id in ROBOT_IDS:
            articulation = Articulation(
                f"/World/Robots/{robot_id}/base_link"
            )
            articulation.initialize()
            wheel_indices = {
                wheel: articulation.dof_names.index(joint)
                for wheel, joint in WHEEL_JOINTS.items()
            }
            robots[robot_id] = {
                "articulation": articulation,
                "wheel_indices": wheel_indices,
                "velocities": np.zeros(
                    articulation.get_joint_positions().shape, dtype=np.float32
                ),
                "last_command_at": 0.0,
                "last_command": (0.0, 0.0, 0.0),
            }

        def drive(robot_id, vx, vy, wz):
            robot = robots[robot_id]
            omegas = wheel_velocities_from_cmd_vel(vx, vy, wz)
            velocities = robot["velocities"]
            velocities[...] = 0.0
            for wheel, omega in omegas.items():
                index = robot["wheel_indices"][wheel]
                if velocities.ndim == 2:
                    velocities[0, index] = omega
                else:
                    velocities[index] = omega
            robot["articulation"].set_joint_velocity_targets(velocities)
            robot["last_command"] = (float(vx), float(vy), float(wz))

        if str(BRIDGE_RCLPY) not in sys.path:
            sys.path.insert(0, str(BRIDGE_RCLPY))
        import rclpy
        from geometry_msgs.msg import Twist
        from nav_msgs.msg import Odometry

        rclpy.init()
        node = rclpy.create_node("dual_hwia_parking_field_bridge")

        def make_command_callback(robot_id):
            def callback(message):
                robots[robot_id]["last_command_at"] = time.monotonic()
                # angular.z 부호 반전: REP-103(+wz=반시계) 실측 정합 (2026-07-21).
                # mecanum IK의 +wz는 이 씬에서 시계방향 회전을 만든다
                # (DEBUG_LOG "wz=+0.3 → -51.5도" 실측과 동일). ROS 경계에서만
                # 뒤집고 공용 mecanum_drive 모듈은 건드리지 않는다.
                drive(
                    robot_id,
                    message.linear.x,
                    message.linear.y,
                    -message.angular.z,
                )

            return callback

        odom_publishers = {}
        subscriptions = []
        for robot_id in ROBOT_IDS:
            subscriptions.append(
                node.create_subscription(
                    Twist,
                    f"/{robot_id}/cmd_vel",
                    make_command_callback(robot_id),
                    10,
                )
            )
            odom_publishers[robot_id] = node.create_publisher(
                Odometry, f"/{robot_id}/odom", 10
            )

        print(
            "ROS2_DUAL_FIELD_READY "
            f"robots={list(ROBOT_IDS)} domain={os.environ.get('ROS_DOMAIN_ID', '0')} "
            f"rmw={os.environ.get('RMW_IMPLEMENTATION', 'default')} "
            f"virtual_target_ros=(-21.85,-2.35)",
            flush=True,
        )

        start = time.monotonic()
        last_log = 0.0
        while app.is_running():
            app.update()
            rclpy.spin_once(node, timeout_sec=0.0)
            now = time.monotonic()

            for robot_id, robot in robots.items():
                if (
                    robot["last_command_at"] > 0.0
                    and now - robot["last_command_at"] > CMD_TIMEOUT_SEC
                    and robot["last_command"] != (0.0, 0.0, 0.0)
                ):
                    drive(robot_id, 0.0, 0.0, 0.0)

                positions, orientations = robot["articulation"].get_world_poses()
                position = np.asarray(positions)
                position = position[0] if position.ndim == 2 else position
                orientation = np.asarray(orientations)
                orientation = orientation[0] if orientation.ndim == 2 else orientation
                ros_x, ros_y, ros_yaw = _isaac_pose_to_ros(position, orientation)

                message = Odometry()
                message.header.stamp = node.get_clock().now().to_msg()
                message.header.frame_id = "map"
                message.child_frame_id = f"{robot_id}/base_link"
                message.pose.pose.position.x = ros_x
                message.pose.pose.position.y = ros_y
                message.pose.pose.orientation.z = math.sin(ros_yaw * 0.5)
                message.pose.pose.orientation.w = math.cos(ros_yaw * 0.5)
                odom_publishers[robot_id].publish(message)

                if now - last_log >= 1.0:
                    vx, vy, wz = robot["last_command"]
                    print(
                        f"[{robot_id}] pose=({ros_x:+.2f},{ros_y:+.2f},"
                        f"{math.degrees(ros_yaw):+.1f}deg) "
                        f"cmd=({vx:+.2f},{vy:+.2f},{wz:+.2f})",
                        flush=True,
                    )
            if now - last_log >= 1.0:
                last_log = now
            if RUN_SECONDS is not None and now - start >= RUN_SECONDS:
                break

        for robot_id in ROBOT_IDS:
            drive(robot_id, 0.0, 0.0, 0.0)
        node.destroy_node()
        rclpy.shutdown()
        timeline.stop()
    except Exception as exc:
        print(f"ROS2_DUAL_FIELD_ERROR={type(exc).__name__}: {exc}", flush=True)
        raise
    finally:
        app.close(wait_for_replicator=False)


if __name__ == "__main__":
    main()
