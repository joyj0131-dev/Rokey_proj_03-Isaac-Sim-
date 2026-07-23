#!/usr/bin/env python3
"""M4 — Isaac 카메라 브링업. 전방 깊이캠 영상을 ROS 2 토픽으로 발행하고 로봇을
A2→A7 로 주행시킨다. 순수 ROS 쪽(marker_localizer_node)이 그 토픽을 받아 마커를
검출·측위한다.

역할 분리(사용자가 그린 구조):
  [이 스크립트=Isaac]  전방 카메라 → /image_raw + /camera_info 발행,  로봇 주행
  [marker_localizer_node=순수 ROS]  토픽 받아 검출+측위, 마커 ID·좌표 로그

카메라 발행은 C++ OmniGraph ROS2 브리지(ROS2CameraHelper)로 한다 — Isaac 인터프리터
안에서 rclpy 를 쓰지 않는다(mecanum_ros2_drive.py 와 같은 방식). 주행은 검증된
A2→A7 횡이동 + hold 보정(m3_drive_localize_test.py 규약)을 그대로 쓴다.

두 터미널 브리지 환경(사용자 강의안 검증본):
  export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
  export FASTRTPS_DEFAULT_PROFILES_FILE=$HOME/.ros/fastdds_whitelist.xml
  export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:.../isaacsim.ros2.bridge/humble/lib

실행:
    python3 aruco_sim_bringup.py                 # 헤드리스, A2→A7 1회 주행 후 대기
    python3 aruco_sim_bringup.py --gui
    python3 aruco_sim_bringup.py --seconds 60    # 60초 후 자동 종료
"""

import math
import os
import sys
from pathlib import Path

WORK_DIR = Path(__file__).resolve().parent
ROBOT_USD = (WORK_DIR.parent / "hwia_parking_robot_final_caster_package"
             / "hwia_depth_cam_mecha_roller.usd")
MARKER_STAGE = WORK_DIR / "parking" / "parking_environment_marker_preview.usd"
MAP_JSON = (WORK_DIR.parents[1] / "src" / "parkbot_aruco"
            / "data" / "marker_map.json")   # 지도는 ROS 패키지가 소유
ISAAC_PYTHON = Path(
    "/home/rokey/dev_ws/isaac_sim/isaacsim/_build/linux-x86_64/release/python.sh")

ROBOT_XFORM = "/World/Robot"
ROBOT_ROOT = "/World/Robot/base_link"
ROBOT_JOINTS = "/World/Robot/joints"
CAM_FRONT = "/World/Robot/cam_front_link/depth_cam_front/Camera_Pseudo_Depth_Front"
GRAPH_PATH = "/World/ArucoCamGraph"
CAM_RES = (640, 480)
CAM_FRAME_ID = "front_cam"
IMAGE_TOPIC = "image_raw"
INFO_TOPIC = "camera_info"
CMD_VEL_TOPIC = "cmd_vel"       # 이 토픽에 한 번 pub 하면 A2→A7 주행 시작

# --auto: 트리거 없이 시작하자마자 주행(예전 동작). 기본은 cmd_vel 트리거 대기.
AUTO_DRIVE = "--auto" in sys.argv[1:]

SPEED = 0.4
MARKER_STANDOFF = 1.25
START_LABEL, GOAL_LABEL = "A2", "A7"

RUN_SECONDS = None
for _i, _a in enumerate(sys.argv):
    if _a == "--seconds" and _i + 1 < len(sys.argv):
        RUN_SECONDS = float(sys.argv[_i + 1])


def _restart():
    if os.environ.get("CARB_APP_PATH"):
        return
    os.execv(str(ISAAC_PYTHON),
             [str(ISAAC_PYTHON), str(Path(__file__).resolve()), *sys.argv[1:]])


def _robot_matrix(Gf, tx, ty, tz):
    return Gf.Matrix4d(0, 0, 1, 0, 1, 0, 0, 0, 0, 1, 0, 0, tx, ty, tz, 1)


def main():
    _restart()
    import json
    mm = json.loads(MAP_JSON.read_text(encoding="utf-8"))
    by_label = {m["label"]: m for m in mm["markers"]}
    start, goal = by_label[START_LABEL], by_label[GOAL_LABEL]
    gui = "--gui" in sys.argv[1:]

    from isaacsim import SimulationApp
    app = SimulationApp({"headless": not gui})
    try:
        from isaacsim.core.utils.extensions import enable_extension
        enable_extension("isaacsim.ros2.bridge")
        app.update()
        app.update()

        import numpy as np
        import omni.graph.core as og
        import omni.physx
        import omni.replicator.core as rep
        import omni.timeline
        import omni.usd
        from isaacsim.core.api import World
        from isaacsim.core.prims import Articulation
        from pxr import Gf, Usd, UsdGeom, UsdPhysics

        from mecanum_drive import (
            WHEEL_JOINTS, configure_hub_drives, wheel_velocities_from_cmd_vel)

        # ---- 씬: 마커 환경 + 로봇을 A2 표준오프에 ----
        lane_z = float(start["z"]) - MARKER_STANDOFF
        stage = Usd.Stage.CreateInMemory()
        UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.y)
        UsdGeom.SetStageMetersPerUnit(stage, 1.0)
        w = UsdGeom.Xform.Define(stage, "/World").GetPrim()
        stage.SetDefaultPrim(w)
        env = UsdGeom.Xform.Define(stage, "/World/Env").GetPrim()
        env.GetReferences().AddReference(str(MARKER_STAGE))
        sc = UsdPhysics.Scene.Define(stage, "/World/PhysicsScene")
        sc.CreateGravityDirectionAttr(Gf.Vec3f(0, -1, 0))
        sc.CreateGravityMagnitudeAttr(9.81)
        r = stage.DefinePrim(ROBOT_XFORM, "Xform")
        r.GetReferences().AddReference(str(ROBOT_USD))
        UsdGeom.Xformable(r).ClearXformOpOrder()
        UsdGeom.Xformable(r).MakeMatrixXform().Set(
            _robot_matrix(Gf, float(start["x"]), 0.0, lane_z))

        tmp = WORK_DIR / "_bringup.usd"
        stage.GetRootLayer().Export(str(tmp))
        ctx = omni.usd.get_context()
        ctx.open_stage(str(tmp))
        for _ in range(30):
            app.update()
        live = ctx.get_stage()
        configure_hub_drives(live, ROBOT_JOINTS)

        # ---- 전방 카메라 렌더 프로덕트 + ROS2 발행 그래프 ----
        rp = rep.create.render_product(CAM_FRONT, CAM_RES)
        og.Controller.edit(
            {"graph_path": GRAPH_PATH, "evaluator_name": "execution"},
            {
                og.Controller.Keys.CREATE_NODES: [
                    ("OnTick", "omni.graph.action.OnPlaybackTick"),
                    ("CamRgb", "isaacsim.ros2.bridge.ROS2CameraHelper"),
                    # camera_info 는 CameraHelper 의 type 이 아니라 별도 노드다
                    # (허용 type: rgb/depth/segmentation…). "type is not supported"
                    # 에러는 여기서 났었다.
                    ("CamInfo", "isaacsim.ros2.bridge.ROS2CameraInfoHelper"),
                    # 주행 트리거: /cmd_vel 에 한 번 pub 하면 A2→A7 출발.
                    ("SubTwist", "isaacsim.ros2.bridge.ROS2SubscribeTwist"),
                ],
                og.Controller.Keys.SET_VALUES: [
                    ("CamRgb.inputs:renderProductPath", rp.path),
                    ("CamRgb.inputs:topicName", IMAGE_TOPIC),
                    ("CamRgb.inputs:type", "rgb"),
                    ("CamRgb.inputs:frameId", CAM_FRAME_ID),
                    ("CamInfo.inputs:renderProductPath", rp.path),
                    ("CamInfo.inputs:topicName", INFO_TOPIC),
                    ("CamInfo.inputs:frameId", CAM_FRAME_ID),
                    ("SubTwist.inputs:topicName", CMD_VEL_TOPIC),
                ],
                og.Controller.Keys.CONNECT: [
                    ("OnTick.outputs:tick", "CamRgb.inputs:execIn"),
                    ("OnTick.outputs:tick", "CamInfo.inputs:execIn"),
                    ("OnTick.outputs:tick", "SubTwist.inputs:execIn"),
                ],
            },
        )
        _twist_node = og.Controller.node(f"{GRAPH_PATH}/SubTwist")
        twist_lin = og.Controller.attribute("outputs:linearVelocity", _twist_node)
        twist_ang = og.Controller.attribute("outputs:angularVelocity", _twist_node)

        def cmd_vel_magnitude():
            lin = og.Controller.get(twist_lin)
            ang = og.Controller.get(twist_ang)
            return (abs(lin[0]) + abs(lin[1]) + abs(lin[2])
                    + abs(ang[0]) + abs(ang[1]) + abs(ang[2]))

        world = World(stage_units_in_meters=1.0, set_defaults=False)
        omni.timeline.get_timeline_interface().play()
        world.reset()
        for _ in range(30):
            world.step(render=True)
        try:
            dt_phys = float(world.get_physics_dt())
        except Exception:
            dt_phys = 1.0 / 60.0

        art = Articulation(ROBOT_ROOT)
        art.initialize()
        idx = {wn: art.dof_names.index(j) for wn, j in WHEEL_JOINTS.items()}
        vel = np.zeros(np.array(art.get_joint_velocities()).shape, dtype=np.float32)
        physx = omni.physx.get_physx_interface()

        def gt_pose():
            t = physx.get_rigidbody_transformation(ROBOT_ROOT)
            p = tuple(float(v) for v in t["position"])
            q = [float(v) for v in t["rotation"]]
            x, y, z, ww = q
            return p[0], p[2], math.degrees(math.atan2(1 - 2 * (y * y + z * z),
                                                       2 * (x * z - y * ww)))

        def drive(vx, vy, wz):
            for wn, om in wheel_velocities_from_cmd_vel(vx, vy, wz).items():
                if vel.ndim == 2:
                    vel[0, idx[wn]] = om
                else:
                    vel[idx[wn]] = om
            art.set_joint_velocity_targets(vel)

        print(f"[bringup] ROS2 카메라 발행 시작: /{IMAGE_TOPIC}, /{INFO_TOPIC} "
              f"(frame={CAM_FRAME_ID}, {CAM_RES[0]}x{CAM_RES[1]})", flush=True)
        print(f"[bringup] 순수 ROS 쪽에서 실행:  "
              f"ros2 run parkbot_aruco marker_localizer_node "
              f"--ros-args -p image_topic:=/{IMAGE_TOPIC} "
              f"-p camera_info_topic:=/{INFO_TOPIC}", flush=True)

        # ---- wz 부호 캘리브 ----
        _, _, gy0 = gt_pose()
        drive(0.0, 0.0, 0.3)
        for _ in range(60):
            world.step(render=True)
        drive(0.0, 0.0, 0.0)
        for _ in range(30):
            world.step(render=True)
        _, _, gy1 = gt_pose()
        yaw_sign = 1.0 if (gy1 - gy0) > 0 else -1.0
        # A2 표준오프로 복귀
        omni.timeline.get_timeline_interface().stop()
        for _ in range(3):
            app.update()
        UsdGeom.Xformable(live.GetPrimAtPath(ROBOT_XFORM)).MakeMatrixXform().Set(
            _robot_matrix(Gf, float(start["x"]), 0.0, lane_z))
        omni.timeline.get_timeline_interface().play()
        world.reset()
        for _ in range(40):
            world.step(render=True)

        # 주행 트리거 대기: 로봇은 A2 표준오프에 정지한 채 카메라만 계속 발행한다.
        # 순수 ROS 쪽에서 /cmd_vel 에 한 번 pub 하면 그때 A2→A7 주행을 시작한다.
        # (--auto 면 대기 없이 바로 출발)
        drive(0.0, 0.0, 0.0)
        triggered = AUTO_DRIVE
        if AUTO_DRIVE:
            print("[bringup] --auto: 트리거 없이 바로 주행", flush=True)
        else:
            print("[bringup] 대기 중 — A2 정지, 카메라 발행. 주행 트리거:", flush=True)
            print(f"[bringup]   ros2 topic pub --once /{CMD_VEL_TOPIC} "
                  f"geometry_msgs/msg/Twist "
                  f"'{{linear: {{x: 0.2}}}}'", flush=True)
            waited = 0
            while app.is_running():
                world.step(render=True)
                waited += 1
                if cmd_vel_magnitude() > 1e-4:
                    print("[bringup] /cmd_vel 수신 — A2→A7 주행 시작!", flush=True)
                    triggered = True
                    break
                if RUN_SECONDS is not None and waited * dt_phys >= RUN_SECONDS:
                    print("[bringup] 트리거 없이 시간 만료, 종료", flush=True)
                    break

        # ---- A2→A7 주행 (검증된 hold 보정) ----
        target_x = float(goal["x"]) + 0.8
        _run_drive = triggered
        step = 0
        elapsed = 0.0
        driving = _run_drive
        if _run_drive:
            drive(0.0, SPEED, 0.0)
            print(f"[bringup] A2→A7 주행 시작 (표준오프 {MARKER_STANDOFF} m)", flush=True)
        while app.is_running() and _run_drive:
            if driving:
                gx, gz, gy = gt_pose()
                cvx = max(-0.15, min(0.15, 1.5 * (lane_z - gz)))
                cwz = max(-0.5, min(0.5, yaw_sign * 0.025 * (0.0 - gy)))
                drive(cvx, SPEED, cwz)
                if gx >= target_x:
                    drive(0.0, 0.0, 0.0)
                    driving = False
                    print(f"[bringup] A7 도착 (x={gx:+.2f}). 카메라 계속 발행 중.",
                          flush=True)
            world.step(render=True)
            step += 1
            elapsed += dt_phys
            if RUN_SECONDS is not None and elapsed >= RUN_SECONDS:
                print(f"[bringup] --seconds {RUN_SECONDS} 경과, 종료", flush=True)
                break
        tmp.unlink(missing_ok=True)
    except Exception:
        import traceback
        print("!! 예외 발생:", flush=True)
        traceback.print_exc()
        sys.stdout.flush()
    finally:
        app.close()


if __name__ == "__main__":
    main()
