#!/usr/bin/env python3
"""v4 주차장 러너 — 로봇 4대(입차팀/출차팀) + 휠 오도메트리 + probe 진입점.

기존 dock_lift_handoff_runner_v2.py(로봇 2대)는 건드리지 않는다. 검증된 2대 구성이
비교 기준으로 남아야 한다.

좌표 규약은 site_map_v4 가 전담한다(입차=z양수, 에셋 라벨과 반대).

실행: parking_v4_runner.sh [--gui] [--headless-test]
"""
import math
import os
import sys
from pathlib import Path

WORK_DIR = Path(__file__).resolve().parent
REPO_ROOT = WORK_DIR.parent.parent
PARKING_USD = WORK_DIR / "parking" / "parking_environment_v4.usd"
ROBOT_USD = (WORK_DIR.parent / "hwia_parking_robot_final_caster_package"
             / "hwia_depth_cam_mecha_roller_lowered.usd")
ISAAC_PYTHON = Path("/home/rokey/dev_ws/isaac_sim/isaacsim/_build/linux-x86_64/release/python.sh")

sys.path.insert(0, str(REPO_ROOT / "src" / "parkbot_aruco"))
from parkbot_aruco import site_map_v4 as sm   # noqa: E402

RENDER_HZ = 60.0
RENDER_WIDTH = 640
RENDER_HEIGHT = 400
PHYSICS_HZ = 120.0
LINEAR_ACCEL = 0.5
LINEAR_DECEL = 0.8
ANGULAR_ACCEL = 0.8
ROBOT_SPAWN_Y = 0.06
# probe B(휠 오도메트리 드리프트) 측정 직전 정착(settle) 프레임 수.
# 드리프트는 초기 settle 정도에 매우 민감하다. 이 값을 명시적으로 고정하지
# 않으면 측정과 무관한 다른 코드 변경(예: 카메라 부착 루프의 app.update()
# 횟수)이 우연히 settle 정도를 바꿔 결과가 재현 불가능해진다.
PROBE_SETTLE_FRAMES = 120


def _restart_with_isaac_python():
    if os.environ.get("CARB_APP_PATH"):
        return
    os.execv(str(ISAAC_PYTHON), [str(ISAAC_PYTHON), str(Path(__file__).resolve()), *sys.argv[1:]])


def read_markers(stage):
    """v4 스테이지의 aruco:* 속성을 읽어 serves -> 정보 dict 로 돌려준다.

    좌표를 손으로 옮겨 적지 않기 위한 것이다. 에셋이 바뀌면 값이 따라온다.
    """
    out = {}
    for prim in stage.Traverse():
        a = prim.GetAttribute("aruco:markerId")
        if not a or not a.IsValid():
            continue
        pos = prim.GetAttribute("aruco:position").Get()
        serves = prim.GetAttribute("aruco:serves").Get()
        yaw_attr = prim.GetAttribute("aruco:yaw")
        kind_attr = prim.GetAttribute("aruco:kind")
        out[str(serves)] = {
            "id": int(a.Get()),
            "x": float(pos[0]),
            "z": float(pos[2]),
            "yaw": float(yaw_attr.Get()) if yaw_attr and yaw_attr.IsValid() else 0.0,
            "kind": str(kind_attr.Get()) if kind_attr and kind_attr.IsValid() else "",
        }
    return out


def marker_visual_center(stage, serves, time_code=None):
    """실제로 렌더되는 마커 데칼 메시의 월드 중심(x, z)을 돌려준다.

    Probe A 조사로 확인: 도크 마커(D_OUT_*/D_IN_*)는 `aruco:position` 속성값과
    데칼 메시의 실제 월드 위치가 z 로 0.7m 어긋난다(속성=도크/스폰 좌표,
    데칼=차선 쪽으로 밀린 실제 그림 위치). x 는 항상 일치했다. 카메라가
    "실제로 보는" 대상은 그림이므로 자세 계측에는 이 함수를 쓰고,
    `read_markers()`(속성 기반)는 기존 용도(로봇 스폰 좌표 등) 그대로 둔다.
    """
    from pxr import Gf, Usd, UsdGeom
    tc = time_code if time_code is not None else Usd.TimeCode.Default()
    for prim in stage.Traverse():
        sv = prim.GetAttribute("aruco:serves")
        if sv and sv.IsValid() and str(sv.Get()) == serves:
            xf = UsdGeom.Xformable(prim)
            wc = xf.ComputeLocalToWorldTransform(tc).Transform(Gf.Vec3d(0, 0, 0))
            return float(wc[0]), float(wc[2])
    raise RuntimeError(f"marker_visual_center: serves={serves!r} 데칼 메시를 찾지 못함")


def _apply_physics(stage):
    from pxr import PhysxSchema
    sc = stage.GetPrimAtPath("/World/PhysicsScene")
    if not sc or not sc.IsValid():
        raise RuntimeError("v4 PhysicsScene 없음")
    px = PhysxSchema.PhysxSceneAPI.Apply(sc)
    px.CreateBroadphaseTypeAttr("GPU")
    px.CreateSolverTypeAttr("TGS")
    px.CreateEnableCCDAttr(True)
    px.CreateEnableStabilizationAttr(True)
    px.CreateEnableGPUDynamicsAttr(True)
    px.CreateTimeStepsPerSecondAttr(PHYSICS_HZ)
    vctx = PhysxSchema.PhysxVehicleContextAPI.Apply(sc)
    vctx.CreateUpdateModeAttr(PhysxSchema.Tokens.velocityChange)
    vctx.CreateVerticalAxisAttr(PhysxSchema.Tokens.posY)
    vctx.CreateLongitudinalAxisAttr(PhysxSchema.Tokens.posZ)


def _disable_sensors(stage):
    """천장 RTX 라이다는 이 작업에 불필요하고 무겁다. 원본은 수정하지 않는다."""
    sensors = stage.GetPrimAtPath("/World/Sensors")
    if sensors and sensors.IsValid():
        n = sum("Lidar" in c.GetName() for c in sensors.GetChildren())
        sensors.SetActive(False)
        return n
    return 0


def robot_prim_path(robot_id):
    return f"/World/Robots/{robot_id}"


def build_stage(app):
    from pxr import Gf, UsdGeom
    import omni.usd

    if not PARKING_USD.is_file():
        avail = sorted(p.name for p in PARKING_USD.parent.glob("parking_environment*.usd"))
        raise RuntimeError(f"주차장 에셋 없음: {PARKING_USD}\n  있는 것: {avail}")

    ctx = omni.usd.get_context()
    ctx.new_stage()
    stage = ctx.get_stage()
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.y)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    stage.SetTimeCodesPerSecond(RENDER_HZ)
    stage.GetRootLayer().subLayerPaths.append(str(PARKING_USD))
    world = stage.GetPrimAtPath("/World")
    if not world or not world.IsValid():
        raise RuntimeError(f"{PARKING_USD.name} 에서 /World 를 찾지 못했습니다.")
    stage.SetDefaultPrim(world)
    _apply_physics(stage)
    for _ in range(30):
        app.update()

    n_lidar = _disable_sensors(stage)

    markers = read_markers(stage)
    problems = sm.validate_markers(
        [{"serves": s, "z": m["z"]} for s, m in markers.items()])
    if problems:
        raise RuntimeError("마커 좌표 규약 위반:\n  " + "\n  ".join(problems))
    print(f"V4_MARKERS_OK count={len(markers)}", flush=True)

    UsdGeom.Xform.Define(stage, "/World/Robots")
    for robot_id in sm.ROBOTS:
        dock_serves = sm.ROBOT_DOCK_MARKER[robot_id]
        if dock_serves not in markers:
            raise RuntimeError(f"도크 마커 {dock_serves} 를 v4 에서 찾지 못함")
        m = markers[dock_serves]
        prim = stage.DefinePrim(robot_prim_path(robot_id), "Xform")
        prim.GetReferences().AddReference(str(ROBOT_USD))
        xf = UsdGeom.Xformable(prim)
        xf.ClearXformOpOrder()
        xf.AddTranslateOp().Set(Gf.Vec3d(m["x"], ROBOT_SPAWN_Y, m["z"]))
        xf.AddRotateXOp().Set(-90.0)          # Z-up 에셋 -> Y-up 스테이지
    for _ in range(30):
        app.update()

    placed = {r: sm.ROBOT_DOCK_MARKER[r] for r in sm.ROBOTS}
    print(f"V4_STAGE_READY robots={placed} disabled_lidar={n_lidar} "
          f"render={RENDER_WIDTH}x{RENDER_HEIGHT}@{RENDER_HZ:.0f}Hz "
          f"physics={PHYSICS_HZ:.0f}Hz", flush=True)
    return stage


def find_front_camera(stage, robot_id):
    """로봇 서브트리에서 전방 카메라 prim 경로를 찾는다.

    에셋에 카메라가 4대 있으므로 이름으로 전방을 고른다. 후보가 없으면
    조용히 넘어가지 않고 실패한다.
    """
    from pxr import Usd
    root = stage.GetPrimAtPath(robot_prim_path(robot_id))
    cams = [p for p in Usd.PrimRange(root)
            if p.GetTypeName() == "Camera"]
    if not cams:
        raise RuntimeError(f"{robot_id}: 카메라 prim 을 찾지 못했습니다")
    for p in cams:
        if "front" in p.GetName().lower():
            return str(p.GetPath())
    return str(cams[0].GetPath())


def attach_camera_graph(robot_id, cam_path, width=640, height=480):
    """C++ OmniGraph 로 image_raw + camera_info 를 발행한다.

    Python rclpy 로 이미지를 퍼블리시하면 Isaac 루프가 죽는다(ARUCO_PLAN 전제).
    camera_info 는 ROS2CameraHelper 의 type 이 아니라 별도 ROS2CameraInfoHelper 노드다
    (DEBUG_LOG 2026-07-21). 노드 타입·속성명은 설치된 Isaac Sim 5.1
    (isaacsim.core.nodes / isaacsim.ros2.bridge 의 .ogn 문서)로 확인했다.
    """
    import omni.graph.core as og
    ns = f"/robot_{robot_id}"
    og.Controller.edit(
        {"graph_path": f"/Graphs/cam_{robot_id}", "evaluator_name": "push"},
        {
            og.Controller.Keys.CREATE_NODES: [
                ("tick", "omni.graph.action.OnPlaybackTick"),
                ("render", "isaacsim.core.nodes.IsaacCreateRenderProduct"),
                ("rgb", "isaacsim.ros2.bridge.ROS2CameraHelper"),
                ("info", "isaacsim.ros2.bridge.ROS2CameraInfoHelper"),
            ],
            og.Controller.Keys.CONNECT: [
                ("tick.outputs:tick", "render.inputs:execIn"),
                ("render.outputs:execOut", "rgb.inputs:execIn"),
                ("render.outputs:execOut", "info.inputs:execIn"),
                ("render.outputs:renderProductPath", "rgb.inputs:renderProductPath"),
                ("render.outputs:renderProductPath", "info.inputs:renderProductPath"),
            ],
            og.Controller.Keys.SET_VALUES: [
                ("render.inputs:cameraPrim", [cam_path]),
                ("render.inputs:width", width),
                ("render.inputs:height", height),
                ("rgb.inputs:type", "rgb"),
                ("rgb.inputs:topicName", f"{ns}/image_raw"),
                ("rgb.inputs:frameId", f"robot_{robot_id}/cam"),
                ("info.inputs:topicName", f"{ns}/camera_info"),
                ("info.inputs:frameId", f"robot_{robot_id}/cam"),
            ],
        },
    )


def main():
    _restart_with_isaac_python()
    from isaacsim import SimulationApp
    headless = "--gui" not in sys.argv[1:]
    app = SimulationApp({
        "headless": headless,
        "width": RENDER_WIDTH,
        "height": RENDER_HEIGHT,
        "disable_viewport_updates": headless,
    })
    from isaacsim.core.utils.extensions import enable_extension
    enable_extension("isaacsim.ros2.bridge")
    for _ in range(12):
        app.update()
    import numpy as np
    import omni.timeline
    from isaacsim.core.prims import Articulation

    stage = build_stage(app)
    timeline = omni.timeline.get_timeline_interface()
    timeline.play()
    for _ in range(30):
        app.update()

    arts = {}
    for robot_id in sm.ROBOTS:
        art = Articulation(f"{robot_prim_path(robot_id)}/base_link")
        art.initialize()
        arts[robot_id] = art

    sys.path.insert(0, str(WORK_DIR))
    from mecanum_drive import configure_hub_drives
    for robot_id in sm.ROBOTS:
        configure_hub_drives(stage, f"{robot_prim_path(robot_id)}/joints")

    from mecanum_drive import WHEEL_JOINTS, cmd_vel_from_wheel_velocities
    from wheel_odometry import WheelOdometry

    wheel_idx = {r: {w: arts[r].dof_names.index(j) for w, j in WHEEL_JOINTS.items()}
                 for r in arts}

    def read_wheel_twist(art, idx):
        """휠 관절 '각속도'로부터 로봇 로컬 twist 를 복원한다.

        각도 차분을 쓰면 연속 회전에서 wrap 되어 폭주한다(DEBUG_LOG 2026-07-21).
        """
        vel = np.asarray(art.get_joint_velocities()).reshape(-1)
        omegas = {w: float(vel[i]) for w, i in idx.items()}
        return cmd_vel_from_wheel_velocities(omegas)

    def gt_pose_xz_yaw(art):
        """계측(채점) 전용 GT. 제어 입력으로 쓰지 않는다."""
        pos, orn = art.get_world_poses()
        pos = np.asarray(pos).reshape(-1)[:3]
        w, x, y, z = (float(v) for v in np.asarray(orn).reshape(-1)[:4])
        fwd_x = 1.0 - 2.0 * (y * y + z * z)
        fwd_z = 2.0 * (x * z - w * y)
        return float(pos[0]), float(pos[2]), math.atan2(fwd_x, fwd_z)

    odom_mode = "wheel"
    for a in sys.argv[1:]:
        if a.startswith("--odom="):
            odom_mode = a.split("=", 1)[1]
    if odom_mode not in ("gt", "wheel"):
        raise SystemExit(f"--odom 은 gt 또는 wheel 이어야 합니다: {odom_mode!r}")

    odom = {}
    for r in sm.ROBOTS:
        gx, gz, gyaw = gt_pose_xz_yaw(arts[r])
        odom[r] = WheelOdometry(x=gx, z=gz, yaw=gyaw)   # 초기 자세만 GT 로 정렬
    print(f"V4_ODOM_MODE={odom_mode}", flush=True)

    n_cams = 0
    for a in sys.argv[1:]:
        if a.startswith("--cameras="):
            n_cams = int(a.split("=", 1)[1])
    if n_cams not in (0, 2, 4):
        raise SystemExit(f"--cameras 는 0, 2, 4 중 하나여야 합니다: {n_cams}")

    # 2대일 때는 팀당 리드에만 단다.
    cam_robots = {0: (), 2: ("entry_lead", "exit_lead"), 4: sm.ROBOTS}[n_cams]
    for r in cam_robots:
        attach_camera_graph(r, find_front_camera(stage, r))
    for _ in range(30):
        app.update()
    print(f"V4_CAMERAS n={n_cams} robots={list(cam_robots)}", flush=True)

    probe = None
    for a in sys.argv[1:]:
        if a.startswith("--probe="):
            probe = a.split("=", 1)[1]

    if probe == "C":
        import time as _time
        import v4_probes as vp
        for _ in range(120):                 # 워밍업
            app.update()
        t_wall = _time.monotonic()
        t_sim = timeline.get_current_time()
        for _ in range(600):
            app.update()
        rtf = ((timeline.get_current_time() - t_sim)
               / max(_time.monotonic() - t_wall, 1e-6))
        path = vp.write_report(f"probe_c_rtf_{n_cams}cam", {
            "cameras": n_cams, "robots": list(cam_robots), "rtf": rtf})
        print(f"PROBE_C_RESULT cameras={n_cams} rtf={rtf:.3f} report={path.name}",
              flush=True)
        if headless:
            app.close()
            return

    if probe == "A":
        # Probe A (외부 검출형): Isaac 은 카메라 영상만 발행하고, 검출·판정은
        # 별도 ROS 노드(probe_a_detector_node)가 시스템 cv2 로 한다. 실제 배포
        # 파이프라인 그대로다(배포엔 Isaac 이 없고 외부 노드가 검출). 러너는
        # 로봇을 마커 앞 거리를 바꿔가며 세우고 현재 거리를 /probe_a/state 로 알린다.
        import json
        from pxr import Gf, UsdGeom
        BRIDGE_RCLPY_A = Path(
            "/home/rokey/dev_ws/isaac_sim/isaacsim/_build/linux-x86_64/release"
            "/exts/isaacsim.ros2.bridge/humble/rclpy")
        if str(BRIDGE_RCLPY_A) not in sys.path:
            sys.path.insert(0, str(BRIDGE_RCLPY_A))
        import rclpy
        from std_msgs.msg import String as RosString
        if not rclpy.ok():
            rclpy.init()
        pa_node = rclpy.create_node("probe_a_bringup")
        state_pub = pa_node.create_publisher(RosString, "/probe_a/state", 10)

        hold_sec = 3.0
        cam_h = None
        loops = 0                       # 0 = 무한(외부 노드가 볼 때까지). 헤드리스는 1회.
        for a in sys.argv[1:]:
            if a.startswith("--hold-sec="):
                hold_sec = float(a.split("=", 1)[1])
            if a.startswith("--cam-height="):
                cam_h = float(a.split("=", 1)[1])
            if a.startswith("--probe-a-loops="):
                loops = int(a.split("=", 1)[1])

        target = "entry_lead"
        art = arts[target]
        markers = read_markers(stage)
        ref = markers[sm.ROBOT_DOCK_MARKER[target]]      # 대상 도크 마커(id/kind)

        # 도크 마커는 aruco:position(도크 중심)과 실제 데칼 위치가 z 로 0.7m 어긋난다.
        # 카메라가 보는 건 데칼이므로 배치는 marker_visual_center() 실좌표를 쓴다.
        mx, mz = marker_visual_center(stage, sm.ROBOT_DOCK_MARKER[target])
        print(f"PROBE_A_GEOMETRY attr_xz=({ref['x']:.3f},{ref['z']:.3f}) "
              f"visual_xz=({mx:.3f},{mz:.3f}) target_marker_id={ref['id']}", flush=True)

        cam_path = find_front_camera(stage, target)
        cam_prim = stage.GetPrimAtPath(cam_path)
        cam_xf = UsdGeom.Xformable(cam_prim)

        # 스폰 자세(로봇 루트 AddRotateXOp(-90))를 그대로 재사용한다. 카메라 정면(-Z)은
        # 월드 +X 를 30도 아래로 본다. "마커 앞 d 미터"는 x 축으로 재 로봇 중심을 mx-d 에 둔다.
        _, spawn_orn = art.get_world_poses()
        spawn_orn = np.asarray(spawn_orn).reshape(-1)[:4].copy()

        # ---- 카메라 높이 오버라이드 ---- (월드 Y dy 를 카메라 로컬로 역변환)
        if cam_h is not None:
            dy_world = cam_h - 0.09
            mat_before = cam_xf.ComputeLocalToWorldTransform(timeline.get_current_time())
            local_delta = mat_before.GetInverse().TransformDir(Gf.Vec3d(0.0, dy_world, 0.0))
            cam_xf.AddTranslateOp(
                UsdGeom.XformOp.PrecisionDouble, "camHeightProbe").Set(local_delta)
            for _ in range(3):
                app.update()
            mat_after = cam_xf.ComputeLocalToWorldTransform(timeline.get_current_time())
            actual_dy = (mat_after.ExtractTranslation()[1]
                         - mat_before.ExtractTranslation()[1])
            print(f"PROBE_A_CAM_HEIGHT requested_dy={dy_world:.4f} "
                  f"actual_world_dy={actual_dy:.4f}", flush=True)

        # 카메라 영상을 C++ OmniGraph 로 발행(image_raw + camera_info). 검출은 외부.
        attach_camera_graph(target, cam_path)
        for _ in range(30):
            app.update()
        print(f"PROBE_A_PUBLISHING image=/robot_{target}/image_raw "
              f"info=/robot_{target}/camera_info state=/probe_a/state "
              f"domain={os.environ.get('ROS_DOMAIN_ID','0')} "
              f"target_marker_id={ref['id']}", flush=True)

        def _publish_state(d, phase):
            msg = RosString()
            msg.data = json.dumps({
                "distance_m": round(float(d), 3),
                "marker_id": int(ref["id"]),
                "marker_serves": sm.ROBOT_DOCK_MARKER[target],
                "camera_height_m": cam_h or 0.09,
                "phase": phase})
            state_pub.publish(msg)
            rclpy.spin_once(pa_node, timeout_sec=0.0)

        def _hold(d, phase, frames):
            for _ in range(frames):
                app.update()
                _publish_state(d, phase)

        loop_i = 0
        while app.is_running():
            for i in range(16):
                d = 0.6 + 0.1 * i                # 마커 앞 0.6~2.1 m
                art.set_world_poses(np.array([[mx - d, ROBOT_SPAWN_Y, mz]]),
                                    np.array([spawn_orn]))
                _hold(d, "sweep", int(hold_sec * RENDER_HZ))
            loop_i += 1
            if (loops and loop_i >= loops) or (headless and loops == 0):
                break
        # 스윕 종료 알림을 잠깐 발행(외부 노드가 요약을 낼 수 있게)
        for _ in range(int(2.0 * RENDER_HZ)):
            app.update()
            _publish_state(0.0, "done")
        if headless:
            app.close()
            return
        # GUI: 창을 닫을 때까지 done 을 계속 발행하며 살아있게 둔다.
        while app.is_running():
            app.update()
            _publish_state(0.0, "done")
        app.close()
        return

    BRIDGE_RCLPY = Path("/home/rokey/dev_ws/isaac_sim/isaacsim/_build/linux-x86_64/release"
                        "/exts/isaacsim.ros2.bridge/humble/rclpy")
    if str(BRIDGE_RCLPY) not in sys.path:
        sys.path.insert(0, str(BRIDGE_RCLPY))
    import rclpy
    from nav_msgs.msg import Odometry
    if not rclpy.ok():
        rclpy.init()
    ros_node = rclpy.create_node("parking_v4_runner")
    odom_pub = {r: ros_node.create_publisher(Odometry, f"/robot_{r}/odom", 10)
                for r in sm.ROBOTS}

    def _active_robots():
        """씬에 살아있는 로봇만. probe B 가 대상 외 로봇을 비활성화해도
        오도메트리 루프가 죽은 프림을 건드리지 않게 한다."""
        return [r for r in sm.ROBOTS
                if stage.GetPrimAtPath(robot_prim_path(r)).IsActive()]

    def publish_odom():
        """--odom 모드에 따라 휠 오도메트리 또는 GT 를 발행한다."""
        for r in _active_robots():
            if odom_mode == "wheel":
                px, pz, pyaw = odom[r].x, odom[r].z, odom[r].yaw
            else:
                px, pz, pyaw = gt_pose_xz_yaw(arts[r])
            msg = Odometry()
            msg.header.stamp = ros_node.get_clock().now().to_msg()
            msg.header.frame_id = "map"
            msg.child_frame_id = f"robot_{r}/base_link"
            msg.pose.pose.position.x = float(px)
            msg.pose.pose.position.z = float(pz)
            msg.pose.pose.orientation.z = math.sin(pyaw * 0.5)
            msg.pose.pose.orientation.w = math.cos(pyaw * 0.5)
            odom_pub[r].publish(msg)

    def step_odometry(dt):
        """휠 각속도를 읽어 각 로봇 오도메트리를 적분한다."""
        for r in _active_robots():
            vx, vy, wz = read_wheel_twist(arts[r], wheel_idx[r])
            odom[r].update(vx, vy, wz, dt)

    if "--headless-test" in sys.argv[1:]:
        def _p(a):
            return np.asarray(a.get_world_poses()[0]).reshape(-1)[:3]
        p0 = {k: _p(a) for k, a in arts.items()}
        for _ in range(180):
            app.update()
        p1 = {k: _p(a) for k, a in arts.items()}
        disp = {k: float(np.linalg.norm(p1[k] - p0[k])) for k in arts}
        ok = all(d < 0.35 for d in disp.values())
        print(f"V4_PHYSICS_TEST={'PASS' if ok else 'FAIL'} "
              f"robot_disp={ {k: round(v, 4) for k, v in disp.items()} }", flush=True)
        app.close()
        return

    if probe == "B":
        import v4_probes as vp
        from mecanum_drive import slew_twist, wheel_velocities_from_cmd_vel

        target = sm.ROBOTS[0]                      # entry_lead 한 대로 측정
        art = arts[target]
        idx = wheel_idx[target]
        vel_buf = np.zeros(np.asarray(art.get_joint_positions()).reshape(-1).shape,
                           dtype=np.float32)

        # 사용자 요청: 실험하는 로봇만 보이게. 나머지 3대를 씬에서 비활성화하면
        # 렌더와 물리에서 함께 빠지므로, 개루프 주행 중 옆 도크 로봇과 부딪히던
        # 현상도 사라진다(측정 대상은 어차피 entry_lead 하나뿐이다).
        hidden = []
        for r in sm.ROBOTS:
            if r == target:
                continue
            p = stage.GetPrimAtPath(robot_prim_path(r))
            if p and p.IsValid():
                p.SetActive(False)
                hidden.append(r)
        for _ in range(5):
            app.update()
        print(f"PROBE_B_HIDDEN robots={hidden}", flush=True)

        def drive(tw, steps):
            cur = (0.0, 0.0, 0.0)
            prev = timeline.get_current_time()
            gt_pts, od_pts = [], []
            for _ in range(steps):
                app.update()
                now = timeline.get_current_time()
                dt = min(0.1, max(0.0, now - prev))
                prev = now
                cur = slew_twist(cur, tw, dt, linear_accel=LINEAR_ACCEL,
                                 linear_decel=LINEAR_DECEL, angular_accel=ANGULAR_ACCEL)
                omegas = wheel_velocities_from_cmd_vel(*cur)
                vel_buf[...] = 0.0
                for w, om in omegas.items():
                    vel_buf[idx[w]] = om
                art.set_joint_velocity_targets(vel_buf)
                vx, vy, wz = read_wheel_twist(art, idx)
                odom[target].update(vx, vy, wz, dt)
                gx, gz, _ = gt_pose_xz_yaw(art)
                gt_pts.append((gx, gz))
                od_pts.append((odom[target].x, odom[target].z))
            return gt_pts, od_pts

        # 측정 시작 직전에 정착(settle) 프레임 수를 명시적으로 통제한다.
        # 드리프트는 초기 settle 정도에 매우 민감하다: 예전에는 카메라 부착
        # 루프(--cameras=0 이어도 무조건 30프레임 app.update() 를 도는 코드,
        # 이 함수 앞부분)가 우연히 로봇을 더 정착시켜 drift 19.65% -> 2.69%
        # 로 결과가 7.3배 달라진 적이 있다. 측정과 무관한 코드 변경이 결과를
        # 바꾸면 안 되므로, probe B 는 여기서 자체적으로 PROBE_SETTLE_FRAMES
        # 만큼 0 twist 로 정착시킨 뒤에만 측정을 시작한다(결과는 버린다).
        drive((0.0, 0.0, 0.0), PROBE_SETTLE_FRAMES)

        gt_all, od_all = [], []
        legs = []
        gx0, gz0, _ = gt_pose_xz_yaw(art)
        prev_gt = (gx0, gz0)
        prev_od = (odom[target].x, odom[target].z)
        for name, tw, steps in (
            ("forward", (0.35, 0.0, 0.0), 420),
            (None, (0.0, 0.0, 0.0), 60),
            ("strafe", (0.0, 0.35, 0.0), 420),
            (None, (0.0, 0.0, 0.0), 60),
        ):
            g, o = drive(tw, steps)
            gt_all += g
            od_all += o
            if name is not None and g:
                leg_gt_len = sum(math.hypot(g[i + 1][0] - g[i][0],
                                            g[i + 1][1] - g[i][1])
                                 for i in range(len(g) - 1))
                # leg 오차는 절대 종점 오차가 아니라 이 leg 동안 (GT - odom)
                # 괴리가 "얼마나 변했는지" 로 잰다. 그래야 각 leg 가 이전
                # leg 의 누적 오차에 오염되지 않고 독립적으로 의미를 갖는다.
                disc_start = (prev_gt[0] - prev_od[0], prev_gt[1] - prev_od[1])
                disc_end = (g[-1][0] - o[-1][0], g[-1][1] - o[-1][1])
                leg_err = math.hypot(disc_end[0] - disc_start[0],
                                     disc_end[1] - disc_start[1])
                leg_rate = (leg_err / leg_gt_len * 100.0) if leg_gt_len > 1e-6 else 0.0
                legs.append({"name": name, "gt_len_m": leg_gt_len,
                             "err_m": leg_err, "drift_pct": leg_rate})
            if g:
                prev_gt = g[-1]
                prev_od = o[-1]

        gx, gz = gt_all[-1]
        ox, oz = od_all[-1]
        err = math.hypot(ox - gx, oz - gz)
        gt_len = sum(math.hypot(gt_all[i + 1][0] - gt_all[i][0],
                                gt_all[i + 1][1] - gt_all[i][1])
                     for i in range(len(gt_all) - 1))
        rate = (err / gt_len * 100.0) if gt_len > 1e-6 else 0.0

        vp.draw_trail(stage, "/World/ProbeB/GT", gt_all, vp.WHITE)
        vp.draw_trail(stage, "/World/ProbeB/Odom", od_all, vp.YELLOW, y=0.03)
        path = vp.write_report("probe_b_odom_drift", {
            "robot": target, "gt_path_len_m": gt_len,
            "final_error_m": err, "drift_rate_pct": rate,
            "gt_end": [gx, gz], "odom_end": [ox, oz],
            "legs": legs,
        })
        legs_str = " ".join(f"{l['name']}={l['drift_pct']:.2f}%" for l in legs)
        print(f"PROBE_B_RESULT gt_len={gt_len:.3f}m final_err={err:.3f}m "
              f"drift={rate:.2f}% legs=[{legs_str}] report={path.name}", flush=True)
        if headless:
            app.close()
            return

    prev_sim = timeline.get_current_time()
    while app.is_running():
        app.update()
        now_sim = timeline.get_current_time()
        dt = min(0.1, max(0.0, now_sim - prev_sim))
        prev_sim = now_sim
        rclpy.spin_once(ros_node, timeout_sec=0.0)
        step_odometry(dt)
        publish_odom()
    app.close()


if __name__ == "__main__":
    main()
