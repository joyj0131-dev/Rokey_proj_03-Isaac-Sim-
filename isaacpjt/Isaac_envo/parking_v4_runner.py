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

import numpy as np

WORK_DIR = Path(__file__).resolve().parent
REPO_ROOT = WORK_DIR.parent.parent
PARKING_USD = WORK_DIR / "parking" / "parking_environment_v4.usd"
ROBOT_USD = (WORK_DIR.parent / "hwia_parking_robot_final_caster_package"
             / "hwia_4cam_mecha_roller_lowered.usd")
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

    from pxr import Usd
    _r0 = sm.ROBOTS[0]
    _cams = [p for p in Usd.PrimRange(stage.GetPrimAtPath(robot_prim_path(_r0)))
             if p.GetTypeName() == "Camera"]
    _has_qr = any("qr_down" in str(p.GetPath()).lower() for p in _cams)
    print(f"V4_CAMERAS_COUNT robot={_r0} n={len(_cams)} qr_down={_has_qr}", flush=True)
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


def find_rear_camera(stage, robot_id):
    """로봇 서브트리에서 후방 카메라 prim 경로를 찾는다(이름/경로에 'rear')."""
    from pxr import Usd
    root = stage.GetPrimAtPath(robot_prim_path(robot_id))
    cams = [p for p in Usd.PrimRange(root) if p.GetTypeName() == "Camera"]
    for p in cams:
        if "rear" in str(p.GetPath()).lower():
            return str(p.GetPath())
    raise RuntimeError(f"{robot_id}: 후방 카메라 prim 을 찾지 못했습니다")


def attach_camera_graph(robot_id, cam_path, role="front", width=640, height=480):
    """C++ OmniGraph 로 image_raw + camera_info 를 발행한다.

    Python rclpy 로 이미지를 퍼블리시하면 Isaac 루프가 죽는다(ARUCO_PLAN 전제).
    camera_info 는 ROS2CameraHelper 의 type 이 아니라 별도 ROS2CameraInfoHelper 노드다
    (DEBUG_LOG 2026-07-21). 노드 타입·속성명은 설치된 Isaac Sim 5.1
    (isaacsim.core.nodes / isaacsim.ros2.bridge 의 .ogn 문서)로 확인했다.

    role 로 네임스페이스/그래프 경로를 분리해 같은 로봇의 전방·후방 카메라를
    동시에(별개 토픽/그래프로) 발행할 수 있다(기본 "front").
    """
    import omni.graph.core as og
    ns = f"/robot_{robot_id}/{role}"
    og.Controller.edit(
        {"graph_path": f"/Graphs/cam_{robot_id}_{role}", "evaluator_name": "push"},
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
                ("rgb.inputs:frameId", f"robot_{robot_id}/{role}_cam"),
                ("info.inputs:topicName", f"{ns}/camera_info"),
                ("info.inputs:frameId", f"robot_{robot_id}/{role}_cam"),
            ],
        },
    )


# ---- M5/FUSE 공용 헬퍼 (카메라 셋업·보정·검출·측위) ----
# M5 분기가 인라인으로 갖고 있던 로직을 모듈 함수로 추출한 것이다. --probe=FUSE 가
# 주행 중에 같은 셋업/보정/검출/측위를 재사용한다. 계산·값·순서는 M5 인라인과 동일.
def fuse_camera_setup(stage, timeline, app, target, cam_h):
    """카메라 높이 오버라이드 + rgb annotator + K/detector/marker_map 컨텍스트."""
    import json
    from pxr import Gf, UsdGeom
    import omni.replicator.core as rep
    sys.path.insert(0, str(REPO_ROOT / "src" / "parkbot_aruco"))
    from parkbot_aruco import aruco_pose
    from parkbot_aruco import marker_localizer as ML

    ref_serves = sm.ROBOT_DOCK_MARKER[target]
    markers = read_markers(stage)
    ref_id = markers[ref_serves]["id"]
    mx, mz = marker_visual_center(stage, ref_serves)

    map_path = REPO_ROOT / "src" / "parkbot_aruco" / "data" / "marker_map_v4.json"
    mm_json = json.loads(map_path.read_text(encoding="utf-8"))
    marker_map = ML.MarkerMap.from_json(mm_json, align_yaw_deg=0.0)
    code_size_m = float(mm_json["code_size_m"])
    detector = aruco_pose.make_detector(mm_json["dictionary"])

    art_path = robot_prim_path(target)
    cam_path = find_front_camera(stage, target)
    cam_prim = stage.GetPrimAtPath(cam_path)
    cam_xf = UsdGeom.Xformable(cam_prim)
    dy_world = cam_h - 0.09
    if abs(dy_world) > 1e-9:
        mb = cam_xf.ComputeLocalToWorldTransform(timeline.get_current_time())
        local_delta = mb.GetInverse().TransformDir(Gf.Vec3d(0.0, dy_world, 0.0))
        cam_xf.AddTranslateOp(UsdGeom.XformOp.PrecisionDouble, "camHeightFuse").Set(local_delta)
        for _ in range(3):
            app.update()

    rp = rep.create.render_product(cam_path, (640, 480))
    rgb_annot = rep.AnnotatorRegistry.get_annotator("rgb")
    rgb_annot.attach([rp])
    ucam = UsdGeom.Camera(cam_prim)
    focal = ucam.GetFocalLengthAttr().Get()
    haper = ucam.GetHorizontalApertureAttr().Get()
    vaper = ucam.GetVerticalApertureAttr().Get()
    K = np.array([[640.0 * focal / haper, 0.0, 320.0],
                  [0.0, 480.0 * focal / vaper, 240.0],
                  [0.0, 0.0, 1.0]], dtype=np.float64)
    return {"aruco_pose": aruco_pose, "ML": ML, "ref_id": ref_id,
            "mx": mx, "mz": mz, "marker_map": marker_map,
            "code_size_m": code_size_m, "detector": detector, "K": K,
            "dist": np.zeros((5, 1), dtype=np.float64), "cam_xf": cam_xf,
            "rgb_annot": rgb_annot}


def _quat_mul(q0, q1):
    w0, x0, y0, z0 = q0
    w1, x1, y1, z1 = q1
    return np.array([
        w1*w0 - x1*x0 - y1*y0 - z1*z0,
        w1*x0 + x1*w0 + y1*z0 - z1*y0,
        w1*y0 - x1*z0 + y1*w0 + z1*x0,
        w1*z0 + x1*y0 - y1*x0 + z1*w0])


def detect_at_pose(ctx, art, app, d, lat, yaw_deg, spawn_orn, settle=20):
    """로봇을 마커 앞 (d,lat,yaw) 에 놓고 렌더→검출, 대상 마커 pose|None."""
    import cv2
    mx, mz = ctx["mx"], ctx["mz"]
    base = np.array([[mx - d, ROBOT_SPAWN_Y, mz + lat]])
    if abs(yaw_deg) < 1e-9:
        orn = np.array([spawn_orn])
    else:
        half = math.radians(yaw_deg) * 0.5
        qy = np.array([math.cos(half), 0.0, math.sin(half), 0.0])
        orn = np.array([_quat_mul(spawn_orn, qy)])
    art.set_world_poses(base, orn)
    for _ in range(settle):
        app.update()
    frame = ctx["rgb_annot"].get_data()
    img = (np.asarray(frame)[:, :, :3] if frame is not None and len(frame) else None)
    if img is None:
        return None
    gray = cv2.cvtColor(np.ascontiguousarray(img), cv2.COLOR_RGB2GRAY)
    det = ctx["aruco_pose"].detect_and_estimate(
        gray, ctx["detector"], ctx["code_size_m"], ctx["K"], ctx["dist"])
    hit = [p for p in det if int(p.marker_id) == ctx["ref_id"]]
    return hit[0] if hit else None


def detect_current(ctx):
    """현재 렌더 프레임에서 대상 마커 pose|None (로봇을 옮기지 않는다, 주행 중용)."""
    import cv2
    frame = ctx["rgb_annot"].get_data()
    img = (np.asarray(frame)[:, :, :3] if frame is not None and len(frame) else None)
    if img is None:
        return None
    gray = cv2.cvtColor(np.ascontiguousarray(img), cv2.COLOR_RGB2GRAY)
    det = ctx["aruco_pose"].detect_and_estimate(
        gray, ctx["detector"], ctx["code_size_m"], ctx["K"], ctx["dist"])
    hit = [p for p in det if int(p.marker_id) == ctx["ref_id"]]
    return hit[0] if hit else None


def localize_pose(ctx, pose, T_base_cam):
    T_cm = ctx["ML"].rvec_tvec_to_T(pose.rvec, pose.tvec)
    return ctx["ML"].robot_pose_from_marker(ctx["ref_id"], T_cm, T_base_cam, ctx["marker_map"])


def calibrate_tbasecam(ctx, art, app, timeline, gt_fn, spawn_orn):
    """GT 로 광학 규약 후보를 스윕해 T_base_cam 확정. (T_base_cam, name, err)."""
    def usd_to_np(gf_m):
        m = np.array([[gf_m[i][j] for j in range(4)] for i in range(4)], dtype=np.float64)
        return m.T
    pose0 = detect_at_pose(ctx, art, app, 1.5, 0.0, 0.0, spawn_orn)
    if pose0 is None:
        raise RuntimeError("FUSE T_base_cam 보정 자세에서 마커 미검출")
    bpos, born = art.get_world_poses()
    bp = np.asarray(bpos).reshape(-1)[:3]
    bw, bx, by, bz = (float(v) for v in np.asarray(born).reshape(-1)[:4])

    def quat_to_R(w, x, y, z):
        return np.array([
            [1-2*(y*y+z*z), 2*(x*y-w*z),   2*(x*z+w*y)],
            [2*(x*y+w*z),   1-2*(x*x+z*z), 2*(y*z-w*x)],
            [2*(x*z-w*y),   2*(y*z+w*x),   1-2*(x*x+y*y)]], dtype=np.float64)
    T_world_base = np.eye(4)
    T_world_base[:3, :3] = quat_to_R(bw, bx, by, bz)
    T_world_base[:3, 3] = bp
    T_world_camusd = usd_to_np(ctx["cam_xf"].ComputeLocalToWorldTransform(
        timeline.get_current_time()))
    candidates = {"I": np.diag([1.0, 1.0, 1.0, 1.0]),
                  "X180": np.diag([1.0, -1.0, -1.0, 1.0]),
                  "Y180": np.diag([-1.0, 1.0, -1.0, 1.0]),
                  "Z180": np.diag([-1.0, -1.0, 1.0, 1.0])}
    gx, gz, gyaw = gt_fn(art)
    best_name, best_T, best_err = None, None, 1e9
    for name, C in candidates.items():
        T_base_cam = np.linalg.inv(T_world_base) @ T_world_camusd @ C
        fix = localize_pose(ctx, pose0, T_base_cam)
        if fix is None:
            continue
        e = math.hypot(fix.x - gx, fix.z - gz)
        if e < best_err:
            best_name, best_T, best_err = name, T_base_cam, e
    if best_T is None or best_err > 0.05:
        raise RuntimeError(f"FUSE T_base_cam 보정 실패(best_err={best_err:.4f})")
    return best_T, best_name, best_err


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

    # probe B 는 측정 대상 외 로봇을 화면·물리에서 뺀다(사용자 요청 + 개루프 주행 중
    # 옆 도크 로봇과의 충돌 제거). 반드시 timeline.play()/Articulation.initialize() '전에'
    # 비활성화한다 — 초기화(=PhysX 텐서 뷰 등록) 후에 SetActive(False)로 프림을 지우면
    # 텐서 뷰가 깨져 세그폴트가 난다(관측된 크래시). play 전에 지우면 PhysX 가 아예
    # 로드하지 않으므로 안전하다.
    probe = None
    for a in sys.argv[1:]:
        if a.startswith("--probe="):
            probe = a.split("=", 1)[1]
    # --mission= 은 --probe= 와 독립된 별도 플래그다("B" 값이 겹치는 건 우연 —
    # probe=B 는 휠 오도 드리프트 측정, mission=B 는 Mission Phase B 스켈레톤).
    mission = None
    for a in sys.argv[1:]:
        if a.startswith("--mission="):
            mission = a.split("=", 1)[1]
    if probe == "B":
        keep = sm.ROBOTS[0]
        hidden = []
        for r in sm.ROBOTS:
            if r == keep:
                continue
            p = stage.GetPrimAtPath(robot_prim_path(r))
            if p and p.IsValid():
                p.SetActive(False)
                hidden.append(r)
        print(f"PROBE_B_HIDDEN kept={keep} hidden={hidden}", flush=True)

    timeline = omni.timeline.get_timeline_interface()
    timeline.play()
    for _ in range(30):
        app.update()

    # 비활성화된 로봇은 아티큘레이션을 만들지 않는다(프림이 없으니 초기화도 불가).
    arts = {}
    for robot_id in sm.ROBOTS:
        if not stage.GetPrimAtPath(robot_prim_path(robot_id)).IsActive():
            continue
        art = Articulation(f"{robot_prim_path(robot_id)}/base_link")
        art.initialize()
        arts[robot_id] = art

    sys.path.insert(0, str(WORK_DIR))
    from mecanum_drive import configure_hub_drives
    for robot_id in arts:
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

    # ---- Task 2: 폐루프 주행 원시요소 (drive_to_pose / rotate_in_place) ----
    # --probe=FUSE(위 참조)가 인라인으로 하던 예측(휠오도)+보정(마커) 상보필터
    # 패턴을 일반화한다. FUSE 는 전진(+X_body)만 명령했지만, 여기서는
    # body_twist_toward 가 내는 전체 메카넘 (vx,vy,wz) 를 그대로 slew 해
    # 목표 자세(x,z,yaw)까지 몬다. 제어 입력은 항상 filt.pose()(융합 자세)만
    # 쓴다 — GT(gt_pose_xz_yaw)는 반환값의 err_*_gt(리포팅 전용)에서만 쓴다.
    # FUSE 자체는 건드리지 않는다(기존 probe 회귀 방지) — 이 두 함수는 신규다.
    from mecanum_drive import wheel_velocities_from_cmd_vel, slew_twist
    from parkbot_aruco.marker_localizer import PoseFilter
    from mission_control import body_twist_toward

    def drive_to_pose(ctx, art, idx, filt, T_base_cam, target_xzyaw, *,
                       max_steps=2000, pos_gain=0.8, yaw_gain=1.2,
                       max_lin=0.25, max_ang=0.6, pos_tol=0.03, yaw_tol=0.5):
        """목표 (x,z,yaw_deg) 까지 오도+마커 융합 폐루프로 주행.

        매 스텝: 예측(휠 관절 각속도 -> cmd_vel_from_wheel_velocities ->
        filt.predict_body) -> 보정(detect_current -> localize_pose ->
        filt.update, 첫 fix 는 filt.set_pose 로 초기화) -> 제어
        (body_twist_toward(filt.pose(), target_xzyaw) -> slew_twist ->
        wheel_velocities_from_cmd_vel -> set_joint_velocity_targets).
        tol 이내(done)로 판정되면 그 뒤로는 0 twist 를 명령해 정지를 기다리고
        (slew_twist 가 실제로 (0,0,0) 에 도달하면) 종료한다. max_steps 는
        안전 상한.

        반환 err_pos_gt/err_yaw_gt 는 종단(정지 후) 자세를 GT 와 비교한 값으로
        **리포팅 전용**이다 — 이 함수의 제어 로직은 filt.pose() 만 쓰고 GT 를
        전혀 참조하지 않는다.
        """
        vel_buf = np.zeros(np.asarray(art.get_joint_positions()).reshape(-1).shape,
                           dtype=np.float32)
        cur_tw = (0.0, 0.0, 0.0)
        prev = timeline.get_current_time()
        steps = 0
        reached = False
        stopping = False        # done 판정 이후 래치: 이후 잔차가 tol 밖으로 흔들려도 계속 정지시킨다.
        for _ in range(max_steps):
            app.update()
            steps += 1
            now = timeline.get_current_time()
            dt = min(0.1, max(0.0, now - prev)); prev = now

            # ---- 예측: 휠 오도(관절 각속도) -> 바디 twist -> predict_body ----
            vel = np.asarray(art.get_joint_velocities()).reshape(-1)
            wv = {w: float(vel[i]) for w, i in idx.items()}
            pvx, pvy, pwz = cmd_vel_from_wheel_velocities(wv)
            filt.predict_body(pvx, pvy, pwz, dt)

            # ---- 보정: 마커 검출 시 fix ----
            pose = detect_current(ctx)
            if pose is not None:
                fix = localize_pose(ctx, pose, T_base_cam)
                if fix is not None:
                    if filt.x is None:
                        filt.set_pose(fix.x, fix.z, fix.yaw_deg)   # 첫 fix 로 초기화(GT 아님)
                    else:
                        filt.update(fix)

            # ---- 제어: 융합 자세 -> 목표까지 body twist(body_twist_toward) ----
            fp = filt.pose()
            if fp is None:
                # 아직 융합 자세가 없다(시딩도, fix 도 없었음) — 안전하게 정지 유지.
                tvx, tvy, twz = 0.0, 0.0, 0.0
            elif not stopping:
                tvx, tvy, twz, done = body_twist_toward(
                    fp, target_xzyaw, pos_gain=pos_gain, yaw_gain=yaw_gain,
                    max_lin=max_lin, max_ang=max_ang, pos_tol=pos_tol, yaw_tol=yaw_tol)
                if done:
                    stopping = True
                    tvx, tvy, twz = 0.0, 0.0, 0.0
            else:
                tvx, tvy, twz = 0.0, 0.0, 0.0
            cur_tw = slew_twist(cur_tw, (tvx, tvy, twz), dt, linear_accel=LINEAR_ACCEL,
                                linear_decel=LINEAR_DECEL, angular_accel=ANGULAR_ACCEL)
            omegas = wheel_velocities_from_cmd_vel(*cur_tw)
            vel_buf[...] = 0.0
            for w, om in omegas.items():
                vel_buf[idx[w]] = om
            art.set_joint_velocity_targets(vel_buf)

            if stopping and cur_tw == (0.0, 0.0, 0.0):
                reached = True
                # 정지 후 몇 프레임 더 보정(FUSE 종단 처리와 동일 관례).
                for _ in range(30):
                    app.update()
                    pose = detect_current(ctx)
                    if pose is not None:
                        fix = localize_pose(ctx, pose, T_base_cam)
                        if fix is not None and filt.x is not None:
                            filt.update(fix)
                break

        gx, gz, gyaw = gt_pose_xz_yaw(art)
        fp = filt.pose()
        if fp is not None:
            err_pos_gt = math.hypot(fp[0] - gx, fp[1] - gz)
            err_yaw_gt = abs((fp[2] - math.degrees(gyaw) + 180.0) % 360.0 - 180.0)
        else:
            err_pos_gt, err_yaw_gt = float("nan"), float("nan")
        return {"reached": reached, "steps": steps, "err_pos_gt": err_pos_gt,
                "err_yaw_gt": err_yaw_gt, "final_filt": fp}

    def rotate_in_place(ctx, art, idx, filt, T_base_cam, target_yaw_deg, **kwargs):
        """제자리 회전: 위치는 현재 융합 x,z 그대로 두고 yaw 만 target_yaw_deg 로."""
        cx, cz, _ = filt.pose()
        return drive_to_pose(ctx, art, idx, filt, T_base_cam, (cx, cz, target_yaw_deg), **kwargs)

    odom_mode = "wheel"
    for a in sys.argv[1:]:
        if a.startswith("--odom="):
            odom_mode = a.split("=", 1)[1]
    if odom_mode not in ("gt", "wheel"):
        raise SystemExit(f"--odom 은 gt 또는 wheel 이어야 합니다: {odom_mode!r}")

    odom = {}
    for r in arts:
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
        attach_camera_graph(r, find_front_camera(stage, r), role="front")
    for _ in range(30):
        app.update()
    print(f"V4_CAMERAS n={n_cams} robots={list(cam_robots)}", flush=True)

    # probe 는 이미 build_stage 직후에 파싱했다(probe B 조기 비활성화 때문).

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
        attach_camera_graph(target, cam_path, role="front")
        for _ in range(30):
            app.update()
        print(f"PROBE_A_PUBLISHING image=/robot_{target}/front/image_raw "
              f"info=/robot_{target}/front/camera_info state=/probe_a/state "
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

    if probe == "REAR":
        # 후방 카메라 실동작: 로봇 뒤(−X 방향)에 마커가 오도록 배치하고 후방 카메라 발행.
        target = "entry_lead"
        cam_path = find_rear_camera(stage, target)
        ref_serves = sm.ROBOT_DOCK_MARKER[target]
        mx, mz = marker_visual_center(stage, ref_serves)
        # 후방 카메라는 −X 를 보므로, 마커가 뒤에 오도록 로봇 중심을 mx + d 에 둔다.
        art = arts[target]
        _, spawn_orn = art.get_world_poses()
        spawn_orn = np.asarray(spawn_orn).reshape(-1)[:4].copy()
        import json as _json
        BRIDGE = Path("/home/rokey/dev_ws/isaac_sim/isaacsim/_build/linux-x86_64/release"
                      "/exts/isaacsim.ros2.bridge/humble/rclpy")
        if str(BRIDGE) not in sys.path:
            sys.path.insert(0, str(BRIDGE))
        import rclpy
        from std_msgs.msg import String as RosString
        if not rclpy.ok():
            rclpy.init()
        pa = rclpy.create_node("rear_probe_bringup")
        state_pub = pa.create_publisher(RosString, "/probe_a/state", 10)
        attach_camera_graph(target, cam_path, role="rear")   # /robot_entry_lead/rear/image_raw 로 후방 발행
        for _ in range(30):
            app.update()
        ref_id = read_markers(stage)[ref_serves]["id"]
        print(f"REAR_PUBLISHING image=/robot_{target}/rear/image_raw target_marker_id={ref_id} "
              f"domain={os.environ.get('ROS_DOMAIN_ID','0')}", flush=True)
        loop = 0
        while app.is_running():
            for i in range(12):
                d = 0.6 + 0.1 * i
                art.set_world_poses(np.array([[mx + d, ROBOT_SPAWN_Y, mz]]),
                                    np.array([spawn_orn]))
                for _ in range(int(3.0 * (60.0 if headless else RENDER_HZ))):
                    app.update()
                    msg = RosString()
                    msg.data = _json.dumps({"distance_m": round(d, 3),
                                            "marker_id": int(ref_id), "phase": "sweep"})
                    state_pub.publish(msg)
                    rclpy.spin_once(pa, timeout_sec=0.0)
            loop += 1
            if headless and loop >= 1:
                break
        if headless:
            app.close()
            return
        while app.is_running():
            app.update()
        app.close()
        return

    if probe == "M5":
        # 마커 측위 정확도 관문. Isaac 안에서 렌더→검출→robot_pose_from_marker 로
        # 월드 자세를 복원해 GT 와 비교한다. T_base_cam(카메라 마운트)은 카메라를
        # 0.15m 로 올렸으므로 v1 기본값을 쓰면 틀린다 → GT 로 자동 보정한다.

        cam_h = 0.15
        for a in sys.argv[1:]:
            if a.startswith("--cam-height="):
                cam_h = float(a.split("=", 1)[1])

        n_frames = 1
        for a in sys.argv[1:]:
            if a.startswith("--m5-frames="):
                n_frames = max(1, int(a.split("=", 1)[1]))

        dmin, dmax = 1.4, 1.8
        for a in sys.argv[1:]:
            if a.startswith("--m5-dmin="):
                dmin = float(a.split("=", 1)[1])
            if a.startswith("--m5-dmax="):
                dmax = float(a.split("=", 1)[1])

        target = "entry_lead"
        art = arts[target]

        # yaw 회전 기준이 되는 스폰 자세. 카메라 높이 오버라이드의 app.update 전에
        # 잡아 리팩터 전 M5 와 동일한 값을 쓴다.
        _, spawn_orn = art.get_world_poses()
        spawn_orn = np.asarray(spawn_orn).reshape(-1)[:4].copy()

        # 카메라 셋업·검출 컨텍스트(높이 오버라이드 + rgb annotator + K + 지도/검출기/마커).
        # FUSE 와 공유하려고 모듈 함수로 추출했다.
        ctx = fuse_camera_setup(stage, timeline, app, target, cam_h)

        def capture_frames(d, lat, yaw_deg, k):
            """같은 자세에서 k 프레임을 잡아 각 프레임의 측위를 리스트로 돌려준다.

            프레임 사이에 app.update() 를 돌려 렌더가 갱신되게 한다. 정지 자세라
            렌더가 결정론적이면 프레임들이 거의 같아 융합 효과가 없다 — 그 경우
            single 과 fused 가 비슷하게 나오며, 그것이 정직한 결과다.
            """
            poses = []
            first = detect_at_pose(ctx, art, app, d, lat, yaw_deg, spawn_orn)  # 첫 프레임(자세 세팅 포함)
            gxx, gzz, gyy = gt_pose_xz_yaw(art)           # 이 자세의 GT
            if first is not None:
                poses.append(first)
            for _ in range(k - 1):
                for _ in range(2):
                    app.update()
                hit = detect_current(ctx)
                if hit is not None:
                    poses.append(hit)
            return poses, (gxx, gzz, gyy)

        def fuse_localize(poses, T_base_cam):
            """여러 프레임의 측위(RobotFix)를 융합: x,z 평균 + yaw 원형평균."""
            fixes = [localize_pose(ctx, p, T_base_cam) for p in poses]
            fixes = [f for f in fixes if f is not None]
            if not fixes:
                return None
            xs = sum(f.x for f in fixes) / len(fixes)
            zs = sum(f.z for f in fixes) / len(fixes)
            sy = sum(math.sin(math.radians(f.yaw_deg)) for f in fixes)
            cy = sum(math.cos(math.radians(f.yaw_deg)) for f in fixes)
            yaw = math.degrees(math.atan2(sy, cy))
            return xs, zs, yaw, fixes[0]      # 융합값 + 첫 프레임(단일 비교용)

        # ---- T_base_cam 자동 보정(모듈 함수: GT 로 광학 규약 후보 스윕) ----
        # 보정·검출·측위는 FUSE 와 공유하는 모듈 함수가 담당한다. 보정 실패 시에도
        # 리팩터 전과 같은 M5_TBASECAM_CAL FAIL 토큰·헤드리스 종료 동작을 유지한다.
        try:
            T_base_cam, best_name, best_err = calibrate_tbasecam(
                ctx, art, app, timeline, gt_pose_xz_yaw, spawn_orn)
        except RuntimeError as e:
            if "미검출" in str(e):
                print("M5_TBASECAM_CAL FAIL: 검증 자세에서 마커 미검출 — 기하 재검토", flush=True)
            else:
                print("M5_TBASECAM_CAL FAIL: 어떤 광학 규약도 5cm 안에 못 맞춤 — "
                      "usd_to_np 전치/규약 재검토 필요", flush=True)
            if headless:
                app.close()
            raise
        print(f"M5_TBASECAM_CAL best={best_name} verify_pos_err={best_err:.4f}m", flush=True)

        # ---- 정확도 스윕(단일+융합 동시 측정) ----
        def err_of(fx, fz, fyaw_deg, gt):
            gxx, gzz, gyy = gt
            ex, ez = fx - gxx, fz - gzz
            dyaw = (fyaw_deg - math.degrees(gyy) + 180.0) % 360.0 - 180.0
            return math.hypot(ex, ez), abs(dyaw), ex, ez, dyaw

        single_pos, single_yaw = [], []
        fused_pos, fused_yaw, fused_vec = [], [], []
        dists = []
        n_total = 0
        for d in (1.3, 1.4, 1.5, 1.6, 1.7, 1.8, 1.9):
            for lat in (-0.15, 0.0, 0.15):
                for yaw_deg in (-8.0, 0.0, 8.0):
                    n_total += 1
                    poses, gt = capture_frames(d, lat, yaw_deg, n_frames)
                    if not poses:
                        continue
                    fused = fuse_localize(poses, T_base_cam)
                    if fused is None:
                        continue
                    fx, fz, fyaw, f0 = fused
                    sp, sy, *_ = err_of(f0.x, f0.z, f0.yaw_deg, gt)   # 단일=첫 프레임
                    single_pos.append(sp); single_yaw.append(sy)
                    fp, fy, ex, ez, dyaw = err_of(fx, fz, fyaw, gt)   # 융합
                    fused_pos.append(fp); fused_yaw.append(fy)
                    fused_vec.append((ex, ez, dyaw))
                    dists.append(d)

        if not fused_pos:
            print("M5_RESULT FAIL: 검출 표본 0개", flush=True)
            if headless:
                app.close()
            raise RuntimeError("M5 검출 표본 0")

        def p95(a):
            b = sorted(a)
            return b[min(len(b) - 1, int(math.ceil(0.95 * len(b)) - 1))]

        def stats(errs):
            b = sorted(errs)
            return {"mean": sum(b) / len(b), "median": b[len(b) // 2],
                    "p95": p95(b), "max": b[-1]}

        cov = np.cov(np.array(fused_vec).T).tolist() if len(fused_vec) > 1 else None
        # fused_pos/fused_yaw 는 각 표본의 오차, dists 는 같은 순서의 거리
        def p95(a):
            b = sorted(a)
            return b[min(len(b) - 1, int(math.ceil(0.95 * len(b)) - 1))] if b else float("nan")

        in_win = [i for i, dd in enumerate(dists) if dmin <= dd <= dmax]
        win_pos = [fused_pos[i] for i in in_win]
        win_yaw = [fused_yaw[i] for i in in_win]

        full_pp, full_yp = p95(fused_pos), p95(fused_yaw)
        win_pp, win_yp = p95(win_pos), p95(win_yaw)
        ok = (len(win_pos) > 0 and win_pp <= 0.02 and win_yp <= 1.0)
        report = {
            "camera_height_m": cam_h, "frames": n_frames,
            "trusted_window": {"dmin": dmin, "dmax": dmax},
            "tbasecam_convention": best_name, "tbasecam_verify_err_m": best_err,
            "n_full": len(fused_pos), "n_window": len(win_pos), "n_total": n_total,
            "full": {"pos_err_m": stats(fused_pos), "yaw_err_deg": stats(fused_yaw)},
            "window": {"pos_err_m": stats(win_pos) if win_pos else None,
                       "yaw_err_deg": stats(win_yaw) if win_yaw else None},
            "cov_ex_ez_eyaw": cov,
        }
        import v4_probes as vp
        path = vp.write_report("m5_accuracy", report)
        print(f"M5_RESULT={'PASS' if ok else 'FAIL'} window=[{dmin},{dmax}]m "
              f"window_pos_p95={win_pp*100:.2f}cm window_yaw_p95={win_yp:.2f}deg "
              f"full_pos_p95={full_pp*100:.2f}cm full_yaw_p95={full_yp:.2f}deg "
              f"n_win={len(win_pos)}/{len(fused_pos)} report={path.name}", flush=True)
        if headless:
            app.close()
            return
        while app.is_running():
            app.update()
        app.close()
        return

    if probe == "FUSE":
        # 주행 접근 중 오도(예측)+마커(보정) 상보필터 융합 검증. 종단 수렴 오차로 판정.
        from pxr import UsdGeom
        from parkbot_aruco.marker_localizer import PoseFilter
        from mecanum_drive import (WHEEL_JOINTS, wheel_velocities_from_cmd_vel,
                                   cmd_vel_from_wheel_velocities, slew_twist)

        speed, d_start, d_end = 0.25, 2.1, 1.25
        pos_gain, yaw_gain = 0.5, 0.5
        for a in sys.argv[1:]:
            if a.startswith("--fuse-speed="):  speed = float(a.split("=", 1)[1])
            if a.startswith("--fuse-dstart="): d_start = float(a.split("=", 1)[1])
            if a.startswith("--fuse-dend="):   d_end = float(a.split("=", 1)[1])
            if a.startswith("--fuse-posgain="): pos_gain = float(a.split("=", 1)[1])
            if a.startswith("--fuse-yawgain="): yaw_gain = float(a.split("=", 1)[1])
        cam_h = 0.15
        for a in sys.argv[1:]:
            if a.startswith("--cam-height="): cam_h = float(a.split("=", 1)[1])

        target = "entry_lead"
        art = arts[target]
        idx = wheel_idx[target]
        ctx = fuse_camera_setup(stage, timeline, app, target, cam_h)
        _, spawn_orn = art.get_world_poses()
        spawn_orn = np.asarray(spawn_orn).reshape(-1)[:4].copy()
        T_base_cam, cal_name, cal_err = calibrate_tbasecam(
            ctx, art, app, timeline, gt_pose_xz_yaw, spawn_orn)
        print(f"FUSE_TBASECAM_CAL best={cal_name} verify_pos_err={cal_err:.4f}m", flush=True)

        # 로봇을 접근 시작점(마커 앞 d_start, 정면 자세)에 놓는다.
        mx, mz = ctx["mx"], ctx["mz"]
        art.set_world_poses(np.array([[mx - d_start, ROBOT_SPAWN_Y, mz]]),
                            np.array([spawn_orn]))
        for _ in range(30):
            app.update()

        vel_buf = np.zeros(np.asarray(art.get_joint_positions()).reshape(-1).shape,
                           dtype=np.float32)
        filt = PoseFilter(pos_gain=pos_gain, yaw_gain=yaw_gain)
        cur_tw = (0.0, 0.0, 0.0)
        prev = timeline.get_current_time()
        traj = []                     # (gt_x,gt_z,gt_yawdeg, f_x,f_z,f_yaw, single_x,single_z,single_yaw|None)
        max_steps = 2000
        for _ in range(max_steps):
            app.update()
            now = timeline.get_current_time()
            dt = min(0.1, max(0.0, now - prev)); prev = now
            # 전진(+X_body) 주행. 종단 도달하면 정지.
            gx, gz, gyaw = gt_pose_xz_yaw(art)
            d_now = mx - gx                     # 마커까지 남은 거리(월드 X)
            tgt = (speed, 0.0, 0.0) if d_now > d_end else (0.0, 0.0, 0.0)
            cur_tw = slew_twist(cur_tw, tgt, dt, linear_accel=LINEAR_ACCEL,
                                linear_decel=LINEAR_DECEL, angular_accel=ANGULAR_ACCEL)
            omegas = wheel_velocities_from_cmd_vel(*cur_tw)
            vel_buf[...] = 0.0
            for w, om in omegas.items():
                vel_buf[idx[w]] = om
            art.set_joint_velocity_targets(vel_buf)

            # ---- 예측: 휠 오도(관절 각속도) → 바디 twist → predict_body ----
            vel = np.asarray(art.get_joint_velocities()).reshape(-1)
            wv = {w: float(vel[i]) for w, i in idx.items()}
            vx, vy, wz = cmd_vel_from_wheel_velocities(wv)
            filt.predict_body(vx, vy, wz, dt)

            # ---- 보정: 마커 검출 시 fix ----
            pose = detect_current(ctx)
            single = None
            if pose is not None:
                fix = localize_pose(ctx, pose, T_base_cam)
                if fix is not None:
                    if filt.x is None:
                        filt.set_pose(fix.x, fix.z, fix.yaw_deg)   # 첫 fix 로 초기화(GT 아님)
                    else:
                        filt.update(fix)
                    single = (fix.x, fix.z, fix.yaw_deg)

            fp = filt.pose()
            if fp is not None:
                traj.append((gx, gz, math.degrees(gyaw), fp[0], fp[1], fp[2],
                             single[0] if single else None,
                             single[1] if single else None,
                             single[2] if single else None))
            if d_now <= d_end and cur_tw == (0.0, 0.0, 0.0):
                # 종단 도달 + 정지. 몇 프레임 더 보정 후 종료.
                for _ in range(30):
                    app.update()
                    pose = detect_current(ctx)
                    if pose is not None:
                        fix = localize_pose(ctx, pose, T_base_cam)
                        if fix is not None and filt.x is not None:
                            filt.update(fix)
                break

        if not traj or filt.x is None:
            print("FUSE_RESULT FAIL: 융합 표본 없음(마커 미검출)", flush=True)
            if headless: app.close()
            raise RuntimeError("FUSE 융합 표본 없음")

        # 종단(마지막) 지점 GT vs 융합
        gx, gz, gyaw = gt_pose_xz_yaw(art)
        fp = filt.pose()
        term_pos = math.hypot(fp[0] - gx, fp[1] - gz)
        term_yaw = abs((fp[2] - math.degrees(gyaw) + 180.0) % 360.0 - 180.0)

        # 궤적 p95(첫 fix 이후 융합 오차) — 참고
        def p95(a):
            b = sorted(a); return b[min(len(b)-1, int(math.ceil(0.95*len(b))-1))] if b else float("nan")
        fpos = [math.hypot(r[3]-r[0], r[4]-r[1]) for r in traj]
        fyaw = [abs((r[5]-r[2]+180.0) % 360.0 - 180.0) for r in traj]
        spos = [math.hypot(r[6]-r[0], r[7]-r[1]) for r in traj if r[6] is not None]

        ok = (term_pos <= 0.02) and (term_yaw <= 1.0)
        import v4_probes as vp
        report = {"camera_height_m": cam_h, "speed": speed,
                  "d_start": d_start, "d_end": d_end,
                  "pos_gain": pos_gain, "yaw_gain": yaw_gain,
                  "tbasecam_convention": cal_name, "tbasecam_verify_err_m": cal_err,
                  "n_traj": len(traj), "n_single": len(spos),
                  "terminal_pos_err_m": term_pos, "terminal_yaw_err_deg": term_yaw,
                  "fused_traj_pos_p95_m": p95(fpos), "fused_traj_yaw_p95_deg": p95(fyaw),
                  "single_traj_pos_p95_m": p95(spos) if spos else None}
        path = vp.write_report("fuse_approach", report)
        print(f"FUSE_RESULT={'PASS' if ok else 'FAIL'} "
              f"term_pos={term_pos*100:.2f}cm term_yaw={term_yaw:.2f}deg "
              f"fused_traj_p95={p95(fpos)*100:.2f}cm "
              f"single_traj_p95={(p95(spos)*100 if spos else float('nan')):.2f}cm "
              f"n={len(traj)} cal={cal_name} report={path.name}", flush=True)
        if headless:
            app.close(); return
        while app.is_running():
            app.update()
        app.close(); return

    if probe is None and mission == "B":
        # Mission Phase B, Task 2 스켈레톤: drive_to_pose 를 1 로봇 짧은 주행으로
        # 검증한다(안무 없음). 실제 후방 카메라 도크 검출 + 회전은 Task 3.
        # 여기서는 도크 마커 좌표(측량된 인프라, GT 아님)로 필터를 시딩하고
        # entry_lead 를 도크에서 +Z 로 1.0m 만 이동시켜 원시요소가 실제 바퀴로
        # 동작하는지만 확인한다.
        target = "entry_lead"
        art = arts[target]
        idx = wheel_idx[target]
        cam_h = 0.15
        for a in sys.argv[1:]:
            if a.startswith("--cam-height="):
                cam_h = float(a.split("=", 1)[1])

        # ---- 카메라 셋업 + T_base_cam 보정 (FUSE 와 동일한 모듈 함수 재사용) ----
        ctx = fuse_camera_setup(stage, timeline, app, target, cam_h)
        _, spawn_orn = art.get_world_poses()
        spawn_orn = np.asarray(spawn_orn).reshape(-1)[:4].copy()
        T_base_cam, cal_name, cal_err = calibrate_tbasecam(
            ctx, art, app, timeline, gt_pose_xz_yaw, spawn_orn)
        print(f"MISSIONB_TBASECAM_CAL best={cal_name} verify_pos_err={cal_err:.4f}m",
              flush=True)

        # ---- 도크 마커 좌표로 로봇을 도크에 두고 필터를 시딩(측량 좌표, GT 아님) ----
        # aruco:position 속성(read_markers) = 도크/스폰 좌표(build_stage 가 로봇을
        # 놓는 바로 그 좌표). marker_visual_center 의 데칼 좌표는 차선쪽으로 ~0.7m
        # 밀려 있어(위 fuse_camera_setup 주석 참조) 시딩에는 쓰면 안 된다.
        dock_serves = sm.ROBOT_DOCK_MARKER[target]
        dock = read_markers(stage)[dock_serves]
        dock_x, dock_z = dock["x"], dock["z"]
        # 스폰 자세는 로봇 루트에 고정 AddRotateXOp(-90) 만 적용된다(build_stage).
        # 이 회전은 로컬 +X 축을 그대로 두므로(회전축이 X 라 X 성분 불변) 로컬
        # +X(전방)가 그대로 월드 +X 로 나온다 -> yaw=atan2(fwd_x=1,fwd_z=0)=90도
        # (yaw=0 -> 월드 +Z 규약). 모든 로봇/도크에 공통인 상수라 GT 조회 없이 안다.
        spawn_yaw_deg = 90.0

        art.set_world_poses(np.array([[dock_x, ROBOT_SPAWN_Y, dock_z]]),
                            np.array([spawn_orn]))
        for _ in range(30):
            app.update()

        filt = PoseFilter()
        filt.set_pose(dock_x, dock_z, spawn_yaw_deg)   # 도크 마커 시딩(측량 좌표, GT 아님)

        result = drive_to_pose(ctx, art, idx, filt, T_base_cam,
                               (dock_x, dock_z + 1.0, spawn_yaw_deg))
        print(f"MISSIONB_DRIVE robot={target} reached={result['reached']} "
              f"err_pos={result['err_pos_gt']:.4f} err_yaw={result['err_yaw_gt']:.2f} "
              f"steps={result['steps']}", flush=True)
        # 진단 전용(요구 토큰 아님): 주행 중 마커 보정이 실제로 몇 번 들어갔는지.
        # +Z 도크 이탈은 전방 카메라가 마커를 옆으로 보내는 기하라 0 이어도
        # 정상(오도만으로 도달) — 정직성 게이트 리포팅용.
        print(f"MISSIONB_FIX_COUNT n_fix={filt.n_fix} n_pred={filt.n_pred}", flush=True)

        if headless:
            app.close(); return
        while app.is_running():
            app.update()
        app.close(); return

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
        art = arts[target]                         # 나머지 3대는 build_stage 직후
        idx = wheel_idx[target]                    # 이미 비활성화됨(PROBE_B_HIDDEN)
        vel_buf = np.zeros(np.asarray(art.get_joint_positions()).reshape(-1).shape,
                           dtype=np.float32)

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
        # 바꾸면 안 되므로, probe B 는 여기서 자체적으로 settle 프레임만큼
        # 0 twist 로 정착시킨 뒤에만 측정을 시작한다(결과는 버린다).
        # 재특성화용으로 --settle-frames 로 정착값을 스윕할 수 있다(부록 C).
        settle_frames = PROBE_SETTLE_FRAMES
        for a in sys.argv[1:]:
            if a.startswith("--settle-frames="):
                settle_frames = int(a.split("=", 1)[1])
        print(f"PROBE_B_SETTLE frames={settle_frames}", flush=True)
        drive((0.0, 0.0, 0.0), settle_frames)

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
