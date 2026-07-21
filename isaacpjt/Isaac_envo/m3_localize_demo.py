#!/usr/bin/env python3
"""M3 데모: 로봇이 마커 한 장을 봤을 때 나오는 측위 로그를 뽑는다.

흐름: 마커 씬에 로봇을 '알려진 월드 자세'로 세움 → 전방 카메라 1프레임 렌더
      → M2 검출(T_cam_marker) → M3 좌표변환(T_world_robot) → 오도메트리와 융합
      → 로그 출력 + 카메라가 본 마커 이미지 저장 → Isaac 정답(GT)과 대조.

이 스크립트는 ARUCO_PLAN 이 M4 앞에 두라고 한 '1-마커 캘리브레이션'이기도 하다:
marker→world 회전 규약을 손으로 맞히지 않고, align_yaw 를 {0,90,180,270} 스윕해
GT 와 맞는 값을 실측으로 고른다.

카메라 장착 T_base_cam 은 손으로 타이핑하지 않고 에셋에서 직접 읽는다.

실행:
    python3 m3_localize_demo.py                 # 기본 마커 A3, 헤드리스
    python3 m3_localize_demo.py --marker A5
    python3 m3_localize_demo.py --gui
"""

import json
import math
import os
import sys
from pathlib import Path

WORK_DIR = Path(__file__).resolve().parent
REPO = WORK_DIR.parents[1]
ROBOT_USD = (WORK_DIR.parent / "hwia_parking_robot_final_caster_package"
             / "hwia_depth_cam_mecha_roller.usd")
MARKER_STAGE = WORK_DIR / "parking" / "parking_environment_marker_preview.usd"
MAP_JSON = (WORK_DIR.parents[1] / "src" / "parkbot_aruco"
            / "data" / "marker_map.json")  # 지도는 ROS 패키지가 소유
OUT_IMG = WORK_DIR / "m3_marker_view.png"
REPORT = WORK_DIR / "m3_localize_report.json"
ISAAC_PYTHON = Path(
    "/home/rokey/dev_ws/isaac_sim/isaacsim/_build/linux-x86_64/release/python.sh")

# M3 순수 모듈을 소스 트리에서 직접 쓴다(colcon 설치 불필요).
sys.path.insert(0, str(REPO / "src" / "parkbot_aruco"))

ROBOT_XFORM = "/World/Robot"
ROBOT_ROOT = "/World/Robot/base_link"
CAM_FRONT = "/World/Robot/cam_front_link/depth_cam_front/Camera_Pseudo_Depth_Front"
CAM_RES = (640, 480)
STANDOFF = 1.25          # 마커 앞 이 거리에 로봇을 세운다(검출 창 한가운데)
WARMUP = 60


def _restart():
    if os.environ.get("CARB_APP_PATH"):
        return
    os.execv(str(ISAAC_PYTHON), [str(ISAAC_PYTHON), str(Path(__file__).resolve()),
                                 *sys.argv[1:]])


def _arg(name, default=None):
    a = sys.argv[1:]
    return a[a.index(name) + 1] if name in a and a.index(name) + 1 < len(a) else default


def _robot_matrix(Gf, tx, ty, tz):
    return Gf.Matrix4d(0, 0, 1, 0, 1, 0, 0, 0, 0, 1, 0, 0, tx, ty, tz, 1)


def _gf_to_np(M):
    """Gf.Matrix4d(행벡터 규약) → numpy 4x4(열벡터 규약). 전치한다."""
    import numpy as np
    return np.array([[M[i][j] for j in range(4)] for i in range(4)],
                    dtype=np.float64).T


def main():
    _restart()

    gui = "--gui" in sys.argv[1:]
    target_label = _arg("--marker", "A3")

    mm = json.loads(MAP_JSON.read_text(encoding="utf-8"))
    code_size = float(mm["code_size_m"])
    by_label = {m["label"]: m for m in mm["markers"]}
    if target_label not in by_label:
        raise SystemExit(f"마커 라벨 {target_label} 없음. 예: A3, A5")
    target = by_label[target_label]
    tgt_id = int(target["id"])

    # omni/isaac 모듈은 SimulationApp 생성 후에만 import 된다.
    from isaacsim import SimulationApp
    app = SimulationApp({"headless": not gui})
    try:
        import numpy as np
        import cv2
        import omni.timeline
        import omni.usd
        from isaacsim.core.api import World
        from isaacsim.sensors.camera import Camera
        from pxr import Gf, Usd, UsdGeom, UsdPhysics

        from parkbot_aruco import aruco_pose as AP
        from parkbot_aruco import marker_localizer as ML

        # 씬: 마커 환경 + 로봇을 마커 앞 STANDOFF 에 세운다(마커 쪽 = +Z).
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
        robot_x, robot_z = float(target["x"]), float(target["z"]) - STANDOFF
        UsdGeom.Xformable(r).ClearXformOpOrder()
        UsdGeom.Xformable(r).MakeMatrixXform().Set(
            _robot_matrix(Gf, robot_x, 0.0, robot_z))

        tmp = WORK_DIR / "_m3.usd"
        stage.GetRootLayer().Export(str(tmp))
        ctx = omni.usd.get_context()
        ctx.open_stage(str(tmp))
        for _ in range(30):
            app.update()
        live = ctx.get_stage()

        world = World(stage_units_in_meters=1.0, set_defaults=False)
        omni.timeline.get_timeline_interface().play()
        world.reset()

        cam = Camera(prim_path=CAM_FRONT, resolution=CAM_RES)
        cam.initialize()
        for _ in range(WARMUP):
            world.step(render=True)

        # ---- T_base_cam: 에셋에서 직접 읽는다(손 타이핑 금지) ----
        xc = UsdGeom.XformCache()
        T_w_base = _gf_to_np(xc.GetLocalToWorldTransform(
            live.GetPrimAtPath(ROBOT_ROOT)))
        T_w_usdcam = _gf_to_np(xc.GetLocalToWorldTransform(
            live.GetPrimAtPath(CAM_FRONT)))
        T_base_usdcam = np.linalg.inv(T_w_base) @ T_w_usdcam
        # USD 카메라(-Z 관측) → OpenCV 광학(+Z 관측): X는 유지, Y·Z 반전.
        Rx180 = np.diag([1.0, -1.0, -1.0, 1.0])
        T_base_cam = T_base_usdcam @ Rx180

        # ---- Ground truth 로봇 자세(base_link 월드) ----
        gt_x, gt_z = float(T_w_base[0, 3]), float(T_w_base[2, 3])
        gt_yaw = ML.yaw_from_R(T_w_base[:3, :3])

        # ---- 카메라 프레임 → M2 검출 ----
        rgba = cam.get_rgba()
        if rgba is None or getattr(rgba, "size", 0) == 0:
            raise RuntimeError("빈 프레임 — 렌더 실패")
        rgb = np.asarray(rgba[..., :3], dtype=np.uint8)
        gray = cv2.cvtColor(rgb, cv2.COLOR_RGB2GRAY)
        K = cam.get_intrinsics_matrix()
        dist = np.zeros((5, 1))
        detector = AP.make_detector(mm["dictionary"])
        poses = AP.detect_and_estimate(gray, detector, code_size, K, dist)
        hit = next((p for p in poses if p.marker_id == tgt_id), None)

        # 검출 이미지 저장
        annotated = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
        corners, ids = detector(gray)
        if ids is not None:
            cv2.aruco.drawDetectedMarkers(annotated, corners, ids)
        cv2.imwrite(str(OUT_IMG), annotated)

        if hit is None:
            print(f"[M3] 마커 {target_label}(ID {tgt_id}) 검출 실패 — "
                  f"검출된 것: {[p.marker_id for p in poses]}", flush=True)
            raise SystemExit(1)

        T_cam_marker = ML.rvec_tvec_to_T(hit.rvec, hit.tvec)

        # ---- align_yaw 캘리브레이션: GT 와 맞는 marker→world 회전을 실측으로 고른다 ----
        best = None
        for align in (0.0, 90.0, 180.0, 270.0):
            mp = ML.MarkerMap.from_json(mm, align_yaw_deg=align)
            fix = ML.robot_pose_from_marker(tgt_id, T_cam_marker, T_base_cam, mp)
            perr = math.hypot(fix.x - gt_x, fix.z - gt_z)
            yerr = abs(ML._wrap_deg(fix.yaw_deg - gt_yaw))
            score = perr + math.radians(yerr)
            if best is None or score < best[0]:
                best = (score, align, fix, perr, yerr)
        _score, align, fix, perr, yerr = best
        mp = ML.MarkerMap.from_json(mm, align_yaw_deg=align)

        # ---- 융합 데모: 오도메트리가 조금 어긋난 추정에서 시작 → 마커로 보정 ----
        odom_x, odom_z, odom_yaw = gt_x + 0.03, gt_z - 0.02, gt_yaw + 1.5
        filt = ML.PoseFilter(pos_gain=0.5, yaw_gain=0.5)
        filt.set_pose(odom_x, odom_z, odom_yaw)   # 추측항법 상태
        filt.update(fix)                          # 마커 관측으로 보정
        fx, fz, fyaw = filt.pose()

        t = hit.tvec.flatten()
        # 로그 출력 --------------------------------------------------------
        print("", flush=True)
        print("┌─ [aruco] 마커 검출 ─────────────────────────────", flush=True)
        print(f"│  마커 ID    : {tgt_id}  ({target_label}, {target['kind']})", flush=True)
        print(f"│  카메라 기준 : 앞 {t[2]:.3f} m,  옆 {t[0]:+.3f} m,  "
              f"아래 {t[1]:+.3f} m   (T_cam_marker)", flush=True)
        print(f"│  재투영오차  : {hit.reproj_err_px:.2f} px    "
              f"모호성 : {hit.ambiguity:.2f}", flush=True)
        print("├─ [localizer] 월드 측위 ─────────────────────────", flush=True)
        print(f"│  마커 월드   : ({target['x']:+.2f}, {target['z']:+.2f})   "
              f"[marker_map]", flush=True)
        print(f"│  align_yaw   : {align:.0f}°  (GT 대조로 실측 선택)", flush=True)
        print(f"│  → 로봇 월드 : x={fix.x:+.3f}  z={fix.z:+.3f}  "
              f"yaw={fix.yaw_deg:+.1f}°   (마커 단독)", flush=True)
        print("├─ [filter] 오도메트리 융합 ──────────────────────", flush=True)
        print(f"│  오도메트리  : x={odom_x:+.3f}  z={odom_z:+.3f}  "
              f"yaw={odom_yaw:+.1f}°  (추측항법)", flush=True)
        print(f"│  융합 결과   : x={fx:+.3f}  z={fz:+.3f}  yaw={fyaw:+.1f}°", flush=True)
        print("├─ [GT] Isaac 정답 ──────────────────────────────", flush=True)
        print(f"│  정답        : x={gt_x:+.3f}  z={gt_z:+.3f}  yaw={gt_yaw:+.1f}°", flush=True)
        print(f"│  마커측위오차: 위치 {perr*100:.1f} cm,  각도 {yerr:.1f}°", flush=True)
        ferr = math.hypot(fx - gt_x, fz - gt_z)
        print(f"│  융합후 오차 : 위치 {ferr*100:.1f} cm,  "
              f"각도 {abs(ML._wrap_deg(fyaw-gt_yaw)):.1f}°", flush=True)
        print("└─────────────────────────────────────────────────", flush=True)
        print(f"[M3] 카메라가 본 마커 이미지: {OUT_IMG}", flush=True)

        REPORT.write_text(json.dumps({
            "marker": {"id": tgt_id, "label": target_label,
                       "world": [target["x"], target["z"]]},
            "align_yaw_deg": align,
            "T_cam_marker_tvec": [float(v) for v in t],
            "reproj_px": hit.reproj_err_px,
            "marker_fix": {"x": fix.x, "z": fix.z, "yaw": fix.yaw_deg},
            "odometry": {"x": odom_x, "z": odom_z, "yaw": odom_yaw},
            "fused": {"x": fx, "z": fz, "yaw": fyaw},
            "ground_truth": {"x": gt_x, "z": gt_z, "yaw": gt_yaw},
            "marker_pos_err_cm": perr * 100, "marker_yaw_err_deg": yerr,
            "fused_pos_err_cm": ferr * 100,
        }, indent=2, ensure_ascii=False), encoding="utf-8")
        print(f"M3_LOCALIZE_POS_ERR_CM={perr*100:.1f}", flush=True)

        tmp.unlink(missing_ok=True)
        if gui:
            while app.is_running():
                app.update()
    except SystemExit:
        raise
    except Exception:
        import traceback
        print("!! 예외 발생:", flush=True)
        traceback.print_exc()
        sys.stdout.flush()
    finally:
        app.close()


if __name__ == "__main__":
    main()
