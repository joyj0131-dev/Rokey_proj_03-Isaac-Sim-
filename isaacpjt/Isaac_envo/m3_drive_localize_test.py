#!/usr/bin/env python3
"""M3 주행 융합 테스트: A2→A7 을 횡주행하며 마커를 인식하고, 인식할 때마다
그 마커 ID·월드좌표와 로봇 측위를 로그로 찍는다. 드리프트 누적 → 마커 리셋을 본다.

두 개의 자세 추적기를 나란히 돌린다:
  - 오도메트리만 : 명령 속도만 적분(마커 보정 없음) → 계속 드리프트
  - 마커 융합    : 오도메트리 예측 + 마커 관측 보정 → 마커마다 리셋
Isaac ground truth 를 기준으로 둘의 오차를 비교한다.

주행/자세유지는 검증된 방식 그대로: 마커행에서 1.25 m 물러나 횡이동, hold_z(표준오프)
+ hold_yaw(0). wz 부호는 실측 캘리브. (verify_depth_cam_mecha.py 와 동일 규약)
카메라 장착 T_base_cam 은 에셋에서 직접 읽고, marker→world 규약은 align_yaw=0
(m3_localize_demo.py 가 GT 로 확인).

실행:
    python3 m3_drive_localize_test.py            # A2→A7, 헤드리스
    python3 m3_drive_localize_test.py --gui
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
REPORT = WORK_DIR / "m3_drive_localize_report.json"
ISAAC_PYTHON = Path(
    "/home/rokey/dev_ws/isaac_sim/isaacsim/_build/linux-x86_64/release/python.sh")

sys.path.insert(0, str(REPO / "src" / "parkbot_aruco"))

ROBOT_XFORM = "/World/Robot"
ROBOT_ROOT = "/World/Robot/base_link"
ROBOT_JOINTS = "/World/Robot/joints"
CAM_FRONT = "/World/Robot/cam_front_link/depth_cam_front/Camera_Pseudo_Depth_Front"
CAM_RES = (640, 480)

SPEED = 0.4               # m/s 횡이동
MARKER_STANDOFF = 1.25    # 마커행에서 물러설 거리
DETECT_EVERY = 3          # 프레임 데시메이션
DT = 1.0 / 60.0           # 물리 스텝
MAX_REPROJ = 2.0          # 이보다 크면 로그·보정에서 제외
START_LABEL, GOAL_LABEL = "A2", "A7"
LANE_IDS = {1: "A2", 2: "A3", 3: "A4", 4: "A5", 5: "A6", 6: "A7"}


def _restart():
    if os.environ.get("CARB_APP_PATH"):
        return
    os.execv(str(ISAAC_PYTHON),
             [str(ISAAC_PYTHON), str(Path(__file__).resolve()), *sys.argv[1:]])


def _robot_matrix(Gf, tx, ty, tz):
    return Gf.Matrix4d(0, 0, 1, 0, 1, 0, 0, 0, 0, 1, 0, 0, tx, ty, tz, 1)


def _gf_to_np(M):
    import numpy as np
    return np.array([[M[i][j] for j in range(4)] for i in range(4)],
                    dtype=np.float64).T


def main():
    _restart()
    gui = "--gui" in sys.argv[1:]

    mm = json.loads(MAP_JSON.read_text(encoding="utf-8"))
    code_size = float(mm["code_size_m"])
    by_label = {m["label"]: m for m in mm["markers"]}
    start, goal = by_label[START_LABEL], by_label[GOAL_LABEL]

    from isaacsim import SimulationApp
    app = SimulationApp({"headless": not gui})
    try:
        import numpy as np
        import cv2
        import omni.physx
        import omni.timeline
        import omni.usd
        from isaacsim.core.api import World
        from isaacsim.core.prims import Articulation
        from isaacsim.sensors.camera import Camera
        from pxr import Gf, Usd, UsdGeom, UsdPhysics

        from mecanum_drive import (
            WHEEL_JOINTS, configure_hub_drives, wheel_velocities_from_cmd_vel)
        from parkbot_aruco import aruco_pose as AP
        from parkbot_aruco import marker_localizer as ML

        # ---- 씬: 마커 환경 + 로봇을 A2 표준오프에 배치 ----
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

        tmp = WORK_DIR / "_m3_drive.usd"
        stage.GetRootLayer().Export(str(tmp))
        ctx = omni.usd.get_context()
        ctx.open_stage(str(tmp))
        for _ in range(30):
            app.update()
        live = ctx.get_stage()
        configure_hub_drives(live, ROBOT_JOINTS)

        world = World(stage_units_in_meters=1.0, set_defaults=False)
        omni.timeline.get_timeline_interface().play()
        world.reset()
        for _ in range(30):
            world.step(render=False)
        try:
            dt_phys = float(world.get_physics_dt())
        except Exception:
            dt_phys = DT

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
            fx = 1 - 2 * (y * y + z * z)
            fz = 2 * (x * z - y * ww)
            return p[0], p[2], math.degrees(math.atan2(fx, fz))

        def drive(vx, vy, wz):
            for wn, om in wheel_velocities_from_cmd_vel(vx, vy, wz).items():
                if vel.ndim == 2:
                    vel[0, idx[wn]] = om
                else:
                    vel[idx[wn]] = om
            art.set_joint_velocity_targets(vel)

        # ---- 엔코더 오도메트리 FK: 모듈 IK를 역행렬로 뒤집어 유도(부호 일관 보장) ----
        # IK: wheel_omega = M @ [vx, vy, wz]. 각 명령축에 단위를 넣어 M 열을 얻는다.
        wheel_order = list(WHEEL_JOINTS.keys())
        M_ik = np.array([
            [wheel_velocities_from_cmd_vel(*cmd)[wn] for wn in wheel_order]
            for cmd in ([1, 0, 0], [0, 1, 0], [0, 0, 1])
        ], dtype=np.float64).T          # 4x3
        M_fk = np.linalg.pinv(M_ik)     # 3x4: wheel Δangle → [Δfwd, Δleft, Δyaw(rad)]

        def wheel_vels():
            # 각도가 아니라 각속도. 연속 회전하는 바퀴 각도는 wrap 되어 차분이
            # 무의미해지지만(구간 캘리브가 27.9x로 폭주했던 원인), 각속도는 안전하다.
            qd = np.array(art.get_joint_velocities())
            if qd.ndim == 2:
                qd = qd[0]
            return np.array([qd[idx[wn]] for wn in wheel_order], dtype=np.float64)

        # ---- T_base_cam (에셋에서 직접) ----
        cam = Camera(prim_path=CAM_FRONT, resolution=CAM_RES)
        cam.initialize()
        for _ in range(60):
            world.step(render=True)
        xc = UsdGeom.XformCache()
        T_w_base = _gf_to_np(xc.GetLocalToWorldTransform(live.GetPrimAtPath(ROBOT_ROOT)))
        T_w_usdcam = _gf_to_np(xc.GetLocalToWorldTransform(live.GetPrimAtPath(CAM_FRONT)))
        T_base_cam = (np.linalg.inv(T_w_base) @ T_w_usdcam) @ np.diag([1.0, -1, -1, 1])
        K = cam.get_intrinsics_matrix()
        dist = np.zeros((5, 1))
        detector = AP.make_detector(mm["dictionary"])
        marker_map = ML.MarkerMap.from_json(mm, align_yaw_deg=0.0)

        # ---- wz 부호 캘리브 (verify 와 동일) ----
        gx0, gz0, gy0 = gt_pose()
        drive(0.0, 0.0, 0.3)
        for _ in range(60):
            world.step(render=False)
        drive(0.0, 0.0, 0.0)
        for _ in range(30):
            world.step(render=False)
        _, _, gy1 = gt_pose()
        yaw_sign = 1.0 if (gy1 - gy0) > 0 else -1.0

        # ---- 오도메트리 횡스케일 캘리브 ----
        # 이 에셋의 실제 횡이동이 mecanum_drive IK 모델보다 ~1.87배 크다(롤러 기하
        # 불일치). 엔코더 FK 는 IK 모델 기준이라 실제의 절반만 센다. 실제 로봇이
        # 하듯 순수 횡이동 구간에서 GT 대비 FK 비를 재서 스케일을 보정한다.
        # 보정 후 남는 드리프트가 진짜 슬립이다.
        # 실주행과 같은 조건(yaw 보정 ON)에서 정상상태로 캘리브한다. 개루프 순수
        # 횡이동은 로봇이 yaw로 휘어 GT_x 가 줄고, 그러면 스케일이 0.56x 로 잘못
        # 나온다(FK 는 경로길이, GT_x 는 직선변위라 휘면 GT_x < FK). yaw 를 잡으면
        # 직선으로 가서 FK≈GT 가 된다.
        for warm in range(60):          # 가속 전이 버림 + yaw 보정
            _, _, gyc = gt_pose()
            drive(0.0, SPEED, max(-0.5, min(0.5, yaw_sign * 0.025 * (0.0 - gyc))))
            world.step(render=False)
        gcx0, _, _ = gt_pose()
        fk_left = 0.0
        for _ in range(150):
            _, _, gyc = gt_pose()
            drive(0.0, SPEED, max(-0.5, min(0.5, yaw_sign * 0.025 * (0.0 - gyc))))
            world.step(render=False)
            fk_left += float((M_fk @ wheel_vels())[1]) * dt_phys
        gcx1, _, _ = gt_pose()
        drive(0.0, 0.0, 0.0)
        for _ in range(20):
            world.step(render=False)
        odom_scale = (gcx1 - gcx0) / fk_left if abs(fk_left) > 1e-6 else 1.0
        print(f"[M3주행] sim dt={dt_phys:.4f}s | 오도메트리 횡스케일(정상상태·yaw보정): "
              f"{odom_scale:.3f}x  (GT {gcx1-gcx0:+.3f} m vs FK {fk_left:+.3f} m)",
              flush=True)

        # 캘리브로 흐트러진 위치를 A2 표준오프로 되돌린다.
        omni.timeline.get_timeline_interface().stop()
        for _ in range(3):
            app.update()
        UsdGeom.Xformable(live.GetPrimAtPath(ROBOT_XFORM)).MakeMatrixXform().Set(
            _robot_matrix(Gf, float(start["x"]), 0.0, lane_z))
        omni.timeline.get_timeline_interface().play()
        world.reset()
        for _ in range(40):
            world.step(render=False)

        # ---- 두 추적기 초기화: 시작 GT 로 맞춘다 ----
        gx, gz, gy = gt_pose()
        odom_only = ML.PoseFilter()
        odom_only.set_pose(gx, gz, gy)
        # 확신 높은 마커(재투영<2px)는 강하게 신뢰한다. 바퀴 오도메트리가
        # 신뢰 불가라(아래 결과 참고) 마커가 잡히면 거의 그쪽으로 스냅한다.
        fused = ML.PoseFilter(pos_gain=0.9, yaw_gain=0.9)
        fused.set_pose(gx, gz, gy)

        print(f"\n[M3주행] {START_LABEL}→{GOAL_LABEL} 횡주행 "
              f"({abs(goal['x']-start['x']):.1f} m), 표준오프 {MARKER_STANDOFF} m, "
              f"wz부호 {yaw_sign:+.0f}", flush=True)
        print(f"[M3주행] 통과 예정 마커: {list(LANE_IDS.values())}\n", flush=True)

        logged = {}          # marker_id -> 로그 dict
        target_x = float(goal["x"]) + 0.8   # 마지막 마커를 지나치도록 오버슛
        drive(0.0, SPEED, 0.0)
        step = 0
        while step < 8000:
            # hold_z(표준오프) + hold_yaw(0) 보정 명령
            gx, gz, gy = gt_pose()
            cvx = max(-0.15, min(0.15, 1.5 * (lane_z - gz)))
            cvy = SPEED
            cwz = max(-0.5, min(0.5, yaw_sign * 0.025 * (0.0 - gy)))
            drive(cvx, cvy, cwz)
            world.step(render=True)
            step += 1

            # 엔코더 오도메트리: 실제 바퀴 '각속도' → FK → 바디 속도 → dt 적분.
            # 각속도라 wrap 없고, IK 모델과 실제 기하의 차이는 odom_scale 로 보정.
            # 남는 차이가 롤러 슬립 = 정직한 드리프트.
            v_fwd, v_left, v_yaw = M_fk @ wheel_vels()
            d_fwd = v_fwd * dt_phys * odom_scale
            d_left = v_left * dt_phys * odom_scale
            dyaw = math.degrees(v_yaw * dt_phys)
            for trk in (odom_only, fused):
                trk.predict_body_delta(float(d_fwd), float(d_left), float(dyaw))

            if step % DETECT_EVERY == 0:
                rgba = cam.get_rgba()
                if rgba is None or getattr(rgba, "size", 0) == 0:
                    if gx >= target_x:
                        break
                    continue
                gray = cv2.cvtColor(np.asarray(rgba[..., :3], np.uint8),
                                    cv2.COLOR_RGB2GRAY)
                for p in AP.detect_and_estimate(gray, detector, code_size, K, dist):
                    if p.marker_id not in LANE_IDS or p.reproj_err_px > MAX_REPROJ:
                        continue
                    T_cm = ML.rvec_tvec_to_T(p.rvec, p.tvec)
                    fix = ML.robot_pose_from_marker(
                        p.marker_id, T_cm, T_base_cam, marker_map)
                    if fix is None:
                        continue
                    first = p.marker_id not in logged
                    if first:
                        # 리셋 직전 상태(직전 마커 이후 쌓인 드리프트) 기록
                        fb = fused.pose()
                        oo = odom_only.pose()
                        g = gt_pose()
                    fused.update(fix)   # 매 관측마다 융합(락 유지)
                    if first:
                        fa = fused.pose()
                        mk = marker_map.by_id[p.marker_id]
                        logged[p.marker_id] = {
                            "id": p.marker_id, "label": LANE_IDS[p.marker_id],
                            "marker_xz": [mk["x"], mk["z"]],
                            "gt": [round(g[0], 3), round(g[1], 3)],
                            "odom_only": [round(oo[0], 3), round(oo[1], 3)],
                            "fused_before": [round(fb[0], 3), round(fb[1], 3)],
                            "fused_after": [round(fa[0], 3), round(fa[1], 3)],
                            "fix": [round(fix.x, 3), round(fix.z, 3)],
                            "reproj_px": round(p.reproj_err_px, 2),
                            "odom_err_cm": round(math.hypot(oo[0]-g[0], oo[1]-g[1])*100, 1),
                            "fused_before_err_cm": round(math.hypot(fb[0]-g[0], fb[1]-g[1])*100, 1),
                            "fused_after_err_cm": round(math.hypot(fa[0]-g[0], fa[1]-g[1])*100, 1),
                        }
                        L = logged[p.marker_id]
                        print(f"[마커 인식] ID {L['id']} ({L['label']})  "
                              f"마커좌표 world=({mk['x']:+.2f}, {mk['z']:+.2f})  "
                              f"재투영 {L['reproj_px']}px", flush=True)
                        print(f"   로봇 GT        x={g[0]:+.3f}  z={g[1]:+.3f}", flush=True)
                        print(f"   오도메트리만    x={oo[0]:+.3f}  z={oo[1]:+.3f}   "
                              f"누적드리프트 {L['odom_err_cm']}cm", flush=True)
                        print(f"   마커융합(직전)  x={fb[0]:+.3f}  z={fb[1]:+.3f}   "
                              f"오차 {L['fused_before_err_cm']}cm", flush=True)
                        print(f"   마커융합(리셋)  x={fa[0]:+.3f}  z={fa[1]:+.3f}   "
                              f"오차 {L['fused_after_err_cm']}cm  ← 리셋\n", flush=True)
            if gx >= target_x:
                break

        drive(0.0, 0.0, 0.0)
        for _ in range(30):
            world.step(render=False)

        seen = sorted(logged)
        missed = [lb for i, lb in LANE_IDS.items() if i not in logged]
        odom_final = odom_only.pose()
        fused_final = fused.pose()
        gxf, gzf, _ = gt_pose()
        summary = {
            "route": f"{START_LABEL}->{GOAL_LABEL}",
            "recognized": [LANE_IDS[i] for i in seen],
            "missed": missed,
            "coverage": f"{len(seen)}/{len(LANE_IDS)}",
            "odom_only_final_err_cm": round(math.hypot(
                odom_final[0]-gxf, odom_final[1]-gzf)*100, 1),
            "fused_final_err_cm": round(math.hypot(
                fused_final[0]-gxf, fused_final[1]-gzf)*100, 1),
            "per_marker": [logged[i] for i in seen],
        }
        REPORT.write_text(json.dumps(summary, indent=2, ensure_ascii=False),
                          encoding="utf-8")
        print(f"[M3주행] 인식 {summary['coverage']}: {summary['recognized']}"
              + (f"  (놓침 {missed})" if missed else ""), flush=True)
        print(f"[M3주행] 최종 오차 — 오도메트리만 "
              f"{summary['odom_only_final_err_cm']}cm  vs  "
              f"마커융합 {summary['fused_final_err_cm']}cm", flush=True)
        print(f"M3_DRIVE_COVERAGE={len(seen)}/{len(LANE_IDS)}", flush=True)
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
