#!/usr/bin/env python3
"""깊이캠+메카넘 합친 에셋(hwia_depth_cam_mecha_roller.usd) 검증.

기존 mecanum_strafe_test.py / mecanum_holonomic_test.py 는 건드리지 않는다.
그쪽은 "베이스 에셋 + 런타임 롤러" 라는 다른 구성을 검증하는 회귀 기준이고,
이 에셋은 롤러가 이미 구워져 있으며 프림 계층도 한 단계 얕다.

단계 1 (물리): 평평한 바닥에서 정착 / 횡이동 / 전진.
              롤러를 넣은 목적이 횡이동이므로 이게 안 되면 나머지는 볼 필요가 없다.
단계 2 (마커): 주차장 마커 씬에서 A2 -> A6 (13.6 m, world +X 횡이동) 주행하며
              전방 카메라로 마커를 인식한다. 종이동(world Z)도 섞는다.
              통과 마커: A2(1) A3(2) A4(3) A5(4) A6(5), 3.40 m 간격.

실행:
    python3 verify_depth_cam_mecha.py             # 1단계만
    python3 verify_depth_cam_mecha.py --markers   # 1+2단계
    python3 verify_depth_cam_mecha.py --markers --gui
"""

import json
import math
import os
import sys
from pathlib import Path

WORK_DIR = Path(__file__).resolve().parent
ROBOT_USD = (WORK_DIR.parent / "hwia_parking_robot_final_caster_package"
             / "hwia_depth_cam_mecha_roller.usd")
MARKER_STAGE = WORK_DIR / "parking" / "parking_environment_marker_preview.usd"
MAP_JSON = (WORK_DIR.parents[1] / "src" / "parkbot_aruco"
            / "data" / "marker_map.json")  # 지도는 ROS 패키지가 소유
REPORT = WORK_DIR / "verify_depth_cam_mecha_report.json"
ISAAC_PYTHON = Path(
    "/home/rokey/dev_ws/isaac_sim/isaacsim/_build/linux-x86_64/release/python.sh"
)

# 로봇 프림 경로 (이 에셋 기준 — 베이스/메카넘 에셋보다 한 단계 얕다)
ROBOT_XFORM = "/World/Robot"
ROBOT_WRAP = "/World/Robot"
ROBOT_ROOT = "/World/Robot/base_link"
ROBOT_JOINTS = "/World/Robot/joints"
CAM_FRONT = "/World/Robot/cam_front_link/depth_cam_front/Camera_Pseudo_Depth_Front"

CAM_RES = (640, 480)
SPEED = 0.4               # m/s
DETECT_EVERY = 4          # 프레임 데시메이션

# 마커 행에서 얼마나 물러서서 횡이동할지 [m].
#
# 전방 카메라는 로봇좌표 x=+0.924, z=+0.090 에서 30도 아래를 본다. 카메라가
# 낮아서 지면을 매우 뉜 각도로 보므로, 마커를 검출할 수 있는 구간이 좁다.
# probe_marker_view.py 로 실측한 결과 로봇 중심 기준 전방 1.1~1.4 m 뿐이었다
# (1.0 m 이하는 시야 아래, 1.6 m 이상은 마커가 눕다 못해 세로 37 px 미만).
# 그래서 그 한가운데인 1.25 m 를 표준 오프셋으로 쓴다.
#
# 이전 버전은 로봇을 마커 좌표 '위'에 스폰하고 횡이동시켜서 마커가 구조적으로
# 시야 밖이었고, 그 결과 293 프레임 전부 0개 검출(0/5)이 나왔다.
MARKER_STANDOFF = 1.25
STANDOFF_FAR = 2.00       # 종이동 시작 지점(검출 구간 밖)
STANDOFF_NEAR = 0.90      # 종이동 종료 지점(검출 구간 밖)


def _restart_with_isaac_python():
    if os.environ.get("CARB_APP_PATH"):
        return
    os.execv(str(ISAAC_PYTHON),
             [str(ISAAC_PYTHON), str(Path(__file__).resolve()), *sys.argv[1:]])


def _robot_matrix(Gf, tx, ty, tz):
    """로봇 로컬 +X -> 월드 +Z, +Y -> +X, +Z -> +Y (carry demo 와 동일 규약)."""
    return Gf.Matrix4d(
        0.0, 0.0, 1.0, 0.0,
        1.0, 0.0, 0.0, 0.0,
        0.0, 1.0, 0.0, 0.0,
        tx, ty, tz, 1.0,
    )


def _make_detector(dictionary):
    import cv2
    if hasattr(cv2.aruco, "ArucoDetector"):
        det = cv2.aruco.ArucoDetector(dictionary, cv2.aruco.DetectorParameters())
        return lambda img: det.detectMarkers(img)[:2]
    params = cv2.aruco.DetectorParameters_create()
    return lambda img: cv2.aruco.detectMarkers(img, dictionary, parameters=params)[:2]


def main():
    _restart_with_isaac_python()
    do_markers = "--markers" in sys.argv[1:]
    gui = "--gui" in sys.argv[1:]

    from isaacsim import SimulationApp
    app = SimulationApp({"headless": not gui})
    report = {}
    try:
        import numpy as np
        import omni.physx
        import omni.timeline
        import omni.usd
        from isaacsim.core.api import World
        from isaacsim.core.prims import Articulation
        from pxr import Gf, Usd, UsdGeom, UsdPhysics, UsdShade

        from mecanum_drive import (
            WHEEL_JOINTS, configure_hub_drives, wheel_velocities_from_cmd_vel,
        )

        # ---------- 공통 헬퍼 ----------
        def build_scene(stage_path, robot_xyz):
            """씬을 만들고 로봇을 배치한다. stage_path=None 이면 빈 바닥 씬."""
            if stage_path is None:
                stage = Usd.Stage.CreateInMemory()
                UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.y)
                UsdGeom.SetStageMetersPerUnit(stage, 1.0)
                w = UsdGeom.Xform.Define(stage, "/World").GetPrim()
                stage.SetDefaultPrim(w)
                g = UsdGeom.Cube.Define(stage, "/World/Ground")
                g.CreateSizeAttr(1.0)
                gx = UsdGeom.Xformable(g)
                gx.AddTranslateOp().Set(Gf.Vec3d(0.0, -0.5, 0.0))
                gx.AddScaleOp().Set(Gf.Vec3f(80.0, 1.0, 80.0))
                UsdPhysics.CollisionAPI.Apply(g.GetPrim())
            else:
                stage = Usd.Stage.CreateInMemory()
                UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.y)
                UsdGeom.SetStageMetersPerUnit(stage, 1.0)
                w = UsdGeom.Xform.Define(stage, "/World").GetPrim()
                stage.SetDefaultPrim(w)
                env = UsdGeom.Xform.Define(stage, "/World/Env").GetPrim()
                env.GetReferences().AddReference(str(stage_path))

            sc = UsdPhysics.Scene.Define(stage, "/World/PhysicsScene")
            sc.CreateGravityDirectionAttr(Gf.Vec3f(0.0, -1.0, 0.0))
            sc.CreateGravityMagnitudeAttr(9.81)

            r = stage.DefinePrim(ROBOT_XFORM, "Xform")
            r.GetReferences().AddReference(str(ROBOT_USD))
            xf = UsdGeom.Xformable(r)
            xf.ClearXformOpOrder()
            xf.MakeMatrixXform().Set(_robot_matrix(Gf, *robot_xyz))

            tmp = WORK_DIR / "_verify_depth_cam.usd"
            stage.GetRootLayer().Export(str(tmp))
            ctx = omni.usd.get_context()
            ctx.open_stage(str(tmp))
            for _ in range(30):
                app.update()
            return ctx.get_stage(), tmp

        def start_physics(stage):
            configure_hub_drives(stage, ROBOT_JOINTS)
            world = World(stage_units_in_meters=1.0, set_defaults=False)
            omni.timeline.get_timeline_interface().play()
            world.reset()
            for _ in range(30):
                world.step(render=False)
            art = Articulation(ROBOT_ROOT)
            art.initialize()
            idx = {w: art.dof_names.index(j) for w, j in WHEEL_JOINTS.items()}
            vel = np.zeros(np.array(art.get_joint_velocities()).shape, dtype=np.float32)
            return world, art, idx, vel

        def drive(art, idx, vel, vx, vy, wz):
            for wname, omega in wheel_velocities_from_cmd_vel(vx, vy, wz).items():
                i = idx[wname]
                if vel.ndim == 2:
                    vel[0, i] = omega
                else:
                    vel[i] = omega
            art.set_joint_velocity_targets(vel)

        physx = omni.physx.get_physx_interface()

        def pos():
            v = physx.get_rigidbody_transformation(ROBOT_ROOT)["position"]
            return tuple(float(x) for x in v)

        def yaw_deg():
            """월드 Y축 둘레 요각. 로봇 전방(+X 로컬)이 월드 +Z 를 볼 때 0도."""
            q = physx.get_rigidbody_transformation(ROBOT_ROOT)["rotation"]
            x, y, z, w = (float(v) for v in q)
            # 로컬 +X 를 월드로 보낸 벡터
            fx = 1 - 2 * (y * y + z * z)
            fz = 2 * (x * z - y * w)
            return math.degrees(math.atan2(fx, fz))

        # ---------- 단계 1: 물리 ----------
        stage, tmp = build_scene(None, (0.0, 0.0, 0.0))
        world, art, idx, vel = start_physics(stage)

        settle = pos()
        for _ in range(120):
            world.step(render=False)
        settled = pos()
        drift = math.dist(settle, settled)

        def run(vx, vy, steps=240):
            a = pos()
            drive(art, idx, vel, vx, vy, 0.0)
            for _ in range(steps):
                world.step(render=False)
            drive(art, idx, vel, 0.0, 0.0, 0.0)
            for _ in range(30):
                world.step(render=False)
            b = pos()
            return (b[0] - a[0], b[1] - a[1], b[2] - a[2])

        strafe = run(0.0, SPEED)      # 로봇 좌(vy) -> 월드 X
        forward = run(SPEED, 0.0)     # 로봇 전방(vx) -> 월드 Z

        # wz 명령의 부호가 요각을 어느 쪽으로 돌리는지 실측한다.
        # Y-up 스테이지 + 로봇 로컬축 재배치가 겹쳐 부호를 추론으로 정하면 틀린다.
        yaw_before = yaw_deg()
        drive(art, idx, vel, 0.0, 0.0, 0.3)
        for _ in range(60):
            world.step(render=False)
        drive(art, idx, vel, 0.0, 0.0, 0.0)
        for _ in range(30):
            world.step(render=False)
        yaw_delta = yaw_deg() - yaw_before
        yaw_sign = 1.0 if yaw_delta > 0 else -1.0
        print(f"[verify] wz=+0.3 -> 요각 {yaw_delta:+.1f}도 "
              f"(방위 제어 부호 {yaw_sign:+.0f})", flush=True)

        phase1 = {
            "settle_height_y": round(settled[1], 4),
            "settle_drift_m": round(drift, 4),
            "strafe_dxyz": [round(v, 4) for v in strafe],
            "forward_dxyz": [round(v, 4) for v in forward],
            "strafe_main_axis_m": round(strafe[0], 4),
            "forward_main_axis_m": round(forward[2], 4),
        }
        phase1["ok"] = (
            abs(settled[1]) < 0.30 and drift < 0.05
            and abs(strafe[0]) > 0.5 and abs(forward[2]) > 0.5
        )
        report["phase1_physics"] = phase1

        print(f"[verify] 정착 높이 y={settled[1]:+.4f} m, 정착 드리프트 {drift:.4f} m", flush=True)
        print(f"[verify] 횡이동(vy) dx={strafe[0]:+.3f} dy={strafe[1]:+.3f} dz={strafe[2]:+.3f}", flush=True)
        print(f"[verify] 전진(vx)   dx={forward[0]:+.3f} dy={forward[1]:+.3f} dz={forward[2]:+.3f}", flush=True)
        print(f"PHASE1_OK={phase1['ok']}", flush=True)

        if not phase1["ok"]:
            print("[verify] 1단계 실패 — 2단계는 건너뛴다.", flush=True)
        elif do_markers:
            # ---------- 단계 2: 마커 인식 ----------
            import cv2
            from isaacsim.sensors.camera import Camera

            mm = json.loads(MAP_JSON.read_text(encoding="utf-8"))
            lane = {m["label"]: m for m in mm["markers"]
                    if m["label"] in ("A2", "A3", "A4", "A5", "A6")}
            start, goal = lane["A2"], lane["A6"]

            # 마커 행에서 MARKER_STANDOFF 만큼 물러선 위치에서 시작한다.
            # 마커 위에 서면 마커가 전방 카메라 시야에 들어오지 않는다.
            lane_z = start["z"] - MARKER_STANDOFF
            stage, tmp2 = build_scene(MARKER_STAGE, (start["x"], 0.0, lane_z))
            world, art, idx, vel = start_physics(stage)

            cam = Camera(prim_path=CAM_FRONT, resolution=CAM_RES)
            cam.initialize()
            for _ in range(60):
                world.step(render=True)

            detect = _make_detector(
                cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, mm["dictionary"]))
            )
            seen, seen_long, samples = {}, {}, 0
            expected = {m["id"]: lb for lb, m in lane.items()}

            def sweep(vx, vy, target_axis, target_value, tag, sink,
                      max_steps=6000, hold_z=None, hold_x=None,
                      every=DETECT_EVERY, hold_yaw=None):
                """hold_z / hold_x 를 주면 그 축을 매 스텝 붙잡는다.

                개루프 메카넘은 진행 방향과 직교한 쪽으로 밀린다 — 실측으로
                횡이동 13.6 m 에 +Z 0.44 m(3.2%), 종이동 1.2 m 에 -X 0.31 m.
                검출 구간이 로봇 앞 1.1~1.4 m 뿐이라 이 드리프트만으로 마커가
                창 밖으로 나가 검출이 0이 됐다. 실제 시스템은 마커 피드백으로
                이걸 잡으므로, 여기서는 그 제어 루프가 있다고 보고 자세를
                유지시켜 '마커 인식 능력' 자체를 분리 측정한다.
                """
                nonlocal samples
                p0, n0 = pos(), samples
                drive(art, idx, vel, vx, vy, 0.0)
                for step in range(max_steps):
                    world.step(render=True)
                    p = pos()
                    if hold_z is not None or hold_x is not None or hold_yaw is not None:
                        cvx = (vx if hold_z is None
                               else max(-0.15, min(0.15, 1.5 * (hold_z - p[2]))))
                        cvy = (vy if hold_x is None
                               else max(-0.15, min(0.15, 1.5 * (hold_x - p[0]))))
                        # 방위 유지. 개루프 횡이동은 13.6 m 에 -12.8도 돌아가는데,
                        # 요각 e 는 1.25 m 앞 지면 패치를 1.25*tan(e) 만큼 옆으로
                        # 밀어 마커를 검출 구간 밖으로 보낸다.
                        cwz = (0.0 if hold_yaw is None
                               else max(-0.5, min(0.5, yaw_sign * 0.025
                                                  * (hold_yaw - yaw_deg()))))
                        drive(art, idx, vel, cvx, cvy, cwz)
                    if step % every == 0:
                        rgba = cam.get_rgba()
                        if rgba is not None and getattr(rgba, "size", 0):
                            gray = cv2.cvtColor(
                                np.asarray(rgba[..., :3], dtype=np.uint8),
                                cv2.COLOR_RGB2GRAY)
                            _c, ids = detect(gray)
                            samples += 1
                            if ids is not None:
                                for v in ids:
                                    mid = int(v[0])
                                    rec = sink.setdefault(
                                        mid, {"count": 0, "first_x": round(p[0], 3),
                                              "first_z": round(p[2], 3)})
                                    rec["count"] += 1
                                    rec["last_x"] = round(p[0], 3)
                                    rec["last_z"] = round(p[2], 3)
                    if (target_value - p[target_axis]) * (1 if vy > 0 or vx > 0 else -1) <= 0:
                        break
                drive(art, idx, vel, 0.0, 0.0, 0.0)
                for _ in range(30):
                    world.step(render=True)
                p1 = pos()
                print(f"[sweep] {tag}: ({p0[0]:+.2f},{p0[2]:+.2f}) -> "
                      f"({p1[0]:+.2f},{p1[2]:+.2f})  요각 {yaw_deg():+.1f}도  "
                      f"프레임 {samples - n0}개 검출 {sorted(sink)}", flush=True)
                return p1

            # (1) 횡이동: 마커 행과 나란히 A2 -> A6. 마커가 계속 1.25 m 앞에 있다.
            # 마지막 마커를 0.8 m 지나쳐서 멈춘다. 검출은 마커를 지난 직후
            # 0.16~0.63 m 구간에서 일어나므로(실측), 목표 x 에서 딱 멈추면
            # 마지막 마커가 검출 구간에 들어오기 전에 정지해 버린다.
            end = sweep(0.0, SPEED, 0, goal["x"] + 0.8, "A2->A6 횡이동", seen,
                        hold_z=lane_z, hold_yaw=0.0)

            # (2) 종이동: A6 앞에서 뒤로 물러났다가(검출 구간 밖) 다시 전진해
            #     마커가 검출 구간을 통과하게 한다. 횡이동과 분리해 집계한다.
            #     횡이동이 마지막 마커를 0.8 m 지나쳐 끝나므로, 먼저 A6 의 x 로
            #     되돌아와 정렬한다. 안 그러면 A6 가 옆으로 벗어나 안 보인다.
            sweep(0.0, -SPEED, 0, goal["x"], "A6 정렬", {}, hold_z=lane_z,
                  hold_yaw=0.0)
            sweep(-SPEED, 0.0, 2, goal["z"] - STANDOFF_FAR, "후진(구간 밖)", {},
                  hold_x=goal["x"], hold_yaw=0.0)
            # 종이동은 저속·매 스텝 샘플링. 검출 창(전방 1.1~1.4 m)이 0.30 m 뿐이라
            # 0.4 m/s + 4스텝 데시메이션으로는 창을 통째로 건너뛴다(실측 23프레임).
            sweep(SPEED * 0.35, 0.0, 2, goal["z"] - STANDOFF_NEAR, "종이동 +Z",
                  seen_long, hold_x=goal["x"], every=1, hold_yaw=0.0)

            detected_expected = {mid: seen[mid] for mid in expected if mid in seen}
            long_ok = goal["id"] in seen_long
            phase2 = {
                "route": "A2 -> A6",
                "distance_m": round(abs(goal["x"] - start["x"]), 3),
                "standoff_m": MARKER_STANDOFF,
                "end_pos": [round(v, 3) for v in end],
                "frames_sampled": samples,
                "expected_ids": expected,
                "detected_expected": detected_expected,
                "detected_all": {str(k): v for k, v in sorted(seen.items())},
                "coverage": f"{len(detected_expected)}/{len(expected)}",
                "longitudinal": {
                    "from_standoff_m": STANDOFF_FAR,
                    "to_standoff_m": STANDOFF_NEAR,
                    "detected": {str(k): v for k, v in sorted(seen_long.items())},
                    "goal_id_detected": long_ok,
                },
                "ok": len(detected_expected) == len(expected) and long_ok,
            }
            report["phase2_markers"] = phase2

            print(f"[verify] A2->A6 {phase2['distance_m']} m 주행, "
                  f"검사 프레임 {samples}개", flush=True)
            for mid, lb in sorted(expected.items()):
                r = seen.get(mid)
                if r:
                    print(f"    OK   ID {mid} {lb}  검출 {r['count']}회 "
                          f"(x {r['first_x']} ~ {r.get('last_x')})", flush=True)
                else:
                    print(f"    MISS ID {mid} {lb}  검출 안 됨", flush=True)
            extra = [k for k in seen if k not in expected]
            if extra:
                print(f"    (구간 외 추가 검출: {sorted(extra)})", flush=True)
            print(f"[verify] 종이동 {STANDOFF_FAR} m -> {STANDOFF_NEAR} m "
                  f"(검출 구간 통과): 검출 ID {sorted(seen_long)}"
                  f" / 목표 ID {goal['id']} {'검출' if long_ok else '미검출'}", flush=True)
            print(f"PHASE2_OK={phase2['ok']}  횡이동 커버리지 {phase2['coverage']}", flush=True)
            tmp2.unlink(missing_ok=True)

        REPORT.write_text(json.dumps(report, indent=2, ensure_ascii=False), encoding="utf-8")
        print(f"[verify] 리포트: {REPORT}", flush=True)
        tmp.unlink(missing_ok=True)
        if gui:
            while app.is_running():
                app.update()
    finally:
        app.close()


if __name__ == "__main__":
    main()
