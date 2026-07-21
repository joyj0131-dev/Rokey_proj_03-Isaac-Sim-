#!/usr/bin/env python3
"""마커 없이 개루프로 3.4 m(마커 간격) 주행할 때 쌓이는 드리프트를 실측한다.

질문: "마커 사이 3.4 m 를 추측항법으로 가는 동안 얼마나 빗나가나?"
이게 다음 마커의 포착 창(±0.15 m)에 들어오면 마커 기반 주행이 성립한다.

지금까지의 0.44 m / 12.8도 는 서로 다른 실행에서 나온 값이라 근거가 약했다.
여기서는 한 번의 깨끗한 개루프 주행에서 병진·요를 동시에 재고, 방향(종/횡/대각)과
반복으로 분포를 낸다. 모든 자세 보정(hold_x/z/yaw)은 끈다 — 순수 개루프가 목적이다.

측정하는 것:
  - lateral_off : 진행 방향과 직교하게 빗나간 거리 [m]  (포착 창과 직접 비교)
  - along_err   : 진행 방향으로 더/덜 간 거리 [m]
  - yaw_deg     : 누적 요각 [deg]
  - cam_shift   : 요각이 1.25 m 앞 카메라 조준점을 옆으로 민 거리 [m] = 1.25*tan(yaw)
  - total_off   : lateral_off + cam_shift (마커가 창에서 벗어나는 실질 오프셋)

실행:
    python3 probe_drift.py            # 종/횡/대각 각 3회
    python3 probe_drift.py --reps 5
"""

import json
import math
import os
import sys
from pathlib import Path

WORK_DIR = Path(__file__).resolve().parent
ROBOT_USD = (WORK_DIR.parent / "hwia_parking_robot_final_caster_package"
             / "hwia_depth_cam_mecha_roller.usd")
REPORT = WORK_DIR / "drift_report.json"
ISAAC_PYTHON = Path(
    "/home/rokey/dev_ws/isaac_sim/isaacsim/_build/linux-x86_64/release/python.sh"
)

ROBOT_XFORM = "/World/Robot"
ROBOT_ROOT = "/World/Robot/base_link"
ROBOT_JOINTS = "/World/Robot/joints"

SPEED = 0.4           # m/s, 검증 스크립트와 동일
LEG = 3.4            # 마커 간격 [m]
CAM_STANDOFF = 1.25   # 카메라 조준 거리 [m]
CAPTURE_WINDOW = 0.15  # 포착 창 반폭 [m]


def _restart():
    if os.environ.get("CARB_APP_PATH"):
        return
    os.execv(str(ISAAC_PYTHON),
             [str(ISAAC_PYTHON), str(Path(__file__).resolve()), *sys.argv[1:]])


def _robot_matrix(Gf, tx, ty, tz):
    return Gf.Matrix4d(
        0.0, 0.0, 1.0, 0.0,
        1.0, 0.0, 0.0, 0.0,
        0.0, 1.0, 0.0, 0.0,
        tx, ty, tz, 1.0,
    )


def main():
    _restart()
    reps = 3
    if "--reps" in sys.argv:
        reps = int(sys.argv[sys.argv.index("--reps") + 1])

    from isaacsim import SimulationApp
    app = SimulationApp({"headless": True})
    try:
        import numpy as np
        import omni.physx
        import omni.timeline
        import omni.usd
        from isaacsim.core.api import World
        from isaacsim.core.prims import Articulation
        from pxr import Gf, Usd, UsdGeom, UsdPhysics

        from mecanum_drive import (
            WHEEL_JOINTS, configure_hub_drives, wheel_velocities_from_cmd_vel,
        )

        # 빈 바닥 씬
        stage = Usd.Stage.CreateInMemory()
        UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.y)
        UsdGeom.SetStageMetersPerUnit(stage, 1.0)
        w = UsdGeom.Xform.Define(stage, "/World").GetPrim()
        stage.SetDefaultPrim(w)
        g = UsdGeom.Cube.Define(stage, "/World/Ground")
        g.CreateSizeAttr(1.0)
        gx = UsdGeom.Xformable(g)
        gx.AddTranslateOp().Set(Gf.Vec3d(0.0, -0.5, 0.0))
        gx.AddScaleOp().Set(Gf.Vec3f(200.0, 1.0, 200.0))
        UsdPhysics.CollisionAPI.Apply(g.GetPrim())
        sc = UsdPhysics.Scene.Define(stage, "/World/PhysicsScene")
        sc.CreateGravityDirectionAttr(Gf.Vec3f(0.0, -1.0, 0.0))
        sc.CreateGravityMagnitudeAttr(9.81)
        r = stage.DefinePrim(ROBOT_XFORM, "Xform")
        r.GetReferences().AddReference(str(ROBOT_USD))
        rx = UsdGeom.Xformable(r)
        rx.ClearXformOpOrder()
        mat = rx.MakeMatrixXform()
        mat.Set(_robot_matrix(Gf, 0.0, 0.0, 0.0))

        tmp = WORK_DIR / "_drift.usd"
        stage.GetRootLayer().Export(str(tmp))
        ctx = omni.usd.get_context()
        ctx.open_stage(str(tmp))
        for _ in range(30):
            app.update()
        live = ctx.get_stage()
        configure_hub_drives(live, ROBOT_JOINTS)

        world = World(stage_units_in_meters=1.0, set_defaults=False)
        timeline = omni.timeline.get_timeline_interface()
        timeline.play()
        world.reset()
        for _ in range(30):
            world.step(render=False)
        art = Articulation(ROBOT_ROOT)
        art.initialize()
        idx = {wn: art.dof_names.index(j) for wn, j in WHEEL_JOINTS.items()}
        vel = np.zeros(np.array(art.get_joint_velocities()).shape, dtype=np.float32)
        live_mat = UsdGeom.Xformable(
            live.GetPrimAtPath(ROBOT_XFORM)).GetOrderedXformOps()[0]

        physx = omni.physx.get_physx_interface()

        def pose():
            t = physx.get_rigidbody_transformation(ROBOT_ROOT)
            p = tuple(float(v) for v in t["position"])
            q = [float(v) for v in t["rotation"]]
            x, y, z, ww = q
            fx = 1 - 2 * (y * y + z * z)
            fz = 2 * (x * z - y * ww)
            yaw = math.degrees(math.atan2(fx, fz))
            return p, yaw

        def drive(vx, vy):
            for wn, om in wheel_velocities_from_cmd_vel(vx, vy, 0.0).items():
                if vel.ndim == 2:
                    vel[0, idx[wn]] = om
                else:
                    vel[idx[wn]] = om
            art.set_joint_velocity_targets(vel)

        # 방향: 명령 (vx, vy) 와 그때 기대되는 월드 이동 단위벡터 (dx, dz).
        # verify 규약: 로봇 전방(vx)->월드 +Z, 로봇 좌(vy)->월드 +X.
        S = SPEED
        dirs = [
            ("종이동(+Z)", (S, 0.0), (0.0, 1.0)),
            ("횡이동(+X)", (0.0, S), (1.0, 0.0)),
            ("대각(+X+Z)", (S * 0.707, S * 0.707), (0.707, 0.707)),
        ]

        steps = int(LEG / SPEED / (1.0 / 60.0))  # 60 Hz 가정, LEG 거리만큼
        results = []
        for name, (vx, vy), (ex, ez) in dirs:
            for rep in range(reps):
                # 리셋: 원점, 요각 0 으로 되돌린다
                timeline.stop()
                for _ in range(3):
                    app.update()
                live_mat.Set(_robot_matrix(Gf, 0.0, 0.0, 0.0))
                timeline.play()
                world.reset()
                for _ in range(40):
                    world.step(render=False)

                p0, y0 = pose()
                drive(vx, vy)
                for _ in range(steps):
                    world.step(render=False)
                drive(0.0, 0.0)
                for _ in range(20):
                    world.step(render=False)
                p1, y1 = pose()

                dx, dz = p1[0] - p0[0], p1[2] - p0[2]
                traveled = math.hypot(dx, dz)
                # 기대 방향 성분(진행) 과 직교 성분(빗나감)
                along = dx * ex + dz * ez
                lateral = dx * (-ez) + dz * ex   # 90도 회전
                dyaw = y1 - y0
                cam_shift = CAM_STANDOFF * abs(math.tan(math.radians(dyaw)))
                total_off = abs(lateral) + cam_shift
                results.append({
                    "dir": name, "rep": rep,
                    "traveled_m": round(traveled, 3),
                    "along_err_m": round(along - LEG, 3),
                    "lateral_off_m": round(lateral, 3),
                    "yaw_deg": round(dyaw, 2),
                    "cam_shift_m": round(cam_shift, 3),
                    "total_off_m": round(total_off, 3),
                    "within_window": total_off <= CAPTURE_WINDOW,
                })

        # 방향별 집계
        summary = {}
        for name, _, _ in dirs:
            rs = [x for x in results if x["dir"] == name]
            lat = [abs(x["lateral_off_m"]) for x in rs]
            tot = [x["total_off_m"] for x in rs]
            yaw = [abs(x["yaw_deg"]) for x in rs]
            summary[name] = {
                "lateral_off_max_m": round(max(lat), 3),
                "yaw_max_deg": round(max(yaw), 2),
                "total_off_max_m": round(max(tot), 3),
                "all_within_window": all(x["within_window"] for x in rs),
            }

        REPORT.write_text(json.dumps({
            "leg_m": LEG, "speed_mps": SPEED, "reps": reps,
            "capture_window_half_m": CAPTURE_WINDOW,
            "cam_standoff_m": CAM_STANDOFF,
            "runs": results, "summary": summary,
        }, indent=2, ensure_ascii=False), encoding="utf-8")

        print(f"\n[드리프트] {LEG} m 개루프 주행 x {reps}회 (보정 전부 끔)", flush=True)
        print(f"[드리프트] 포착 창 = ±{CAPTURE_WINDOW} m\n", flush=True)
        print(f"{'방향':<14}{'횡오프셋':>9}{'요각':>8}{'조준밀림':>9}"
              f"{'실질오프셋':>11}  판정", flush=True)
        for name, _, _ in dirs:
            s = summary[name]
            verdict = "창 안 (OK)" if s["all_within_window"] else "창 벗어남"
            print(f"{name:<14}{s['lateral_off_max_m']:>8.3f}m"
                  f"{s['yaw_max_deg']:>7.1f}°{s['total_off_max_m'] - s['lateral_off_max_m']:>8.3f}m"
                  f"{s['total_off_max_m']:>10.3f}m  {verdict}", flush=True)
        print(f"\n(횡오프셋=옆으로 빗나감, 조준밀림=요각이 카메라 조준점을 민 거리,"
              f" 실질오프셋=둘 합)", flush=True)
        print(f"[드리프트] 리포트: {REPORT}", flush=True)
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
