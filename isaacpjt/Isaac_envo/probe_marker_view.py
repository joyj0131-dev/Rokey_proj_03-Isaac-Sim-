#!/usr/bin/env python3
"""전방 깊이캠이 바닥 마커를 실제로 볼 수 있는 거리 구간을 실측한다.

verify_depth_cam_mecha.py 의 2단계가 0/5 로 나온 뒤 원인을 좁히기 위한 계측용.
그 테스트는 로봇을 마커 '위'에 스폰하고 횡이동시켰는데, 전방 카메라는
로봇 앞 0.96 m 부터의 지면만 본다. 즉 마커가 구조적으로 시야 밖이었다.

여기서는 로봇을 마커 뒤 여러 거리에 정지 배치해 놓고 한 프레임씩 받아
검출 여부와 픽셀 크기를 잰다. 이 결과가 "마커를 몇 m 앞에서 잡을 수 있나"
= 경로 계획의 MAX_LEG 상한을 실측으로 확정한다.

카메라 사양(에셋 실측): 로봇좌표 (x=+0.924, z=+0.090), 하향 30도, 수평 FOV 90.5도.

실행:
    python3 probe_marker_view.py           # 기본 거리 스윕
    python3 probe_marker_view.py --gui
"""

import json
import os
import sys
from pathlib import Path

WORK_DIR = Path(__file__).resolve().parent
ROBOT_USD = (WORK_DIR.parent / "hwia_parking_robot_final_caster_package"
             / "hwia_depth_cam_mecha_roller.usd")
MARKER_STAGE = WORK_DIR / "parking" / "parking_environment_marker_preview.usd"
MAP_JSON = (WORK_DIR.parents[1] / "src" / "parkbot_aruco"
            / "data" / "marker_map.json")  # 지도는 ROS 패키지가 소유
OUT_DIR = WORK_DIR / "marker_view_probe"
REPORT = WORK_DIR / "marker_view_probe_report.json"
ISAAC_PYTHON = Path(
    "/home/rokey/dev_ws/isaac_sim/isaacsim/_build/linux-x86_64/release/python.sh"
)

ROBOT_XFORM = "/World/Robot"
CAM_FRONT = "/World/Robot/cam_front_link/depth_cam_front/Camera_Pseudo_Depth_Front"
CAM_RES = (640, 480)

TARGET_LABEL = "A3"
# 로봇 중심에서 마커까지의 전방(+Z) 거리 [m]
OFFSETS = (0.6, 0.8, 1.0, 1.1, 1.2, 1.4, 1.6, 1.8, 2.0, 2.5, 3.0)
WARMUP = 45
SETTLE = 12


def _restart_with_isaac_python():
    if os.environ.get("CARB_APP_PATH"):
        return
    os.execv(str(ISAAC_PYTHON),
             [str(ISAAC_PYTHON), str(Path(__file__).resolve()), *sys.argv[1:]])


def _robot_matrix(Gf, tx, ty, tz):
    """로봇 로컬 +X -> 월드 +Z (verify_depth_cam_mecha.py 와 동일 규약)."""
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
    gui = "--gui" in sys.argv[1:]

    from isaacsim import SimulationApp
    app = SimulationApp({"headless": not gui})
    try:
        import cv2
        import numpy as np
        import omni.timeline
        import omni.usd
        from isaacsim.core.api import World
        from isaacsim.sensors.camera import Camera
        from pxr import Gf, Usd, UsdGeom, UsdPhysics

        mm = json.loads(MAP_JSON.read_text(encoding="utf-8"))
        target = next(m for m in mm["markers"] if m["label"] == TARGET_LABEL)

        # 씬 1회 구성 후 로봇만 옮겨가며 관측한다(스테이지 재오픈은 느리다).
        stage = Usd.Stage.CreateInMemory()
        UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.y)
        UsdGeom.SetStageMetersPerUnit(stage, 1.0)
        w = UsdGeom.Xform.Define(stage, "/World").GetPrim()
        stage.SetDefaultPrim(w)
        env = UsdGeom.Xform.Define(stage, "/World/Env").GetPrim()
        env.GetReferences().AddReference(str(MARKER_STAGE))
        sc = UsdPhysics.Scene.Define(stage, "/World/PhysicsScene")
        sc.CreateGravityDirectionAttr(Gf.Vec3f(0.0, -1.0, 0.0))
        sc.CreateGravityMagnitudeAttr(9.81)
        r = stage.DefinePrim(ROBOT_XFORM, "Xform")
        r.GetReferences().AddReference(str(ROBOT_USD))
        xf = UsdGeom.Xformable(r)
        xf.ClearXformOpOrder()
        mat = xf.MakeMatrixXform()
        mat.Set(_robot_matrix(Gf, target["x"], 0.0, target["z"] - OFFSETS[0]))

        tmp = WORK_DIR / "_probe_marker.usd"
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

        detect = _make_detector(
            cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, mm["dictionary"]))
        )
        live_mat = UsdGeom.Xformable(
            live.GetPrimAtPath(ROBOT_XFORM)).GetOrderedXformOps()[0]

        OUT_DIR.mkdir(parents=True, exist_ok=True)
        for old in OUT_DIR.glob("*.png"):
            old.unlink()

        rows = []
        for off in OFFSETS:
            # 물리를 켠 채 순간이동시키면 롤러가 딸려오지 않는다. 타임라인을 멈추고 옮긴다.
            omni.timeline.get_timeline_interface().stop()
            for _ in range(3):
                app.update()
            live_mat.Set(_robot_matrix(Gf, target["x"], 0.0, target["z"] - off))
            omni.timeline.get_timeline_interface().play()
            world.reset()
            for _ in range(SETTLE):
                world.step(render=True)

            rgba = cam.get_rgba()
            if rgba is None or getattr(rgba, "size", 0) == 0:
                rows.append({"offset_m": off, "ok": False, "reason": "빈 프레임"})
                continue
            rgb = np.asarray(rgba[..., :3], dtype=np.uint8)
            gray = cv2.cvtColor(rgb, cv2.COLOR_RGB2GRAY)
            corners, ids = detect(gray)
            found = [] if ids is None else [int(v[0]) for v in ids]
            hit = target["id"] in found

            px = None
            if hit:
                c = corners[found.index(target["id"])].reshape(-1, 2)
                px = {
                    "폭": round(float(np.linalg.norm(c[1] - c[0])), 1),
                    "높이": round(float(np.linalg.norm(c[2] - c[1])), 1),
                }
            rows.append({
                "offset_m": off, "ok": hit, "detected": found, "px": px,
                "gray": [int(gray.min()), int(gray.max())],
            })
            cv2.imwrite(
                str(OUT_DIR / f"off_{off:.1f}m_{'ok' if hit else 'FAIL'}.png"),
                cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR),
            )

        good = [r["offset_m"] for r in rows if r.get("ok")]
        report = {
            "target": {"label": TARGET_LABEL, "id": target["id"],
                       "x": target["x"], "z": target["z"]},
            "camera": {"height_m": 0.090, "pitch_down_deg": 30.0,
                       "hfov_deg": 90.5, "resolution": list(CAM_RES)},
            "rows": rows,
            "detect_range_m": [min(good), max(good)] if good else None,
            "any_detected": bool(good),
        }
        REPORT.write_text(json.dumps(report, indent=2, ensure_ascii=False),
                          encoding="utf-8")

        print(f"[probe] 대상 {TARGET_LABEL} (ID {target['id']}) "
              f"@ x={target['x']:.2f} z={target['z']:.2f}", flush=True)
        print("[probe] 로봇중심~마커 전방거리별 검출:", flush=True)
        for rrow in rows:
            mark = "OK  " if rrow.get("ok") else "MISS"
            extra = f" 마커픽셀 {rrow['px']}" if rrow.get("px") else \
                    f" 검출={rrow.get('detected', rrow.get('reason'))}"
            print(f"   {mark} {rrow['offset_m']:.1f} m{extra}", flush=True)
        print(f"[probe] 검출 가능 구간: {report['detect_range_m']}", flush=True)
        print(f"[probe] 이미지: {OUT_DIR}", flush=True)
        print(f"PROBE_ANY_DETECTED={report['any_detected']}", flush=True)

        tmp.unlink(missing_ok=True)
        if gui:
            while app.is_running():
                app.update()
    finally:
        app.close()


if __name__ == "__main__":
    main()
