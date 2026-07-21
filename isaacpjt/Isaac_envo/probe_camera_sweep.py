#!/usr/bin/env python3
"""카메라 기울기 / 높이 / 해상도가 마커 검출 구간에 미치는 영향을 실측한다.

ARUCO_PLAN.md M5 가 요구하는 스윕 장치의 첫 조각. "카메라 높이"와 "기울기"를
논쟁이 아니라 데이터로 닫기 위한 것.

팀원 에셋의 실제 전방 카메라는 로봇좌표 (x=+0.924, z=+0.090), 하향 30도,
수평 FOV 90.5도, 640x480 이다. 여기서는 그 위치에 **검사용 카메라를 따로 만들어**
파라미터만 바꿔가며 같은 마커(A3)를 여러 거리에서 본다. 원본 카메라는 건드리지 않는다.

물리는 쓰지 않는다 — 정지 관측이라 로봇을 놓고 렌더만 하면 된다.

실행:
    python3 probe_camera_sweep.py            # 기울기/높이/해상도 3종 스윕
    python3 probe_camera_sweep.py --tilt     # 기울기만
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
REPORT = WORK_DIR / "camera_sweep_report.json"
SHOT_DIR = WORK_DIR / "camera_sweep_shots"
ISAAC_PYTHON = Path(
    "/home/rokey/dev_ws/isaac_sim/isaacsim/_build/linux-x86_64/release/python.sh"
)

ROBOT_XFORM = "/World/Robot"
TARGET_LABEL = "A3"

# 에셋 실측값 = 기준선
BASE_TILT = 30.0
BASE_HEIGHT = 0.090
BASE_WIDTH = 640
BASE_HFOV = 90.5
CAM_FORWARD = 0.924        # 로봇 중심에서 카메라까지 전방 오프셋

# 로봇 중심 ~ 마커 전방거리 [m]
DISTANCES = (0.9, 1.0, 1.1, 1.2, 1.4, 1.6, 1.8, 2.0, 2.4, 2.8)
SETTLE = 8


def _restart_with_isaac_python():
    if os.environ.get("CARB_APP_PATH"):
        return
    os.execv(str(ISAAC_PYTHON),
             [str(ISAAC_PYTHON), str(Path(__file__).resolve()), *sys.argv[1:]])


def _robot_matrix(Gf, tx, ty, tz):
    """로봇 로컬 +X -> 월드 +Z (다른 검증 스크립트와 같은 규약)."""
    return Gf.Matrix4d(
        0.0, 0.0, 1.0, 0.0,
        1.0, 0.0, 0.0, 0.0,
        0.0, 1.0, 0.0, 0.0,
        tx, ty, tz, 1.0,
    )


def _camera_matrix(Gf, tilt_deg, height):
    """로봇 로컬 프레임(Z-up)에서, 전방을 tilt_deg 만큼 내려다보는 카메라 자세.

    USD 카메라는 자기 로컬 -Z 를 본다. 시선 d = (cos t, 0, -sin t) 이 되도록
    기저를 잡는다:  +X=(0,-1,0)  +Y=(sin t, 0, cos t)  +Z=-d
    (행벡터 규약이라 행이 곧 기저벡터다.)
    """
    t = math.radians(tilt_deg)
    s, c = math.sin(t), math.cos(t)
    return Gf.Matrix4d(
        0.0, -1.0, 0.0, 0.0,
        s,    0.0, c,   0.0,
        -c,   0.0, s,   0.0,
        CAM_FORWARD, 0.0, height, 1.0,
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
    only_tilt = "--tilt" in sys.argv[1:]

    from isaacsim import SimulationApp
    app = SimulationApp({"headless": True})
    try:
        import cv2
        import numpy as np
        import omni.timeline
        import omni.usd
        from isaacsim.core.api import World
        from isaacsim.sensors.camera import Camera
        from pxr import Gf, Usd, UsdGeom

        mm = json.loads(MAP_JSON.read_text(encoding="utf-8"))
        target = next(m for m in mm["markers"] if m["label"] == TARGET_LABEL)

        # 스윕할 구성. (라벨, 기울기, 높이, 가로해상도)
        configs = [("기준(에셋 실측)", BASE_TILT, BASE_HEIGHT, BASE_WIDTH)]
        configs += [(f"기울기 {t:.0f}도", t, BASE_HEIGHT, BASE_WIDTH)
                    for t in (20.0, 45.0, 60.0)]
        if not only_tilt:
            configs += [(f"높이 {h:.2f} m", BASE_TILT, h, BASE_WIDTH)
                        for h in (0.15, 0.25, 0.40)]
            configs += [(f"해상도 {w}", BASE_TILT, BASE_HEIGHT, w)
                        for w in (1280, 1920)]

        # cv2.putText 는 한글을 못 그린다. 이미지 라벨용 ASCII 슬러그를 따로 둔다.
        def _slug(tilt, height, wid):
            return f"tilt{tilt:.0f}_h{height:.2f}_w{wid}"

        slugs = [_slug(t, h, wd) for (_l, t, h, wd) in configs]

        SHOT_DIR.mkdir(parents=True, exist_ok=True)
        for old in SHOT_DIR.glob("*.png"):
            old.unlink()

        # 씬 구성 (물리 없음)
        stage = Usd.Stage.CreateInMemory()
        UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.y)
        UsdGeom.SetStageMetersPerUnit(stage, 1.0)
        w = UsdGeom.Xform.Define(stage, "/World").GetPrim()
        stage.SetDefaultPrim(w)
        env = UsdGeom.Xform.Define(stage, "/World/Env").GetPrim()
        env.GetReferences().AddReference(str(MARKER_STAGE))
        r = stage.DefinePrim(ROBOT_XFORM, "Xform")
        r.GetReferences().AddReference(str(ROBOT_USD))
        rx = UsdGeom.Xformable(r)
        rx.ClearXformOpOrder()
        rx.MakeMatrixXform().Set(
            _robot_matrix(Gf, target["x"], 0.0, target["z"] - DISTANCES[0]))

        # 구성마다 별도 카메라 프림 (해상도가 바뀌면 렌더 프로덕트를 새로 만들어야 한다)
        for i, (_lb, tilt, height, _wid) in enumerate(configs):
            cp = UsdGeom.Camera.Define(stage, f"{ROBOT_XFORM}/SweepCam_{i}")
            # 수평 FOV 를 기준값으로 고정: aperture 는 두고 focal 을 맞춘다
            ap = 3.896
            cp.CreateHorizontalApertureAttr(ap)
            cp.CreateFocalLengthAttr(ap / (2.0 * math.tan(math.radians(BASE_HFOV) / 2)))
            cp.CreateClippingRangeAttr(Gf.Vec2f(0.01, 50.0))
            cx = UsdGeom.Xformable(cp.GetPrim())
            cx.ClearXformOpOrder()
            cx.MakeMatrixXform().Set(_camera_matrix(Gf, tilt, height))

        tmp = WORK_DIR / "_sweep.usd"
        stage.GetRootLayer().Export(str(tmp))
        ctx = omni.usd.get_context()
        ctx.open_stage(str(tmp))
        for _ in range(40):
            app.update()
        live = ctx.get_stage()
        robot_op = UsdGeom.Xformable(
            live.GetPrimAtPath(ROBOT_XFORM)).GetOrderedXformOps()[0]

        # 렌더 프로덕트는 타임라인이 돌아야 프레임을 낸다(aruco_render_check.py 와 동일).
        # app.update() 만으로는 get_rgba() 가 계속 빈 배열이라 전 구성이 '검출 없음'이 된다.
        world = World(stage_units_in_meters=1.0, set_defaults=False)
        timeline = omni.timeline.get_timeline_interface()
        timeline.play()
        world.reset()

        detect = _make_detector(
            cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, mm["dictionary"])))

        cams = []
        for i, (_lb, _t, _h, wid) in enumerate(configs):
            cam = Camera(prim_path=f"{ROBOT_XFORM}/SweepCam_{i}",
                         resolution=(wid, int(wid * 3 / 4)))
            cam.initialize()
            cams.append(cam)
        for _ in range(40):
            world.step(render=True)

        results = []
        shots = {}
        for i, (label, tilt, height, wid) in enumerate(configs):
            cam = cams[i]
            rows = []
            for d in DISTANCES:
                timeline.stop()
                for _ in range(3):
                    app.update()
                robot_op.Set(_robot_matrix(Gf, target["x"], 0.0, target["z"] - d))
                timeline.play()
                world.reset()
                for _ in range(SETTLE):
                    world.step(render=True)
                rgba = cam.get_rgba()
                if rgba is None or getattr(rgba, "size", 0) == 0:
                    # 빈 프레임과 '보였지만 미검출'은 전혀 다른 실패다. 구분해 기록한다.
                    rows.append({"d": d, "ok": False, "px": None, "frame": "빈 프레임"})
                    continue
                gray = cv2.cvtColor(
                    np.asarray(rgba[..., :3], dtype=np.uint8), cv2.COLOR_RGB2GRAY)
                corners, ids = detect(gray)
                found = [] if ids is None else [int(v[0]) for v in ids]
                hit = target["id"] in found
                px = None
                if hit:
                    c = corners[found.index(target["id"])].reshape(-1, 2)
                    px = round(float(min(np.linalg.norm(c[1] - c[0]),
                                         np.linalg.norm(c[2] - c[1]))), 1)

                shot = cv2.cvtColor(np.asarray(rgba[..., :3], dtype=np.uint8),
                                    cv2.COLOR_RGB2BGR)
                if ids is not None:
                    cv2.aruco.drawDetectedMarkers(shot, corners, ids)
                cv2.putText(shot, f"{slugs[i]} d={d:.1f}m "
                                  f"{'OK ' + str(px) + 'px' if hit else 'MISS'}",
                            (8, 24), cv2.FONT_HERSHEY_SIMPLEX,
                            0.6 * wid / 640, (0, 255, 0) if hit else (0, 0, 255),
                            max(1, wid // 640))
                cv2.imwrite(str(SHOT_DIR /
                                f"{i:02d}_{slugs[i]}_d{d:.1f}_"
                                f"{'ok' if hit else 'MISS'}.png"), shot)
                shots.setdefault(i, {})[d] = shot
                rows.append({"d": d, "ok": hit, "px": px,
                             "gray": [int(gray.min()), int(gray.max())]})
            good = [x["d"] for x in rows if x["ok"]]
            results.append({
                "label": label, "tilt_deg": tilt, "height_m": height,
                "width_px": wid,
                "window": [min(good), max(good)] if good else None,
                "window_span_m": round(max(good) - min(good), 2) if good else 0.0,
                "rows": rows,
            })

        # 격자 몽타주: 행=구성, 열=거리. 한 장으로 비교할 수 있게.
        TW, TH, LW, HH = 256, 192, 210, 26
        canvas = np.full((HH + TH * len(configs), LW + TW * len(DISTANCES), 3),
                         30, dtype=np.uint8)
        for j, d in enumerate(DISTANCES):
            cv2.putText(canvas, f"{d:.1f}m", (LW + TW * j + 8, 18),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1)
        for i, res in enumerate(results):
            y = HH + TH * i
            cv2.putText(canvas, slugs[i], (6, y + TH // 2 - 6),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1)
            win = (f"{res['window'][0]:.1f}-{res['window'][1]:.1f}m"
                   if res["window"] else "none")
            cv2.putText(canvas, win, (6, y + TH // 2 + 14),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 220, 255), 1)
            for j, d in enumerate(DISTANCES):
                img = shots.get(i, {}).get(d)
                if img is None:
                    continue
                canvas[y:y + TH, LW + TW * j:LW + TW * (j + 1)] = \
                    cv2.resize(img, (TW, TH))
        MONTAGE = WORK_DIR / "camera_sweep_montage.png"
        cv2.imwrite(str(MONTAGE), canvas)

        REPORT.write_text(json.dumps(
            {"target": TARGET_LABEL, "hfov_deg": BASE_HFOV,
             "distances_m": list(DISTANCES), "configs": results},
            indent=2, ensure_ascii=False), encoding="utf-8")

        hdr = "  ".join(f"{d:>4.1f}" for d in DISTANCES)
        print(f"\n[스윕] 대상 {TARGET_LABEL}, 수평 FOV {BASE_HFOV}도 고정", flush=True)
        print(f"{'구성':<20}{hdr}   검출구간", flush=True)
        for res in results:
            cells = "  ".join(
                ("  O " if x["ok"] else "  . ") for x in res["rows"])
            win = (f"{res['window'][0]:.1f}~{res['window'][1]:.1f} m "
                   f"(폭 {res['window_span_m']:.1f})" if res["window"] else "검출 없음")
            print(f"{res['label']:<20}{cells}   {win}", flush=True)
        blank = sum(1 for res in results for x in res["rows"]
                    if x.get("frame") == "빈 프레임")
        print("\n(O=검출, .=실패, 숫자는 로봇 중심에서 마커까지 전방거리 m)", flush=True)
        if blank:
            print(f"!! 빈 프레임 {blank}개 — 렌더가 안 나온 것이므로 결과 무효", flush=True)
        print(f"[스윕] 리포트: {REPORT}", flush=True)
        print(f"[스윕] 개별 사진 {len(list(SHOT_DIR.glob('*.png')))}장: {SHOT_DIR}", flush=True)
        print(f"[스윕] 격자 몽타주: {MONTAGE}", flush=True)
        tmp.unlink(missing_ok=True)
    finally:
        app.close()


if __name__ == "__main__":
    main()
