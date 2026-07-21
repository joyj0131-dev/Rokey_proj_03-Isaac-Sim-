#!/usr/bin/env python3
"""M1 최종 검증: 렌더된 화면에서 마커가 실제로 검출되는가.

PNG 왕복 검출(build_marker_textures.py --check)이 통과해도, USD 텍스처 바인딩이
잘못되면 씬에서는 단색으로 보인다. 이 스크립트는 마커 바로 위에 카메라를 놓고
한 프레임 렌더해서 cv2 로 검출한다. 여기까지 통과해야 M1 이 끝난다.

카메라 배치는 로봇과 무관하다 — 로봇 카메라 사양이 미정이므로(팀원 제작 중),
여기서는 마커를 위에서 내려다보는 검사용 카메라만 쓴다.

실행:
    python3 aruco_render_check.py            # headless, 마커 3장 표본
    python3 aruco_render_check.py --all      # 44장 전부
    python3 aruco_render_check.py --gui      # 창 띄워서 눈으로도 확인
"""

import json
import os
import sys
from pathlib import Path

WORK_DIR = Path(__file__).resolve().parent
MAP_JSON = (WORK_DIR.parents[1] / "src" / "parkbot_aruco"
            / "data" / "marker_map.json")  # 지도는 ROS 패키지가 소유
STAGE_USD = WORK_DIR / "parking" / "parking_environment_marker_preview.usd"
OUT_DIR = WORK_DIR / "aruco_render_check"
REPORT = WORK_DIR / "aruco_render_check_report.json"
ISAAC_PYTHON = Path(
    "/home/rokey/dev_ws/isaac_sim/isaacsim/_build/linux-x86_64/release/python.sh"
)

CAM_HEIGHT = 0.60          # 마커 위 높이 [m]. 검사용이라 넉넉히 잡는다
RESOLUTION = (640, 480)
WARMUP_FRAMES = 60         # 첫 프레임은 비어 있다. 라이다 데모와 같은 이유
SAMPLE_IDS = (0, 30, 60)   # slot / crossing / handoff_lane 각 1장


def _restart_with_isaac_python():
    if os.environ.get("CARB_APP_PATH"):
        return
    if not ISAAC_PYTHON.is_file():
        raise FileNotFoundError(f"Isaac python.sh를 찾을 수 없습니다: {ISAAC_PYTHON}")
    os.execv(
        str(ISAAC_PYTHON),
        [str(ISAAC_PYTHON), str(Path(__file__).resolve()), *sys.argv[1:]],
    )


def _make_detector(dictionary):
    """cv2 4.5 / 4.7+ 양쪽 API 흡수 (build_marker_textures.py 와 동일 규칙)."""
    import cv2

    if hasattr(cv2.aruco, "ArucoDetector"):
        det = cv2.aruco.ArucoDetector(dictionary, cv2.aruco.DetectorParameters())
        return lambda img: det.detectMarkers(img)[:2]
    params = cv2.aruco.DetectorParameters_create()
    return lambda img: cv2.aruco.detectMarkers(img, dictionary, parameters=params)[:2]


def main():
    _restart_with_isaac_python()

    gui = "--gui" in sys.argv[1:]
    check_all = "--all" in sys.argv[1:]

    from isaacsim import SimulationApp

    app = SimulationApp({"headless": not gui})
    try:
        import cv2
        import numpy as np
        import omni.usd
        from pxr import Gf, UsdGeom
        from isaacsim.sensors.camera import Camera

        if not MAP_JSON.is_file():
            raise FileNotFoundError(
                f"{MAP_JSON} 가 없습니다. build_marker_textures.py 를 먼저 실행하세요."
            )
        marker_map = json.loads(MAP_JSON.read_text(encoding="utf-8"))
        by_id = {m["id"]: m for m in marker_map["markers"]}
        targets = sorted(by_id) if check_all else [i for i in SAMPLE_IDS if i in by_id]

        context = omni.usd.get_context()
        context.open_stage(str(STAGE_USD))
        for _ in range(30):
            app.update()
        stage = context.get_stage()

        # 마커를 수직으로 내려다보는 검사용 카메라.
        cam_geom = UsdGeom.Camera.Define(stage, "/World/ArucoCheckCam")
        cam_geom.CreateFocalLengthAttr(24.0)
        cam_geom.CreateClippingRangeAttr(Gf.Vec2f(0.01, 100.0))
        xf = UsdGeom.Xformable(cam_geom.GetPrim())
        xf.ClearXformOpOrder()
        move = xf.AddTranslateOp()
        rot = xf.AddRotateXYZOp()
        rot.Set(Gf.Vec3f(-90.0, 0.0, 0.0))     # 카메라 -Z 를 월드 -Y(아래)로

        # 렌더 프로덕트는 타임라인이 돌아야 프레임을 낸다. 라이다 데모(run_ceiling_lidar_demo)도
        # attach_annotator 전에 play() + reset() 을 한다. 이걸 빼면 get_rgba()가 계속 빈 배열이다.
        import omni.timeline
        from isaacsim.core.api import World

        world = World(stage_units_in_meters=1.0, set_defaults=False)
        omni.timeline.get_timeline_interface().play()
        world.reset()

        camera = Camera(prim_path="/World/ArucoCheckCam", resolution=RESOLUTION)
        camera.initialize()
        for _ in range(WARMUP_FRAMES):
            world.step(render=True)

        probe = camera.get_rgba()
        print(f"[render] 워밍업 후 프레임: "
              f"{'None' if probe is None else getattr(probe, 'shape', '?')}", flush=True)

        dictionary = cv2.aruco.getPredefinedDictionary(
            getattr(cv2.aruco, marker_map["dictionary"])
        )
        detect = _make_detector(dictionary)

        OUT_DIR.mkdir(parents=True, exist_ok=True)
        for old in OUT_DIR.glob("*.png"):
            old.unlink()

        results = []
        for marker_id in targets:
            m = by_id[marker_id]
            move.Set(Gf.Vec3d(m["x"], CAM_HEIGHT, m["z"]))
            # 카메라를 옮긴 직후 렌더하면 이전 프레임이 나온다. 몇 스텝 돌려야 한다.
            for _ in range(12):
                world.step(render=True)

            rgba = camera.get_rgba()
            if rgba is None or getattr(rgba, "size", 0) == 0:
                results.append({
                    "id": marker_id, "label": m["label"], "kind": m["kind"],
                    "ok": False, "detected": [], "reason": "빈 프레임",
                })
                continue

            rgb = np.asarray(rgba[..., :3], dtype=np.uint8)
            gray = cv2.cvtColor(rgb, cv2.COLOR_RGB2GRAY)
            _corners, ids = detect(gray)
            found = [] if ids is None else [int(v[0]) for v in ids]
            ok = marker_id in found
            results.append({
                "id": marker_id, "label": m["label"], "kind": m["kind"],
                "ok": ok, "detected": found,
                "gray_min": int(gray.min()), "gray_max": int(gray.max()),
            })
            cv2.imwrite(
                str(OUT_DIR / f"marker_{marker_id:03d}_{'ok' if ok else 'FAIL'}.png"),
                cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR),
            )

        passed = sum(1 for r in results if r["ok"])
        report = {
            "stage": str(STAGE_USD),
            "dictionary": marker_map["dictionary"],
            "camera_height_m": CAM_HEIGHT,
            "resolution": list(RESOLUTION),
            "checked": len(results),
            "passed": passed,
            "all_passed": passed == len(results),
            "results": results,
        }
        REPORT.write_text(json.dumps(report, indent=2, ensure_ascii=False), encoding="utf-8")

        print(f"[render] 검사 {len(results)}장 중 {passed}장 검출", flush=True)
        for r in results:
            mark = "OK  " if r["ok"] else "FAIL"
            extra = f" 밝기 {r.get('gray_min')}~{r.get('gray_max')}"
            print(f"  {mark} ID {r['id']:>3} {r.get('label','')}"
                  f" 검출={r.get('detected', [])}{extra}", flush=True)
        print(f"[render] 이미지: {OUT_DIR}", flush=True)
        print(f"RENDER_CHECK_PASSED={passed == len(results)}", flush=True)

        if gui:
            while app.is_running():
                app.update()
    finally:
        app.close()


if __name__ == "__main__":
    main()
