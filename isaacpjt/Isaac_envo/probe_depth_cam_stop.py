#!/usr/bin/env python3
"""hwia_depth_cam_mecha_roller.usd 의 전방 뎁스캠이 실제로 유효한
distance_to_image_plane 값을 내는지, 프레임 축 순서가 어느 쪽인지 실측한다.

빈 바닥 + 로봇만 있는 최소 씬(verify_depth_cam_mecha.py 1단계와 동일한 방식).
차량/주차장은 필요 없다 — 카메라 센서 자체의 유효성만 확인한다.

실행:
    python3 probe_depth_cam_stop.py
"""

import json
import math
import os
import sys
from pathlib import Path

WORK_DIR = Path(__file__).resolve().parent
ROBOT_USD = (WORK_DIR.parent / "hwia_parking_robot_final_caster_package"
             / "hwia_depth_cam_mecha_roller.usd")
REPORT = WORK_DIR / "probe_depth_cam_stop_report.json"
ISAAC_PYTHON = Path(
    "/home/rokey/dev_ws/isaac_sim/isaacsim/_build/linux-x86_64/release/python.sh"
)

ROBOT_XFORM = "/World/Robot"
ROBOT_JOINTS = "/World/Robot/joints"
CAM_FRONT = "/World/Robot/cam_front_link/depth_cam_front/Camera_Pseudo_Depth_Front"
CAM_RES = (640, 480)


def _restart_with_isaac_python():
    if os.environ.get("CARB_APP_PATH"):
        return
    os.execv(str(ISAAC_PYTHON), [str(ISAAC_PYTHON), str(Path(__file__).resolve()), *sys.argv[1:]])


def main():
    _restart_with_isaac_python()

    from isaacsim import SimulationApp
    app = SimulationApp({"headless": True})
    report = {}
    try:
        import numpy as np
        import omni.timeline
        import omni.usd
        from isaacsim.core.api import World
        from isaacsim.sensors.camera import Camera
        from pxr import Gf, Usd, UsdGeom, UsdPhysics

        from depth_stop_detector import roi_min_depth
        from mecanum_drive import configure_hub_drives

        if not ROBOT_USD.is_file():
            raise FileNotFoundError(f"robot asset not found: {ROBOT_USD}")

        stage = Usd.Stage.CreateInMemory()
        UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.y)
        UsdGeom.SetStageMetersPerUnit(stage, 1.0)
        w = UsdGeom.Xform.Define(stage, "/World").GetPrim()
        stage.SetDefaultPrim(w)

        ground = UsdGeom.Cube.Define(stage, "/World/Ground")
        ground.CreateSizeAttr(1.0)
        gx = UsdGeom.Xformable(ground)
        gx.AddTranslateOp().Set(Gf.Vec3d(0.0, -0.5, 0.0))
        gx.AddScaleOp().Set(Gf.Vec3f(40.0, 1.0, 40.0))
        UsdPhysics.CollisionAPI.Apply(ground.GetPrim())

        scene = UsdPhysics.Scene.Define(stage, "/World/PhysicsScene")
        scene.CreateGravityDirectionAttr(Gf.Vec3f(0.0, -1.0, 0.0))
        scene.CreateGravityMagnitudeAttr(9.81)

        robot = stage.DefinePrim(ROBOT_XFORM, "Xform")
        robot.GetReferences().AddReference(str(ROBOT_USD))
        rxf = UsdGeom.Xformable(robot)
        rxf.ClearXformOpOrder()
        rxf.MakeMatrixXform().Set(Gf.Matrix4d(
            0.0, 0.0, 1.0, 0.0,
            1.0, 0.0, 0.0, 0.0,
            0.0, 1.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 1.0,
        ))

        tmp = WORK_DIR / "_probe_depth_cam_stop.usd"
        stage.GetRootLayer().Export(str(tmp))
        ctx = omni.usd.get_context()
        ctx.open_stage(str(tmp))
        for _ in range(30):
            app.update()
        stage = ctx.get_stage()

        configure_hub_drives(stage, ROBOT_JOINTS)
        world = World(stage_units_in_meters=1.0, set_defaults=False)
        omni.timeline.get_timeline_interface().play()
        world.reset()
        for _ in range(30):
            world.step(render=False)

        cam = Camera(prim_path=CAM_FRONT, resolution=CAM_RES)
        cam.initialize()
        cam.add_distance_to_image_plane_to_frame()
        # M1 교훈: 타임라인이 돌아야 렌더 프로덕트가 프레임을 낸다. 몇 스텝 워밍업 필요.
        for _ in range(60):
            world.step(render=True)

        rgba = cam.get_rgba()
        depth = cam.get_depth()

        rgba_shape = tuple(int(v) for v in getattr(rgba, "shape", ()))
        depth_shape = tuple(int(v) for v in getattr(depth, "shape", ()))
        report["rgba_frame_shape"] = rgba_shape
        report["depth_frame_shape"] = depth_shape

        if depth is None or not getattr(depth, "size", 0):
            report["depth_axis_order"] = "unknown"
            report["finite_fraction"] = 0.0
            ok = False
        else:
            depth_arr = np.asarray(depth, dtype=np.float64).squeeze()
            # rgba는 (height, width, 4)가 확정값. depth의 앞 두 축이 rgba와 같은
            # 순서면 (height, width), 뒤집혀 있으면 (width, height)로 판단한다.
            if depth_arr.ndim == 2 and rgba_shape[:2] == depth_arr.shape:
                axis_order = "height_width"
                depth_hw = depth_arr
            elif depth_arr.ndim == 2 and rgba_shape[:2] == depth_arr.shape[::-1]:
                axis_order = "width_height"
                depth_hw = depth_arr.T
            else:
                axis_order = f"unexpected ndim={depth_arr.ndim} shape={depth_arr.shape}"
                depth_hw = depth_arr if depth_arr.ndim == 2 else depth_arr.reshape(-1, 1)
            report["depth_axis_order"] = axis_order

            finite = depth_hw[np.isfinite(depth_hw)]
            report["finite_fraction"] = float(finite.size) / float(depth_hw.size)
            report["depth_min_m"] = float(finite.min()) if finite.size else None
            report["depth_max_m"] = float(finite.max()) if finite.size else None
            report["depth_mean_m"] = float(finite.mean()) if finite.size else None
            report["roi_min_depth_m"] = roi_min_depth(depth_hw)
            ok = finite.size > 0 and math.isfinite(report["roi_min_depth_m"])

        report["ok"] = ok
        REPORT.write_text(json.dumps(report, indent=2, ensure_ascii=False), encoding="utf-8")
        print(f"[probe] rgba shape={rgba_shape} depth shape={depth_shape}", flush=True)
        print(f"[probe] axis_order={report.get('depth_axis_order')}", flush=True)
        print(f"[probe] finite_fraction={report.get('finite_fraction')}", flush=True)
        print(f"[probe] roi_min_depth_m={report.get('roi_min_depth_m')}", flush=True)
        print(f"[probe] 리포트: {REPORT}", flush=True)
        print(f"DEPTH_PROBE_OK={ok}", flush=True)
        tmp.unlink(missing_ok=True)
        if not ok:
            raise SystemExit(1)
    finally:
        app.close()


if __name__ == "__main__":
    main()
