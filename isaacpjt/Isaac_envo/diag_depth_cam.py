#!/usr/bin/env python3
"""depth_stop_lift_test.py 의 뎁스 정지 미검출 원인 진단용 스크립트.

빌드된 depth_stop_lift_test.usd 를 열어 로봇을 전진시키면서 몇 스텝마다
뎁스 프레임 전체 통계(min/mean/percentile)와 카메라 월드 변환, RGB 스냅샷을
저장한다. roi_min_depth 가 시종일관 ~0.078m 로 고정되는 원인(자기 차체 오클루전인지,
카메라 각도/FOV 문제인지)을 실측으로 확인한다.
"""

import json
import math
import os
import sys
from pathlib import Path

WORK_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(WORK_DIR))

OUTPUT_USD = WORK_DIR / "depth_stop_lift_test.usd"
DIAG_DIR = WORK_DIR / "diag_depth_cam_out"

ISAAC_ROOT = Path("/home/rokey/dev_ws/isaac_sim/isaacsim/_build/linux-x86_64/release")
ISAAC_PYTHON = ISAAC_ROOT / "python.sh"

ROBOT_ROOT = "/World/Robot/base_link"
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
    try:
        import numpy as np
        import omni.physx
        import omni.timeline
        import omni.usd
        from isaacsim.core.prims import Articulation
        from isaacsim.sensors.camera import Camera
        from pxr import Gf, Usd, UsdGeom

        from depth_stop_detector import roi_min_depth
        from mecanum_drive import WHEEL_JOINTS, wheel_velocities_from_cmd_vel

        DIAG_DIR.mkdir(exist_ok=True)

        context = omni.usd.get_context()
        if not context.open_stage(str(OUTPUT_USD)):
            raise RuntimeError(f"failed to open test stage: {OUTPUT_USD}")
        for _ in range(12):
            app.update()
        stage = context.get_stage()
        timeline = omni.timeline.get_timeline_interface()
        timeline.play()
        for _ in range(12):
            app.update()

        robot = Articulation(ROBOT_ROOT)
        robot.initialize()
        physx = omni.physx.get_physx_interface()
        joint_positions = robot.get_joint_positions()
        wheel_idx = {w: robot.dof_names.index(j) for w, j in WHEEL_JOINTS.items()}
        velocity_targets = np.zeros(joint_positions.shape, dtype=np.float32)

        def rigid_position(path):
            value = physx.get_rigidbody_transformation(path)
            return tuple(float(x) for x in value["position"])

        def drive(vx, vy, wz):
            for wname, omega in wheel_velocities_from_cmd_vel(vx, vy, wz).items():
                i = wheel_idx[wname]
                if velocity_targets.ndim == 2:
                    velocity_targets[0, i] = omega
                else:
                    velocity_targets[i] = omega
            robot.set_joint_velocity_targets(velocity_targets)

        drive(0.0, 0.0, 0.0)
        for _ in range(120):
            app.update()

        cam = Camera(prim_path=CAM_FRONT, resolution=CAM_RES)
        cam.initialize()
        cam.add_distance_to_image_plane_to_frame()
        for _ in range(30):
            app.update()

        cam_prim = stage.GetPrimAtPath(CAM_FRONT)
        usd_camera = UsdGeom.Camera(cam_prim)
        print(f"[diag] clippingRange={usd_camera.GetClippingRangeAttr().Get()}", flush=True)
        print(f"[diag] focalLength={usd_camera.GetFocalLengthAttr().Get()}", flush=True)
        print(f"[diag] horizontalAperture={usd_camera.GetHorizontalApertureAttr().Get()}", flush=True)

        rgba = cam.get_rgba()
        rgba_hw = tuple(int(v) for v in rgba.shape[:2])

        samples = []
        drive(0.4, 0.0, 0.0)
        for step in range(1, 901):
            app.update()
            if step in (1, 30, 60, 90, 120, 150, 180, 210, 240, 270, 300, 400, 500, 600, 700, 800, 900):
                depth = cam.get_depth()
                arr = np.asarray(depth, dtype=np.float64).squeeze()
                if arr.shape != rgba_hw:
                    arr = arr.T if arr.shape == rgba_hw[::-1] else arr
                finite = arr[np.isfinite(arr)]
                cam_world = UsdGeom.Xformable(cam_prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
                cam_pos = tuple(float(x) for x in cam_world.ExtractTranslation())
                robot_pos = rigid_position(ROBOT_ROOT)
                roi_val = roi_min_depth(arr)
                sample = {
                    "step": step,
                    "robot_z": round(robot_pos[2], 3),
                    "cam_world_pos": tuple(round(v, 3) for v in cam_pos),
                    "depth_min": float(finite.min()) if finite.size else None,
                    "depth_mean": float(finite.mean()) if finite.size else None,
                    "depth_p05": float(np.percentile(finite, 5)) if finite.size else None,
                    "depth_p50": float(np.percentile(finite, 50)) if finite.size else None,
                    "roi_min_depth": None if math.isinf(roi_val) else roi_val,
                    "finite_fraction": float(finite.size) / float(arr.size),
                }
                samples.append(sample)
                print(f"[diag] {json.dumps(sample)}", flush=True)
                # save rgba + a simple depth visualization for this step
                rgba_now = cam.get_rgba()
                np.save(DIAG_DIR / f"rgba_step{step:04d}.npy", rgba_now)
                np.save(DIAG_DIR / f"depth_step{step:04d}.npy", arr)

        (DIAG_DIR / "samples.json").write_text(json.dumps(samples, indent=2), encoding="utf-8")
        print(f"[diag] wrote {len(samples)} samples to {DIAG_DIR}/samples.json", flush=True)
        timeline.stop()
        app.update()
    finally:
        app.close()


if __name__ == "__main__":
    main()
