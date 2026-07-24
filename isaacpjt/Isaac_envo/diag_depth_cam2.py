#!/usr/bin/env python3
"""RGB/뎁스 스냅샷을 PNG로 저장해 카메라가 실제로 무엇을 보는지 육안 확인한다."""

import math
import os
import sys
from pathlib import Path

WORK_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(WORK_DIR))

OUTPUT_USD = WORK_DIR / "depth_stop_lift_test.usd"
DIAG_DIR = WORK_DIR / "diag_depth_cam_out2"

ISAAC_ROOT = Path("/home/rokey/dev_ws/isaac_sim/isaacsim/_build/linux-x86_64/release")
ISAAC_PYTHON = ISAAC_ROOT / "python.sh"

ROBOT_ROOT = "/World/Robot/base_link"
CAM_FRONT = "/World/Robot/cam_front_link/depth_cam_front/Camera_Pseudo_Depth_Front"
CAM_RES = (640, 480)


def _restart_with_isaac_python():
    if os.environ.get("CARB_APP_PATH"):
        return
    os.execv(str(ISAAC_PYTHON), [str(ISAAC_PYTHON), str(Path(__file__).resolve()), *sys.argv[1:]])


def _save_png(path, rgb_uint8):
    import numpy as np
    h, w = rgb_uint8.shape[:2]
    import struct, zlib

    def chunk(tag, data):
        return (struct.pack(">I", len(data)) + tag + data +
                struct.pack(">I", zlib.crc32(tag + data) & 0xffffffff))

    raw = b"".join(b"\x00" + rgb_uint8[y].tobytes() for y in range(h))
    compressed = zlib.compress(raw, 9)
    sig = b"\x89PNG\r\n\x1a\n"
    ihdr = struct.pack(">IIBBBBB", w, h, 8, 2, 0, 0, 0)
    png = sig + chunk(b"IHDR", ihdr) + chunk(b"IDAT", compressed) + chunk(b"IEND", b"")
    Path(path).write_bytes(png)


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

        from mecanum_drive import WHEEL_JOINTS, wheel_velocities_from_cmd_vel

        DIAG_DIR.mkdir(exist_ok=True)

        context = omni.usd.get_context()
        if not context.open_stage(str(OUTPUT_USD)):
            raise RuntimeError(f"failed to open test stage: {OUTPUT_USD}")
        for _ in range(12):
            app.update()
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

        def rigid_position(path):
            value = physx.get_rigidbody_transformation(path)
            return tuple(float(x) for x in value["position"])

        drive(0.4, 0.0, 0.0)
        capture_steps = {1, 150, 300, 350, 400, 450, 500, 600, 900}
        for step in range(1, 901):
            app.update()
            if step in capture_steps:
                rgba = np.asarray(cam.get_rgba())
                rgb = rgba[:, :, :3].astype(np.uint8)
                _save_png(DIAG_DIR / f"rgb_step{step:04d}.png", rgb)

                depth = np.asarray(cam.get_depth(), dtype=np.float64).squeeze()
                finite = depth[np.isfinite(depth)]
                dmin = float(finite.min()) if finite.size else float("nan")
                dmax = float(finite.max()) if finite.size else float("nan")
                norm = np.clip((depth - dmin) / max(dmax - dmin, 1e-6), 0, 1)
                norm = np.nan_to_num(norm, nan=1.0, posinf=1.0)
                gray = (255 * (1.0 - norm)).astype(np.uint8)  # 가까울수록 밝게
                depth_rgb = np.stack([gray, gray, gray], axis=-1)
                _save_png(DIAG_DIR / f"depth_step{step:04d}.png", depth_rgb)

                z = rigid_position(ROBOT_ROOT)[2]
                print(f"[diag2] step={step} robot_z={z:.3f} depth_min={dmin:.4f} depth_max={dmax:.4f}", flush=True)

        timeline.stop()
        app.update()
        print(f"[diag2] saved images to {DIAG_DIR}", flush=True)
    finally:
        app.close()


if __name__ == "__main__":
    main()
