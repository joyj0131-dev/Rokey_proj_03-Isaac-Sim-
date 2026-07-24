#!/usr/bin/env python3
"""좌측 사이드 뎁스캠(Camera_Pseudo_Depth_Left)이 뒷축 통과 시점에 무엇을 보는지 실측.

로봇 길이 중심(로컬 x=0)에 장착 -> world Z 로 ROBOT_ROOT 와 거의 동일 위치.
차량 뒷바퀴(왼쪽)가 이 카메라 옆을 지나가는 순간 근접 신호가 뚜렷하게 잡히는지 확인."""

import math
import os
import sys
from pathlib import Path

WORK_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(WORK_DIR))

OUTPUT_USD = WORK_DIR / "depth_stop_lift_test.usd"
DIAG_DIR = WORK_DIR / "diag_side_cam_out"
ISAAC_ROOT = Path("/home/rokey/dev_ws/isaac_sim/isaacsim/_build/linux-x86_64/release")
ISAAC_PYTHON = ISAAC_ROOT / "python.sh"

ROBOT_ROOT = "/World/Robot/base_link"
CAM_LEFT = "/World/Robot/cam_side_left_link/depth_cam_left/Camera_Pseudo_Depth_Left"
CAM_RES = (640, 480)
TELEPORT_BACK_Z = 4.0
CAPTURE_STEPS = set(range(1, 421, 10))


def _restart_with_isaac_python():
    if os.environ.get("CARB_APP_PATH"):
        return
    os.execv(str(ISAAC_PYTHON), [str(ISAAC_PYTHON), str(Path(__file__).resolve()), *sys.argv[1:]])


def _save_png(path, rgb_uint8):
    import struct, zlib
    h, w = rgb_uint8.shape[:2]

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

        from depth_stop_detector import roi_min_depth
        from mecanum_drive import WHEEL_JOINTS, wheel_velocities_from_cmd_vel

        DIAG_DIR.mkdir(exist_ok=True)

        context = omni.usd.get_context()
        context.open_stage(str(OUTPUT_USD))
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

        cur = robot.get_world_poses()
        pos = np.array(cur[0], dtype=np.float64, copy=True)
        pos[..., 2] = TELEPORT_BACK_Z
        robot.set_world_poses(positions=pos)
        for _ in range(20):
            app.update()

        def drive(vx, vy, wz):
            for wname, omega in wheel_velocities_from_cmd_vel(vx, vy, wz).items():
                i = wheel_idx[wname]
                if velocity_targets.ndim == 2:
                    velocity_targets[0, i] = omega
                else:
                    velocity_targets[i] = omega
            robot.set_joint_velocity_targets(velocity_targets)

        def rigid_position(path):
            value = physx.get_rigidbody_transformation(path)
            return tuple(float(x) for x in value["position"])

        drive(0.0, 0.0, 0.0)
        for _ in range(120):
            app.update()

        cam = Camera(prim_path=CAM_LEFT, resolution=CAM_RES)
        cam.initialize()
        cam.add_distance_to_image_plane_to_frame()
        for _ in range(30):
            app.update()

        cam_prim_path = CAM_LEFT
        print(f"[side] cam world pos initial: {rigid_position(ROBOT_ROOT)}", flush=True)

        drive(0.4, 0.0, 0.0)
        snap_done = set()
        for step in range(1, 421):
            app.update()
            if step in CAPTURE_STEPS:
                depth = np.asarray(cam.get_depth(), dtype=np.float64).squeeze()
                finite = depth[np.isfinite(depth)]
                z = rigid_position(ROBOT_ROOT)[2]
                dmin = float(finite.min()) if finite.size else float("nan")
                dmean = float(finite.mean()) if finite.size else float("nan")
                roi_full = roi_min_depth(depth, roi_frac=(0.0, 1.0, 0.0, 1.0))
                roi_center = roi_min_depth(depth, roi_frac=(0.3, 0.7, 0.3, 0.7))
                roi_bottom = roi_min_depth(depth, roi_frac=(0.2, 0.8, 0.5, 1.0))
                print(f"[side] step={step:3d} z={z:.3f} full_min={dmin:.3f} mean={dmean:.3f} "
                      f"roi_full={roi_full:.3f} roi_center={roi_center:.3f} roi_bottom={roi_bottom:.3f}",
                      flush=True)
                if step in (1, 150, 180, 200, 220, 240, 270, 300, 400):
                    rgba = np.asarray(cam.get_rgba())
                    _save_png(DIAG_DIR / f"left_rgb_step{step:04d}.png", rgba[:, :, :3].astype(np.uint8))
                    dmax = float(finite.max()) if finite.size else 1.0
                    norm = np.clip((depth - dmin) / max(dmax - dmin, 1e-6), 0, 1)
                    norm = np.nan_to_num(norm, nan=1.0, posinf=1.0)
                    gray = (255 * (1.0 - norm)).astype(np.uint8)
                    _save_png(DIAG_DIR / f"left_depth_step{step:04d}.png", np.stack([gray]*3, axis=-1))

        timeline.stop()
        app.update()
    finally:
        app.close()


if __name__ == "__main__":
    main()
