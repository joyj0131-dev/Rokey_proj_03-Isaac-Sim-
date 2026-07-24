#!/usr/bin/env python3
"""뒷축 통과 시점 부근에서 전체 폭(가로) 컬럼별 최소 뎁스를 확인 — 바퀴가 좌우 어디에
잡히는지 실측."""

import os
import sys
from pathlib import Path

WORK_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(WORK_DIR))

OUTPUT_USD = WORK_DIR / "depth_stop_lift_test.usd"
ISAAC_ROOT = Path("/home/rokey/dev_ws/isaac_sim/isaacsim/_build/linux-x86_64/release")
ISAAC_PYTHON = ISAAC_ROOT / "python.sh"

ROBOT_ROOT = "/World/Robot/base_link"
CAM_FRONT = "/World/Robot/cam_front_link/depth_cam_front/Camera_Pseudo_Depth_Front"
CAM_RES = (640, 480)
TELEPORT_BACK_Z = 4.5
CAPTURE_STEPS = set(range(1, 361, 15))


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

        from mecanum_drive import WHEEL_JOINTS, wheel_velocities_from_cmd_vel

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

        cam = Camera(prim_path=CAM_FRONT, resolution=CAM_RES)
        cam.initialize()
        cam.add_distance_to_image_plane_to_frame()
        for _ in range(30):
            app.update()

        drive(0.4, 0.0, 0.0)
        for step in range(1, 361):
            app.update()
            if step in CAPTURE_STEPS:
                depth = np.asarray(cam.get_depth(), dtype=np.float64).squeeze()
                h, w = depth.shape
                z = rigid_position(ROBOT_ROOT)[2]
                # 세로 하단 50%(로봇 바로 앞 바닥+측면), 가로는 8구간으로 나눠 각 구간 최소값
                row_lo = int(h * 0.4)
                band = depth[row_lo:, :]
                n_cols = 10
                col_edges = [int(w * i / n_cols) for i in range(n_cols + 1)]
                mins = []
                for i in range(n_cols):
                    seg = band[:, col_edges[i]:col_edges[i + 1]]
                    finite = seg[np.isfinite(seg)]
                    mins.append(round(float(finite.min()), 3) if finite.size else None)
                print(f"step={step:3d} z={z:.3f} col_mins(10 bands L->R)={mins}", flush=True)

        timeline.stop()
        app.update()
    finally:
        app.close()


if __name__ == "__main__":
    main()
