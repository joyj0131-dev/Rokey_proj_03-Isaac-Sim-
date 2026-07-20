#!/usr/bin/env python3
"""Headless recorder for two_robot_carry_demo.

Runs the demo headless with RTX rendering, captures frames from a fixed camera
by wrapping app.update (so the demo's run_demo is reused unchanged), and writes
PNG frames to an output dir. A separate step assembles them into a GIF.

  cd /home/rokey/dev_ws/isaac_sim/isaacsim/_build/linux-x86_64/release
  ./python.sh /home/rokey/cobot3_ws/isaacpjt/Isaac_envo/record_carry_demo.py
"""

import math
import os
import sys
from pathlib import Path

FRAMES_DIR = Path(os.environ.get(
    "CARRY_FRAMES_DIR",
    "/tmp/claude-1000/-home-rokey-cobot3-ws/c73a0a43-5bcb-4140-bc6f-df6c4e17c9ca/scratchpad/frames",
))
CAPTURE_EVERY = 8
CAM_RES = (960, 540)
# 3/4 view from the rear-right, above, looking at the car / both robots.
CAM_EYE = (9.5, 3.2, 5.0)
CAM_TARGET = (2.0, 0.55, 8.3)

ISAAC_PYTHON = Path("/home/rokey/dev_ws/isaac_sim/isaacsim/_build/linux-x86_64/release/python.sh")


def _restart_with_isaac_python():
    if os.environ.get("CARB_APP_PATH"):
        return
    os.execv(str(ISAAC_PYTHON), [str(ISAAC_PYTHON), str(Path(__file__).resolve()), *sys.argv[1:]])


def _lookat_matrix(Gf, eye, target, up=(0.0, 1.0, 0.0)):
    e = Gf.Vec3d(*eye)
    f = (Gf.Vec3d(*target) - e).GetNormalized()          # view dir (camera -Z)
    r = (Gf.Cross(f, Gf.Vec3d(*up))).GetNormalized()      # camera +X
    u = Gf.Cross(r, f)                                    # camera +Y
    return Gf.Matrix4d(
        r[0], r[1], r[2], 0.0,
        u[0], u[1], u[2], 0.0,
        -f[0], -f[1], -f[2], 0.0,
        e[0], e[1], e[2], 1.0,
    )


def main():
    _restart_with_isaac_python()
    from isaacsim import SimulationApp

    app = SimulationApp({"headless": True})

    import numpy as np
    import imageio.v2 as imageio
    import omni.usd
    from pxr import Gf, UsdGeom
    from isaacsim.sensors.camera import Camera

    sys.path.insert(0, str(Path(__file__).resolve().parent))
    import two_robot_carry_demo as demo

    FRAMES_DIR.mkdir(parents=True, exist_ok=True)
    for old in FRAMES_DIR.glob("*.png"):
        old.unlink()

    state = {"cam": None, "n": 0, "frame": 0}
    orig_update = app.update

    def wrapped_update():
        orig_update()
        state["n"] += 1
        if state["cam"] is None and state["n"] == 45:
            stage = omni.usd.get_context().get_stage()
            cam_geom = UsdGeom.Camera.Define(stage, "/World/RecCam")
            xf = UsdGeom.Xformable(cam_geom.GetPrim())
            xf.ClearXformOpOrder()
            xf.MakeMatrixXform().Set(_lookat_matrix(Gf, CAM_EYE, CAM_TARGET))
            cam = Camera(prim_path="/World/RecCam", resolution=CAM_RES)
            cam.initialize()
            state["cam"] = cam
        cam = state["cam"]
        if cam is not None and state["n"] % CAPTURE_EVERY == 0:
            rgba = cam.get_rgba()
            if rgba is not None and getattr(rgba, "size", 0) and int(rgba[..., :3].max()) > 4:
                imageio.imwrite(
                    str(FRAMES_DIR / f"f{state['frame']:05d}.png"),
                    np.asarray(rgba[..., :3], dtype=np.uint8),
                )
                state["frame"] += 1

    app.update = wrapped_update

    try:
        demo.build_test_stage()
        demo.run_demo(app)
        print(f"FRAMES_CAPTURED={state['frame']}", flush=True)
        print(f"FRAMES_DIR={FRAMES_DIR}", flush=True)
    except Exception as exc:
        print(f"RECORD_EXCEPTION={type(exc).__name__}: {exc}", flush=True)
        raise
    finally:
        app.close()


if __name__ == "__main__":
    main()
