#!/usr/bin/env python3
"""Attach the working depth Camera sensor prims to the left/right camera
housings (cam_side_left_link / cam_side_right_link) -- the ones that now
have an actual camera-shaped mesh (housing + sensor strip + 3 lens
barrels) from add_camera_mesh.py -- and bake a new, self-contained USD
asset.

Run add_camera_mesh.py on the URDF FIRST, re-import that URDF into Isaac
Sim to produce the USD, then point SOURCE_USD below at that imported file
before running this script.

Why a plain UsdGeom.Camera and not a separate RealSense/Kinect asset:
  Isaac Sim gets RGB *and* depth from the same UsdGeom.Camera prim via a
  Replicator render product + annotator (rgb, distance_to_image_plane, ...).
  No branded sensor asset is required for the depth topic to work -- the
  camera-shaped mesh you now have is the visible housing/lens, and this
  script just adds the invisible working Camera prim positioned at the
  center lens, as if "inside" that lens barrel.

IMPORTANT - verify after running:
  The exact aim direction (which way local -Z points once the parent
  joint's rpy is applied) could not be verified without opening this in
  Isaac Sim. A single tunable `AIM_OFFSET_DEG` is exposed below --
  open the baked asset, look through each camera (or check the RGB render
  product), and adjust that number (multiples of 90 deg about local X)
  until both cameras look outward/downward at the tire area instead of
  into the robot body.

Run:
  cd /home/rokey/dev_ws/isaac_sim/isaacsim/_build/linux-x86_64/release
  ./python.sh /home/rokey/cobot3_ws/isaacpjt/hwia_parking_robot_final_caster_package/add_depth_cameras.py
"""

import os
import sys
from pathlib import Path

PKG_DIR = Path(__file__).resolve().parent

# ---- INPUT: which baked asset to attach the cameras to -----------------
# Point this at the USD produced by re-importing
# hwia_parking_robot_final_caster_camera_mesh.urdf (the version with the
# real camera-shaped mesh) into Isaac Sim. Update this path once you've
# done that import + export.
SOURCE_USD = PKG_DIR / "hwia_parking_robot_final_caster_camera_mesh.usd"
OUTPUT_USD = PKG_DIR / "hwia_parking_robot_final_caster_camera_mesh_depth_cam.usd"

ISAAC_PYTHON = Path("/home/rokey/dev_ws/isaac_sim/isaacsim/_build/linux-x86_64/release/python.sh")

# Link names as authored in the URDF (see Sensors section).
LEFT_LINK_NAME = "cam_side_left_link"
RIGHT_LINK_NAME = "cam_side_right_link"

# ---- Depth camera intrinsics (rough RealSense D435-like defaults) ------
HORIZONTAL_APERTURE = 20.955      # mm, standard USD/Isaac camera default sensor width
FOCAL_LENGTH = 12.6                # mm -> ~87 deg horizontal FOV w/ the aperture above
CLIPPING_RANGE = (0.05, 3.0)        # meters; tight near range since this looks under a car
RESOLUTION_HINT = (640, 480)        # just metadata/comment; actual render product res is
                                     # set later when you create the render product in Isaac Sim

# Local offset (meters) of the sensor Camera prim relative to its parent
# link's origin. Matches the center lens ("*_lens_1") placed by
# add_camera_mesh.py at local x=0.0185, y=0, z=0 -- i.e. the RGB lens of
# the 3-lens strip. Update this if you change STRIP_X/LENS_X over there.
CAMERA_LOCAL_OFFSET = (0.0185, 0.0, 0.0)

# One tunable knob: rotation (deg) about local X applied on TOP of whatever
# rotation the parent joint already gives the link. Try 0 first, then 90,
# 180, 270 while watching the camera's RGB viewport until it looks outward
# instead of into the chassis.
AIM_OFFSET_DEG = 0.0


def _restart_with_isaac_python():
    if os.environ.get("CARB_APP_PATH"):
        return
    if not ISAAC_PYTHON.is_file():
        raise FileNotFoundError(f"Isaac Sim python.sh not found: {ISAAC_PYTHON}")
    os.execv(str(ISAAC_PYTHON), [str(ISAAC_PYTHON), str(Path(__file__).resolve()), *sys.argv[1:]])


def _find_prim_by_name(stage, name):
    """Search the whole stage for a prim with this exact name (robust to
    however the URDF importer nested/renamed the hierarchy)."""
    matches = [p for p in stage.Traverse() if p.GetName() == name]
    if not matches:
        return None
    if len(matches) > 1:
        print(f"WARNING: multiple prims named '{name}' found, using the first: "
              f"{[m.GetPath().pathString for m in matches]}", flush=True)
    return matches[0]


def _add_depth_camera(stage, parent_prim, cam_name="depth_cam"):
    from pxr import UsdGeom, Gf

    cam_path = parent_prim.GetPath().AppendChild(cam_name)
    camera = UsdGeom.Camera.Define(stage, cam_path)

    camera.CreateFocalLengthAttr(FOCAL_LENGTH)
    camera.CreateHorizontalApertureAttr(HORIZONTAL_APERTURE)
    camera.CreateClippingRangeAttr(Gf.Vec2f(*CLIPPING_RANGE))
    camera.CreateProjectionAttr(UsdGeom.Tokens.perspective)

    xform = UsdGeom.Xformable(camera.GetPrim())
    xform.ClearXformOpOrder()
    xform.AddTranslateOp().Set(Gf.Vec3d(*CAMERA_LOCAL_OFFSET))
    if AIM_OFFSET_DEG != 0.0:
        xform.AddRotateXOp().Set(AIM_OFFSET_DEG)

    print(f"Added camera: {cam_path.pathString}", flush=True)
    return camera


def build():
    from pxr import Usd

    if not SOURCE_USD.is_file():
        raise FileNotFoundError(SOURCE_USD)
    if OUTPUT_USD.exists():
        OUTPUT_USD.unlink()

    stage = Usd.Stage.Open(str(SOURCE_USD))

    left_link = _find_prim_by_name(stage, LEFT_LINK_NAME)
    right_link = _find_prim_by_name(stage, RIGHT_LINK_NAME)

    if left_link is None or right_link is None:
        raise RuntimeError(
            f"Could not find placeholder links. "
            f"left={'OK' if left_link else 'MISSING'} "
            f"right={'OK' if right_link else 'MISSING'}. "
            f"Check the actual prim names inside {SOURCE_USD.name} in Isaac Sim's "
            f"stage tree and update LEFT_LINK_NAME/RIGHT_LINK_NAME above if they differ."
        )

    print(f"left link:  {left_link.GetPath()}", flush=True)
    print(f"right link: {right_link.GetPath()}", flush=True)

    _add_depth_camera(stage, left_link, "depth_cam_left")
    _add_depth_camera(stage, right_link, "depth_cam_right")

    # Bake to a standalone, self-contained asset (same flatten pattern as
    # build_mecha_roller_asset.py) so it drops into the scene the same way.
    flat = stage.Flatten()
    flat.Export(str(OUTPUT_USD))

    # --- verify -----------------------------------------------------------
    check = Usd.Stage.Open(str(OUTPUT_USD))
    cam_left_ok = check.GetPrimAtPath(
        left_link.GetPath().AppendChild("depth_cam_left")).IsValid()
    cam_right_ok = check.GetPrimAtPath(
        right_link.GetPath().AppendChild("depth_cam_right")).IsValid()

    print(f"BAKED_ASSET={OUTPUT_USD}", flush=True)
    print(f"CAM_LEFT_OK={cam_left_ok} CAM_RIGHT_OK={cam_right_ok}", flush=True)
    ok = cam_left_ok and cam_right_ok
    print(f"BAKE_OK={ok}", flush=True)
    if not ok:
        raise RuntimeError("depth camera bake failed the presence check")


def main():
    _restart_with_isaac_python()
    from isaacsim import SimulationApp

    app = SimulationApp({"headless": True})
    try:
        build()
    except Exception as exc:
        print(f"BAKE_EXCEPTION={type(exc).__name__}: {exc}", flush=True)
        raise
    finally:
        app.close()


if __name__ == "__main__":
    main()


"""
=============================================================================
NEXT STEP (do this inside Isaac Sim, not in this script) -- getting the
actual /depth topic out to ROS2:

A plain UsdGeom.Camera prim (what this script adds) does NOT publish
anything by itself. In the Isaac Sim GUI, per camera:

  1. Window > Graph Editor > Action Graph, create/open your robot's graph.
  2. Add "Isaac Create Render Product" node, set its `cameraPrim` input to
     e.g. /hwia_parking_robot_final_caster/cam_side_left_link/depth_cam_left
  3. Add "ROS2 Camera Helper" node, feed it the render product, set
     `type` = rgb for a color topic, and a second Helper node with
     `type` = depth for the depth topic.
  4. Wire both Helper nodes' execIn from the same OnPlaybackTick as the
     rest of your sensors, and set distinct `topicName`s, e.g.:
       /depth_cam_left/rgb, /depth_cam_left/depth
       /depth_cam_right/rgb, /depth_cam_right/depth

Node names above are for the current Isaac Sim ROS2 bridge extension;
if your Isaac Sim version uses the older `omni.isaac.ros2_bridge` naming
instead of `isaacsim.ros2.bridge`, the node titles in the search box are
the same ("ROS2 Camera Helper", "Isaac Create Render Product") even
though the underlying extension name differs.
=============================================================================
"""
