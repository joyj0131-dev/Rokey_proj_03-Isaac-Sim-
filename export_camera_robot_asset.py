#!/usr/bin/env python3
"""Bake the depth-camera-equipped HWIA parking robot (URDF import + 4x real
RealSense D455 sensors, see run_depth_camera_sim.py) into a standalone,
flattened, self-contained USD asset -- same pattern as
hwia_parking_robot_final_caster_package/build_mecha_roller_asset.py -- so it
can be referenced into other scenes (e.g. the team's
isaacpjt/Isaac_envo/parking/parking_environment.usd) exactly like the plain
robot asset, instead of only existing inline inside this project's own
throwaway test scene.

Carries NO PhysicsScene of its own -- the referencing scene's PhysicsScene
governs it, same convention as hwia_parking_robot_final_caster_mecha_roller.usd.

Run:
  cd /home/rokey/dev_ws/isaac_sim/isaacsim/_build/linux-x86_64/release
  ./python.sh /home/rokey/p3/Rokey_proj_03-Isaac-Sim-/export_camera_robot_asset.py
"""

import os
import sys
from pathlib import Path

PROJECT_DIR = Path(__file__).resolve().parent
OUTPUT_USD = PROJECT_DIR / "hwia_parking_robot_final_caster_camera_mesh_depth_cam.usd"
ISAAC_PYTHON = Path("/home/rokey/dev_ws/isaac_sim/isaacsim/_build/linux-x86_64/release/python.sh")


def _restart_with_isaac_python():
    if os.environ.get("CARB_APP_PATH"):
        return
    os.execv(str(ISAAC_PYTHON), [str(ISAAC_PYTHON), str(Path(__file__).resolve()), *sys.argv[1:]])


def build(app):
    import omni.usd
    from pxr import Usd, UsdGeom

    sys.path.insert(0, str(PROJECT_DIR))
    import run_depth_camera_sim as rdcs

    robot_prim_path = rdcs._import_robot()
    print(f"IMPORTED_ROOT={robot_prim_path}", flush=True)
    for _ in range(5):
        app.update()

    stage = omni.usd.get_context().get_stage()

    links = {name: rdcs._find_prim_by_name(stage, name) for name in
             (rdcs.LEFT_LINK_NAME, rdcs.RIGHT_LINK_NAME, rdcs.FRONT_LINK_NAME, rdcs.REAR_LINK_NAME)}
    missing = [name for name, prim in links.items() if prim is None]
    if missing:
        raise RuntimeError(f"camera housing links missing after import: {missing}")

    from isaacsim.core.utils.nucleus import get_assets_root_path
    assets_root = get_assets_root_path()
    if not assets_root:
        raise RuntimeError("Could not resolve the Isaac Sim asset root (nucleus not reachable)")
    rsd455_usd_path = assets_root.rstrip("/") + rdcs.RSD455_RELATIVE_PATH

    rdcs._add_rsd455_camera(
        stage, links[rdcs.LEFT_LINK_NAME], "depth_cam_left", rsd455_usd_path, "Left",
        rdcs._ROT_LEFT, rdcs.SIDE_LOCAL_OFFSET,
    )
    rdcs._add_rsd455_camera(
        stage, links[rdcs.RIGHT_LINK_NAME], "depth_cam_right", rsd455_usd_path, "Right",
        rdcs._ROT_RIGHT, rdcs.SIDE_LOCAL_OFFSET,
    )
    rdcs._add_rsd455_camera(
        stage, links[rdcs.FRONT_LINK_NAME], "depth_cam_front", rsd455_usd_path, "Front",
        rdcs._ROT_FRONT, rdcs.FRONT_LOCAL_OFFSET,
    )
    rdcs._add_rsd455_camera(
        stage, links[rdcs.REAR_LINK_NAME], "depth_cam_rear", rsd455_usd_path, "Rear",
        rdcs._ROT_REAR, rdcs.REAR_LOCAL_OFFSET,
    )
    for _ in range(10):
        app.update()

    if OUTPUT_USD.exists():
        OUTPUT_USD.unlink()
    flat = stage.Flatten()
    flat.Export(str(OUTPUT_USD))

    # --- verify the exported asset is self-contained and complete ------------
    check = Usd.Stage.Open(str(OUTPUT_USD))
    ext_deps = [d for d in check.GetRootLayer().GetCompositionAssetDependencies() if d]
    base_link_ok = any(p.GetName() == "base_link" for p in check.Traverse())
    depth_cams = [p.GetName() for p in check.Traverse()
                  if p.IsA(UsdGeom.Camera) and p.GetName().startswith("Camera_Pseudo_Depth_")]
    geom_prims = [p for p in check.Traverse() if p.IsA(UsdGeom.Gprim)]

    print(f"BAKED_ASSET={OUTPUT_USD}", flush=True)
    print(f"SELF_CONTAINED={len(ext_deps) == 0} (external deps: {list(ext_deps)})", flush=True)
    print(f"BASE_LINK_PRESENT={base_link_ok}", flush=True)
    print(f"DEPTH_CAMS={sorted(depth_cams)} (expected 4)", flush=True)
    print(f"GEOM_PRIMS={len(geom_prims)}", flush=True)
    ok = len(ext_deps) == 0 and base_link_ok and len(depth_cams) == 4 and len(geom_prims) > 0
    print(f"BAKE_OK={ok}", flush=True)
    if not ok:
        raise RuntimeError("flattened camera-robot asset failed the self-containment/composition check")


def main():
    _restart_with_isaac_python()
    from isaacsim import SimulationApp

    app = SimulationApp({"headless": True})
    try:
        build(app)
    except Exception as exc:
        print(f"BAKE_EXCEPTION={type(exc).__name__}: {exc}", flush=True)
        raise
    finally:
        app.close()


if __name__ == "__main__":
    main()
