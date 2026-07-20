#!/usr/bin/env python3
"""Bake a mecanum-roller variant of the HWIA parking robot into a new USD asset.

Produces ``hwia_parking_robot_final_caster_mecha_roller.usd`` next to the base
asset. The baked file is FLATTENED and fully self-contained: the original robot
geometry/physics is inlined (no external reference), the passive 45-degree
rollers from ``mecanum_drive.add_mecanum_rollers`` are baked in, plus a
self-contained roller friction material. The original asset is never modified.

Reference the baked asset into any scene exactly like the base robot; the
rollers travel with it. Wheel/joint layout under the referenced prim is
identical to the base robot, so scene code keeps the same relative paths.

Run:
  cd /home/rokey/dev_ws/isaac_sim/isaacsim/_build/linux-x86_64/release
  ./python.sh /home/rokey/cobot3_ws/isaacpjt/hwia_parking_robot_final_caster_package/build_mecha_roller_asset.py
"""

import os
import sys
from pathlib import Path


PKG_DIR = Path(__file__).resolve().parent
ISAAC_ENVO = PKG_DIR.parent / "Isaac_envo"
ORIGINAL_USD = PKG_DIR / "hwia_parking_robot_final_caster.usd"
ORIGINAL_REL = ORIGINAL_USD.name  # relative reference within the package dir
BAKED_USD = PKG_DIR / "hwia_parking_robot_final_caster_mecha_roller.usd"

ISAAC_PYTHON = Path("/home/rokey/dev_ws/isaac_sim/isaacsim/_build/linux-x86_64/release/python.sh")

ROOT = "/robot_mecha"
WRAP = "/robot_mecha/base_link"
JOINTS = "/robot_mecha/base_link/joints"


def _restart_with_isaac_python():
    if os.environ.get("CARB_APP_PATH"):
        return
    if not ISAAC_PYTHON.is_file():
        raise FileNotFoundError(f"Isaac Sim python.sh not found: {ISAAC_PYTHON}")
    os.execv(
        str(ISAAC_PYTHON),
        [str(ISAAC_PYTHON), str(Path(__file__).resolve()), *sys.argv[1:]],
    )


def build():
    from pxr import Usd, UsdGeom, UsdPhysics, UsdShade

    sys.path.insert(0, str(ISAAC_ENVO))
    from mecanum_drive import add_mecanum_rollers, N_ROLLERS

    if not ORIGINAL_USD.is_file():
        raise FileNotFoundError(ORIGINAL_USD)
    if BAKED_USD.exists():
        BAKED_USD.unlink()

    # Build in memory: reference the original, add rollers, then FLATTEN so the
    # exported asset is fully self-contained (no dependency on the original).
    stage = Usd.Stage.CreateInMemory()
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.y)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)

    root = UsdGeom.Xform.Define(stage, ROOT).GetPrim()
    root.GetReferences().AddReference(str(ORIGINAL_USD))  # absolute; flattened away
    stage.SetDefaultPrim(root)

    # Self-contained roller friction material (matches the validated tests).
    mat = UsdShade.Material.Define(stage, f"{ROOT}/RollerMaterial")
    mat_api = UsdPhysics.MaterialAPI.Apply(mat.GetPrim())
    mat_api.CreateStaticFrictionAttr(1.1)
    mat_api.CreateDynamicFrictionAttr(0.9)
    mat_api.CreateRestitutionAttr(0.0)

    add_mecanum_rollers(stage, WRAP, JOINTS, grip_material=mat)

    # Flatten every composition arc (reference + the original's sublayers) into
    # one layer and export it -> standalone asset.
    flat = stage.Flatten()
    flat.Export(str(BAKED_USD))

    # --- verify the exported asset is self-contained and complete ------------
    check = Usd.Stage.Open(str(BAKED_USD))
    # Filter empty/anonymous entries that Flatten can leave behind.
    ext_deps = [d for d in check.GetRootLayer().GetCompositionAssetDependencies() if d]
    rollers = [p for p in check.Traverse()
               if p.GetName().startswith("roller_") and p.HasAPI(UsdPhysics.RigidBodyAPI)]
    roller_joints = [p for p in check.Traverse()
                     if p.GetName().startswith("roller_") and p.IsA(UsdPhysics.RevoluteJoint)]
    wheel_ok = check.GetPrimAtPath(f"{ROOT}/base_link/wheel_fl").IsValid()
    body_ok = check.GetPrimAtPath(f"{ROOT}/base_link/base_link").IsValid()
    default_ok = check.GetDefaultPrim().GetPath().pathString == ROOT
    # geometry actually inlined (Gprim covers Mesh/Cube/Cylinder/Sphere/Capsule/...)
    geom_prims = [p for p in check.Traverse() if p.IsA(UsdGeom.Gprim)]

    expected = 4 * N_ROLLERS
    print(f"BAKED_ASSET={BAKED_USD}", flush=True)
    print(f"SELF_CONTAINED={len(ext_deps) == 0} (external deps: {list(ext_deps)})", flush=True)
    print(f"DEFAULT_PRIM_OK={default_ok} ({check.GetDefaultPrim().GetPath()})", flush=True)
    print(f"ROLLER_BODIES={len(rollers)} (expected {expected})", flush=True)
    print(f"ROLLER_JOINTS={len(roller_joints)} (expected {expected})", flush=True)
    print(f"WHEEL_PRESENT={wheel_ok} BODY_PRESENT={body_ok} GEOM_PRIMS={len(geom_prims)}", flush=True)
    ok = (len(ext_deps) == 0 and default_ok and len(rollers) == expected
          and len(roller_joints) == expected and wheel_ok and body_ok and len(geom_prims) > 0)
    print(f"BAKE_OK={ok}", flush=True)
    if not ok:
        raise RuntimeError("flattened mecha_roller asset failed self-containment/composition check")


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
