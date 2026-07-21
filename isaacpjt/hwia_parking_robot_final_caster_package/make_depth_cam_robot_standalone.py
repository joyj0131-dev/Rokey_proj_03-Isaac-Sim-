#!/usr/bin/env python3
"""깊이카메라+메카넘 HWIA 로봇을 외부 USD 의존성 없는 단일 파일로 만든다.

기존 hwia_depth_cam_mecha_roller.usd는 롤러 override만 담고 카메라 로봇
원본을 subLayer로 참조한다. 이 도구는 그 두 레이어를 flatten해 같은 파일명으로
교체하며, 카메라/바퀴/롤러/물리 몸체와 외부 composition 의존성을 검증한다.
"""

from __future__ import annotations

import argparse
import os
import sys
from pathlib import Path


PKG_DIR = Path(__file__).resolve().parent
OUTPUT_USD = PKG_DIR / "hwia_depth_cam_mecha_roller.usd"
ISAAC_PYTHON = Path(
    "/home/rokey/dev_ws/isaac_sim/isaacsim/_build/linux-x86_64/release/python.sh"
)
SOURCE_CANDIDATES = (
    PKG_DIR.parent.parent / "hwia_parking_robot_final_caster_camera_mesh_depth_cam.usd",
    Path(
        "/home/rokey/p3/Rokey_proj_03-Isaac-Sim-/"
        "hwia_parking_robot_final_caster_camera_mesh_depth_cam.usd"
    ),
)


def restart_with_isaac_python():
    if os.environ.get("CARB_APP_PATH"):
        return
    os.execv(
        str(ISAAC_PYTHON),
        [str(ISAAC_PYTHON), str(Path(__file__).resolve()), *sys.argv[1:]],
    )


def resolve_source(explicit: str | None) -> Path:
    candidates = (Path(explicit).expanduser(),) if explicit else SOURCE_CANDIDATES
    for candidate in candidates:
        if candidate.is_file():
            return candidate.resolve()
    raise FileNotFoundError(
        "카메라 로봇 원본 USD를 찾지 못했습니다: "
        + ", ".join(str(path) for path in candidates)
    )


def build(source: Path):
    from pxr import Sdf, Usd, UsdGeom, UsdPhysics

    if not OUTPUT_USD.is_file():
        raise FileNotFoundError(OUTPUT_USD)

    # 원래 crate를 문자열 USDA로 복제해야 subLayer 경로만 메모리에서 안전하게
    # 교체할 수 있다. 출력 검증 전까지 기존 파일은 건드리지 않는다.
    original = Sdf.Layer.FindOrOpen(str(OUTPUT_USD))
    overlay = Sdf.Layer.CreateAnonymous("depth_cam_mecha_overlay.usda")
    if not overlay.ImportFromString(original.ExportToString()):
        raise RuntimeError("기존 롤러 overlay를 메모리 레이어로 복제하지 못했습니다.")
    overlay.subLayerPaths = [str(source)]

    stage = Usd.Stage.Open(overlay)
    if stage is None:
        raise RuntimeError("카메라 원본과 롤러 overlay 합성에 실패했습니다.")
    flat = stage.Flatten()
    temporary = OUTPUT_USD.with_name(f".{OUTPUT_USD.stem}.standalone.tmp.usd")
    if temporary.exists():
        temporary.unlink()
    if not flat.Export(str(temporary)):
        raise RuntimeError(f"flatten USD 저장 실패: {temporary}")

    check = Usd.Stage.Open(str(temporary))
    dependencies = [
        dep for dep in check.GetRootLayer().GetCompositionAssetDependencies() if dep
    ]
    cameras = [prim for prim in check.Traverse() if prim.IsA(UsdGeom.Camera)]
    rollers = [
        prim
        for prim in check.Traverse()
        if prim.GetName().startswith("roller_")
        and prim.HasAPI(UsdPhysics.RigidBodyAPI)
    ]
    wheel_names = {"wheel_fl", "wheel_fr", "wheel_rl", "wheel_rr"}
    wheels = {prim.GetName() for prim in check.Traverse()} & wheel_names
    rigid_bodies = [
        prim for prim in check.Traverse() if prim.HasAPI(UsdPhysics.RigidBodyAPI)
    ]
    default_prim = check.GetDefaultPrim()
    report = {
        "source": str(source),
        "externalCompositionDependencies": list(dependencies),
        "defaultPrim": str(default_prim.GetPath()) if default_prim else "",
        "cameras": len(cameras),
        "rollerBodies": len(rollers),
        "wheels": sorted(wheels),
        "rigidBodies": len(rigid_bodies),
        "sizeBytes": temporary.stat().st_size,
    }
    print(f"[standalone] 검증 결과: {report}", flush=True)
    valid = (
        not dependencies
        and bool(default_prim)
        and len(cameras) >= 4
        and len(rollers) == 40
        and wheels == wheel_names
        and len(rigid_bodies) > 0
    )
    if not valid:
        temporary.unlink(missing_ok=True)
        raise RuntimeError("단일 USD 검증 실패 — 기존 파일을 유지합니다.")

    temporary.replace(OUTPUT_USD)
    print(f"STANDALONE_DEPTH_CAM_ROBOT={OUTPUT_USD}", flush=True)
    print("STANDALONE_OK=True", flush=True)


def main():
    restart_with_isaac_python()
    parser = argparse.ArgumentParser()
    parser.add_argument("--source", help="카메라 로봇 원본 USD 경로")
    args = parser.parse_args()

    from isaacsim import SimulationApp

    app = SimulationApp({"headless": True})
    try:
        build(resolve_source(args.source))
    finally:
        app.close(wait_for_replicator=False)


if __name__ == "__main__":
    main()
