#!/usr/bin/env python3
"""생성된 테스트 스테이지에서 Sedan 휠이 실제로 어떤 충돌체를 쓰는지 출력한다.

GUI를 띄우지 않고(따라서 GPU도 쓰지 않고) 확인용. Isaac의 pxr만 빌려 쓴다.

실행:
    python3 inspect_wheel_colliders.py                       # 운반 데모 스테이지
    python3 inspect_wheel_colliders.py <다른.usd>
"""

import os
import subprocess
import sys
from pathlib import Path

WORK_DIR = Path(__file__).resolve().parent
DEFAULT_USD = WORK_DIR / "two_robot_carry_demo.usd"
ISAAC_RELEASE = Path("/home/rokey/dev_ws/isaac_sim/isaacsim/_build/linux-x86_64/release")

SEDAN_ROOT = "/World/VehicleAsset/Vehicles/Sedan"
WHEELS = ("FrontLeftWheel", "FrontRightWheel", "RearLeftWheel", "RearRightWheel")


def _reexec():
    try:
        import pxr  # noqa: F401

        return
    except ModuleNotFoundError:
        pass
    if os.environ.get("_INSPECT_REEXEC"):
        raise RuntimeError("재실행 후에도 pxr을 import하지 못했습니다.")
    libs = next(
        (c for c in sorted((ISAAC_RELEASE / "extscache").glob("omni.usd.libs-*"))
         if (c / "pxr").exists()),
        None,
    )
    if libs is None:
        raise FileNotFoundError("omni.usd.libs(pxr)를 찾지 못했습니다.")
    env = dict(
        os.environ,
        _INSPECT_REEXEC="1",
        PYTHONPATH=os.pathsep.join([str(libs), os.environ.get("PYTHONPATH", "")]).strip(os.pathsep),
        LD_LIBRARY_PATH=os.pathsep.join(
            [str(libs / "bin"), os.environ.get("LD_LIBRARY_PATH", "")]
        ).strip(os.pathsep),
    )
    raise SystemExit(
        subprocess.call(
            [str(ISAAC_RELEASE / "python.sh"), str(Path(__file__).resolve()), *sys.argv[1:]],
            env=env,
            cwd=str(ISAAC_RELEASE),
        )
    )


def main():
    _reexec()
    from pxr import Usd, UsdGeom, UsdPhysics

    path = Path(sys.argv[1]).resolve() if len(sys.argv) > 1 else DEFAULT_USD
    if not path.is_file():
        raise SystemExit(f"스테이지가 없습니다: {path}\n먼저 데모를 한 번 실행하세요.")
    stage = Usd.Stage.Open(str(path))

    print(f"=== {path.name} ===\n")
    for wheel in WHEELS:
        wheel_prim = stage.GetPrimAtPath(f"{SEDAN_ROOT}/{wheel}")
        if not wheel_prim.IsValid():
            print(f"  {wheel}: 프림 없음")
            continue
        print(f"  {wheel}")
        for child in wheel_prim.GetChildren():
            if not child.HasAPI(UsdPhysics.CollisionAPI):
                continue
            kind = str(child.GetTypeName())
            detail = ""
            if kind == "Cylinder":
                c = UsdGeom.Cylinder(child)
                detail = f"r={c.GetRadiusAttr().Get():.4f} h={c.GetHeightAttr().Get():.4f} axis={c.GetAxisAttr().Get()}"
            elif kind == "Sphere":
                detail = f"r={UsdGeom.Sphere(child).GetRadiusAttr().Get():.4f}"
            elif kind == "Mesh":
                # Mesh만 Approximation 설정이 의미가 있다.
                api = UsdPhysics.MeshCollisionAPI(child)
                approx = api.GetApproximationAttr().Get() if child.HasAPI(UsdPhysics.MeshCollisionAPI) else None
                detail = f"approximation={approx}"
            state = "활성" if child.IsActive() else "비활성"
            mark = "  <== 실제 물리에 쓰임" if child.IsActive() else ""
            print(f"      {child.GetName():<18} {kind:<9} [{state}] {detail}{mark}")
        print()

    print("참고: Approximation 드롭다운은 Mesh 충돌체에만 적용된다.")
    print("      Sphere/Cube/Capsule은 PhysX 네이티브 프리미티브라 근사 없이 정확히 계산된다.")
    print("      Cylinder는 PhysX에 네이티브가 없어 항상 볼록체로 근사된다(면진 = rocking 원인).")


if __name__ == "__main__":
    main()
