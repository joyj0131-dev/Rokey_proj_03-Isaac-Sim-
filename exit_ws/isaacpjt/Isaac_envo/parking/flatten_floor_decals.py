#!/usr/bin/env python3
"""바닥 데칼(주차선·아웃라인·ArUco 마커)을 바닥면에 거의 붙게 낮춘다.

배경: 이 데칼들은 충돌이 없어 로봇을 물리적으로 못 걸린다(실측 확인). 다만
바닥면(Y=0)보다 0.4~1.2mm 위에 떠서 눈에 턱처럼 보인다. 전부 바닥 바로 위
FLUSH_Y 로 낮춰 시각적 턱을 없앤다(z-fighting 직전까지).

대상: parking_environment_with_markers.usd 안에서 충돌 없이 바닥 근처(Y_max가
0<Y<=0.01)에 있는 얇은 프림. translate 기반(Cube/Mesh 마커)은 translate Y를
조정하고, 좌표가 점에 구워진 Mesh(도색 선분)는 points Y를 조정한다.

실행: python3 flatten_floor_decals.py   (GPU/SimulationApp 불필요)
"""
import os
import subprocess
import sys
from pathlib import Path

TARGET = Path("/home/rokey/cobot3_ws/isaacpjt/Isaac_envo/parking"
              "/parking_environment_with_markers.usd")
ISAAC = Path("/home/rokey/dev_ws/isaac_sim/isaacsim/_build/linux-x86_64/release")
FLUSH_TOP_Y = 0.0003   # 바닥면 위 0.3mm — 육안 턱 없음, z-fighting 회피


def _reexec():
    try:
        import pxr  # noqa
        return
    except ModuleNotFoundError:
        pass
    if os.environ.get("_R"):
        raise RuntimeError("pxr import fail")
    libs = next(c for c in sorted((ISAAC / "extscache").glob("omni.usd.libs-*"))
                if (c / "pxr").exists())
    raise SystemExit(subprocess.call(
        [str(ISAAC / "python.sh"), str(Path(__file__).resolve()), *sys.argv[1:]],
        env=dict(os.environ, _R="1", PYTHONPATH=str(libs),
                 LD_LIBRARY_PATH=str(libs / "bin")), cwd=str(ISAAC)))


_reexec()
from pxr import Usd, UsdGeom, UsdPhysics, Gf, Sdf  # noqa: E402


def y_extent(prim):
    try:
        r = UsdGeom.Imageable(prim).ComputeWorldBound(
            Usd.TimeCode.Default(), UsdGeom.Tokens.default_).ComputeAlignedRange()
        if r.IsEmpty():
            return None
        return r.GetMin()[1], r.GetMax()[1]
    except Exception:
        return None


def main():
    stage = Usd.Stage.Open(str(TARGET))
    moved_t = moved_p = 0
    for prim in stage.Traverse():
        if prim.HasAPI(UsdPhysics.CollisionAPI):
            continue
        if prim.GetTypeName() not in ("Mesh", "Cube"):
            continue
        ext = y_extent(prim)
        if ext is None:
            continue
        ymin, ymax = ext
        # 바닥 바로 위에 떠 있는 얇은 데칼만 (벽/차량/기둥 제외)
        if not (1e-5 < ymax <= 0.01) or (ymax - ymin) > 0.01:
            continue
        drop = ymax - FLUSH_TOP_Y   # 이만큼 내리면 top 이 FLUSH_TOP_Y
        if drop <= 1e-6:
            continue
        xf = UsdGeom.Xformable(prim)
        ops = xf.GetOrderedXformOps()
        t_op = next((o for o in ops if o.GetOpType() ==
                     UsdGeom.XformOp.TypeTranslate), None)
        if t_op is not None:
            v = t_op.Get()
            t_op.Set(Gf.Vec3d(v[0], v[1] - drop, v[2]))
            moved_t += 1
        elif prim.GetTypeName() == "Mesh":
            pts_attr = UsdGeom.Mesh(prim).GetPointsAttr()
            pts = pts_attr.Get()
            if pts is None:
                continue
            pts_attr.Set([Gf.Vec3f(p[0], p[1] - drop, p[2]) for p in pts])
            moved_p += 1
    stage.GetRootLayer().Save()
    print(f"[flatten] 데칼 낮춤: translate {moved_t}개 + points {moved_p}개 "
          f"→ 상단 Y≈{FLUSH_TOP_Y}m")

    # 검증: 대표 데칼 몇 개의 새 높이
    for name in ("ArucoMarkerPreview", "Outline", "Markings"):
        for prim in stage.Traverse():
            if name in prim.GetName() and prim.GetTypeName() in ("Mesh", "Cube"):
                ext = y_extent(prim)
                if ext:
                    print(f"  {prim.GetName()}: Y {ext[0]:.5f}~{ext[1]:.5f}")
                break


if __name__ == "__main__":
    main()
