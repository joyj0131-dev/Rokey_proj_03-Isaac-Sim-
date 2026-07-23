#!/usr/bin/env python3
"""바퀴를 통째로 DROP m 내린 로봇 변형 에셋을 만든다 (원본 비파괴).

원본: hwia_depth_cam_mecha_roller.usd (수정 안 함)
출력: hwia_depth_cam_mecha_roller_lowered.usd

목적: 로봇 지상고를 올린다. 로봇은 롤러 최저점(z=-0.011)으로 지면에 서는데,
바퀴 조립체(허브 링크 + base→wheel 조인트 + 롤러 40개)를 통째로 DROP 만큼
Z로 내리면 롤러 접지점이 DROP 낮아져 물리 정착 시 차체가 DROP 만큼 올라간다.
메카넘 근기학(WHEEL_RADIUS)·롤러 상대 앵커(0.042)는 바뀌지 않아 주행은 불변.

건드리는 것:
  - wheel_{fl,fr,rl,rr} 링크: xformOp:translate Z -= DROP
  - wheel_{..}_joint: physics:localPos0 Z -= DROP  (base_link 쪽 앵커)
  - roller_wheel_{..}_k 바디: xformOp:translate Z -= DROP
안 건드리는 것: 롤러 조인트 상대 앵커, 캐스터/암/카메라/섀시.

실행: python3 make_lowered_wheel_asset.py [--drop 0.03]
"""
import os
import re
import subprocess
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
SRC = HERE / "hwia_depth_cam_mecha_roller.usd"
DST = HERE / "hwia_depth_cam_mecha_roller_lowered.usd"
ISAAC = Path("/home/rokey/dev_ws/isaac_sim/isaacsim/_build/linux-x86_64/release")

WHEEL_LINKS = {"wheel_fl", "wheel_fr", "wheel_rl", "wheel_rr"}
ROLLER_RE = re.compile(r"^roller_wheel_(fl|fr|rl|rr)_\d+$")


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


def main():
    args = sys.argv[1:]
    drop = float(args[args.index("--drop") + 1]) if "--drop" in args else 0.03
    _reexec_done()

    from pxr import Usd, UsdGeom, Gf

    stage = Usd.Stage.Open(str(SRC))
    n_link = n_joint = n_roller = 0
    for prim in stage.Traverse():
        name = prim.GetName()
        path = str(prim.GetPath())
        # 1) 바퀴 허브 링크 (콜라이더/비주얼 하위 제외)
        if name in WHEEL_LINKS and "/visuals/" not in path and "/colliders/" not in path:
            xf = UsdGeom.Xformable(prim)
            t = next((o for o in xf.GetOrderedXformOps()
                      if o.GetOpType() == UsdGeom.XformOp.TypeTranslate), None)
            if t is not None:
                v = t.Get()
                t.Set(Gf.Vec3d(v[0], v[1], v[2] - drop))
                n_link += 1
        # 2) base->wheel 조인트 localPos0 (base_link 쪽 앵커)
        elif name in {f"{w}_joint" for w in WHEEL_LINKS}:
            a = prim.GetAttribute("physics:localPos0")
            if a and a.Get() is not None:
                v = a.Get()
                a.Set(Gf.Vec3f(v[0], v[1], v[2] - drop))
                n_joint += 1
        # 3) 롤러 바디
        elif ROLLER_RE.match(name) and "/joints/" not in path:
            xf = UsdGeom.Xformable(prim)
            t = next((o for o in xf.GetOrderedXformOps()
                      if o.GetOpType() == UsdGeom.XformOp.TypeTranslate), None)
            if t is not None:
                v = t.Get()
                t.Set(Gf.Vec3d(v[0], v[1], v[2] - drop))
                n_roller += 1

    assert n_link == 4, f"바퀴 링크 4개 기대, {n_link}개 처리됨"
    assert n_joint == 4, f"바퀴 조인트 4개 기대, {n_joint}개 처리됨"
    assert n_roller == 40, f"롤러 40개 기대, {n_roller}개 처리됨"

    if DST.exists():
        DST.unlink()
    stage.GetRootLayer().Export(str(DST))
    print(f"[lowered] DROP={drop} m 적용: 링크 {n_link} + 조인트 {n_joint} + 롤러 {n_roller}")
    print(f"[lowered] 출력: {DST}")


def _reexec_done():
    pass


if __name__ == "__main__":
    _reexec()
    main()
