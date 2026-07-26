#!/usr/bin/env python3
"""v4 Phase 0 probe 공용 — 바닥 시각화와 리포트 저장.

측정 결과를 씬 안에 그려서 GUI 로 바로 이해되게 하는 것이 목적이다.
"""
import json
from pathlib import Path

REPORT_DIR = Path(__file__).resolve().parent / "probe_reports"

WHITE = (1.0, 1.0, 1.0)
YELLOW = (1.0, 0.85, 0.1)
GREEN = (0.15, 0.85, 0.25)
RED = (0.9, 0.15, 0.15)


def write_report(name, payload):
    """probe 결과 JSON 저장. 반환값은 저장 경로."""
    REPORT_DIR.mkdir(parents=True, exist_ok=True)
    p = REPORT_DIR / f"{name}.json"
    p.write_text(json.dumps(payload, indent=2, ensure_ascii=False), encoding="utf-8")
    return p


def draw_trail(stage, path, points, color, width=0.04, y=0.02):
    """(x, z) 점열을 바닥 위 얇은 큐브 띠로 그린다.

    BasisCurves 대신 큐브를 쓰는 이유: 뷰포트 기본 설정에서 곡선은 두께가
    렌더러 의존이라 잘 안 보인다. 큐브는 어디서나 동일하게 보인다.
    """
    import math
    from pxr import Gf, UsdGeom
    root = UsdGeom.Xform.Define(stage, path)
    for i in range(len(points) - 1):
        x0, z0 = points[i]
        x1, z1 = points[i + 1]
        dx, dz = x1 - x0, z1 - z0
        seg = math.hypot(dx, dz)
        if seg < 1e-6:
            continue
        cube = UsdGeom.Cube.Define(stage, f"{path}/seg_{i:04d}")
        cube.GetSizeAttr().Set(1.0)
        xf = UsdGeom.Xformable(cube)
        xf.ClearXformOpOrder()
        xf.AddTranslateOp().Set(Gf.Vec3d((x0 + x1) * 0.5, y, (z0 + z1) * 0.5))
        xf.AddRotateYOp().Set(math.degrees(math.atan2(dx, dz)))
        xf.AddScaleOp().Set(Gf.Vec3f(width, 0.004, seg))
        cube.GetDisplayColorAttr().Set([Gf.Vec3f(*color)])
    return root


def draw_marker_dot(stage, path, x, z, color, size=0.18, y=0.02):
    """측정점 표식 하나."""
    from pxr import Gf, UsdGeom
    cube = UsdGeom.Cube.Define(stage, path)
    cube.GetSizeAttr().Set(1.0)
    xf = UsdGeom.Xformable(cube)
    xf.ClearXformOpOrder()
    xf.AddTranslateOp().Set(Gf.Vec3d(x, y, z))
    xf.AddScaleOp().Set(Gf.Vec3f(size, 0.004, size))
    cube.GetDisplayColorAttr().Set([Gf.Vec3f(*color)])
    return cube


def draw_band(stage, path, x0, x1, z, color, width=0.6, y=0.015):
    """검출 가능 구간을 바닥 띠로 그린다. x0~x1 구간, z 중심."""
    from pxr import Gf, UsdGeom
    cube = UsdGeom.Cube.Define(stage, path)
    cube.GetSizeAttr().Set(1.0)
    xf = UsdGeom.Xformable(cube)
    xf.ClearXformOpOrder()
    xf.AddTranslateOp().Set(Gf.Vec3d((x0 + x1) * 0.5, y, z))
    xf.AddScaleOp().Set(Gf.Vec3f(max(abs(x1 - x0), 0.02), 0.004, width))
    cube.GetDisplayColorAttr().Set([Gf.Vec3f(*color)])
    return cube
