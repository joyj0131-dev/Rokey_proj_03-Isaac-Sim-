#!/usr/bin/env python3
"""ArUco 바닥 마커를 주차장 USD에 얹는다 (원본 비파괴).

팀원 원본 parking_environment.usd 는 수정하지 않고, 서브레이어 override로만 마커 판을
올린 별도 파일을 만든다. 배치 정의는 marker_layout.py 하나만 본다.

주의: 마커의 시각 디자인은 아직 확정 전이다. ID 체계, ArUco 사전, 최종 크기는 미정이고
      여기 값은 placeholder다. 이 스크립트가 답하는 질문은 "어디에 놓이는가" 하나다.

실행 (GPU/SimulationApp 불필요):
    python3 build_marker_layout.py
"""

import os
import subprocess
import sys
from pathlib import Path

import marker_layout as ML

WORK_DIR = Path(__file__).resolve().parent
SOURCE_USD = WORK_DIR / "parking" / "parking_environment.usd"
OUTPUT_USD = WORK_DIR / "parking" / "parking_environment_marker_preview.usd"
ISAAC_RELEASE = Path("/home/rokey/dev_ws/isaac_sim/isaacsim/_build/linux-x86_64/release")

TEX_DIR = WORK_DIR / "textures" / "aruco"


def _reexec():
    """pxr을 쓸 수 있는 인터프리터로 옮겨탄다. SimulationApp은 띄우지 않는다."""
    try:
        import pxr  # noqa: F401

        return
    except ModuleNotFoundError:
        pass
    if os.environ.get("_MARKER_REEXEC"):
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
        _MARKER_REEXEC="1",
        PYTHONPATH=os.pathsep.join(
            [str(libs), str(WORK_DIR), os.environ.get("PYTHONPATH", "")]
        ).strip(os.pathsep),
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


def _marker_material(stage, path, texture_path):
    """ArUco PNG 를 입힌 무광 머티리얼.

    specular 를 0으로 두는 것이 핵심이다. 천장 조명이 흑백 패턴에 정반사를 만들면
    검출기의 이진화가 깨져 검출이 통째로 실패한다. 가장 흔한 실패 모드다.
    """
    from pxr import Sdf, UsdShade

    material = UsdShade.Material.Define(stage, path)

    reader = UsdShade.Shader.Define(stage, path.AppendChild("StReader"))
    reader.CreateIdAttr("UsdPrimvarReader_float2")
    reader.CreateInput("varname", Sdf.ValueTypeNames.Token).Set("st")
    reader.CreateOutput("result", Sdf.ValueTypeNames.Float2)

    texture = UsdShade.Shader.Define(stage, path.AppendChild("Texture"))
    texture.CreateIdAttr("UsdUVTexture")
    texture.CreateInput("file", Sdf.ValueTypeNames.Asset).Set(texture_path)
    # 흑백 패턴이므로 감마 보정을 걸면 안 된다.
    texture.CreateInput("sourceColorSpace", Sdf.ValueTypeNames.Token).Set("raw")
    # 흰 여백이 타일링으로 잘려나가지 않게 clamp.
    texture.CreateInput("wrapS", Sdf.ValueTypeNames.Token).Set("clamp")
    texture.CreateInput("wrapT", Sdf.ValueTypeNames.Token).Set("clamp")
    texture.CreateInput("st", Sdf.ValueTypeNames.Float2).ConnectToSource(
        reader.ConnectableAPI(), "result"
    )
    texture.CreateOutput("rgb", Sdf.ValueTypeNames.Float3)

    shader = UsdShade.Shader.Define(stage, path.AppendChild("Shader"))
    shader.CreateIdAttr("UsdPreviewSurface")
    shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).ConnectToSource(
        texture.ConnectableAPI(), "rgb"
    )
    shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(1.0)
    shader.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(0.0)
    shader.CreateInput("specular", Sdf.ValueTypeNames.Float).Set(0.0)

    material.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")
    return material


def author_markers(stage):
    """스테이지에 마커 42장 + ID별 머티리얼을 authoring 한다 (marker_layout 이 단일 출처).

    build()의 서브레이어 프리뷰와 build_parking_with_markers.py 의 flatten 빌드가
    같은 함수를 쓰므로 두 산출물의 마커는 항상 동일하다.
    """
    from pxr import Gf, Sdf, UsdGeom, UsdShade

    root = UsdGeom.Xform.Define(stage, "/World/ArucoMarkerPreview").GetPath()
    looks = Sdf.Path("/World/Looks")

    half = ML.MARKER_TILE * 0.5
    # 바닥에 눕힌 쿼드. 점 순서와 st 순서가 짝이 맞아야 패턴이 안 뒤집힌다.
    # 여기서는 텍스처의 v=1(위)을 월드 -Z 로 보낸다. 방향이 틀리면 3단계
    # 1-마커 캘리브레이션에서 잡히므로 그때 st만 스왑하면 된다.
    points = [
        Gf.Vec3f(-half, 0.0,  half),
        Gf.Vec3f( half, 0.0,  half),
        Gf.Vec3f( half, 0.0, -half),
        Gf.Vec3f(-half, 0.0, -half),
    ]
    uvs = [Gf.Vec2f(0, 0), Gf.Vec2f(1, 0), Gf.Vec2f(1, 1), Gf.Vec2f(0, 1)]

    used = set()
    for marker_id, kind, label, x, z, yaw, note in ML.assign_ids():
        safe = "".join(ch if ch.isalnum() else "_" for ch in label)
        name = f"{kind}_{safe}"
        while name in used:            # 라벨 충돌 방지(A·W 등 기호 치환 후)
            name += "_"
        used.add(name)

        texture_file = TEX_DIR / f"aruco_{ML.ARUCO_DICT}_{marker_id:03d}.png"
        if not texture_file.is_file():
            raise FileNotFoundError(
                f"마커 텍스처가 없습니다: {texture_file}\n"
                "먼저 build_marker_textures.py 를 실행하세요."
            )

        mesh = UsdGeom.Mesh.Define(stage, root.AppendChild(name))
        mesh.CreatePointsAttr(points)
        mesh.CreateFaceVertexCountsAttr([4])
        mesh.CreateFaceVertexIndicesAttr([0, 1, 2, 3])
        mesh.CreateNormalsAttr([Gf.Vec3f(0, 1, 0)] * 4)
        mesh.SetNormalsInterpolation(UsdGeom.Tokens.vertex)
        mesh.CreateExtentAttr([points[0], points[2]])
        st = UsdGeom.PrimvarsAPI(mesh).CreatePrimvar(
            "st", Sdf.ValueTypeNames.TexCoord2fArray, UsdGeom.Tokens.vertex
        )
        st.Set(uvs)

        xf = UsdGeom.Xformable(mesh)
        xf.AddTranslateOp().Set(Gf.Vec3d(x, ML.MARKER_Y, z))
        # 마커 평면내 방향. yaw=0 이면 회전 없음(현재 전부 0). 0 이 아니면 json 의
        # yaw 와 같은 값이라 지도와 실제 바닥이 일치한다. 스테이지 up=+Y 라 세로축 회전.
        if yaw:
            xf.AddRotateYOp().Set(float(yaw))

        # 머티리얼은 마커마다 다르다(ID별 텍스처).
        material = _marker_material(
            stage,
            looks.AppendChild(f"ArucoMat_{marker_id:03d}"),
            Sdf.AssetPath(f"./../textures/aruco/{texture_file.name}"),
        )
        UsdShade.MaterialBindingAPI.Apply(mesh.GetPrim()).Bind(material)

        prim = mesh.GetPrim()
        prim.CreateAttribute("aruco:markerId", Sdf.ValueTypeNames.Int).Set(marker_id)
        prim.CreateAttribute("aruco:dictionary", Sdf.ValueTypeNames.String).Set(
            ML.ARUCO_DICT
        )
        prim.CreateAttribute("aruco:kind", Sdf.ValueTypeNames.String).Set(kind)
        prim.CreateAttribute("aruco:yaw", Sdf.ValueTypeNames.Float).Set(float(yaw))
        prim.CreateAttribute("aruco:serves", Sdf.ValueTypeNames.String).Set(label)
        prim.CreateAttribute("aruco:note", Sdf.ValueTypeNames.String).Set(note)
        prim.CreateAttribute("aruco:codeSize", Sdf.ValueTypeNames.Float).Set(
            ML.MARKER_CODE_SIZE
        )
        prim.CreateAttribute("aruco:position", Sdf.ValueTypeNames.Float3).Set(
            Gf.Vec3f(x, 0.0, z)
        )


def build():
    from pxr import Usd, UsdGeom

    if not SOURCE_USD.is_file():
        raise FileNotFoundError(f"주차장 USD가 없습니다: {SOURCE_USD}")
    if OUTPUT_USD.exists():
        OUTPUT_USD.unlink()

    stage = Usd.Stage.CreateNew(str(OUTPUT_USD))
    # 원본을 서브레이어로 깔면 원본 파일은 한 바이트도 바뀌지 않는다.
    stage.GetRootLayer().subLayerPaths.append(f"./{SOURCE_USD.name}")
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.y)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)

    world = stage.GetPrimAtPath("/World")
    if not world or not world.IsValid():
        raise RuntimeError("서브레이어에서 /World를 찾지 못했습니다.")
    stage.SetDefaultPrim(world)

    author_markers(stage)

    stage.GetRootLayer().Save()


def main():
    _reexec()
    build()
    rows, kinds = ML.summary()
    print(f"[marker] 원본(무수정): {SOURCE_USD.name}")
    print(f"[marker] 출력(override): {OUTPUT_USD.name}")
    print(f"[marker] 차선 z = ±{ML.LANE_Z:.2f} m, 간격 {ML.SPACE_WIDTH:.2f} m")
    print(f"[marker] {ML.ARUCO_DICT}, 타일 {ML.MARKER_TILE:.3f} m / "
          f"코드 {ML.MARKER_CODE_SIZE:.6f} m, y={ML.MARKER_Y}")
    print("[marker] " + ", ".join(f"{k} {v}" for k, v in kinds.items())
          + f" = 총 {len(rows)}장")


if __name__ == "__main__":
    main()
