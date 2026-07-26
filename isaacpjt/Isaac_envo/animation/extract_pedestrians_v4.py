#!/usr/bin/env python3
"""ssssssss.usd에서 Pedestrians 2명과 my_navmap Character 1명만 추출한다.

이전 anim_human_walk_test 기반 AnimationGraphs/SkelAnimation은 복사하지 않는다.
Character가 참조하는 걷기 그래프 때문에 my_navmap.usd의 Biped_Setup은 보이지
않는 런타임 의존성으로 함께 복사한다.
"""

import argparse
from pathlib import Path


HERE = Path(__file__).resolve().parent
DEFAULT_SOURCE = Path("/home/rokey/Downloads/ssssssss.usd")
DEFAULT_CHARACTER_SOURCE = Path("/home/rokey/Downloads/my_navmap.usd")
DEFAULT_OUTPUT = HERE / "pedestrians_v4.usda"

PEDESTRIAN_SPECS = (
    ("/World/Pedestrians", "/World/Pedestrians"),
)
CHARACTER_SPECS = (
    ("/World/Characters/Biped_Setup", "/World/Characters/Biped_Setup"),
    ("/World/Characters/Character", "/World/Characters/Character"),
)
CHARACTER_PARENT_PROPERTIES = (
    "xformOp:orient",
    "xformOp:rotateX:unitsResolve",
    "xformOp:scale",
    "xformOp:translate",
    "xformOpOrder",
)


def _layer_for_output(path: Path):
    from pxr import Sdf

    existing = Sdf.Layer.FindOrOpen(str(path)) if path.exists() else None
    if existing is not None:
        existing.Clear()
        return existing
    return Sdf.Layer.CreateNew(str(path))


def _copy(source, target, source_path, target_path):
    from pxr import Sdf

    if source.GetObjectAtPath(source_path) is None:
        raise RuntimeError(f"원본 spec 없음: {source_path}")
    if not Sdf.CopySpec(source, source_path, target, target_path):
        raise RuntimeError(f"spec 복사 실패: {source_path} → {target_path}")


def extract(
    source_path: Path,
    character_source_path: Path,
    output_path: Path,
):
    from pxr import Sdf, Usd, UsdGeom

    source_path = source_path.expanduser().resolve()
    character_source_path = character_source_path.expanduser().resolve()
    output_path = output_path.expanduser().resolve()
    for required in (source_path, character_source_path):
        if not required.is_file():
            raise FileNotFoundError(f"원본 USD 없음: {required}")
    output_path.parent.mkdir(parents=True, exist_ok=True)

    source = Sdf.Layer.FindOrOpen(str(source_path))
    character_source = Sdf.Layer.FindOrOpen(str(character_source_path))
    if source is None or character_source is None:
        raise RuntimeError("원본 USD 레이어 열기 실패")
    target = _layer_for_output(output_path)

    world = Sdf.CreatePrimInLayer(target, "/World")
    world.specifier = Sdf.SpecifierDef
    world.typeName = "Xform"
    characters = Sdf.CreatePrimInLayer(target, "/World/Characters")
    characters.specifier = Sdf.SpecifierDef
    characters.typeName = "Xform"

    for source_spec, target_spec in PEDESTRIAN_SPECS:
        _copy(source, target, source_spec, target_spec)
    for source_spec, target_spec in CHARACTER_SPECS:
        _copy(character_source, target, source_spec, target_spec)

    # my_navmap.usd는 Z-up이고 ssssssss.usd가 /World/my_navmap에 좌표 변환을
    # 저작한다. Character만 최상위로 빼도 같은 위치/축을 유지하도록 부모
    # Xform 속성만 복사한다. payload와 NavMesh/Cameras는 가져오지 않는다.
    for name in CHARACTER_PARENT_PROPERTIES:
        _copy(
            source,
            target,
            f"/World/my_navmap.{name}",
            f"/World/Characters.{name}",
        )
    target.Save()

    stage = Usd.Stage.Open(target, load=Usd.Stage.LoadNone)
    stage.SetStartTimeCode(0)
    stage.SetEndTimeCode(600)
    stage.SetTimeCodesPerSecond(60)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.y)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    stage.SetDefaultPrim(stage.GetPrimAtPath("/World"))
    world_prim = stage.GetPrimAtPath("/World")
    world_prim.CreateAttribute(
        "people:sourceFile", Sdf.ValueTypeNames.String, custom=True
    ).Set(str(source_path))
    world_prim.CreateAttribute(
        "people:characterSourceFile", Sdf.ValueTypeNames.String, custom=True
    ).Set(str(character_source_path))
    world_prim.CreateAttribute(
        "people:contents", Sdf.ValueTypeNames.String, custom=True
    ).Set(
        "ssssssss.usd:/World/Pedestrians (2); "
        "my_navmap.usd:/World/Characters/Character (1); "
        "Biped_Setup is a hidden Character animation dependency"
    )
    stage.GetRootLayer().Save()
    return output_path


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", type=Path, default=DEFAULT_SOURCE)
    parser.add_argument(
        "--character-source", type=Path, default=DEFAULT_CHARACTER_SOURCE
    )
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    args = parser.parse_args()

    from isaacsim import SimulationApp

    app = SimulationApp({"headless": True})
    try:
        output = extract(args.source, args.character_source, args.output)
        print(f"V4_PEOPLE_EXTRACTED={output}", flush=True)
        print("V4_PEOPLE_CONTENTS=pedestrians:2 character:1 animation_layer:none")
    finally:
        app.close()


if __name__ == "__main__":
    main()
