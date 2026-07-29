#!/usr/bin/env python3
"""parking_environment_with_markers.usd 를 마커 42장 포함 단일(flatten) USD로 재생성한다.

입력  : parking/parking_environment_with_markers.usd (07-21 인계장 재설계 환경)
출력  : 같은 파일 (덮어쓰기, 임시파일 경유)

하는 일:
  1. 기존 마커/머티리얼을 걷어내고 marker_layout.py 기준 42장(실내 26 + 인계장 16)을
     다시 얹는다 — build_marker_layout.author_markers 를 그대로 재사용하므로
     marker_map.json 과 USD 배치가 항상 일치한다.
  2. Stage.Flatten 으로 ../fab_vehicles.usd 참조를 파일 안에 굽는다 (자체포함).
  3. 남는 외부 의존은 두 가지뿐이며 의도된 것이다:
       - 텍스처 PNG (../textures/aruco/*.png) — USD 에는 이미지를 내장할 수 없다
       - 천장 라이다 2대의 온라인 S3 참조 — flatten 시 사라지므로 다시 붙여 둔다
         (오프라인에서는 어차피 빈 프림. 로컬 에셋화는 별도 과제)
  4. 자체 검증: 로컬 파일 참조 0개, 마커 42장, 텍스처 상대경로 해석 가능,
     차량 8대(주차 6 + 인계 대기 2) 존재.

실행 (GPU/SimulationApp 불필요):
    python3 build_parking_with_markers.py
"""

import os
import sys
from pathlib import Path

WORK_DIR = Path(__file__).resolve().parent
TARGET_USD = WORK_DIR / "parking" / "parking_environment_with_markers.usd"
TEX_DIR = WORK_DIR / "textures" / "aruco"

sys.path.insert(0, str(WORK_DIR))
import build_marker_layout as BML  # noqa: E402
import marker_layout as ML  # noqa: E402

ISAAC_RELEASE = Path("/home/rokey/dev_ws/isaac_sim/isaacsim/_build/linux-x86_64/release")


def _reexec():
    """pxr을 쓸 수 있는 인터프리터로 '이 파일'을 재실행한다.

    BML._reexec 를 빌리면 안 된다 — 그쪽 __file__ 은 build_marker_layout.py 라서
    엉뚱한 스크립트가 실행된다 (실제로 그렇게 한 번 틀렸다).
    """
    import subprocess

    try:
        import pxr  # noqa: F401

        return
    except ModuleNotFoundError:
        pass
    if os.environ.get("_FLAT_REEXEC"):
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
        _FLAT_REEXEC="1",
        PYTHONPATH=os.pathsep.join(
            [str(libs), str(WORK_DIR), os.environ.get("PYTHONPATH", "")]
        ).strip(os.pathsep),
        LD_LIBRARY_PATH=os.pathsep.join(
            [str(libs / "bin"), os.environ.get("LD_LIBRARY_PATH", "")]
        ).strip(os.pathsep),
    )
    raise SystemExit(
        subprocess.call(
            [str(ISAAC_RELEASE / "python.sh"), str(Path(__file__).resolve()),
             *sys.argv[1:]],
            env=env,
            cwd=str(ISAAC_RELEASE),
        )
    )


def _collect_lidar_refs(layer):
    """루트 레이어에서 온라인(URL) 참조를 가진 프림들을 찾아 (경로, URL) 목록으로."""
    from pxr import Sdf

    out = []

    def visit(spec):
        for item in spec.referenceList.GetAddedOrExplicitItems():
            if item.assetPath.startswith(("http://", "https://", "omniverse://")):
                out.append((str(spec.path), item.assetPath))
        for child in spec.nameChildren:
            visit(child)

    for root in layer.rootPrims:
        visit(root)
    return out


def build():
    from pxr import Sdf, Usd, UsdShade

    stage = Usd.Stage.Open(str(TARGET_USD))
    src_layer = stage.GetRootLayer()
    online_refs = _collect_lidar_refs(src_layer)

    # 1) 기존 마커·마커 머티리얼 제거 (메모리에서만 — 원본 파일은 저장하지 않는다)
    if stage.GetPrimAtPath("/World/ArucoMarkerPreview"):
        stage.RemovePrim("/World/ArucoMarkerPreview")
    looks = stage.GetPrimAtPath("/World/Looks")
    if looks:
        for child in list(looks.GetChildren()):
            if child.GetName().startswith("ArucoMat_"):
                stage.RemovePrim(child.GetPath())

    # 2) marker_layout 기준으로 42장 다시 authoring
    BML.author_markers(stage)

    # 3) flatten — 로컬 파일 참조(fab_vehicles)를 전부 굽는다
    flat_layer = stage.Flatten()
    flat = Usd.Stage.Open(flat_layer)

    # 4) 텍스처 경로를 출력 파일 기준 상대경로로 정규화
    #    (flatten 이 절대경로로 앵커링하는 것을 되돌린다 — 절대경로 굽기 금지 관례)
    fixed = 0
    for prim in flat.Traverse():
        if prim.GetTypeName() != "Shader":
            continue
        shader = UsdShade.Shader(prim)
        inp = shader.GetInput("file")
        if not inp:
            continue
        val = inp.Get()
        if val and "textures/aruco" in val.path.replace("\\", "/"):
            inp.Set(Sdf.AssetPath(f"./../textures/aruco/{Path(val.path).name}"))
            fixed += 1

    # 5) 라이다 온라인 참조 복원 (flatten 은 미해석 참조를 잃어버린다)
    for prim_path, url in online_refs:
        prim = flat.GetPrimAtPath(prim_path)
        if not prim:
            prim = flat.DefinePrim(prim_path)
        prim.GetReferences().AddReference(url)

    # 6) 임시 파일로 내보낸 뒤 교체
    tmp = TARGET_USD.with_suffix(".usd.tmp")
    flat.GetRootLayer().Export(str(tmp))
    os.replace(tmp, TARGET_USD)
    return len(online_refs), fixed


def verify():
    from pxr import Sdf, Usd

    layer = Sdf.Layer.FindOrOpen(str(TARGET_USD))
    stage = Usd.Stage.Open(str(TARGET_USD))

    local_refs, online = [], []

    def visit(spec):
        for item in spec.referenceList.GetAddedOrExplicitItems():
            ap = item.assetPath
            (online if ap.startswith(("http://", "https://", "omniverse://"))
             else local_refs).append((str(spec.path), ap))
        for child in spec.nameChildren:
            visit(child)

    for root in layer.rootPrims:
        visit(root)
    assert not layer.subLayerPaths, f"서브레이어 잔존: {list(layer.subLayerPaths)}"
    assert not local_refs, f"로컬 파일 참조 잔존: {local_refs}"

    ids = []
    bad_tex = []
    for prim in stage.Traverse():
        a = prim.GetAttribute("aruco:markerId")
        if a and a.Get() is not None:
            ids.append(int(a.Get()))
        for attr in prim.GetAttributes():
            if attr.GetTypeName() == Sdf.ValueTypeNames.Asset:
                v = attr.Get()
                if v and "aruco" in v.path:
                    if not v.path.startswith("./../textures/aruco/"):
                        bad_tex.append(v.path)
                    elif not (TARGET_USD.parent / v.path).resolve().is_file():
                        bad_tex.append(v.path + " (파일 없음)")
    expected = sorted(r[0] for r in ML.assign_ids())
    assert sorted(ids) == expected, f"마커 ID 불일치: {sorted(ids)} != {expected}"
    assert not bad_tex, f"텍스처 경로 문제: {bad_tex[:5]}"

    parked = stage.GetPrimAtPath("/World/ParkingVehicles/Parked")
    queue = stage.GetPrimAtPath("/World/ParkingVehicles/HandoffQueue")
    n_parked = len(list(parked.GetChildren())) if parked else 0
    n_queue = len(list(queue.GetChildren())) if queue else 0
    assert (n_parked, n_queue) == (6, 2), f"차량 수 이상: 주차 {n_parked}, 대기 {n_queue}"

    size_mb = TARGET_USD.stat().st_size / 1e6
    print(f"[flat] 검증 통과 — 마커 {len(ids)}장, 로컬 참조 0, "
          f"온라인 라이다 참조 {len(online)}개 유지, 차량 6+2, {size_mb:.2f} MB")


def main():
    _reexec()
    n_lidar, n_tex = build()
    verify()
    print(f"[flat] 출력: {TARGET_USD}")
    print(f"[flat] 라이다 온라인 참조 복원 {n_lidar}개, 텍스처 경로 정규화 {n_tex}개")
    print("[flat] 외부 의존: ../textures/aruco/*.png (이미지는 USD 내장 불가) "
          "+ 라이다 S3 (오프라인이면 빈 프림)")


if __name__ == "__main__":
    main()
