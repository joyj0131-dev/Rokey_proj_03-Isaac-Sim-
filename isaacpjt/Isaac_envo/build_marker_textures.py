#!/usr/bin/env python3
"""ArUco 마커 텍스처(PNG)와 marker_map.json 을 생성한다.

산출물:
  textures/aruco/aruco_<사전>_<ID3자리>.png   마커 44장
  marker_map.json                             측위 노드가 읽는 유일한 지도 파일

marker_map.json 을 따로 내보내는 이유: ROS 노드가 marker_layout.py 를 import 하지
않아도 되게 하기 위함이다. 노드는 Isaac 밖 python 3.10에서 돌고, 지도는 검사 가능한
데이터 파일로 남는 편이 낫다.

cv2 가 필요하므로 Isaac python(3.11)으로 갈아탄다. SimulationApp 은 띄우지 않으므로
GPU 도 쓰지 않는다.

실행:
    python3 build_marker_textures.py
    python3 build_marker_textures.py --check   # 기존 PNG 재검증만
"""

import json
import os
import subprocess
import sys
from pathlib import Path

import marker_layout as ML

WORK_DIR = Path(__file__).resolve().parent
REPO_ROOT = WORK_DIR.parents[1]
TEX_DIR = WORK_DIR / "textures" / "aruco"
# 텍스처(PNG)는 Isaac 쪽에 남는다(실제 배포에선 물리 인쇄물). 그러나 측위 지도
# marker_map.json 은 **순수 ROS 패키지가 소유**한다 — 실제 로봇엔 Isaac 이 없고
# 노드가 런타임에 이 지도로 측위하기 때문. colcon 이 share/ 로 설치한다.
MAP_JSON = REPO_ROOT / "src" / "parkbot_aruco" / "data" / "marker_map.json"
ISAAC_RELEASE = Path("/home/rokey/dev_ws/isaac_sim/isaacsim/_build/linux-x86_64/release")

# 셀 하나를 몇 픽셀로 그릴지. CODE_PX 가 CODE_CELLS 의 정수배여야 리샘플링
# 아티팩트가 안 생긴다(7 x 64 = 448).
CELL_PX = 64
CODE_PX = ML.CODE_CELLS * CELL_PX     # 448
TILE_PX = ML.TILE_CELLS * CELL_PX     # 576


def _has_cv2():
    """cv2 의 '존재'가 아니라 '쓸 수 있는 aruco API'가 있는지 본다.

    시스템 python(3.10)의 cv2 4.5.4 와 Isaac(3.11)의 4.11 은 aruco API가 완전히 다르다.
    존재만 확인하면 4.5.4 에 머물러 generateImageMarker 에서 터진다.
    """
    try:
        import cv2

        a = cv2.aruco
        return hasattr(a, "generateImageMarker") or hasattr(a, "drawMarker")
    except ModuleNotFoundError:
        return False


def _reexec():
    if _has_cv2():
        return
    if os.environ.get("_MARKER_TEX_REEXEC"):
        raise RuntimeError("재실행 후에도 쓸 수 있는 cv2.aruco를 찾지 못했습니다.")
    python_sh = ISAAC_RELEASE / "python.sh"
    if not python_sh.is_file():
        raise FileNotFoundError(f"Isaac python.sh를 찾을 수 없습니다: {python_sh}")
    env = dict(
        os.environ,
        _MARKER_TEX_REEXEC="1",
        PYTHONPATH=os.pathsep.join(
            [str(WORK_DIR), os.environ.get("PYTHONPATH", "")]
        ).strip(os.pathsep),
    )
    raise SystemExit(
        subprocess.call(
            [str(python_sh), str(Path(__file__).resolve()), *sys.argv[1:]],
            env=env,
            cwd=str(ISAAC_RELEASE),
        )
    )


def texture_name(marker_id):
    return f"aruco_{ML.ARUCO_DICT}_{marker_id:03d}.png"


# --- cv2 aruco 호환 계층 -------------------------------------------------
# OpenCV 4.7 에서 aruco API가 갈아엎어졌다. 이 프로젝트는 두 버전을 다 만난다:
#   Isaac  python 3.11 : 4.11  (generateImageMarker / ArucoDetector)
#   시스템 python 3.10 : 4.5.4 (drawMarker / detectMarkers 함수)
# 같은 코드가 양쪽에서 돌아야 하므로 여기서 흡수한다.

def _make_marker_image(dictionary, marker_id, side_px, border_bits):
    import cv2

    if hasattr(cv2.aruco, "generateImageMarker"):          # >= 4.7
        return cv2.aruco.generateImageMarker(
            dictionary, marker_id, side_px, borderBits=border_bits
        )
    return cv2.aruco.drawMarker(dictionary, marker_id, side_px, borderBits=border_bits)


def _make_detector(dictionary):
    """(image) -> (corners, ids) 함수를 돌려준다."""
    import cv2

    if hasattr(cv2.aruco, "ArucoDetector"):                # >= 4.7
        det = cv2.aruco.ArucoDetector(dictionary, cv2.aruco.DetectorParameters())
        return lambda img: det.detectMarkers(img)[:2]
    params = cv2.aruco.DetectorParameters_create()
    return lambda img: cv2.aruco.detectMarkers(img, dictionary, parameters=params)[:2]


def _get_dictionary():
    import cv2

    return cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, ML.ARUCO_DICT))


def generate():
    import cv2
    import numpy as np

    TEX_DIR.mkdir(parents=True, exist_ok=True)
    for old in TEX_DIR.glob("aruco_*.png"):
        old.unlink()

    dictionary = _get_dictionary()
    rows = ML.assign_ids()

    for marker_id, *_rest in rows:
        # sidePixels 는 테두리를 포함한 크기다.
        code = _make_marker_image(dictionary, marker_id, CODE_PX, ML.BORDER_CELLS)
        # 흰 여백(quiet zone)을 한 칸 두른다. 이게 없으면 검출 자체가 안 된다.
        tile = np.full((TILE_PX, TILE_PX), 255, dtype=np.uint8)
        tile[CELL_PX:CELL_PX + CODE_PX, CELL_PX:CELL_PX + CODE_PX] = code
        cv2.imwrite(str(TEX_DIR / texture_name(marker_id)), tile)

    payload = {
        "dictionary": ML.ARUCO_DICT,
        "tile_size_m": ML.MARKER_TILE,
        "code_size_m": ML.MARKER_CODE_SIZE,
        "marker_y_m": ML.MARKER_Y,
        "tile_cells": ML.TILE_CELLS,
        "code_cells": ML.CODE_CELLS,
        "note": (
            "code_size_m 이 solvePnP 에 넘길 값이다. tile_size_m 을 쓰면 "
            "거리가 tile/code = 9/7 = 1.286배로 어긋난다."
        ),
        "markers": [
            {
                "id": marker_id,
                "kind": kind,
                "label": label,
                "x": round(x, 4),
                "z": round(z, 4),
                # yaw = 마커의 평면내 방향[도]. 측위(marker_localizer)가 로봇 yaw 를
                # 복원할 때 쓴다. USD 배치(build_marker_layout)도 같은 값으로 돌린다.
                "yaw": round(yaw, 2),
                "texture": f"textures/aruco/{texture_name(marker_id)}",
            }
            for marker_id, kind, label, x, z, yaw, _note in rows
        ],
    }
    MAP_JSON.write_text(
        json.dumps(payload, indent=2, ensure_ascii=False), encoding="utf-8"
    )
    return len(rows)


def check():
    """생성한 PNG 를 다시 검출해 원래 ID 가 나오는지 확인한다.

    텍스처 파이프라인 자체의 셀프테스트다. 시뮬레이션이 필요 없고,
    여기서 실패하면 렌더링·카메라·자세추정을 볼 필요가 없다.
    """
    import cv2

    detect = _make_detector(_get_dictionary())

    ok, bad = 0, []
    for marker_id, *_rest in ML.assign_ids():
        path = TEX_DIR / texture_name(marker_id)
        if not path.is_file():
            bad.append((marker_id, "파일 없음"))
            continue
        image = cv2.imread(str(path), cv2.IMREAD_GRAYSCALE)
        _corners, ids = detect(image)
        if ids is None or len(ids) != 1:
            bad.append((marker_id, f"검출 {0 if ids is None else len(ids)}개"))
        elif int(ids[0][0]) != marker_id:
            bad.append((marker_id, f"ID 불일치 -> {int(ids[0][0])}"))
        else:
            ok += 1
    return ok, bad


def main():
    _reexec()
    if "--check" not in sys.argv[1:]:
        count = generate()
        print(f"[tex] {ML.ARUCO_DICT}, 마커 {count}장")
        print(f"[tex] 타일 {ML.MARKER_TILE:.3f} m ({TILE_PX}px) / "
              f"코드 {ML.MARKER_CODE_SIZE:.6f} m ({CODE_PX}px), 셀 {CELL_PX}px")
        print(f"[tex] PNG: {TEX_DIR}")
        print(f"[tex] 지도: {MAP_JSON}")

    ok, bad = check()
    print(f"[tex] 왕복 검출: {ok}장 통과, {len(bad)}장 실패")
    for marker_id, reason in bad:
        print(f"        ID {marker_id}: {reason}")
    if bad:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
