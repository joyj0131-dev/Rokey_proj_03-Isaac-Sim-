#!/usr/bin/env python3
"""v4 주차장 USD(ASCII) → marker_map_v4.json 생성 (순수 파이썬, Isaac 불필요).

마커 위치는 **실제 데칼 메시의 xformOp:translate(x,z)** 를 쓴다. aruco:position 속성은
도크 마커에서 데칼과 z 로 0.7m 어긋나 있어(스펙 부록 A/C), 카메라가 실제로 보는 데칼
좌표를 써야 측위가 맞는다. 데칼 좌표를 쓰면 도크/슬롯 구분 없이 전 마커가 정확하다.

손으로 좌표를 옮겨 적지 않는다 — 에셋이 바뀌면 재실행 한 번으로 지도가 갱신된다.
"""
import json
import re
import sys
from pathlib import Path

WORK_DIR = Path(__file__).resolve().parent
REPO_ROOT = WORK_DIR.parent.parent
V4_USD = WORK_DIR / "parking" / "parking_environment_v4.usd"
OUT_JSON = REPO_ROOT / "src" / "parkbot_aruco" / "data" / "marker_map_v4.json"

sys.path.insert(0, str(REPO_ROOT / "src" / "parkbot_aruco"))
from parkbot_aruco import site_map_v4 as sm   # noqa: E402

_NUM = r"[-+]?\d*\.?\d+(?:[eE][-+]?\d+)?"


def parse_v4_markers(usd_text):
    """ASCII USD 에서 aruco 마커 Mesh 를 추출한다.

    마커는 leaf `def Mesh` 프림이라 그 안에 다른 def 가 없다. 'def Mesh "' 로 쪼갠 각
    청크에서 첫 aruco:markerId / serves / kind / yaw 와 첫 xformOp:translate 를 읽는다.
    translate 는 마커 자신의 데칼 위치다.
    """
    out = []
    for chunk in usd_text.split('def Mesh "')[1:]:
        mid = re.search(r"aruco:markerId\s*=\s*(\d+)", chunk)
        if not mid:
            continue
        serves = re.search(r'aruco:serves\s*=\s*"([^"]+)"', chunk)
        kind = re.search(r'aruco:kind\s*=\s*"([^"]+)"', chunk)
        yaw = re.search(rf"aruco:yaw\s*=\s*({_NUM})", chunk)
        tr = re.search(
            rf"xformOp:translate\s*=\s*\(\s*({_NUM})\s*,\s*({_NUM})\s*,\s*({_NUM})\s*\)",
            chunk)
        if not (serves and tr):
            continue
        out.append({
            "id": int(mid.group(1)),
            "serves": serves.group(1),
            "kind": kind.group(1) if kind else "",
            "yaw": float(yaw.group(1)) if yaw else 0.0,
            "x": float(tr.group(1)),      # 데칼 x
            "z": float(tr.group(3)),      # 데칼 z (translate[2])
        })
    return out


def build_map(usd_text):
    """파싱한 마커에 role(ENTRY/EXIT)을 붙이고 지도 dict 를 만든다."""
    markers = parse_v4_markers(usd_text)
    code_size = 0.19444445
    dict_name = "DICT_5X5_100"
    m0 = re.search(r'aruco:codeSize\s*=\s*(' + _NUM + ')', usd_text)
    if m0:
        code_size = float(m0.group(1))
    d0 = re.search(r'aruco:dictionary\s*=\s*"([^"]+)"', usd_text)
    if d0:
        dict_name = d0.group(1)
    out_markers = []
    for m in markers:
        out_markers.append({
            "id": m["id"], "serves": m["serves"], "kind": m["kind"],
            "x": m["x"], "z": m["z"], "yaw": m["yaw"],
            "role": sm.role_of(m["serves"]),
        })
    out_markers.sort(key=lambda x: x["id"])
    return {
        "dictionary": dict_name,
        "code_size_m": code_size,
        "source": "parking_environment_v4.usd (decal xformOp:translate)",
        "note": ("마커 x,z 는 데칼 실좌표(translate). 도크 마커는 aruco:position 과 "
                 "z 로 0.7m 다르므로 데칼을 써야 측위가 맞는다."),
        "markers": out_markers,
    }


def main():
    usd_text = V4_USD.read_text(encoding="utf-8")
    data = build_map(usd_text)
    OUT_JSON.parent.mkdir(parents=True, exist_ok=True)
    OUT_JSON.write_text(json.dumps(data, indent=2, ensure_ascii=False), encoding="utf-8")
    print(f"marker_map_v4.json 저장: {OUT_JSON}  마커 {len(data['markers'])}장 "
          f"(dict={data['dictionary']} code_size={data['code_size_m']})")


if __name__ == "__main__":
    main()
