"""build_marker_map_v4 파서 단위테스트."""
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(REPO / "isaacpjt" / "Isaac_envo"))
sys.path.insert(0, str(REPO / "src" / "parkbot_aruco"))

import build_marker_map_v4 as B    # noqa: E402


# v4 USD 의 도크 마커 한 블록(발췌). aruco:position(z=2.2)과 데칼 translate(z=2.9)이 다르다.
DOCK_BLOCK = '''
        def Mesh "dock_ROBOT_OUT" (
            prepend apiSchemas = ["MaterialBindingAPI"]
        )
        {
            custom float aruco:codeSize = 0.19444445
            custom string aruco:dictionary = "DICT_5X5_100"
            custom string aruco:kind = "dock"
            custom int aruco:markerId = 21
            custom float3 aruco:position = (-3.2, 0, 2.2)
            custom string aruco:serves = "D_OUT_1"
            custom float aruco:yaw = 0
            double3 xformOp:translate = (-3.2, 0.0012, 2.9000000432133675)
            uniform token[] xformOpOrder = ["xformOp:translate"]
        }
        def Mesh "slot_A1" (
        )
        {
            custom string aruco:dictionary = "DICT_5X5_100"
            custom string aruco:kind = "slot"
            custom int aruco:markerId = 0
            custom float3 aruco:position = (2.8, 0, -6.875)
            custom string aruco:serves = "A1"
            custom float aruco:yaw = 0
            double3 xformOp:translate = (2.8, 0.0012, -6.875)
        }
'''


def test_dock_marker_uses_decal_translate_not_aruco_position():
    """도크 마커 x,z 는 데칼 translate(2.9)여야 한다 — aruco:position(2.2)이 아니라."""
    ms = {m["serves"]: m for m in B.parse_v4_markers(DOCK_BLOCK)}
    d = ms["D_OUT_1"]
    assert d["id"] == 21
    assert d["kind"] == "dock"
    assert abs(d["x"] - (-3.2)) < 1e-6
    assert abs(d["z"] - 2.9) < 1e-3        # 데칼 z, 0.7m 오프셋 반영
    assert abs(d["z"] - 2.2) > 0.5         # aruco:position 이 아님을 확실히


def test_slot_marker_parsed():
    ms = {m["serves"]: m for m in B.parse_v4_markers(DOCK_BLOCK)}
    a1 = ms["A1"]
    assert a1["id"] == 0 and a1["kind"] == "slot"
    assert abs(a1["x"] - 2.8) < 1e-6 and abs(a1["z"] - (-6.875)) < 1e-6


def test_build_map_adds_role_and_top_level():
    m = B.build_map(DOCK_BLOCK)
    assert m["dictionary"] == "DICT_5X5_100"
    assert abs(m["code_size_m"] - 0.19444445) < 1e-6
    by = {x["serves"]: x for x in m["markers"]}
    assert by["D_OUT_1"]["role"] == "ENTRY"    # z 양수 = 입차
    assert by["A1"]["role"] == "EXIT"          # z 음수 = 출차


def test_real_v4_usd_has_16_markers():
    usd = (REPO / "isaacpjt" / "Isaac_envo" / "parking"
           / "parking_environment_v4.usd").read_text(encoding="utf-8")
    markers = B.parse_v4_markers(usd)
    assert len(markers) == 16
    ids = sorted(m["id"] for m in markers)
    assert len(set(ids)) == 16                 # id 중복 없음
