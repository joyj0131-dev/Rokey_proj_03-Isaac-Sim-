#!/usr/bin/env python3
"""M2 관문 검증: 합성 이미지에서 마커 자세가 복원되는가. (Isaac/ROS 불필요)

방법: 알려진 자세 T_cam_marker 로 마커를 3D 에 놓고, K 로 코너를 픽셀에 투영해
마커 타일 이미지를 그 사각형에 워프한다. 그 합성 이미지를 aruco_pose 모듈에
흘려보내 복원된 tvec/rvec 가 넣은 값과 같은지 잰다.

여러 자세(거리·기울기·평면내 회전)를 스윕해 위치·회전 오차 분포를 낸다.
solvePnP 가 마커 크기를 실제와 다르게 잡으면 거리 오차로 곧장 드러난다.

실행:
    python3 test_aruco_pose.py            # 표준 스윕 + PASS/FAIL
    pytest test_aruco_pose.py             # 동일 검증을 assert 로
"""

import json
import math
import os
import sys
from pathlib import Path

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from parkbot_aruco import aruco_pose as AP  # noqa: E402

# 지도는 이 패키지가 소유한다. test/ → 패키지 루트 → data/marker_map.json
MAP_JSON = Path(__file__).resolve().parents[1] / "data" / "marker_map.json"

# 검출 성공 기준(제안). 합성이라 렌즈 왜곡·노이즈가 없어 실제보다 낙관적이다.
#
# 케이스는 **실제 관측 기하**를 쓴다: 바닥 마커를 30도 기울인 카메라로 보면
# 마커는 카메라 프레임에서 늘 크게 눕는다(pitch 25~60도). 정면(pitch 0) 마커는
# 이 시스템에 존재하지 않으므로 검증하지 않는다.
#
# 게이트를 물리에 맞춰 셋으로 나눈다:
#  - 재투영 < 1px    : solver 가 코너를 실제로 설명하는가(순수 품질 지표).
#  - 회전 < 3도       : 마커 방향 복원.
#  - **횡방향** 위치 < 15mm : 카메라 시선에 직교한 성분. 차선 추종에 쓰는 값.
# 깊이(시선 방향) 오차는 따로 '보고만' 한다 — 작은 평면 사각형은 깊이 관측성이
# 원리적으로 약해(단일 프레임 최대 수십 mm) solver 품질과 무관하고, M3 필터가
# 오도메트리와 융합해 줄인다. 차선 추종은 횡방향·yaw 로 하지 깊이로 하지 않는다.
REPROJ_TOL_PX = 1.0     # 재투영 오차 허용 [px]
ROT_TOL_DEG = 3.0       # 회전 오차 허용 [deg]
LATERAL_TOL_MM = 15.0   # 횡방향(시선 직교) 위치 오차 허용 [mm]
IMG_W, IMG_H = 640, 480
HFOV_DEG = 90.5         # 팀원 깊이캠 실측


def _load_map():
    d = json.loads(MAP_JSON.read_text(encoding="utf-8"))
    return d["dictionary"], float(d["code_size_m"]), int(d["tile_cells"]), \
        int(d["code_cells"])


def _tile_image(dictionary_name, marker_id, code_cells, tile_cells, cell_px=40):
    """흰 정숙지대 + 코드로 이루어진 마커 타일(그레이). M1 배치와 동일."""
    dic = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, dictionary_name))
    code_px = code_cells * cell_px
    if hasattr(cv2.aruco, "generateImageMarker"):
        code = cv2.aruco.generateImageMarker(dic, marker_id, code_px, borderBits=1)
    else:
        code = cv2.aruco.drawMarker(dic, marker_id, code_px, borderBits=1)
    quiet = (tile_cells - code_cells) // 2 * cell_px
    tile = np.full((code_px + 2 * quiet, code_px + 2 * quiet), 255, np.uint8)
    tile[quiet:quiet + code_px, quiet:quiet + code_px] = code
    return tile


def _rvec_from_euler(roll, pitch, yaw):
    """도 단위 오일러 → 로드리게스 rvec (T_cam_marker 의 R).

    기저는 '마커가 카메라를 향한 자세' = X축 180도(marker +Z 가 카메라 쪽).
    이게 있어야 estimate 쪽 marker_object_points(Y-up)와 규약이 일치해 복원된
    rvec 를 gt 와 직접 비교할 수 있다. 기저를 빼면 렌더가 상하 반전돼 마커가
    깨지거나(구 버전), 회전오차가 180도씩 어긋난다.
    그 위에 pitch(하향 관측), roll, yaw(평면내 회전)를 얹는다.
    """
    R_face, _ = cv2.Rodrigues(np.array([math.pi, 0, 0]))
    R_r, _ = cv2.Rodrigues(np.array([math.radians(roll), 0, 0]))
    R_p, _ = cv2.Rodrigues(np.array([0, math.radians(pitch), 0]))
    R_y, _ = cv2.Rodrigues(np.array([0, 0, math.radians(yaw)]))
    rvec, _ = cv2.Rodrigues(R_p @ R_r @ R_y @ R_face)
    return rvec


def _render(tile, code_size, tile_size, rvec, tvec, K, dist):
    """마커를 자세(rvec,tvec)로 놓고 합성 이미지를 만든다. 반환: 그레이 이미지."""
    h = tile_size / 2.0
    # estimate 와 같은 Y-up 규약(marker_object_points 와 동일 순서). gt rvec 에
    # '카메라 향함' 기저가 들어 있어, 이 점들을 gt 자세로 투영하면 마커가 상하
    # 반전 없이 시각적으로 올바르게 그려진다. 타일 이미지 코너 [TL,TR,BR,BL] ↔
    # [(-h,h),(h,h),(h,-h),(-h,-h)].
    obj_tile = np.array([[-h, h, 0], [h, h, 0], [h, -h, 0], [-h, -h, 0]],
                        dtype=np.float64)
    img_pts, _ = cv2.projectPoints(obj_tile, rvec, tvec, K, dist)
    dst = img_pts.reshape(-1, 2).astype(np.float32)
    th, tw = tile.shape
    src = np.array([[0, 0], [tw - 1, 0], [tw - 1, th - 1], [0, th - 1]],
                   dtype=np.float32)
    H = cv2.getPerspectiveTransform(src, dst)
    canvas = np.full((IMG_H, IMG_W), 255, np.uint8)
    # dst=canvas + BORDER_TRANSPARENT: 사각형 밖은 흰 캔버스를 그대로 둔다.
    # dst 를 안 넘기면 밖이 0(검정)이 되어 마커 정숙지대가 사라져 검출이 실패한다.
    cv2.warpPerspective(tile, H, (IMG_W, IMG_H), dst=canvas,
                        borderMode=cv2.BORDER_TRANSPARENT)
    return canvas


def _rot_err_deg(rvec_gt, rvec_est):
    Rg, _ = cv2.Rodrigues(rvec_gt)
    Re, _ = cv2.Rodrigues(rvec_est)
    dR = Rg.T @ Re
    cos = (np.trace(dR) - 1.0) / 2.0
    return math.degrees(math.acos(max(-1.0, min(1.0, cos))))


def run_sweep(verbose=True):
    dictionary, code_size, tile_cells, code_cells = _load_map()
    tile_size = code_size * tile_cells / code_cells
    K, dist = AP.intrinsics_from_fov(IMG_W, IMG_H, HFOV_DEG)
    detector = AP.make_detector(dictionary)
    tile = _tile_image(dictionary, 2, code_cells, tile_cells)

    # (라벨, tvec, 오일러) — 바닥 마커를 기울인 카메라로 보는 실제 기하.
    # pitch 는 카메라 하향각(~30도)에 마커까지 거리에 따른 부각이 더해진 값.
    # 가까울수록 부각이 커져 pitch 가 크다(1.1m ~55도, 1.4m ~45도 수준).
    cases = [
        ("근거리 pitch55", [0.0, 0.0, 1.1], (0, 55, 0)),
        ("표준 pitch45", [0.0, 0.0, 1.25], (0, 45, 0)),
        ("원거리 pitch35", [0.0, 0.0, 1.4], (0, 35, 0)),
        ("좌치우침 pitch50", [-0.25, 0.0, 1.2], (0, 50, 0)),
        ("우치우침 pitch50", [0.25, 0.0, 1.2], (0, 50, 0)),
        ("평면내회전 yaw30", [0.0, 0.0, 1.25], (0, 45, 30)),
        ("평면내회전 yaw-25", [0.1, 0.0, 1.2], (0, 48, -25)),
        ("복합 roll10", [0.15, -0.05, 1.2], (10, 45, 20)),
    ]

    rows, lat_errs, dep_errs, rot_errs = [], [], [], []
    for label, tvec, euler in cases:
        tvec = np.array(tvec, dtype=np.float64).reshape(3, 1)
        rvec = _rvec_from_euler(*euler)
        gray = _render(tile, code_size, tile_size, rvec, tvec, K, dist)
        poses = AP.detect_and_estimate(gray, detector, code_size, K, dist)
        hit = next((p for p in poses if p.marker_id == 2), None)
        if hit is None:
            rows.append((label, None))
            continue
        err = (hit.tvec - tvec).flatten()                 # 카메라 프레임 오차 [m]
        lateral_mm = float(np.hypot(err[0], err[1])) * 1000.0   # 시선 직교
        depth_mm = float(err[2]) * 1000.0                       # 시선 방향
        rot_err = _rot_err_deg(rvec, hit.rvec)
        lat_errs.append(lateral_mm)
        dep_errs.append(abs(depth_mm))
        rot_errs.append(rot_err)
        ok = (hit.reproj_err_px <= REPROJ_TOL_PX
              and rot_err <= ROT_TOL_DEG
              and lateral_mm <= LATERAL_TOL_MM)
        rows.append((label, {"ok": ok, "lateral_mm": lateral_mm,
                             "depth_mm": depth_mm, "rot_deg": rot_err,
                             "reproj_px": hit.reproj_err_px}))

    detected = sum(1 for _l, r in rows if r is not None)
    passed = sum(1 for _l, r in rows if r is not None and r["ok"])
    result = {
        "cases": len(cases), "detected": detected, "passed": passed,
        "lateral_err_mm": {"max": round(max(lat_errs), 3) if lat_errs else None},
        "depth_err_mm": {"max": round(max(dep_errs), 3) if dep_errs else None},
        "rot_err_deg": {"max": round(max(rot_errs), 3) if rot_errs else None},
        "tol": {"reproj_px": REPROJ_TOL_PX, "rot_deg": ROT_TOL_DEG,
                "lateral_mm": LATERAL_TOL_MM},
        "all_passed": passed == len(cases) and detected == len(cases),
    }

    if verbose:
        print(f"\n[M2] 합성 자세복원 검증 — {len(cases)}케이스", flush=True)
        print(f"[M2] 게이트: 재투영<{REPROJ_TOL_PX}px, 회전<{ROT_TOL_DEG}°, "
              f"횡방향<{LATERAL_TOL_MM}mm (깊이는 보고만)", flush=True)
        for label, r in rows:
            if r is None:
                print(f"  검출실패 {label}", flush=True)
            else:
                mark = "OK  " if r["ok"] else "FAIL"
                print(f"  {mark} {label:<20} 횡 {r['lateral_mm']:5.1f}mm  "
                      f"깊이 {r['depth_mm']:+6.1f}mm  회전 {r['rot_deg']:4.2f}°  "
                      f"재투영 {r['reproj_px']:.2f}px", flush=True)
        print(f"[M2] 검출 {detected}/{len(cases)}, 통과 {passed}/{len(cases)} | "
              f"횡오차 최대 {result['lateral_err_mm']['max']}mm, "
              f"깊이오차 최대 {result['depth_err_mm']['max']}mm, "
              f"회전오차 최대 {result['rot_err_deg']['max']}°", flush=True)
        print(f"M2_POSE_PASSED={result['all_passed']}", flush=True)
    return result


def test_aruco_pose_recovery():
    """pytest 진입점."""
    r = run_sweep(verbose=False)
    assert r["detected"] == r["cases"], f"검출 실패 케이스 있음: {r}"
    assert r["all_passed"], f"자세 오차 허용 초과: {r}"


if __name__ == "__main__":
    res = run_sweep(verbose=True)
    sys.exit(0 if res["all_passed"] else 1)
