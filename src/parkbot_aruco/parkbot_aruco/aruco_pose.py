"""ArUco 검출 + 마커별 자세 추정 — 순수 파이썬(ROS/Isaac 무관).

이 모듈은 카메라 프레임에서 **마커의 자세** `T_cam_marker` 만 낸다.
월드 좌표로의 합성(`T_world_marker · inv(T_cam_marker) · inv(T_robot_cam)`)은
M3 의 몫이라 여기서는 하지 않는다. 그래야 ROS/Isaac 없이 단위 테스트가 된다.

왜 이 파일이 따로 있나:
  - ROS 노드(`aruco_detector.py`)는 rclpy·cv_bridge·메시지가 있어야 돌아가고
    카메라 토픽이 필요하다. 그걸 켜지 않고도 "검출·자세 복원이 맞는가"를
    합성 이미지로 검증할 수 있어야 한다(ARUCO_PLAN M2 검증 항목).

cv2 API 분기 (실측):
  - 시스템 python 3.10 : cv2 4.5.4  → aruco.detectMarkers / DetectorParameters_create
                                       estimatePoseSingleMarkers / solvePnP
  - Isaac  python 3.11 : cv2 4.11   → aruco.ArucoDetector / DetectorParameters
                                       estimatePoseSingleMarkers 는 제거됨
  겹치는 건 getPredefinedDictionary 뿐. 이 모듈은 노드(4.5.4)에서 돌지만
  누가 4.11 에서 import 해도 깨지지 않게 양쪽 API 를 모두 흡수한다.

자세 추정은 estimatePoseSingleMarkers 를 쓰지 않는다(4.11 에서 제거).
바닥에 평평한 마커는 평면 자세 모호성(두 해)에 걸리므로 solvePnPGeneric +
SOLVEPNP_IPPE_SQUARE 로 두 해를 받아 재투영 오차가 작은 해를 고르는 게 원래 계획이다.

그런데 실측으로 이 시스템의 cv2 4.5.4 빌드에서 **solvePnPGeneric+IPPE_SQUARE 가
정면·순수피치 마커에 대해 항등행렬(재투영 68~102px)만 반환하는 버그**가 확인됐다.
반면 SOLVEPNP_ITERATIVE 는 같은 코너에 전 케이스 재투영 0.4px 미만으로 맞았고,
4.5.4·4.11 양쪽에 다 있다. 그래서:
  1순위: IPPE_SQUARE 로 두 해를 받아 재투영 작은 해를 고른다(모호성 비도 같이 낸다).
  안전망: 1순위 최적해의 재투영이 크면(버그 케이스) ITERATIVE 로 다시 푼다.
바닥 마커는 기울인 카메라로 보면 늘 크게 눕는데(정면일 일이 없다), 그 영역에서는
IPPE 가 정상 동작하고 모호성도 잘 풀린다. 정면 버그는 안전망이 받는다.
"""

from dataclasses import dataclass, field
from typing import List, Optional, Tuple

import cv2
import numpy as np


# ---------- 검출기: 4.5 / 4.7+ API 흡수 ----------

def make_detector(dictionary_name: str):
    """dictionary 이름(예: 'DICT_5X5_100') → (gray)->(corners, ids) 호출체."""
    dic = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, dictionary_name))
    if hasattr(cv2.aruco, "ArucoDetector"):                 # >= 4.7
        params = cv2.aruco.DetectorParameters()
        det = cv2.aruco.ArucoDetector(dic, params)
        return lambda gray: det.detectMarkers(gray)[:2]
    params = cv2.aruco.DetectorParameters_create()           # 4.5
    return lambda gray: cv2.aruco.detectMarkers(gray, dic, parameters=params)[:2]


# ---------- 자세 추정 ----------

def marker_object_points(code_size_m: float) -> np.ndarray:
    """정사각 마커의 오브젝트 점 4개(마커 자체 프레임).

    detectMarkers 코너 순서(좌상→우상→우하→좌하)에 맞춘 표준 ArUco 규약:
    마커 평면에서 X=오른쪽, Y=위, Z=관측자 쪽(면 밖). estimatePoseSingleMarkers 와
    동일한 배치라 기존 자료와 부호가 어긋나지 않는다. 월드로의 매핑은 M3.
    """
    h = code_size_m / 2.0
    return np.array([
        [-h,  h, 0.0],
        [ h,  h, 0.0],
        [ h, -h, 0.0],
        [-h, -h, 0.0],
    ], dtype=np.float64)


def _reprojection_error(obj, img, rvec, tvec, K, dist) -> float:
    proj, _ = cv2.projectPoints(obj, rvec, tvec, K, dist)
    return float(np.sqrt(np.mean(np.sum((proj.reshape(-1, 2) - img) ** 2, axis=1))))


@dataclass
class MarkerPose:
    marker_id: int
    corners: np.ndarray               # (4,2) 픽셀
    rvec: np.ndarray                  # (3,1) 로드리게스, T_cam_marker
    tvec: np.ndarray                  # (3,1) m, 카메라 프레임에서 마커 위치
    reproj_err_px: float
    ambiguity: float                  # 2등해/1등해 재투영오차 비. 1에 가까우면 모호
    distance_m: float = field(init=False)

    def __post_init__(self):
        self.distance_m = float(np.linalg.norm(self.tvec))


# IPPE 최적해의 재투영이 이보다 크면 solver 버그로 보고 ITERATIVE 로 다시 푼다.
_IPPE_REPROJ_FALLBACK_PX = 2.0


def estimate_pose(corners: np.ndarray, code_size_m: float,
                  K: np.ndarray, dist: np.ndarray) -> Optional[MarkerPose]:
    """단일 마커 코너(4,2) → MarkerPose. 실패 시 None.

    IPPE_SQUARE 로 평면 두 해를 받아 재투영 작은 해를 고른다(ambiguity = 2등/1등
    오차 비, 게이팅용). 단 이 빌드의 IPPE 는 정면 마커에서 깨지므로, 최적해
    재투영이 크면 ITERATIVE 로 다시 푼다(모듈 상단 주석 참고).
    """
    img = np.asarray(corners, dtype=np.float64).reshape(-1, 2)
    obj = marker_object_points(code_size_m)

    ambiguity = 0.0
    best = None
    try:
        n, rvecs, tvecs, _ = cv2.solvePnPGeneric(
            obj, img, K, dist, flags=cv2.SOLVEPNP_IPPE_SQUARE)
        if n:
            scored = sorted(
                ((_reprojection_error(obj, img, rvecs[i], tvecs[i], K, dist),
                  rvecs[i], tvecs[i]) for i in range(n)),
                key=lambda s: s[0])
            best = scored[0]
            if len(scored) > 1 and scored[1][0] > 1e-9:
                ambiguity = best[0] / scored[1][0]
    except cv2.error:
        best = None

    # IPPE 실패/저품질 → ITERATIVE 안전망. ambiguity 는 알 수 없으므로 -1.
    if best is None or best[0] > _IPPE_REPROJ_FALLBACK_PX:
        try:
            ok, rvec, tvec = cv2.solvePnP(
                obj, img, K, dist, flags=cv2.SOLVEPNP_ITERATIVE)
        except cv2.error:
            return None
        if not ok:
            return None
        e = _reprojection_error(obj, img, rvec, tvec, K, dist)
        return MarkerPose(marker_id=-1, corners=img, rvec=rvec, tvec=tvec,
                          reproj_err_px=e, ambiguity=-1.0)

    return MarkerPose(marker_id=-1, corners=img, rvec=best[1], tvec=best[2],
                      reproj_err_px=best[0], ambiguity=ambiguity)


def detect_and_estimate(gray: np.ndarray, detector, code_size_m: float,
                        K: np.ndarray, dist: np.ndarray) -> List[MarkerPose]:
    """그레이 이미지 → 검출된 마커별 MarkerPose 리스트."""
    corners, ids = detector(gray)
    out: List[MarkerPose] = []
    if ids is None:
        return out
    for c, i in zip(corners, ids.flatten()):
        pose = estimate_pose(c.reshape(-1, 2), code_size_m, K, dist)
        if pose is None:
            continue
        pose.marker_id = int(i)
        out.append(pose)
    return out


# ---------- camera_info K/dist 헬퍼 ----------

def intrinsics_from_fov(width: int, height: int, hfov_deg: float
                        ) -> Tuple[np.ndarray, np.ndarray]:
    """수평 FOV → 핀홀 K(왜곡 0). 합성 테스트/센서 스펙에서 K 를 만들 때."""
    fx = (width / 2.0) / np.tan(np.radians(hfov_deg) / 2.0)
    fy = fx
    K = np.array([[fx, 0, width / 2.0],
                  [0, fy, height / 2.0],
                  [0, 0, 1.0]], dtype=np.float64)
    return K, np.zeros((5, 1), dtype=np.float64)
