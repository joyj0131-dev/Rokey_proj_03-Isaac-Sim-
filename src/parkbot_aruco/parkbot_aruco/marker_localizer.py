"""M3 — 마커 관측을 월드 로봇 자세로 바꾸고 오도메트리와 융합한다. (순수 파이썬)

M2 는 마커의 카메라 기준 자세 T_cam_marker 만 낸다. M3 는 그걸 "로봇이 주차장
월드 어디에 있나"(T_world_robot)로 바꾸고, 마커가 안 보이는 구간(경로의 ~91%)을
오도메트리로 메워 연속 자세를 낸다.

좌표 변환 체인:
    T_world_robot = T_world_marker · inv(T_cam_marker) · inv(T_base_cam)
                          │                  │                  │
                   marker_map(위치)      M2 결과          카메라 장착(고정)

주의 — 규약은 손으로 추론하지 않는다. ARUCO_PLAN 0절이 경고하듯 우수계·이미지
up·와인딩이 얽혀 첫 시도에 틀린다. marker→world 회전(R_base)과 카메라 장착
T_base_cam 은 Isaac ground truth 와 대조해 실측으로 맞춘다(m3_localize_demo.py).
여기서는 그 체인을 '파라미터화'만 해두고 값은 데모가 캘리브레이션한다.

스테이지 up=+Y, 바닥=XZ. 로봇 yaw ψ = atan2(fwd_x, fwd_z) (0=월드 +Z),
회전축 월드 +Y. (ARUCO_PLAN 0절 계약)
"""

import json
import math
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, Optional, Tuple

import numpy as np


def default_marker_map_path() -> Path:
    """측위 지도 marker_map.json 의 표준 위치를 찾는다.

    지도는 이 ROS 패키지가 소유한다(실제 배포엔 Isaac 이 없고 노드가 런타임에
    이 지도로 측위한다). 설치 트리(share/)를 먼저, 없으면 소스 트리(data/)를 본다.
    """
    try:
        from ament_index_python.packages import get_package_share_directory
        cand = Path(get_package_share_directory("parkbot_aruco")) / "data" / "marker_map.json"
        if cand.is_file():
            return cand
    except Exception:
        pass
    # 소스 트리 폴백: parkbot_aruco/parkbot_aruco/marker_localizer.py → 패키지 루트/data
    src = Path(__file__).resolve().parents[1] / "data" / "marker_map.json"
    return src


def load_marker_map(path: Optional[Path] = None, align_yaw_deg: float = 0.0
                    ) -> "MarkerMap":
    """지도 파일을 읽어 MarkerMap 을 만든다. path 생략 시 표준 위치."""
    p = Path(path) if path is not None else default_marker_map_path()
    data = json.loads(p.read_text(encoding="utf-8"))
    return MarkerMap.from_json(data, align_yaw_deg=align_yaw_deg)


# marker 프레임(M2 objectPoints: X=오른쪽, Y=위, Z=면 밖) → world 기본 회전.
# 마커가 바닥에 눕고 텍스처가 위(+Y)를 보면 Z_m→world +Y. 오른손계 유지로
# X_m→+X, Y_m→−Z 가 따라온다(= ARUCO_PLAN 0절 "X_m=+X, Y_m=−Z, Z_m=+Y").
# 마커별 평면내 회전(텍스처가 돌아간 각)은 R_align(yaw about +Y)로 더한다.
R_BASE_MARKER_TO_WORLD = np.array([
    [1.0, 0.0, 0.0],
    [0.0, 0.0, 1.0],
    [0.0, -1.0, 0.0],
], dtype=np.float64)


def rot_y(deg: float) -> np.ndarray:
    r = math.radians(deg)
    c, s = math.cos(r), math.sin(r)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]], dtype=np.float64)


def make_T(R: np.ndarray, t) -> np.ndarray:
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = np.asarray(t, dtype=np.float64).reshape(3)
    return T


def rvec_tvec_to_T(rvec, tvec) -> np.ndarray:
    import cv2
    R, _ = cv2.Rodrigues(np.asarray(rvec, dtype=np.float64))
    return make_T(R, np.asarray(tvec, dtype=np.float64).reshape(3))


def yaw_from_R(R: np.ndarray) -> float:
    """로봇 회전행렬 → nav yaw [deg]. 로봇 로컬 +X 를 월드로 보낸 벡터로 계산.

    ψ = atan2(fwd_x, fwd_z), 0 = 월드 +Z (ARUCO_PLAN 계약). 로봇 로컬 +X 가
    yaw=0 일 때 월드 +Z 를 향한다는 규약과 일치.
    """
    fwd = R @ np.array([1.0, 0.0, 0.0])   # 로봇 전방(로컬 +X)의 월드 방향
    return math.degrees(math.atan2(fwd[0], fwd[2]))


class MarkerMap:
    """marker_map.json → id별 월드 자세 T_world_marker."""

    def __init__(self, markers: Dict[int, dict], align_yaw_deg: float = 0.0):
        # align_yaw_deg: 텍스처→월드의 '전역 규약' 회전(GT 캘리브로 0 확인).
        # 마커별 배치 회전(json 의 "yaw")은 그 위에 더해진다.
        self.by_id = markers
        self.align_yaw_deg = align_yaw_deg

    @classmethod
    def from_json(cls, data: dict, align_yaw_deg: float = 0.0) -> "MarkerMap":
        by_id = {int(m["id"]): m for m in data["markers"]}
        return cls(by_id, align_yaw_deg)

    def has(self, marker_id: int) -> bool:
        return marker_id in self.by_id

    def T_world_marker(self, marker_id: int) -> np.ndarray:
        m = self.by_id[marker_id]
        # 마커는 바닥면(y = marker_y). 위치는 (x, z), 높이는 무시 가능 수준.
        # 회전 = 전역 규약(align_yaw) + 마커별 배치 yaw(json). yaw 필드가 없으면 0.
        yaw = float(m.get("yaw", 0.0))
        R = rot_y(self.align_yaw_deg + yaw) @ R_BASE_MARKER_TO_WORLD
        t = np.array([float(m["x"]), 0.0, float(m["z"])])
        return make_T(R, t)

    def label(self, marker_id: int) -> str:
        return self.by_id[marker_id].get("label", str(marker_id))


@dataclass
class RobotFix:
    """단일 마커 관측에서 복원한 로봇 월드 자세."""
    marker_id: int
    x: float
    z: float
    yaw_deg: float
    T_world_robot: np.ndarray = field(repr=False)


def robot_pose_from_marker(marker_id: int, T_cam_marker: np.ndarray,
                           T_base_cam: np.ndarray,
                           marker_map: MarkerMap) -> Optional[RobotFix]:
    """마커 하나 관측 → 로봇 월드 자세. 지도에 없는 마커면 None.

    T_world_robot = T_world_marker · inv(T_cam_marker) · inv(T_base_cam)
    """
    if not marker_map.has(marker_id):
        return None
    T_wm = marker_map.T_world_marker(marker_id)
    T_wr = T_wm @ np.linalg.inv(T_cam_marker) @ np.linalg.inv(T_base_cam)
    x, z = float(T_wr[0, 3]), float(T_wr[2, 3])
    yaw = yaw_from_R(T_wr[:3, :3])
    return RobotFix(marker_id, x, z, yaw, T_wr)


class PoseFilter:
    """오도메트리 예측 + 마커 관측 보정. (x, z, yaw) 3-DOF 상보 필터.

    마커는 경로의 ~9%에서만 보인다(창 1.1~1.4 m, 간격 3.4 m). 나머지는 오도메트리로
    적분하고, 마커가 잡히면 그 방향으로 당긴다. 실측 드리프트가 3.4 m에 2 cm라
    오도메트리 신뢰가 높아 게인을 낮게 둔다(마커 오검출에 덜 흔들리게).

    깊이(마커 시선 방향) 관측은 부정확(합성 최대 26 mm)하지만 여기선 이미 월드
    (x,z)로 바뀐 뒤라, 관측 신뢰를 위치/각도 각각의 게인으로만 조절한다.
    """

    def __init__(self, pos_gain: float = 0.5, yaw_gain: float = 0.5):
        self.x = None
        self.z = None
        self.yaw = None       # deg
        self.pos_gain = pos_gain
        self.yaw_gain = yaw_gain
        self.n_fix = 0
        self.n_pred = 0

    def set_pose(self, x: float, z: float, yaw_deg: float):
        self.x, self.z, self.yaw = x, z, yaw_deg

    def predict(self, dx: float, dz: float, dyaw_deg: float):
        """오도메트리 증분(월드 프레임 이동량)으로 예측 갱신."""
        if self.x is None:
            return
        self.x += dx
        self.z += dz
        self.yaw = _wrap_deg(self.yaw + dyaw_deg)
        self.n_pred += 1

    def predict_body_delta(self, d_fwd: float, d_left: float, dyaw_deg: float):
        """바디 프레임 '이동량' 증분을 자기 헤딩으로 월드 적분(dt 불필요).

        엔코더 오도메트리용: 바퀴 각도 변화 → FK → 바디 이동량(전방/좌/yaw).
        속도가 아니라 이동량이라 sim dt 를 몰라도 되고, 실제 바퀴 회전만큼만
        누적하므로 슬립이 곧 드리프트로 정직하게 드러난다.
        """
        if self.x is None:
            return
        psi = math.radians(self.yaw)
        self.x += d_fwd * math.sin(psi) + d_left * math.cos(psi)
        self.z += d_fwd * math.cos(psi) - d_left * math.sin(psi)
        self.yaw = _wrap_deg(self.yaw + dyaw_deg)
        self.n_pred += 1

    def predict_body(self, vx: float, vy: float, wz: float, dt: float):
        """바디 속도(전방 vx, 좌 vy, yaw율 wz[rad/s])를 자기 헤딩으로 월드 적분.

        로봇 규약: 전방(+X_body)→월드 +Z, 좌(+Y_body)→월드 +X (yaw=0 기준).
        nav yaw ψ 에서 전방=(sinψ,cosψ), 좌=(cosψ,−sinψ) [(x,z) 평면].
        오도메트리는 명령 속도를 적분하므로 실제 슬립만큼 GT 와 벌어진다(=드리프트).
        """
        if self.x is None:
            return
        psi = math.radians(self.yaw)
        self.x += (vx * math.sin(psi) + vy * math.cos(psi)) * dt
        self.z += (vx * math.cos(psi) - vy * math.sin(psi)) * dt
        self.yaw = _wrap_deg(self.yaw + math.degrees(wz * dt))
        self.n_pred += 1

    def update(self, fix: RobotFix):
        """마커 관측으로 보정. 첫 관측이면 그대로 초기화."""
        if self.x is None:
            self.set_pose(fix.x, fix.z, fix.yaw_deg)
        else:
            self.x += self.pos_gain * (fix.x - self.x)
            self.z += self.pos_gain * (fix.z - self.z)
            dyaw = _wrap_deg(fix.yaw_deg - self.yaw)
            self.yaw = _wrap_deg(self.yaw + self.yaw_gain * dyaw)
        self.n_fix += 1

    def pose(self) -> Optional[Tuple[float, float, float]]:
        if self.x is None:
            return None
        return (self.x, self.z, self.yaw)


def _wrap_deg(a: float) -> float:
    return (a + 180.0) % 360.0 - 180.0
