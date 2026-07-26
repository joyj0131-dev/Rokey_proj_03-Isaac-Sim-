"""휠 오도메트리 적분 단위 테스트."""
import math
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(REPO / "isaacpjt" / "Isaac_envo"))

from wheel_odometry import WheelOdometry     # noqa: E402


def test_forward_integrates_along_plus_z_when_yaw_zero():
    """yaw=0 은 월드 +Z 를 향한다(프로젝트 규약)."""
    od = WheelOdometry()
    od.update(1.0, 0.0, 0.0, 1.0)
    assert math.isclose(od.z, 1.0, abs_tol=1e-9)
    assert math.isclose(od.x, 0.0, abs_tol=1e-9)


def test_left_strafe_integrates_along_plus_x_when_yaw_zero():
    """+Z 를 볼 때 로봇 좌측(+vy)은 월드 +X 다.

    marker_localizer.PoseFilter.predict_body 의 캘리브된 규약과 동일:
    바디 프레임은 X-전방, Y-좌, Z-업 우수 좌표계이고 월드는 Y-업 우수
    좌표계이므로, 바디-좌 = up × forward = (+Y_world) × (+Z_world) = +X_world.
    """
    od = WheelOdometry()
    od.update(0.0, 1.0, 0.0, 1.0)
    assert math.isclose(od.x, 1.0, abs_tol=1e-9)
    assert math.isclose(od.z, 0.0, abs_tol=1e-9)


def test_matches_calibrated_predict_body():
    """이미 GT 로 캘리브된 marker_localizer.PoseFilter.predict_body 와 일치해야 한다.

    독립적인 오라클로 검증한다 — 구현이 스스로를 정당화하지 못하게 한다.
    """
    sys.path.insert(0, str(REPO / "src" / "parkbot_aruco"))
    from parkbot_aruco.marker_localizer import PoseFilter

    for vx, vy, wz, yaw0 in [(1.0, 0.0, 0.0, 0.0), (0.0, 1.0, 0.0, 0.0),
                             (0.7, -0.4, 0.0, math.radians(35.0)),
                             (-0.3, 0.9, 0.0, math.radians(-110.0))]:
        od = WheelOdometry(yaw=yaw0)
        od.update(vx, vy, wz, 0.5)
        pf = PoseFilter()
        pf.x, pf.z, pf.yaw = 0.0, 0.0, math.degrees(yaw0)
        pf.predict_body(vx, vy, wz, 0.5)
        assert math.isclose(od.x, pf.x, abs_tol=1e-9), (vx, vy, yaw0)
        assert math.isclose(od.z, pf.z, abs_tol=1e-9), (vx, vy, yaw0)


def test_yaw_accumulates_and_wraps():
    od = WheelOdometry()
    od.update(0.0, 0.0, 1.0, math.pi)          # +pi
    assert math.isclose(abs(od.yaw), math.pi, abs_tol=1e-6)
    od.update(0.0, 0.0, 1.0, math.pi)          # 총 2pi -> 0 근처
    assert math.isclose(od.yaw, 0.0, abs_tol=1e-6)


def test_rotated_then_forward():
    od = WheelOdometry(yaw=math.pi / 2)        # +X 를 향함
    od.update(1.0, 0.0, 0.0, 1.0)
    assert math.isclose(od.x, 1.0, abs_tol=1e-9)
    assert math.isclose(od.z, 0.0, abs_tol=1e-9)


def test_zero_dt_is_noop():
    od = WheelOdometry(x=3.0, z=4.0)
    od.update(9.0, 9.0, 9.0, 0.0)
    assert (od.x, od.z) == (3.0, 4.0)
