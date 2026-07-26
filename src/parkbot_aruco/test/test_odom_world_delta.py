"""marker_localizer_node.odom_world_delta 순수 함수 회귀 테스트.

_on_odom 이 프레임마다 계산하는 월드 증분(dx, dz, dyaw) 로직을 노드/rclpy
스핀 없이 검증한다. 기존 스크래치패드 스크립트 검증을 커밋된 회귀 테스트로
승격한다.
"""
import math
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(REPO / "src" / "parkbot_aruco"))

from parkbot_aruco.marker_localizer_node import odom_world_delta  # noqa: E402


def test_first_sample_is_noop():
    """prev 가 None(첫 odom 표본)이면 증분 없이 None 을 반환한다."""
    assert odom_world_delta(None, (0.0, 0.0, 0.0)) is None


def test_straight_motion_uses_x_and_z():
    """직선 이동: dx, dz 는 단순 뺄셈, dyaw=0.

    x, z(러너 odom 의 XZ 평면)가 실제로 쓰인다는 걸 못박는다 — 다른 곳의
    position.y 버그는 여기서 못 잡지만, 델타 계산 자체는 지킨다.
    """
    delta = odom_world_delta((1.0, 2.0, 0.0), (1.5, 2.7, 0.0))
    assert delta is not None
    dx, dz, dyaw = delta
    assert math.isclose(dx, 0.5, abs_tol=1e-9)
    assert math.isclose(dz, 0.7, abs_tol=1e-9)
    assert math.isclose(dyaw, 0.0, abs_tol=1e-9)


def test_yaw_wrap_positive_edge():
    """170 -> -170 은 -340 이 아니라 +20 으로 접혀야 한다."""
    _, _, dyaw = odom_world_delta((0.0, 0.0, 170.0), (0.0, 0.0, -170.0))
    assert math.isclose(dyaw, 20.0, abs_tol=1e-9)


def test_yaw_wrap_negative_edge():
    """-170 -> 170 은 +340 이 아니라 -20 으로 접혀야 한다."""
    _, _, dyaw = odom_world_delta((0.0, 0.0, -170.0), (0.0, 0.0, 170.0))
    assert math.isclose(dyaw, -20.0, abs_tol=1e-9)


def test_yaw_no_wrap():
    """랩어라운드가 필요 없는 일반적인 경우엔 그대로 뺄셈."""
    _, _, dyaw = odom_world_delta((0.0, 0.0, 10.0), (0.0, 0.0, 40.0))
    assert math.isclose(dyaw, 30.0, abs_tol=1e-9)
