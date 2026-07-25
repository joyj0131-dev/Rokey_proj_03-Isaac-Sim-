"""pose_controller_node 의 쿼터니언<->yaw_deg 변환 순수 단위테스트.

rclpy 는 이 모듈(pose_controller_node.py)이 import-time 에 필요로 하지만(Node
서브클래스 정의), 이 테스트가 실제로 부르는 두 함수(odom_quat_to_yaw_deg,
goal_quat_to_yaw_deg)는 평범한 float 4개를 받는 순수 함수라 rclpy.init()/노드
생성/스핀이 전혀 필요 없다 — "ROS 없이"의 실질(런타임 의존 없음)은 만족한다.
이 개발 머신에는 시스템 ROS2 Humble(rclpy)가 설치돼 있어(다른 parkbot_aruco
테스트들도 이미 geometry_msgs/nav_msgs 를 import 한다) import 자체는 항상
성공한다.
"""
import math

import pytest

from parkbot_motion.pose_controller_node import goal_quat_to_yaw_deg, odom_quat_to_yaw_deg


def _odom_quat_for_yaw_deg(yaw_deg):
    """R2 브리지 인코딩(publish_odom): x=y=0, z=sin(yaw/2), w=cos(yaw/2)."""
    yr = math.radians(yaw_deg)
    return (0.0, 0.0, math.sin(yr * 0.5), math.cos(yr * 0.5))


def _goal_quat_for_yaw_deg(yaw_deg):
    """순수 월드 Y축 회전: x=0, z=0, y=sin(yaw/2), w=cos(yaw/2)."""
    yr = math.radians(yaw_deg)
    return (0.0, math.sin(yr * 0.5), 0.0, math.cos(yr * 0.5))


@pytest.mark.parametrize("yaw_deg", [0.0, 1.0, 30.0, 45.0, 89.9, 90.0, 91.0,
                                      135.0, 179.0, -30.0, -90.0, -135.0])
def test_odom_quat_round_trips(yaw_deg):
    x, y, z, w = _odom_quat_for_yaw_deg(yaw_deg)
    got = odom_quat_to_yaw_deg(x, y, z, w)
    assert got == pytest.approx(yaw_deg, abs=1e-6)


@pytest.mark.parametrize("yaw_deg", [0.0, 1.0, 30.0, 45.0, 89.9, 90.0, 91.0,
                                      135.0, 179.0, -30.0, -90.0, -135.0])
def test_goal_quat_round_trips(yaw_deg):
    x, y, z, w = _goal_quat_for_yaw_deg(yaw_deg)
    got = goal_quat_to_yaw_deg(x, y, z, w)
    assert got == pytest.approx(yaw_deg, abs=1e-6)


def test_identity_quaternion_is_zero_yaw_both_conventions():
    assert odom_quat_to_yaw_deg(0.0, 0.0, 0.0, 1.0) == pytest.approx(0.0, abs=1e-9)
    assert goal_quat_to_yaw_deg(0.0, 0.0, 0.0, 1.0) == pytest.approx(0.0, abs=1e-9)


@pytest.mark.parametrize("yaw_deg", [-179.0, -90.0, -1.0, 0.0, 1.0, 90.0, 179.0])
def test_odom_and_goal_conventions_agree_on_same_project_yaw(yaw_deg):
    """같은 프로젝트 yaw 를 각자의(다른) 쿼터니언 인코딩으로 표현해도 두 변환
    함수가 같은 값을 복원해야 한다 — odom(z-슬롯 인코딩)과 goal(진짜 월드
    Y축 회전)은 서로 다른 쿼터니언이지만 같은 "이 프로젝트의 yaw" 의미를
    나타내므로."""
    ox, oy, oz, ow = _odom_quat_for_yaw_deg(yaw_deg)
    gx, gy, gz, gw = _goal_quat_for_yaw_deg(yaw_deg)
    assert odom_quat_to_yaw_deg(ox, oy, oz, ow) == pytest.approx(
        goal_quat_to_yaw_deg(gx, gy, gz, gw), abs=1e-6)
