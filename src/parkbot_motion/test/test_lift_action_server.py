"""lift_action_server 의 순수 함수(ROS 무의존) 단위테스트.

rclpy/parking_robot_interfaces 는 이 모듈이 import-time 에 필요로 하지만
(Node 서브클래스 정의, ControlLift 액션 타입 임포트), 이 테스트가 실제로
부르는 함수(normalize_lift_command)는 문자열 하나만 받는 평범한 함수라
rclpy.init()/노드 생성/스핀이 전혀 필요 없다.
"""
import pathlib
import sys

import pytest

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))
from parkbot_motion.lift_action_server import normalize_lift_command


def test_normalize_lift_command_up():
    assert normalize_lift_command('UP') == 'UP'


def test_normalize_lift_command_down():
    assert normalize_lift_command('DOWN') == 'DOWN'


def test_normalize_lift_command_case_insensitive():
    assert normalize_lift_command('up') == 'UP'
    assert normalize_lift_command(' Down ') == 'DOWN'


def test_normalize_lift_command_rejects_unknown():
    with pytest.raises(ValueError):
        normalize_lift_command('SIDEWAYS')
