#!/usr/bin/env python3
"""출차존 고정좌표 테스트 인형 스폰 위상 + 로봇 근접 판정 단위 테스트
(Isaac/ROS 불필요).

parking_v4_runner.py는 SimulationApp을 함수 안에서만 임포트하므로, 모듈
자체는 일반 python3로 그냥 import된다(exit_test_dummy_visible/
robot_within_person_radius는 pxr에도 의존하지 않는 순수 함수).

실행:
    pytest isaacpjt/Isaac_envo/test_exit_test_dummy.py
"""

from parking_v4_runner import (
    EXIT_TEST_ACTIVE_SEC,
    EXIT_TEST_CYCLE_SEC,
    PERSON_STOP_RADIUS_M,
    exit_test_dummy_visible,
    robot_within_person_radius,
)


def test_visible_at_cycle_start():
    assert exit_test_dummy_visible(0.0) is True


def test_visible_throughout_active_window():
    for t in (0.0, 2.5, 5.0, EXIT_TEST_ACTIVE_SEC - 0.01):
        assert exit_test_dummy_visible(t) is True


def test_hidden_right_after_active_window_ends():
    assert exit_test_dummy_visible(EXIT_TEST_ACTIVE_SEC) is False
    assert exit_test_dummy_visible(EXIT_TEST_ACTIVE_SEC + 1.0) is False


def test_hidden_at_end_of_cycle_just_before_next_spawn():
    assert exit_test_dummy_visible(EXIT_TEST_CYCLE_SEC - 0.01) is False


def test_robot_within_radius_true_when_close():
    assert robot_within_person_radius((0.0, 0.0), [(2.0, 0.0)], PERSON_STOP_RADIUS_M)


def test_robot_within_radius_false_when_far():
    assert not robot_within_person_radius(
        (0.0, 0.0), [(10.0, 10.0)], PERSON_STOP_RADIUS_M)


def test_robot_within_radius_false_when_no_person():
    assert not robot_within_person_radius((0.0, 0.0), [], PERSON_STOP_RADIUS_M)


def test_robot_within_radius_true_if_any_of_multiple_people_is_close():
    people = [(100.0, 100.0), (0.5, 0.5)]
    assert robot_within_person_radius((0.0, 0.0), people, PERSON_STOP_RADIUS_M)
