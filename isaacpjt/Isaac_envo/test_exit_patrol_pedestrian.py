#!/usr/bin/env python3
"""출차존 순찰 보행자 위상 계산 + 로봇 근접 판정 단위 테스트 (Isaac/ROS 불필요).

parking_v4_runner.py는 SimulationApp을 함수 안에서만 임포트하므로, 모듈
자체는 일반 python3로 그냥 import된다(exit_patrol_position/
robot_within_person_radius는 pxr에도 의존하지 않는 순수 함수).

실행:
    pytest isaacpjt/Isaac_envo/test_exit_patrol_pedestrian.py
"""

from parking_v4_runner import (
    EXIT_PATROL_ACTIVE_SEC,
    EXIT_PATROL_CYCLE_SEC,
    EXIT_PATROL_Z_FAR,
    EXIT_PATROL_Z_NEAR,
    PERSON_STOP_RADIUS_M,
    exit_patrol_position,
    robot_within_person_radius,
)


def test_hidden_outside_active_window():
    visible, z = exit_patrol_position(EXIT_PATROL_ACTIVE_SEC + 1.0)
    assert visible is False
    assert z == EXIT_PATROL_Z_NEAR
    # 사이클 끝자락(다음 60초 스폰 직전)도 여전히 숨겨져 있어야 한다.
    visible, _ = exit_patrol_position(EXIT_PATROL_CYCLE_SEC - 1.0)
    assert visible is False


def test_visible_and_starts_at_near_endpoint_when_spawned():
    visible, z = exit_patrol_position(0.0)
    assert visible is True
    assert z == EXIT_PATROL_Z_NEAR


def test_walks_back_and_forth_between_endpoints_during_active_window():
    lo, hi = sorted((EXIT_PATROL_Z_NEAR, EXIT_PATROL_Z_FAR))
    samples = [
        exit_patrol_position(t)
        for t in (i * 0.25 for i in range(int(EXIT_PATROL_ACTIVE_SEC / 0.25)))
    ]
    assert all(visible for visible, _ in samples)
    zs = [z for _, z in samples]
    assert all(lo - 1e-9 <= z <= hi + 1e-9 for z in zs)
    # 왕복이라면 갔다가 되돌아오는 지점이 있어야 한다(단조 증가/감소만은 아님).
    assert max(zs) > min(zs)
    increasing = any(b > a for a, b in zip(zs, zs[1:]))
    decreasing = any(b < a for a, b in zip(zs, zs[1:]))
    assert increasing and decreasing


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
