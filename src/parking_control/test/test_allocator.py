"""할당 전략 단위 테스트. 그래프 없이 좌표 직선거리를 cost_fn으로 사용."""

import math

from parking_control.core.allocator import (
    NearestAllocator, RobotState, TaskRequest,
)

TARGETS = {"T1": (0.0, 0.0), "T2": (10.0, 0.0)}


def dist_cost(robot, task):
    tx, ty = TARGETS[task.task_id]
    return math.hypot(robot.x - tx, robot.y - ty)


def test_nearest_picks_closest_robot():
    robots = [RobotState("r_far", 8.0, 0.0), RobotState("r_near", 1.0, 0.0)]
    tasks = [TaskRequest("T1", "entrance")]
    result = NearestAllocator().assign(robots, tasks, dist_cost)
    assert len(result) == 1
    assert result[0].robot_id == "r_near"
    assert result[0].cost == 1.0


def test_no_double_assignment():
    robots = [RobotState("r1", 1.0, 0.0)]
    tasks = [TaskRequest("T1", "entrance"), TaskRequest("T2", "entrance")]
    result = NearestAllocator().assign(robots, tasks, dist_cost)
    assert len(result) == 1  # 로봇 1대는 작업 1개만


def test_unreachable_robot_skipped():
    robots = [RobotState("r1", 1.0, 0.0)]
    tasks = [TaskRequest("T1", "entrance")]
    result = NearestAllocator().assign(robots, tasks, lambda r, t: None)
    assert result == []
