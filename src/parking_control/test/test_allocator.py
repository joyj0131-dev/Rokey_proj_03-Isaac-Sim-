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


# ---- 헝가리안 vs 최근접: 교차배정이 총거리를 줄이는 반례 ----
#
# 로봇 rA(0,0), rB(100,0) / 작업 T1 목표(10,0), T2 목표(0,0)
#   최근접(탐욕): T1부터 처리 → rA(비용 10) 선점 → T2는 rB(비용 100) → 총 110
#   헝가리안:     rA→T2(0), rB→T1(90)                             → 총  90
# 탐욕이 앞 작업에서 "T2에게 꼭 필요한 로봇"을 뺏는 상황을 재현한다.

CROSS_TARGETS = {"T1": (10.0, 0.0), "T2": (0.0, 0.0)}


def cross_cost(robot, task):
    tx, ty = CROSS_TARGETS[task.task_id]
    return math.hypot(robot.x - tx, robot.y - ty)


def _cross_fixture():
    robots = [RobotState("rA", 0.0, 0.0), RobotState("rB", 100.0, 0.0)]
    tasks = [TaskRequest("T1", "entrance"), TaskRequest("T2", "entrance")]
    return robots, tasks


def test_nearest_is_suboptimal_on_cross_case():
    robots, tasks = _cross_fixture()
    result = NearestAllocator().assign(robots, tasks, cross_cost)
    assert sum(a.cost for a in result) == 110.0


def test_hungarian_finds_global_optimum_on_cross_case():
    from parking_control.core.hungarian import HungarianAllocator
    robots, tasks = _cross_fixture()
    result = HungarianAllocator().assign(robots, tasks, cross_cost)
    assert sum(a.cost for a in result) == 90.0
    pairs = {(a.robot_id, a.task_id) for a in result}
    assert pairs == {("rA", "T2"), ("rB", "T1")}


def test_hungarian_skips_unreachable():
    from parking_control.core.hungarian import HungarianAllocator
    robots, tasks = _cross_fixture()
    result = HungarianAllocator().assign(
        robots, tasks,
        lambda r, t: None if r.robot_id == "rB" else cross_cost(r, t))
    # rB는 어디에도 못 가므로 rA 하나만 배정된다
    assert len(result) == 1
    assert result[0].robot_id == "rA"
