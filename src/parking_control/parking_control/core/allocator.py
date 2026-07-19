"""로봇-작업 할당 전략. 순수 Python (ROS import 금지).

Allocator 인터페이스를 고정해 두고 구현만 갈아끼운다:
  - NearestAllocator: 작업별로 가장 비용 낮은 로봇 (MVP)
  - HungarianAllocator: scipy 헝가리안 전역 최적 (7단계에서 추가)
dispatcher는 Allocator 타입만 알고 있으므로 교체는 파라미터 한 줄이다.
"""

from abc import ABC, abstractmethod
from dataclasses import dataclass


@dataclass(frozen=True)
class RobotState:
    robot_id: str
    x: float
    y: float


@dataclass(frozen=True)
class TaskRequest:
    task_id: str
    target_node: str   # 비용 계산 기준 노드 (예: 'entrance')


@dataclass(frozen=True)
class Assignment:
    robot_id: str
    task_id: str
    cost: float


class Allocator(ABC):

    @abstractmethod
    def assign(self, robots, tasks, cost_fn) -> list:
        """cost_fn(robot, task) -> float | None (도달 불가면 None).

        반환: Assignment 목록. 로봇/작업은 각각 최대 1회만 배정된다.
        """


class NearestAllocator(Allocator):
    """작업 순서대로, 남은 로봇 중 비용 최소를 탐욕적으로 배정."""

    def assign(self, robots, tasks, cost_fn) -> list:
        assignments = []
        remaining = list(robots)
        for task in tasks:
            best = None
            for robot in remaining:
                cost = cost_fn(robot, task)
                if cost is not None and (best is None or cost < best[1]):
                    best = (robot, cost)
            if best is not None:
                robot, cost = best
                assignments.append(Assignment(robot.robot_id, task.task_id, cost))
                remaining.remove(robot)
        return assignments


def make_allocator(name: str) -> Allocator:
    """ROS 파라미터 문자열 → 구현 선택."""
    if name == "nearest":
        return NearestAllocator()
    if name == "hungarian":
        from .hungarian import HungarianAllocator
        return HungarianAllocator()
    raise ValueError(f"알 수 없는 allocator: {name}")
