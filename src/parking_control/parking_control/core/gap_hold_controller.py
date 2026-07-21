"""로봇 2대(leader/follower) 간격 유지 제어. 순수 Python (ROS import 금지).

leader가 이동하면 follower는 "leader의 로컬 좌표계 기준 뒤로 gap_m만큼
떨어진 지점"을 목표로 추종한다. 전진/후진 어느 방향이든 이 식은 그대로
적용된다 — leader가 방향을 반대로 틀면 목표점도 같이 뒤집히므로,
"누가 진행 방향 기준 앞이냐"에 따라 leader/follower 역할을 바꿀 필요가
없다(2026-07-21 논의로 확정된 설계). 역할은 항상 고정, 물리적 앞/뒤만
바뀐다.
"""

import math
from dataclasses import dataclass


@dataclass(frozen=True)
class Pose2D:
    x: float
    y: float
    yaw: float   # rad, world frame


@dataclass(frozen=True)
class GapHoldCommand:
    linear_x: float
    angular_z: float


def follower_target(leader: Pose2D, gap_m: float) -> Pose2D:
    """leader 로컬 -X(뒤) 방향으로 gap_m 떨어진 지점의 world 좌표."""
    return Pose2D(
        x=leader.x - gap_m * math.cos(leader.yaw),
        y=leader.y - gap_m * math.sin(leader.yaw),
        yaw=leader.yaw,
    )


def yaw_from_quaternion(x, y, z, w):
    """평면(2D) 이동만 다루므로 z축 회전(yaw)만 뽑는다."""
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def _wrap_angle(angle):
    """[-pi, pi] 범위로 정규화."""
    return (angle + math.pi) % (2 * math.pi) - math.pi


def _clamp(value, low, high):
    return max(low, min(high, value))


class GapHoldController:
    """follower가 매 tick마다 (자기 pose, leader pose)를 받아 cmd_vel 계산.

    단순 비례 제어: 목표까지 직선거리로 전진 속도, 목표 방향과의 각도
    오차로 회전 속도를 정한다. 목표가 거의 옆/뒤(heading_error가 90도
    초과)일 때 전진을 0으로 죽이는 건, 후진 지원용 별도 로직 없이도
    제자리 회전으로 먼저 방향을 맞추게 하기 위함이다(단순화; 실제
    로봇에서 후진이 필요하면 추후 보완).
    """

    def __init__(self, gap_m, k_linear=1.0, k_angular=2.0,
                max_linear=0.5, max_angular=1.0):
        self.gap_m = gap_m
        self.k_linear = k_linear
        self.k_angular = k_angular
        self.max_linear = max_linear
        self.max_angular = max_angular

    def compute(self, follower: Pose2D, leader: Pose2D) -> GapHoldCommand:
        target = follower_target(leader, self.gap_m)
        dx = target.x - follower.x
        dy = target.y - follower.y
        dist = math.hypot(dx, dy)
        heading_error = _wrap_angle(math.atan2(dy, dx) - follower.yaw)

        linear = _clamp(self.k_linear * dist, -self.max_linear, self.max_linear)
        if abs(heading_error) > math.pi / 2:
            linear = 0.0
        angular = _clamp(
            self.k_angular * heading_error, -self.max_angular, self.max_angular)
        return GapHoldCommand(linear_x=linear, angular_z=angular)
