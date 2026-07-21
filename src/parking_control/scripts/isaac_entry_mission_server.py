#!/usr/bin/env python3
"""Isaac Sim 두 로봇으로 ENTRY 데모를 수행하는 ExecuteParkingTask 서버.

현재 검증 대상은 A1이다. 로봇 대기 위치 -> 가상 입고대기 H1 -> 롤러 암
전개(차량 픽업 모션) -> A1 대열 주행 -> 암 복귀 순서로 동작한다. 위치 추종은
ArUco 도입 전의 map/odom 기반 임시 제어이며, 향후 마지막 정렬 구간만 마커
제어기로 교체할 수 있도록 동작 단계를 분리했다.
"""

from __future__ import annotations

import math
import sys
import threading
import time
from dataclasses import dataclass

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from parking_robot_interfaces.action import ExecuteParkingTask
from parking_robot_interfaces.msg import TaskState
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import Float32


ROBOT_IDS = ("robot_1", "robot_2")
CONTROL_HZ = 20.0
MAX_LINEAR_MPS = 1.40  # 기존 임시 미션 0.35m/s의 4배
MAX_ANGULAR_RPS = 0.40
POSITION_TOLERANCE_M = 0.20
YAW_TOLERANCE_RAD = math.radians(8.0)
FORMATION_GAP_M = 2.90
WAYPOINT_TIMEOUT_SEC = 40.0


@dataclass(frozen=True)
class Pose2D:
    x: float
    y: float
    yaw: float = 0.0


def _clamp(value: float, limit: float) -> float:
    return max(-limit, min(limit, value))


def _wrap(angle: float) -> float:
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def _yaw_from_odom(message: Odometry) -> float:
    q = message.pose.pose.orientation
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


class IsaacEntryMissionServer(Node):

    RENDEZVOUS_ROUTES = {
        "robot_1": [
            Pose2D(-15.3, -4.0),
            Pose2D(-15.3, -2.35),
            Pose2D(-17.0, -2.35),
        ],
        "robot_2": [
            Pose2D(-15.3, 4.0),
            Pose2D(-15.3, 0.0),
            Pose2D(-14.1, -2.35),
        ],
    }
    H1_LEADER_ROUTE = [
        Pose2D(-18.1, -2.35),
        Pose2D(-20.0, -2.35),
        Pose2D(-21.85, -2.35),
    ]
    A1_CENTER_ROUTE = [
        Pose2D(-18.1, -2.35),
        Pose2D(-17.0, 0.0),
        Pose2D(-13.6, 0.0),
        Pose2D(-11.9, -2.5),
        Pose2D(-11.9, -5.0),
        Pose2D(-11.9, -7.8),
    ]

    def __init__(self):
        super().__init__("isaac_entry_mission_server")
        self._poses: dict[str, Pose2D] = {}
        self._arm_progress = {robot_id: 0.0 for robot_id in ROBOT_IDS}
        self._lock = threading.Lock()
        self._busy = False
        group = ReentrantCallbackGroup()

        self._cmd_publishers = {
            robot_id: self.create_publisher(Twist, f"/{robot_id}/cmd_vel", 10)
            for robot_id in ROBOT_IDS
        }
        self._arm_publishers = {
            robot_id: self.create_publisher(
                Float32, f"/{robot_id}/arm_command", 10
            )
            for robot_id in ROBOT_IDS
        }
        self._task_state_publisher = self.create_publisher(TaskState, "task_state", 10)
        self._subscriptions = []
        for robot_id in ROBOT_IDS:
            self._subscriptions.append(
                self.create_subscription(
                    Odometry,
                    f"/{robot_id}/odom",
                    lambda msg, rid=robot_id: self._on_odom(rid, msg),
                    10,
                    callback_group=group,
                )
            )
            self._subscriptions.append(
                self.create_subscription(
                    Float32,
                    f"/{robot_id}/arm_progress",
                    lambda msg, rid=robot_id: self._on_arm(rid, msg),
                    10,
                    callback_group=group,
                )
            )

        self._action_server = ActionServer(
            self,
            ExecuteParkingTask,
            "execute_parking_task",
            execute_callback=self._execute,
            goal_callback=self._goal,
            cancel_callback=self._cancel,
            callback_group=group,
        )
        self.get_logger().info(
            "Isaac ENTRY 서버 준비: H1 픽업 -> A1, 최고속도 1.40m/s"
        )

    def _on_odom(self, robot_id: str, message: Odometry):
        p = message.pose.pose.position
        with self._lock:
            self._poses[robot_id] = Pose2D(
                float(p.x), float(p.y), _yaw_from_odom(message)
            )

    def _on_arm(self, robot_id: str, message: Float32):
        with self._lock:
            self._arm_progress[robot_id] = float(message.data)

    def _goal(self, goal) -> GoalResponse:
        if self._busy:
            self.get_logger().warn("다른 ENTRY 임무가 진행 중이어서 goal 거부")
            return GoalResponse.REJECT
        if goal.request_type != "ENTRY" or goal.slot_id.upper() != "A1":
            self.get_logger().warn(
                f"현재 데모는 ENTRY/A1만 지원: {goal.request_type}/{goal.slot_id}"
            )
            return GoalResponse.REJECT
        self._busy = True
        return GoalResponse.ACCEPT

    def _cancel(self, _goal_handle) -> CancelResponse:
        return CancelResponse.ACCEPT

    def _pose(self, robot_id: str) -> Pose2D | None:
        with self._lock:
            return self._poses.get(robot_id)

    def _stop_all(self):
        for publisher in self._cmd_publishers.values():
            publisher.publish(Twist())

    def _command_pose(self, robot_id: str, target: Pose2D) -> bool:
        current = self._pose(robot_id)
        if current is None:
            return False
        dx, dy = target.x - current.x, target.y - current.y
        distance = math.hypot(dx, dy)
        yaw_error = _wrap(target.yaw - current.yaw)
        message = Twist()
        if distance > POSITION_TOLERANCE_M:
            body_x = math.cos(current.yaw) * dx + math.sin(current.yaw) * dy
            body_y = -math.sin(current.yaw) * dx + math.cos(current.yaw) * dy
            speed = min(MAX_LINEAR_MPS, 0.9 * distance)
            message.linear.x = speed * body_x / max(distance, 1e-6)
            message.linear.y = speed * body_y / max(distance, 1e-6)
        if abs(yaw_error) > YAW_TOLERANCE_RAD:
            # 이 HWIA 모델은 +wz의 실측 ROS yaw 부호가 반대다.
            message.angular.z = -_clamp(0.8 * yaw_error, MAX_ANGULAR_RPS)
        self._cmd_publishers[robot_id].publish(message)
        return (
            distance <= POSITION_TOLERANCE_M
            and abs(yaw_error) <= YAW_TOLERANCE_RAD
        )

    def _set_step(self, goal_handle, state: str, detail: str, progress: float):
        feedback = ExecuteParkingTask.Feedback()
        feedback.current_step = detail
        feedback.progress = float(progress)
        goal_handle.publish_feedback(feedback)
        goal = goal_handle.request
        for robot_id in ROBOT_IDS:
            self._task_state_publisher.publish(
                TaskState(
                    robot_id=robot_id,
                    task_id=goal.task_id,
                    state=state,
                    current_step=detail,
                )
            )
        self.get_logger().info(f"{state} {progress:.0%}: {detail}")

    def _interrupted(self, goal_handle) -> bool:
        if goal_handle.is_cancel_requested:
            self._stop_all()
            goal_handle.canceled()
            return True
        return False

    def _wait_for_odom(self, goal_handle, timeout=15.0) -> bool:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if self._interrupted(goal_handle):
                return False
            with self._lock:
                if all(robot_id in self._poses for robot_id in ROBOT_IDS):
                    return True
            time.sleep(0.05)
        return False

    def _drive_to(self, goal_handle, targets: dict[str, Pose2D]) -> bool:
        deadline = time.monotonic() + WAYPOINT_TIMEOUT_SEC
        while time.monotonic() < deadline:
            if self._interrupted(goal_handle):
                return False
            reached = [self._command_pose(rid, target) for rid, target in targets.items()]
            if all(reached):
                for rid in targets:
                    self._cmd_publishers[rid].publish(Twist())
                return True
            time.sleep(1.0 / CONTROL_HZ)
        self._stop_all()
        return False

    def _follow_single_route(self, goal_handle, robot_id, route) -> bool:
        other_id = "robot_2" if robot_id == "robot_1" else "robot_1"
        self._cmd_publishers[other_id].publish(Twist())
        return all(
            self._drive_to(goal_handle, {robot_id: target}) for target in route
        )

    def _follow_h1_formation(self, goal_handle) -> bool:
        for leader_target in self.H1_LEADER_ROUTE:
            leader = self._pose("robot_1")
            if leader is None:
                return False
            targets = {
                "robot_1": leader_target,
                "robot_2": Pose2D(leader.x + FORMATION_GAP_M, leader.y),
            }
            if not self._drive_to(goal_handle, targets):
                return False
        final = self._pose("robot_1")
        if final is None:
            return False
        return self._drive_to(
            goal_handle,
            {
                "robot_1": Pose2D(-21.85, -2.35),
                "robot_2": Pose2D(-21.85 + FORMATION_GAP_M, -2.35),
            },
        )

    def _follow_center_route(self, goal_handle, route) -> bool:
        half_gap = FORMATION_GAP_M * 0.5
        for center in route:
            targets = {
                "robot_1": Pose2D(center.x - half_gap, center.y, center.yaw),
                "robot_2": Pose2D(center.x + half_gap, center.y, center.yaw),
            }
            if not self._drive_to(goal_handle, targets):
                return False
        return True

    def _move_arms(self, goal_handle, target: float, timeout=8.0) -> bool:
        message = Float32(data=float(target))
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if self._interrupted(goal_handle):
                return False
            for publisher in self._arm_publishers.values():
                publisher.publish(message)
            with self._lock:
                if all(
                    abs(self._arm_progress[rid] - target) <= 0.04
                    for rid in ROBOT_IDS
                ):
                    return True
            time.sleep(0.1)
        return False

    def _execute(self, goal_handle):
        result = ExecuteParkingTask.Result()
        try:
            self._set_step(goal_handle, "SEARCHING", "두 로봇 odom 연결 확인", 0.02)
            if not self._wait_for_odom(goal_handle):
                raise RuntimeError("robot_1/robot_2 odom 수신 시간 초과")

            self._set_step(goal_handle, "APPROACHING", "robot_1 입고대기 H1 합류", 0.10)
            if not self._follow_single_route(
                goal_handle, "robot_1", self.RENDEZVOUS_ROUTES["robot_1"]
            ):
                raise RuntimeError("robot_1 합류 경로 시간 초과")

            self._set_step(goal_handle, "APPROACHING", "robot_2 입고대기 H1 합류", 0.24)
            if not self._follow_single_route(
                goal_handle, "robot_2", self.RENDEZVOUS_ROUTES["robot_2"]
            ):
                raise RuntimeError("robot_2 합류 경로 시간 초과")

            self._set_step(goal_handle, "APPROACHING", "2.9m 대열로 H1 차량 하부 접근", 0.36)
            if not self._follow_h1_formation(goal_handle):
                raise RuntimeError("H1 대열 접근 시간 초과")

            self._stop_all()
            self._set_step(goal_handle, "PICKED_UP", "롤러 암 전개: 차량 픽업 모션", 0.48)
            if not self._move_arms(goal_handle, 1.0):
                raise RuntimeError("롤러 암 전개 시간 초과")

            self._set_step(goal_handle, "MOVING", "차량 운반 대열로 A1 이동", 0.58)
            if not self._follow_center_route(goal_handle, self.A1_CENTER_ROUTE):
                raise RuntimeError("A1 이동 경로 시간 초과")

            self._stop_all()
            self._set_step(goal_handle, "ARRIVED", "A1 도착, 롤러 암 복귀/하차", 0.92)
            if not self._move_arms(goal_handle, 0.0):
                raise RuntimeError("롤러 암 복귀 시간 초과")

            self._set_step(goal_handle, "PARKED", "A1 입고 데모 완료", 1.0)
            goal_handle.succeed()
            result.success = True
            result.message = "H1 픽업 모션 및 A1 대열 이동 완료"
            return result
        except RuntimeError as exc:
            self._stop_all()
            if not goal_handle.is_cancel_requested:
                goal_handle.abort()
            result.success = False
            result.message = str(exc)
            self.get_logger().error(result.message)
            return result
        finally:
            self._busy = False

    def destroy_node(self):
        if rclpy.ok():
            self._stop_all()
        self._action_server.destroy()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = IsaacEntryMissionServer()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main(sys.argv)
