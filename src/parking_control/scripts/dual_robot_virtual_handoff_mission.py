#!/usr/bin/env python3
"""임시 관제 역할로 두 HWIA 로봇의 합류/대열 주행을 명령한다.

이 파일의 좌표 추종은 통신과 동작 흐름 검증을 위한 임시 구현이다. 향후 실제
주행에서는 이 부분을 ArUco 마커 인식 기반 제어기로 교체한다.
"""

from __future__ import annotations

import math
import sys
import time
from dataclasses import dataclass

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import String


CONTROL_HZ = 20.0
MAX_LINEAR = 0.35
MAX_ANGULAR = 0.60
POSITION_TOLERANCE = 0.18
YAW_TOLERANCE = math.radians(7.0)
FORMATION_GAP_M = 2.9
MISSION_TIMEOUT_SEC = 180.0


@dataclass
class Pose2D:
    x: float
    y: float
    yaw: float


def clamp(value: float, limit: float) -> float:
    return max(-limit, min(limit, value))


def wrap(angle: float) -> float:
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def yaw_from_odom(message: Odometry) -> float:
    q = message.pose.pose.orientation
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


class VirtualHandoffMission(Node):

    ROUTES = {
        "robot_1": [
            Pose2D(-15.3, -4.0, 0.0),
            Pose2D(-15.3, -2.35, 0.0),
            Pose2D(-17.0, -2.35, 0.0),
        ],
        "robot_2": [
            Pose2D(-15.3, 4.0, 0.0),
            Pose2D(-15.3, 0.0, 0.0),
            Pose2D(-14.1, -2.35, 0.0),
        ],
    }
    LEADER_ROUTE = [
        Pose2D(-18.1, -2.35, 0.0),
        Pose2D(-20.0, -2.35, 0.0),
        Pose2D(-21.85, -2.35, 0.0),
    ]

    def __init__(self):
        super().__init__("temporary_parking_supervisor")
        self.poses: dict[str, Pose2D] = {}
        self._cmd_publishers = {
            robot_id: self.create_publisher(Twist, f"/{robot_id}/cmd_vel", 10)
            for robot_id in ("robot_1", "robot_2")
        }
        self._odom_subscriptions = [
            self.create_subscription(
                Odometry,
                f"/{robot_id}/odom",
                lambda message, rid=robot_id: self._on_odom(rid, message),
                10,
            )
            for robot_id in ("robot_1", "robot_2")
        ]
        self._status_publisher = self.create_publisher(
            String, "/parking_demo/mission_status", 10
        )
        self.state = "WAIT_ODOM"
        self.route_index = 0
        self.state_started = time.monotonic()
        self.mission_started = self.state_started
        self.last_log = 0.0
        self.hold_started = None
        self.max_gap_error = 0.0
        self.max_cross_track_error = 0.0
        self.done = False
        self.create_timer(1.0 / CONTROL_HZ, self._tick)
        self._status("WAIT_ODOM", "Isaac Sim의 robot_1/robot_2 odom 대기")

    def _on_odom(self, robot_id: str, message: Odometry):
        p = message.pose.pose.position
        self.poses[robot_id] = Pose2D(float(p.x), float(p.y), yaw_from_odom(message))

    def _status(self, state: str, detail: str):
        self.state = state
        self.state_started = time.monotonic()
        message = String()
        message.data = f"{state}: {detail}"
        self._status_publisher.publish(message)
        self.get_logger().info(message.data)

    def _publish_stop(self, robot_id: str):
        self._cmd_publishers[robot_id].publish(Twist())

    def _stop_all(self):
        self._publish_stop("robot_1")
        self._publish_stop("robot_2")

    def _command_pose(self, robot_id: str, target: Pose2D) -> bool:
        current = self.poses[robot_id]
        dx = target.x - current.x
        dy = target.y - current.y
        distance = math.hypot(dx, dy)
        yaw_error = wrap(target.yaw - current.yaw)

        command = Twist()
        if distance > POSITION_TOLERANCE and abs(yaw_error) <= math.radians(25.0):
            # map 좌표 오차를 로봇 body 좌표의 전진/좌측 속도로 변환한다.
            body_x = math.cos(current.yaw) * dx + math.sin(current.yaw) * dy
            body_y = -math.sin(current.yaw) * dx + math.cos(current.yaw) * dy
            scale = min(MAX_LINEAR, 0.8 * distance) / max(distance, 1e-6)
            command.linear.x = body_x * scale
            command.linear.y = body_y * scale
            # HWIA 휠 조합의 +wz 물리 회전은 ROS map yaw와 반대 부호로 실측됐다.
            command.angular.z = -clamp(1.0 * yaw_error, MAX_ANGULAR)
        elif abs(yaw_error) > YAW_TOLERANCE:
            # 방향 오차가 크면 병진을 멈추고 먼저 자세부터 복구한다.
            command.angular.z = -clamp(1.0 * yaw_error, MAX_ANGULAR)
        self._cmd_publishers[robot_id].publish(command)
        return distance <= POSITION_TOLERANCE and abs(yaw_error) <= YAW_TOLERANCE

    def _follow_route(self, robot_id: str, route: list[Pose2D]) -> bool:
        if self._command_pose(robot_id, route[self.route_index]):
            self.route_index += 1
            if self.route_index >= len(route):
                self._publish_stop(robot_id)
                return True
        return False

    def _formation_command(self) -> bool:
        leader = self.poses["robot_1"]
        follower = self.poses["robot_2"]
        target = self.LEADER_ROUTE[self.route_index]
        leader_at_waypoint = self._command_pose("robot_1", target)

        # 두 로봇은 초기 방향(yaw=0)을 유지하고 서쪽으로 후진한다. 따라서 진행
        # 방향 기준 뒤쪽 follower 목표는 leader보다 world +X로 2.9 m 지점이다.
        # 제자리 회전의 기구 캘리브레이션은 ArUco 적용 전에 별도로 수행한다.
        follower_target = Pose2D(
            leader.x + FORMATION_GAP_M,
            leader.y,
            0.0,
        )
        follower_ready = self._command_pose("robot_2", follower_target)
        gap_error = math.hypot(
            follower_target.x - follower.x, follower_target.y - follower.y
        )
        cross_track = abs(leader.y + 2.35)
        self.max_gap_error = max(self.max_gap_error, gap_error)
        self.max_cross_track_error = max(self.max_cross_track_error, cross_track)

        if leader_at_waypoint and self.route_index < len(self.LEADER_ROUTE) - 1:
            self.route_index += 1
        final = self.LEADER_ROUTE[-1]
        leader_final = math.hypot(final.x - leader.x, final.y - leader.y) <= POSITION_TOLERANCE
        return leader_final and follower_ready and gap_error <= 0.25

    def _log_progress(self):
        now = time.monotonic()
        if now - self.last_log < 1.0 or len(self.poses) < 2:
            return
        r1, r2 = self.poses["robot_1"], self.poses["robot_2"]
        distance = math.hypot(r1.x - r2.x, r1.y - r2.y)
        self.get_logger().info(
            f"{self.state} | r1=({r1.x:.2f},{r1.y:.2f}) "
            f"r2=({r2.x:.2f},{r2.y:.2f}) distance={distance:.2f}m"
        )
        self.last_log = now

    def _tick(self):
        if self.done:
            return
        now = time.monotonic()
        if now - self.mission_started > MISSION_TIMEOUT_SEC:
            self._stop_all()
            self._status("FAILED", "180초 임무 시간 초과")
            self.done = True
            return
        if len(self.poses) < 2:
            return

        if self.state == "WAIT_ODOM":
            self.route_index = 0
            self._status("RENDEZVOUS_R1", "robot_1을 합류 위치로 이동")
        elif self.state == "RENDEZVOUS_R1":
            self._publish_stop("robot_2")
            if self._follow_route("robot_1", self.ROUTES["robot_1"]):
                self.route_index = 0
                self._status("RENDEZVOUS_R2", "robot_2를 합류 위치로 이동")
        elif self.state == "RENDEZVOUS_R2":
            self._publish_stop("robot_1")
            if self._follow_route("robot_2", self.ROUTES["robot_2"]):
                self.route_index = 0
                self._status(
                    "FORMATION_MOVE",
                    "초기 방향 유지, 2.9m 후진 대열로 가상 H1 위치에 접근",
                )
        elif self.state == "FORMATION_MOVE":
            if self._formation_command():
                self._stop_all()
                self.hold_started = now
                self._status("ARRIVED", "가상 차량 하부 위치 도착, 동작 대기")
        elif self.state == "ARRIVED":
            self._stop_all()
            if now - self.hold_started >= 5.0:
                detail = (
                    f"완료; 최대 대열 오차={self.max_gap_error:.3f}m, "
                    f"최대 경로 횡오차={self.max_cross_track_error:.3f}m"
                )
                self._status("DONE", detail)
                self.done = True
        self._log_progress()

    def destroy_node(self):
        if rclpy.ok():
            self._stop_all()
        super().destroy_node()


def main():
    rclpy.init(args=sys.argv)
    node = VirtualHandoffMission()
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
