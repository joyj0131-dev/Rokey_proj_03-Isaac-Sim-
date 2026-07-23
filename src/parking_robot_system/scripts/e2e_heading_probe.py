#!/usr/bin/env python3
"""A2 입차 E2E를 요청하고 운반 중 차량/편대 heading 오차를 측정한다."""

import json
import math
import sys
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node

from parking_robot_interfaces.msg import TaskState
from parking_robot_interfaces.srv import ParkInSlot


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def formation_heading(poses):
    """front->rear 축을 로봇 odom과 같은 yaw 규약(0=+X, +pi/2=-Z)으로 표현."""
    rear = poses.get("rear")
    front = poses.get("front")
    if rear is None or front is None:
        return None
    dx = rear[0] - front[0]
    dz = rear[1] - front[1]
    return math.atan2(-dz, dx)


class HeadingProbe(Node):
    def __init__(self):
        super().__init__("e2e_heading_probe")
        self.vehicle_yaw = None
        self.poses = {"rear": None, "front": None}
        self.state = None
        self.terminal = None
        self.started = False
        self.finished = False
        self.vehicle_ref = None
        self.formation_ref = None
        self.vehicle_errors = []
        self.formation_errors = []
        self.baseline_ref = None
        self.baseline_deltas = []

        self.create_subscription(PoseStamped, "/vehicle/pose", self._vehicle, 20)
        for rid in self.poses:
            self.create_subscription(
                Odometry, f"/robot_{rid}/odom",
                lambda msg, key=rid: self._odom(key, msg), 20)
        self.create_subscription(TaskState, "/task_state", self._task, 20)
        self.client = self.create_client(ParkInSlot, "/park_in_slot")

    def _vehicle(self, msg):
        self.vehicle_yaw = yaw_of(msg.pose.orientation)
        self._sample()

    def _odom(self, rid, msg):
        p = msg.pose.pose.position
        self.poses[rid] = (float(p.x), float(p.z))
        self._sample()

    def _task(self, msg):
        self.state = msg.state
        if msg.state == "MOVING" and not self.started:
            self.started = True
            self.vehicle_ref = self.vehicle_yaw
            self.formation_ref = formation_heading(self.poses)
            self.baseline_ref = self._baseline()
            self.get_logger().info("MOVING 시작: heading 기준값 고정")
        if msg.state == "RETURNING":
            # 차량 안착·팔 접힘 이후 로봇들이 서로 다른 도크로 가기 전에 측정을 동결한다.
            self.finished = True
        if msg.state in ("DONE", "FAILED"):
            self.terminal = msg.state
        self._sample()

    def _baseline(self):
        rear, front = self.poses["rear"], self.poses["front"]
        if rear is None or front is None:
            return None
        return math.hypot(rear[0] - front[0], rear[1] - front[1])

    def _sample(self):
        if not self.started or self.finished:
            return
        if self.vehicle_yaw is not None and self.vehicle_ref is not None:
            self.vehicle_errors.append(wrap(self.vehicle_yaw - self.vehicle_ref))
        heading = formation_heading(self.poses)
        if heading is not None and self.formation_ref is not None:
            self.formation_errors.append(wrap(heading - self.formation_ref))
        baseline = self._baseline()
        if baseline is not None and self.baseline_ref is not None:
            self.baseline_deltas.append(baseline - self.baseline_ref)

    def result(self):
        deg = lambda value: math.degrees(value) if value is not None else None
        final_vehicle = self.vehicle_errors[-1] if self.vehicle_errors else None
        final_formation = self.formation_errors[-1] if self.formation_errors else None
        return {
            "terminal_state": self.terminal,
            "vehicle_final_error_deg": deg(final_vehicle),
            "vehicle_max_abs_error_deg": (
                max(abs(deg(v)) for v in self.vehicle_errors)
                if self.vehicle_errors else None),
            "formation_final_error_deg": deg(final_formation),
            "formation_max_abs_error_deg": (
                max(abs(deg(v)) for v in self.formation_errors)
                if self.formation_errors else None),
            "baseline_max_delta_m": (
                max(abs(v) for v in self.baseline_deltas)
                if self.baseline_deltas else None),
            "samples": len(self.vehicle_errors),
        }


def main():
    rclpy.init()
    node = HeadingProbe()
    try:
        ready_deadline = time.monotonic() + 120.0
        while time.monotonic() < ready_deadline:
            rclpy.spin_once(node, timeout_sec=0.2)
            if (node.client.service_is_ready() and node.vehicle_yaw is not None
                    and all(node.poses.values())):
                break
        else:
            print("HEADING_PROBE=" + json.dumps({"error": "ROS data/service timeout"}))
            return 2

        future = node.client.call_async(ParkInSlot.Request(slot_id="A2"))
        while rclpy.ok() and not future.done():
            rclpy.spin_once(node, timeout_sec=0.2)
        response = future.result()
        if response is None or not response.accepted:
            print("HEADING_PROBE=" + json.dumps({
                "error": response.message if response else "service call failed"}))
            return 3

        deadline = time.monotonic() + 1500.0
        while rclpy.ok() and node.terminal is None and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.2)

        result = node.result()
        if node.terminal is None:
            result["error"] = "E2E timeout"
        print("HEADING_PROBE=" + json.dumps(result, sort_keys=True), flush=True)
        if node.terminal != "DONE":
            return 4
        return 0
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
