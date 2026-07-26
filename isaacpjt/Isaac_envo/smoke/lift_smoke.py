#!/usr/bin/env python3
"""R4 리프트 스모크 (순수 ROS2, 시스템 rclpy).

parking_v4_runner.py --bridge 가 떠 있고(BRIDGE_ALIVE 로그에 truck_y/truck_rise
가 찍힘 — 이 스크립트가 직접 트럭 GT 를 볼 방법은 없다, 브리지 콘솔을 같이
관찰해서 대조한다), lift_action_server(run_lift_action_server.sh, robot_id=
<ROBOT>)가 떠 있다고 가정한다. ControlLift 액션에 UP 목표를 보내고 결과를
찍은 뒤, 잠깐 대기하고 DOWN 목표를 보낸다.

ros2 CLI 가 이 머신에서 세그폴트하므로(taskB0 기록) 이 스크립트로 대신한다.
실행: run_lift_smoke.sh <ROBOT> [--wait-between=SEC]
"""
import sys
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from parking_robot_interfaces.action import ControlLift


class LiftSmoke(Node):
    def __init__(self, robot):
        super().__init__("lift_smoke")
        self.client = ActionClient(self, ControlLift, f"/robot_{robot}/control_lift")

    def send(self, command):
        self.get_logger().info(f"control_lift: {command} 목표 전송 대기 중 서버 탐색…")
        if not self.client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error("control_lift 액션 서버를 찾지 못했습니다")
            return None
        goal = ControlLift.Goal()
        goal.command = command
        send_future = self.client.send_goal_async(
            goal, feedback_callback=self._on_feedback)
        rclpy.spin_until_future_complete(self, send_future, timeout_sec=10.0)
        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error(f"control_lift: {command} 목표 거부됨")
            return None
        result_future = goal_handle.get_result_async()
        t0 = time.monotonic()
        while not result_future.done() and time.monotonic() - t0 < 30.0:
            rclpy.spin_once(self, timeout_sec=0.1)
        if not result_future.done():
            self.get_logger().error(f"control_lift: {command} 결과 타임아웃")
            return None
        result = result_future.result().result
        self.get_logger().info(
            f"=== LIFT_SMOKE_RESULT command={command} success={result.success} "
            f"support_state={result.support_state} ===")
        return result

    def _on_feedback(self, fb):
        self.get_logger().info(f"control_lift feedback: status={fb.feedback.status}")


def main():
    robot = "entry_lead"
    wait_between = 3.0
    for a in sys.argv[1:]:
        if a.startswith("--robot="):
            robot = a.split("=", 1)[1]
        elif a.startswith("--wait-between="):
            wait_between = float(a.split("=", 1)[1])

    rclpy.init()
    node = LiftSmoke(robot)
    try:
        node.send("UP")
        time.sleep(wait_between)
        node.send("DOWN")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
