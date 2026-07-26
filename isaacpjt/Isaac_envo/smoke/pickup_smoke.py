#!/usr/bin/env python3
"""R5b pickup_orchestrator_node 스모크 (순수 ROS2, 시스템 rclpy).

`bringup_pickup_e2e.sh`(신규, R5a `run_ingress_smoke.sh` 관례를 다중 노드로
확장)로 브리지 + marker_localizer(x2) + pose_controller_node(x4, 로봇별
odom/fused 두 인스턴스) + axle_detector_node(x2) + ingress_node(x2) +
lift_action_server(x2) + pickup_orchestrator_node 가 모두 떠 있다고 가정한다.
`ExecuteParkingTask` 액션에 leader/follower 목표를 보내고 feedback/result 를
리포트한다.

정지좌표/트럭 상승의 **GT 검증**은 이 스크립트가 하지 않는다(이 노드/스크립트는
GT 를 전혀 모른다 -- 브리지의 GT 원칙과 동일, taskR4/taskR5a 보고서와 동일
관례). 브리지 콘솔의 ``BRIDGE_ALIVE ... gt=[...] truck_rise=...`` 로그(2초
간격 하트비트)를 별도로 대조해야 한다.

ros2 CLI 가 이 머신에서 세그폴트하므로(taskB0 기록) 이 스크립트로 대신한다.
실행: pickup_smoke.py --leader=entry_lead --follower=entry_follow [--timeout=SEC]
"""
import sys
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from parking_robot_interfaces.action import ExecuteParkingTask


class PickupSmoke(Node):
    def __init__(self):
        super().__init__("pickup_smoke")
        self.client = ActionClient(self, ExecuteParkingTask, "execute_pickup_choreography")
        self._last_fb_log = 0.0
        self._t0 = None

    def send(self, leader, follower, task_id, timeout):
        self.get_logger().info(
            f"execute_pickup_choreography: leader={leader} follower={follower} "
            "목표 전송 대기 중 서버 탐색…")
        if not self.client.wait_for_server(timeout_sec=15.0):
            self.get_logger().error("execute_pickup_choreography 액션 서버를 찾지 못했습니다")
            return None
        goal = ExecuteParkingTask.Goal()
        goal.task_id = task_id
        goal.request_type = "ENTRY"
        goal.vehicle_id = "HANDOFF_VEHICLE"
        goal.slot_id = ""
        goal.leader_robot_id = leader
        goal.follower_robot_id = follower

        self._t0 = time.monotonic()
        send_future = self.client.send_goal_async(goal, feedback_callback=self._on_feedback)
        rclpy.spin_until_future_complete(self, send_future, timeout_sec=15.0)
        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error("execute_pickup_choreography: 목표 거부됨")
            return None
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=timeout)
        elapsed = time.monotonic() - self._t0
        if not result_future.done():
            self.get_logger().error(f"execute_pickup_choreography: 결과 타임아웃(wall={elapsed:.1f}s)")
            return None, elapsed
        return result_future.result().result, elapsed

    def _on_feedback(self, fb_msg):
        now = time.monotonic()
        if now - self._last_fb_log < 1.0:
            return
        self._last_fb_log = now
        fb = fb_msg.feedback
        elapsed = now - self._t0 if self._t0 else 0.0
        print(f"PICKUP_FEEDBACK t={elapsed:.1f}s step={fb.current_step} "
              f"progress={fb.progress:.2f}", flush=True)


def main():
    leader = "entry_lead"
    follower = "entry_follow"
    task_id = "R5B_E2E"
    timeout = 1800.0
    for a in sys.argv[1:]:
        if a.startswith("--leader="):
            leader = a.split("=", 1)[1]
        elif a.startswith("--follower="):
            follower = a.split("=", 1)[1]
        elif a.startswith("--task-id="):
            task_id = a.split("=", 1)[1]
        elif a.startswith("--timeout="):
            timeout = float(a.split("=", 1)[1])

    rclpy.init()
    node = PickupSmoke()
    try:
        outcome = node.send(leader, follower, task_id, timeout)
        if outcome is None:
            print("=== PICKUP_SMOKE_RESULT success=False stop_reason=client_error ===", flush=True)
            return
        result, elapsed = outcome
        if result is None:
            print(f"=== PICKUP_SMOKE_RESULT success=False stop_reason=timeout wall_s={elapsed:.1f} ===",
                  flush=True)
        else:
            print(f"=== PICKUP_SMOKE_RESULT success={result.success} message={result.message!r} "
                  f"wall_s={elapsed:.1f} ===", flush=True)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
