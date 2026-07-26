#!/usr/bin/env python3
"""R5a ingress_node 스모크 (순수 ROS2, 시스템 rclpy).

parking_v4_runner.py --bridge --bridge-cameras=<ROBOT> --bridge-depth-pose=<ROBOT>
(트럭 진입선 배치) + axle_detector_node(robot_id=<ROBOT>, run_axle_detector_node.sh)
+ ingress_node(robot_id=<ROBOT>, run_ingress_node.sh) 가 떠 있다고 가정한다.
IngressUnderTruck 액션에 목표(trough_index)를 보내고 피드백/결과를 리포트한다.

정지좌표/횡편차의 **GT 검증**은 이 스크립트가 하지 않는다(이 노드/스크립트는
GT 를 전혀 모른다 — 브리지의 GT 원칙과 동일). 브리지 콘솔의 ``BRIDGE_ALIVE
... gt=[...]`` 로그(2초 간격 하트비트)를 별도로 관찰/grep 해서 대조해야 한다
(taskR4/taskR5a 보고서가 이 방식을 그대로 씀).

ros2 CLI 가 이 머신에서 세그폴트하므로(taskB0 기록) 이 스크립트로 대신한다.
실행: run_ingress_smoke.sh <ROBOT> <TROUGH_INDEX> [--timeout=SEC]
"""
import sys
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from parking_robot_interfaces.action import IngressUnderTruck


class IngressSmoke(Node):
    def __init__(self, robot):
        super().__init__("ingress_smoke")
        self.robot = robot
        self.client = ActionClient(self, IngressUnderTruck,
                                    f"/robot_{robot}/ingress_under_truck")
        self._last_fb_log = 0.0

    def send(self, trough_index, timeout):
        self.get_logger().info(
            f"ingress_under_truck: trough_index={trough_index} 목표 전송 대기 중 서버 탐색…")
        if not self.client.wait_for_server(timeout_sec=15.0):
            self.get_logger().error("ingress_under_truck 액션 서버를 찾지 못했습니다")
            return None
        goal = IngressUnderTruck.Goal()
        goal.trough_index = trough_index
        goal.forward_speed = 0.0   # 0 => 노드 파라미터 기본값
        goal.return_speed = 0.0
        send_future = self.client.send_goal_async(goal, feedback_callback=self._on_feedback)
        rclpy.spin_until_future_complete(self, send_future, timeout_sec=15.0)
        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error("ingress_under_truck: 목표 거부됨")
            return None
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=timeout)
        if not result_future.done():
            self.get_logger().error("ingress_under_truck: 결과 타임아웃")
            return None
        return result_future.result().result

    def _on_feedback(self, fb_msg):
        now = time.monotonic()
        if now - self._last_fb_log < 2.0:
            return
        self._last_fb_log = now
        fb = fb_msg.feedback
        print(f"INGRESS_FEEDBACK phase={fb.phase} x={fb.current_x:.3f} "
              f"troughs_seen={fb.troughs_seen} vy_cmd={fb.vy_cmd:.3f}", flush=True)


def main():
    robot = "entry_lead"
    trough_index = 0
    timeout = 180.0
    for a in sys.argv[1:]:
        if a.startswith("--robot="):
            robot = a.split("=", 1)[1]
        elif a.startswith("--trough-index="):
            trough_index = int(a.split("=", 1)[1])
        elif a.startswith("--timeout="):
            timeout = float(a.split("=", 1)[1])
        elif not a.startswith("--"):
            robot = a

    rclpy.init()
    node = IngressSmoke(robot)
    try:
        result = node.send(trough_index, timeout)
        if result is not None:
            print(f"=== INGRESS_SMOKE_RESULT robot={robot} trough_index={trough_index} "
                  f"success={result.success} stop_reason={result.stop_reason} "
                  f"stop_x={result.stop_x:.4f} target_axle_x={result.target_axle_x:.4f} "
                  f"est_max_lateral_dev_m={result.est_max_lateral_dev_m:.4f} ===", flush=True)
        else:
            print(f"=== INGRESS_SMOKE_RESULT robot={robot} trough_index={trough_index} "
                  f"success=False stop_reason=client_error ===", flush=True)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
