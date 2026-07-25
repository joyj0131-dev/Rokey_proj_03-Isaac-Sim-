#!/usr/bin/env python3
"""R2 브리지 스모크 테스트 (순수 ROS2, 시스템 rclpy).

parking_v4_runner.py --bridge 가 떠 있다고 가정하고, entry_lead 로봇에
/cmd_vel 로 전진 명령을 몇 초 보낸 뒤 정지시키고, 그동안 /odom 을 구독해
포즈가 실제로 앞으로 나아가는지 확인한다. ros2 CLI 가 이 머신에서 세그폴트
하므로(taskB0 기록) `ros2 topic pub/echo` 대신 이 스크립트를 쓴다.

실행: run_bridge_smoke.sh (도메인 126 + fastdds 화이트리스트가 세팅된다)
"""
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

ROBOT = "entry_lead"
DRIVE_SEC = 3.0
LIN_X = 0.15


class BridgeSmoke(Node):
    def __init__(self):
        super().__init__("bridge_smoke")
        self.cmd_pub = self.create_publisher(Twist, f"/robot_{ROBOT}/cmd_vel", 10)
        self.odom_sub = self.create_subscription(
            Odometry, f"/robot_{ROBOT}/odom", self._on_odom, 10)
        self.samples = []
        self.get_logger().info(
            f"bridge_smoke 시작: robot={ROBOT} lin_x={LIN_X} drive_sec={DRIVE_SEC}. "
            f"러너(parking_v4_runner.py --bridge)의 /robot_{ROBOT}/odom 을 기다립니다…")

    def _on_odom(self, msg: Odometry):
        p = msg.pose.pose.position
        t = msg.twist.twist
        sample = (time.monotonic(), p.x, p.z, t.linear.x, t.linear.y, t.angular.z)
        self.samples.append(sample)
        if len(self.samples) <= 5 or len(self.samples) % 20 == 0:
            self.get_logger().info(
                f"ODOM #{len(self.samples)} pos=({p.x:.3f},{p.z:.3f}) "
                f"twist=({t.linear.x:.3f},{t.linear.y:.3f},{t.angular.z:.3f})")


def main():
    rclpy.init()
    node = BridgeSmoke()
    try:
        # 디스커버리 대기(probe_a_detector 관례상 ~수 초~15초 걸릴 수 있다).
        t0 = time.monotonic()
        while time.monotonic() - t0 < 5.0:
            rclpy.spin_once(node, timeout_sec=0.2)

        n_before = len(node.samples)
        node.get_logger().info(f"discovery 창 종료: odom {n_before}개 수신")

        tw = Twist()
        tw.linear.x = LIN_X
        t_start = time.monotonic()
        while time.monotonic() - t_start < DRIVE_SEC:
            node.cmd_pub.publish(tw)
            rclpy.spin_once(node, timeout_sec=0.05)

        tw.linear.x = 0.0
        t_stop = time.monotonic()
        while time.monotonic() - t_stop < 2.0:
            node.cmd_pub.publish(tw)
            rclpy.spin_once(node, timeout_sec=0.05)

        n_total = len(node.samples)
        if n_total >= 2:
            _, x0, z0, *_ = node.samples[0]
            _, x1, z1, *_ = node.samples[-1]
            dist = ((x1 - x0) ** 2 + (z1 - z0) ** 2) ** 0.5
            node.get_logger().info(
                f"=== SMOKE_RESULT odom_msgs={n_total} start=({x0:.3f},{z0:.3f}) "
                f"end=({x1:.3f},{z1:.3f}) dist={dist:.3f}m ===")
        else:
            node.get_logger().warn(
                f"=== SMOKE_RESULT odom_msgs={n_total} — 메시지가 부족합니다. "
                f"러너가 --bridge 로 떠 있는지, 도메인/화이트리스트가 맞는지 확인하세요. ===")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
