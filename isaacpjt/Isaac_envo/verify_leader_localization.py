#!/usr/bin/env python3
"""robot_1 마커 측위 vs GT odom 대조 (외부 ROS).

robot_1을 A차선(ros y=-2.5)으로 이동시킨 뒤 천천히 전진, 마커 fix 를 GT와 비교.
위치 오차 <0.15m & yaw 오차 <8도 fix 가 3회 이상이면 PASS.
"""
import math
import sys
import time

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class Check(Node):
    def __init__(self):
        super().__init__("verify_leader_localization")
        self.gt = None
        self.hits = []
        self.create_subscription(Odometry, "/robot_1/odom",
                                 lambda m: setattr(self, "gt", m), 10)
        self.create_subscription(PoseStamped, "/robot_1/robot_pose", self.on_fix, 10)
        self.cmd = self.create_publisher(Twist, "/robot_1/cmd_vel", 10)

    def on_fix(self, m):
        if self.gt is None:
            return
        g = self.gt.pose.pose
        dp = math.hypot(m.pose.position.x - g.position.x,
                        m.pose.position.y - g.position.y)
        dyaw = abs((yaw_of(m.pose.orientation) - yaw_of(g.orientation)
                    + math.pi) % (2 * math.pi) - math.pi)
        self.hits.append((dp, math.degrees(dyaw)))
        print(f"fix#{len(self.hits)}: 위치오차={dp:.3f}m yaw오차={math.degrees(dyaw):.1f}deg")

    def gt_xy(self):
        p = self.gt.pose.pose.position
        return p.x, p.y

    def drive(self, vx, vy, until, timeout):
        """시간이 아니라 GT 조건으로 종료 — 카메라 렌더링으로 시뮬이 실시간보다
        느려도(실측 ~0.4x) 목표 지점에 정확히 도달한다. GT는 주행 세팅용이고
        검증 대상(마커 fix)과 무관하다."""
        t = Twist(); t.linear.x, t.linear.y = float(vx), float(vy)
        end = time.time() + timeout
        while time.time() < end and not until():
            self.cmd.publish(t)
            rclpy.spin_once(self, timeout_sec=0.05)
        self.cmd.publish(Twist())


def main():
    rclpy.init()
    n = Check()
    end = time.time() + 15
    while time.time() < end and n.gt is None:
        rclpy.spin_once(n, timeout_sec=0.2)
    if n.gt is None:
        print("LOCALIZE_VERIFY=FAIL 이유=GT odom 미수신"); sys.exit(1)
    # 도크(y=-7.8) → A차선(y=-2.5)까지 좌 strafe, 이후 저속 전진으로 마커 열 통과
    n.drive(0.0, 0.45, until=lambda: n.gt_xy()[1] > -2.55, timeout=90)
    n.drive(0.25, 0.0, until=lambda: n.gt_xy()[0] > -4.0, timeout=240)
    good = [h for h in n.hits if h[0] < 0.15 and h[1] < 8.0]
    verdict = "PASS" if len(good) >= 3 else "FAIL"
    print(f"LOCALIZE_VERIFY={verdict} fixes={len(n.hits)} good={len(good)}")
    sys.exit(0 if verdict == "PASS" else 1)


if __name__ == "__main__":
    main()
