#!/usr/bin/env python3
"""편대 추종 첫 실증 (외부 ROS 터미널).

절차: ① robot_2에 follower 배정 방송 → ② robot_1을 스크립트가 직접 조종해
도크→통로(y≈0)→동쪽 6m 주행 → ③ robot_2가 gap-hold로 리더 2.9m 뒤에
수렴·유지하는지 측정. gap 오차 < 0.45m가 3초 유지되면 PASS.

주의: 리더 조종은 임시(Plan 2에서 ArUco navigate로 교체). 배정 방송은 본래
task_dispatcher 몫이지만, 여기서는 편대 계층만 떼어 검증하므로 직접 쏜다.
"""
import math
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node

from parking_robot_interfaces.msg import FormationAssignment

GAP = 2.9
TOL = 0.45
HOLD_SEC = 3.0
TIMEOUT = 120.0


def yaw_of(m):
    q = m.pose.pose.orientation
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class Demo(Node):
    def __init__(self):
        super().__init__("foundation_formation_demo")
        self.odom = {}
        for r in ("robot_1", "robot_2"):
            self.create_subscription(
                Odometry, f"/{r}/odom",
                lambda m, rid=r: self.odom.__setitem__(rid, m), 10)
        self.cmd1 = self.create_publisher(Twist, "/robot_1/cmd_vel", 10)
        self.assign = self.create_publisher(FormationAssignment, "/formation_assignment", 10)

    def pose(self, r):
        m = self.odom[r]
        p = m.pose.pose.position
        return p.x, p.y, yaw_of(m)

    def wait_odom(self):
        end = time.time() + 20
        while time.time() < end and len(self.odom) < 2:
            rclpy.spin_once(self, timeout_sec=0.2)
        return len(self.odom) == 2

    def broadcast(self, active):
        for rid, role, partner in (("robot_2", "follower", "robot_1"),
                                   ("robot_1", "leader", "robot_2")):
            self.assign.publish(FormationAssignment(
                robot_id=rid, task_id="foundation-demo" if active else "",
                role=role, partner_robot_id=partner, active=active))

    def drive_leader(self, vx, vy, until, timeout):
        t = Twist()
        t.linear.x, t.linear.y = float(vx), float(vy)
        end = time.time() + timeout
        while time.time() < end and not until():
            self.cmd1.publish(t)
            rclpy.spin_once(self, timeout_sec=0.05)
        self.cmd1.publish(Twist())


def main():
    rclpy.init()
    node = Demo()
    if not node.wait_odom():
        print("FORMATION_DEMO=FAIL 이유=odom 미수신"); sys.exit(1)

    node.broadcast(True)
    # 배정 방송이 컨트롤러에 닿도록 잠깐 반복
    for _ in range(10):
        node.broadcast(True)
        rclpy.spin_once(node, timeout_sec=0.1)

    # 리더: 도크(y=-7.8)에서 통로 중심(y≈-0.3)으로 좌 strafe (A1/A2 빈 슬롯 위 통과)
    node.drive_leader(0.0, 0.45, lambda: node.pose("robot_1")[1] > -0.3, 40)
    # 리더: 통로를 동쪽으로 6m (팔로워가 이 사이 수렴해야 함)
    x_start = node.pose("robot_1")[0]
    hold_since, verdict = None, "FAIL"
    end = time.time() + TIMEOUT

    def gap_error():
        lx, ly, lw = node.pose("robot_1")
        fx, fy, _ = node.pose("robot_2")
        tx, ty = lx - GAP * math.cos(lw), ly - GAP * math.sin(lw)
        return math.hypot(tx - fx, ty - fy)

    t = Twist(); t.linear.x = 0.3
    while time.time() < end:
        node.cmd1.publish(t)
        rclpy.spin_once(node, timeout_sec=0.05)
        err = gap_error()
        if err < TOL:
            hold_since = hold_since or time.time()
            if time.time() - hold_since >= HOLD_SEC:
                verdict = "PASS"; break
        else:
            hold_since = None
        if node.pose("robot_1")[0] - x_start > 6.0:
            t.linear.x = 0.0   # 리더 도착 — 정지 상태 수렴도 인정
    node.cmd1.publish(Twist())
    node.broadcast(False)
    print(f"FORMATION_DEMO={verdict} 최종 gap 오차={gap_error():.2f}m")
    sys.exit(0 if verdict == "PASS" else 1)


if __name__ == "__main__":
    main()
