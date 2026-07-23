#!/usr/bin/env python3
"""Isaac 듀얼 필드의 odom 좌표 규약을 실측 검증한다 (외부 ROS 터미널에서 실행).

로봇별로 3상: +x 전진 → odom x 증가 / +y 좌 strafe → odom y 증가(도크에서 yaw≈0 기준)
/ +wz → yaw 증가(CCW). 각 상 후 원위치 복귀는 하지 않는다(소변위, 빈 슬롯 위라 안전).
"""
import math
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node

ROBOTS = ("robot_1", "robot_2")
PUSH_SEC = 3.0
SPEED = 0.3
TURN = 0.4


def yaw_of(msg):
    q = msg.pose.pose.orientation
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class Verify(Node):
    def __init__(self):
        super().__init__("verify_dual_odom")
        self.odom = {}
        for r in ROBOTS:
            self.create_subscription(
                Odometry, f"/{r}/odom",
                lambda m, rid=r: self.odom.__setitem__(rid, m), 10)
        self.pubs = {r: self.create_publisher(Twist, f"/{r}/cmd_vel", 10) for r in ROBOTS}

    def snap(self, r):
        m = self.odom[r]
        p = m.pose.pose.position
        return p.x, p.y, yaw_of(m)

    def push(self, r, vx=0.0, vy=0.0, wz=0.0, sec=PUSH_SEC):
        t = Twist()
        t.linear.x, t.linear.y, t.angular.z = float(vx), float(vy), float(wz)
        end = time.time() + sec
        while time.time() < end:
            self.pubs[r].publish(t)
            rclpy.spin_once(self, timeout_sec=0.05)
        self.pubs[r].publish(Twist())  # 정지
        for _ in range(10):
            rclpy.spin_once(self, timeout_sec=0.1)


def main():
    rclpy.init()
    node = Verify()
    deadline = time.time() + 20
    while time.time() < deadline and len(node.odom) < len(ROBOTS):
        rclpy.spin_once(node, timeout_sec=0.2)
    if len(node.odom) < len(ROBOTS):
        print(f"VERIFY_RESULT=FAIL 이유=odom 미수신 ({list(node.odom)})")
        sys.exit(1)

    ok = True
    for r in ROBOTS:
        x0, y0, w0 = node.snap(r)
        node.push(r, vx=SPEED)
        x1, y1, w1 = node.snap(r)
        fwd = (x1 - x0) * math.cos(w0) + (y1 - y0) * math.sin(w0)
        f_ok = fwd > 0.4
        node.push(r, vy=SPEED)
        x2, y2, _ = node.snap(r)
        left = -(x2 - x1) * math.sin(w1) + (y2 - y1) * math.cos(w1)
        l_ok = left > 0.4
        node.push(r, wz=TURN, sec=2.0)
        _, _, w2 = node.snap(r)
        dyaw = (w2 - w1 + math.pi) % (2 * math.pi) - math.pi
        y_ok = dyaw > 0.3
        print(f"{r}: forward={fwd:+.2f}({'OK' if f_ok else 'BAD'}) "
              f"left={left:+.2f}({'OK' if l_ok else 'BAD'}) "
              f"dyaw={math.degrees(dyaw):+.1f}deg({'OK' if y_ok else 'BAD'})")
        ok = ok and f_ok and l_ok and y_ok
    print(f"VERIFY_RESULT={'PASS' if ok else 'FAIL'}")
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
