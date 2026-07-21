#!/usr/bin/env python3
"""도킹·리프트·운반 중 두 로봇 모션 품질 검사 (외부 ROS).

dock 러너 odom 규약: position.y=세로 높이(점프), position.z=운반 진행축, yaw=방위.
/dock_lift 시퀀스 동안 로봇별로:
  - 점프: 높이(Y) 최대 편차 (기준 대비). 파지·리프트 중 로봇이 튀는가.
  - 발레: yaw 범위. 진입·운반은 직진이라 ~0 이어야 한다.
판정: 높이 편차 < 0.05m AND yaw 범위 < 15도.

사용: python3 dock_motion_check.py --seconds 120 [--out trace.csv]
"""
import math
import sys
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node

ROBOTS = ("robot_rear", "robot_front")
JUMP_MAX = 0.05      # m, 높이 편차 허용
YAW_RANGE_MAX = 0.26  # rad, 약 15도


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class Check(Node):
    def __init__(self):
        super().__init__("dock_motion_check")
        self.samples = {r: [] for r in ROBOTS}
        for r in ROBOTS:
            self.create_subscription(Odometry, f"/{r}/odom",
                                     lambda m, rid=r: self._rec(rid, m), 50)

    def _rec(self, rid, m):
        p = m.pose.pose.position
        self.samples[rid].append((time.time(), p.y, yaw_of(m.pose.pose.orientation)))


def analyze(rid, s):
    if len(s) < 20:
        print(f"{rid}: 표본 부족 ({len(s)})"); return False
    ys = [r[1] for r in s]
    yaws = [r[2] for r in s]
    base_y = sorted(ys[:max(5, len(ys) // 20)])[len(ys[:max(5, len(ys) // 20)]) // 2]
    max_rise = max(y - base_y for y in ys)
    min_dip = min(y - base_y for y in ys)
    max_dev = max(abs(max_rise), abs(min_dip))
    yaw_range = max(yaws) - min(yaws)
    jump_ok = max_dev <= JUMP_MAX
    ballet_ok = yaw_range <= YAW_RANGE_MAX
    print(f"  {rid}: 점프 최대편차 {max_dev:+.3f}m ({'OK' if jump_ok else 'FAIL'}) | "
          f"yaw범위 {math.degrees(yaw_range):.0f}° ({'OK' if ballet_ok else 'FAIL'})")
    return jump_ok and ballet_ok


def main():
    args = sys.argv[1:]
    secs = float(args[args.index("--seconds") + 1]) if "--seconds" in args else 120.0
    rclpy.init()
    n = Check()
    end = time.time() + secs
    while time.time() < end:
        rclpy.spin_once(n, timeout_sec=0.1)
    results = [analyze(r, n.samples[r]) for r in ROBOTS]
    print(f"DOCK_MOTION={'PASS' if all(results) else 'FAIL'}")
    sys.exit(0 if all(results) else 1)


if __name__ == "__main__":
    main()
