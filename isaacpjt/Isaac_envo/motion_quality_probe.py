#!/usr/bin/env python3
"""모션 품질 측정기 — 도착 좌표만 보던 검증의 구멍을 메운다 (외부 ROS).

두 로봇의 /odom 을 고속 기록해서 "잘 도착했나"가 아니라 "가는 동안 멀쩡했나"를
숫자로 판정한다:
  - 점프(jump)   : 세로 높이(odom.pose.position.z = USD Y-up)가 정지 기준에서
                   얼마나 튀는가. 지면 로봇은 높이가 거의 상수여야 한다.
  - 발레(ballet) : yaw 총 경로(누적 |Δyaw|)가 순수 회전량보다 얼마나 큰가.
                   회전 없는 경로라면 둘이 비슷해야 한다. 마구 돌면 총경로가 폭주.
                   + 최대 |yaw 각속도|.
  - 오버슈트     : 실내+인계장 x 범위(대략 -41 ~ +16)를 벗어나면 이탈로 본다.

사용:
  python3 motion_quality_probe.py --seconds 400 [--out trace.csv]
정지 기준 높이는 처음 2초(로봇이 아직 명령 전) 중앙값으로 잡는다.
"""
import math
import sys
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node

ROBOTS = ("robot_1", "robot_2")
# 판정 임계 (초기값 — 이 수치 자체가 이번 작업의 산출물)
# 발레(격렬한 회전)와 리더의 미세 yaw 떨림을 구분하는 핵심 지표는 "yaw 범위"와
# "최대 각속도"다. 누적경로(excess)는 시간에 비례해 커져 미세떨림도 잡으므로
# 판정에서 빼고 참고용으로만 출력한다.
JUMP_RISE_MAX = 0.05      # m, 기준 높이 대비 최대 상승 허용
JUMP_STD_MAX = 0.02       # m, 높이 표준편차 허용
YAW_RANGE_MAX = 0.79      # rad, (max yaw - min yaw). 약 45도 — 회전 없는 경로 기준
YAWRATE_MAX = 0.6         # rad/s, 최대 각속도 허용 (리더 실측 0.14 → 여유)
X_MIN, X_MAX = -41.5, 16.5


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class Probe(Node):
    def __init__(self):
        super().__init__("motion_quality_probe")
        self.samples = {r: [] for r in ROBOTS}   # (t, x, y, h, yaw)
        for r in ROBOTS:
            self.create_subscription(
                Odometry, f"/{r}/odom",
                lambda m, rid=r: self._rec(rid, m), 50)

    def _rec(self, rid, m):
        p = m.pose.pose.position
        self.samples[rid].append(
            (time.time(), p.x, p.y, p.z, yaw_of(m.pose.pose.orientation)))


def analyze(rid, s, out=None):
    if len(s) < 20:
        print(f"{rid}: 표본 부족 ({len(s)})"); return False
    ts = [r[0] for r in s]
    heights = [r[3] for r in s]
    yaws = [r[4] for r in s]
    n_base = max(5, sum(1 for t in ts if t < ts[0] + 2.0))
    base_h = sorted(heights[:n_base])[n_base // 2]
    rises = [h - base_h for h in heights]
    max_rise = max(rises)
    min_dip = min(rises)
    mean_h = sum(heights) / len(heights)
    std_h = (sum((h - mean_h) ** 2 for h in heights) / len(heights)) ** 0.5

    yaw_path = 0.0
    max_rate = 0.0
    # 각속도는 표본 노이즈에 튀므로 0.2s 창으로 평균 낸 속도의 최대를 본다.
    win = []
    for i in range(1, len(s)):
        d = (yaws[i] - yaws[i - 1] + math.pi) % (2 * math.pi) - math.pi
        yaw_path += abs(d)
        dt = ts[i] - ts[i - 1]
        win.append((ts[i], d))
        win = [(t, dd) for (t, dd) in win if t > ts[i] - 0.2]
        span = win[-1][0] - win[0][0]
        if span > 0.05:
            rate = abs(sum(dd for _, dd in win)) / span
            max_rate = max(max_rate, rate)
    net_yaw = abs((yaws[-1] - yaws[0] + math.pi) % (2 * math.pi) - math.pi)
    excess = yaw_path - net_yaw
    yaw_range = max(yaws) - min(yaws)

    xs = [r[1] for r in s]
    out_of_bounds = sum(1 for x in xs if x < X_MIN or x > X_MAX)

    jump_ok = max_rise <= JUMP_RISE_MAX and std_h <= JUMP_STD_MAX
    ballet_ok = yaw_range <= YAW_RANGE_MAX and max_rate <= YAWRATE_MAX
    bounds_ok = out_of_bounds == 0
    ok = jump_ok and ballet_ok and bounds_ok

    print(f"\n=== {rid} ===")
    print(f"  점프  : 기준높이 {base_h:.3f}m | 최대상승 {max_rise:+.3f}m "
          f"최대하강 {min_dip:+.3f}m 높이std {std_h:.3f}m → "
          f"{'OK' if jump_ok else 'FAIL'}")
    print(f"  발레  : yaw범위 {math.degrees(yaw_range):.0f}° "
          f"최대각속도 {max_rate:.2f}rad/s "
          f"(참고: 총경로 {math.degrees(yaw_path):.0f}°/순회전 {math.degrees(net_yaw):.0f}°) → "
          f"{'OK' if ballet_ok else 'FAIL'}")
    print(f"  이탈  : x범위밖 {out_of_bounds}표본 "
          f"(x {min(xs):.1f}~{max(xs):.1f}) → {'OK' if bounds_ok else 'FAIL'}")
    if out is not None:
        for row in s:
            out.write(f"{rid},{row[0]:.3f},{row[1]:.3f},{row[2]:.3f},"
                      f"{row[3]:.4f},{math.degrees(row[4]):.1f}\n")
    return ok


def analyze_csv(path):
    """저장된 궤적 CSV를 재분석한다 (ROS/재실행 불필요)."""
    samples = {r: [] for r in ROBOTS}
    with open(path) as f:
        next(f)
        for line in f:
            rid, t, x, y, h, yaw = line.strip().split(",")
            if rid in samples:
                samples[rid].append(
                    (float(t), float(x), float(y), float(h), math.radians(float(yaw))))
    results = [analyze(r, samples[r]) for r in ROBOTS]
    print(f"\nMOTION_QUALITY={'PASS' if all(results) else 'FAIL'}")


def main():
    args = sys.argv[1:]
    if "--from-csv" in args:
        analyze_csv(args[args.index("--from-csv") + 1])
        return
    secs = float(args[args.index("--seconds") + 1]) if "--seconds" in args else 400.0
    out_path = args[args.index("--out") + 1] if "--out" in args else None
    rclpy.init()
    n = Probe()
    end = time.time() + secs
    while time.time() < end:
        rclpy.spin_once(n, timeout_sec=0.1)
    out = open(out_path, "w") if out_path else None
    if out:
        out.write("robot,t,x,y,height,yaw_deg\n")
    # list 로 먼저 전부 평가 (all() 단축평가로 두 번째 로봇이 누락되던 버그 수정)
    results = [analyze(r, n.samples[r], out) for r in ROBOTS]
    if out:
        out.close()
        print(f"\n궤적 저장: {out_path}")
    all_ok = all(results)
    print(f"\nMOTION_QUALITY={'PASS' if all_ok else 'FAIL'}")
    sys.exit(0 if all_ok else 1)


if __name__ == "__main__":
    main()
