#!/usr/bin/env python3
"""임시 미션: robot_1 이 ArUco 폐루프로 도크→H_B 접근점까지 (외부 ROS).

물리 경로는 그래프 중앙선(y=0)이 아니라 **마커 차선(y=-2.5)** 위로 잡는다 —
차선에는 3.40 m 간격으로 마커가 있어 폐루프 보정이 촘촘하다(중앙선은 횡단
마커 2장뿐). 로봇은 각 구간의 진행 방향을 바라보도록 회전한다(무적재 회전 허용).
그래프 경로는 존 산정 근거로만 쓰며, 구간별 존을 leg 에 수동 매핑했다
(임시 관제 — 실제 orchestrator 가 오면 대체).

- 존: leg 진입 전에 acquire_zones(robot_1), 이전 존 release (팀원 권장 규약).
- 편대: robot_2 팔로워 배정(호송).
- GT는 최종 채점(도착 오차)만 — 제어에는 절대 불사용.
"""
import math
import sys
import time

import rclpy
from nav_msgs.msg import Odometry
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node

from parking_robot_interfaces.msg import FormationAssignment
from parking_robot_interfaces.srv import AcquireZones, ReleaseZones

# (x, y, yaw, 이 leg 진입 전에 잡을 존 또는 None)
# ★ 회전 없는 경로 (전 구간 yaw=0, 동향 유지) ★
#  - 서진은 후진으로: 로봇이 서쪽으로 이동하면 지나친 마커가 (검증된) 동향
#    검출창(전방 1.1~1.4m)에 들어와 3.4m마다 보정된다.
#  - 제자리 회전은 이 에셋에서 방향까지 비결정적(빠른 wz)이거나 게인 3배(느린
#    wz)라 배제. 서향 시 마커 검출 0 문제(정반사 의심)도 함께 회피.
LEGS = [
    (-15.3, -2.5, 0.0, None),            # 도크 → A차선 (좌 strafe, 초기 pose 기반)
    (-13.15, -2.5, 0.0, None),           # 조기 보정 우회: A1 마커가 검출창에
                                         # 들어오는 지점까지 동진 → 첫 절대 보정
    (-18.1, -2.5, 0.0, "Z_ENTRANCE"),    # 후진 서진 시작 (도크 마커·GW 재검출)
    (-22.8, -2.5, 0.0, "ZH_GATE"),       # 인계장 진입 (HE3, HE2 통과)
    (-26.2, -2.5, 0.0, "ZH01"),          # HE1
    (-29.6, -2.5, 0.0, "ZH02"),          # H_A 마커 열(베이 x)
    (-29.6, 2.5, 0.0, "ZH03"),           # H_B 접근선 — 좌 strafe 5m (블라인드)
    (-27.55, 2.5, 0.0, None),            # 종단 정밀 보정: HE1'(-26.2) 검출창까지 동진
    (-29.6, 2.5, 0.0, None),             # 후진 복귀 — 초반 HE1' 재검출로 보정된
                                         # 상태에서 정밀 도착 (H_B 접근점)
]
GOAL_TOL_GT = 0.35   # 최종 채점(GT 기준) 허용 오차


class LegMission(Node):
    def __init__(self):
        super().__init__("aruco_leg_mission")
        self.gt = None
        self.create_subscription(Odometry, "/robot_1/odom",
                                 lambda m: setattr(self, "gt", m), 10)
        self.nav = ActionClient(self, NavigateToPose, "/robot_1/navigate_to_pose")
        self.acquire = self.create_client(AcquireZones, "/acquire_zones")
        self.release = self.create_client(ReleaseZones, "/release_zones")
        self.assign = self.create_publisher(
            FormationAssignment, "/formation_assignment", 10)

    def call(self, client, req, timeout=5.0):
        fut = client.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=timeout)
        return fut.result()

    def acquire_zone(self, zone):
        while True:
            r = self.call(self.acquire, AcquireZones.Request(
                robot_id="robot_1", task_id="", zone_ids=[zone]))
            if r and r.granted:
                self.get_logger().info(f"존 획득: {zone}")
                return
            wait = (r.retry_after_sec if r else 1.0) or 1.0
            self.get_logger().info(f"존 대기: {zone} ({wait:.0f}s)")
            time.sleep(wait)

    def release_zone(self, zone):
        self.call(self.release, ReleaseZones.Request(
            robot_id="robot_1", task_id="", zone_ids=[zone]))

    def goto(self, x, y, yaw):
        goal = NavigateToPose.Goal()
        goal.pose.pose.position.x = float(x)
        goal.pose.pose.position.y = float(y)
        goal.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal.pose.pose.orientation.w = math.cos(yaw / 2.0)
        self.nav.wait_for_server()
        send = self.nav.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send, timeout_sec=15)
        handle = send.result()
        if handle is None or not handle.accepted:
            return False
        res = handle.get_result_async()
        rclpy.spin_until_future_complete(self, res, timeout_sec=300)
        return res.done()

    def broadcast(self, active):
        for rid, role, partner in (("robot_2", "follower", "robot_1"),
                                   ("robot_1", "leader", "robot_2")):
            self.assign.publish(FormationAssignment(
                robot_id=rid, task_id="aruco-leg" if active else "",
                role=role, partner_robot_id=partner, active=active))


def main():
    rclpy.init()
    n = LegMission()
    end = time.time() + 20
    while time.time() < end and n.gt is None:
        rclpy.spin_once(n, timeout_sec=0.2)
    if n.gt is None:
        print("ARUCO_LEG=FAIL 이유=GT odom 미수신"); sys.exit(1)

    n.broadcast(True)
    for _ in range(10):
        n.broadcast(True)
        rclpy.spin_once(n, timeout_sec=0.1)

    held = None
    try:
        for i, (x, y, yaw, zone) in enumerate(LEGS):
            if zone:
                n.acquire_zone(zone)
                if held:
                    n.release_zone(held)
                held = zone
            n.get_logger().info(
                f"leg {i + 1}/{len(LEGS)}: ({x:+.1f},{y:+.1f}) yaw={math.degrees(yaw):.0f}°")
            if not n.goto(x, y, yaw):
                print("ARUCO_LEG=FAIL 이유=goal 실패"); sys.exit(1)
    finally:
        if held:
            n.release_zone(held)
        n.broadcast(False)

    end = time.time() + 5
    while time.time() < end:
        rclpy.spin_once(n, timeout_sec=0.2)
    g = n.gt.pose.pose.position
    gx, gy = LEGS[-1][0], LEGS[-1][1]
    err = math.hypot(g.x - gx, g.y - gy)
    verdict = "PASS" if err < GOAL_TOL_GT else "FAIL"
    print(f"ARUCO_LEG={verdict} GT기준 도착 오차={err:.2f}m (목표 {gx:+.1f},{gy:+.1f} / 실제 {g.x:+.2f},{g.y:+.2f})")
    sys.exit(0 if verdict == "PASS" else 1)


if __name__ == "__main__":
    main()
