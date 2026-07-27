#!/usr/bin/env python3
"""carry_action_server — Phase D 2로봇 가상중심(virtual-center) 운반 액션 서버.

트럭을 든 lead/follow 를 하나의 강체로 보고, 두 로봇의 **중점(=가상중심)** 을 목표
슬롯 pose 로 몬다.

- 측위: /robot_<lead>/pose + /robot_<follow>/pose (marker_localizer 의 바닥마커 기반
  융합 PoseStamped, **GT 아님**) -> formation.center_pose_from_robots 로 중심 pose.
  대칭 오프셋(중심=중점)이라 좌표 규약과 무관하게 중심 = 두 위치 평균(회전항 상쇄).
- 제어: 그 중심 pose 를 PoseController(기존 P제어법 그대로 재사용) 에 넣어 중심 twist
  -> formation.robot_twist_from_center 로 각 로봇 body twist 로 변환 -> 두 /cmd_vel.

책임은 "중심을 슬롯까지 몰기"까지. 안착(lift down)·후진 이탈·도크 복귀는 오케가
CarryToSlot 성공 뒤 기존 액션(ControlLift/pose_controller)으로 순차 처리(단일 책임).
"""
import math
import threading
import time

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped, Twist

from parking_robot_interfaces.action import CarryToSlot
from parkbot_motion import formation
from parkbot_motion.pose_controller import PoseController
from parkbot_motion.pose_controller_node import odom_quat_to_yaw_deg


class CarryActionServer(Node):
    def __init__(self):
        super().__init__('carry_action_server')
        self._cbg = ReentrantCallbackGroup()
        # 강체 편성: 중심 기준 각 로봇 x 오프셋[m](포메이션 프레임, x=진행방향).
        # lead 앞(+)/follow 뒤(-). **대칭이어야** 중심=중점이 되어 규약 무관.
        # 값은 두 축 간격의 절반 — 하드웨어 실측으로 튜닝(ponytail: calibration knob).
        self.declare_parameter('lead_offset_x', 1.8)
        self.declare_parameter('follow_offset_x', -1.8)
        self.declare_parameter('control_hz', 20.0)
        self.declare_parameter('pose_stale_sec', 1.0)
        self.declare_parameter('pos_gain', 0.8)
        self.declare_parameter('yaw_gain', 1.2)
        self.declare_parameter('max_lin', 0.20)
        self.declare_parameter('max_ang', 0.5)
        self.declare_parameter('pos_tol', 0.06)
        self.declare_parameter('yaw_tol', 1.0)
        self.declare_parameter('goal_timeout_sec', 300.0)

        gp = lambda n: self.get_parameter(n).value           # noqa: E731
        self.lead_off = (float(gp('lead_offset_x')), 0.0)
        self.follow_off = (float(gp('follow_offset_x')), 0.0)
        self.control_hz = float(gp('control_hz'))
        self.pose_stale_sec = float(gp('pose_stale_sec'))
        self.gains = dict(
            pos_gain=float(gp('pos_gain')), yaw_gain=float(gp('yaw_gain')),
            max_lin=float(gp('max_lin')), max_ang=float(gp('max_ang')),
            pos_tol=float(gp('pos_tol')), yaw_tol=float(gp('yaw_tol')))
        self.goal_timeout = float(gp('goal_timeout_sec'))

        self._lock = threading.Lock()
        self._pose = {}     # robot_id -> (x, z, yaw_deg, mono_time)
        self._subs = {}     # robot_id -> pose 구독(지연 생성)
        self._cmd = {}      # robot_id -> cmd_vel 퍼블리셔(지연 생성)

        self._server = ActionServer(
            self, CarryToSlot, 'carry_to_slot', self._execute,
            callback_group=self._cbg,
            goal_callback=lambda _g: GoalResponse.ACCEPT,
            cancel_callback=lambda _g: CancelResponse.ACCEPT)
        self.get_logger().info(
            f'carry_action_server 시작 offsets lead={self.lead_off[0]} '
            f'follow={self.follow_off[0]}')

    def _ensure_io(self, rid):
        if rid not in self._cmd:
            self._cmd[rid] = self.create_publisher(Twist, f'/robot_{rid}/cmd_vel', 10)
        if rid not in self._subs:
            self._subs[rid] = self.create_subscription(
                PoseStamped, f'/robot_{rid}/pose',
                lambda m, r=rid: self._on_pose(r, m), 10, callback_group=self._cbg)

    def _on_pose(self, rid, msg):
        p, q = msg.pose.position, msg.pose.orientation
        yaw_deg = odom_quat_to_yaw_deg(q.x, q.y, q.z, q.w)
        with self._lock:
            self._pose[rid] = (float(p.x), float(p.z), yaw_deg, time.monotonic())

    def _center_pose(self, lead, follow):
        """캐시된 두 pose -> 중심 (x,z,yaw_rad). 하나라도 스테일/없음이면 None(안전정지).
        ponytail: 둘 다 요구. 한쪽 오클루전 잦으면 단일+오프셋 폴백 추가."""
        now = time.monotonic()
        with self._lock:
            lp, fp = self._pose.get(lead), self._pose.get(follow)
        est = []
        for p, off in ((lp, self.lead_off), (fp, self.follow_off)):
            if p is None or now - p[3] > self.pose_stale_sec:
                return None
            est.append(((p[0], p[1], math.radians(p[2])), off))
        return formation.center_pose_from_robots(est)

    def _pub(self, rid, tw):
        m = Twist()
        m.linear.x, m.linear.y, m.angular.z = float(tw[0]), float(tw[1]), float(tw[2])
        self._cmd[rid].publish(m)

    def _stop(self, lead, follow):
        for rid in (lead, follow):
            self._cmd[rid].publish(Twist())

    def _execute(self, goal_handle):
        g = goal_handle.request
        lead, follow = g.lead_robot_id, g.follow_robot_id
        self._ensure_io(lead)
        self._ensure_io(follow)
        target = (g.target_x, g.target_z, g.target_yaw_deg)
        ctrl = PoseController(target, **self.gains)
        self.get_logger().info(f'carry 시작 lead={lead} follow={follow} target={target}')

        period = 1.0 / self.control_hz
        prev = time.monotonic()
        deadline = prev + self.goal_timeout
        result = CarryToSlot.Result()

        while rclpy.ok():
            time.sleep(period)
            now = time.monotonic()
            dt = now - prev
            prev = now
            if goal_handle.is_cancel_requested:
                self._stop(lead, follow)
                goal_handle.canceled()
                result.success, result.message = False, 'canceled'
                return result
            if now > deadline:
                self._stop(lead, follow)
                goal_handle.abort()
                result.success, result.message = False, 'timeout'
                return result

            center = self._center_pose(lead, follow)      # (x,z,yaw_rad) | None
            if center is None:
                twist = ctrl.step(None, dt)               # 안전 감속
            else:
                twist = ctrl.step((center[0], center[1], math.degrees(center[2])), dt)
                fb = CarryToSlot.Feedback()
                fb.phase = 'DRIVING'
                fb.dist_remaining = math.hypot(target[0] - center[0], target[1] - center[1])
                goal_handle.publish_feedback(fb)

            self._pub(lead, formation.robot_twist_from_center(twist, self.lead_off))
            self._pub(follow, formation.robot_twist_from_center(twist, self.follow_off))

            if ctrl.done:      # 도달래치 + 감속 0 도달. ponytail: settle-median 스킵.
                break          #   슬롯 정밀 부족하면 PoseController settle 창 추가.

        self._stop(lead, follow)
        final = self._center_pose(lead, follow)
        goal_handle.succeed()
        result.success, result.message = True, 'carry done'
        if final is not None:
            result.final_x, result.final_z = final[0], final[1]
            result.final_yaw_deg = math.degrees(final[2])
        return result


def main():
    rclpy.init()
    node = CarryActionServer()
    ex = MultiThreadedExecutor()
    ex.add_node(node)
    try:
        ex.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
