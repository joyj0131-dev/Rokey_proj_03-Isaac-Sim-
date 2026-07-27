#!/usr/bin/env python3
"""carry_action_server — Phase D 2로봇 가상중심(virtual-center) 운반 액션 서버.

트럭을 든 lead/follow 를 하나의 강체로 보고, 두 로봇의 **중점** 을 목표 위치로 몬다.

2026-07-27 재안무: 진입 후 lead 서향(-90°)/follow 동향(+90°)으로 **반평행**이다.
이 편성에서는 두 로봇 헤딩 평균이 degenerate(±180) 라 옛 방식(center_pose_from_robots
로 중심 yaw + robot_twist_from_center 로 중심프레임 분배)이 깨져 나선 발산했다(실측).
- 측위: /robot_<id>/pose(marker_localizer 바닥마커 융합, **GT 아님**) 두 개. 중심=참
  중점(위치 평균), 트럭방향=formation.truck_yaw_from_robots(위치기반, 헤딩 무관).
- 제어: **월드프레임**에서 중심 위치오차 -> 월드속도(순수 병진, omega=0 — 회전 안 함).
  formation.robot_twist_world 로 각 로봇 **자기 heading** 에 투영 -> 두 /cmd_vel.
  lead 는 서향이라 후진, follow 는 동향이라 전진으로 자연히 동쪽 운반(강체 병진).

책임은 "중심을 목표 위치로 몰기(방향 유지)"까지. 슬롯 앞 90° 회전·진입·안착(lift
down)은 오케가 CarryToSlot 성공 뒤 순차 처리(단일 책임).
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
from std_msgs.msg import Float32

from parking_robot_interfaces.action import CarryToSlot
from parkbot_motion import formation
from parkbot_motion.pose_controller_node import odom_quat_to_yaw_deg


class CarryActionServer(Node):
    def __init__(self):
        super().__init__('carry_action_server')
        self._cbg = ReentrantCallbackGroup()
        # 중심 = 두 로봇 위치의 참 중점(오프셋 파라미터 불필요 — 실제 pose 를 직접 쓴다).
        # 레버암도 실제 (P_i - C) 로 잡으므로 옛 lead/follow_offset_x 는 폐기됨.
        self.declare_parameter('control_hz', 20.0)
        self.declare_parameter('pose_stale_sec', 1.0)
        self.declare_parameter('pos_gain', 0.8)
        self.declare_parameter('yaw_gain', 1.2)
        # 운반 속도명령 상한[m/s]. 0.20 은 들어올린(무거운) 트럭을 밀기엔 낮았다
        # (실측 명령0.20→실제0.03m/s 정체). 속도제어라 목표가 높을수록 바퀴 토크가
        # 커져 부하를 이긴다 — 0.5 로 올려 견인력 확보(튜닝 노브, 더 올려도 됨).
        self.declare_parameter('max_lin', 0.5)
        # 전진 견인 데드밴드 하한[m/s]. TRANSLATE 는 목표 x오차 비례로 감속하는데,
        # 무거운(들어올린) 트럭은 명령이 작으면 바퀴 토크가 부족해 목표 앞에서
        # 정체한다(명령0.2→실제0.03 실측). 목표 밖(|ex|>pos_tol)에서 비례항이 이
        # 값보다 작으면 부호를 지키며 이 최소 명령으로 깔아 견인력을 확보한다.
        # max_lin~0.2 사이에서 하드웨어로 튜닝(더 무거우면 올림). pos_tol 안에선
        # 발동 안 해 정지를 막지 않는다.
        self.declare_parameter('fwd_min', 0.3)
        self.declare_parameter('max_ang', 0.5)
        self.declare_parameter('pos_tol', 0.06)
        self.declare_parameter('yaw_tol', 1.0)
        # 직진 정지 트리거: 차밑 카메라(양 로봇)가 목표 마커를 **같은 거리**로 인식하면 회전.
        self.declare_parameter('marker_match_tol', 0.10)   # |d_lead-d_follow| 이하면 일치[m]
        self.declare_parameter('marker_fresh_sec', 0.5)    # 두 거리 다 이 시간 내여야 유효
        # 직진 중 옆드리프트(슬립) 보정: 마커보정된 /pose 로 중심을 레인 z 에 붙인다.
        # 전진(±V)은 그대로, 옆으로만 살짝. 0 이면 순수 x(보정 끔).
        self.declare_parameter('lat_gain', 1.0)            # 옆보정 게인[1/s]
        self.declare_parameter('lat_max', 0.15)            # 옆보정 속도 상한[m/s]
        self.declare_parameter('goal_timeout_sec', 600.0)   # 저rtf 헤드리스 14m 운반 여유

        gp = lambda n: self.get_parameter(n).value           # noqa: E731
        self.control_hz = float(gp('control_hz'))
        self.pose_stale_sec = float(gp('pose_stale_sec'))
        self.marker_match_tol = float(gp('marker_match_tol'))
        self.marker_fresh_sec = float(gp('marker_fresh_sec'))
        self.lat_gain = float(gp('lat_gain'))
        self.lat_max = float(gp('lat_max'))
        self.fwd_min = float(gp('fwd_min'))
        self.gains = dict(
            pos_gain=float(gp('pos_gain')), yaw_gain=float(gp('yaw_gain')),
            max_lin=float(gp('max_lin')), max_ang=float(gp('max_ang')),
            pos_tol=float(gp('pos_tol')), yaw_tol=float(gp('yaw_tol')))
        self.goal_timeout = float(gp('goal_timeout_sec'))

        self._lock = threading.Lock()
        self._pose = {}     # robot_id -> (x, z, yaw_deg, mono_time)
        self._subs = {}     # robot_id -> pose 구독(지연 생성)
        self._cmd = {}      # robot_id -> cmd_vel 퍼블리셔(지연 생성)
        self._mdist = {}    # robot_id -> (목표마커까지 거리[m], mono_time). 정지 트리거용.

        self._server = ActionServer(
            self, CarryToSlot, 'carry_to_slot', self._execute,
            callback_group=self._cbg,
            goal_callback=lambda _g: GoalResponse.ACCEPT,
            cancel_callback=lambda _g: CancelResponse.ACCEPT)
        self.get_logger().info('carry_action_server 시작 (월드프레임 반평행 중심제어)')

    def _ensure_io(self, rid):
        if rid not in self._cmd:
            self._cmd[rid] = self.create_publisher(Twist, f'/robot_{rid}/cmd_vel', 10)
        if rid not in self._subs:
            self._subs[rid] = self.create_subscription(
                PoseStamped, f'/robot_{rid}/pose',
                lambda m, r=rid: self._on_pose(r, m), 10, callback_group=self._cbg)
            self.create_subscription(
                Float32, f'/robot_{rid}/ref_marker_dist',
                lambda m, r=rid: self._on_mdist(r, m), 10, callback_group=self._cbg)

    def _on_mdist(self, rid, msg):
        with self._lock:
            self._mdist[rid] = (float(msg.data), time.monotonic())

    def _marker_seen(self, lead, follow):
        """차밑 카메라가 타깃 슬롯마커를 **최근에**(marker_fresh_sec 내) 봤는가.
        둘 중 **하나라도** 신선하면 True — 트럭 중심이 그 마커 근처에 왔다는 신호.
        (예전 '양쪽 동시 등거리' 는 두 카메라가 ~2m 떨어져 마커를 보는 시점이 어긋나
        기하학적으로 거의 안 걸렸다. 정지 정밀도는 중심-마커위치 도달로 따로 잡는다.)"""
        now = time.monotonic()
        with self._lock:
            ml, mf = self._mdist.get(lead), self._mdist.get(follow)
        fresh = lambda m: m is not None and now - m[1] <= self.marker_fresh_sec  # noqa: E731
        return fresh(ml) or fresh(mf)

    def _on_pose(self, rid, msg):
        p, q = msg.pose.position, msg.pose.orientation
        yaw_deg = odom_quat_to_yaw_deg(q.x, q.y, q.z, q.w)
        with self._lock:
            self._pose[rid] = (float(p.x), float(p.z), yaw_deg, time.monotonic())

    def _robot_poses(self, lead, follow):
        """캐시된 두 pose -> ((lx,lz,lyaw_rad),(fx,fz,fyaw_rad)). 하나라도 스테일/없음이면
        None(안전정지). 반평행 편성이라 중심수식(center_pose_from_robots)에 안 넣고 개별
        pose 를 그대로 넘긴다 — 제어는 각 로봇 실제 heading 을 써야 한다."""
        now = time.monotonic()
        with self._lock:
            lp, fp = self._pose.get(lead), self._pose.get(follow)
        out = []
        for p in (lp, fp):
            if p is None or now - p[3] > self.pose_stale_sec:
                return None
            out.append((p[0], p[1], math.radians(p[2])))
        return tuple(out)

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
        target = (g.target_x, g.target_z)
        # target_yaw_deg 는 **상대 회전량(delta)** 으로 해석한다(절대각 아님). 위치기반
        # truck_yaw 가 그립 삐뚤어짐에 취약해(실측 트럭은 x정렬인데 -74° 로 오독) 절대
        # 목표(-180)를 쓰면 오독분만큼 오버회전한다. 시작 truck_yaw 를 기준으로 delta 만큼
        # 만 돌면 오독 오프셋이 상쇄돼 실제 90° 회전이 정확히 나온다. rel_ref 잡을 때 확정.
        turn_delta = math.radians(g.target_yaw_deg)
        target_yaw = None
        self.get_logger().info(
            f'carry 시작 lead={lead} follow={follow} target=({g.target_x},{g.target_z}) '
            f'turn_delta={g.target_yaw_deg}° (상대)')

        # 2페이즈 월드프레임 중심제어(반평행 대응):
        #  TRANSLATE: omega=0 순수병진으로 중심을 (target_x,target_z) 로. 방향은 강체그립 유지.
        #  ROTATE   : 위치 유지하며 트럭 yaw 를 target_yaw 로(슬롯 진입용 90° 회전).
        # ROTATE 에서 omega 는 **적응 제한** — 레버암 strafe(omega*r)가 속도예산을 넘으면
        # 메카넘 포화로 편성이 뒤틀리므로(운반 실측), omega ≤ 0.6*max_lin/r_max 로 눌러 천천히.
        gp = self.gains
        pos_gain, yaw_gain = gp['pos_gain'], gp['yaw_gain']
        max_lin, max_ang, pos_tol, yaw_tol = (
            gp['max_lin'], gp['max_ang'], gp['pos_tol'], gp['yaw_tol'])
        yaw_tol_rad = math.radians(yaw_tol)
        phase = 'TRANSLATE'
        reached = 0
        settle_need = 5
        marker_confirmed = False   # 타깃 슬롯마커를 이 carry 중 한 번이라도 봤나(래치)

        period = 1.0 / self.control_hz
        prev = time.monotonic()
        deadline = prev + self.goal_timeout
        result = CarryToSlot.Result()
        tick = 0
        center = None            # 최근 유효 중심(로그/결과용)
        theta = 0.0
        rel_ref = None           # 시작 상대벡터(lp-fp) — 이걸 유지해 truck_yaw 고정

        while rclpy.ok():
            time.sleep(period)
            now = time.monotonic()
            prev = now
            tick += 1
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

            poses = self._robot_poses(lead, follow)   # (lp,fp) each (x,z,yaw_rad) | None
            if poses is None:
                self._stop(lead, follow)              # 스테일 -> 안전 정지
                continue
            lp, fp = poses
            center = ((lp[0] + fp[0]) / 2.0, (lp[1] + fp[1]) / 2.0)   # 참 중점
            theta = formation.truck_yaw_from_robots(lp, fp)          # 트럭 장축(위치기반)
            if rel_ref is None:                                      # 시작 편성 = 유지목표
                rel_ref = (lp[0] - fp[0], lp[1] - fp[1])
                target_yaw = theta + turn_delta                      # 상대: 시작각 + delta

            # ROTATE 용 중심 월드속도(위치 유지). TRANSLATE(순수 x)는 안 씀.
            ex, ez = target[0] - center[0], target[1] - center[1]
            dist = math.hypot(ex, ez)
            vwx, vwz = pos_gain * ex, pos_gain * ez
            vmag = math.hypot(vwx, vwz)
            if vmag > max_lin:
                vwx, vwz = vwx * max_lin / vmag, vwz * max_lin / vmag

            dtheta = ((target_yaw - theta + math.pi) % (2 * math.pi)) - math.pi
            if phase == 'TRANSLATE':
                # 전진(월드 +x)을 목표 x오차 비례로 감속한다 — 옛 등속 ±max_lin 은
                # 목표에서 못 멈추고 그냥 지나쳤다(dist<=pos_tol 5틱이 등속 통과라 안
                # 걸림). v_fwd 는 목표 근처에서 0 으로 줄어 자연 정지한다. 무거운 트럭
                # 견인: 목표 밖(|ex|>pos_tol)인데 비례항이 견인 데드밴드(fwd_min)보다
                # 작으면 부호 지키며 fwd_min 으로 깐다(안 그러면 목표 앞에서 정체).
                # + 마커보정된 /pose 로 옆드리프트만 잡는다(vzc 를 각 로봇 body 로 투영,
                # E-W 편성이라 대부분 y(strafe)). 레인마커로 /pose 가 정확해야 이 보정
                # 이 옳게 먹는다(핵심).
                ex_c = target[0] - center[0]
                v_fwd = max(-max_lin, min(max_lin, pos_gain * ex_c))
                if abs(ex_c) > pos_tol and abs(v_fwd) < self.fwd_min:
                    v_fwd = math.copysign(self.fwd_min, ex_c)
                ez_c = target[1] - center[1]
                vzc = max(-self.lat_max, min(self.lat_max, self.lat_gain * ez_c))
                tl = (-v_fwd + vzc * math.cos(lp[2]), -vzc * math.sin(lp[2]), 0.0)
                tf = (v_fwd + vzc * math.cos(fp[2]), -vzc * math.sin(fp[2]), 0.0)
                # 회전 게이트: 타깃 슬롯마커를 이 carry 중 **한 번이라도 봤고**(래치;
                # 두 카메라가 ~2m 떨어져 마커 위 통과 시점과 중심 도달 시점이 어긋나므로
                # "지금 보임" 대신 "이 접근 중 확인됨"으로 게이트) + 중심이 그 마커
                # 위치(target)에 도달하면 정지·회전. 마커를 아예 못 보면 래치가 안 켜져
                # 정지 안 하고 timeout 안전망까지 직진(마커 게이트 유지).
                if self._marker_seen(lead, follow):
                    marker_confirmed = True
                reached = reached + 1 if (marker_confirmed and dist <= pos_tol) else 0
                if reached >= settle_need:
                    phase = 'ROTATE'
                    reached = 0
                    self.get_logger().info(
                        f"carry: 슬롯마커 인식+중심도달 → 정지·회전 delta={g.target_yaw_deg}° "
                        f"center=({center[0]:.2f},{center[1]:.2f}) dist={dist:.3f}")
                    self._stop(lead, follow)
                    continue
            else:  # ROTATE — 위치 유지 + 트럭을 delta 만큼 회전(적응제한 omega)
                r_max = max(math.hypot(lp[0] - center[0], lp[1] - center[1]),
                            math.hypot(fp[0] - center[0], fp[1] - center[1]), 0.1)
                omega_cap = 0.6 * max_lin / r_max     # 레버암 strafe 가 속도예산 안 넘게
                omega = max(-min(max_ang, omega_cap),
                            min(min(max_ang, omega_cap), yaw_gain * dtheta))
                if dist <= pos_tol and abs(dtheta) <= yaw_tol_rad:
                    reached += 1
                else:
                    reached = 0
                if reached >= settle_need:
                    break
                tl = formation.robot_twist_world((vwx, vwz), omega, lp, center, lp[2])
                tf = formation.robot_twist_world((vwx, vwz), omega, fp, center, fp[2])

            self._pub(lead, tl)
            self._pub(follow, tf)

            fb = CarryToSlot.Feedback()
            fb.phase = phase
            fb.dist_remaining = dist
            goal_handle.publish_feedback(fb)

            # 진단 계측(~1Hz): 페이즈·각 로봇 /pose·중심·트럭yaw·잔여·명령. ponytail: 안정 뒤 삭제 가능.
            if tick % max(1, int(self.control_hz)) == 0:
                self.get_logger().info(
                    f'CARRY_DBG[{phase}] pose[{lead}]=({lp[0]:.2f},{lp[1]:.2f},{math.degrees(lp[2]):.1f}) '
                    f'pose[{follow}]=({fp[0]:.2f},{fp[1]:.2f},{math.degrees(fp[2]):.1f}) '
                    f'center=({center[0]:.2f},{center[1]:.2f}) truck_yaw={math.degrees(theta):.1f} '
                    f'dist={dist:.2f} dyaw={math.degrees(dtheta):.1f} mconf={int(marker_confirmed)} '
                    f'cmd[{lead}]=({tl[0]:.2f},{tl[1]:.2f},{tl[2]:.2f}) '
                    f'cmd[{follow}]=({tf[0]:.2f},{tf[1]:.2f},{tf[2]:.2f})')

        self._stop(lead, follow)
        goal_handle.succeed()
        result.success, result.message = True, 'carry done'
        if center is not None:
            result.final_x, result.final_z = center[0], center[1]
            result.final_yaw_deg = math.degrees(theta)
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
