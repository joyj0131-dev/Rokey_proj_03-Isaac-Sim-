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

from parking_robot_interfaces.action import CarryToSlot
from parkbot_motion import formation
from parkbot_motion.pose_controller_node import odom_quat_to_yaw_deg

# 2026-07-28 라이브 실측 정지마찰 교착 방지용 최소 명령 크기(§ _execute 사용처
# 주석) — pose_controller.py 의 _MIN_LIN_CMD/_MIN_ANG_CMD 와 같은 근거, 이
# 파일은 독립된 제어루프(PoseController 미사용)라 별도로 정의한다.
_MIN_LIN_WORLD = 0.03   # m/s
_MIN_ANG_ROTATE = 0.05  # rad/s


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
        self.declare_parameter('max_lin', 0.20)
        self.declare_parameter('max_ang', 0.5)
        self.declare_parameter('pos_tol', 0.06)
        self.declare_parameter('yaw_tol', 1.0)
        self.declare_parameter('goal_timeout_sec', 600.0)   # 저rtf 헤드리스 14m 운반 여유

        gp = lambda n: self.get_parameter(n).value           # noqa: E731
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
        self.get_logger().info('carry_action_server 시작 (월드프레임 반평행 중심제어)')

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
        target_yaw = math.radians(g.target_yaw_deg)
        self.get_logger().info(
            f'carry 시작 lead={lead} follow={follow} target=({g.target_x},{g.target_z}) '
            f'target_yaw={g.target_yaw_deg}')

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

        period = 1.0 / self.control_hz
        prev = time.monotonic()
        deadline = prev + self.goal_timeout
        result = CarryToSlot.Result()
        tick = 0
        center = None            # 최근 유효 중심(로그/결과용)
        theta = 0.0

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

            # 월드 위치오차 -> 중심 월드속도(크기 max_lin 클램프). 두 페이즈 공통(ROTATE 도
            # 위치 유지).
            ex, ez = target[0] - center[0], target[1] - center[1]
            dist = math.hypot(ex, ez)
            vwx, vwz = pos_gain * ex, pos_gain * ez
            vmag = math.hypot(vwx, vwz)
            if vmag > max_lin:
                vwx, vwz = vwx * max_lin / vmag, vwz * max_lin / vmag
                vmag = max_lin
            # 2026-07-28 라이브 실측: pose_controller.py 에서 발견한 것과 같은 종류의
            # 정지마찰 교착 — dist 가 pos_tol 바로 위로 좁혀지면 P 출력(vmag)이 실제
            # 휠 정지마찰보다 작아져 두 로봇이 사실상 안 움직인 채 무한정 남는다("리프트
            # 하고 뒤로 갔다가 멈췄다"). 아직 도달 전(dist>pos_tol)에만 최소 크기를
            # 강제한다 — 도달 후(다음 틱 vwx/vwz=0 목표)는 영향 없음.
            if dist > pos_tol and 0.0 < vmag < _MIN_LIN_WORLD:
                vwx, vwz = vwx * (_MIN_LIN_WORLD / vmag), vwz * (_MIN_LIN_WORLD / vmag)

            dtheta = ((target_yaw - theta + math.pi) % (2 * math.pi)) - math.pi
            if phase == 'TRANSLATE':
                # 2026-07-28 라이브 실측(두 번째 교착): 위 along 판정 덕에 조기종료는
                # 막았지만, 장거리(예 회랑 서진 18m) 동안 실제 메카넘 슬립으로 로봇
                # yaw 가 서서히 틀어지며(수십 초에 수십 도) heading 축과 수직인 잔여
                # 오차가 누적된다 — along 은 여전히 "아직 미도달"로 정확히 판정하지만,
                # 남은 오차 대부분이 heading 과 거의 직각이 돼 클램프된 세계속도의
                # 투영값(along 성분)이 다시 정지마찰 이하로 잦아든다(실측:
                # dist=1.78 에서 cmd≈0.02 로 고착 — 위 _MIN_LIN_WORLD 플로어는 클램프
                # *전* 세계속도 크기만 보므로 이 경우엔 못 걸린다). 근본 원인은 heading
                # 자체가 틀어진 것이므로, target_yaw 로 향하는 아주 약한 회전 보정을
                # 직진 중에도 계속 걸어 슬립이 쌓이기 전에 막는다.
                #
                # 예전에 강한 omega/strafe 보정이 로봇을 180도 뒤집어 폭주시킨 전례
                # (§ 아래 tl/tf 주석)는 **반평행**(lead 서향/follow 동향) 편성 얘기다 —
                # 지금 이 미션은 두 로봇이 항상 **평행**(같은 방향)이고, omega 는
                # 강체로 공유돼 개별 로봇이 따로 안 돈다(§ formation.robot_twist_world
                # — 반환하는 wz 가 그대로 omega). 그 위험이 구조적으로 없다.
                #
                # **2026-07-28 두 번째 실측(라이브 스크린샷 — 트럭이 대각선으로 틀어진
                # 채 정지)**: 최초 시도(상한 0.15*max_ang, 게인 0.4*yaw_gain)는 너무
                # 약했다 — 18m 를 계속 전진하며 쌓이는 슬립 속도를 이 보정이 못
                # 따라잡았다. ROTATE 전용 상한(omega_cap≈0.6*max_lin/r_max, 실측
                # 0.06~0.08rad/s)이 -37°/-76° 같은 큰 오차도 몇십 초 안에 확실히
                # 되돌리는 걸 이미 봤으므로, TRANSLATE 보정도 그와 비슷한 크기까지
                # 올린다(순수 회전이 아니라 전진과 동시라 오히려 더 강해야 슬립을
                # 따라잡는다) — 게인도 거의 낮추지 않는다.
                trans_ang_cap = 0.6 * max_ang
                omega = max(-trans_ang_cap, min(trans_ang_cap, 0.9 * yaw_gain * dtheta))
                # 2026-07-28 라이브 실측 교착: TRANSLATE 는 각 로봇이 **자기 heading
                # 축으로만** 밀 수 있다(vy 강제 0, § 아래 tl/tf 처리) — 그 축과 수직인
                # 잔여오차는 원리적으로 못 줄인다. dist(전체 오차)로 도달을 판정하면
                # 축-수직 잔여오차가 남아있는 한 영원히 도달 못 하고 멈춘다(실측:
                # dist=0.20 에서 40초+ 고정, cmd≈0 — 세계속도 자체는 0.16m/s 로 충분히
                # 컸지만 로봇 heading 이 오차방향과 거의 직각이라 투영값만 0 근처였다).
                # lead 의 yaw 를 기준 축으로 오차를 투영해, 그 축 성분만으로 판정한다
                # (반평행 편성이어도 follow 축은 같은 직선의 반대부호일 뿐이라 lead
                # 하나만 써도 안전 — abs() 를 쓰므로 부호 무관). 축과 수직인 잔여오차는
                # vy 를 허용하는 ROTATE 로 넘겨 거기서 마저 잡는다(§ 아래 else 분기 —
                # ROTATE 는 TRANSLATE 와 달리 tl/tf 를 그대로 내보내 vy 가 살아있다).
                along = ex * math.sin(lp[2]) + ez * math.cos(lp[2])
                reached = reached + 1 if abs(along) <= pos_tol else 0
                if reached >= settle_need:
                    phase = 'ROTATE'
                    reached = 0
                    self.get_logger().info(
                        f'carry: 직진 완료 center=({center[0]:.2f},{center[1]:.2f}) '
                        f'→ 회전 시작 truck_yaw={math.degrees(theta):.1f}→{g.target_yaw_deg}')
            else:  # ROTATE — 위치 유지 + 트럭 yaw 를 target 으로(적응제한 omega)
                r_max = max(math.hypot(lp[0] - center[0], lp[1] - center[1]),
                            math.hypot(fp[0] - center[0], fp[1] - center[1]), 0.1)
                omega_cap = 0.6 * max_lin / r_max     # 레버암 strafe 가 속도예산 안 넘게
                eff_ang_cap = min(max_ang, omega_cap)
                omega = max(-eff_ang_cap, min(eff_ang_cap, yaw_gain * dtheta))
                # § 위 TRANSLATE 최소속도와 동일 근거 — dtheta 가 남아 있는데 omega_cap
                # 자체가 이미 작아(레버암이 길수록 더 작아짐) 실질적으로 못 도는 교착을
                # 막는다. 안전상한(omega_cap/max_ang)은 절대 넘지 않는다(min 으로 이중 보호).
                if abs(dtheta) > yaw_tol_rad:
                    min_omega = min(_MIN_ANG_ROTATE, eff_ang_cap)
                    if 0.0 < abs(omega) < min_omega:
                        omega = math.copysign(min_omega, omega)
                ok_pose = dist <= pos_tol and abs(dtheta) <= yaw_tol_rad
                reached = reached + 1 if ok_pose else 0
                if reached >= settle_need:
                    break

            tl = formation.robot_twist_world((vwx, vwz), omega, lp, center, lp[2])
            tf = formation.robot_twist_world((vwx, vwz), omega, fp, center, fp[2])
            if phase == 'TRANSLATE' and dist > pos_tol:
                # 2026-07-28 라이브 실측(재발 방지 백스톱): 위 세계속도 플로어
                # (_MIN_LIN_WORLD)는 클램프 *전* 세계속도 크기만 본다 — 로봇 heading
                # 이 오차방향과 거의 직각이면 세계속도는 충분히 커도 **투영 후**
                # 로봇별 실제 전진명령(tl[0]/tf[0])만 정지마찰 이하로 작아질 수
                # 있다(실측: 세계속도 0.20m/s 인데도 투영 후 0.02 로 고착). 위
                # omega 보정이 이런 헤딩 틀어짐 자체를 앞으로는 막아야 하지만,
                # 혹시 남는 경우를 대비해 투영 *후* 값에도 같은 최소치를 건다.
                if 0.0 < abs(tl[0]) < _MIN_LIN_WORLD:
                    tl = (math.copysign(_MIN_LIN_WORLD, tl[0]), tl[1], tl[2])
                if 0.0 < abs(tf[0]) < _MIN_LIN_WORLD:
                    tf = (math.copysign(_MIN_LIN_WORLD, tf[0]), tf[1], tf[2])
            if phase == 'TRANSLATE':
                # strafe(vy)는 계속 제거 — 반평행 편성에서 yaw 드리프트를 유발한
                # 전례(실측 follow 30° 흘러 대각선·0.03m/s 정체) 그대로 유지한다.
                # omega 는 더 이상 0 으로 죽이지 않는다(§ 위 계산부 주석 — 강체공유
                # 회전이라 반평행때의 "로봇이 트럭밑에서 헛돌림·180도 뒤집힘" 위험이
                # 구조적으로 없고, 낮은 상한의 약한 보정만 흐른다). 각 로봇이 자기
                # heading 축으로 미는 힘이 여전히 추력의 대부분이고, omega 는 그
                # heading 자체가 슬립으로 틀어지지 않게 붙잡는 역할만 한다.
                tl = (tl[0], 0.0, tl[2])
                tf = (tf[0], 0.0, tf[2])
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
                    f'dist={dist:.2f} dyaw={math.degrees(dtheta):.1f} '
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
