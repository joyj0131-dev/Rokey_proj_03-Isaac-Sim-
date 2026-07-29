#!/usr/bin/env python3
"""carry_action_server — 운반(가상중심 제어) 액션 서버.

2026-07-27 재작성(사용자 지시): follow 단독 제어(발산)를 버리고 **가상중심(virtual
-center) 강체 제어**로 되돌린다. 단, 트럭 자세 측위는 **follow rear 하나로 대표**한다.

측위(follow rear 하나로 트럭 대표):
  차 든 직후 follow /pose yaw 는 90°(x축 평행)로 시딩되고 follow marker_localizer 는
  rear-only 로 바닥 레인마커만 본다. 이 follow /pose(x,z,yaw)가 곧 트럭 뒷축 자세다.
  lead 는 **강체 가정**으로 유도: 시작 시 lead/follow world pose 로 축간거리 L(부호
  포함, 트럭 heading 투영)을 한 번 재고, 이후 매 틱 lead = follow + L·heading(fyaw),
  center = 두 축 중점 = follow + ½L·heading(fyaw). yaw 가 돌면 heading 이 같이 돌아
  lead·center 도 따라 돈다(트럭 강체).

제어(둘 다 cmd_vel — 가상중심):
  center 를 목표(target_x,target_z)로, 트럭 yaw 를 target_yaw 로 몰아가는 **월드 중심
  twist**(v_world 병진 + omega yaw회전)를 세운 뒤, formation.robot_twist_world 로
  follow·lead **각자 heading 에 투영**해 body twist 를 따로 발행한다. 반평행(lead 서향
  /follow 동향)이라도 성립. 슬립으로 자세가 틀어지면 follow rear 가 다시 재면 그 오차가
  v_world·omega 로 자동 반영돼 둘 다 보정된다.
  · **제어 철학(사용자 지시 2026-07-28 재확정)**: 차를 든 뒤 yaw≈yaw_target 이므로,
    **마커 안 보이면 오직 경로방향 body-x 순항**만 낸다(vy=wz=0). 두 로봇이 각자 body-x
    로만 밀고(반평행이라 follow=+, lead=−) 옆·회전은 일절 안 낸다. **yaw·위치(옆) 정렬은
    follow rear 가 마커를 봐서 자기 위치·yaw 를 실제로 알 때만** — 그때만 가상중심
    월드투영으로 along(목표복귀)+cross(경로복귀)+omega(yaw정렬)를 낸다. 마커 없이 odom 만
    믿고 vy/wz 를 내면 슬립 드리프트를 추종하거나 애먼 옆·회전으로 트럭을 흔든다(실측:
    lane 락업·발산). stop 은 마커로 목표 도달(along·perp·yaw)을 확인했을 때만.
  · **미리 멈추지 말 것**: follow rear 근거리 사각 탓에 목표 좌표에 딱 서면 도착 마커가
    안 보이고 서 버린다 → 마커 못 봤으면 **순항(cruise) 으로 계속 전진**해 사각 밖에서
    마커를 찾고(과주행 상한), 한 번 본 뒤엔 지나쳤어도 **후진**해서 마커 좌표에 맞춘다
    (앵커된 pose 로 몰아 사각 순간통과에도 진동·hang 없음). ref_marker_dist 는
    mdist_marker_id=-1(아무 ref 마커나)로 발행해 "봤나" 신호로 쓴다.
  · **회전 선행 게이트**: yaw 오차가 align_gate 보다 크면 along 을 죽이고 회전만
    → 슬롯 앞 90° 회전이 끝나기 전에 남진해 입구를 비스듬히 긁는 걸 막는다.

한 세그먼트가 lane 운반(경로 +x)·슬롯 주차(경로 -z, 회전 후 진입) 둘 다 커버한다.
정지: 마커 측위 중(신선) + 경로상 목표 도달/지남(along) + 경로정렬(perp) + yaw.
축간거리 L 은 첫 세그먼트에서 한 번 재 캐시(트럭 강체 물리상수)해 재사용.
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
from parkbot_motion.pose_controller_node import odom_quat_to_yaw_deg
from parkbot_motion import formation


def lead_center_from_follow(follow_pos, fyaw, l_signed):
    """follow world pose + 트럭 yaw + 부호축간거리 -> (lead_pos, center) world.

    lead·center 는 트럭 heading=(sinψ,cosψ) 위에 있다(둘 다 트럭 장축=센터라인).
    l_signed = (lead_start-follow_start)·heading_start(부호 포함) 한 번 고정.
    """
    hx, hz = math.sin(fyaw), math.cos(fyaw)
    lead = (follow_pos[0] + l_signed * hx, follow_pos[1] + l_signed * hz)
    center = (follow_pos[0] + 0.5 * l_signed * hx, follow_pos[1] + 0.5 * l_signed * hz)
    return lead, center


def carry_translation(e, path_dir, eyaw, marker, confirmed, align_gate, max_lin,
                      pos_gain, fwd_min, pos_tol, cross_max, cruise, overshoot_max):
    """경로기반 중심 병진속도 v_world(x,z). (사용자 지시 2026-07-27/28)

    계획 경로 = 세그먼트 시작중심→목표(직선), path_dir=진행 단위벡터.
    along = 목표까지 경로투영(부호). marker = 현재 마커 신선(방금 봤나), confirmed = 이
    세그먼트에서 한 번이라도 봤나.

    - |eyaw|>align_gate: along 0 (회전 먼저 — 슬롯 앞 90° 회전 클리핑 방지).
    - **마커 한 번도 못 봄(not confirmed)**: 옆 없이 **순항(cruise) 전진**만(마커 찾기,
      과주행 상한).
    - **마커 한 번 봄(confirmed)**: 목표(마커) 좌표로 **정밀 P**(along>0 전진, along<0 후진).
      |along|<=pos_tol 정지. → **직진(along)은 마커 잠깐 잃어도 계속**(경로 마커공백 통과).
    - **cross(경로 수직 이탈 옆보정)는 현재 marker 일 때만**: 마커로 위치를 실제로 재확인한
      순간만 옆으로 잡는다. 마커 없을 땐 안 함 — 없을 때 하면 드리프트하는 odom 을 추종해
      발산한다(슬롯 진입 중 마커 3→65 공백 실측). 마커 없으면 현재 헤딩으로 곧게만.
    """
    px, pz = path_dir
    along = e[0] * px + e[1] * pz
    if abs(eyaw) > align_gate:
        a_speed = 0.0
    elif not confirmed:                          # 마커 탐색: 순항 전진(옆보정 없음)
        return (0.0, 0.0) if along < -overshoot_max else (cruise * px, cruise * pz)
    elif abs(along) <= pos_tol:
        a_speed = 0.0                            # 마커 좌표 도달 → 정지
    else:
        a_speed = max(-max_lin, min(max_lin, pos_gain * along))  # 전진/후진 P
        if abs(a_speed) < fwd_min:
            a_speed = math.copysign(fwd_min, along)              # 견인하한
    vx, vz = a_speed * px, a_speed * pz
    if marker:                                    # 현재 마커 신선할 때만 옆보정
        ex_p = e[0] - along * px
        ez_p = e[1] - along * pz
        perp = math.hypot(ex_p, ez_p)
        if perp > pos_tol:
            c = min(cross_max, pos_gain * perp)
            vx += c * ex_p / perp
            vz += c * ez_p / perp
    return (vx, vz)


def blind_cruise_vx(cruise, fyaw, path_dir, along, confirmed, overshoot_max):
    """마커 없을 때 follow 의 body-x 순항속도[m/s]. **vy=wz=0** (사용자 지시 2026-07-28:
    차를 든 뒤 마커가 안 보이면 위치·yaw 를 신뢰할 수 없으니, 옆·회전 없이 오직
    경로방향으로 x 만 민다). follow heading=(sinψ,cosψ) 와 path_dir 의 내적으로 전진
    부호를 정한다(heading 이 path 와 반대면 후진, 수직이면 0 — 마커 볼 때 omega 로
    회전해야 진행). lead 는 반평행이라 이 값의 부호만 뒤집어 쓴다. confirmed(한 번
    마커 봄) 후 목표를 overshoot_max 이상 지나치면 0(마커 영영 못 봄 안전정지)."""
    if confirmed and along < -overshoot_max:
        return 0.0
    proj = path_dir[0] * math.sin(fyaw) + path_dir[1] * math.cos(fyaw)
    return cruise * proj


def carry_yaw_omega(eyaw, yaw_gain, max_ang, yaw_tol_rad, yaw_min_cmd,
                    r_lever, rot_budget, max_lin):
    """yaw 오차 -> 중심 회전각속도 omega[rad/s]. 비례(yaw_gain)+포화(max_ang)+메카넘
    데드밴드 하한(yaw_min_cmd)+회전strafe 캡(omega·레버암 ≤ rot_budget·max_lin)."""
    omega = max(-max_ang, min(max_ang, yaw_gain * eyaw))
    if abs(eyaw) > yaw_tol_rad and abs(omega) < yaw_min_cmd:
        omega = math.copysign(yaw_min_cmd, eyaw)
    if r_lever > 1e-6:
        cap = rot_budget * max_lin / r_lever
        omega = max(-cap, min(cap, omega))
    return omega


class CarryActionServer(Node):
    def __init__(self):
        super().__init__('carry_action_server')
        self._cbg = ReentrantCallbackGroup()
        self.declare_parameter('control_hz', 20.0)
        self.declare_parameter('pose_stale_sec', 1.0)
        self.declare_parameter('pos_gain', 0.8)
        self.declare_parameter('yaw_gain', 1.2)
        # 운반 전진 속도 상한[m/s]. 무거운(들어올린) 트럭 견인 — 튜닝 노브.
        # 2026-07-27 사용자: 0.5→0.3(미끄러짐↓).
        self.declare_parameter('max_lin', 0.3)
        # 전진 견인 데드밴드 하한[m/s]: 마커 보고 목표에 감속접근할 때 비례속도가 이
        # 값보다 작아도 이 최소명령으로 깔아 바퀴 토크 확보(명령0.2→실제0.03 정체 실측).
        # 감속 여지 위해 0.15(트럭이 이보다 낮은 명령에 정체하면 올릴 것 — 캘리브레이션).
        self.declare_parameter('fwd_min', 0.15)
        # 마커 못 볼 때 순항 전진속도[m/s]: 목표 좌표에 미리 멈추지 않고 이 속도로 계속
        # 가서 follow rear 근거리 사각(~1.1m) 밖으로 나가 마커를 찾는다.
        self.declare_parameter('cruise_speed', 0.2)
        # 과주행 안전상한[m]: 마커 **못 본 채** 목표를 이만큼 지나치면 순항 정지(무한전진
        # 방지). 실측상 목표 0.84m 지나서야 마커를 봤으므로 그보다 넉넉히(마커 보면 즉시
        # 후진 P 로 되돌리니 이 값은 "마커 영영 못 봄" 실패거리일 뿐).
        self.declare_parameter('overshoot_max', 1.2)
        self.declare_parameter('max_ang', 0.5)
        self.declare_parameter('pos_tol', 0.06)
        self.declare_parameter('yaw_tol', 1.0)
        # 메카넘 회전 데드밴드 보정[rad/s]: yaw 오차가 tol 밖인데 비례 wz 가 이 값보다
        # 작으면 이 최소 회전속도로 깐다(mission_control.yaw_min_cmd 와 동형).
        self.declare_parameter('yaw_min_cmd', 0.05)
        # ref_marker_dist 신선도[s]. 이 안에 rear 마커검출이 있으면 "방금 봄"=정렬 지속.
        # 2.0 으로 넓힘(구 0.5): 저RTF(0.2배속)에서 카메라 검출이 벽시계로 띄엄띄엄 와
        # 0.5s 창이 자꾸 만료→정렬이 깜빡 끊겨 "랜덤 정렬"이 됐다(실측). 넓히면 검출
        # 간격을 덮어 정렬이 연속으로 돌아 수렴이 안정적. 융합 /pose 는 그새 odom 으로
        # 유지되니(느린 운반속도) 드리프트 무시가능. RTF 오르면 다시 줄이는 노브.
        self.declare_parameter('marker_fresh_sec', 2.0)
        # 회전 strafe 포화 방지: omega·레버암 ≤ rot_budget·max_lin 로 omega 캡.
        self.declare_parameter('rot_budget', 0.6)
        # 병진 개시 게이트[deg]: yaw 오차가 이 값보다 크면 **병진을 죽이고 제자리 회전
        # 먼저** 한다. 슬롯 진입 전 90° 회전이 끝나기 전에 남진하면 트럭이 슬롯 입구를
        # 비스듬히 긁는 걸 막는다(운반 lane 은 시작부터 yaw≈목표라 즉시 병진).
        self.declare_parameter('align_gate_deg', 8.0)
        # 경로이탈 옆보정 속도 상한[m/s]: 마커로 경로 벗어남 확인 시 옆으로 되돌리는
        # 속도 캡. 전진(max_lin)보다 작게 둬 옆이동이 전진을 압도 않게.
        self.declare_parameter('cross_max', 0.15)
        # 세그먼트 **완료(다음 단계 진행)** 판정 tol — 제어용 pos_tol(0.06)/yaw_tol(1°)보다
        # 헐겁게(사용자 지시 2026-07-28: 완벽 정렬 말고 일정 값 이내면 넘어가라). 너무
        # 빡빡하면 마커 깜빡임·슬립으로 도착판정을 못 채워 다음 단계(슬롯 회전/하강)로
        # 못 넘어간다(실측 CARRY_LANE 헌팅). 완료는 marker_confirmed(이 구간 마커 봤음)+
        # 이 tol 로 판정.
        self.declare_parameter('goal_pos_tol', 0.15)
        # 슬롯 종단 yaw 완료 tol. 4°→15°(2026-07-28 로그 분석): 슬롯에서 마커66 이 rear
        # 근거리 사각(~1.1m)으로 거의 안 잡혀 90° 회전이 ~15° 삐뚤게 끝나는데, 그 잔여 yaw 를
        # 마커 없이 못 줄여 eyaw≤4° 가 영영 안 떠서 슬롯을 지나쳤다(실측 CARRY_SLOT). 15°면
        # along≈0·perp≤0.15 구간에서 at_goal 이 떠 슬롯 안에 멈춘다(약간 삐뚤지만 안 지나침).
        # 근본해결(똑바로 주차)은 슬롯 마커 커버리지/회전 정확도 개선 필요 — 별개 과제.
        self.declare_parameter('goal_yaw_tol_deg', 15.0)
        self.declare_parameter('goal_timeout_sec', 600.0)

        gp = lambda n: self.get_parameter(n).value           # noqa: E731
        self.control_hz = float(gp('control_hz'))
        self.pose_stale_sec = float(gp('pose_stale_sec'))
        self.marker_fresh_sec = float(gp('marker_fresh_sec'))
        self.fwd_min = float(gp('fwd_min'))
        self.yaw_min_cmd = float(gp('yaw_min_cmd'))
        self.rot_budget = float(gp('rot_budget'))
        self.align_gate = math.radians(float(gp('align_gate_deg')))
        self.cross_max = float(gp('cross_max'))
        self.cruise = float(gp('cruise_speed'))
        self.overshoot_max = float(gp('overshoot_max'))
        self.goal_pos_tol = float(gp('goal_pos_tol'))
        self.goal_yaw_tol_rad = math.radians(float(gp('goal_yaw_tol_deg')))
        self.gains = dict(
            pos_gain=float(gp('pos_gain')), yaw_gain=float(gp('yaw_gain')),
            max_lin=float(gp('max_lin')), max_ang=float(gp('max_ang')),
            pos_tol=float(gp('pos_tol')), yaw_tol=float(gp('yaw_tol')))
        self.goal_timeout = float(gp('goal_timeout_sec'))

        self._lock = threading.Lock()
        self._pose = {}     # robot_id -> (x, z, yaw_deg, mono_time)
        self._subs = {}     # robot_id -> pose 구독(지연 생성)
        self._cmd = {}      # robot_id -> cmd_vel 퍼블리셔(지연 생성)
        self._mdist = {}    # robot_id -> (목표 주차앞 마커까지 거리[m], mono_time)
        # 부호축간거리 L: 첫 세그먼트에서 한 번 재고(양쪽 측위 신선할 때) 캐시해
        # 이후 재사용. 트럭 강체라 물리상수 — lane 이동·회전 뒤 lead 측위가 오도로
        # 드리프트해도 여기 값은 안 흔들린다.
        self._axle_l_signed = None

        # 서빙할 액션 이름 — 파라미터화(2026-07-28): 같은 도메인(126)에 출차팀이 이 노드를
        # 복붙해 돌려 이름이 겹치므로, 입차는 launch 에서 'entry_carry_to_slot' 로 준다.
        # 기본값은 하위호환(단독 실행/테스트). orchestrator 의 carry_action 파라미터와 일치해야.
        self.declare_parameter('carry_action_name', 'carry_to_slot')
        carry_action_name = self.get_parameter('carry_action_name').value
        self._server = ActionServer(
            self, CarryToSlot, carry_action_name, self._execute,
            callback_group=self._cbg,
            goal_callback=lambda _g: GoalResponse.ACCEPT,
            cancel_callback=lambda _g: CancelResponse.ACCEPT)
        self.get_logger().info(
            f"carry_action_server 시작 (가상중심 제어, follow rear 대표) action='{carry_action_name}'")

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

    def _marker_seen(self, follow):
        """follow rear 가 목표 주차앞 마커를 최근(marker_fresh_sec 내)에 봤는가."""
        now = time.monotonic()
        with self._lock:
            mf = self._mdist.get(follow)
        return mf is not None and now - mf[1] <= self.marker_fresh_sec

    def _on_pose(self, rid, msg):
        p, q = msg.pose.position, msg.pose.orientation
        yaw_deg = odom_quat_to_yaw_deg(q.x, q.y, q.z, q.w)
        with self._lock:
            self._pose[rid] = (float(p.x), float(p.z), yaw_deg, time.monotonic())

    def _pose_of(self, rid):
        """캐시 pose -> (x, z, yaw_rad) | None(스테일/없음)."""
        now = time.monotonic()
        with self._lock:
            fp = self._pose.get(rid)
        if fp is None or now - fp[3] > self.pose_stale_sec:
            return None
        return (fp[0], fp[1], math.radians(fp[2]))

    def _pub(self, rid, tw):
        m = Twist()
        m.linear.x, m.linear.y, m.angular.z = float(tw[0]), float(tw[1]), float(tw[2])
        self._cmd[rid].publish(m)

    def _stop(self, lead, follow):
        for rid in (lead, follow):
            self._cmd[rid].publish(Twist())

    def _init_axle_dist(self, lead, follow, deadline, goal_handle):
        """시작 시 lead·follow world pose 로 부호축간거리 L 을 한 번 잰다.

        L = (lead-follow)·heading(follow_yaw). follow heading 위 lead 성분(부호 포함).
        pose 신선해질 때까지 대기. 반환 l_signed | None(취소/타임아웃)."""
        while rclpy.ok():
            if goal_handle.is_cancel_requested or time.monotonic() > deadline:
                return None
            fp, lp = self._pose_of(follow), self._pose_of(lead)
            if fp is not None and lp is not None:
                dx, dz = lp[0] - fp[0], lp[1] - fp[1]
                hx, hz = math.sin(fp[2]), math.cos(fp[2])
                return dx * hx + dz * hz
            self._stop(lead, follow)
            time.sleep(1.0 / self.control_hz)
        return None

    def _execute(self, goal_handle):
        g = goal_handle.request
        lead, follow = g.lead_robot_id, g.follow_robot_id
        self._ensure_io(lead)
        self._ensure_io(follow)
        target_x, target_z = g.target_x, g.target_z
        yaw_target = math.radians(g.target_yaw_deg)   # 절대(운반 중 90° 유지)
        # 세그먼트별 과주행 상한(2026-07-28 슬롯 과주행 근본수정):
        #  · 슬롯 종단(target_yaw≈0): 목표 좌표가 곧 도착마커(66) — 지나치면 주차칸을
        #    벗어난다. goal_pos_tol(0.15) **밑**으로 잡아 blind 순항이 z=0 을 살짝만
        #    지나 멈추게 → marker_confirmed(하강 중 마커 봤음)만 latch 돼 있으면 막판
        #    마커66 을 못 봐도 odom 만으로 at_goal 이 떠 그 자리에 선다.
        #  · 레인(target_yaw≈90): 도착마커(slot-lane 3)가 rear 뒤라 **일부러 지나쳐**
        #    되돌아와야 rear 가 본다 → 넉넉한 self.overshoot_max 유지.
        # 레인은 지나쳐 마커 찾기, 슬롯은 지나치면 이탈 — 정반대 요구를 세그먼트로 가른다.
        seg_overshoot = 0.10 if abs(g.target_yaw_deg) < 45.0 else self.overshoot_max
        self.get_logger().info(
            f'carry 시작 lead={lead} follow={follow} target=({target_x},{target_z}) '
            f'yaw_target={g.target_yaw_deg}° overshoot={seg_overshoot:.2f} '
            f'(가상중심, follow rear 대표)')

        gp = self.gains
        pos_gain, yaw_gain = gp['pos_gain'], gp['yaw_gain']
        max_lin, max_ang, pos_tol, yaw_tol = (
            gp['max_lin'], gp['max_ang'], gp['pos_tol'], gp['yaw_tol'])
        yaw_tol_rad = math.radians(yaw_tol)

        period = 1.0 / self.control_hz
        deadline = time.monotonic() + self.goal_timeout
        result = CarryToSlot.Result()

        # 축간거리 L: 캐시 있으면 재사용, 없으면(첫 세그먼트) 한 번 측정 후 캐시.
        if self._axle_l_signed is None:
            l_signed = self._init_axle_dist(lead, follow, deadline, goal_handle)
            if l_signed is None:
                self._stop(lead, follow)
                goal_handle.abort()
                result.success, result.message = False, 'no start pose (lead/follow)'
                return result
            self._axle_l_signed = l_signed
            self.get_logger().info(f'carry 축간거리 L={l_signed:.3f} m 측정·캐시')
        l_signed = self._axle_l_signed
        r_lever = 0.5 * abs(l_signed)                 # 중점에서 각 축까지 레버암

        reached = 0
        settle_need = 5
        marker_confirmed = False   # 이 세그먼트에서 마커를 한 번이라도 봤나(래치)
        path_dir = None            # 세그먼트 시작중심→목표 단위벡터(첫 유효틱에 확정)
        tick = 0
        cx = cz = 0.0
        center_yaw = yaw_target

        while rclpy.ok():
            time.sleep(period)
            now = time.monotonic()
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

            fp = self._pose_of(follow)
            if fp is None:
                self._stop(lead, follow)        # follow pose 스테일 -> 안전 정지
                continue
            fx, fz, fyaw = fp
            center_yaw = fyaw                    # follow rear = 트럭 yaw
            lead_pos, (cx, cz) = lead_center_from_follow((fx, fz), fyaw, l_signed)
            lead_yaw = fyaw + math.pi            # 반평행(트럭 반대편)
            if path_dir is None:                 # 계획 경로 = 시작중심→목표(직선)
                dpx, dpz = target_x - cx, target_z - cz
                dn = math.hypot(dpx, dpz)
                path_dir = (dpx / dn, dpz / dn) if dn > 1e-6 else (0.0, 0.0)

            e = (target_x - cx, target_z - cz)   # 중점→목표 world
            along = e[0] * path_dir[0] + e[1] * path_dir[1]   # 경로방향 남은거리(부호)
            perp = math.hypot(e[0] - along * path_dir[0], e[1] - along * path_dir[1])
            eyaw = ((yaw_target - center_yaw + math.pi) % (2 * math.pi)) - math.pi
            marker = self._marker_seen(follow)   # follow rear 가 ref 마커 방금 봤나(현재 신선)
            marker_confirmed = marker_confirmed or marker

            if marker:
                # ── 마커 보임 = follow 가 자기 위치·yaw 를 실제로 확인 ──
                # 그때만 가상중심 월드투영으로 정밀 보정: along(목표복귀)+cross(경로복귀)
                # +omega(yaw정렬). 두 로봇이 강체로 협조(반평행이라도 성립).
                v_world = carry_translation(
                    e, path_dir, eyaw, True, True, self.align_gate, max_lin,
                    pos_gain, self.fwd_min, pos_tol, self.cross_max, self.cruise,
                    seg_overshoot)
                omega = carry_yaw_omega(
                    eyaw, yaw_gain, max_ang, yaw_tol_rad, self.yaw_min_cmd,
                    r_lever, self.rot_budget, max_lin)
                follow_cmd = formation.robot_twist_world(
                    v_world, omega, (fx, fz), (cx, cz), fyaw)
                lead_cmd = formation.robot_twist_world(
                    v_world, omega, lead_pos, (cx, cz), lead_yaw)
            elif abs(eyaw) > self.align_gate:
                # ── 마커 없지만 **의도된 큰 회전 대기**(슬롯 90° 턴/큰 yaw 드리프트) ──
                # 제자리 회전만(병진 0). 90° 턴은 도는 내내 rear 가 마커를 볼 수 없어
                # (마커3 서→턴중 사각→65/66 남) marker 게이트로는 회전이 시작을 못 한다
                # (실측 CARRY_SLOT 락업). 회전은 0.408 보정된 양이라 odom 으로 돌려도 신뢰.
                # rot_budget=1.0: 순수 회전이라 병진 여유 남길 필요 없음(레버암속도=max_lin).
                omega = carry_yaw_omega(
                    eyaw, yaw_gain, max_ang, yaw_tol_rad, self.yaw_min_cmd,
                    r_lever, 1.0, max_lin)
                follow_cmd = formation.robot_twist_world(
                    (0.0, 0.0), omega, (fx, fz), (cx, cz), fyaw)
                lead_cmd = formation.robot_twist_world(
                    (0.0, 0.0), omega, lead_pos, (cx, cz), lead_yaw)
            else:
                # ── 마커 없고 yaw 정렬됨 → 오직 경로방향 body-x 순항 ──
                # (사용자 지시 2026-07-28: 차 든 뒤 yaw≈yaw_target 이니 그냥 x 로만 밀고,
                #  yaw·위치 정렬은 follow rear 가 마커 봐서 odom 알 때만. vy=wz=0 강제.)
                fvx = blind_cruise_vx(self.cruise, fyaw, path_dir, along,
                                      marker_confirmed, seg_overshoot)
                follow_cmd = (fvx, 0.0, 0.0)
                lead_cmd = (-fvx, 0.0, 0.0)   # 반평행(트럭 반대편) → 반대 body-x
                omega = 0.0

            self._pub(follow, follow_cmd)
            self._pub(lead, lead_cmd)

            # 정지(다음 단계 진행): 이 구간 마커를 봤고(marker_confirmed) 가상중심이
            # 목표에 **헐거운 tol 이내**로 들어오면 완료 — 완벽 정렬을 기다리지 않는다
            # (사용자 지시 2026-07-28). 현재 마커 신선(marker)까지 요구하면 도착 순간
            # 마커가 깜빡여 카운트가 리셋돼 영영 못 넘어간다(실측). marker_confirmed 라
            # 최근 마커로 앵커된 odom 이니 이 tol 이면 충분.
            at_goal = (marker_confirmed and abs(along) <= self.goal_pos_tol
                       and perp <= self.goal_pos_tol
                       and abs(eyaw) <= self.goal_yaw_tol_rad)
            reached = reached + 1 if at_goal else 0
            if reached >= settle_need:
                break

            fb = CarryToSlot.Feedback()
            fb.phase = 'CARRY'
            fb.dist_remaining = abs(along)
            goal_handle.publish_feedback(fb)

            if tick % max(1, int(self.control_hz)) == 0:
                self.get_logger().info(
                    f'CARRY_DBG center=({cx:.2f},{cz:.2f},{math.degrees(center_yaw):.1f}) '
                    f'along={along:.2f} perp={perp:.2f} eyaw={math.degrees(eyaw):.1f} '
                    f'omega={omega:.3f} mark={int(marker)} '
                    f'follow=({follow_cmd[0]:.2f},{follow_cmd[1]:.2f},{follow_cmd[2]:.2f}) '
                    f'lead=({lead_cmd[0]:.2f},{lead_cmd[1]:.2f},{lead_cmd[2]:.2f})')

        self._stop(lead, follow)
        goal_handle.succeed()
        result.success, result.message = True, 'carry done'
        result.final_x, result.final_z = cx, cz
        result.final_yaw_deg = math.degrees(center_yaw)
        return result


def _demo():
    """가상중심 유도·제어 기하 자기검증(노드 없이)."""
    # 트럭 yaw=90°(x평행), follow 서쪽(0,7), lead 동쪽(L=1.7 앞). center=중점.
    L = 1.7
    lead, center = lead_center_from_follow((0.0, 7.0), math.radians(90), L)
    assert abs(lead[0] - 1.7) < 1e-9 and abs(lead[1] - 7.0) < 1e-9, lead
    assert abs(center[0] - 0.85) < 1e-9 and abs(center[1] - 7.0) < 1e-9, center

    # 동쪽 목표로 병진(omega=0): follow(동향+90) 전진(+), lead(서향-90) 후진(-), 둘 다 world +x.
    v_world = (0.5, 0.0)
    fcmd = formation.robot_twist_world(v_world, 0.0, (0.0, 7.0), center, math.radians(90))
    lcmd = formation.robot_twist_world(v_world, 0.0, lead, center, math.radians(90) + math.pi)
    assert fcmd[0] > 0.49 and lcmd[0] < -0.49, (fcmd, lcmd)

    # yaw 회전(omega>0=CW) 강체 검증: body twist 를 world 로 되돌리면 follow·lead 가
    # z 로 **반대** 이동해야(강체 회전). 반평행이라 body vy 부호는 같게 나오는 게 정상.
    def _body_to_world_z(cmd, yaw):
        # world = vx*(sinψ,cosψ) + vy*(cosψ,-sinψ); z 성분만.
        return cmd[0] * math.cos(yaw) + cmd[1] * (-math.sin(yaw))
    fyaw_r, lyaw_r = math.radians(90), math.radians(90) + math.pi
    fr = formation.robot_twist_world((0.0, 0.0), 0.3, (0.0, 7.0), center, fyaw_r)
    lr = formation.robot_twist_world((0.0, 0.0), 0.3, lead, center, lyaw_r)
    assert _body_to_world_z(fr, fyaw_r) * _body_to_world_z(lr, lyaw_r) < 0, (fr, lr)

    # carry_translation: 마커못봄=순항(안멈춤) / 마커봄=감속+cross(후진금지) / 회전선행.
    gate = math.radians(8.0)
    lane = (1.0, 0.0)   # 운반 경로 +x
    P = dict(align_gate=gate, max_lin=0.3, pos_gain=0.8, fwd_min=0.15,
             pos_tol=0.06, cross_max=0.15, cruise=0.2, overshoot_max=1.2)
    ct = lambda e, pd, eyaw, mk, cf: carry_translation(
        e, pd, eyaw, mk, cf, P['align_gate'], P['max_lin'], P['pos_gain'], P['fwd_min'],
        P['pos_tol'], P['cross_max'], P['cruise'], P['overshoot_max'])
    # (1) 마커 못 봄(첫 탐색) + 목표 앞 + z이탈: **순항(0.2) 전진만**, 옆보정 없음.
    vw = ct((1.0, 0.3), lane, 0.0, False, False)
    assert abs(vw[0] - 0.2) < 1e-9 and abs(vw[1]) < 1e-9, vw
    # (2) 현재 마커 봄 + 목표 앞: P(0.3) + cross(z 0.15 되돌림).
    vw = ct((1.0, 0.3), lane, 0.0, True, True)
    assert abs(vw[0] - 0.3) < 1e-9 and abs(vw[1] - 0.15) < 1e-9, vw
    # (2b) ★confirmed 지만 마커 **지금 안 보임** + z이탈: along(0.3) 유지, **cross 안 함**
    #      (마커 공백에서 odom 추종 금지 — 발산 방지). 곧게만 전진.
    vw = ct((1.0, 0.3), lane, 0.0, False, True)
    assert abs(vw[0] - 0.3) < 1e-9 and abs(vw[1]) < 1e-9, vw
    # (3) 현재 마커 봄 + 목표 지남(along=-0.2): **후진**해서 마커 좌표로 되돌림.
    vw = ct((-0.2, 0.0), lane, 0.0, True, True)
    assert abs(vw[0] + 0.16) < 1e-9 and abs(vw[1]) < 1e-9, vw
    # (3b) confirmed + 마커 좌표 도달(|along|<=tol): 정지.
    vw = ct((-0.03, 0.0), lane, 0.0, True, True)
    assert vw == (0.0, 0.0), vw
    # (4) yaw 미정렬(20°>8°): along 죽음(회전 먼저).
    vw = ct((1.0, 0.0), lane, math.radians(20), True, True)
    assert vw == (0.0, 0.0), vw
    # (5a) 마커 못 봄(첫 탐색) + 목표 0.7 지남(< overshoot 1.2): 아직 순항.
    vw = ct((-0.7, 0.0), lane, 0.0, False, False)
    assert abs(vw[0] - 0.2) < 1e-9 and abs(vw[1]) < 1e-9, vw
    # (5b) 마커 못 봄(첫 탐색) + 과주행(1.5 지남 > 1.2): 정지(마커 영영 못 봄 안전).
    vw = ct((-1.5, 0.0), lane, 0.0, False, False)
    assert vw == (0.0, 0.0), vw
    # (6) 슬롯 진입(경로 -z) confirmed(마커 잠깐 없음): along -z 0.3 곧게 남진.
    vw = ct((0.0, -7.0), (0.0, -1.0), 0.0, False, True)
    assert abs(vw[0]) < 1e-9 and abs(vw[1] + 0.3) < 1e-9, vw

    # blind_cruise_vx: 마커 없을 때 follow body-x 순항(vy=wz=0), heading·path 부호.
    # lane yaw=90 path +x: proj=1 → +cruise (follow 전진 동쪽).
    assert abs(blind_cruise_vx(0.2, math.radians(90), (1.0, 0.0), 5.0, False, 1.2) - 0.2) < 1e-9
    # slot 회전 전 yaw=90 path -z: proj=0 → 0 (마커 봐서 omega 로 회전해야 진행).
    assert abs(blind_cruise_vx(0.2, math.radians(90), (0.0, -1.0), 5.0, True, 1.2)) < 1e-9
    # slot 회전 후 yaw=0 path -z: proj=-1 → -cruise (body 후진=−z 하강).
    assert abs(blind_cruise_vx(0.2, 0.0, (0.0, -1.0), 5.0, True, 1.2) + 0.2) < 1e-9
    # confirmed 후 과주행(along<-1.2): 안전정지 0.
    assert blind_cruise_vx(0.2, math.radians(90), (1.0, 0.0), -1.5, True, 1.2) == 0.0

    # carry_yaw_omega: +eyaw→+omega, -eyaw→-omega(대칭), 캡(rot_budget·max_lin/lever) 이내.
    om = carry_yaw_omega(math.radians(90), 1.2, 0.5, math.radians(1), 0.05, 1.78, 1.0, 0.3)
    om2 = carry_yaw_omega(math.radians(-90), 1.2, 0.5, math.radians(1), 0.05, 1.78, 1.0, 0.3)
    assert om > 0 and om2 < 0 and abs(om + om2) < 1e-9, (om, om2)
    assert abs(om - 0.3 / 1.78) < 1e-9, om            # 큰 오차 → 캡(1.0*0.3/1.78) 포화
    # 작은 오차(0.5°<1° tol)도 데드밴드 하한 없이 비례값(캡 이내).
    small = carry_yaw_omega(math.radians(0.5), 1.2, 0.5, math.radians(1), 0.05, 1.78, 1.0, 0.3)
    assert 0 < small < 0.05 + 1e-9, small
    print("carry _demo OK")


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
    import sys
    if len(sys.argv) > 1 and sys.argv[1] == 'demo':
        _demo()
    else:
        main()
