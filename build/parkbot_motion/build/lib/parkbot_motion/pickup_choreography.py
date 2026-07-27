#!/usr/bin/env python3
"""pickup_choreography — R5b: 2로봇 픽업 안무의 순수 시퀀싱 로직 (rclpy 비의존).

``pickup_orchestrator_node.py`` 가 이 모듈의 함수/클래스를 그대로 감싸 ROS2
배관(액션 클라이언트/서버, 콜백)만 붙인다 — ``ingress_control.py``/
``pose_controller.py`` 가 이미 확립한 이 저장소의 관례("순수 로직은 패키지
최상위 모듈에, ROS2 배관은 별도 ``_node.py`` 에")를 그대로 따른다.

설계서 ``docs/superpowers/specs/2026-07-25-ros2-node-refactor-design.md`` §R5.
전제: R4 ``lift_action_server``(taskR4-report.md §3.4, 로봇 2대 동시 호출이
아니면 트럭이 안 들림), R5a ``ingress_node``(taskR5a-report.md §9.5, 회랑
동시점유 로봇-로봇 충돌 위험은 R5b 몫으로 명시 이관됨).
"""
from collections import namedtuple

# 회랑(트럭 밑) 순회 한 단계. phase: 'approach'|'align'|'ingress'.
# trough_index 는 phase=='ingress' 일 때만 의미 있음(그 외 None).
CorridorStep = namedtuple('CorridorStep', ['phase', 'robot_id', 'trough_index'])

# ---- Phase B(도크 스폰→XN 융합주행) 순수 시퀀싱 (R6/T3) ----------------------
#
# 인프로세스 러너(parking_v4_runner.py `_run_entry_lead_b`(2956)/`_run_entry_follow_b`
# (2878)/`_mission_setup`(2715))의 로봇별 6단계를 ROS2 경로용 순수 스텝으로 이식한다.
# 값·순서·근거는 전부 그 러너에서 그대로 가져온다("reuse, don't rebuild").
#
#   seed_dock   : localizer ref_ids=[dock_id] 로 하드필터 + 위치전용 보정 모드.
#                 (러너 `_mission_setup` ⑤: filt 를 도크 좌표로 시딩. ROS2 융합
#                  localizer 는 첫 마커 fix 로 자기 시딩하므로 여기선 ref 전환만.)
#   rotate_90   : 제자리 90° 회전(+X→+Z, filt-yaw 90→0, 북향). **오도 인스턴스**로
#                 실행(단일 전방캠이 회전 중 마커를 잃어 융합만으론 회전 불가 —
#                 R3c §7 실측). 러너 `rotate_in_place(target_yaw=0.0)`.
#   dock_check  : 후방캠 융합, correct_yaw=False, 도크 데칼 위치보정하며 북진.
#                 러너 Step3 `drive_to_pose(rear_ctx, ..., correct_yaw=False)`.
#   xn_align_x  : ref_ids=[xn_id] 로 전환 + 전방캠, x 만 XN 축선(xn_x)으로 정렬
#                 (z 유지). 러너 Step4a `drive_to_pose(front_ctx, (xn_x, cur_z, 0))`.
#   xn_align_z  : x=xn_x 고정한 채 순수 북진해 XN 남쪽 standoff 로(검출창 통과).
#                 러너 Step4b `drive_to_pose((xn_x, xn_z-xn_standoff, 0))`.
#                 (대각 주행 금지 — 검출창 진입 시 횡오차로 n_fix=0 났던 실측 때문에
#                  x 정렬과 z 접근을 분리한다.)
#   offset      : **entry_lead(leader)만** — 충돌회피 x 오프셋(final_x_offset=-1.7).
#                 러너 Step5 `drive_to_pose((xn_x+final_x_offset, xn_z-standoff, 0))`.
#                 entry_follow(follower)는 XN 축선에 정지(오프셋 없음).
PhaseBStep = namedtuple('PhaseBStep', ['phase', 'robot_id'])

# leader(entry_lead) 는 6단계 전체, follower(entry_follow) 는 offset 을 뺀 5단계.
_PHASE_B_LEADER_PHASES = (
    'seed_dock', 'rotate_90', 'dock_check', 'xn_align_x', 'xn_align_z', 'offset')
_PHASE_B_FOLLOWER_PHASES = (
    'seed_dock', 'rotate_90', 'dock_check', 'xn_align_x', 'xn_align_z')


def phase_b_robot_phases(is_leader):
    """한 로봇이 밟을 Phase B 단계 이름 리스트. leader 면 offset 포함(6), follower
    면 offset 제외(5). 오케스트레이터 `_run_phase_b` 가 이 순서를 그대로 실행한다."""
    return list(_PHASE_B_LEADER_PHASES if is_leader else _PHASE_B_FOLLOWER_PHASES)


def phase_b_plan(leader_id, follower_id):
    """스태거링된 Phase B 전체 계획 — **leader(entry_lead) 6단계 전체가 먼저** 끝나고,
    그 다음에야 follower(entry_follow) 5단계가 시작되는 순서로 반환한다.

    **왜 leader 먼저인가**(corridor_plan 은 follower 먼저와 반대): 러너
    `_run_mission_c_choreo`(3128행 부근)가 정확히 이 순서(entry_lead 먼저 완주 →
    entry_follow)를 쓴다 — entry_lead 의 Phase B 종점(XN 서쪽 x 오프셋)이 확정돼야
    entry_follow 가 XN 축선으로 안전하게 들어오고, 두 종점이 x 로 |−1.7|=1.7m 벌어져
    HARD REQUIREMENT(분리 ≥1.5m)를 만족한다. entry_follow 가 먼저 XN 축선에 서면
    아직 대피 안 한 entry_lead 의 접근 대각선과 겹칠 위험이 있다.

    반환: `PhaseBStep(phase, robot_id)` 리스트. leader 6개 + follower 5개, 이 순서.
    """
    steps = [PhaseBStep(p, leader_id) for p in _PHASE_B_LEADER_PHASES]
    steps += [PhaseBStep(p, follower_id) for p in _PHASE_B_FOLLOWER_PHASES]
    return steps


def corridor_plan(leader_id, follower_id, leader_trough_index=0, follower_trough_index=1):
    """스태거링된 전체 안무 계획 -- follower 의 회랑 전체(approach+align+ingress)가
    먼저 끝나고, 그 다음에야 leader 의 회랑 전체가 시작되는 순서로 6단계를 반환한다.

    **왜 ingress 만이 아니라 회랑 전체(approach 포함)를 스태거링하는가**: 두 로봇의
    베이 진입 정렬 목표(APPROACH_X,z_center_truck)가 완전히 같은 지점이고, 그
    지점까지 가는 대각 이동 경로도 같은 좁은 회랑을 지난다 — approach 단계만
    동시에 돌려도 R4 가 실측한 로봇-로봇 충돌(taskR4-report.md §2.5: 두 로봇이
    같은 z=7.075 중심선을 동시에 점유해 실제로 충돌, 오도가 헛돎)과 같은 위험이
    그대로 있다. in-process 기준 구현(``parking_v4_runner.py`` --mission=C,
    3486행 부근 주석: "entry_follow 먼저(...) entry_lead 는 Phase B 종단
    자세가 entry_follow 의 접근 대각선 경로에 너무 가까워 ... 전체가 끝날 때까지
    entry_lead 를 회랑 밖으로 대피")도 정확히 이 순서(follower 전체 -> leader
    전체)를 쓴다 -- 이 함수는 그 관례를 ROS2 경로에 그대로 이식한다.

    반환: ``CorridorStep`` 6개 리스트, follower 3개(approach/align/ingress) +
    leader 3개(approach/align/ingress), 이 순서 그대로.
    """
    def _steps(robot_id, trough_index):
        return [
            CorridorStep('approach', robot_id, None),
            CorridorStep('align', robot_id, None),
            CorridorStep('ingress', robot_id, trough_index),
        ]

    return _steps(follower_id, follower_trough_index) + _steps(leader_id, leader_trough_index)


class CorridorStaggerGuard:
    """회랑(트럭 밑) 동시 점유 방지 -- 명시적 가드(브리프 지시: "Guard it explicitly").

    ``enter(robot_id)`` 는 **다른** 로봇이 아직 ``clear()`` 되지 않은 채 회랑을
    점유하고 있으면 ``RuntimeError`` 를 낸다(같은 로봇이 재진입하는 것은 허용 --
    approach/align/ingress 여러 단계에 걸쳐 점유 상태를 유지하는 정상 사용).
    ``pickup_orchestrator_node`` 는 각 로봇의 회랑 시퀀스 시작 직전(approach
    이전)에 ``enter``, ingress 결과가 확정된 직후(성공/실패 무관 -- 이 시점엔
    로봇이 트럭 밑 자기 축 위치에 멈춰 있어 진입 회랑 자체는 비었다고 본다) 에
    ``clear`` 를 호출해, 두 번째 ingress(또는 approach)가 첫 번째가 "치워졌다"고
    보고하기 전에 시작되지 않게 강제한다.
    """

    def __init__(self):
        self._occupant = None

    def enter(self, robot_id):
        if self._occupant is not None and self._occupant != robot_id:
            raise RuntimeError(
                f'corridor stagger violation: {robot_id} tried to enter the corridor '
                f'while {self._occupant} has not cleared it yet')
        self._occupant = robot_id

    def clear(self, robot_id):
        if self._occupant == robot_id:
            self._occupant = None

    @property
    def occupant(self):
        return self._occupant


def run_concurrent(pairs):
    """``pairs`` = ``[(send, wait), ...]``. 모든 ``send()`` 를 먼저(순서대로,
    루프 한 번에) 호출해 토큰을 전부 모은 **뒤에야** ``wait(token)`` 을 호출한다.

    존재 이유(taskR4-report.md §3.4 실측 그대로): ``ControlLift`` 는 두 로봇에
    "거의 동시에" 보내야 트럭이 실제로 들린다(순차 호출은 실패 모드로 실측
    확인됨 -- 첫 로봇 대기 중 두 번째가 아직 안 눌려 있으면 안 들림). 순차
    ``send()``+``wait()``(로봇1 send, 로봇1 wait 로 블로킹, 그 다음에야 로봇2
    send)이면 로봇1 의 ``ramp_wait_sec`` 대기 동안 로봇2 의 ``lift_cmd`` 발행이
    통째로 늦어져 두 램프가 겹치는 시간이 줄거나 사라질 위험이 있다 -- 이
    함수는 "모든 send 가 모든 wait 보다 먼저"라는 순서를 코드 구조로 강제해
    그 위험을 원천적으로 없앤다.

    반환: ``[wait(token) for ...]``, 입력 ``pairs`` 와 같은 순서.
    """
    tokens = [send() for send, _wait in pairs]
    return [wait(token) for (_send, wait), token in zip(pairs, tokens)]


def lift_both_succeeded(results):
    """두 ``ControlLift`` 결과의 성공여부(bool 리스트/튜플)가 **정확히 2개, 둘 다**
    True 인지.

    taskR4-report.md §3.3: 로봇 1대만으로는 ``success=True`` 가 나와도 트럭이
    안 들린다(0.0000m) -- 길이를 정확히 2(leader+follower)로 고정해, 어느 한쪽
    호출이 누락되거나(리스트 길이 1) 셋 이상이 섞여 들어온 경우(호출부 버그)를
    "성공"으로 오인하지 않는다. 빈 리스트/단일 원소는 성공이 아니다.
    """
    results = list(results)
    return len(results) == 2 and all(bool(r) for r in results)
