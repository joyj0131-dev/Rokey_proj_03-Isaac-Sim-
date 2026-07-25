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
