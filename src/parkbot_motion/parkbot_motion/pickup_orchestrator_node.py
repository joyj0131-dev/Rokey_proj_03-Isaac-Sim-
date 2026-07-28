#!/usr/bin/env python3
"""pickup_orchestrator_node — R5b: 2로봇 픽업 안무를 ``ExecuteParkingTask`` 액션으로.

설계서 ``docs/superpowers/specs/2026-07-25-ros2-node-refactor-design.md`` §3~4(R5).
전제: R3c ``pose_controller_node``(``NavigateToPose``, taskR3c-report.md),
R4 ``lift_action_server``(``ControlLift``, taskR4-report.md), R5a ``ingress_node``
(``IngressUnderTruck``, taskR5a-report.md).

## 이 파일을 새로 만든 이유 — ``robot_task_orchestrator.py``(``parking_robot_system``)
## 를 확장하지 않은 이유

그 파일도 이미 ``ExecuteParkingTask`` 액션서버(``execute_parking_task``)이지만,
아래 세 가지 이유로 "확장"이 아니라 사실상 "전면 재작성"이 될 것이라 판단해
(R3b 가 레거시 ``navigate_action_server``/``align_action_server`` 를 놔두고
``parkbot_motion`` 에 새 v4 브리지 전용 노드를 만든 것과 동일한 판단, 그
전례 그대로) 새 파일로 분리했다:

1. **다른 백엔드**: 그 파일의 액션 클라이언트(``DetectVehicle``/``AlignVehicle``/
   전역 ``navigate_to_pose``(``formation_motion`` 기반, ``map`` 프레임, 로봇 1쌍을
   한 몸처럼 다루는 편대 이동)/전역 ``control_lift``)는 전부 레거시(v2 계열,
   ``parking_robot_system``) 백엔드용이다. R5b 가 조합해야 하는 건 R2~R5a 가
   새로 만든 **로봇별** 네임스페이스 액션(``/robot_<id>/navigate_to_pose``,
   ``/robot_<id>/ingress_under_truck``, ``/robot_<id>/control_lift``, 전부
   ``parkbot_motion``) — 액션 타입도, 목표 좌표계(USD 월드 그대로 vs ``map``)도,
   로봇 1대/1쌍 단위 구분도 전부 다르다. 클라이언트를 갈아 끼우는 정도가 아니라
   상태머신 자체를 다시 짜야 한다.
2. **다른 범위**: 그 파일의 8단계(SEARCHING..RETURNING)는 탐지+픽업+슬롯까지
   운반+안착+복귀까지 포함하는 전체 생애주기다. R5b 브리프는 명시적으로 베이
   진입~픽업(안착 이전까지)만 범위로 하고 Phase B(도크~XN~베이)/운반/주차/복귀는
   범위 밖으로 뺐다 — "확장"하면 그 파일의 기존 계약(및 그걸 테스트하는
   ``test_orchestrator.py``)을 붕괴시키지 않으면서 부분집합만 골라 쓰기가
   어렵다.
3. **패키지 응집**: R2~R5a 는 v4 브리지 전용 노드를 전부 ``parkbot_motion`` 에
   모아왔다(``pose_controller_node``/``axle_detector_node``/``lift_action_server``/
   ``ingress_node``, 각 파일 docstring 이 이미 이 관례를 명시). 이 오케스트레이터가
   호출하는 세 백엔드가 전부 그 패키지 안에 있으므로, 오케스트레이터도 같은
   곳에 두는 게 일관적이다(``robot_task_orchestrator.py`` 도 자신이 부리는
   레거시 백엔드와 같은 패키지 ``parking_robot_system`` 에 있다 -- 그 대칭을
   반대쪽에도 그대로 적용).

**R6 을 위한 재조정 메모**: 레거시 파일을 건드리지 않았으므로 그쪽 회귀 위험은
없다. 두 오케스트레이터가 액션 이름을 공유하면 ROS 그래프에서 서로 밟으므로,
이 노드는 별개 액션 이름(기본 ``execute_pickup_choreography``)을 쓴다(§ 아래
``action_name`` 파라미터). 두 orchestrator 를 최종적으로 합칠지(레거시
APPROACHING/PICKED_UP 단계 내부에서 이 안무를 호출하는 식)는 후속 단계 판단
(설계서 R6 "러너에서 미션 코드 제거·문서 갱신"이 그 판단을 다루기 좋은 시점).

## 안무(2026-07-27 재안무: Phase B 동시정렬 → 동시진입 → 리프트 → 운반 → 안착)

0. **Phase B 동시 정렬**(``_run_phase_b`` ×2 스레드): 도크 스폰 → 90° 회전 →
   회랑마커 중앙정렬 → 최종마커 yaw 정렬. 종단에 lead 서향(-90°)/follow 동향
   (+90°), 둘 다 1° 이내. 상세는 ``pickup_choreography._PHASE_B_PHASES``.
1. **동시 진입**(``_ingress`` ×2 스레드, 배리어): 두 로봇이 **동시에** world -x 로
   트럭 밑 진입 — lead 는 서향이라 전진, follow 는 동향이라 후진(``ingress_node``
   ``drive_sign`` 파라미터가 vx 부호를 처리; 런처가 follow 에 -1 배선).
   ``/robot_<id>/ingress_under_truck``(trough_index)로 lead 는 2번째(front axle,
   더 깊이)·follow 는 1번째(rear axle) 트로프에서 측면 뎁스캠 중앙유지하며 정지.
   lead 가 먼저 출발+더 깊은 축이라 항상 follow 를 앞서 같은 z 중심선이어도
   충돌 없음(옛 stagger guard 불필요). 어느 한쪽 진입 실패면 태스크 ``FAILED``
   (로봇이 트럭 밑 정위치에 없다는 뜻이라 뒤이은 리프트가 위험하다).
2. **동시 리프트**(브리프 필수 지시): 두 진입이 모두 끝난 뒤,
   ``pickup_choreography.run_concurrent`` 로 ``/robot_<id>/control_lift``
   UP 을 **두 로봇에 순서대로 send, 그 다음에야 순서대로 wait**(모든 send 가
   모든 wait 보다 먼저 -- taskR4-report.md §3.4: 순차 send+wait 이면 트럭이
   안 들리는 실패모드가 실측 재현됨) 호출한다. ``pickup_choreography.
   lift_both_succeeded`` 로 "정확히 둘 다 성공"만 전체 성공으로 인정한다.

## 동시성 패턴 -- ``robot_task_orchestrator.py`` 와 동일 교정 패턴(재사용, 재발명 아님)

이 노드도 액션 서버(``execute_pickup_choreography``)이면서 그 콜백 안에서 여러
액션 클라이언트를 순차/동시 호출·대기한다 -- 재진입 spin 데드락 위험이 똑같이
있다. ``robot_task_orchestrator.py`` 가 이미 검증한 해법을 그대로 가져온다:
액션서버+모든 클라이언트를 같은 ``ReentrantCallbackGroup`` 에 두고,
``spin_until_future_complete`` 류의 재진입 spin 없이 ``time.monotonic()``
데드라인 + ``sleep(POLL_INTERVAL)`` non-respin 폴링으로 future 를 기다리며,
``main()`` 은 ``MultiThreadedExecutor`` 로 스핀한다.
"""
import math
import threading
import time

import rclpy
from action_msgs.msg import GoalStatus
from rcl_interfaces.srv import SetParameters
from rclpy.action import ActionClient, ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from nav2_msgs.action import NavigateToPose
from std_msgs.msg import Bool

from parking_robot_interfaces.action import (
    CarryToSlot, ControlLift, ExecuteParkingTask, IngressUnderTruck)
from parkbot_motion.pickup_choreography import (
    lift_both_succeeded, phase_x_plan, phase_x_robot_phases, run_concurrent,
)
# phase_b_plan/phase_b_robot_phases(입차) 는 주석처리된 Phase B 코드가 되돌아올 때
# 다시 import 할 것 — 지금은 미사용.

# ---- 동시성 패턴 상수(robot_task_orchestrator.py 와 동일 관례) ----
POLL_INTERVAL = 0.05
# taskR5bfix 진단: entry_follow approach 는 t=856(pose_controller "목표 수락"
# 로그) 에 이미 수락됐는데, 오케스트레이터의 send_goal_async future 는 10s 뒤인
# t=866 까지도 done() 이 안 됐다 -- 헤드리스 RTF≈0.5 + 여러 노드가 붐비는
# DDS/executor 스케줄 지연이 accept 콜백 배달 자체를 10s 넘게 늦춘 실측
# 사례(구조적 데드락이 아니라 지연). 5~10s 는 이 머신에서 근본적으로 너무
# 타이트 -- 넉넉히 키운다.
ACTION_SERVER_WAIT_TIMEOUT = 15.0
SEND_GOAL_TIMEOUT = 45.0

# 결과 대기 상한 -- 하위 서버 자체 goal_timeout_sec(bringup_pickup_e2e.sh 가
# 실전 기동 시 오버라이드하는 값, RTF 저하 여유 반영) + 오케스트레이터측 폴링/
# 콜백 배달 지연 여유. bringup_pickup_e2e.sh GOAL_TIMEOUT 기본 300.0s(둘 다
# pose_controller_node/ingress_node 에 -p goal_timeout_sec 로 전달) -- 이 값
# 미만으로 잡으면 하위 서버가 정상 진행 중인데도 오케스트레이터가 먼저
# 결과타임아웃으로 실패 처리해버린다(§ 위 SEND_GOAL_TIMEOUT 진단과 같은 종류의
# 실패모드). robot_task_orchestrator.py NAVIGATE_RESULT_TIMEOUT(330.0, 레거시
# formation_motion CARRY_TO_TIMEOUT=300 + 여유)과 동일 여유폭 관례를 맞춘다.
NAVIGATE_RESULT_TIMEOUT = 330.0
# ingress_node.goal_timeout_sec 도 bringup 이 300.0 으로 올려 기동한다(위와
# 동일 근거) -- approach/align 보다 실제 이동거리가 길어(왕복 최대 ~13m,
# taskR5a-report.md §5.2) 실행시간도 분 단위로 걸릴 수 있음(브리프 "ingress
# minutes").
INGRESS_RESULT_TIMEOUT = 550.0   # 진입 서버 goal_timeout(500)보다 커야 클라가 먼저 안 포기
CARRY_RESULT_TIMEOUT = 600.0    # 14m 운반(저RTF 헤드리스)·carry goal_timeout(600)과 맞춤
# lift_action_server.ramp_wait_sec -- bringup 이 10.0s 로 올려 기동한다(기본
# 3.0s 는 RTF 저하 미반영) -- 그 위에 discovery/폴링 여유를 넉넉히.
LIFT_RESULT_TIMEOUT = 30.0


def _navigate_goal(clock, x, z, yaw_deg):
    """USD 월드 프레임 그대로의 ``NavigateToPose`` 목표(``pose_controller_node``
    계약, taskR3b-report.md: ``goal.pose`` 를 ``map`` 이 아니라 USD 그대로 해석).
    yaw 는 순수 월드 Y축 회전 쿼터니언으로 인코딩(``navigate_smoke_fused.py`` 와
    동일 공식)."""
    goal = NavigateToPose.Goal()
    goal.pose.header.frame_id = 'usd_world'
    goal.pose.header.stamp = clock.now().to_msg()
    goal.pose.pose.position.x = float(x)
    goal.pose.pose.position.z = float(z)
    yr = math.radians(yaw_deg)
    goal.pose.pose.orientation.y = math.sin(yr * 0.5)
    goal.pose.pose.orientation.w = math.cos(yr * 0.5)
    return goal


class _AutoRequest:
    """auto_start 자율실행용 최소 request 대역(ExecuteParkingTask.Goal 필드만)."""
    def __init__(self, leader, follower, task_id):
        self.leader_robot_id = leader
        self.follower_robot_id = follower
        self.task_id = task_id


class _AutoGoalHandle:
    """auto_start 자율실행이 ``_on_execute_parking_task`` 를 **그대로 재사용**하게
    하는 최소 goal_handle 대역. 액션 서버 goal 없이도 같은 안무 코드를 돌린다 —
    피드백은 로그로, abort/succeed 는 무시(자율 실행이라 액션 결과 대상이 없음).
    """
    def __init__(self, request, logger):
        self.request = request
        self._logger = logger

    def publish_feedback(self, fb):
        self._logger.info(f'[auto] {fb.current_step} progress={fb.progress:.2f}')

    def abort(self):
        pass

    def succeed(self):
        pass


class PickupOrchestratorNode(Node):

    def __init__(self):
        super().__init__('pickup_orchestrator_node')

        self.declare_parameter('action_name', 'execute_pickup_choreography')
        # 2026-07-27 재안무: lead 는 2번째로 뎁스변하는 바퀴(=front axle, 더 깊이),
        # follow 는 1번째(=rear axle). 둘 다 -x 로 진입하므로 index 0=먼저 만나는
        # (높은 x) 후축, index 1=다음(낮은 x) 전축. lead 가 먼저 출발+더 깊은 축이라
        # 항상 follow 를 앞서 같은 z 중심선에서도 충돌 없음.
        self.declare_parameter('leader_trough_index', 1)
        self.declare_parameter('follower_trough_index', 0)
        # 로봇별 액션 이름 접미사 -- ROS2 배관 컨벤션(§ 클래스 docstring "회전
        # 포즈소스 전환 메커니즘"). odom 인스턴스는 pose_controller_node 기본값과
        # 일치해야 별도 -p action_name 없이도 바로 붙는다.
        self.declare_parameter('navigate_odom_action', 'navigate_to_pose')
        # 2026-07-27 여덟 번째 재설계 — 아루코 마커 인식을 로봇 동작에서 제거(사용자
        # 지시: "일정이 안된다, 그냥 GT로 빠르게 해줘" / "마커는 바닥에 그대로 두고
        # 인식만 빼자"). 모든 이동을 'nav_fused'(마커 융합) 대신 'nav_odom'으로
        # 돌린다 — sim_bridge 를 ``--odom=gt``로 띄우면 /odom 자체가 GT 라 오도
        # 드리프트 걱정 없이 즉시 정확하다. ``navigate_fused_action``/
        # ``_set_localizer_ref`` 는 코드에 남겨뒀지만(마커 노드 자체는 계속 돌며
        # 바닥 마커도 유지) 더 이상 어떤 내비게이션 호출에서도 쓰이지 않는다.
        self.declare_parameter('navigate_fused_action', 'navigate_to_pose_fused')
        self.declare_parameter('ingress_action', 'ingress_under_truck')
        # Phase D 운반: carry_action_server 액션 + 목표 슬롯 pose(우리 (x,z,yaw) 규약).
        # 기본 A2'(6.2, 6.875) — UI gateway 연동 전엔 이 파라미터로 목표 지정.
        # ponytail: goal.slot_pose(Pose) 매핑은 UI gateway 증분에서.
        self.declare_parameter('carry_action', 'carry_to_slot')
        # ---- 입차 Stage 4(슬롯 주차) 파라미터 — 2026-07-27 출차 설계로 주석처리.
        # 되돌릴 때 이 블록과 _on_execute_parking_task 의 "CARRY_LANE/CARRY_SLOT"
        # 두 단계를 함께 복원할 것(park_* 는 그 두 단계에서만 쓰인다).
        # self.declare_parameter('park_slot_x', 2.8)
        # self.declare_parameter('park_lane_z', 7.075)
        # self.declare_parameter('park_center_z', 0.0)
        # self.declare_parameter('park_truck_yaw_deg', -180.0)
        # ---- 출차대기구역(W_IN 베이) 운반 목표 — 2026-07-27 신규.
        # 베이 좌표는 marker_map_v4.json "W_IN"(id50, x=-8.5,z=-7.075) 실측 그대로
        # (spawn_handoff_vehicle() 이 쓰던 것과 같은 베이 — 이제 트럭 A3 이전으로
        # 비어 있어 되돌려 놓을 자리로 재사용). 베이 Pad 는 긴 축이 x(scale 6.4×3.2)
        # 이므로 목표 yaw 는 A3 의 z축-정렬(길이=z, yaw=180)에서 다시 x축-정렬로
        # 90° 되돌린다 — spawn_handoff_vehicle() 이 쓰던 부호 관례(front 가 -x 쪽)
        # 를 그대로 따라 -90 을 기본값으로 둔다(캐리서버 실제 yaw 부호는 미검증 —
        # 첫 설계, 실측 후 조정).
        self.declare_parameter('exit_wait_x', -8.5)
        self.declare_parameter('exit_wait_z', -7.075)
        self.declare_parameter('exit_wait_truck_yaw_deg', -90.0)
        self.declare_parameter('lift_action', 'control_lift')

        # ---- Phase B(입차, 도크→XN 융합주행) 레그 파라미터 — 2026-07-27 출차 설계로
        # 주석처리(§ 클래스 docstring, 되돌릴 때 이 블록 + _phase_b_params/_run_phase_b
        # + _on_execute_parking_task 의 "Phase B 동시 시작" 블록을 함께 복원할 것).
        # self.declare_parameter('run_phase_b_first', True)
        # self.declare_parameter('phase_b_xn_id', 31)
        # self.declare_parameter('phase_b_xn_x', -3.2)
        # self.declare_parameter('phase_b_xn_z', 7.075)
        # self.declare_parameter('phase_b_xn_standoff', 1.3)
        # self.declare_parameter('phase_b_dockcheck_standoff', 1.4)
        # self.declare_parameter('phase_b_final_x_offset', -1.7)
        # self.declare_parameter('phase_b_align_pos_tol', 0.06)
        # self.declare_parameter('phase_b_leader_dock_id', 21)
        # self.declare_parameter('phase_b_leader_dock_x', -3.2)
        # self.declare_parameter('phase_b_leader_dock_z', 2.2)
        # self.declare_parameter('phase_b_leader_dock_decal_z', 3.8)
        # self.declare_parameter('phase_b_follower_dock_id', 23)
        # self.declare_parameter('phase_b_follower_dock_x', -1.2)
        # self.declare_parameter('phase_b_follower_dock_z', 2.2)
        # self.declare_parameter('phase_b_follower_dock_decal_z', 3.8)
        # self.declare_parameter('phase_b_leader_localizer_node', '')
        # self.declare_parameter('phase_b_follower_localizer_node', '')
        # self.declare_parameter('phase_b_leader_corridor_id', 31)
        # self.declare_parameter('phase_b_follower_corridor_id', 63)
        # self.declare_parameter('phase_b_leader_final_id', 62)
        # self.declare_parameter('phase_b_leader_final_yaw', -90.0)
        # self.declare_parameter('phase_b_follower_final_id', 64)
        # self.declare_parameter('phase_b_follower_final_yaw', 90.0)

        # ---- Phase X(출차, 도크→남측 회랑→A3 접근) 레그 파라미터 — 2026-07-27 신규 ----
        # 사용자 설계 그대로: 도크 스폰(yaw=90°동향) → ①제자리 90° 회전(남향 180°) →
        # ②남진하며 남측 회랑마커(z=-7.075) 인식 → ③제자리 90° 회전(동향 90°) →
        # ④마커를 보며 동진해 A3 열(x=9.6)까지. 좌표는 전부 marker_map_v4.json 실측값
        # (build_marker_map_v4.py 로 생성, 손으로 베끼지 않음 — 이 파일 도입 시 확인).
        self.declare_parameter('run_phase_x_first', True)
        self.declare_parameter('phase_x_corridor_z', -7.075)       # 남측 회랑 행(공용)
        self.declare_parameter('phase_x_dockcheck_standoff', 1.4)  # Phase B 관례 재사용
        # 슬롯 열 마커 "A3"(id2, x=9.6,z=-7.075) — 남측 회랑 행 위에 이미 있어 그대로
        # 최종 목적지 마커로 쓴다(입차 XN/LANE_2 처럼 별도 오프셋 마커를 새로 만들
        # 필요 없음). 두 로봇 모두 이 열까지 간다(§ ④, 축 간격은 이후 진입 단계에서
        # ingress trough_index 가 흡수 — Phase B 의 lead/follow 최종열 분리와 다른
        # 첫 설계 — 실측 후 필요하면 로봇별로 x 를 갈라도 된다).
        self.declare_parameter('phase_x_slot_marker_id', 2)
        self.declare_parameter('phase_x_slot_x', 9.6)
        # 도크(러너 sm.ROBOT_DOCK_MARKER): leader=exit_lead→D_IN_1(id20,x=-3.2),
        # follower=exit_follow→D_IN_2(id22,x=-1.2). dock_z(스폰/회전 목표)=-2.2,
        # dock_decal_z(데칼 실좌표, marker_map_v4.json 실측)=-2.9 — 입차와 부호만
        # 반전, 크기는 그 파일이 이미 다르므로(2.2/2.9 vs 2.2/2.9 동일 크기, z 부호만
        # 반전) 손으로 베끼지 않고 실측값 그대로 옮겼다.
        self.declare_parameter('phase_x_leader_dock_id', 20)
        self.declare_parameter('phase_x_leader_dock_x', -3.2)
        self.declare_parameter('phase_x_leader_dock_z', -2.2)
        self.declare_parameter('phase_x_leader_dock_decal_z', -2.9)
        self.declare_parameter('phase_x_follower_dock_id', 22)
        self.declare_parameter('phase_x_follower_dock_x', -1.2)
        self.declare_parameter('phase_x_follower_dock_z', -2.2)
        self.declare_parameter('phase_x_follower_dock_decal_z', -2.9)
        # 남측 회랑마커(corridor_lock, § ②): lead=XS(id30,x=-3.2,z=-7.075,
        # 입차 XN 의 정확한 대칭 — 2026-07-27 XS/XN 정렬 수정으로 좌표 일치 확인됨),
        # follow=LANE_3X(id76,x=-1.2,z=-7.075, LANE_3(id63) 의 신규 대칭 마커).
        self.declare_parameter('phase_x_leader_localizer_node', '')
        self.declare_parameter('phase_x_follower_localizer_node', '')
        self.declare_parameter('phase_x_leader_corridor_id', 30)     # XS @ (-3.2,-7.075)
        self.declare_parameter('phase_x_follower_corridor_id', 76)   # LANE_3X @ (-1.2,-7.075)
        # dock_check/corridor_lock 중간 스텝 요 허용(Phase B dock_check_yaw_tol 관례).
        self.declare_parameter('dock_check_yaw_tol', 5.0)
        # ---- 트럭 접근(§ _drive_to_slot_column/_approach_truck_front). 2026-07-27
        # 재설계 이력: ①동시실행(둘 다 같은 점으로 향하다 충돌) ②순차+z오프셋
        # (2번의 진입 목표가 1번이 서 있던 자리를 관통해야 해서 또 막힘)
        # ③A3 열 동시출발+홀드백 ④A3 열 진입까지 완전 순차(2번은 1번이 트럭
        # 밑에 자리잡을 때까지 전혀 안 움직임) ⑤**아루코 정밀정렬(front_align)
        # 을 걷어내고 북향 회전만 한 뒤 곧장 ingress 로 넘김** — 이 ⑤가 라이브
        # 실측(사용자 스크린샷: 로봇이 트럭 바퀴열에 삐뚤게 진입, 진입 결과
        # ``est_max_lateral_dev_m=26.7``m — 정상 범위 1~2 대비 이상치)로 **기각
        # 됐다**. 마커 없이 북향 회전 지점(z=corridor_z)에서 곧장 ingress 를
        # 태우면, 회전 직후 남은 헤딩 오차·오도 드리프트가 트럭까지 남은
        # ~3.5m 이상 구간 내내 보정되지 않고 누적돼 축 정렬에서 크게 벗어난다.
        # **여섯 번째 재설계(사용자 지시 그대로) — front_align 복원 + A3 열
        # 동시출발+홀드백도 복원**: "A3구역에 도착하고 아루코 마커를 보면서
        # 정렬을 하면 저런 상황이 없을듯 함" — 마커 정렬 자체가 아니라 마커를
        # 완전히 뺀 것이 원인이었다. 동시에 "뒷로봇이 너무 늦게 따라와서 (...)
        # 어느정도는 같이 가다가 (앞로봇과 충돌하지 않을 정도로만) 앞로봇이
        # A3 에 도착하면 그때 멈추고" — ④의 완전정지 대기도 사용자가 원한
        # 안무가 아니었다. 결국 ③(동시출발+홀드백)+front_align 복원 조합이
        # 맞다 — 트럭 접근+진입만 순차(①②의 교훈은 유지), A3 열 진입은 다시
        # 동시(홀드백 간격 유지). 아래 파라미터·``_approach_truck_front``·
        # "A3 열 방향 동진" 블록을 모두 원복했다.
        self.declare_parameter('phase_x_truck_front_marker_id', 73)  # A3_FX
        self.declare_parameter('phase_x_truck_front_z', -3.5)
        # A3 열 동진(§ _drive_to_slot_column) 2번(뒤차) 홀드백 거리 — 1번의 목표
        # (phase_x_slot_x) 보다 이만큼 못 미친 지점을 2번의 1차 목표로 준다.
        # 같은 속도로 동시 출발해도 2번이 먼저 도착해 멈추므로, 1번이 A3 에
        # 도착할 때는 2번이 이미 정지해 있고 그 사이 간격은 최소 이 값으로
        # 보장된다(속도 추정에 기대지 않음). 5.0m 는 로봇 실측 풋프린트
        # (HANDOFF.md 추정 반경 0.68m, 스윙암 포함시 더 큼) 대비 넉넉한 첫
        # 추정치 — 그래도 부족하면 이 값을 더 키울 것.
        self.declare_parameter('phase_x_second_holdback_gap', 5.0)
        # 2026-07-27 일곱 번째 재설계 — 2번(뒷로봇) 출발을 1번(앞로봇) 출발보다
        # 이만큼 늦춘다(사용자 지시: "앞로봇이 먼저 출발하고 5초후에 뒷로봇이
        # 따라가는걸로 하자"). 홀드백 거리만으로는 정지 지점의 최종 간격만
        # 보장되고 주행 중 간격은 보장되지 않아 실측에서 추돌이 관찰됐다
        # (§ 아래 "A3 열 방향 동진" 블록 주석).
        self.declare_parameter('phase_x_second_start_delay_sec', 5.0)

        gp = self.get_parameter
        self.action_name = gp('action_name').value
        self.leader_trough_index = int(gp('leader_trough_index').value)
        self.follower_trough_index = int(gp('follower_trough_index').value)
        self.navigate_odom_action = gp('navigate_odom_action').value
        self.navigate_fused_action = gp('navigate_fused_action').value
        self.ingress_action = gp('ingress_action').value
        self.carry_action = gp('carry_action').value
        self.exit_wait_x = float(gp('exit_wait_x').value)
        self.exit_wait_z = float(gp('exit_wait_z').value)
        self.exit_wait_truck_yaw_deg = float(gp('exit_wait_truck_yaw_deg').value)
        self.lift_action = gp('lift_action').value

        # ---- Phase B(입차) 파라미터 캐시 — 주석처리(§ 위 declare_parameter 블록과 동일
        # 이유). 되돌릴 때 함께 복원.
        # self.run_phase_b_first = bool(gp('run_phase_b_first').value)
        # self.phase_b_xn_id = int(gp('phase_b_xn_id').value)
        # self.phase_b_xn_x = float(gp('phase_b_xn_x').value)
        # self.phase_b_xn_z = float(gp('phase_b_xn_z').value)
        # self.phase_b_xn_standoff = float(gp('phase_b_xn_standoff').value)
        # self.phase_b_dockcheck_standoff = float(gp('phase_b_dockcheck_standoff').value)
        # self.phase_b_final_x_offset = float(gp('phase_b_final_x_offset').value)
        # self.phase_b_leader_dock_id = int(gp('phase_b_leader_dock_id').value)
        # self.phase_b_leader_dock_x = float(gp('phase_b_leader_dock_x').value)
        # self.phase_b_leader_dock_z = float(gp('phase_b_leader_dock_z').value)
        # self.phase_b_leader_dock_decal_z = float(gp('phase_b_leader_dock_decal_z').value)
        # self.phase_b_follower_dock_id = int(gp('phase_b_follower_dock_id').value)
        # self.phase_b_follower_dock_x = float(gp('phase_b_follower_dock_x').value)
        # self.phase_b_follower_dock_z = float(gp('phase_b_follower_dock_z').value)
        # self.phase_b_follower_dock_decal_z = float(gp('phase_b_follower_dock_decal_z').value)
        # self.phase_b_leader_localizer_node = gp('phase_b_leader_localizer_node').value
        # self.phase_b_follower_localizer_node = gp('phase_b_follower_localizer_node').value
        # self.phase_b_leader_corridor_id = int(gp('phase_b_leader_corridor_id').value)
        # self.phase_b_follower_corridor_id = int(gp('phase_b_follower_corridor_id').value)
        # self.phase_b_leader_final_id = int(gp('phase_b_leader_final_id').value)
        # self.phase_b_leader_final_yaw = float(gp('phase_b_leader_final_yaw').value)
        # self.phase_b_follower_final_id = int(gp('phase_b_follower_final_id').value)
        # self.phase_b_follower_final_yaw = float(gp('phase_b_follower_final_yaw').value)

        # ---- Phase X(출차) 파라미터 캐시 — 2026-07-27 신규 ----
        self.dock_check_yaw_tol = float(gp('dock_check_yaw_tol').value)
        self.run_phase_x_first = bool(gp('run_phase_x_first').value)
        self.phase_x_corridor_z = float(gp('phase_x_corridor_z').value)
        self.phase_x_dockcheck_standoff = float(gp('phase_x_dockcheck_standoff').value)
        self.phase_x_slot_marker_id = int(gp('phase_x_slot_marker_id').value)
        self.phase_x_slot_x = float(gp('phase_x_slot_x').value)
        self.phase_x_leader_dock_id = int(gp('phase_x_leader_dock_id').value)
        self.phase_x_leader_dock_x = float(gp('phase_x_leader_dock_x').value)
        self.phase_x_leader_dock_z = float(gp('phase_x_leader_dock_z').value)
        self.phase_x_leader_dock_decal_z = float(gp('phase_x_leader_dock_decal_z').value)
        self.phase_x_follower_dock_id = int(gp('phase_x_follower_dock_id').value)
        self.phase_x_follower_dock_x = float(gp('phase_x_follower_dock_x').value)
        self.phase_x_follower_dock_z = float(gp('phase_x_follower_dock_z').value)
        self.phase_x_follower_dock_decal_z = float(gp('phase_x_follower_dock_decal_z').value)
        self.phase_x_leader_localizer_node = gp('phase_x_leader_localizer_node').value
        self.phase_x_follower_localizer_node = gp('phase_x_follower_localizer_node').value
        self.phase_x_leader_corridor_id = int(gp('phase_x_leader_corridor_id').value)
        self.phase_x_follower_corridor_id = int(gp('phase_x_follower_corridor_id').value)
        self.phase_x_truck_front_marker_id = int(gp('phase_x_truck_front_marker_id').value)
        self.phase_x_truck_front_z = float(gp('phase_x_truck_front_z').value)
        self.phase_x_second_holdback_gap = float(gp('phase_x_second_holdback_gap').value)
        self.phase_x_second_start_delay_sec = float(gp('phase_x_second_start_delay_sec').value)

        self._cbg = ReentrantCallbackGroup()
        # 이름 주의: rclpy.node.Node 가 이미 인스턴스 속성 ``self._clients``
        # (서비스 클라이언트 리스트, ``create_client``/``destroy_node`` 내부용,
        # ``Node.__init__`` 에서 ``[]`` 로 초기화)를 쓴다 -- 여기서 같은 이름으로
        # 덮어쓰면(원래 버그였음) ``destroy_node()`` 의
        # ``self.destroy_client(self._clients[0])`` 가 dict 를 정수 0 으로
        # 인덱싱하다 ``KeyError: 0`` 로 죽는다(taskR5bfix 진단 그대로). 액션
        # 클라이언트 캐시라 이름도 구분해 ``_action_clients`` 로 둔다.
        self._action_clients = {}  # (robot_id, action_key) -> ActionClient
        self._carry_client = None  # /carry_to_slot 액션 클라이언트(로봇쌍 공용, 지연 생성)
        # 크로스노드 set_parameters(SetParameters 서비스) 클라이언트 캐시(노드명별).
        # Node.create_client 는 내부 self._clients(서비스 클라이언트 리스트)에도
        # 등록하지만, 우리 캐시는 별도 dict 라 그 리스트를 덮어쓰지 않는다(§ 위
        # _action_clients 이름주의 주석과 동일 근거).
        self._param_clients = {}  # node_name -> SetParameters 서비스 클라이언트
        # 측면 뎁스캠 게이팅 퍼블리셔(로봇별 캐시). sim_bridge 가 구독 —
        # 픽업 approach~ingress 구간에만 True 를 쏴 뎁스 렌더를 켠다(그 밖엔 pause).
        self._depth_enable_pubs = {}  # robot_id -> Bool 퍼블리셔

        self._execute_task_server = ActionServer(
            self, ExecuteParkingTask, self.action_name, self._on_execute_parking_task,
            callback_group=self._cbg)

        self.get_logger().info(
            f'pickup_orchestrator_node 시작: action={self.action_name} '
            f'leader_trough={self.leader_trough_index} follower_trough={self.follower_trough_index}')

        # 자율 실행: auto_start=true 면 노드 기동 후 auto_delay_sec 뒤에 스스로
        # 미션(ExecuteParkingTask 안무)을 실행한다 — 외부 트리거(스모크 스크립트)
        # 없이 launch 만으로 전체 미션이 돈다. 하위 노드/토픽 디스커버리 여유를 위해
        # 딜레이를 둔다. 1회성 타이머.
        self.declare_parameter('auto_start', False)
        self.declare_parameter('auto_leader', 'entry_lead')
        self.declare_parameter('auto_follower', 'entry_follow')
        self.declare_parameter('auto_task_id', 'AUTO')
        self.declare_parameter('auto_delay_sec', 25.0)
        if bool(self.get_parameter('auto_start').value):
            self._auto_leader = self.get_parameter('auto_leader').value
            self._auto_follower = self.get_parameter('auto_follower').value
            self._auto_task_id = self.get_parameter('auto_task_id').value
            delay = float(self.get_parameter('auto_delay_sec').value)
            self.get_logger().info(
                f'auto_start=true: {delay:.0f}s 뒤 자율 미션 실행 '
                f'(leader={self._auto_leader} follower={self._auto_follower})')
            self._auto_timer = self.create_timer(
                delay, self._auto_run, callback_group=self._cbg)

    # ---- 액션 클라이언트 지연 생성/캐시(로봇 id 는 goal 로 런타임에 옴) ----

    def _client(self, robot_id, action_key, action_type, action_suffix):
        key = (robot_id, action_key)
        client = self._action_clients.get(key)
        if client is None:
            name = f'/robot_{robot_id}/{action_suffix}'
            client = ActionClient(self, action_type, name, callback_group=self._cbg)
            self._action_clients[key] = client
        return client

    def _param_client(self, node_name):
        """대상 localizer 노드의 SetParameters 서비스 클라이언트(노드명별 캐시)."""
        client = self._param_clients.get(node_name)
        if client is None:
            client = self.create_client(
                SetParameters, f'{node_name}/set_parameters', callback_group=self._cbg)
            self._param_clients[node_name] = client
        return client

    def _depth_enable(self, robot_id, on):
        """측면 뎁스캠 렌더를 sim_bridge 에 켜/꺼 요청. bridge 가 항상 먼저 떠 있고
        기본 reliable QoS 라 연결된 구독자에 전달 보장됨(래칭 불필요)."""
        pub = self._depth_enable_pubs.get(robot_id)
        if pub is None:
            pub = self.create_publisher(Bool, f'/robot_{robot_id}/depth_enable', 10)
            self._depth_enable_pubs[robot_id] = pub
        pub.publish(Bool(data=bool(on)))
        self.get_logger().info(f'depth_enable[{robot_id}]={on}')

    # ---- 재진입 spin 없는 액션 호출(robot_task_orchestrator.py 와 동일 패턴) ----

    def _send_goal(self, client, goal, label):
        if not client.wait_for_server(timeout_sec=ACTION_SERVER_WAIT_TIMEOUT):
            reason = f'{label} 서버 미기동({ACTION_SERVER_WAIT_TIMEOUT:.0f}s)'
            self.get_logger().warn(reason)
            return None, reason

        send_fut = client.send_goal_async(goal)
        deadline = time.monotonic() + SEND_GOAL_TIMEOUT
        while not send_fut.done() and time.monotonic() < deadline:
            time.sleep(POLL_INTERVAL)
        if not send_fut.done():
            reason = f'{label} goal 전송 타임아웃'
            self.get_logger().warn(reason)
            return None, reason

        goal_handle = send_fut.result()
        if goal_handle is None or not goal_handle.accepted:
            reason = f'{label} goal 거부'
            self.get_logger().warn(reason)
            return None, reason
        return goal_handle, None

    def _wait_result(self, goal_handle, label, result_timeout):
        result_fut = goal_handle.get_result_async()
        deadline = time.monotonic() + result_timeout
        while not result_fut.done() and time.monotonic() < deadline:
            time.sleep(POLL_INTERVAL)
        if not result_fut.done():
            reason = f'{label} 결과 타임아웃({result_timeout:.0f}s)'
            self.get_logger().warn(reason)
            return None, None, reason
        response = result_fut.result()
        return response.result, response.status, None

    def _call_action(self, client, goal, *, label, result_timeout):
        goal_handle, reason = self._send_goal(client, goal, label)
        if goal_handle is None:
            return None, None, reason
        return self._wait_result(goal_handle, label, result_timeout)

    # ---- 단계별 호출 래퍼 ----

    def _ingress(self, robot_id, trough_index):
        client = self._client(robot_id, 'ingress', IngressUnderTruck, self.ingress_action)
        goal = IngressUnderTruck.Goal()
        goal.trough_index = int(trough_index)
        goal.forward_speed = 0.0
        goal.return_speed = 0.0
        result, _status, reason = self._call_action(
            client, goal, label=f'ingress[{robot_id}]', result_timeout=INGRESS_RESULT_TIMEOUT)
        if result is None:
            return False, reason
        if not result.success:
            return False, (f'ingress[{robot_id}] 실패(stop_reason={result.stop_reason}, '
                            f'stop_x={result.stop_x:.3f})')
        self.get_logger().info(
            f'ingress[{robot_id}] 완료: stop_x={result.stop_x:.4f} '
            f'target_axle_x={result.target_axle_x:.4f}')
        return True, None

    def _carry(self, leader_id, follower_id, target_x, target_z, target_yaw_deg, label):
        """가상중심 운반 한 세그먼트: 중심을 (target_x,target_z) 로 직진 후 트럭 yaw 를
        target_yaw_deg 로 회전(carry_action_server 2페이즈). Stage 4 가 lane→회전,
        가운데 진입을 각각 한 번씩 부른다."""
        if self._carry_client is None:
            self._carry_client = ActionClient(
                self, CarryToSlot, self.carry_action, callback_group=self._cbg)
        goal = CarryToSlot.Goal()
        goal.lead_robot_id = leader_id
        goal.follow_robot_id = follower_id
        goal.target_x = float(target_x)
        goal.target_z = float(target_z)
        goal.target_yaw_deg = float(target_yaw_deg)
        result, status, reason = self._call_action(
            self._carry_client, goal, label=label, result_timeout=CARRY_RESULT_TIMEOUT)
        if result is None:
            return False, reason
        if status != GoalStatus.STATUS_SUCCEEDED:
            return False, f'{label} 실패(status={status})'
        if not result.success:
            return False, f'{label} 실패({result.message})'
        return True, (f'{label} 완료: final=({result.final_x:.3f},{result.final_z:.3f},'
                      f'{result.final_yaw_deg:.1f})')

    def _lift_concurrent(self, leader_id, follower_id, command):
        """§클래스 docstring "동시 리프트" -- pickup_choreography.run_concurrent 로
        두 로봇의 send 를 wait 보다 먼저 전부 실행되게 강제한다."""
        leader_client = self._client(leader_id, 'lift', ControlLift, self.lift_action)
        follower_client = self._client(follower_id, 'lift', ControlLift, self.lift_action)
        goal = ControlLift.Goal(command=command)

        def _make_pair(robot_id, client):
            def send():
                return self._send_goal(client, goal, label=f'control_lift[{robot_id}][{command}]')

            def wait(sent):
                goal_handle, reason = sent
                if goal_handle is None:
                    return False, reason
                result, _status, reason = self._wait_result(
                    goal_handle, f'control_lift[{robot_id}][{command}]', LIFT_RESULT_TIMEOUT)
                if result is None:
                    return False, reason
                if not result.success:
                    return False, f'control_lift[{robot_id}] success=False'
                return True, None

            return send, wait

        pairs = [_make_pair(leader_id, leader_client), _make_pair(follower_id, follower_client)]
        outcomes = run_concurrent(pairs)  # [(ok, reason), (ok, reason)]
        oks = [ok for ok, _reason in outcomes]
        reasons = [reason for _ok, reason in outcomes if reason]
        overall = lift_both_succeeded(oks)
        if not overall:
            self.get_logger().warn(
                f'control_lift[{command}] 동시호출 실패: {reasons}')
        return overall, ('; '.join(reasons) if reasons else None)

    # ---- Phase B(도크 스폰→XN 융합주행) 레그 ----

    def _set_localizer_ref(self, node_name, ref_ids, correct_yaw, label):
        """융합 localizer 의 ``ref_ids``/``correct_yaw`` 를 크로스노드 set_parameters
        로 전환한다(T1 계약: ``ref_ids`` 는 반드시 명시적 INTEGER_ARRAY 로).

        비재진입 폴링(§ 클래스 docstring "동시성 패턴")으로 서비스 응답을 기다린다.
        """
        client = self._param_client(node_name)
        if not client.wait_for_service(timeout_sec=ACTION_SERVER_WAIT_TIMEOUT):
            return False, f'{label} localizer set_parameters 서비스 미기동({node_name})'
        req = SetParameters.Request()
        # T1 필수 계약: 빈배열 타입추론 경로를 쓰지 않고 명시적 INTEGER_ARRAY 로.
        req.parameters = [
            Parameter('ref_ids', Parameter.Type.INTEGER_ARRAY,
                      [int(i) for i in ref_ids]).to_parameter_msg(),
            Parameter('correct_yaw', Parameter.Type.BOOL,
                      bool(correct_yaw)).to_parameter_msg(),
        ]
        fut = client.call_async(req)
        deadline = time.monotonic() + SEND_GOAL_TIMEOUT
        while not fut.done() and time.monotonic() < deadline:
            time.sleep(POLL_INTERVAL)
        if not fut.done():
            return False, f'{label} set_parameters 응답 타임아웃'
        resp = fut.result()
        if resp is None or not all(r.successful for r in resp.results):
            reasons = ('; '.join(r.reason for r in resp.results if not r.successful)
                       if resp is not None else 'no response')
            return False, f'{label} set_parameters 거부: {reasons}'
        return True, None

    def _navigate_phase_b(self, robot_id, action_key, action_suffix, x, z, yaw_deg, label,
                          yaw_tol=None):
        """한 세그먼트의 NavigateToPose(USD 프레임) 호출 — 이름은 Phase B(입차) 시절
        그대로지만 로직은 목적지 좌표만 받는 범용 래퍼라 Phase X(출차, § 아래
        _run_phase_x)와 _return_to_dock 도 그대로 재사용한다(이름을 바꾸면 diff 만
        커지고 얻는 게 없어 유지).
        ``yaw_tol`` 지정 시 goal.behavior_tree 로 실어 그 스텝만 요 허용 완화(dock_check)."""
        client = self._client(robot_id, action_key, NavigateToPose, action_suffix)
        goal = _navigate_goal(self.get_clock(), x, z, yaw_deg)
        if yaw_tol is not None:
            goal.behavior_tree = f'yaw_tol={yaw_tol}'
        result, status, reason = self._call_action(
            client, goal, label=label, result_timeout=NAVIGATE_RESULT_TIMEOUT)
        if result is None:
            return False, reason
        if status != GoalStatus.STATUS_SUCCEEDED:
            return False, f'{label} 실패(status={status})'
        return True, None

    # ---- Phase B(입차) 실행 메서드 — 2026-07-27 출차 설계로 주석처리(§ declare_parameter
    # 블록과 동일 이유). 되돌릴 때 이 두 메서드 + 위 파라미터 블록들 + phase_b_plan
    # import + _on_execute_parking_task 의 "Phase B 동시 시작" 블록을 함께 복원.
    #
    # def _phase_b_params(self, robot_id, is_leader):
    #     """로봇 역할(leader/follower)에 맞는 Phase B 좌표/노드 파라미터 dict."""
    #     ... (원본은 git 이력 또는 이전 버전 참고)
    #
    # def _run_phase_b(self, robot_id, params):
    #     """한 로봇의 Phase B 레그를 localizer ref 전환 + odom/융합 navigate 로 실행."""
    #     ... (원본은 git 이력 또는 이전 버전 참고)

    # ---- Phase X(출차) 파라미터/실행 — 2026-07-27 신규, 사용자 설계 ----

    def _phase_x_params(self, robot_id, is_leader):
        """로봇 역할(leader/follower)에 맞는 Phase X 좌표/노드 파라미터 dict.
        구조는 ``_phase_b_params``(주석처리됨, 위 참고)를 그대로 미러링한다."""
        if is_leader:
            dock_id = self.phase_x_leader_dock_id
            dock_x = self.phase_x_leader_dock_x
            dock_z = self.phase_x_leader_dock_z
            dock_decal_z = self.phase_x_leader_dock_decal_z
            node = self.phase_x_leader_localizer_node
            corridor_id = self.phase_x_leader_corridor_id
        else:
            dock_id = self.phase_x_follower_dock_id
            dock_x = self.phase_x_follower_dock_x
            dock_z = self.phase_x_follower_dock_z
            dock_decal_z = self.phase_x_follower_dock_decal_z
            node = self.phase_x_follower_localizer_node
            corridor_id = self.phase_x_follower_corridor_id
        if not node:
            node = f'/robot_{robot_id}/marker_localizer_node'
        return {
            'localizer_node': node,
            'dock_id': dock_id, 'dock_x': dock_x, 'dock_z': dock_z,
            'dock_decal_z': dock_decal_z,
            'dockcheck_standoff': self.phase_x_dockcheck_standoff,
            'corridor_id': corridor_id,          # 남측 회랑마커(도크 x 그대로 남진, 중앙에)
            'corridor_z': self.phase_x_corridor_z,  # 회랑마커 z 라인(=-7.075)
        }

    def _run_phase_x(self, robot_id, params):
        """한 로봇의 Phase X(출차 회랑) 레그, **발산 구간만**(도크 → A3 열 진입
        직전까지) — 사용자 지시 앞 2단계를 코드화: ①도크에서 제자리 90° 회전(남향)
        ②남진하며 남측 회랑마커 인식 ③제자리 90° 회전(동향). A3 열 진입부터는
        (구 4단계 "마커따라 트럭까지") lead/follow 가 같은 x=9.6 으로 수렴하므로
        여기 포함하지 않고 ``_drive_to_slot_column``(동시)+``_approach_truck_front``
        (순차)가 담당한다(§ 그 함수들 docstring — 2026-07-27 라이브 실측 충돌로 재설계).

        ``pickup_choreography.phase_x_robot_phases()`` 의 4단계(seed_dock 포함)를
        그대로 밟는다 — 구조는 ``_run_phase_b``(주석처리됨, 위 참고)와 동일한
        패턴(오도 회전 ↔ 마커융합 직진 교대)이나 방향이 반대(북→남, 회전각도
        부호도 반대)다.

        **2026-07-27 실측으로 기각된 가정**: 이 구간은 lead/follow 가 서로 다른
        도크 x 열(-3.2/-1.2, 중심간 2.0m)에 머무르므로 동시 실행해도 안전하다고
        가정했었다. 라이브 관찰(사용자: "둘이 충돌했는데?") — GT 로그로는 두
        로봇이 ``rotate_east``(제자리 90° 회전)를 동시에 밟는 구간에서 중심간
        거리 약 1.94m 였고, 목표 90°로 수렴하지 못하고 102~108° 사이를 오가며
        같이 흔들렸다(단독 회전이라면 나올 수 없는 패턴 — 서로 스치며 밀어낸
        것으로 추정). 로봇 풋프린트 반경 0.68m(HANDOFF 7절, **실측 미검증**
        추정치)로 원 근사해도 가장자리 여유가 0.58m 뿐이라 여유가 빠듯했다.
        **제자리 회전(rotate_south/rotate_east)은 스윕 반경이 병진 구간보다
        커서 위험하고, corridor_lock(남진 병진)은 각자 x 열을 유지한 채
        진행방향으로만 움직여 서로의 경로를 가로지르지 않으므로 안전하다** —
        이 구분에 따라 호출측(``_on_execute_parking_task``)이 이제 Phase X
        전체를 두 로봇 **완전 순차**로 실행한다(§ 그 블록 주석). 동시 실행은
        걷어냈다.

        **미검증 표시**: Phase B 는 여러 날에 걸친 실측(HANDOFF/DEBUG_LOG)으로
        standoff·tol 값이 확정됐지만, 이 함수는 그 값(dockcheck_standoff=1.4 등)을
        방향만 반전해 재사용한 첫 설계다 — 실제 Isaac 구동으로 재검증 전까지는
        추정치로 취급할 것.
        """
        node = params['localizer_node']
        dock_id = params['dock_id']
        dock_x, dock_z = params['dock_x'], params['dock_z']
        # 도크체크 목표 z = 데칼 z - standoff(남진이므로 뺀다 — Phase B 의 "+"와
        # 부호만 반대, dock_decal_z 자체가 이미 음수라 결과적으로 더 남쪽으로 간다).
        dock_check_z = params['dock_decal_z'] - params['dockcheck_standoff']
        corridor_id = params['corridor_id']
        corridor_z = params['corridor_z']    # 남측 회랑마커 z 라인(=-7.075). 여기까지 남진.

        for phase in phase_x_robot_phases():
            label = f'phase_x:{phase}[{robot_id}]'
            if phase == 'seed_dock':
                # ref_ids=[dock_id] 하드필터 + yaw 보정 ON(Phase B seed_dock 과 동일 근거).
                ok, reason = self._set_localizer_ref(node, [dock_id], True, label)
            elif phase == 'rotate_south':
                # ①제자리 90° 회전해 남향(yaw 180). **오도 인스턴스**(회전 중 마커 상실,
                # Phase B rotate_90 과 동일 근거). 도크 스폰 yaw=90(동향)에서 90° 회전.
                ok, reason = self._navigate_phase_b(
                    robot_id, 'nav_odom', self.navigate_odom_action,
                    dock_x, dock_z, 180.0, label)
            elif phase == 'corridor_lock':
                # ②남진하며 남측 회랑마커(lead=XS/follow=LANE_3X)를 인식 — ref 전환 +
                # 후방캠 융합으로 도크 데칼 위치보정하며 남진(dock_check_z) 한 뒤,
                # 그대로 회랑 z 라인(corridor_z)까지 남진해 마커를 로봇 정중앙에 둔다.
                # (Phase B 의 dock_check+corridor_center 두 단계를 한 phase 이름으로
                # 합쳤다 — 사용자 지시가 "인식"을 한 동작으로 표현했기 때문.)
                ok, reason = self._navigate_phase_b(
                    robot_id, 'nav_odom', self.navigate_odom_action,
                    dock_x, dock_check_z, 180.0, label, yaw_tol=self.dock_check_yaw_tol)
                if ok:
                    ok, reason = self._set_localizer_ref(node, [corridor_id], True, label)
                if ok:
                    ok, reason = self._navigate_phase_b(
                        robot_id, 'nav_odom', self.navigate_odom_action,
                        dock_x, corridor_z, 180.0, label)
            elif phase == 'rotate_east':
                # ③다시 제자리 90° 회전해 동향(yaw 90) — A3(x=9.6) 쪽으로 갈 방향.
                # 오도 인스턴스(Phase B rotate_90/final_align 과 동일하게 회전은 오도).
                ok, reason = self._navigate_phase_b(
                    robot_id, 'nav_odom', self.navigate_odom_action,
                    dock_x, corridor_z, 90.0, label)
            else:  # pragma: no cover
                ok, reason = False, f'알 수 없는 Phase X 단계: {phase}'
            if not ok:
                return False, f'{label} 실패: {reason}'
        return True, None

    def _drive_to_slot_column(self, robot_id, node, target_x):
        """A3 열 방향으로 동진(구 Phase X ④ "마커따라 트럭까지") — ref 를 A3 열
        마커로 전환해 ``target_x`` 까지 융합 직진한다. ``target_x`` 를 호출측이
        고른다: 1번(앞선 로봇)은 ``phase_x_slot_x``(=9.6, A3 그 자리)까지,
        2번(뒤차)은 그보다 ``phase_x_second_holdback_gap`` 만큼 못 미친 지점
        까지만(첫 호출) — 두 로봇이 **동시에 출발**해도 뒤차가 못 미친
        지점에서 먼저 멈추므로 앞차가 A3 에 도착할 무렵엔 이미 확실한 간격이
        벌어져 있다. 그 뒤 1번이 트럭 접근+진입을 전부 끝내면, 2번은 이 함수를
        ``target_x=phase_x_slot_x`` 로 **다시** 호출해 남은 구간을 마저 간다
        (§ 호출측 ``_on_execute_parking_task`` "A3 열 방향 동진"/"2번: 이제
        재개" 블록 — 2026-07-27 여섯 번째 재설계로 홀드백 방식을 복원했다,
        사용자 지시: "어느정도는 같이 가다가(...) 앞로봇이 A3 에 도착하면
        그때 멈추고").
        """
        label = f'drive_to_slot[{robot_id}]'
        ok, reason = self._set_localizer_ref(node, [self.phase_x_slot_marker_id], True, label)
        if ok:
            ok, reason = self._navigate_phase_b(
                robot_id, 'nav_odom', self.navigate_odom_action,
                target_x, self.phase_x_corridor_z, 90.0, label)
        if not ok:
            return False, f'{label} 실패: {reason}'
        return True, None

    def _approach_truck_front(self, robot_id, node):
        """A3 열에서 제자리 북향 회전 + 트럭 앞(A3_FX 마커)까지 정밀 정렬. ingress
        (실제로 트럭 밑까지 전진해 축을 찾는 것)는 호출측이 이어서 별도 액션으로
        부른다(§ ``_do_ingress``) — 이 함수는 그 직전 자세까지만 만든다.

        **2026-07-27 다섯 번째 재설계에서 이 front_align 을 걷어냈다가, 라이브
        실측(사용자 스크린샷 — 로봇이 트럭 바퀴열에 삐뚤게 진입, 그 진입의
        ``est_max_lateral_dev_m=26.7``m — 정상 1~2 대비 이상치)으로 **여섯 번째
        재설계에서 복원**했다. 마커 없이 북향 회전 지점(z=corridor_z)에서 곧장
        ingress 로 넘기면, 트럭까지 남은 ~3.5m+ 구간의 헤딩·횡오차가 전혀
        보정되지 않고 그대로 누적돼 축에서 크게 벗어난 채 "진입 완료"로
        오판정된다(ingress 자체의 축 검출이 엉뚱한 것을 축으로 오인). 사용자
        지시 그대로: "A3구역에 도착하고 아루코 마커를 보면서 정렬을 하면 저런
        상황이 없을듯 함" — 문제는 마커 정렬 자체가 아니라 그것을 뺀 것이었다.

        **2026-07-27 재설계 배경(접근+진입을 순차로 만든 이유, 계속 유지)**:
        처음엔 이 구간을 lead/follow 동시 실행했다가(둘 다 같은 목표점으로
        향해 충돌), 그 다음 "먼저/나중 순차 + z 오프셋"으로 고쳤는데도 라이브
        구동에서 또 실패했다(뒤차가 앞차가 서 있던 자리를 관통해야 했음).
        앞차가 접근+진입을 전부 끝내고 자기 축에 완전히 자리잡은 뒤에야
        뒤차가 출발하므로 관통할 대상이 없다 — 이 순차 원칙은 유지한다
        (§ ``_on_execute_parking_task`` "A3 열 방향 동진" 블록).
        """
        label = f'approach:rotate_north[{robot_id}]'
        ok, reason = self._navigate_phase_b(
            robot_id, 'nav_odom', self.navigate_odom_action,
            self.phase_x_slot_x, self.phase_x_corridor_z, 0.0, label)
        if not ok:
            return False, f'{label} 실패: {reason}'
        label = f'approach:front_align[{robot_id}]'
        ok, reason = self._set_localizer_ref(
            node, [self.phase_x_truck_front_marker_id], True, label)
        if ok:
            ok, reason = self._navigate_phase_b(
                robot_id, 'nav_odom', self.navigate_odom_action,
                self.phase_x_slot_x, self.phase_x_truck_front_z, 0.0, label)
        if not ok:
            return False, f'{label} 실패: {reason}'
        return True, None

    def _return_to_dock(self, robot_id, params):
        """운반·안착이 끝난 뒤 도크로 복귀 — Phase X 를 좌표만 뒤집어 역주행한다
        (§ 클래스 docstring 신규 단계, 사용자 지시 "로봇 대기장소에 다시 복귀").
        **첫 설계, 미검증**: 안착 직후 로봇의 실제 자세(carry_action_server 가
        가상중심 강체제어로 어디에 남겨두는지)를 정확히 모델링하지 않고, 출차대기
        구역 부근(exit_wait_x, corridor_z)에서 출발한다고 가정한다 — 실측 후
        보정 필요.

        ①제자리 90° 회전(서향, yaw -90) — 도크가 있는 동쪽(x 가 더 큰 도크 열)
        ...이 아니라 출차대기구역(x=-8.5)이 도크(x=-3.2/-1.2)보다 더 서쪽이므로
        실제로는 동쪽을 봐야 한다 — yaw 90(동향)으로 회전.
        ②회랑마커(corridor_id)로 남측 회랑 행을 동진해 자기 도크 x 열까지.
        ③제자리 90° 회전(북향, yaw 0) — 도크는 회랑보다 북쪽(z=-2.2 > -7.075).
        ④도크마커(dock_id)로 북진해 도크 데칼 근처, 마지막에 스폰 yaw(90, 동향)
        로 재정렬해 대기 상태로 복귀.
        """
        node = params['localizer_node']
        dock_id = params['dock_id']
        dock_x, dock_z = params['dock_x'], params['dock_z']
        dock_check_z = params['dock_decal_z'] - params['dockcheck_standoff']
        corridor_id = params['corridor_id']
        corridor_z = params['corridor_z']

        label = f'return:rotate_east[{robot_id}]'
        ok, reason = self._navigate_phase_b(
            robot_id, 'nav_odom', self.navigate_odom_action,
            self.exit_wait_x, corridor_z, 90.0, label)
        if not ok:
            return False, f'{label} 실패: {reason}'

        label = f'return:corridor_to_dock_column[{robot_id}]'
        ok, reason = self._set_localizer_ref(node, [corridor_id], True, label)
        if ok:
            ok, reason = self._navigate_phase_b(
                robot_id, 'nav_odom', self.navigate_odom_action,
                dock_x, corridor_z, 90.0, label)
        if not ok:
            return False, f'{label} 실패: {reason}'

        label = f'return:rotate_north[{robot_id}]'
        ok, reason = self._navigate_phase_b(
            robot_id, 'nav_odom', self.navigate_odom_action,
            dock_x, corridor_z, 0.0, label)
        if not ok:
            return False, f'{label} 실패: {reason}'

        label = f'return:dock_check[{robot_id}]'
        ok, reason = self._set_localizer_ref(node, [dock_id], True, label)
        if ok:
            ok, reason = self._navigate_phase_b(
                robot_id, 'nav_odom', self.navigate_odom_action,
                dock_x, dock_check_z, 0.0, label, yaw_tol=self.dock_check_yaw_tol)
        if not ok:
            return False, f'{label} 실패: {reason}'

        label = f'return:final_dock[{robot_id}]'
        ok, reason = self._navigate_phase_b(
            robot_id, 'nav_odom', self.navigate_odom_action,
            dock_x, dock_z, 90.0, label)
        if not ok:
            return False, f'{label} 실패: {reason}'
        return True, None

    # ---- feedback ----

    def _publish_feedback(self, goal_handle, step, idx, total):
        fb = ExecuteParkingTask.Feedback()
        fb.current_step = step
        fb.progress = float(idx) / float(total) if total else 1.0
        try:
            goal_handle.publish_feedback(fb)
        except Exception:  # noqa: BLE001 -- 목표가 막 종료된 경합은 무시
            pass

    # ---- 상태머신 실행부 ----

    def _auto_run(self):
        """auto_start 타이머 콜백(1회성): 외부 goal 없이 스스로 안무를 실행한다.
        ``_AutoGoalHandle`` 로 액션 콜백 본체(``_on_execute_parking_task``)를 그대로
        재사용한다 — 자율 실행과 액션 실행이 완전히 같은 안무 코드를 탄다."""
        self._auto_timer.cancel()  # 1회만 실행
        self.get_logger().info('auto_start: 자율 미션 시작')
        req = _AutoRequest(self._auto_leader, self._auto_follower, self._auto_task_id)
        handle = _AutoGoalHandle(req, self.get_logger())
        result = self._on_execute_parking_task(handle)
        self.get_logger().info(
            f'auto_start: 미션 종료 success={result.success} message={result.message}')

    def _on_execute_parking_task(self, goal_handle):
        goal = goal_handle.request
        leader_id = goal.leader_robot_id
        follower_id = goal.follower_robot_id

        if not leader_id or not follower_id:
            msg = ('execute_pickup_choreography 는 leader_robot_id/follower_robot_id '
                   f'둘 다 필요합니다(받음: leader={leader_id!r} follower={follower_id!r})')
            self.get_logger().warn(msg)
            goal_handle.abort()
            result = ExecuteParkingTask.Result(success=False, message=msg)
            return result

        self.get_logger().info(
            f'execute_pickup_choreography(출차) 시작: task_id={goal.task_id} '
            f'leader={leader_id} follower={follower_id}')

        # ============================================================
        # 2026-07-27: 이 메서드는 원래 입차(픽업+주차) 안무였다. 출차 설계로
        # 전환하며 본문을 교체했다 — 원본(Phase B 동시정렬 → 순차진입 → 동시리프트
        # UP → 운반(슬롯 lane→가운데) → 동시리프트 DOWN)은 git 이력에 남아 있고,
        # 되돌릴 때는 이 메서드 본문을 그 커밋으로 되돌리면서 위쪽에 주석처리해둔
        # Phase B 파라미터/실행 메서드 블록들도 함께 복원하면 된다.
        # ============================================================

        # 뎁스캠 게이팅 퍼블리셔를 미션 시작(=approach 보다 한참 전)에 미리 만들어
        # sim_bridge 구독자와 DDS 매칭을 끝내둔다(입차와 동일 근거).
        for rid in (leader_id, follower_id):
            self._depth_enable(rid, False)

        # 진행률 분모: Phase X 스텝(양쪽 4×2=8) + A3열 동시진입(1번+2번 홀드백) 2 +
        # 1번 접근+진입 2 + 2번 재개 이동+접근+진입 3 + 리프트UP 1 +
        # 운반(회랑복귀+회랑서진+대기베이 3단계) 3 + 안착DOWN 1 + 도크복귀 2.
        px_steps = phase_x_plan(leader_id, follower_id) if self.run_phase_x_first else []
        total = len(px_steps) + 2 + 2 + 3 + 1 + 3 + 1 + 2

        idx = 0

        # ---- 트럭 접근+진입 순서 결정: 목표(A3)에 더 가까운 도크(=트럭 앞바퀴 쪽,
        # 진행방향상 먼저 만나는 축)가 1번(앞로봇), 반대가 2번(뒷로봇). "leader/
        # follower" 역할명이 아니라 실제 도크 x 가 목표에 더 가까운 쪽을 계산으로
        # 골라 먼저 보낸다(도크 배치가 바뀌어도 항상 옳다, 하드코딩 아님).
        # **2026-07-27 여덟 번째 재설계로 이 계산을 Phase X 보다 앞으로 옮겼다**
        # (기존엔 Phase X 뒤에 있어 Phase X 는 항상 ``leader_id`` 를 먼저 돌렸는데,
        # 이 launch 설정에서 leader=exit_lead 가 실제로는 2번/뒷로봇이라 "뒷로봇이
        # 먼저 움직인다"는 관찰이 나왔다 — 사용자 지시: "처음에 뒷로봇 먼저
        # 움직이지 말고 앞로봇 먼저 움직이자"). 이제 Phase X 부터 A3 진입·트럭
        # 접근까지 미션 전체가 이 하나의 first_id/second_id 순서를 일관되게 쓴다.
        if (abs(self.phase_x_slot_x - self.phase_x_leader_dock_x)
                <= abs(self.phase_x_slot_x - self.phase_x_follower_dock_x)):
            first_id, second_id = leader_id, follower_id
        else:
            first_id, second_id = follower_id, leader_id
        self.get_logger().info(
            f'접근 순서: {first_id}(앞바퀴) 먼저 -> {second_id}(뒷바퀴) 나중')

        # ---- Phase X(도크→남측 회랑→A3 열): **지연출발 동시 실행**,
        # **1번(앞로봇) 먼저 출발, phase_x_second_start_delay_sec 후 2번** ----
        # **2026-07-27 재설계(동시→완전순차)**: 원래 스레드+join 배리어 동시실행이
        # 라이브 충돌(사용자: "둘이 충돌했는데?", 중심간 1.94m 에서 rotate_south/
        # rotate_east 동시 수행 시 90°로 수렴 못 하고 102~108° 사이 흔들림)로
        # 깨져 완전순차로 갔었다.
        # **2026-07-28 재수정(완전순차→지연출발)**: 완전순차는 안전하지만 2번이
        # 1번의 Phase X(약 3분) 내내 도크에서 그냥 대기해 총 시간이 거의 2배가
        # 된다(사용자 지시: "앞로봇이 뒷로봇 기다리는데 기다리지말고 먼저
        # 출발하자" — Phase X 도입부에도 반복 지적). 뒤이은 "A3 열 방향 동진"
        # 단계가 이미 같은 지연출발 패턴(고정 시간차만 두고 완전동시 join 없이
        # 진행, phase_x_second_start_delay_sec)으로 여러 차례 실측 통과했으므로,
        # 같은 메커니즘을 여기에도 적용한다 — **부분적 phase 단위 교대**(기각된
        # 안)와는 다르다: 각 로봇은 여전히 자기 Phase X 전체를 처음부터 끝까지
        # 독립적으로 실행할 뿐, 배리어/턴테이킹을 늘리지 않는다. 이 지연출발만
        # 으로 회전 겹침을 완전히 배제하지는 못한다(각 로봇 스텝 소요시간이
        # 갈리면 회전 구간이 우연히 겹칠 수 있음) — 실측으로 재검증 필요.
        phase_x_params = {}   # rid -> params dict(뒤의 approach_truck/return 에도 재사용)
        if self.run_phase_x_first:
            self._publish_feedback(goal_handle, 'PHASE_X_STAGGERED', idx, total)
            self.get_logger().info(
                f'Phase X 지연출발: {first_id}(앞로봇) 먼저 -> '
                f'{self.phase_x_second_start_delay_sec:.0f}s 후 {second_id}(뒷로봇)')
            legs = {}   # rid -> (ok, reason)

            def _phase_x_leg(rid):
                is_leader = (rid == leader_id)
                params = self._phase_x_params(rid, is_leader)
                phase_x_params[rid] = params
                legs[rid] = self._run_phase_x(rid, params)

            threads0 = [threading.Thread(target=_phase_x_leg, args=(first_id,), daemon=True),
                        threading.Thread(target=_phase_x_leg, args=(second_id,), daemon=True)]
            threads0[0].start()
            time.sleep(self.phase_x_second_start_delay_sec)
            threads0[1].start()
            for t in threads0:
                t.join()
            idx += len(phase_x_robot_phases()) * 2
            for rid in (first_id, second_id):
                ok, reason = legs.get(rid, (False, f'{rid} Phase X 미완'))
                if not ok:
                    self._publish_feedback(goal_handle, 'FAILED', idx, total)
                    self.get_logger().warn(f'execute_pickup_choreography 실패(Phase X {rid}): {reason}')
                    goal_handle.abort()
                    return ExecuteParkingTask.Result(success=False, message=reason)
            self.get_logger().info(
                'Phase X 지연출발 완료: 두 로봇 각자 도크 열에서 남측 회랑 정렬 '
                '(동향 90°) — A3 열 진입은 이어지는 _drive_to_slot_column 담당')
        else:
            phase_x_params = {
                leader_id: self._phase_x_params(leader_id, True),
                follower_id: self._phase_x_params(follower_id, False),
            }
        # ---- A3 열 방향 동진: **1번(앞로봇) 먼저 출발, 2번(뒷로봇)은
        # phase_x_second_start_delay_sec(기본 5초) 뒤에 출발, 2번은 못 미친
        # 지점(홀드백)에서 정지** — 2026-07-27 일곱 번째 재설계(사용자 지시
        # 그대로): "앞로봇이 먼저 출발하고 5초후에 뒷로봇이 따라가는걸로
        # 하자." 여섯 번째(홀드백만, 동시 출발)로는 라이브 실측에서 여전히
        # 뒷로봇이 앞로봇을 밀고 가는 충돌이 관찰됐다(사용자: "뒷로봇이
        # 앞로봇을 밀고 간다") — 두 로봇의 도크간 간격이 2.0m 뿐이라, 동시에
        # 출발하면 순간적인 속도차만으로도 그 간격이 쉽게 사라진다. 홀드백
        # 거리(``phase_x_second_holdback_gap``)는 **정지 지점**의 최종 간격만
        # 보장할 뿐 주행 중 간격은 보장하지 못했던 것 — 이번 재설계는 출발
        # 자체를 시간차로 벌려 주행 중에도 실질적인 간격이 유지되게 한다.
        # **2026-07-27 아홉 번째 재설계**: 여기서 두 스레드를 모두 join 하고
        # 나서야 다음(트럭 접근)으로 넘어갔었는데, 그러면 1번(앞로봇)이 이미
        # A3 에 도착해도 2번(뒷로봇)의 홀드백 주행이 끝날 때까지 **아무것도
        # 안 하고 대기**하게 된다 — 사용자 지시: "앞로봇은 뒷로봇을
        # 기다리지말고 계속 진행하자." 1번 스레드만 먼저 join 해 결과를 확인하고
        # 곧바로 트럭 접근+진입으로 넘어간다. 2번 스레드는 백그라운드에서 계속
        # 돌게 두고, 2번이 실제로 움직이기 시작하는 시점(1번의 진입 완료 뒤,
        # "2번: 이제 재개" 블록 직전)에 join 해 결과를 확인한다 — 그 시점엔
        # 어차피 2번의 짧은 홀드백 주행은 진작 끝나 있을 것이므로 실질적인
        # 추가 대기는 없다.
        self._publish_feedback(goal_handle, 'DRIVE_TO_SLOT', idx, total)
        second_holdback_x = self.phase_x_slot_x - self.phase_x_second_holdback_gap
        legs2 = {}

        def _drive_leg(rid, target_x):
            node = phase_x_params[rid]['localizer_node']
            legs2[rid] = self._drive_to_slot_column(rid, node, target_x)

        threads2 = [threading.Thread(target=_drive_leg, args=(first_id, self.phase_x_slot_x),
                                     daemon=True),
                    threading.Thread(target=_drive_leg, args=(second_id, second_holdback_x),
                                     daemon=True)]
        threads2[0].start()
        time.sleep(self.phase_x_second_start_delay_sec)
        threads2[1].start()

        threads2[0].join()
        idx += 1
        ok, reason = legs2.get(first_id, (False, f'{first_id} A3열 진입 미완'))
        if not ok:
            self._publish_feedback(goal_handle, 'FAILED', idx, total)
            self.get_logger().warn(f'execute_pickup_choreography 실패(A3열 진입 1번): {reason}')
            goal_handle.abort()
            return ExecuteParkingTask.Result(success=False, message=reason)
        self.get_logger().info(
            f'{first_id} A3 도착(x={self.phase_x_slot_x}) — {second_id} 는 '
            f'{self.phase_x_second_holdback_gap}m 못 미친 x={second_holdback_x:.2f} 로 '
            f'주행 중(백그라운드, 1번은 기다리지 않고 계속 진행)')
        node1 = phase_x_params[first_id]['localizer_node']

        def _do_ingress(rid, trough_index, tag):
            self._depth_enable(rid, True)
            ok, reason = self._ingress(rid, trough_index)
            self._depth_enable(rid, False)
            if not ok:
                self._publish_feedback(goal_handle, 'FAILED', idx, total)
                self.get_logger().warn(f'execute_pickup_choreography 실패(진입 {tag}): {reason}')
                goal_handle.abort()
                return ExecuteParkingTask.Result(success=False, message=reason)
            return None

        # ---- 1번: 마커보정 끄고 접근(북향 회전+A3_FX 정밀정렬) → 진입(앞바퀴,
        # trough 0) 완주. **2번은 이 구간 내내 홀드백 지점(x=second_holdback_x)
        # 에 정지 상태** — "앞로봇이 트럭 앞바퀴에 도착할 때까지 뒷로봇은 정지"를
        # 코드 구조로 강제한다(2번 관련 코드가 이 블록에 전혀 없다는 것 자체가
        # 증거).
        self._set_localizer_ref(node1, [-1], False, f'exit-marker-off[{first_id}]')
        self._publish_feedback(goal_handle, 'APPROACH_FIRST', idx, total)
        ok, reason = self._approach_truck_front(first_id, node1)
        if not ok:
            self._publish_feedback(goal_handle, 'FAILED', idx, total)
            self.get_logger().warn(f'execute_pickup_choreography 실패(트럭접근 1번): {reason}')
            goal_handle.abort()
            return ExecuteParkingTask.Result(success=False, message=reason)
        idx += 1

        # ---- 2번 재개를 여기(1번 진입 *전*)로 앞당긴다 — 2026-07-28 사용자
        # 지시("뒷로봇이 계속 기다리지 말고 A3 위치로 이동해서 시간을
        # 단축하자"). 이전엔 1번의 진입(수십 초)까지 다 끝난 뒤에야 2번이
        # 홀드백에서 재출발했다. 안전한 이유: 2번의 재개 구간은 z=corridor_z
        # (=-7.075) 에 고정한 채 x 만 홀드백→A3 로 움직인다 — 1번은 이 시점
        # 이미 트럭 앞(z=phase_x_truck_front_z=-3.5)까지 나가 있고 진입 중에는
        # 더 북쪽(z 증가)으로만 이동하므로, 2번이 z=-7.075 에 머무는 한 최소
        # 3.5m 이상 분리돼 물리적으로 겹칠 수 없다(§ Phase X 자체를 동시화하지
        # 않는 이유는 아래 "Phase X 순차 시작" 블록 주석 참고 — 그건 도크 인근
        # 제자리회전이 겹칠 수 있어 실측 충돌이 났던 별개 구간).
        node2 = phase_x_params[second_id]['localizer_node']
        threads2[1].join()  # 홀드백 스레드 — 이 시점엔 이미 끝나 있을 것(실질 대기 없음)
        ok2, reason2 = legs2.get(second_id, (False, f'{second_id} 홀드백 주행 미완'))
        if not ok2:
            self._publish_feedback(goal_handle, 'FAILED', idx, total)
            self.get_logger().warn(f'execute_pickup_choreography 실패(A3열 진입 2번 홀드백): {reason2}')
            goal_handle.abort()
            return ExecuteParkingTask.Result(success=False, message=reason2)
        self._set_localizer_ref(node2, [-1], False, f'exit-marker-off[{second_id}]')
        second_resume = {}

        def _second_resume_drive():
            second_resume['result'] = self._drive_to_slot_column(
                second_id, node2, self.phase_x_slot_x)

        second_resume_thread = threading.Thread(target=_second_resume_drive, daemon=True)
        second_resume_thread.start()

        self._publish_feedback(goal_handle, 'INGRESS_FIRST', idx, total)
        # 2026-07-28 사용자 지시로 수정: 1번(앞바퀴 담당)은 진입 첫 번째로 만나는
        # 축(근축)을 무시하고 두 번째 축(원축=실제 앞축)을 잡아야 한다 — 그래야
        # 2번(뒷바퀴)이 첫 번째 축을 잡을 때 두 로봇이 서로 다른 축(앞/뒤)을
        # 하나씩 맡아 트럭 전체를 들어올린다. 그래서 leader_trough_index(기본 1,
        # "두 번째로 뎁스변하는 바퀴")를 쓴다(이전엔 follower_trough_index=0을
        # 써서 1번이 근축에 멈췄고, 2번도 leader_trough_index=1로 같은 원축을
        # 다시 잡으려 해 두 로봇이 같은 축을 놓고 충돌하던 버그였다).
        fail = _do_ingress(first_id, self.leader_trough_index, '1번(앞바퀴)')
        if fail is not None:
            return fail
        idx += 1
        self.get_logger().info(f'{first_id} 앞바퀴 안착 완료 — {second_id} 재개 확인')

        # ---- 2번: 재개 주행(§ 위에서 1번 진입과 동시에 이미 시작해 둔
        # second_resume_thread)이 끝났는지 확인만 한다. 1번의 진입(수십 초)과
        # 겹쳐서 돌았으므로 여기 도달했을 때 이미 끝나 있는 경우가 많다(§ 위
        # 배경 주석). 같은 로봇에 다음 목표(접근)를 보내기 전에 반드시
        # join 해 이전 스레드가 확실히 끝났는지 확인한다(안 그러면 같은
        # 액션 서버에 목표 두 개가 겹쳐 들어가는 경합이 생길 수 있다).
        second_resume_thread.join()
        idx += 1
        ok, reason = second_resume.get('result', (False, f'{second_id} 재개 주행 미완'))
        if not ok:
            self._publish_feedback(goal_handle, 'FAILED', idx, total)
            self.get_logger().warn(f'execute_pickup_choreography 실패(A3열 진입 2번 재개): {reason}')
            goal_handle.abort()
            return ExecuteParkingTask.Result(success=False, message=reason)
        idx += 1
        self._publish_feedback(goal_handle, 'APPROACH_SECOND', idx, total)
        ok, reason = self._approach_truck_front(second_id, node2)
        if not ok:
            self._publish_feedback(goal_handle, 'FAILED', idx, total)
            self.get_logger().warn(f'execute_pickup_choreography 실패(트럭접근 2번): {reason}')
            goal_handle.abort()
            return ExecuteParkingTask.Result(success=False, message=reason)
        idx += 1
        self._publish_feedback(goal_handle, 'INGRESS_SECOND', idx, total)
        # § 위 "1번" 수정과 짝: 2번(뒷바퀴 담당)은 진입 첫 번째로 만나는 근축(실제
        # 뒷축)에서 바로 멈춰야 한다 — follower_trough_index(기본 0).
        fail = _do_ingress(second_id, self.follower_trough_index, '2번(뒷바퀴)')
        if fail is not None:
            return fail
        idx += 1
        self.get_logger().info(f'순차 진입 완료: {first_id}(앞바퀴)->{second_id}(뒷바퀴) 둘 다 축 정렬')

        # ---- 동시 리프트(입차와 동일 필수 지시 — run_concurrent 로 send 를 wait 보다
        # 먼저 전부 실행) ----
        self._publish_feedback(goal_handle, 'LIFT_UP', idx, total)
        ok, reason = self._lift_concurrent(leader_id, follower_id, 'UP')
        if not ok:
            self._publish_feedback(goal_handle, 'FAILED', idx, total)
            fail_reason = f'concurrent lift UP 실패: {reason}'
            self.get_logger().warn(f'execute_pickup_choreography 실패: {fail_reason}')
            goal_handle.abort()
            return ExecuteParkingTask.Result(success=False, message=fail_reason)
        idx += 1

        # ---- 운반: 왔던 길 그대로 출차대기구역(W_IN 베이)까지 3단계로 되짚어간다
        # (2026-07-27 사용자 지시: "왔던길 그대로 차량 대기 구역에 차를 내려놓자").
        # ①A3 열에서 남측 회랑(z=corridor_z)까지 후진하면서 **동시에** 베이 정지
        # yaw(exit_wait_truck_yaw_deg, 긴 축 x·앞이 -x/출구쪽)로 회전 ②그 상태로
        # (이미 서향이므로) 회랑을 따라 서진해 베이 열(exit_wait_x)까지 곧장
        # ③베이에서 정확한 정지위치로 마무리.
        #
        # 2026-07-28 라이브 실측으로 재설계: 원래는 ①②구간을 A3 배치 그대로
        # (180°, 로봇이 여전히 북향) 유지하고 회전을 베이 도착 뒤(③) 한 번만
        # 했다. TRANSLATE 단계는 로봇이 **자기 heading 축으로만** 밀 수 있는데
        # (§ carry_action_server.py 주석), ②구간의 실제 이동방향(서쪽, exit_wait_x
        # 까지 약 18m)이 그때 로봇 heading(북향)과 거의 직각이라 사실상 진행이
        # 안 됐다(실측: 벽 쪽 모서리에서 정체, 사용자 관찰 "리프트하고 뒤로 간
        # 후에 동작이 실행이 안되네"). 사용자 지시("뒤로 이동하고 90도 회전해야
        # 그래서 쭉 직진하면 돼", "회전을 한 차량의 앞이 출구쪽을 향하게") 그대로
        # — 회전을 ①로 앞당겨 로봇이 ②를 시작할 때 이미 서향이 되게 한다.
        #
        # **lead/follow 인자 순서 수정(2026-07-28 라이브 실측)**: ``formation.
        # truck_yaw_from_robots(lead_pos, follow_pos)`` 는 "lead=앞축, follow=뒷축"
        # 을 전제로 rear→front 벡터를 트럭 방향으로 삼는다(§ 그 함수 docstring).
        # 그런데 이 미션(출차)의 실제 물리 배치는 ``leader_id``(미션 고정 역할명,
        # 예 exit_lead)가 아니라 **first_id**(§ 위 트로프 배정, leader_trough_index
        # =1=원축=앞축을 잡는 쪽)가 앞축이다 — 이번 dock 배치에서는 first_id가
        # follower_id 였다(§ "접근 순서" 로그). 지금까지 ``self._carry(leader_id,
        # follower_id, ...)``로 불러 실제로는 뒷축 로봇을 "lead(앞축)"로 넘기고
        # 있었다 — 그래서 트럭 방향이 180° 뒤집혀 계산됐고(회전 방향이 반대로
        # 보이고 직진도 삐뚤어짐). first_id/second_id 는 항상 앞축/뒷축을
        # 가리키므로(로봇 이름이 아니라 역할로 고정) 이걸 그대로 넘긴다.
        self._publish_feedback(goal_handle, 'CARRY_TO_CORRIDOR', idx, total)
        ok, reason = self._carry(first_id, second_id,
                                 self.phase_x_slot_x, self.phase_x_corridor_z,
                                 self.exit_wait_truck_yaw_deg,
                                 f'carry-to-corridor[{first_id}+{second_id}]')
        if not ok:
            self._publish_feedback(goal_handle, 'FAILED', idx, total)
            self.get_logger().warn(f'execute_pickup_choreography 실패: {reason}')
            goal_handle.abort()
            return ExecuteParkingTask.Result(success=False, message=f'운반(회랑복귀) 실패: {reason}')
        idx += 1
        self._publish_feedback(goal_handle, 'CARRY_ALONG_CORRIDOR', idx, total)
        # § 위 CARRY_TO_CORRIDOR 주석 — 로봇이 이미 서향(exit_wait_truck_yaw_deg)이라
        # 이 구간(서진 약 18m)이 로봇 heading 을 따라가는 순수 전진이 된다.
        ok, reason = self._carry(first_id, second_id,
                                 self.exit_wait_x, self.phase_x_corridor_z,
                                 self.exit_wait_truck_yaw_deg,
                                 f'carry-along-corridor[{first_id}+{second_id}]')
        if not ok:
            self._publish_feedback(goal_handle, 'FAILED', idx, total)
            self.get_logger().warn(f'execute_pickup_choreography 실패: {reason}')
            goal_handle.abort()
            return ExecuteParkingTask.Result(success=False, message=f'운반(회랑서진) 실패: {reason}')
        idx += 1
        self._publish_feedback(goal_handle, 'CARRY_TO_WAIT', idx, total)
        ok, reason = self._carry(first_id, second_id,
                                 self.exit_wait_x, self.exit_wait_z, self.exit_wait_truck_yaw_deg,
                                 f'carry-to-wait[{first_id}+{second_id}]')
        if not ok:
            self._publish_feedback(goal_handle, 'FAILED', idx, total)
            self.get_logger().warn(f'execute_pickup_choreography 실패: {reason}')
            goal_handle.abort()
            return ExecuteParkingTask.Result(success=False, message=f'운반(출차대기) 실패: {reason}')
        idx += 1

        self._publish_feedback(goal_handle, 'PLACE', idx, total)
        ok, reason = self._lift_concurrent(leader_id, follower_id, 'DOWN')
        if not ok:
            self._publish_feedback(goal_handle, 'FAILED', idx, total)
            self.get_logger().warn(f'execute_pickup_choreography 실패(안착): {reason}')
            goal_handle.abort()
            return ExecuteParkingTask.Result(success=False, message=f'안착(lift DOWN) 실패: {reason}')
        idx += 1

        # ---- 도크로 복귀(§ _return_to_dock, 미검증 첫 설계): 동시 실행 ----
        self._publish_feedback(goal_handle, 'RETURN_TO_DOCK', idx, total)
        legs3 = {}

        def _return_leg(rid):
            legs3[rid] = self._return_to_dock(rid, phase_x_params[rid])

        threads3 = [threading.Thread(target=_return_leg, args=(leader_id,), daemon=True),
                    threading.Thread(target=_return_leg, args=(follower_id,), daemon=True)]
        for t in threads3:
            t.start()
        for t in threads3:
            t.join()
        idx += 2
        for rid in (leader_id, follower_id):
            ok, reason = legs3.get(rid, (False, f'{rid} 도크복귀 미완'))
            if not ok:
                self._publish_feedback(goal_handle, 'FAILED', idx, total)
                self.get_logger().warn(f'execute_pickup_choreography 실패(도크복귀): {reason}')
                goal_handle.abort()
                return ExecuteParkingTask.Result(success=False, message=reason)

        idx = total
        self._publish_feedback(goal_handle, 'DONE', idx, total)
        self.get_logger().info(
            f'execute_pickup_choreography(출차) 완료: task_id={goal.task_id} '
            f'leader={leader_id} follower={follower_id}')
        goal_handle.succeed()
        return ExecuteParkingTask.Result(
            success=True, message='출차 완료(회랑진입+진입+리프트+운반+안착+도크복귀)')


def main(args=None):
    rclpy.init(args=args)
    node = PickupOrchestratorNode()
    executor = MultiThreadedExecutor(num_threads=8)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
