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
    lift_both_succeeded, phase_b_plan, phase_b_robot_phases, run_concurrent,
)

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

# goal.slot_id → (park_slot_x, 레인마커, 가운데마커, 앞마커). marker_map_v4 기준.
# UI/dispatcher 가 이 중 하나를 goal.slot_id 로 보내면 그 슬롯으로 운반한다.
PARK_SLOTS = {
    'A1': (2.8, 3, 66, 65),
    'A2': (6.2, 4, 68, 67),
    'A3': (9.6, 5, 70, 69),
}


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
    def __init__(self, leader, follower, task_id, slot_id=''):
        self.leader_robot_id = leader
        self.follower_robot_id = follower
        self.task_id = task_id
        self.slot_id = slot_id


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
        self.declare_parameter('navigate_fused_action', 'navigate_to_pose_fused')
        self.declare_parameter('ingress_action', 'ingress_under_truck')
        # Phase D 운반: carry_action_server 액션 + 목표 슬롯 pose(우리 (x,z,yaw) 규약).
        # 기본 A2'(6.2, 6.875) — UI gateway 연동 전엔 이 파라미터로 목표 지정.
        # ponytail: goal.slot_pose(Pose) 매핑은 UI gateway 증분에서.
        self.declare_parameter('carry_action', 'carry_to_slot')
        # ---- Stage 4 주차: 슬롯 lane(z≈7.075) 도달+회전 → 가운데(z≈0) 진입 ----
        # 기본 슬롯 A1(x=2.8). lane 마커 id 3(2.8,7.075), 가운데 마커 id 66(2.8,0).
        self.declare_parameter('park_slot_x', 2.8)          # 슬롯 x(A1=2.8/A2=6.2/A3=9.6)
        # 슬롯 앞 마커 id(A1_F=65/A2_F=67/A3_F=69). carry 직진 정지 트리거 — 양 로봇
        # localizer 를 이 마커로 켜서 검출거리(ref_marker_dist)를 내고, 같은 거리면 회전.
        self.declare_parameter('park_slot_front_id', 65)
        # ★carry 직진 정지·회전 트리거 마커 = 선택 슬롯의 **레인 위** 앞마커
        #   (A1'=3/A2'=4/A3'=5, z=7.075 로 carry 경로 위에 있음). 65/67/69 는 z=3.5 라
        #   경로 밖(슬롯진입 때나 보임)이므로 정지 트리거엔 안 맞다 — 레인 위 3/4/5 를 쓴다.
        self.declare_parameter('park_slot_lane_id', 3)
        # 슬롯 진입(CARRY_SLOT) 정지 트리거 = 슬롯 가운데 마커(A1_C=66/A2_C=68/A3_C=70,
        # z=0). 회전 후 -z 진입 중 양 카메라가 이 마커를 등거리로 보면 정지·안착.
        self.declare_parameter('park_slot_center_id', 66)
        # carry 경로(z≈7.08) 위 레인 마커 id 들. carry 중 이걸로 localizer 를 켜서 /pose 를
        # 실시간 마커보정 → 옆드리프트 즉시 잡음(안 켜면 순수 오도 드리프트로 제어 발산).
        self.declare_parameter('park_lane_marker_ids', [61, 62, 63, 64])
        self.declare_parameter('park_lane_z', 7.075)        # 슬롯 앞 lane z(운반 도달선)
        self.declare_parameter('park_center_z', 0.0)        # 슬롯 가운데 z(최종 정지)
        # 복귀(RETURN): 주차 후 두 로봇을 원래 도크로 되돌린다(Phase B 역순, 독립주행).
        # lead 는 주차 완료 시 남향(yaw180)에 슬롯보다 남쪽(z≈center-½L)에 있어, 이탈 전
        # 제자리 회전(odom)의 목표 z. L≈3.57 → half≈1.78, park_center_z(0) 기준 -1.8.
        self.declare_parameter('return_lead_parked_z', -1.8)
        self.declare_parameter('return_dock_yaw', 90.0)     # 도크 최종 yaw(스폰 seed=90=동향)
        # 슬롯 진입 회전량[도, **상대**]. 위치기반 truck_yaw 가 그립 삐뚤어짐에 취약해
        # 절대각(-180) 대신 "직진 종료 시점 방향에서 이만큼 돈다"로 준다(CCW 90°=-90).
        self.declare_parameter('park_turn_deg', -90.0)
        self.declare_parameter('lift_action', 'control_lift')

        # ---- Phase B(도크 스폰→XN 융합주행) 레그 파라미터 (R6/T3) ----
        # corridor_plan(픽업 회랑) **앞에** 두 로봇의 도크→XN 레그를 붙일지.
        # 기본 True — bringup_pickup_e2e.sh 가 도크 스폰에서 기동할 때 켠다. False 면
        # 기존처럼 XN 종단 자세에서 곧장 픽업만 실행(하위호환, 기존 스모크/테스트).
        self.declare_parameter('run_phase_b_first', True)
        # XN 이동(2026-07-26 사용자 재배치): (-2.5,6.875) -> (-3.2,7.075) 회랑 마커.
        self.declare_parameter('phase_b_xn_x', -3.2)
        self.declare_parameter('phase_b_xn_z', 7.075)
        # 러너 미션 상수: XN 남쪽 standoff 1.3, 도크체크 standoff 1.4, 오프셋 -1.7.
        self.declare_parameter('phase_b_xn_standoff', 1.3)
        self.declare_parameter('phase_b_dockcheck_standoff', 1.4)
        self.declare_parameter('phase_b_final_x_offset', -1.7)
        # align_pos_tol(러너 0.06) 은 goal 에 실어보내지 않는다 — pose_controller_node
        # 의 pos_tol 파라미터(기본 0.03)로 강제되므로, bringup 이 융합 인스턴스에
        # -p pos_tol:=0.06 로 걸어야 한다(T4). 여기선 기록/문서용으로만 선언.
        self.declare_parameter('phase_b_align_pos_tol', 0.06)
        # 로봇별 도크(러너 sm.ROBOT_DOCK_MARKER): leader=entry_lead→D_OUT_1(id21,
        # x=-3.2), follower=entry_follow→D_OUT_2(id23, x=-1.2). dock_z 는 스폰/회전
        # 목표(aruco:position z, 기본 2.2), dock_decal_z 는 도크체크 목표 산정용
        # (데칼 xformOp:translate z 는 aruco:position 보다 0.7m 북쪽 — 러너
        # marker_visual_center 실측, 기본 2.9). 각 값은 T4/T5 bringup 이 실측으로
        # 오버라이드할 수 있게 파라미터로 노출.
        # 도크 데칼 z 이동(2026-07-26 사용자 재배치): 2.9 -> 3.8 (도크 마커 북상).
        self.declare_parameter('phase_b_leader_dock_id', 21)
        self.declare_parameter('phase_b_leader_dock_x', -3.2)
        self.declare_parameter('phase_b_leader_dock_z', 2.2)
        self.declare_parameter('phase_b_leader_dock_decal_z', 3.8)
        self.declare_parameter('phase_b_follower_dock_id', 23)
        self.declare_parameter('phase_b_follower_dock_x', -1.2)
        self.declare_parameter('phase_b_follower_dock_z', 2.2)
        self.declare_parameter('phase_b_follower_dock_decal_z', 3.8)
        # 융합 localizer 인스턴스의 **완전수식 노드명** — 크로스노드 set_parameters
        # (ref_ids/correct_yaw)의 서비스 대상 `<node_name>/set_parameters`. 빈 값이면
        # 런타임에 `/robot_<id>/marker_localizer_node` 로 유도한다(T4 bringup 이 로봇별
        # localizer 를 이 네임스페이스/노드명으로 띄우는 것을 전제 — 현재 bringup 은
        # 두 인스턴스가 같은 노드명 `marker_localizer_node` 라 T4 가 로봇별로
        # 구분해줘야 한다. 보고서 우려 참조).
        self.declare_parameter('phase_b_leader_localizer_node', '')
        self.declare_parameter('phase_b_follower_localizer_node', '')
        # 2026-07-27 재안무: 로봇별 회랑마커/최종마커/최종 yaw. corridor 는 도크 x 그대로
        # 북진해 중앙에 두는 회랑마커, final 은 제자리 회전해 정렬하는 진입직전 마커.
        self.declare_parameter('phase_b_leader_corridor_id', 31)     # XN @ (-3.2,7.075)
        self.declare_parameter('phase_b_follower_corridor_id', 63)   # LANE_3 @ (-1.2,7.075)
        self.declare_parameter('phase_b_leader_final_id', 62)        # LANE_2 handoff @ (-5.0)
        self.declare_parameter('phase_b_leader_final_yaw', -90.0)    # 서쪽(-x) 봄
        self.declare_parameter('phase_b_follower_final_id', 64)      # LANE_4 @ (0.4)
        self.declare_parameter('phase_b_follower_final_yaw', 90.0)   # 동쪽(+x) 봄
        # dock_check 요 허용(도) — 중간 스텝이라 완화(final_align 1° 는 노드기본 유지).
        self.declare_parameter('dock_check_yaw_tol', 5.0)

        gp = self.get_parameter
        self.action_name = gp('action_name').value
        self.leader_trough_index = int(gp('leader_trough_index').value)
        self.follower_trough_index = int(gp('follower_trough_index').value)
        self.navigate_odom_action = gp('navigate_odom_action').value
        self.navigate_fused_action = gp('navigate_fused_action').value
        self.ingress_action = gp('ingress_action').value
        self.carry_action = gp('carry_action').value
        self.park_slot_x = float(gp('park_slot_x').value)
        self.park_slot_front_id = int(gp('park_slot_front_id').value)
        self.park_slot_lane_id = int(gp('park_slot_lane_id').value)
        self.park_slot_center_id = int(gp('park_slot_center_id').value)
        self.park_lane_marker_ids = [int(i) for i in gp('park_lane_marker_ids').value]
        self.park_lane_z = float(gp('park_lane_z').value)
        self.park_center_z = float(gp('park_center_z').value)
        self.return_lead_parked_z = float(gp('return_lead_parked_z').value)
        self.return_dock_yaw = float(gp('return_dock_yaw').value)
        self.park_turn_deg = float(gp('park_turn_deg').value)
        self.lift_action = gp('lift_action').value

        # ---- Phase B 파라미터 캐시 ----
        self.dock_check_yaw_tol = float(gp('dock_check_yaw_tol').value)
        self.run_phase_b_first = bool(gp('run_phase_b_first').value)
        self.phase_b_xn_x = float(gp('phase_b_xn_x').value)
        self.phase_b_xn_z = float(gp('phase_b_xn_z').value)
        self.phase_b_xn_standoff = float(gp('phase_b_xn_standoff').value)
        self.phase_b_dockcheck_standoff = float(gp('phase_b_dockcheck_standoff').value)
        self.phase_b_final_x_offset = float(gp('phase_b_final_x_offset').value)
        self.phase_b_leader_dock_id = int(gp('phase_b_leader_dock_id').value)
        self.phase_b_leader_dock_x = float(gp('phase_b_leader_dock_x').value)
        self.phase_b_leader_dock_z = float(gp('phase_b_leader_dock_z').value)
        self.phase_b_leader_dock_decal_z = float(gp('phase_b_leader_dock_decal_z').value)
        self.phase_b_follower_dock_id = int(gp('phase_b_follower_dock_id').value)
        self.phase_b_follower_dock_x = float(gp('phase_b_follower_dock_x').value)
        self.phase_b_follower_dock_z = float(gp('phase_b_follower_dock_z').value)
        self.phase_b_follower_dock_decal_z = float(gp('phase_b_follower_dock_decal_z').value)
        self.phase_b_leader_localizer_node = gp('phase_b_leader_localizer_node').value
        self.phase_b_follower_localizer_node = gp('phase_b_follower_localizer_node').value
        self.phase_b_leader_corridor_id = int(gp('phase_b_leader_corridor_id').value)
        self.phase_b_follower_corridor_id = int(gp('phase_b_follower_corridor_id').value)
        self.phase_b_leader_final_id = int(gp('phase_b_leader_final_id').value)
        self.phase_b_leader_final_yaw = float(gp('phase_b_leader_final_yaw').value)
        self.phase_b_follower_final_id = int(gp('phase_b_follower_final_id').value)
        self.phase_b_follower_final_yaw = float(gp('phase_b_follower_final_yaw').value)

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
        self.declare_parameter('auto_slot_id', '')   # ''=park_slot_* 파라미터 기본값(A1). 'A2'/'A3' 로 자율실행 슬롯 선택
        self.declare_parameter('auto_delay_sec', 25.0)
        if bool(self.get_parameter('auto_start').value):
            self._auto_leader = self.get_parameter('auto_leader').value
            self._auto_follower = self.get_parameter('auto_follower').value
            self._auto_task_id = self.get_parameter('auto_task_id').value
            self._auto_slot_id = self.get_parameter('auto_slot_id').value
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

    def _egress(self, robot_id, target_z, forward_speed=0.0):
        """차밑 나오기: 좌우 뎁스 중앙유지하며 world +z(북)로 target_z 까지 전진(들어간
        방식과 대칭 → 바퀴 안 긁힘). 축검출·마커 무관, ingress_node egress 모드."""
        client = self._client(robot_id, 'ingress', IngressUnderTruck, self.ingress_action)
        goal = IngressUnderTruck.Goal()
        goal.egress = True
        goal.egress_target_z = float(target_z)
        goal.forward_speed = float(forward_speed)
        result, _status, reason = self._call_action(
            client, goal, label=f'egress[{robot_id}]', result_timeout=INGRESS_RESULT_TIMEOUT)
        if result is None:
            return False, reason
        if not result.success:
            return False, f'egress[{robot_id}] 실패(stop_reason={result.stop_reason})'
        self.get_logger().info(f'egress[{robot_id}] 완료: stop_z={result.stop_x:.4f}')
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

    def _set_localizer_ref(self, node_name, ref_ids, correct_yaw, label,
                           mdist_marker_id=None, use_front=None, force_yaw_deg=None):
        """융합 localizer 의 ``ref_ids``/``correct_yaw`` 를 크로스노드 set_parameters
        로 전환한다(T1 계약: ``ref_ids`` 는 반드시 명시적 INTEGER_ARRAY 로).
        ``mdist_marker_id`` 지정 시 ref_marker_dist 발행 대상 마커도 같이 설정한다.
        ``use_front``(운반): False 면 rear-only(전방캠 무시). ``force_yaw_deg``: 지정
        시 융합필터 yaw 를 즉시 그 각도로 리셋(차 든 직후 90°). None 이면 안 건드림.

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
        if use_front is not None:
            req.parameters.append(
                Parameter('use_front', Parameter.Type.BOOL,
                          bool(use_front)).to_parameter_msg())
        if force_yaw_deg is not None:
            req.parameters.append(
                Parameter('force_yaw_deg', Parameter.Type.DOUBLE,
                          float(force_yaw_deg)).to_parameter_msg())
        if mdist_marker_id is not None:
            req.parameters.append(
                Parameter('mdist_marker_id', Parameter.Type.INTEGER,
                          int(mdist_marker_id)).to_parameter_msg())
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
        """Phase B 한 세그먼트의 NavigateToPose(USD 프레임) 호출.
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

    def _phase_b_params(self, robot_id, is_leader):
        """로봇 역할(leader/follower)에 맞는 Phase B 좌표/노드 파라미터 dict."""
        if is_leader:
            dock_id = self.phase_b_leader_dock_id
            dock_x = self.phase_b_leader_dock_x
            dock_z = self.phase_b_leader_dock_z
            dock_decal_z = self.phase_b_leader_dock_decal_z
            node = self.phase_b_leader_localizer_node
            corridor_id = self.phase_b_leader_corridor_id
            final_id = self.phase_b_leader_final_id
            final_yaw = self.phase_b_leader_final_yaw
        else:
            dock_id = self.phase_b_follower_dock_id
            dock_x = self.phase_b_follower_dock_x
            dock_z = self.phase_b_follower_dock_z
            dock_decal_z = self.phase_b_follower_dock_decal_z
            node = self.phase_b_follower_localizer_node
            corridor_id = self.phase_b_follower_corridor_id
            final_id = self.phase_b_follower_final_id
            final_yaw = self.phase_b_follower_final_yaw
        if not node:
            node = f'/robot_{robot_id}/marker_localizer_node'
        return {
            'is_leader': is_leader,
            'localizer_node': node,
            'dock_id': dock_id, 'dock_x': dock_x, 'dock_z': dock_z,
            'dock_decal_z': dock_decal_z,
            'dockcheck_standoff': self.phase_b_dockcheck_standoff,
            'corridor_id': corridor_id,        # 회랑마커(도크 x 그대로 북진, 중앙에)
            'xn_z': self.phase_b_xn_z,          # 회랑마커 z 라인(=7.075)
            'final_id': final_id, 'final_yaw': final_yaw,  # 진입직전 회전+정렬
        }

    def _run_phase_b(self, robot_id, params):
        """한 로봇의 Phase B 레그(러너 `_run_entry_lead_b`/`_run_entry_follow_b`)를
        localizer ref 전환 + odom/융합 navigate 로 실행한다.

        단계(2026-07-27 재안무): seed_dock→rotate_90→dock_check→corridor_center→
        final_align. 어느 한 단계라도 실패하면 즉시 (False, 사유). 좌표는 USD 월드 프레임.
        **동시 실행**: 오케스트레이터가 이 함수를 두 로봇에 대해 스레드로 병렬 호출한다.
        """
        node = params['localizer_node']
        dock_id = params['dock_id']
        dock_x, dock_z = params['dock_x'], params['dock_z']
        # 도크체크 목표 z = 데칼 z + standoff.
        dock_check_z = params['dock_decal_z'] + params['dockcheck_standoff']
        corridor_id = params['corridor_id']
        corridor_z = params['xn_z']         # 회랑마커 z 라인(=7.075). 여기까지 북진.
        final_id, final_yaw = params['final_id'], params['final_yaw']

        for phase in phase_b_robot_phases(params['is_leader']):
            label = f'phase_b:{phase}[{robot_id}]'
            if phase == 'seed_dock':
                # ref_ids=[dock_id] 하드필터 + **yaw 보정 ON**(2026-07-27: 위치전용
                # 으론 다중미터 주행에 yaw 몇 도 드리프트가 안 잡혀 dock_check 가 얼어붙음
                # — 정면 마커로 yaw까지 보정해야 수렴/1° 달성). rotate_90 은 오도라 무관.
                ok, reason = self._set_localizer_ref(node, [dock_id], True, label)
            elif phase == 'rotate_90':
                # 제자리 회전해 북향(yaw 0). **오도 인스턴스**(회전 중 마커 상실).
                ok, reason = self._navigate_phase_b(
                    robot_id, 'nav_odom', self.navigate_odom_action,
                    dock_x, dock_z, 0.0, label)
            elif phase == 'dock_check':
                # 후방캠 융합으로 북진하며 도크 데칼 위치보정(ref=[dock_id] 유지).
                # 요 정밀 불필요(다음 corridor_center 가 재정렬) — 메카넘 요 데드밴드(~1°)가
                # 기본 tol 1.0° 에 딱 걸려 status=6 플레이크(실측) 나므로 완화한다.
                # final_align 1° 게이트는 노드기본 tol(1.0) 그대로라 영향 없음.
                ok, reason = self._navigate_phase_b(
                    robot_id, 'nav_fused', self.navigate_fused_action,
                    dock_x, dock_check_z, 0.0, label, yaw_tol=self.dock_check_yaw_tol)
            elif phase == 'corridor_center':
                # ref 를 회랑마커(lead=XN31/follow=LANE_3 63)로 전환 후, 도크 x 그대로
                # 회랑 z 라인(7.075)까지 북진 — 마커를 로봇 정중앙에. yaw=0(북향).
                ok, reason = self._set_localizer_ref(node, [corridor_id], True, label)
                if ok:
                    ok, reason = self._navigate_phase_b(
                        robot_id, 'nav_fused', self.navigate_fused_action,
                        dock_x, corridor_z, 0.0, label)
            elif phase == 'final_align':
                # ref 를 최종마커(lead=62/follow=64)로 전환 후, 제자리 회전해 최종
                # 방향(lead=서쪽 -90 / follow=동쪽 +90) 정렬. yaw_tol=1.0 → 1° 게이트.
                # ponytail: 위치는 (dock_x, corridor_z) 유지 — 회전+마커 yaw보정만.
                #   진입 정밀은 뎁스가 맡으니 여기선 방향·1° 정렬이 목적.
                ok, reason = self._set_localizer_ref(node, [final_id], True, label)
                if ok:
                    ok, reason = self._navigate_phase_b(
                        robot_id, 'nav_fused', self.navigate_fused_action,
                        dock_x, corridor_z, final_yaw, label)
            else:  # pragma: no cover
                ok, reason = False, f'알 수 없는 Phase B 단계: {phase}'
            if not ok:
                return False, f'{label} 실패: {reason}'
        return True, None

    # ---- 복귀(RETURN): 주차 후 두 로봇을 원래 도크로 (Phase B 역순, 독립주행) ----

    def _return_localizer_node(self, rid, leader_id):
        return ((self.phase_b_leader_localizer_node if rid == leader_id
                 else self.phase_b_follower_localizer_node)
                or f'/robot_{rid}/marker_localizer_node')

    def _return_egress_to_corridor(self, rid, is_leader, dock_x, corridor_id, leader_id):
        """슬롯(차밑)→회랑(dock_x, 7.075) 정렬. lead=뎁스 egress, follow=마커 북진.
        회랑 도달 후 서향(yaw −90)으로 90° 회전(다음 서진 준비)."""
        node = self._return_localizer_node(rid, leader_id)
        slot_x = self.park_slot_x
        corr_z = self.phase_b_xn_z
        slot_ref = [self.park_slot_lane_id, self.park_slot_front_id, self.park_slot_center_id]
        if is_leader:
            # 남향(yaw180)→북향(yaw0) 제자리 회전(odom). 그 뒤 뎁스 중앙유지로 북진 이탈.
            ok, reason = self._navigate_phase_b(
                rid, 'nav_odom', self.navigate_odom_action,
                slot_x, self.return_lead_parked_z, 0.0, f'return:lead-rotate[{rid}]')
            if not ok:
                return False, reason
            ok, reason = self._egress(rid, corr_z)          # 뎁스 egress → (slot_x, ~corr_z)
        else:
            # follow: 얕아서 마커(slot_ref, 양캠)로 북진 이탈.
            ok, reason = self._set_localizer_ref(
                node, slot_ref, True, f'return:egress-ref[{rid}]', use_front=True)
            if ok:
                ok, reason = self._navigate_phase_b(
                    rid, 'nav_fused', self.navigate_fused_action,
                    slot_x, corr_z, 0.0, f'return:egress[{rid}]')
        if not ok:
            return False, reason
        # 서향 90° 회전(제자리, odom) — 위치 유지, yaw 0→−90.
        ok, reason = self._navigate_phase_b(
            rid, 'nav_odom', self.navigate_odom_action,
            slot_x, corr_z, -90.0, f'return:turn-west[{rid}]')
        if not ok:
            return False, reason
        # 회랑 바닥마커(61~64+회랑마커)로 전환 후 서진(양캠, yaw −90 유지).
        ok, reason = self._set_localizer_ref(
            node, self.park_lane_marker_ids + [corridor_id], True,
            f'return:corridor-ref[{rid}]', use_front=True)
        if ok:
            ok, reason = self._navigate_phase_b(
                rid, 'nav_fused', self.navigate_fused_action,
                dock_x, corr_z, -90.0, f'return:corridor-west[{rid}]')
        return ok, reason

    def _return_dock(self, rid, is_leader, leader_id):
        """회랑(dock_x, 7.075, −90)→북향 회전→후진(−z)으로 도크 진입→동향90 복원."""
        node = self._return_localizer_node(rid, leader_id)
        dock_id = self.phase_b_leader_dock_id if is_leader else self.phase_b_follower_dock_id
        dock_x = self.phase_b_leader_dock_x if is_leader else self.phase_b_follower_dock_x
        dock_z = self.phase_b_leader_dock_z if is_leader else self.phase_b_follower_dock_z
        corr_z = self.phase_b_xn_z
        # 북향 90° 회전(제자리, odom): yaw −90→0. 이후 북향에서 후진하면 world −z(남진).
        ok, reason = self._navigate_phase_b(
            rid, 'nav_odom', self.navigate_odom_action,
            dock_x, corr_z, 0.0, f'return:turn-north[{rid}]')
        if not ok:
            return False, reason
        # 도크마커로 ref 전환 후 후진 도크진입(양캠, yaw 0 유지). pose_controller 가 −z 로 몬다.
        ok, reason = self._set_localizer_ref(
            node, [dock_id], True, f'return:dock-ref[{rid}]', use_front=True)
        if ok:
            ok, reason = self._navigate_phase_b(
                rid, 'nav_fused', self.navigate_fused_action,
                dock_x, dock_z, 0.0, f'return:dock[{rid}]')
        if not ok:
            return False, reason
        # 동향(yaw 90) 복원(제자리, odom) — 다음 입차 seed(yaw90)와 자세 일치.
        return self._navigate_phase_b(
            rid, 'nav_odom', self.navigate_odom_action,
            dock_x, dock_z, self.return_dock_yaw, f'return:dock-face[{rid}]')

    def _run_return(self, goal_handle, leader_id, follower_id, idx, total):
        """주차 후 복귀: 순차 이탈(follow 북쪽이라 먼저→lead) 후 동시 도크 안착.
        반환 (ok, reason, idx). 이탈은 같은 x=slot_x 라인이라 순차(충돌 회피),
        도크 주행은 회랑에서 x 가 갈려(-1.2/-3.2) 동시."""
        # Stage 1: 순차 이탈 (follow 먼저 — 주차 시 북쪽=출구에 가까움, lead 는 남쪽/깊음)
        self._publish_feedback(goal_handle, 'RETURN_FOLLOW_OUT', idx, total)
        ok, reason = self._return_egress_to_corridor(
            follower_id, False, self.phase_b_follower_dock_x,
            self.phase_b_follower_corridor_id, leader_id)
        if not ok:
            return False, f'복귀 이탈(follow) 실패: {reason}', idx
        idx += 1
        self._publish_feedback(goal_handle, 'RETURN_LEAD_OUT', idx, total)
        ok, reason = self._return_egress_to_corridor(
            leader_id, True, self.phase_b_leader_dock_x,
            self.phase_b_leader_corridor_id, leader_id)
        if not ok:
            return False, f'복귀 이탈(lead) 실패: {reason}', idx
        idx += 1

        # Stage 2: 동시 도크 안착 (Phase B 병렬 스레드 패턴 재사용)
        self._publish_feedback(goal_handle, 'RETURN_DOCK', idx, total)
        dock = {}

        def _leg(rid, is_leader):
            dock[rid] = self._return_dock(rid, is_leader, leader_id)

        threads = [threading.Thread(target=_leg, args=(leader_id, True), daemon=True),
                   threading.Thread(target=_leg, args=(follower_id, False), daemon=True)]
        for t in threads:
            t.start()
        for t in threads:
            t.join()
        idx += 1
        for rid in (leader_id, follower_id):
            ok, reason = dock.get(rid, (False, f'{rid} 도크 복귀 미완'))
            if not ok:
                return False, f'복귀 도크안착 실패: {reason}', idx
        return True, None, idx

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
        req = _AutoRequest(self._auto_leader, self._auto_follower, self._auto_task_id,
                           self._auto_slot_id)
        handle = _AutoGoalHandle(req, self.get_logger())
        result = self._on_execute_parking_task(handle)
        self.get_logger().info(
            f'auto_start: 미션 종료 success={result.success} message={result.message}')

    def _on_execute_parking_task(self, goal_handle):
        goal = goal_handle.request
        # goal 에 로봇이 비면 노드 파라미터(auto_leader/auto_follower) 기본값 사용 —
        # action call 에 slot_id 만 줘도 되게(2026-07-27). 둘 다 비면 아래 가드가 잡는다.
        leader_id = goal.leader_robot_id or self.get_parameter('auto_leader').value
        follower_id = goal.follower_robot_id or self.get_parameter('auto_follower').value

        if not leader_id or not follower_id:
            msg = ('execute_pickup_choreography 는 leader_robot_id/follower_robot_id '
                   f'둘 다 필요합니다(받음: leader={leader_id!r} follower={follower_id!r})')
            self.get_logger().warn(msg)
            goal_handle.abort()
            result = ExecuteParkingTask.Result(success=False, message=msg)
            return result

        self.get_logger().info(
            f'execute_pickup_choreography 시작: task_id={goal.task_id} '
            f'leader={leader_id} follower={follower_id}')

        # goal.slot_id('A1'/'A2'/'A3')로 목표 슬롯 선택 → 좌표·마커 매핑. 빈 값이면
        # 노드 파라미터(park_slot_*) 기본값 유지(테스트 호환). 한 번에 한 미션이라
        # self 에 확정해 아래 CARRY 흐름이 그대로 집어쓴다.
        if goal.slot_id:
            slot = PARK_SLOTS.get(goal.slot_id.strip().upper())
            if slot is None:
                msg = f'알 수 없는 slot_id={goal.slot_id!r} (지원: {sorted(PARK_SLOTS)})'
                self.get_logger().warn(msg)
                goal_handle.abort()
                return ExecuteParkingTask.Result(success=False, message=msg)
            (self.park_slot_x, self.park_slot_lane_id,
             self.park_slot_center_id, self.park_slot_front_id) = slot
            self.get_logger().info(
                f'목표 슬롯 {goal.slot_id.strip().upper()}: x={self.park_slot_x} '
                f'lane_id={self.park_slot_lane_id} center_id={self.park_slot_center_id}')

        # 뎁스캠 게이팅 퍼블리셔를 미션 시작(=approach 보다 한참 전, Phase B 앞)에
        # 미리 만들어 sim_bridge 구독자와 DDS 매칭을 끝내둔다 — approach 에서 처음
        # publish 할 때 discovery 레이스로 첫 True 가 유실되지 않게(reliable QoS 는
        # 매칭 이후에만 전달 보장).
        for rid in (leader_id, follower_id):
            self._depth_enable(rid, False)

        # 진행률 분모: Phase B 스텝 + 진입 2 + 리프트UP + 운반 + 안착(DOWN).
        pb_steps = phase_b_plan(leader_id, follower_id) if self.run_phase_b_first else []
        total = len(pb_steps) + 2 + 4 + 3   # +진입2 +리프트UP +운반2 +안착DOWN +복귀3

        idx = 0
        fail_reason = None

        # ---- Phase B(도크→회랑정렬): 두 로봇 **동시** 실행 ----
        # 2026-07-27 재안무: lead/follow 를 스레드로 병렬 실행, join = 배리어(둘 다
        # final_align 1° 이내 정렬해야 진입). MultiThreadedExecutor+ReentrantCallbackGroup
        # 이라 두 레그의 폴링 액션호출이 동시에 진행된다. 로봇별로 액션클라이언트/토픽/
        # localizer 노드가 갈려 상호간섭 없음. ponytail: _client 캐시 dict 를 두 스레드가
        # 채우지만 키가 로봇별로 달라 경합 없음(같은 키 동시생성 나면 lock 추가).
        if self.run_phase_b_first:
            self._publish_feedback(goal_handle, 'PHASE_B_CONCURRENT', idx, total)
            self.get_logger().info(
                f'Phase B 동시 시작: leader={leader_id} follower={follower_id}')
            legs = {}   # rid -> (ok, reason)

            def _leg(rid, is_leader):
                legs[rid] = self._run_phase_b(rid, self._phase_b_params(rid, is_leader))

            threads = [threading.Thread(target=_leg, args=(leader_id, True), daemon=True),
                       threading.Thread(target=_leg, args=(follower_id, False), daemon=True)]
            for t in threads:
                t.start()
            for t in threads:
                t.join()   # 배리어: 둘 다 정렬 끝나야 다음(진입)
            idx += len(phase_b_robot_phases(True)) + len(phase_b_robot_phases(False))
            for rid in (leader_id, follower_id):
                ok, reason = legs.get(rid, (False, f'{rid} Phase B 레그 미완'))
                if not ok:
                    self._publish_feedback(goal_handle, 'FAILED', idx, total)
                    self.get_logger().warn(f'execute_pickup_choreography 실패(Phase B): {reason}')
                    goal_handle.abort()
                    return ExecuteParkingTask.Result(success=False, message=reason)
            self.get_logger().info('Phase B 동시 완료: 두 로봇 최종 정렬(1° 이내)')

            # Phase B 종료 후 진입 전: 두 로봇 localizer ref 를 **회랑마커(61~64)** 로
            # 켜고 correct_yaw=True. (2026-07-28 사용자 지시로 재설계)
            # 왜 켜나: 진입은 이제 **직진**(옛 XN 90° 회전 approach 없음)이라, 트럭 진입
            # 경로 바닥에 깔린 회랑마커 61~64 를 down 캠이 깨끗이 본다. 진입 중 이 마커로
            # yaw 를 보정해야(ingress_node hold_yaw), 먼 마커서 정렬 후 트럭까지 요가
            # 드리프트해 바퀴에 부딪히던 문제가 풀린다(옛 마커오염 근거=회전 중 XN 극단
            # 각도였는데 직진 진입엔 해당 없음). 정지는 여전히 뎁스 축검출(같은 융합
            # 프레임 상대정지). 마커는 x=-6.75(61)까지만이라 그보다 깊은 구간은 오도예측.
            for rid in (leader_id, follower_id):
                node = ((self.phase_b_leader_localizer_node if rid == leader_id
                         else self.phase_b_follower_localizer_node)
                        or f'/robot_{rid}/marker_localizer_node')
                self._set_localizer_ref(node, self.park_lane_marker_ids, True,
                                        f'ingress-corridor-ref[{rid}]', use_front=True)
        # ---- 트럭 밑 진입: **순차(스태거)** (2026-07-27 사용자 지시) ----
        # lead 가 **먼저** 들어가 안쪽(더 깊은) 축(leader_trough_index=1)에 자리잡고, 그
        # 다음에야 follow 가 들어가 바깥쪽 축(follower_trough_index=0)에 멈춘다. 이 순서면
        # follow 가 lead 를 지나칠 필요가 없어(lead 가 더 깊이 있음) 같은 z 중심선에서도
        # 충돌 없음. 동시진입 대비 (1) 충돌위험 원천차단 (2) 한 번에 로봇 1대 뎁스만 렌더
        # → RTF 부담↓(저rtf 진입 신뢰도↑). 리프트는 follow 까지 자리잡은 뒤(둘 다 그립).
        # 주의: 이 순서는 lead 축이 follow 축보다 깊을 때만 안전 — GUI 에서 follow 가 lead
        # 를 지나쳐야 하면 순서를 뒤집어야 한다(trough/진입순서 swap).
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

        self._publish_feedback(goal_handle, 'INGRESS_LEAD', idx, total)
        fail = _do_ingress(leader_id, self.leader_trough_index, 'lead(안쪽축)')
        if fail is not None:
            return fail
        idx += 1
        self._publish_feedback(goal_handle, 'INGRESS_FOLLOW', idx, total)
        fail = _do_ingress(follower_id, self.follower_trough_index, 'follow(바깥축)')
        if fail is not None:
            return fail
        idx += 1
        self.get_logger().info('순차 진입 완료: lead(안쪽)→follow(바깥) 둘 다 축 정렬')

        # ---- 동시 리프트(브리프 필수 지시) ----
        self._publish_feedback(goal_handle, 'LIFT_UP', idx, total)
        ok, reason = self._lift_concurrent(leader_id, follower_id, 'UP')
        if not ok:
            self._publish_feedback(goal_handle, 'FAILED', idx, total)
            fail_reason = f'concurrent lift UP 실패: {reason}'
            self.get_logger().warn(f'execute_pickup_choreography 실패: {fail_reason}')
            goal_handle.abort()
            return ExecuteParkingTask.Result(success=False, message=fail_reason)

        # ---- Stage 4 운반: follow rear 기준 (2026-07-27 재작성, 사용자 지시) ----
        # 차 든 후 follow marker_localizer 를 rear-only(use_front=False) + yaw90 시딩
        # (force_yaw_deg=90) + correct_yaw=True(rear 레인마커로 yaw 보정)로 켠다. ref=
        # 경로 레인마커(61~64) + 목표 주차앞 마커(slot lane). mdist 는 주차앞 마커에만
        # → follow rear 가 그걸 봐야 정지. lead 는 -v 만이라 localizer 설정 불필요.
        follow_node = (self.phase_b_follower_localizer_node
                       or f'/robot_{follower_id}/marker_localizer_node')
        carry_ref = self.park_lane_marker_ids + [self.park_slot_lane_id]
        # mdist=-1: ref 마커(61~64,3) **아무거나** 보면 ref_marker_dist 발행 → carry 가
        # "follow rear 가 방금 측위했나"(=옆·yaw 보정 켜는 신호)로 쓴다(특정 도착마커 아님).
        self._set_localizer_ref(follow_node, carry_ref, True,
                                f'carry-follow[{follower_id}]',
                                mdist_marker_id=-1,
                                use_front=False, force_yaw_deg=90.0)
        # ① 운반: 주차앞(park_slot_x, park_lane_z=7.075)로 yaw90 유지하며 이동, follow
        #    rear 가 주차앞 마커 보고 x 도달하면 정지(회전·진입은 SP3 주차에서).
        self._publish_feedback(goal_handle, 'CARRY_LANE', idx, total)
        ok, reason = self._carry(leader_id, follower_id,
                                 self.park_slot_x, self.park_lane_z, 90.0,
                                 f'carry-lane[{leader_id}+{follower_id}]')
        if not ok:
            self._publish_feedback(goal_handle, 'FAILED', idx, total)
            self.get_logger().warn(f'execute_pickup_choreography 실패: {reason}')
            goal_handle.abort()
            return ExecuteParkingTask.Result(success=False, message=f'운반(lane) 실패: {reason}')
        idx += 1
        # 슬롯 마커로 ref 전환(follow 만). **use_front=True 로 전환(2026-07-28 로그근거)**:
        # 회전 끝나면 follow 가 북향→rear 는 남쪽(슬롯 65/66)을, front 는 북쪽(레인마커
        # 3·61~64)을 본다. rear-only 였을 땐 하강 초반(z≈6.5) 남쪽 슬롯마커가 멀어 블라인드
        # →odom yaw 드리프트→align_gate 오발동→헛회전으로 트럭을 옆으로 밀어버렸다(실측
        # CARRY_SLOT 로그: mark=0 구간에서 center perp 0.95→1.57). front 를 켜면 하강 내내
        # 북쪽 레인마커로 pose 를 앵커해 그 블라인드/헛회전을 없앤다. ref 에 레인마커(61~64)
        # 도 추가해 front 가 무엇을 보든 잡히게. rear 는 그대로 65/66(막판 정지 기준) 담당.
        # correct_yaw=True. force_yaw 는 안 줌(필터 현 yaw 유지 → carry 가 90→0 회전 몬다).
        # mdist=-1: ref 아무거나 보면 ref_marker_dist 발행 = carry 의 "측위됨" 신호.
        slot_ref = self.park_lane_marker_ids + [
            self.park_slot_lane_id, self.park_slot_front_id, self.park_slot_center_id]
        self._set_localizer_ref(follow_node, slot_ref, True,
                                f'carry-slot-marker[{follower_id}]',
                                mdist_marker_id=-1, use_front=True)
        # ② 주차: carry 한 세그먼트가 트럭을 90°→0°(N-S) 제자리 회전(회전선행 게이트)
        #    후 슬롯 가운데(park_center_z=0)까지 -z 진입. follow rear 가 가운데마커(66)
        #    를 보고(래치) 중점이 (slot_x,0)·yaw0 도달하면 정지.
        self._publish_feedback(goal_handle, 'CARRY_SLOT', idx, total)
        ok, reason = self._carry(leader_id, follower_id,
                                 self.park_slot_x, self.park_center_z, 0.0,
                                 f'carry-slot[{leader_id}+{follower_id}]')
        if not ok:
            self._publish_feedback(goal_handle, 'FAILED', idx, total)
            self.get_logger().warn(f'execute_pickup_choreography 실패: {reason}')
            goal_handle.abort()
            return ExecuteParkingTask.Result(success=False, message=f'주차진입 실패: {reason}')

        self._publish_feedback(goal_handle, 'PLACE', idx, total)
        ok, reason = self._lift_concurrent(leader_id, follower_id, 'DOWN')
        if not ok:
            self._publish_feedback(goal_handle, 'FAILED', idx, total)
            self.get_logger().warn(f'execute_pickup_choreography 실패(안착): {reason}')
            goal_handle.abort()
            return ExecuteParkingTask.Result(success=False, message=f'안착(lift DOWN) 실패: {reason}')

        # ---- 복귀: 두 로봇을 원래 도크로 (Phase B 역순, 순차이탈→동시안착) ----
        idx = total - 3   # 남은 3틱 = 복귀(follow이탈/lead이탈/도크안착)
        ok, reason, idx = self._run_return(goal_handle, leader_id, follower_id, idx, total)
        if not ok:
            self._publish_feedback(goal_handle, 'FAILED', idx, total)
            self.get_logger().warn(f'execute_pickup_choreography 실패(복귀): {reason}')
            goal_handle.abort()
            return ExecuteParkingTask.Result(success=False, message=reason)

        idx = total
        self._publish_feedback(goal_handle, 'DONE', idx, total)
        self.get_logger().info(
            f'execute_pickup_choreography 완료(복귀 포함): task_id={goal.task_id} '
            f'leader={leader_id} follower={follower_id}')
        goal_handle.succeed()
        return ExecuteParkingTask.Result(
            success=True, message='입차+복귀 완료(픽업+리프트+운반+안착+도크복귀)')


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
