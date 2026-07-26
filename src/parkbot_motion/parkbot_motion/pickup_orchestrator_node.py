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

## 안무(``pickup_choreography.corridor_plan`` 그대로 실행)

1. **스태거링된 회랑 순회**(브리프 필수 지시, 회랑=트럭 밑 통로):
   follower 전체(approach->align->ingress) 완주 -> 그 다음에야 leader 전체
   시작. ``pickup_choreography.CorridorStaggerGuard`` 로 명시적으로 강제한다
   (다른 로봇이 아직 못 비운 회랑에 진입하면 즉시 ``RuntimeError`` -- 이게
   나면 안무 로직 자체의 버그다, 정상 경로에서는 절대 발생하지 않아야 함).
   - **approach**: ``/robot_<id>/navigate_to_pose``(``pose_controller_node``
     기본 인스턴스, ``/robot_<id>/odom`` 자세 소스)로 베이 진입 정렬 자세
     (기본 x=-3.50,z=7.075,yaw=-90 -- 러너 ``APPROACH_X``/``z_center_truck``/
     ``APPROACH_YAW`` 실측값과 동일, 파라미터로 오버라이드 가능)까지 이동+회전.
     **회전 포즈소스 전환 메커니즘**: 이 단계가 바로 브리프가 말한 "회전이
     필요하면 odom 소스로 전환"이다 -- odom 기반 인스턴스를 쓰는 것 자체가
     그 전환이다(taskR3c-report.md §5.3: ``YAW_ODOM_SCALE`` 배선 후 스윕
     오차 +3%, 마커융합으로는 애초에 회전을 못 끝냄, §7). 별도 파라미터
     set_parameters 호출로 "전환"하지 않는다 -- ``pose_controller_node`` 가
     구독 토픽을 초기화 시점에 고정하므로(런타임 재구독 미지원), **로봇마다
     인스턴스 2개**(odom 용/융합용, 서로 다른 ``action_name``)를 띄워두고
     오케스트레이터가 "이번 단계엔 어느 액션을 부를지"로 전환을 대신한다.
   - **align**: ``/robot_<id>/navigate_to_pose_fused``(``pose_controller_node``
     의 두 번째 인스턴스, ``pose_topic=/robot_<id>/pose``(``marker_localizer_node``
     ``fuse:=true`` 융합), ``pose_msg_type=posestamped``)로 **같은** 목표 자세에
     한 번 더 수렴 -- approach 가 이미 목표 yaw 에 도달해 있으므로 이 단계의
     회전 델타는 ~0°(§ 위 회전 메커니즘 설명과 동일 근거로, R3c 가 실패한
     "±90° 회전을 융합만으로" 케이스에 해당하지 않는다), 위치만 마커 피드백으로
     정밀 보정한다(taskR3c-report.md §5.4: GT 오차 19cm->2.3~2.6cm). **베스트
     에포트**: 이 단계가 실패(타임아웃/마커 미검출)해도 전체 태스크를 중단하지
     않는다 -- approach 가 이미 odom 만으로 도달해 있고(§5.3 스윕 오차 +3%,
     순수이동 오차도 8cm 대), 통로 여유(±16.5cm)에 여전히 들어가는 근사값이기
     때문(정직하게: 최종 진입 정밀도가 마커 보정 없이는 다소 떨어질 수 있음,
     보고서에 실측 기록).
   - **ingress**: ``/robot_<id>/ingress_under_truck``(trough_index) -- 실패하면
     (align 과 달리) 태스크 전체를 ``FAILED`` 로 중단한다(이 실패는 로봇이
     트럭 밑 정위치에 없다는 뜻이라 뒤이은 리프트가 위험하다).
2. **동시 리프트**(브리프 필수 지시): 두 회랑 순회가 모두 끝난 뒤,
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

from parking_robot_interfaces.action import ControlLift, ExecuteParkingTask, IngressUnderTruck
from parkbot_motion.pickup_choreography import (
    CorridorStaggerGuard, corridor_plan, lift_both_succeeded, phase_b_plan,
    phase_b_robot_phases, run_concurrent,
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
INGRESS_RESULT_TIMEOUT = 330.0
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


class PickupOrchestratorNode(Node):

    def __init__(self):
        super().__init__('pickup_orchestrator_node')

        self.declare_parameter('action_name', 'execute_pickup_choreography')
        # 베이 진입 정렬 자세(러너 실측값, parking_v4_runner.py APPROACH_X/
        # z_center_truck/APPROACH_YAW 와 동일 -- docstring §안무 참고).
        self.declare_parameter('approach_x', -3.50)
        self.declare_parameter('approach_z', 7.075)
        self.declare_parameter('approach_yaw_deg', -90.0)
        # leader=후축(rear)=trough 0, follower=전축(front)=trough 1
        # (taskR5a-report.md 실측: entry_lead->index0, entry_follow->index1).
        self.declare_parameter('leader_trough_index', 0)
        self.declare_parameter('follower_trough_index', 1)
        # 로봇별 액션 이름 접미사 -- ROS2 배관 컨벤션(§ 클래스 docstring "회전
        # 포즈소스 전환 메커니즘"). odom 인스턴스는 pose_controller_node 기본값과
        # 일치해야 별도 -p action_name 없이도 바로 붙는다.
        self.declare_parameter('navigate_odom_action', 'navigate_to_pose')
        self.declare_parameter('navigate_fused_action', 'navigate_to_pose_fused')
        self.declare_parameter('ingress_action', 'ingress_under_truck')
        self.declare_parameter('lift_action', 'control_lift')
        # align(융합) 단계 실패를 치명적으로 볼지 -- 기본 False(베스트에포트,
        # § 클래스 docstring "align" 절 근거).
        self.declare_parameter('align_required', False)

        # ---- Phase B(도크 스폰→XN 융합주행) 레그 파라미터 (R6/T3) ----
        # corridor_plan(픽업 회랑) **앞에** 두 로봇의 도크→XN 레그를 붙일지.
        # 기본 True — bringup_pickup_e2e.sh 가 도크 스폰에서 기동할 때 켠다. False 면
        # 기존처럼 XN 종단 자세에서 곧장 픽업만 실행(하위호환, 기존 스모크/테스트).
        self.declare_parameter('run_phase_b_first', True)
        # XN(크로싱) 마커 — 두 로봇 공용(러너 read_markers["XN"], id31, x=-2.5, z=6.875).
        self.declare_parameter('phase_b_xn_id', 31)
        self.declare_parameter('phase_b_xn_x', -2.5)
        self.declare_parameter('phase_b_xn_z', 6.875)
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
        # 목표(속성 좌표, +2.9), dock_decal_z 는 도크체크 목표 산정용(데칼은 속성보다
        # z 로 0.7m 남쪽 — 러너 marker_visual_center 실측, 기본 2.2). 각 값은 T4/T5
        # bringup 이 실측으로 오버라이드할 수 있게 파라미터로 노출.
        self.declare_parameter('phase_b_leader_dock_id', 21)
        self.declare_parameter('phase_b_leader_dock_x', -3.2)
        self.declare_parameter('phase_b_leader_dock_z', 2.9)
        self.declare_parameter('phase_b_leader_dock_decal_z', 2.2)
        self.declare_parameter('phase_b_follower_dock_id', 23)
        self.declare_parameter('phase_b_follower_dock_x', -1.2)
        self.declare_parameter('phase_b_follower_dock_z', 2.9)
        self.declare_parameter('phase_b_follower_dock_decal_z', 2.2)
        # 융합 localizer 인스턴스의 **완전수식 노드명** — 크로스노드 set_parameters
        # (ref_ids/correct_yaw)의 서비스 대상 `<node_name>/set_parameters`. 빈 값이면
        # 런타임에 `/robot_<id>/marker_localizer_node` 로 유도한다(T4 bringup 이 로봇별
        # localizer 를 이 네임스페이스/노드명으로 띄우는 것을 전제 — 현재 bringup 은
        # 두 인스턴스가 같은 노드명 `marker_localizer_node` 라 T4 가 로봇별로
        # 구분해줘야 한다. 보고서 우려 참조).
        self.declare_parameter('phase_b_leader_localizer_node', '')
        self.declare_parameter('phase_b_follower_localizer_node', '')

        gp = self.get_parameter
        self.action_name = gp('action_name').value
        self.approach_x = float(gp('approach_x').value)
        self.approach_z = float(gp('approach_z').value)
        self.approach_yaw_deg = float(gp('approach_yaw_deg').value)
        self.leader_trough_index = int(gp('leader_trough_index').value)
        self.follower_trough_index = int(gp('follower_trough_index').value)
        self.navigate_odom_action = gp('navigate_odom_action').value
        self.navigate_fused_action = gp('navigate_fused_action').value
        self.ingress_action = gp('ingress_action').value
        self.lift_action = gp('lift_action').value
        self.align_required = bool(gp('align_required').value)

        # ---- Phase B 파라미터 캐시 ----
        self.run_phase_b_first = bool(gp('run_phase_b_first').value)
        self.phase_b_xn_id = int(gp('phase_b_xn_id').value)
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

        self._cbg = ReentrantCallbackGroup()
        # 이름 주의: rclpy.node.Node 가 이미 인스턴스 속성 ``self._clients``
        # (서비스 클라이언트 리스트, ``create_client``/``destroy_node`` 내부용,
        # ``Node.__init__`` 에서 ``[]`` 로 초기화)를 쓴다 -- 여기서 같은 이름으로
        # 덮어쓰면(원래 버그였음) ``destroy_node()`` 의
        # ``self.destroy_client(self._clients[0])`` 가 dict 를 정수 0 으로
        # 인덱싱하다 ``KeyError: 0`` 로 죽는다(taskR5bfix 진단 그대로). 액션
        # 클라이언트 캐시라 이름도 구분해 ``_action_clients`` 로 둔다.
        self._action_clients = {}  # (robot_id, action_key) -> ActionClient
        # 크로스노드 set_parameters(SetParameters 서비스) 클라이언트 캐시(노드명별).
        # Node.create_client 는 내부 self._clients(서비스 클라이언트 리스트)에도
        # 등록하지만, 우리 캐시는 별도 dict 라 그 리스트를 덮어쓰지 않는다(§ 위
        # _action_clients 이름주의 주석과 동일 근거).
        self._param_clients = {}  # node_name -> SetParameters 서비스 클라이언트

        self._execute_task_server = ActionServer(
            self, ExecuteParkingTask, self.action_name, self._on_execute_parking_task,
            callback_group=self._cbg)

        self.get_logger().info(
            f'pickup_orchestrator_node 시작: action={self.action_name} '
            f'approach=(x={self.approach_x},z={self.approach_z},yaw={self.approach_yaw_deg}) '
            f'leader_trough={self.leader_trough_index} follower_trough={self.follower_trough_index}')

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

    def _approach(self, robot_id):
        client = self._client(robot_id, 'nav_odom', NavigateToPose, self.navigate_odom_action)
        goal = _navigate_goal(self.get_clock(), self.approach_x, self.approach_z,
                               self.approach_yaw_deg)
        result, status, reason = self._call_action(
            client, goal, label=f'approach[{robot_id}]', result_timeout=NAVIGATE_RESULT_TIMEOUT)
        if result is None:
            return False, reason
        if status != GoalStatus.STATUS_SUCCEEDED:
            return False, f'approach[{robot_id}] 실패(status={status})'
        return True, None

    def _align(self, robot_id):
        client = self._client(robot_id, 'nav_fused', NavigateToPose, self.navigate_fused_action)
        goal = _navigate_goal(self.get_clock(), self.approach_x, self.approach_z,
                               self.approach_yaw_deg)
        result, status, reason = self._call_action(
            client, goal, label=f'align[{robot_id}]', result_timeout=NAVIGATE_RESULT_TIMEOUT)
        if result is None:
            return False, reason
        if status != GoalStatus.STATUS_SUCCEEDED:
            return False, f'align[{robot_id}] 실패(status={status})'
        return True, None

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

    def _navigate_phase_b(self, robot_id, action_key, action_suffix, x, z, yaw_deg, label):
        """Phase B 한 세그먼트의 NavigateToPose(USD 프레임) 호출. approach/align 과
        동일한 ``_client``/``_navigate_goal``/``_call_action`` 을 그대로 재사용한다."""
        client = self._client(robot_id, action_key, NavigateToPose, action_suffix)
        goal = _navigate_goal(self.get_clock(), x, z, yaw_deg)
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
        else:
            dock_id = self.phase_b_follower_dock_id
            dock_x = self.phase_b_follower_dock_x
            dock_z = self.phase_b_follower_dock_z
            dock_decal_z = self.phase_b_follower_dock_decal_z
            node = self.phase_b_follower_localizer_node
        if not node:
            node = f'/robot_{robot_id}/marker_localizer_node'
        return {
            'is_leader': is_leader,
            'localizer_node': node,
            'dock_id': dock_id, 'dock_x': dock_x, 'dock_z': dock_z,
            'dock_decal_z': dock_decal_z,
            'dockcheck_standoff': self.phase_b_dockcheck_standoff,
            'xn_id': self.phase_b_xn_id, 'xn_x': self.phase_b_xn_x,
            'xn_z': self.phase_b_xn_z, 'xn_standoff': self.phase_b_xn_standoff,
            'final_x_offset': self.phase_b_final_x_offset,
        }

    def _run_phase_b(self, robot_id, params):
        """한 로봇의 Phase B 레그(러너 `_run_entry_lead_b`/`_run_entry_follow_b`)를
        localizer ref 전환 + odom/융합 navigate 로 실행한다.

        단계(phase_b_robot_phases): seed_dock→rotate_90→dock_check→xn_align_x→
        xn_align_z→(leader 만)offset. 어느 한 단계라도 실패하면 즉시 (False, 사유).
        모든 목표는 yaw=0(북향), 좌표는 USD 월드 프레임.
        """
        node = params['localizer_node']
        dock_id, xn_id = params['dock_id'], params['xn_id']
        dock_x, dock_z = params['dock_x'], params['dock_z']
        # 도크체크 목표 z = 데칼 z + standoff (러너 seg1_target = dock_decal[1]+standoff).
        dock_check_z = params['dock_decal_z'] + params['dockcheck_standoff']
        xn_x = params['xn_x']
        # XN 정렬/오프셋 종단 z = xn_z - standoff (러너 seg2_target).
        xn_align_z = params['xn_z'] - params['xn_standoff']

        for phase in phase_b_robot_phases(params['is_leader']):
            label = f'phase_b:{phase}[{robot_id}]'
            if phase == 'seed_dock':
                # ref_ids=[dock_id] 하드필터 + 위치전용 보정(correct_yaw=False).
                # 융합 localizer 는 첫 도크 fix 로 자기 시딩한다(SEED_DOCK 은 ref 전환).
                ok, reason = self._set_localizer_ref(node, [dock_id], False, label)
            elif phase == 'rotate_90':
                # 제자리 90° 회전(+X→+Z, yaw 0). **오도 인스턴스**(융합은 회전 중
                # 마커 상실 — R3c §7). 위치는 도크 스폰(dock_x, dock_z) 유지.
                ok, reason = self._navigate_phase_b(
                    robot_id, 'nav_odom', self.navigate_odom_action,
                    dock_x, dock_z, 0.0, label)
            elif phase == 'dock_check':
                # 후방캠 융합으로 북진하며 도크 데칼 위치보정(ref 는 seed_dock 에서
                # 이미 [dock_id]). 융합 인스턴스 사용.
                ok, reason = self._navigate_phase_b(
                    robot_id, 'nav_fused', self.navigate_fused_action,
                    dock_x, dock_check_z, 0.0, label)
            elif phase == 'xn_align_x':
                # ref 를 XN(31)로 전환(전방캠 검출) 후, x 만 xn_x 로 정렬(z 유지).
                ok, reason = self._set_localizer_ref(node, [xn_id], False, label)
                if ok:
                    ok, reason = self._navigate_phase_b(
                        robot_id, 'nav_fused', self.navigate_fused_action,
                        xn_x, dock_check_z, 0.0, label)
            elif phase == 'xn_align_z':
                # x=xn_x 고정, 순수 북진해 XN 남쪽 standoff 로(검출창 통과).
                ok, reason = self._navigate_phase_b(
                    robot_id, 'nav_fused', self.navigate_fused_action,
                    xn_x, xn_align_z, 0.0, label)
            elif phase == 'offset':
                # leader 전용 충돌회피 x 오프셋 → 최종 대기자세.
                ok, reason = self._navigate_phase_b(
                    robot_id, 'nav_fused', self.navigate_fused_action,
                    xn_x + params['final_x_offset'], xn_align_z, 0.0, label)
            else:  # pragma: no cover -- phase_b_robot_phases 만이 이 루프를 채운다
                ok, reason = False, f'알 수 없는 Phase B 단계: {phase}'
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
            f'execute_pickup_choreography 시작: task_id={goal.task_id} '
            f'leader={leader_id} follower={follower_id}')

        plan = corridor_plan(leader_id, follower_id,
                              self.leader_trough_index, self.follower_trough_index)
        # 진행률 분모: 회랑 6단계 + 동시 리프트 1 + (활성 시)Phase B 스텝 수.
        pb_steps = phase_b_plan(leader_id, follower_id) if self.run_phase_b_first else []
        total = len(plan) + 1 + len(pb_steps)
        guard = CorridorStaggerGuard()

        idx = 0
        fail_reason = None

        # ---- Phase B(도크 스폰→XN) 레그: 픽업 회랑 **앞에** 스태거 실행 ----
        # leader(entry_lead) 전체가 먼저 끝난 뒤 follower(entry_follow) 시작
        # (phase_b_plan 스태거 근거). 어느 로봇이라도 실패하면 전체 태스크 FAILED.
        if self.run_phase_b_first:
            for rid, is_leader in ((leader_id, True), (follower_id, False)):
                self._publish_feedback(goal_handle, f'PHASE_B_{rid}', idx, total)
                self.get_logger().info(f'Phase B 레그 시작: robot={rid} leader={is_leader}')
                ok, reason = self._run_phase_b(rid, self._phase_b_params(rid, is_leader))
                if not ok:
                    self._publish_feedback(goal_handle, 'FAILED', idx, total)
                    self.get_logger().warn(f'execute_pickup_choreography 실패(Phase B): {reason}')
                    goal_handle.abort()
                    return ExecuteParkingTask.Result(success=False, message=reason)
                idx += len(phase_b_robot_phases(is_leader))
                self.get_logger().info(f'Phase B 레그 완료: robot={rid}')
        for step in plan:
            phase, robot_id, trough_index = step
            step_label = f'{phase.upper()}_{robot_id}'
            self._publish_feedback(goal_handle, step_label, idx, total)

            if phase == 'approach':
                guard.enter(robot_id)  # 회랑 진입 -- 다른 로봇이 안 비웠으면 예외
                ok, reason = self._approach(robot_id)
                if not ok:
                    fail_reason = reason
                    break
            elif phase == 'align':
                ok, reason = self._align(robot_id)
                if not ok:
                    self.get_logger().warn(
                        f'align[{robot_id}] 베스트에포트 실패(계속 진행): {reason}')
                    if self.align_required:
                        fail_reason = reason
                        break
            elif phase == 'ingress':
                ok, reason = self._ingress(robot_id, trough_index)
                guard.clear(robot_id)  # 성공/실패 무관 -- 회랑에서는 벗어난 상태
                if not ok:
                    fail_reason = reason
                    break
            idx += 1

        if fail_reason is not None:
            self._publish_feedback(goal_handle, 'FAILED', idx, total)
            self.get_logger().warn(f'execute_pickup_choreography 실패: {fail_reason}')
            goal_handle.abort()
            return ExecuteParkingTask.Result(success=False, message=fail_reason)

        # ---- 동시 리프트(브리프 필수 지시) ----
        self._publish_feedback(goal_handle, 'LIFT_UP', idx, total)
        ok, reason = self._lift_concurrent(leader_id, follower_id, 'UP')
        if not ok:
            self._publish_feedback(goal_handle, 'FAILED', idx, total)
            fail_reason = f'concurrent lift UP 실패: {reason}'
            self.get_logger().warn(f'execute_pickup_choreography 실패: {fail_reason}')
            goal_handle.abort()
            return ExecuteParkingTask.Result(success=False, message=fail_reason)

        idx = total
        self._publish_feedback(goal_handle, 'DONE', idx, total)
        self.get_logger().info(
            f'execute_pickup_choreography 완료: task_id={goal.task_id} '
            f'leader={leader_id} follower={follower_id}')
        goal_handle.succeed()
        return ExecuteParkingTask.Result(success=True, message='픽업 완료(양 로봇 진입+동시 리프트)')


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
