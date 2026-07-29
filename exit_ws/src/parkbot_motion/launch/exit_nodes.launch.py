#!/usr/bin/env python3
"""nodes.launch.py — 순수 ROS2 노드 스택.  `ros2 launch parkbot_motion nodes.launch.py`

Isaac(sim_bridge)와는 다른 프로세스/컴퓨터. DDS(도메인 126)로만 대화하므로 순서 무관 —
브리지가 먼저 떠 있으면 노드들이 바로 붙는다. orchestrator 는 auto_start 로 자율 미션.

env(도메인/RMW)는 각 Node 에 additional_env 로 **직접** 박는다 — SetEnvironmentVariable
만으론 이 셋업에서 노드 서브프로세스까지 안 내려가 브리지와 도메인이 갈렸다(구독 0).

FastDDS 화이트리스트 **필수** — rokey 머신 실측(2026-07-27 재확인): builtin 전송이면
크로스버전 FastDDS(Isaac 번들 vs 시스템 Humble)의 SHM 데이터전송이 비호환이라
discovery 로 토픽은 보여도 데이터가 안 흐른다(publisher count 0, echo 타임아웃).
화이트리스트(useBuiltinTransports=false, UDP 를 127.0.0.1/10.10.0.x 로 강제)로 SHM 을
우회하고 wifi/tailscale 인터페이스 혼선을 없애야 브리지↔노드 데이터가 흐른다.
브리지(sim_bridge.sh)와 반드시 동일 프로파일을 걸어야 한다.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

MARKER_MAP = os.path.join(
    get_package_share_directory('parkbot_aruco'), 'data', 'marker_map_v4.json')
GOAL_TIMEOUT = 300.0        # pose_controller 서버측 상한(RTF 저하 여유)
INGRESS_TIMEOUT = 500.0     # 진입은 lead 깊은 traverse(front axle)가 저rtf 에서 느려 더 여유
RAMP_WAIT = 10.0            # lift_action_server 벽시계 대기

# 브리지(sim_bridge.sh)와 반드시 동일해야 하는 DDS env. 각 노드에 직접 주입.
# 화이트리스트 프로파일을 브리지와 동일하게 건다(위 docstring — SHM 우회, 데이터 흐름).
_WHITELIST = os.path.expanduser('~/.ros/fastdds_whitelist.xml')
ENV = {
    'ROS_DOMAIN_ID': os.environ.get('ROS_DOMAIN_ID', '126'),
    'ROS_LOCALHOST_ONLY': '0',
    'RMW_IMPLEMENTATION': 'rmw_fastrtps_cpp',
    # ROS Humble cv_bridge는 시스템 NumPy 1.x로 빌드됐다. 사용자 site-packages의
    # NumPy 2.x가 먼저 잡히면 _ARRAY_API import 오류가 나므로 로봇 노드에서만
    # 사용자 site-packages를 제외한다(관제 UI의 MySQL/NetworkX 환경은 유지).
    'PYTHONNOUSERSITE': '1',
    'FASTRTPS_DEFAULT_PROFILES_FILE': _WHITELIST,
    'FASTDDS_DEFAULT_PROFILES_FILE': _WHITELIST,
}


def _node(**kw):
    kw.setdefault('output', 'screen')
    return Node(additional_env=ENV, **kw)


def _localizer(rid, seed):
    # marker_localizer: 로봇 네임스페이스 + 전후방 이중카메라 + 도크 스폰 자세 seed.
    # (orchestrator 가 ref_ids 를 런타임 전환) FQN=/robot_<rid>/marker_localizer_node.
    return _node(
        package='parkbot_aruco', executable='marker_localizer_node',
        namespace=f'/robot_{rid}',
        parameters=[{
            'robot_id': rid,
            'image_topic': f'/robot_{rid}/front/image_raw',
            'camera_info_topic': f'/robot_{rid}/front/camera_info',
            'odom_topic': f'/robot_{rid}/odom',
            'pose_topic': f'/robot_{rid}/pose',
            'diagnostics_topic': f'/robot_{rid}/vision/alignment',
            'rear_image_topic': f'/robot_{rid}/rear/image_raw',
            'rear_camera_info_topic': f'/robot_{rid}/rear/camera_info',
            'seed_pose': seed,
            'marker_map': MARKER_MAP, 'fuse': True, 'frame': 'usd'}])


def generate_launch_description():
    # auto_start: 기본 False — 2026-07-28 관제 UI(EXIT_DB 브랜치 user_request_gateway_node
    # + parking_control_mvp dual 모드) 연동을 위해 orchestrator 를 "요청 대기" 모드가
    # 기본이 되게 뒤집었다(이전엔 launch 만으로 25s 뒤 자율 미션이 항상 돌았다 — 그건
    # 개발 중 반복검증용이었지, UI 트리거 배관과는 상충한다). 지금까지처럼 launch 만으로
    # 자율 검증하려면 `auto_start:=true` 로 오버라이드.
    auto_start_arg = DeclareLaunchArgument(
        'auto_start', default_value='false',
        description='true 면 launch 후 25s 뒤 자율 미션(검증용). false(기본)면 '
                     'user_request_gateway_node 의 액션 호출을 기다린다.')
    auto_start_param = ParameterValue(LaunchConfiguration('auto_start'), value_type=bool)

    # ---- 입차(entry_lead/entry_follow) — 2026-07-27 출차 설계로 전환하며 주석처리.
    # 되돌릴 때는 아래 블록 주석을 풀고 EXIT_IDS 루프들을 다시 주석처리하면 된다.
    # nodes = [
    #     _localizer('entry_lead', [-3.2, 2.2, 90.0]),
    #     _localizer('entry_follow', [-1.2, 2.2, 90.0]),
    # ]
    #
    # for rid in ('entry_lead', 'entry_follow'):
    #     # pose_controller ×2/로봇: odom용(approach 회전) + 융합용(정밀).
    #     nodes.append(_node(
    #         package='parkbot_motion', executable='pose_controller_node',
    #         name=f'pose_controller_odom_{rid}',
    #         parameters=[{'robot_id': rid, 'goal_timeout_sec': GOAL_TIMEOUT}]))
    #     nodes.append(_node(
    #         package='parkbot_motion', executable='pose_controller_node',
    #         name=f'pose_controller_fused_{rid}',
    #         parameters=[{
    #             'robot_id': rid, 'pose_topic': f'/robot_{rid}/pose',
    #             'pose_msg_type': 'posestamped',
    #             'action_name': f'/robot_{rid}/navigate_to_pose_fused',
    #             'pos_tol': 0.06, 'yaw_tol': 1.0, 'pose_stale_timeout_sec': 15.0,
    #             'goal_timeout_sec': GOAL_TIMEOUT}]))
    #
    # for rid in ('entry_lead', 'entry_follow'):
    #     nodes.append(_node(
    #         package='parkbot_motion', executable='axle_detector_node',
    #         name=f'axle_detector_{rid}',
    #         parameters=[{'robot_id': rid, 'pose_topic': f'/robot_{rid}/pose',
    #                      'pose_msg_type': 'posestamped'}]))
    # for rid in ('entry_lead', 'entry_follow'):
    #     drive_sign = -1.0 if rid == 'entry_follow' else 1.0
    #     nodes.append(_node(
    #         package='parkbot_motion', executable='ingress_node',
    #         name=f'ingress_{rid}',
    #         parameters=[{'robot_id': rid, 'goal_timeout_sec': INGRESS_TIMEOUT,
    #                      'pose_topic': f'/robot_{rid}/pose',
    #                      'pose_msg_type': 'posestamped',
    #                      'drive_sign': drive_sign}]))
    #
    # for rid in ('entry_lead', 'entry_follow'):
    #     nodes.append(_node(
    #         package='parkbot_motion', executable='lift_action_server',
    #         name=f'lift_{rid}',
    #         parameters=[{'robot_id': rid, 'ramp_wait_sec': RAMP_WAIT}]))
    #
    # nodes.append(_node(
    #     package='parkbot_motion', executable='pickup_orchestrator_node',
    #     parameters=[{
    #         'auto_start': True, 'auto_leader': 'entry_lead',
    #         'auto_follower': 'entry_follow',
    #         'phase_b_leader_localizer_node': '/robot_entry_lead/marker_localizer_node',
    #         'phase_b_follower_localizer_node': '/robot_entry_follow/marker_localizer_node'}]))

    # ---- 출차(exit_lead/exit_follow) — 2026-07-27 신규 설계, 입차 블록을 그대로
    # 미러링(로봇 id/도크/회랑만 남측으로 교체). orchestrator 쪽 Phase X 안무는
    # pickup_orchestrator_node.py/_run_phase_x 참고 — 아직 Isaac 실측 미검증.
    EXIT_IDS = ('exit_lead', 'exit_follow')
    nodes = [
        # D_IN_1(x=-3.2,z=-2.2)/D_IN_2(x=-1.2,z=-2.2) 도크 스폰 — 입차 도크와 같은
        # yaw=90°(동향) 스폰(V4_STAGE_READY GT 실측 확인), x 만 다르고 z 부호 반전.
        _localizer('exit_lead', [-3.2, -2.2, 90.0]),
        _localizer('exit_follow', [-1.2, -2.2, 90.0]),
    ]

    for rid in EXIT_IDS:
        # pose_controller ×2/로봇: odom용(제자리회전) + 융합용(마커 정밀주행).
        nodes.append(_node(
            package='parkbot_motion', executable='pose_controller_node',
            name=f'pose_controller_odom_{rid}',
            parameters=[{'robot_id': rid, 'goal_timeout_sec': GOAL_TIMEOUT}]))
        nodes.append(_node(
            package='parkbot_motion', executable='pose_controller_node',
            name=f'pose_controller_fused_{rid}',
            parameters=[{
                'robot_id': rid, 'pose_topic': f'/robot_{rid}/pose',
                'pose_msg_type': 'posestamped',
                'action_name': f'/robot_{rid}/navigate_to_pose_fused',
                # yaw_tol/pos_tol 값은 입차 인스턴스와 동일 근거(메카넘 데드밴드) —
                # 출차에서 아직 재검증되지 않았다.
                'pos_tol': 0.06, 'yaw_tol': 1.0, 'pose_stale_timeout_sec': 15.0,
                'goal_timeout_sec': GOAL_TIMEOUT}]))

    # axle_detector ×2, ingress ×2: A3 트럭(길이축 z) 아래 진입용. 진입 방향이
    # 입차(world -x)와 달리 world +z(남→북)라 drive_sign 은 둘 다 +1(로컬 forward
    # 가 곧 world +z, Phase X 가 로봇을 북향으로 정렬해준 뒤 호출). 두 노드 모두
    # 원래 "주행좌표=world x" 하드코딩이었다(entry 전용 배치, 각 노드 docstring
    # "주행좌표" 절) — 2026-07-27 최초 라이브 실측(GT 오도로 접근은 정밀했는데도
    # 1번 로봇이 트럭 하부 엉뚱한 지점에서 정지)으로 발견: travel_axis='x' 그대로면
    # 실제 전진축(z)이 아니라 거의 안 변하는 횡축(x)을 "주행좌표"로 오인해 트로프
    # 판정 자체가 무의미해진다. travel_axis='-z'로 전진축을 z로 바꾸고(부호는
    # ingress_control 의 "SEEK가 주행좌표를 감소시킨다" 내장 가정에 맞춘 것 —
    # 북향 전진은 z가 증가하므로 -z가 감소한다), drive_sign=1.0(변경 없음, 이미
    # 그 가정과 일치)은 그대로 둔다. pose_controller_node.travel_axis_value 참고.
    for rid in EXIT_IDS:
        nodes.append(_node(
            package='parkbot_motion', executable='axle_detector_node',
            name=f'axle_detector_{rid}',
            parameters=[{'robot_id': rid, 'pose_topic': f'/robot_{rid}/pose',
                         'pose_msg_type': 'posestamped', 'travel_axis': '-z'}]))
    for rid in EXIT_IDS:
        nodes.append(_node(
            package='parkbot_motion', executable='ingress_node',
            name=f'ingress_{rid}',
            parameters=[{'robot_id': rid, 'goal_timeout_sec': INGRESS_TIMEOUT,
                         'pose_topic': f'/robot_{rid}/pose',
                         'pose_msg_type': 'posestamped',
                         'travel_axis': '-z',
                         'drive_sign': 1.0}]))

    # lift ×2
    for rid in EXIT_IDS:
        nodes.append(_node(
            package='parkbot_motion', executable='lift_action_server',
            name=f'lift_{rid}',
            parameters=[{'robot_id': rid, 'ramp_wait_sec': RAMP_WAIT}]))

    # carry ×1 (Phase D 운반: 가상중심 강체 제어, 로봇쌍 공용이라 입/출차 겸용)
    nodes.append(_node(
        package='parkbot_motion', executable='exit_carry_action_server',
        name='exit_carry_action_server',
        parameters=[{'action_name': '/exit/carry_to_slot'}]))

    # orchestrator: auto_start(기본 false, § 위 DeclareLaunchArgument)면 launch 만으로도
    # 자율 미션(도크→남측 회랑 Phase X + 출차 안무), 아니면 아래 게이트웨이의 액션
    # 호출을 대기.
    nodes.append(_node(
        package='parkbot_motion', executable='exit_pickup_orchestrator_node',
        name='exit_pickup_orchestrator_node',
        parameters=[{
            'action_name': '/exit/execute_parking_task',
            'carry_action': '/exit/carry_to_slot',
            'auto_start': auto_start_param, 'auto_leader': 'exit_lead',
            'auto_follower': 'exit_follow',
            'phase_x_leader_localizer_node': '/robot_exit_lead/marker_localizer_node',
            'phase_x_follower_localizer_node': '/robot_exit_follow/marker_localizer_node'}]))

    # user_request_gateway: 관제 UI(parking_control_mvp, dual 모드)의 출차 요청을
    # 위 orchestrator 액션으로 잇는다. DB(parking_control_mvp/core/db.py) robot_groups
    # 의 exit 그룹 기본값과 이름을 맞췄다(dispatch_service='dispatch_parking_task_exit')
    # — 관제 쪽 코드/DB 변경 없이 이 launch 만으로 그대로 연결된다. 입차팀
    # (entry_lead/entry_follow) 게이트웨이는 이 컴퓨터 범위 밖(별도 PC, § user_request_
    # gateway_node.py 클래스 docstring "한 인스턴스는 한 팀만 담당").
    nodes.append(_node(
        package='parkbot_motion', executable='user_request_gateway_node',
        name='exit_user_request_gateway',
        parameters=[{
            'action_name': '/exit/execute_parking_task',
            'leader_robot_id': 'exit_lead', 'follower_robot_id': 'exit_follow',
            'dispatch_service': 'dispatch_parking_task_exit',
            'park_in_slot_service': '/park_in_slot_exit'}]))
    return LaunchDescription([auto_start_arg, *nodes])
