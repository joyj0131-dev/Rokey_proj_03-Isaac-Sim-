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
from launch_ros.actions import Node

MARKER_MAP = os.path.join(
    get_package_share_directory('parkbot_aruco'), 'data', 'marker_map_v4.json')
GOAL_TIMEOUT = 300.0        # pose_controller 서버측 상한(RTF 저하 여유)
INGRESS_TIMEOUT = 500.0     # 진입은 lead 깊은 traverse(front axle)가 저rtf 에서 느려 더 여유
RAMP_WAIT = 10.0            # lift_action_server 벽시계 대기

# 브리지(sim_bridge.sh)와 반드시 동일해야 하는 DDS env. 각 노드에 직접 주입.
# 화이트리스트 프로파일을 브리지와 동일하게 건다(위 docstring — SHM 우회, 데이터 흐름).
# 입차(entry) 액션 이름 — 통합 관제(featureUI)의 중앙 task_dispatcher 컨벤션에 맞춘다.
# dispatcher 가 request_type=ENTRY 를 '/entry/execute_parking_task' 로 라우팅하므로 그 이름을
# 그대로 서빙한다(출차는 '/exit/execute_parking_task'). carry 는 입/출차 분리를 위해
# '/entry/carry_to_slot'(출차 '/exit/carry_to_slot'). dispatch 접수는 dispatcher 가 유일
# 창구라 이 머신에선 bypass gateway 를 안 띄운다(dispatch_parking_task 2대 서버 충돌 방지).
ENTRY_PICKUP_ACTION = '/entry/execute_parking_task'
ENTRY_CARRY_ACTION = '/entry/carry_to_slot'

_WHITELIST = os.path.expanduser('~/.ros/fastdds_whitelist.xml')
ENV = {
    # 셸의 ROS_DOMAIN_ID 를 따른다(기본 126). 솔로 테스트 시 ROS_DOMAIN_ID=130 등으로 띄우면
    # 도메인 126 을 공유하는 다른 PC(관제/출차)와 완전히 격리된다(중복 orchestrator 차단).
    # sim_bridge.sh 도 ${ROS_DOMAIN_ID:-126} 라 같은 값을 export 하면 Isaac 과 도메인이 맞는다.
    'ROS_DOMAIN_ID': os.environ.get('ROS_DOMAIN_ID', '126'),
    'ROS_LOCALHOST_ONLY': '0',
    'RMW_IMPLEMENTATION': 'rmw_fastrtps_cpp',
    'FASTRTPS_DEFAULT_PROFILES_FILE': _WHITELIST,
    'FASTDDS_DEFAULT_PROFILES_FILE': _WHITELIST,
}


def _node(**kw):
    kw.setdefault('output', 'screen')
    return Node(additional_env=ENV, **kw)


def _localizer(rid, seed):
    # marker_localizer: 로봇 네임스페이스 + 전후방 이중카메라 + 도크 스폰 자세 seed.
    # (orchestrator 가 ref_ids 를 런타임 전환) FQN=/robot_<rid>/marker_localizer_node.
    params = {
        'image_topic': f'/robot_{rid}/front/image_raw',
        'camera_info_topic': f'/robot_{rid}/front/camera_info',
        'odom_topic': f'/robot_{rid}/odom',
        'pose_topic': f'/robot_{rid}/pose',
        'rear_image_topic': f'/robot_{rid}/rear/image_raw',
        'rear_camera_info_topic': f'/robot_{rid}/rear/camera_info',
        'seed_pose': seed,
        'marker_map': MARKER_MAP, 'fuse': True, 'frame': 'usd'}
    return _node(
        package='parkbot_aruco', executable='marker_localizer_node',
        namespace=f'/robot_{rid}', parameters=[params])


def generate_launch_description():
    # 복귀 테스트(RETURN_TEST=1): sim_bridge 가 주차완료 상태로 스폰하므로 localizer
    # 필터 seed 도 그 주차 자세로 맞춘다(도크 seed 면 첫 측위가 90° 틀어짐).
    _return = os.environ.get('RETURN_TEST', '0') not in ('0', '', 'false', 'False')
    lead_seed = [2.8, -1.8, 180.0] if _return else [-3.2, 2.2, 90.0]
    follow_seed = [2.8, 1.8, 0.0] if _return else [-1.2, 2.2, 90.0]
    nodes = [
        _localizer('entry_lead', lead_seed),
        _localizer('entry_follow', follow_seed),
    ]

    for rid in ('entry_lead', 'entry_follow'):
        # pose_controller ×2/로봇: odom용(approach 회전) + 융합용(정밀).
        nodes.append(_node(
            package='parkbot_motion', executable='pose_controller_node',
            name=f'pose_controller_odom_{rid}',
            # 슬립 감소 다운스케일(2026-07-27): 회전 0.6→0.35, 직진 0.25→0.15.
            parameters=[{'robot_id': rid, 'goal_timeout_sec': GOAL_TIMEOUT,
                         'max_lin': 0.15, 'max_ang': 0.35}]))
        nodes.append(_node(
            package='parkbot_motion', executable='pose_controller_node',
            name=f'pose_controller_fused_{rid}',
            parameters=[{
                'robot_id': rid, 'pose_topic': f'/robot_{rid}/pose',
                'pose_msg_type': 'posestamped',
                'action_name': f'/robot_{rid}/navigate_to_pose_fused',
                # yaw_tol 0.5: 진입 직전 정렬이 0.5° 넘게 틀어지면 직진 시 트럭 바퀴에
                # 부딪혀 차밑 진입 실패(사용자 실측). 예전엔 메카넘 데드밴드(wz≈0.012
                # rad/s 아래 안 돎)로 0.5°를 못 맞춰 스톨→1.0 으로 완화했었으나, 이제
                # yaw_min_cmd(데드밴드 보정)로 최소 회전속도를 깔아 0.5°까지 인칭 도달.
                'yaw_min_cmd': 0.05,
                # 슬립 감소 다운스케일(2026-07-27): 회전 0.6→0.35, 직진 0.25→0.15.
                'max_lin': 0.15, 'max_ang': 0.35,
                'pos_tol': 0.06, 'yaw_tol': 0.5, 'pose_stale_timeout_sec': 15.0,
                'goal_timeout_sec': GOAL_TIMEOUT}]))

    # axle_detector ×2, ingress ×2: 픽업 자세원을 융합 /pose 로(드리프트 상쇄).
    for rid in ('entry_lead', 'entry_follow'):
        nodes.append(_node(
            package='parkbot_motion', executable='axle_detector_node',
            name=f'axle_detector_{rid}',
            parameters=[{'robot_id': rid, 'pose_topic': f'/robot_{rid}/pose',
                         'pose_msg_type': 'posestamped'}]))
    for rid in ('entry_lead', 'entry_follow'):
        # 2026-07-27 재안무: lead 서향(-90°)→전진진입(drive_sign +1),
        # follow 동향(+90°)→후진진입(drive_sign -1). 둘 다 world -x 로 트럭 밑에.
        drive_sign = -1.0 if rid == 'entry_follow' else 1.0
        # 2026-07-28: 진입 중 마커융합 /pose yaw 로 이 헤딩 유지(Phase B 최종 yaw 와
        # 동일: lead -90/follow +90) → 먼 마커서 정렬 후 트럭까지 요 드리프트로 바퀴에
        # 부딪히던 문제 해결. 정지는 depth 축검출 그대로.
        hold_yaw = 90.0 if rid == 'entry_follow' else -90.0
        nodes.append(_node(
            package='parkbot_motion', executable='ingress_node',
            name=f'ingress_{rid}',
            parameters=[{'robot_id': rid, 'goal_timeout_sec': INGRESS_TIMEOUT,
                         'pose_topic': f'/robot_{rid}/pose',
                         'pose_msg_type': 'posestamped',
                         'drive_sign': drive_sign, 'hold_yaw_deg': hold_yaw}]))
                         # 진입 속도 원복(2026-07-27): 다운스케일(forward 0.2/lat 0.1)이
                         # 진입 중 z 드리프트를 키워 트럭 중심선 이탈→축 미검출 유발(실측
                         # lead z=6.51 vs 트럭 7.075). 노드 기본값(forward 0.4/return 0.15/
                         # lat_vy_max 0.15) 사용. 슬립은 진입 밖(Phase B/carry)에서만 관리.

    # lift ×2
    for rid in ('entry_lead', 'entry_follow'):
        nodes.append(_node(
            package='parkbot_motion', executable='lift_action_server',
            name=f'lift_{rid}',
            parameters=[{'robot_id': rid, 'ramp_wait_sec': RAMP_WAIT}]))

    # carry ×1 (Phase D 운반: 가상중심 강체 제어). 액션 이름 입차전용으로 격리.
    nodes.append(_node(
        package='parkbot_motion', executable='carry_action_server',
        parameters=[{'carry_action_name': ENTRY_CARRY_ACTION}]))

    # orchestrator: auto_start=False (2026-07-27) — launch 는 노드만 띄우고 대기.
    # 목표 주차 자리는 다른 터미널에서 action call 로 지정:
    #   ros2 action send_goal /execute_pickup_choreography \
    #     parking_robot_interfaces/action/ExecuteParkingTask "{slot_id: 'A1'}"
    # leader/follower 를 goal 에 안 주면 auto_leader/auto_follower 기본값을 쓴다.
    nodes.append(_node(
        package='parkbot_motion', executable='pickup_orchestrator_node',
        parameters=[{
            'auto_start': False, 'auto_leader': 'entry_lead',
            'auto_follower': 'entry_follow',
            # 입차전용 액션 이름(출차 복붙 스택과 격리) — carry_action 은 carry 서버와 일치.
            'action_name': ENTRY_PICKUP_ACTION,
            'carry_action': ENTRY_CARRY_ACTION,
            # 복귀 테스트(RETURN_TEST=1): 입차~주차 생략, 복귀만 실행.
            'skip_to_return': _return,
            'phase_b_leader_localizer_node': '/robot_entry_lead/marker_localizer_node',
            'phase_b_follower_localizer_node': '/robot_entry_follow/marker_localizer_node'}]))

    # 입차 요청 접수는 통합 관제(UI)의 중앙 task_dispatcher 가 담당한다:
    #   UI → dispatch_parking_task(단일 서버) → '/entry/execute_parking_task'(위 orchestrator).
    # (구 bypass user_request_gateway 노드는 제거됨 — dispatch_parking_task 서버가 2대가 되면
    #  UI 요청이 랜덤 라우팅돼 로봇이 안 움직였던 문제 때문.)
    # 솔로 테스트는 orchestrator 액션에 직접 발행:
    #   ros2 action send_goal /entry/execute_parking_task \
    #     parking_robot_interfaces/action/ExecuteParkingTask "{slot_id: 'A1'}"
    return LaunchDescription(nodes)
