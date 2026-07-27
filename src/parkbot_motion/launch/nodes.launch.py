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
    nodes = [
        _localizer('entry_lead', [-3.2, 2.2, 90.0]),
        _localizer('entry_follow', [-1.2, 2.2, 90.0]),
    ]

    for rid in ('entry_lead', 'entry_follow'):
        # pose_controller ×2/로봇: odom용(approach 회전) + 융합용(정밀).
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
                # yaw_tol 0.5->1.0: 메카넘 드라이브 데드밴드(잔여 yaw wz≈0.012rad/s
                # 가 못 돌아감)로 approach 가 0.6°에서 스톨→실패하던 것 방지. 트럭
                # 진입 정밀정렬은 뎁스 축검출이 맡으므로 1° yaw 는 무관(라이브 실측).
                'pos_tol': 0.06, 'yaw_tol': 1.0, 'pose_stale_timeout_sec': 15.0,
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
        nodes.append(_node(
            package='parkbot_motion', executable='ingress_node',
            name=f'ingress_{rid}',
            parameters=[{'robot_id': rid, 'goal_timeout_sec': INGRESS_TIMEOUT,
                         'pose_topic': f'/robot_{rid}/pose',
                         'pose_msg_type': 'posestamped',
                         'drive_sign': drive_sign}]))

    # lift ×2
    for rid in ('entry_lead', 'entry_follow'):
        nodes.append(_node(
            package='parkbot_motion', executable='lift_action_server',
            name=f'lift_{rid}',
            parameters=[{'robot_id': rid, 'ramp_wait_sec': RAMP_WAIT}]))

    # carry ×1 (Phase D 운반: 가상중심 강체 제어)
    nodes.append(_node(
        package='parkbot_motion', executable='carry_action_server'))

    # orchestrator: auto_start 로 자율 미션(도크→XN Phase B + 픽업 회랑).
    nodes.append(_node(
        package='parkbot_motion', executable='pickup_orchestrator_node',
        parameters=[{
            'auto_start': True, 'auto_leader': 'entry_lead',
            'auto_follower': 'entry_follow',
            'phase_b_leader_localizer_node': '/robot_entry_lead/marker_localizer_node',
            'phase_b_follower_localizer_node': '/robot_entry_follow/marker_localizer_node'}]))
    return LaunchDescription(nodes)
