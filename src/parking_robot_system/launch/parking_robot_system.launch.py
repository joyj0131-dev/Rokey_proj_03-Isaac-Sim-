"""parking_robot_system 실행부를 입차 전용 로봇쌍 / 출차 전용 로봇쌍 두 세트로 띄우는
launch 파일.

우리 아키텍처에서 실제로 쓰는 노드만 담는다 — user_request_gateway/task_dispatcher/
parking_slot_manager/safety_monitor는 이 패키지의 TODO 스텁일 뿐, 관제탑 역할은 우리
parking_control 패키지(MySQL 기반 task_dispatcher_node/parking_slot_manager_node)가
대신하므로 여기서 띄우지 않는다(같이 띄우면 이름이 겹치는 미완성 스텁 노드가 그래프에
섞여 들어갈 뿐이다).

두 세트로 나누는 이유(하드웨어 배치 확정): 로봇 4대가 입차 전용(entry_lead/
entry_follow) / 출차 전용(exit_lead/exit_follow)으로 고정 배치됐다(로봇 ID는
src/parkbot_aruco/parkbot_aruco/site_map_v4.ROBOTS/ROBOT_DOCK_MARKER가 유일한
출처 — 2026-07-25 이전에는 robot_rear/robot_front/robot_rear2/robot_front2라는
잘못된 이름을 썼다). navigate_action_server/align_action_server/lift_action_server는
FormationMotion을 통해 물리 로봇 이름 + 인계지점/게이트/도크/전용 차로 좌표를 직접
다루므로 세트별로 다른 파라미터 값을 줘 각자 다른 ROS 네임스페이스(entry/, exit/)에
띄운다. robot_task_orchestrator/vehicle_detection_node는 좌표 대부분과 무관하지만,
액션 서버 이름(execute_parking_task, detect_vehicle)이 네임스페이스로 분리돼야 하므로
마찬가지로 세트마다 하나씩 띄운다. parking_control의 task_dispatcher_node가
request_type(ENTRY/EXIT)에 따라 entry/execute_parking_task 또는
exit/execute_parking_task로 goal을 보낸다.

좌표는 전부 parking_environment_v4.usd ArucoMarkerPreview 스코프 실측 기반이다
(generate_map.py의 SLOTS_USD/ENTRY_*_USD/EXIT_*_USD/DOCK_*_USD 상수와 같은 원천) —
단 ENTRY/EXIT 배정은 에셋 자체의 aruco:note 라벨이 아니라
src/parkbot_aruco/parkbot_aruco/site_map_v4.py의 팀 규약(z 양수=입차, z 음수=출차)을
따른다(에셋 라벨은 정반대다 — 2026-07-25 수정, 이전 버전은 이 반전을 놓쳐 입/출차
좌표가 통째로 뒤바뀌어 있었다). carry 구간(인계지점↔슬롯)에서 입차는 lane_z=6.875
(XN/A1' 계열, 슬롯과 반대쪽이라 carry 2단계에서 그 경계를 가로질러 슬롯까지 들어간다),
출차는 lane_z=-6.875(슬롯 자신과 같은 차로)를 써서 두 세트가 물리적으로 다른 차로를
지나가게 한다 — 동시에 움직여도 서로 안 겹친다.

이 파일은 ROS_DOMAIN_ID/RMW_IMPLEMENTATION 등 환경변수를 스스로 설정하지 않는다 —
`ros2 launch`를 실행하는 터미널이 미리 아래 환경을 소싱해 둬야 Isaac runner와 같은
ROS 그래프에서 서로를 발견한다. 실측 ROS_DOMAIN_ID는 122다(코드 주석에 126이라
적힌 곳이 있으면 오기다 — 2026-07-25 실측 확인):

    source /opt/ros/humble/setup.bash
    source install/setup.bash
    export ROS_DOMAIN_ID=122
    export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
    unset FASTRTPS_DEFAULT_PROFILES_FILE FASTDDS_DEFAULT_PROFILES_FILE
"""

from launch import LaunchDescription
from launch.actions import GroupAction
from launch_ros.actions import Node, PushRosNamespace

PACKAGE = 'parking_robot_system'

# 세트별 물리 상수 — v4.usd ArucoMarkerPreview 실측(USD 프레임 x,z, FormationMotion과
# 동일 좌표계). bay_x_map/bay_y_map만 map 프레임(robot_task_orchestrator._bay_pose용,
# frame_transform 규약 x_map=x_usd/y_map=-z_usd로 변환된 값) — 출차 세트에서만 실제로
# 쓰인다. ENTRY/EXIT 배정은 site_map_v4.py 규약(z 양수=입차) 기준 — 마커별 대응:
#   entry_lead/entry_follow ↔ D_OUT_1/D_OUT_2, exit_lead/exit_follow ↔ D_IN_1/D_IN_2.
SITES = {
    'entry': dict(
        rear_id='entry_lead', front_id='entry_follow',
        handoff_x=-8.5, handoff_z=7.075, gate_x=-12.55,       # W_OUT / GATE_OUT
        dock_rear_x=-3.2, dock_rear_z=2.2, dock_front_x=-1.2, dock_front_z=2.2,  # D_OUT_1/2
        lane_z=6.875, site_role='entry',                       # XN/A1' 계열
        pickup_x_usd=-8.5, pickup_z_usd=7.075,
        bay_x_map=-8.5, bay_y_map=-7.075,   # 이 세트에서는 안 쓰이지만 무해하게 채워둠
    ),
    'exit': dict(
        rear_id='exit_lead', front_id='exit_follow',
        handoff_x=-8.5, handoff_z=-7.075, gate_x=-12.55,      # W_IN / GATE_IN
        dock_rear_x=-3.2, dock_rear_z=-2.2, dock_front_x=-1.2, dock_front_z=-2.2,  # D_IN_1/2
        lane_z=-6.875, site_role='exit',                       # XS/슬롯(A1/A2/A3) 계열
        pickup_x_usd=-8.5, pickup_z_usd=-7.075,   # 이 세트에서는 안 쓰임(detect_vehicle 미사용)
        bay_x_map=-8.5, bay_y_map=7.075,
    ),
}

# rear_id/front_id + 인계지점/게이트/도크/차로 좌표를 받는 노드(FormationMotion 생성).
FORMATION_EXECUTABLES = ['navigate_action_server', 'align_action_server']
# 위와 같은 파라미터를 받되 FormationMotion을 직접 안 쓰는 노드(로봇 이름/좌표는 필요).
LIFT_EXECUTABLES = ['lift_action_server']   # rear_id/front_id만 필요
# 네임스페이스 분리만 필요한 노드 — bay_x_map/y_map(orchestrator), pickup_x/z_usd(vehicle_detection).
ORCHESTRATOR_EXECUTABLES = ['robot_task_orchestrator']
DETECTION_EXECUTABLES = ['vehicle_detection_node']


def _set_group(set_name, site):
    formation_params = {
        'rear_id': site['rear_id'], 'front_id': site['front_id'],
        'handoff_x': site['handoff_x'], 'handoff_z': site['handoff_z'],
        'gate_x': site['gate_x'],
        'dock_rear_x': site['dock_rear_x'], 'dock_rear_z': site['dock_rear_z'],
        'dock_front_x': site['dock_front_x'], 'dock_front_z': site['dock_front_z'],
        'lane_z': site['lane_z'],
    }
    align_params = dict(formation_params, site_role=site['site_role'])
    lift_params = {'rear_id': site['rear_id'], 'front_id': site['front_id']}
    orchestrator_params = {
        'bay_x_map': site['bay_x_map'],
        'bay_y_map': site['bay_y_map'],
        'team_role': set_name,
    }
    detection_params = {'pickup_x_usd': site['pickup_x_usd'], 'pickup_z_usd': site['pickup_z_usd']}

    nodes = [
        Node(package=PACKAGE, executable='navigate_action_server',
             name='navigate_action_server', output='screen',
             parameters=[formation_params]),
        Node(package=PACKAGE, executable='align_action_server',
             name='align_action_server', output='screen',
             parameters=[align_params]),
        Node(package=PACKAGE, executable='lift_action_server',
             name='lift_action_server', output='screen',
             parameters=[lift_params]),
        Node(package=PACKAGE, executable='robot_task_orchestrator',
             name='robot_task_orchestrator', output='screen',
             parameters=[orchestrator_params]),
        Node(package=PACKAGE, executable='vehicle_detection_node',
             name='vehicle_detection_node', output='screen',
             parameters=[detection_params]),
    ]
    return GroupAction([PushRosNamespace(set_name), *nodes])


def generate_launch_description():
    return LaunchDescription([
        _set_group(set_name, site) for set_name, site in SITES.items()
    ])
