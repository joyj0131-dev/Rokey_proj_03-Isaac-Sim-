"""parking_robot_system 실행부(오케스트레이터 + 액션서버 4개)를 한 번에 띄우는 launch 파일.

우리 아키텍처에서 실제로 쓰는 노드만 담는다 — user_request_gateway/task_dispatcher/
parking_slot_manager/safety_monitor는 이 패키지의 TODO 스텁일 뿐, 관제탑 역할은 우리
parking_control 패키지(MySQL 기반 task_dispatcher_node/parking_slot_manager_node)가
대신하므로 여기서 띄우지 않는다(같이 띄우면 이름이 겹치는 미완성 스텁 노드가 그래프에
섞여 들어갈 뿐이다).

이 파일은 ROS_DOMAIN_ID/RMW_IMPLEMENTATION 등 환경변수를 스스로 설정하지 않는다 —
`ros2 launch`를 실행하는 터미널이 미리 아래 환경을 소싱해 둬야 Isaac runner(내부 rclpy,
도메인 126, FastRTPS 프로파일 unset)와 같은 ROS 그래프에서 서로를 발견한다:

    source /opt/ros/humble/setup.bash
    source install/setup.bash
    export ROS_DOMAIN_ID=126
    export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
    unset FASTRTPS_DEFAULT_PROFILES_FILE FASTDDS_DEFAULT_PROFILES_FILE
"""

from launch import LaunchDescription
from launch_ros.actions import Node

PACKAGE = 'parking_robot_system'

NODE_EXECUTABLES = [
    'robot_task_orchestrator',
    'vehicle_detection_node',
    'navigate_action_server',
    'align_action_server',
    'lift_action_server',
]


def generate_launch_description():
    return LaunchDescription([
        Node(
            package=PACKAGE,
            executable=executable,
            name=executable,
            output='screen',
        )
        for executable in NODE_EXECUTABLES
    ])
