"""parking_robot_system 노드 9개를 한 번에 실행하는 통합 launch 파일.

이 파일은 ROS_DOMAIN_ID/RMW_IMPLEMENTATION 등 환경변수를 스스로 설정하지 않는다 —
`ros2 launch`를 실행하는 터미널이 미리 아래 환경을 소싱해 둬야 Isaac runner(내부 rclpy,
도메인 126, FastRTPS 프로파일 unset)와 같은 ROS 그래프에서 서로를 발견한다:

    source /opt/ros/humble/setup.bash
    source install/setup.bash
    export ROS_DOMAIN_ID=126
    export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
    unset FASTRTPS_DEFAULT_PROFILES_FILE FASTDDS_DEFAULT_PROFILES_FILE

전체 실행 순서(Isaac runner --gui 포함 4터미널)와 검증 절차는
`docs/runbook-park-in-slot.md` 참고.
"""

from launch import LaunchDescription
from launch_ros.actions import Node

PACKAGE = 'parking_robot_system'

NODE_EXECUTABLES = [
    'user_request_gateway',
    'task_dispatcher',
    'parking_slot_manager',
    'robot_task_orchestrator',
    'safety_monitor',
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
