"""parking_robot_system 노드 9개를 한 번에 실행하는 통합 launch 파일."""

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
