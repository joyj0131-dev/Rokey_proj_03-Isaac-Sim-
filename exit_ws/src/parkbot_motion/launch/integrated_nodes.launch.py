#!/usr/bin/env python3
"""입차·출차 로봇 제어 스택을 같은 ROS_DOMAIN_ID에서 함께 실행한다.

입차와 출차의 로봇 ID, Action, carry Action, gateway 서비스 이름이 모두 분리되어
있어 한 DDS 그래프에서 동시에 대기할 수 있다. 실제 작업 접수와 DB 갱신은
parking_control의 중앙 task_dispatcher가 담당한다.
"""

from pathlib import Path

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    launch_dir = Path(__file__).resolve().parent
    entry = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(launch_dir / "nodes.launch.py"))
    )
    exit_team = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(launch_dir / "exit_nodes.launch.py")),
        launch_arguments={"auto_start": "false"}.items(),
    )
    return LaunchDescription([entry, exit_team])
