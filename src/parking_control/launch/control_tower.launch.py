"""관제탑(우리 쪽) 노드 2개 — parking_slot_manager + task_dispatcher — 를 한 번에 실행.

이 두 노드는 MySQL을 직접 읽고 쓰며 ENTRY/EXIT 판단(빈 슬롯 찾기 / 이 차가 있는 슬롯 찾기)을
담당한다. sim_orchestrator(가짜 로봇 데모용)/isaac_parking_bridge(예전 ENTRY 전용 다리,
robot_task_orchestrator + parking_robot_system 액션서버 4개로 대체됨)는 의도적으로 포함하지
않는다 — 같이 띄우면 DB에 가짜/중복 갱신이 섞여 들어간다.
"""

from launch import LaunchDescription
from launch_ros.actions import Node

PACKAGE = 'parking_control'

NODE_EXECUTABLES = [
    'parking_slot_manager',
    'task_dispatcher',
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
