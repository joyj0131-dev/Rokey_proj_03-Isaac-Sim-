"""관제탑(우리 쪽) 노드 3개를 한 번에 실행.

  - parking_slot_manager / task_dispatcher: MySQL을 직접 읽고 쓰며 ENTRY/EXIT
    판단(빈 슬롯 찾기 / 이 차가 있는 슬롯 찾기)을 담당.
  - robot_position_bridge: 로봇 실측 odom(/robot_rear/odom, /robot_front/odom)을
    받아 DB(robots.x/y)에 기록 — 이게 있어야 웹 대시보드에 로봇이 실시간으로
    움직이는 게 보인다(전에는 이 다리가 없어서 안 보였음).

sim_orchestrator(가짜 로봇 데모용)/isaac_parking_bridge(예전 ENTRY 전용 다리,
robot_task_orchestrator + parking_robot_system 액션서버 4개로 대체됨)는 의도적으로
포함하지 않는다 — 같이 띄우면 DB에 가짜/중복 갱신이 섞여 들어간다.
"""

from launch import LaunchDescription
from launch_ros.actions import Node

PACKAGE = 'parking_control'

NODE_EXECUTABLES = [
    'parking_slot_manager',
    'task_dispatcher',
    'robot_position_bridge',
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
