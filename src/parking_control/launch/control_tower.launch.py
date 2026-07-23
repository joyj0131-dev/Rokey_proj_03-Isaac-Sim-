"""관제탑(우리 쪽) 노드 4개를 한 번에 실행.

  - parking_slot_manager / task_dispatcher: MySQL을 직접 읽고 쓰며 ENTRY/EXIT
    판단(빈 슬롯 찾기 / 이 차가 있는 슬롯 찾기)을 담당.
  - robot_position_bridge: 로봇 실측 odom(/robot_rear/odom, /robot_front/odom)을
    받아 DB(robots.x/y)에 기록 — 이게 있어야 웹 대시보드에 로봇이 실시간으로
    움직이는 게 보인다(전에는 이 다리가 없어서 안 보였음).
  - safety_monitor: 천장 LiDAR 2대로 통로 장애물·슬롯 점유를 실시간 판정 +
    RViz2 시각화(월드 포인트클라우드 + 슬롯/장애물 MarkerArray) 발행.
  - pedestrian_cue: task_state를 보고 입차 시작/출차 완료 순간에 Isaac Sim
    쪽 "사람 걷기" 연출 신호(/pedestrian_cue)를 쏜다. Isaac Sim 쪽에서 이
    신호를 받아 실제로 캐릭터를 움직이는 스크립트는 아직 best-effort 단계.

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
    'safety_monitor',
    'pedestrian_cue',
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
