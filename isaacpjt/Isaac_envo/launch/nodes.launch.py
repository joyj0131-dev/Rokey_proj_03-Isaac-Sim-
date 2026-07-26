#!/usr/bin/env python3
"""nodes.launch.py — 순수 ROS2(py3.10) 노드 스택만 기동한다. Isaac 은 별개.

Isaac 시뮬(sim_bridge)과 ROS2 노드는 **다른 컴퓨터**라 생각하고 각자 실행한다 —
둘은 DDS(도메인 126)로만 대화하므로 순서·프로세스 공유가 필요없다.

  [컴퓨터 A · Isaac py3.11]  bash isaacpjt/Isaac_envo/sim_bridge.sh [--gui]
        → 씬/로봇/차량/마커 스폰, /odom·image·depth 발행, /cmd_vel 구독.
          BRIDGE_READY 가 뜰 때까지 기다린다(콜드스타트 ~2분).

  [컴퓨터 B · ROS2 py3.10]   ros2 launch isaacpjt/Isaac_envo/launch/nodes.launch.py
        → 이 파일. marker_localizer×2 / pose_controller×4 / axle×2 / ingress×2 /
          lift×2 / pickup_orchestrator(auto_start) 를 띄운다. 노드들은 토픽을
          구독하며 대기하므로 브리지가 먼저 떠 있으면 바로 붙는다. orchestrator
          는 auto_start 로 스스로 미션을 실행한다(외부 트리거 없음).

정리: 각 터미널 Ctrl-C. 잔여는 pkill -9 -f 'sim_bridge.py' / 'parkbot_(aruco|motion)'.
"""
import os

from launch import LaunchDescription
from launch.actions import ExecuteProcess

# 이 파일: <ws>/isaacpjt/Isaac_envo/launch/pickup_mission.launch.py
_LAUNCH_DIR = os.path.dirname(os.path.abspath(__file__))
ENVO = os.path.dirname(_LAUNCH_DIR)                       # isaacpjt/Isaac_envo
WS = os.path.dirname(os.path.dirname(ENVO))               # <ws>
MARKER_MAP = os.path.join(WS, "src/parkbot_aruco/data/marker_map_v4.json")

GOAL_TIMEOUT = "300.0"      # pose_controller/ingress 서버측 상한(RTF 저하 여유)
RAMP_WAIT = "10.0"          # lift_action_server 벽시계 대기


def _wrap(script, *args):
    """isaac_envo 의 run_*.sh 래퍼(시스템 ROS2 환경 세팅 + --ros-args passthrough)를
    ExecuteProcess 로 감싼다. 검증된 노드 기동 커맨드를 그대로 재사용."""
    return ExecuteProcess(
        cmd=["bash", os.path.join(ENVO, script), *args],
        output="screen",
    )


def _ros2_node_stack():
    """시스템 ROS2 노드들(로봇별 배선). Isaac 브리지와 무관하게 독립 기동."""
    nodes = []

    # ── marker_localizer ×2: 로봇 네임스페이스로 FQN 구분 + 전후방 이중카메라 +
    #    도크 스폰 자세 seed. (오케스트레이터가 ref_ids 를 런타임 전환) ──
    nodes.append(_wrap(
        "run_marker_localizer_node.sh", "--ros-args",
        "-r", "__ns:=/robot_entry_lead",
        "-p", "image_topic:=/robot_entry_lead/front/image_raw",
        "-p", "camera_info_topic:=/robot_entry_lead/front/camera_info",
        "-p", "odom_topic:=/robot_entry_lead/odom",
        "-p", "pose_topic:=/robot_entry_lead/pose",
        "-p", "rear_image_topic:=/robot_entry_lead/rear/image_raw",
        "-p", "rear_camera_info_topic:=/robot_entry_lead/rear/camera_info",
        "-p", "seed_pose:=[-3.2,2.2,90.0]",
        "-p", f"marker_map:={MARKER_MAP}", "-p", "fuse:=true", "-p", "frame:=usd"))
    nodes.append(_wrap(
        "run_marker_localizer_node.sh", "--ros-args",
        "-r", "__ns:=/robot_entry_follow",
        "-p", "image_topic:=/robot_entry_follow/front/image_raw",
        "-p", "camera_info_topic:=/robot_entry_follow/front/camera_info",
        "-p", "odom_topic:=/robot_entry_follow/odom",
        "-p", "pose_topic:=/robot_entry_follow/pose",
        "-p", "rear_image_topic:=/robot_entry_follow/rear/image_raw",
        "-p", "rear_camera_info_topic:=/robot_entry_follow/rear/camera_info",
        "-p", "seed_pose:=[-1.2,2.2,90.0]",
        "-p", f"marker_map:={MARKER_MAP}", "-p", "fuse:=true", "-p", "frame:=usd"))

    # ── pose_controller ×4: 로봇당 odom용(approach 회전)·융합용(정밀) ──
    for rid in ("entry_lead", "entry_follow"):
        nodes.append(_wrap(
            "run_pose_controller_node.sh", "--ros-args",
            "-p", f"robot_id:={rid}", "-p", f"goal_timeout_sec:={GOAL_TIMEOUT}"))
        nodes.append(_wrap(
            "run_pose_controller_node.sh", "--ros-args",
            "-p", f"robot_id:={rid}", "-p", f"pose_topic:=/robot_{rid}/pose",
            "-p", "pose_msg_type:=posestamped",
            "-p", f"action_name:=/robot_{rid}/navigate_to_pose_fused",
            "-p", "pos_tol:=0.06", "-p", "pose_stale_timeout_sec:=15.0",
            "-p", f"goal_timeout_sec:={GOAL_TIMEOUT}"))

    # ── axle_detector ×2, ingress ×2: 픽업 자세원을 융합 /pose 로(드리프트 상쇄) ──
    for rid in ("entry_lead", "entry_follow"):
        nodes.append(_wrap(
            "run_axle_detector_node.sh", "--ros-args", "-p", f"robot_id:={rid}",
            "-p", f"pose_topic:=/robot_{rid}/pose", "-p", "pose_msg_type:=posestamped"))
    for rid in ("entry_lead", "entry_follow"):
        nodes.append(_wrap(
            "run_ingress_node.sh", "--ros-args", "-p", f"robot_id:={rid}",
            "-p", f"goal_timeout_sec:={GOAL_TIMEOUT}",
            "-p", f"pose_topic:=/robot_{rid}/pose", "-p", "pose_msg_type:=posestamped"))

    # ── lift ×2 ──
    for rid in ("entry_lead", "entry_follow"):
        nodes.append(_wrap(
            "run_lift_action_server.sh", "--ros-args", "-p", f"robot_id:={rid}",
            "-p", f"ramp_wait_sec:={RAMP_WAIT}"))

    # ── orchestrator: auto_start 로 자율 미션(도크→XN Phase B + 픽업 회랑) ──
    nodes.append(_wrap(
        "run_pickup_orchestrator_node.sh", "--ros-args",
        "-p", "auto_start:=true", "-p", "auto_leader:=entry_lead",
        "-p", "auto_follower:=entry_follow", "-p", "approach_use_fused:=true",
        "-p", "phase_b_leader_localizer_node:=/robot_entry_lead/marker_localizer_node",
        "-p", "phase_b_follower_localizer_node:=/robot_entry_follow/marker_localizer_node"))
    return nodes


def generate_launch_description():
    return LaunchDescription(_ros2_node_stack())
