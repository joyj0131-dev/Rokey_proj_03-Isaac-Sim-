#!/bin/bash
# 인계장 도킹·리프트·오미 오케스트레이터 (외부 ROS2 Humble).
# 러너(dock_lift_handoff_runner.sh)가 먼저 떠 있어야 한다. 그 다음 이 노드를 띄우고
# 다른 터미널에서:  ros2 service call /dock_lift std_srvs/srv/Trigger
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source /opt/ros/humble/setup.bash  # ROS setup 은 nounset 비호환 → set -u 미사용
# 내부 rclpy(러너)와 발견되려면 화이트리스트 프로파일 반드시 unset(실측).
unset FASTRTPS_DEFAULT_PROFILES_FILE FASTDDS_DEFAULT_PROFILES_FILE
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-126}"
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
exec python3 "$SCRIPT_DIR/dock_lift_handoff_mission.py" "$@"
