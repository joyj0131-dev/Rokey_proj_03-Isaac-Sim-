#!/bin/bash
# parking_robot_system 관제 UI 실행 래퍼.
# 러너(dock_lift_handoff_runner)와 parking_robot_system 런치가 떠 있어야 실동작.
WS="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../../.." && pwd)"
source /opt/ros/humble/setup.bash                 # ROS setup 은 nounset 비호환 → set -u 미사용
source "$WS/install/setup.bash"
# 내부 rclpy(러너)와 서로 발견되려면 화이트리스트 프로파일 unset(실측).
unset FASTRTPS_DEFAULT_PROFILES_FILE FASTDDS_DEFAULT_PROFILES_FILE
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-126}"
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
exec python3 "$WS/src/parking_robot_system/scripts/ui_dashboard.py" "$@"
