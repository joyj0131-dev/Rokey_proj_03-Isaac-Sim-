#!/bin/bash
set -eu

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
ISAAC_RELEASE="${ISAAC_SIM_ROOT:-/home/rokey/dev_ws/isaac_sim/isaacsim/_build/linux-x86_64/release}"
BRIDGE_LIB="$ISAAC_RELEASE/exts/isaacsim.ros2.bridge/humble/lib"

# Isaac Sim 5.1은 내부 ROS 2 Humble을 사용한다. 시스템 /opt/ros 경로가
# 상속되면 3.10 rclpy가 섞여 브리지가 죽는다.
# FastDDS 화이트리스트는 반드시 해제한다 — rokey 머신 실측(2026-07-21):
# 화이트리스트(useBuiltinTransports=false)를 켜면 내부 rclpy(3.11 fastdds)와
# 시스템 Humble 사이 discovery가 양방향 모두 실패한다. 외부 터미널도 이
# 러너와 통신할 때는 FASTRTPS_DEFAULT_PROFILES_FILE 을 unset 할 것.
unset PYTHONPATH AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH
unset FASTRTPS_DEFAULT_PROFILES_FILE FASTDDS_DEFAULT_PROFILES_FILE
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-126}"
export ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export LD_LIBRARY_PATH="$BRIDGE_LIB"

exec "$ISAAC_RELEASE/python.sh" "$SCRIPT_DIR/run_dual_robot_ros2_field.py" "$@"
