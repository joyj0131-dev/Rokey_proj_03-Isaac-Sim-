#!/bin/bash
set -eu

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
ISAAC_RELEASE="${ISAAC_SIM_ROOT:-/home/rokey/dev_ws/isaac_sim/isaacsim/_build/linux-x86_64/release}"
BRIDGE_LIB="$ISAAC_RELEASE/exts/isaacsim.ros2.bridge/humble/lib"

# Isaac Sim 5.1은 내부 ROS 2 Humble을 사용한다. 시스템 /opt/ros 경로와
# 이전 FastDDS 화이트리스트가 상속되면 외부 관제 노드가 발견되지 않는다.
unset PYTHONPATH AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH
unset FASTRTPS_DEFAULT_PROFILES_FILE FASTDDS_DEFAULT_PROFILES_FILE
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-122}"
export ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export LD_LIBRARY_PATH="$BRIDGE_LIB"

exec "$ISAAC_RELEASE/python.sh" "$SCRIPT_DIR/run_dual_robot_ros2_field.py" "$@"
