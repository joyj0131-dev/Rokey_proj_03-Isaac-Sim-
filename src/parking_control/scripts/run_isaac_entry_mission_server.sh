#!/bin/bash
set -e

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(cd -- "$SCRIPT_DIR/../../.." && pwd)"

source /opt/ros/humble/setup.bash
if [ -f "$WORKSPACE_ROOT/install/setup.bash" ]; then
  source "$WORKSPACE_ROOT/install/setup.bash"
fi
set -u
unset FASTRTPS_DEFAULT_PROFILES_FILE FASTDDS_DEFAULT_PROFILES_FILE
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-122}"
export ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_LOG_DIR="${ROS_LOG_DIR:-/tmp/parking_robot_ros_logs}"
mkdir -p "$ROS_LOG_DIR"

exec python3 "$SCRIPT_DIR/isaac_entry_mission_server.py" "$@"
