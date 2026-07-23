#!/bin/bash
set -u
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REL=$HOME/dev_ws/isaac_sim/isaacsim/_build/linux-x86_64/release
# Isaac 내부 Humble libs 사용. /opt/ros 소싱 금지, 화이트리스트 unset(내부 rclpy 통신).
unset PYTHONPATH AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH
unset FASTRTPS_DEFAULT_PROFILES_FILE FASTDDS_DEFAULT_PROFILES_FILE
export ROS_DISTRO=humble
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-126}"
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export LD_LIBRARY_PATH="$REL/exts/isaacsim.ros2.bridge/humble/lib"
exec "$REL/python.sh" "$SCRIPT_DIR/dock_lift_handoff_runner.py" "$@"
