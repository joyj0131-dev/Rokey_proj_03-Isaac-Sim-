#!/bin/bash
# Launch the Isaac Sim mecanum /cmd_vel driver with the team ROS 2 setup for
# Isaac Sim 5.1 (Python 3.11). Mirrors the exports you run in the terminal:
#
#   export ROS_DOMAIN_ID=122                # 팀 도메인
#   export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
#   export FASTRTPS_DEFAULT_PROFILES_FILE="$HOME/.ros/fastdds_whitelist.xml"
#   export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$HOME/dev_ws/isaac_sim/isaacsim/_build/\
#          linux-x86_64/release/exts/isaacsim.ros2.bridge/humble/lib
#
# Isaac uses its INTERNAL ROS 2 Humble (built for Python 3.11); do NOT source
# /opt/ros here (that pulls in the 3.10 rclpy/rmw and the bridge fails to start).
# The external publisher machine uses its own system ROS 2 (3.10); they meet over
# DDS on the same ROS_DOMAIN_ID + RMW + FastDDS whitelist.
#
# Usage:
#   ./run_mecanum_ros2_drive.sh --gui                 # watch it
#   ./run_mecanum_ros2_drive.sh                        # headless
#   ROS_DOMAIN_ID=7 ./run_mecanum_ros2_drive.sh --gui  # override domain
set -u

REL=$HOME/dev_ws/isaac_sim/isaacsim/_build/linux-x86_64/release
BRIDGE_LIB=$REL/exts/isaacsim.ros2.bridge/humble/lib
DRIVER=/home/rokey/cobot3_ws/isaacpjt/Isaac_envo/mecanum_ros2_drive.py

export ROS_DISTRO=humble
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-122}
export RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}
export FASTRTPS_DEFAULT_PROFILES_FILE=${FASTRTPS_DEFAULT_PROFILES_FILE:-$HOME/.ros/fastdds_whitelist.xml}

# Drop any sourced system ROS 2 from the paths (so a sourced shell still works),
# then append Isaac's internal Humble libs so the bridge can load its rmw.
export LD_LIBRARY_PATH="$(echo "${LD_LIBRARY_PATH:-}" | tr ':' '\n' | grep -v '/opt/ros' | paste -sd:):$BRIDGE_LIB"
export PYTHONPATH="$(echo "${PYTHONPATH:-}" | tr ':' '\n' | grep -v '/opt/ros' | paste -sd:)"
unset AMENT_PREFIX_PATH AMENT_CURRENT_PREFIX COLCON_PREFIX_PATH CMAKE_PREFIX_PATH 2>/dev/null || true

[ -f "$FASTRTPS_DEFAULT_PROFILES_FILE" ] || echo "[warn] FastDDS whitelist not found: $FASTRTPS_DEFAULT_PROFILES_FILE"
echo "[run_mecanum_ros2_drive] domain=$ROS_DOMAIN_ID rmw=$RMW_IMPLEMENTATION whitelist=$FASTRTPS_DEFAULT_PROFILES_FILE (internal Humble libs)"
cd "$REL"
exec ./python.sh "$DRIVER" "$@"
