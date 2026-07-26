#!/bin/bash
# R3b navigate_to_pose 스모크 런처 (터미널 C, 시스템 ROS 2 Humble).
# Isaac 러너(--bridge)와 pose_controller_node 둘 다와 같은 도메인/화이트리스트를 쓴다.
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"

set +u
source /opt/ros/humble/setup.bash
set -u
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-126}"
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE="${FASTRTPS_DEFAULT_PROFILES_FILE:-$HOME/.ros/fastdds_whitelist.xml}"

exec python3 "$SCRIPT_DIR/navigate_smoke.py" "$@"
