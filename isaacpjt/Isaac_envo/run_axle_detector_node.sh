#!/bin/bash
# R4 axle_detector_node 런처 (터미널 B, 시스템 ROS 2 Humble).
# Isaac 러너(--bridge)와 같은 도메인/화이트리스트를 쓴다. run_pose_controller_node.sh
# 와 동일 패턴(콜콘 빌드 없이 PYTHONPATH 로 패키지를 얹는다).
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
PKG_SRC="$(cd -- "$SCRIPT_DIR/../../src/parkbot_motion" && pwd)"

set +u
source /opt/ros/humble/setup.bash
set -u
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-126}"
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE="${FASTRTPS_DEFAULT_PROFILES_FILE:-$HOME/.ros/fastdds_whitelist.xml}"
export PYTHONPATH="$PKG_SRC:${PYTHONPATH:-}"

exec python3 -m parkbot_motion.axle_detector_node "$@"
