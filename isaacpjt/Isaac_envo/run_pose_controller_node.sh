#!/bin/bash
# R3b pose_controller_node 런처 (터미널 B, 시스템 ROS 2 Humble).
# Isaac 러너(--bridge)와 같은 도메인/화이트리스트를 쓴다. 콜콘 빌드 없이도 돌게
# PYTHONPATH 로 패키지를 얹는다(run_probe_a_detector.sh 와 동일 패턴).
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
PKG_SRC="$(cd -- "$SCRIPT_DIR/../../src/parkbot_motion" && pwd)"

set +u
source /opt/ros/humble/setup.bash
set -u
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-126}"
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE="${FASTRTPS_DEFAULT_PROFILES_FILE:-$HOME/.ros/fastdds_whitelist.xml}"
export PYTHONPATH="$PKG_SRC:${PYTHONPATH:-}"

exec python3 -m parkbot_motion.pose_controller_node "$@"
