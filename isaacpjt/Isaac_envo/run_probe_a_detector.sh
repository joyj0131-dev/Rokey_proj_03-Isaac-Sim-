#!/bin/bash
# Probe A 외부 검출 노드 런처 (터미널 B, 시스템 ROS 2 Humble).
# Isaac 러너와 같은 도메인/화이트리스트를 쓴다. 콜콘 빌드 없이도 돌게 PYTHONPATH 로
# 패키지를 얹는다(원하면 `colcon build --packages-select parkbot_aruco` 후
# `ros2 run parkbot_aruco probe_a_detector` 로도 실행 가능).
set -u
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
PKG_SRC="$(cd -- "$SCRIPT_DIR/../../src/parkbot_aruco" && pwd)"

source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-126}"
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE="${FASTRTPS_DEFAULT_PROFILES_FILE:-$HOME/.ros/fastdds_whitelist.xml}"
export PYTHONPATH="$PKG_SRC:${PYTHONPATH:-}"

exec python3 -m parkbot_aruco.probe_a_detector_node "$@"
