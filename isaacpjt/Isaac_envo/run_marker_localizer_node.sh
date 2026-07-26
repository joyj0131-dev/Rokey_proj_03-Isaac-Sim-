#!/bin/bash
# marker_localizer_node 런처(인자 passthrough). run_pose_controller_node.sh 와
# 동일 패턴 — 시스템 ROS2 Humble 환경에서 parkbot_aruco 를 PYTHONPATH 로 얹어
# 콜콘 빌드 없이 실행. 모든 --ros-args 는 그대로 전달된다.
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
PKG_SRC="$(cd -- "$SCRIPT_DIR/../../src/parkbot_aruco" && pwd)"

set +u
source /opt/ros/humble/setup.bash
set -u
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-126}"
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE="${FASTRTPS_DEFAULT_PROFILES_FILE:-$HOME/.ros/fastdds_whitelist.xml}"
export PYTHONPATH="$PKG_SRC:${PYTHONPATH:-}"

exec python3 -m parkbot_aruco.marker_localizer_node "$@"
