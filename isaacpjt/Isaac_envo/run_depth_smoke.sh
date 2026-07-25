#!/bin/bash
# R4 측면 뎁스 토픽 스모크 런처. run_bridge_smoke.sh 와 동일 DDS 환경 패턴.
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"

set +u
source /opt/ros/humble/setup.bash
set -u
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-126}"
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE="${FASTRTPS_DEFAULT_PROFILES_FILE:-$HOME/.ros/fastdds_whitelist.xml}"

exec python3 "$SCRIPT_DIR/depth_smoke.py" "$@"
