#!/bin/bash
# R3c 마커융합 navigate_to_pose 스모크 런처 — run_navigate_smoke.sh 와 동일 패턴.
# Isaac 러너(--bridge)와 marker_localizer_node(fuse:=true)/pose_controller_node
# (pose_msg_type:=posestamped) 모두와 같은 도메인/화이트리스트를 쓴다.
# 사용: bash run_navigate_smoke_fused.sh <robot_id> <test_a_dist_m>
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"

set +u
source /opt/ros/humble/setup.bash
set -u
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-126}"
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE="${FASTRTPS_DEFAULT_PROFILES_FILE:-$HOME/.ros/fastdds_whitelist.xml}"

exec python3 "$SCRIPT_DIR/navigate_smoke_fused.py" "$@"
