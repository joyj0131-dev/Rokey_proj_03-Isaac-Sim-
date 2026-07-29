#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE="$(cd -- "$SCRIPT_DIR/.." && pwd)"
RVIZ_CONFIG="$WORKSPACE/src/parking_control/config/slot_occupancy.rviz"
WORLD_RELAY="$WORKSPACE/src/parking_control/scripts/lidar/ros_pointcloud_world_relay.py"
WEB_RUNNER="$WORKSPACE/parking_control_mvp/run_prs.sh"

children=()
cleanup() {
    set +e
    for pid in "${children[@]}"; do
        if kill -0 "$pid" 2>/dev/null; then
            kill -INT -- "-$pid" 2>/dev/null
        fi
    done
    for pid in "${children[@]}"; do
        wait "$pid" 2>/dev/null
    done
}
handle_signal() {
    exit 130
}
trap cleanup EXIT
trap handle_signal INT TERM

# ROS setup 스크립트 내부의 미정의 변수를 허용한 뒤 nounset을 다시 켠다.
set +u
source /opt/ros/humble/setup.bash
if [[ ! -f "$WORKSPACE/install/setup.bash" ]]; then
    echo "install/setup.bash가 없습니다. 먼저 colcon build를 실행하세요." >&2
    exit 1
fi
source "$WORKSPACE/install/setup.bash"
set -u

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-126}"
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-0}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
dds_profile="${FASTRTPS_DEFAULT_PROFILES_FILE:-${FASTDDS_DEFAULT_PROFILES_FILE:-}}"
default_dds_profile="$HOME/.ros/fastdds_whitelist.xml"
if [[ -z "$dds_profile" && -f "$default_dds_profile" ]]; then
    dds_profile="$default_dds_profile"
fi
if [[ -n "$dds_profile" && ! -f "$dds_profile" ]]; then
    echo "Fast DDS profile을 찾을 수 없습니다: $dds_profile" >&2
    dds_profile="$default_dds_profile"
fi
if [[ -f "$dds_profile" ]]; then
    export FASTRTPS_DEFAULT_PROFILES_FILE="$dds_profile"
    export FASTDDS_DEFAULT_PROFILES_FILE="$dds_profile"
else
    unset FASTRTPS_DEFAULT_PROFILES_FILE FASTDDS_DEFAULT_PROFILES_FILE
fi

if [[ "${START_LIDAR_RELAY:-1}" == "1" ]]; then
    setsid python3 "$WORLD_RELAY" &
    children+=("$!")
fi

if [[ "${START_CONTROL_TOWER:-1}" == "1" ]]; then
    setsid ros2 launch parking_control control_tower.launch.py &
    children+=("$!")
fi

setsid rviz2 -d "$RVIZ_CONFIG" &
rviz_pid=$!
children+=("$rviz_pid")

setsid env UI_PORT="${UI_PORT:-8000}" PARKING_MODE=ros2 \
    bash "$WEB_RUNNER" &
web_pid=$!
children+=("$web_pid")

echo "웹 UI: http://127.0.0.1:${UI_PORT}"
echo "RViz2를 닫아도 웹 서버는 계속 실행됩니다."
echo "전체 종료: 이 터미널에서 Ctrl+C"

# RViz 종료는 웹을 종료시키지 않는다. 웹 서버 또는 스크립트 신호를 기다린다.
wait "$web_pid"
