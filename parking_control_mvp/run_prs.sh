#!/bin/bash
# parking_control_mvp 웹 UI를 parkbot_motion + parking_control ROS2 스택에 연동한다.
# sim_bridge.sh, parkbot_motion nodes.launch.py, control_tower.launch.py와 함께 사용한다.
APP_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
WS="$(cd -- "$APP_DIR/.." && pwd)"
source /opt/ros/humble/setup.bash            # ROS setup 은 nounset 비호환 → set -u 미사용
source "$WS/install/setup.bash"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-126}"
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
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
export PARKING_MODE=ros2
export PATH="$HOME/.local/bin:$PATH"
cd "$APP_DIR"
exec python3 -m uvicorn main:app --host 127.0.0.1 --port "${UI_PORT:-8000}"
