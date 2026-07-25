#!/usr/bin/env bash
# 타 PC용: LiDAR 월드 변환 + 2D 점유 지도 + RViz를 한 번에 실행한다.
set -eo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
PACKAGE_ROOT="$(cd -- "$SCRIPT_DIR/../.." && pwd)"
RVIZ_CONFIG="$PACKAGE_ROOT/config/lidar_live.rviz"
WORLD_RELAY="$SCRIPT_DIR/ros_pointcloud_world_relay.py"

source /opt/ros/humble/setup.bash
set -u
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-122}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"

relay_pid=""
grid_pid=""
rviz_pid=""
cleanup() {
    set +e
    for pid in "$rviz_pid" "$grid_pid" "$relay_pid"; do
        if [[ -n "$pid" ]]; then
            kill -INT "$pid" 2>/dev/null
            wait "$pid" 2>/dev/null
        fi
    done
}
trap cleanup EXIT INT TERM

python3 "$WORLD_RELAY" &
relay_pid=$!

PYTHONPATH="$PACKAGE_ROOT:${PYTHONPATH:-}" python3 -m \
    parking_control.lidar_occupancy_grid_node &
grid_pid=$!

rviz2 -d "$RVIZ_CONFIG" &
rviz_pid=$!
wait "$rviz_pid"
