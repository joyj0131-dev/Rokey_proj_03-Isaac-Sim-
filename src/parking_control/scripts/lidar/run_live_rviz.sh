#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
PACKAGE_ROOT="$(cd -- "$SCRIPT_DIR/../.." && pwd)"
CAPTURE_SCRIPT="$SCRIPT_DIR/capture_lidar.py"
RVIZ_CONFIG="$PACKAGE_ROOT/config/lidar_live.rviz"
WORLD_RELAY="$SCRIPT_DIR/ros_pointcloud_world_relay.py"

source /opt/ros/humble/setup.bash
set -u
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-122}"

isaac_pid=""
rviz_pid=""
relay_pid=""
cleanup() {
    set +e
    if [[ -n "$rviz_pid" ]]; then
        kill -INT "$rviz_pid" 2>/dev/null
        wait "$rviz_pid" 2>/dev/null
    fi
    if [[ -n "$isaac_pid" ]]; then
        kill -INT "$isaac_pid" 2>/dev/null
        wait "$isaac_pid" 2>/dev/null
    fi
    if [[ -n "$relay_pid" ]]; then
        kill -INT "$relay_pid" 2>/dev/null
        wait "$relay_pid" 2>/dev/null
    fi
}
trap cleanup EXIT INT TERM

python3 "$CAPTURE_SCRIPT" --live --headless --ros2 "$@" &
isaac_pid=$!

python3 "$WORLD_RELAY" &
relay_pid=$!

rviz2 -d "$RVIZ_CONFIG" &
rviz_pid=$!
wait "$rviz_pid"
