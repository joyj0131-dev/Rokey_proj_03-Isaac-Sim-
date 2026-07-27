#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
PACKAGE_ROOT="$(cd -- "$SCRIPT_DIR/../.." && pwd)"
RVIZ_CONFIG="$PACKAGE_ROOT/config/lidar_live.rviz"
WORLD_RELAY="$SCRIPT_DIR/ros_pointcloud_world_relay.py"
PUBLISHER_SCRIPT="$SCRIPT_DIR/run_lidar_publisher_v4.sh"

source /opt/ros/humble/setup.bash
set -u
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-122}"

isaac_pid=""
rviz_pid=""
relay_pid=""
grid_pid=""
pedestrian_pid=""
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
    if [[ -n "$grid_pid" ]]; then
        kill -INT "$grid_pid" 2>/dev/null
        wait "$grid_pid" 2>/dev/null
    fi
    if [[ -n "$pedestrian_pid" ]]; then
        kill -INT "$pedestrian_pid" 2>/dev/null
        wait "$pedestrian_pid" 2>/dev/null
    fi
}
trap cleanup EXIT INT TERM

bash "$PUBLISHER_SCRIPT" "$@" &
isaac_pid=$!

python3 "$WORLD_RELAY" &
relay_pid=$!

PYTHONPATH="$PACKAGE_ROOT:${PYTHONPATH:-}" python3 -m \
    parking_control.lidar_occupancy_grid_node &
grid_pid=$!

# 보행자 대기 차로(X=-8.5)의 사람/비사람 LiDAR 기하 분류 — points_world를
# occupancy grid와 같이 구독하므로 릴레이 하나로 둘 다 충분하다.
PYTHONPATH="$PACKAGE_ROOT:${PYTHONPATH:-}" python3 -m \
    parking_control.pedestrian_obstacle_node &
pedestrian_pid=$!

rviz2 -d "$RVIZ_CONFIG" &
rviz_pid=$!
wait "$rviz_pid"
