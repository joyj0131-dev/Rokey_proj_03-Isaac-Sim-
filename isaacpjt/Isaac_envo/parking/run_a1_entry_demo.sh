#!/bin/bash
set -eu

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
MISSION_LAUNCHER="$SCRIPT_DIR/../../../src/parking_control/scripts/run_isaac_entry_mission_server.sh"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-122}"

"$MISSION_LAUNCHER" &
MISSION_PID=$!

cleanup() {
  kill "$MISSION_PID" 2>/dev/null || true
  wait "$MISSION_PID" 2>/dev/null || true
}
trap cleanup EXIT INT TERM

"$SCRIPT_DIR/run_dual_robot_ros2_field.sh" "$@"
