#!/bin/bash
# R5b pickup_smoke.py 런처 (터미널 클라이언트, 시스템 ROS 2 Humble).
# run_ingress_smoke.sh 와 동일 패턴.
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="$(cd -- "$SCRIPT_DIR/../../.." && pwd)"
IFACE_DIST="$WS_ROOT/install/parking_robot_interfaces/local/lib/python3.10/dist-packages"

set +u
source /opt/ros/humble/setup.bash
set -u
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-126}"
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE="${FASTRTPS_DEFAULT_PROFILES_FILE:-$HOME/.ros/fastdds_whitelist.xml}"
export PYTHONPATH="$IFACE_DIST:${PYTHONPATH:-}"
export AMENT_PREFIX_PATH="$WS_ROOT/install/parking_robot_interfaces:${AMENT_PREFIX_PATH:-}"
export LD_LIBRARY_PATH="$WS_ROOT/install/parking_robot_interfaces/lib:${LD_LIBRARY_PATH:-}"

exec python3 "$SCRIPT_DIR/pickup_smoke.py" "$@"
