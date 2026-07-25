#!/bin/bash
# R4 리프트 스모크 런처. run_lift_action_server.sh 와 동일하게 parking_robot_interfaces
# 빌드 결과(install/)를 PYTHONPATH 에 얹어야 ControlLift 액션 타입을 임포트할 수 있다.
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="$(cd -- "$SCRIPT_DIR/../.." && pwd)"
IFACE_DIST="$WS_ROOT/install/parking_robot_interfaces/local/lib/python3.10/dist-packages"

set +u
source /opt/ros/humble/setup.bash
set -u
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-126}"
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE="${FASTRTPS_DEFAULT_PROFILES_FILE:-$HOME/.ros/fastdds_whitelist.xml}"
export PYTHONPATH="$IFACE_DIST:${PYTHONPATH:-}"
export LD_LIBRARY_PATH="$WS_ROOT/install/parking_robot_interfaces/lib:${LD_LIBRARY_PATH:-}"

exec python3 "$SCRIPT_DIR/lift_smoke.py" "$@"
