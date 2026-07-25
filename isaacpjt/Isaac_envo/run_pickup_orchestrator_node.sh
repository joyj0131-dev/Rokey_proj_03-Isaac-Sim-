#!/bin/bash
# R5b pickup_orchestrator_node 런처 (터미널 B, 시스템 ROS 2 Humble).
# Isaac 러너(--bridge)와 같은 도메인/화이트리스트를 쓴다. run_ingress_node.sh 와
# 동일 패턴 -- ExecuteParkingTask/ControlLift/IngressUnderTruck 액션 타입
# (parking_robot_interfaces) 은 코드생성이 필요해 콜콘 빌드 결과(install/)를
# PYTHONPATH 에 얹어야 한다.
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="$(cd -- "$SCRIPT_DIR/../.." && pwd)"
PKG_SRC="$WS_ROOT/src/parkbot_motion"
IFACE_DIST="$WS_ROOT/install/parking_robot_interfaces/local/lib/python3.10/dist-packages"

set +u
source /opt/ros/humble/setup.bash
set -u
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-126}"
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE="${FASTRTPS_DEFAULT_PROFILES_FILE:-$HOME/.ros/fastdds_whitelist.xml}"
export PYTHONPATH="$PKG_SRC:$IFACE_DIST:${PYTHONPATH:-}"
export AMENT_PREFIX_PATH="$WS_ROOT/install/parking_robot_interfaces:${AMENT_PREFIX_PATH:-}"
export LD_LIBRARY_PATH="$WS_ROOT/install/parking_robot_interfaces/lib:${LD_LIBRARY_PATH:-}"

exec python3 -m parkbot_motion.pickup_orchestrator_node "$@"
