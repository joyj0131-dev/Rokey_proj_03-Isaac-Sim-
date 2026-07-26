#!/bin/bash
# Isaac Sim(py3.11) 시뮬 브리지 런처. sim_bridge.py 를 Isaac 내장 파이썬으로 띄운다.
# ROS2 노드(py3.10)와는 DDS(domain 126, FastDDS 화이트리스트)로만 통신한다.
set -u
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REL=$HOME/dev_ws/isaac_sim/isaacsim/_build/linux-x86_64/release
# Isaac 내부 Humble libs 사용. /opt/ros 소싱 금지(버전 충돌).
unset PYTHONPATH AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH
export ROS_DISTRO=humble
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-126}"
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE="${FASTRTPS_DEFAULT_PROFILES_FILE:-$HOME/.ros/fastdds_whitelist.xml}"
export LD_LIBRARY_PATH="$REL/exts/isaacsim.ros2.bridge/humble/lib"
exec "$REL/python.sh" "$SCRIPT_DIR/sim_bridge.py" "$@"
