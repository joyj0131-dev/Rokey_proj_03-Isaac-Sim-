#!/bin/bash
set -u
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REL=$HOME/dev_ws/isaac_sim/isaacsim/_build/linux-x86_64/release
# Isaac 내부 Humble libs 사용. /opt/ros 소싱 금지.
unset PYTHONPATH AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH
export ROS_DISTRO=humble
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-126}"
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
# 외부 검출 노드(probe_a_detector)와 DDS 디스커버리를 맞추려면 양쪽이 같은 FastDDS
# 화이트리스트를 써야 한다(DEBUG_LOG 2-터미널 설정). 화이트리스트에 127.0.0.1 이
# 있어 로컬 통신도 정상 동작한다. 파일이 없으면(폐쇄망 밖) 무시된다.
export FASTRTPS_DEFAULT_PROFILES_FILE="${FASTRTPS_DEFAULT_PROFILES_FILE:-$HOME/.ros/fastdds_whitelist.xml}"
export LD_LIBRARY_PATH="$REL/exts/isaacsim.ros2.bridge/humble/lib"
exec "$REL/python.sh" "$SCRIPT_DIR/parking_v4_runner.py" "$@"
