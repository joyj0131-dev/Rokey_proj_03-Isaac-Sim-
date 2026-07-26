#!/bin/bash
# 배포 측위(터미널 B): marker_localizer_node 를 v4 지도 + fuse + 보정 마운트로 실행.
# Isaac 러너(parking_v4_runner.sh)와 같은 도메인/화이트리스트를 쓴다. 콜콘 빌드 없이도
# 돌게 PYTHONPATH 로 패키지를 얹는다.
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
PKG_SRC="$(cd -- "$SCRIPT_DIR/../../src/parkbot_aruco" && pwd)"
MAP="$PKG_SRC/data/marker_map_v4.json"

# /opt/ros 의 setup.bash 는 미설정 변수를 참조하므로 nounset(set -u)을 켠 채
# 소싱하면 그 줄에서 스크립트가 죽는다. 소싱 동안만 nounset 을 끈다.
set +u
source /opt/ros/humble/setup.bash
set -u
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-122}"
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE="${FASTRTPS_DEFAULT_PROFILES_FILE:-$HOME/.ros/fastdds_whitelist.xml}"
export PYTHONPATH="$PKG_SRC:${PYTHONPATH:-}"

exec python3 -m parkbot_aruco.marker_localizer_node --ros-args \
  -p image_topic:=/robot_entry_lead/image_raw \
  -p camera_info_topic:=/robot_entry_lead/camera_info \
  -p odom_topic:=/robot_entry_lead/odom \
  -p marker_map:="$MAP" -p fuse:=true -p frame:=usd "$@"
