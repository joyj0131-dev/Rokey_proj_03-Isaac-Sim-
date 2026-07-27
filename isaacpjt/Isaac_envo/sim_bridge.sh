#!/bin/bash
# Isaac Sim(py3.11) 시뮬 브리지 런처. sim_bridge.py 를 Isaac 내장 파이썬으로 띄운다.
# ROS2 노드(py3.10)와는 DDS(domain 126)로만 통신한다.
set -u
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REL=$HOME/dev_ws/isaac_sim/isaacsim/_build/linux-x86_64/release
# Isaac 내부 Humble libs 사용. /opt/ros 소싱 금지(버전 충돌).
unset PYTHONPATH AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH
export ROS_DISTRO=humble
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-126}"
export ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
# FastDDS 화이트리스트 필수 — rokey 머신 실측(2026-07-27 재확인): builtin 전송이면
# 크로스버전 FastDDS(Isaac 번들 vs 시스템 Humble)의 SHM 데이터전송이 비호환이라
# discovery 로 토픽은 보여도 **데이터가 안 흐른다**(publisher count 0). 화이트리스트
# (useBuiltinTransports=false, UDP 를 127.0.0.1/10.10.0.x 로 강제)로 SHM 을 우회하고
# 여러 인터페이스(wifi/tailscale) 혼선을 없애야 브리지↔노드 데이터가 흐른다.
# 브리지·노드(nodes.launch.py) 양쪽에 반드시 동일 프로파일을 걸어야 한다.
export FASTRTPS_DEFAULT_PROFILES_FILE="$HOME/.ros/fastdds_whitelist.xml"
export FASTDDS_DEFAULT_PROFILES_FILE="$HOME/.ros/fastdds_whitelist.xml"
export LD_LIBRARY_PATH="$REL/exts/isaacsim.ros2.bridge/humble/lib"
# 입차팀 카메라만 브리지 — 출차팀(exit_lead/follow) 카메라 렌더 OFF 로 GPU 렌더부하↓
# (입차 개발 중 RTF 절약). 전체 복원: BRIDGE_CAMERAS=all bash sim_bridge.sh
# 다른 조합: BRIDGE_CAMERAS=entry_lead,entry_follow,exit_lead ...
exec "$REL/python.sh" "$SCRIPT_DIR/sim_bridge.py" \
  --bridge-cameras="${BRIDGE_CAMERAS:-entry_lead,entry_follow}" "$@"
