#!/bin/bash
# parking_control_mvp 웹 UI를 dual 모드로 실행 — 입차/출차 요청을 서로 다른
# 로봇 그룹(서로 다른 Isaac Sim PC의 user_request_gateway_node)으로 분리 라우팅.
# 라우팅 표(dispatch_service 이름 등)는 core/db.py가 최초 실행 시 SQLite에 시드한다.
APP_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
WS="$(cd -- "$APP_DIR/.." && pwd)"
source /opt/ros/humble/setup.bash            # ROS setup 은 nounset 비호환 → set -u 미사용
source "$WS/install/setup.bash"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-126}"
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE="${FASTRTPS_DEFAULT_PROFILES_FILE:-$HOME/.ros/fastdds_whitelist.xml}"
export FASTDDS_DEFAULT_PROFILES_FILE="${FASTDDS_DEFAULT_PROFILES_FILE:-$HOME/.ros/fastdds_whitelist.xml}"
export PARKING_MODE=dual
export PATH="$HOME/.local/bin:$PATH"
cd "$APP_DIR"
exec python3 -m uvicorn main:app --host 127.0.0.1 --port "${UI_PORT:-8000}"
