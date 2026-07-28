#!/bin/bash
# parking_control_mvp 웹 UI를 dual 모드로 실행 — 입차/출차 요청을 서로 다른
# 로봇 그룹(서로 다른 Isaac Sim PC의 user_request_gateway_node)으로 분리 라우팅.
# 라우팅 표(dispatch_service 이름 등)는 core/db.py가 최초 실행 시 SQLite에 시드한다.
APP_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
WS="$(cd -- "$APP_DIR/.." && pwd)"
source /opt/ros/humble/setup.bash            # ROS setup 은 nounset 비호환 → set -u 미사용
source "$WS/install/setup.bash"
# 2026-07-28 수정: 화이트리스트를 끄지 않는다 — 이 프로젝트 실측(nodes.launch.py/
# sim_bridge.sh 도크스트링): builtin transport 로는 크로스버전 FastDDS 의 SHM
# 데이터전송이 비호환이라 discovery 는 되어도 서비스/토픽 데이터가 안 흐른다.
# 로봇 스택(nodes.launch.py)과 반드시 동일 화이트리스트 프로파일을 써야 이
# UI 프로세스도 dispatch_parking_task_exit 서비스를 실제로 호출할 수 있다.
export FASTRTPS_DEFAULT_PROFILES_FILE="$HOME/.ros/fastdds_whitelist.xml"
export FASTDDS_DEFAULT_PROFILES_FILE="$HOME/.ros/fastdds_whitelist.xml"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-126}"
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export PARKING_MODE=dual
export PATH="$HOME/.local/bin:$PATH"
cd "$APP_DIR"
exec python3 -m uvicorn main:app --host 127.0.0.1 --port "${UI_PORT:-8000}"
