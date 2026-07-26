#!/bin/bash
# parking_control_mvp 웹 UI를 팀원 parking_robot_system(feat/camera)에 연동해 실행.
# 러너(dock_lift_handoff_runner) + parking_robot_system 런치가 떠 있어야 실동작.
APP_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
WS="$(cd -- "$APP_DIR/.." && pwd)"
source /opt/ros/humble/setup.bash            # ROS setup 은 nounset 비호환 → set -u 미사용
source "$WS/install/setup.bash"
unset FASTRTPS_DEFAULT_PROFILES_FILE FASTDDS_DEFAULT_PROFILES_FILE
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-126}"
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export PARKING_MODE=prs
export PATH="$HOME/.local/bin:$PATH"
cd "$APP_DIR"
exec python3 -m uvicorn main:app --host 127.0.0.1 --port "${UI_PORT:-8000}"
