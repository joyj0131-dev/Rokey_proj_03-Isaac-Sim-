#!/bin/bash
# 속도 조절 UI 실행기. dock_lift_handoff_mission.sh 와 같은 방식으로 화이트리스트를
# 풀어야 mission 노드의 파라미터를 discover/set 할 수 있다(실측).
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source /opt/ros/humble/setup.bash  # ROS setup 은 nounset 비호환 → set -u 미사용
unset FASTRTPS_DEFAULT_PROFILES_FILE FASTDDS_DEFAULT_PROFILES_FILE
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-122}"
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
exec python3 "$SCRIPT_DIR/speed_control_ui.py" "$@"
