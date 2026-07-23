#!/bin/bash
# 관제 컴퓨터에서 띄워야 하는 ROS2 쪽을 터미널 1개로 묶는다:
#   1) parking_robot_system: robot_task_orchestrator + 액션서버 4개 (ros2 launch, 백그라운드)
#   2) parking_control:      parking_slot_manager + task_dispatcher (ros2 launch, 백그라운드)
#
# 웹 대시보드(FastAPI/uvicorn)는 여기 포함하지 않는다 — uvicorn 접속 로그
# ("GET /api/dashboard ...")가 매초 찍혀서 ROS2 노드 로그가 그 사이에 묻힌다.
# 대시보드는 별도 터미널에서 따로 띄운다:
#   cd parking_control_mvp && PARKING_MODE=ros2 python3 -m uvicorn main:app --port 8000
#
# Isaac Sim 쪽 씬(dock_lift_handoff_runner.sh)은 별도 컴퓨터에서 먼저 띄워둔 상태여야 한다.
# ROS_DOMAIN_ID는 Isaac 쪽 dock_lift_handoff_runner.sh/mission.sh 기본값(122)과 맞춘 것 —
# 팀이 다른 값을 쓰기로 했으면 아래 export도 맞춰서 바꿀 것.
set -e
# ROS setup.bash는 nounset(set -u)과 비호환(내부적으로 정의 안 된 변수를 참조함) —
# dock_lift_handoff_mission.sh와 동일한 이유로 -u는 쓰지 않는다.

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$(cd -- "$SCRIPT_DIR/../../.." && pwd)"

source /opt/ros/humble/setup.bash
source "$WS_DIR/install/setup.bash"
unset FASTRTPS_DEFAULT_PROFILES_FILE FASTDDS_DEFAULT_PROFILES_FILE
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-122}"
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

PIDS=()
cleanup() {
    echo "종료 중 — 백그라운드 노드도 함께 내림..."
    for pid in "${PIDS[@]}"; do
        kill "$pid" 2>/dev/null || true
    done
}
trap cleanup EXIT INT TERM

echo "[1/2] parking_robot_system 실행부(오케스트레이터+액션서버4) 기동..."
ros2 launch parking_robot_system parking_robot_system.launch.py &
PIDS+=($!)

echo "[2/2] parking_control 관제탑(슬롯관리자+디스패처+robot_position_bridge) 기동..."
ros2 launch parking_control control_tower.launch.py &
PIDS+=($!)

echo ""
echo "ROS2 쪽 다 떴습니다. 웹 대시보드는 로그가 섞이지 않도록 별도 터미널에서 따로 실행하세요:"
echo "  cd $WS_DIR/parking_control_mvp && PARKING_MODE=ros2 python3 -m uvicorn main:app --port 8000"
echo ""
echo "Ctrl+C로 이 터미널의 노드 전부 종료"
wait
