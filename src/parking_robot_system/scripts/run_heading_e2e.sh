#!/bin/bash
# Isaac + ROS P1 파이프라인 + heading probe를 한 환경에서 반복 실행한다.
set -eo pipefail

WS="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../../.." && pwd)"
RUN_DIR="$(mktemp -d /tmp/parking-heading-e2e.XXXXXX)"
ISAAC_LOG="$RUN_DIR/isaac.log"
ROS_LOG="$RUN_DIR/ros.log"
PROBE_LOG="$RUN_DIR/probe.log"
PIDS=()

cleanup() {
    for pid in "${PIDS[@]}"; do
        kill -TERM "$pid" 2>/dev/null || true
    done
    for _ in $(seq 1 10); do
        alive=0
        for pid in "${PIDS[@]}"; do
            if kill -0 "$pid" 2>/dev/null; then
                alive=1
            fi
        done
        [[ "$alive" == 0 ]] && break
        sleep 1
    done
    for pid in "${PIDS[@]}"; do
        kill -KILL "$pid" 2>/dev/null || true
    done
    for pid in "${PIDS[@]}"; do
        wait "$pid" 2>/dev/null || true
    done
    echo "HEADING_E2E_LOG_DIR=$RUN_DIR"
}
trap cleanup EXIT INT TERM

# 사용자가 같은 머신에서 띄운 개발 파이프라인(domain 126)과 서비스/액션 서버가
# 중복 발견되지 않도록 E2E 자동시험은 전용 domain에서만 실행한다.
export ROS_DOMAIN_ID="${HEADING_ROS_DOMAIN_ID:-127}"

echo "[heading-e2e] Isaac runner 시작"
bash "$WS/isaacpjt/Isaac_envo/dock_lift_handoff_runner.sh" >"$ISAAC_LOG" 2>&1 &
PIDS+=("$!")

ready=0
for attempt in $(seq 1 120); do
    if grep -q "DOCK_LIFT_HANDOFF_READY" "$ISAAC_LOG"; then
        ready=1
        break
    fi
    if ! kill -0 "${PIDS[0]}" 2>/dev/null; then
        echo "[heading-e2e] Isaac runner 조기 종료"
        tail -120 "$ISAAC_LOG"
        exit 10
    fi
    if (( attempt % 5 == 0 )); then
        echo "[heading-e2e] Isaac 준비 대기 ${attempt}/120"
    fi
    sleep 2
done
if [[ "$ready" != 1 ]]; then
    echo "[heading-e2e] Isaac READY 시간 초과"
    tail -120 "$ISAAC_LOG"
    exit 11
fi
grep "DOCK_STAGE_READY\|DOCK_LIFT_HANDOFF_READY" "$ISAAC_LOG" | tail -5

source /opt/ros/humble/setup.bash
export PYTHONPATH="$WS/src/parking_robot_system:$WS/build/parking_robot_interfaces/rosidl_generator_py:${PYTHONPATH:-}"
export LD_LIBRARY_PATH="$WS/build/parking_robot_interfaces:${LD_LIBRARY_PATH:-}"
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
unset FASTRTPS_DEFAULT_PROFILES_FILE FASTDDS_DEFAULT_PROFILES_FILE

start_node() {
    local module="$1"
    python3 -u -m "parking_robot_system.$module" >>"$ROS_LOG" 2>&1 &
    PIDS+=("$!")
}

echo "[heading-e2e] ROS 노드 시작"
start_node user_request_gateway
start_node task_dispatcher
start_node parking_slot_manager
start_node robot_task_orchestrator
start_node safety_monitor
start_node vehicle_detection_node
start_node navigate_action_server
start_node align_action_server
start_node lift_action_server
sleep 5

for pid in "${PIDS[@]:1}"; do
    if ! kill -0 "$pid" 2>/dev/null; then
        echo "[heading-e2e] ROS 노드 조기 종료 pid=$pid"
        tail -160 "$ROS_LOG"
        exit 12
    fi
done

echo "[heading-e2e] A2 입차 및 heading 측정 시작"
python3 -u "$WS/src/parking_robot_system/scripts/e2e_heading_probe.py" >"$PROBE_LOG" 2>&1 &
probe_pid="$!"
PIDS+=("$probe_pid")

elapsed=0
while kill -0 "$probe_pid" 2>/dev/null; do
    sleep 15
    elapsed=$((elapsed + 15))
    echo "[heading-e2e] E2E 진행 중 ${elapsed}s"
    grep -E "execute_parking_task (시작|종료)|출차 운반|입차 운반|운반 heading|복귀|픽업 시퀀스" "$ROS_LOG" | tail -8 || true
done

set +e
wait "$probe_pid"
probe_status="$?"
set -e
cat "$PROBE_LOG"
echo "[heading-e2e] probe exit=$probe_status"
grep -E "execute_parking_task (시작|종료)|운반 heading|픽업 시퀀스|리프트|실패" "$ROS_LOG" | tail -80 || true
exit "$probe_status"
