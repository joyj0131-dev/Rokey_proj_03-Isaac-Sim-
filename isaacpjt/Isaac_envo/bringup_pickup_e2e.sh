#!/bin/bash
# R5b 종단검증 브링업 -- ExecuteParkingTask 2로봇 픽업 안무 전체 스택을 한 번에
# 띄우고/내린다. 개별 run_*.sh 런처(R2~R5a 가 만든 것들)를 그대로 조합만 한다
# (로직 재구현 없음). 각 노드는 자기 log 파일로 백그라운드 기동한다.
#
# 사용:
#   bringup_pickup_e2e.sh up      -- 브리지+전체 ROS2 노드스택 기동(BRIDGE_READY까지 대기)
#   bringup_pickup_e2e.sh down    -- 전부 정리(래퍼+자식 PID 모두 시그널, taskR5a §7 교훈)
#   bringup_pickup_e2e.sh status  -- 살아있는 관련 프로세스 나열
#
# 로봇 배치(브리지 --bridge-depth-pose 재사용, docstring 근거는 실행 로그에 남김):
#   entry_follow: x=-1.50 (베이 진입선 APPROACH_X=-3.50 앞쪽 2m -- 실제 접근 이동을
#                 검증하기 위해 목표에 미리 놓지 않는다)
#   entry_lead:   x=+2.50 (entry_follow 의 전체 회랑 왕복 경로(x<=-1.5, 특히 최종
#                 정차 x≈-10.16)와 절대 겹치지 않는 양(+)의 x -- in-process 미션의
#                 "_park_away" 대역(entry_lead 를 entry_follow 완주까지 회랑 밖에
#                 대기시키는 것, parking_v4_runner.py 3486행 부근)과 같은 목적을
#                 "회랑 반대편에 미리 놓기"로 대신한다. Phase B 체인이 없는 이번
#                 범위(R5b 브리프: Phase B 는 범위 밖)에서 가장 단순하고 물리적으로
#                 안전한 시작 배치.
# 둘 다 --bridge-depth-pose 로 스폰 yaw 를 180도 반전(=-90, -X 를 보게)해서
# 시작한다 -- 실제 Phase B 종단자세는 이미 대략 이 방향을 향한 채 도착한다(그
# 이후 대각이동+제자리회전으로 미세보정만 남는 상태, parking_v4_runner.py
# _approach() 주석 참고)는 전제를 근사한다.
set -u
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="$(cd -- "$SCRIPT_DIR/../.." && pwd)"
LOG_DIR="${LOG_DIR:-/tmp/claude-1000/-home-rokey-p3-cobot-ws/9b4dfc19-ad9a-491c-860d-86ec5e4ba26b/scratchpad/pickup_e2e_logs}"
PID_FILE="$LOG_DIR/pids.txt"
MARKER_MAP="$WS_ROOT/src/parkbot_aruco/data/marker_map_v4.json"

# START_AT_DOCK=1(기본): Phase B 전체(도크→XN→베이→픽업). 로봇을 도크 기본
#   스폰(build_stage 가 dock 마커 x, z=2.2, +X 를 보게 배치 -- 러너 미션 스폰과
#   동일)에 그대로 두고, odom 은 line 1395 에서 도크 GT 로 시드된다. 오케스트레이터
#   run_phase_b_first(기본 True)가 Phase B 레그를 corridor_plan 앞에 실행.
# START_AT_DOCK=0: 레거시 R5b 스모크 — 로봇을 접근선(FOLLOW_X/LEAD_X)으로
#   텔레포트(--bridge-depth-pose), Phase B 없이 XN 종단자세 근사에서 시작.
START_AT_DOCK="${START_AT_DOCK:-1}"
FOLLOW_X="${FOLLOW_X:--1.50}"
LEAD_X="${LEAD_X:-2.50}"
CAM_ROBOTS="${CAM_ROBOTS:-entry_lead,entry_follow}"
GOAL_TIMEOUT="${GOAL_TIMEOUT:-300.0}"    # pose_controller_node/ingress_node 서버측 상한(RTF 저하 여유)
RAMP_WAIT="${RAMP_WAIT:-10.0}"           # lift_action_server 벽시계 대기(RTF 저하 여유, 기본 3.0s -> 10.0s)

mkdir -p "$LOG_DIR"

_wait_for_log_pattern() {
    # _wait_for_log_pattern <logfile> <pattern> <timeout_sec>
    local log="$1" pat="$2" timeout="$3"
    local waited=0
    while [ "$waited" -lt "$timeout" ]; do
        if grep -q "$pat" "$log" 2>/dev/null; then
            return 0
        fi
        sleep 1
        waited=$((waited + 1))
    done
    return 1
}

cmd_up() {
    echo "=== bringup_pickup_e2e up: LOG_DIR=$LOG_DIR ===" | tee "$LOG_DIR/bringup.log"
    : > "$PID_FILE"

    echo "[1/13] Isaac 브리지 기동 중… (BRIDGE_READY 까지 최대 180s 대기)"
    # START_AT_DOCK=1: 도크 기본 스폰 유지(depth-pose 텔레포트 없음) + 후방캠 발행
    # (--bridge-rear, T2) — Phase B 후방캠 도크점검용 /robot_<id>/rear/image_raw.
    # START_AT_DOCK=0: 레거시 접근선 텔레포트(--bridge-depth-pose).
    _bridge_pose_args=()
    _bridge_rear_arg=()
    if [ "$START_AT_DOCK" = "1" ]; then
        _bridge_rear_arg=(--bridge-rear)
    else
        _bridge_pose_args=(--bridge-depth-pose="entry_follow:${FOLLOW_X}" \
                           --bridge-depth-pose="entry_lead:${LEAD_X}")
    fi
    ( cd "$SCRIPT_DIR" && \
      bash parking_v4_runner.sh --bridge \
        --bridge-cameras="$CAM_ROBOTS" \
        "${_bridge_rear_arg[@]}" \
        "${_bridge_pose_args[@]}" \
        > "$LOG_DIR/bridge.log" 2>&1 & )
    if ! _wait_for_log_pattern "$LOG_DIR/bridge.log" "BRIDGE_READY" 180; then
        echo "!!! BRIDGE_READY 를 180s 안에 못 봤습니다 -- $LOG_DIR/bridge.log 확인" >&2
        return 1
    fi
    grep "BRIDGE_READY" "$LOG_DIR/bridge.log" | tail -1

    _launch() {
        # _launch <label> <logfile> <cmd...>
        local label="$1" log="$2"; shift 2
        ( set +u; source /opt/ros/humble/setup.bash; set -u
          export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-126}"
          export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
          export FASTRTPS_DEFAULT_PROFILES_FILE="${FASTRTPS_DEFAULT_PROFILES_FILE:-$HOME/.ros/fastdds_whitelist.xml}"
          export PYTHONPATH="$WS_ROOT/src/parkbot_aruco:${PYTHONPATH:-}"
          "$@" > "$log" 2>&1 &
          echo "$label $!" >> "$PID_FILE" )
        sleep 0.3
    }

    # Phase B(START_AT_DOCK=1): 로봇당 localizer 1개가 전방(XN)+후방(도크) 두
    # 카메라를 구독해 한 필터를 공유(T3b). rear_image_topic 은 T2 --bridge-rear
    # 발행 토픽. rear_t_base_cam 은 기본값(전방 Ry180 유도, T3b) — T5 에서 GT 검증.
    # 노드는 로봇 네임스페이스로 띄워 FQN 을 /robot_<id>/marker_localizer_node 로
    # 구분(오케스트레이터 크로스노드 set_parameters 대상, T3 기본 유도값과 일치).
    # seed_pose: 도크 스폰 GT 자세로 필터를 미리 시딩(첫 마커 fix 전에도 odom
    # 예측으로 융합자세가 나와 dock_check 같은 융합주행이 시작 가능 — 러너
    # _mission_setup 의 도크 직접시딩 등가물). yaw=90=+X(도크 스폰 방향).
    _rear_lead=()
    _rear_follow=()
    if [ "$START_AT_DOCK" = "1" ]; then
        _rear_lead=(-p rear_image_topic:=/robot_entry_lead/rear/image_raw \
                    -p rear_camera_info_topic:=/robot_entry_lead/rear/camera_info \
                    -p "seed_pose:=[-3.2,2.2,90.0]")
        _rear_follow=(-p rear_image_topic:=/robot_entry_follow/rear/image_raw \
                      -p rear_camera_info_topic:=/robot_entry_follow/rear/camera_info \
                      -p "seed_pose:=[-1.2,2.2,90.0]")
    fi

    echo "[2/13] marker_localizer_node entry_lead"
    _launch "marker_localizer_entry_lead" "$LOG_DIR/marker_localizer_entry_lead.log" \
        python3 -m parkbot_aruco.marker_localizer_node --ros-args \
        -r __ns:=/robot_entry_lead \
        -p image_topic:=/robot_entry_lead/front/image_raw \
        -p camera_info_topic:=/robot_entry_lead/front/camera_info \
        -p odom_topic:=/robot_entry_lead/odom \
        -p pose_topic:=/robot_entry_lead/pose \
        "${_rear_lead[@]}" \
        -p marker_map:="$MARKER_MAP" -p fuse:=true -p frame:=usd

    echo "[3/13] marker_localizer_node entry_follow"
    _launch "marker_localizer_entry_follow" "$LOG_DIR/marker_localizer_entry_follow.log" \
        python3 -m parkbot_aruco.marker_localizer_node --ros-args \
        -r __ns:=/robot_entry_follow \
        -p image_topic:=/robot_entry_follow/front/image_raw \
        -p camera_info_topic:=/robot_entry_follow/front/camera_info \
        -p odom_topic:=/robot_entry_follow/odom \
        -p pose_topic:=/robot_entry_follow/pose \
        "${_rear_follow[@]}" \
        -p marker_map:="$MARKER_MAP" -p fuse:=true -p frame:=usd

    echo "[4/13] pose_controller_node entry_lead (odom, approach)"
    _launch "pose_controller_odom_entry_lead" "$LOG_DIR/pose_controller_odom_entry_lead.log" \
        bash "$SCRIPT_DIR/run_pose_controller_node.sh" --ros-args \
        -p robot_id:=entry_lead -p goal_timeout_sec:="$GOAL_TIMEOUT"

    echo "[5/13] pose_controller_node entry_lead (fused, align)"
    _launch "pose_controller_fused_entry_lead" "$LOG_DIR/pose_controller_fused_entry_lead.log" \
        bash "$SCRIPT_DIR/run_pose_controller_node.sh" --ros-args \
        -p robot_id:=entry_lead -p pose_topic:=/robot_entry_lead/pose \
        -p pose_msg_type:=posestamped -p action_name:=/robot_entry_lead/navigate_to_pose_fused \
        -p pos_tol:=0.06 -p pose_stale_timeout_sec:=15.0 -p goal_timeout_sec:="$GOAL_TIMEOUT"

    echo "[6/13] pose_controller_node entry_follow (odom, approach)"
    _launch "pose_controller_odom_entry_follow" "$LOG_DIR/pose_controller_odom_entry_follow.log" \
        bash "$SCRIPT_DIR/run_pose_controller_node.sh" --ros-args \
        -p robot_id:=entry_follow -p goal_timeout_sec:="$GOAL_TIMEOUT"

    echo "[7/13] pose_controller_node entry_follow (fused, align)"
    _launch "pose_controller_fused_entry_follow" "$LOG_DIR/pose_controller_fused_entry_follow.log" \
        bash "$SCRIPT_DIR/run_pose_controller_node.sh" --ros-args \
        -p robot_id:=entry_follow -p pose_topic:=/robot_entry_follow/pose \
        -p pose_msg_type:=posestamped -p action_name:=/robot_entry_follow/navigate_to_pose_fused \
        -p pos_tol:=0.06 -p pose_stale_timeout_sec:=15.0 -p goal_timeout_sec:="$GOAL_TIMEOUT"

    # Phase B(START_AT_DOCK=1): 픽업 자세원을 마커보정된 융합 /pose 로 준다 -- 긴
    # Phase B 경로(도크→XN ~12m+회전) 뒤 raw 오도 드리프트(~1m 실측)를 피한다.
    # axle_detector 와 ingress 가 **같은 융합 프레임**을 쓰면 뎁스 검출축 기준
    # 상대정지라 절대 드리프트가 상쇄된다(axle_detector docstring §"주행좌표"가
    # 도크 인근 구간에서 /pose 로 바꿔 쓰는 이 구성을 명시). START_AT_DOCK=0 이면
    # 기존 오도(하위호환, R5b 베이근처 시작).
    _fused_pose_lead=()
    _fused_pose_follow=()
    if [ "$START_AT_DOCK" = "1" ]; then
        _fused_pose_lead=(-p pose_topic:=/robot_entry_lead/pose -p pose_msg_type:=posestamped)
        _fused_pose_follow=(-p pose_topic:=/robot_entry_follow/pose -p pose_msg_type:=posestamped)
    fi

    echo "[8/13] axle_detector_node entry_lead / entry_follow"
    _launch "axle_detector_entry_lead" "$LOG_DIR/axle_detector_entry_lead.log" \
        bash "$SCRIPT_DIR/run_axle_detector_node.sh" --ros-args -p robot_id:=entry_lead \
        "${_fused_pose_lead[@]}"
    _launch "axle_detector_entry_follow" "$LOG_DIR/axle_detector_entry_follow.log" \
        bash "$SCRIPT_DIR/run_axle_detector_node.sh" --ros-args -p robot_id:=entry_follow \
        "${_fused_pose_follow[@]}"

    echo "[9/13] ingress_node entry_lead / entry_follow"
    _launch "ingress_entry_lead" "$LOG_DIR/ingress_entry_lead.log" \
        bash "$SCRIPT_DIR/run_ingress_node.sh" --ros-args \
        -p robot_id:=entry_lead -p goal_timeout_sec:="$GOAL_TIMEOUT" \
        "${_fused_pose_lead[@]}"
    _launch "ingress_entry_follow" "$LOG_DIR/ingress_entry_follow.log" \
        bash "$SCRIPT_DIR/run_ingress_node.sh" --ros-args \
        -p robot_id:=entry_follow -p goal_timeout_sec:="$GOAL_TIMEOUT" \
        "${_fused_pose_follow[@]}"

    echo "[10/13] lift_action_server entry_lead / entry_follow"
    _launch "lift_entry_lead" "$LOG_DIR/lift_entry_lead.log" \
        bash "$SCRIPT_DIR/run_lift_action_server.sh" --ros-args \
        -p robot_id:=entry_lead -p ramp_wait_sec:="$RAMP_WAIT"
    _launch "lift_entry_follow" "$LOG_DIR/lift_entry_follow.log" \
        bash "$SCRIPT_DIR/run_lift_action_server.sh" --ros-args \
        -p robot_id:=entry_follow -p ramp_wait_sec:="$RAMP_WAIT"

    echo "[11/13] pickup_orchestrator_node"
    # Phase B(START_AT_DOCK=1): run_phase_b_first(오케스트레이터 기본 True)로 도크→XN
    # 레그를 corridor_plan 앞에 실행. 크로스노드 set_parameters 대상 localizer FQN 을
    # 명시(위 -r __ns 로 띄운 노드명과 일치). START_AT_DOCK=0 이면 Phase B 를 끈다.
    _orch_args=()
    if [ "$START_AT_DOCK" = "1" ]; then
        # approach_use_fused: Phase B 뒤 raw 오도 드리프트를 피해 approach 를 마커보정
        # 융합 자세로 시작(axle_detector/ingress 도 위에서 융합 프레임 -> 상대정지).
        _orch_args=(--ros-args \
            -p approach_use_fused:=true \
            -p phase_b_leader_localizer_node:=/robot_entry_lead/marker_localizer_node \
            -p phase_b_follower_localizer_node:=/robot_entry_follow/marker_localizer_node)
    else
        _orch_args=(--ros-args -p run_phase_b_first:=false)
    fi
    _launch "pickup_orchestrator" "$LOG_DIR/pickup_orchestrator.log" \
        bash "$SCRIPT_DIR/run_pickup_orchestrator_node.sh" "${_orch_args[@]}"

    echo "[12/13] discovery 안정화 대기(10s)"
    sleep 10

    echo "[13/13] 기동 완료. PID 목록:"
    cat "$PID_FILE"
    echo "로그: $LOG_DIR"
}

cmd_down() {
    echo "=== bringup_pickup_e2e down ==="
    # taskR5a-report.md §7 교훈: 러너는 exec 로 완전 치환되지 않는다(python.sh
    # 가 실제 kit 프로세스를 자식으로 fork) -- pgrep -f 로 래퍼+자식 전부 잡는다.
    local patterns=(
        "parking_v4_runner.py"
        "parkbot_aruco.marker_localizer_node"
        "parkbot_motion.pose_controller_node"
        "parkbot_motion.axle_detector_node"
        "parkbot_motion.ingress_node"
        "parkbot_motion.lift_action_server"
        "parkbot_motion.pickup_orchestrator_node"
        "pickup_smoke.py"
    )
    local pids=()
    for pat in "${patterns[@]}"; do
        for pid in $(pgrep -f "$pat" 2>/dev/null); do
            pids+=("$pid")
        done
    done
    if [ "${#pids[@]}" -eq 0 ]; then
        echo "정리할 프로세스 없음"
        return 0
    fi
    echo "SIGINT: ${pids[*]}"
    kill -INT "${pids[@]}" 2>/dev/null
    local waited=0
    while [ "$waited" -lt 15 ]; do
        local alive=0
        for pid in "${pids[@]}"; do
            kill -0 "$pid" 2>/dev/null && alive=1
        done
        [ "$alive" -eq 0 ] && break
        sleep 1
        waited=$((waited + 1))
    done
    local still_alive=()
    for pid in "${pids[@]}"; do
        kill -0 "$pid" 2>/dev/null && still_alive+=("$pid")
    done
    if [ "${#still_alive[@]}" -gt 0 ]; then
        echo "SIGKILL(SIGINT 무시): ${still_alive[*]}"
        kill -9 "${still_alive[@]}" 2>/dev/null
    fi
    sleep 1
    echo "좀비 확인:"
    ps aux | awk '$8 ~ /Z/'
    echo "down 완료"
}

cmd_status() {
    echo "=== 관련 프로세스 ==="
    pgrep -fal "parking_v4_runner.py|parkbot_aruco.marker_localizer_node|parkbot_motion\.(pose_controller_node|axle_detector_node|ingress_node|lift_action_server|pickup_orchestrator_node)|pickup_smoke.py" 2>/dev/null \
        || echo "(없음)"
}

case "${1:-}" in
    up) cmd_up ;;
    down) cmd_down ;;
    status) cmd_status ;;
    *) echo "usage: $0 {up|down|status}" >&2; exit 1 ;;
esac
