#!/usr/bin/env bash
# 관제 노드 4개(입·출차 테스트용 가짜 로봇 포함) + 주차장 도면 대시보드 + 웹 UI를
# 한 번에 띄우는 스크립트. 더미 데이터로 입고/출차 전체 흐름을 눈으로
# 확인하기 위한 테스트 전용 도구다 (실제 로봇 동작은 아직 B/C 미구현).
# 안전 복구 운영 절차: docs/safety-recovery-runbook.md
#
# 실행:  bash UI/run_test_stack.sh   (어느 경로에서 실행해도 된다)
# 종료:  Ctrl+C  (노드/서버 전부 정리. DB 데이터는 남겨둔다)
#
# (set -u는 ROS의 setup.bash와 충돌하므로 쓰지 않는다)
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
# 관제 colcon 워크스페이스는 이 스크립트가 있는 UI/ 가 아니라 그 아래 ui_ws/ 다.
# (src/parking_control, install/, parking_control_mvp 전부 ui_ws 기준)
WS="$SCRIPT_DIR/ui_ws"
DB="mysql -u parking -pparking1234 parking"
LOG_DIR="$(mktemp -d /tmp/parking_test_stack.XXXX)"
CLEANED_UP=0

cleanup() {
    if [ "$CLEANED_UP" -eq 1 ]; then
        return
    fi
    CLEANED_UP=1
    echo
    echo "[정리] 모든 프로세스 종료 중..."
    pkill -f "install/[p]arking" 2>/dev/null
    pkill -f "bin/[r]os2 run" 2>/dev/null
    pkill -f "scripts/[d]ashboard.py" 2>/dev/null
    pkill -f "[u]vicorn main:app" 2>/dev/null
    sleep 0.2
    # 백그라운드 대시보드의 rclpy 스핀 스레드가 종료 신호를 붙잡는 경우가
    # 있어, 이 테스트 스택의 정확한 프로세스만 마지막에 강제 정리한다.
    pkill -KILL -f "src/parking_control/scripts/[d]ashboard.py" 2>/dev/null
    echo "[정리] 완료. 로그는 $LOG_DIR 에 남아있습니다. (DB 데이터는 유지됨)"
}

# 비정상 종료된 예전 테스트 스택의 보조 도면 서버만 정리한다. 8000 포트는
# 다른 웹 앱일 수 있으므로 임의 종료하지 않고, 점유 중이면 원인을 알려주고 멈춘다.
pkill -KILL -f "src/parking_control/scripts/[d]ashboard.py" 2>/dev/null
sleep 0.2
for port in 8000 8080; do
    if ss -H -ltn "sport = :$port" | grep -q .; then
        echo "[오류] $port 포트가 이미 사용 중입니다."
        echo "       기존 서버를 종료한 뒤 이 스크립트를 다시 실행해주세요."
        exit 1
    fi
done
trap cleanup EXIT INT TERM

# 빌드 산출물이 없으면 ros2 run 이 전부 조용히 실패하므로 미리 잡아준다.
if [ ! -f "$WS/install/setup.bash" ]; then
    echo "[오류] $WS/install/setup.bash 가 없습니다."
    echo "       먼저 빌드하세요:  cd $WS && colcon build --symlink-install"
    exit 1
fi

echo "=== 1/4: DB를 깨끗한 상태로 리셋 ==="
echo "    (V4 스키마/슬롯/로봇 4대 동기화, 이전 테스트 작업 이력 삭제)"

# 예전 스키마에는 robots.target_node가 없다. sim_orchestrator는 이동 경로를
# 표시하기 위해 첫 waypoint부터 이 컬럼을 갱신하므로, 누락된 상태에서는 모든
# 작업이 APPROACHING 직후 Unknown column 오류로 취소된다.
if ! $DB -Nse \
    "SELECT 1 FROM information_schema.COLUMNS
     WHERE TABLE_SCHEMA='parking' AND TABLE_NAME='robots'
       AND COLUMN_NAME='target_node'" | grep -q 1; then
    echo "    - 로봇 이동 목적지 컬럼(003) 적용"
    $DB < "$WS/src/parking_control/db/003_add_robot_target.sql"
fi

# 예전 단일 로봇 DB로 실행한 뒤 feature/parking-control을 병합한 경우,
# tasks.follower_robot_id와 zone_locks.task_id가 없다. 이 상태에서는 로봇쌍을
# 등록하더라도 첫 요청에서 SQL 오류가 나므로 테스트 시작 전에 한 번만 마이그레이션한다.
if ! $DB -Nse \
    "SELECT 1 FROM information_schema.COLUMNS
     WHERE TABLE_SCHEMA='parking' AND TABLE_NAME='tasks'
       AND COLUMN_NAME='follower_robot_id'" | grep -q 1; then
    echo "    - 2대 로봇 작업 스키마(004) 적용"
    $DB < "$WS/src/parking_control/db/004_dual_robot_zone_owner.sql"
fi

# 중앙 비상정지 상태/관제 승인 감사 테이블(멱등).
$DB < "$WS/src/parking_control/db/005_safety_supervisor.sql"

# 현재 parking_map.yaml(V4)의 슬롯/존 좌표를 DB의 단일 기준으로 맞춘다.
$DB < "$WS/src/parking_control/db/002_seed.sql"
$DB -e "
DELETE FROM zone_locks;
DELETE FROM tasks;
DELETE FROM vehicles;
DELETE FROM parking_slots WHERE slot_id NOT IN ('A1', 'A2', 'A3');
UPDATE parking_slots SET status='EMPTY';
INSERT INTO robots
    (robot_id, status, x, y, battery_percent, target_node)
VALUES
    ('entry_lead',   'IDLE', -3.2, -2.2, 100, NULL),
    ('entry_follow', 'IDLE', -1.2, -2.2, 100, NULL),
    ('exit_lead',    'IDLE', -3.2,  2.2, 100, NULL),
    ('exit_follow',  'IDLE', -1.2,  2.2, 100, NULL)
ON DUPLICATE KEY UPDATE
    status=VALUES(status),
    x=VALUES(x),
    y=VALUES(y),
    battery_percent=VALUES(battery_percent),
    target_node=NULL;" 2>/dev/null

cd "$WS"
source /opt/ros/humble/setup.bash
source install/setup.bash

echo "=== 2/4: 중앙 안전 관리자 + 관제 노드 실행 (입·출차 가짜 로봇 포함) ==="
# task_dispatcher가 사용하는 액션 이름은 /entry/execute_parking_task와
# /exit/execute_parking_task다. 각 시뮬레이터를 같은 네임스페이스로 띄우고,
# 진행 상태는 웹 UI가 구독하는 공용 /task_state로 모은다.
ros2 run parking_control safety_supervisor \
    > "$LOG_DIR/safety_supervisor.log" 2>&1 &
sleep 1
ros2 run parking_control sim_orchestrator --ros-args \
    -r __ns:=/entry -r __node:=entry_sim_orchestrator \
    -r task_state:=/task_state -p robot_id:=entry_lead \
    > "$LOG_DIR/entry_orch.log" 2>&1 &
ros2 run parking_control sim_orchestrator --ros-args \
    -r __ns:=/exit -r __node:=exit_sim_orchestrator \
    -r task_state:=/task_state -p robot_id:=exit_lead \
    > "$LOG_DIR/exit_orch.log" 2>&1 &
ros2 run parking_control parking_slot_manager \
    --ros-args -p allow_accessible_slots:=true \
    > "$LOG_DIR/slot.log" 2>&1 &
ros2 run parking_control task_dispatcher > "$LOG_DIR/disp.log" 2>&1 &
sleep 3

echo "=== 3/4: 주차장 도면 대시보드 실행 ==="
python3 src/parking_control/scripts/dashboard.py > "$LOG_DIR/dash.log" 2>&1 &
sleep 1

echo "=== 4/4: 웹 UI 실행 (ros2 모드) ==="
cd parking_control_mvp
PARKING_MODE=ros2 python3 -m uvicorn main:app --host 127.0.0.1 --port 8000 \
    > "$LOG_DIR/ui.log" 2>&1 &
sleep 2

cat <<'BANNER'

=====================================================================
  준비 완료! 브라우저 탭 2개를 여세요.

  1) 관제 웹 UI — 입고/출차 요청, 작업 목록, 알림
     http://127.0.0.1:8000

  2) 주차장 실시간 도면 — 슬롯 상태 + 로봇이 움직이는 모습
     http://localhost:8080

  테스트 방법
  -----------
  - UI에서 차량번호를 입력해 "입차 요청" 등록
      → 대시보드에서 입차 리더 로봇이 입구까지 갔다가 배정된 칸으로 이동하는
        모습이 실시간으로 보입니다. 도착하면 그 칸이 채워집니다.
  - 같은 차량번호로 "출차 요청" 등록
      → 출차 리더 로봇이 그 칸으로 가서 차를 꺼내 나가고, 완료되면 칸이 다시
        비워집니다.
  - 주차 기록이 없는 차량번호로 출차를 시도하면 정상적으로 거부됩니다.
  - 현재 V4 도면의 주차면은 A1~A3 총 3칸입니다.
  - 입차/출차는 각 전용 로봇쌍이 담당하므로 서로 동시에 요청할 수 있습니다.
    시뮬레이터는 각 쌍의 리더와 팔로워 좌표를 편대 간격으로 함께 움직입니다.
  - 비상정지 후에는 "현장 점검·해제 요청"과 "운영 복귀 승인"을 순서대로
    완료해야 하며, 중단된 작업은 자동 재개되지 않습니다.
  - 테스트 스택을 종료했다가 다시 실행해도 비상정지 상태는 DB에 유지됩니다.

  Ctrl+C 를 누르면 전부 종료됩니다.
=====================================================================
BANNER

wait
