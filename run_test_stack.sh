#!/usr/bin/env bash
# 관제 노드 3개(테스트용 가짜 로봇 포함) + 주차장 도면 대시보드 + 웹 UI를
# 한 번에 띄우는 스크립트. 더미 데이터로 입고/출차 전체 흐름을 눈으로
# 확인하기 위한 테스트 전용 도구다 (실제 로봇 동작은 아직 B/C 미구현).
#
# 실행:  bash run_test_stack.sh     (cobot3_ws 루트에서)
# 종료:  Ctrl+C  (노드/서버 전부 정리. DB 데이터는 남겨둔다)
#
# (set -u는 ROS의 setup.bash와 충돌하므로 쓰지 않는다)
WS="$(cd "$(dirname "$0")" && pwd)"
DB="mysql -u parking -pparking1234 parking"
LOG_DIR="$(mktemp -d /tmp/parking_test_stack.XXXX)"

cleanup() {
    echo
    echo "[정리] 모든 프로세스 종료 중..."
    pkill -f "install/[p]arking" 2>/dev/null
    pkill -f "bin/[r]os2 run" 2>/dev/null
    pkill -f "scripts/[d]ashboard.py" 2>/dev/null
    pkill -f "[u]vicorn main:app" 2>/dev/null
    echo "[정리] 완료. 로그는 $LOG_DIR 에 남아있습니다. (DB 데이터는 유지됨)"
}
trap cleanup EXIT INT TERM

echo "=== 1/4: DB를 깨끗한 상태로 리셋 ==="
echo "    (슬롯 전부 EMPTY, robot_1을 대기 위치로, 이전 작업 이력 삭제)"
$DB -e "
UPDATE parking_slots SET status='EMPTY';
UPDATE robots SET status='IDLE', x=-15.3, y=-7.8 WHERE robot_id='robot_1';
DELETE FROM zone_locks;
DELETE FROM tasks;
DELETE FROM vehicles;" 2>/dev/null

cd "$WS"
source /opt/ros/humble/setup.bash
source install/setup.bash

echo "=== 2/4: 관제 노드 3개 실행 (가짜 로봇 sim_orchestrator 포함) ==="
ros2 run parking_control sim_orchestrator > "$LOG_DIR/orch.log" 2>&1 &
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
  - UI에서 차량번호를 1~18 중 하나로 "입고(PARK_IN)" 요청
      → 대시보드에서 로봇이 입구까지 갔다가 배정된 칸으로 이동하는
        모습이 실시간으로 보입니다. 도착하면 그 칸이 채워집니다.
  - 같은 차량번호로 "출차(PARK_OUT)" 요청
      → 로봇이 그 칸으로 가서 차를 꺼내 나가고, 완료되면 칸이 다시
        비워집니다.
  - 주차 기록이 없는 차량번호로 출차를 시도하면 정상적으로 거부됩니다.
  - 일반 칸 14개(A3~A8, B1~B8)가 다 차면, 다음 입고 요청은 자동으로
    교통약자 칸(A1/A2)에 배정됩니다 — 장애인 주차 테스트용입니다.
  - 로봇이 1대뿐이라 요청은 한 번에 하나씩 순서대로 처리되고, 슬롯 위치에
    따라 왕복 10~25초 정도 걸립니다 (실제 거리만큼 등속으로 이동).

  Ctrl+C 를 누르면 전부 종료됩니다.
=====================================================================
BANNER

wait
