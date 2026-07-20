#!/usr/bin/env bash
# 관제 시스템 데모: 노드 3개를 띄우고 시나리오 4개를 순서대로 실행한다.
# 사용법:  bash src/parking_control/scripts/demo_scenarios.sh   (cobot3_ws에서)
# 종료 시 노드와 DB 변경(슬롯 점유)을 원상복구한다.
# (set -u는 ROS setup.bash와 충돌하므로 쓰지 않는다)
WS="$(cd "$(dirname "$0")/../../.." && pwd)"
DB="mysql -u parking -pparking1234 parking"
LOG_DIR="$(mktemp -d /tmp/parking_demo.XXXX)"

cleanup() {
    echo
    echo "[정리] 노드 종료 및 DB 원상복구..."
    pkill -f "install/[p]arking" 2>/dev/null
    pkill -f "bin/[r]os2 run" 2>/dev/null
    $DB -e "UPDATE parking_slots SET status='EMPTY';
            UPDATE robots SET status='IDLE' WHERE robot_id='robot_1';
            DELETE FROM zone_locks;" 2>/dev/null
    echo "[정리] 완료. 로그: $LOG_DIR"
}
trap cleanup EXIT

req() {  # req <type> <vehicle_id>
    ros2 service call /dispatch_parking_task \
        parking_robot_interfaces/srv/RequestParkingTask \
        "{request_type: $1, vehicle_id: $2}" 2>/dev/null \
        | grep -o "accepted=[^,]*\|task_id='[^']*'\|message='[^']*'" | tr '\n' ' '
    echo
}

find_slot() {  # find_slot <length> <width>
    ros2 service call /find_empty_slot \
        parking_robot_interfaces/srv/FindEmptySlot \
        "{vehicle_length: $1, vehicle_width: $2}" 2>/dev/null \
        | grep -o "success=[^,]*\|slot_id='[^']*'" | tr '\n' ' '
    echo
}

cd "$WS"
source /opt/ros/humble/setup.bash
source install/setup.bash

echo "=== 준비: DB 초기화 + 노드 3개 실행 ==="
$DB -e "UPDATE parking_slots SET status='EMPTY';
        UPDATE robots SET status='IDLE', x=-15.3, y=-7.8 WHERE robot_id='robot_1';
        DELETE FROM zone_locks;" 2>/dev/null
ros2 run parking_robot_system robot_task_orchestrator > "$LOG_DIR/orch.log" 2>&1 &
ros2 run parking_control parking_slot_manager          > "$LOG_DIR/slot.log" 2>&1 &
ros2 run parking_control task_dispatcher               > "$LOG_DIR/disp.log" 2>&1 &
sleep 4
echo "노드 준비 완료"
echo

echo "=== 시나리오 1: 승용차 입고 요청 (정상 흐름) ==="
echo " 기대: accepted=True, robot_1 배정, 슬롯 B1"
req ENTRY CAR_DEMO1
sleep 2

echo
echo "=== 시나리오 2: 슬롯보다 큰 차량 (거부 확인) ==="
echo " 기대: success=False (슬롯 6.6m보다 긴 7.5m 차량)"
find_slot 7.5 2.3

echo
echo "=== 시나리오 3: B행이 다 찼을 때 (다음 후보 확인) ==="
echo " 기대: B1이 아닌 다른 슬롯 (B행 전체 OCCUPIED 처리 후)"
$DB -e "UPDATE parking_slots SET status='OCCUPIED' WHERE slot_id LIKE 'B%';" 2>/dev/null
find_slot 4.5 1.9
$DB -e "UPDATE parking_slots SET status='EMPTY';" 2>/dev/null

echo
echo "=== 시나리오 4: 존 락 경합 (다른 로봇이 이미 잡은 존) ==="
echo " 기대: robot_1이 Z05 획득 → robot_2는 거부(granted=False)"
$DB -e "INSERT IGNORE INTO robots (robot_id,status) VALUES ('robot_2','IDLE');" 2>/dev/null
# db 모드 dispatcher를 따로 띄워 실제 락으로 확인
pkill -f "install/.*[t]ask_dispatcher" 2>/dev/null; sleep 1
ros2 run parking_control task_dispatcher --ros-args -p zone_lock_mode:=db \
    > "$LOG_DIR/disp_db.log" 2>&1 &
sleep 3
for r in robot_1 robot_2; do
    echo -n " $r → "
    ros2 service call /acquire_zones parking_robot_interfaces/srv/AcquireZones \
        "{robot_id: $r, zone_ids: [Z05]}" 2>/dev/null \
        | grep -o "granted=[^,)]*"
done
$DB -e "SELECT zone_id, robot_id AS holder FROM zone_locks;" 2>/dev/null

echo
echo "=== 최종: DB에 기록된 작업 이력 ==="
$DB -e "SELECT LEFT(task_id,8) AS task, request_type, state, vehicle_id,
               robot_id, slot_id, created_at
        FROM tasks ORDER BY created_at DESC LIMIT 5;" 2>/dev/null
