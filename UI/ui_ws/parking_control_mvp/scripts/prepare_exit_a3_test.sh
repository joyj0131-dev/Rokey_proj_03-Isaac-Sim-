#!/usr/bin/env bash
# feat/exit Phase X 반복 시연용 A3 출차 차량 fixture.
#
# 사용:
#   ./parking_control_mvp/scripts/prepare_exit_a3_test.sh
#   ./parking_control_mvp/scripts/prepare_exit_a3_test.sh 3333
#
# 현재 Phase X 로봇 안무는 A3 좌표에 맞춰져 있다. 이 스크립트는 지정 차량의
# 마지막 완료 작업을 A3 ENTRY로 만들어 웹 UI의 출차 사전 검증과 dispatcher의
# find_vehicle_slot()이 같은 결과를 보도록 한다. 실행 중인 작업이 하나라도 있으면
# DB 이력과 실제 로봇 동작이 어긋날 수 있으므로 변경하지 않고 종료한다.

set -euo pipefail

EXIT_TEST_VEHICLE_ID="${1:-3333}"
PARKING_DB_HOST="${PARKING_DB_HOST:-localhost}"
PARKING_DB_USER="${PARKING_DB_USER:-parking}"
PARKING_DB_PASSWORD="${PARKING_DB_PASSWORD:-parking1234}"
PARKING_DB_NAME="${PARKING_DB_NAME:-parking}"

if ((${#EXIT_TEST_VEHICLE_ID} < 1 || ${#EXIT_TEST_VEHICLE_ID} > 32)) \
  || [[ ! "$EXIT_TEST_VEHICLE_ID" =~ ^[[:alnum:]-]+$ ]]; then
  echo "오류: 차량번호는 숫자·영문·한글·하이픈 1~32자만 사용할 수 있습니다." >&2
  exit 2
fi

export MYSQL_PWD="$PARKING_DB_PASSWORD"
MYSQL=(
  mysql
  --host="$PARKING_DB_HOST"
  --user="$PARKING_DB_USER"
  --database="$PARKING_DB_NAME"
  --batch
  --skip-column-names
)

ACTIVE_COUNT="$("${MYSQL[@]}" --execute="
  SELECT COUNT(*)
  FROM tasks
  WHERE state IN ('WAITING', 'PROCESSING');
")"

if [[ "$ACTIVE_COUNT" != "0" ]]; then
  echo "오류: 진행 중인 작업 ${ACTIVE_COUNT}건이 있어 A3 테스트 차량을 준비하지 않았습니다." >&2
  echo "작업을 완료·취소하고 로봇이 정지한 뒤 다시 실행하세요." >&2
  exit 3
fi

"${MYSQL[@]}" --execute="
  START TRANSACTION;

  DELETE zl
  FROM zone_locks AS zl
  INNER JOIN tasks AS t ON t.task_id = zl.task_id
  WHERE t.vehicle_id = '${EXIT_TEST_VEHICLE_ID}';

  DELETE FROM tasks
  WHERE vehicle_id = '${EXIT_TEST_VEHICLE_ID}'
     OR task_id = 'exit-a3-test-fixture';

  INSERT INTO vehicles (vehicle_id)
  VALUES ('${EXIT_TEST_VEHICLE_ID}')
  ON DUPLICATE KEY UPDATE vehicle_id = VALUES(vehicle_id);

  UPDATE parking_slots
  SET status = CASE WHEN slot_id = 'A3' THEN 'OCCUPIED' ELSE status END
  WHERE slot_id IN ('A1', 'A2', 'A3');

  INSERT INTO tasks (
    task_id, request_type, state, vehicle_id,
    robot_id, follower_robot_id, slot_id
  ) VALUES (
    'exit-a3-test-fixture', 'ENTRY', 'DONE', '${EXIT_TEST_VEHICLE_ID}',
    NULL, NULL, 'A3'
  );

  UPDATE robots
  SET status = 'IDLE', target_node = NULL
  WHERE robot_id IN ('exit_lead', 'exit_follow');

  COMMIT;
"

echo "A3 출차 테스트 준비 완료"
echo "  차량: ${EXIT_TEST_VEHICLE_ID}"
echo "  슬롯: A3 · OCCUPIED"
echo "  웹 UI: 출차 선택 → 차량번호 ${EXIT_TEST_VEHICLE_ID} → 출차 요청 등록"
echo
echo "참고: 현재 feat/exit Phase X는 A3 전용입니다. 실제 운영용 임의 슬롯 출차는 아직 지원하지 않습니다."
