"""MySQL 접근 계층. 순수 Python (ROS import 금지).

노드가 직접 SQL을 들고 있지 않도록 쿼리를 여기에 모은다.
커넥션이 끊겨도 다음 호출에서 자동 재접속한다.
"""

import json

import mysql.connector

DUPLICATE_KEY_ERRNO = 1062  # zone lock 획득 실패 판정에 사용


def _zone_owner(robot_id, task_id):
    """zone_locks 소유자 검증. robot_id/task_id 중 정확히 하나만 허용."""
    if (robot_id is None) == (task_id is None):
        raise ValueError("robot_id와 task_id 중 정확히 하나만 지정해야 합니다")
    return robot_id, task_id


class ParkingDB:

    def __init__(self, host="localhost", user="parking",
                 password="parking1234", database="parking"):
        self._config = dict(host=host, user=user, password=password,
                            database=database, autocommit=True)
        self._conn = None

    def _connection(self):
        if self._conn is None or not self._conn.is_connected():
            self._conn = mysql.connector.connect(**self._config)
        return self._conn

    def _query(self, sql, params=()):
        cursor = self._connection().cursor(dictionary=True)
        try:
            cursor.execute(sql, params)
            if cursor.with_rows:
                return cursor.fetchall()
            return []
        finally:
            cursor.close()

    def close(self):
        if self._conn is not None and self._conn.is_connected():
            self._conn.close()
        self._conn = None

    # ---- parking_slots ----

    def find_empty_slots(self, include_accessible=False):
        """비어 있고, 진행 중인 task에 예약되지 않은 슬롯 목록."""
        sql = """
            SELECT s.slot_id, s.x, s.y, s.is_accessible
            FROM parking_slots s
            LEFT JOIN tasks t ON t.slot_id = s.slot_id
                 AND t.state IN ('WAITING', 'PROCESSING')
            WHERE s.status = 'EMPTY' AND t.task_id IS NULL
        """
        if not include_accessible:
            sql += " AND s.is_accessible = FALSE"
        return self._query(sql)

    def set_slot_status(self, slot_id, status):
        self._query("UPDATE parking_slots SET status = %s WHERE slot_id = %s",
                    (status, slot_id))

    def find_vehicle_slot(self, vehicle_id):
        """차량이 현재 주차 중인 슬롯 id. 마지막 완료 작업이 ENTRY일 때만
        유효(그 이후 EXIT가 없었다는 뜻) — 아니면 None."""
        rows = self._query(
            "SELECT request_type, slot_id FROM tasks"
            " WHERE vehicle_id = %s AND state = 'DONE' AND slot_id IS NOT NULL"
            " ORDER BY created_at DESC LIMIT 1",
            (vehicle_id,))
        if rows and rows[0]["request_type"] == "ENTRY":
            return rows[0]["slot_id"]
        return None

    # ---- robots / vehicles / tasks ----

    def idle_robots(self):
        return self._query(
            "SELECT robot_id, x, y FROM robots WHERE status = 'IDLE'")

    def robot_statuses(self, robot_ids):
        """요청한 로봇 ID의 현재 상태를 반환한다.

        반환값에 없는 ID는 DB에 등록되지 않은 로봇이다. 디스패처가 이를
        BUSY와 구분해 운영자에게 정확한 초기화 오류를 알릴 때 사용한다.
        """
        robot_ids = tuple(robot_ids)
        if not robot_ids:
            return {}
        placeholders = ", ".join(["%s"] * len(robot_ids))
        rows = self._query(
            f"SELECT robot_id, status FROM robots"
            f" WHERE robot_id IN ({placeholders})",
            robot_ids,
        )
        return {row["robot_id"]: row["status"] for row in rows}

    def all_robot_positions(self):
        """상태와 무관하게 좌표가 있는 모든 로봇 위치. 장애물 감지에서
        로봇 자신을 장애물로 오인하지 않도록 제외할 때 쓴다 — BUSY 로봇도
        물리적으로 존재하므로 idle_robots()로는 부족하다."""
        rows = self._query(
            "SELECT x, y FROM robots WHERE x IS NOT NULL AND y IS NOT NULL")
        return [(float(r["x"]), float(r["y"])) for r in rows]

    def upsert_robot(self, robot_id):
        """robots 행이 없으면 만든다(기본 상태 OFFLINE). 이미 있으면 그대로 둔다 —
        robot_position_bridge_node가 시작할 때마다 불러도 기존 status/x/y를
        덮어쓰지 않는다(upsert_vehicle과 같은 패턴)."""
        self._query(
            "INSERT INTO robots (robot_id) VALUES (%s)"
            " ON DUPLICATE KEY UPDATE robot_id = robot_id",
            (robot_id,))

    def set_robot_status(self, robot_id, status):
        self._query("UPDATE robots SET status = %s WHERE robot_id = %s",
                    (status, robot_id))

    def update_robot_position(self, robot_id, x, y):
        self._query("UPDATE robots SET x = %s, y = %s WHERE robot_id = %s",
                    (x, y, robot_id))

    def update_robot_positions(self, positions):
        """여러 로봇 좌표를 한 SQL 문으로 함께 갱신한다.

        협업 운반 화면은 리더/팔로워 좌표를 같은 프레임으로 읽어야 한다.
        로봇별 UPDATE를 따로 실행하면 DB 폴링이 두 문장 사이를 읽어 한 대만
        움직인 프레임을 만들 수 있으므로, 다중 VALUES upsert를 사용한다.
        기존 로봇의 status/battery 등 다른 필드는 건드리지 않는다.
        """
        positions = list(positions)
        if not positions:
            return
        placeholders = ", ".join(["(%s, %s, %s)"] * len(positions))
        params = tuple(
            value
            for robot_id, x, y in positions
            for value in (robot_id, x, y)
        )
        self._query(
            "INSERT INTO robots (robot_id, x, y)"
            f" VALUES {placeholders}"
            " ON DUPLICATE KEY UPDATE x = VALUES(x), y = VALUES(y)",
            params,
        )

    def get_robot_position(self, robot_id):
        rows = self._query(
            "SELECT x, y FROM robots WHERE robot_id = %s", (robot_id,))
        if rows and rows[0]["x"] is not None:
            return float(rows[0]["x"]), float(rows[0]["y"])
        return None

    def update_robot_target(self, robot_id, target_node):
        """현재 이동 중인 목적지 노드. 대시보드가 이 값 + 현재 좌표로
        "가야 할 경로"를 계산한다. 이동이 끝나면 None으로 지운다."""
        self._query("UPDATE robots SET target_node = %s WHERE robot_id = %s",
                    (target_node, robot_id))

    def upsert_vehicle(self, vehicle_id):
        self._query(
            "INSERT INTO vehicles (vehicle_id) VALUES (%s)"
            " ON DUPLICATE KEY UPDATE vehicle_id = vehicle_id",
            (vehicle_id,))

    def create_task(self, task_id, request_type, vehicle_id):
        self._query(
            "INSERT INTO tasks (task_id, request_type, vehicle_id)"
            " VALUES (%s, %s, %s)",
            (task_id, request_type, vehicle_id))

    def active_task_for_vehicle(self, vehicle_id):
        """같은 차량에 대해 아직 끝나지 않은 가장 최근 작업.

        UI의 중복 클릭 방지만으로는 브라우저 재전송이나 복수 클라이언트의
        동시 요청을 완전히 막을 수 없다. dispatcher가 새 작업을 만들기 전에
        DB 원장을 확인해 동일 차량의 Action goal 중복 생성을 차단한다.
        """
        rows = self._query(
            "SELECT task_id, request_type, state FROM tasks"
            " WHERE vehicle_id = %s"
            " AND state IN ('WAITING', 'PROCESSING')"
            " ORDER BY created_at DESC LIMIT 1",
            (vehicle_id,),
        )
        return rows[0] if rows else None

    def update_task(self, task_id, state=None, robot_id=None,
                    follower_robot_id=None, slot_id=None):
        sets, params = [], []
        for column, value in (("state", state), ("robot_id", robot_id),
                              ("follower_robot_id", follower_robot_id),
                              ("slot_id", slot_id)):
            if value is not None:
                sets.append(f"{column} = %s")
                params.append(value)
        if sets:
            self._query(
                f"UPDATE tasks SET {', '.join(sets)} WHERE task_id = %s",
                (*params, task_id))

    def get_task(self, task_id):
        rows = self._query("SELECT * FROM tasks WHERE task_id = %s", (task_id,))
        return rows[0] if rows else None

    def active_task_ids(self):
        """비상정지 복구를 막는 미종결 작업 ID 목록."""
        rows = self._query(
            "SELECT task_id FROM tasks"
            " WHERE state IN ('WAITING', 'PROCESSING')"
            " ORDER BY created_at"
        )
        return [row["task_id"] for row in rows]

    def unsafe_robot_statuses(self):
        """운영 복귀를 막는 로봇 상태. IDLE/CHARGING만 안전 대기로 본다."""
        return self._query(
            "SELECT robot_id, status FROM robots"
            " WHERE status NOT IN ('IDLE', 'CHARGING')"
            " ORDER BY robot_id"
        )

    # ---- 중앙 안전 상태 / 감사 이력 ----

    def ensure_safety_schema(self):
        """기존 설치에서도 supervisor 단독 기동이 가능하도록 멱등 생성한다."""
        self._query(
            """
            CREATE TABLE IF NOT EXISTS safety_state (
                singleton_id TINYINT PRIMARY KEY,
                state VARCHAR(32) NOT NULL DEFAULT 'NORMAL',
                stop_epoch BIGINT UNSIGNED NOT NULL DEFAULT 0,
                reason VARCHAR(500) NOT NULL DEFAULT '',
                operator_id VARCHAR(64) NOT NULL DEFAULT '',
                inspection_note VARCHAR(1000) NOT NULL DEFAULT '',
                affected_task_ids JSON NOT NULL,
                blockers JSON NOT NULL,
                updated_at TIMESTAMP NOT NULL DEFAULT CURRENT_TIMESTAMP
                    ON UPDATE CURRENT_TIMESTAMP,
                CONSTRAINT chk_safety_singleton CHECK (singleton_id = 1)
            )
            """
        )
        self._query(
            """
            CREATE TABLE IF NOT EXISTS safety_events (
                event_id BIGINT UNSIGNED AUTO_INCREMENT PRIMARY KEY,
                stop_epoch BIGINT UNSIGNED NOT NULL,
                event_type VARCHAR(32) NOT NULL,
                state VARCHAR(32) NOT NULL,
                operator_id VARCHAR(64) NOT NULL DEFAULT '',
                note VARCHAR(1000) NOT NULL DEFAULT '',
                details JSON NOT NULL,
                created_at TIMESTAMP NOT NULL DEFAULT CURRENT_TIMESTAMP
            )
            """
        )
        self._query(
            """
            INSERT INTO safety_state (
                singleton_id, state, affected_task_ids, blockers
            ) VALUES (1, 'NORMAL', JSON_ARRAY(), JSON_ARRAY())
            ON DUPLICATE KEY UPDATE singleton_id = singleton_id
            """
        )

    def get_safety_state(self):
        rows = self._query(
            "SELECT state, stop_epoch, reason, operator_id, inspection_note,"
            " affected_task_ids, blockers,"
            " DATE_FORMAT(updated_at, '%Y-%m-%dT%H:%i:%s') AS updated_at"
            " FROM safety_state WHERE singleton_id = 1"
        )
        if not rows:
            return None
        row = rows[0]
        for field in ("affected_task_ids", "blockers"):
            value = row.get(field)
            if isinstance(value, str):
                row[field] = json.loads(value)
            elif value is None:
                row[field] = []
        return row

    def set_safety_state(
        self,
        *,
        state,
        stop_epoch,
        reason,
        operator_id,
        inspection_note,
        affected_task_ids,
        blockers,
    ):
        self._query(
            """
            UPDATE safety_state
            SET state = %s, stop_epoch = %s, reason = %s,
                operator_id = %s, inspection_note = %s,
                affected_task_ids = %s, blockers = %s
            WHERE singleton_id = 1
            """,
            (
                state,
                stop_epoch,
                reason,
                operator_id,
                inspection_note,
                json.dumps(list(affected_task_ids), ensure_ascii=False),
                json.dumps(list(blockers), ensure_ascii=False),
            ),
        )

    def add_safety_event(
        self, *, stop_epoch, event_type, state, operator_id, note, details=None
    ):
        self._query(
            """
            INSERT INTO safety_events (
                stop_epoch, event_type, state, operator_id, note, details
            ) VALUES (%s, %s, %s, %s, %s, %s)
            """,
            (
                stop_epoch,
                event_type,
                state,
                operator_id,
                note,
                json.dumps(details or {}, ensure_ascii=False),
            ),
        )

    # ---- zone_locks ----

    def try_acquire_zone(self, zone_id, robot_id=None, task_id=None) -> bool:
        """INSERT 성공 = 락 획득. PK 중복(1062) = 다른 소유자가 보유 중.

        robot_id(로봇 개인) / task_id(로봇 2대 팀) 중 정확히 하나만 준다.
        """
        owner_robot, owner_task = _zone_owner(robot_id, task_id)
        try:
            self._query(
                "INSERT INTO zone_locks (zone_id, robot_id, task_id)"
                " VALUES (%s, %s, %s)",
                (zone_id, owner_robot, owner_task))
            return True
        except mysql.connector.Error as err:
            if err.errno == DUPLICATE_KEY_ERRNO:
                return False
            raise

    def release_zones(self, robot_id=None, task_id=None, zone_ids=None):
        """zone_ids가 None이면 해당 소유자의 보유 락 전체 해제."""
        owner_robot, owner_task = _zone_owner(robot_id, task_id)
        owner_column = "robot_id" if owner_robot is not None else "task_id"
        owner_value = owner_robot if owner_robot is not None else owner_task
        if zone_ids is None:
            self._query(
                f"DELETE FROM zone_locks WHERE {owner_column} = %s", (owner_value,))
        elif zone_ids:
            placeholders = ", ".join(["%s"] * len(zone_ids))
            self._query(
                f"DELETE FROM zone_locks WHERE {owner_column} = %s"
                f" AND zone_id IN ({placeholders})",
                (owner_value, *zone_ids))

    def reap_expired_locks(self, timeout_sec) -> int:
        """죽은 로봇의 만료 락 회수. 회수한 개수를 반환."""
        cursor = self._connection().cursor()
        try:
            cursor.execute(
                "DELETE FROM zone_locks"
                " WHERE locked_at < NOW() - INTERVAL %s SECOND",
                (timeout_sec,))
            return cursor.rowcount
        finally:
            cursor.close()
