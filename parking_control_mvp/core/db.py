"""입차/출차 로봇 그룹 라우팅용 경량 SQLite DB.

dual 모드(sources/ros2_dual_source.py) 전용. 어느 요청 유형(PARK_IN/PARK_OUT)을
어느 로봇 그룹(= 어느 Isaac Sim PC에 떠 있는 user_request_gateway_node 서비스)으로
보낼지를 robot_groups 테이블에 저장하고, 실제 전달 결과를 dispatch_log 테이블에
기록한다. 로봇 스택(src/parkbot_motion) 쪽은 전혀 건드리지 않는다 — 두 번째
Isaac Sim PC(출차 로봇)가 dispatch_service 파라미터를 이 테이블의 값과 일치하게
launch 하면 바로 연결된다.
"""

import sqlite3
import threading
from datetime import datetime

import config

_SCHEMA = """
CREATE TABLE IF NOT EXISTS robot_groups (
    group_id TEXT PRIMARY KEY,          -- 'entry' | 'exit'
    display_name TEXT NOT NULL,
    request_type TEXT NOT NULL,         -- 'PARK_IN' | 'PARK_OUT' 담당
    dispatch_service TEXT NOT NULL,     -- RequestParkingTask 서비스 이름 (dispatch_parking_task 계열)
    leader_robot_id TEXT,
    follower_robot_id TEXT,
    enabled INTEGER NOT NULL DEFAULT 1
);

CREATE TABLE IF NOT EXISTS dispatch_log (
    id INTEGER PRIMARY KEY AUTOINCREMENT,
    task_id TEXT,
    group_id TEXT NOT NULL,
    request_type TEXT NOT NULL,
    vehicle_number TEXT,
    accepted INTEGER NOT NULL,
    message TEXT,
    created_at TEXT NOT NULL
);
"""

#: 최초 실행 시 시드하는 기본 라우팅. entry는 현재 실제로 떠 있는
#: user_request_gateway_node(nodes.launch.py) 기본값과 동일하게 맞춘다.
#: exit는 아직 로봇 안무가 없으므로(2026-07-28 기준) 서비스가 없다는 전제로
#: 이름만 미리 배정해둔다 — 출차 Isaac Sim PC가 이 이름으로 서비스를 열면
#: 관제 쪽 코드 변경 없이 그대로 연결된다.
_DEFAULT_GROUPS = [
    dict(
        group_id="entry",
        display_name="입차로봇 그룹",
        request_type="PARK_IN",
        dispatch_service="dispatch_parking_task",
        leader_robot_id="entry_lead",
        follower_robot_id="entry_follow",
    ),
    dict(
        group_id="exit",
        display_name="출차로봇 그룹",
        request_type="PARK_OUT",
        dispatch_service="dispatch_parking_task_exit",
        leader_robot_id="exit_lead",
        follower_robot_id="exit_follow",
    ),
]

_conn: sqlite3.Connection | None = None
_lock = threading.Lock()


def init_db() -> None:
    """최초 호출 시 연결·스키마·기본 라우팅 시드를 준비한다. 이후 호출은 no-op."""
    global _conn
    if _conn is not None:
        return
    with _lock:
        if _conn is not None:  # 다른 스레드가 먼저 초기화했을 수 있음
            return
        conn = sqlite3.connect(config.DB_SQLITE_PATH, check_same_thread=False)
        conn.row_factory = sqlite3.Row
        conn.executescript(_SCHEMA)
        _seed_defaults(conn)
        conn.commit()
        _conn = conn


def _seed_defaults(conn: sqlite3.Connection) -> None:
    existing = {row["group_id"] for row in conn.execute("SELECT group_id FROM robot_groups")}
    for group in _DEFAULT_GROUPS:
        if group["group_id"] in existing:
            continue
        conn.execute(
            "INSERT INTO robot_groups"
            " (group_id, display_name, request_type, dispatch_service,"
            "  leader_robot_id, follower_robot_id, enabled)"
            " VALUES (:group_id, :display_name, :request_type, :dispatch_service,"
            "         :leader_robot_id, :follower_robot_id, 1)",
            group,
        )


def list_robot_groups() -> list[sqlite3.Row]:
    """활성화된 로봇 그룹 전체 (group_id 순)."""
    init_db()
    with _lock:
        return list(_conn.execute("SELECT * FROM robot_groups WHERE enabled = 1 ORDER BY group_id"))


def get_group_for_request_type(request_type: str) -> sqlite3.Row | None:
    """요청 유형(PARK_IN/PARK_OUT)을 담당하는 로봇 그룹 1개."""
    init_db()
    with _lock:
        return _conn.execute(
            "SELECT * FROM robot_groups WHERE request_type = ? AND enabled = 1 LIMIT 1",
            (request_type,),
        ).fetchone()


def log_dispatch(
    *,
    task_id: str | None,
    group_id: str,
    request_type: str,
    vehicle_number: str,
    accepted: bool,
    message: str,
) -> None:
    """로봇 그룹으로의 전달 시도 결과를 기록한다 (성공/실패 모두)."""
    init_db()
    with _lock:
        _conn.execute(
            "INSERT INTO dispatch_log"
            " (task_id, group_id, request_type, vehicle_number, accepted, message, created_at)"
            " VALUES (?, ?, ?, ?, ?, ?, ?)",
            (
                task_id,
                group_id,
                request_type,
                vehicle_number,
                int(accepted),
                message,
                datetime.now().isoformat(timespec="seconds"),
            ),
        )
        _conn.commit()


def last_dispatch_at(group_id: str) -> str | None:
    """해당 그룹으로 마지막으로 전달을 시도한 시각 (성공/실패 무관)."""
    init_db()
    with _lock:
        row = _conn.execute(
            "SELECT created_at FROM dispatch_log WHERE group_id = ?"
            " ORDER BY id DESC LIMIT 1",
            (group_id,),
        ).fetchone()
        return row["created_at"] if row else None


def recent_dispatch_log(limit: int = 50) -> list[sqlite3.Row]:
    init_db()
    with _lock:
        return list(
            _conn.execute("SELECT * FROM dispatch_log ORDER BY id DESC LIMIT ?", (limit,))
        )
