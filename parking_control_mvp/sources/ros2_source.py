"""ROS2 데이터 소스: task_dispatcher(feature/parking-control) 연동.

feature/parking-control 브랜치의 실제 구현을 기준으로 한다.
  - 요청 등록  : dispatch_parking_task 서비스 (RequestParkingTask.srv) 호출.
  - 상태 조회  : 로봇/슬롯/작업 전체 목록을 주는 ROS2 서비스가 아직 없어서,
                dispatcher가 쓰는 MySQL(robots/parking_slots/tasks)을
                읽기 전용으로 폴링한다 (Team A의 scripts/dashboard.py와 동일 방식).
  - 실시간 알림: obstacle_alert 토픽(ObstacleAlert.msg) 구독.
  - 세부 진행 : task_state 토픽(TaskState.msg) 구독. robot_task_orchestrator가
                아직 스켈레톤 단계라 발행되지 않을 수 있음 — 그 경우 PROCESSING
                동안 세부 단계 갱신 없이 ROBOT_ASSIGNED로 유지된다.

주의(2026-07-20 기준, Team A와 확정 필요): DB tasks.state는 4단계
(WAITING/PROCESSING/DONE/FAILED)인데 우리 UI는 6단계라, task_state 토픽의
DETECTING/NAVIGATING/ALIGNING/LIFTING로 세분화한다. NAVIGATING은 리프트 전
(차량 접근)과 후(주차 위치 이동) 두 번 나오므로, 해당 task에서 LIFTING을
이미 관측했는지로 구분한다. FAILED는 우리 쪽 CANCELLED로 매핑한다.
"""

import threading
import time
from collections import deque
from datetime import datetime

import mysql.connector
import rclpy
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from sensor_msgs.msg import PointCloud2

from parking_robot_interfaces.msg import ObstacleAlert, SafetyState, TaskState
from parking_robot_interfaces.srv import (
    ActivateEmergencyStop,
    ApproveOperation,
    RequestParkingTask,
    RequestSafetyReset,
)

from parking_control.core.graph import ParkingMap
from parking_control.parking_slot_manager_node import _default_map_yaml

import config
from core.datasource import DataSource, DataSourceError
from core.models import (
    TERMINAL_STATUSES,
    Alert,
    AlertCategory,
    AlertLevel,
    ParkingRequest,
    ParkingSlot,
    RequestStatus,
    RequestType,
    Robot,
)

_REQUEST_TYPE_TO_ROS = {RequestType.PARK_IN: "ENTRY", RequestType.PARK_OUT: "EXIT"}
_ROS_TYPE_TO_REQUEST = {v: k for k, v in _REQUEST_TYPE_TO_ROS.items()}

_DB_STATE_TO_STATUS = {
    "WAITING": RequestStatus.WAITING,
    "PROCESSING": RequestStatus.ROBOT_ASSIGNED,
    "DONE": RequestStatus.COMPLETED,
    "FAILED": RequestStatus.CANCELLED,
}

_TASK_STATE_BEFORE_LIFT = {
    "DETECTING": RequestStatus.APPROACHING,
    "NAVIGATING": RequestStatus.APPROACHING,
    "ALIGNING": RequestStatus.APPROACHING,
    "LIFTING": RequestStatus.LIFTING,
    "RETURNING": RequestStatus.RETURNING,
}
_TASK_STATE_AFTER_LIFT = {
    "NAVIGATING": RequestStatus.MOVING_TO_SLOT,
    "RETURNING": RequestStatus.RETURNING,
}

# parking_slot_manager_node / task_dispatcher_node 어느 쪽도 find_empty_slot
# 응답 이후 parking_slots.status를 갱신하지 않는다 (2026-07-20 기준 실제 코드
# 확인 — set_slot_status() 호출이 아예 없음). DB 컬럼을 그대로 믿으면 슬롯이
# 영원히 EMPTY로 보이므로, 같은 슬롯의 가장 최근 task로 상태를 역산한다.
_SLOT_STATUS_FROM_TASK = {
    ("ENTRY", "WAITING"): "RESERVED",
    ("ENTRY", "PROCESSING"): "RESERVED",
    ("ENTRY", "DONE"): "OCCUPIED",
    ("EXIT", "WAITING"): "OCCUPIED",
    ("EXIT", "PROCESSING"): "OCCUPIED",
    ("EXIT", "DONE"): "EMPTY",
}

_LIDAR_CONTRACTS = (
    ("L1", "주차장 전체", -7.82, 0.0, "/parking/lidar/points_world"),
)


def _now() -> str:
    return datetime.now().isoformat(timespec="seconds")


def _extract_map_info(parking_map: ParkingMap) -> dict:
    """parking_map.yaml에서 실시간 도면용 v4 고정 배치를 뽑아낸다."""
    nodes = [
        {
            "id": node_id,
            "kind": attributes.get("kind"),
            "role": attributes.get("role"),
            "is_accessible": bool(attributes.get("is_accessible", False)),
            "x": attributes["x"],
            "y": attributes["y"],
        }
        for node_id, attributes in parking_map.graph.nodes(data=True)
        if attributes.get("x") is not None and attributes.get("y") is not None
    ]
    docks = [
        {
            "id": node_id,
            "role": parking_map.graph.nodes[node_id].get("role"),
            "x": parking_map.graph.nodes[node_id]["x"],
            "y": parking_map.graph.nodes[node_id]["y"],
        }
        for node_id in parking_map.nodes_of_kind("dock")
    ]
    node_by_id = {node["id"]: node for node in nodes}
    entrance = node_by_id.get("entry_outer")
    vehicle_zones = [
        {
            "id": "exit_outer",
            "role": "exit",
            "label": "출차 차량 대기 구역",
            **{
                key: node_by_id["exit_outer"][key]
                for key in ("x", "y")
            },
        },
        {
            "id": "entry_outer",
            "role": "entry",
            "label": "입차 차량 대기 구역",
            **{
                key: node_by_id["entry_outer"][key]
                for key in ("x", "y")
            },
        },
    ] if "entry_outer" in node_by_id and "exit_outer" in node_by_id else []

    return {
        "layout": "v4",
        "nodes": nodes,
        "docks": docks,
        "vehicle_zones": vehicle_zones,
        "entrance": entrance,
    }


class _ParkingDbReader:
    """dispatcher가 쓰는 MySQL을 조회한다.

    기본은 읽기 전용이지만, execute()는 테스트 환경 초기화(DB 초기화 버튼)
    용도로만 제한적으로 사용한다 — 일반 조회/조작 경로에서는 쓰지 않는다.

    같은 커넥션을 폴링 스레드(_poll_loop)와 FastAPI 요청 스레드(DB 초기화
    등)가 동시에 건드릴 수 있는데, mysql-connector의 C 확장 커넥션은
    스레드 간 동시 접근을 지원하지 않아 그대로 두면 프로세스가 죽는다
    (실제로 폴링 주기를 0.25초로 줄인 뒤 DB 초기화를 호출하다가
    "free(): invalid next size" 크래시로 재현됨). 그래서 모든 연결/쿼리
    접근을 락으로 직렬화한다.
    """

    def __init__(self, host: str, user: str, password: str, database: str) -> None:
        self._config = dict(
            host=host, user=user, password=password,
            database=database, autocommit=True,
        )
        self._conn = None
        self._lock = threading.Lock()
        self._has_follower_robot_column: bool | None = None

    def _connection(self):
        if self._conn is None or not self._conn.is_connected():
            self._conn = mysql.connector.connect(**self._config)
        return self._conn

    def _query(self, sql: str, params: tuple = ()) -> list[dict]:
        with self._lock:
            cursor = self._connection().cursor(dictionary=True)
            try:
                cursor.execute(sql, params)
                return cursor.fetchall()
            finally:
                cursor.close()

    def fetch_robots(self) -> list[dict]:
        return self._query(
            "SELECT robot_id, status, battery_percent, x, y FROM robots"
        )

    def fetch_slots(self) -> list[dict]:
        return self._query(
            "SELECT slot_id, status, x, y, is_accessible FROM parking_slots"
        )

    def fetch_tasks(self, limit: int = 100) -> list[dict]:
        if self._has_follower_robot_column is None:
            self._has_follower_robot_column = bool(
                self._query(
                    "SHOW COLUMNS FROM tasks LIKE 'follower_robot_id'"
                )
            )
        follower_column = (
            "follower_robot_id"
            if self._has_follower_robot_column
            else "NULL AS follower_robot_id"
        )
        return self._query(
            "SELECT task_id, request_type, state, vehicle_id, robot_id,"
            f" {follower_column}, slot_id, created_at"
            " FROM tasks ORDER BY created_at DESC LIMIT %s",
            (limit,),
        )

    def execute(self, sql: str) -> None:
        with self._lock:
            cursor = self._connection().cursor()
            try:
                cursor.execute(sql)
            finally:
                cursor.close()

    def close(self) -> None:
        if self._conn is not None and self._conn.is_connected():
            self._conn.close()
        self._conn = None


class Ros2DataSource(DataSource):
    supports_mock_controls = False

    def __init__(self, store) -> None:
        super().__init__(store)
        self._node: Node | None = None
        self._executor: SingleThreadedExecutor | None = None
        self._spin_thread: threading.Thread | None = None
        self._poll_thread: threading.Thread | None = None
        self._stop_event = threading.Event()
        self._dispatch_client = None
        self._emergency_stop_client = None
        self._safety_reset_client = None
        self._operation_approval_client = None
        self._db: _ParkingDbReader | None = None
        self._last_safety_state_at: float | None = None
        self._safety_state.update(
            state="UNKNOWN",
            motion_allowed=False,
            blockers=["중앙 safety_supervisor 상태 수신 대기"],
        )

        self._map_lock = threading.Lock()
        self._task_id_map: dict[str, int] = {}     # external_task_id -> internal id
        self._lifted_tasks: set[str] = set()        # LIFTING을 관측한 external_task_id
        self._fine_status: dict[str, RequestStatus] = {}  # task_state 토픽 기반 세부 상태
        self._map_info: dict = {
            "docks": [],
            "entrance": None,
            "sensors": [
                {"id": sensor_id, "zone": zone, "x": x, "y": y}
                for sensor_id, zone, x, y, _topic in _LIDAR_CONTRACTS
            ],
        }
        self._sensor_lock = threading.Lock()
        self._sensor_received: dict[str, deque] = {
            sensor_id: deque(maxlen=30)
            for sensor_id, _zone, _x, _y, _topic in _LIDAR_CONTRACTS
        }

    # ------------------------------------------------------------------
    # 기동/종료
    # ------------------------------------------------------------------
    def start(self) -> None:
        if not rclpy.ok():
            rclpy.init()

        self._node = Node("parking_control_web_bridge")
        self._dispatch_client = self._node.create_client(
            RequestParkingTask, config.DISPATCH_SERVICE_NAME
        )
        self._emergency_stop_client = self._node.create_client(
            ActivateEmergencyStop, "/safety/activate_emergency_stop"
        )
        self._safety_reset_client = self._node.create_client(
            RequestSafetyReset, "/safety/request_reset"
        )
        self._operation_approval_client = self._node.create_client(
            ApproveOperation, "/safety/approve_operation"
        )
        safety_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._node.create_subscription(
            SafetyState, "/safety/state", self._on_safety_state, safety_qos
        )
        self._node.create_subscription(
            ObstacleAlert, config.OBSTACLE_ALERT_TOPIC, self._on_obstacle_alert, 10
        )
        self._node.create_subscription(
            TaskState, config.TASK_STATE_TOPIC, self._on_task_state, 10
        )
        for sensor_id, _zone, _x, _y, topic in _LIDAR_CONTRACTS:
            self._node.create_subscription(
                PointCloud2,
                topic,
                lambda _msg, sid=sensor_id: self._on_lidar(sid),
                qos_profile_sensor_data,
            )

        self._executor = SingleThreadedExecutor()
        self._executor.add_node(self._node)
        self._spin_thread = threading.Thread(
            target=self._spin_executor, daemon=True
        )
        self._spin_thread.start()

        self._db = _ParkingDbReader(
            config.DB_HOST, config.DB_USER, config.DB_PASSWORD, config.DB_NAME
        )
        self._stop_event.clear()
        self._poll_thread = threading.Thread(target=self._poll_loop, daemon=True)
        self._poll_thread.start()

        try:
            parking_map = ParkingMap.load(_default_map_yaml())
            self._map_info = _extract_map_info(parking_map)
            self._map_info["sensors"] = [
                {"id": sensor_id, "zone": zone, "x": x, "y": y}
                for sensor_id, zone, x, y, _topic in _LIDAR_CONTRACTS
            ]
        except Exception as exc:  # 지도 파일이 없어도 나머지 기능은 계속 동작
            self._node.get_logger().warn(
                f"parking_map.yaml 로드 실패, 도면에 도크/입구 생략: {exc}"
            )

        self._node.get_logger().info(
            f"parking_control_web_bridge 시작 (dispatch={config.DISPATCH_SERVICE_NAME})"
        )

    def _spin_executor(self) -> None:
        try:
            self._executor.spin()
        except ExternalShutdownException:
            pass

    def stop(self) -> None:
        self._stop_event.set()
        if self._poll_thread is not None:
            self._poll_thread.join(timeout=3)
        if self._executor is not None:
            self._executor.shutdown()
        if self._node is not None:
            self._node.destroy_node()
        if self._db is not None:
            self._db.close()
        if rclpy.ok():
            rclpy.shutdown()

    # ------------------------------------------------------------------
    # DB 폴링 → StateStore 반영
    # ------------------------------------------------------------------
    def _poll_loop(self) -> None:
        while not self._stop_event.is_set():
            try:
                self._poll_once()
            except Exception as exc:  # DB 재연결 실패 등 — 로그만 남기고 계속 재시도
                if self._node is not None:
                    self._node.get_logger().warn(f"DB poll 실패: {exc}")
            self._stop_event.wait(config.DB_POLL_INTERVAL_SEC)

    def _poll_once(self) -> None:
        robot_rows = self._db.fetch_robots()
        slot_rows = self._db.fetch_slots()
        task_rows = self._db.fetch_tasks()

        with self.store.lock, self._map_lock:
            map_node_by_id = {
                node["id"]: node for node in self._map_info.get("nodes", [])
            }
            valid_slot_ids = {
                node["id"]
                for node in self._map_info.get("nodes", [])
                if node.get("kind") == "slot"
            }
            if valid_slot_ids:
                # v3 DB 행(A4~B8)이 남아 있어도 v4 웹 도면에는 현재 지도에
                # 실제로 존재하는 A1~A3만 노출한다.
                slot_rows = [
                    row for row in slot_rows if row["slot_id"] in valid_slot_ids
                ]
            requests = []
            active_task_by_robot: dict[str, int] = {}
            # slot_id -> (파생 상태, 차량번호). task_rows는 created_at DESC라
            # 슬롯당 처음 매칭되는(=가장 최근) 유효 상태를 채택한다.
            slot_status_override: dict[str, tuple[str, str | None]] = {}

            for row in task_rows:
                internal_id = self._task_id_map.get(row["task_id"])
                if internal_id is None:
                    internal_id = self.store.next_request_id()
                    self._task_id_map[row["task_id"]] = internal_id

                status = _DB_STATE_TO_STATUS.get(row["state"], RequestStatus.WAITING)
                if row["state"] == "PROCESSING":
                    status = self._fine_status.get(row["task_id"], status)

                created_at = row["created_at"]
                requests.append(
                    ParkingRequest(
                        id=internal_id,
                        request_type=_ROS_TYPE_TO_REQUEST.get(
                            row["request_type"], RequestType.PARK_IN
                        ),
                        vehicle_number=row["vehicle_id"],
                        slot_id=row["slot_id"],
                        robot_id=row["robot_id"],
                        robot_ids=[
                            robot_id
                            for robot_id in (
                                row["robot_id"],
                                row["follower_robot_id"],
                            )
                            if robot_id
                        ],
                        status=status,
                        created_at=(
                            created_at.isoformat(timespec="seconds")
                            if hasattr(created_at, "isoformat")
                            else str(created_at)
                        ),
                        external_task_id=row["task_id"],
                    )
                )

                if row["robot_id"] and row["state"] in ("WAITING", "PROCESSING"):
                    active_task_by_robot[row["robot_id"]] = internal_id
                if (
                    row["follower_robot_id"]
                    and row["state"] in ("WAITING", "PROCESSING")
                ):
                    active_task_by_robot[row["follower_robot_id"]] = internal_id

                if row["slot_id"] and row["slot_id"] not in slot_status_override:
                    derived = _SLOT_STATUS_FROM_TASK.get(
                        (row["request_type"], row["state"])
                    )
                    if derived is not None:
                        vehicle = row["vehicle_id"] if derived == "OCCUPIED" else None
                        slot_status_override[row["slot_id"]] = (derived, vehicle)

            # DB는 최신순 LIMIT이라 오래된 요청이 위로 오도록 뒤집는다
            # (mock과 동일하게 store에는 생성 순으로 쌓고, API가 reversed() 처리).
            requests.reverse()
            self.store.requests.clear()
            self.store.requests.extend(requests)

            self.store.parking_slots.clear()
            self.store.parking_slots.extend(
                ParkingSlot(
                    id=row["slot_id"],
                    status=slot_status_override.get(row["slot_id"], (row["status"], None))[0],
                    vehicle_number=slot_status_override.get(row["slot_id"], (row["status"], None))[1],
                    # DB에 v3 좌표가 남아 있어도 웹 도면은 현재 로드한
                    # parking_map.yaml을 단일 좌표 원본으로 사용한다.
                    x=float(map_node_by_id[row["slot_id"]]["x"])
                    if row["slot_id"] in map_node_by_id
                    else (float(row["x"]) if row["x"] is not None else None),
                    y=float(map_node_by_id[row["slot_id"]]["y"])
                    if row["slot_id"] in map_node_by_id
                    else (float(row["y"]) if row["y"] is not None else None),
                    is_accessible=(
                        map_node_by_id[row["slot_id"]]["is_accessible"]
                        if row["slot_id"] in map_node_by_id
                        else bool(row["is_accessible"])
                    ),
                )
                for row in slot_rows
            )

            self.store.robots.clear()
            self.store.robots.extend(
                Robot(
                    id=row["robot_id"],
                    status=row["status"],
                    battery=(
                        int(row["battery_percent"])
                        if row["battery_percent"] is not None
                        else 0
                    ),
                    current_task_id=active_task_by_robot.get(row["robot_id"]),
                    error_message=(
                        "로봇 오류 (DB status=ERROR, 상세 메시지 없음)"
                        if row["status"] == "ERROR"
                        else None
                    ),
                    x=float(row["x"]) if row["x"] is not None else None,
                    y=float(row["y"]) if row["y"] is not None else None,
                )
                for row in robot_rows
            )

    def get_map_info(self) -> dict:
        return self._map_info

    def _on_lidar(self, sensor_id: str) -> None:
        with self._sensor_lock:
            self._sensor_received[sensor_id].append(time.monotonic())

    def get_sensor_status(self) -> list[dict]:
        now = time.monotonic()
        statuses = []
        with self._sensor_lock:
            for sensor_id, _zone, _x, _y, topic in _LIDAR_CONTRACTS:
                received = self._sensor_received[sensor_id]
                age = now - received[-1] if received else None
                rate = None
                if len(received) >= 2:
                    elapsed = received[-1] - received[0]
                    if elapsed > 0:
                        rate = round((len(received) - 1) / elapsed, 1)
                statuses.append(
                    {
                        "id": sensor_id,
                        "topic": topic,
                        "status": "ONLINE" if age is not None and age <= 3.0 else "OFFLINE",
                        "rate_hz": rate,
                        "last_seen_sec": round(age, 1) if age is not None else None,
                    }
                )
        return statuses

    # ------------------------------------------------------------------
    # 토픽 콜백 (rclpy 스핀 스레드에서 호출됨)
    # ------------------------------------------------------------------
    def _on_obstacle_alert(self, msg: ObstacleAlert) -> None:
        if not msg.obstacle_detected:
            return
        with self.store.lock:
            self.store.alerts.append(
                Alert(
                    id=self.store.next_alert_id(),
                    level=AlertLevel.WARNING,
                    category=AlertCategory.OBSTACLE,
                    message=msg.description or "주행 경로에서 장애물이 감지되었습니다.",
                    robot_id=None,
                    created_at=_now(),
                )
            )

    def _on_task_state(self, msg: TaskState) -> None:
        with self._map_lock:
            if msg.state == "LIFTING":
                self._lifted_tasks.add(msg.task_id)
            table = (
                _TASK_STATE_AFTER_LIFT
                if msg.task_id in self._lifted_tasks
                else _TASK_STATE_BEFORE_LIFT
            )
            status = table.get(msg.state)
            if status is not None:
                self._fine_status[msg.task_id] = status

        if msg.state == "FAILED":
            with self.store.lock:
                self.store.alerts.append(
                    Alert(
                        id=self.store.next_alert_id(),
                        level=AlertLevel.ERROR,
                        category=AlertCategory.ROBOT_ERROR,
                        message=(
                            f"{msg.robot_id} 작업 실패"
                            f"{f' ({msg.current_step})' if msg.current_step else ''}"
                        ),
                        robot_id=msg.robot_id or None,
                        created_at=_now(),
                    )
                )

    def _on_safety_state(self, msg: SafetyState) -> None:
        self._last_safety_state_at = time.monotonic()
        state = {
            "state": msg.state,
            "motion_allowed": msg.motion_allowed,
            "stop_epoch": int(msg.stop_epoch),
            "reason": msg.reason,
            "operator_id": msg.operator_id,
            "inspection_note": msg.inspection_note,
            "affected_task_ids": list(msg.affected_task_ids),
            "blockers": list(msg.blockers),
            "updated_at": msg.updated_at,
        }
        with self.store.lock:
            self._safety_state = state
            self._emergency_stop_active = msg.state != "NORMAL"
            alert = next(
                (
                    item
                    for item in self.store.alerts
                    if item.category == AlertCategory.EMERGENCY_STOP and item.active
                ),
                None,
            )
            if msg.state == "NORMAL":
                if alert is not None:
                    alert.active = False
                return

            if msg.state == "READY_FOR_OPERATION":
                message = (
                    "안전 점검이 승인되었습니다. 로봇은 계속 정지 상태이며 "
                    "별도의 운영 복귀 승인이 필요합니다."
                )
            elif msg.state == "UNKNOWN":
                message = "중앙 안전 관리자 상태를 확인할 수 없습니다."
            else:
                message = (
                    "전체 로봇 비상정지가 유지되고 있습니다. "
                    "현장 점검 후 관제 해제 요청을 진행해주세요."
                )
            if alert is None:
                self.store.alerts.append(
                    Alert(
                        id=self.store.next_alert_id(),
                        level=AlertLevel.ERROR,
                        category=AlertCategory.EMERGENCY_STOP,
                        message=message,
                        robot_id=None,
                        created_at=_now(),
                    )
                )
            else:
                alert.message = message

    @property
    def safety_state(self) -> dict:
        state = super().safety_state
        if (
            self._last_safety_state_at is None
            or time.monotonic() - self._last_safety_state_at > 3.0
        ):
            return {
                **state,
                "state": "UNKNOWN",
                "motion_allowed": False,
                "blockers": ["중앙 safety_supervisor heartbeat가 수신되지 않습니다."],
            }
        return state

    @property
    def emergency_stop_active(self) -> bool:
        return self.safety_state["state"] != "NORMAL"

    # ------------------------------------------------------------------
    # 요청 등록 (FastAPI 워커 스레드에서 호출됨)
    # ------------------------------------------------------------------
    def create_request(self, payload) -> ParkingRequest:
        vehicle_number = payload.vehicle_number.strip()
        if not vehicle_number:
            raise DataSourceError("차량 번호를 입력해주세요.", status_code=400)
        if self.emergency_stop_active:
            raise DataSourceError(
                "비상정지 상태에서는 새 작업을 등록할 수 없습니다.",
                status_code=423,
            )

        if not self._dispatch_client.wait_for_service(
            timeout_sec=config.DISPATCH_SERVICE_TIMEOUT_SEC
        ):
            raise DataSourceError(
                f"task_dispatcher 서비스({config.DISPATCH_SERVICE_NAME})에 "
                "연결할 수 없습니다. dispatcher 노드가 떠 있는지 확인해주세요.",
                status_code=503,
            )

        request = RequestParkingTask.Request()
        request.request_type = _REQUEST_TYPE_TO_ROS[payload.request_type]
        request.vehicle_id = vehicle_number

        done = threading.Event()
        outcome: dict = {}

        def _on_done(future) -> None:
            outcome["response"] = future.result()
            outcome["exception"] = future.exception()
            done.set()

        future = self._dispatch_client.call_async(request)
        future.add_done_callback(_on_done)

        if not done.wait(timeout=config.DISPATCH_SERVICE_TIMEOUT_SEC):
            raise DataSourceError("dispatcher 응답이 시간 내에 오지 않았습니다.", status_code=504)

        if outcome.get("exception") is not None:
            raise DataSourceError(
                f"dispatcher 호출 중 오류: {outcome['exception']}", status_code=502
            )

        response = outcome["response"]
        if not response.accepted:
            raise DataSourceError(
                response.message or "요청이 거절되었습니다.", status_code=409
            )

        with self.store.lock, self._map_lock:
            internal_id = self._task_id_map.get(response.task_id)
            if internal_id is None:
                internal_id = self.store.next_request_id()
                self._task_id_map[response.task_id] = internal_id

            parking_request = ParkingRequest(
                id=internal_id,
                request_type=payload.request_type,
                vehicle_number=vehicle_number,
                slot_id=payload.slot_id,
                robot_id=None,
                robot_ids=[],
                status=RequestStatus.WAITING,
                created_at=_now(),
                external_task_id=response.task_id,
            )
            if self.store.find_request(internal_id) is None:
                self.store.requests.append(parking_request)

        return parking_request

    # ------------------------------------------------------------------
    # 비상정지 / mock 전용 제어
    # ------------------------------------------------------------------
    def emergency_stop(self) -> int:
        """중앙 safety_supervisor에 영속 비상정지를 요청한다."""
        with self.store.lock:
            active_requests = [
                request
                for request in self.store.requests
                if request.status not in TERMINAL_STATUSES
            ]
            if self.safety_state["state"] in {
                "STOPPED_LATCHED",
                "READY_FOR_OPERATION",
            }:
                return len(active_requests)
            task_ids = [
                request.external_task_id
                for request in active_requests
                if request.external_task_id
            ]

        request = ActivateEmergencyStop.Request(
            operator_id="control_ui",
            reason="관제 UI 전체 비상정지",
            affected_task_ids=task_ids,
        )
        response = self._call_safety_service(
            self._emergency_stop_client,
            request,
            "중앙 비상정지",
        )
        if not response.accepted:
            raise DataSourceError(response.message, status_code=409)
        return len(active_requests)

    def request_safety_reset(self, payload) -> dict:
        request = RequestSafetyReset.Request(
            operator_id=payload.operator_id.strip(),
            inspection_note=payload.inspection_note.strip(),
            area_clear=payload.area_clear,
            robots_stopped=payload.robots_stopped,
            load_secured=payload.load_secured,
            sensors_checked=payload.sensors_checked,
        )
        response = self._call_safety_service(
            self._safety_reset_client, request, "안전 해제 요청"
        )
        if not response.accepted:
            detail = "; ".join(response.blockers) or response.message
            raise DataSourceError(detail, status_code=409)
        return {
            "state": response.state,
            "stop_epoch": int(response.stop_epoch),
            "message": response.message,
            "blockers": list(response.blockers),
        }

    def approve_operation(self, payload) -> dict:
        request = ApproveOperation.Request(
            operator_id=payload.operator_id.strip(),
            approval_note=payload.approval_note.strip(),
        )
        response = self._call_safety_service(
            self._operation_approval_client, request, "운영 복귀 승인"
        )
        if not response.accepted:
            detail = "; ".join(response.blockers) or response.message
            raise DataSourceError(detail, status_code=409)
        return {
            "state": response.state,
            "stop_epoch": int(response.stop_epoch),
            "message": response.message,
            "blockers": list(response.blockers),
        }

    def _call_safety_service(self, client, request, label):
        if client is None or not client.wait_for_service(
            timeout_sec=config.DISPATCH_SERVICE_TIMEOUT_SEC
        ):
            raise DataSourceError(
                f"{label} 서비스에 연결할 수 없습니다. safety_supervisor를 확인해주세요.",
                status_code=503,
            )
        done = threading.Event()
        outcome = {}

        def _on_done(future):
            outcome["response"] = future.result()
            outcome["exception"] = future.exception()
            done.set()

        future = client.call_async(request)
        future.add_done_callback(_on_done)
        if not done.wait(timeout=config.DISPATCH_SERVICE_TIMEOUT_SEC):
            raise DataSourceError(f"{label} 응답 시간 초과", status_code=504)
        if outcome.get("exception") is not None:
            raise DataSourceError(
                f"{label} 호출 오류: {outcome['exception']}", status_code=502
            )
        return outcome["response"]

    def advance_request(self, request_id: int) -> ParkingRequest:
        raise DataSourceError(
            "ros2 모드에서는 단계를 수동으로 진행할 수 없습니다 (dispatcher가 제어합니다).",
            status_code=403,
        )

    def reset(self) -> None:
        raise DataSourceError("ros2 모드에서는 초기화를 지원하지 않습니다.", status_code=403)

    # ------------------------------------------------------------------
    # DB 초기화 (테스트/개발 환경 전용) — run_test_stack.sh의 정리 로직과 동일한 범위.
    # 로봇의 x/y는 건드리지 않는다: 실제 하드웨어에서는 그 좌표가 로봇 자신의
    # 위치 보고값이라, 웹에서 임의로 덮어쓰면 실제 위치와 어긋난 값이 남는다.
    # ------------------------------------------------------------------
    def reset_test_environment(self) -> None:
        if self._emergency_stop_active:
            raise DataSourceError(
                "비상정지 상태에서는 DB 초기화를 사용할 수 없습니다. 시스템을 재기동해주세요.",
                status_code=423,
            )
        with self.store.lock, self._map_lock:
            # 진행 중인 작업이 있으면 초기화를 거부한다. sim_orchestrator(또는
            # 실제 orchestrator)의 실행은 DB 밖(ROS2 액션)에서 계속 도는 중이라
            # 여기서 취소할 방법이 없고, 그대로 초기화하면 뒤늦게 끝난 실행이
            # 슬롯을 다시 OCCUPIED로 덮어써 "초기화했는데 슬롯이 도로 찬" 상태가
            # 된다 (실제로 재현된 버그).
            active = [
                r for r in self.store.requests if r.status not in TERMINAL_STATUSES
            ]
            if active:
                raise DataSourceError(
                    "진행 중인 작업이 있어 DB를 초기화할 수 없습니다. "
                    "작업이 완료된 뒤 다시 시도해주세요.",
                    status_code=409,
                )

            self._db.execute("UPDATE parking_slots SET status='EMPTY'")
            self._db.execute("UPDATE robots SET status='IDLE', target_node=NULL")
            self._db.execute(
                "INSERT INTO robots "
                "(robot_id, status, x, y, battery_percent, target_node) VALUES "
                "('entry_lead', 'IDLE', -3.2, -2.2, 100, NULL), "
                "('entry_follow', 'IDLE', -1.2, -2.2, 100, NULL), "
                "('exit_lead', 'IDLE', -3.2, 2.2, 100, NULL), "
                "('exit_follow', 'IDLE', -1.2, 2.2, 100, NULL) "
                "ON DUPLICATE KEY UPDATE status=VALUES(status), "
                "target_node=NULL"
            )
            self._db.execute("DELETE FROM zone_locks")
            self._db.execute("DELETE FROM tasks")
            self._db.execute("DELETE FROM vehicles")

            self._task_id_map.clear()
            self._lifted_tasks.clear()
            self._fine_status.clear()
            self.store.requests.clear()
            self.store.alerts.clear()
