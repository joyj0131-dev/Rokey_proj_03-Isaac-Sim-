"""ROS2 데이터 소스: task_dispatcher(feature/parking-control) 연동.

feature/parking-control 브랜치의 실제 구현을 기준으로 한다.
  - 요청 등록  : dispatch_parking_task 서비스 (RequestParkingTask.srv) 호출.
  - 상태 조회  : 로봇/슬롯/작업 전체 목록을 주는 ROS2 서비스가 아직 없어서,
                dispatcher가 쓰는 MySQL(robots/parking_slots/tasks)을
                읽기 전용으로 폴링한다 (Team A의 scripts/dashboard.py와 동일 방식).
  - 실시간 알림: obstacle_alert 토픽(ObstacleAlert.msg) 구독.
  - 세부 진행 : task_state 토픽(TaskState.msg) 구독. 세부 상태를 받지 못한
                PROCESSING 작업만 ROBOT_ASSIGNED로 유지한다.

주의(2026-07-20 기준, Team A와 확정 필요): DB tasks.state는 4단계
(WAITING/PROCESSING/DONE/FAILED)인데 우리 UI는 6단계라, task_state 토픽의
DETECTING/NAVIGATING/ALIGNING/LIFTING 계약과 실제 orchestrator가 발행하는
SEARCHING/APPROACHING/PICKED_UP/MOVING 상태를 모두 수용한다. NAVIGATING은
리프트 전(차량 접근)과 후(주차 위치 이동) 두 번 나오므로, 해당 task에서
LIFTING 또는 PICKED_UP을 이미 관측했는지로 구분한다.
"""

import copy
import json
import math
import threading
import time
from collections import deque
from datetime import datetime

import mysql.connector
import rclpy
from rclpy._rclpy_pybind11 import RCLError
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from geometry_msgs.msg import PointStamped, PoseStamped, Twist
from sensor_msgs.msg import JointState, PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Float32, String

from parking_robot_interfaces.msg import (
    ObstacleAlert,
    SafetyState,
    SlotOccupancy,
    SlotOccupancyArray,
    TaskState,
)
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
from core.lidar_visualization import build_lidar_visualization
from core.obstacle_scope import blocking_obstacle, blocking_request_message
from core.safety_incident import (
    open_obstacle_incident,
    recover_obstacle_incident,
)
from core.models import (
    TERMINAL_STATUSES,
    Alert,
    AlertCategory,
    AlertLevel,
    CooperativeLoadState,
    ParkingRequest,
    ParkingSlot,
    RequestStatus,
    RequestType,
    Robot,
    RobotRecoveryRequest,
    SupportPointState,
    VisionAlignmentState,
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
    "SEARCHING": RequestStatus.APPROACHING,
    "APPROACHING": RequestStatus.APPROACHING,
    "LIFTING": RequestStatus.LIFTING,
    "PICKED_UP": RequestStatus.LIFTING,
    "MOVING": RequestStatus.MOVING_TO_SLOT,
    "ARRIVED": RequestStatus.MOVING_TO_SLOT,
    "PARKED": RequestStatus.MOVING_TO_SLOT,
    "UNPARKED": RequestStatus.MOVING_TO_SLOT,
    "RETURNING": RequestStatus.RETURNING,
    "DONE": RequestStatus.COMPLETED,
    "FAILED": RequestStatus.CANCELLED,
}
_TASK_STATE_AFTER_LIFT = {
    "LIFTING": RequestStatus.LIFTING,
    "PICKED_UP": RequestStatus.LIFTING,
    "NAVIGATING": RequestStatus.MOVING_TO_SLOT,
    "MOVING": RequestStatus.MOVING_TO_SLOT,
    "ARRIVED": RequestStatus.MOVING_TO_SLOT,
    "PARKED": RequestStatus.MOVING_TO_SLOT,
    "UNPARKED": RequestStatus.MOVING_TO_SLOT,
    "RETURNING": RequestStatus.RETURNING,
    "DONE": RequestStatus.COMPLETED,
    "FAILED": RequestStatus.CANCELLED,
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
    ("L1", "주차장 전체", 0.5, 0.0, "/parking/lidar/points_world"),
)

_V4_ROBOT_IDS = (
    "entry_lead",
    "entry_follow",
    "exit_lead",
    "exit_follow",
)
_ARM_TARGET_RAD = math.pi / 2.0
_SUPPORT_POINT_JOINTS = (
    ("left", ("arm_left_front_joint", "arm_left_rear_joint")),
    ("right", ("arm_right_front_joint", "arm_right_rear_joint")),
)
_SAFETY_TASK_TERMINATION_MARKERS = (
    "비상정지",
    "안전 정지",
    "전체 정지",
    "취소",
    "emergency stop",
    "emergency_stop",
    "safety stop",
    "e-stop",
    "estop",
    "cancel",
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


class ParkingDbReader:
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
            f" {follower_column}, slot_id, created_at, updated_at"
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
        self._db: ParkingDbReader | None = None
        self._last_safety_state_at: float | None = None
        self._safety_state.update(
            state="UNKNOWN",
            motion_allowed=False,
            blockers=["중앙 safety_supervisor 상태 수신 대기"],
        )
        self._recovery_state.update(
            control_available=False,
            message="실제 로봇 안전 복귀 제어기 연결 대기",
        )

        self._map_lock = threading.Lock()
        self._task_id_map: dict[str, int] = {}     # external_task_id -> internal id
        self._lifted_tasks: set[str] = set()        # LIFTING을 관측한 external_task_id
        self._fine_status: dict[str, RequestStatus] = {}  # task_state 토픽 기반 세부 상태
        self._map_info: dict = {
            "docks": [],
            "entrance": None,
            "sensors": [
                {
                    "id": sensor_id,
                    "zone": zone,
                    "x": x,
                    "y": y,
                    "z": 5.12,
                    "fov_deg": 360,
                    "frame_id": "map",
                }
                for sensor_id, zone, x, y, _topic in _LIDAR_CONTRACTS
            ],
        }
        self._sensor_lock = threading.Lock()
        self._sensor_received: dict[str, deque] = {
            sensor_id: deque(maxlen=30)
            for sensor_id, _zone, _x, _y, _topic in _LIDAR_CONTRACTS
        }
        self._lidar_visualization: dict | None = None
        self._lidar_visualization_updated_at: float | None = None
        self._slot_occupancy: dict | None = None
        self._slot_occupancy_updated_at: float | None = None
        self._perception_lock = threading.Lock()
        self._robot_pose_x: dict[str, float] = {}
        self._robot_motion: dict[str, dict[str, float]] = {}
        self._robot_command_motion: dict[str, tuple[float, float]] = {}
        self._robot_slip_since: dict[str, float] = {}
        self._axle_center_x: dict[str, float] = {}
        self._arm_actual_percent: dict[str, dict[str, float]] = {}
        self._lift_command_percent: dict[str, float] = {}
        self._load_telemetry_received: dict[str, deque] = {
            robot_id: deque(maxlen=60) for robot_id in _V4_ROBOT_IDS
        }
        self._vision_alignment: dict[
            str, tuple[VisionAlignmentState, float]
        ] = {}

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
        self._node.create_subscription(
            SlotOccupancyArray,
            config.SLOT_OCCUPANCY_TOPIC,
            self._on_slot_occupancy,
            safety_qos,
        )
        for sensor_id, _zone, _x, _y, topic in _LIDAR_CONTRACTS:
            self._node.create_subscription(
                PointCloud2,
                topic,
                lambda msg, sid=sensor_id: self._on_lidar(sid, msg),
                qos_profile_sensor_data,
            )
        for robot_id in _V4_ROBOT_IDS:
            self._node.create_subscription(
                PoseStamped,
                f"/robot_{robot_id}/pose",
                lambda msg, rid=robot_id: self._on_robot_pose(rid, msg),
                qos_profile_sensor_data,
            )
            self._node.create_subscription(
                PointStamped,
                f"/robot_{robot_id}/axle_center",
                lambda msg, rid=robot_id: self._on_axle_center(rid, msg),
                10,
            )
            self._node.create_subscription(
                JointState,
                f"/robot_{robot_id}/joint_states",
                lambda msg, rid=robot_id: self._on_joint_state(rid, msg),
                qos_profile_sensor_data,
            )
            self._node.create_subscription(
                Float32,
                f"/robot_{robot_id}/lift_cmd",
                lambda msg, rid=robot_id: self._on_lift_command(rid, msg),
                10,
            )
            self._node.create_subscription(
                Twist,
                f"/robot_{robot_id}/cmd_vel",
                lambda msg, rid=robot_id: self._on_cmd_vel(rid, msg),
                10,
            )
            self._node.create_subscription(
                String,
                f"/robot_{robot_id}/vision/alignment",
                lambda msg, rid=robot_id: self._on_vision_alignment(rid, msg),
                qos_profile_sensor_data,
            )

        self._executor = SingleThreadedExecutor()
        self._executor.add_node(self._node)
        self._spin_thread = threading.Thread(
            target=self._spin_executor, daemon=True
        )
        self._spin_thread.start()

        self._db = ParkingDbReader(
            config.DB_HOST, config.DB_USER, config.DB_PASSWORD, config.DB_NAME
        )
        self._stop_event.clear()
        self._poll_thread = threading.Thread(target=self._poll_loop, daemon=True)
        self._poll_thread.start()

        try:
            parking_map = ParkingMap.load(_default_map_yaml())
            self._map_info = _extract_map_info(parking_map)
            self._map_info["sensors"] = [
                {
                    "id": sensor_id,
                    "zone": zone,
                    "x": x,
                    "y": y,
                    "z": 5.12,
                    "fov_deg": 360,
                    "frame_id": "map",
                }
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
        except (ExternalShutdownException, RCLError):
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
                completed_at = (
                    row["updated_at"]
                    if row["state"] in ("DONE", "FAILED")
                    else None
                )
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
                        completed_at=(
                            completed_at.isoformat(timespec="seconds")
                            if hasattr(completed_at, "isoformat")
                            else str(completed_at)
                            if completed_at is not None
                            else None
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
                    status=(
                        "SAFETY_STOPPED"
                        if (
                            row["robot_id"] in self._recovery_state["robot_ids"]
                            and self._recovery_state["status"]
                            == "SAFETY_STOPPED"
                        )
                        else "RECOVERY_REQUIRED"
                        if (
                            row["robot_id"] in self._recovery_state["robot_ids"]
                            and self._recovery_state["status"]
                            in {"REQUIRED", "BLOCKED"}
                        )
                        else "RECOVERING"
                        if (
                            row["robot_id"] in self._recovery_state["robot_ids"]
                            and self._recovery_state["status"] == "RECOVERING"
                        )
                        else row["status"]
                    ),
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

    def _on_lidar(self, sensor_id: str, message: PointCloud2) -> None:
        now = time.monotonic()
        with self._sensor_lock:
            self._sensor_received[sensor_id].append(now)
            last_snapshot = self._lidar_visualization_updated_at

        # 원본 PointCloud2 변환은 상세 화면에 필요한 1 Hz만 수행한다.
        if last_snapshot is not None and now - last_snapshot < 1.0:
            return

        cloud = point_cloud2.read_points(
            message, field_names=("x", "y", "z"), skip_nans=True
        )
        with self.store.lock:
            slots = [
                slot.model_copy(deep=True)
                for slot in self.store.parking_slots
                if slot.x is not None and slot.y is not None
            ]
        if not slots:
            slots = [
                {
                    "id": node["id"],
                    "x": node["x"],
                    "y": node["y"],
                }
                for node in self._map_info.get("nodes", [])
                if node.get("kind") == "slot"
            ]
        contract = next(
            (
                item for item in _LIDAR_CONTRACTS
                if item[0] == sensor_id
            ),
            (
                sensor_id,
                "주차장 전체",
                0.5,
                0.0,
                "/parking/lidar/points_world",
            ),
        )
        snapshot = build_lidar_visualization(
            cloud,
            slots,
            sensor_id=sensor_id,
            sensor_status="ONLINE",
            topic=contract[4],
            frame_id=message.header.frame_id or "map",
            sensor_x=contract[2],
            sensor_y=contract[3],
            last_seen_sec=0.0,
            source="ROS2_POINTCLOUD",
        )
        with self._sensor_lock:
            self._lidar_visualization = snapshot
            self._lidar_visualization_updated_at = now

    def _on_slot_occupancy(self, message: SlotOccupancyArray) -> None:
        """safety_monitor가 확정한 판정을 웹용 계약으로 캐시한다."""
        status_labels = {
            SlotOccupancy.STATUS_WAITING: "WAITING",
            SlotOccupancy.STATUS_EMPTY: "EMPTY",
            SlotOccupancy.STATUS_OCCUPIED: "OCCUPIED",
            SlotOccupancy.STATUS_UNCERTAIN: "UNCERTAIN",
        }
        measurement_labels = {
            SlotOccupancyArray.MEASUREMENT_WAITING: "WAITING",
            SlotOccupancyArray.MEASUREMENT_OK: "OK",
            SlotOccupancyArray.MEASUREMENT_NO_VALID_POINTS: "NO_VALID_POINTS",
            SlotOccupancyArray.MEASUREMENT_TF_ERROR: "TF_ERROR",
        }
        slots = {
            slot.slot_id: {
                "id": slot.slot_id,
                "status": status_labels.get(slot.status, "WAITING"),
                "point_count": int(slot.point_count),
                "point_threshold": int(slot.point_threshold),
                "height_threshold_m": float(slot.height_threshold_m),
                "x": float(slot.center.x),
                "y": float(slot.center.y),
                "width": float(slot.width),
                "length": float(slot.length),
            }
            for slot in message.slots
        }
        snapshot = {
            "sensor_id": message.sensor_id or "L1",
            "topic": message.source_topic or "/parking/lidar/points_world",
            "frame_id": message.header.frame_id or "map",
            "measurement_status": measurement_labels.get(
                message.measurement_status, "WAITING"
            ),
            "status_message": message.status_message,
            "point_total": int(message.received_point_count),
            "valid_point_count": int(message.filtered_point_count),
            "slot_point_count": int(message.slot_point_count),
            "stabilization_frames": int(message.stabilization_frames),
            "slots": slots,
        }
        if slots:
            first = next(iter(slots.values()))
            snapshot["height_threshold_m"] = first["height_threshold_m"]
            snapshot["point_threshold"] = first["point_threshold"]
        with self._sensor_lock:
            self._slot_occupancy = snapshot
            self._slot_occupancy_updated_at = time.monotonic()

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

    def get_lidar_visualization(self) -> dict:
        statuses = self.get_sensor_status()
        status = statuses[0] if statuses else {
            "id": "L1",
            "status": "OFFLINE",
            "topic": "/parking/lidar/points_world",
            "rate_hz": None,
            "last_seen_sec": None,
        }
        now = time.monotonic()
        with self._sensor_lock:
            snapshot = (
                copy.deepcopy(self._lidar_visualization)
                if self._lidar_visualization is not None
                else None
            )
            occupancy = (
                copy.deepcopy(self._slot_occupancy)
                if self._slot_occupancy is not None
                else None
            )
            occupancy_age = (
                now - self._slot_occupancy_updated_at
                if self._slot_occupancy_updated_at is not None
                else None
            )

        if snapshot is None:
            with self.store.lock:
                slots = [
                    slot.model_copy(deep=True)
                    for slot in self.store.parking_slots
                    if slot.x is not None and slot.y is not None
                ]
            if not slots:
                slots = [
                    {
                        "id": node["id"],
                        "x": node["x"],
                        "y": node["y"],
                    }
                    for node in self._map_info.get("nodes", [])
                    if node.get("kind") == "slot"
                ]
            snapshot = build_lidar_visualization(
                [],
                slots,
                sensor_id=status["id"],
                sensor_status=status["status"],
                topic=status["topic"],
                rate_hz=status["rate_hz"],
                last_seen_sec=status["last_seen_sec"],
                source="ROS2_POINTCLOUD",
            )
        snapshot["sensor_status"] = status["status"]
        snapshot["topic"] = status["topic"]
        snapshot["rate_hz"] = status["rate_hz"]
        snapshot["last_seen_sec"] = status["last_seen_sec"]
        if status["status"] != "ONLINE":
            for slot in snapshot["slots"]:
                slot["status"] = "UNAVAILABLE"
                slot["status_match"] = None
                slot["point_count"] = None
            snapshot["measurement_status"] = "NO_DATA"
            snapshot["status_message"] = (
                "PointCloud2 메시지가 들어오지 않습니다."
            )
            snapshot["coordinate_status"] = "WAITING"
            snapshot["occupied_count"] = 0
            snapshot["empty_count"] = 0
            snapshot["uncertain_count"] = 0
            snapshot["mismatch_count"] = 0
            return snapshot

        if occupancy is None or occupancy_age is None or occupancy_age > 3.0:
            for slot in snapshot["slots"]:
                slot["status"] = "WAITING"
                slot["status_match"] = None
                slot["point_count"] = None
            snapshot["measurement_status"] = "RESULT_WAITING"
            snapshot["status_message"] = (
                "PointCloud2는 수신 중이지만 슬롯 판정 결과를 기다리고 있습니다."
            )
            snapshot["coordinate_status"] = (
                "OK" if snapshot.get("frame_id") == "map" else "CHECK"
            )
            snapshot["valid_point_count"] = None
            snapshot["slot_point_count"] = None
            snapshot["occupied_count"] = 0
            snapshot["empty_count"] = 0
            snapshot["uncertain_count"] = 0
            snapshot["mismatch_count"] = 0
            return snapshot

        occupancy_slots = occupancy.pop("slots")
        snapshot.update(occupancy)
        snapshot["source"] = "ROS2_SHARED_OCCUPANCY"
        snapshot["occupancy_last_seen_sec"] = round(occupancy_age, 2)
        snapshot["coordinate_status"] = (
            "ERROR"
            if snapshot["measurement_status"] == "TF_ERROR"
            else "OK"
            if snapshot.get("frame_id") == "map"
            else "CHECK"
        )
        merged_slots = []
        seen = set()
        for slot in snapshot["slots"]:
            lidar_slot = occupancy_slots.get(slot["id"])
            if lidar_slot is None:
                slot.update(
                    status="WAITING",
                    point_count=None,
                    status_match=None,
                )
            else:
                control_status = slot.get("control_status", "UNKNOWN")
                lidar_status = lidar_slot["status"]
                slot.update(lidar_slot)
                slot["control_status"] = control_status
                slot["status_match"] = (
                    lidar_status == control_status
                    if lidar_status in {"OCCUPIED", "EMPTY"}
                    and control_status in {"OCCUPIED", "EMPTY"}
                    else None
                )
                seen.add(slot["id"])
            merged_slots.append(slot)
        for slot_id, lidar_slot in occupancy_slots.items():
            if slot_id not in seen:
                merged_slots.append({
                    **lidar_slot,
                    "control_status": "UNKNOWN",
                    "status_match": None,
                })
        snapshot["slots"] = merged_slots
        snapshot["total_slots"] = len(merged_slots)
        snapshot["occupied_count"] = sum(
            slot["status"] == "OCCUPIED" for slot in merged_slots
        )
        snapshot["empty_count"] = sum(
            slot["status"] == "EMPTY" for slot in merged_slots
        )
        snapshot["uncertain_count"] = sum(
            slot["status"] == "UNCERTAIN" for slot in merged_slots
        )
        snapshot["mismatch_count"] = sum(
            slot["status_match"] is False for slot in merged_slots
        )
        return snapshot

    def get_cooperative_load_states(self) -> list[CooperativeLoadState]:
        with self.store.lock:
            requests = [
                request.model_copy(deep=True)
                for request in self.store.requests
                if request.status not in TERMINAL_STATUSES
                and len(request.robot_ids) >= 2
            ]
        with self._perception_lock:
            pose_x = dict(self._robot_pose_x)
            axle_x = dict(self._axle_center_x)
            arm_actual = {
                robot_id: dict(values)
                for robot_id, values in self._arm_actual_percent.items()
            }
            lift_command = dict(self._lift_command_percent)
            load_telemetry_received = {
                robot_id: list(received)
                for robot_id, received in self._load_telemetry_received.items()
            }
            robot_motion = {
                robot_id: dict(values)
                for robot_id, values in self._robot_motion.items()
            }
            command_motion = dict(self._robot_command_motion)

        states: list[CooperativeLoadState] = []
        for request in requests:
            lead, follow = request.robot_ids[:2]
            load_samples = sorted(
                received_at
                for robot_id in (lead, follow)
                for received_at in load_telemetry_received.get(robot_id, [])
            )
            telemetry_age = (
                round(time.monotonic() - load_samples[-1], 1)
                if load_samples else None
            )
            telemetry_rate = None
            if len(load_samples) > 1:
                elapsed = load_samples[-1] - load_samples[0]
                if elapsed > 0:
                    telemetry_rate = round(
                        (len(load_samples) - 1) / elapsed, 1
                    )

            def alignment_error(robot_id):
                if robot_id not in pose_x or robot_id not in axle_x:
                    return None
                return round(abs(pose_x[robot_id] - axle_x[robot_id]) * 1000.0, 1)

            support_points = []
            for axle, robot_id, axle_label in (
                ("front", lead, "앞축"),
                ("rear", follow, "뒤축"),
            ):
                for side, joint_names in _SUPPORT_POINT_JOINTS:
                    values = [
                        arm_actual.get(robot_id, {}).get(name)
                        for name in joint_names
                    ]
                    available = [value for value in values if value is not None]
                    actual = round(min(available), 1) if len(available) == 2 else None
                    command = lift_command.get(robot_id)
                    support_points.append(
                        SupportPointState(
                            id=f"{axle}_{side}",
                            label=f"{axle_label} {'좌' if side == 'left' else '우'}",
                            robot_id=robot_id,
                            joint_names=list(joint_names),
                            command_percent=command,
                            actual_percent=actual,
                            supported=actual >= 90.0 if actual is not None else None,
                        )
                    )

            actual_by_robot = {}
            for robot_id in (lead, follow):
                values = list(arm_actual.get(robot_id, {}).values())
                if values:
                    actual_by_robot[robot_id] = sum(values) / len(values)
            synchronized = None
            if lead in actual_by_robot and follow in actual_by_robot:
                synchronized = (
                    abs(actual_by_robot[lead] - actual_by_robot[follow]) <= 5.0
                )
            support_values = [point.supported for point in support_points]
            tire_count = (
                sum(value is True for value in support_values)
                if any(value is not None for value in support_values)
                else None
            )
            all_arm_values = [
                value
                for robot_id in (lead, follow)
                for value in arm_actual.get(robot_id, {}).values()
            ]
            load_anomaly = (
                max(all_arm_values) - min(all_arm_values) > 12.0
                if len(all_arm_values) == 8
                else None
            )
            if load_anomaly:
                synchronized = False

            now_mono = time.monotonic()
            slip_samples = []
            for robot_id in (lead, follow):
                command = command_motion.get(robot_id)
                motion = robot_motion.get(robot_id)
                if (
                    command is None
                    or motion is None
                    or now_mono - command[1] > 1.0
                    or now_mono - motion.get("received_at", 0.0) > 1.0
                ):
                    with self._perception_lock:
                        self._robot_slip_since.pop(robot_id, None)
                    continue
                command_speed = command[0]
                actual_speed = motion.get("speed_mps", 0.0)
                if command_speed >= 0.10:
                    stalled = actual_speed < max(0.03, command_speed * 0.2)
                    with self._perception_lock:
                        if stalled:
                            stalled_since = self._robot_slip_since.setdefault(
                                robot_id, now_mono
                            )
                            slip_samples.append(
                                now_mono - stalled_since >= 0.8
                            )
                        else:
                            self._robot_slip_since.pop(robot_id, None)
                            slip_samples.append(False)
                else:
                    with self._perception_lock:
                        self._robot_slip_since.pop(robot_id, None)
            slip_suspected = (
                any(slip_samples) if slip_samples else None
            )
            stable = (
                synchronized
                and tire_count == 4
                and load_anomaly is not True
                and slip_suspected is not True
                if synchronized is not None and tire_count is not None
                else None
            )
            commands = [
                lift_command[robot_id]
                for robot_id in (lead, follow)
                if robot_id in lift_command
            ]
            has_team_signal = any(
                robot_id in arm_actual
                or robot_id in axle_x
                or robot_id in lift_command
                for robot_id in (lead, follow)
            )
            states.append(
                CooperativeLoadState(
                    request_id=request.id,
                    vehicle_number=request.vehicle_number,
                    lead_robot_id=lead,
                    follow_robot_id=follow,
                    front_alignment_error_mm=alignment_error(lead),
                    rear_alignment_error_mm=alignment_error(follow),
                    support_points=support_points,
                    lift_command_percent=(
                        round(sum(commands) / len(commands), 1)
                        if commands else None
                    ),
                    tire_support_count=tire_count,
                    synchronized=synchronized,
                    stable=stable,
                    # 접촉 센서 실측이 아니라 cmd_vel 대비 융합 위치 속도와
                    # 8개 암 관절 편차로 계산한 "의심" 지표다.
                    slip_suspected=slip_suspected,
                    load_anomaly_suspected=load_anomaly,
                    source=(
                        "MEASURED_ESTIMATED"
                        if has_team_signal
                        else "UNAVAILABLE"
                    ),
                    telemetry_age_sec=telemetry_age,
                    telemetry_rate_hz=telemetry_rate,
                    updated_at=_now(),
                )
            )
        return states

    def get_vision_alignment_states(self) -> list[VisionAlignmentState]:
        now = time.monotonic()
        with self._perception_lock:
            states = []
            for state, received_at in self._vision_alignment.values():
                copy = state.model_copy(deep=True)
                if now - received_at > 2.0:
                    copy.connected = False
                    copy.alignment_state = "NO_DATA"
                states.append(copy)
            return states

    def _on_robot_pose(self, robot_id: str, msg: PoseStamped) -> None:
        now = time.monotonic()
        x = float(msg.pose.position.x)
        plane_y = (
            float(msg.pose.position.y)
            if msg.header.frame_id == "map"
            else float(msg.pose.position.z)
        )
        with self._perception_lock:
            self._robot_pose_x[robot_id] = x
            previous = self._robot_motion.get(robot_id)
            speed = previous.get("speed_mps", 0.0) if previous else 0.0
            if previous is not None:
                dt = now - previous["received_at"]
                if dt > 1e-3:
                    instant = math.hypot(
                        x - previous["x"], plane_y - previous["plane_y"]
                    ) / dt
                    speed = 0.4 * instant + 0.6 * speed
            self._robot_motion[robot_id] = {
                "x": x,
                "plane_y": plane_y,
                "speed_mps": speed,
                "received_at": now,
            }

    def _on_axle_center(self, robot_id: str, msg: PointStamped) -> None:
        with self._perception_lock:
            self._axle_center_x[robot_id] = float(msg.point.x)
            self._load_telemetry_received[robot_id].append(time.monotonic())

    def _on_joint_state(self, robot_id: str, msg: JointState) -> None:
        positions = dict(zip(msg.name, msg.position))
        arms = {
            name: round(
                min(1.0, abs(float(positions[name])) / _ARM_TARGET_RAD) * 100.0,
                1,
            )
            for _side, names in _SUPPORT_POINT_JOINTS
            for name in names
            if name in positions
        }
        if arms:
            with self._perception_lock:
                self._arm_actual_percent[robot_id] = arms
                self._load_telemetry_received[robot_id].append(time.monotonic())

    def _on_lift_command(self, robot_id: str, msg: Float32) -> None:
        with self._perception_lock:
            self._lift_command_percent[robot_id] = round(
                max(0.0, min(1.0, float(msg.data))) * 100.0, 1
            )
            self._load_telemetry_received[robot_id].append(time.monotonic())

    def _on_cmd_vel(self, robot_id: str, msg: Twist) -> None:
        speed = math.hypot(float(msg.linear.x), float(msg.linear.y))
        with self._perception_lock:
            self._robot_command_motion[robot_id] = (
                speed, time.monotonic()
            )

    def _on_vision_alignment(self, robot_id: str, msg: String) -> None:
        try:
            raw = json.loads(msg.data)
        except (TypeError, ValueError):
            if self._node is not None:
                self._node.get_logger().warn(
                    f"{robot_id} vision/alignment JSON 파싱 실패"
                )
            return
        detected = bool(raw.get("detected", False))
        corners = raw.get("marker_corners") or []
        state = VisionAlignmentState(
            robot_id=robot_id,
            camera=str(raw.get("camera", "front")),
            connected=bool(raw.get("connected", True)),
            marker_detected=detected,
            marker_id=raw.get("marker_id") if detected else None,
            distance_m=raw.get("distance_m") if detected else None,
            reprojection_error_px=(
                raw.get("reprojection_error_px") if detected else None
            ),
            marker_corners=corners if detected else [],
            target_center=[0.5, 0.5],
            alignment_state="ADJUSTING" if detected else "SEARCHING",
            source="ARUCO_FUSED",
            updated_at=_now(),
        )
        with self._perception_lock:
            self._vision_alignment[robot_id] = (state, time.monotonic())

    # ------------------------------------------------------------------
    # 토픽 콜백 (rclpy 스핀 스레드에서 호출됨)
    # ------------------------------------------------------------------
    def _on_obstacle_alert(self, msg: ObstacleAlert) -> None:
        if not msg.obstacle_detected:
            # safety_monitor는 PointCloud 프레임마다 현재 판정값을 발행한다.
            # clear 프레임을 받으면 수동 해제 없이 지도/경고를 즉시 정리한다.
            with self.store.lock:
                for alert in self.store.alerts:
                    if (
                        alert.active
                        and alert.category == AlertCategory.OBSTACLE
                    ):
                        alert.active = False
                        recover_obstacle_incident(self.store, alert)
            return
        description = msg.description or "주행 경로에서 장애물이 감지되었습니다."
        zone_ids = []
        if description.startswith("통로 막힘:"):
            zone_ids = [
                zone.strip()
                for zone in description.split(":", 1)[1].split(",")
                if zone.strip()
            ]
        zone_id = ", ".join(zone_ids) or None
        # ObstacleAlert.msg에는 위치 유효성 플래그가 없다. 현재 실제
        # safety_monitor는 통로 막힘 설명과 함께 location을 항상 채운다.
        # 다른 구형 발행자의 기본값 (0, 0)은 실제 위치로 오인하지 않는다.
        has_location = bool(zone_ids) or any(
            abs(value) > 1e-6
            for value in (float(msg.location.x), float(msg.location.y))
        )
        sensor_id = _LIDAR_CONTRACTS[0][0] if len(_LIDAR_CONTRACTS) == 1 else None
        with self.store.lock:
            # safety_monitor 메시지는 특정 이벤트 추가가 아니라 현재 프레임의
            # 전체 장애물 상태다. 감지 구역이 바뀌었으면 이전 프레임의 경보를
            # 남겨 두지 않는다.
            for alert in self.store.alerts:
                if (
                    alert.active
                    and alert.category == AlertCategory.OBSTACLE
                    and (
                        alert.sensor_id != sensor_id
                        or alert.zone_id != zone_id
                    )
                ):
                    alert.active = False
                    recover_obstacle_incident(self.store, alert)
            existing = next(
                (
                    alert
                    for alert in self.store.alerts
                    if alert.active
                    and alert.category == AlertCategory.OBSTACLE
                    and alert.sensor_id == sensor_id
                    and alert.zone_id == zone_id
                ),
                None,
            )
            if existing is not None:
                existing.message = description
                existing.location_x = (
                    float(msg.location.x) if has_location else None
                )
                existing.location_y = (
                    float(msg.location.y) if has_location else None
                )
                open_obstacle_incident(self.store, existing)
                return
            alert = Alert(
                id=self.store.next_alert_id(),
                level=AlertLevel.WARNING,
                category=AlertCategory.OBSTACLE,
                message=description,
                robot_id=None,
                sensor_id=sensor_id,
                zone_id=zone_id,
                location_x=float(msg.location.x) if has_location else None,
                location_y=float(msg.location.y) if has_location else None,
                created_at=_now(),
            )
            self.store.alerts.append(alert)
            open_obstacle_incident(self.store, alert)

    def _on_task_state(self, msg: TaskState) -> None:
        with self._map_lock:
            if msg.state in {"LIFTING", "PICKED_UP"}:
                self._lifted_tasks.add(msg.task_id)
            table = (
                _TASK_STATE_AFTER_LIFT
                if msg.task_id in self._lifted_tasks
                else _TASK_STATE_BEFORE_LIFT
            )
            status = table.get(msg.state)
            if status is not None:
                self._fine_status[msg.task_id] = status

        if msg.state == "FAILED" and not self._is_safety_task_termination(msg):
            with self.store.lock:
                message = (
                    f"{msg.robot_id} 작업 실패"
                    f"{f' ({msg.current_step})' if msg.current_step else ''}"
                )
                duplicate = any(
                    alert.active
                    and alert.category == AlertCategory.ROBOT_ERROR
                    and alert.robot_id == (msg.robot_id or None)
                    and alert.message == message
                    for alert in self.store.alerts
                )
                if duplicate:
                    return
                self.store.alerts.append(
                    Alert(
                        id=self.store.next_alert_id(),
                        level=AlertLevel.ERROR,
                        category=AlertCategory.ROBOT_ERROR,
                        message=message,
                        robot_id=msg.robot_id or None,
                        created_at=_now(),
                    )
                )

    def _is_safety_task_termination(self, msg: TaskState) -> bool:
        """비상정지·취소로 끝난 action을 로봇 고장으로 오분류하지 않는다."""
        detail = (msg.current_step or "").strip().lower()
        if any(
            marker in detail
            for marker in _SAFETY_TASK_TERMINATION_MARKERS
        ):
            return True
        return (
            self._safety_state["state"] != "NORMAL"
            and msg.task_id in self._safety_state["affected_task_ids"]
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
            if msg.affected_task_ids and not self._recovery_state["robot_ids"]:
                affected_external_ids = set(msg.affected_task_ids)
                affected_requests = [
                    request
                    for request in self.store.requests
                    if request.external_task_id in affected_external_ids
                ]
                recovery_robot_ids = list(
                    dict.fromkeys(
                        robot_id
                        for request in affected_requests
                        for robot_id in request.robot_ids
                    )
                )
                if recovery_robot_ids:
                    self._recovery_state = {
                        "status": "SAFETY_STOPPED",
                        "robot_ids": recovery_robot_ids,
                        "source_request_ids": [
                            request.id for request in affected_requests
                        ],
                        "load_state": "LOAD_STATE_REQUIRES_INSPECTION",
                        "control_available": False,
                        "message": (
                            "중앙 비상정지의 영향 로봇을 확인했습니다. "
                            "현재 위치를 유지하며 별도 도크 복귀가 필요합니다."
                        ),
                        "started_at": None,
                        "completed_at": None,
                    }
            if self._recovery_state["robot_ids"]:
                if msg.state == "STOPPED_LATCHED":
                    self._recovery_state.update(
                        status="SAFETY_STOPPED",
                        message=(
                            "로봇은 현재 위치에서 안전 정지했습니다. "
                            "원 작업은 취소되며 별도 복귀가 필요합니다."
                        ),
                    )
                elif msg.state == "READY_FOR_OPERATION":
                    self._recovery_state.update(
                        status="REQUIRED",
                        message=(
                            "현장 점검이 완료되었습니다. 제한 안전 복귀 "
                            "제어기를 통해 도크 복귀를 먼저 완료해야 합니다."
                        ),
                    )
                elif (
                    msg.state == "NORMAL"
                    and self._recovery_state["status"] == "SAFETY_STOPPED"
                ):
                    self._recovery_state.update(
                        status="REQUIRED",
                        message=(
                            "운영 복귀는 승인되었지만 영향 로봇의 도크 복귀는 "
                            "완료되지 않았습니다. 안전 복귀 제어기 연결이 필요합니다."
                        ),
                    )
                recovery_robot_status = {
                    "SAFETY_STOPPED": "SAFETY_STOPPED",
                    "REQUIRED": "RECOVERY_REQUIRED",
                    "BLOCKED": "RECOVERY_REQUIRED",
                    "RECOVERING": "RECOVERING",
                }.get(self._recovery_state["status"])
                if recovery_robot_status:
                    for robot_id in self._recovery_state["robot_ids"]:
                        robot = self.store.find_robot(robot_id)
                        if robot is not None:
                            robot.status = recovery_robot_status
                            robot.current_task_id = None
            # TaskState FAILED가 SafetyState보다 먼저 도착해 이미 만들어진
            # "작업 실패" 경고도, 영향 로봇의 안전정지 결과라면 중복 노출하지
            # 않는다. 구동부/통신 등 구체적인 로봇 고장 알림은 그대로 둔다.
            affected_recovery_robots = set(
                self._recovery_state["robot_ids"]
            )
            if msg.state != "NORMAL" and affected_recovery_robots:
                for item in self.store.alerts:
                    if (
                        item.active
                        and item.category == AlertCategory.ROBOT_ERROR
                        and item.robot_id in affected_recovery_robots
                        and item.message.startswith(
                            f"{item.robot_id} 작업 실패"
                        )
                    ):
                        item.active = False
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
                    if self.recovery_pending:
                        alert.level = AlertLevel.WARNING
                        alert.message = (
                            "운영 복귀가 승인되었지만 대상 로봇은 복구 대기입니다. "
                            "실제 도크 복귀 제어기를 연결해주세요."
                        )
                    else:
                        alert.active = False
                return

            if msg.state == "READY_FOR_OPERATION":
                message = (
                    "안전 점검이 승인되었습니다. 로봇은 계속 정지 상태이며 "
                    "대상 로봇의 제한 안전 복귀가 필요합니다."
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
                        level=(
                            AlertLevel.WARNING
                            if msg.state == "READY_FOR_OPERATION"
                            else AlertLevel.ERROR
                        ),
                        category=AlertCategory.EMERGENCY_STOP,
                        message=message,
                        robot_id=None,
                        created_at=_now(),
                    )
                )
            else:
                alert.level = (
                    AlertLevel.WARNING
                    if msg.state == "READY_FOR_OPERATION"
                    else AlertLevel.ERROR
                )
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
        if self.recovery_pending:
            raise DataSourceError(
                "안전 복귀가 완료되지 않은 로봇이 있습니다. "
                "실제 도크 복귀와 위치 확인 후 새 작업을 등록해주세요.",
                status_code=423,
            )
        with self.store.lock:
            obstacle = blocking_obstacle(
                self.store.alerts, payload.request_type
            )
        if obstacle is not None:
            raise DataSourceError(
                blocking_request_message(obstacle, payload.request_type),
                status_code=409,
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
            if self.safety_state["state"] == "STOPPED_LATCHED":
                return len(active_requests)
            previous_recovery_robot_ids = (
                list(self._recovery_state["robot_ids"])
                if self.recovery_pending
                else []
            )
            previous_source_ids = (
                list(self._recovery_state["source_request_ids"])
                if previous_recovery_robot_ids
                else []
            )
            recovery_source_requests = [
                request
                for request in self.store.requests
                if request.id in previous_source_ids
            ]
            task_ids = list(
                dict.fromkeys(
                    request.external_task_id
                    for request in [
                        *active_requests,
                        *recovery_source_requests,
                    ]
                    if request.external_task_id
                )
            )
            recovery_robot_ids = list(
                dict.fromkeys(
                    [
                        *previous_recovery_robot_ids,
                        *[
                            robot_id
                            for request in active_requests
                            for robot_id in request.robot_ids
                        ],
                    ]
                )
            )
            load_present = any(
                request.status
                in {
                    RequestStatus.LIFTING,
                    RequestStatus.MOVING_TO_SLOT,
                    RequestStatus.RETURNING,
                }
                for request in active_requests
            )

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
        with self.store.lock:
            self._recovery_state = {
                "status": (
                    "SAFETY_STOPPED" if recovery_robot_ids else "NONE"
                ),
                "robot_ids": recovery_robot_ids,
                "source_request_ids": list(
                    dict.fromkeys(
                        [
                            *previous_source_ids,
                            *[request.id for request in active_requests],
                        ]
                    )
                ),
                "load_state": (
                    "LOAD_REQUIRES_CLEARANCE"
                    if load_present
                    or self._recovery_state["load_state"]
                    == "LOAD_REQUIRES_CLEARANCE"
                    else "CLEAR"
                ),
                "control_available": False,
                "message": (
                    "로봇은 현재 위치에서 안전 정지했습니다. 실제 도크 복귀 "
                    "제어기가 연결되기 전에는 대기 상태로 표시하지 않습니다."
                    if recovery_robot_ids
                    else ""
                ),
                "started_at": None,
                "completed_at": None,
            }
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
        if self._recovery_state["status"] == "SAFETY_STOPPED":
            self._recovery_state.update(
                status="REQUIRED",
                message=(
                    "점검이 완료되었습니다. 실제 로봇의 제한 안전 복귀 "
                    "제어기를 연결해 도크 복귀를 먼저 실행해야 합니다."
                ),
            )
        return {
            "state": response.state,
            "stop_epoch": int(response.stop_epoch),
            "message": response.message,
            "blockers": list(response.blockers),
        }

    def approve_operation(self, payload) -> dict:
        if self.recovery_pending:
            raise DataSourceError(
                "대상 로봇의 도크 복귀와 위치 확인을 먼저 완료해주세요.",
                status_code=409,
            )
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

    def start_safe_recovery(self, payload: RobotRecoveryRequest) -> dict:
        if self._safety_state["state"] != "READY_FOR_OPERATION":
            raise DataSourceError(
                "현장 안전 점검 승인 후에만 제한 안전 복귀를 시작할 수 있습니다.",
                status_code=409,
            )
        if self._recovery_state["status"] != "REQUIRED":
            raise DataSourceError(
                "안전 복귀가 필요한 로봇이 없습니다.", status_code=409
            )
        raise DataSourceError(
            "실제 로봇 안전 복귀 제어기가 아직 연결되지 않았습니다. "
            "현재 위치에서 정지를 유지하며 fusion 제어 측에 도크 복귀 action을 "
            "연결한 뒤 실행해주세요.",
            status_code=503,
        )

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
            self.store.safety_incidents.clear()
