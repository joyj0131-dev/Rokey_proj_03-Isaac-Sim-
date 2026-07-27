"""Mock 데이터 소스.

기존 main.py의 인메모리 시뮬레이션 로직을 이관한 구현체.
추가로 장애물 감지/로봇 오류 이벤트 시뮬레이션을 지원한다.
"""

import math
import threading
import time
from datetime import datetime

from core.datasource import DataSource, DataSourceError
from core.lidar_visualization import (
    build_lidar_visualization,
    build_mock_pointcloud,
)
from core.obstacle_scope import blocking_obstacle, blocking_request_message
from core.safety_incident import (
    open_obstacle_incident,
    recover_obstacle_incident,
)
from core.models import (
    STATUS_TRANSITIONS,
    TERMINAL_STATUSES,
    Alert,
    AlertCategory,
    AlertLevel,
    CooperativeLoadState,
    OperationApprovalRequest,
    ParkingRequest,
    ParkingRequestCreate,
    ParkingSlot,
    RequestStatus,
    RequestType,
    Robot,
    RobotRecoveryRequest,
    SafetyResetRequest,
    SupportPointState,
    VisionAlignmentState,
)
from core.state_store import StateStore

#: parking_map.yaml v4와 동일한 입·출차 구역 및 로봇 도크 좌표.
_ENTRY_VEHICLE_ZONE = (-21.0, -7.075)
_EXIT_VEHICLE_ZONE = (-21.0, 7.075)
_DOCK_ENTRY_1 = (-3.2, -2.2)
_DOCK_ENTRY_2 = (-1.2, -2.2)
_DOCK_EXIT_1 = (-3.2, 2.2)
_DOCK_EXIT_2 = (-1.2, 2.2)
_ENTRY_LANE_Y = -6.875
_EXIT_LANE_Y = 6.875
_CROSSING_X = -2.5

#: id, status, battery, (x, y) — 로봇은 각자의 도크 위치에서 시작.
_DEFAULT_ROBOTS = [
    ("entry_lead", "IDLE", 92, _DOCK_ENTRY_1),
    ("entry_follow", "IDLE", 88, _DOCK_ENTRY_2),
    ("exit_lead", "IDLE", 84, _DOCK_EXIT_1),
    ("exit_follow", "IDLE", 80, _DOCK_EXIT_2),
]

_WAITING_DOCK_BY_ROBOT = {
    "entry_lead": _DOCK_ENTRY_1,
    "entry_follow": _DOCK_ENTRY_2,
    "exit_lead": _DOCK_EXIT_1,
    "exit_follow": _DOCK_EXIT_2,
}

#: id, status, vehicle, (x, y), is_accessible
_DEFAULT_SLOTS = [
    ("A1", "OCCUPIED", "12가3456", (2.8, 0.0), False),
    ("A2", "EMPTY", None, (6.2, 0.0), False),
    ("A3", "EMPTY", None, (9.6, 0.0), False),
]


def _now() -> str:
    return datetime.now().isoformat(timespec="seconds")


class MockDataSource(DataSource):
    supports_mock_controls = True
    mock_auto_advance = True

    _TICK_SEC = 0.1
    _ASSIGN_HOLD_SEC = 0.7
    _LIFT_HOLD_SEC = 1.2
    _MOVE_SPEED_MPS = 4.0
    _PAIR_HALF_GAP_M = 1.2

    def __init__(self, store: StateStore) -> None:
        super().__init__(store)
        self._stop_event = threading.Event()
        self._auto_thread: threading.Thread | None = None
        self._stage_started: dict[int, float] = {}
        self._route_progress: dict[int, tuple[RequestStatus, int]] = {}
        self._recovery_routes: dict[str, list[tuple[float, float]]] = {}
        self._recovery_route_progress: dict[str, int] = {}
        self._load_defaults(clear_counters=False)

    def start(self) -> None:
        if self._auto_thread is not None and self._auto_thread.is_alive():
            return
        self._stop_event.clear()
        self._auto_thread = threading.Thread(
            target=self._auto_loop,
            name="parking-mock-auto-flow",
            daemon=True,
        )
        self._auto_thread.start()

    def stop(self) -> None:
        self._stop_event.set()
        if self._auto_thread is not None:
            self._auto_thread.join(timeout=2)
        self._auto_thread = None

    # ------------------------------------------------------------------
    # 초기화
    # ------------------------------------------------------------------
    def _load_defaults(self, clear_counters: bool) -> None:
        with self.store.lock:
            self.store.robots.clear()
            self.store.robots.extend(
                Robot(id=robot_id, status=status, battery=battery, x=x, y=y)
                for robot_id, status, battery, (x, y) in _DEFAULT_ROBOTS
            )

            self.store.parking_slots.clear()
            self.store.parking_slots.extend(
                ParkingSlot(
                    id=slot_id,
                    status=status,
                    vehicle_number=vehicle,
                    x=x,
                    y=y,
                    is_accessible=accessible,
                )
                for slot_id, status, vehicle, (x, y), accessible in _DEFAULT_SLOTS
            )

            self.store.requests.clear()
            self.store.alerts.clear()
            self.store.safety_incidents.clear()
            self._stage_started.clear()
            self._route_progress.clear()
            self._recovery_routes.clear()
            self._recovery_route_progress.clear()
            self._recovery_state = {
                "status": "NONE",
                "robot_ids": [],
                "source_request_ids": [],
                "load_state": "CLEAR",
                "control_available": True,
                "message": "",
                "started_at": None,
                "completed_at": None,
            }

    def get_map_info(self) -> dict:
        nodes = [
            {"id": "A1", "kind": "slot", "x": 2.8, "y": 0.0},
            {"id": "A2", "kind": "slot", "x": 6.2, "y": 0.0},
            {"id": "A3", "kind": "slot", "x": 9.6, "y": 0.0},
            {"id": "entry_outer", "kind": "junction", "x": -21.0, "y": -7.075},
            {"id": "entry_gate", "kind": "junction", "x": -12.55, "y": -7.075},
            {"id": "entry_wait", "kind": "junction", "x": -8.5, "y": -7.075},
            {"id": "crossing_entry", "kind": "junction", "x": -2.5, "y": -6.875},
            {"id": "entry_a1", "kind": "junction", "x": 2.8, "y": -6.875},
            {"id": "entry_a2", "kind": "junction", "x": 6.2, "y": -6.875},
            {"id": "entry_a3", "kind": "junction", "x": 9.6, "y": -6.875},
            {"id": "crossing_exit", "kind": "junction", "x": -2.5, "y": 6.875},
            {"id": "exit_wait", "kind": "junction", "x": -8.5, "y": 7.075},
            {"id": "exit_gate", "kind": "junction", "x": -12.55, "y": 7.075},
            {"id": "exit_outer", "kind": "junction", "x": -21.0, "y": 7.075},
            {"id": "exit_a1", "kind": "junction", "x": 2.8, "y": 6.875},
            {"id": "exit_a2", "kind": "junction", "x": 6.2, "y": 6.875},
            {"id": "exit_a3", "kind": "junction", "x": 9.6, "y": 6.875},
            {"id": "dock_entry_1", "kind": "dock", "role": "entry", "x": -3.2, "y": -2.2},
            {"id": "dock_entry_2", "kind": "dock", "role": "entry", "x": -1.2, "y": -2.2},
            {"id": "dock_exit_1", "kind": "dock", "role": "exit", "x": -3.2, "y": 2.2},
            {"id": "dock_exit_2", "kind": "dock", "role": "exit", "x": -1.2, "y": 2.2},
        ]
        return {
            "layout": "v4",
            "nodes": nodes,
            "docks": [
                {"id": "dock_entry_1", "role": "entry", "x": _DOCK_ENTRY_1[0], "y": _DOCK_ENTRY_1[1]},
                {"id": "dock_entry_2", "role": "entry", "x": _DOCK_ENTRY_2[0], "y": _DOCK_ENTRY_2[1]},
                {"id": "dock_exit_1", "role": "exit", "x": _DOCK_EXIT_1[0], "y": _DOCK_EXIT_1[1]},
                {"id": "dock_exit_2", "role": "exit", "x": _DOCK_EXIT_2[0], "y": _DOCK_EXIT_2[1]},
            ],
            "vehicle_zones": [
                {"id": "exit_outer", "role": "exit", "label": "출차 차량 대기 구역", "x": _EXIT_VEHICLE_ZONE[0], "y": _EXIT_VEHICLE_ZONE[1]},
                {"id": "entry_outer", "role": "entry", "label": "입차 차량 대기 구역", "x": _ENTRY_VEHICLE_ZONE[0], "y": _ENTRY_VEHICLE_ZONE[1]},
            ],
            "entrance": {"x": _ENTRY_VEHICLE_ZONE[0], "y": _ENTRY_VEHICLE_ZONE[1]},
            "sensors": [
                {"id": "L1", "zone": "주차장 전체", "x": 0.5, "y": 0.0},
            ],
        }

    def get_sensor_status(self) -> list[dict]:
        return [
            {
                "id": "L1",
                "topic": "/parking/lidar/points_world",
                "status": "MOCK",
                "rate_hz": None,
                "last_seen_sec": None,
            },
        ]

    def get_lidar_visualization(self) -> dict:
        with self.store.lock:
            slots = [
                slot.model_copy(deep=True)
                for slot in self.store.parking_slots
            ]
        points = build_mock_pointcloud(slots)
        return build_lidar_visualization(
            points,
            slots,
            sensor_status="MOCK",
            rate_hz=1.0,
            last_seen_sec=0.0,
            source="MOCK_SAMPLE",
        )

    def _mock_lift_progress(self, request: ParkingRequest) -> float:
        """현재 단계에서의 리프트 전개율. 실제 센서값이 아닌 Mock 시나리오 값."""
        if request.status == RequestStatus.LIFTING:
            age = time.monotonic() - self._stage_started.get(
                request.id, time.monotonic()
            )
            return max(0.0, min(1.0, age / self._LIFT_HOLD_SEC))
        if request.status == RequestStatus.MOVING_TO_SLOT:
            return 1.0
        return 0.0

    def _mock_alignment_error(
        self, request: ParkingRequest, robot_index: int
    ) -> float | None:
        if len(request.robot_ids) != 2:
            return None
        robot = self.store.find_robot(request.robot_ids[robot_index])
        route = self._paired_route(request)
        if request.status == RequestStatus.APPROACHING and robot and route:
            target = route[-1][robot_index]
            if robot.x is None or robot.y is None:
                return None
            # 목표까지의 이동 거리 자체를 mm 정렬 오차로 표시하면 접근 초기에
            # 10~20m가 차축 오차처럼 보인다. Mock에서는 남은 거리에 따라
            # 40mm 안팎에서 최종 8/11mm로 수렴하는 정렬 잔차를 만든다.
            distance_m = math.hypot(
                target[0] - robot.x, target[1] - robot.y
            )
            settled_error = 8.0 if robot_index == 0 else 11.0
            return round(
                settled_error + min(32.0, distance_m * 1.8), 1
            )
        if request.status in {
            RequestStatus.LIFTING,
            RequestStatus.MOVING_TO_SLOT,
        }:
            return 8.0 if robot_index == 0 else 11.0
        return None

    def get_cooperative_load_states(self) -> list[CooperativeLoadState]:
        now = _now()
        states: list[CooperativeLoadState] = []
        with self.store.lock:
            for request in self.store.requests:
                if request.status in TERMINAL_STATUSES or len(request.robot_ids) != 2:
                    continue
                lift = self._mock_lift_progress(request)
                actual = round(lift * 100.0, 1)
                supported = lift >= 0.9
                support_points = [
                    SupportPointState(
                        id=point_id,
                        label=label,
                        robot_id=request.robot_ids[robot_index],
                        joint_names=joint_names,
                        command_percent=100.0 if lift > 0 else 0.0,
                        actual_percent=actual,
                        supported=supported,
                    )
                    for point_id, label, robot_index, joint_names in (
                        ("front_left", "앞축 좌", 0, [
                            "arm_left_front_joint", "arm_left_rear_joint"
                        ]),
                        ("front_right", "앞축 우", 0, [
                            "arm_right_front_joint", "arm_right_rear_joint"
                        ]),
                        ("rear_left", "뒤축 좌", 1, [
                            "arm_left_front_joint", "arm_left_rear_joint"
                        ]),
                        ("rear_right", "뒤축 우", 1, [
                            "arm_right_front_joint", "arm_right_rear_joint"
                        ]),
                    )
                ]
                front_error = self._mock_alignment_error(request, 0)
                rear_error = self._mock_alignment_error(request, 1)
                aligned = (
                    front_error <= 30.0 and rear_error <= 30.0
                    if front_error is not None and rear_error is not None
                    else None
                )
                tire_count = sum(point.supported is True for point in support_points)
                synchronized = (
                    aligned and tire_count in {0, 4}
                    if aligned is not None
                    else None
                )
                stable = (
                    synchronized and tire_count == 4
                    if lift > 0 and synchronized is not None
                    else None
                )
                states.append(
                    CooperativeLoadState(
                        request_id=request.id,
                        vehicle_number=request.vehicle_number,
                        lead_robot_id=request.robot_ids[0],
                        follow_robot_id=request.robot_ids[1],
                        front_alignment_error_mm=front_error,
                        rear_alignment_error_mm=rear_error,
                        support_points=support_points,
                        lift_command_percent=100.0 if lift > 0 else 0.0,
                        vehicle_rise_mm=round(30.0 * lift, 1),
                        pitch_deg=round(0.4 * (1.0 - lift), 2) if lift > 0 else 0.0,
                        roll_deg=round(0.2 * (1.0 - lift), 2) if lift > 0 else 0.0,
                        tire_support_count=tire_count,
                        synchronized=synchronized,
                        stable=stable,
                        slip_suspected=False,
                        load_anomaly_suspected=False,
                        source="MOCK",
                        telemetry_age_sec=0.0,
                        telemetry_rate_hz=10.0,
                        updated_at=now,
                    )
                )
        return states

    def get_vision_alignment_states(self) -> list[VisionAlignmentState]:
        now = _now()
        with self.store.lock:
            active = next(
                (
                    request
                    for request in reversed(self.store.requests)
                    if request.status not in TERMINAL_STATUSES and request.robot_ids
                ),
                None,
            )
            if active is None:
                return []
            robot_id = active.robot_ids[0]
            has_detection = active.status in {
                RequestStatus.APPROACHING,
                RequestStatus.LIFTING,
            }
            if not has_detection:
                return [
                    VisionAlignmentState(
                        robot_id=robot_id,
                        connected=True,
                        marker_detected=False,
                        alignment_state="SEARCHING",
                        source="MOCK",
                        updated_at=now,
                    )
                ]
            alignment_error = self._mock_alignment_error(active, 0)
            lateral = 14.0 if alignment_error is None else min(85.0, alignment_error * 0.12)
            aligned = active.status == RequestStatus.LIFTING and lateral <= 20.0
            center_x = max(0.25, min(0.75, 0.5 + lateral / 1000.0))
            return [
                VisionAlignmentState(
                    robot_id=robot_id,
                    connected=True,
                    marker_detected=True,
                    marker_id=32,
                    distance_m=0.84,
                    reprojection_error_px=0.72,
                    lateral_error_mm=round(lateral, 1),
                    longitudinal_error_mm=-8.0,
                    yaw_error_deg=0.7,
                    marker_corners=[
                        [center_x - 0.11, 0.30],
                        [center_x + 0.11, 0.30],
                        [center_x + 0.10, 0.64],
                        [center_x - 0.10, 0.64],
                    ],
                    target_center=[0.5, 0.47],
                    alignment_state="ALIGNED" if aligned else "ADJUSTING",
                    source="MOCK",
                    updated_at=now,
                )
            ]

    def reset(self) -> None:
        if self._emergency_stop_active or self.recovery_pending:
            raise DataSourceError(
                "비상정지 또는 로봇 안전 복귀 중에는 화면 초기화를 사용할 수 없습니다.",
                status_code=423,
            )
        self._load_defaults(clear_counters=True)

    # ------------------------------------------------------------------
    # 요청 등록
    # ------------------------------------------------------------------
    def create_request(self, payload: ParkingRequestCreate) -> ParkingRequest:
        vehicle_number = payload.vehicle_number.strip()

        if not vehicle_number:
            raise DataSourceError("차량 번호를 입력해주세요.", status_code=400)

        with self.store.lock:
            if self._emergency_stop_active:
                raise DataSourceError(
                    "비상정지 상태에서는 새 작업을 등록할 수 없습니다.",
                    status_code=423,
                )
            if self.recovery_pending:
                raise DataSourceError(
                    "안전 복귀가 완료되지 않은 로봇이 있습니다. "
                    "도크 복귀와 위치 확인 후 새 작업을 등록해주세요.",
                    status_code=423,
                )
            obstacle = blocking_obstacle(
                self.store.alerts, payload.request_type
            )
            if obstacle is not None:
                raise DataSourceError(
                    blocking_request_message(obstacle, payload.request_type),
                    status_code=409,
                )
            if payload.request_type == RequestType.PARK_IN:
                already_parked = next(
                    (
                        slot
                        for slot in self.store.parking_slots
                        if slot.vehicle_number == vehicle_number
                        and slot.status in {"RESERVED", "OCCUPIED"}
                    ),
                    None,
                )
                if already_parked:
                    raise DataSourceError(
                        "이미 입차 또는 예약된 차량입니다.", status_code=409
                    )

                slot = next(
                    (s for s in self.store.parking_slots if s.status == "EMPTY"),
                    None,
                )
                if slot is None:
                    raise DataSourceError(
                        "사용 가능한 주차면이 없습니다.", status_code=409
                    )

                slot.status = "RESERVED"
                slot.vehicle_number = vehicle_number
                selected_slot_id = slot.id
            else:
                slot = next(
                    (
                        s
                        for s in self.store.parking_slots
                        if s.status == "OCCUPIED"
                        and s.vehicle_number == vehicle_number
                    ),
                    None,
                )
                if slot is None:
                    raise DataSourceError(
                        "주차된 차량을 찾을 수 없습니다.", status_code=404
                    )
                selected_slot_id = slot.id

            team_prefix = (
                "entry_" if payload.request_type == RequestType.PARK_IN else "exit_"
            )
            idle_robots = [
                robot
                for robot in self.store.robots
                if robot.status == "IDLE" and robot.id.startswith(team_prefix)
            ][:2]
            has_robot_pair = len(idle_robots) == 2

            request = ParkingRequest(
                id=self.store.next_request_id(),
                request_type=payload.request_type,
                vehicle_number=vehicle_number,
                slot_id=selected_slot_id,
                robot_id=idle_robots[0].id if has_robot_pair else None,
                robot_ids=(
                    [robot.id for robot in idle_robots] if has_robot_pair else []
                ),
                status=(
                    RequestStatus.ROBOT_ASSIGNED
                    if has_robot_pair
                    else RequestStatus.WAITING
                ),
                created_at=_now(),
            )
            self.store.requests.append(request)
            self._stage_started[request.id] = time.monotonic()

            for robot in idle_robots if has_robot_pair else []:
                robot.status = "BUSY"
                robot.current_task_id = request.id

            return request.model_copy(deep=True)

    # ------------------------------------------------------------------
    # 단계 진행 (Mock 제어)
    # ------------------------------------------------------------------
    def advance_request(self, request_id: int) -> ParkingRequest:
        with self.store.lock:
            if self._emergency_stop_active:
                raise DataSourceError(
                    "비상정지 상태에서는 작업을 진행할 수 없습니다.",
                    status_code=423,
                )
            request = self.store.find_request(request_id)

            if request is None:
                raise DataSourceError("요청을 찾을 수 없습니다.", status_code=404)

            if request.status not in STATUS_TRANSITIONS:
                return request.model_copy(deep=True)

            # 협업 로봇 중 한 대라도 오류면 작업 전체를 진행하지 않는다.
            for robot_id in request.robot_ids:
                robot = self.store.find_robot(robot_id)
                if robot and robot.status == "ERROR":
                    raise DataSourceError(
                        f"{robot.id} 로봇이 오류 상태입니다. 알림을 해제한 뒤 진행해주세요.",
                        status_code=409,
                    )

            if request.status == RequestStatus.WAITING:
                team_prefix = (
                    "entry_"
                    if request.request_type == RequestType.PARK_IN
                    else "exit_"
                )
                idle_robots = [
                    robot
                    for robot in self.store.robots
                    if robot.status == "IDLE" and robot.id.startswith(team_prefix)
                ][:2]
                if len(idle_robots) < 2:
                    raise DataSourceError(
                        "협업 운반에 필요한 대기 로봇 2대가 없습니다.", status_code=409
                    )

                request.robot_id = idle_robots[0].id
                request.robot_ids = [robot.id for robot in idle_robots]
                for robot in idle_robots:
                    robot.status = "BUSY"
                    robot.current_task_id = request.id

            request.status = STATUS_TRANSITIONS[request.status]
            self._stage_started[request.id] = time.monotonic()
            self._route_progress.pop(request.id, None)

            if request.status == RequestStatus.RETURNING:
                # 차량 입·출차 결과는 먼저 반영하되, 로봇 작업은 대기 구역에
                # 경로를 따라 복귀할 때까지 완료로 처리하지 않는다.
                self._apply_parking_result(request)

            if request.status == RequestStatus.COMPLETED:
                request.completed_at = _now()
                self._complete_request(request)

            return request.model_copy(deep=True)

    def _formation(self, center: tuple[float, float]) -> list[tuple[float, float]]:
        """차량 중심을 기준으로 R1/R2가 유지할 앞·뒤 상대 위치."""
        x, y = center
        return [
            (x, y - self._PAIR_HALF_GAP_M),
            (x, y + self._PAIR_HALF_GAP_M),
        ]

    def _paired_route(self, request: ParkingRequest) -> list[list[tuple[float, float]]]:
        """각 waypoint마다 두 로봇의 목표 좌표를 반환한다.

        주차면과 입구 사이를 곧장 대각선으로 잇지 않고 y=0 중앙 통로를
        먼저 따라간 뒤 목표 x에서 주차면 방향으로 진입한다.
        """
        slot = self.store.find_slot(request.slot_id) if request.slot_id else None
        if slot is None or slot.x is None or slot.y is None:
            return []

        is_entry = request.request_type == RequestType.PARK_IN
        waiting_docks = (
            [_DOCK_ENTRY_1, _DOCK_ENTRY_2]
            if is_entry
            else [_DOCK_EXIT_1, _DOCK_EXIT_2]
        )
        vehicle_zone = _ENTRY_VEHICLE_ZONE if is_entry else _EXIT_VEHICLE_ZONE
        lane_y = _ENTRY_LANE_Y if is_entry else _EXIT_LANE_Y
        dock_to_lane = self._formation((_CROSSING_X, lane_y))
        slot_center = (slot.x, slot.y)
        slot_lane = (slot.x, lane_y)

        if request.status == RequestStatus.APPROACHING:
            if is_entry:
                return [dock_to_lane, self._formation(vehicle_zone)]
            return [
                dock_to_lane,
                self._formation(slot_lane),
                self._formation(slot_center),
            ]

        if request.status == RequestStatus.MOVING_TO_SLOT:
            if is_entry:
                return [
                    self._formation((_CROSSING_X, lane_y)),
                    self._formation(slot_lane),
                    self._formation(slot_center),
                ]
            return [
                self._formation(slot_lane),
                self._formation((_CROSSING_X, lane_y)),
                self._formation(vehicle_zone),
            ]

        if request.status == RequestStatus.RETURNING:
            if is_entry:
                return [
                    self._formation(slot_lane),
                    dock_to_lane,
                    waiting_docks,
                ]
            return [dock_to_lane, waiting_docks]
        return []

    def _move_robot(self, robot: Robot, target: tuple[float, float], elapsed: float) -> bool:
        if robot.x is None or robot.y is None:
            robot.x, robot.y = target
            return True
        dx = target[0] - robot.x
        dy = target[1] - robot.y
        distance = math.hypot(dx, dy)
        step = self._MOVE_SPEED_MPS * elapsed
        if distance <= max(step, 0.02):
            robot.x, robot.y = target
            return True

        ratio = step / distance
        robot.x += dx * ratio
        robot.y += dy * ratio
        return False

    def _move_pair_along_route(self, request: ParkingRequest, elapsed: float) -> bool:
        robots = [
            self.store.find_robot(robot_id) for robot_id in request.robot_ids
        ]
        if len(robots) != 2 or any(robot is None for robot in robots):
            return False
        if any(robot.status in {"ERROR", "OFFLINE"} for robot in robots):
            return False
        # 협업 로봇 중 하나의 경로에서 장애물이 감지되면 두 로봇을 함께
        # 정지한다. 경보 해제 후 route_progress를 유지한 채 남은 경로부터
        # 다시 진행한다.
        obstacle_active = any(
            alert.active
            and alert.category == AlertCategory.OBSTACLE
            and (alert.robot_id is None or alert.robot_id in request.robot_ids)
            for alert in self.store.alerts
        )
        if obstacle_active:
            return False

        route = self._paired_route(request)
        if not route:
            return False

        stage, index = self._route_progress.get(
            request.id, (request.status, 0)
        )
        if stage != request.status:
            index = 0
        if index >= len(route):
            return True

        targets = route[index]
        reached = [
            self._move_robot(robot, target, elapsed)
            for robot, target in zip(robots, targets)
        ]
        if all(reached):
            index += 1
        self._route_progress[request.id] = (request.status, index)
        return index >= len(route)

    def _auto_loop(self) -> None:
        """실제 관제 흐름처럼 Mock 요청·로봇 위치를 시간에 따라 진행한다."""
        previous = time.monotonic()
        while not self._stop_event.wait(self._TICK_SEC):
            now = time.monotonic()
            elapsed = max(0.0, now - previous)
            previous = now
            advance_ids: list[int] = []

            with self.store.lock:
                if self._recovery_state["status"] == "RECOVERING":
                    self._advance_safe_recovery(elapsed)
                    continue
                if self._emergency_stop_active:
                    continue
                for request in self.store.requests:
                    if request.status in TERMINAL_STATUSES:
                        continue

                    stage_age = now - self._stage_started.setdefault(request.id, now)
                    if request.status == RequestStatus.WAITING:
                        if stage_age >= self._ASSIGN_HOLD_SEC:
                            advance_ids.append(request.id)
                    elif request.status == RequestStatus.ROBOT_ASSIGNED:
                        if stage_age >= self._ASSIGN_HOLD_SEC:
                            advance_ids.append(request.id)
                    elif request.status == RequestStatus.APPROACHING:
                        if self._move_pair_along_route(request, elapsed):
                            advance_ids.append(request.id)
                    elif request.status == RequestStatus.LIFTING:
                        if stage_age >= self._LIFT_HOLD_SEC:
                            advance_ids.append(request.id)
                    elif request.status == RequestStatus.MOVING_TO_SLOT:
                        if self._move_pair_along_route(request, elapsed):
                            advance_ids.append(request.id)
                    elif request.status == RequestStatus.RETURNING:
                        if self._move_pair_along_route(request, elapsed):
                            advance_ids.append(request.id)

            for request_id in advance_ids:
                try:
                    self.advance_request(request_id)
                except DataSourceError:
                    # 대기 요청에 아직 가용 로봇이 없으면 다음 tick에 재시도한다.
                    continue

    def _build_safe_recovery_routes(self) -> None:
        """현재 위치에서 전용 통로를 거쳐 각 로봇 도크로 가는 경로를 만든다."""
        self._recovery_routes.clear()
        self._recovery_route_progress.clear()
        for robot_index, robot_id in enumerate(
            self._recovery_state["robot_ids"]
        ):
            robot = self.store.find_robot(robot_id)
            dock = _WAITING_DOCK_BY_ROBOT.get(robot_id)
            if robot is None or dock is None:
                continue
            if (
                robot.x is not None
                and robot.y is not None
                and math.hypot(robot.x - dock[0], robot.y - dock[1]) <= 0.15
            ):
                self._recovery_routes[robot_id] = [dock]
                self._recovery_route_progress[robot_id] = 0
                continue
            is_entry = robot_id.startswith("entry_")
            lane_y = _ENTRY_LANE_Y if is_entry else _EXIT_LANE_Y
            formation_index = 0 if robot_id.endswith("lead") else 1
            crossing_target = self._formation((_CROSSING_X, lane_y))[
                formation_index
            ]
            current_x = robot.x if robot.x is not None else crossing_target[0]
            lane_target = (current_x, crossing_target[1])
            self._recovery_routes[robot_id] = [
                lane_target,
                crossing_target,
                dock,
            ]
            self._recovery_route_progress[robot_id] = 0

    def _advance_safe_recovery(self, elapsed: float) -> None:
        """별도 복귀 작업을 진행하고 도크 도달이 확인된 뒤에만 IDLE 처리한다."""
        if any(
            alert.active and alert.category == AlertCategory.OBSTACLE
            for alert in self.store.alerts
        ):
            return

        all_reached = True
        for robot_id in self._recovery_state["robot_ids"]:
            robot = self.store.find_robot(robot_id)
            route = self._recovery_routes.get(robot_id, [])
            index = self._recovery_route_progress.get(robot_id, 0)
            if robot is None or not route:
                all_reached = False
                continue
            if index < len(route):
                if self._move_robot(robot, route[index], elapsed):
                    index += 1
                    self._recovery_route_progress[robot_id] = index
            if index < len(route):
                all_reached = False

        if not all_reached:
            return

        for robot_id in self._recovery_state["robot_ids"]:
            robot = self.store.find_robot(robot_id)
            dock = _WAITING_DOCK_BY_ROBOT.get(robot_id)
            if robot is None or dock is None:
                continue
            robot.x, robot.y = dock
            robot.status = "IDLE"
            robot.current_task_id = None
        self._recovery_state.update(
            status="COMPLETED",
            message="모든 대상 로봇의 도크 복귀와 위치 확인이 완료되었습니다.",
            completed_at=_now(),
        )
        for alert in self.store.alerts:
            if alert.active and alert.category == AlertCategory.EMERGENCY_STOP:
                alert.level = AlertLevel.WARNING
                alert.message = (
                    "대상 로봇의 도크 복귀가 완료되었습니다. "
                    "최종 확인 후 정상 운영 복귀를 승인해주세요."
                )

    def _apply_parking_result(self, request: ParkingRequest) -> None:
        slot = self.store.find_slot(request.slot_id) if request.slot_id else None

        if slot:
            if request.request_type == RequestType.PARK_IN:
                slot.status = "OCCUPIED"
                slot.vehicle_number = request.vehicle_number
            else:
                slot.status = "EMPTY"
                slot.vehicle_number = None

    def _complete_request(self, request: ParkingRequest) -> None:

        for robot_id in request.robot_ids:
            robot = self.store.find_robot(robot_id)
            if robot is None:
                continue
            robot.status = "IDLE"
            robot.current_task_id = None
            robot.battery = max(0, robot.battery - 4)
            waiting_position = _WAITING_DOCK_BY_ROBOT.get(robot.id)
            if waiting_position is not None:
                robot.x, robot.y = waiting_position

    # ------------------------------------------------------------------
    # 이벤트 시뮬레이션 (Mock 전용)
    # ------------------------------------------------------------------
    def trigger_obstacle(self) -> Alert:
        """장애물 감지 이벤트를 발생시킨다. 작업 중 로봇이 있으면 해당 로봇 기준."""
        with self.store.lock:
            busy_robot = next(
                (r for r in self.store.robots if r.status == "BUSY"), None
            )
            robot = busy_robot or (
                self.store.robots[0] if self.store.robots else None
            )
            location_x = (
                busy_robot.x + 1.0
                if busy_robot and busy_robot.x is not None
                else _CROSSING_X
            )
            location_y = (
                busy_robot.y
                if busy_robot and busy_robot.y is not None
                else _ENTRY_LANE_Y
            )
            zone_id = (
                "ZOUT01"
                if busy_robot and busy_robot.id.startswith("exit")
                else "ZIN03"
            )

            alert = Alert(
                id=self.store.next_alert_id(),
                level=AlertLevel.WARNING,
                category=AlertCategory.OBSTACLE,
                message=(
                    f"{robot.id} 주행 경로에서 장애물이 감지되었습니다."
                    if robot
                    else "주행 경로에서 장애물이 감지되었습니다."
                ),
                robot_id=busy_robot.id if busy_robot else None,
                sensor_id="L1",
                zone_id=zone_id,
                location_x=location_x,
                location_y=location_y,
                created_at=_now(),
            )
            self.store.alerts.append(alert)
            open_obstacle_incident(self.store, alert)
            return alert.model_copy(deep=True)

    def emergency_stop(self) -> int:
        """Mock 이동을 중단하고 중앙 안전 복구 절차와 같은 상태를 만든다."""
        with self.store.lock:
            if self._safety_state["state"] == "STOPPED_LATCHED":
                return sum(
                    request.status not in TERMINAL_STATUSES
                    for request in self.store.requests
                )

            self._emergency_stop_active = True
            active_requests = [
                request
                for request in self.store.requests
                if request.status not in TERMINAL_STATUSES
            ]
            active_count = len(active_requests)
            previous_recovery_robot_ids = (
                list(self._recovery_state["robot_ids"])
                if self.recovery_pending
                else []
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
            source_request_ids = list(
                dict.fromkeys(
                    [
                        *(
                            self._recovery_state["source_request_ids"]
                            if previous_recovery_robot_ids
                            else []
                        ),
                        *[request.id for request in active_requests],
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
            for request in active_requests:
                request.status = RequestStatus.CANCELLED
                request.completed_at = _now()
                if request.request_type == RequestType.PARK_IN and request.slot_id:
                    slot = self.store.find_slot(request.slot_id)
                    if slot and slot.status == "RESERVED":
                        slot.status = "EMPTY"
                        slot.vehicle_number = None
            for robot_id in recovery_robot_ids:
                robot = self.store.find_robot(robot_id)
                if robot is not None:
                    robot.current_task_id = None
                    robot.status = "SAFETY_STOPPED"
            self._recovery_state = {
                "status": (
                    "SAFETY_STOPPED" if recovery_robot_ids else "NONE"
                ),
                "robot_ids": recovery_robot_ids,
                "source_request_ids": source_request_ids,
                "load_state": (
                    "LOAD_REQUIRES_CLEARANCE"
                    if load_present
                    or self._recovery_state["load_state"]
                    == "LOAD_REQUIRES_CLEARANCE"
                    else "CLEAR"
                ),
                "control_available": True,
                "message": (
                    "로봇은 현재 위치에서 안전 정지했습니다. "
                    "원 작업은 취소되며 점검 후 별도 도크 복귀가 필요합니다."
                    if recovery_robot_ids
                    else ""
                ),
                "started_at": None,
                "completed_at": None,
            }
            self._safety_state = {
                "state": "STOPPED_LATCHED",
                "motion_allowed": False,
                "stop_epoch": self._safety_state["stop_epoch"] + 1,
                "reason": "관제 UI 전체 비상정지",
                "operator_id": "control_ui",
                "inspection_note": "",
                "affected_task_ids": [str(request.id) for request in active_requests],
                "blockers": ["현장 안전 점검 및 관제 해제 승인 필요"],
                "updated_at": _now(),
            }
            self.store.alerts.append(
                Alert(
                    id=self.store.next_alert_id(),
                    level=AlertLevel.ERROR,
                    category=AlertCategory.EMERGENCY_STOP,
                    message=(
                        "운영자가 비상정지를 실행했습니다. "
                        "장비 점검 후 관제 해제 요청을 진행해주세요."
                    ),
                    robot_id=None,
                    created_at=_now(),
                )
            )
            return active_count

    def request_safety_reset(self, payload: SafetyResetRequest) -> dict:
        with self.store.lock:
            if self._safety_state["state"] != "STOPPED_LATCHED":
                raise DataSourceError(
                    "현재 상태에서는 안전 해제 요청을 접수할 수 없습니다.",
                    status_code=409,
                )
            checks = {
                "작업 구역 안전": payload.area_clear,
                "전체 로봇 정지": payload.robots_stopped,
                "차량·리프트 상태": payload.load_secured,
                "센서·통신 상태": payload.sensors_checked,
            }
            missing = [label for label, checked in checks.items() if not checked]
            if missing:
                raise DataSourceError(
                    f"확인하지 않은 점검 항목: {', '.join(missing)}",
                    status_code=409,
                )
            self._safety_state.update(
                state="READY_FOR_OPERATION",
                motion_allowed=False,
                operator_id=payload.operator_id.strip(),
                inspection_note=payload.inspection_note.strip(),
                blockers=[],
                updated_at=_now(),
            )
            if self._recovery_state["status"] == "SAFETY_STOPPED":
                self._recovery_state.update(
                    status="REQUIRED",
                    message=(
                        "현장 점검이 완료되었습니다. 제한된 안전 복귀로 "
                        "대상 로봇을 먼저 도크에 복귀시켜주세요."
                    ),
                )
                for robot_id in self._recovery_state["robot_ids"]:
                    robot = self.store.find_robot(robot_id)
                    if robot is not None:
                        robot.status = "RECOVERY_REQUIRED"
            for alert in self.store.alerts:
                if alert.active and alert.category == AlertCategory.EMERGENCY_STOP:
                    alert.level = AlertLevel.WARNING
                    alert.message = (
                        "안전 점검이 승인되었습니다. 로봇은 계속 정지 상태이며 "
                        "대상 로봇의 제한 안전 복귀가 필요합니다."
                    )
            result = self.safety_state
            result["message"] = (
                "점검 결과가 승인되었습니다. 대상 로봇을 제한 안전 복귀로 "
                "도크에 이동시킨 뒤 정상 운영을 승인해주세요."
                if self.recovery_pending
                else "점검 결과가 승인되었습니다. 정상 운영 복귀 승인이 필요합니다."
            )
            return result

    def approve_operation(self, payload: OperationApprovalRequest) -> dict:
        with self.store.lock:
            if self._safety_state["state"] != "READY_FOR_OPERATION":
                raise DataSourceError(
                    "안전 점검 승인 후에만 운영 복귀할 수 있습니다.",
                    status_code=409,
                )
            if self._recovery_state["status"] in {
                "SAFETY_STOPPED",
                "REQUIRED",
                "RECOVERING",
                "BLOCKED",
            }:
                raise DataSourceError(
                    "대상 로봇의 도크 복귀와 위치 확인을 먼저 완료해주세요.",
                    status_code=409,
                )
            self._safety_state.update(
                state="NORMAL",
                motion_allowed=True,
                operator_id=payload.operator_id.strip(),
                affected_task_ids=[],
                blockers=[],
                updated_at=_now(),
            )
            self._emergency_stop_active = False
            for alert in self.store.alerts:
                if alert.active and alert.category == AlertCategory.EMERGENCY_STOP:
                    alert.active = False
            result = self.safety_state
            result["message"] = (
                "정상 운영 복귀가 승인되었습니다. 기존 취소 작업은 재개되지 않으며 "
                "새 작업만 접수합니다."
            )
            return result

    def start_safe_recovery(self, payload: RobotRecoveryRequest) -> dict:
        with self.store.lock:
            if self._safety_state["state"] != "READY_FOR_OPERATION":
                raise DataSourceError(
                    "현장 안전 점검 승인 후에만 제한 안전 복귀를 시작할 수 있습니다.",
                    status_code=409,
                )
            if self._recovery_state["status"] != "REQUIRED":
                raise DataSourceError(
                    "안전 복귀가 필요한 로봇이 없거나 이미 복귀 중입니다.",
                    status_code=409,
                )
            checks = {
                "복귀 경로 안전": payload.path_clear,
                "차량·적재물 분리 또는 별도 안전 확보": payload.load_cleared,
                "리프트 암 회수": payload.arms_retracted,
                "복귀용 센서·통신": payload.sensors_ready,
            }
            missing = [label for label, checked in checks.items() if not checked]
            if missing:
                raise DataSourceError(
                    f"확인하지 않은 복귀 조건: {', '.join(missing)}",
                    status_code=409,
                )
            self._build_safe_recovery_routes()
            if len(self._recovery_routes) != len(
                self._recovery_state["robot_ids"]
            ):
                self._recovery_state.update(
                    status="BLOCKED",
                    message="일부 로봇의 도크 또는 위치 정보를 확인할 수 없습니다.",
                )
                raise DataSourceError(
                    self._recovery_state["message"], status_code=409
                )
            for robot_id in self._recovery_state["robot_ids"]:
                robot = self.store.find_robot(robot_id)
                if robot is not None:
                    robot.status = "RECOVERING"
            self._recovery_state.update(
                status="RECOVERING",
                message=(
                    "취소된 작업은 재개하지 않고 대상 로봇만 전용 통로를 따라 "
                    "각 도크로 복귀합니다."
                ),
                started_at=_now(),
                completed_at=None,
            )
            return self.recovery_state

    def trigger_robot_error(self) -> Alert:
        """로봇 오류 이벤트를 발생시킨다. 작업 중 로봇 우선, 없으면 첫 정상 로봇."""
        with self.store.lock:
            robot = next(
                (r for r in self.store.robots if r.status == "BUSY"), None
            ) or next(
                (r for r in self.store.robots if r.status != "ERROR"), None
            )

            if robot is None:
                raise DataSourceError(
                    "오류를 발생시킬 로봇이 없습니다.", status_code=409
                )

            robot.status = "ERROR"
            robot.error_message = "구동부 통신 오류 (모의)"

            alert = Alert(
                id=self.store.next_alert_id(),
                level=AlertLevel.ERROR,
                category=AlertCategory.ROBOT_ERROR,
                message=f"{robot.id} 오류: {robot.error_message}",
                robot_id=robot.id,
                created_at=_now(),
            )
            self.store.alerts.append(alert)
            return alert.model_copy(deep=True)

    def resolve_alert(self, alert_id: int) -> None:
        with self.store.lock:
            alert = self.store.find_alert(alert_id)
            if alert is None:
                raise DataSourceError("알림을 찾을 수 없습니다.", status_code=404)
            if alert.category == AlertCategory.EMERGENCY_STOP:
                raise DataSourceError(
                    "비상정지는 알림 해제로 복구할 수 없습니다. 안전 복구 절차를 진행해주세요.",
                    status_code=409,
                )

            alert.active = False

            if alert.category == AlertCategory.OBSTACLE:
                recover_obstacle_incident(self.store, alert)

            # 로봇 오류 알림 해제 시 로봇 복구
            if alert.category == AlertCategory.ROBOT_ERROR and alert.robot_id:
                robot = self.store.find_robot(alert.robot_id)
                if robot and robot.status == "ERROR":
                    robot.error_message = None
                    robot.status = "BUSY" if robot.current_task_id else "IDLE"
