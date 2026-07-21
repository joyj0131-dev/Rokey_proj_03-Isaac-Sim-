"""Mock 데이터 소스.

기존 main.py의 인메모리 시뮬레이션 로직을 이관한 구현체.
추가로 장애물 감지/로봇 오류 이벤트 시뮬레이션을 지원한다.
"""

import math
import threading
import time
from datetime import datetime

from core.datasource import DataSource, DataSourceError
from core.models import (
    STATUS_TRANSITIONS,
    TERMINAL_STATUSES,
    Alert,
    AlertCategory,
    AlertLevel,
    ParkingRequest,
    ParkingRequestCreate,
    ParkingSlot,
    RequestStatus,
    RequestType,
    Robot,
)
from core.state_store import StateStore

#: 실제 map 좌표가 없는 mock 모드용 가상 배치. 실제 parking_map.yaml과 같은
#: 규칙(입구 → 대기/충전 도크 → 통로(y=0) → A행(y<0)/B행(y>0))을 따른다.
_ENTRANCE = (-18.1, 0.0)
_DOCK_WAIT_A = (-15.3, -7.8)
_DOCK_WAIT_B = (-15.3, 7.8)
_DOCK_CHARGE_A = (15.3, -7.8)
_DOCK_CHARGE_B = (15.3, 7.8)

#: id, status, battery, (x, y) — 로봇은 각자의 도크 위치에서 시작.
_DEFAULT_ROBOTS = [
    ("robot_01", "IDLE", 92, _DOCK_WAIT_A),
    ("robot_02", "IDLE", 64, _DOCK_WAIT_B),
]

_WAITING_DOCK_BY_ROBOT = {
    "robot_01": _DOCK_WAIT_A,
    "robot_02": _DOCK_WAIT_B,
}

#: id, status, vehicle, (x, y), is_accessible
_DEFAULT_SLOTS = [
    ("A1", "OCCUPIED", "12가3456", (-11.9, -7.8), True),
    ("A2", "EMPTY", None, (-8.5, -7.8), True),
    ("A3", "EMPTY", None, (-5.1, -7.8), False),
    ("A4", "EMPTY", None, (-1.7, -7.8), False),
    ("A5", "EMPTY", None, (1.7, -7.8), False),
    ("A6", "EMPTY", None, (5.1, -7.8), False),
    ("A7", "EMPTY", None, (8.5, -7.8), False),
    ("A8", "EMPTY", None, (11.9, -7.8), False),
    ("B1", "OCCUPIED", "34나7890", (-11.9, 7.8), False),
    ("B2", "EMPTY", None, (-8.5, 7.8), False),
    ("B3", "EMPTY", None, (-5.1, 7.8), False),
    ("B4", "EMPTY", None, (-1.7, 7.8), False),
    ("B5", "EMPTY", None, (1.7, 7.8), False),
    ("B6", "EMPTY", None, (5.1, 7.8), False),
    ("B7", "EMPTY", None, (8.5, 7.8), False),
    ("B8", "EMPTY", None, (11.9, 7.8), False),
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
            self._stage_started.clear()
            self._route_progress.clear()

    def get_map_info(self) -> dict:
        return {
            "docks": [
                {"role": "waiting", "x": _DOCK_WAIT_A[0], "y": _DOCK_WAIT_A[1]},
                {"role": "waiting", "x": _DOCK_WAIT_B[0], "y": _DOCK_WAIT_B[1]},
                {"role": "charging", "x": _DOCK_CHARGE_A[0], "y": _DOCK_CHARGE_A[1]},
                {"role": "charging", "x": _DOCK_CHARGE_B[0], "y": _DOCK_CHARGE_B[1]},
            ],
            "entrance": {"x": _ENTRANCE[0], "y": _ENTRANCE[1]},
            "sensors": [
                {"id": "L1", "zone": "서쪽", "x": -7.82, "y": 0.0},
                {"id": "L2", "zone": "동쪽", "x": 7.82, "y": 0.0},
            ],
        }

    def get_sensor_status(self) -> list[dict]:
        return [
            {
                "id": "L1",
                "topic": "/parking/lidar/ceiling_01/points",
                "status": "MOCK",
                "rate_hz": None,
                "last_seen_sec": None,
            },
            {
                "id": "L2",
                "topic": "/parking/lidar/ceiling_02/points",
                "status": "MOCK",
                "rate_hz": None,
                "last_seen_sec": None,
            },
        ]

    def reset(self) -> None:
        self._load_defaults(clear_counters=True)

    # ------------------------------------------------------------------
    # 요청 등록
    # ------------------------------------------------------------------
    def create_request(self, payload: ParkingRequestCreate) -> ParkingRequest:
        vehicle_number = payload.vehicle_number.strip()

        if not vehicle_number:
            raise DataSourceError("차량 번호를 입력해주세요.", status_code=400)

        with self.store.lock:
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
                        "이미 입고 또는 예약된 차량입니다.", status_code=409
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

            idle_robots = [
                robot for robot in self.store.robots if robot.status == "IDLE"
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
                idle_robots = [
                    robot for robot in self.store.robots if robot.status == "IDLE"
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

        dock_to_aisle = [
            (_DOCK_WAIT_A[0], -self._PAIR_HALF_GAP_M),
            (_DOCK_WAIT_B[0], self._PAIR_HALF_GAP_M),
        ]
        slot_center = (slot.x, slot.y)
        slot_aisle = (slot.x, 0.0)

        if request.status == RequestStatus.APPROACHING:
            if request.request_type == RequestType.PARK_IN:
                return [dock_to_aisle, self._formation(_ENTRANCE)]
            return [
                dock_to_aisle,
                self._formation(slot_aisle),
                self._formation(slot_center),
            ]

        if request.status == RequestStatus.MOVING_TO_SLOT:
            if request.request_type == RequestType.PARK_IN:
                return [self._formation(slot_aisle), self._formation(slot_center)]
            return [self._formation(slot_aisle), self._formation(_ENTRANCE)]

        if request.status == RequestStatus.RETURNING:
            waiting_docks = [_DOCK_WAIT_A, _DOCK_WAIT_B]
            if request.request_type == RequestType.PARK_IN:
                return [
                    self._formation(slot_aisle),
                    dock_to_aisle,
                    waiting_docks,
                ]
            return [dock_to_aisle, waiting_docks]
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
            robot = next(
                (r for r in self.store.robots if r.status == "BUSY"), None
            ) or (self.store.robots[0] if self.store.robots else None)

            alert = Alert(
                id=self.store.next_alert_id(),
                level=AlertLevel.WARNING,
                category=AlertCategory.OBSTACLE,
                message=(
                    f"{robot.id} 주행 경로에서 장애물이 감지되었습니다."
                    if robot
                    else "주행 경로에서 장애물이 감지되었습니다."
                ),
                robot_id=robot.id if robot else None,
                created_at=_now(),
            )
            self.store.alerts.append(alert)
            return alert.model_copy(deep=True)

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

            alert.active = False

            # 로봇 오류 알림 해제 시 로봇 복구
            if alert.category == AlertCategory.ROBOT_ERROR and alert.robot_id:
                robot = self.store.find_robot(alert.robot_id)
                if robot and robot.status == "ERROR":
                    robot.error_message = None
                    robot.status = "BUSY" if robot.current_task_id else "IDLE"
