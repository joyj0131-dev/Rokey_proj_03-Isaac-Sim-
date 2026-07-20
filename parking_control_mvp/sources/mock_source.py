"""Mock 데이터 소스.

기존 main.py의 인메모리 시뮬레이션 로직을 이관한 구현체.
추가로 장애물 감지/로봇 오류 이벤트 시뮬레이션을 지원한다.
"""

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
    ("robot_02", "CHARGING", 64, _DOCK_CHARGE_B),
]

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

    def __init__(self, store: StateStore) -> None:
        super().__init__(store)
        self._load_defaults(clear_counters=False)

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

    def get_map_info(self) -> dict:
        return {
            "docks": [
                {"role": "waiting", "x": _DOCK_WAIT_A[0], "y": _DOCK_WAIT_A[1]},
                {"role": "waiting", "x": _DOCK_WAIT_B[0], "y": _DOCK_WAIT_B[1]},
                {"role": "charging", "x": _DOCK_CHARGE_A[0], "y": _DOCK_CHARGE_A[1]},
                {"role": "charging", "x": _DOCK_CHARGE_B[0], "y": _DOCK_CHARGE_B[1]},
            ],
            "entrance": {"x": _ENTRANCE[0], "y": _ENTRANCE[1]},
        }

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

            idle_robot = next(
                (r for r in self.store.robots if r.status == "IDLE"), None
            )

            request = ParkingRequest(
                id=self.store.next_request_id(),
                request_type=payload.request_type,
                vehicle_number=vehicle_number,
                slot_id=selected_slot_id,
                robot_id=idle_robot.id if idle_robot else None,
                status=(
                    RequestStatus.ROBOT_ASSIGNED
                    if idle_robot
                    else RequestStatus.WAITING
                ),
                created_at=_now(),
            )
            self.store.requests.append(request)

            if idle_robot:
                idle_robot.status = "BUSY"
                idle_robot.current_task_id = request.id

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

            # 담당 로봇이 오류 상태이면 진행 불가
            if request.robot_id:
                robot = self.store.find_robot(request.robot_id)
                if robot and robot.status == "ERROR":
                    raise DataSourceError(
                        f"{robot.id} 로봇이 오류 상태입니다. 알림을 해제한 뒤 진행해주세요.",
                        status_code=409,
                    )

            if request.status == RequestStatus.WAITING:
                idle_robot = next(
                    (r for r in self.store.robots if r.status == "IDLE"), None
                )
                if idle_robot is None:
                    raise DataSourceError(
                        "대기 중인 로봇이 없습니다.", status_code=409
                    )

                request.robot_id = idle_robot.id
                idle_robot.status = "BUSY"
                idle_robot.current_task_id = request.id

            request.status = STATUS_TRANSITIONS[request.status]

            if request.status == RequestStatus.COMPLETED:
                self._complete_request(request)

            return request.model_copy(deep=True)

    def _complete_request(self, request: ParkingRequest) -> None:
        slot = self.store.find_slot(request.slot_id) if request.slot_id else None

        if slot:
            if request.request_type == RequestType.PARK_IN:
                slot.status = "OCCUPIED"
                slot.vehicle_number = request.vehicle_number
            else:
                slot.status = "EMPTY"
                slot.vehicle_number = None

        robot = self.store.find_robot(request.robot_id) if request.robot_id else None
        if robot:
            robot.status = "IDLE"
            robot.current_task_id = None
            robot.battery = max(0, robot.battery - 4)

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
