"""스레드 안전 상태 저장소.

FastAPI 워커 스레드와 향후 ROS2 콜백 스레드가 동시에 접근하므로,
모든 읽기/쓰기는 lock으로 보호한다. 외부에는 스냅샷(복사본)만 노출한다.
"""

import threading
from itertools import count

from .models import Alert, ParkingRequest, ParkingSlot, Robot


class StateStore:
    def __init__(self) -> None:
        self._lock = threading.RLock()
        self.robots: list[Robot] = []
        self.parking_slots: list[ParkingSlot] = []
        self.requests: list[ParkingRequest] = []
        self.alerts: list[Alert] = []
        self._request_id_counter = count(1)
        self._alert_id_counter = count(1)

    @property
    def lock(self) -> threading.RLock:
        """데이터 소스가 복합 갱신 시 사용할 락."""
        return self._lock

    def next_request_id(self) -> int:
        return next(self._request_id_counter)

    def next_alert_id(self) -> int:
        return next(self._alert_id_counter)

    # ------------------------------------------------------------------
    # 조회 (스냅샷)
    # ------------------------------------------------------------------
    def snapshot(self) -> dict:
        """대시보드용 전체 스냅샷을 복사본으로 반환한다."""
        with self._lock:
            robots = [robot.model_copy(deep=True) for robot in self.robots]
            slots = [slot.model_copy(deep=True) for slot in self.parking_slots]
            requests = [req.model_copy(deep=True) for req in self.requests]
            alerts = [
                alert.model_copy(deep=True)
                for alert in self.alerts
                if alert.active
            ]
        return {
            "robots": robots,
            "slots": slots,
            "requests": requests,
            "alerts": alerts,
        }

    # ------------------------------------------------------------------
    # 내부 검색 헬퍼 (호출 측에서 lock을 잡은 상태에서 사용)
    # ------------------------------------------------------------------
    def find_robot(self, robot_id: str) -> Robot | None:
        return next((r for r in self.robots if r.id == robot_id), None)

    def find_slot(self, slot_id: str) -> ParkingSlot | None:
        return next((s for s in self.parking_slots if s.id == slot_id), None)

    def find_request(self, request_id: int) -> ParkingRequest | None:
        return next((r for r in self.requests if r.id == request_id), None)

    def find_alert(self, alert_id: int) -> Alert | None:
        return next((a for a in self.alerts if a.id == alert_id), None)
