"""데이터 소스 추상 인터페이스.

웹 계층은 이 인터페이스만 사용한다. 구현체:
  - MockDataSource  : 인메모리 시뮬레이션 (현재)
  - Ros2DataSource  : ROS2 토픽/서비스 연동 (향후, task_dispatcher 스펙 확정 후)

task_dispatcher 인터페이스가 ROS2든 HTTP든, create_request()의 구현만
달라지고 웹 계층은 수정하지 않는 것이 목표.
"""

from abc import ABC, abstractmethod

from .models import ParkingRequest, ParkingRequestCreate
from .state_store import StateStore


class DataSourceError(Exception):
    """데이터 소스 처리 오류. status_code는 HTTP 응답 코드에 매핑된다."""

    def __init__(self, detail: str, status_code: int = 400) -> None:
        super().__init__(detail)
        self.detail = detail
        self.status_code = status_code


class DataSource(ABC):
    """관제 시스템 데이터 소스."""

    #: Mock 제어(다음 단계 / 초기화 / 이벤트 시뮬레이션) 지원 여부.
    #: 실제 시스템(ROS2) 모드에서는 False가 되어 UI에서 관련 버튼이 숨겨진다.
    supports_mock_controls: bool = False

    def __init__(self, store: StateStore) -> None:
        self.store = store

    def start(self) -> None:
        """백그라운드 리소스 기동 (ROS2 spin 스레드 등). Mock은 no-op."""

    def stop(self) -> None:
        """백그라운드 리소스 정리. Mock은 no-op."""

    @abstractmethod
    def create_request(self, payload: ParkingRequestCreate) -> ParkingRequest:
        """입고/출차 요청 등록. 실제 모드에서는 task_dispatcher로 전달."""

    @abstractmethod
    def advance_request(self, request_id: int) -> ParkingRequest:
        """(Mock 전용) 작업 단계를 한 단계 진행."""

    @abstractmethod
    def reset(self) -> None:
        """(Mock 전용) 상태 초기화."""

    def resolve_alert(self, alert_id: int) -> None:
        """알림 해제. 기본 구현은 StateStore에서 비활성화만 수행."""
        with self.store.lock:
            alert = self.store.find_alert(alert_id)
            if alert is None:
                raise DataSourceError("알림을 찾을 수 없습니다.", status_code=404)
            alert.active = False
