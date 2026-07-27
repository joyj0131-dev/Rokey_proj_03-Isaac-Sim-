"""데이터 소스 추상 인터페이스.

웹 계층은 이 인터페이스만 사용한다. 구현체:
  - MockDataSource  : 인메모리 시뮬레이션 (현재)
  - Ros2DataSource  : ROS2 토픽/서비스 연동 (향후, task_dispatcher 스펙 확정 후)

task_dispatcher 인터페이스가 ROS2든 HTTP든, create_request()의 구현만
달라지고 웹 계층은 수정하지 않는 것이 목표.
"""

from abc import ABC, abstractmethod

from .models import (
    CooperativeLoadState,
    OperationApprovalRequest,
    ParkingRequest,
    ParkingRequestCreate,
    SafetyResetRequest,
    VisionAlignmentState,
)
from .safety_incident import recover_obstacle_incident
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
    mock_auto_advance: bool = False

    def __init__(self, store: StateStore) -> None:
        self.store = store
        # 비상정지는 안전상 UI에서 해제하지 않는 래치 상태다. 프로세스 재기동
        # 또는 실제 장비 점검 절차를 거쳐야만 초기화된다.
        self._emergency_stop_active = False
        self._safety_state = {
            "state": "NORMAL",
            "motion_allowed": True,
            "stop_epoch": 0,
            "reason": "",
            "operator_id": "",
            "inspection_note": "",
            "affected_task_ids": [],
            "blockers": [],
            "updated_at": "",
        }

    @property
    def emergency_stop_active(self) -> bool:
        return self._safety_state["state"] != "NORMAL"

    @property
    def safety_state(self) -> dict:
        return {
            **self._safety_state,
            "affected_task_ids": list(self._safety_state["affected_task_ids"]),
            "blockers": list(self._safety_state["blockers"]),
        }

    def start(self) -> None:
        """백그라운드 리소스 기동 (ROS2 spin 스레드 등). Mock은 no-op."""

    def stop(self) -> None:
        """백그라운드 리소스 정리. Mock은 no-op."""

    @abstractmethod
    def create_request(self, payload: ParkingRequestCreate) -> ParkingRequest:
        """입차/출차 요청 등록. 실제 모드에서는 task_dispatcher로 전달."""

    @abstractmethod
    def advance_request(self, request_id: int) -> ParkingRequest:
        """(Mock 전용) 작업 단계를 한 단계 진행."""

    @abstractmethod
    def reset(self) -> None:
        """(Mock 전용) 상태 초기화."""

    def reset_test_environment(self) -> None:
        """(ROS2 전용, 테스트/개발 환경) 연동 DB를 초기 상태로 되돌린다."""
        raise DataSourceError("현재 모드에서는 DB 초기화를 사용할 수 없습니다.", status_code=403)

    def emergency_stop(self) -> int:
        """전체 로봇 비상정지. 반환값은 정지 신호를 보낸 활성 작업 수."""
        raise DataSourceError(
            "현재 모드에서는 비상정지를 사용할 수 없습니다.", status_code=403
        )

    def request_safety_reset(self, payload: SafetyResetRequest) -> dict:
        raise DataSourceError(
            "현재 모드에서는 안전 해제 요청을 사용할 수 없습니다.", status_code=403
        )

    def approve_operation(self, payload: OperationApprovalRequest) -> dict:
        raise DataSourceError(
            "현재 모드에서는 운영 복귀 승인을 사용할 수 없습니다.", status_code=403
        )

    def resolve_alert(self, alert_id: int) -> None:
        """알림 해제. 기본 구현은 StateStore에서 비활성화만 수행."""
        with self.store.lock:
            alert = self.store.find_alert(alert_id)
            if alert is None:
                raise DataSourceError("알림을 찾을 수 없습니다.", status_code=404)
            if alert.category == "EMERGENCY_STOP":
                raise DataSourceError(
                    "비상정지는 알림 해제로 복구할 수 없습니다. 관제 안전 복구 절차를 진행해주세요.",
                    status_code=409,
                )
            alert.active = False
            if alert.category == "OBSTACLE":
                recover_obstacle_incident(self.store, alert)

    def get_map_info(self) -> dict:
        """실시간 도면 패널용 정적 레이아웃 정보.

        슬롯/로봇처럼 매번 바뀌는 값이 아니라 지도 자체의 고정 배치라
        StateStore가 아니라 여기서 별도로 제공한다. 기본값은 빈 레이아웃.
        """
        return {
            "layout": None,
            "nodes": [],
            "docks": [],
            "vehicle_zones": [],
            "entrance": None,
            "sensors": [],
        }

    def get_sensor_status(self) -> list[dict]:
        """웹 도면에 표시할 센서 연결 상태. 구현이 없으면 빈 목록."""
        return []

    def get_cooperative_load_states(self) -> list[CooperativeLoadState]:
        """진행 작업별 협동 적재 상태. 데이터 계약이 없으면 빈 목록."""
        return []

    def get_vision_alignment_states(self) -> list[VisionAlignmentState]:
        """로봇별 ArUco 검출·정렬 상태. 데이터 계약이 없으면 빈 목록."""
        return []
