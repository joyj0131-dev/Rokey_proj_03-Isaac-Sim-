"""공용 데이터 모델.

웹 계층(FastAPI)과 데이터 소스(Mock / 향후 ROS2 Bridge)가 함께 사용한다.
ROS2 메시지 수신 시에도 이 모델로 변환하여 StateStore에 저장한다.
"""

from enum import Enum
from typing import Literal

from pydantic import BaseModel, Field


class RequestType(str, Enum):
    PARK_IN = "PARK_IN"
    PARK_OUT = "PARK_OUT"


class RequestStatus(str, Enum):
    """작업 진행 단계.

    task_dispatcher 예상 흐름에 복귀 감시 단계를 더한 상태 정의:
    요청 대기 → 로봇 할당 → 차량 접근 → 차량 리프트 → 주차 위치 이동
    → 대기 구역 복귀 → 완료
    """

    WAITING = "WAITING"                  # 요청 대기
    ROBOT_ASSIGNED = "ROBOT_ASSIGNED"    # 로봇 할당
    APPROACHING = "APPROACHING"          # 차량 접근
    LIFTING = "LIFTING"                  # 차량 리프트
    MOVING_TO_SLOT = "MOVING_TO_SLOT"    # 주차 위치 이동
    RETURNING = "RETURNING"              # 작업 후 대기 구역 복귀
    COMPLETED = "COMPLETED"              # 완료
    CANCELLED = "CANCELLED"              # 취소


# 정상 흐름 상태 전이 (Mock 및 검증용)
STATUS_TRANSITIONS: dict[RequestStatus, RequestStatus] = {
    RequestStatus.WAITING: RequestStatus.ROBOT_ASSIGNED,
    RequestStatus.ROBOT_ASSIGNED: RequestStatus.APPROACHING,
    RequestStatus.APPROACHING: RequestStatus.LIFTING,
    RequestStatus.LIFTING: RequestStatus.MOVING_TO_SLOT,
    RequestStatus.MOVING_TO_SLOT: RequestStatus.RETURNING,
    RequestStatus.RETURNING: RequestStatus.COMPLETED,
}

TERMINAL_STATUSES = {RequestStatus.COMPLETED, RequestStatus.CANCELLED}


class ParkingRequestCreate(BaseModel):
    request_type: RequestType
    vehicle_number: str = Field(min_length=1, max_length=20)
    slot_id: str | None = None


class ParkingRequest(BaseModel):
    id: int
    request_type: RequestType
    vehicle_number: str
    slot_id: str | None
    robot_id: str | None
    #: 협업 운반에 참여하는 로봇 목록. robot_id는 기존 API 호환용 대표 로봇.
    robot_ids: list[str] = Field(default_factory=list)
    status: RequestStatus
    created_at: str
    #: task_dispatcher가 발급한 task_id (UUID). mock 모드에서는 None.
    external_task_id: str | None = None


class Robot(BaseModel):
    id: str
    status: Literal["IDLE", "BUSY", "CHARGING", "ERROR", "OFFLINE"]
    battery: int
    current_task_id: int | None = None
    error_message: str | None = None
    #: 실시간 도면 표시용 좌표. ros2 모드는 DB의 실측값, mock 모드는 가상 배치.
    x: float | None = None
    y: float | None = None


class ParkingSlot(BaseModel):
    id: str
    status: Literal["EMPTY", "RESERVED", "OCCUPIED"]
    vehicle_number: str | None = None
    x: float | None = None
    y: float | None = None
    is_accessible: bool = False


class AlertLevel(str, Enum):
    WARNING = "WARNING"  # 주의 (예: 장애물 감지)
    ERROR = "ERROR"      # 오류 (예: 로봇 이상)


class AlertCategory(str, Enum):
    OBSTACLE = "OBSTACLE"        # 장애물 감지
    ROBOT_ERROR = "ROBOT_ERROR"  # 로봇 오류
    SYSTEM = "SYSTEM"            # 기타 시스템 이벤트


class Alert(BaseModel):
    id: int
    level: AlertLevel
    category: AlertCategory
    message: str
    robot_id: str | None = None
    created_at: str
    active: bool = True
