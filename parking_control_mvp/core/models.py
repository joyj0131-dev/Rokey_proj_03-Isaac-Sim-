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
    #: 완료·취소 시각. 진행 작업은 None이며 완료 결과의 실제 소요시간 계산에 쓴다.
    completed_at: str | None = None
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
    EMERGENCY_STOP = "EMERGENCY_STOP"  # 운영자 비상정지
    SYSTEM = "SYSTEM"            # 기타 시스템 이벤트


class SafetyStatus(str, Enum):
    NORMAL = "NORMAL"
    STOPPED_LATCHED = "STOPPED_LATCHED"
    READY_FOR_OPERATION = "READY_FOR_OPERATION"
    UNKNOWN = "UNKNOWN"


class SafetyResetRequest(BaseModel):
    operator_id: str = Field(min_length=1, max_length=64)
    inspection_note: str = Field(min_length=5, max_length=1000)
    area_clear: bool
    robots_stopped: bool
    load_secured: bool
    sensors_checked: bool


class OperationApprovalRequest(BaseModel):
    operator_id: str = Field(min_length=1, max_length=64)
    approval_note: str = Field(min_length=5, max_length=1000)


class Alert(BaseModel):
    id: int
    level: AlertLevel
    category: AlertCategory
    message: str
    robot_id: str | None = None
    #: 감지 이벤트를 발행한 센서/영역과 ROS map 기준 위치.
    #: 기존 알림 생산자는 값을 생략할 수 있도록 모두 선택 필드로 둔다.
    sensor_id: str | None = None
    zone_id: str | None = None
    location_x: float | None = None
    location_y: float | None = None
    created_at: str
    active: bool = True


class SafetyIncidentEvent(BaseModel):
    """하나의 안전 사건 안에서 발생한 순차 조치."""

    stage: Literal[
        "DETECTED",
        "ROBOTS_STOPPED",
        "TASK_PAUSED",
        "OBSTACLE_CLEARED",
        "OPERATION_RESUMED",
    ]
    message: str
    created_at: str


class SafetyIncident(BaseModel):
    """장애물 감지부터 자동 재개까지 추적하는 관제 안전 사건."""

    id: int
    alert_id: int
    status: Literal["MONITORING", "SAFETY_STOPPED", "RECOVERED"]
    sensor_id: str | None = None
    zone_id: str | None = None
    location_x: float | None = None
    location_y: float | None = None
    affected_robot_ids: list[str] = Field(default_factory=list)
    affected_request_ids: list[int] = Field(default_factory=list)
    detected_at: str
    resolved_at: str | None = None
    events: list[SafetyIncidentEvent] = Field(default_factory=list)


class SupportPointState(BaseModel):
    """차량 타이어 한 곳을 받치는 두 암 관절의 요약 상태."""

    id: str
    label: str
    robot_id: str
    joint_names: list[str] = Field(default_factory=list)
    command_percent: float | None = None
    actual_percent: float | None = None
    supported: bool | None = None


class CooperativeLoadState(BaseModel):
    """두 로봇의 차축 정렬·리프트 상태를 관제 화면용으로 정규화한 값."""

    request_id: int
    vehicle_number: str
    lead_robot_id: str | None = None
    follow_robot_id: str | None = None
    front_alignment_error_mm: float | None = None
    rear_alignment_error_mm: float | None = None
    support_points: list[SupportPointState] = Field(default_factory=list)
    lift_command_percent: float | None = None
    vehicle_rise_mm: float | None = None
    pitch_deg: float | None = None
    roll_deg: float | None = None
    tire_support_count: int | None = None
    synchronized: bool | None = None
    stable: bool | None = None
    slip_suspected: bool | None = None
    load_anomaly_suspected: bool | None = None
    source: Literal["MOCK", "MEASURED_ESTIMATED", "UNAVAILABLE"] = "UNAVAILABLE"
    #: 협동 적재 관련 ROS 토픽 중 가장 최근 표본의 경과 시간과 합산 수신률.
    #: updated_at은 상태를 만든 시각이고, 아래 두 값이 실제 통신 신선도를 뜻한다.
    telemetry_age_sec: float | None = None
    telemetry_rate_hz: float | None = None
    updated_at: str | None = None


class VisionAlignmentState(BaseModel):
    """ArUco 검출 결과와 위치 보정 상태의 웹 표시용 계약."""

    robot_id: str
    camera: str = "front"
    connected: bool = False
    marker_detected: bool = False
    marker_id: int | None = None
    distance_m: float | None = None
    reprojection_error_px: float | None = None
    lateral_error_mm: float | None = None
    longitudinal_error_mm: float | None = None
    yaw_error_deg: float | None = None
    marker_corners: list[list[float]] = Field(default_factory=list)
    target_center: list[float] | None = None
    alignment_state: Literal[
        "NO_DATA", "SEARCHING", "ADJUSTING", "ALIGNED"
    ] = "NO_DATA"
    source: Literal["MOCK", "ARUCO_FUSED", "UNAVAILABLE"] = "UNAVAILABLE"
    updated_at: str | None = None
