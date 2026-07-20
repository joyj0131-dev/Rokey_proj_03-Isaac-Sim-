"""주차로봇 관제 웹 서버 (웹 계층).

비즈니스 로직은 core/ 및 sources/ 에 있으며, 이 파일은 라우팅과
HTTP 변환만 담당한다. PARKING_MODE 환경변수로 데이터 소스를 선택한다.
"""

from contextlib import asynccontextmanager
from pathlib import Path

from fastapi import FastAPI, HTTPException
from fastapi.responses import FileResponse
from fastapi.staticfiles import StaticFiles

import config
from core.datasource import DataSource, DataSourceError
from core.models import (
    Alert,
    ParkingRequest,
    ParkingRequestCreate,
    RequestStatus,
)
from core.state_store import StateStore
from sources.mock_source import MockDataSource

BASE_DIR = Path(__file__).resolve().parent
STATIC_DIR = BASE_DIR / "static"

store = StateStore()


def _create_datasource() -> DataSource:
    if config.PARKING_MODE == "mock":
        return MockDataSource(store)

    # 지연 import: rclpy/parking_robot_interfaces는 ROS2 환경이 source된
    # 상태에서만 존재하므로, mock 모드 실행 시에는 아예 건드리지 않는다.
    from sources.ros2_source import Ros2DataSource

    return Ros2DataSource(store)


datasource: DataSource = _create_datasource()


@asynccontextmanager
async def lifespan(app: FastAPI):
    datasource.start()
    yield
    datasource.stop()


app = FastAPI(
    title="Parking Robot Control API",
    description="주차로봇 관제 웹 UI API (mock / ros2 모드 지원 구조)",
    version="0.3.0",
    lifespan=lifespan,
)

app.mount("/static", StaticFiles(directory=STATIC_DIR), name="static")


def _handle(func, *args, **kwargs):
    """DataSourceError를 HTTPException으로 변환한다."""
    try:
        return func(*args, **kwargs)
    except DataSourceError as error:
        raise HTTPException(
            status_code=error.status_code, detail=error.detail
        ) from error


def _require_mock_controls() -> None:
    if not datasource.supports_mock_controls:
        raise HTTPException(
            status_code=403,
            detail="현재 모드에서는 Mock 제어를 사용할 수 없습니다.",
        )


# ----------------------------------------------------------------------
# 페이지
# ----------------------------------------------------------------------
@app.get("/")
def read_index():
    return FileResponse(STATIC_DIR / "index.html")


# ----------------------------------------------------------------------
# 조회
# ----------------------------------------------------------------------
@app.get("/api/system")
def get_system():
    """실행 모드와 시스템 상태 요약."""
    snapshot = store.snapshot()
    alerts = snapshot["alerts"]

    has_error = any(alert.level == "ERROR" for alert in alerts)
    has_warning = any(alert.level == "WARNING" for alert in alerts)

    health = "ERROR" if has_error else "WARNING" if has_warning else "OK"

    return {
        "mode": config.PARKING_MODE,
        "mock_controls": datasource.supports_mock_controls,
        "health": health,
    }


@app.get("/api/dashboard")
def get_dashboard():
    snapshot = store.snapshot()
    slots = snapshot["slots"]
    requests = snapshot["requests"]
    alerts = snapshot["alerts"]

    has_error = any(alert.level == "ERROR" for alert in alerts)
    has_warning = any(alert.level == "WARNING" for alert in alerts)

    return {
        "robots": snapshot["robots"],
        "slots": slots,
        "requests": list(reversed(requests)),
        "alerts": list(reversed(alerts)),
        "summary": {
            "empty_slots": sum(slot.status == "EMPTY" for slot in slots),
            "occupied_slots": sum(slot.status == "OCCUPIED" for slot in slots),
            "active_requests": sum(
                request.status
                not in {RequestStatus.COMPLETED, RequestStatus.CANCELLED}
                for request in requests
            ),
        },
        "system": {
            "mode": config.PARKING_MODE,
            "mock_controls": datasource.supports_mock_controls,
            "health": (
                "ERROR" if has_error else "WARNING" if has_warning else "OK"
            ),
        },
    }


# ----------------------------------------------------------------------
# 요청 처리
# ----------------------------------------------------------------------
@app.post("/api/requests", response_model=ParkingRequest, status_code=201)
def create_request(payload: ParkingRequestCreate):
    return _handle(datasource.create_request, payload)


@app.post("/api/requests/{request_id}/advance", response_model=ParkingRequest)
def advance_request(request_id: int):
    _require_mock_controls()
    return _handle(datasource.advance_request, request_id)


# ----------------------------------------------------------------------
# 알림
# ----------------------------------------------------------------------
@app.post("/api/alerts/{alert_id}/resolve")
def resolve_alert(alert_id: int):
    _handle(datasource.resolve_alert, alert_id)
    return {"message": "알림이 해제되었습니다."}


# ----------------------------------------------------------------------
# Mock 제어 (mock 모드 전용)
# ----------------------------------------------------------------------
@app.post("/api/reset")
def reset_mock_data():
    _require_mock_controls()
    _handle(datasource.reset)
    return {"message": "Mock 데이터가 초기화되었습니다."}


@app.post("/api/mock/obstacle", response_model=Alert, status_code=201)
def trigger_obstacle():
    _require_mock_controls()
    return _handle(datasource.trigger_obstacle)


@app.post("/api/mock/robot-error", response_model=Alert, status_code=201)
def trigger_robot_error():
    _require_mock_controls()
    return _handle(datasource.trigger_robot_error)
