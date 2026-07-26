import pytest

from core.datasource import DataSourceError
from core.models import (
    AlertCategory,
    OperationApprovalRequest,
    ParkingRequestCreate,
    RequestType,
    SafetyResetRequest,
)
from core.state_store import StateStore
from sources.mock_source import MockDataSource


def test_mock_emergency_stop_latches_blocks_new_requests_and_adds_alert():
    source = MockDataSource(StateStore())

    affected = source.emergency_stop()

    assert affected == 0
    assert source.emergency_stop_active is True
    alert = source.store.snapshot()["alerts"][-1]
    assert alert.category == AlertCategory.EMERGENCY_STOP

    with pytest.raises(DataSourceError) as error:
        source.create_request(ParkingRequestCreate(
            request_type=RequestType.PARK_IN,
            vehicle_number="12가3456",
        ))
    assert error.value.status_code == 423

    with pytest.raises(DataSourceError) as reset_error:
        source.reset()
    assert reset_error.value.status_code == 423


def test_mock_emergency_stop_is_idempotent():
    source = MockDataSource(StateStore())

    source.emergency_stop()
    source.emergency_stop()

    alerts = source.store.snapshot()["alerts"]
    assert sum(
        alert.category == AlertCategory.EMERGENCY_STOP
        for alert in alerts
    ) == 1


def test_mock_recovery_requires_two_deliberate_steps_and_never_resumes_task():
    source = MockDataSource(StateStore())
    request = source.create_request(ParkingRequestCreate(
        request_type=RequestType.PARK_IN,
        vehicle_number="99하9999",
    ))
    source.emergency_stop()

    stopped_request = source.store.find_request(request.id)
    assert stopped_request.status.value == "CANCELLED"
    assert source.safety_state["state"] == "STOPPED_LATCHED"

    source.request_safety_reset(SafetyResetRequest(
        operator_id="operator-1",
        inspection_note="현장과 차량 지지 상태 확인 완료",
        area_clear=True,
        robots_stopped=True,
        load_secured=True,
        sensors_checked=True,
    ))

    assert source.safety_state["state"] == "READY_FOR_OPERATION"
    assert source.safety_state["motion_allowed"] is False
    assert source.emergency_stop_active is True
    assert source.store.find_request(request.id).status.value == "CANCELLED"

    source.approve_operation(OperationApprovalRequest(
        operator_id="supervisor-1",
        approval_note="현재 위치 기준 새 작업 접수 승인",
    ))

    assert source.safety_state["state"] == "NORMAL"
    assert source.safety_state["motion_allowed"] is True
    assert source.emergency_stop_active is False
    assert source.store.find_request(request.id).status.value == "CANCELLED"


def test_mock_recovery_rejects_incomplete_inspection():
    source = MockDataSource(StateStore())
    source.emergency_stop()

    with pytest.raises(DataSourceError) as error:
        source.request_safety_reset(SafetyResetRequest(
            operator_id="operator-1",
            inspection_note="센서 점검을 제외한 현장 확인 완료",
            area_clear=True,
            robots_stopped=True,
            load_secured=True,
            sensors_checked=False,
        ))

    assert error.value.status_code == 409
    assert source.safety_state["state"] == "STOPPED_LATCHED"
