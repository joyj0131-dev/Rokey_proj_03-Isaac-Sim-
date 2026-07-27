import pytest

from core.datasource import DataSourceError
from core.models import (
    AlertCategory,
    OperationApprovalRequest,
    ParkingRequestCreate,
    RobotRecoveryRequest,
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


def test_mock_recovery_requires_inspection_dock_return_then_operation_approval():
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

    assert source.recovery_state["status"] == "REQUIRED"
    assert all(
        source.store.find_robot(robot_id).status == "RECOVERY_REQUIRED"
        for robot_id in request.robot_ids
    )

    with pytest.raises(DataSourceError) as early_approval:
        source.approve_operation(OperationApprovalRequest(
            operator_id="supervisor-1",
            approval_note="복귀 전 정상 운영 승인 시도",
        ))
    assert early_approval.value.status_code == 409

    source.start_safe_recovery(RobotRecoveryRequest(
        operator_id="operator-1",
        recovery_note="복귀 경로와 도크 확인 완료",
        path_clear=True,
        load_cleared=True,
        arms_retracted=True,
        sensors_ready=True,
    ))
    with source.store.lock:
        source._advance_safe_recovery(elapsed=20.0)
    assert source.recovery_state["status"] == "COMPLETED"
    assert source.safety_state["state"] == "READY_FOR_OPERATION"
    assert source.safety_state["motion_allowed"] is False

    source.approve_operation(OperationApprovalRequest(
        operator_id="supervisor-1",
        approval_note="도크 도달 확인 후 정상 운영 승인",
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


def test_mock_stopped_robots_need_separate_recovery_and_become_idle_at_docks():
    source = MockDataSource(StateStore())
    request = source.create_request(ParkingRequestCreate(
        request_type=RequestType.PARK_IN,
        vehicle_number="77가7777",
    ))
    source.advance_request(request.id)  # 접근 단계
    with source.store.lock:
        source._move_pair_along_route(
            source.store.find_request(request.id), elapsed=1.0
        )
        stopped_positions = {
            robot_id: (
                source.store.find_robot(robot_id).x,
                source.store.find_robot(robot_id).y,
            )
            for robot_id in request.robot_ids
        }

    source.emergency_stop()

    assert all(
        source.store.find_robot(robot_id).status == "SAFETY_STOPPED"
        for robot_id in request.robot_ids
    )
    assert all(
        (
            source.store.find_robot(robot_id).x,
            source.store.find_robot(robot_id).y,
        ) == stopped_positions[robot_id]
        for robot_id in request.robot_ids
    )

    source.request_safety_reset(SafetyResetRequest(
        operator_id="operator-1",
        inspection_note="정지 위치와 차량 상태 확인 완료",
        area_clear=True,
        robots_stopped=True,
        load_secured=True,
        sensors_checked=True,
    ))
    source.start_safe_recovery(RobotRecoveryRequest(
        operator_id="operator-1",
        recovery_note="전용 통로와 도크 위치 확인 완료",
        path_clear=True,
        load_cleared=True,
        arms_retracted=True,
        sensors_ready=True,
    ))
    assert source.recovery_state["status"] == "RECOVERING"
    assert all(
        source.store.find_robot(robot_id).status == "RECOVERING"
        for robot_id in request.robot_ids
    )

    with source.store.lock:
        for _step in range(4):
            source._advance_safe_recovery(elapsed=20.0)

    assert source.recovery_state["status"] == "COMPLETED"
    assert source.safety_state["state"] == "READY_FOR_OPERATION"
    source.approve_operation(OperationApprovalRequest(
        operator_id="supervisor-1",
        approval_note="도크 위치 확인 후 정상 운영 승인",
    ))
    assert all(
        source.store.find_robot(robot_id).status == "IDLE"
        for robot_id in request.robot_ids
    )
    docks = source.get_map_info()["docks"]
    dock_positions = {(dock["x"], dock["y"]) for dock in docks}
    assert all(
        (
            source.store.find_robot(robot_id).x,
            source.store.find_robot(robot_id).y,
        ) in dock_positions
        for robot_id in request.robot_ids
    )


def test_mock_emergency_stop_during_recovery_keeps_recovery_targets():
    source = MockDataSource(StateStore())
    request = source.create_request(ParkingRequestCreate(
        request_type=RequestType.PARK_IN,
        vehicle_number="66가6666",
    ))
    source.emergency_stop()
    source.request_safety_reset(SafetyResetRequest(
        operator_id="operator-1",
        inspection_note="현장 안전 상태 확인 완료",
        area_clear=True,
        robots_stopped=True,
        load_secured=True,
        sensors_checked=True,
    ))
    source.start_safe_recovery(RobotRecoveryRequest(
        operator_id="operator-1",
        recovery_note="복귀 조건 확인 완료",
        path_clear=True,
        load_cleared=True,
        arms_retracted=True,
        sensors_ready=True,
    ))

    source.emergency_stop()

    assert source.recovery_state["status"] == "SAFETY_STOPPED"
    assert source.recovery_state["robot_ids"] == request.robot_ids
    assert source.recovery_state["source_request_ids"] == [request.id]
    assert all(
        source.store.find_robot(robot_id).status == "SAFETY_STOPPED"
        for robot_id in request.robot_ids
    )
