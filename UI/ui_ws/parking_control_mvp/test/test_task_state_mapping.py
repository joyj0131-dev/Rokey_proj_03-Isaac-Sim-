"""ROS2 작업 상태가 관제 6단계와 일치하는지 검증한다."""

from parking_robot_interfaces.msg import TaskState

from core.models import AlertCategory, RequestStatus
from core.state_store import StateStore
from sources.ros2_source import Ros2DataSource


def _task_state(task_id: str, state: str) -> TaskState:
    message = TaskState()
    message.task_id = task_id
    message.robot_id = "entry_lead"
    message.state = state
    return message


def test_orchestrator_states_advance_beyond_robot_assigned():
    source = Ros2DataSource(StateStore())
    task_id = "task-state-sequence"

    expected = [
        ("SEARCHING", RequestStatus.APPROACHING),
        ("APPROACHING", RequestStatus.APPROACHING),
        ("PICKED_UP", RequestStatus.LIFTING),
        ("MOVING", RequestStatus.MOVING_TO_SLOT),
        ("ARRIVED", RequestStatus.MOVING_TO_SLOT),
        ("PARKED", RequestStatus.MOVING_TO_SLOT),
        ("RETURNING", RequestStatus.RETURNING),
        ("DONE", RequestStatus.COMPLETED),
    ]

    for state, status in expected:
        source._on_task_state(_task_state(task_id, state))
        assert source._fine_status[task_id] == status


def test_failed_orchestrator_state_maps_to_cancelled():
    source = Ros2DataSource(StateStore())
    task_id = "task-state-failed"

    source._on_task_state(_task_state(task_id, "FAILED"))

    assert source._fine_status[task_id] == RequestStatus.CANCELLED


def test_emergency_stop_failure_is_not_reported_as_robot_error():
    source = Ros2DataSource(StateStore())
    message = _task_state("task-emergency-stop", "FAILED")
    message.current_step = "관제 UI 전체 비상정지"

    source._on_task_state(message)

    assert source._fine_status[message.task_id] == RequestStatus.CANCELLED
    assert not any(
        alert.category == AlertCategory.ROBOT_ERROR
        for alert in source.store.snapshot()["alerts"]
    )


def test_real_control_failure_is_reported_once_as_robot_error():
    source = Ros2DataSource(StateStore())
    message = _task_state("task-drive-failure", "FAILED")
    message.current_step = "좌측 구동부 통신 실패"

    source._on_task_state(message)
    source._on_task_state(message)

    alerts = source.store.snapshot()["alerts"]
    robot_errors = [
        alert for alert in alerts
        if alert.category == AlertCategory.ROBOT_ERROR
    ]
    assert len(robot_errors) == 1
    assert "구동부 통신 실패" in robot_errors[0].message
