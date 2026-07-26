"""ROS2 작업 상태가 관제 6단계와 일치하는지 검증한다."""

from parking_robot_interfaces.msg import TaskState

from core.models import RequestStatus
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
