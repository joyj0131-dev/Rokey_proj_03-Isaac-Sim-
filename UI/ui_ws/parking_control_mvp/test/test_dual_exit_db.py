from core.state_store import StateStore
from sources.ros2_dual_source import Ros2DualDataSource


class _FakeControlDb:
    def __init__(self, tasks):
        self._tasks = tasks

    def fetch_slots(self):
        return [
            {
                "slot_id": slot_id,
                "status": "EMPTY",
                "x": x,
                "y": 0.0,
                "is_accessible": False,
            }
            for slot_id, x in (("A1", 2.8), ("A2", 6.2), ("A3", 9.6))
        ]

    def fetch_tasks(self):
        return self._tasks

    def fetch_robots(self):
        return [
            {
                "robot_id": robot_id,
                "status": "IDLE",
                "battery_percent": 100,
                "x": x,
                "y": -2.2,
            }
            for robot_id, x in (("exit_lead", -3.2), ("exit_follow", -1.2))
        ]


def _task(request_type, state, vehicle_id="3333"):
    return {
        "task_id": f"{request_type}-{state}",
        "request_type": request_type,
        "state": state,
        "vehicle_id": vehicle_id,
        "robot_id": None,
        "follower_robot_id": None,
        "slot_id": "A3",
        "created_at": "2026-07-28T00:00:00",
        "updated_at": "2026-07-28T00:00:00",
    }


def test_completed_entry_exposes_a3_vehicle_for_exit_preflight():
    store = StateStore()
    source = Ros2DualDataSource(store)
    source._control_db = _FakeControlDb([_task("ENTRY", "DONE")])

    source._poll_control_db_once()

    a3 = next(slot for slot in store.parking_slots if slot.id == "A3")
    assert a3.status == "OCCUPIED"
    assert a3.vehicle_number == "3333"
    assert {robot.id: robot.status for robot in store.robots} == {
        "exit_lead": "IDLE",
        "exit_follow": "IDLE",
    }


def test_latest_completed_exit_keeps_a3_empty():
    store = StateStore()
    source = Ros2DualDataSource(store)
    # DB reader가 최신순으로 반환한다. 최신 EXIT가 이전 ENTRY보다 우선해야 한다.
    source._control_db = _FakeControlDb(
        [_task("EXIT", "DONE"), _task("ENTRY", "DONE")]
    )

    source._poll_control_db_once()

    a3 = next(slot for slot in store.parking_slots if slot.id == "A3")
    assert a3.status == "EMPTY"
    assert a3.vehicle_number is None
