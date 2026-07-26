"""장애물 범위에 따라 신규 입·출차 요청이 안전하게 차단되는지 검증한다."""

import time

import pytest

from core.datasource import DataSourceError
from core.models import (
    Alert,
    AlertCategory,
    AlertLevel,
    ParkingRequestCreate,
    RequestType,
)
from core.obstacle_scope import blocking_obstacle
from core.state_store import StateStore
from sources.mock_source import MockDataSource
from sources.ros2_source import Ros2DataSource


def _alert(zone_id: str) -> Alert:
    return Alert(
        id=1,
        level=AlertLevel.WARNING,
        category=AlertCategory.OBSTACLE,
        message=f"통로 막힘: {zone_id}",
        sensor_id="L1",
        zone_id=zone_id,
        created_at="2026-07-27T00:00:00",
    )


def test_entry_obstacle_blocks_entry_but_not_exit_request():
    alert = _alert("ZIN03")

    assert blocking_obstacle([alert], RequestType.PARK_IN) is alert
    assert blocking_obstacle([alert], RequestType.PARK_OUT) is None


def test_unknown_or_cross_team_obstacle_blocks_both_request_types():
    alert = _alert("ZIN03, ZOUT01")

    assert blocking_obstacle([alert], RequestType.PARK_IN) is alert
    assert blocking_obstacle([alert], RequestType.PARK_OUT) is alert


def test_mock_rejects_entry_before_reserving_slot_or_assigning_robots():
    store = StateStore()
    source = MockDataSource(store)
    source.trigger_obstacle()

    with pytest.raises(DataSourceError) as caught:
        source.create_request(ParkingRequestCreate(
            request_type=RequestType.PARK_IN,
            vehicle_number="11가1111",
        ))

    assert caught.value.status_code == 409
    assert "입차 통로" in caught.value.detail
    snapshot = store.snapshot()
    assert snapshot["requests"] == []
    assert all(robot.status == "IDLE" for robot in snapshot["robots"])
    assert next(slot for slot in snapshot["slots"] if slot.id == "A2").status == "EMPTY"


def test_ros2_rejects_before_dispatch_service_call():
    store = StateStore()
    source = Ros2DataSource(store)
    source._safety_state.update(state="NORMAL", motion_allowed=True)
    source._last_safety_state_at = time.monotonic()
    store.alerts.append(_alert("ZIN03"))

    with pytest.raises(DataSourceError) as caught:
        source.create_request(ParkingRequestCreate(
            request_type=RequestType.PARK_IN,
            vehicle_number="11가1111",
        ))

    assert caught.value.status_code == 409
    assert "장애물 해소 후" in caught.value.detail
