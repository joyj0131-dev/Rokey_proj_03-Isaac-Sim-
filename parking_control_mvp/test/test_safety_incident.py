"""장애물 감지부터 작업 재개까지 하나의 안전 사건으로 추적하는지 검증한다."""

from core.models import (
    Alert,
    AlertCategory,
    AlertLevel,
    ParkingRequestCreate,
    RequestStatus,
    RequestType,
)
from core.safety_incident import open_obstacle_incident
from core.state_store import StateStore
from sources.mock_source import MockDataSource


def test_obstacle_incident_links_detection_stop_task_and_recovery():
    store = StateStore()
    source = MockDataSource(store)
    request = source.create_request(
        ParkingRequestCreate(
            request_type=RequestType.PARK_IN,
            vehicle_number="11가1111",
        )
    )
    source.advance_request(request.id)

    alert = source.trigger_obstacle()
    snapshot = store.snapshot()

    assert len(snapshot["safety_incidents"]) == 1
    incident = snapshot["safety_incidents"][0]
    assert incident.alert_id == alert.id
    assert incident.status == "SAFETY_STOPPED"
    assert incident.sensor_id == "L1"
    assert incident.zone_id == "ZIN03"
    assert set(incident.affected_robot_ids) == {"entry_lead", "entry_follow"}
    assert incident.affected_request_ids == [request.id]
    assert [event.stage for event in incident.events] == [
        "DETECTED",
        "ROBOTS_STOPPED",
        "TASK_PAUSED",
    ]

    source.resolve_alert(alert.id)
    recovered_snapshot = store.snapshot()
    recovered = recovered_snapshot["safety_incidents"][0]

    assert recovered.status == "RECOVERED"
    assert recovered.resolved_at is not None
    assert [event.stage for event in recovered.events][-2:] == [
        "OBSTACLE_CLEARED",
        "OPERATION_RESUMED",
    ]
    assert recovered_snapshot["alerts"] == []
    active_request = recovered_snapshot["requests"][0]
    assert active_request.status == RequestStatus.APPROACHING


def test_repeated_obstacle_update_keeps_single_incident_identity():
    store = StateStore()
    source = MockDataSource(store)

    alert = source.trigger_obstacle()
    with store.lock:
        stored_alert = store.find_alert(alert.id)
        stored_alert.location_x = 1.25
        stored_alert.location_y = -6.75
        updated = open_obstacle_incident(store, stored_alert)

    assert len(store.safety_incidents) == 1
    assert updated.id == store.safety_incidents[0].id
    assert updated.location_x == 1.25
    assert updated.location_y == -6.75


def test_incident_without_active_work_is_monitoring_and_recovers():
    store = StateStore()
    source = MockDataSource(store)

    alert = source.trigger_obstacle()
    incident = store.snapshot()["safety_incidents"][0]

    assert incident.status == "MONITORING"
    assert incident.affected_robot_ids == []
    assert incident.affected_request_ids == []
    assert [event.stage for event in incident.events] == ["DETECTED"]

    source.resolve_alert(alert.id)
    assert store.snapshot()["safety_incidents"][0].status == "RECOVERED"


def test_location_y_scopes_incident_to_entry_team_when_zone_is_missing():
    store = StateStore()
    source = MockDataSource(store)
    entry = source.create_request(
        ParkingRequestCreate(
            request_type=RequestType.PARK_IN,
            vehicle_number="22나2222",
        )
    )
    exit_request = source.create_request(
        ParkingRequestCreate(
            request_type=RequestType.PARK_OUT,
            vehicle_number="12가3456",
        )
    )
    alert = Alert(
        id=store.next_alert_id(),
        level=AlertLevel.WARNING,
        category=AlertCategory.OBSTACLE,
        message="위치 기반 장애물 감지",
        sensor_id="L1",
        location_x=-4.0,
        location_y=-6.875,
        created_at="2026-07-27T03:00:00",
    )

    with store.lock:
        store.alerts.append(alert)
        incident = open_obstacle_incident(store, alert)

    assert set(incident.affected_robot_ids) == {"entry_lead", "entry_follow"}
    assert incident.affected_request_ids == [entry.id]
    assert exit_request.id not in incident.affected_request_ids
