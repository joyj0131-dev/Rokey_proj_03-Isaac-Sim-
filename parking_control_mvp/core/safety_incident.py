"""장애물 경보의 감지·정지·복구를 하나의 안전 사건으로 기록한다."""

from datetime import datetime

from .models import (
    TERMINAL_STATUSES,
    Alert,
    SafetyIncident,
    SafetyIncidentEvent,
)
from .obstacle_scope import obstacle_affects_request, obstacle_scope


def _now() -> str:
    return datetime.now().isoformat(timespec="seconds")


def _robot_role(robot_id: str) -> str | None:
    normalized = str(robot_id or "").lower()
    if normalized.startswith("entry"):
        return "entry"
    if normalized.startswith("exit"):
        return "exit"
    return None


def _affected_robots(store, alert: Alert) -> list[str]:
    scope = obstacle_scope(alert.zone_id, alert.message, alert.location_y)
    affected = []
    for robot in store.robots:
        if robot.status != "BUSY" and robot.current_task_id is None:
            continue
        role = _robot_role(robot.id)
        if scope is None or role is None or role == scope:
            affected.append(robot.id)
    return affected


def _affected_requests(store, alert: Alert, robot_ids: list[str]) -> list[int]:
    robot_id_set = set(robot_ids)
    affected = []
    for request in store.requests:
        if request.status in TERMINAL_STATUSES:
            continue
        assigned = set(
            request.robot_ids
            or ([request.robot_id] if request.robot_id else [])
        )
        if assigned.intersection(robot_id_set) or obstacle_affects_request(
            alert, request.request_type
        ):
            affected.append(request.id)
    return affected


def open_obstacle_incident(store, alert: Alert) -> SafetyIncident:
    """활성 장애물 경보에 대응하는 안전 사건을 생성하거나 최신화한다.

    호출자는 ``store.lock``을 잡은 상태여야 한다.
    """
    existing = store.find_safety_incident_by_alert(alert.id)
    if existing is not None:
        existing.sensor_id = alert.sensor_id
        existing.zone_id = alert.zone_id
        existing.location_x = alert.location_x
        existing.location_y = alert.location_y
        return existing

    robot_ids = _affected_robots(store, alert)
    request_ids = _affected_requests(store, alert, robot_ids)
    created_at = alert.created_at or _now()
    events = [
        SafetyIncidentEvent(
            stage="DETECTED",
            message=(
                f"{alert.sensor_id or '센서'} · "
                f"{alert.zone_id or '주행 통로'} 장애물 감지"
            ),
            created_at=created_at,
        )
    ]
    if robot_ids:
        events.append(
            SafetyIncidentEvent(
                stage="ROBOTS_STOPPED",
                message=f"영향 로봇 {', '.join(robot_ids)} 일시정지",
                created_at=created_at,
            )
        )
    if request_ids:
        events.append(
            SafetyIncidentEvent(
                stage="TASK_PAUSED",
                message=(
                    "관련 작업 "
                    + ", ".join(f"#{request_id}" for request_id in request_ids)
                    + " 안전정지"
                ),
                created_at=created_at,
            )
        )

    incident = SafetyIncident(
        id=store.next_safety_incident_id(),
        alert_id=alert.id,
        status="SAFETY_STOPPED" if robot_ids or request_ids else "MONITORING",
        sensor_id=alert.sensor_id,
        zone_id=alert.zone_id,
        location_x=alert.location_x,
        location_y=alert.location_y,
        affected_robot_ids=robot_ids,
        affected_request_ids=request_ids,
        detected_at=created_at,
        events=events,
    )
    store.safety_incidents.append(incident)
    return incident


def recover_obstacle_incident(
    store,
    alert: Alert,
    resolved_at: str | None = None,
) -> SafetyIncident | None:
    """장애물 해소와 자동 재개를 동일 사건의 후속 단계로 기록한다."""
    incident = store.find_safety_incident_by_alert(alert.id)
    if incident is None or incident.status == "RECOVERED":
        return incident

    timestamp = resolved_at or _now()
    incident.status = "RECOVERED"
    incident.resolved_at = timestamp
    incident.events.extend(
        [
            SafetyIncidentEvent(
                stage="OBSTACLE_CLEARED",
                message="센서 장애물 해소 확인",
                created_at=timestamp,
            ),
            SafetyIncidentEvent(
                stage="OPERATION_RESUMED",
                message=(
                    "관련 로봇·작업 자동 재개"
                    if incident.affected_robot_ids or incident.affected_request_ids
                    else "통로 신규 진입 제한 해제"
                ),
                created_at=timestamp,
            ),
        ]
    )
    return incident
