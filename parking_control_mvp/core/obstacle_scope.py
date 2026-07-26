"""관제 API에서 장애물 경보의 입·출차 영향 범위를 판정한다."""

import math
import re


_ZONE_PATTERN = re.compile(r"\b(ZIN|ZOUT)[A-Z0-9_]*\b", re.IGNORECASE)


def obstacle_scope(
    zone_id: str | None = None,
    message: str | None = None,
    location_y: float | None = None,
) -> str | None:
    """경보가 영향을 주는 ``entry``/``exit`` 팀을 반환한다.

    양쪽 구역이 함께 감지되거나 범위를 판정할 수 없으면 ``None``이다.
    호출자는 이 값을 안전 우선의 양쪽 영향으로 취급한다.
    """
    prefixes = {
        match.group(1).upper()
        for match in _ZONE_PATTERN.finditer(f"{zone_id or ''} {message or ''}")
    }
    if prefixes == {"ZIN"}:
        return "entry"
    if prefixes == {"ZOUT"}:
        return "exit"
    if prefixes:
        return None

    if location_y is not None:
        value = float(location_y)
        if math.isfinite(value):
            if value <= -1.0:
                return "entry"
            if value >= 1.0:
                return "exit"
    return None


def request_role(request_type) -> str:
    """Pydantic enum 또는 문자열 요청 유형을 팀 역할로 변환한다."""
    normalized = str(getattr(request_type, "value", request_type)).upper()
    return "entry" if normalized == "PARK_IN" else "exit"


def obstacle_affects_request(alert, request_type) -> bool:
    """활성 장애물 경보가 해당 요청 유형의 통로에 영향을 주는지 반환한다."""
    if not getattr(alert, "active", True):
        return False
    category = getattr(alert, "category", "")
    if str(getattr(category, "value", category)).upper() != "OBSTACLE":
        return False

    scope = obstacle_scope(
        getattr(alert, "zone_id", None),
        getattr(alert, "message", None),
        getattr(alert, "location_y", None),
    )
    return scope is None or scope == request_role(request_type)


def blocking_obstacle(alerts, request_type):
    """해당 요청을 차단하는 첫 활성 장애물 경보를 반환한다."""
    return next(
        (
            alert
            for alert in alerts
            if obstacle_affects_request(alert, request_type)
        ),
        None,
    )


def blocking_request_message(alert, request_type) -> str:
    """API와 UI에서 그대로 표시할 수 있는 명확한 요청 차단 사유."""
    role_label = "입차" if request_role(request_type) == "entry" else "출차"
    sensor = getattr(alert, "sensor_id", None)
    sensor_label = f" ({sensor})" if sensor else ""
    return (
        f"{role_label} 통로에 장애물이 감지되어 요청을 등록할 수 없습니다"
        f"{sensor_label}. 장애물 해소 후 다시 시도해주세요."
    )
