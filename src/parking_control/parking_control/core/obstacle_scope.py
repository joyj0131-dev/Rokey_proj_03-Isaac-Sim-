"""장애물 경보가 영향을 주는 입·출차 팀을 판정하는 순수 Python 규칙."""

import math
import re


_ZONE_PATTERN = re.compile(r"\b(ZIN|ZOUT)[A-Z0-9_]*\b", re.IGNORECASE)


def robot_team_role(robot_id: str | None) -> str | None:
    """고정 로봇 ID에서 ``entry``/``exit`` 역할을 반환한다."""
    normalized = (robot_id or "").strip().lower()
    if normalized.startswith("entry"):
        return "entry"
    if normalized.startswith("exit"):
        return "exit"
    return None


def obstacle_scope(
    description: str | None = None,
    location_y: float | None = None,
) -> str | None:
    """경보 범위를 ``entry``/``exit``로 판정한다.

    둘 이상의 팀 구역이 함께 포함되거나 어떤 팀인지 판정할 수 없으면
    ``None``을 반환한다. 호출자는 이를 안전 우선의 전역 영향으로 처리한다.
    """
    prefixes = {
        match.group(1).upper()
        for match in _ZONE_PATTERN.finditer(description or "")
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


def obstacle_affects_team(
    team_role: str | None,
    description: str | None = None,
    location_y: float | None = None,
) -> bool:
    """해당 팀이 경보 범위에 포함되는지 반환한다.

    팀 또는 구역을 판정할 수 없으면 잘못된 부분 운행보다 전체 정지가
    안전하므로 ``True``를 반환한다.
    """
    normalized_role = (team_role or "").strip().lower()
    scope = obstacle_scope(description, location_y)
    if normalized_role not in {"entry", "exit"} or scope is None:
        return True
    return normalized_role == scope
