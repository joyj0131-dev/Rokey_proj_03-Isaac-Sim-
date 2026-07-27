"""웹 LiDAR 상세 화면용 경량 포인트클라우드 변환.

원본 PointCloud2 전체를 브라우저에 보내지 않는다. 슬롯별 점유 수는 전체
포인트로 계산하고, 화면에는 일정 간격으로 추린 2D 점만 전달한다.
"""

from __future__ import annotations

import math
from typing import Any, Sequence


HEIGHT_THRESHOLD_M = 0.15
POINT_THRESHOLD = 30
SLOT_WIDTH_M = 3.4
SLOT_LENGTH_M = 6.6
MAX_DISPLAY_POINTS = 1800
DEFAULT_BOUNDS = {
    "min_x": -24.0,
    "max_x": 14.0,
    "min_y": -12.0,
    "max_y": 12.0,
}


def _value(item: Any, name: str, default: Any = None) -> Any:
    if isinstance(item, dict):
        return item.get(name, default)
    try:
        return item[name]
    except (IndexError, KeyError, TypeError):
        return getattr(item, name, default)


def _point_xyz(point: Any) -> tuple[float, float, float]:
    if isinstance(point, (tuple, list)):
        return float(point[0]), float(point[1]), float(point[2])
    return (
        float(_value(point, "x", 0.0)),
        float(_value(point, "y", 0.0)),
        float(_value(point, "z", 0.0)),
    )


def _slot_contract(slot: Any) -> dict | None:
    x = _value(slot, "x")
    y = _value(slot, "y")
    if x is None or y is None:
        return None
    return {
        "id": str(_value(slot, "id", "")),
        "x": float(x),
        "y": float(y),
        "width": SLOT_WIDTH_M,
        "length": SLOT_LENGTH_M,
        "control_status": str(_value(slot, "status", "UNKNOWN")),
    }


def build_lidar_visualization(
    points: Sequence,
    slots: Sequence,
    *,
    sensor_id: str = "L1",
    sensor_status: str = "OFFLINE",
    topic: str = "/parking/lidar/points_world",
    frame_id: str = "map",
    sensor_x: float = 0.5,
    sensor_y: float = 0.0,
    rate_hz: float | None = None,
    last_seen_sec: float | None = None,
    source: str = "ROS2",
    max_display_points: int = MAX_DISPLAY_POINTS,
) -> dict:
    """원본 XYZ 점과 슬롯을 웹 표시용 JSON 호환 딕셔너리로 변환한다."""
    slot_items = [
        contract for slot in slots
        if (contract := _slot_contract(slot)) is not None
    ]
    slot_counts = {slot["id"]: 0 for slot in slot_items}
    ignored_points: list[list[float]] = []
    used_points: list[list[float]] = []
    slot_used_points: list[list[float]] = []

    point_total = len(points)
    stride = max(1, math.ceil(point_total / max(1, max_display_points)))
    sampled_total = 0
    valid_point_count = 0
    slot_point_count = 0

    for index, point in enumerate(points):
        x, y, z = _point_xyz(point)
        if not (math.isfinite(x) and math.isfinite(y) and math.isfinite(z)):
            continue

        used = z > HEIGHT_THRESHOLD_M
        matched_slot = False
        if used:
            valid_point_count += 1
            for slot in slot_items:
                if (
                    abs(x - slot["x"]) < slot["width"] / 2
                    and abs(y - slot["y"]) < slot["length"] / 2
                ):
                    slot_counts[slot["id"]] += 1
                    matched_slot = True
            if matched_slot:
                slot_point_count += 1

        if index % stride != 0 or sampled_total >= max_display_points:
            continue
        target = (
            slot_used_points
            if matched_slot
            else used_points
            if used
            else ignored_points
        )
        target.append([round(x, 3), round(y, 3)])
        sampled_total += 1

    rendered_slots = []
    has_live_measurement = sensor_status in {"ONLINE", "MOCK"}
    for slot in slot_items:
        count = slot_counts[slot["id"]]
        occupied = count >= POINT_THRESHOLD
        lidar_status = (
            "UNAVAILABLE"
            if not has_live_measurement
            else "OCCUPIED"
            if occupied
            else "EMPTY"
        )
        control_status = slot["control_status"]
        status_match = (
            lidar_status == control_status
            if lidar_status in {"OCCUPIED", "EMPTY"}
            and control_status in {"OCCUPIED", "EMPTY"}
            else None
        )
        rendered_slots.append(
            {
                **slot,
                "status": lidar_status,
                "status_match": status_match,
                "point_count": count,
            }
        )

    occupied_count = sum(
        slot["status"] == "OCCUPIED" for slot in rendered_slots
    )
    mismatch_count = sum(
        slot["status_match"] is False for slot in rendered_slots
    )
    return {
        "sensor_id": sensor_id,
        "sensor_status": sensor_status,
        "topic": topic,
        "source": source,
        "frame_id": frame_id,
        "coordinate_status": (
            "WAITING"
            if not has_live_measurement
            else "OK"
            if frame_id == "map"
            else "CHECK"
        ),
        "measurement_status": (
            "OK" if has_live_measurement and point_total else "NO_DATA"
        ),
        "status_message": (
            "정상 수신"
            if has_live_measurement and point_total
            else "PointCloud2 메시지 수신 대기"
        ),
        "sensor_position": {
            "x": sensor_x,
            "y": sensor_y,
        },
        "rate_hz": rate_hz,
        "last_seen_sec": last_seen_sec,
        "height_threshold_m": HEIGHT_THRESHOLD_M,
        "point_threshold": POINT_THRESHOLD,
        "bounds": dict(DEFAULT_BOUNDS),
        "point_total": point_total,
        "valid_point_count": valid_point_count,
        "slot_point_count": slot_point_count,
        "display_point_count": sampled_total,
        "occupied_count": occupied_count,
        "empty_count": sum(
            slot["status"] == "EMPTY" for slot in rendered_slots
        ),
        "uncertain_count": 0,
        "mismatch_count": mismatch_count,
        "total_slots": len(rendered_slots),
        "slots": rendered_slots,
        "points": {
            "ignored": ignored_points,
            "used": used_points,
            "slot_used": slot_used_points,
        },
    }


def build_mock_pointcloud(slots: Sequence) -> list[tuple[float, float, float]]:
    """실제 센서가 없어도 상세 화면을 검증할 수 있는 고정 샘플 스캔."""
    points: list[tuple[float, float, float]] = []

    # 바닥 스캔 링과 통로의 곡선형 잔상.
    for row in range(13):
        y_base = -4.8 + row * 0.8
        for column in range(95):
            x = -22.0 + column * 0.39
            y = y_base + math.sin(column * 0.18 + row) * 0.22
            points.append((x, y, 0.04 + (column % 4) * 0.015))

    # 벽·기둥처럼 보이는 유효 반사점.
    for column in range(120):
        x = -20.0 + column * 0.29
        points.append((x, -10.6 + math.sin(column * 0.31) * 0.12, 1.2))
        points.append((x, 10.6 + math.cos(column * 0.29) * 0.12, 1.2))
    for row in range(75):
        y = -10.5 + row * 0.28
        points.append((-18.4 + math.sin(row * 0.4) * 0.08, y, 1.0))
        points.append((13.0 + math.cos(row * 0.37) * 0.08, y, 1.0))

    # 슬롯 상태에 맞는 차량 반사점. RESERVED는 임계값 아래의 소량 점만 둔다.
    for slot in slots:
        x = _value(slot, "x")
        y = _value(slot, "y")
        if x is None or y is None:
            continue
        status = str(_value(slot, "status", "EMPTY"))
        target_count = 150 if status == "OCCUPIED" else 18 if status == "RESERVED" else 6
        for index in range(target_count):
            dx = ((index * 17) % 43) / 43.0 * 2.4 - 1.2
            dy = ((index * 29) % 61) / 61.0 * 4.8 - 2.4
            z = 0.45 + (index % 9) * 0.12
            points.append((float(x) + dx, float(y) + dy, z))

    return points
