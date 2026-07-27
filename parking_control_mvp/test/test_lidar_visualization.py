"""LiDAR 상세 화면 데이터 계약 회귀 테스트."""

from core.lidar_visualization import (
    HEIGHT_THRESHOLD_M,
    MAX_DISPLAY_POINTS,
    POINT_THRESHOLD,
    build_lidar_visualization,
)
from core.state_store import StateStore
from sources.mock_source import MockDataSource


SLOTS = [
    {"id": "A1", "x": 2.8, "y": 0.0},
    {"id": "A2", "x": 6.2, "y": 0.0},
    {"id": "A3", "x": 9.6, "y": 0.0},
]


def test_height_filter_and_slot_point_threshold_are_exposed():
    points = [
        (2.8, 0.0, HEIGHT_THRESHOLD_M - 0.01)
        for _ in range(100)
    ] + [
        (2.8, 0.0, HEIGHT_THRESHOLD_M + 0.01)
        for _ in range(POINT_THRESHOLD)
    ]

    result = build_lidar_visualization(
        points,
        SLOTS,
        sensor_status="ONLINE",
    )

    assert result["height_threshold_m"] == HEIGHT_THRESHOLD_M
    assert result["point_threshold"] == POINT_THRESHOLD
    assert result["point_total"] == 100 + POINT_THRESHOLD
    assert result["valid_point_count"] == POINT_THRESHOLD
    assert result["slot_point_count"] == POINT_THRESHOLD
    assert result["occupied_count"] == 1
    assert result["slots"][0]["status"] == "OCCUPIED"
    assert result["slots"][0]["point_count"] == POINT_THRESHOLD


def test_browser_payload_is_downsampled_but_counts_use_all_points():
    points = [(2.8, 0.0, 1.0) for _ in range(MAX_DISPLAY_POINTS * 4)]

    result = build_lidar_visualization(
        points,
        SLOTS,
        sensor_status="ONLINE",
    )

    assert result["point_total"] == MAX_DISPLAY_POINTS * 4
    assert result["display_point_count"] <= MAX_DISPLAY_POINTS
    assert result["slots"][0]["point_count"] == MAX_DISPLAY_POINTS * 4


def test_offline_sensor_is_not_misreported_as_three_empty_slots():
    result = build_lidar_visualization(
        [],
        SLOTS,
        sensor_status="OFFLINE",
    )

    assert result["occupied_count"] == 0
    assert all(slot["status"] == "UNAVAILABLE" for slot in result["slots"])


def test_lidar_and_control_slot_mismatch_is_reported():
    control_slots = [
        {"id": "A1", "x": 2.8, "y": 0.0, "status": "EMPTY"},
    ]
    points = [
        (2.8, 0.0, HEIGHT_THRESHOLD_M + 0.1)
        for _ in range(POINT_THRESHOLD)
    ]

    result = build_lidar_visualization(
        points,
        control_slots,
        sensor_status="ONLINE",
        frame_id="map",
    )

    assert result["coordinate_status"] == "OK"
    assert result["mismatch_count"] == 1
    assert result["slots"][0]["status"] == "OCCUPIED"
    assert result["slots"][0]["control_status"] == "EMPTY"
    assert result["slots"][0]["status_match"] is False


def test_mock_datasource_provides_three_slot_visual_demo():
    source = MockDataSource(StateStore())

    result = source.get_lidar_visualization()

    assert result["sensor_status"] == "MOCK"
    assert result["source"] == "MOCK_SAMPLE"
    assert result["total_slots"] == 3
    assert result["occupied_count"] == 1
    assert result["display_point_count"] <= MAX_DISPLAY_POINTS
