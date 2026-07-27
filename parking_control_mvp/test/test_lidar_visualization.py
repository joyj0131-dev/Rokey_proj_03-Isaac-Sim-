"""LiDAR 상세 화면 데이터 계약 회귀 테스트."""

import time

from parking_robot_interfaces.msg import SlotOccupancy, SlotOccupancyArray

from core.lidar_visualization import (
    HEIGHT_THRESHOLD_M,
    MAX_DISPLAY_POINTS,
    POINT_THRESHOLD,
    build_lidar_visualization,
)
from core.state_store import StateStore
from sources.mock_source import MockDataSource
from sources.ros2_source import Ros2DataSource


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


def _occupancy_slot(slot_id, status, count, x):
    slot = SlotOccupancy()
    slot.slot_id = slot_id
    slot.status = status
    slot.point_count = count
    slot.point_threshold = POINT_THRESHOLD
    slot.height_threshold_m = HEIGHT_THRESHOLD_M
    slot.center.x = x
    slot.width = 3.4
    slot.length = 6.6
    return slot


def test_ros2_web_uses_authoritative_shared_occupancy_result():
    source = Ros2DataSource(StateStore())
    now = time.monotonic()
    control_slots = [
        {"id": "A1", "x": 2.8, "y": 0.0, "status": "EMPTY"},
        {"id": "A2", "x": 6.2, "y": 0.0, "status": "EMPTY"},
        {"id": "A3", "x": 9.6, "y": 0.0, "status": "EMPTY"},
    ]
    source._lidar_visualization = build_lidar_visualization(
        [(2.8, 0.0, 1.0)] * 100,
        control_slots,
        sensor_status="ONLINE",
    )
    source._lidar_visualization_updated_at = now
    source._sensor_received["L1"].append(now)

    message = SlotOccupancyArray()
    message.header.frame_id = "map"
    message.sensor_id = "L1"
    message.source_topic = "/parking/lidar/points_world"
    message.measurement_status = SlotOccupancyArray.MEASUREMENT_OK
    message.status_message = "정상 수신"
    message.received_point_count = 1500
    message.filtered_point_count = 950
    message.slot_point_count = 948
    message.stabilization_frames = 3
    message.slots = [
        _occupancy_slot("A1", SlotOccupancy.STATUS_EMPTY, 19, 2.8),
        _occupancy_slot("A2", SlotOccupancy.STATUS_OCCUPIED, 920, 6.2),
        _occupancy_slot("A3", SlotOccupancy.STATUS_EMPTY, 9, 9.6),
    ]
    source._on_slot_occupancy(message)

    result = source.get_lidar_visualization()
    slots = {slot["id"]: slot for slot in result["slots"]}

    assert result["source"] == "ROS2_SHARED_OCCUPANCY"
    assert result["point_total"] == 1500
    assert result["valid_point_count"] == 950
    assert result["occupied_count"] == 1
    assert slots["A1"]["status"] == "EMPTY"
    assert slots["A1"]["point_count"] == 19
    assert slots["A2"]["status"] == "OCCUPIED"
    assert slots["A2"]["point_count"] == 920
    assert slots["A2"]["status_match"] is False


def test_ros2_raw_cloud_without_shared_result_stays_in_waiting_state():
    source = Ros2DataSource(StateStore())
    now = time.monotonic()
    source._lidar_visualization = build_lidar_visualization(
        [(6.2, 0.0, 1.0)] * 100,
        SLOTS,
        sensor_status="ONLINE",
    )
    source._lidar_visualization_updated_at = now
    source._sensor_received["L1"].append(now)

    result = source.get_lidar_visualization()

    assert result["measurement_status"] == "RESULT_WAITING"
    assert result["occupied_count"] == 0
    assert all(slot["status"] == "WAITING" for slot in result["slots"])


def test_ros2_tf_failure_is_exposed_separately_from_topic_offline():
    source = Ros2DataSource(StateStore())
    now = time.monotonic()
    source._lidar_visualization = build_lidar_visualization(
        [(6.2, 0.0, 1.0)],
        SLOTS,
        sensor_status="ONLINE",
    )
    source._lidar_visualization_updated_at = now
    source._sensor_received["L1"].append(now)
    message = SlotOccupancyArray()
    message.header.frame_id = "map"
    message.sensor_id = "L1"
    message.source_topic = "/parking/lidar/points_world"
    message.measurement_status = SlotOccupancyArray.MEASUREMENT_TF_ERROR
    message.status_message = "map 좌표 변환 실패"
    message.stabilization_frames = 3
    message.slots = [
        _occupancy_slot("A1", SlotOccupancy.STATUS_WAITING, 0, 2.8),
        _occupancy_slot("A2", SlotOccupancy.STATUS_WAITING, 0, 6.2),
        _occupancy_slot("A3", SlotOccupancy.STATUS_WAITING, 0, 9.6),
    ]
    source._on_slot_occupancy(message)

    result = source.get_lidar_visualization()

    assert result["sensor_status"] == "ONLINE"
    assert result["measurement_status"] == "TF_ERROR"
    assert result["coordinate_status"] == "ERROR"
    assert all(slot["status"] == "WAITING" for slot in result["slots"])
