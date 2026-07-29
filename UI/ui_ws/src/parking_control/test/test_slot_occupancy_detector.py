from pathlib import Path

import numpy as np
from geometry_msgs.msg import Transform
from std_msgs.msg import Header

from parking_control.core.graph import ParkingMap
from parking_control.core.slot_occupancy_detector import (
    STATUS_EMPTY,
    STATUS_OCCUPIED,
    STATUS_UNCERTAIN,
    STATUS_WAITING,
    SlotDecisionStabilizer,
    detect_with_masks,
)
from parking_control.safety_monitor_node import SafetyMonitorNode
from parking_robot_interfaces.msg import SlotOccupancyArray


MAP_YAML = Path(__file__).resolve().parent.parent / "config" / "parking_map.yaml"


def _points_in_slot(parking_map, slot_id, count, z=0.8):
    cx, cy = parking_map.node_pos(slot_id)
    offsets = np.linspace(-0.4, 0.4, count)
    return np.column_stack([
        cx + offsets,
        cy + offsets * 0.5,
        np.full(count, z),
    ])


def test_filter_masks_and_slot_counts_share_the_same_points():
    parking_map = ParkingMap.load(MAP_YAML)
    points = np.vstack([
        _points_in_slot(parking_map, "A2", 31),
        _points_in_slot(parking_map, "A1", 12, z=0.05),
        np.array([[-10.0, 8.0, 1.0]]),
    ])

    results, height_mask, slot_mask = detect_with_masks(points, parking_map)

    assert int(height_mask.sum()) == 32
    assert int(slot_mask.sum()) == 31
    assert results["A2"]["point_count"] == 31
    assert results["A2"]["occupied"] is True
    assert results["A1"]["point_count"] == 0


def test_three_equal_frames_are_required_before_confirming_occupancy():
    stabilizer = SlotDecisionStabilizer(["A1"], frames=3)
    frame = {"A1": {"occupied": True, "point_count": 31}}

    assert stabilizer.update(frame)["A1"]["status"] == STATUS_WAITING
    assert stabilizer.update(frame)["A1"]["status"] == STATUS_WAITING
    assert stabilizer.update(frame)["A1"]["status"] == STATUS_OCCUPIED


def test_threshold_flapping_becomes_uncertain_until_three_frames_agree():
    stabilizer = SlotDecisionStabilizer(["A1"], frames=3)

    stabilizer.update({"A1": {"occupied": False, "point_count": 29}})
    stabilizer.update({"A1": {"occupied": True, "point_count": 30}})
    result = stabilizer.update(
        {"A1": {"occupied": False, "point_count": 29}}
    )
    assert result["A1"]["status"] == STATUS_UNCERTAIN

    stabilizer.update({"A1": {"occupied": False, "point_count": 28}})
    result = stabilizer.update(
        {"A1": {"occupied": False, "point_count": 27}}
    )
    assert result["A1"]["status"] == STATUS_EMPTY


def test_a1_and_a3_can_be_occupied_without_marking_a2():
    parking_map = ParkingMap.load(MAP_YAML)
    points = np.vstack([
        _points_in_slot(parking_map, "A1", 35),
        _points_in_slot(parking_map, "A3", 40),
    ])

    results, _height_mask, _slot_mask = detect_with_masks(points, parking_map)

    assert results["A1"]["occupied"] is True
    assert results["A2"]["occupied"] is False
    assert results["A3"]["occupied"] is True


def test_all_slots_are_empty_when_only_floor_points_are_received():
    parking_map = ParkingMap.load(MAP_YAML)
    points = np.vstack([
        _points_in_slot(parking_map, slot_id, 60, z=0.05)
        for slot_id in ("A1", "A2", "A3")
    ])

    results, height_mask, slot_mask = detect_with_masks(points, parking_map)

    assert int(height_mask.sum()) == 0
    assert int(slot_mask.sum()) == 0
    assert all(not result["occupied"] for result in results.values())


def test_tf_translation_is_applied_to_cloud_points():
    transform = Transform()
    transform.translation.x = 2.0
    transform.translation.y = -1.0
    transform.translation.z = 0.5
    transform.rotation.w = 1.0
    points = np.array([[1.0, 2.0, 3.0]])

    transformed = SafetyMonitorNode._apply_transform(points, transform)

    assert transformed.tolist() == [[3.0, 1.0, 3.5]]


def test_rviz_waiting_markers_use_ascii_single_line_labels():
    parking_map = ParkingMap.load(MAP_YAML)
    node = object.__new__(SafetyMonitorNode)
    node._map = parking_map
    node._zone_boxes = {}
    node._point_threshold = 30
    node._height_threshold = 0.15
    node._stabilization_frames = 3
    node._sensor_id = "L1"
    node._sensor_position = (0.5, 0.0, 5.12)
    waiting = node._waiting_results()
    occupancy = SlotOccupancyArray()
    occupancy.measurement_status = SlotOccupancyArray.MEASUREMENT_WAITING

    markers = node._build_markers(Header(frame_id="map"), {}, waiting, occupancy)
    text_markers = [
        marker for marker in markers.markers
        if marker.text
    ]
    text_values = [marker.text for marker in text_markers]

    assert all("\n" not in value for value in text_values)
    assert all(value.isascii() for value in text_values)
    assert {"A1", "A2", "A3"}.issubset(text_values)
    assert text_values.count("--") == 6
    assert "L1/DISCONNECTED" in text_values
    assert "NO_SENSOR_DATA|LAST=NONE" in text_values
    assert "TOPIC=/parking/lidar/points_world" in text_values
    assert "TOTAL=3|OCCUPIED=0|EMPTY=0|PENDING=3|UNCERTAIN=0" in text_values


def test_rviz_slot_text_rows_stay_inside_each_slot():
    parking_map = ParkingMap.load(MAP_YAML)
    node = object.__new__(SafetyMonitorNode)
    node._map = parking_map
    node._zone_boxes = {}
    node._point_threshold = 30
    node._height_threshold = 0.15
    node._stabilization_frames = 3
    node._sensor_id = "L1"
    node._sensor_position = (0.5, 0.0, 5.12)
    waiting = node._waiting_results()

    markers = node._build_markers(Header(frame_id="map"), {}, waiting)
    names = {
        marker.text: marker
        for marker in markers.markers
        if marker.ns == "slot_name"
    }
    counts = [
        marker for marker in markers.markers
        if marker.ns == "slot_count"
    ]
    half_length = parking_map.meta["params"]["space_length"] / 2

    for slot_id in ("A1", "A2", "A3"):
        center_x, center_y = parking_map.node_pos(slot_id)
        assert names[slot_id].pose.position.x == center_x
        assert abs(names[slot_id].pose.position.y - center_y) < half_length
    assert len(counts) == 3
    assert all(
        abs(marker.pose.position.y) < half_length for marker in counts
    )


def test_rviz_online_status_uses_measured_counts_and_rate():
    parking_map = ParkingMap.load(MAP_YAML)
    node = object.__new__(SafetyMonitorNode)
    node._map = parking_map
    node._zone_boxes = {}
    node._point_threshold = 30
    node._height_threshold = 0.15
    node._stabilization_frames = 3
    node._sensor_id = "L1"
    node._sensor_position = (0.5, 0.0, 5.12)
    node._observed_hz = 10.2
    results = node._waiting_results()
    occupancy = SlotOccupancyArray()
    occupancy.measurement_status = SlotOccupancyArray.MEASUREMENT_OK
    occupancy.received_point_count = 12480
    occupancy.filtered_point_count = 2814
    occupancy.slot_point_count = 924

    markers = node._build_markers(Header(frame_id="map"), {}, results, occupancy)
    text_values = [
        marker.text for marker in markers.markers if marker.text
    ]

    assert "L1/ONLINE|10.2Hz|TF/OK" in text_values
    assert "RAW=12480|FILTER=2814|IN_SLOT=924" in text_values


def test_rviz_offline_sensor_marker_is_red():
    parking_map = ParkingMap.load(MAP_YAML)
    node = object.__new__(SafetyMonitorNode)
    node._map = parking_map
    node._zone_boxes = {}
    node._point_threshold = 30
    node._height_threshold = 0.15
    node._stabilization_frames = 3
    node._sensor_id = "L1"
    node._sensor_position = (0.5, 0.0, 5.12)
    occupancy = SlotOccupancyArray()
    occupancy.measurement_status = SlotOccupancyArray.MEASUREMENT_WAITING

    markers = node._build_markers(
        Header(frame_id="map"), {}, node._waiting_results(), occupancy
    )
    sensor_shapes = [
        marker for marker in markers.markers
        if marker.ns == "lidar_sensor" and not marker.text
    ]

    assert len(sensor_shapes) == 2
    assert all(marker.color.r == 1.0 for marker in sensor_shapes)
    assert all(marker.color.g == 0.22 for marker in sensor_shapes)
    assert all(marker.color.b == 0.16 for marker in sensor_shapes)


def test_rviz_monitoring_cards_are_published():
    parking_map = ParkingMap.load(MAP_YAML)
    node = object.__new__(SafetyMonitorNode)
    node._map = parking_map
    node._zone_boxes = {}
    node._point_threshold = 30
    node._height_threshold = 0.15
    node._stabilization_frames = 3
    node._sensor_id = "L1"
    node._sensor_position = (0.5, 0.0, 5.12)

    markers = node._build_markers(
        Header(frame_id="map"), {}, node._waiting_results()
    )
    namespaces = {marker.ns for marker in markers.markers}

    assert "status_panel" in namespaces
    assert "info_panel" in namespaces
    assert "monitor_title" in namespaces
    assert "occupancy_summary" in namespaces
