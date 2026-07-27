from pathlib import Path

import numpy as np
from geometry_msgs.msg import Transform

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
