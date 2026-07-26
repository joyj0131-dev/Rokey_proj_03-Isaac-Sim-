"""장애물 감지 위치가 ROS2 → 관제 API 모델까지 보존되는지 검증한다."""

import pytest

from parking_robot_interfaces.msg import ObstacleAlert

from core.state_store import StateStore
from sources.mock_source import MockDataSource
from sources.ros2_source import Ros2DataSource


def test_ros2_obstacle_location_and_zone_are_preserved():
    store = StateStore()
    source = Ros2DataSource(store)
    message = ObstacleAlert()
    message.obstacle_detected = True
    message.description = "통로 막힘: ZIN02, ZIN03"
    message.location.x = -10.525
    message.location.y = -7.075

    source._on_obstacle_alert(message)

    alert = store.snapshot()["alerts"][0]
    assert alert.sensor_id == "L1"
    assert alert.zone_id == "ZIN02, ZIN03"
    assert alert.location_x == pytest.approx(-10.525)
    assert alert.location_y == pytest.approx(-7.075)


def test_legacy_zero_location_is_not_exposed_as_real_detection_position():
    store = StateStore()
    source = Ros2DataSource(store)
    message = ObstacleAlert()
    message.obstacle_detected = True
    message.description = "장애물이 감지되었습니다."

    source._on_obstacle_alert(message)

    alert = store.snapshot()["alerts"][0]
    assert alert.location_x is None
    assert alert.location_y is None


def test_repeated_ros2_frames_do_not_duplicate_same_obstacle_alert():
    store = StateStore()
    source = Ros2DataSource(store)
    message = ObstacleAlert()
    message.obstacle_detected = True
    message.description = "통로 막힘: ZOUT02"
    message.location.x = -10.0
    message.location.y = 7.075

    source._on_obstacle_alert(message)
    message.location.x = -9.8
    source._on_obstacle_alert(message)

    alerts = store.snapshot()["alerts"]
    assert len(alerts) == 1
    assert alerts[0].location_x == pytest.approx(-9.8)


def test_changed_ros2_obstacle_scope_replaces_previous_frame():
    store = StateStore()
    source = Ros2DataSource(store)
    message = ObstacleAlert()
    message.obstacle_detected = True
    message.description = "통로 막힘: ZIN03"
    message.location.y = -6.875
    source._on_obstacle_alert(message)

    message.description = "통로 막힘: ZOUT01"
    message.location.y = 6.875
    source._on_obstacle_alert(message)

    active_alerts = store.snapshot()["alerts"]
    assert len(active_alerts) == 1
    assert active_alerts[0].zone_id == "ZOUT01"
    assert len(store.alerts) == 2
    assert store.alerts[0].active is False


def test_cross_team_ros2_obstacle_keeps_all_zone_ids_for_fail_safe_scope():
    store = StateStore()
    source = Ros2DataSource(store)
    message = ObstacleAlert()
    message.obstacle_detected = True
    message.description = "통로 막힘: ZIN03, ZOUT01"

    source._on_obstacle_alert(message)

    assert store.snapshot()["alerts"][0].zone_id == "ZIN03, ZOUT01"


def test_ros2_clear_frame_resolves_active_obstacle_alert():
    store = StateStore()
    source = Ros2DataSource(store)
    detected = ObstacleAlert()
    detected.obstacle_detected = True
    detected.description = "통로 막힘: ZIN03"
    detected.location.x = -5.5
    detected.location.y = -7.075
    source._on_obstacle_alert(detected)

    cleared = ObstacleAlert()
    cleared.obstacle_detected = False
    source._on_obstacle_alert(cleared)

    assert store.snapshot()["alerts"] == []
    assert store.alerts[0].active is False


def test_mock_obstacle_has_clickable_map_location():
    source = MockDataSource(StateStore())

    alert = source.trigger_obstacle()

    assert alert.sensor_id == "L1"
    assert alert.zone_id
    assert alert.location_x is not None
    assert alert.location_y is not None
