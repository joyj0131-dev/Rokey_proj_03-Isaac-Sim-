from parking_robot_system.frame_transform import (
    usd_to_map, map_to_usd, usd_yaw_to_map_deg, map_to_usd_yaw_deg)


def test_position_reflection():
    assert usd_to_map(-8.5, 7.8) == (-8.5, -7.8)     # A2
    assert map_to_usd(-8.5, -7.8) == (-8.5, 7.8)


def test_position_roundtrip():
    for x, z in [(-8.5, 7.8), (1.7, -7.8), (0.0, 0.0)]:
        assert map_to_usd(*usd_to_map(x, z)) == (x, z)


def test_yaw_reflection_and_slots():
    # y=-z 반사 → 회전 부호 반전. 180/0 은 불변.
    assert usd_yaw_to_map_deg(180.0) % 360 == 180.0
    assert usd_yaw_to_map_deg(0.0) % 360 == 0.0
    assert usd_yaw_to_map_deg(90.0) % 360 == 270.0
    assert map_to_usd_yaw_deg(usd_yaw_to_map_deg(37.0)) % 360 == 37.0 % 360
