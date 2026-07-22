from parking_robot_system.slot_geometry import (
    parse_slot, slot_center_usd, slot_target_yaw_usd_deg, is_accessible)


def test_parse_valid():
    assert parse_slot("A2") == ("A", 2)
    assert parse_slot("B8") == ("B", 8)


def test_parse_invalid():
    for bad in ["A0", "A9", "C1", "A", "", "AA", "b3", "A2 "]:
        assert parse_slot(bad) is None


def test_center_matches_build_script():
    # A2: x=-17+2.5*3.4=-8.5, z(A)=+7.8
    assert slot_center_usd("A2") == (-8.5, 7.8)
    # B3: x=-17+3.5*3.4=-5.1, z(B)=-7.8
    assert slot_center_usd("B3") == (-5.1, -7.8)


def test_target_yaw():
    assert slot_target_yaw_usd_deg("A2") == 180.0
    assert slot_target_yaw_usd_deg("B3") == 0.0


def test_accessible():
    assert is_accessible("A1") and is_accessible("A2")
    assert not is_accessible("A3") and not is_accessible("B1")
