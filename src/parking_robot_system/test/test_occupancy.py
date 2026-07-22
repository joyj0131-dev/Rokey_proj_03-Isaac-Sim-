from parking_robot_system.occupancy import slot_occupied


def test_vehicle_at_center_occupies():
    assert slot_occupied("A2", [(-8.5, 7.8)]) is True


def test_empty_slot():
    assert slot_occupied("A2", [(5.1, 7.8)]) is False  # A6 위치 차량


def test_tolerance_box():
    # 슬롯 반폭 1.7, 반길이 3.3 안/밖 경계
    assert slot_occupied("A2", [(-8.5 + 1.6, 7.8 + 3.2)]) is True
    assert slot_occupied("A2", [(-8.5 + 1.8, 7.8)]) is False


def test_exclude_carried_vehicle():
    assert slot_occupied("A2", [(-8.5, 7.8)], exclude_xz=(-8.5, 7.8)) is False


def test_unknown_slot_is_false():
    assert slot_occupied("Z9", [(-8.5, 7.8)]) is False
