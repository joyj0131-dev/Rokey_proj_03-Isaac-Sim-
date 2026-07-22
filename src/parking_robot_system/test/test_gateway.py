from parking_robot_system.user_request_gateway import normalize_slot_id


def test_normalize():
    assert normalize_slot_id(" a2 ") == "A2"
    assert normalize_slot_id("b8") == "B8"
    assert normalize_slot_id("A2") == "A2"
