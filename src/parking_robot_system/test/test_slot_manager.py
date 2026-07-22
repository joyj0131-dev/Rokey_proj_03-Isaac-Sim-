import json
from parking_robot_system.parking_slot_manager import SlotCache


def test_empty_cache_not_ready():
    c = SlotCache()
    assert c.ready is False
    assert c.query("A2") is None


def test_update_and_query():
    c = SlotCache()
    c.update_from_json(json.dumps([
        {"slot_id": "A2", "occupied": False, "is_accessible": True, "x": -8.5, "y": -7.8, "yaw_deg": 180.0},
        {"slot_id": "A3", "occupied": True, "is_accessible": False, "x": -5.1, "y": -7.8, "yaw_deg": 180.0},
    ]))
    assert c.ready is True
    assert c.query("A2")["occupied"] is False
    assert c.query("A3")["occupied"] is True
    assert c.query("B9") is None


def test_malformed_payload_does_not_crash_and_keeps_prior():
    c = SlotCache()
    c.update_from_json('[{"slot_id":"A2","occupied":false,"is_accessible":true,"x":-8.5,"y":-7.8,"yaw_deg":180.0}]')
    # non-list top level and non-dict/element-without-slot_id must be ignored, cache unchanged
    for bad in ['{"slot_id":"A2"}', '[1,2,3]', 'null', '[{"no_slot":1}]']:
        c.update_from_json(bad)
        assert c.query("A2") is not None   # prior good data preserved
