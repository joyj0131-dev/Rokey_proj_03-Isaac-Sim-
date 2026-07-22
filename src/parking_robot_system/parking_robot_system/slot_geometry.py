"""slot_id('A1'~'B8') → USD 슬롯 중심·목표 yaw. build_parking_environment.py 규약과 동일."""

HALF_W = 17.0
SPACE_WIDTH = 3.4
ROW_CENTER = 7.8
PARKING_INDICES = range(1, 9)
ACCESSIBLE = {"A1", "A2"}


def parse_slot(slot_id):
    if not isinstance(slot_id, str) or len(slot_id) < 2:
        return None
    row, num = slot_id[0], slot_id[1:]
    if row not in ("A", "B") or not num.isdigit():
        return None
    index = int(num)
    if index not in PARKING_INDICES:
        return None
    return (row, index)


def slot_center_usd(slot_id):
    parsed = parse_slot(slot_id)
    if parsed is None:
        return None
    row, index = parsed
    x = -HALF_W + (index + 0.5) * SPACE_WIDTH
    z = ROW_CENTER if row == "A" else -ROW_CENTER
    return (round(x, 3), round(z, 3))


def slot_target_yaw_usd_deg(slot_id):
    parsed = parse_slot(slot_id)
    if parsed is None:
        return None
    return 180.0 if parsed[0] == "A" else 0.0


def is_accessible(slot_id):
    return slot_id in ACCESSIBLE
