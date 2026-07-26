"""차량 실제 위치(USD XZ)로 슬롯 점유를 기하학적으로 판정."""

from parking_robot_system.slot_geometry import slot_center_usd, SPACE_WIDTH

HALF_LEN = 6.6 / 2.0
HALF_WID = SPACE_WIDTH / 2.0


def slot_occupied(slot_id, vehicle_positions_usd, *, exclude_xz=None):
    center = slot_center_usd(slot_id)
    if center is None:
        return False
    cx, cz = center
    for vx, vz in vehicle_positions_usd:
        if exclude_xz is not None and abs(vx - exclude_xz[0]) < 1e-6 and abs(vz - exclude_xz[1]) < 1e-6:
            continue
        if abs(vx - cx) <= HALF_WID and abs(vz - cz) <= HALF_LEN:
            return True
    return False
