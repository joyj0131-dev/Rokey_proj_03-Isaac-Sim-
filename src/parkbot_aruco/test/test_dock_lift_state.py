"""순차 도킹 상태기계 (ROS/Isaac 불필요)."""
from parkbot_aruco.dock_lift_state import DockLiftPlan


def _plan():
    # rear 축 z=-1.36, front 축 z=+1.36 (Coupe 축거 2.715 근사), center_x=0
    return DockLiftPlan(rear_target_z=-1.36, front_target_z=1.36,
                        center_x=0.0, carry_distance=1.0)


def test_ingress_rear_drives_rear_only():
    p = _plan()
    cmd = p.ingress_cmd("ingress_rear", rear_z=-3.11, front_z=3.11)
    assert cmd["robot_rear"] > 0     # rear 전진(차량 쪽)
    assert cmd["robot_front"] == 0   # front 는 순차 — 아직 대기


def test_rear_arrival_advances_to_front():
    p = _plan()
    nxt = p.next_phase("ingress_rear", rear_z=-1.35, front_z=3.11,
                       car_lift_m=0.0, carried_z=0.0)
    assert nxt == "ingress_front"


def test_both_arrived_advances_to_grip():
    p = _plan()
    nxt = p.next_phase("ingress_front", rear_z=-1.35, front_z=1.35,
                       car_lift_m=0.0, carried_z=0.0)
    assert nxt == "grip"


def test_grip_lift_advances_to_carry():
    p = _plan()
    nxt = p.next_phase("grip", rear_z=-1.36, front_z=1.36,
                       car_lift_m=0.03, carried_z=0.0)   # 실제 상승
    assert nxt == "carry"


def test_grip_no_lift_fails():
    p = _plan()
    nxt = p.next_phase("grip", rear_z=-1.36, front_z=1.36,
                       car_lift_m=0.001, carried_z=0.0)   # 안 들림
    assert nxt == "fail"


def test_carry_distance_reached_done():
    p = _plan()
    nxt = p.next_phase("carry", rear_z=-1.36, front_z=1.36,
                       car_lift_m=0.03, carried_z=1.05)   # 목표 1.0 초과
    assert nxt == "done"
