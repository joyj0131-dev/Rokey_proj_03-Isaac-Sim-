"""ROS2 촉발 도킹·리프트·운반 순수 상태기계 (표준 라이브러리만).

단계: ingress_rear -> ingress_front -> grip -> carry -> done / fail.
좌표: 차량 길이축 = world z. 로봇은 자기 target_z 를 향해 로컬 forward(vx>0) 직진.
rear 로봇은 -z 쪽에서 +z 로, front 로봇은 +z 쪽에서 -z 로 진입한다(둘 다 vx>0).
"""
POS_TOL_M = 0.05
INGRESS_SPEED = 0.30
CARRY_SPEED = 0.35
LIFT_MIN_M = 0.025


class DockLiftPlan:
    def __init__(self, rear_target_z, front_target_z, center_x, carry_distance):
        self.rear_tz = float(rear_target_z)
        self.front_tz = float(front_target_z)
        self.center_x = float(center_x)
        self.carry_distance = float(carry_distance)

    def rear_arrived(self, rear_z):
        return abs(rear_z - self.rear_tz) <= POS_TOL_M

    def front_arrived(self, front_z):
        return abs(front_z - self.front_tz) <= POS_TOL_M

    def ingress_cmd(self, phase, rear_z, front_z):
        cmd = {"robot_rear": 0.0, "robot_front": 0.0}
        if phase == "ingress_rear" and not self.rear_arrived(rear_z):
            cmd["robot_rear"] = INGRESS_SPEED       # 로컬 forward = 차량 쪽(+Z)
        elif phase == "ingress_front":
            if not self.rear_arrived(rear_z):
                cmd["robot_rear"] = INGRESS_SPEED   # 미세 보정 유지
            if not self.front_arrived(front_z):
                cmd["robot_front"] = INGRESS_SPEED  # 로컬 forward = 차량 쪽(-Z)
        return cmd

    def next_phase(self, phase, rear_z, front_z, car_lift_m, carried_z):
        if phase == "ingress_rear":
            return "ingress_front" if self.rear_arrived(rear_z) else "ingress_rear"
        if phase == "ingress_front":
            if self.rear_arrived(rear_z) and self.front_arrived(front_z):
                return "grip"
            return "ingress_front"
        if phase == "grip":
            return "carry" if car_lift_m >= LIFT_MIN_M else "fail"
        if phase == "carry":
            return "done" if carried_z >= self.carry_distance else "carry"
        return phase
