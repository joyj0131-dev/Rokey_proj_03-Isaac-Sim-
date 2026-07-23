"""편대 모션 프리미티브. dock_lift_handoff_mission.py에서 순수 기하부를 추출."""
import math

# dock_lift_handoff_mission.py 와 동일 값 (실측 대조 완료).
K_LIN, MAX_LIN = 0.8, 1.08   # 직선 이동 속도 상한. 사용자 요청 1.2배(0.9→1.08). 원래 0.6.
K_STRAFE = 0.8
K_YAW, MAX_YAW = 0.5, 0.36   # 회전 상한. 1.2배(0.30→0.36). 원래 0.15.
INGRESS_SPEED = 0.68         # 차 밑 진입 속도. 1.2배(0.40→0.48). 정밀도는 tol로 유지.
CARRY_SPEED = 0.30
CONTROL_HZ = 20.0
POS_TOL = 0.10
YAW_TOL = math.radians(4.0)


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def body_twist_from_world_error(ex, ez, yaw):
    """world 오차(ex,ez) → body (fwd=vx, left=vy). odom 규약 역행렬."""
    c, s = math.cos(yaw), math.sin(yaw)
    fwd = ex * c - ez * s
    left = -(ex * s + ez * c)
    return (fwd, left)


def formation_heading(rear_pose, front_pose):
    """두 로봇의 front->rear 축 heading을 odom yaw와 같은 규약으로 반환한다."""
    dx = rear_pose[0] - front_pose[0]
    dz = rear_pose[1] - front_pose[1]
    return math.atan2(-dz, dx)


def rigid_body_world_velocity(center_vx, center_vz, omega, rx, rz):
    """Y-up XZ 평면 강체 속도 ``V_center + omega x r``를 반환한다."""
    return (center_vx + omega * rz, center_vz - omega * rx)


def heading_hold_omega(reference, current, kp, max_omega, deadband=0.0):
    """heading P제어. 작은 오차는 0으로 만들고 출력은 대칭 포화한다."""
    error = wrap(reference - current)
    if abs(error) <= deadband:
        return 0.0
    return clamp(kp * error, max_omega)


def clamp(v, m):
    return max(-m, min(m, v))
