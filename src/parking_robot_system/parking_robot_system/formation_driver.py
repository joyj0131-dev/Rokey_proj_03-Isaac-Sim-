"""편대 모션 프리미티브. dock_lift_handoff_mission.py에서 순수 기하부를 추출."""
import math

# dock_lift_handoff_mission.py 와 동일 값 (실측 대조 완료).
K_LIN, MAX_LIN = 0.8, 1.08   # 직선 이동 속도 상한. 사용자 요청 1.2배(0.9→1.08). 원래 0.6.
K_STRAFE = 0.8
K_YAW, MAX_YAW = 0.5, 0.36   # 회전 상한. 1.2배(0.30→0.36). 원래 0.15.
INGRESS_SPEED = 0.48         # 차 밑 진입 속도. 1.2배(0.40→0.48). 정밀도는 tol로 유지.
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


def clamp(v, m):
    return max(-m, min(m, v))
