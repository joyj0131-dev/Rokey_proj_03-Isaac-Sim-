"""미션 안무의 Isaac 비의존 순수 로직(제어법·상수). 단위테스트 대상."""
import math

def body_twist_toward(cur, tgt, *, pos_gain=0.8, yaw_gain=1.2,
                      max_lin=0.25, max_ang=0.6, pos_tol=0.03, yaw_tol=0.5):
    """월드 현재/목표 자세 → body frame twist(vx,vy,wz)+done.

    자세 규약(wheel_odometry 와 동일): yaw=0→+Z. heading 단위벡터
    fwd=(sin yaw, cos yaw), 좌(+90°) left=(cos yaw, -sin yaw). (x,z 평면)
    """
    cx, cz, cyaw = cur
    tx, tz, tyaw = tgt
    dx, dz = tx - cx, tz - cz
    yr = math.radians(cyaw)
    fwd_x, fwd_z = math.sin(yr), math.cos(yr)
    left_x, left_z = math.cos(yr), -math.sin(yr)
    fwd = dx * fwd_x + dz * fwd_z            # body +x(전진)
    left = dx * left_x + dz * left_z         # body +y(좌)
    dyaw = ((tyaw - cyaw + 180.0) % 360.0) - 180.0
    clamp = lambda v, m: max(-m, min(m, v))
    vx = clamp(pos_gain * fwd, max_lin)
    vy = clamp(pos_gain * left, max_lin)
    wz = clamp(yaw_gain * math.radians(dyaw), max_ang)
    done = (math.hypot(dx, dz) <= pos_tol) and (abs(dyaw) <= yaw_tol)
    return vx, vy, wz, done
