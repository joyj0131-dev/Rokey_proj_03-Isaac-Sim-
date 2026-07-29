"""미션 안무의 Isaac 비의존 순수 로직(제어법·상수). 단위테스트 대상."""
import math

def body_twist_toward(cur, tgt, *, pos_gain=0.8, yaw_gain=1.2,
                      max_lin=0.25, max_ang=0.6, pos_tol=0.03, yaw_tol=0.5,
                      yaw_min_cmd=0.0):
    """월드 현재/목표 자세 → body frame twist(vx,vy,wz)+done.

    자세 규약(wheel_odometry 와 동일): yaw=0→+Z. heading 단위벡터
    fwd=(sin yaw, cos yaw), 좌(+90°) left=(cos yaw, -sin yaw). (x,z 평면)

    ``yaw_min_cmd``(>0): 메카넘 회전 데드밴드 보정. 요 오차가 아직 tol 밖인데
    비례항 wz 가 이 값보다 작으면(데드밴드~0.012rad/s 아래라 바퀴가 안 돎) wz 를
    이 최소 회전속도로 깔아준다 — 마지막 <1° 를 인칭해서 좁혀 스톨을 막는다.
    0(기본)이면 보정 없음(하위호환·단위테스트).
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
    if yaw_min_cmd > 0.0 and abs(dyaw) > yaw_tol and abs(wz) < yaw_min_cmd:
        wz = math.copysign(yaw_min_cmd, dyaw)   # 데드밴드 밑이면 최소 회전속도로
    done = (math.hypot(dx, dz) <= pos_tol) and (abs(dyaw) <= yaw_tol)
    return vx, vy, wz, done


if __name__ == '__main__':
    # 데드밴드 보정 self-check: 목표 근처(요오차 0.6°>tol 0.5°)에서 비례 wz 는
    # ~0.013rad/s 로 데드밴드급이라 로봇이 못 돈다. yaw_min_cmd=0.05 를 주면 부호를
    # 지키며 그 최소속도로 깔려야 한다. 끄면(0) 손 안 댄다.
    cur, tgt = (0.0, 0.0, 0.0), (0.0, 0.0, 0.6)
    _, _, wz_off, done_off = body_twist_toward(cur, tgt, yaw_min_cmd=0.0)
    _, _, wz_on, _ = body_twist_toward(cur, tgt, yaw_min_cmd=0.05)
    _, _, wz_neg, _ = body_twist_toward((0.0, 0.0, 0.6), (0.0, 0.0, 0.0), yaw_min_cmd=0.05)
    assert abs(wz_off) < 0.05, f'보정 꺼짐인데 wz 가 큼: {wz_off}'
    assert not done_off, '0.6°>0.5°tol 인데 done'
    assert abs(wz_on - 0.05) < 1e-9, f'양의 요오차 최소속도 아님: {wz_on}'
    assert abs(wz_neg + 0.05) < 1e-9, f'음의 요오차 부호/크기 틀림: {wz_neg}'
    # tol 안(0.4°<0.5°)이면 done — 보정 발동 안 함
    _, _, _, done_in = body_twist_toward((0, 0, 0), (0, 0, 0.4), yaw_min_cmd=0.05)
    assert done_in, 'tol 안인데 done 아님'
    print('body_twist_toward 데드밴드 보정 self-check 통과')
