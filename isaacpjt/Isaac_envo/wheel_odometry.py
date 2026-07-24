#!/usr/bin/env python3
"""휠 엔코더 오도메트리 — 로봇 로컬 twist 를 월드 (x, z, yaw) 로 적분한다.

규약: yaw=0 은 월드 +Z 를 향한다(러너의 odom yaw 계산과 동일).
따라서 로봇 전진(+vx)은 월드 (sin yaw, cos yaw) 방향,
로봇 좌측(+vy)은 그를 CCW 90도 돌린 (-cos yaw, sin yaw) 방향이다.

순수 파이썬이라 Isaac 없이 테스트된다.
"""
import math


def _wrap(a):
    """각을 (-pi, pi] 로 접는다."""
    return math.atan2(math.sin(a), math.cos(a))


class WheelOdometry:
    """로봇 로컬 twist 적분기. GT 를 쓰지 않는다."""

    def __init__(self, x=0.0, z=0.0, yaw=0.0):
        self.x = float(x)
        self.z = float(z)
        self.yaw = _wrap(float(yaw))

    def update(self, vx, vy, wz, dt):
        """dt 동안 로컬 twist (vx 전진[m/s], vy 좌측[m/s], wz CCW[rad/s]) 적분."""
        dt = float(dt)
        if dt <= 0.0:
            return
        # 구간 중앙 yaw 로 적분해 1차 오차를 줄인다.
        yaw_mid = self.yaw + 0.5 * float(wz) * dt
        s, c = math.sin(yaw_mid), math.cos(yaw_mid)
        fwd = float(vx) * dt
        left = float(vy) * dt
        self.x += fwd * s - left * c
        self.z += fwd * c + left * s
        self.yaw = _wrap(self.yaw + float(wz) * dt)
