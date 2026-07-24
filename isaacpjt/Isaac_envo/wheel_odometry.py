#!/usr/bin/env python3
"""휠 엔코더 오도메트리 — 로봇 로컬 twist 를 월드 (x, z, yaw) 로 적분한다.

규약(marker_localizer.PoseFilter.predict_body 와 동일해야 한다):
  전방(+X_body) -> 월드 +Z,  좌(+Y_body) -> 월드 +X   (yaw=0 기준)
  nav yaw ψ 에서  전방 = (sinψ,  cosψ),  좌 = (cosψ, -sinψ)   [(x,z) 평면]

좌측 항의 부호를 뒤집으면 모든 메카넘 스트레이핑이 좌우 반전되어 적분된다.

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
        # 전방=(sinψ, cosψ), 좌=(cosψ, -sinψ). predict_body 와 동일.
        self.x += fwd * s + left * c
        self.z += fwd * c - left * s
        self.yaw = _wrap(self.yaw + float(wz) * dt)
