"""마커 fix + 휠 twist 데드레커닝 융합 (순수 파이썬, ros_map 프레임).

M3의 융합 규약을 따른다: 마커가 보이면 게인 0.9로 절대 보정, 안 보이는 구간은
로봇 로컬 twist 를 heading 프레임으로 회전해 적분한다.
"""
import math


def _wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


class PoseEstimator:
    def __init__(self, x=0.0, y=0.0, yaw=0.0):
        self.x, self.y, self.yaw = float(x), float(y), float(yaw)

    @property
    def pose(self):
        return self.x, self.y, self.yaw

    def predict(self, vx, vy, wz, dt):
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        self.x += (vx * c - vy * s) * dt
        self.y += (vx * s + vy * c) * dt
        self.yaw = _wrap(self.yaw + wz * dt)

    def correct(self, x, y, yaw, gain=0.9):
        self.x += gain * (x - self.x)
        self.y += gain * (y - self.y)
        self.yaw = _wrap(self.yaw + gain * _wrap(yaw - self.yaw))
