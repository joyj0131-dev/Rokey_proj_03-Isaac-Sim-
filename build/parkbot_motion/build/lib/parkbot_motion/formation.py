#!/usr/bin/env python3
"""Phase D 운반용 2로봇 가상중심(virtual-center) 강체 kinematics.

두 로봇이 트럭을 들고 하나의 강체처럼 움직인다. 제어·측위를 "두 로봇 사이의
가상중심 C" 기준으로 하고, 각 로봇은 C 에서 고정 오프셋(리프트 시 결정된 편성)에
있다고 본다. 트럭을 물리적으로 함께 들고 있어 실제로도 강체다.

프레임 규약(이 프로젝트 body twist 와 동일): x=전진, y=좌, yaw=CCW+.
평면 pose = (x, y, yaw[rad]), twist = (vx, vy, wz). 오프셋 (ox, oy) 는
**포메이션 프레임**(=C 프레임)에서 로봇 위치. 축 매핑(USD x/z 등)은 호출자 몫 —
이 모듈은 프레임 무관하게 순수 평면 기하만 다룬다.

핵심 두 함수:
  robot_twist_from_center: C 의 목표 twist -> 각 로봇 twist (제어; 강체라 회전은
    공유, 병진은 레버암만큼 보정).
  center_pose_from_robots: 각 로봇 world pose(바닥마커 측위) -> C 의 world pose
    (측위; lead 후방캠/follow 전방캠 추정 2개를 융합).
"""
import math


def robot_twist_from_center(center_twist, offset):
    """가상중심 twist -> 오프셋 위치 로봇의 twist.

    강체 속도장: v_P = v_C + ω × r. 2D 에서 ω×(ox,oy) = (-wz*oy, wz*ox).
    각 로봇 각속도 = 중심 각속도(강체 = 같은 회전). 메카넘이라 (vx,vy,wz) 를
    그대로 실행 가능.
    """
    vx, vy, wz = center_twist
    ox, oy = offset
    return (vx - wz * oy, vy + wz * ox, wz)


def _rot(x, y, yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    return (c * x - s * y, s * x + c * y)


def center_pose_from_robots(estimates):
    """로봇 world pose 추정들 -> 가상중심 world pose(융합).

    estimates: [(pose, offset), ...]  pose=(x,y,yaw) world, offset=(ox,oy) C프레임.
    강체라 모든 로봇 yaw == C yaw. 각 추정에서 C = robot_world - R(yaw)*offset.
    여러 추정은 평균(yaw 는 원형평균)으로 융합해 단일 마커 노이즈를 눌러준다.
    반환: (x, y, yaw).
    """
    if not estimates:
        raise ValueError("estimates 비어있음")
    xs, ys, sin_s, cos_s = 0.0, 0.0, 0.0, 0.0
    for (rx, ry, ryaw), (ox, oy) in estimates:
        dx, dy = _rot(ox, oy, ryaw)
        xs += rx - dx
        ys += ry - dy
        sin_s += math.sin(ryaw)
        cos_s += math.cos(ryaw)
    n = len(estimates)
    return (xs / n, ys / n, math.atan2(sin_s, cos_s))


def _demo():
    # 1) 순수 병진(wz=0): 모든 로봇이 중심 twist 그대로.
    assert robot_twist_from_center((0.5, 0.1, 0.0), (0.85, 0.0)) == (0.5, 0.1, 0.0)

    # 2) 순수 회전(중심 제자리 회전, wz=+1): lead(+x)·follow(-x)가 좌우 반대로 미끄러짐.
    lead = robot_twist_from_center((0.0, 0.0, 1.0), (0.85, 0.0))
    follow = robot_twist_from_center((0.0, 0.0, 1.0), (-0.85, 0.0))
    assert abs(lead[1] - 0.85) < 1e-9 and abs(follow[1] + 0.85) < 1e-9
    assert lead[2] == follow[2] == 1.0            # 각속도 공유(강체)

    # 3) 측위 왕복: 알려진 중심에서 오프셋만큼 놓은 로봇 pose -> 중심 복원.
    cx, cy, cyaw = 3.0, -2.0, math.radians(30)
    offs = [(0.85, 0.0), (-0.85, 0.0)]
    est = []
    for ox, oy in offs:
        dx, dy = _rot(ox, oy, cyaw)
        est.append(((cx + dx, cy + dy, cyaw), (ox, oy)))
    rx, ry, ryaw = center_pose_from_robots(est)
    assert abs(rx - cx) < 1e-9 and abs(ry - cy) < 1e-9 and abs(ryaw - cyaw) < 1e-9

    # 4) 회전+병진 합성도 각속도는 공유.
    a = robot_twist_from_center((0.3, 0.0, 0.4), (0.85, 0.0))
    b = robot_twist_from_center((0.3, 0.0, 0.4), (-0.85, 0.0))
    assert a[2] == b[2] == 0.4
    print("formation _demo OK")


if __name__ == "__main__":
    _demo()
