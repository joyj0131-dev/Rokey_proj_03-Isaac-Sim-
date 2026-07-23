"""차량을 붙든 로봇 2대의 "차량 중심 기준 제자리 피벗 회전" 제어. 순수 Python.

gap_hold_controller.py는 회전 중심을 "리더 로봇 자신"으로 두고 그 뒤를
따라가는 모델이라, 리더가 제자리에서 방향만 트는 상황(차량을 들고 90도
회전)에는 안 맞는다 — 그 식대로면 팔로워가 리더 위치를 중심으로 도는
원을 그리게 되는데, 실제로 돌아야 하는 중심은 "두 로봇 위치의 중점"
(≈ 차량 중심)이기 때문이다.

강체(rigid body) 운동학: 중심에서 반지름 벡터 r만큼 떨어진 점의 접선속도는
ω × r 이다. 두 로봇은 이 중심을 사이에 두고 반대편에 있으므로(r이 서로
반대 방향), 이 식 하나로 계산하면 선속도는 자동으로 반대 방향이 되고,
자기 몸이 도는 속도(각속도)는 두 로봇이 항상 동일하게 나온다 — 방향에
따라 if문으로 나눠 처리할 필요가 없다.

두 로봇 다 별도 통신 없이 이 컨트롤러를 하나씩 띄우기만 하면 된다: 입력
(자기 pose, 파트너 pose, 회전 시작 시점 두 pose)이 같으면 항상 같은 omega가
나오므로(결정적 계산), gap_hold_controller가 쓰는 partner-odom 구독 구조
그대로 재사용할 수 있다.

2026-07-21 실측 제약 (mecanum_drive.py 참고): 이 로봇은 제자리 yaw가
롤러 미끄러짐 지배적이라, 빠르게 돌릴수록 실제 회전량이 명령보다 덜
나온다. 그래서 "이 속도로 몇 초 돌리면 90도"라는 오픈루프 계산은 쓰지
않는다 — 매 tick 두 로봇의 실측 yaw로 "지금까지 실제로 돈 각도"를 계산해
남은 각도에 비례해서만 속도를 낸다(피드백), 그리고 그 속도도
gap_hold_controller의 holonomic 모드와 같은 안전 상한(기본 0.15 rad/s)으로
클램프한다.
"""

import math
from dataclasses import dataclass


@dataclass(frozen=True)
class Pose2D:
    x: float
    y: float
    yaw: float


@dataclass(frozen=True)
class PivotCommand:
    linear_x: float
    linear_y: float
    angular_z: float


def _wrap_angle(angle):
    """[-pi, pi] 범위로 정규화."""
    return (angle + math.pi) % (2 * math.pi) - math.pi


def _clamp(value, low, high):
    return max(low, min(high, value))


def formation_center(pose_a: Pose2D, pose_b: Pose2D):
    """두 로봇 위치의 중점 (≈ 차량 중심)."""
    return ((pose_a.x + pose_b.x) / 2.0, (pose_a.y + pose_b.y) / 2.0)


def axis_alignment_rotation(current_yaw, target_axis_rad):
    """차량을 슬롯 축(graph.py의 slot_axis_rad, mod pi)에 맞추는 데 필요한
    최소 회전량(rad, signed).

    코/꼬리 방향(0~2pi)이 아니라 축(0~pi)만 맞으면 되므로 — 직사각형
    슬롯은 180도 돌려도 같은 자리에 들어간다 — 그 반대 방향(180도 더 돎)은
    항상 손해다. 그래서 결과를 (-pi/2, pi/2] 범위로 접어서, "이보다 큰
    회전은 반대쪽으로 도는 게 항상 더 짧다"가 항상 성립하게 한다.

    반환값을 그대로 PivotRotateController의 target_angle_rad로 넘기면 된다.
    """
    diff = (target_axis_rad - current_yaw) % math.pi
    if diff > math.pi / 2:
        diff -= math.pi
    return diff


def needs_rotation(current_yaw, target_axis_rad, tol_rad=0.05) -> bool:
    """축 오차가 허용 범위를 넘어서 실제로 피벗 회전이 필요한지."""
    return abs(axis_alignment_rotation(current_yaw, target_axis_rad)) > tol_rad


def group_yaw_progress(own_yaw, partner_yaw, start_own_yaw, start_partner_yaw):
    """회전 시작 시점 대비 "그룹 전체가 지금까지 실제로 돈 각도".

    두 로봇 각각의 yaw 변화량 평균을 쓴다 — 한쪽만 보면 그 로봇의 미끄러짐/
    오차에 결과가 통째로 좌우되므로, 둘 다 반영해서 강체 가정이 살짝
    깨졌을 때(둘이 조금 다르게 돌았을 때)도 완만하게 수렴시킨다.
    """
    d_own = _wrap_angle(own_yaw - start_own_yaw)
    d_partner = _wrap_angle(partner_yaw - start_partner_yaw)
    return _wrap_angle((d_own + d_partner) / 2.0)


class PivotRotateController:
    """두 로봇이 차량 중심 기준으로 목표각(rad, CCW+)만큼 제자리 회전하도록
    매 tick cmd_vel을 계산한다. 로봇 1대마다 인스턴스 하나씩 띄운다.
    """

    def __init__(self, target_angle_rad, k_omega=1.0, max_omega=0.15,
                 max_linear=0.3):
        self.target_angle_rad = target_angle_rad
        self.k_omega = k_omega
        self.max_omega = max_omega
        self.max_linear = max_linear

    def compute(self, own: Pose2D, partner: Pose2D,
                start_own: Pose2D, start_partner: Pose2D) -> PivotCommand:
        progress = group_yaw_progress(
            own.yaw, partner.yaw, start_own.yaw, start_partner.yaw)
        remaining = _wrap_angle(self.target_angle_rad - progress)
        omega = _clamp(self.k_omega * remaining, -self.max_omega, self.max_omega)

        cx, cy = formation_center(own, partner)
        rx, ry = own.x - cx, own.y - cy
        # 접선속도 = ω × r (world frame). (-ry, rx)는 r을 90도(CCW) 돌린 방향.
        world_vx = -omega * ry
        world_vy = omega * rx

        # world -> 자기 body frame (own.yaw 기준 역회전)
        c, s = math.cos(own.yaw), math.sin(own.yaw)
        body_vx = world_vx * c + world_vy * s
        body_vy = -world_vx * s + world_vy * c

        body_vx = _clamp(body_vx, -self.max_linear, self.max_linear)
        body_vy = _clamp(body_vy, -self.max_linear, self.max_linear)

        return PivotCommand(linear_x=body_vx, linear_y=body_vy, angular_z=omega)

    def is_settled(self, own: Pose2D, partner: Pose2D,
                   start_own: Pose2D, start_partner: Pose2D, tol_rad) -> bool:
        progress = group_yaw_progress(
            own.yaw, partner.yaw, start_own.yaw, start_partner.yaw)
        return abs(_wrap_angle(self.target_angle_rad - progress)) <= tol_rad
