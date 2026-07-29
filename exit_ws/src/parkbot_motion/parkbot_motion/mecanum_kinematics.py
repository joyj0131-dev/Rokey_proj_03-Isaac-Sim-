#!/usr/bin/env python3
"""Mecanum-drive kinematics — pure Python, no Isaac/USD dependency.

Extracted (R1 이관, byte-for-byte logic unchanged) from the pure half of
``isaacpjt/Isaac_envo/mecanum_drive.py``. The USD-authoring half
(``add_mecanum_rollers``, ``configure_hub_drives``, ...) stayed behind in
that file since it imports ``pxr`` and is Isaac-only; that file now
re-exports the names below from here for backward compatibility.

``wheel_velocities_from_cmd_vel`` -- mecanum inverse kinematics mapping a
holonomic ``/cmd_vel`` (vx forward, vy left, wz yaw, all robot-frame) to the
four hub angular velocities [rad/s]. Sign calibration is verified by
``mecanum_holonomic_test.py``.

The robot frame is X-forward, Y-left, Z-up.
"""

import math


# --- Drive-wheel geometry (robot frame, from the URDF) ----------------------
WHEEL_RADIUS = 0.060
WHEEL_JOINTS = {
    "wheel_fl": "wheel_fl_joint",
    "wheel_fr": "wheel_fr_joint",
    "wheel_rl": "wheel_rl_joint",
    "wheel_rr": "wheel_rr_joint",
}
LX = 0.68   # half wheelbase along X (forward)
LY = 0.35   # half track along Y (left)

# --- cmd_vel -> wheel sign / scale calibration ------------------------------
# vx/vy sign+scale verified by mecanum_holonomic_test.py (that file does not
# exist in this repo any more, but vx/vy are separately confirmed good by the
# mission's ~1cm position accuracy -- do not touch SIGN_FORWARD/SIGN_STRAFE).
SIGN_FORWARD = +1.0
SIGN_STRAFE = -1.0     # so +vy (robot left) drives the chassis toward +Y
#
# YAW re-measured 2026-07-25 (taskBYAW-report.md, --probe=YAWSTEP, fixed-duration
# open-loop spins on hwia_4cam_mecha_roller_lowered.usd -- the asset actually in
# use today, not the caster.usd the old YAW_SCALE=1.12/mecanum_holonomic_test.py
# comment referred to). The previous SIGN_YAW=+1.0/YAW_SCALE=1.12 made
# cmd_vel_from_wheel_velocities (the exact least-squares IK inverse) agree with
# the *commanded* wz, but the robot's *physical* GT rotation was ~2.5-2.95x
# larger and in the OPPOSITE direction across all four measured wz_cmd in
# {+0.3,-0.3,+0.6,-0.6} (k_gt_over_cmd = -2.93, -2.95, -2.61, -2.49; avg=-2.745,
# same sign / same order of magnitude in all 4 -- a single sign+scale
# correction is valid). A commanded 90 deg turn was physically ~-263 deg
# (matches the user's GUI observation of ~270 deg the wrong way); it only
# "worked" for 90/180 deg targets because -270 == +90 and -540 == +180 (mod
# 360) hide a 3x error that any other angle (e.g. 45 deg) would expose. Fix:
# new SIGN_YAW*YAW_SCALE = old(+1.0*1.12) / avg_k = 1.12 / -2.745 = -0.4080, so
# that a commanded wz now produces a physical wz (verified after the fix:
# --probe=YAWSTEP k_gt_over_cmd ~= +1.0, --probe=ROTCHK/ROTCHK180/ROTCHK45
# small gt_err_deg, see report). In-place yaw is still roller-slip dominated,
# so this remains an empirical fit, not a geometric constant -- re-measure with
# --probe=YAWSTEP if the wheel/roller asset changes again.
SIGN_YAW = -1.0
YAW_SCALE = 0.4080


def _move_toward(current, target, max_delta):
    """Move one scalar toward its target without overshooting."""
    delta = target - current
    if abs(delta) <= max_delta:
        return target
    return current + math.copysign(max_delta, delta)


def slew_twist(current, target, dt, linear_accel=0.5, linear_decel=0.8,
               angular_accel=0.8):
    """Apply a simulation-time acceleration limit to a body twist.

    Linear X/Y are limited as one vector so diagonal motion does not receive
    sqrt(2) times more acceleration.  A command that removes velocity uses the
    higher deceleration limit; top speed is not changed.
    """
    current = tuple(float(v) for v in current)
    target = tuple(float(v) for v in target)
    dt = max(0.0, float(dt))
    if dt == 0.0:
        return current

    cvx, cvy, cwz = current
    tvx, tvy, twz = target
    dvx, dvy = tvx - cvx, tvy - cvy
    delta_norm = math.hypot(dvx, dvy)
    # current·delta < 0 means the command is braking or reversing.
    linear_rate = linear_decel if cvx * dvx + cvy * dvy < 0.0 else linear_accel
    max_linear_delta = max(0.0, linear_rate) * dt
    if delta_norm > max_linear_delta > 0.0:
        scale = max_linear_delta / delta_norm
        cvx += dvx * scale
        cvy += dvy * scale
    else:
        cvx, cvy = tvx, tvy

    cwz = _move_toward(cwz, twz, max(0.0, angular_accel) * dt)
    return cvx, cvy, cwz


def wheel_velocities_from_cmd_vel(vx, vy, wz):
    """Map a robot-frame holonomic /cmd_vel to hub angular velocities [rad/s].

    vx: forward [m/s], vy: left [m/s], wz: yaw (CCW+) [rad/s].
    Returns {wheel_name: omega_rad_s}.
    """
    fx = SIGN_FORWARD * vx
    sy = SIGN_STRAFE * vy
    wl = SIGN_YAW * YAW_SCALE * wz * (LX + LY)
    return {
        "wheel_fl": (fx - sy - wl) / WHEEL_RADIUS,
        "wheel_fr": (fx + sy + wl) / WHEEL_RADIUS,
        "wheel_rl": (fx + sy - wl) / WHEEL_RADIUS,
        "wheel_rr": (fx - sy + wl) / WHEEL_RADIUS,
    }


def cmd_vel_from_wheel_velocities(omegas):
    """IK의 최소자승 역: 휠 각속도 dict -> (vx, vy, wz) 로봇 로컬 twist.

    IK가 선형이므로 4x3 행렬의 pseudo-inverse 로 정확히 복원된다. 계수는
    IK 함수에서 수치적으로 추출한다(상수 중복 금지 — IK 가 바뀌면 FK 도 따라간다).
    """
    import numpy as np

    wheels = list(WHEEL_JOINTS)
    basis = [(1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0)]
    A = np.array([[wheel_velocities_from_cmd_vel(*b)[w] for b in basis]
                  for w in wheels])                       # (4, 3)
    vec = np.array([float(omegas[w]) for w in wheels])    # (4,)
    vx, vy, wz = np.linalg.lstsq(A, vec, rcond=None)[0]
    return float(vx), float(vy), float(wz)
