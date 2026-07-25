#!/usr/bin/env python3
"""Reusable mecanum-drive overlay for the HWIA parking robot.

Two pieces the rest of the project can share:

1. ``add_mecanum_rollers`` -- authors passive 45-degree rollers on the four
   fixed drive wheels of a robot instance, as a non-destructive override on the
   test stage (the source robot asset is never modified). Validated by
   ``mecanum_strafe_test.py`` (~4.37 m clean strafe, 2/2 runs).

2. ``wheel_velocities_from_cmd_vel`` -- mecanum inverse kinematics mapping a
   holonomic ``/cmd_vel`` (vx forward, vy left, wz yaw, all robot-frame) to the
   four hub angular velocities [rad/s]. Sign calibration is verified by
   ``mecanum_holonomic_test.py``.

The robot frame is X-forward, Y-left, Z-up.
"""

import math


# --- Drive-wheel geometry (robot frame, from the URDF) ----------------------
WHEEL_RADIUS = 0.060
WHEEL_CENTERS = {
    "wheel_fl": (0.68, 0.35, 0.060),
    "wheel_fr": (0.68, -0.35, 0.060),
    "wheel_rl": (-0.68, 0.35, 0.060),
    "wheel_rr": (-0.68, -0.35, 0.060),
}
WHEEL_JOINTS = {
    "wheel_fl": "wheel_fl_joint",
    "wheel_fr": "wheel_fr_joint",
    "wheel_rl": "wheel_rl_joint",
    "wheel_rr": "wheel_rr_joint",
}
LX = 0.68   # half wheelbase along X (forward)
LY = 0.35   # half track along Y (left)

# --- Mecanum roller overlay -------------------------------------------------
# X-configuration chirality: +1 => rollers tilted toward the +Y axle.
ROLLER_CHIRALITY = {"wheel_fl": +1, "wheel_fr": -1, "wheel_rl": -1, "wheel_rr": +1}
N_ROLLERS = 10
ROLLER_RADIUS = 0.018
ROLLER_HALF_LEN = 0.010            # cylindrical half-height of the capsule
ROLLER_MASS = 0.05
R_MOUNT = WHEEL_RADIUS - ROLLER_RADIUS

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


def _quat_from_z_to(direction, Gf):
    """Quaternion (Quatf) rotating local +Z onto the given unit direction."""
    z = Gf.Vec3d(0.0, 0.0, 1.0)
    d = direction.GetNormalized()
    dot = max(-1.0, min(1.0, z * d))
    if dot > 0.999999:
        return Gf.Quatf(1.0, 0.0, 0.0, 0.0)
    if dot < -0.999999:
        return Gf.Quatf(0.0, 1.0, 0.0, 0.0)  # 180 deg about X
    axis = Gf.Cross(z, d).GetNormalized()
    half = math.acos(dot) * 0.5
    s = math.sin(half)
    return Gf.Quatf(math.cos(half), float(axis[0] * s), float(axis[1] * s), float(axis[2] * s))


def _disable_collisions(prim, Usd, UsdPhysics):
    for p in Usd.PrimRange(prim):
        if p.HasAPI(UsdPhysics.CollisionAPI):
            UsdPhysics.CollisionAPI(p).CreateCollisionEnabledAttr(False)


def add_mecanum_rollers(stage, robot_wrap, robot_joints, grip_material=None):
    """Replace each drive wheel's ground contact with passive 45-degree rollers.

    stage:        Usd.Stage
    robot_wrap:   prim path of the referenced-robot wrapper (holds wheel links)
    robot_joints: prim path of the robot's joint scope
    grip_material: optional UsdShade.Material to bind on each roller
    """
    from pxr import Gf, Usd, UsdGeom, UsdPhysics, UsdShade

    for wheel_name, center in WHEEL_CENTERS.items():
        wheel_path = f"{robot_wrap}/{wheel_name}"
        wheel_prim = stage.GetPrimAtPath(wheel_path)
        if not wheel_prim.IsValid():
            raise RuntimeError(f"wheel prim missing: {wheel_path}")

        # The bare hub must not touch the floor; only the rollers do.
        _disable_collisions(wheel_prim, Usd, UsdPhysics)

        chir = ROLLER_CHIRALITY[wheel_name]
        for k in range(N_ROLLERS):
            theta = 2.0 * math.pi * k / N_ROLLERS
            local_off = Gf.Vec3d(R_MOUNT * math.cos(theta), 0.0, R_MOUNT * math.sin(theta))
            tangential = Gf.Vec3d(-math.sin(theta), 0.0, math.cos(theta))
            axle = Gf.Vec3d(0.0, float(chir), 0.0)
            spin_axis = (tangential + axle).GetNormalized()
            q = _quat_from_z_to(spin_axis, Gf)

            roller_path = f"{robot_wrap}/roller_{wheel_name}_{k}"
            capsule = UsdGeom.Capsule.Define(stage, roller_path)
            capsule.CreateRadiusAttr(ROLLER_RADIUS)
            capsule.CreateHeightAttr(2.0 * ROLLER_HALF_LEN)
            capsule.CreateAxisAttr("Z")
            capsule.CreateDisplayColorAttr([Gf.Vec3f(0.85, 0.7, 0.2)])
            roller_prim = capsule.GetPrim()

            world_pos = Gf.Vec3d(center[0], center[1], center[2]) + local_off
            xform = UsdGeom.Xformable(roller_prim)
            xform.ClearXformOpOrder()
            xform.AddTranslateOp().Set(world_pos)
            xform.AddOrientOp(UsdGeom.XformOp.PrecisionFloat).Set(q)

            UsdPhysics.CollisionAPI.Apply(roller_prim)
            UsdPhysics.RigidBodyAPI.Apply(roller_prim)
            UsdPhysics.MassAPI.Apply(roller_prim).CreateMassAttr(ROLLER_MASS)
            if grip_material is not None:
                UsdShade.MaterialBindingAPI.Apply(roller_prim).Bind(
                    grip_material, UsdShade.Tokens.weakerThanDescendants, "physics"
                )

            joint_path = f"{robot_joints}/roller_{wheel_name}_{k}_joint"
            joint = UsdPhysics.RevoluteJoint.Define(stage, joint_path)
            joint.CreateBody0Rel().SetTargets([wheel_path])
            joint.CreateBody1Rel().SetTargets([roller_path])
            joint.CreateAxisAttr("Z")
            joint.CreateLocalPos0Attr(Gf.Vec3f(local_off))
            joint.CreateLocalRot0Attr(q)
            joint.CreateLocalPos1Attr(Gf.Vec3f(0.0, 0.0, 0.0))
            joint.CreateLocalRot1Attr(Gf.Quatf(1.0, 0.0, 0.0, 0.0))


def configure_hub_drives(stage, robot_joints, damping=1500.0, max_force=6000.0):
    """Set the four hubs to velocity drive (stiffness 0) for cmd_vel control."""
    from pxr import UsdPhysics

    for jname in WHEEL_JOINTS.values():
        joint = stage.GetPrimAtPath(f"{robot_joints}/{jname}")
        drive = UsdPhysics.DriveAPI.Get(joint, "angular")
        if not drive:
            drive = UsdPhysics.DriveAPI.Apply(joint, "angular")
        drive.CreateStiffnessAttr(0.0)
        drive.CreateDampingAttr(damping)
        drive.CreateMaxForceAttr(max_force)
        drive.CreateTargetVelocityAttr(0.0)
