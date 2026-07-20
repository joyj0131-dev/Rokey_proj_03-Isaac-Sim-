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
# Verified by mecanum_holonomic_test.py against the authored roller chirality.
SIGN_FORWARD = +1.0
SIGN_STRAFE = -1.0     # so +vy (robot left) drives the chassis toward +Y
SIGN_YAW = +1.0
# vx/vy map ~1:1. In-place yaw is roller-slip dominated (faster hub spin -> more
# slip -> less realised yaw), so the yaw gain is fit empirically at the wz~0.5
# operating point rather than from geometry. See mecanum_holonomic_test.py.
YAW_SCALE = 1.12


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
