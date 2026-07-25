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

--- R1 이관(2026-07-25) 안내 ---
이 파일은 USD/pxr 에 의존하는 "저작(authoring)" 절반(add_mecanum_rollers,
configure_hub_drives 등)만 여기 남았다. Isaac 을 전혀 모르는 "순수 기구학"
절반(상수 WHEEL_JOINTS/WHEEL_RADIUS/LX/LY/SIGN_*/YAW_SCALE, 함수 slew_twist/
wheel_velocities_from_cmd_vel/cmd_vel_from_wheel_velocities)은
``src/parkbot_motion/parkbot_motion/mecanum_kinematics.py`` 로 이전됐다(ROS2
노드도 같은 로직을 import 해야 하는데 이 파일은 pxr 없이는 import조차 안 되기
때문). 아래는 그 이전된 이름들을 그대로 재-export 하는 하위호환 shim이다 —
기존에 ``from mecanum_drive import WHEEL_JOINTS, ...`` 로 쓰던 코드
(dock_lift_handoff_runner(_v2).py, build_depth_cam_mecha_asset.py,
build_mecha_roller_asset.py 등)는 한 글자도 안 고쳐도 그대로 동작한다.
"""

import math
import sys
from pathlib import Path

# 순수 기구학은 parkbot_motion 패키지가 단일 소스다(중복 정의 금지 — 값이
# 갈라지면 Isaac 저작 쪽과 제어 쪽이 서로 다른 상수를 쓰게 된다). 이 파일을
# 직접 import 하는 레거시 스크립트들은 저마다 다른 방식으로 sys.path 를
# 세팅하므로(WORK_DIR, ISAAC_ENVO 등) 여기서 __file__ 기준 절대경로로 한 번 더
# 넣어 항상 찾아지게 한다.
_REPO_ROOT = Path(__file__).resolve().parent.parent.parent
sys.path.insert(0, str(_REPO_ROOT / "src" / "parkbot_motion"))
from parkbot_motion.mecanum_kinematics import (  # noqa: E402
    WHEEL_RADIUS,
    WHEEL_JOINTS,
    LX,
    LY,
    SIGN_FORWARD,
    SIGN_STRAFE,
    SIGN_YAW,
    YAW_SCALE,
    _move_toward,
    slew_twist,
    wheel_velocities_from_cmd_vel,
    cmd_vel_from_wheel_velocities,
)

# --- Drive-wheel geometry (robot frame, from the URDF) ----------------------
# Isaac 저작(add_mecanum_rollers)만 쓰는 좌표 — 순수 기구학 쪽은 필요 없어서
# mecanum_kinematics.py 로 옮기지 않았다.
WHEEL_CENTERS = {
    "wheel_fl": (0.68, 0.35, 0.060),
    "wheel_fr": (0.68, -0.35, 0.060),
    "wheel_rl": (-0.68, 0.35, 0.060),
    "wheel_rr": (-0.68, -0.35, 0.060),
}

# --- Mecanum roller overlay -------------------------------------------------
# X-configuration chirality: +1 => rollers tilted toward the +Y axle.
ROLLER_CHIRALITY = {"wheel_fl": +1, "wheel_fr": -1, "wheel_rl": -1, "wheel_rr": +1}
N_ROLLERS = 10
ROLLER_RADIUS = 0.018
ROLLER_HALF_LEN = 0.010            # cylindrical half-height of the capsule
ROLLER_MASS = 0.05
R_MOUNT = WHEEL_RADIUS - ROLLER_RADIUS


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
