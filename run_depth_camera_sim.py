#!/usr/bin/env python3
"""End-to-end test: import the camera-mesh URDF straight into Isaac Sim,
replace the crude box+cylinders camera mesh add_camera_mesh.py drew in the
URDF with a real Isaac Sim sensor asset (Intel RealSense D455, referenced
from the Isaac asset library) on all four camera housings (side x2, front,
rear), run physics with the robot driving forward, and capture RGB + depth
frames to prove the whole pipeline works.

The front/rear cameras are pitched 45 deg down (see _ROT_FRONT/_ROT_REAR)
so they can see parking-lot floor markers as the robot approaches/leaves,
rather than looking level like the side cameras.

This supersedes the manual "import via GUI in between" step implied by
add_camera_mesh.py / add_depth_cameras.py: the URDF -> USD import is done
here programmatically (URDFParseAndImportFile), so the whole thing runs as
one headless script.

Sensor orientation: cameras must look straight OUT from the chassis (world
+Y for the left housing, -Y for the right one) so that, once the robot
drives under a vehicle, each side camera can see the tire next to it. Both
cam_side_*_link joints mount their link with a +/-90 deg rotation about X --
see _add_rsd455_camera for the derivation of the wrapper rotation this
requires. (An earlier version of this script used a bare synthetic Camera
prim rotated -90 deg about Y to align with the lens-strip visuals' local +X
face from add_camera_mesh.py; that made it look along world +X/forward --
i.e. mostly at the robot's own folded arms -- instead of outward. Fixed,
and then superseded by the real RSD455 asset here.)

Run:
  cd /home/rokey/dev_ws/isaac_sim/isaacsim/_build/linux-x86_64/release
  ./python.sh /home/rokey/p3/Rokey_proj_03-Isaac-Sim-/run_depth_camera_sim.py
"""

import json
import math
import os
import sys
from pathlib import Path

PROJECT_DIR = Path(__file__).resolve().parent
URDF_PATH = PROJECT_DIR / "hwia_parking_robot_final_caster_camera_mesh.urdf"
OUTPUT_USD = PROJECT_DIR / "hwia_parking_robot_final_caster_camera_mesh_depth_cam_scene.usd"
REPORT_PATH = PROJECT_DIR / "depth_camera_sim_report.json"

FRAMES_DIR = Path(os.environ.get(
    "DEPTH_CAM_FRAMES_DIR",
    "/tmp/claude-1000/-home-rokey-p3/9c903056-d71c-4d61-b5f4-934c3a661485/scratchpad/depth_cam_frames",
))

ISAAC_PYTHON = Path("/home/rokey/dev_ws/isaac_sim/isaacsim/_build/linux-x86_64/release/python.sh")

LEFT_LINK_NAME = "cam_side_left_link"
RIGHT_LINK_NAME = "cam_side_right_link"
FRONT_LINK_NAME = "cam_front_link"
REAR_LINK_NAME = "cam_rear_link"

# The crude housing box add_camera_mesh.py/the URDF bakes in for each of
# these links (local Z half-depth 0.0125 m for the sides, local X
# half-depth 0.017 m for front/rear) is centered at the link origin; mount
# the real sensor just proud of the outward-facing face -- local -Z for the
# sides, local +/-X for front/rear -- see _add_rsd455_camera's docstring for
# why those are the outward faces, not the old lens-strip visual's local +X
# (which was authored without accounting for the joint rotation each link
# gets, see add_camera_mesh.py).
SIDE_LOCAL_OFFSET = (0.0, 0.0, -0.0145)
FRONT_LOCAL_OFFSET = (0.014, 0.0, 0.0)
REAR_LOCAL_OFFSET = (-0.014, 0.0, 0.0)

# Side cameras: identity view/up mapped through the link's own +/-90 deg
# joint rotation about X already lands outward and level -- see
# _add_rsd455_camera's docstring for the row-vector derivation.
_ROT_LEFT = (0, 0, -1, -1, 0, 0, 0, 1, 0)
_ROT_RIGHT = (0, 0, -1, 1, 0, 0, 0, -1, 0)

# Front/rear cameras: front_link/rear_link have identity joint rpy (same
# frame as base_link), and referencing rsd455.usd with an identity wrapper
# already looks down link-local +X with up = link-local +Z (the asset's own
# intrinsic camera-in-/Root orientation -- see _add_rsd455_camera). So a
# level, forward-facing front camera would need NO wrapper rotation at all.
# We want it pitched FLOOR_TILT_DEG downward instead (to see parking-lot
# floor markers as the robot approaches), which is just a pitch about local
# Y on top of that: Ry(theta) applied to view=(1,0,0), left=(0,1,0), and
# up=(0,0,1) gives view=(cos,0,-sin), left=(0,1,0) [unchanged -- pitch about
# Y doesn't move Y-axis vectors], up=(sin,0,cos) -- tilting toward the
# ground while staying pointed forward.
#
# The rear camera is the same thing yawed 180 deg about Z first (so "up"
# stays up, unlike a 180-about-X/Y flip): Rz(180) sends (x,y,z)->(-x,-y,z),
# so it negates the X and Y components of each of the front vectors above:
# view=(-cos,0,-sin), left=(0,-1,0), up=(-sin,0,cos). (First version of this
# had up=(-cos,0,sin) instead -- a copy/paste slip from `view`'s pattern
# rather than actually applying Rz(180) to `up`. It went unnoticed because
# at exactly 45 deg, sin==cos, so the wrong and right formulas coincide;
# raising FLOOR_TILT_DEG to 65 exposed it -- the rear camera stopped
# responding to the angle change at all, and checking the matrix showed why:
# row0 (view) and row2 (the wrong "up") were no longer orthogonal, i.e. not
# a valid rotation, once sin != cos.) Both verified empirically via rendered
# RGB frames, same as the sides.
FLOOR_TILT_DEG = 30.0
_TILT = math.radians(FLOOR_TILT_DEG)
_C, _S = math.cos(_TILT), math.sin(_TILT)
_ROT_FRONT = (_C, 0, -_S, 0, 1, 0, _S, 0, _C)
_ROT_REAR = (-_C, 0, -_S, 0, -1, 0, -_S, 0, _C)

CLIPPING_RANGE = (0.05, 8.0)   # m -- override the asset's default (0.01, 1e6):
                                # far enough for this test scene, keeps depth stats sane
RESOLUTION = (640, 480)

# Real Isaac Sim sensor asset (Intel RealSense D455 -- proper housing mesh,
# glass, mount, USB-C, materials, plus RGB/stereo/pseudo-depth Camera prims)
# instead of the crude box+cylinders mesh add_camera_mesh.py draws directly
# in the URDF. Resolved against the configured Isaac Sim asset root at
# runtime (nucleus.get_assets_root_path()).
RSD455_RELATIVE_PATH = "/Isaac/Sensors/Intel/RealSense/rsd455.usd"

WHEEL_JOINT_NAMES = ["wheel_fl_joint", "wheel_fr_joint", "wheel_rl_joint", "wheel_rr_joint"]
DRIVE_VELOCITY_RAD_S = 4.0
DRIVE_DAMPING = 3000.0
DRIVE_MAX_FORCE = 8000.0

SETTLE_STEPS = 60
DRIVE_STEPS = 240
CAPTURE_EVERY = 20

OVERVIEW_CAM_EYE = (4.2, -3.4, 3.4)
OVERVIEW_CAM_TARGET = (0.8, -0.3, 0.15)

# Two long static walls flanking the drive path so the side depth cameras
# have real environment geometry to range against (not just the robot's own
# folded arms right next to the housings).
WALL_Y = 1.3        # +/- from the centerline
WALL_HALF_LEN = 4.0  # along X
WALL_HEIGHT = 1.0
WALL_THICKNESS = 0.05

# Flat floor markers (parking-lot-style paint patches) ahead of and behind
# the robot's start position, so the front/rear cameras have something
# marker-like on the ground to actually range/see once they're tilted down.
MARKER_SIZE = 0.35
MARKER_X = 1.6   # ahead of the robot's start pose
MARKER_THICKNESS = 0.01


def _restart_with_isaac_python():
    if os.environ.get("CARB_APP_PATH"):
        return
    if not ISAAC_PYTHON.is_file():
        raise FileNotFoundError(f"Isaac Sim python.sh not found: {ISAAC_PYTHON}")
    os.execv(str(ISAAC_PYTHON), [str(ISAAC_PYTHON), str(Path(__file__).resolve()), *sys.argv[1:]])


def _find_prim_by_name(stage, name):
    matches = [p for p in stage.Traverse() if p.GetName() == name]
    if not matches:
        return None
    if len(matches) > 1:
        print(f"WARNING: multiple prims named '{name}', using first: "
              f"{[m.GetPath().pathString for m in matches]}", flush=True)
    return matches[0]


def _lookat_matrix(Gf, eye, target, up=(0.0, 0.0, 1.0)):
    e = Gf.Vec3d(*eye)
    f = (Gf.Vec3d(*target) - e).GetNormalized()
    r = (Gf.Cross(f, Gf.Vec3d(*up))).GetNormalized()
    u = Gf.Cross(r, f)
    return Gf.Matrix4d(
        r[0], r[1], r[2], 0.0,
        u[0], u[1], u[2], 0.0,
        -f[0], -f[1], -f[2], 0.0,
        e[0], e[1], e[2], 1.0,
    )


def _import_robot():
    import omni.kit.commands
    from isaacsim.core.utils.extensions import enable_extension

    enable_extension("isaacsim.asset.importer.urdf")

    status, import_config = omni.kit.commands.execute("URDFCreateImportConfig")
    import_config.merge_fixed_joints = False   # keep cam_side_*_link as separate prims
    import_config.convex_decomp = False
    import_config.import_inertia_tensor = True
    import_config.fix_base = False             # robot drives on its wheels
    import_config.distance_scale = 1.0
    import_config.self_collision = False

    status, prim_path = omni.kit.commands.execute(
        "URDFParseAndImportFile",
        urdf_path=str(URDF_PATH),
        import_config=import_config,
        get_articulation_root=True,
    )
    if not status or not prim_path:
        raise RuntimeError(f"URDF import failed: status={status} prim_path={prim_path}")
    return prim_path


def _setup_physics_and_env(stage):
    from pxr import Gf, PhysicsSchemaTools, PhysxSchema, Sdf, UsdLux, UsdPhysics

    scene = UsdPhysics.Scene.Define(stage, Sdf.Path("/physicsScene"))
    scene.CreateGravityDirectionAttr().Set(Gf.Vec3f(0.0, 0.0, -1.0))
    scene.CreateGravityMagnitudeAttr().Set(9.81)
    PhysxSchema.PhysxSceneAPI.Apply(stage.GetPrimAtPath("/physicsScene"))
    physx_scene = PhysxSchema.PhysxSceneAPI.Get(stage, "/physicsScene")
    physx_scene.CreateEnableCCDAttr(True)
    physx_scene.CreateEnableStabilizationAttr(True)
    physx_scene.CreateEnableGPUDynamicsAttr(False)
    physx_scene.CreateBroadphaseTypeAttr("MBP")
    physx_scene.CreateSolverTypeAttr("TGS")

    PhysicsSchemaTools.addGroundPlane(stage, "/groundPlane", "Z", 50.0, Gf.Vec3f(0, 0, 0), Gf.Vec3f(0.55))

    light = UsdLux.DistantLight.Define(stage, Sdf.Path("/DistantLight"))
    light.CreateIntensityAttr(1200)
    light.CreateAngleAttr(1.0)


def _add_flanking_walls(stage):
    from pxr import Gf, UsdGeom, UsdPhysics

    for side, y in (("left", WALL_Y), ("right", -WALL_Y)):
        path = f"/World_Wall_{side}"
        cube = UsdGeom.Cube.Define(stage, path)
        cube.CreateSizeAttr(1.0)
        cube.CreateDisplayColorAttr([Gf.Vec3f(0.55, 0.57, 0.6)])
        xform = UsdGeom.Xformable(cube.GetPrim())
        xform.ClearXformOpOrder()
        xform.AddTranslateOp().Set(Gf.Vec3d(WALL_HALF_LEN - 1.5, y, WALL_HEIGHT / 2.0))
        xform.AddScaleOp().Set(Gf.Vec3d(WALL_HALF_LEN, WALL_THICKNESS, WALL_HEIGHT))
        UsdPhysics.CollisionAPI.Apply(cube.GetPrim())


def _add_floor_markers(stage):
    from pxr import Gf, UsdGeom

    for side, x, color in (
        ("front", MARKER_X, Gf.Vec3f(0.95, 0.75, 0.05)),   # yellow, ahead of start
        ("rear", -MARKER_X, Gf.Vec3f(0.95, 0.35, 0.05)),   # orange, behind start
    ):
        path = f"/World_FloorMarker_{side}"
        cube = UsdGeom.Cube.Define(stage, path)
        cube.CreateSizeAttr(1.0)
        cube.CreateDisplayColorAttr([color])
        xform = UsdGeom.Xformable(cube.GetPrim())
        xform.ClearXformOpOrder()
        xform.AddTranslateOp().Set(Gf.Vec3d(x, 0.0, MARKER_THICKNESS / 2.0))
        xform.AddScaleOp().Set(Gf.Vec3d(MARKER_SIZE, MARKER_SIZE, MARKER_THICKNESS))


def _add_rsd455_camera(stage, parent_prim, cam_name, rsd455_usd_path, side_label, rot, local_offset):
    """Hide the crude box+cylinders mesh add_camera_mesh.py/the URDF draws
    directly for this link's "visuals" and replace it with a referenced,
    real Intel RealSense D455 asset (actual housing mesh/materials + its own
    Camera_Pseudo_Depth prim), oriented per the caller-supplied wrapper
    rotation `rot` (a Gf.Matrix3d; see module-level _ROT_LEFT/_ROT_RIGHT/
    _ROT_FRONT/_ROT_REAR for the derivations, all following the same
    method): referencing rsd455.usd with an identity wrapper transform, its
    Camera_Pseudo_Depth looks down the wrapper's local +X with "up" = the
    wrapper's local +Z (measured directly from the asset: Camera_Pseudo_
    Depth's world matrix when /Root has no transform). `rot` is then simply
    the rotation that carries that (view=+X, up=+Z) baseline to whatever the
    desired final view/up directions are *in the link's own local frame*,
    expressed as row vectors (image of each input axis, matching Usd/Gf's
    row-vector convention) -- i.e. rot's row0 is the desired view direction,
    row2 is the desired up direction, and row1 (right) follows from
    orthonormality. Built directly as a matrix (rather than hand-composed
    Euler angles) to avoid sign mistakes; verified empirically via rendered
    RGB frames each time a new one was derived.
    """
    from pxr import Gf, Sdf, UsdGeom, UsdPhysics

    visuals_prim = parent_prim.GetChild("visuals")
    if visuals_prim.IsValid():
        UsdGeom.Imageable(visuals_prim).MakeInvisible()

    wrap_path = parent_prim.GetPath().AppendChild(cam_name)
    wrap_prim = UsdGeom.Xform.Define(stage, wrap_path).GetPrim()
    wrap_prim.GetReferences().AddReference(rsd455_usd_path)

    # rsd455.usd authors its /Root/RSD455 as a free-standing RigidBodyAPI
    # prim (for referencing in stand-alone as its own dynamic object).
    # Parented under cam_side_*_link -- itself already a rigid body link in
    # this robot's articulation -- that produces an invalid nested-rigid-
    # body hierarchy ("missing xformstack reset when child of another
    # enabled rigid body"). Disable it: this mount should move rigidly with
    # the link, not be its own physics body.
    rsd_prim = stage.GetPrimAtPath(wrap_path.AppendChild("RSD455"))
    if rsd_prim.HasAPI(UsdPhysics.RigidBodyAPI):
        UsdPhysics.RigidBodyAPI(rsd_prim).CreateRigidBodyEnabledAttr(False)
        rsd_prim.RemoveAPI(UsdPhysics.RigidBodyAPI)

    xform_matrix = Gf.Matrix4d(Gf.Matrix3d(*rot), Gf.Vec3d(*local_offset))

    xform = UsdGeom.Xformable(wrap_prim)
    xform.ClearXformOpOrder()
    xform.AddTransformOp().Set(xform_matrix)

    # rsd455.usd's 4 Camera prims (Color/Left/Right/Pseudo_Depth) get the
    # same leaf names on both the left and right mount, so the viewport's
    # camera picker shows indistinguishable duplicate entries. Kit's
    # MovePrim command refuses to rename them in place ("Cannot move/rename
    # ancestral prim" -- prims that only exist because of a reference arc
    # can't be renamed through it, confirmed empirically). So: deactivate
    # the original Camera_Pseudo_Depth (and the 3 unused OmniVision
    # RGB/stereo prims -- we don't use those at all) and define a plain,
    # clearly-named sibling Camera carrying the same intrinsics, at the
    # same transform. That transform is RSD455/Camera_Pseudo_Depth's fixed,
    # asset-intrinsic local-to-/Root matrix (measured directly by opening
    # rsd455.usd standalone: translate 0, and this rotation matrix) --
    # constant regardless of side, since it's baked into the source asset
    # rather than depending on our own left/right wrapper rotation above.
    for unused_name in ("Camera_Pseudo_Depth", "Camera_OmniVision_OV9782_Color",
                        "Camera_OmniVision_OV9782_Left", "Camera_OmniVision_OV9782_Right"):
        unused_prim = stage.GetPrimAtPath(wrap_path.AppendPath(f"RSD455/{unused_name}"))
        if unused_prim.IsValid():
            unused_prim.SetActive(False)

    cam_path = wrap_path.AppendChild(f"Camera_Pseudo_Depth_{side_label}")
    camera = UsdGeom.Camera.Define(stage, cam_path)
    camera.CreateFocalLengthAttr(1.93)
    camera.CreateHorizontalApertureAttr(3.896)
    camera.CreateVerticalApertureAttr(2.453)
    camera.CreateClippingRangeAttr(Gf.Vec2f(*CLIPPING_RANGE))
    camera.CreateProjectionAttr(UsdGeom.Tokens.perspective)
    asset_cam_rot = Gf.Matrix3d(0, -1, 0, 0, 0, 1, -1, 0, 0)
    UsdGeom.Xformable(camera.GetPrim()).AddTransformOp().Set(Gf.Matrix4d(asset_cam_rot, Gf.Vec3d(0, 0, 0)))

    return cam_path.pathString


GUI_MODE = os.environ.get("DEPTH_CAM_GUI", "0") == "1"


def build_and_run(app):
    import numpy as np
    import imageio.v2 as imageio
    import omni.timeline
    import omni.usd
    from pxr import Gf, UsdGeom
    from isaacsim.core.prims import Articulation
    from isaacsim.sensors.camera import Camera

    FRAMES_DIR.mkdir(parents=True, exist_ok=True)
    for old in FRAMES_DIR.glob("*.png"):
        old.unlink()

    robot_prim_path = _import_robot()
    print(f"IMPORTED_ROOT={robot_prim_path}", flush=True)
    for _ in range(5):
        app.update()

    stage = omni.usd.get_context().get_stage()
    _setup_physics_and_env(stage)
    _add_flanking_walls(stage)
    _add_floor_markers(stage)

    links = {name: _find_prim_by_name(stage, name) for name in
             (LEFT_LINK_NAME, RIGHT_LINK_NAME, FRONT_LINK_NAME, REAR_LINK_NAME)}
    missing = [name for name, prim in links.items() if prim is None]
    if missing:
        raise RuntimeError(f"camera housing links missing after import: {missing}")
    left_link, right_link, front_link, rear_link = (
        links[LEFT_LINK_NAME], links[RIGHT_LINK_NAME], links[FRONT_LINK_NAME], links[REAR_LINK_NAME]
    )

    from isaacsim.core.utils.nucleus import get_assets_root_path
    assets_root = get_assets_root_path()
    if not assets_root:
        raise RuntimeError("Could not resolve the Isaac Sim asset root (nucleus not reachable)")
    rsd455_usd_path = assets_root.rstrip("/") + RSD455_RELATIVE_PATH
    print(f"RSD455_USD={rsd455_usd_path}", flush=True)

    left_cam_path = _add_rsd455_camera(
        stage, left_link, "depth_cam_left", rsd455_usd_path, "Left", _ROT_LEFT, SIDE_LOCAL_OFFSET
    )
    right_cam_path = _add_rsd455_camera(
        stage, right_link, "depth_cam_right", rsd455_usd_path, "Right", _ROT_RIGHT, SIDE_LOCAL_OFFSET
    )
    front_cam_path = _add_rsd455_camera(
        stage, front_link, "depth_cam_front", rsd455_usd_path, "Front", _ROT_FRONT, FRONT_LOCAL_OFFSET
    )
    rear_cam_path = _add_rsd455_camera(
        stage, rear_link, "depth_cam_rear", rsd455_usd_path, "Rear", _ROT_REAR, REAR_LOCAL_OFFSET
    )
    print(f"CAM_LEFT_PATH={left_cam_path}", flush=True)
    print(f"CAM_RIGHT_PATH={right_cam_path}", flush=True)
    print(f"CAM_FRONT_PATH={front_cam_path}", flush=True)
    print(f"CAM_REAR_PATH={rear_cam_path}", flush=True)

    # Fixed 3rd-person overview camera so we can visually sanity-check the
    # imported robot + housings from outside.
    overview = UsdGeom.Camera.Define(stage, "/World_OverviewCam")
    overview.CreateClippingRangeAttr(Gf.Vec2f(0.05, 100.0))
    UsdGeom.Xformable(overview.GetPrim()).MakeMatrixXform().Set(
        _lookat_matrix(Gf, OVERVIEW_CAM_EYE, OVERVIEW_CAM_TARGET)
    )

    # Close-up on the left RSD455 housing itself (mesh/material sanity check).
    # NOTE: UsdGeom.Camera's schema-fallback clippingRange is (1, 1e6) --
    # without overriding it, anything closer than 1m (everything, here) gets
    # near-clipped and renders solid black. Must set this explicitly for any
    # camera placed this close to its subject.
    left_link_pos = UsdGeom.XformCache().GetLocalToWorldTransform(left_link).ExtractTranslation()
    closeup = UsdGeom.Camera.Define(stage, "/World_HousingCloseupCam")
    closeup.CreateClippingRangeAttr(Gf.Vec2f(0.05, 5.0))
    UsdGeom.Xformable(closeup.GetPrim()).MakeMatrixXform().Set(_lookat_matrix(
        Gf,
        (left_link_pos[0] + 0.18, left_link_pos[1] + 0.28, left_link_pos[2] + 0.15),
        (left_link_pos[0], left_link_pos[1] + 0.01, left_link_pos[2]),
    ))

    for _ in range(10):
        app.update()

    cam_left = Camera(prim_path=left_cam_path, resolution=RESOLUTION)
    cam_right = Camera(prim_path=right_cam_path, resolution=RESOLUTION)
    cam_front = Camera(prim_path=front_cam_path, resolution=RESOLUTION)
    cam_rear = Camera(prim_path=rear_cam_path, resolution=RESOLUTION)
    cam_overview = Camera(prim_path="/World_OverviewCam", resolution=(960, 540))
    cam_closeup = Camera(prim_path="/World_HousingCloseupCam", resolution=(960, 720))
    depth_cams = {"left": cam_left, "right": cam_right, "front": cam_front, "rear": cam_rear}
    for cam in (*depth_cams.values(), cam_overview, cam_closeup):
        cam.initialize()
    for cam in depth_cams.values():
        cam.add_distance_to_image_plane_to_frame()

    timeline = omni.timeline.get_timeline_interface()
    timeline.play()
    for _ in range(SETTLE_STEPS):
        app.update()

    # Articulation.initialize() needs SimulationManager's physics sim view
    # ready, which occasionally isn't yet on the first attempt in GUI mode
    # (rendering slows/paces frame delivery differently than headless) --
    # seen as "AttributeError: 'NoneType' object has no attribute
    # 'create_articulation_view'". Retry a few frames rather than fail the
    # whole run over a startup race.
    from isaacsim.core.simulation_manager import SimulationManager

    for _ in range(120):
        if SimulationManager.get_physics_sim_view() is not None:
            break
        app.update()
    robot = Articulation(robot_prim_path)
    robot.initialize()

    def base_pos():
        # Articulation.get_world_poses() (tensor-API backed) rather than the
        # raw omni.physx.get_physx_interface().get_rigidbody_transformation()
        # call -- the latter intermittently fails to resolve robot_prim_path
        # ("did not locate any objects at the specified path") when running
        # non-headless with rendering on, even though the same path is valid.
        pos, _ = robot.get_world_poses()
        return tuple(float(x) for x in np.asarray(pos)[0])

    start_pos = base_pos()

    # NOTE: Articulation's joint_names= kwarg resolves names against the full
    # joint list (which also includes the robot's fixed sensor joints), but
    # applies the resulting index into DOF-only arrays -- an index-space
    # mismatch that silently drives the wrong joints whenever any fixed
    # joint appears before the target in the URDF (verified empirically: it
    # was spinning the bearing rollers/casters instead of the wheels here).
    # Resolving indices via get_dof_index (DOF-space) and passing
    # joint_indices= instead sidesteps the bug.
    n_wheels = len(WHEEL_JOINT_NAMES)
    wheel_dof_indices = np.array([robot.get_dof_index(n) for n in WHEEL_JOINT_NAMES])
    robot.set_gains(
        kps=np.zeros((1, n_wheels), dtype=np.float32),
        kds=np.full((1, n_wheels), DRIVE_DAMPING, dtype=np.float32),
        joint_indices=wheel_dof_indices,
    )
    robot.set_max_efforts(
        np.full((1, n_wheels), DRIVE_MAX_FORCE, dtype=np.float32),
        joint_indices=wheel_dof_indices,
    )
    robot.set_joint_velocity_targets(
        np.full((1, n_wheels), DRIVE_VELOCITY_RAD_S, dtype=np.float32),
        joint_indices=wheel_dof_indices,
    )

    frames_report = []

    def capture(step_idx, tag):
        result = {"step": step_idx, "tag": tag}
        for name, cam in depth_cams.items():
            rgba = cam.get_rgba()
            depth = cam.get_depth()
            rgb_valid = rgba is not None and getattr(rgba, "size", 0) and int(rgba[..., :3].max()) > 4
            if rgb_valid:
                imageio.imwrite(
                    str(FRAMES_DIR / f"{tag}_{name}_rgb_{step_idx:04d}.png"),
                    np.asarray(rgba[..., :3], dtype=np.uint8),
                )
            depth_stats = None
            if depth is not None and getattr(depth, "size", 0):
                d = np.asarray(depth, dtype=np.float32)
                finite = d[np.isfinite(d) & (d > 0)]
                depth_stats = {
                    "min": float(finite.min()) if finite.size else None,
                    "max": float(finite.max()) if finite.size else None,
                    "mean": float(finite.mean()) if finite.size else None,
                    "valid_fraction": float(finite.size) / float(d.size) if d.size else 0.0,
                }
            result[name] = {"rgb_captured": bool(rgb_valid), "depth": depth_stats}
        for cam_name, cam_obj in (("overview", cam_overview), ("closeup", cam_closeup)):
            frame_rgba = cam_obj.get_rgba()
            if frame_rgba is not None and getattr(frame_rgba, "size", 0) and int(frame_rgba[..., :3].max()) > 4:
                imageio.imwrite(
                    str(FRAMES_DIR / f"{tag}_{cam_name}_{step_idx:04d}.png"),
                    np.asarray(frame_rgba[..., :3], dtype=np.uint8),
                )
        frames_report.append(result)

    capture(0, "before_drive")

    for i in range(1, DRIVE_STEPS + 1):
        app.update()
        if i % CAPTURE_EVERY == 0:
            capture(i, "driving")

    if not GUI_MODE:
        timeline.pause()
    end_pos = base_pos()
    displacement = tuple(float(e - s) for e, s in zip(end_pos, start_pos))
    distance = float(sum(d * d for d in displacement) ** 0.5)

    stage.GetRootLayer().Export(str(OUTPUT_USD))

    report = {
        "urdf": str(URDF_PATH),
        "robot_prim_path": robot_prim_path,
        "cam_left_path": left_cam_path,
        "cam_right_path": right_cam_path,
        "cam_front_path": front_cam_path,
        "cam_rear_path": rear_cam_path,
        "start_pos": start_pos,
        "end_pos": end_pos,
        "displacement": displacement,
        "distance_traveled_m": distance,
        "drive_velocity_rad_s": DRIVE_VELOCITY_RAD_S,
        "drive_steps": DRIVE_STEPS,
        "frames_dir": str(FRAMES_DIR),
        "output_scene_usd": str(OUTPUT_USD),
        "frames": frames_report,
    }
    REPORT_PATH.write_text(json.dumps(report, indent=2))
    print(f"DISTANCE_TRAVELED_M={distance:.3f}", flush=True)
    print(f"REPORT={REPORT_PATH}", flush=True)
    print(f"FRAMES_DIR={FRAMES_DIR}", flush=True)
    print(f"OUTPUT_SCENE_USD={OUTPUT_USD}", flush=True)
    print("RUN_OK=True", flush=True)

    if GUI_MODE:
        print("GUI_MODE: leaving the robot driving -- close the Isaac Sim window to stop.", flush=True)
        while app.is_running():
            app.update()


def main():
    _restart_with_isaac_python()
    from isaacsim import SimulationApp

    app = SimulationApp({"headless": not GUI_MODE})
    try:
        build_and_run(app)
    except Exception as exc:
        print(f"RUN_EXCEPTION={type(exc).__name__}: {exc}", flush=True)
        raise
    finally:
        app.close()


if __name__ == "__main__":
    main()
