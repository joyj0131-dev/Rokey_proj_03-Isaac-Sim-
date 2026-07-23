#!/usr/bin/env python3
"""마찰 A/B 실험: 바닥+로봇 최소 씬에서 전진 지령 0.35로 실속 측정.
바닥·롤러 마찰만 바꿔(인자 F) 메카넘 30% 슬립이 변하는지 확인.
사용: friction_experiment.sh <F>  (예: 1.0, 3.0)
"""
import os
import sys
from pathlib import Path

WORK_DIR = Path(__file__).resolve().parent
ROBOT_USD = (WORK_DIR.parent / "hwia_parking_robot_final_caster_package"
             / "hwia_depth_cam_mecha_roller_lowered.usd")
ISAAC_PYTHON = Path("/home/rokey/dev_ws/isaac_sim/isaacsim/_build/linux-x86_64/release/python.sh")
FRIC = float(sys.argv[1]) if len(sys.argv) > 1 else 1.0


def _restart():
    if os.environ.get("CARB_APP_PATH"):
        return
    os.execv(str(ISAAC_PYTHON), [str(ISAAC_PYTHON), str(Path(__file__).resolve()), *sys.argv[1:]])


def main():
    _restart()
    from isaacsim import SimulationApp
    app = SimulationApp({"headless": True})
    import numpy as np
    import omni.timeline
    import omni.usd
    from pxr import Gf, UsdGeom, UsdPhysics, UsdShade, PhysxSchema
    from isaacsim.core.prims import Articulation

    ctx = omni.usd.get_context(); ctx.new_stage(); stage = ctx.get_stage()
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.y)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)

    scene = UsdPhysics.Scene.Define(stage, "/World/PhysicsScene")
    scene.CreateGravityDirectionAttr(Gf.Vec3f(0, -1, 0))
    scene.CreateGravityMagnitudeAttr(9.81)
    px = PhysxSchema.PhysxSceneAPI.Apply(scene.GetPrim())
    px.CreateBroadphaseTypeAttr("GPU"); px.CreateSolverTypeAttr("TGS")
    px.CreateEnableGPUDynamicsAttr(True); px.CreateTimeStepsPerSecondAttr(240)

    # 큰 바닥 + 물리 재질(마찰=FRIC)
    ground = UsdGeom.Cube.Define(stage, "/World/Ground"); ground.CreateSizeAttr(1.0)
    gx = UsdGeom.Xformable(ground)
    gx.AddTranslateOp().Set(Gf.Vec3d(0, -0.05, 0))
    gx.AddScaleOp().Set(Gf.Vec3f(200, 0.1, 200))
    UsdPhysics.CollisionAPI.Apply(ground.GetPrim())
    fmat = UsdShade.Material.Define(stage, "/World/FloorMat")
    fapi = UsdPhysics.MaterialAPI.Apply(fmat.GetPrim())
    fapi.CreateStaticFrictionAttr(FRIC); fapi.CreateDynamicFrictionAttr(max(0.1, FRIC - 0.2))
    fapi.CreateRestitutionAttr(0.0)
    UsdShade.MaterialBindingAPI.Apply(ground.GetPrim())
    UsdShade.MaterialBindingAPI(ground.GetPrim()).Bind(
        fmat, UsdShade.Tokens.weakerThanDescendants, "physics")

    # 로봇(Z-up->Y-up)
    robot = stage.DefinePrim("/World/Robot", "Xform")
    robot.GetReferences().AddReference(str(ROBOT_USD))
    rxf = UsdGeom.Xformable(robot); rxf.ClearXformOpOrder()
    rxf.AddTranslateOp().Set(Gf.Vec3d(0, 0.06, 0)); rxf.AddRotateXOp().Set(-90.0)
    for _ in range(40):
        app.update()

    # 롤러 재질 마찰도 FRIC 로 덮기
    n_over = 0
    for p in stage.Traverse():
        if p.HasAPI(UsdPhysics.MaterialAPI) and "Robot" in str(p.GetPath()):
            api = UsdPhysics.MaterialAPI(p)
            if api.GetStaticFrictionAttr():
                api.GetStaticFrictionAttr().Set(FRIC)
                api.GetDynamicFrictionAttr().Set(max(0.1, FRIC - 0.2))
                n_over += 1

    timeline = omni.timeline.get_timeline_interface(); timeline.play()
    for _ in range(30):
        app.update()

    art = Articulation("/World/Robot/base_link"); art.initialize()
    sys.path.insert(0, str(WORK_DIR))
    from mecanum_drive import WHEEL_JOINTS, configure_hub_drives, wheel_velocities_from_cmd_vel
    configure_hub_drives(stage, "/World/Robot/joints")
    widx = {w: art.dof_names.index(j) for w, j in WHEEL_JOINTS.items()}
    vel = np.zeros(art.get_joint_positions().shape, dtype=np.float32)

    def drive(vx, vy, wz):
        om = wheel_velocities_from_cmd_vel(vx, vy, wz)
        vel[...] = 0.0
        for w, o in om.items():
            i = widx[w]
            if vel.ndim == 2:
                vel[0, i] = o
            else:
                vel[i] = o
        art.set_joint_velocity_targets(vel)

    def pos():
        return np.asarray(art.get_world_poses()[0]).reshape(-1)[:3]

    # 스핀업(정상속도 도달) 후 측정 창
    for _ in range(60):
        drive(0.35, 0.0, 0.0); app.update()
    t0 = timeline.get_current_time(); p0 = pos()
    for _ in range(180):
        drive(0.35, 0.0, 0.0); app.update()
    t1 = timeline.get_current_time(); p1 = pos()
    drive(0.0, 0.0, 0.0)
    for _ in range(20):
        app.update()

    disp = float(np.linalg.norm((p1 - p0)[[0, 2]]))
    dt = max(1e-3, t1 - t0)
    v = disp / dt
    print(f"FRICTION_RESULT F={FRIC} roller_overrides={n_over} | "
          f"전진 {disp:.3f}m / {dt:.2f}s = {v:.3f} m/s = 지령0.35의 {v/0.35*100:.0f}%",
          flush=True)
    app.close()


if __name__ == "__main__":
    main()
