#!/usr/bin/env python3
"""RTF 확인: 무거운 주차장 씬에서 로봇 전진을 sim시간·벽시계 둘 다로 측정.
가설: 인계장의 '30% 슬립'은 실제 슬립이 아니라 무거운 씬의 real-time factor(RTF≈0.3).
  - sim속도 ≈ 100% (지령대로) 이고
  - 벽시계속도 ≈ sim속도 × RTF ≈ 30%
이면 가설 확정.
"""
import os
import sys
import time as _time
from pathlib import Path

WORK_DIR = Path(__file__).resolve().parent
PARKING_USD = WORK_DIR / "parking" / "parking_environment_with_markers.usd"
ROBOT_USD = (WORK_DIR.parent / "hwia_parking_robot_final_caster_package"
             / "hwia_depth_cam_mecha_roller_lowered.usd")
ISAAC_PYTHON = Path("/home/rokey/dev_ws/isaac_sim/isaacsim/_build/linux-x86_64/release/python.sh")


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
    from pxr import Gf, UsdGeom, UsdPhysics, PhysxSchema
    from isaacsim.core.prims import Articulation

    ctx = omni.usd.get_context(); ctx.new_stage(); stage = ctx.get_stage()
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.y)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    # 무거운 주차장 전체 씬 서브레이어
    stage.GetRootLayer().subLayerPaths.append(str(PARKING_USD))
    for _ in range(5):
        app.update()
    # 차량 폭발 방지(vehicle context)
    sc = stage.GetPrimAtPath("/World/PhysicsScene")
    px = PhysxSchema.PhysxSceneAPI.Apply(sc)
    px.CreateBroadphaseTypeAttr("GPU"); px.CreateEnableGPUDynamicsAttr(True)
    px.CreateTimeStepsPerSecondAttr(240)
    vctx = PhysxSchema.PhysxVehicleContextAPI.Apply(sc)
    vctx.CreateUpdateModeAttr(PhysxSchema.Tokens.velocityChange)
    vctx.CreateVerticalAxisAttr(PhysxSchema.Tokens.posY)
    vctx.CreateLongitudinalAxisAttr(PhysxSchema.Tokens.posZ)

    # 로봇을 아일(원점, 개방부) 에 배치 (Z-up->Y-up)
    robot = stage.DefinePrim("/World/Robot", "Xform")
    robot.GetReferences().AddReference(str(ROBOT_USD))
    rxf = UsdGeom.Xformable(robot); rxf.ClearXformOpOrder()
    rxf.AddTranslateOp().Set(Gf.Vec3d(0.0, 0.06, 0.0)); rxf.AddRotateXOp().Set(-90.0)
    for _ in range(40):
        app.update()

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

    for _ in range(60):
        drive(0.35, 0.0, 0.0); app.update()
    sim0 = timeline.get_current_time(); wall0 = _time.time(); p0 = pos()
    for _ in range(180):
        drive(0.35, 0.0, 0.0); app.update()
    sim1 = timeline.get_current_time(); wall1 = _time.time(); p1 = pos()
    drive(0.0, 0.0, 0.0)
    for _ in range(20):
        app.update()

    disp = float(np.linalg.norm((p1 - p0)[[0, 2]]))
    sim_dt = max(1e-3, sim1 - sim0); wall_dt = max(1e-3, wall1 - wall0)
    v_sim = disp / sim_dt; v_wall = disp / wall_dt
    print(f"RTF_RESULT | 전진 {disp:.3f}m | sim {sim_dt:.2f}s→{v_sim:.3f} m/s ({v_sim/0.35*100:.0f}%) "
          f"| wall {wall_dt:.2f}s→{v_wall:.3f} m/s ({v_wall/0.35*100:.0f}%) "
          f"| RTF={sim_dt/wall_dt:.2f}", flush=True)
    app.close()


if __name__ == "__main__":
    main()
