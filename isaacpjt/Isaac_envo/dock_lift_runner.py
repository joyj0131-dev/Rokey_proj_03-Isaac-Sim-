#!/usr/bin/env python3
"""ROS2 촉발 도킹·리프트·운반 Isaac 러너.

씬: A5_Coupe(콜라이더 수정 + 그립 재질) + 낮춘 바퀴 로봇 2대(축 앞 접근 위치).
ROS2: /robot_N/cmd_vel 구독, /robot_N/odom 발행, /robot_N/arm_control 서비스.
프로그램은 켜두고 대기 — 외부 오케스트레이터(dock_lift_mission.py)가 구동한다.

실행: dock_lift_runner.sh [--gui] [--headless-test]
"""
import math
import os
import sys
from pathlib import Path

WORK_DIR = Path(__file__).resolve().parent
ROBOT_USD = (WORK_DIR.parent / "hwia_parking_robot_final_caster_package"
             / "hwia_depth_cam_mecha_roller_lowered.usd")
VEHICLES_USD = WORK_DIR / "fab_vehicles.usd"
ISAAC_PYTHON = Path("/home/rokey/dev_ws/isaac_sim/isaacsim/_build/linux-x86_64/release/python.sh")
BRIDGE_RCLPY = Path("/home/rokey/dev_ws/isaac_sim/isaacsim/_build/linux-x86_64/release"
                    "/exts/isaacsim.ros2.bridge/humble/rclpy")

TARGET_VEHICLE = "Coupe"                 # fab_vehicles: /World/Vehicles/Coupe
# fab 전체를 /World/VehicleAsset 로 참조(재질·차량물리 바인딩 유지) 후 대상만 재배치.
COUPE_PATH = f"/World/VehicleAsset/Vehicles/{TARGET_VEHICLE}"
FAB_VEHICLE_TYPES = ("Compact", "Coupe", "Hatchback", "Minivan", "Offroad",
                     "Pickup", "Sedan", "Sport", "SUV", "Wagon")
# 휠 콜라이더 폭(원본 Coupe height=0.504)을 이 값으로 축소해 로봇 진입 여유 확보.
# 정적 측정이 엇갈려(시각 0.739 vs 콜라이더 0.504) 튜너블 상수로 두고 GUI 실측 조정.
TARGET_COLLIDER_WIDTH = 0.30
ARM_TARGETS = {
    "arm_left_front_joint": 90.0, "arm_left_rear_joint": -90.0,
    "arm_right_front_joint": -90.0, "arm_right_rear_joint": 90.0,
}
VEHICLE_WHEELS = ("FrontLeftWheel", "FrontRightWheel", "RearLeftWheel", "RearRightWheel")
ROBOT_APPROACH_GAP_M = 2.60   # 축에서 이만큼 떨어져 스폰(차 오버행과 겹침 방지, 실측)

ROBOTS = {
    "rear":  {"xform": "/World/Robots/robot_rear",  "facing": +1},
    "front": {"xform": "/World/Robots/robot_front", "facing": -1},
}
AXLE = {}   # build_stage()가 채운다


def _restart_with_isaac_python():
    if os.environ.get("CARB_APP_PATH"):
        return
    os.execv(str(ISAAC_PYTHON), [str(ISAAC_PYTHON), str(Path(__file__).resolve()), *sys.argv[1:]])


def _fix_vehicle_colliders(stage, vehicle_path):
    """대상 차량 바퀴 Collision 실린더의 height(축 X=횡폭)를 TARGET_COLLIDER_WIDTH로 축소.

    원본 비파괴 — 런타임 stage override. 구동계 raycast 서스펜션은 콜라이더 형상을
    안 쓰므로 물리 무관. 반환: 수정한 바퀴 수(0이면 프림 매칭 실패 → 트리 확인).
    """
    from pxr import UsdGeom
    fixed = 0
    for wheel_name in VEHICLE_WHEELS:
        col = stage.GetPrimAtPath(f"{vehicle_path}/{wheel_name}/Collision")
        if col and col.IsValid() and col.IsA(UsdGeom.Cylinder):
            UsdGeom.Cylinder(col).GetHeightAttr().Set(float(TARGET_COLLIDER_WIDTH))
            fixed += 1
    return fixed


def _grip_material(stage):
    """파지 마찰재(static 1.2 / dynamic 1.0). carry demo 검증값."""
    from pxr import UsdShade, UsdPhysics
    mat = UsdShade.Material.Define(stage, "/World/Looks/Grip")
    api = UsdPhysics.MaterialAPI.Apply(mat.GetPrim())
    api.CreateStaticFrictionAttr(1.2)
    api.CreateDynamicFrictionAttr(1.0)
    api.CreateRestitutionAttr(0.0)
    return mat


def _place_robot(UsdGeom, Gf, prim, facing, tx, tz):
    """로봇을 세우고(Z-up->Y-up) world ±Z를 향하게 배치.

    듀얼 필드 검증 규약(Translate + RotateX-90)에 facing용 RotateY를 더한다.
    op 순서상 RotateX(세움) → RotateY(방향) → Translate 로 적용된다.
    facing=+1 -> +Z 향함(RotateY -90), -1 -> -Z 향함(RotateY +90).
    """
    face_yaw = -90.0 if facing > 0 else 90.0
    xf = UsdGeom.Xformable(prim)
    xf.ClearXformOpOrder()
    xf.AddTranslateOp().Set(Gf.Vec3d(tx, 0.06, tz))   # 정착 높이 위에서 떨어뜨려 안정
    xf.AddRotateYOp().Set(face_yaw)
    xf.AddRotateXOp().Set(-90.0)


def build_stage(app):
    from pxr import Gf, PhysxSchema, UsdGeom, UsdPhysics
    import omni.usd

    ctx = omni.usd.get_context()
    ctx.new_stage()
    stage = ctx.get_stage()
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.y)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    world = UsdGeom.Xform.Define(stage, "/World")
    stage.SetDefaultPrim(world.GetPrim())

    # 물리씬 — carry demo 검증 설정 그대로(차량 리프트 안정성에 CCD·안정화·240Hz 필요).
    scene = UsdPhysics.Scene.Define(stage, "/World/PhysicsScene")
    scene.CreateGravityDirectionAttr(Gf.Vec3f(0, -1, 0))
    scene.CreateGravityMagnitudeAttr(9.81)
    px = PhysxSchema.PhysxSceneAPI.Apply(scene.GetPrim())
    px.CreateBroadphaseTypeAttr("GPU")
    px.CreateSolverTypeAttr("TGS")
    px.CreateEnableCCDAttr(True)
    px.CreateEnableStabilizationAttr(True)
    px.CreateEnableGPUDynamicsAttr(True)
    px.CreateTimeStepsPerSecondAttr(240)
    # PhysX Vehicle 컨텍스트 — 차량 구동계(raycast 서스펜션)가 이걸 요구한다.
    # 없으면 기본값으로 오작동해 폭발한다(실측).
    vctx = PhysxSchema.PhysxVehicleContextAPI.Apply(scene.GetPrim())
    vctx.CreateUpdateModeAttr(PhysxSchema.Tokens.velocityChange)
    vctx.CreateVerticalAxisAttr(PhysxSchema.Tokens.posY)
    vctx.CreateLongitudinalAxisAttr(PhysxSchema.Tokens.posZ)

    ground = UsdGeom.Cube.Define(stage, "/World/Ground")
    ground.CreateSizeAttr(1.0)
    UsdGeom.Xformable(ground).AddTranslateOp().Set(Gf.Vec3d(0, -0.05, 0))
    UsdGeom.Xformable(ground).AddScaleOp().Set(Gf.Vec3f(40, 0.1, 40))
    UsdPhysics.CollisionAPI.Apply(ground.GetPrim())

    _grip_material(stage)

    # fab_vehicles 전체 참조 → 재질(Looks)·차량물리(VehiclePhysics) 바인딩이 살아 있다.
    # (서브프림만 참조하면 형제 스코프가 끊겨 재질·물리가 깨지고 fab 진열 회전이 딸려온다.)
    asset = stage.DefinePrim("/World/VehicleAsset", "Xform")
    asset.GetReferences().AddReference(str(VEHICLES_USD))
    for _ in range(20):
        app.update()

    # fab 부속(자체 물리씬·조명·바닥·쇼룸 디스크)과 비대상 차량 9종 비활성화.
    for name in ("PhysicsScene", "DriveGround", "FabLighting", "Cylinder001"):
        p = stage.GetPrimAtPath(f"/World/VehicleAsset/{name}")
        if p and p.IsValid():
            p.SetActive(False)
    for vt in FAB_VEHICLE_TYPES:
        if vt == TARGET_VEHICLE:
            continue
        p = stage.GetPrimAtPath(f"/World/VehicleAsset/Vehicles/{vt}")
        if p and p.IsValid():
            p.SetActive(False)

    # 대상 Coupe: fab 진열 회전을 지우고 원점에 yaw=0(세로=길이축 z)으로 재배치.
    coupe = stage.GetPrimAtPath(COUPE_PATH)
    if not coupe or not coupe.IsValid():
        raise RuntimeError(f"대상 차량 없음: {COUPE_PATH}")
    xf = UsdGeom.Xformable(coupe)
    xf.ClearXformOpOrder()
    m = Gf.Matrix4d(1.0)
    m.SetTranslateOnly(Gf.Vec3d(0.0, 0.035, 0.0))   # 주차 배치 규약(y=0.035)
    xf.AddTransformOp().Set(m)
    for _ in range(20):
        app.update()

    n_fixed = _fix_vehicle_colliders(stage, COUPE_PATH)

    cache = UsdGeom.XformCache()
    centers = {}
    for wn in VEHICLE_WHEELS:
        w = stage.GetPrimAtPath(f"{COUPE_PATH}/{wn}")
        if not w.IsValid():
            raise RuntimeError(f"휠 없음: {COUPE_PATH}/{wn}")
        centers[wn] = cache.GetLocalToWorldTransform(w).ExtractTranslation()
    front_z = (centers["FrontLeftWheel"][2] + centers["FrontRightWheel"][2]) * 0.5
    rear_z = (centers["RearLeftWheel"][2] + centers["RearRightWheel"][2]) * 0.5
    center_x = sum(c[0] for c in centers.values()) / 4.0
    low_z, high_z = sorted((front_z, rear_z))
    AXLE.update(rear_z=low_z, front_z=high_z, center_x=center_x)
    ROBOTS["rear"]["target_z"] = low_z
    ROBOTS["rear"]["start_z"] = low_z - ROBOT_APPROACH_GAP_M
    ROBOTS["front"]["target_z"] = high_z
    ROBOTS["front"]["start_z"] = high_z + ROBOT_APPROACH_GAP_M

    for key, cfg in ROBOTS.items():
        r = stage.DefinePrim(cfg["xform"], "Xform")
        r.GetReferences().AddReference(str(ROBOT_USD))
        _place_robot(UsdGeom, Gf, r, cfg["facing"], center_x, cfg["start_z"])
    for _ in range(30):
        app.update()
    print(f"DOCK_STAGE_READY axle rear_z={low_z:.3f} front_z={high_z:.3f} "
          f"wheelbase={abs(high_z-low_z):.3f} center_x={center_x:.3f} colliders_fixed={n_fixed}",
          flush=True)
    return stage


def main():
    _restart_with_isaac_python()
    from isaacsim import SimulationApp
    headless = "--gui" not in sys.argv[1:]
    app = SimulationApp({"headless": headless, "width": 1280, "height": 800})
    try:
        from isaacsim.core.utils.extensions import enable_extension
        enable_extension("isaacsim.ros2.bridge")
        for _ in range(12):
            app.update()
        import numpy as np
        import omni.timeline
        from isaacsim.core.prims import Articulation

        build_stage(app)
        timeline = omni.timeline.get_timeline_interface()
        timeline.play()
        for _ in range(30):
            app.update()

        arts = {}
        for key, cfg in ROBOTS.items():
            art = Articulation(f"{cfg['xform']}/base_link")
            art.initialize()
            arts[key] = art

        if "--headless-test" in sys.argv[1:]:
            def _p(a):
                return np.asarray(a.get_world_poses()[0]).reshape(-1)[:3]
            p0 = {k: _p(a) for k, a in arts.items()}
            for _ in range(180):
                app.update()
            p1 = {k: _p(a) for k, a in arts.items()}
            disp = {k: float(np.linalg.norm(p1[k] - p0[k])) for k in arts}
            ok = all(d < 0.35 for d in disp.values())
            print(f"DOCK_PHYSICS_TEST={'PASS' if ok else 'FAIL'} "
                  f"disp={ {k: round(v,4) for k,v in disp.items()} }", flush=True)
            app.close()
            return
        # (Task 2~3에서 ROS2 루프 추가)
        app.close()
    finally:
        pass


if __name__ == "__main__":
    main()
