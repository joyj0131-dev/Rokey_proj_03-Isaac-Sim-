#!/usr/bin/env python3
"""v3 주차장(parking_environment_v3.usd) 기반 도킹·리프트·오미 운반 Isaac 러너.

v1 러너(dock_lift_handoff_runner.py)를 팀원의 소형 주차장 에셋(v3)에 맞춰 변형한 것.
v1 원본은 그대로 보존한다.

씬 구성:
  - 주차장 전체 환경 parking_environment_v3.usd (PhysX VehicleContext·ArUco 마커 14장 내장).
  - 로봇 2대(낮춘 깊이캠+메카넘 로봇)를 v3의 두 로봇 도크에 배치:
      robot_rear  -> EntryRobotDock  dockPose (-8.5, 0, -1.7)
      robot_front -> ExitRobotDock   dockPose (-8.5, 0,  1.7)
  - 입차 차량 Pickup 을 게이트 바깥 에이프런(z+ 쪽) (-17.3, 0.040, +5.5) 에
      yaw=-90(길이축을 x축에 맞추고 게이트를 등짐)으로 배치. 운전자가 두고 간 상태.
      -> 콜라이더 폭 보정 + 축 좌표 계산 + /vehicle/pose 발행.
      (참고: 실내 대기 베이 ExitVehicleWait(-8.5,*,+5.5)=ArUco id 51 은 별도 구역이다.)
  - 오프로드 차를 슬롯 A3 (9.6, 0.035, 0) 에 이미 주차된 상태로 배치(출차/장애물 대상).
      -> 구동계 유지(레이캐스트 서스펜션이 지지해 안정). /parking_slots 에서 A3 점유로 보고.

ROS2: /robot_N/cmd_vel 구독, /robot_N/odom 발행(x,y=높이,z,yaw),
      /robot_N/arm_control 서비스, /vehicle/pose(운반 대상 Pickup) 발행,
      /parking_slots(A1~A3 점유) 발행. parking_robot_system 의 액션 서버·오케스트레이터가 구동.

실행: dock_lift_handoff_runner_v2.sh [--gui] [--headless-test] [--drive-test]
"""
import json
import math
import os
import sys
import time
from pathlib import Path

WORK_DIR = Path(__file__).resolve().parent
# 팀원이 환경 에셋을 v2 -> v3 로 갈아끼웠다(좌표는 동일: 슬롯 A1~A3 x=2.8/6.2/9.6 z=0,
# 도크 (-8.5,0,+-1.7), 에이프런 (-17.3,-0.055,+-5.5) 8x5, 실내 바닥 윗면 y=0, 마커 14장).
# 파일명이 계속 바뀌므로 없으면 무엇이 있는지 찍어주고 즉시 죽는다(조용한 오작동 방지).
PARKING_USD = WORK_DIR / "parking" / "parking_environment_v3.usd"
ROBOT_USD = (WORK_DIR.parent / "hwia_parking_robot_final_caster_package"
             / "hwia_depth_cam_mecha_roller_lowered.usd")
VEHICLES_USD = WORK_DIR / "fab_vehicles.usd"
ISAAC_PYTHON = Path("/home/rokey/dev_ws/isaac_sim/isaacsim/_build/linux-x86_64/release/python.sh")
BRIDGE_RCLPY = Path("/home/rokey/dev_ws/isaac_sim/isaacsim/_build/linux-x86_64/release"
                    "/exts/isaacsim.ros2.bridge/humble/rclpy")

# 운반 대상(입차) + 이미 주차된 차(출차/장애물).
CARRY_VEHICLE = "Pickup"                       # 인계장 z+ 에서 슬롯으로 운반할 대상
PARKED_VEHICLE = "Offroad"                     # 슬롯 A3 에 이미 주차돼 있는 차
CARRY_VEHICLE_PATH = f"/World/VehicleAsset/Vehicles/{CARRY_VEHICLE}"
PARKED_VEHICLE_PATH = f"/World/VehicleAsset/Vehicles/{PARKED_VEHICLE}"
FAB_VEHICLE_TYPES = ("Compact", "Coupe", "Hatchback", "Minivan", "Offroad",
                     "Pickup", "Sedan", "Sport", "SUV", "Wagon")
KEEP_VEHICLES = (CARRY_VEHICLE, PARKED_VEHICLE)
TARGET_COLLIDER_WIDTH = 0.30
RENDER_HZ = 60.0
RENDER_WIDTH = 640
RENDER_HEIGHT = 400
PHYSICS_HZ = 120.0
LINEAR_ACCEL = 0.5       # m/s^2, body X/Y 벡터 가속도
LINEAR_DECEL = 0.8       # m/s^2, 감속·반전은 조금 더 빠르게
ANGULAR_ACCEL = 0.8      # rad/s^2

# 입차 차량은 게이트 **바깥** 외부 에이프런에 둔다(z+ 쪽 = ExitApron 패드).
#   패드: 중심 x=-17.3, z=+5.5, 크기 8(x) x 5(z) -> x∈[-21.3,-13.3], z∈[3.0,8.0]
#   표면 높이: Floor cube scale.y=0.12, translate.y=-0.055 -> 윗면 y=+0.005
#   (실내 바닥은 scale.y=0.12, translate.y=-0.06 -> 윗면 y=0.0. 에이프런이 5mm 높다.)
# 따라서 실내와 같은 3.5cm 여유를 주려면 0.005+0.035 = 0.040.
CARRY_VEHICLE_POS = (-17.3, 0.040, 5.5)
# 패드가 x로 8m / z로 5m라 길이축을 x축에 맞춰야 들어간다. 차량 길이축은 로컬 +Z 이고
# RotateY(t) 는 로컬 +Z 를 (sin t, 0, cos t) 로 보낸다.
# yaw=-90 -> 월드 -X, 즉 게이트(x=-13.15) 를 **등지고** 바깥(서쪽)을 향해 선다.
CARRY_VEHICLE_YAW_DEG = -90.0
# 슬롯 A3 중심. v2 Spaces/A3 = (9.6, *, 0).
PARKED_VEHICLE_POS = (9.6, 0.035, 0.0)

ARM_TARGETS = {
    "arm_left_front_joint": 90.0, "arm_left_rear_joint": -90.0,
    "arm_right_front_joint": -90.0, "arm_right_rear_joint": 90.0,
}
VEHICLE_WHEELS = ("FrontLeftWheel", "FrontRightWheel", "RearLeftWheel", "RearRightWheel")

# 로봇: robot_rear -> EntryRobotDock, robot_front -> ExitRobotDock. 초기엔 +X 향함(도크 기본).
ROBOT_SERVICE = "/World/ParkingEnvironment/RobotServiceArea"
ROBOTS = {
    "rear":  {"xform": "/World/Robots/robot_rear",
              "dock": f"{ROBOT_SERVICE}/EntryRobotDock"},
    "front": {"xform": "/World/Robots/robot_front",
              "dock": f"{ROBOT_SERVICE}/ExitRobotDock"},
}
AXLE = {}

# --- /parking_slots 발행용 (v2 는 A1~A3 세 칸. Spaces/A{i} 중심 z=0) ---
# (x_center, z_center, yaw_deg). 점유 판정은 차량 world (x,z) 가 슬롯 박스 안인지로 한다.
_SLOTS_V2 = {"A1": (2.8, 0.0, 0.0), "A2": (6.2, 0.0, 0.0), "A3": (9.6, 0.0, 0.0)}
_HALF_LEN, _HALF_WID = 2.5, 1.7          # z(길이축) 반, x(폭) 반
_ACCESSIBLE = {"A1"}


def _vehicle_world_positions(stage):
    """활성 fab 차량(운반 대상 + 주차 차량)의 world (x,z)."""
    import omni.usd
    from pxr import UsdGeom
    positions = []
    root = stage.GetPrimAtPath("/World/VehicleAsset/Vehicles")
    if not root or not root.IsValid():
        return positions
    for child in root.GetChildren():
        if not child.IsActive() or not child.IsA(UsdGeom.Xformable):
            continue
        m = UsdGeom.Xformable(child).ComputeLocalToWorldTransform(0)
        t = m.ExtractTranslation()
        positions.append((float(t[0]), float(t[2])))
    return positions


def _restart_with_isaac_python():
    if os.environ.get("CARB_APP_PATH"):
        return
    os.execv(str(ISAAC_PYTHON), [str(ISAAC_PYTHON), str(Path(__file__).resolve()), *sys.argv[1:]])


def _fix_vehicle_colliders(stage, vehicle_path):
    from pxr import UsdGeom
    fixed = 0
    for wheel_name in VEHICLE_WHEELS:
        col = stage.GetPrimAtPath(f"{vehicle_path}/{wheel_name}/Collision")
        if col and col.IsValid() and col.IsA(UsdGeom.Cylinder):
            UsdGeom.Cylinder(col).GetHeightAttr().Set(float(TARGET_COLLIDER_WIDTH))
            fixed += 1
    return fixed


def _grip_material(stage):
    from pxr import UsdShade, UsdPhysics
    mat = UsdShade.Material.Define(stage, "/World/Looks/GripDock")
    api = UsdPhysics.MaterialAPI.Apply(mat.GetPrim())
    api.CreateStaticFrictionAttr(1.2)
    api.CreateDynamicFrictionAttr(1.0)
    api.CreateRestitutionAttr(0.0)
    return mat


def _dock_position(stage, dock_path):
    prim = stage.GetPrimAtPath(dock_path)
    if not prim or not prim.IsValid():
        raise RuntimeError(f"도크 없음: {dock_path}")
    v = prim.GetAttribute("robot:dockPose").Get()
    if v is None:
        raise RuntimeError(f"dockPose 없음: {dock_path}")
    return tuple(float(c) for c in v)


def _place_robot_dock(UsdGeom, Gf, prim, pos):
    """도크에 로봇을 세운다(Z-up->Y-up). 초기 방향은 +X(도크 기본, 회전은 미션이)."""
    xf = UsdGeom.Xformable(prim)
    xf.ClearXformOpOrder()
    xf.AddTranslateOp().Set(Gf.Vec3d(pos[0], 0.06, pos[2]))
    xf.AddRotateXOp().Set(-90.0)


def _place_vehicle(UsdGeom, Gf, prim, pos, yaw_deg=0.0):
    """차량을 world pos 에 yaw_deg 로 놓는다.

    차량 길이축은 로컬 +Z. xformOpOrder=[translate, rotateY] 는 T*R 이므로
    로컬 원점에서 회전한 뒤 이동한다. RotateY(t) 는 로컬 +Z 를 (sin t, 0, cos t) 로
    보내므로 yaw=0 이면 월드 +Z(세로), yaw=90 이면 월드 +X 를 향한다.
    """
    xf = UsdGeom.Xformable(prim)
    xf.ClearXformOpOrder()
    xf.AddTranslateOp().Set(Gf.Vec3d(*pos))
    if abs(float(yaw_deg)) > 1e-9:
        xf.AddRotateYOp().Set(float(yaw_deg))


def _apply_vehicle_context(stage):
    """v2 PhysicsScene 에 GPU/120Hz PhysX 설정을 보강한다.

    v2 에셋의 PhysicsScene 은 이미 PhysxVehicleContextAPI 를 갖고 있으나(에셋 저작 시),
    Broadphase/Solver/GPU/timestep 등 씬 물리 설정은 명시돼 있지 않을 수 있다. v1 러너와
    동일한 설정을 세션 레이어에서 덮어 결정론적 GPU 120Hz 로 맞춘다(재적용은 idempotent).
    """
    from pxr import PhysxSchema
    sc = stage.GetPrimAtPath("/World/PhysicsScene")
    if not sc or not sc.IsValid():
        raise RuntimeError("v2 PhysicsScene 없음 — vehicle context 적용 불가")
    px = PhysxSchema.PhysxSceneAPI.Apply(sc)
    px.CreateBroadphaseTypeAttr("GPU")
    px.CreateSolverTypeAttr("TGS")
    px.CreateEnableCCDAttr(True)
    px.CreateEnableStabilizationAttr(True)
    px.CreateEnableGPUDynamicsAttr(True)
    px.CreateTimeStepsPerSecondAttr(PHYSICS_HZ)
    vctx = PhysxSchema.PhysxVehicleContextAPI.Apply(sc)
    vctx.CreateUpdateModeAttr(PhysxSchema.Tokens.velocityChange)
    vctx.CreateVerticalAxisAttr(PhysxSchema.Tokens.posY)
    vctx.CreateLongitudinalAxisAttr(PhysxSchema.Tokens.posZ)


def _disable_unused_scene(stage):
    """미션에 필요 없는 천장 라이다를 세션 레이어에서 비활성화한다(원본 미수정).

    v2 는 v1 과 달리 사전 배치된 주차 차량(/World/ParkingVehicles)이 없다. /World/VehiclePhysics
    는 타이어 마찰 테이블·충돌 그룹·바닥 그룹을 정의하므로 살려둔다(끄면 차량 접지가 깨진다).
    """
    sensors = stage.GetPrimAtPath("/World/Sensors")
    sensor_count = 0
    if sensors and sensors.IsValid():
        sensor_count = sum(
            "Lidar" in child.GetName() for child in sensors.GetChildren())
        sensors.SetActive(False)
    return sensor_count


def build_stage(app):
    from pxr import Gf, UsdGeom
    import omni.usd

    ctx = omni.usd.get_context()
    ctx.new_stage()
    stage = ctx.get_stage()
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.y)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    stage.SetTimeCodesPerSecond(RENDER_HZ)
    # v2 주차장 전체 환경을 서브레이어로. 런타임 익명 stage라 절대경로.
    if not PARKING_USD.is_file():
        avail = sorted(p.name for p in PARKING_USD.parent.glob("parking_environment*.usd"))
        raise RuntimeError(
            f"주차장 에셋 없음: {PARKING_USD}\n"
            f"  parking/ 안에 있는 것: {avail}\n"
            f"  팀원이 파일명을 바꿨다면 PARKING_USD 상수를 갱신할 것.")
    stage.GetRootLayer().subLayerPaths.append(str(PARKING_USD))
    world = stage.GetPrimAtPath("/World")
    if not world or not world.IsValid():
        raise RuntimeError(f"서브레이어 {PARKING_USD.name} 에서 /World를 찾지 못했습니다.")
    stage.SetDefaultPrim(world)
    _apply_vehicle_context(stage)
    for _ in range(30):
        app.update()

    disabled_sensors = _disable_unused_scene(stage)
    _grip_material(stage)

    # fab 전체 참조 후 Pickup·Offroad 만 남기고 재배치(재질·차량물리 바인딩 유지).
    asset = stage.DefinePrim("/World/VehicleAsset", "Xform")
    asset.GetReferences().AddReference(str(VEHICLES_USD))
    for _ in range(20):
        app.update()
    for name in ("PhysicsScene", "DriveGround", "FabLighting", "Cylinder001"):
        p = stage.GetPrimAtPath(f"/World/VehicleAsset/{name}")
        if p and p.IsValid():
            p.SetActive(False)
    for vt in FAB_VEHICLE_TYPES:
        if vt in KEEP_VEHICLES:
            continue
        p = stage.GetPrimAtPath(f"/World/VehicleAsset/Vehicles/{vt}")
        if p and p.IsValid():
            p.SetActive(False)

    carry = stage.GetPrimAtPath(CARRY_VEHICLE_PATH)
    if not carry or not carry.IsValid():
        raise RuntimeError(f"{CARRY_VEHICLE} 없음: {CARRY_VEHICLE_PATH}")
    _place_vehicle(UsdGeom, Gf, carry, CARRY_VEHICLE_POS, CARRY_VEHICLE_YAW_DEG)

    parked = stage.GetPrimAtPath(PARKED_VEHICLE_PATH)
    if not parked or not parked.IsValid():
        raise RuntimeError(f"{PARKED_VEHICLE} 없음: {PARKED_VEHICLE_PATH}")
    _place_vehicle(UsdGeom, Gf, parked, PARKED_VEHICLE_POS)
    for _ in range(20):
        app.update()

    # 콜라이더 폭 보정·축 계산은 운반 대상(Pickup)에만. 주차된 Offroad 는 구동계 유지로 안정.
    n_fixed = _fix_vehicle_colliders(stage, CARRY_VEHICLE_PATH)

    cache = UsdGeom.XformCache()
    centers = {}
    for wn in VEHICLE_WHEELS:
        w = stage.GetPrimAtPath(f"{CARRY_VEHICLE_PATH}/{wn}")
        if not w.IsValid():
            raise RuntimeError(f"휠 없음: {CARRY_VEHICLE_PATH}/{wn}")
        centers[wn] = cache.GetLocalToWorldTransform(w).ExtractTranslation()
    # 차량이 yaw 로 돌아가면 앞/뒤 축을 가르는 세계축이 z 에서 x 로 바뀐다.
    # 길이축 방향 u=(sin yaw, 0, cos yaw) 에 투영해 앞/뒤를 가르고, 축 중심은
    # 그와 직교한 횡축 v=(cos yaw, 0, -sin yaw) 로 잡는다.
    yaw_rad = math.radians(CARRY_VEHICLE_YAW_DEG)
    u = (math.sin(yaw_rad), math.cos(yaw_rad))      # (x, z) 길이축
    v = (math.cos(yaw_rad), -math.sin(yaw_rad))     # (x, z) 횡축
    def _proj(c, a):
        return float(c[0]) * a[0] + float(c[2]) * a[1]
    front_l = (_proj(centers["FrontLeftWheel"], u) + _proj(centers["FrontRightWheel"], u)) * 0.5
    rear_l = (_proj(centers["RearLeftWheel"], u) + _proj(centers["RearRightWheel"], u)) * 0.5
    lateral_c = sum(_proj(c, v) for c in centers.values()) / 4.0
    AXLE.update(rear_l=min(front_l, rear_l), front_l=max(front_l, rear_l),
                lateral_c=lateral_c, yaw_deg=CARRY_VEHICLE_YAW_DEG,
                wheelbase=abs(front_l - rear_l))

    # 로봇 2대 도크 배치
    UsdGeom.Xform.Define(stage, "/World/Robots")
    for key, cfg in ROBOTS.items():
        pos = _dock_position(stage, cfg["dock"])
        r = stage.DefinePrim(cfg["xform"], "Xform")
        r.GetReferences().AddReference(str(ROBOT_USD))
        _place_robot_dock(UsdGeom, Gf, r, pos)
    for _ in range(30):
        app.update()
    print(f"DOCK_STAGE_READY carry={CARRY_VEHICLE}@{CARRY_VEHICLE_POS}"
          f"yaw={CARRY_VEHICLE_YAW_DEG:.0f} parked={PARKED_VEHICLE}@{PARKED_VEHICLE_POS} "
          f"axle rear_l={AXLE['rear_l']:.3f} front_l={AXLE['front_l']:.3f} "
          f"wheelbase={AXLE['wheelbase']:.3f} lateral_c={AXLE['lateral_c']:.3f} "
          f"colliders_fixed={n_fixed}", flush=True)
    print(
        f"SCENE_OPTIMIZED disabled_sensors={disabled_sensors} "
        f"render={RENDER_WIDTH}x{RENDER_HEIGHT}@{RENDER_HZ:.0f}Hz "
        f"physics={PHYSICS_HZ:.0f}Hz",
        flush=True)
    print(
        f"MOTION_SMOOTHING linear_accel={LINEAR_ACCEL:.2f}m/s2 "
        f"linear_decel={LINEAR_DECEL:.2f}m/s2 "
        f"angular_accel={ANGULAR_ACCEL:.2f}rad/s2",
        flush=True)
    return stage


def main():
    _restart_with_isaac_python()
    from isaacsim import SimulationApp
    headless = "--gui" not in sys.argv[1:]
    app = SimulationApp({
        "headless": headless,
        "width": RENDER_WIDTH,
        "height": RENDER_HEIGHT,
        "disable_viewport_updates": headless,
    })
    try:
        from isaacsim.core.utils.extensions import enable_extension
        enable_extension("isaacsim.ros2.bridge")
        for _ in range(12):
            app.update()
        import numpy as np
        import omni.timeline
        from isaacsim.core.prims import Articulation

        stage = build_stage(app)
        timeline = omni.timeline.get_timeline_interface()
        timeline.play()
        for _ in range(30):
            app.update()

        arts = {}
        for key, cfg in ROBOTS.items():
            art = Articulation(f"{cfg['xform']}/base_link")
            art.initialize()
            arts[key] = art

        sys.path.insert(0, str(WORK_DIR))
        from mecanum_drive import (WHEEL_JOINTS, configure_hub_drives, slew_twist,
                                   wheel_velocities_from_cmd_vel)
        for key, cfg in ROBOTS.items():
            configure_hub_drives(stage, f"{cfg['xform']}/joints")
        wheel_idx = {k: {w: arts[k].dof_names.index(j) for w, j in WHEEL_JOINTS.items()}
                     for k in arts}
        vel_buf = {k: np.zeros(arts[k].get_joint_positions().shape, dtype=np.float32)
                   for k in arts}
        target_twist = {k: (0.0, 0.0, 0.0) for k in arts}
        applied_twist = {k: (0.0, 0.0, 0.0) for k in arts}

        def apply_wheel_velocity(key, vx, vy, wz):
            omegas = wheel_velocities_from_cmd_vel(vx, vy, wz)
            buf = vel_buf[key]; buf[...] = 0.0
            for w, om in omegas.items():
                i = wheel_idx[key][w]
                if buf.ndim == 2:
                    buf[0, i] = om
                else:
                    buf[i] = om
            arts[key].set_joint_velocity_targets(buf)

        if "--headless-test" in sys.argv[1:]:
            from isaacsim.core.prims import RigidPrim
            def _p(a):
                return np.asarray(a.get_world_poses()[0]).reshape(-1)[:3]
            carry_rb = RigidPrim(CARRY_VEHICLE_PATH)
            parked_rb = RigidPrim(PARKED_VEHICLE_PATH)
            def _rb_p(rb):
                return np.asarray(rb.get_world_poses()[0]).reshape(-1)[:3]
            p0 = {k: _p(a) for k, a in arts.items()}
            carry0, parked0 = _rb_p(carry_rb), _rb_p(parked_rb)
            for _ in range(180):
                app.update()
            p1 = {k: _p(a) for k, a in arts.items()}
            carry1, parked1 = _rb_p(carry_rb), _rb_p(parked_rb)
            disp = {k: float(np.linalg.norm(p1[k] - p0[k])) for k in arts}
            carry_disp = float(np.linalg.norm(carry1 - carry0))
            parked_disp = float(np.linalg.norm(parked1 - parked0))
            robots_ok = all(d < 0.35 for d in disp.values())
            carry_ok = carry_disp < 0.30
            parked_ok = parked_disp < 0.30
            ok = robots_ok and carry_ok and parked_ok
            print(f"DOCK_PHYSICS_TEST={'PASS' if ok else 'FAIL'} "
                  f"robot_disp={ {k: round(v,4) for k,v in disp.items()} } "
                  f"carry_disp={carry_disp:.4f} parked_disp={parked_disp:.4f} "
                  f"carry_pos1={ [round(float(v),2) for v in carry1] } "
                  f"parked_pos1={ [round(float(v),2) for v in parked1] }", flush=True)
            app.close()
            return

        if "--drive-test" in sys.argv[1:]:
            # ROS 없이 메카넘 구동만 검증한다. v2 바닥/마찰에서 로봇이 실제로
            # 전진·횡이동하는지 확인하는 용도(--headless-test 는 정지 안정성만 본다).
            def _pos(a):
                return np.asarray(a.get_world_poses()[0]).reshape(-1)[:3]

            def run_twist(tw, steps):
                cur = {k: (0.0, 0.0, 0.0) for k in arts}
                prev = timeline.get_current_time()
                for _ in range(steps):
                    app.update()
                    now = timeline.get_current_time()
                    dt = min(0.1, max(0.0, now - prev))
                    prev = now
                    for k in arts:
                        cur[k] = slew_twist(
                            cur[k], tw, dt, linear_accel=LINEAR_ACCEL,
                            linear_decel=LINEAR_DECEL, angular_accel=ANGULAR_ACCEL)
                        apply_wheel_velocity(k, *cur[k])

            run_twist((0.0, 0.0, 0.0), 60)          # 정착
            p0 = {k: _pos(a) for k, a in arts.items()}
            run_twist((0.35, 0.0, 0.0), 180)        # 전진 3s
            run_twist((0.0, 0.0, 0.0), 60)
            p1 = {k: _pos(a) for k, a in arts.items()}
            run_twist((0.0, 0.35, 0.0), 180)        # 좌 strafe 3s
            run_twist((0.0, 0.0, 0.0), 60)
            p2 = {k: _pos(a) for k, a in arts.items()}
            fwd = {k: float(np.linalg.norm(p1[k] - p0[k])) for k in arts}
            strafe = {k: float(np.linalg.norm(p2[k] - p1[k])) for k in arts}
            ok = (all(v > 0.30 for v in fwd.values())
                  and all(v > 0.30 for v in strafe.values()))
            print(f"DRIVE_TEST={'PASS' if ok else 'FAIL'} "
                  f"forward={ {k: round(v,3) for k,v in fwd.items()} } "
                  f"strafe={ {k: round(v,3) for k,v in strafe.items()} }", flush=True)
            app.close()
            return

        if str(BRIDGE_RCLPY) not in sys.path:
            sys.path.insert(0, str(BRIDGE_RCLPY))
        import rclpy
        from geometry_msgs.msg import Twist, PoseStamped
        from nav_msgs.msg import Odometry
        from std_srvs.srv import SetBool
        from std_msgs.msg import String as RosString
        from isaacsim.core.prims import RigidPrim
        if not rclpy.ok():
            rclpy.init()
        node = rclpy.create_node("dock_lift_handoff_v2_bridge")
        veh_pub = node.create_publisher(PoseStamped, "/vehicle/pose", 10)
        veh_rb = RigidPrim(CARRY_VEHICLE_PATH)

        def make_cb(key):
            def cb(msg):
                target_twist[key] = (
                    float(msg.linear.x), float(msg.linear.y), -float(msg.angular.z))
            return cb
        odom_pub = {}
        for key in ROBOTS:
            node.create_subscription(Twist, f"/robot_{key}/cmd_vel", make_cb(key), 10)
            odom_pub[key] = node.create_publisher(Odometry, f"/robot_{key}/odom", 10)
        slots_pub = node.create_publisher(RosString, "/parking_slots", 10)

        arm_idx = {k: {n: arts[k].dof_names.index(n) for n in ARM_TARGETS} for k in arts}
        arm_cmd = {k: 0.0 for k in arts}
        arm_applied = {k: 0.0 for k in arts}

        def apply_arms(key):
            tgt, cur = arm_cmd[key], arm_applied[key]
            if abs(tgt - cur) > 1e-4:
                cur += max(-0.02, min(0.02, tgt - cur))
                arm_applied[key] = cur
            pos = np.array(arts[key].get_joint_positions(), dtype=np.float32, copy=True)
            for n, deg in ARM_TARGETS.items():
                v = math.radians(deg * cur)
                if pos.ndim == 2:
                    pos[0, arm_idx[key][n]] = v
                else:
                    pos[arm_idx[key][n]] = v
            arts[key].set_joint_position_targets(pos)

        def make_arm_cb(key):
            def cb(req, resp):
                arm_cmd[key] = 1.0 if req.data else 0.0
                resp.success = True
                resp.message = "arm target set: " + ("open" if req.data else "fold")
                return resp
            return cb
        for key in arts:
            node.create_service(SetBool, f"/robot_{key}/arm_control", make_arm_cb(key))

        print(f"DOCK_LIFT_HANDOFF_V2_READY robots=['robot_rear','robot_front'] "
              f"domain={os.environ.get('ROS_DOMAIN_ID','0')}", flush=True)
        _slot_tick = 0
        _rtf_wall = time.monotonic()
        _rtf_sim = timeline.get_current_time()
        _drive_sim = _rtf_sim
        while app.is_running():
            app.update()
            now_sim = timeline.get_current_time()
            if now_sim - _rtf_sim >= 5.0:
                now_wall = time.monotonic()
                sim_dt = now_sim - _rtf_sim
                wall_dt = now_wall - _rtf_wall
                print(
                    f"RUNTIME_RTF={sim_dt / max(wall_dt, 1e-6):.3f} "
                    f"sim_dt={sim_dt:.2f}s wall_dt={wall_dt:.2f}s",
                    flush=True)
                _rtf_sim, _rtf_wall = now_sim, now_wall
            rclpy.spin_once(node, timeout_sec=0.0)
            drive_dt = min(0.1, max(0.0, now_sim - _drive_sim))
            _drive_sim = now_sim
            for key, cfg in ROBOTS.items():
                applied_twist[key] = slew_twist(
                    applied_twist[key], target_twist[key], drive_dt,
                    linear_accel=LINEAR_ACCEL,
                    linear_decel=LINEAR_DECEL,
                    angular_accel=ANGULAR_ACCEL,
                )
                apply_wheel_velocity(key, *applied_twist[key])
                apply_arms(key)
                pos, orn = arts[key].get_world_poses()
                pos = np.asarray(pos).reshape(-1)[:3]
                orn = np.asarray(orn).reshape(-1)[:4]
                w, x, y, z = (float(v) for v in orn)
                fwd_x = 1.0 - 2.0 * (y * y + z * z)
                fwd_z = 2.0 * (x * z - w * y)
                yaw = math.atan2(-fwd_z, fwd_x)
                od = Odometry()
                od.header.stamp = node.get_clock().now().to_msg()
                od.header.frame_id = "map"
                od.child_frame_id = f"robot_{key}/base_link"
                od.pose.pose.position.x = float(pos[0])
                od.pose.pose.position.y = float(pos[1])
                od.pose.pose.position.z = float(pos[2])
                od.pose.pose.orientation.z = math.sin(yaw * 0.5)
                od.pose.pose.orientation.w = math.cos(yaw * 0.5)
                odom_pub[key].publish(od)
            veh_pos, veh_orn = veh_rb.get_world_poses()
            vp = np.asarray(veh_pos).reshape(-1)[:3]
            vq = np.asarray(veh_orn).reshape(-1)[:4]
            vw, vx, vy, vz = (float(v) for v in vq)
            vehicle_axis_x = 2.0 * (vx * vz + vw * vy)
            vehicle_axis_z = 1.0 - 2.0 * (vx * vx + vy * vy)
            vehicle_yaw = math.atan2(vehicle_axis_x, vehicle_axis_z)
            ps = PoseStamped()
            ps.header.stamp = node.get_clock().now().to_msg()
            ps.header.frame_id = "map"
            ps.pose.position.x = float(vp[0])
            ps.pose.position.y = float(vp[1])
            ps.pose.position.z = float(vp[2])
            ps.pose.orientation.z = math.sin(vehicle_yaw * 0.5)
            ps.pose.orientation.w = math.cos(vehicle_yaw * 0.5)
            veh_pub.publish(ps)
            _slot_tick += 1
            if _slot_tick % max(1, int(RENDER_HZ / 2.0)) == 0:
                positions = _vehicle_world_positions(stage)
                arr = []
                for sid, (sx, sz, yaw) in _SLOTS_V2.items():
                    occ = any(abs(px - sx) <= _HALF_WID and abs(pz - sz) <= _HALF_LEN
                              for px, pz in positions)
                    arr.append({"slot_id": sid, "occupied": occ,
                                "is_accessible": sid in _ACCESSIBLE,
                                "x": round(sx, 3), "y": round(-sz, 3), "yaw_deg": yaw})
                msg = RosString(); msg.data = json.dumps(arr); slots_pub.publish(msg)
        app.close()
    finally:
        pass


if __name__ == "__main__":
    main()
