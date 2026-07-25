#!/usr/bin/env python3
"""v4 주차장 러너 — 로봇 4대(입차팀/출차팀) + 휠 오도메트리 + probe 진입점.

기존 dock_lift_handoff_runner_v2.py(로봇 2대)는 건드리지 않는다. 검증된 2대 구성이
비교 기준으로 남아야 한다.

좌표 규약은 site_map_v4 가 전담한다(입차=z양수, 에셋 라벨과 반대).

실행: parking_v4_runner.sh [--gui] [--headless-test]
"""
import math
import os
import sys
from pathlib import Path

import numpy as np

WORK_DIR = Path(__file__).resolve().parent
REPO_ROOT = WORK_DIR.parent.parent
PARKING_USD = WORK_DIR / "parking" / "parking_environment_v4.usd"
ROBOT_USD = (WORK_DIR.parent / "hwia_parking_robot_final_caster_package"
             / "hwia_4cam_mecha_roller_lowered.usd")
ISAAC_PYTHON = Path("/home/rokey/dev_ws/isaac_sim/isaacsim/_build/linux-x86_64/release/python.sh")

# ---- Mission Phase C, Task C1: 인계장 베이 Pickup ----
# p4_depth 의 depth_stop_lift_test_dual.py 가 같은 fab_vehicles.usd 차량을 참조·배치·
# 물리완화하는 방식을 그대로 따른다(git show origin/p4_depth:isaacpjt/Isaac_envo/
# depth_stop_lift_test_dual.py). VEHICLE_ROOT 관례(/World/VehicleAsset/Vehicles/<이름>)도
# 동일하게 재사용한다.
VEHICLES_USD = WORK_DIR / "fab_vehicles.usd"
HANDOFF_VEHICLE_NAME = "Pickup"
HANDOFF_VEHICLE_ROOT = f"/World/VehicleAsset/Vehicles/{HANDOFF_VEHICLE_NAME}"
# 인계장 베이(ExitVehicleWait) 블루패드 중심 — parking_environment_v4.usd 실측:
# VehicleWaitAreas/ExitVehicleWait/Pad translate=(-8.5, 0.0001, 7.075), scale=(6.4,·,3.2).
# 에셋 라벨은 "출차"지만 site_map_v4.py 의 z 부호반전 규약상 우리 프로세스에서는
# ENTRY 팀 베이다(W_OUT 마커, id 51, 같은 좌표) — spawn_handoff_vehicle() 은 좌표만
# 쓰고 라벨/역할은 다루지 않는다.
HANDOFF_BAY_CENTER = (-8.5, 7.075)  # (x, z)
HANDOFF_VEHICLE_WHEELS = ("FrontLeftWheel", "FrontRightWheel", "RearLeftWheel", "RearRightWheel")
# v4 바닥 상판 y — parking_environment_v4.usd 의 ParkingEnvironment/Floor(scale.y=0.12,
# translate.y=-0.06 -> 상판 y=0.0)와 VehicleWaitAreas 각 Pad(translate.y≈0.0001)로 실측
# 확인. spawn_handoff_vehicle() 은 이 상수에 짐작 없이 실측 bbox 오프셋을 더한다.
V4_FLOOR_Y = 0.0
HANDOFF_SETTLE_FRAMES = 60

sys.path.insert(0, str(REPO_ROOT / "src" / "parkbot_aruco"))
from parkbot_aruco import site_map_v4 as sm   # noqa: E402

RENDER_HZ = 60.0
RENDER_WIDTH = 640
RENDER_HEIGHT = 400
PHYSICS_HZ = 120.0
LINEAR_ACCEL = 0.5
LINEAR_DECEL = 0.8
ANGULAR_ACCEL = 0.8
# 회전 오도 보정 배율(Mission Phase B 회전 버그, a869b49 에서 발견: 제자리 90도 회전이
# GT 대비 ~31° 어긋남). 진단: mecanum_drive.YAW_SCALE=1.12 는 wz~0.5 한 동작점에서만
# 실측 보정된 근사치이고, cmd_vel_from_wheel_velocities 는 그 IK 의 정확한 최소자승
# 역이라 "명령된" 회전율을 그대로 돌려줄 뿐 실제 미끄러짐은 반영하지 않는다 — 그래서
# 오도가 회전을 실제보다 크게 믿는다(진단은 맞았다). 그러나 이 값은 물리 상수 하나로
# 깔끔히 보정되지 않았다 — 아래는 실측 과정 요약(전체는 taskBROT-report.md):
#
# 1) 폐루프(--probe=ROTCHK, entry_lead, n_fix=0 순수오도) 기준선 실측: d_filt_deg=
#    -89.51 대 d_gt_deg=-58.31 -> scale=0.6514. a869b49 MISSIONB_ROT(entry_follow,
#    같은 지오메트리) err_yaw_gt=31.39 에서 역산한 값(≈0.6512)과 0.0002 차이로
#    독립 수렴 — 진단 자체의 신뢰도는 높다.
# 2) 그런데 0.6514 를 predict_body 에 그대로 적용하면 ROTCHK 가 **더 나빠졌다**
#    (gt_err_deg 31.69 -> 162.57, 로봇이 -252.57° 까지 과회전). 원인: 이 보정은
#    폐루프 **안에서** filt 의 수렴 속도(=P 제어기가 명령을 얼마나 오래 유지하는지)
#    를 바꾸는데, 이 로봇의 실제 제자리회전은 "명령 지속시간"에 따라 물리적 회전율이
#    비선형·비단조로 변한다(--probe=YAWCAL 로 열린루프 정속회전을 따로 재보니 부호까지
#    반대인 전혀 다른 값이 나왔다 -2.5~-3.3 — 원래 YAW_SCALE=1.12 를 보정한
#    mecanum_holonomic_test.py 는 지금과 다른 로봇 에셋(hwia_parking_robot_final_
#    caster.usd, 현재는 hwia_4cam_mecha_roller_lowered.usd)으로 측정된 것이라 이
#    에셋에서 재검증된 적이 없었다 — report 참조). 즉 "단일 물리 상수" 가정이 이
#    폐루프에서는 성립하지 않는다.
# 3) 그래서 scale 자체를 폐루프 ROTCHK 로 직접 스윕(0.65/0.85/0.95/1.0/1.10/1.11/
#    1.12/1.15)해 gt_err_deg 최소 지점을 찾았다: 1.10 에서 gt_err_deg=3.45°(재실행시
#    steps=299,d_gt_deg=-86.55 로 정확히 동일 — 결정적/재현 가능 확인). 31.69° 대비
#    91% 감소. 이 값은 "물리적 슬립비"가 아니라 이 컨트롤러·래치 구조에 대해 경험적
#    으로 맞춘 값이다(한계는 report 참조 — 일반화 검증 안 됨, 노이즈 바닥 ~1°).
# 4) Task 3b-harden Item1(두 번째 로봇의 180° 재정렬 대비, taskBharden-report.md):
#    위 3)의 한계("일반화 검증 안 됨")를 --probe=ROTCHK180(spawn yaw 90->target -90,
#    90°의 2배 지속시간)으로 직접 확인했다 — 1.10 에서 gt_err_deg=12.61°(d_filt_deg=
#    -179.52 대 d_gt_deg=-167.39: filt 가 GT 보다 앞서 나가 "도달"로 판단했지만 실제
#    로는 덜 돈 상태, 즉 물리적 언더슈트)로 ~4° 목표의 3배 이상 벗어났다 — 90°에서
#    맞춘 상수가 180°로 그대로 일반화되지 않았다(3)이 우려한 비선형·구동시간 의존이
#    실측으로 확인됨). 각도별 스케일 같은 새 메커니즘을 만들기 전에 "두 각도를 동시에
#    만족하는 단일 상수가 있는가"부터 직접 스윕했다(90°·180° 둘 다 재측정):
#      scale=1.05: 90°=13.58°, 180°=31.89° (1.10 보다 둘 다 나빠짐 — 기각)
#      scale=1.10: 90°=3.45°,  180°=12.61° (기존값 — 180° 미달)
#      scale=1.12: 90°=4.20°,  180°=2.23°  (**둘 다 ~4° 이내 — 채택**)
#      scale=1.13: 90°=6.48°,  180°=11.84° (1.12 보다 둘 다 나빠짐 — 기각)
#    1.11/1.13 모두 1.12 한쪽 또는 양쪽보다 못해(위 3의 "1.11 이 1.10 과 1.12 둘 다보다
#    나쁨"과 같은 종류의 비단조 민감성), 1.12 가 우연한 단일 샘플이 아니라 두 이웃값에
#    둘러싸인 실제 국소 최적점임을 확인했다. 그래서 1.10 -> 1.12 로 교체한다(90°는
#    3.45->4.20 로 소폭 후퇴하지만 여전히 목표 이내, 180°는 12.61->2.23 로 크게 개선).
#    주의: 이 창은 매우 좁다 — 여전히 "물리 상수"가 아니라 90°·180° 둘을 함께 만족하는
#    폐루프 경험값이며, 검증되지 않은 다른 각도(예: 45°/135°)로 그대로 확장된다는
#    보장은 없다(그 경우 이 두 probe 로 재검증 없이 이 상수에 의존하지 말 것).
# 5) taskBYAW(2026-07-25): 위 1.12 는 애초에 "결함이 있는 회전 기구학"을 폐루프
#    안에서 억지로 상쇄하던 보정값이었다 — 근본 원인은 이 파일이 아니라
#    mecanum_drive.py 의 SIGN_YAW/YAW_SCALE 이었다: cmd_vel_from_wheel_velocities 는
#    IK 의 정확한 최소자승 역이라 오도가 "명령된" 회전율을 그대로 돌려주는데, 그
#    IK(SIGN_YAW=+1.0, YAW_SCALE=1.12)가 명령하는 물리적 회전은 실측(--probe=YAWSTEP,
#    고정 지속시간 열린루프, mecanum_drive.py 주석 참조) 결과 명령의 ~2.5~2.95배·반대
#    방향이었다(90도 명령이 물리적으로 ~-263도 — 사용자가 GUI 로 본 "270도 반대방향"과
#    일치). 이 YAW_ODOM_SCALE=1.12 는 3)/4)에서 폐루프 스윕으로 찾은 값인데, 우연히
#    "90도·180도 두 각도에서만" 그 3배 결함을 부분적으로 가리는 방향으로 작동했을 뿐
#    45도 같은 다른 각도에서는 그대로 파탄났을 것이다(끝점이 mod 360 으로 우연히
#    맞아떨어지는 것과 같은 종류의 함정). mecanum_drive.py 에서 SIGN_YAW/YAW_SCALE 을
#    실측대로 재보정한 뒤에는 cmd_vel_from_wheel_velocities(IK 의 정확한 역)가 훨씬
#    더 물리와 잘 맞는다(부호·자릿수 모두 정상화) — 그러나 완전히 1:1 은 아니다.
#    브리프 원안은 여기서 YAW_ODOM_SCALE=1.0(무보정)을 지시했지만, **실측으로
#    검증한 결과 1.0 은 틀렸다**: --probe=ROTCHK(90도)가 gt_err_deg=20.46°(목표 ≤4°
#    한참 초과, steps=2000=max_steps 소진 — 수렴 못 함)를 냈고, 결정적으로
#    --mission=B 가 2/2 회 모두 ok=False 로 실패했다(entry_follow 가 마커를 단 한
#    번도 못 잡음: n_fix=0, err_pos_gt 0.57~0.64m — "몇 cm" 목표의 20배 이상, 도크
#    체크·XN정렬 안무 전체가 카메라 시야 밖으로 새 나간 것으로 보인다). 원인:
#    cmd_vel_from_wheel_velocities 가 복원하는 값은 "명령된(=휠이 미끄러지지 않았다고
#    가정한) 회전율"이지 물리적 GT 회전율이 아니다 — SIGN_YAW/YAW_SCALE 을 고쳐도
#    롤러-지면 슬립 자체는 여전히 존재해서(YAW_SCALE 정의부 주석 참조, "roller-slip
#    dominated") 명령 대비 물리는 여전히 소폭(실측 +13~+20%) 더 돈다. 그래서
#    YAW_ODOM_SCALE 을 1.0 이 아니라, **같은 --probe=YAWSTEP 실측치의
#    k_gt_over_odom 평균**(1.1953, 1.1885, 1.1332, 1.1538 → avg=1.1677 — cmd 기준이
#    아니라 odom 기준 비율이라는 점에 주의, mecanum_drive.py 주석의 k_gt_over_cmd
#    와는 다른 숫자다)으로 설정한다. 이 값은 3)/4)의 1.12 처럼 폐루프를 직접
#    스윕해서 끼워맞춘 게 아니라, mecanum_drive.py 수정과 동일한 열린루프 실측
#    데이터에서 그대로 가져온 값이다 — 재검증: ROTCHK 5.45°(20.46->), ROTCHK180
#    10.94°(42.57->, 참고: 180도는 시작점에서 정확히 반대편이라 +180/-180 둘 다
#    "정답" 경로라 스윕각 부호 자체는 진단적 의미가 적다), ROTCHK45 1.93°(목표
#    이내), --mission=B 2/2 ok=True(entry_lead err_pos_gt 0.15m, entry_follow
#    0.01m, n_fix 234/238 — 실측치는 taskBYAW-report.md 참조). 상수 자체는 지우지
#    않고 남겨둔다: 이 보정 지점(seam)이 앞으로도 남아있어야, 만약 다른 원인(예: 새
#    로봇 에셋)으로 오도-물리 불일치가 다시 생기면 같은 자리에서 --probe=YAWSTEP/
#    ROTCHK/ROTCHK180/ROTCHK45 로 재보정할 수 있다.
YAW_ODOM_SCALE = 1.1677
ROBOT_SPAWN_Y = 0.06
# probe B(휠 오도메트리 드리프트) 측정 직전 정착(settle) 프레임 수.
# 드리프트는 초기 settle 정도에 매우 민감하다. 이 값을 명시적으로 고정하지
# 않으면 측정과 무관한 다른 코드 변경(예: 카메라 부착 루프의 app.update()
# 횟수)이 우연히 settle 정도를 바꿔 결과가 재현 불가능해진다.
PROBE_SETTLE_FRAMES = 120


def _restart_with_isaac_python():
    if os.environ.get("CARB_APP_PATH"):
        return
    os.execv(str(ISAAC_PYTHON), [str(ISAAC_PYTHON), str(Path(__file__).resolve()), *sys.argv[1:]])


def read_markers(stage):
    """v4 스테이지의 aruco:* 속성을 읽어 serves -> 정보 dict 로 돌려준다.

    좌표를 손으로 옮겨 적지 않기 위한 것이다. 에셋이 바뀌면 값이 따라온다.
    """
    out = {}
    for prim in stage.Traverse():
        a = prim.GetAttribute("aruco:markerId")
        if not a or not a.IsValid():
            continue
        pos = prim.GetAttribute("aruco:position").Get()
        serves = prim.GetAttribute("aruco:serves").Get()
        yaw_attr = prim.GetAttribute("aruco:yaw")
        kind_attr = prim.GetAttribute("aruco:kind")
        out[str(serves)] = {
            "id": int(a.Get()),
            "x": float(pos[0]),
            "z": float(pos[2]),
            "yaw": float(yaw_attr.Get()) if yaw_attr and yaw_attr.IsValid() else 0.0,
            "kind": str(kind_attr.Get()) if kind_attr and kind_attr.IsValid() else "",
        }
    return out


def marker_visual_center(stage, serves, time_code=None):
    """실제로 렌더되는 마커 데칼 메시의 월드 중심(x, z)을 돌려준다.

    Probe A 조사로 확인: 도크 마커(D_OUT_*/D_IN_*)는 `aruco:position` 속성값과
    데칼 메시의 실제 월드 위치가 z 로 0.7m 어긋난다(속성=도크/스폰 좌표,
    데칼=차선 쪽으로 밀린 실제 그림 위치). x 는 항상 일치했다. 카메라가
    "실제로 보는" 대상은 그림이므로 자세 계측에는 이 함수를 쓰고,
    `read_markers()`(속성 기반)는 기존 용도(로봇 스폰 좌표 등) 그대로 둔다.
    """
    from pxr import Gf, Usd, UsdGeom
    tc = time_code if time_code is not None else Usd.TimeCode.Default()
    for prim in stage.Traverse():
        sv = prim.GetAttribute("aruco:serves")
        if sv and sv.IsValid() and str(sv.Get()) == serves:
            xf = UsdGeom.Xformable(prim)
            wc = xf.ComputeLocalToWorldTransform(tc).Transform(Gf.Vec3d(0, 0, 0))
            return float(wc[0]), float(wc[2])
    raise RuntimeError(f"marker_visual_center: serves={serves!r} 데칼 메시를 찾지 못함")


def _apply_physics(stage):
    from pxr import PhysxSchema
    sc = stage.GetPrimAtPath("/World/PhysicsScene")
    if not sc or not sc.IsValid():
        raise RuntimeError("v4 PhysicsScene 없음")
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


def _disable_sensors(stage):
    """천장 RTX 라이다는 이 작업에 불필요하고 무겁다. 원본은 수정하지 않는다."""
    sensors = stage.GetPrimAtPath("/World/Sensors")
    if sensors and sensors.IsValid():
        n = sum("Lidar" in c.GetName() for c in sensors.GetChildren())
        sensors.SetActive(False)
        return n
    return 0


def robot_prim_path(robot_id):
    return f"/World/Robots/{robot_id}"


def spawn_handoff_vehicle(stage):
    """인계장 베이 블루패드에 Pickup 을 참조 스폰한다 (Mission Phase C, Task C1).

    참조·배치·물리완화 방식은 p4_depth 의 depth_stop_lift_test_dual.py 를 그대로
    따른다(git show origin/p4_depth:isaacpjt/Isaac_envo/depth_stop_lift_test_dual.py):
    fab_vehicles.usd 를 통째로 참조해 목표 차종만 남기고 나머지는 비활성화하고,
    fab_vehicles.usd 자체의 프리뷰용 부속물(조명/지면/독립 물리씬)은 끈다. FAB 차량의
    휠은 Cylinder 콜라이더인데 PhysX 근사가 나빠 정지 상태에서도 지터한다고 그 파일이
    보고하고 있어(--sphere-wheels 플래그 주석), PhysX Vehicle 구동계(엔진/기어/서스펜션
    API)를 통째로 제거하고 휠 콜라이더를 Sphere 로 바꾼다 — 그 파일이 기본 실행에서
    쓰는 조합과 동일(구동계 제거는 항상, sphere-wheels 는 플래그 없이도 항상 적용:
    우리 차량은 절대 스스로 구르지 않고 그냥 서 있기만 하면 되므로 더 보수적으로 간다).
    """
    from pxr import Gf, PhysxSchema, Usd, UsdGeom, UsdPhysics, UsdShade

    if not VEHICLES_USD.is_file():
        raise RuntimeError(f"차량 에셋 없음: {VEHICLES_USD}")

    vehicle_asset = stage.GetPrimAtPath("/World/VehicleAsset")
    if not vehicle_asset.IsValid():
        vehicle_asset = UsdGeom.Xform.Define(stage, "/World/VehicleAsset").GetPrim()
    vehicle_asset.GetReferences().AddReference(str(VEHICLES_USD))

    # fab_vehicles.usd 자체의 프리뷰용 부속물 — v4 스테이지의 PhysicsScene/조명과
    # 충돌하므로 끈다(depth_stop_lift_test_dual.py 의 deactivate 목록과 동일).
    for name in ("PhysicsScene", "DriveGround", "FabLighting", "Cylinder001"):
        p = stage.GetPrimAtPath(f"/World/VehicleAsset/{name}")
        if p.IsValid():
            p.SetActive(False)

    vehicles = stage.GetPrimAtPath("/World/VehicleAsset/Vehicles")
    if not vehicles.IsValid():
        raise RuntimeError("/World/VehicleAsset/Vehicles 를 찾지 못했습니다")
    for vehicle in vehicles.GetChildren():
        if vehicle.GetName() != HANDOFF_VEHICLE_NAME:
            vehicle.SetActive(False)

    target = stage.GetPrimAtPath(HANDOFF_VEHICLE_ROOT)
    if not target.IsValid():
        raise RuntimeError(f"차량을 찾지 못했습니다: {HANDOFF_VEHICLE_ROOT}")

    # 차종마다 축거가 달라 앞/뒤축 로컬 Z 를 실제 휠 좌표에서 뽑는다(하드코딩 금지 —
    # depth_stop_lift_test_dual.py 관례와 동일). 아직 이동 전 local translate 값이므로
    # 참조 원본 자세 기준이다.
    wheel_local = {}
    for wn in HANDOFF_VEHICLE_WHEELS:
        w = stage.GetPrimAtPath(f"{HANDOFF_VEHICLE_ROOT}/{wn}")
        if not w.IsValid():
            raise RuntimeError(f"휠을 찾지 못했습니다: {HANDOFF_VEHICLE_ROOT}/{wn}")
        wheel_local[wn] = UsdGeom.Xformable(w).GetLocalTransformation().ExtractTranslation()
    front_local_z = (wheel_local["FrontLeftWheel"][2] + wheel_local["FrontRightWheel"][2]) * 0.5
    rear_local_z = (wheel_local["RearLeftWheel"][2] + wheel_local["RearRightWheel"][2]) * 0.5

    # 참조 원본(무회전)은 로컬 Z 가 세계 Z 와 그대로 겹친다(depth_stop_lift_test_dual.py
    # 는 이 차량을 회전 없이 translate 만으로 배치했고 그때 차 길이가 world Z 를 따랐다
    # — 로봇 에셋과 달리 이 차량 에셋은 Z-up->Y-up 축변환이 필요 없다). 길이축을 X로
    # 돌리려면 Y축 ±90°: world_x = local_x*cosθ + local_z*sinθ 이므로(축 중심의
    # local_x≈0 가정) front_world_x - rear_world_x ≈ (front_local_z-rear_local_z)*sinθ.
    # 앞축이 게이트쪽(-x, 먼 쪽)에 오려면 이 값이 음수여야 한다 — 그래서 두 로컬 Z 의
    # 대소로 회전 부호를 정한다(차종이 바뀌어도 축 좌표만 보면 되므로 하드코딩 아님).
    yaw_deg = -90.0 if front_local_z > rear_local_z else 90.0

    # 주의: target(Pickup) 자신의 로컬 트랜스폼은 build_fab_vehicles.py 가 이미
    # "FBX 로컬축(X=좌우,Y=전후,Z=위) -> PhysX/Isaac 축(X=좌우,Y=위,Z=전후)" 정렬 회전을
    # 구워 넣은 것이다(vehicle_root_world = yaw_basis * fbx_to_vehicle_basis *
    # body_world — 위 build_fab_vehicles.py 참고). depth_stop_lift_test_dual.py 는 이
    # 회전을 절대 지우지 않고 translate 성분만 GetLocalTransformation().SetTranslate()
    # 로 바꿔치기해 재사용한다(_replace_matrix_xform). 여기서도 그 관례를 그대로
    # 따른다: target 자체는 회전을 보존하고 translate 만 (0,0,0)으로 바꿔 VehicleAsset
    # (래퍼) 원점에 "정렬된 채"(길이=로컬Z, 좌우=로컬X, 높이=로컬Y — 아래 실측으로
    # 확인) 두고, 내가 필요한 추가 Y축 회전은 VehicleAsset 래퍼 쪽에 얹는다 — 이
    # 파일이 로봇을 놓을 때 이미 쓰는 것과 똑같은 "AddTranslateOp 후 AddRotateOp"
    # 관례(Z-up 에셋 -> Y-up 스테이지 변환에 씀)를 그대로 재사용하므로 새 행렬 곱
    # 규약을 만들지 않는다. (디버깅 메모: target 의 회전을 통째로 지우고 새
    # translate+rotateY 를 바로 얹는 첫 시도도 동일한 bbox 를 냈다 — 바디/휠 등
    # 자식 prim 들이 이미 자기 자신의 xformOp 로 "정렬된 로컬 프레임"을 갖고 있어서,
    # target 루트에 어떤 강체변환을 얹어도 결과가 같았다. 그래도 이 방식을 유지하는
    # 이유는 p4_depth 가 검증한 패턴을 그대로 재사용해 회귀 위험을 줄이기 위해서다.)
    target_xf = UsdGeom.Xformable(target)
    target_matrix = target_xf.GetLocalTransformation()
    target_matrix.SetTranslate(Gf.Vec3d(0.0, 0.0, 0.0))
    target_xf.ClearXformOpOrder()
    target_xf.MakeMatrixXform().Set(target_matrix)

    cx, cz = HANDOFF_BAY_CENTER
    va_xf = UsdGeom.Xformable(vehicle_asset)
    va_xf.ClearXformOpOrder()
    translate_op = va_xf.AddTranslateOp()
    translate_op.Set(Gf.Vec3d(cx, 0.0, cz))
    va_xf.AddRotateYOp().Set(yaw_deg)

    # 바닥에 닿게: 일단 y=0 으로 놓고 실측 bbox 최저점으로 오프셋을 구한다(짐작 금지 —
    # taskC1-brief.md 요구사항). V4_FLOOR_Y 는 parking_environment_v4.usd Floor/Pad
    # 상판을 실측한 값(=0.0)이고, 여기서는 차량 자체의 최저점만 bbox 로 구한다.
    bbox_cache = UsdGeom.BBoxCache(
        Usd.TimeCode.Default(),
        [UsdGeom.Tokens.default_, UsdGeom.Tokens.render, UsdGeom.Tokens.proxy],
        useExtentsHint=True,
    )
    world_range = bbox_cache.ComputeWorldBound(target).ComputeAlignedRange()
    if world_range.IsEmpty():
        raise RuntimeError(f"{HANDOFF_VEHICLE_ROOT} bbox 계산 실패(빈 범위)")
    y_min = world_range.GetMin()[1]
    y_offset = V4_FLOOR_Y - y_min
    translate_op.Set(Gf.Vec3d(cx, y_offset, cz))

    # 최종 배치 후 실측 bbox 로 배향을 증명한다: len_x≈5.83(길이, taskC1-brief.md
    # 실측치와 일치). wid_z 는 이 스폰 시점에 처음 실측한 값(≈2.33)이 나와야 한다 —
    # brief 의 "width 1.910/height(third dim) 2.327" 라벨은 실측(휠 world y=0.423 이
    # 타이어 반경과 정확히 일치 -> Y 가 확실히 위쪽축)과 대조해보면 서로 뒤바뀌어
    # 있었다(진짜 폭=2.327, 진짜 높이=1.910). len_x/wid_z 토큰은 "길이=X, 나머지
    # 수평축=Z, 높이=Y(안 찍음, 회전에 안 바뀜)"만 증명하면 되므로 이 라벨 오류는
    # 배치 정확성에 영향 없다 — 패드 z 한도(3.2m) 안에 2.327 이든 1.910 이든 다 들어간다.
    bbox_cache.Clear()
    world_range = bbox_cache.ComputeWorldBound(target).ComputeAlignedRange()
    size = world_range.GetSize()
    len_x, wid_z = float(size[0]), float(size[2])

    # 진단용: 앞축이 정말 게이트쪽(-x)에 있는지 실측 세계좌표로 직접 확인한다(부호
    # 유도만 믿지 않는다 — taskC1-brief.md 정직성 요구사항). V4_VEHICLE 토큰 형식은
    # 고정이므로 별도 줄로 출력한다.
    time_code = Usd.TimeCode.Default()
    front_world_x = 0.5 * (
        UsdGeom.Xformable(stage.GetPrimAtPath(f"{HANDOFF_VEHICLE_ROOT}/FrontLeftWheel"))
        .ComputeLocalToWorldTransform(time_code).ExtractTranslation()[0]
        + UsdGeom.Xformable(stage.GetPrimAtPath(f"{HANDOFF_VEHICLE_ROOT}/FrontRightWheel"))
        .ComputeLocalToWorldTransform(time_code).ExtractTranslation()[0]
    )
    rear_world_x = 0.5 * (
        UsdGeom.Xformable(stage.GetPrimAtPath(f"{HANDOFF_VEHICLE_ROOT}/RearLeftWheel"))
        .ComputeLocalToWorldTransform(time_code).ExtractTranslation()[0]
        + UsdGeom.Xformable(stage.GetPrimAtPath(f"{HANDOFF_VEHICLE_ROOT}/RearRightWheel"))
        .ComputeLocalToWorldTransform(time_code).ExtractTranslation()[0]
    )
    print(f"V4_VEHICLE_AXLES front_world_x={front_world_x:.3f} rear_world_x={rear_world_x:.3f} "
          f"(front should be < rear, gate side -x)", flush=True)

    # ---- 물리 완화(p4_depth 참고, "Key facts"·"reference implementation" 지시대로) ----
    vehicle_single_apis = (
        PhysxSchema.PhysxVehicleAPI,
        PhysxSchema.PhysxVehicleDriveStandardAPI,
        PhysxSchema.PhysxVehicleEngineAPI,
        PhysxSchema.PhysxVehicleGearsAPI,
        PhysxSchema.PhysxVehicleAutoGearBoxAPI,
        PhysxSchema.PhysxVehicleClutchAPI,
        PhysxSchema.PhysxVehicleControllerAPI,
        PhysxSchema.PhysxVehicleAckermannSteeringAPI,
        PhysxSchema.PhysxVehicleMultiWheelDifferentialAPI,
    )
    wheel_apis = (
        PhysxSchema.PhysxVehicleWheelAttachmentAPI,
        PhysxSchema.PhysxVehicleWheelAPI,
        PhysxSchema.PhysxVehicleTireAPI,
        PhysxSchema.PhysxVehicleSuspensionAPI,
        PhysxSchema.PhysxVehicleSuspensionComplianceAPI,
    )
    for api_schema in vehicle_single_apis:
        if target.HasAPI(api_schema):
            target.RemoveAPI(api_schema)
    for instance_name in (PhysxSchema.Tokens.brakes0, PhysxSchema.Tokens.brakes1):
        if target.HasAPI(PhysxSchema.PhysxVehicleBrakesAPI, instance_name):
            target.RemoveAPI(PhysxSchema.PhysxVehicleBrakesAPI, instance_name)
    for wheel_name in HANDOFF_VEHICLE_WHEELS:
        wheel = stage.GetPrimAtPath(f"{HANDOFF_VEHICLE_ROOT}/{wheel_name}")
        for api_schema in wheel_apis:
            if wheel.HasAPI(api_schema):
                wheel.RemoveAPI(api_schema)

    # 휠 Cylinder 콜라이더 -> Sphere (--sphere-wheels 와 동일 완화, 항상 적용).
    sphere_radius = None
    for wheel_name in HANDOFF_VEHICLE_WHEELS:
        wheel_path = f"{HANDOFF_VEHICLE_ROOT}/{wheel_name}"
        cylinder = stage.GetPrimAtPath(f"{wheel_path}/Collision")
        if not cylinder.IsValid():
            raise RuntimeError(f"휠 충돌체를 찾지 못했습니다: {wheel_path}/Collision")
        sphere_radius = float(UsdGeom.Cylinder(cylinder).GetRadiusAttr().Get())
        cylinder.SetActive(False)
        sphere = UsdGeom.Sphere.Define(stage, f"{wheel_path}/CollisionSphere")
        sphere.CreateRadiusAttr(sphere_radius)
        sphere.CreatePurposeAttr(UsdGeom.Tokens.guide)
        UsdPhysics.CollisionAPI.Apply(sphere.GetPrim())

    vehicle_rigid = PhysxSchema.PhysxRigidBodyAPI.Apply(target)
    vehicle_rigid.GetDisableGravityAttr().Set(False)
    vehicle_rigid.CreateEnableCCDAttr(True)
    vehicle_rigid.GetSolverPositionIterationCountAttr().Set(16)
    vehicle_rigid.GetSolverVelocityIterationCountAttr().Set(8)

    print(f"V4_VEHICLE name={HANDOFF_VEHICLE_NAME} pos=({cx},{cz}) yaw={yaw_deg:.1f} "
          f"len_x={len_x:.3f} wid_z={wid_z:.3f}", flush=True)
    return HANDOFF_VEHICLE_ROOT


def build_stage(app):
    from pxr import Gf, UsdGeom
    import omni.usd

    if not PARKING_USD.is_file():
        avail = sorted(p.name for p in PARKING_USD.parent.glob("parking_environment*.usd"))
        raise RuntimeError(f"주차장 에셋 없음: {PARKING_USD}\n  있는 것: {avail}")

    ctx = omni.usd.get_context()
    ctx.new_stage()
    stage = ctx.get_stage()
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.y)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    stage.SetTimeCodesPerSecond(RENDER_HZ)
    stage.GetRootLayer().subLayerPaths.append(str(PARKING_USD))
    world = stage.GetPrimAtPath("/World")
    if not world or not world.IsValid():
        raise RuntimeError(f"{PARKING_USD.name} 에서 /World 를 찾지 못했습니다.")
    stage.SetDefaultPrim(world)
    _apply_physics(stage)
    for _ in range(30):
        app.update()

    n_lidar = _disable_sensors(stage)

    markers = read_markers(stage)
    problems = sm.validate_markers(
        [{"serves": s, "z": m["z"]} for s, m in markers.items()])
    if problems:
        raise RuntimeError("마커 좌표 규약 위반:\n  " + "\n  ".join(problems))
    print(f"V4_MARKERS_OK count={len(markers)}", flush=True)

    UsdGeom.Xform.Define(stage, "/World/Robots")
    for robot_id in sm.ROBOTS:
        dock_serves = sm.ROBOT_DOCK_MARKER[robot_id]
        if dock_serves not in markers:
            raise RuntimeError(f"도크 마커 {dock_serves} 를 v4 에서 찾지 못함")
        m = markers[dock_serves]
        prim = stage.DefinePrim(robot_prim_path(robot_id), "Xform")
        prim.GetReferences().AddReference(str(ROBOT_USD))
        xf = UsdGeom.Xformable(prim)
        xf.ClearXformOpOrder()
        xf.AddTranslateOp().Set(Gf.Vec3d(m["x"], ROBOT_SPAWN_Y, m["z"]))
        xf.AddRotateXOp().Set(-90.0)          # Z-up 에셋 -> Y-up 스테이지
    for _ in range(30):
        app.update()

    # Mission Phase C, Task C1: 인계장 베이에 Pickup 을 얹는다. 로봇 도크/경로(x≈
    # -3.2~-1.2, XN x≈-2.5) 와 베이(x=-8.5±3.2 -> -11.7~-5.3) 사이 x 방향 여유가
    # ≥1.1m 있어 --mission=B 경로를 막지 않을 것으로 보이나, 아래 회귀 실행으로
    # 반드시 확인한다(짐작 금지 — taskC1-brief.md).
    spawn_handoff_vehicle(stage)
    for _ in range(30):
        app.update()

    placed = {r: sm.ROBOT_DOCK_MARKER[r] for r in sm.ROBOTS}
    print(f"V4_STAGE_READY robots={placed} disabled_lidar={n_lidar} "
          f"render={RENDER_WIDTH}x{RENDER_HEIGHT}@{RENDER_HZ:.0f}Hz "
          f"physics={PHYSICS_HZ:.0f}Hz", flush=True)

    from pxr import Usd
    _r0 = sm.ROBOTS[0]
    _cams = [p for p in Usd.PrimRange(stage.GetPrimAtPath(robot_prim_path(_r0)))
             if p.GetTypeName() == "Camera"]
    _has_qr = any("qr_down" in str(p.GetPath()).lower() for p in _cams)
    print(f"V4_CAMERAS_COUNT robot={_r0} n={len(_cams)} qr_down={_has_qr}", flush=True)
    return stage


def find_front_camera(stage, robot_id):
    """로봇 서브트리에서 전방 카메라 prim 경로를 찾는다.

    에셋에 카메라가 4대 있으므로 이름으로 전방을 고른다. 후보가 없으면
    조용히 넘어가지 않고 실패한다.
    """
    from pxr import Usd
    root = stage.GetPrimAtPath(robot_prim_path(robot_id))
    cams = [p for p in Usd.PrimRange(root)
            if p.GetTypeName() == "Camera"]
    if not cams:
        raise RuntimeError(f"{robot_id}: 카메라 prim 을 찾지 못했습니다")
    for p in cams:
        if "front" in p.GetName().lower():
            return str(p.GetPath())
    return str(cams[0].GetPath())


def find_rear_camera(stage, robot_id):
    """로봇 서브트리에서 후방 카메라 prim 경로를 찾는다(이름/경로에 'rear')."""
    from pxr import Usd
    root = stage.GetPrimAtPath(robot_prim_path(robot_id))
    cams = [p for p in Usd.PrimRange(root) if p.GetTypeName() == "Camera"]
    for p in cams:
        if "rear" in str(p.GetPath()).lower():
            return str(p.GetPath())
    raise RuntimeError(f"{robot_id}: 후방 카메라 prim 을 찾지 못했습니다")


def find_side_camera(stage, robot_id, side="left"):
    """로봇 서브트리에서 측면(side) 카메라 prim 경로를 찾는다.

    find_front_camera/find_rear_camera 와 같은 패턴: 경로에 "side_<side>"
    (예: cam_side_left_link/depth_cam_left/Camera_Pseudo_Depth_Left)가 있는
    카메라를 고른다. 못 찾으면 조용히 넘어가지 않고 실패한다.
    """
    from pxr import Usd
    root = stage.GetPrimAtPath(robot_prim_path(robot_id))
    cams = [p for p in Usd.PrimRange(root) if p.GetTypeName() == "Camera"]
    needle = f"side_{side}".lower()
    for p in cams:
        if needle in str(p.GetPath()).lower():
            return str(p.GetPath())
    raise RuntimeError(f"{robot_id}: {side} 측면 카메라 prim 을 찾지 못했습니다")


def attach_camera_graph(robot_id, cam_path, role="front", width=640, height=480):
    """C++ OmniGraph 로 image_raw + camera_info 를 발행한다.

    Python rclpy 로 이미지를 퍼블리시하면 Isaac 루프가 죽는다(ARUCO_PLAN 전제).
    camera_info 는 ROS2CameraHelper 의 type 이 아니라 별도 ROS2CameraInfoHelper 노드다
    (DEBUG_LOG 2026-07-21). 노드 타입·속성명은 설치된 Isaac Sim 5.1
    (isaacsim.core.nodes / isaacsim.ros2.bridge 의 .ogn 문서)로 확인했다.

    role 로 네임스페이스/그래프 경로를 분리해 같은 로봇의 전방·후방 카메라를
    동시에(별개 토픽/그래프로) 발행할 수 있다(기본 "front").
    """
    import omni.graph.core as og
    ns = f"/robot_{robot_id}/{role}"
    og.Controller.edit(
        {"graph_path": f"/Graphs/cam_{robot_id}_{role}", "evaluator_name": "push"},
        {
            og.Controller.Keys.CREATE_NODES: [
                ("tick", "omni.graph.action.OnPlaybackTick"),
                ("render", "isaacsim.core.nodes.IsaacCreateRenderProduct"),
                ("rgb", "isaacsim.ros2.bridge.ROS2CameraHelper"),
                ("info", "isaacsim.ros2.bridge.ROS2CameraInfoHelper"),
            ],
            og.Controller.Keys.CONNECT: [
                ("tick.outputs:tick", "render.inputs:execIn"),
                ("render.outputs:execOut", "rgb.inputs:execIn"),
                ("render.outputs:execOut", "info.inputs:execIn"),
                ("render.outputs:renderProductPath", "rgb.inputs:renderProductPath"),
                ("render.outputs:renderProductPath", "info.inputs:renderProductPath"),
            ],
            og.Controller.Keys.SET_VALUES: [
                ("render.inputs:cameraPrim", [cam_path]),
                ("render.inputs:width", width),
                ("render.inputs:height", height),
                ("rgb.inputs:type", "rgb"),
                ("rgb.inputs:topicName", f"{ns}/image_raw"),
                ("rgb.inputs:frameId", f"robot_{robot_id}/{role}_cam"),
                ("info.inputs:topicName", f"{ns}/camera_info"),
                ("info.inputs:frameId", f"robot_{robot_id}/{role}_cam"),
            ],
        },
    )


# ---- M5/FUSE 공용 헬퍼 (카메라 셋업·보정·검출·측위) ----
# M5 분기가 인라인으로 갖고 있던 로직을 모듈 함수로 추출한 것이다. --probe=FUSE 가
# 주행 중에 같은 셋업/보정/검출/측위를 재사용한다. 계산·값·순서는 M5 인라인과 동일.
def fuse_camera_setup(stage, timeline, app, target, cam_h, cam_role="front"):
    """카메라 높이 오버라이드 + rgb annotator + K/detector/marker_map 컨텍스트.

    cam_role: "front"(기본, 하위호환) | "rear" — 어느 카메라에 붙을지 선택한다.
    mx/mz/ref_id 는 cam_role 과 무관하게 항상 target 의 도크 마커 기준이다(호출부가
    다른 마커를 검출하려면 반환된 ctx["ref_id"] 를 직접 바꿔치기한다 — 예:
    --probe=REARXN).
    """
    import json
    from pxr import Gf, UsdGeom
    import omni.replicator.core as rep
    sys.path.insert(0, str(REPO_ROOT / "src" / "parkbot_aruco"))
    from parkbot_aruco import aruco_pose
    from parkbot_aruco import marker_localizer as ML

    ref_serves = sm.ROBOT_DOCK_MARKER[target]
    markers = read_markers(stage)
    ref_id = markers[ref_serves]["id"]
    mx, mz = marker_visual_center(stage, ref_serves)

    map_path = REPO_ROOT / "src" / "parkbot_aruco" / "data" / "marker_map_v4.json"
    mm_json = json.loads(map_path.read_text(encoding="utf-8"))
    marker_map = ML.MarkerMap.from_json(mm_json, align_yaw_deg=0.0)
    code_size_m = float(mm_json["code_size_m"])
    detector = aruco_pose.make_detector(mm_json["dictionary"])

    art_path = robot_prim_path(target)
    cam_path = find_front_camera(stage, target) if cam_role == "front" else find_rear_camera(stage, target)
    cam_prim = stage.GetPrimAtPath(cam_path)
    cam_xf = UsdGeom.Xformable(cam_prim)
    dy_world = cam_h - 0.09
    if abs(dy_world) > 1e-9:
        mb = cam_xf.ComputeLocalToWorldTransform(timeline.get_current_time())
        local_delta = mb.GetInverse().TransformDir(Gf.Vec3d(0.0, dy_world, 0.0))
        cam_xf.AddTranslateOp(UsdGeom.XformOp.PrecisionDouble, "camHeightFuse").Set(local_delta)
        for _ in range(3):
            app.update()

    rp = rep.create.render_product(cam_path, (640, 480))
    rgb_annot = rep.AnnotatorRegistry.get_annotator("rgb")
    rgb_annot.attach([rp])
    ucam = UsdGeom.Camera(cam_prim)
    focal = ucam.GetFocalLengthAttr().Get()
    haper = ucam.GetHorizontalApertureAttr().Get()
    vaper = ucam.GetVerticalApertureAttr().Get()
    K = np.array([[640.0 * focal / haper, 0.0, 320.0],
                  [0.0, 480.0 * focal / vaper, 240.0],
                  [0.0, 0.0, 1.0]], dtype=np.float64)
    return {"aruco_pose": aruco_pose, "ML": ML, "ref_id": ref_id,
            "mx": mx, "mz": mz, "marker_map": marker_map,
            "code_size_m": code_size_m, "detector": detector, "K": K,
            "dist": np.zeros((5, 1), dtype=np.float64), "cam_xf": cam_xf,
            "rgb_annot": rgb_annot}


# ---- Mission Phase C, Task C2: 측면 뎁스캠 파이프라인 ----
# p4_depth 의 depth_stop_lift_test_dual.py 는 isaacsim.sensors.camera.Camera 고수준
# 래퍼(add_distance_to_image_plane_to_frame/get_depth)를 쓰지만, 그 내부가 결국
# omni.replicator.core 로 "distance_to_image_plane" annotator 를 render_product 에
# attach 하는 것과 동일하다(isaacsim.sensors.camera.camera.Camera.
# add_distance_to_image_plane_to_frame, 828행: self.attach_annotator(annotator_name=
# "distance_to_image_plane", ...) -> rep.AnnotatorRegistry.get_annotator(...)). 여기서는
# fuse_camera_setup 의 rgb annotator 배선(rep.create.render_product + AnnotatorRegistry)
# 패턴을 그대로 따르되 annotator 이름만 "distance_to_image_plane" 으로 바꾼다 — p4_depth
# 가 이미 검증한 이름을 그대로 재사용(짐작 아님).
DEPTH_CAM_RES = (640, 480)
# p4_depth 의 depth_stop_detector.DEFAULT_ROI_FRAC 을 그대로 재사용한다(taskC2-brief.md
# 지시 — dual 러너 자체가 차종별 튜닝으로 쓴 DEPTH_ROI_FRAC=(0.44,0.56,0.44,0.56) 이
# 아니라, detector 모듈의 범용 기본값). (col_lo, col_hi, row_lo, row_hi), 0~1 비율.
DEPTH_ROI_FRAC = (0.30, 0.70, 0.50, 1.00)


def depth_setup(stage, timeline, app, robot_id, side="left", cam_res=DEPTH_CAM_RES):
    """측면 뎁스캠 render_product + distance_to_image_plane annotator 컨텍스트.

    fuse_camera_setup 의 카메라 배선 부분(render_product 생성 -> annotator attach)만
    떼어낸 구조다 — 마커/보정 없이 뎁스 스트림만 필요한 --probe=DEPTH(C2)와
    이후 이식될 정지판단(C3)이 공용으로 쓴다.
    """
    import omni.replicator.core as rep
    cam_path = find_side_camera(stage, robot_id, side)
    rp = rep.create.render_product(cam_path, cam_res)
    depth_annot = rep.AnnotatorRegistry.get_annotator("distance_to_image_plane")
    depth_annot.attach([rp])
    for _ in range(3):
        app.update()
    return {"cam_path": cam_path, "render_product": rp, "depth_annot": depth_annot}


def depth_roi_min(ctx, roi_frac=DEPTH_ROI_FRAC):
    """ctx(depth_setup 반환값)의 현재 프레임에서 ROI 최소(유효) 뎁스 [m], 없으면 None.

    p4_depth depth_stop_detector.roi_min_depth 와 같은 ROI 규약(col_lo,col_hi,row_lo,
    row_hi, 0~1 비율)이나, Task C3 에서 그 모듈을 이식하기 전까지는 이 파일 하나로
    완결되도록(브리프 지시) numpy 몇 줄로 인라인 계산한다. 프레임이 아직 준비 안
    됐거나(빈 배열/2차원 아님) ROI 안에 유효(finite) 픽셀이 하나도 없으면 None.
    """
    frame = ctx["depth_annot"].get_data()
    if frame is None or not getattr(frame, "size", 0):
        return None
    arr = np.asarray(frame, dtype=np.float64).squeeze()
    if arr.ndim != 2:
        return None
    h, w = arr.shape
    col_lo_f, col_hi_f, row_lo_f, row_hi_f = roi_frac
    row_lo, row_hi = int(h * row_lo_f), max(int(h * row_hi_f), int(h * row_lo_f) + 1)
    col_lo, col_hi = int(w * col_lo_f), max(int(w * col_hi_f), int(w * col_lo_f) + 1)
    patch = arr[row_lo:row_hi, col_lo:col_hi]
    finite = patch[np.isfinite(patch)]
    if finite.size == 0:
        return None
    return float(finite.min())


def _quat_mul(q0, q1):
    w0, x0, y0, z0 = q0
    w1, x1, y1, z1 = q1
    return np.array([
        w1*w0 - x1*x0 - y1*y0 - z1*z0,
        w1*x0 + x1*w0 + y1*z0 - z1*y0,
        w1*y0 - x1*z0 + y1*w0 + z1*x0,
        w1*z0 + x1*y0 - y1*x0 + z1*w0])


def detect_at_pose(ctx, art, app, d, lat, yaw_deg, spawn_orn, settle=20):
    """로봇을 마커 앞 (d,lat,yaw) 에 놓고 렌더→검출, 대상 마커 pose|None."""
    import cv2
    mx, mz = ctx["mx"], ctx["mz"]
    base = np.array([[mx - d, ROBOT_SPAWN_Y, mz + lat]])
    if abs(yaw_deg) < 1e-9:
        orn = np.array([spawn_orn])
    else:
        half = math.radians(yaw_deg) * 0.5
        qy = np.array([math.cos(half), 0.0, math.sin(half), 0.0])
        orn = np.array([_quat_mul(spawn_orn, qy)])
    art.set_world_poses(base, orn)
    for _ in range(settle):
        app.update()
    frame = ctx["rgb_annot"].get_data()
    img = (np.asarray(frame)[:, :, :3] if frame is not None and len(frame) else None)
    if img is None:
        return None
    gray = cv2.cvtColor(np.ascontiguousarray(img), cv2.COLOR_RGB2GRAY)
    det = ctx["aruco_pose"].detect_and_estimate(
        gray, ctx["detector"], ctx["code_size_m"], ctx["K"], ctx["dist"])
    hit = [p for p in det if int(p.marker_id) == ctx["ref_id"]]
    return hit[0] if hit else None


def detect_current(ctx):
    """현재 렌더 프레임에서 대상 마커 pose|None (로봇을 옮기지 않는다, 주행 중용)."""
    import cv2
    frame = ctx["rgb_annot"].get_data()
    img = (np.asarray(frame)[:, :, :3] if frame is not None and len(frame) else None)
    if img is None:
        return None
    gray = cv2.cvtColor(np.ascontiguousarray(img), cv2.COLOR_RGB2GRAY)
    det = ctx["aruco_pose"].detect_and_estimate(
        gray, ctx["detector"], ctx["code_size_m"], ctx["K"], ctx["dist"])
    hit = [p for p in det if int(p.marker_id) == ctx["ref_id"]]
    return hit[0] if hit else None


def localize_pose(ctx, pose, T_base_cam):
    T_cm = ctx["ML"].rvec_tvec_to_T(pose.rvec, pose.tvec)
    return ctx["ML"].robot_pose_from_marker(ctx["ref_id"], T_cm, T_base_cam, ctx["marker_map"])


def calibrate_tbasecam(ctx, art, app, timeline, gt_fn, spawn_orn):
    """GT 로 광학 규약 후보를 스윕해 T_base_cam 확정. (T_base_cam, name, err)."""
    def usd_to_np(gf_m):
        m = np.array([[gf_m[i][j] for j in range(4)] for i in range(4)], dtype=np.float64)
        return m.T
    pose0 = detect_at_pose(ctx, art, app, 1.5, 0.0, 0.0, spawn_orn)
    if pose0 is None:
        raise RuntimeError("FUSE T_base_cam 보정 자세에서 마커 미검출")
    bpos, born = art.get_world_poses()
    bp = np.asarray(bpos).reshape(-1)[:3]
    bw, bx, by, bz = (float(v) for v in np.asarray(born).reshape(-1)[:4])

    def quat_to_R(w, x, y, z):
        return np.array([
            [1-2*(y*y+z*z), 2*(x*y-w*z),   2*(x*z+w*y)],
            [2*(x*y+w*z),   1-2*(x*x+z*z), 2*(y*z-w*x)],
            [2*(x*z-w*y),   2*(y*z+w*x),   1-2*(x*x+y*y)]], dtype=np.float64)
    T_world_base = np.eye(4)
    T_world_base[:3, :3] = quat_to_R(bw, bx, by, bz)
    T_world_base[:3, 3] = bp
    T_world_camusd = usd_to_np(ctx["cam_xf"].ComputeLocalToWorldTransform(
        timeline.get_current_time()))
    candidates = {"I": np.diag([1.0, 1.0, 1.0, 1.0]),
                  "X180": np.diag([1.0, -1.0, -1.0, 1.0]),
                  "Y180": np.diag([-1.0, 1.0, -1.0, 1.0]),
                  "Z180": np.diag([-1.0, -1.0, 1.0, 1.0])}
    gx, gz, gyaw = gt_fn(art)
    best_name, best_T, best_err = None, None, 1e9
    for name, C in candidates.items():
        T_base_cam = np.linalg.inv(T_world_base) @ T_world_camusd @ C
        fix = localize_pose(ctx, pose0, T_base_cam)
        if fix is None:
            continue
        e = math.hypot(fix.x - gx, fix.z - gz)
        if e < best_err:
            best_name, best_T, best_err = name, T_base_cam, e
    if best_T is None or best_err > 0.05:
        raise RuntimeError(f"FUSE T_base_cam 보정 실패(best_err={best_err:.4f})")
    return best_T, best_name, best_err


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
    from isaacsim.core.utils.extensions import enable_extension
    enable_extension("isaacsim.ros2.bridge")
    for _ in range(12):
        app.update()
    import numpy as np
    import omni.timeline
    from isaacsim.core.prims import Articulation

    stage = build_stage(app)

    # probe B 는 측정 대상 외 로봇을 화면·물리에서 뺀다(사용자 요청 + 개루프 주행 중
    # 옆 도크 로봇과의 충돌 제거). 반드시 timeline.play()/Articulation.initialize() '전에'
    # 비활성화한다 — 초기화(=PhysX 텐서 뷰 등록) 후에 SetActive(False)로 프림을 지우면
    # 텐서 뷰가 깨져 세그폴트가 난다(관측된 크래시). play 전에 지우면 PhysX 가 아예
    # 로드하지 않으므로 안전하다.
    probe = None
    for a in sys.argv[1:]:
        if a.startswith("--probe="):
            probe = a.split("=", 1)[1]
    # --mission= 은 --probe= 와 독립된 별도 플래그다("B" 값이 겹치는 건 우연 —
    # probe=B 는 휠 오도 드리프트 측정, mission=B 는 Mission Phase B 스켈레톤).
    mission = None
    for a in sys.argv[1:]:
        if a.startswith("--mission="):
            mission = a.split("=", 1)[1]
    if probe == "B":
        keep = sm.ROBOTS[0]
        hidden = []
        for r in sm.ROBOTS:
            if r == keep:
                continue
            p = stage.GetPrimAtPath(robot_prim_path(r))
            if p and p.IsValid():
                p.SetActive(False)
                hidden.append(r)
        print(f"PROBE_B_HIDDEN kept={keep} hidden={hidden}", flush=True)
    if probe == "DEPTH":
        # Task C2: entry_follow 하나만 남긴다(다른 로봇과의 물리 간섭/충돌 제거 —
        # probe=B 와 동일한 이유·동일한 시점: Articulation.initialize() 전).
        keep = "entry_follow"
        hidden = []
        for r in sm.ROBOTS:
            if r == keep:
                continue
            p = stage.GetPrimAtPath(robot_prim_path(r))
            if p and p.IsValid():
                p.SetActive(False)
                hidden.append(r)
        print(f"PROBE_DEPTH_HIDDEN kept={keep} hidden={hidden}", flush=True)

    timeline = omni.timeline.get_timeline_interface()
    timeline.play()
    for _ in range(30):
        app.update()

    # Task C1 Step 2: 인계장 베이 차량(Pickup) 정착 확인 — spawn_handoff_vehicle() 이
    # 물리완화를 적용했지만 실제로 지터하지 않는지는 몇 프레임 굴려 실측해야 안다
    # (짐작 금지). physx 인터페이스로 직접 읽는다(omni.physx, depth_stop_lift_test_dual.py
    # 의 rigid_position() 관례와 동일 — USD 속성 캐시가 아니라 물리 시뮬레이션 값).
    import omni.physx as _omni_physx
    _physx_iface = _omni_physx.get_physx_interface()
    _veh_p0 = tuple(float(x) for x in
                     _physx_iface.get_rigidbody_transformation(HANDOFF_VEHICLE_ROOT)["position"])
    for _ in range(HANDOFF_SETTLE_FRAMES):
        app.update()
    _veh_p1 = tuple(float(x) for x in
                     _physx_iface.get_rigidbody_transformation(HANDOFF_VEHICLE_ROOT)["position"])
    _veh_drift = math.sqrt(sum((a - b) ** 2 for a, b in zip(_veh_p0, _veh_p1)))
    print(f"V4_VEHICLE_SETTLE drift_m={_veh_drift:.5f}", flush=True)

    # 비활성화된 로봇은 아티큘레이션을 만들지 않는다(프림이 없으니 초기화도 불가).
    arts = {}
    for robot_id in sm.ROBOTS:
        if not stage.GetPrimAtPath(robot_prim_path(robot_id)).IsActive():
            continue
        art = Articulation(f"{robot_prim_path(robot_id)}/base_link")
        art.initialize()
        arts[robot_id] = art

    sys.path.insert(0, str(WORK_DIR))
    from mecanum_drive import configure_hub_drives
    for robot_id in arts:
        configure_hub_drives(stage, f"{robot_prim_path(robot_id)}/joints")

    from mecanum_drive import WHEEL_JOINTS, cmd_vel_from_wheel_velocities
    from wheel_odometry import WheelOdometry

    wheel_idx = {r: {w: arts[r].dof_names.index(j) for w, j in WHEEL_JOINTS.items()}
                 for r in arts}

    def read_wheel_twist(art, idx):
        """휠 관절 '각속도'로부터 로봇 로컬 twist 를 복원한다.

        각도 차분을 쓰면 연속 회전에서 wrap 되어 폭주한다(DEBUG_LOG 2026-07-21).
        """
        vel = np.asarray(art.get_joint_velocities()).reshape(-1)
        omegas = {w: float(vel[i]) for w, i in idx.items()}
        return cmd_vel_from_wheel_velocities(omegas)

    def gt_pose_xz_yaw(art):
        """계측(채점) 전용 GT. 제어 입력으로 쓰지 않는다."""
        pos, orn = art.get_world_poses()
        pos = np.asarray(pos).reshape(-1)[:3]
        w, x, y, z = (float(v) for v in np.asarray(orn).reshape(-1)[:4])
        fwd_x = 1.0 - 2.0 * (y * y + z * z)
        fwd_z = 2.0 * (x * z - w * y)
        return float(pos[0]), float(pos[2]), math.atan2(fwd_x, fwd_z)

    # ---- Task 2: 폐루프 주행 원시요소 (drive_to_pose / rotate_in_place) ----
    # --probe=FUSE(위 참조)가 인라인으로 하던 예측(휠오도)+보정(마커) 상보필터
    # 패턴을 일반화한다. FUSE 는 전진(+X_body)만 명령했지만, 여기서는
    # body_twist_toward 가 내는 전체 메카넘 (vx,vy,wz) 를 그대로 slew 해
    # 목표 자세(x,z,yaw)까지 몬다. 제어 입력은 항상 filt.pose()(융합 자세)만
    # 쓴다 — GT(gt_pose_xz_yaw)는 반환값의 err_*_gt(리포팅 전용)에서만 쓴다.
    # FUSE 자체는 건드리지 않는다(기존 probe 회귀 방지) — 이 두 함수는 신규다.
    from mecanum_drive import wheel_velocities_from_cmd_vel, slew_twist
    from parkbot_aruco.marker_localizer import PoseFilter
    from mission_control import body_twist_toward

    def drive_to_pose(ctx, art, idx, filt, T_base_cam, target_xzyaw, *,
                       max_steps=2000, pos_gain=0.8, yaw_gain=1.2,
                       max_lin=0.25, max_ang=0.6, pos_tol=0.03, yaw_tol=0.5,
                       correct_yaw=True):
        """목표 (x,z,yaw_deg) 까지 오도+마커 융합 폐루프로 주행.

        매 스텝: 예측(휠 관절 각속도 -> cmd_vel_from_wheel_velocities ->
        filt.predict_body) -> 보정(detect_current -> localize_pose ->
        filt.update, 첫 fix 는 filt.set_pose 로 초기화) -> 제어
        (body_twist_toward(filt.pose(), target_xzyaw) -> slew_twist ->
        wheel_velocities_from_cmd_vel -> set_joint_velocity_targets).
        tol 이내(done)로 판정되면 그 뒤로는 0 twist 를 명령해 정지를 기다리고
        (slew_twist 가 실제로 (0,0,0) 에 도달하면) 종료한다. max_steps 는
        안전 상한.

        correct_yaw(Task 3b-1 추가, 기본 True=기존 동작 그대로 — ROTCHK/스켈레톤
        등 기존 호출자 회귀 없음): False 면 마커 fix 를 _apply_fix 의 위치전용
        분기로 반영한다 — x,z 만 filt.pos_gain 으로 마커 쪽으로 당기고 yaw 는
        오도메트리 값을 그대로 둔다(filt.update 를 쓰지 않음). 실측 배경: 후방캠이
        도크를 비스듬히/원거리에서 보면 yaw 관측 노이즈가 커서(최대 45° 오염 실측)
        filt.update 의 yaw 블렌딩이 정확한 오도 헤딩을 되레 망가뜨렸다 — 마커는
        위치엔 강하고 yaw 엔 약하다는 표준 센서융합 가정을 반영한 수정이다.

        반환 err_pos_gt/err_yaw_gt 는 종단(정지 후) 자세를 GT 와 비교한 값으로
        **리포팅 전용**이다 — 이 함수의 제어 로직은 filt.pose() 만 쓰고 GT 를
        전혀 참조하지 않는다.
        """
        def _apply_fix(fix):
            """마커 fix 한 건을 filt 에 반영(첫 fix 시딩 / 전체보정 / 위치전용보정).

            미션 주행은 항상 filt.set_pose(dock...) 로 사전 시딩하므로 실전에서
            filt.x 는 이 함수 안에서 None 이 아니다 — 아래 첫 분기는 시딩 없이
            drive_to_pose 를 단독 호출하는 경우(예: 과거 probe)를 위한 방어적
            폴백이며, 이때는 yaw 도 오도 예측이 없어 correct_yaw 값과 무관하게
            그대로 시딩한다.
            """
            if filt.x is None:
                filt.set_pose(fix.x, fix.z, fix.yaw_deg)   # 첫 fix 로 초기화(GT 아님)
            elif correct_yaw:
                filt.update(fix)
            else:
                # 위치전용 보정: yaw 는 오도메트리 값 유지, x/z 만 filt.pos_gain 으로
                # 마커 관측 쪽으로 블렌딩(filt.update 의 위치 블렌딩과 동일 공식).
                filt.set_pose(filt.x + filt.pos_gain * (fix.x - filt.x),
                              filt.z + filt.pos_gain * (fix.z - filt.z),
                              filt.yaw)
                filt.n_fix += 1   # filt.update() 와 동일하게: fix 는 실제로 일어났다.

        vel_buf = np.zeros(np.asarray(art.get_joint_positions()).reshape(-1).shape,
                           dtype=np.float32)
        cur_tw = (0.0, 0.0, 0.0)
        prev = timeline.get_current_time()
        steps = 0
        n_fix = 0                # 이 세그먼트(이 호출)에서 성공한 마커 fix 횟수(정지-후 보정 포함).
        stopping = False        # done 판정 이후 래치: 이후 잔차가 tol 밖으로 흔들려도 계속 정지시킨다.
        settle_poses = []       # settle 창(정지 후 보정)에서 매 프레임 관측한 filt.pose() 표본(Item3).
        # taskBYAW: 끝점(도달 여부)만으로는 90도 명령이 실제로 -270도를 돌고도 mod 360
        # 으로 우연히 맞아떨어지는 결함을 못 잡는다(회귀 원인 그 자체). 그래서 경로
        # 자체를 계측한다 — 매 스텝 GT yaw 의 wrap180 델타를 누적(unwrap)해 이 호출이
        # 실제로 "쓸고 지나간" 총 회전각을 별도로 남긴다(제어에는 쓰지 않음, 리포팅
        # 전용 — gt_pose_xz_yaw 와 같은 용도).
        swept_gt_deg = 0.0
        _, _, _swept_prev = gt_pose_xz_yaw(art)
        for _ in range(max_steps):
            app.update()
            steps += 1
            now = timeline.get_current_time()
            dt = min(0.1, max(0.0, now - prev)); prev = now

            # ---- 예측: 휠 오도(관절 각속도) -> 바디 twist -> predict_body ----
            # wz 에만 YAW_ODOM_SCALE 을 곱한다(vx/vy 는 미보정 — probe B 실측상
            # 선형 오도 드리프트는 이미 낮다, 회전만 별도 버그). --probe=FUSE 는
            # 이 함수를 쓰지 않고 자체 인라인 predict_body 를 그대로 유지한다(회귀
            # 방지 기준선 — YAW_ODOM_SCALE 미적용).
            vel = np.asarray(art.get_joint_velocities()).reshape(-1)
            wv = {w: float(vel[i]) for w, i in idx.items()}
            pvx, pvy, pwz = cmd_vel_from_wheel_velocities(wv)
            filt.predict_body(pvx, pvy, pwz * YAW_ODOM_SCALE, dt)

            # ---- 계측: 실제 스윕 각(GT, unwrap 누적) — 리포팅 전용, 제어에 안 씀 ----
            _, _, _swept_now = gt_pose_xz_yaw(art)
            _swept_dyaw = (_swept_now - _swept_prev + math.pi) % (2.0 * math.pi) - math.pi
            if abs(_swept_dyaw) > math.radians(5.0):
                # 물리적으로 한 스텝(120Hz)에 5도 이상 도는 것은 이 로봇의 최대
                # 각속도(~0.6rad/s≈0.005rad/step)로 불가능하다 — atan2 분기 근처
                # 샘플링 아티팩트나 실제 이상 거동을 놓치지 않기 위한 안전장치.
                print(f"SWEPT_JUMP_WARN step={steps} dyaw_deg={math.degrees(_swept_dyaw):.2f} "
                      f"raw_prev={math.degrees(_swept_prev):.2f} raw_now={math.degrees(_swept_now):.2f}",
                      flush=True)
            swept_gt_deg += math.degrees(_swept_dyaw)
            _swept_prev = _swept_now

            # ---- 보정: 마커 검출 시 fix ----
            pose = detect_current(ctx)
            if pose is not None:
                fix = localize_pose(ctx, pose, T_base_cam)
                if fix is not None:
                    _apply_fix(fix)
                    n_fix += 1

            # ---- 제어: 융합 자세 -> 목표까지 body twist(body_twist_toward) ----
            fp = filt.pose()
            if fp is None:
                # 아직 융합 자세가 없다(시딩도, fix 도 없었음) — 안전하게 정지 유지.
                tvx, tvy, twz = 0.0, 0.0, 0.0
            elif not stopping:
                tvx, tvy, twz, done = body_twist_toward(
                    fp, target_xzyaw, pos_gain=pos_gain, yaw_gain=yaw_gain,
                    max_lin=max_lin, max_ang=max_ang, pos_tol=pos_tol, yaw_tol=yaw_tol)
                if done:
                    stopping = True
                    tvx, tvy, twz = 0.0, 0.0, 0.0
            else:
                tvx, tvy, twz = 0.0, 0.0, 0.0
            cur_tw = slew_twist(cur_tw, (tvx, tvy, twz), dt, linear_accel=LINEAR_ACCEL,
                                linear_decel=LINEAR_DECEL, angular_accel=ANGULAR_ACCEL)
            omegas = wheel_velocities_from_cmd_vel(*cur_tw)
            vel_buf[...] = 0.0
            for w, om in omegas.items():
                vel_buf[idx[w]] = om
            art.set_joint_velocity_targets(vel_buf)

            if stopping and cur_tw == (0.0, 0.0, 0.0):
                # 정지 후 몇 프레임 더 보정(FUSE 종단 처리와 동일 관례). reached 는
                # 여기서 확정하지 않는다 — 이 보정이 filt.pose() 를 움직일 수 있어
                # 루프 종료 후 최종 자세 기준으로 재판정한다(아래 reached 재검증, Item3:
                # 마지막 한 프레임이 아니라 이 창 전체의 강건 중앙값으로 재판정한다).
                for _ in range(30):
                    app.update()
                    pose = detect_current(ctx)
                    if pose is not None:
                        fix = localize_pose(ctx, pose, T_base_cam)
                        if fix is not None and filt.x is not None:
                            _apply_fix(fix)
                            n_fix += 1
                    fp_settle = filt.pose()
                    if fp_settle is not None:
                        settle_poses.append(fp_settle)
                break

        # ---- settle 표본 중앙값으로 filt 잡음 억제(Task 3b-harden Item3,
        # taskBharden-report.md): settle 창은 로봇이 완전히 정지(twist=0)한 채로 계속
        # 마커 fix 를 받는 구간이라, fix 자체의 프레임간 잡음이 정지한 로봇 주위에서
        # filt 를 미세하게 흔든다 — 실측: 어떤 run 은 물리적으로 err_pos_gt=3.3cm 로
        # 정상 수렴했는데도 창의 "마지막 한 프레임"만 우연히 잡음이 커 filt-target 거리가
        # 5.5cm 로 튀어 reached=False 가 됐다(MISSIONB_RESULT ok 가 2/3 로 비결정적이었던
        # 원인 — 3cm pos_tol 을 5cm 로 늘려도 5.5cm 미스는 못 가린다). 로봇을 다시 몰지
        # 않는다(물리적으로 이미 수렴해 있다는 게 그 사례의 err_pos_gt 로 확인된다) — 대신
        # 창에서 관측한 filt 표본들의 중앙값(x,z)·원형평균(yaw)으로 마지막 한 표본의 잡음을
        # 눌러 filt 자체를 갱신한다. 표본이 없으면(마커가 한 번도 안 잡힌 세그먼트, 예:
        # ROTCHK/ROTCHK180 의 순수오도 회전, 또는 max_steps 소진으로 settle 을 못 밟은
        # 경우) 원래 filt 그대로 두어 기존 동작을 보존한다.
        if settle_poses:
            xs = sorted(p[0] for p in settle_poses)
            zs = sorted(p[1] for p in settle_poses)
            n = len(xs)
            mid = n // 2
            med_x = xs[mid] if n % 2 else 0.5 * (xs[mid - 1] + xs[mid])
            med_z = zs[mid] if n % 2 else 0.5 * (zs[mid - 1] + zs[mid])
            sy = sum(math.sin(math.radians(p[2])) for p in settle_poses)
            cy = sum(math.cos(math.radians(p[2])) for p in settle_poses)
            med_yaw = math.degrees(math.atan2(sy, cy))
            filt.set_pose(med_x, med_z, med_yaw)

        # ---- reached 재검증(settle 후): 루프 중간에 래치한 값을 쓰지 않고, settle
        # 루프가 끝난 뒤의 최종(=위에서 표본이 있었다면 중앙값으로 잡음을 억제한)
        # filt.pose() 를 target_xzyaw 에 다시 견주어 판정한다. 루프가 정상 정지(break)로
        # 끝났든 max_steps 소진으로 끝났든 동일하게 적용된다. 제어 경로와 마찬가지로
        # GT 가 아니라 filt.pose() 만 쓴다.
        fp = filt.pose()
        if fp is None:
            reached = False
        else:
            _, _, _, reached = body_twist_toward(
                fp, target_xzyaw, pos_gain=pos_gain, yaw_gain=yaw_gain,
                max_lin=max_lin, max_ang=max_ang, pos_tol=pos_tol, yaw_tol=yaw_tol)

        gx, gz, gyaw = gt_pose_xz_yaw(art)
        if fp is not None:
            err_pos_gt = math.hypot(fp[0] - gx, fp[1] - gz)
            err_yaw_gt = abs((fp[2] - math.degrees(gyaw) + 180.0) % 360.0 - 180.0)
        else:
            err_pos_gt, err_yaw_gt = float("nan"), float("nan")
        return {"reached": reached, "steps": steps, "err_pos_gt": err_pos_gt,
                "err_yaw_gt": err_yaw_gt, "final_filt": fp, "n_fix": n_fix,
                "swept_gt_deg": swept_gt_deg}

    def rotate_in_place(ctx, art, idx, filt, T_base_cam, target_yaw_deg, **kwargs):
        """제자리 회전: 위치는 현재 융합 x,z 그대로 두고 yaw 만 target_yaw_deg 로."""
        fp = filt.pose()
        if fp is None:
            # drive_to_pose 의 fp is None 처리(제어 루프 안에서 0 twist 로 대기하며
            # 다음 fix 를 기다림)와 달리, 여기서는 대기할 target 자체를 아직 만들 수
            # 없다(현재 x,z 가 없으면 (cx,cz,target_yaw_deg) 를 구성할 수 없다).
            # GT 로 대신 채우면 제어 순수성이 깨지고 (0,0) 등으로 채우면 엉뚱한
            # 좌표로 폭주할 수 있으므로, 조용히 넘기지 않고 명확한 에러로 실패한다.
            raise RuntimeError(
                "rotate_in_place: filt 가 아직 시딩되지 않았습니다(filt.pose() is "
                "None). 회전 전에 filt.set_pose(...) 로 최소 한 번 시딩하거나 "
                "drive_to_pose 로 먼저 위치를 확보한 뒤 호출하세요.")
        cx, cz, _ = fp
        return drive_to_pose(ctx, art, idx, filt, T_base_cam, (cx, cz, target_yaw_deg), **kwargs)

    odom_mode = "wheel"
    for a in sys.argv[1:]:
        if a.startswith("--odom="):
            odom_mode = a.split("=", 1)[1]
    if odom_mode not in ("gt", "wheel"):
        raise SystemExit(f"--odom 은 gt 또는 wheel 이어야 합니다: {odom_mode!r}")

    odom = {}
    for r in arts:
        gx, gz, gyaw = gt_pose_xz_yaw(arts[r])
        odom[r] = WheelOdometry(x=gx, z=gz, yaw=gyaw)   # 초기 자세만 GT 로 정렬
    print(f"V4_ODOM_MODE={odom_mode}", flush=True)

    n_cams = 0
    for a in sys.argv[1:]:
        if a.startswith("--cameras="):
            n_cams = int(a.split("=", 1)[1])
    if n_cams not in (0, 2, 4):
        raise SystemExit(f"--cameras 는 0, 2, 4 중 하나여야 합니다: {n_cams}")

    # 2대일 때는 팀당 리드에만 단다.
    cam_robots = {0: (), 2: ("entry_lead", "exit_lead"), 4: sm.ROBOTS}[n_cams]
    for r in cam_robots:
        attach_camera_graph(r, find_front_camera(stage, r), role="front")
    for _ in range(30):
        app.update()
    print(f"V4_CAMERAS n={n_cams} robots={list(cam_robots)}", flush=True)

    # probe 는 이미 build_stage 직후에 파싱했다(probe B 조기 비활성화 때문).

    if probe == "C":
        import time as _time
        import v4_probes as vp
        for _ in range(120):                 # 워밍업
            app.update()
        t_wall = _time.monotonic()
        t_sim = timeline.get_current_time()
        for _ in range(600):
            app.update()
        rtf = ((timeline.get_current_time() - t_sim)
               / max(_time.monotonic() - t_wall, 1e-6))
        path = vp.write_report(f"probe_c_rtf_{n_cams}cam", {
            "cameras": n_cams, "robots": list(cam_robots), "rtf": rtf})
        print(f"PROBE_C_RESULT cameras={n_cams} rtf={rtf:.3f} report={path.name}",
              flush=True)
        if headless:
            app.close()
            return

    if probe == "A":
        # Probe A (외부 검출형): Isaac 은 카메라 영상만 발행하고, 검출·판정은
        # 별도 ROS 노드(probe_a_detector_node)가 시스템 cv2 로 한다. 실제 배포
        # 파이프라인 그대로다(배포엔 Isaac 이 없고 외부 노드가 검출). 러너는
        # 로봇을 마커 앞 거리를 바꿔가며 세우고 현재 거리를 /probe_a/state 로 알린다.
        import json
        from pxr import Gf, UsdGeom
        BRIDGE_RCLPY_A = Path(
            "/home/rokey/dev_ws/isaac_sim/isaacsim/_build/linux-x86_64/release"
            "/exts/isaacsim.ros2.bridge/humble/rclpy")
        if str(BRIDGE_RCLPY_A) not in sys.path:
            sys.path.insert(0, str(BRIDGE_RCLPY_A))
        import rclpy
        from std_msgs.msg import String as RosString
        if not rclpy.ok():
            rclpy.init()
        pa_node = rclpy.create_node("probe_a_bringup")
        state_pub = pa_node.create_publisher(RosString, "/probe_a/state", 10)

        hold_sec = 3.0
        cam_h = None
        loops = 0                       # 0 = 무한(외부 노드가 볼 때까지). 헤드리스는 1회.
        for a in sys.argv[1:]:
            if a.startswith("--hold-sec="):
                hold_sec = float(a.split("=", 1)[1])
            if a.startswith("--cam-height="):
                cam_h = float(a.split("=", 1)[1])
            if a.startswith("--probe-a-loops="):
                loops = int(a.split("=", 1)[1])

        target = "entry_lead"
        art = arts[target]
        markers = read_markers(stage)
        ref = markers[sm.ROBOT_DOCK_MARKER[target]]      # 대상 도크 마커(id/kind)

        # 도크 마커는 aruco:position(도크 중심)과 실제 데칼 위치가 z 로 0.7m 어긋난다.
        # 카메라가 보는 건 데칼이므로 배치는 marker_visual_center() 실좌표를 쓴다.
        mx, mz = marker_visual_center(stage, sm.ROBOT_DOCK_MARKER[target])
        print(f"PROBE_A_GEOMETRY attr_xz=({ref['x']:.3f},{ref['z']:.3f}) "
              f"visual_xz=({mx:.3f},{mz:.3f}) target_marker_id={ref['id']}", flush=True)

        cam_path = find_front_camera(stage, target)
        cam_prim = stage.GetPrimAtPath(cam_path)
        cam_xf = UsdGeom.Xformable(cam_prim)

        # 스폰 자세(로봇 루트 AddRotateXOp(-90))를 그대로 재사용한다. 카메라 정면(-Z)은
        # 월드 +X 를 30도 아래로 본다. "마커 앞 d 미터"는 x 축으로 재 로봇 중심을 mx-d 에 둔다.
        _, spawn_orn = art.get_world_poses()
        spawn_orn = np.asarray(spawn_orn).reshape(-1)[:4].copy()

        # ---- 카메라 높이 오버라이드 ---- (월드 Y dy 를 카메라 로컬로 역변환)
        if cam_h is not None:
            dy_world = cam_h - 0.09
            mat_before = cam_xf.ComputeLocalToWorldTransform(timeline.get_current_time())
            local_delta = mat_before.GetInverse().TransformDir(Gf.Vec3d(0.0, dy_world, 0.0))
            cam_xf.AddTranslateOp(
                UsdGeom.XformOp.PrecisionDouble, "camHeightProbe").Set(local_delta)
            for _ in range(3):
                app.update()
            mat_after = cam_xf.ComputeLocalToWorldTransform(timeline.get_current_time())
            actual_dy = (mat_after.ExtractTranslation()[1]
                         - mat_before.ExtractTranslation()[1])
            print(f"PROBE_A_CAM_HEIGHT requested_dy={dy_world:.4f} "
                  f"actual_world_dy={actual_dy:.4f}", flush=True)

        # 카메라 영상을 C++ OmniGraph 로 발행(image_raw + camera_info). 검출은 외부.
        attach_camera_graph(target, cam_path, role="front")
        for _ in range(30):
            app.update()
        print(f"PROBE_A_PUBLISHING image=/robot_{target}/front/image_raw "
              f"info=/robot_{target}/front/camera_info state=/probe_a/state "
              f"domain={os.environ.get('ROS_DOMAIN_ID','0')} "
              f"target_marker_id={ref['id']}", flush=True)

        def _publish_state(d, phase):
            msg = RosString()
            msg.data = json.dumps({
                "distance_m": round(float(d), 3),
                "marker_id": int(ref["id"]),
                "marker_serves": sm.ROBOT_DOCK_MARKER[target],
                "camera_height_m": cam_h or 0.09,
                "phase": phase})
            state_pub.publish(msg)
            rclpy.spin_once(pa_node, timeout_sec=0.0)

        def _hold(d, phase, frames):
            for _ in range(frames):
                app.update()
                _publish_state(d, phase)

        loop_i = 0
        while app.is_running():
            for i in range(16):
                d = 0.6 + 0.1 * i                # 마커 앞 0.6~2.1 m
                art.set_world_poses(np.array([[mx - d, ROBOT_SPAWN_Y, mz]]),
                                    np.array([spawn_orn]))
                _hold(d, "sweep", int(hold_sec * RENDER_HZ))
            loop_i += 1
            if (loops and loop_i >= loops) or (headless and loops == 0):
                break
        # 스윕 종료 알림을 잠깐 발행(외부 노드가 요약을 낼 수 있게)
        for _ in range(int(2.0 * RENDER_HZ)):
            app.update()
            _publish_state(0.0, "done")
        if headless:
            app.close()
            return
        # GUI: 창을 닫을 때까지 done 을 계속 발행하며 살아있게 둔다.
        while app.is_running():
            app.update()
            _publish_state(0.0, "done")
        app.close()
        return

    if probe == "REAR":
        # 후방 카메라 실동작: 로봇 뒤(−X 방향)에 마커가 오도록 배치하고 후방 카메라 발행.
        target = "entry_lead"
        cam_path = find_rear_camera(stage, target)
        ref_serves = sm.ROBOT_DOCK_MARKER[target]
        mx, mz = marker_visual_center(stage, ref_serves)
        # 후방 카메라는 −X 를 보므로, 마커가 뒤에 오도록 로봇 중심을 mx + d 에 둔다.
        art = arts[target]
        _, spawn_orn = art.get_world_poses()
        spawn_orn = np.asarray(spawn_orn).reshape(-1)[:4].copy()
        import json as _json
        BRIDGE = Path("/home/rokey/dev_ws/isaac_sim/isaacsim/_build/linux-x86_64/release"
                      "/exts/isaacsim.ros2.bridge/humble/rclpy")
        if str(BRIDGE) not in sys.path:
            sys.path.insert(0, str(BRIDGE))
        import rclpy
        from std_msgs.msg import String as RosString
        if not rclpy.ok():
            rclpy.init()
        pa = rclpy.create_node("rear_probe_bringup")
        state_pub = pa.create_publisher(RosString, "/probe_a/state", 10)
        attach_camera_graph(target, cam_path, role="rear")   # /robot_entry_lead/rear/image_raw 로 후방 발행
        for _ in range(30):
            app.update()
        ref_id = read_markers(stage)[ref_serves]["id"]
        print(f"REAR_PUBLISHING image=/robot_{target}/rear/image_raw target_marker_id={ref_id} "
              f"domain={os.environ.get('ROS_DOMAIN_ID','0')}", flush=True)
        loop = 0
        while app.is_running():
            for i in range(12):
                d = 0.6 + 0.1 * i
                art.set_world_poses(np.array([[mx + d, ROBOT_SPAWN_Y, mz]]),
                                    np.array([spawn_orn]))
                for _ in range(int(3.0 * (60.0 if headless else RENDER_HZ))):
                    app.update()
                    msg = RosString()
                    msg.data = _json.dumps({"distance_m": round(d, 3),
                                            "marker_id": int(ref_id), "phase": "sweep"})
                    state_pub.publish(msg)
                    rclpy.spin_once(pa, timeout_sec=0.0)
            loop += 1
            if headless and loop >= 1:
                break
        if headless:
            app.close()
            return
        while app.is_running():
            app.update()
        app.close()
        return

    if probe == "M5":
        # 마커 측위 정확도 관문. Isaac 안에서 렌더→검출→robot_pose_from_marker 로
        # 월드 자세를 복원해 GT 와 비교한다. T_base_cam(카메라 마운트)은 카메라를
        # 0.15m 로 올렸으므로 v1 기본값을 쓰면 틀린다 → GT 로 자동 보정한다.

        cam_h = 0.15
        for a in sys.argv[1:]:
            if a.startswith("--cam-height="):
                cam_h = float(a.split("=", 1)[1])

        n_frames = 1
        for a in sys.argv[1:]:
            if a.startswith("--m5-frames="):
                n_frames = max(1, int(a.split("=", 1)[1]))

        dmin, dmax = 1.4, 1.8
        for a in sys.argv[1:]:
            if a.startswith("--m5-dmin="):
                dmin = float(a.split("=", 1)[1])
            if a.startswith("--m5-dmax="):
                dmax = float(a.split("=", 1)[1])

        target = "entry_lead"
        art = arts[target]

        # yaw 회전 기준이 되는 스폰 자세. 카메라 높이 오버라이드의 app.update 전에
        # 잡아 리팩터 전 M5 와 동일한 값을 쓴다.
        _, spawn_orn = art.get_world_poses()
        spawn_orn = np.asarray(spawn_orn).reshape(-1)[:4].copy()

        # 카메라 셋업·검출 컨텍스트(높이 오버라이드 + rgb annotator + K + 지도/검출기/마커).
        # FUSE 와 공유하려고 모듈 함수로 추출했다.
        ctx = fuse_camera_setup(stage, timeline, app, target, cam_h)

        def capture_frames(d, lat, yaw_deg, k):
            """같은 자세에서 k 프레임을 잡아 각 프레임의 측위를 리스트로 돌려준다.

            프레임 사이에 app.update() 를 돌려 렌더가 갱신되게 한다. 정지 자세라
            렌더가 결정론적이면 프레임들이 거의 같아 융합 효과가 없다 — 그 경우
            single 과 fused 가 비슷하게 나오며, 그것이 정직한 결과다.
            """
            poses = []
            first = detect_at_pose(ctx, art, app, d, lat, yaw_deg, spawn_orn)  # 첫 프레임(자세 세팅 포함)
            gxx, gzz, gyy = gt_pose_xz_yaw(art)           # 이 자세의 GT
            if first is not None:
                poses.append(first)
            for _ in range(k - 1):
                for _ in range(2):
                    app.update()
                hit = detect_current(ctx)
                if hit is not None:
                    poses.append(hit)
            return poses, (gxx, gzz, gyy)

        def fuse_localize(poses, T_base_cam):
            """여러 프레임의 측위(RobotFix)를 융합: x,z 평균 + yaw 원형평균."""
            fixes = [localize_pose(ctx, p, T_base_cam) for p in poses]
            fixes = [f for f in fixes if f is not None]
            if not fixes:
                return None
            xs = sum(f.x for f in fixes) / len(fixes)
            zs = sum(f.z for f in fixes) / len(fixes)
            sy = sum(math.sin(math.radians(f.yaw_deg)) for f in fixes)
            cy = sum(math.cos(math.radians(f.yaw_deg)) for f in fixes)
            yaw = math.degrees(math.atan2(sy, cy))
            return xs, zs, yaw, fixes[0]      # 융합값 + 첫 프레임(단일 비교용)

        # ---- T_base_cam 자동 보정(모듈 함수: GT 로 광학 규약 후보 스윕) ----
        # 보정·검출·측위는 FUSE 와 공유하는 모듈 함수가 담당한다. 보정 실패 시에도
        # 리팩터 전과 같은 M5_TBASECAM_CAL FAIL 토큰·헤드리스 종료 동작을 유지한다.
        try:
            T_base_cam, best_name, best_err = calibrate_tbasecam(
                ctx, art, app, timeline, gt_pose_xz_yaw, spawn_orn)
        except RuntimeError as e:
            if "미검출" in str(e):
                print("M5_TBASECAM_CAL FAIL: 검증 자세에서 마커 미검출 — 기하 재검토", flush=True)
            else:
                print("M5_TBASECAM_CAL FAIL: 어떤 광학 규약도 5cm 안에 못 맞춤 — "
                      "usd_to_np 전치/규약 재검토 필요", flush=True)
            if headless:
                app.close()
            raise
        print(f"M5_TBASECAM_CAL best={best_name} verify_pos_err={best_err:.4f}m", flush=True)

        # ---- 정확도 스윕(단일+융합 동시 측정) ----
        def err_of(fx, fz, fyaw_deg, gt):
            gxx, gzz, gyy = gt
            ex, ez = fx - gxx, fz - gzz
            dyaw = (fyaw_deg - math.degrees(gyy) + 180.0) % 360.0 - 180.0
            return math.hypot(ex, ez), abs(dyaw), ex, ez, dyaw

        single_pos, single_yaw = [], []
        fused_pos, fused_yaw, fused_vec = [], [], []
        dists = []
        n_total = 0
        for d in (1.3, 1.4, 1.5, 1.6, 1.7, 1.8, 1.9):
            for lat in (-0.15, 0.0, 0.15):
                for yaw_deg in (-8.0, 0.0, 8.0):
                    n_total += 1
                    poses, gt = capture_frames(d, lat, yaw_deg, n_frames)
                    if not poses:
                        continue
                    fused = fuse_localize(poses, T_base_cam)
                    if fused is None:
                        continue
                    fx, fz, fyaw, f0 = fused
                    sp, sy, *_ = err_of(f0.x, f0.z, f0.yaw_deg, gt)   # 단일=첫 프레임
                    single_pos.append(sp); single_yaw.append(sy)
                    fp, fy, ex, ez, dyaw = err_of(fx, fz, fyaw, gt)   # 융합
                    fused_pos.append(fp); fused_yaw.append(fy)
                    fused_vec.append((ex, ez, dyaw))
                    dists.append(d)

        if not fused_pos:
            print("M5_RESULT FAIL: 검출 표본 0개", flush=True)
            if headless:
                app.close()
            raise RuntimeError("M5 검출 표본 0")

        def p95(a):
            b = sorted(a)
            return b[min(len(b) - 1, int(math.ceil(0.95 * len(b)) - 1))]

        def stats(errs):
            b = sorted(errs)
            return {"mean": sum(b) / len(b), "median": b[len(b) // 2],
                    "p95": p95(b), "max": b[-1]}

        cov = np.cov(np.array(fused_vec).T).tolist() if len(fused_vec) > 1 else None
        # fused_pos/fused_yaw 는 각 표본의 오차, dists 는 같은 순서의 거리
        def p95(a):
            b = sorted(a)
            return b[min(len(b) - 1, int(math.ceil(0.95 * len(b)) - 1))] if b else float("nan")

        in_win = [i for i, dd in enumerate(dists) if dmin <= dd <= dmax]
        win_pos = [fused_pos[i] for i in in_win]
        win_yaw = [fused_yaw[i] for i in in_win]

        full_pp, full_yp = p95(fused_pos), p95(fused_yaw)
        win_pp, win_yp = p95(win_pos), p95(win_yaw)
        ok = (len(win_pos) > 0 and win_pp <= 0.02 and win_yp <= 1.0)
        report = {
            "camera_height_m": cam_h, "frames": n_frames,
            "trusted_window": {"dmin": dmin, "dmax": dmax},
            "tbasecam_convention": best_name, "tbasecam_verify_err_m": best_err,
            "n_full": len(fused_pos), "n_window": len(win_pos), "n_total": n_total,
            "full": {"pos_err_m": stats(fused_pos), "yaw_err_deg": stats(fused_yaw)},
            "window": {"pos_err_m": stats(win_pos) if win_pos else None,
                       "yaw_err_deg": stats(win_yaw) if win_yaw else None},
            "cov_ex_ez_eyaw": cov,
        }
        import v4_probes as vp
        path = vp.write_report("m5_accuracy", report)
        print(f"M5_RESULT={'PASS' if ok else 'FAIL'} window=[{dmin},{dmax}]m "
              f"window_pos_p95={win_pp*100:.2f}cm window_yaw_p95={win_yp:.2f}deg "
              f"full_pos_p95={full_pp*100:.2f}cm full_yaw_p95={full_yp:.2f}deg "
              f"n_win={len(win_pos)}/{len(fused_pos)} report={path.name}", flush=True)
        if headless:
            app.close()
            return
        while app.is_running():
            app.update()
        app.close()
        return

    if probe == "FUSE":
        # 주행 접근 중 오도(예측)+마커(보정) 상보필터 융합 검증. 종단 수렴 오차로 판정.
        from pxr import UsdGeom
        from parkbot_aruco.marker_localizer import PoseFilter
        from mecanum_drive import (WHEEL_JOINTS, wheel_velocities_from_cmd_vel,
                                   cmd_vel_from_wheel_velocities, slew_twist)

        speed, d_start, d_end = 0.25, 2.1, 1.25
        pos_gain, yaw_gain = 0.5, 0.5
        for a in sys.argv[1:]:
            if a.startswith("--fuse-speed="):  speed = float(a.split("=", 1)[1])
            if a.startswith("--fuse-dstart="): d_start = float(a.split("=", 1)[1])
            if a.startswith("--fuse-dend="):   d_end = float(a.split("=", 1)[1])
            if a.startswith("--fuse-posgain="): pos_gain = float(a.split("=", 1)[1])
            if a.startswith("--fuse-yawgain="): yaw_gain = float(a.split("=", 1)[1])
        cam_h = 0.15
        for a in sys.argv[1:]:
            if a.startswith("--cam-height="): cam_h = float(a.split("=", 1)[1])

        target = "entry_lead"
        art = arts[target]
        idx = wheel_idx[target]
        ctx = fuse_camera_setup(stage, timeline, app, target, cam_h)
        _, spawn_orn = art.get_world_poses()
        spawn_orn = np.asarray(spawn_orn).reshape(-1)[:4].copy()
        T_base_cam, cal_name, cal_err = calibrate_tbasecam(
            ctx, art, app, timeline, gt_pose_xz_yaw, spawn_orn)
        print(f"FUSE_TBASECAM_CAL best={cal_name} verify_pos_err={cal_err:.4f}m", flush=True)

        # 로봇을 접근 시작점(마커 앞 d_start, 정면 자세)에 놓는다.
        mx, mz = ctx["mx"], ctx["mz"]
        art.set_world_poses(np.array([[mx - d_start, ROBOT_SPAWN_Y, mz]]),
                            np.array([spawn_orn]))
        for _ in range(30):
            app.update()

        vel_buf = np.zeros(np.asarray(art.get_joint_positions()).reshape(-1).shape,
                           dtype=np.float32)
        filt = PoseFilter(pos_gain=pos_gain, yaw_gain=yaw_gain)
        cur_tw = (0.0, 0.0, 0.0)
        prev = timeline.get_current_time()
        traj = []                     # (gt_x,gt_z,gt_yawdeg, f_x,f_z,f_yaw, single_x,single_z,single_yaw|None)
        max_steps = 2000
        for _ in range(max_steps):
            app.update()
            now = timeline.get_current_time()
            dt = min(0.1, max(0.0, now - prev)); prev = now
            # 전진(+X_body) 주행. 종단 도달하면 정지.
            gx, gz, gyaw = gt_pose_xz_yaw(art)
            d_now = mx - gx                     # 마커까지 남은 거리(월드 X)
            tgt = (speed, 0.0, 0.0) if d_now > d_end else (0.0, 0.0, 0.0)
            cur_tw = slew_twist(cur_tw, tgt, dt, linear_accel=LINEAR_ACCEL,
                                linear_decel=LINEAR_DECEL, angular_accel=ANGULAR_ACCEL)
            omegas = wheel_velocities_from_cmd_vel(*cur_tw)
            vel_buf[...] = 0.0
            for w, om in omegas.items():
                vel_buf[idx[w]] = om
            art.set_joint_velocity_targets(vel_buf)

            # ---- 예측: 휠 오도(관절 각속도) → 바디 twist → predict_body ----
            vel = np.asarray(art.get_joint_velocities()).reshape(-1)
            wv = {w: float(vel[i]) for w, i in idx.items()}
            vx, vy, wz = cmd_vel_from_wheel_velocities(wv)
            filt.predict_body(vx, vy, wz, dt)

            # ---- 보정: 마커 검출 시 fix ----
            pose = detect_current(ctx)
            single = None
            if pose is not None:
                fix = localize_pose(ctx, pose, T_base_cam)
                if fix is not None:
                    if filt.x is None:
                        filt.set_pose(fix.x, fix.z, fix.yaw_deg)   # 첫 fix 로 초기화(GT 아님)
                    else:
                        filt.update(fix)
                    single = (fix.x, fix.z, fix.yaw_deg)

            fp = filt.pose()
            if fp is not None:
                traj.append((gx, gz, math.degrees(gyaw), fp[0], fp[1], fp[2],
                             single[0] if single else None,
                             single[1] if single else None,
                             single[2] if single else None))
            if d_now <= d_end and cur_tw == (0.0, 0.0, 0.0):
                # 종단 도달 + 정지. 몇 프레임 더 보정 후 종료.
                for _ in range(30):
                    app.update()
                    pose = detect_current(ctx)
                    if pose is not None:
                        fix = localize_pose(ctx, pose, T_base_cam)
                        if fix is not None and filt.x is not None:
                            filt.update(fix)
                break

        if not traj or filt.x is None:
            print("FUSE_RESULT FAIL: 융합 표본 없음(마커 미검출)", flush=True)
            if headless: app.close()
            raise RuntimeError("FUSE 융합 표본 없음")

        # 종단(마지막) 지점 GT vs 융합
        gx, gz, gyaw = gt_pose_xz_yaw(art)
        fp = filt.pose()
        term_pos = math.hypot(fp[0] - gx, fp[1] - gz)
        term_yaw = abs((fp[2] - math.degrees(gyaw) + 180.0) % 360.0 - 180.0)

        # 궤적 p95(첫 fix 이후 융합 오차) — 참고
        def p95(a):
            b = sorted(a); return b[min(len(b)-1, int(math.ceil(0.95*len(b))-1))] if b else float("nan")
        fpos = [math.hypot(r[3]-r[0], r[4]-r[1]) for r in traj]
        fyaw = [abs((r[5]-r[2]+180.0) % 360.0 - 180.0) for r in traj]
        spos = [math.hypot(r[6]-r[0], r[7]-r[1]) for r in traj if r[6] is not None]

        ok = (term_pos <= 0.02) and (term_yaw <= 1.0)
        import v4_probes as vp
        report = {"camera_height_m": cam_h, "speed": speed,
                  "d_start": d_start, "d_end": d_end,
                  "pos_gain": pos_gain, "yaw_gain": yaw_gain,
                  "tbasecam_convention": cal_name, "tbasecam_verify_err_m": cal_err,
                  "n_traj": len(traj), "n_single": len(spos),
                  "terminal_pos_err_m": term_pos, "terminal_yaw_err_deg": term_yaw,
                  "fused_traj_pos_p95_m": p95(fpos), "fused_traj_yaw_p95_deg": p95(fyaw),
                  "single_traj_pos_p95_m": p95(spos) if spos else None}
        path = vp.write_report("fuse_approach", report)
        print(f"FUSE_RESULT={'PASS' if ok else 'FAIL'} "
              f"term_pos={term_pos*100:.2f}cm term_yaw={term_yaw:.2f}deg "
              f"fused_traj_p95={p95(fpos)*100:.2f}cm "
              f"single_traj_p95={(p95(spos)*100 if spos else float('nan')):.2f}cm "
              f"n={len(traj)} cal={cal_name} report={path.name}", flush=True)
        if headless:
            app.close(); return
        while app.is_running():
            app.update()
        app.close(); return

    if probe == "DEPTH":
        # Mission Phase C, Task C2: 측면 뎁스캠 파이프라인 증명. entry_follow 를
        # 인계장 베이 진입선(+X 끝, 트럭 밖)에 -X 를 보게 놓고 트럭(Pickup, C1이 배치)
        # 밑을 천천히 -X 로 통과시키며 매 프레임 ROI 최소뎁스를 찍는다. 검출 로직(C3)
        # 은 아직 없다 — 여기서는 depth_setup/depth_roi_min 로 만든 스트림 자체가
        # 바퀴를 지날 때 뚜렷한 트로프(하강->회복)를 두 번(후축, 전축) 보이는지만
        # 실측으로 증명한다.
        target = "entry_follow"
        art = arts[target]
        idx = wheel_idx[target]

        speed = 0.4
        for a in sys.argv[1:]:
            if a.startswith("--depth-speed="):
                speed = float(a.split("=", 1)[1])

        # x_start: 베이 진입선. 트럭 후미 x = HANDOFF_BAY_CENTER[0] + 길이/2 =
        # -8.5+5.829/2 = -5.586(brief 의 length=5.829). 로봇 전체가 트럭 밖에 서도록
        # 1m 남짓 여유를 둔다. x_end: 양 축(후축≈-6.569, 전축≈-10.163)을 다 지나
        # 전방 범퍼(x≈-11.415)까지 넘어서 두 번째 트로프의 "회복"까지 보이게 한다.
        x_start = -4.5
        x_end = -11.8

        # 로봇을 -X 를 보게 180도 반전(REARXN 의 _yaw_quat(spawn_orn,180.0)과 동일
        # 관례 — spawn yaw=0 은 +X 를 본다, "REAR" 확률 주석 "후방 카메라는 -X 를
        # 보므로" 참고). detect_at_pose(320행)의 yaw_deg 회전과 같은 합성(_quat_mul).
        _, spawn_orn = art.get_world_poses()
        spawn_orn = np.asarray(spawn_orn).reshape(-1)[:4].copy()
        half = math.radians(180.0) * 0.5
        qy = np.array([math.cos(half), 0.0, math.sin(half), 0.0])
        face_neg_x = _quat_mul(spawn_orn, qy)

        # z_bay 정렬: 실측(diag, taskC2-report.md) 결과 측면 뎁스캠은 URDF 설명과 달리
        # "옆으로"가 아니라 로봇 중심선에서 좌/우로 ±0.39m 떨어진 지점에서 바로 아래
        # (world -Y)를 본다 — 다운룩 카메라다. 그래서 로봇 중심선을 트럭 중심선(z=7.075)
        # 에 맞추면 카메라는 항상 트럭 중심 부근의 빈 바닥만 보고(트럭 트레드 반폭
        # ≈0.82m는 카메라 오프셋 0.39m보다 훨씬 커 바퀴가 시야 밖에 남는다), 실측
        # 트로프 0개(n_troughs=0, baseline=0.1726 그대로 유지, 앞선 실행 로그 참고)로
        # 확인됐다. 짐작 대신 두 값을 모두 "지금" 실측해 정렬한다: ① 트럭 좌측 휠(원본
        # 진입 방향 기준 로봇 -X 진행 중 먼저 지나는 쪽이 아니라, 그냥 하나의 트레드 행)
        # 의 실제 world z(FrontLeftWheel/RearLeftWheel 평균), ② LEFT 카메라의
        # base_link 대비 z 오프셋(테스트 위치에 놓고 카메라 prim 의 world z 를 직접
        # 읽어 뺀다). 두 값으로 z_bay = wheel_z - cam_offset_z 를 풀어 카메라가 그
        # 휠의 궤적 바로 위를 지나가게 한다.
        from pxr import UsdGeom as _UsdGeom
        _tc = timeline.get_current_time()
        _wl = _UsdGeom.Xformable(stage.GetPrimAtPath(
            f"{HANDOFF_VEHICLE_ROOT}/FrontLeftWheel")).ComputeLocalToWorldTransform(_tc).ExtractTranslation()
        _wl2 = _UsdGeom.Xformable(stage.GetPrimAtPath(
            f"{HANDOFF_VEHICLE_ROOT}/RearLeftWheel")).ComputeLocalToWorldTransform(_tc).ExtractTranslation()
        wheel_row_z = 0.5 * (float(_wl[2]) + float(_wl2[2]))

        art.set_world_poses(np.array([[x_start, ROBOT_SPAWN_Y, HANDOFF_BAY_CENTER[1]]]),
                            np.array([face_neg_x]))
        for _ in range(15):
            app.update()
        _cam_path_probe = find_side_camera(stage, target, "left")
        _cam_z0 = _UsdGeom.Xformable(stage.GetPrimAtPath(_cam_path_probe)) \
            .ComputeLocalToWorldTransform(timeline.get_current_time()).ExtractTranslation()[2]
        cam_offset_z = float(_cam_z0) - HANDOFF_BAY_CENTER[1]
        z_bay = wheel_row_z - cam_offset_z
        print(f"DEPTH_PROBE_ALIGN wheel_row_z={wheel_row_z:.4f} cam_offset_z={cam_offset_z:.4f} "
              f"z_bay={z_bay:.4f}", flush=True)

        art.set_world_poses(np.array([[x_start, ROBOT_SPAWN_Y, z_bay]]),
                            np.array([face_neg_x]))
        for _ in range(30):
            app.update()

        ctx = depth_setup(stage, timeline, app, target, side="left")
        for _ in range(10):
            app.update()

        gx0, gz0, gyaw0 = gt_pose_xz_yaw(art)
        print(f"DEPTH_PROBE_START target={target} x0={gx0:.3f} z0={gz0:.3f} "
              f"yaw0_deg={math.degrees(gyaw0):.1f} cam={ctx['cam_path']}", flush=True)

        vel_buf = np.zeros(np.asarray(art.get_joint_positions()).reshape(-1).shape,
                           dtype=np.float32)
        cur_tw = (0.0, 0.0, 0.0)
        prev = timeline.get_current_time()
        samples = []   # (step, gx, gz, roi_min|None)
        max_steps = 5000   # 1차 시도(2500)가 x=-8 부근 알 수 없는 저항으로 정체돼 시간 내
                           # 양 축을 다 못 지났다(taskC2-report.md) — 회복 여지를 주려고 늘림.
        for step in range(1, max_steps + 1):
            app.update()
            now = timeline.get_current_time()
            dt = min(0.1, max(0.0, now - prev)); prev = now

            gx, gz, gyaw = gt_pose_xz_yaw(art)
            tgt = (speed, 0.0, 0.0) if gx > x_end else (0.0, 0.0, 0.0)
            cur_tw = slew_twist(cur_tw, tgt, dt, linear_accel=LINEAR_ACCEL,
                                linear_decel=LINEAR_DECEL, angular_accel=ANGULAR_ACCEL)
            omegas = wheel_velocities_from_cmd_vel(*cur_tw)
            vel_buf[...] = 0.0
            for w, om in omegas.items():
                vel_buf[idx[w]] = om
            art.set_joint_velocity_targets(vel_buf)

            roi = depth_roi_min(ctx, roi_frac=DEPTH_ROI_FRAC)
            samples.append((step, gx, gz, roi))
            if step % 15 == 0:
                roi_txt = f"{roi:.4f}" if roi is not None else "None"
                print(f"DEPTH_STREAM step={step} x={gx:.3f} roi_min={roi_txt}", flush=True)

            if gx <= x_end and cur_tw == (0.0, 0.0, 0.0):
                for _ in range(20):
                    app.update()
                    roi = depth_roi_min(ctx, roi_frac=DEPTH_ROI_FRAC)
                    samples.append((step, gx, gz, roi))
                break

        n_samples = len(samples)
        finite_rois = [r for _, _, _, r in samples if r is not None]
        early = [r for _, _, _, r in samples[:60] if r is not None]
        baseline = float(np.median(early)) if early else float("nan")
        min_seen = min(finite_rois) if finite_rois else float("nan")
        z_all = [gz for _, _, gz, _ in samples]
        print(f"DEPTH_PROBE_ZDRIFT z_min={min(z_all):.4f} z_max={max(z_all):.4f} "
              f"z_target={z_bay:.4f}", flush=True)

        # 트로프 개수: baseline-0.05 아래로 내려간 연속 구간을 하나로 센다
        # (depth_stop_detector.DepthStopDetector 의 drop_margin=0.05 관례 재사용).
        n_troughs = 0
        in_trough = False
        thresh = baseline - 0.05 if math.isfinite(baseline) else float("nan")
        for _, _, _, r in samples:
            below = (r is not None) and math.isfinite(thresh) and (r < thresh)
            if below and not in_trough:
                n_troughs += 1
                in_trough = True
            elif not below:
                in_trough = False

        print(f"DEPTH_PROBE_SUMMARY n_samples={n_samples} baseline={baseline:.4f} "
              f"min_seen={min_seen:.4f} n_troughs={n_troughs}", flush=True)

        if headless:
            app.close(); return
        while app.is_running():
            app.update()
        app.close(); return

    if probe == "REARXN":
        # Task 3a 증명: fuse_camera_setup(cam_role="rear") 로 entry_lead 후방캠이
        # 핸드오프 마커 XN(id 31)을 인프로세스로(Isaac 안에서 렌더→검출→측위) 잡아내는지
        # 확인한다. FUSE(위)가 셋업→보정→검출→측위 패턴이다 — 여기선 그 패턴을 후방캠
        # 기하에 맞게 두 지점만 조정한다(fuse_camera_setup/calibrate_tbasecam/
        # detect_at_pose 자체는 건드리지 않는다, 호출부 인자·ctx 필드만 조정):
        #
        #   ① calibrate_tbasecam 은 내부에서 detect_at_pose(...,1.5,0,0,spawn_orn) 로
        #      "마커가 로봇 정면(+X, spawn 방향)"이 되도록 로봇을 놓는다 — 이건 전방캠
        #      전제다. 후방캠은 반대(-X)를 본다(REAR_probe 실측 주석: "마커가 뒤에
        #      오도록 mx+d 에 둔다" — DEBUG_LOG 2026-07-24). 같은 mx-1.5 배치에서
        #      후방캠에 도크 마커가 잡히게 하려면 로봇을 180° 반전한 방향으로 세워야
        #      한다(그러면 로봇 앞은 -X 를 보고 뒤(-로컬X=후방캠 시선)가 +X 를 봐서
        #      도크 마커 쪽을 향한다). spawn_orn 을 그대로 넘기지 않고 180° 돌려 넘긴다.
        #   ② ctx["ref_id"]/["mx"]/["mz"] 는 Step1 요구대로 fuse_camera_setup 안에서는
        #      항상 도크 마커 기준이다(안 건드림). detect_current/localize_pose 는 실제로
        #      ctx["ref_id"] 로 필터링한다(코드 확인 완료) — 도크가 아니라 XN 을 검출하려면
        #      보정이 끝난 뒤 이 호출부에서 ctx["ref_id"] 를 XN id 로 바꿔치기해야 한다.
        #      (Task 3b 브리프도 entry_lead 가 후방캠 하나로 도크→XN 을 순서대로 봐야
        #      한다고 명시한다 — 이 ref_id 전환이 바로 그 메커니즘이다.)
        cam_h = 0.15
        target = "entry_lead"
        art = arts[target]
        ctx = fuse_camera_setup(stage, timeline, app, target, cam_h, cam_role="rear")
        _, spawn_orn = art.get_world_poses()
        spawn_orn = np.asarray(spawn_orn).reshape(-1)[:4].copy()

        def _yaw_quat(base_orn, extra_yaw_deg):
            """base_orn 을 월드 수직축 기준 extra_yaw_deg 만큼 더 돌린 쿼터니언.

            detect_at_pose(320줄)의 yaw_deg 회전과 동일한 관례(_quat_mul(base,qy))다.
            검산: fwd0=(sin ψ0,·,cos ψ0) 에 이 합성을 적용하면 새 yaw = ψ0+extra_yaw_deg
            (필터-yaw convention, ARUCO_PLAN 0절과 일치) — 브리프의 "+X→−Z 는 수직축
            기준 +90°" 힌트와 일치한다.
            """
            half = math.radians(extra_yaw_deg) * 0.5
            qy = np.array([math.cos(half), 0.0, math.sin(half), 0.0])
            return _quat_mul(base_orn, qy)

        # ---- ① 보정: 도크 마커로 T_base_cam(후방캠 마운트, 고정 외부파라미터) 확정 ----
        # calib_orn = spawn_orn 을 180° 반전 — calibrate_tbasecam 내부의 고정 배치
        # (mx-1.5, orn 방향 그대로)에서도 도크 마커가 후방캠 시야에 들어오게 한다.
        calib_orn = _yaw_quat(spawn_orn, 180.0)
        try:
            T_base_cam, cal_name, cal_err = calibrate_tbasecam(
                ctx, art, app, timeline, gt_pose_xz_yaw, calib_orn)
        except RuntimeError as e:
            if "미검출" in str(e):
                print("REARXN_TBASECAM_CAL FAIL: 보정 자세(도크, 180°반전)에서 "
                      "후방캠 마커 미검출 — 반전 기하 재검토", flush=True)
            else:
                print("REARXN_TBASECAM_CAL FAIL: 어떤 광학 규약도 5cm 안에 못 맞춤", flush=True)
            if headless:
                app.close()
            raise
        print(f"REARXN_TBASECAM_CAL best={cal_name} verify_pos_err={cal_err:.4f}m", flush=True)

        # ---- ② 검출 대상을 도크→XN 으로 전환(보정 완료 후에만; fuse_camera_setup 은 안 건드림) ----
        xn_id = read_markers(stage)["XN"]["id"]
        xn_x, xn_z = marker_visual_center(stage, "XN")
        ctx["ref_id"] = xn_id

        # 로봇을 XN 남쪽에 남향(월드 -Z, 필터-yaw≈180°)으로: spawn(+X,yaw90)에서 +90°.
        south_orn = _yaw_quat(spawn_orn, 90.0)

        def _detect_all_ids():
            """진단 전용: ref_id 필터 없이 현재 프레임에서 검출된 모든 마커 id."""
            import cv2
            frame = ctx["rgb_annot"].get_data()
            img = (np.asarray(frame)[:, :, :3] if frame is not None and len(frame) else None)
            if img is None:
                return []
            gray = cv2.cvtColor(np.ascontiguousarray(img), cv2.COLOR_RGB2GRAY)
            det = ctx["aruco_pose"].detect_and_estimate(
                gray, ctx["detector"], ctx["code_size_m"], ctx["K"], ctx["dist"])
            return sorted(int(p.marker_id) for p in det)

        # ---- ③ 근거리 사각(<1.1m, DEBUG_LOG 2026-07-24 REAR 실측) 밖에서 프레이밍될 때까지
        # (거리,좌우) 를 이터레이트한다. yaw 는 위 south_orn 으로 고정(브리프 지시).
        pose = None
        for d in (1.3, 1.2, 1.4, 1.1, 1.5, 1.6, 1.7):
            for lat in (0.0, 0.15, -0.15):
                art.set_world_poses(
                    np.array([[xn_x + lat, ROBOT_SPAWN_Y, xn_z - d]]),
                    np.array([south_orn]))
                for _ in range(30):
                    app.update()
                pose = detect_current(ctx)
                gxx, gzz, gyy = gt_pose_xz_yaw(art)
                seen = [xn_id] if pose is not None else _detect_all_ids()
                print(f"REARXN_TRY d={d:.2f} lat={lat:+.2f} hit={pose is not None} "
                      f"seen_ids={seen} gt=({gxx:.3f},{gzz:.3f}) gt_yaw={math.degrees(gyy):.1f}",
                      flush=True)
                if pose is not None:
                    break
            if pose is not None:
                break

        fix = localize_pose(ctx, pose, T_base_cam) if pose is not None else None
        gx, gz, gyaw = gt_pose_xz_yaw(art)
        if fix is not None:
            fix_err = math.hypot(fix.x - gx, fix.z - gz)
            fix_str = f"({fix.x:.3f},{fix.z:.3f})"
        else:
            fix_err = float("nan")
            fix_str = "(nan,nan)"
        seen_str = str(int(pose.marker_id)) if pose is not None else "none"
        print(f"REARXN_DETECT locked={fix is not None} fix={fix_str} "
              f"marker_seen={seen_str} tbasecam={cal_name} cal_err={cal_err:.4f}", flush=True)
        print(f"REARXN_DIAG xn=({xn_x:.3f},{xn_z:.3f}) gt=({gx:.3f},{gz:.3f}) "
              f"gt_yaw={math.degrees(gyaw):.1f} fix_err_vs_gt_m={fix_err:.4f} "
              f"last_tried_pose=(x={xn_x + lat:.3f},z={xn_z - d:.3f},yaw~180)", flush=True)

        if headless:
            app.close(); return
        while app.is_running():
            app.update()
        app.close(); return

    if probe == "YAWCAL":
        # Phase B 회전 오도 보정 측정(원인: mecanum_drive.YAW_SCALE=1.12 는 wz~0.5
        # 한 동작점에서만 실측 보정된 근사치이고, cmd_vel_from_wheel_velocities 는
        # 그 IK 의 정확한 최소자승 역이라 "명령된" 회전율을 그대로 되돌려줄 뿐 실제
        # 미끄러짐(roller-slip)은 반영하지 않는다 — 그래서 predict_body 가 회전을
        # 과대추정한다. a869b49/MISSIONB_ROT 에서 제자리 90도 회전이 GT 대비 ~31°
        # 어긋나는 것으로 처음 발견됐다(순수 오도 구간, 마커 fix 없음).
        #
        # entry_lead 를 스폰 도크(world x=-3.2, 바닥 위 ROBOT_SPAWN_Y, 접지 유지)
        # 그 자리에서 그대로 열린루프로 제자리 회전시켜 GT(물리) 회전량과 오도(휠
        # 각속도 기반) 회전량을 동시에 적분·비교한다. 텔레포트하지 않는다 — 이전
        # 시도가 로봇을 z=+52.2 로 대피시켜 바닥 밖으로 나가 접지를 잃고 바퀴만
        # 헛돌게 만들어 gt_deg=0.00(무의미) 결과를 낸 실패를 되풀이하지 않기
        # 위함이다. 옆 도크 entry_follow(x≈-1.2, D_OUT_2)는 ~2m 떨어져 있고
        # 회전 중인 로봇의 풋프린트는 ~0.7m 반경이라 제자리 회전으로는 충돌하지
        # 않는다(이 미션의 실제 시나리오 그대로 — 로봇은 도크에서 제자리 회전한다).
        # 구현 메모(1차 시도 이상 실측 — 정직하게 남긴다, report 참조): 처음엔 브리프
        # 문구 그대로 wz_cmd 를 slew 없이 "즉시" set_joint_velocity_targets 했다.
        # 결과가 물리적으로 말이 안 됐다: GT 가 매번 명령 크기(0.6 이든 0.3 이든)와
        # 거의 무관하게 ~90°를 아주 짧은 시간에 돌았고, 심지어 odom 과 부호가
        # 반대였다(scale -2.49~-3.09, spread 22%). 이 코드베이스의 다른 모든 경로
        # (drive_to_pose/FUSE/probe B)는 wz 를 절대 즉시 걸지 않고 항상
        # slew_twist(ANGULAR_ACCEL)로 램프한다 — 즉시-스텝 명령이 4륜에 순간적으로
        # 큰 반대부호 토크를 걸어(damping=1500,max_force=6000 속도드라이브) 정상
        # 미끄러짐이 아니라 튐/불안정 과도응답을 유발한 것으로 보고, 실제 미션이
        # 로봇을 구동하는 것과 동일한 방식(slew_twist)으로 바꿔 재측정한다. drift_m/
        # dy_m(수평 이동/부양) 을 함께 로그해 이 재측정이 실제로 "제자리" 회전인지
        # 진단 근거를 남긴다.
        from mecanum_drive import (wheel_velocities_from_cmd_vel,
                                   cmd_vel_from_wheel_velocities, slew_twist)

        target = "entry_lead"
        art = arts[target]
        idx = wheel_idx[target]
        vel_buf = np.zeros(np.asarray(art.get_joint_positions()).reshape(-1).shape,
                           dtype=np.float32)

        def _spin_measure(wz_cmd, max_deg=90.0, max_steps=4000):
            """wz_cmd 로 정속 제자리회전(slew_twist 가속 램프 — drive_to_pose/FUSE 와
            동일한 명령 방식·ANGULAR_ACCEL), GT/오도 회전각을 매 스텝 동시 적분.

            |ΔGT|>=max_deg 에서 조기 종료(그 전이면 max_steps 소진). 반환:
            (gt_deg, odom_deg, steps, drift_m, dy_m) — 앞 둘은 부호 있는 누적각(도),
            drift_m/dy_m 은 진단용(수평 이동/부양 — 튐·전복이 아니라 정말 제자리
            회전인지 확인).
            """
            cur_tw = (0.0, 0.0, 0.0)
            prev = timeline.get_current_time()
            gx0, gz0, gyaw_prev = gt_pose_xz_yaw(art)
            y0 = float(np.asarray(art.get_world_poses()[0]).reshape(-1)[1])
            gt_deg = 0.0
            odom_deg = 0.0
            steps = 0
            for _ in range(max_steps):
                app.update()
                steps += 1
                now = timeline.get_current_time()
                dt = min(0.1, max(0.0, now - prev)); prev = now

                vel = np.asarray(art.get_joint_velocities()).reshape(-1)
                wv = {w: float(vel[i]) for w, i in idx.items()}
                _, _, odom_wz = cmd_vel_from_wheel_velocities(wv)
                odom_deg += math.degrees(odom_wz * dt)

                _, _, gyaw = gt_pose_xz_yaw(art)
                dyaw = (gyaw - gyaw_prev + math.pi) % (2.0 * math.pi) - math.pi
                gt_deg += math.degrees(dyaw)
                gyaw_prev = gyaw

                cur_tw = slew_twist(cur_tw, (0.0, 0.0, wz_cmd), dt,
                                    linear_accel=LINEAR_ACCEL, linear_decel=LINEAR_DECEL,
                                    angular_accel=ANGULAR_ACCEL)
                omegas = wheel_velocities_from_cmd_vel(*cur_tw)
                vel_buf[...] = 0.0
                for w, om in omegas.items():
                    vel_buf[idx[w]] = om
                art.set_joint_velocity_targets(vel_buf)

                if abs(gt_deg) >= max_deg:
                    break
            gx1, gz1, _ = gt_pose_xz_yaw(art)
            y1 = float(np.asarray(art.get_world_poses()[0]).reshape(-1)[1])
            drift_m = math.hypot(gx1 - gx0, gz1 - gz0)
            # 정지 + 정착(다음 run 이 정지 상태에서 시작하도록).
            vel_buf[...] = 0.0
            art.set_joint_velocity_targets(vel_buf)
            for _ in range(30):
                app.update()
            return gt_deg, odom_deg, steps, drift_m, (y1 - y0)

        runs = (("ccw", 0.6), ("cw", -0.6), ("ccw", 0.3))   # 양방향 + 저속 1개
        scales = []
        aborted = False
        for dir_label, wz_cmd in runs:
            gt_deg, odom_deg, steps, drift_m, dy_m = _spin_measure(wz_cmd)
            if abs(gt_deg) < 5.0:
                # 게이트: GT 가 사실상 안 움직였다 — 로봇이 물리적으로 회전하지
                # 않았다는 뜻(바닥 이탈/접지 상실/미구동 등). scale 을 계산하지
                # 않고 중단한다(브리프 지시 — 억지로 scale 을 내지 않는다).
                print(f"YAWCAL_ABORT dir={dir_label} wz={wz_cmd:+.2f} gt_deg={gt_deg:.2f} "
                      f"odom_deg={odom_deg:.2f} steps={steps} "
                      "reason=gt_not_moving(robot_not_physically_rotating)", flush=True)
                aborted = True
                break
            scale = gt_deg / odom_deg if abs(odom_deg) > 1e-9 else float("nan")
            scales.append(scale)
            print(f"YAWCAL dir={dir_label} wz={wz_cmd:+.2f} gt_deg={gt_deg:.2f} "
                  f"odom_deg={odom_deg:.2f} scale={scale:.4f} steps={steps} "
                  f"drift_m={drift_m:.3f} dy_m={dy_m:.3f}", flush=True)

        if not aborted:
            avg_scale = sum(scales) / len(scales)
            spread_pct = ((max(scales) - min(scales)) / avg_scale * 100.0
                          if avg_scale else float("nan"))
            print(f"YAWCAL_SUMMARY avg_scale={avg_scale:.4f} spread_pct={spread_pct:.2f}",
                  flush=True)

        if headless:
            app.close(); return
        while app.is_running():
            app.update()
        app.close(); return

    if probe == "YAWSTEP":
        # taskBYAW: 회전 기구학(mecanum_drive.SIGN_YAW/YAW_SCALE) 재보정을 위한 열린루프
        # 실측. YAWCAL(위)은 "|ΔGT|>=90도"에서 조기 종료하는데, 90도/180도는 mod 360
        # 에서 각각 -270도/±180도와 구별이 안 돼(-270≡+90, -180≡+180) 결함의 크기·부호를
        # 오독하기 쉽다(끝점만 보면 우연히 맞아 보임 — 사용자가 GUI 로 실제로 본 "90도
        # 명령에 270도 반대방향 회전"이 그 사례). 그래서 여기서는 대신 "고정 지속시간·
        # 매 스텝 wrap180 누적(unwrap)"으로 잰다 — 총 회전량이 커도 랩 모호성 자체가
        # 없다(설계상, 스텝당 각변화가 180도를 넘지 않는 한). 명령은 slew_twist 없이
        # 매 스텝 (0,0,wz_cmd)의 휠 각속도를 그대로 재발행한다 — YAWCAL 코드의 "즉시
        # 스텝은 과도응답을 유발한다"는 우려와 무관하게, 이 probe 의 목적은 바로 그
        # "실제 물리가 명령과 얼마나/어느 방향으로 다른가"를 있는 그대로 재는 것이다
        # (브리프 지시, taskBYAW-report.md 참조).
        from mecanum_drive import (wheel_velocities_from_cmd_vel,
                                   cmd_vel_from_wheel_velocities)

        target = "entry_lead"
        art = arts[target]
        idx = wheel_idx[target]
        vel_buf = np.zeros(np.asarray(art.get_joint_positions()).reshape(-1).shape,
                           dtype=np.float32)
        dur_s = 1.0   # 고정 지속시간(브리프 권장 1.0~1.5s). wz<=0.6 이면 3x 배율을
                      # 가정해도 물리 회전량이 <=~2.5*wz*dur*180/pi 로, wz=0.6 에서
                      # 최악 ~124도 -- 180도 근방(랩 경계)에 닿지 않아 안전하다.

        def _settle(n=30):
            vel_buf[...] = 0.0
            art.set_joint_velocity_targets(vel_buf)
            for _ in range(n):
                app.update()

        def _yaw_step(wz_cmd, dur_s):
            omegas = wheel_velocities_from_cmd_vel(0.0, 0.0, wz_cmd)
            for w, om in omegas.items():
                vel_buf[idx[w]] = om
            gx0, gz0, gyaw_prev = gt_pose_xz_yaw(art)
            t0 = timeline.get_current_time()
            prev = t0
            gt_deg = 0.0
            odom_deg = 0.0
            steps = 0
            while (timeline.get_current_time() - t0) < dur_s:
                art.set_joint_velocity_targets(vel_buf)   # 매 스텝 재발행(slew 없음)
                app.update()
                steps += 1
                now = timeline.get_current_time()
                dt = min(0.1, max(0.0, now - prev)); prev = now

                vel = np.asarray(art.get_joint_velocities()).reshape(-1)
                wv = {w: float(vel[i]) for w, i in idx.items()}
                _, _, odom_wz = cmd_vel_from_wheel_velocities(wv)
                odom_deg += math.degrees(odom_wz * dt)

                _, _, gyaw = gt_pose_xz_yaw(art)
                dyaw = (gyaw - gyaw_prev + math.pi) % (2.0 * math.pi) - math.pi
                gt_deg += math.degrees(dyaw)
                gyaw_prev = gyaw

            gx1, gz1, _ = gt_pose_xz_yaw(art)
            drift_m = math.hypot(gx1 - gx0, gz1 - gz0)
            _settle()
            return gt_deg, odom_deg, steps, drift_m

        ks = []
        for wz_cmd in (0.3, -0.3, 0.6, -0.6):
            gt_deg, odom_deg, steps, drift_m = _yaw_step(wz_cmd, dur_s)
            cmd_deg = math.degrees(wz_cmd * dur_s)
            k_cmd = gt_deg / cmd_deg if abs(cmd_deg) > 1e-9 else float("nan")
            k_odom = gt_deg / odom_deg if abs(odom_deg) > 1e-9 else float("nan")
            ks.append(k_cmd)
            print(f"YAWSTEP wz_cmd={wz_cmd:+.2f} dur_s={dur_s:.2f} "
                  f"cmd_deg={cmd_deg:.2f} odom_deg={odom_deg:.2f} gt_deg={gt_deg:.2f} "
                  f"k_gt_over_cmd={k_cmd:.4f} k_gt_over_odom={k_odom:.4f} "
                  f"steps={steps} drift_m={drift_m:.3f}", flush=True)

        avg_k = sum(ks) / len(ks)
        spread_pct = ((max(ks) - min(ks)) / avg_k * 100.0) if avg_k else float("nan")
        same_sign = all((k > 0) == (avg_k > 0) for k in ks)
        print(f"YAWSTEP_SUMMARY avg_k={avg_k:.4f} spread_pct={spread_pct:.2f} "
              f"same_sign={same_sign}", flush=True)

        if headless:
            app.close(); return
        while app.is_running():
            app.update()
        app.close(); return

    if probe in ("ROTCHK", "ROTCHK180", "ROTCHK45"):
        # 폐루프 회전 검증(YAW_ODOM_SCALE 적용 후 gt_err_deg 가 줄어드는지 확인).
        # entry_lead 를 스폰 도크의 GT 자세로 filt 를 시딩하고(스폰 직후라 GT ==
        # 도크 실좌표 — MISSIONB_ROT 이 쓰는 read_markers 도크좌표 시딩과 동치)
        # rotate_in_place 로 목표각만큼 튼다. 로직은 세 probe 가 완전히 동일하고
        # target_yaw/출력 라벨만 다르다(중복 방지를 위해 한 분기로 합침):
        #   ROTCHK   : 90도(spawn yaw≈90 -> target 0, a869b49/MISSIONB_ROT 과 동일
        #              지오메트리 — 그때 err_yaw_gt≈31.39 실측, 목표는 ≤~4°).
        #   ROTCHK180: 180도(spawn yaw≈90 -> target -90, 즉 90도 회전량의 2배).
        #              Task 3b-harden Item1 — 두 번째 로봇이 180도 재정렬을 써야 해서
        #              추가했다. 이 probe 로 실측한 결과 YAW_ODOM_SCALE=1.10(당시 값, 90도
        #              폐루프 스윕만으로 튜닝됨)은 180도로 일반화되지 않았다(gt_err_deg=
        #              12.61°, 목표 ~4°의 3배 초과) — 그래서 90도·180도를 동시에 만족하는
        #              값을 다시 스윕해 1.12 로 교체했다(전체 스윕 수치·판단 근거는 위
        #              YAW_ODOM_SCALE 정의부 주석 4) 및 taskBharden-report.md 참조). 이제
        #              이 두 probe 는 그 상수가 두 각도 모두에서 계속 ≤~4°를 유지하는지
        #              확인하는 회귀 검증용이다.
        #   ROTCHK45 : 45도(spawn yaw≈90 -> target 45). taskBYAW(2026-07-25) 추가 —
        #              90도·180도는 mod 360 에서 각각 -270도/±180도와 endpoint 가
        #              구별 안 돼(-270≡+90, -540≡+180) YAW_ODOM_SCALE=1.12 라는 순수
        #              폐루프 경험값(위 3)/4) 참조, "물리 상수 아님, 일반화 검증 안 됨"
        #              이라고 그 자신이 경고했다)으로도 우연히 통과했다. mecanum_drive.py
        #              의 회전 기구학 자체를 --probe=YAWSTEP 실측대로 재보정한 뒤에는
        #              YAW_ODOM_SCALE=1.0(무보정)이어도 45도 같은 "mod 360 으로 가려지지
        #              않는" 각도가 맞아야 진짜 고쳐진 것이다 — 그 회귀 방지용 probe.
        #
        # drive_to_pose 는 매 스텝 detect_current(ctx) 를 호출하므로(크래시 방지) 진짜
        # 카메라 ctx(fuse_camera_setup)를 만들되 ref_id 를 존재하지 않는 값으로 바꿔
        # 마커 fix 가 전혀 섞이지 않게 한다 — predict_body 의 YAW_ODOM_SCALE 보정 그
        # 자체만 격리해서 검증하는 것이 목적이다. T_base_cam 도 None 으로 두는데,
        # ref_id 불일치로 detect_current 가 항상 None 을 돌려줘 localize_pose 호출
        # 자체가 없으므로 안전하다(순수 오도 회전 검증).
        cam_h = 0.15
        for a in sys.argv[1:]:
            if a.startswith("--cam-height="):
                cam_h = float(a.split("=", 1)[1])
        target = "entry_lead"
        art = arts[target]
        idx = wheel_idx[target]
        ctx = fuse_camera_setup(stage, timeline, app, target, cam_h)
        ctx["ref_id"] = -1     # 존재하지 않는 id -> detect_current 는 항상 None(순수 오도)

        gx0, gz0, gyaw0 = gt_pose_xz_yaw(art)
        filt = PoseFilter(pos_gain=0.5, yaw_gain=0.9)
        filt.set_pose(gx0, gz0, math.degrees(gyaw0))

        target_yaw = {"ROTCHK": 0.0, "ROTCHK180": -90.0, "ROTCHK45": 45.0}[probe]
        rot_res = rotate_in_place(ctx, art, idx, filt, None, target_yaw)
        fp = filt.pose()
        gx, gz, gyaw = gt_pose_xz_yaw(art)
        gt_yaw_deg = math.degrees(gyaw)
        gt_err_deg = abs((gt_yaw_deg - target_yaw + 180.0) % 360.0 - 180.0)
        # seed_yaw/d_filt_deg/d_gt_deg/implied_scale: 진단용 부가 필드. d_filt_deg(필터가
        # 움직였다고 믿은 양, predict_body 에 이미 현재 YAW_ODOM_SCALE 이 적용된 뒤 값) 대
        # d_gt_deg(실제 GT 가 움직인 양)의 비율이다 — YAW_ODOM_SCALE=1.0(무보정) 상태로
        # 돌리면 "폐루프 안에서 필요한 배율"의 기준선을 바로 보여준다(실측: 0.6514,
        # a869b49 MISSIONB_ROT 역산치 0.6512 와 0.0002 차 독립 수렴). 단, 이 기준선 값을
        # 그대로 YAW_ODOM_SCALE 에 넣으면 폐루프 피드백 때문에 오히려 악화된다(report 참조
        # — 최종 채택값은 이 필드를 이용한 폐루프 스윕으로 별도로 찾았다). 지금
        # YAW_ODOM_SCALE(현재 1.12)로 돌리면 implied_scale≈1 에 가깝게 나오는 것이 정상
        # (filt 와 GT 가 서로 잘 맞아간다는 뜻)이며, 1.12 는 90도·180도 둘 다에서 이
        # 근사가 성립하도록 고른 값이다(위 YAW_ODOM_SCALE 정의부 주석 4 참조).
        seed_yaw_deg = math.degrees(gyaw0)
        d_filt = fp[2] - seed_yaw_deg
        d_gt = gt_yaw_deg - seed_yaw_deg
        implied_scale = (d_gt / d_filt) if abs(d_filt) > 1e-6 else float("nan")
        print(f"{probe} target_yaw={target_yaw:.0f} filt_yaw={fp[2]:.2f} "
              f"gt_yaw={gt_yaw_deg:.2f} gt_err_deg={gt_err_deg:.2f} "
              f"reached={rot_res['reached']} steps={rot_res['steps']} "
              f"n_fix={rot_res['n_fix']} seed_yaw={seed_yaw_deg:.2f} "
              f"d_filt_deg={d_filt:.2f} d_gt_deg={d_gt:.2f} implied_scale={implied_scale:.4f} "
              f"swept_gt_deg={rot_res['swept_gt_deg']:.2f}",
              flush=True)

        if headless:
            app.close(); return
        while app.is_running():
            app.update()
        app.close(); return

    if probe is None and mission == "B":
        # Mission Phase B, Task 3b-2(→ redo): 입차팀 두 대(entry_lead=후축, entry_follow=전축)
        # 완주 + 스태거 + 합산 결과. Task 3b-1(entry_follow 단독: 도크체크(후방캠)->
        # 90도 회전->XN정렬(전방캠))의 인라인 로직을 Part 1 에서 재사용 가능한 헬퍼
        # (_mission_setup/_run_entry_follow_b)로 추출했다 — 동작 자체는 바꾸지 않았고,
        # 리팩터 직후 entry_follow 단독 재검증으로 xn_locked=True/ok=True 재현을 확인했다
        # (report 참조, 이 파일 diff 로는 확인 불가 — 검증 로그가 근거다).
        #
        # entry_lead(후축) redo(사용자 결정, e0666c2 WIP 대체): 도크체크는 그대로 REAR
        # 카메라(ref_id=21)지만, XN정렬은 REAR 가 아니라 **FRONT 카메라**(ref_id=31)로
        # 한다. e0666c2 WIP 는 "도크체크·XN정렬 모두 REAR 하나로" 시도했는데, 그러려면
        # 도크체크(북향 필요)와 XN정렬(REAR 로 북쪽의 XN 을 보려면 로봇이 남향이어야
        # 함) 사이에 제자리 180도 재정렬이 필요했다 — mecanum 제자리회전은 명령
        # 지속시간에 비선형으로 반응해(YAW_ODOM_SCALE 정의부 주석 4 참조) 180도가
        # 간헐적으로 수렴 실패했다(실측: err_yaw_gt 최대 96°, target=(-4.200,5.575,
        # 180.0) 인데 n_fix=0 — 이 실패가 이 redo 의 근거다). 사용자 결정: 180도
        # 회전을 아예 없애고 entry_lead 도 entry_follow 와 똑같이 "북향 유지 + 전방캠
        # 으로 XN 을 정면에서" 검출한다 — entry_follow 가 이미 3/3 로 안정적으로 검증한
        # 바로 그 지오메트리·카메라 조합을 재사용하는 것이라 저위험이다(entry_lead 는
        # 도크가 다르고(D_OUT_1, id 21) 종점에 충돌회피용 x 오프셋이 하나 더 붙는다는
        # 점만 entry_follow 와 다르다). 안무: 스폰(yaw90) -> 제자리 90도 회전(북향,
        # 후방캠은 남향=도크 쪽) -> 도크체크 북진(REAR, ref_id=21) -> XN 정렬 2단계
        # (FRONT, ref_id=31 — x 정렬 후 순수 북진, entry_follow Step4 와 동일 패턴:
        # 대각선 진입은 검출창에서 횡오차가 남아 n_fix=0 을 낸 실측 때문에 피한다) ->
        # 충돌회피 x 오프셋으로 최종 대기자세(아래 HARD REQUIREMENT 절 및
        # _run_entry_lead_b 참조). 로봇은 XN 정렬부터 끝까지 계속 북향(yaw=0)이다 —
        # 더 이상 180도로 끝나지 않는다.
        #
        # 순서(사용자 스펙): entry_lead(늦게 입차하지만 먼저 자리를 잡아야 하는 쪽)가
        # 먼저 완주하고, 그 다음 entry_follow 가 시작한다(스태거). _mission_setup 은
        # 호출 시점의 sibling 자세를 캡처했다가 그대로 복원하므로(도크 고정좌표가
        # 아니라 "지금" 자세), entry_follow 차례가 됐을 때 entry_lead 는 이미 도크가
        # 아니라 자신의 최종 대기자세에 있고 그 자세로 정확히 복원된다.

        def _yaw_quat(base_orn, extra_yaw_deg):
            """base_orn 을 월드 수직축 기준 extra_yaw_deg 만큼 더 돌린 쿼터니언
            (Task3a --probe=REARXN 이 확립한 패턴 재사용 — detect_at_pose 의 yaw_deg
            합성과 동일 관례: 새 yaw = ψ0+extra_yaw_deg)."""
            half = math.radians(extra_yaw_deg) * 0.5
            qy = np.array([math.cos(half), 0.0, math.sin(half), 0.0])
            return _quat_mul(base_orn, qy)

        def _calibrate_tbasecam_yawchecked(ctx, art, app, timeline, gt_fn, test_orn,
                                           cam_label):
            """calibrate_tbasecam(공용 함수, 370줄대, 미변경)의 로컬 사본 + yaw 검증.

            실측(Task 3b-1 에서 발견): 원본은 후보(I/X180/Y180/Z180) 중 **위치오차만**으로
            고른다(e=hypot(fix.x-gx,fix.z-gz)) — yaw 는 전혀 비교하지 않는다. 후방캠에서
            이 때문에 위치는 정확(X180, err 0.0062m, Task3a REARXN 과 동일)한데 yaw 는
            GT 와 107° 어긋난 후보가 선택됐다(MISSIONB_DOCKCHECK err_yaw_gt=107.27 로
            실측 — filt 는 40회 fix 로 이 틀린 yaw 에 확신을 갖고 수렴해, 이후 모든 주행이
            엉뚱한 방향으로 나갔다). 원본 calibrate_tbasecam 은 FUSE/M5/REAR/REARXN 회귀
            방지를 위해 건드리지 않고, 이 로컬 사본만 후보마다 위치·yaw 오차를 함께 계산해
            **둘 다** 허용치 안인 후보를 우선 선택한다(없으면 원본과 동일하게 위치 최우선
            폴백 + 경고 로그). GT 는 여기서도 정적 1회 보정 검증에만 쓴다(제어 아님).
            Task 3b-2: entry_lead 도 재사용하도록 공용 스코프(미션 블록 최상단)로
            옮겼다 — 함수 본문은 미변경.
            """
            def usd_to_np(gf_m):
                m = np.array([[gf_m[i][j] for j in range(4)] for i in range(4)], dtype=np.float64)
                return m.T
            pose0 = detect_at_pose(ctx, art, app, 1.5, 0.0, 0.0, test_orn)
            if pose0 is None:
                raise RuntimeError(f"보정 자세에서 마커 미검출(cam={cam_label})")
            bpos, born = art.get_world_poses()
            bp = np.asarray(bpos).reshape(-1)[:3]
            bw, bx, by, bz = (float(v) for v in np.asarray(born).reshape(-1)[:4])

            def quat_to_R(w, x, y, z):
                return np.array([
                    [1-2*(y*y+z*z), 2*(x*y-w*z),   2*(x*z+w*y)],
                    [2*(x*y+w*z),   1-2*(x*x+z*z), 2*(y*z-w*x)],
                    [2*(x*z-w*y),   2*(y*z+w*x),   1-2*(x*x+y*y)]], dtype=np.float64)
            T_world_base = np.eye(4)
            T_world_base[:3, :3] = quat_to_R(bw, bx, by, bz)
            T_world_base[:3, 3] = bp
            T_world_camusd = usd_to_np(ctx["cam_xf"].ComputeLocalToWorldTransform(
                timeline.get_current_time()))
            candidates = {"I": np.diag([1.0, 1.0, 1.0, 1.0]),
                          "X180": np.diag([1.0, -1.0, -1.0, 1.0]),
                          "Y180": np.diag([-1.0, 1.0, -1.0, 1.0]),
                          "Z180": np.diag([-1.0, -1.0, 1.0, 1.0])}
            gx, gz, gyaw = gt_fn(art)
            gyaw_deg = math.degrees(gyaw)
            scored = []
            for name, C in candidates.items():
                T_base_cam = np.linalg.inv(T_world_base) @ T_world_camusd @ C
                fix = localize_pose(ctx, pose0, T_base_cam)
                if fix is None:
                    continue
                e_pos = math.hypot(fix.x - gx, fix.z - gz)
                e_yaw = abs((fix.yaw_deg - gyaw_deg + 180.0) % 360.0 - 180.0)
                scored.append((name, T_base_cam, e_pos, e_yaw))
                print(f"MISSIONB_CALCANDIDATE cam={cam_label} name={name} "
                      f"e_pos={e_pos:.4f} e_yaw={e_yaw:.2f}", flush=True)
            both_ok = [s for s in scored if s[2] < 0.05 and s[3] < 5.0]
            if both_ok:
                both_ok.sort(key=lambda s: s[2])
                name, T_base_cam, e_pos, e_yaw = both_ok[0]
                return T_base_cam, name, e_pos
            pos_ok = [s for s in scored if s[2] < 0.05]
            if not pos_ok:
                raise RuntimeError(
                    f"어떤 광학 규약도 위치 5cm 안에 못 맞춤(cam={cam_label})")
            pos_ok.sort(key=lambda s: s[2])
            name, T_base_cam, e_pos, e_yaw = pos_ok[0]
            print(f"MISSIONB_CAL_YAW_FALLBACK cam={cam_label} name={name} "
                  f"e_pos={e_pos:.4f} e_yaw={e_yaw:.2f} — 위치·yaw 모두 통과하는 후보가 "
                  "없어 위치 최우선으로 폴백(원본 calibrate_tbasecam 과 동일 기준)",
                  flush=True)
            return T_base_cam, name, e_pos

        def _mission_setup(target, sibling_id, *, need_front, need_rear, cam_h):
            """Task 3b-2 Part 1: entry_follow/entry_lead 공용 세팅 헬퍼.

            Task 3b-1 이 entry_follow 하나만을 위해 인라인으로 했던 일 — ① sibling 을
            임시 대피(멀리 텔레포트)시킨 채 필요한 카메라(전방/후방)를
            _calibrate_tbasecam_yawchecked 로 보정, ② (후방캠이 있으면) 도크 스윕(반복
            텔레포트+정착 — Task 3b-harden Item2 실측: "반복 텔레포트 자체"가 다음
            회전의 오도 안정성에 필요하다는 게 근본원인으로 좁혀졌다), ③ sibling 을
            호출 시점 자세로 복원, ④ target 을 자기 도크 스폰 자세로 되돌리고 filt 를
            도크 좌표로 시딩 — 을 target/sibling/need_front/need_rear 로 매개변수화한
            것이다. 두 로봇이 같은 도크 열(z 는 거의 동일, x 만 다름)에 있어 한쪽의
            보정 테스트자세(대상 도크에서 x 로 1.5m)가 다른쪽 도크와 지나치게 가까워질
            수 있다(Task 3b-1 §3.1 실측: entry_follow 보정자세는 entry_lead 도크와
            0.86m — 그래서 sibling 대피가 필요했다. 반대로 entry_lead 자신의 보정자세는
            entry_follow 도크와 3.57m 로 원래 충분히 멀지만, 헬퍼는 두 호출 모두 같은
            패턴을 획일 적용한다 — 단순하고 안전한 쪽을 택함, 매 호출 비용은 텔레포트
            수 프레임뿐).

            반환 dict: art, idx, filt(도크로 시딩 완료), rear_ctx/T_rear(need_rear 이면
            둘 다 값, 아니면 둘 다 None), front_ctx/T_front(need_front 대칭),
            dock_x/z/id, dock_decal(x,z), xn_x/z/id, xn_decal(x,z), spawn_orn,
            spawn_yaw_deg.
            """
            target_art = arts[target]
            idx = wheel_idx[target]

            dock_serves = sm.ROBOT_DOCK_MARKER[target]
            dock = read_markers(stage)[dock_serves]
            dock_x, dock_z, dock_id = dock["x"], dock["z"], dock["id"]
            xn = read_markers(stage)["XN"]
            xn_x, xn_z, xn_id = xn["x"], xn["z"], xn["id"]
            # 도크는 aruco:position(속성=스폰좌표) 과 데칼(실제 렌더 위치)이 z 로
            # 0.7m 어긋난다(Task 3b-1 실측, USD 직접 확인) — 카메라가 "보는" 건
            # 데칼이므로 검출거리 산정엔 데칼을 쓴다. XN 은 크로싱 마커라 오프셋이
            # 없다(속성=데칼, Task 3b-1 MISSIONB_GEOMETRY 로 재확인).
            dock_decal = marker_visual_center(stage, dock_serves)
            xn_decal = marker_visual_center(stage, "XN")

            _, spawn_orn = target_art.get_world_poses()
            spawn_orn = np.asarray(spawn_orn).reshape(-1)[:4].copy()
            # 스폰 자세는 모든 로봇에 build_stage 가 동일하게 고정 AddRotateXOp(-90)
            # 만 적용한다(Task2 스켈레톤 근거, YAWCAL/ROTCHK 도 entry_lead 로 동일 값
            # 재확인) -> 로컬 +X(전방)가 그대로 월드 +X 로 나옴 -> yaw=atan2(1,0)=90도.
            spawn_yaw_deg = 90.0

            # ---- ① sibling 대피(근접-도크 보정 회귀 회피, Task 3b-1 §3.1) ----
            # Articulation.initialize() 이후 SetActive(False) 는 세그폴트 위험이 있어
            # (probe=B 주석 참조) 쓸 수 없다 — 대신 멀리 텔레포트했다가 정확히
            # 복원한다. 호출 시점의 "현재" 자세를 캡처하므로(도크 고정좌표가 아니라),
            # 스태거로 sibling 이 이미 자기 미션을 끝내고 다른 자세에 있어도 옳게
            # 복원된다.
            sib_art = arts.get(sibling_id)
            sib_pos0 = sib_orn0 = None
            if sib_art is not None:
                sib_pos0, sib_orn0 = sib_art.get_world_poses()
                sib_pos0 = np.asarray(sib_pos0).reshape(1, 3).copy()
                sib_orn0 = np.asarray(sib_orn0).reshape(1, 4).copy()
                away = sib_pos0.copy()
                away[0, 2] += 50.0                # z 로 멀리(사이트 밖) 대피
                sib_art.set_world_poses(away, sib_orn0)
                for _ in range(5):
                    app.update()
                print(f"MISSIONB_PARK_CLEAR robot={sibling_id} away_z={float(away[0, 2]):.2f}",
                      flush=True)

            # ---- ② 카메라 보정(요청된 것만) ----
            rear_ctx = T_rear = front_ctx = T_front = None
            if need_rear:
                rear_ctx = fuse_camera_setup(stage, timeline, app, target, cam_h, cam_role="rear")
                # 후방캠은 calibrate 내부 detect_at_pose 의 "마커가 로봇 정면" 전제와
                # 반대를 보므로 180도 반전한 자세를 넘긴다(Task3a REARXN 확립).
                calib_orn = _yaw_quat(spawn_orn, 180.0)
                try:
                    T_rear, rear_cal_name, rear_cal_err = _calibrate_tbasecam_yawchecked(
                        rear_ctx, target_art, app, timeline, gt_pose_xz_yaw, calib_orn, "rear")
                except RuntimeError as e:
                    if "미검출" in str(e):
                        print(f"MISSIONB_TBASECAM_CAL FAIL robot={target} cam=rear: "
                              "보정 자세(도크, 180°반전)에서 마커 미검출", flush=True)
                    else:
                        print(f"MISSIONB_TBASECAM_CAL FAIL robot={target} cam=rear: "
                              f"어떤 광학 규약도 5cm 안에 못 맞춤 ({e})", flush=True)
                    if headless:
                        app.close()
                    raise
                rear_ctx["ref_id"] = dock_id                  # 도크 검출(기본값과 동일, 명시)
                print(f"MISSIONB_TBASECAM_CAL robot={target} cam=rear best={rear_cal_name} "
                      f"verify_pos_err={rear_cal_err:.4f}m", flush=True)

            if need_front:
                front_ctx = fuse_camera_setup(stage, timeline, app, target, cam_h, cam_role="front")
                try:
                    T_front, front_cal_name, front_cal_err = _calibrate_tbasecam_yawchecked(
                        front_ctx, target_art, app, timeline, gt_pose_xz_yaw, spawn_orn, "front")
                except RuntimeError as e:
                    if "미검출" in str(e):
                        print(f"MISSIONB_TBASECAM_CAL FAIL robot={target} cam=front: "
                              "보정 자세에서 마커 미검출", flush=True)
                    else:
                        print(f"MISSIONB_TBASECAM_CAL FAIL robot={target} cam=front: "
                              f"어떤 광학 규약도 5cm 안에 못 맞춤 ({e})", flush=True)
                    if headless:
                        app.close()
                    raise
                front_ctx["ref_id"] = xn_id                    # 보정 후에만 XN 검출로 전환
                print(f"MISSIONB_TBASECAM_CAL robot={target} cam=front best={front_cal_name} "
                      f"verify_pos_err={front_cal_err:.4f}m", flush=True)

            # ---- ③ 도크 스윕(Task 3b-harden Item2 — "반복 텔레포트+정착 자체"가 다음
            # 회전의 오도 안정성에 필요하다는 게 실측으로 좁혀진 근본원인; 12회 유지,
            # report 근거는 taskBharden-report.md). 후방캠이 있을 때만(현재 두 호출
            # 모두 need_rear=True 라 항상 실행된다) — REAR 가 없는 가상의 전방-전용
            # 호출자는 이 스윕의 북향/후방-확인 기하가 맞지 않아 지원하지 않는다(현재
            # 호출자 없음, 필요해지면 별도로 설계).
            if rear_ctx is not None:
                north_orn = _yaw_quat(spawn_orn, -90.0)   # yaw=90(스폰)-90=0(북향)
                for _d in (1.0, 1.1, 1.2, 1.3, 1.4, 1.5, 1.6, 1.7, 1.8, 1.9, 2.0, 2.2):
                    target_art.set_world_poses(
                        np.array([[dock_x, ROBOT_SPAWN_Y, dock_decal[1] + _d]]),
                        np.array([north_orn]))
                    for _ in range(20):
                        app.update()
                    _hit = detect_current(rear_ctx)
                    print(f"MISSIONB_DOCKSWEEP robot={target} d={_d:.2f} hit={_hit is not None}",
                          flush=True)

            # ---- ④ sibling 복원(대피시켰던 경우만) ----
            if sib_art is not None:
                sib_art.set_world_poses(sib_pos0, sib_orn0)
                for _ in range(5):
                    app.update()
                print(f"MISSIONB_PARK_RESTORE robot={sibling_id}", flush=True)

            print(f"MISSIONB_GEOMETRY robot={target} dock_attr=({dock_x:.3f},{dock_z:.3f}) "
                  f"dock_decal=({dock_decal[0]:.3f},{dock_decal[1]:.3f}) "
                  f"xn_attr=({xn_x:.3f},{xn_z:.3f}) "
                  f"xn_decal=({xn_decal[0]:.3f},{xn_decal[1]:.3f})", flush=True)

            # ---- ⑤ 도크 스폰 자세로 복귀 + filt 시딩 ----
            # 보정/스윕이 로봇을 텔레포트했으므로 도크 스폰 자세(aruco:position 속성 —
            # build_stage 가 실제로 이 좌표에 스폰했다, GT 아님)로 되돌린 뒤 필터를
            # 시딩한다. yaw_gain 을 기본값(0.5)보다 훨씬 높게 둔다 — 제자리 회전 직후
            # (마커 fix 전혀 없음, 순수 오도)만으로 GT 와 최대 ~31° 어긋난 실측
            # (MISSIONB_ROT, mecanum_drive.py "in-place yaw is roller-slip dominated")
            # 때문에 yaw 는 오도를 신뢰할 근거가 없다 — fix 가 잡히면 오도 예측을 거의
            # 덮어쓰도록 올려, 후속 세그먼트가 회전 오차를 오래 끌고 가지 않게 한다.
            target_art.set_world_poses(np.array([[dock_x, ROBOT_SPAWN_Y, dock_z]]),
                                       np.array([spawn_orn]))
            for _ in range(30):
                app.update()
            filt = PoseFilter(pos_gain=0.5, yaw_gain=0.9)
            filt.set_pose(dock_x, dock_z, spawn_yaw_deg)

            return {
                "art": target_art, "idx": idx, "filt": filt,
                "rear_ctx": rear_ctx, "T_rear": T_rear,
                "front_ctx": front_ctx, "T_front": T_front,
                "dock_x": dock_x, "dock_z": dock_z, "dock_id": dock_id,
                "dock_decal": dock_decal,
                "xn_x": xn_x, "xn_z": xn_z, "xn_id": xn_id, "xn_decal": xn_decal,
                "spawn_orn": spawn_orn, "spawn_yaw_deg": spawn_yaw_deg,
            }

        def _run_entry_follow_b(setup, *, seg1_standoff, xn_standoff, align_pos_tol):
            """Task 3b-1 entry_follow(전축) 안무 — 리팩터(Part 1): 로직은 미변경, setup
            dict 에서 값을 읽도록 배선만 바꿨다(도크체크=REAR, XN정렬=FRONT 2단계)."""
            target = "entry_follow"
            art, idx, filt = setup["art"], setup["idx"], setup["filt"]
            rear_ctx, T_rear = setup["rear_ctx"], setup["T_rear"]
            front_ctx, T_front = setup["front_ctx"], setup["T_front"]
            dock_x, dock_decal = setup["dock_x"], setup["dock_decal"]
            xn_x, xn_z = setup["xn_x"], setup["xn_z"]

            # ---- Step 2: 제자리 90도 회전(+X -> +Z, yaw 90 -> 0, 북향) ----
            # front_ctx(ref_id=XN)로 돌지만 이 시점엔 XN 이 멀어(도크 근방) 보정은
            # 기대하지 않는다 — filt 는 시드+오도로 충분(브리프 지시). 마커 fix 가
            # 전혀 없는 순수 오도(회전) 구간이라 GT 와 비교하면 상당한 오차가 실측됐다
            # (err_yaw_gt≈31°, mecanum_drive.YAW_SCALE 자체의 상수 보정 오차로 보임 —
            # mecanum_drive.py/drive_to_pose 는 공용 파일이라 이 태스크에서는 수정하지
            # 않는다).
            rot_res = rotate_in_place(front_ctx, art, idx, filt, T_front, 0.0)
            fp = filt.pose()
            print(f"MISSIONB_ROT robot={target} yaw={fp[2]:.2f} reached={rot_res['reached']} "
                  f"steps={rot_res['steps']} err_pos_gt={rot_res['err_pos_gt']:.4f} "
                  f"err_yaw_gt={rot_res['err_yaw_gt']:.2f}", flush=True)

            # ---- Step 3: 세그먼트1 — 도크체크 주행(후방 ctx). x 는 도크와 동일하게
            # 유지해 후방캠이 뒤의 도크 데칼을 프레임 중앙 부근에 유지한다.
            # correct_yaw=False: 후방캠은 도크를 비스듬히/원거리에서 보기 때문에 yaw
            # 관측 노이즈가 크다 — yaw 는 Step2 의 오도값을 그대로 믿고, 마커 fix 는
            # 위치(x,z)만 반영한다.
            seg1_target = (dock_x, dock_decal[1] + seg1_standoff, 0.0)
            seg1 = drive_to_pose(rear_ctx, art, idx, filt, T_rear, seg1_target,
                                 correct_yaw=False, pos_tol=align_pos_tol)
            dock_seen = seg1["n_fix"] > 0
            fp = filt.pose()
            print(f"MISSIONB_DOCKCHECK robot={target} dock_seen={dock_seen} "
                  f"filt=({fp[0]:.3f},{fp[1]:.3f},{fp[2]:.2f}) n_fix={seg1['n_fix']} "
                  f"reached={seg1['reached']} steps={seg1['steps']} "
                  f"target=({seg1_target[0]:.3f},{seg1_target[1]:.3f},{seg1_target[2]:.1f}) "
                  f"err_pos_gt={seg1['err_pos_gt']:.4f} err_yaw_gt={seg1['err_yaw_gt']:.2f}",
                  flush=True)

            # ---- Step 4: 세그먼트2 — XN 정렬 주행(전방 ctx). XN 남쪽 standoff, 북향
            # (사각 밖). 2단계로 나눈다: 4a 먼저 x 만 XN 에 맞추고(이 시점 z 는 아직
            # 검출창 밖이라 문제 없음), 4b 에서 x=xn_x 고정한 채 순수 북진해 검출창을
            # 통과시킨다(대각선 주행은 검출창 진입 시 횡오차가 남아 n_fix=0 이 났던
            # 실측 때문에 피한다). correct_yaw=False: 회전은 Step2 하나뿐이고 그 오도
            # yaw 는 이미 정확하다 — 4a/4b 모두 마커 fix 는 위치만 반영.
            fp1 = filt.pose()
            seg2a_target = (xn_x, fp1[1], 0.0)
            seg2a = drive_to_pose(front_ctx, art, idx, filt, T_front, seg2a_target,
                                  correct_yaw=False, pos_tol=align_pos_tol)
            seg2_target = (xn_x, xn_z - xn_standoff, 0.0)
            seg2b = drive_to_pose(front_ctx, art, idx, filt, T_front, seg2_target,
                                  correct_yaw=False, pos_tol=align_pos_tol)
            n_fix2 = seg2a["n_fix"] + seg2b["n_fix"]
            xn_locked = n_fix2 > 0
            fp = filt.pose()
            print(f"MISSIONB_XNALIGN robot={target} n_fix_a={seg2a['n_fix']} "
                  f"n_fix_b={seg2b['n_fix']} reached_a={seg2a['reached']} "
                  f"reached_b={seg2b['reached']} filt_a=({seg2a['final_filt'][0]:.3f},"
                  f"{seg2a['final_filt'][1]:.3f},{seg2a['final_filt'][2]:.2f}) "
                  f"target_a=({seg2a_target[0]:.3f},{seg2a_target[1]:.3f},{seg2a_target[2]:.1f}) "
                  f"err_pos_gt_a={seg2a['err_pos_gt']:.4f} err_pos_gt_b={seg2b['err_pos_gt']:.4f}",
                  flush=True)

            # ---- Step 5: 완료 토큰 ----
            # gt_pos/gt_yaw: 충돌회피 여유(HARD REQUIREMENT)를 실측으로 검증하기 위한
            # 리포팅 전용 GT — filt/제어 경로에는 영향 없다(gt_pose_xz_yaw 호출만
            # 추가, drive_to_pose 호출부는 미변경).
            gx, gz, gyaw = gt_pose_xz_yaw(art)
            print(f"MISSIONB_DONE robot={target} role=front_axle xn_locked={xn_locked} "
                  f"reached={seg2b['reached']} pos=({fp[0]:.3f},{fp[1]:.3f}) yaw={fp[2]:.2f} "
                  f"gt_pos=({gx:.3f},{gz:.3f}) gt_yaw={math.degrees(gyaw):.2f} "
                  f"err_pos_gt={seg2b['err_pos_gt']:.4f} n_fix={n_fix2} "
                  f"target=({seg2_target[0]:.3f},{seg2_target[1]:.3f},{seg2_target[2]:.1f})",
                  flush=True)
            return {"xn_locked": xn_locked, "reached": seg2b["reached"],
                    "gt_pos": (gx, gz), "filt_pos": (fp[0], fp[1])}

        def _run_entry_lead_b(setup, *, dockcheck_standoff, xn_standoff,
                              final_x_offset, align_pos_tol):
            """entry_lead(후축) 안무 — redo(사용자 결정): XN정렬을 REAR 가 아니라
            **FRONT 카메라**(ref_id=31)로 한다. 구조적으로 entry_follow 와 동일하다 —
            스폰(yaw90) -> 90도 회전(북향) -> 도크체크(REAR, ref_id=21) -> XN정렬 2단계
            (FRONT, ref_id=31, x 정렬 후 순수 북진) -> 충돌회피 x 오프셋. entry_follow
            와 다른 점은 도크(D_OUT_1, id 21)와 종점의 +x 오프셋뿐이다.

            이전 버전(e0666c2 WIP)은 REAR 카메라 하나로 도크체크·XN정렬을 모두
            하려고 제자리 180도 재정렬을 끼워 넣었는데, mecanum 제자리회전이 180도
            에서 간헐적으로 수렴하지 않아(비선형 동역학, YAW_ODOM_SCALE 정의부 주석
            4 참조) XN 을 놓치는 경우가 실측됐다(n_fix=0, err_yaw_gt 최대 96°).
            180도 회전을 아예 없애고 로봇이 XN정렬부터 끝까지 계속 북향(yaw=0)을
            유지한 채 FRONT 카메라로 XN 을 정면에서 보게 하면, entry_follow 가 이미
            3/3 검증한 것과 동일한 지오메트리·카메라 조합이 된다 — 그래서 이 redo 를
            저위험으로 본다. 최종 그랩 지오메트리(반대편 축 배치)는 Phase C 에서 실제
            차량으로 다듬는다 — 여기서는 XN 정렬(충돌없는 대기자세)까지만 다룬다.
            """
            target = "entry_lead"
            art, idx, filt = setup["art"], setup["idx"], setup["filt"]
            rear_ctx, T_rear = setup["rear_ctx"], setup["T_rear"]
            front_ctx, T_front = setup["front_ctx"], setup["T_front"]
            dock_x, dock_decal = setup["dock_x"], setup["dock_decal"]
            xn_x, xn_z = setup["xn_x"], setup["xn_z"]

            # ---- Step 2: 제자리 90도 회전(+X -> +Z, yaw 90 -> 0, 북향). 후방캠은
            # heading 반대인 남향(도크 쪽)이 된다. entry_follow Step2 와 마찬가지로
            # 이 시점엔 마커 fix 를 기대하지 않는다(filt 는 시드+오도로 충분). ----
            rot_res = rotate_in_place(rear_ctx, art, idx, filt, T_rear, 0.0)
            fp = filt.pose()
            print(f"MISSIONB_ROT robot={target} yaw={fp[2]:.2f} reached={rot_res['reached']} "
                  f"steps={rot_res['steps']} err_pos_gt={rot_res['err_pos_gt']:.4f} "
                  f"err_yaw_gt={rot_res['err_yaw_gt']:.2f}", flush=True)

            # ---- Step 3: 도크체크 — 북진(+Z, x 는 도크와 동일 유지), 후방캠이 남쪽의
            # 도크 데칼을 본다(ref_id=21, entry_follow Step3 와 동일 패턴). 미변경.
            # correct_yaw=False(위치전용) — Step2 오도 yaw 를 그대로 믿는다.
            seg1_target = (dock_x, dock_decal[1] + dockcheck_standoff, 0.0)
            seg1 = drive_to_pose(rear_ctx, art, idx, filt, T_rear, seg1_target,
                                 correct_yaw=False, pos_tol=align_pos_tol)
            dock_seen = seg1["n_fix"] > 0
            fp = filt.pose()
            print(f"MISSIONB_DOCKCHECK robot={target} dock_seen={dock_seen} "
                  f"filt=({fp[0]:.3f},{fp[1]:.3f},{fp[2]:.2f}) n_fix={seg1['n_fix']} "
                  f"reached={seg1['reached']} steps={seg1['steps']} "
                  f"target=({seg1_target[0]:.3f},{seg1_target[1]:.3f},{seg1_target[2]:.1f}) "
                  f"err_pos_gt={seg1['err_pos_gt']:.4f} err_yaw_gt={seg1['err_yaw_gt']:.2f}",
                  flush=True)

            # ---- Step 4(THE CHANGE): XN 정렬 — REAR 가 아니라 FRONT 카메라로 본다
            # (ref_id=31, _mission_setup 이 need_front=True 보정 후 이미 xn_id 로
            # 전환해 뒀다). 180도 재정렬 없이 로봇은 계속 북향(yaw=0) — 전방캠이 정면의
            # XN 을 그대로 본다. entry_follow Step4 와 동일하게 2단계로 나눈다: 4a 먼저
            # x 만 xn_x 에 맞추고(이 시점 z 는 아직 도크체크 z 라 검출창 밖 — fix 없음이
            # 정상), 4b 에서 x=xn_x 고정한 채 순수 북진해 검출창을 통과시킨다(대각선
            # 주행은 검출창 진입 시 횡오차가 남아 n_fix=0 이 났던 entry_follow 실측
            # 때문에 피한다 — body_twist_toward 는 yaw 고정 상태에서 dx,dz 를 동시에
            # 좌/전진으로 섞어 몰기 때문에, 목표를 한 번에 주면 대각선으로 접근하게
            # 된다). correct_yaw=False — 마커 fix 는 위치만 반영, yaw 는 Step2 오도값
            # 유지.
            fp1 = filt.pose()
            seg2a_target = (xn_x, fp1[1], 0.0)
            seg2a = drive_to_pose(front_ctx, art, idx, filt, T_front, seg2a_target,
                                  correct_yaw=False, pos_tol=align_pos_tol)
            seg2_target = (xn_x, xn_z - xn_standoff, 0.0)
            seg2b = drive_to_pose(front_ctx, art, idx, filt, T_front, seg2_target,
                                  correct_yaw=False, pos_tol=align_pos_tol)
            n_fix2 = seg2a["n_fix"] + seg2b["n_fix"]
            xn_locked = n_fix2 > 0
            fp = filt.pose()
            print(f"MISSIONB_XNALIGN robot={target} n_fix_a={seg2a['n_fix']} "
                  f"n_fix_b={seg2b['n_fix']} reached_a={seg2a['reached']} "
                  f"reached_b={seg2b['reached']} filt_a=({seg2a['final_filt'][0]:.3f},"
                  f"{seg2a['final_filt'][1]:.3f},{seg2a['final_filt'][2]:.2f}) "
                  f"target_a=({seg2a_target[0]:.3f},{seg2a_target[1]:.3f},{seg2a_target[2]:.1f}) "
                  f"err_pos_gt_a={seg2a['err_pos_gt']:.4f} err_pos_gt_b={seg2b['err_pos_gt']:.4f}",
                  flush=True)

            # ---- Step 5: 충돌회피 x 오프셋 -> 최종 후축 대기자세. entry_follow 의 XN
            # 정렬 종점(xn_x, xn_z-standoff)과 이 스텝4 종점이 사실상 같은 지점이라
            # (둘 다 (-2.5,5.575) 부근), 그대로 두면 겹친다 — HARD REQUIREMENT: 최종 두
            # 로봇 위치가 >=1.5m 떨어져야 한다. final_x_offset(기본 -1.7m, 서쪽=
            # entry_lead 자기 도크 쪽)만큼 x 로 이동해 분리한다 — 이 시점 yaw=0(북향)
            # 에서는 body 기준 좌/우 스트레이프 이동이라 mecanum 홀로노믹으로
            # 자연스럽다(회전 불필요). front_ctx 로 계속 보정한다(오프셋이 커지면 XN
            # 이 프레임 밖으로 나가 n_fix=0 이 될 수 있으나 이 세그먼트의 n_fix 는
            # xn_locked 판정에 쓰지 않는다 — Step4 에서 이미 확정됨). 정확한 오프셋
            # 부호/크기는 이 태스크의 판단이다(아래 HARD REQUIREMENT 주석 및 report 에
            # 충돌 계산 근거를 남긴다). Phase C 에서 실제 차량 배치로 다시 다듬는다.
            final_target = (xn_x + final_x_offset, xn_z - xn_standoff, 0.0)
            seg_final = drive_to_pose(front_ctx, art, idx, filt, T_front, final_target,
                                      correct_yaw=False, pos_tol=align_pos_tol)
            fp = filt.pose()
            # gt_pos/gt_yaw: entry_follow Step5 와 동일한 목적(충돌회피 여유 실측
            # 근거) — 리포팅 전용, filt/제어 경로 미변경. 이 직후 entry_follow 의
            # _mission_setup 이 이 로봇을 대피(park)시켰다가 복원하므로, 최종
            # MISSIONB_SEPARATION 은 이 값과 별개로 전체 종료 시점에 재실측한다.
            gx, gz, gyaw = gt_pose_xz_yaw(art)
            print(f"MISSIONB_DONE robot={target} role=rear_axle xn_locked={xn_locked} "
                  f"reached={seg_final['reached']} pos=({fp[0]:.3f},{fp[1]:.3f}) "
                  f"yaw={fp[2]:.2f} gt_pos=({gx:.3f},{gz:.3f}) "
                  f"gt_yaw={math.degrees(gyaw):.2f} "
                  f"err_pos_gt={seg_final['err_pos_gt']:.4f} "
                  f"n_fix={n_fix2} "
                  f"target=({final_target[0]:.3f},{final_target[1]:.3f},{final_target[2]:.1f})",
                  flush=True)
            return {"xn_locked": xn_locked, "reached": seg_final["reached"],
                    "gt_pos": (gx, gz), "filt_pos": (fp[0], fp[1])}

        # ==== HARD REQUIREMENT: 충돌 없는 최종 지오메트리 ====
        # 도크(entry_lead x=-3.2, entry_follow x=-1.2)와 XN(x=-2.5)이 가까워 naive 하게
        # 두 로봇을 XN 남쪽 같은 지점에 세우면 겹친다(로봇 풋프린트 반경 ~0.68m, REARXN
        # probe 가 검증한 entry_lead 의 자연스러운 XN 정렬 종점이 entry_follow 의 최종
        # 정지 좌표 (-2.5, 5.575) 와 사실상 같은 지점이라는 게 실측으로 확인됨).
        # 선택한 좌표(모두 world x,z):
        #   entry_lead  최종: (xn_x + LEAD_FINAL_X_OFFSET, xn_z - XN_STANDOFF, yaw=0)
        #               기본값 대입 시 (-4.20, 5.575, 0)   [redo: 이전 버전은 REAR 로
        #               XN 을 보려고 180도 재정렬해 남향(yaw=180)으로 끝났으나, 이
        #               버전은 그 회전을 없애 북향(yaw=0)으로 끝난다 — 아래 클리어런스
        #               계산은 로봇 풋프린트를 반경 0.68m 원으로 모델링하므로 yaw 는
        #               계산에 들어가지 않는다(원은 방향 무관) — 180->0 변경이 이
        #               계산을 무효화하지 않는다.]
        #   entry_follow 최종: (xn_x, xn_z - XN_STANDOFF, yaw=0) = (-2.50, 5.575, 0)  [미변경]
        # 분리 거리(nominal) = |LEAD_FINAL_X_OFFSET| = 1.70m > 1.5m 요구치(0.34m 여유 —
        # 로봇 몸체 사이 간극 0.34m, 반경합 1.36m 기준). taskBSEP(MISSIONB_SEPARATION,
        # gt_pos 기반) 실측 2회: run1 gt_sep_m=1.6405(at_done)/1.6399(final), run2
        # gt_sep_m=1.6305(at_done)/1.5958(final) — 회전기구학 수정(051aaa6) 이후
        # entry_lead 종단오차가 15~22cm 로 커졌음에도(대부분 z 축, 분리축인 x 는 거의
        # 안 바뀜) 4회 전부 >=1.5m 요구치를 만족했다(여유 최소 9.6cm, run2 final).
        # 즉 nominal 1.70m 오프셋을 바꿀 필요는 없었다 — 다만 run2 의 at_done->final
        # 하락(1.6305->1.5958, 3.5cm)은 entry_follow 의 _mission_setup 이 entry_lead 를
        # 대피/복원하는 과정에서 생기는 재정착 오차로 보이며(원인 미분석, 여유가 커서
        # 이 태스크 스코프 밖), 향후 오프셋을 더 줄이는 결정을 할 때는 이 변동폭도
        # 감안해야 한다. 서쪽(오프셋 음수)을 택한 이유: entry_lead
        # 자신의 도크(x=-3.2)와 같은 편이라 entry_follow 의 L자 경로(도크체크: x=-1.2,
        # z 2.2~4.3 / XN 접근 가로구간: z=4.3, x -1.2~-2.5 / XN 접근 세로구간: x=-2.5,
        # z 4.3~5.575) 세 구간 전부에서 벌어진다(코너까지 최단거리 계산: 1.7m/2.13m/
        # 3.26m, 모두 >=1.5m). 반대로 +x(동쪽, entry_follow 쪽)로 옮기면 entry_follow
        # 도크(-1.2)에 더 가까워지고 세로구간 코너까지 최단거리가 1.5m 미만으로
        # 좁혀져(계산: offset=+1.5 일 때 가로구간 코너까지 1.29m) 요구치를 못 만족했다
        # — 그래서 서쪽을 택했다(브리프 문구는 "+x offset"이라 방향을 일반적으로
        # 표현했을 뿐, 부호/크기 선택은 이 태스크에 위임돼 있다: "Exact offsets are
        # nominal/tunable"). entry_lead 자신의 이동 경로(도크체크 북진 x=-3.2, XN 부근
        # 접근 가로구간 z~4.3 x -3.2~-2.5)도 entry_follow 도크(-1.2,2.2)와 항상 x 로
        # >=2.0m 떨어져 스태거 중(entry_follow 는 아직 도크에 대기) 충돌하지 않는다.
        # standoff(XN 남쪽 1.3m, 도크체크 1.4m)는 Task3a/3b-1 이 검증한 사각(<1.1m)
        # 밖 창을 그대로 재사용한다. redo 로 바뀐 건 XN 을 보는 카메라(REAR->FRONT)와
        # 180도 회전의 유무뿐이다 — entry_lead 가 실제로 지나가는 x,z 웨이포인트
        # 자체는 이전 버전과 동일하므로(도크체크 -> x 정렬 -> z 접근 -> x 오프셋,
        # 제자리회전은 위치를 바꾸지 않는다) 위 클리어런스 계산을 다시 유도할 필요가
        # 없다. 이 좌표는 모두 nominal — Phase C 가 실제 차량 배치로 다시 다듬는다.
        LEAD_FINAL_X_OFFSET = -1.7

        cam_h = 0.15
        for a in sys.argv[1:]:
            if a.startswith("--cam-height="):
                cam_h = float(a.split("=", 1)[1])
        seg1_standoff = 1.4
        for a in sys.argv[1:]:
            if a.startswith("--seg1-standoff="):
                seg1_standoff = float(a.split("=", 1)[1])
        standoff = 1.3
        for a in sys.argv[1:]:
            if a.startswith("--xn-standoff="):
                standoff = float(a.split("=", 1)[1])
        align_pos_tol = 0.06
        lead_final_x_offset = LEAD_FINAL_X_OFFSET
        for a in sys.argv[1:]:
            if a.startswith("--lead-final-xoffset="):
                lead_final_x_offset = float(a.split("=", 1)[1])

        # ---- Part 3: 스태거 — entry_lead 먼저 완주 -> entry_follow 시작(사용자 스펙) ----
        lead_setup = _mission_setup("entry_lead", "entry_follow",
                                    need_front=True, need_rear=True, cam_h=cam_h)
        lead_result = _run_entry_lead_b(lead_setup, dockcheck_standoff=seg1_standoff,
                                        xn_standoff=standoff,
                                        final_x_offset=lead_final_x_offset,
                                        align_pos_tol=align_pos_tol)

        follow_setup = _mission_setup("entry_follow", "entry_lead",
                                      need_front=True, need_rear=True, cam_h=cam_h)
        follow_result = _run_entry_follow_b(follow_setup, seg1_standoff=seg1_standoff,
                                            xn_standoff=standoff,
                                            align_pos_tol=align_pos_tol)

        # ---- 최종 분리거리 실측(GT) — HARD REQUIREMENT(>=1.5m) 근거 ----
        # ROBOT_RADIUS_M: 브리프/HARD REQUIREMENT 주석의 "로봇 풋프린트 반경
        # ~0.68m"를 그대로 상수화(원 모델, yaw 무관).
        ROBOT_RADIUS_M = 0.68

        def _sep_report(tag, lead_gt, follow_gt, lead_filt, follow_filt):
            dx_gt = lead_gt[0] - follow_gt[0]
            dz_gt = lead_gt[1] - follow_gt[1]
            gt_sep = math.hypot(dx_gt, dz_gt)
            dx_f = lead_filt[0] - follow_filt[0]
            dz_f = lead_filt[1] - follow_filt[1]
            filt_sep = math.hypot(dx_f, dz_f)
            clearance = gt_sep - 2.0 * ROBOT_RADIUS_M
            ok_sep = gt_sep >= 1.5
            print(f"MISSIONB_SEPARATION tag={tag} gt_sep_m={gt_sep:.4f} "
                  f"filt_sep_m={filt_sep:.4f} clearance_m={clearance:.4f} "
                  f"ok_sep={ok_sep}", flush=True)
            return gt_sep

        # ① 각 로봇이 자기 DONE 시점에 실측한 GT(entry_lead 쪽은 이후
        # entry_follow 의 _mission_setup 이 대피/복원을 거친다 — 그 영향이
        # 있었는지는 아래 ②의 재실측과 비교해 확인한다).
        _sep_report("at_done", lead_result["gt_pos"], follow_result["gt_pos"],
                    lead_result["filt_pos"], follow_result["filt_pos"])

        # ② 모든 대피/복원이 끝난 뒤(=미션 전체 종료 시점) 두 로봇을 다시 GT로
        # 직접 재실측한 것 — 이게 권위값(authoritative)이다.
        final_lead_gt = gt_pose_xz_yaw(arts["entry_lead"])[:2]
        final_follow_gt = gt_pose_xz_yaw(arts["entry_follow"])[:2]
        _sep_report("final", final_lead_gt, final_follow_gt,
                    lead_result["filt_pos"], follow_result["filt_pos"])

        ok = bool(lead_result["xn_locked"] and lead_result["reached"]
                  and follow_result["xn_locked"] and follow_result["reached"])
        print(f"MISSIONB_RESULT ok={ok}", flush=True)

        if headless:
            app.close(); return
        while app.is_running():
            app.update()
        app.close(); return

    BRIDGE_RCLPY = Path("/home/rokey/dev_ws/isaac_sim/isaacsim/_build/linux-x86_64/release"
                        "/exts/isaacsim.ros2.bridge/humble/rclpy")
    if str(BRIDGE_RCLPY) not in sys.path:
        sys.path.insert(0, str(BRIDGE_RCLPY))
    import rclpy
    from nav_msgs.msg import Odometry
    if not rclpy.ok():
        rclpy.init()
    ros_node = rclpy.create_node("parking_v4_runner")
    odom_pub = {r: ros_node.create_publisher(Odometry, f"/robot_{r}/odom", 10)
                for r in sm.ROBOTS}

    def _active_robots():
        """씬에 살아있는 로봇만. probe B 가 대상 외 로봇을 비활성화해도
        오도메트리 루프가 죽은 프림을 건드리지 않게 한다."""
        return [r for r in sm.ROBOTS
                if stage.GetPrimAtPath(robot_prim_path(r)).IsActive()]

    def publish_odom():
        """--odom 모드에 따라 휠 오도메트리 또는 GT 를 발행한다."""
        for r in _active_robots():
            if odom_mode == "wheel":
                px, pz, pyaw = odom[r].x, odom[r].z, odom[r].yaw
            else:
                px, pz, pyaw = gt_pose_xz_yaw(arts[r])
            msg = Odometry()
            msg.header.stamp = ros_node.get_clock().now().to_msg()
            msg.header.frame_id = "map"
            msg.child_frame_id = f"robot_{r}/base_link"
            msg.pose.pose.position.x = float(px)
            msg.pose.pose.position.z = float(pz)
            msg.pose.pose.orientation.z = math.sin(pyaw * 0.5)
            msg.pose.pose.orientation.w = math.cos(pyaw * 0.5)
            odom_pub[r].publish(msg)

    def step_odometry(dt):
        """휠 각속도를 읽어 각 로봇 오도메트리를 적분한다."""
        for r in _active_robots():
            vx, vy, wz = read_wheel_twist(arts[r], wheel_idx[r])
            odom[r].update(vx, vy, wz, dt)

    if "--headless-test" in sys.argv[1:]:
        def _p(a):
            return np.asarray(a.get_world_poses()[0]).reshape(-1)[:3]
        p0 = {k: _p(a) for k, a in arts.items()}
        for _ in range(180):
            app.update()
        p1 = {k: _p(a) for k, a in arts.items()}
        disp = {k: float(np.linalg.norm(p1[k] - p0[k])) for k in arts}
        ok = all(d < 0.35 for d in disp.values())
        print(f"V4_PHYSICS_TEST={'PASS' if ok else 'FAIL'} "
              f"robot_disp={ {k: round(v, 4) for k, v in disp.items()} }", flush=True)
        app.close()
        return

    if probe == "B":
        import v4_probes as vp
        from mecanum_drive import slew_twist, wheel_velocities_from_cmd_vel

        target = sm.ROBOTS[0]                      # entry_lead 한 대로 측정
        art = arts[target]                         # 나머지 3대는 build_stage 직후
        idx = wheel_idx[target]                    # 이미 비활성화됨(PROBE_B_HIDDEN)
        vel_buf = np.zeros(np.asarray(art.get_joint_positions()).reshape(-1).shape,
                           dtype=np.float32)

        def drive(tw, steps):
            cur = (0.0, 0.0, 0.0)
            prev = timeline.get_current_time()
            gt_pts, od_pts = [], []
            for _ in range(steps):
                app.update()
                now = timeline.get_current_time()
                dt = min(0.1, max(0.0, now - prev))
                prev = now
                cur = slew_twist(cur, tw, dt, linear_accel=LINEAR_ACCEL,
                                 linear_decel=LINEAR_DECEL, angular_accel=ANGULAR_ACCEL)
                omegas = wheel_velocities_from_cmd_vel(*cur)
                vel_buf[...] = 0.0
                for w, om in omegas.items():
                    vel_buf[idx[w]] = om
                art.set_joint_velocity_targets(vel_buf)
                vx, vy, wz = read_wheel_twist(art, idx)
                odom[target].update(vx, vy, wz, dt)
                gx, gz, _ = gt_pose_xz_yaw(art)
                gt_pts.append((gx, gz))
                od_pts.append((odom[target].x, odom[target].z))
            return gt_pts, od_pts

        # 측정 시작 직전에 정착(settle) 프레임 수를 명시적으로 통제한다.
        # 드리프트는 초기 settle 정도에 매우 민감하다: 예전에는 카메라 부착
        # 루프(--cameras=0 이어도 무조건 30프레임 app.update() 를 도는 코드,
        # 이 함수 앞부분)가 우연히 로봇을 더 정착시켜 drift 19.65% -> 2.69%
        # 로 결과가 7.3배 달라진 적이 있다. 측정과 무관한 코드 변경이 결과를
        # 바꾸면 안 되므로, probe B 는 여기서 자체적으로 settle 프레임만큼
        # 0 twist 로 정착시킨 뒤에만 측정을 시작한다(결과는 버린다).
        # 재특성화용으로 --settle-frames 로 정착값을 스윕할 수 있다(부록 C).
        settle_frames = PROBE_SETTLE_FRAMES
        for a in sys.argv[1:]:
            if a.startswith("--settle-frames="):
                settle_frames = int(a.split("=", 1)[1])
        print(f"PROBE_B_SETTLE frames={settle_frames}", flush=True)
        drive((0.0, 0.0, 0.0), settle_frames)

        gt_all, od_all = [], []
        legs = []
        gx0, gz0, _ = gt_pose_xz_yaw(art)
        prev_gt = (gx0, gz0)
        prev_od = (odom[target].x, odom[target].z)
        for name, tw, steps in (
            ("forward", (0.35, 0.0, 0.0), 420),
            (None, (0.0, 0.0, 0.0), 60),
            ("strafe", (0.0, 0.35, 0.0), 420),
            (None, (0.0, 0.0, 0.0), 60),
        ):
            g, o = drive(tw, steps)
            gt_all += g
            od_all += o
            if name is not None and g:
                leg_gt_len = sum(math.hypot(g[i + 1][0] - g[i][0],
                                            g[i + 1][1] - g[i][1])
                                 for i in range(len(g) - 1))
                # leg 오차는 절대 종점 오차가 아니라 이 leg 동안 (GT - odom)
                # 괴리가 "얼마나 변했는지" 로 잰다. 그래야 각 leg 가 이전
                # leg 의 누적 오차에 오염되지 않고 독립적으로 의미를 갖는다.
                disc_start = (prev_gt[0] - prev_od[0], prev_gt[1] - prev_od[1])
                disc_end = (g[-1][0] - o[-1][0], g[-1][1] - o[-1][1])
                leg_err = math.hypot(disc_end[0] - disc_start[0],
                                     disc_end[1] - disc_start[1])
                leg_rate = (leg_err / leg_gt_len * 100.0) if leg_gt_len > 1e-6 else 0.0
                legs.append({"name": name, "gt_len_m": leg_gt_len,
                             "err_m": leg_err, "drift_pct": leg_rate})
            if g:
                prev_gt = g[-1]
                prev_od = o[-1]

        gx, gz = gt_all[-1]
        ox, oz = od_all[-1]
        err = math.hypot(ox - gx, oz - gz)
        gt_len = sum(math.hypot(gt_all[i + 1][0] - gt_all[i][0],
                                gt_all[i + 1][1] - gt_all[i][1])
                     for i in range(len(gt_all) - 1))
        rate = (err / gt_len * 100.0) if gt_len > 1e-6 else 0.0

        vp.draw_trail(stage, "/World/ProbeB/GT", gt_all, vp.WHITE)
        vp.draw_trail(stage, "/World/ProbeB/Odom", od_all, vp.YELLOW, y=0.03)
        path = vp.write_report("probe_b_odom_drift", {
            "robot": target, "gt_path_len_m": gt_len,
            "final_error_m": err, "drift_rate_pct": rate,
            "gt_end": [gx, gz], "odom_end": [ox, oz],
            "legs": legs,
        })
        legs_str = " ".join(f"{l['name']}={l['drift_pct']:.2f}%" for l in legs)
        print(f"PROBE_B_RESULT gt_len={gt_len:.3f}m final_err={err:.3f}m "
              f"drift={rate:.2f}% legs=[{legs_str}] report={path.name}", flush=True)
        if headless:
            app.close()
            return

    prev_sim = timeline.get_current_time()
    while app.is_running():
        app.update()
        now_sim = timeline.get_current_time()
        dt = min(0.1, max(0.0, now_sim - prev_sim))
        prev_sim = now_sim
        rclpy.spin_once(ros_node, timeout_sec=0.0)
        step_odometry(dt)
        publish_odom()
    app.close()


if __name__ == "__main__":
    main()
