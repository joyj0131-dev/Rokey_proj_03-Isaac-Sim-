#!/usr/bin/env python3
"""v4 주차장 시뮬 브리지 — Isaac Sim(py3.11) ↔ ROS2 노드 DDS 브리지.

parking_v4_runner.py 의 --bridge 경로만 추출한 얇은 전용 브리지다(진단 프로브·
인프로세스 검출/제어는 전부 제거). 미션 로직 없이 스테이지 저작 후 구독/발행만
한다: /cmd_vel·/robot_<id>/lift_cmd 구독, /odom·/joint_states·전방/후방/측면뎁스
image 발행. 검출·측위·제어·안무는 전부 외부 ROS2 노드가 수행한다.

인자 없이 실행하면 Phase B 전체 모드로 뜬다: 로봇 4대를 도크 기본 스폰에 두고
(텔레포트 없음), 전방·측면뎁스·후방캠을 전부 발행한다. bringup_pickup_e2e.sh 의
--bridge --bridge-cameras=... --bridge-rear 호출과 동등한 기본 동작이다.

좌표 규약은 site_map_v4 가 전담한다(입차=z양수, 에셋 라벨과 반대).

실행: python3 sim_bridge.py [--gui] [--bridge-cameras=<id[,id...]|all|none>]
"""
import math
import os
import sys
import time
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

# ---- Mission Phase C, Task C5: 앞축·뒤축 리프트 ----
# 관절명/목표각(도) — origin/p4_depth:depth_stop_lift_test_dual.py 의 ARM_TARGETS 와
# hwia_parking_robot_final_caster.urdf(548~551줄) 양쪽에서 동일 값으로 실측 확인됨.
ARM_TARGETS = {
    "arm_left_front_joint": 90.0,
    "arm_left_rear_joint": -90.0,
    "arm_right_front_joint": -90.0,
    "arm_right_rear_joint": 90.0,
}
# 뒷축 리프트 판정 관례(HANDOFF.md 3절 "완료된 1로봇 뒷바퀴 리프트 통합 테스트" +
# depth_stop_lift_test_dual.py 의 dual lift_pass) — 축당 최소 상승·좌우 대칭 허용폭.
ARM_LIFT_MIN_RISE_M = 0.025
ARM_LIFT_SYM_TOL_M = 0.03
ARM_LIFT_RAMP_STEPS = 180
# v4 바닥 상판 y — parking_environment_v4.usd 의 ParkingEnvironment/Floor(scale.y=0.12,
# translate.y=-0.06 -> 상판 y=0.0)와 VehicleWaitAreas 각 Pad(translate.y≈0.0001)로 실측
# 확인. spawn_handoff_vehicle() 은 이 상수에 짐작 없이 실측 bbox 오프셋을 더한다.
V4_FLOOR_Y = 0.0
HANDOFF_SETTLE_FRAMES = 60

# ---- Mission Phase C, Task C4: 인계 베이 입구 ArUco 마커(런타임 스폰) ----
# C4 WIP(commit 85e219a) 가 남긴 블로커: entry_follow 가 XN 이후 ~6m 를 마커 없이
# 순수 오도로 주행해 도착 시점 융합오차 22.9cm(err_pos_gt) — 통로 여유 16.5cm(=
# (1.28-0.95)/2, HANDOFF 브리프 실측)를 초과해 바퀴에 충돌한다. 진입 직전 재보정용
# 마커를 하나 더 놓아 이 드리프트 구간을 끊는다.
#   x=-5.0: 트럭 후미(=x≈-5.59, len_x=5.829 를 베이 중심 -8.5 에 실은 절반=2.9145)
#     보다 0.585m 앞(+x) — 트럭에 가려지지 않으면서도 최대한 후미에 가깝다(브리프
#     지시 "가능한 한 트럭 후미에 가깝게"). --probe=BAYMARK 로 실측 검증한다.
#   z=7.075: HANDOFF_BAY_CENTER 와 동일한 통로 중심선(트럭 좌우축 중앙).
#   yaw=0: 다른 v4 마커와 동일(현재 전부 0, marker_layout.MARKER_YAW_BY_KIND 참고).
#   id=60: 이미 텍스처가 생성돼 있는 미사용 ID 블록(마커 배치 브리프가 지정,
#     textures/aruco/aruco_DICT_5X5_100_060.png 재사용 — 신규 생성 불필요).
BAY_MARKER_ID = 60
BAY_MARKER_SERVES = "BAY_OUT_ENTRY"
BAY_MARKER_KIND = "handoff_entry"
BAY_MARKER_X = -5.0
BAY_MARKER_Z = HANDOFF_BAY_CENTER[1]
BAY_MARKER_YAW = 0.0

sys.path.insert(0, str(REPO_ROOT / "src" / "parkbot_aruco"))
sys.path.insert(0, str(REPO_ROOT / "src" / "parkbot_motion"))
from parkbot_aruco import site_map_v4 as sm   # noqa: E402

RENDER_HZ = 20.0   # 물리(PHYSICS_HZ)와 독립. 낮출수록 sim초당 렌더 횟수↓ → GPU↓ → RTF↑
                   # (물리 정확도 무관). 60→20 은 렌더 1/3, 카메라 토픽도 20Hz 발행.
RENDER_WIDTH = 640
RENDER_HEIGHT = 400
PHYSICS_HZ = 120.0   # 판별용: 120→60 으로 물리 계산 절반. rtf 오르면 물리가 병목,
                    # 그대로면 병목은 렌더(카메라+GUI 뷰포트 지오메트리) 확정.
# 접촉 솔버 최소 반복(씬 전역). 기본(TGS pos~4/vel~1)은 롤러 접촉엔 부족 —
# 올리면 주행 흔들림·슬립·비결정성 감소(RTF 소폭↓). env 로 튜닝.
SOLVER_POS_ITERS = int(os.environ.get("SOLVER_POS_ITERS", "16"))
SOLVER_VEL_ITERS = int(os.environ.get("SOLVER_VEL_ITERS", "4"))
LINEAR_ACCEL = 0.3     # 슬립 감소 다운스케일(2026-07-27, 0.5→0.3): 가속 부드럽게
LINEAR_DECEL = 0.8
ANGULAR_ACCEL = 0.4    # 슬립 감소 다운스케일(2026-07-27, 0.8→0.4): 회전 가속 부드럽게
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
    # ★접촉 안정화: 메카넘은 롤러 캡슐이 바닥을 미끄러지는 물리로만 움직인다.
    # 기본 솔버 반복(TGS pos~4/vel~1)으론 롤러 8개+트럭 무게 접촉이 불안정해
    # 주행이 비틀거리고 실행마다 결과가 달라진다(비결정). 씬 전역 최소 반복을 올려
    # 모든 바디가 더 많이 수렴하게 한다 — 흔들림·슬립·들쭉날쭉의 1차 레버.
    # RTF 를 조금 먹지만 안정성 우선(튜닝 노브: 여전히 흔들리면 더 올림).
    px.CreateMinPositionIterationCountAttr(SOLVER_POS_ITERS)
    px.CreateMinVelocityIterationCountAttr(SOLVER_VEL_ITERS)
    vctx = PhysxSchema.PhysxVehicleContextAPI.Apply(sc)
    vctx.CreateUpdateModeAttr(PhysxSchema.Tokens.velocityChange)
    vctx.CreateVerticalAxisAttr(PhysxSchema.Tokens.posY)
    vctx.CreateLongitudinalAxisAttr(PhysxSchema.Tokens.posZ)


# 트럭 밑 그늘에서도 바닥 아루코가 검출되게 ambient 를 올린다(원본 dome=80 은 낮아
# 트럭 밑 카메라가 마커를 못 봄, 실측). 천장 SphereLight 는 트럭 몸체에 가려 밑을
# 못 비추므로 사방에서 오는 dome 앰비언트를 키워 그늘을 채운다. 튜닝 노브(GUI 로 조정).
DOME_INTENSITY = float(os.environ.get("DOME_INTENSITY", "800"))


def _brighten_lighting(stage):
    dome = stage.GetPrimAtPath("/World/Lighting/Dome")
    if dome and dome.IsValid():
        a = dome.GetAttribute("inputs:intensity")
        if a and a.IsValid():
            a.Set(DOME_INTENSITY)
            print(f"DOME_BRIGHTEN intensity=80→{DOME_INTENSITY:.0f} (트럭밑 마커검출용)",
                  flush=True)
            return True
    print("DOME_BRIGHTEN 경고: /World/Lighting/Dome 못 찾음", flush=True)
    return False


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


def _bay_marker_material(stage, path, texture_path):
    """build_marker_layout._marker_material 과 동일한 셰이더 네트워크(UsdPreviewSurface
    + UsdUVTexture + UsdPrimvarReader_float2, specular=0). 그 함수는 SimulationApp
    없이 도는 오프라인 스크립트(build_marker_layout.py)라 이 파일에서 직접 import 할
    수 없어 같은 검증된 USD Python API 호출을 그대로 재현한다 — 이 패턴은 실제로
    parking_environment_v4.usd 의 기존 ArucoMat_* 16개를 만든 코드와 동일하다."""
    from pxr import Sdf, UsdShade

    material = UsdShade.Material.Define(stage, path)

    reader = UsdShade.Shader.Define(stage, path.AppendChild("StReader"))
    reader.CreateIdAttr("UsdPrimvarReader_float2")
    reader.CreateInput("varname", Sdf.ValueTypeNames.Token).Set("st")
    reader.CreateOutput("result", Sdf.ValueTypeNames.Float2)

    texture = UsdShade.Shader.Define(stage, path.AppendChild("Texture"))
    texture.CreateIdAttr("UsdUVTexture")
    texture.CreateInput("file", Sdf.ValueTypeNames.Asset).Set(texture_path)
    texture.CreateInput("sourceColorSpace", Sdf.ValueTypeNames.Token).Set("raw")
    texture.CreateInput("wrapS", Sdf.ValueTypeNames.Token).Set("clamp")
    texture.CreateInput("wrapT", Sdf.ValueTypeNames.Token).Set("clamp")
    texture.CreateInput("st", Sdf.ValueTypeNames.Float2).ConnectToSource(
        reader.ConnectableAPI(), "result")
    texture.CreateOutput("rgb", Sdf.ValueTypeNames.Float3)

    shader = UsdShade.Shader.Define(stage, path.AppendChild("Shader"))
    shader.CreateIdAttr("UsdPreviewSurface")
    shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).ConnectToSource(
        texture.ConnectableAPI(), "rgb")
    shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(1.0)
    shader.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(0.0)
    shader.CreateInput("specular", Sdf.ValueTypeNames.Float).Set(0.0)

    material.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")
    return material


def spawn_bay_marker(stage, marker_id=BAY_MARKER_ID, x=BAY_MARKER_X, z=BAY_MARKER_Z,
                      yaw=BAY_MARKER_YAW, serves=BAY_MARKER_SERVES, kind=BAY_MARKER_KIND):
    """인계 베이 입구에 ArUco 마커를 런타임 스폰한다 (Mission Phase C, Task C4).

    build_stage() 는 이 라이브 스테이지의 루트 레이어를 새로 만들고
    parking_environment_v4.usd 는 그 밑에 서브레이어로만 얹는다(435행,
    `stage.GetRootLayer().subLayerPaths.append(...)`) — 그래서 여기서 무엇을
    Define 해도 이 세션의 인메모리 루트 레이어에만 쓰이고, 디스크의 팀 공용 USD
    파일은 한 바이트도 바뀌지 않는다(.Save() 를 호출하지 않는다 — 로봇/차량
    스폰과 동일한 방식, spawn_handoff_vehicle 참고).

    코드 크기(0.19444445)·타일(0.25)·사전(DICT_5X5_100)·바닥 y(0.0012)는 기존
    마커와 동일(marker_layout.MARKER_CODE_SIZE/MARKER_TILE/ARUCO_DICT/MARKER_Y를
    그대로 재사용하지 않고 리터럴로 박아 넣은 이유: marker_layout.py 는 v4 가 아닌
    구세대(v2/v3) 레이아웃 상수 모듈이라 값이 우연히 같을 뿐 이 파일이 의존할
    권위 있는 출처가 아니다 — 권위는 parking_environment_v4.usd 자체의 기존
    aruco:codeSize 값이며, 위 스크립트 조사로 0.19444445 로 확인했다). 텍스처는
    이미 생성돼 있는 aruco_DICT_5X5_100_060.png 를 그대로 쓴다(신규 생성 불필요).

    read_markers()/marker_visual_center() 는 stage.Traverse() 로 aruco:markerId
    속성을 찾을 뿐이라 런타임에 붙은 이 프림도 코드 변경 없이 그대로 잡힌다. 기존
    도크 마커와 달리 aruco:position 과 데칼 xformOp:translate 를 항상 같은 (x,z)로
    쓴다 — 그 0.7m 어긋남은 원본 에셋의 과거 편집 이력일 뿐 이 신규 마커에는 해당
    사항이 없다(둘 다 우리가 직접 같은 값으로 설정한다).
    """
    from pxr import Gf, Sdf, UsdGeom, UsdShade

    code_size = 0.19444445
    tile = 0.25
    marker_y = 0.0012
    dict_name = "DICT_5X5_100"

    texture_file = WORK_DIR / "textures" / "aruco" / f"aruco_{dict_name}_{marker_id:03d}.png"
    if not texture_file.is_file():
        raise FileNotFoundError(f"마커 텍스처가 없습니다: {texture_file}")

    root = UsdGeom.Xform.Define(stage, "/World/ArucoMarkerPreview").GetPath()
    looks = Sdf.Path("/World/Looks")

    half = tile * 0.5
    points = [
        Gf.Vec3f(-half, 0.0,  half),
        Gf.Vec3f( half, 0.0,  half),
        Gf.Vec3f( half, 0.0, -half),
        Gf.Vec3f(-half, 0.0, -half),
    ]
    uvs = [Gf.Vec2f(0, 0), Gf.Vec2f(1, 0), Gf.Vec2f(1, 1), Gf.Vec2f(0, 1)]

    mesh_path = root.AppendChild(f"{kind}_{serves}")
    mesh = UsdGeom.Mesh.Define(stage, mesh_path)
    mesh.CreatePointsAttr(points)
    mesh.CreateFaceVertexCountsAttr([4])
    mesh.CreateFaceVertexIndicesAttr([0, 1, 2, 3])
    mesh.CreateNormalsAttr([Gf.Vec3f(0, 1, 0)] * 4)
    mesh.SetNormalsInterpolation(UsdGeom.Tokens.vertex)
    mesh.CreateExtentAttr([points[0], points[2]])
    st = UsdGeom.PrimvarsAPI(mesh).CreatePrimvar(
        "st", Sdf.ValueTypeNames.TexCoord2fArray, UsdGeom.Tokens.vertex)
    st.Set(uvs)

    xf = UsdGeom.Xformable(mesh)
    xf.AddTranslateOp().Set(Gf.Vec3d(float(x), marker_y, float(z)))
    if yaw:
        xf.AddRotateYOp().Set(float(yaw))

    material = _bay_marker_material(
        stage, looks.AppendChild(f"ArucoMat_{marker_id:03d}"),
        Sdf.AssetPath(str(texture_file)))
    UsdShade.MaterialBindingAPI.Apply(mesh.GetPrim()).Bind(material)

    prim = mesh.GetPrim()
    prim.CreateAttribute("aruco:markerId", Sdf.ValueTypeNames.Int).Set(int(marker_id))
    prim.CreateAttribute("aruco:dictionary", Sdf.ValueTypeNames.String).Set(dict_name)
    prim.CreateAttribute("aruco:kind", Sdf.ValueTypeNames.String).Set(kind)
    prim.CreateAttribute("aruco:yaw", Sdf.ValueTypeNames.Float).Set(float(yaw))
    prim.CreateAttribute("aruco:serves", Sdf.ValueTypeNames.String).Set(serves)
    prim.CreateAttribute("aruco:note", Sdf.ValueTypeNames.String).Set(
        "인계 베이 입구 진입정렬(Mission Phase C, Task C4, 런타임 스폰)")
    prim.CreateAttribute("aruco:codeSize", Sdf.ValueTypeNames.Float).Set(code_size)
    prim.CreateAttribute("aruco:position", Sdf.ValueTypeNames.Float3).Set(
        Gf.Vec3f(float(x), 0.0, float(z)))

    print(f"BAY_MARKER_SPAWNED id={marker_id} serves={serves} pos=({x:.3f},{z:.3f}) "
          f"path={mesh_path}", flush=True)
    return str(mesh_path)


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
    _brighten_lighting(stage)
    for _ in range(30):
        app.update()

    n_lidar = _disable_sensors(stage)

    # Mission Phase C, Task C4: 인계 베이 입구 마커. validate_markers() 가 다른
    # 마커와 동일한 z-부호 규약 검사를 이 마커에도 적용하도록 markers 딕셔너리
    # 조립 이전에 스폰한다.
    spawn_bay_marker(stage)

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


def attach_camera_graph(robot_id, cam_path, role="front", width=640, height=480,
                         image_type="rgb"):
    """C++ OmniGraph 로 이미지(+옵션 camera_info) 를 발행한다.

    Python rclpy 로 이미지를 퍼블리시하면 Isaac 루프가 죽는다(ARUCO_PLAN 전제).
    camera_info 는 ROS2CameraHelper 의 type 이 아니라 별도 ROS2CameraInfoHelper 노드다
    (DEBUG_LOG 2026-07-21). 노드 타입·속성명은 설치된 Isaac Sim 5.1
    (isaacsim.core.nodes / isaacsim.ros2.bridge 의 .ogn 문서)로 확인했다.

    role 로 네임스페이스/그래프 경로를 분리해 같은 로봇의 전방·후방·좌우측면
    카메라를 동시에(별개 토픽/그래프로) 발행할 수 있다(기본 "front").

    image_type: ROS2CameraHelper 의 ``inputs:type`` 토큰(Task R4 실측,
    ``OgnROS2CameraHelper.ogn`` allowedTokens: rgb/depth/depth_pcl/
    instance_segmentation/semantic_segmentation/bbox_2d_*):
      - "rgb"(기본, 하위호환): 토픽 ``{ns}/image_raw``, ``sensor_msgs/Image``
        encoding "rgb8". camera_info 노드도 같이 붙는다(``{ns}/camera_info``).
      - "depth": 토픽 ``{ns}/depth``, ``sensor_msgs/Image`` encoding
        **"32FC1"**(픽셀당 미터, float32 1채널) — Isaac 내부적으로
        ``DistanceToImagePlane`` rendervar 를 그대로 이미지로 실어보낸다
        (``isaacsim.ros2.bridge`` extension.py 176~189행 확인: depth writer 가
        명시적으로 ``encoding="32FC1"`` 로 등록됨). camera_info 노드는 붙이지
        않는다(축 감지는 내재파라미터가 필요 없다 — ROI-min 만 씀).
    """
    import omni.graph.core as og
    ns = f"/robot_{robot_id}/{role}"
    topic_name = f"{ns}/depth" if image_type == "depth" else f"{ns}/image_raw"
    nodes = [
        ("tick", "omni.graph.action.OnPlaybackTick"),
        ("render", "isaacsim.core.nodes.IsaacCreateRenderProduct"),
        ("rgb", "isaacsim.ros2.bridge.ROS2CameraHelper"),
    ]
    connects = [
        ("tick.outputs:tick", "render.inputs:execIn"),
        ("render.outputs:execOut", "rgb.inputs:execIn"),
        ("render.outputs:renderProductPath", "rgb.inputs:renderProductPath"),
    ]
    values = [
        ("render.inputs:cameraPrim", [cam_path]),
        ("render.inputs:width", width),
        ("render.inputs:height", height),
        ("rgb.inputs:type", image_type),
        ("rgb.inputs:topicName", topic_name),
        ("rgb.inputs:frameId", f"robot_{robot_id}/{role}_cam"),
    ]
    if image_type != "depth":
        nodes.append(("info", "isaacsim.ros2.bridge.ROS2CameraInfoHelper"))
        connects.append(("render.outputs:execOut", "info.inputs:execIn"))
        connects.append(("render.outputs:renderProductPath", "info.inputs:renderProductPath"))
        values.append(("info.inputs:topicName", f"{ns}/camera_info"))
        values.append(("info.inputs:frameId", f"robot_{robot_id}/{role}_cam"))
    og.Controller.edit(
        {"graph_path": f"/Graphs/cam_{robot_id}_{role}", "evaluator_name": "push"},
        {
            og.Controller.Keys.CREATE_NODES: nodes,
            og.Controller.Keys.CONNECT: connects,
            og.Controller.Keys.SET_VALUES: values,
        },
    )


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
# p4_depth 의 depth_stop_detector.DEFAULT_ROI_FRAC=(0.30,0.70,0.50,1.00) 을 처음에는
# 그대로 재사용했으나(taskC2fix-brief.md 지시), 이 로봇 에셋에서는 실측(diag 스크립트,
# raw 프레임 행별 min 덤프, taskC2fix-report.md 참고)으로 그 ROI 가 구조적으로 못
# 쓴다는 게 확인됐다: 이 카메라는 수직 FOV 가 65°(vaper/focal 실측)로 넓고 지상
# 0.16m 높이에 달려 있어, row_hi=1.00(프레임 맨 아래, 최대 하향각)까지 내려가면
# 트럭과 무관하게 "항상" 바닥을 근거리에서 본다 — x 를 -4.5(트럭 밖)에 둔 채로도
# 맨 아래 행이 이미 0.172m 로 고정돼(행별 덤프: row=240(수평)=3.033m(먼 배경) ->
# row=264(0.55)=1.684 -> row=360(0.75)=0.342 -> row=479(1.00)=0.172, 매끄러운
# 바닥-교차 기하와 정확히 일치) 실제 바퀴 근접거리(로봇이 중앙정렬 상태일 때
# ≈0.25~0.6m)보다 항상 더 가까워 min() 을 완전히 집어삼킨다 — 그래서 첫 실행에서
# 트럭을 끝까지 무사고로 통과했는데도 n_troughs_left=n_troughs_right=0 이 나왔다
# (자기 오클루전이 원인이라는 첫 가설은 RSD455/Visual 을 숨겨도 값이 그대로여서
# 실측으로 기각했다 — depth_setup() 의 occluder-hide 는 p4_depth 검증 관례를 따라
# 남겨두지만 이 문제의 원인은 아니었다). 카메라 높이(0.16m)가 바퀴 높이범위
# (0~0.846m) 안에 있으므로 수평에 가까운 광선만으로도(하향각 불필요) 바퀴 옆면과
# 바로 교차한다 — row_hi 를 바닥이 안 잡히는 범위로 좁힌다(0.58: 행별 덤프 보간상
# 바닥의 최악값이 ≈1.0m 로 바퀴 근접거리보다 한참 멀어 안전). col 범위는 원래
# 참조값을 그대로 둔다(바닥 문제와 무관). (col_lo, col_hi, row_lo, row_hi), 0~1 비율.
DEPTH_ROI_FRAC = (0.30, 0.70, 0.48, 0.58)


def hide_side_cam_occluder(stage, app, cam_path):
    """side 카메라 자기 오클루전(자기 하우징) 은닉 — depth_setup() 과 브리지
    측면 뎁스 발행(--bridge, Task R4) 양쪽이 공유하는 헬퍼로 추출했다.

    이 에셋의 cam_side_<side>_link 밑에는 front 카메라와 마찬가지로 RSD455
    물리 모델이 통째로 붙어있고, 그 RSD455/Visual 서브트리(Case_front/Glass/
    Front_mask/camera_mask 등)가 카메라 prim 바로 앞 0~0.04m 거리에 있다
    (diag_side_cam2.py 오프라인 실측, taskC2fix-report.md 참고). 이걸 숨기지
    않으면 x 위치·트럭 유무와 무관하게 ROI 최소뎁스가 항상 자기 하우징까지의
    고정거리(~0.1725m)로 눌러붙는다(n_troughs=0, 트럭을 전혀 못 봄).

    카메라가 어느 파이프라인(omni.replicator annotator vs OmniGraph
    ROS2CameraHelper)으로 읽히든 은닉 대상은 동일한 USD prim 가시성이라 한
    번만 적용하면 양쪽 다 적용된다 — 같은 카메라에 두 파이프라인을 동시에
    붙이지 않는 한(현재 어디서도 그렇게 하지 않음) 중복 호출은 안전(idempotent)
    하다.
    """
    from pxr import UsdGeom as _UsdGeom2
    occluder_path = cam_path.rsplit("/", 1)[0] + "/RSD455/Visual"
    occluder = stage.GetPrimAtPath(occluder_path)
    if occluder.IsValid():
        _UsdGeom2.Imageable(occluder).CreateVisibilityAttr(_UsdGeom2.Tokens.invisible)
        for _ in range(3):
            app.update()
    return bool(occluder.IsValid())


def depth_setup(stage, timeline, app, robot_id, side="left", cam_res=DEPTH_CAM_RES):
    """측면 뎁스캠 render_product + distance_to_image_plane annotator 컨텍스트.

    fuse_camera_setup 의 카메라 배선 부분(render_product 생성 -> annotator attach)만
    떼어낸 구조다 — 마커/보정 없이 뎁스 스트림만 필요한 --probe=DEPTH(C2)와
    이후 이식된 정지판단(C3, 인프로세스)이 공용으로 쓴다. Task R4 의 ROS2
    ``--bridge`` 측면 뎁스 발행은 이 함수를 쓰지 않는다(OmniGraph 경로가 자체
    render_product 를 만든다 — 같은 카메라에 두 렌더프로덕트를 만드는 중복
    비용을 피하려고 ``hide_side_cam_occluder()`` 만 재사용한다, 아래 --bridge
    블록 참고).

    자기 오클루전 수정은 ``hide_side_cam_occluder()`` 로 옮겼다(taskC2fix 실측
    으로 발견, 위 docstring 참고) — 로직은 그대로다.
    """
    import omni.replicator.core as rep
    cam_path = find_side_camera(stage, robot_id, side)
    occluder_hidden = hide_side_cam_occluder(stage, app, cam_path)
    rp = rep.create.render_product(cam_path, cam_res)
    depth_annot = rep.AnnotatorRegistry.get_annotator("distance_to_image_plane")
    depth_annot.attach([rp])
    for _ in range(3):
        app.update()
    return {"cam_path": cam_path, "render_product": rp, "depth_annot": depth_annot,
            "occluder_hidden": occluder_hidden}


def _quat_mul(q0, q1):
    w0, x0, y0, z0 = q0
    w1, x1, y1, z1 = q1
    return np.array([
        w1*w0 - x1*x0 - y1*y0 - z1*z0,
        w1*x0 + x1*w0 + y1*z0 - z1*y0,
        w1*y0 - x1*z0 + y1*w0 + z1*x0,
        w1*z0 + x1*y0 - y1*x0 + z1*w0])


def configure_arm_drives(stage, robot_joints, stiffness=1800.0, damping=140.0, max_force=5000.0):
    """스윙 암 4개(ARM_TARGETS)를 위치 드라이브로 설정한다 — configure_hub_drives(휠,
    속도드라이브 stiffness=0)와 짝을 이루는 팔 전용 버전. 값은
    origin/p4_depth:depth_stop_lift_test_dual.py 의 build_test_stage() 가 검증한
    stiffness=1800/damping=140/maxForce=5000 을 그대로 재사용한다(URDF 자체
    dynamics damping=8.0 은 임포트시 기본값일 뿐이며, 두 참조 구현 모두 런타임에
    이 값으로 덮어써야 실제로 트럭을 들어올릴 만큼 팔이 버틴다 — Task C5 브리프
    지시대로 검증된 값을 재사용, 새로 추측하지 않는다)."""
    from pxr import UsdPhysics

    for jname in ARM_TARGETS:
        joint = stage.GetPrimAtPath(f"{robot_joints}/{jname}")
        drive = UsdPhysics.DriveAPI.Get(joint, "angular")
        if not drive:
            drive = UsdPhysics.DriveAPI.Apply(joint, "angular")
        drive.CreateStiffnessAttr(stiffness)
        drive.CreateDampingAttr(damping)
        drive.CreateMaxForceAttr(max_force)
        drive.CreateTargetPositionAttr(0.0)


def deploy_arms(art, idx, scale):
    """ARM_TARGETS 를 위치 목표로 적용한다(scale∈[0,1]). 호출측이 매 스텝 scale 을
    0->1 로 늘려가며 반복 호출하면 각도가 아니라 시간에 걸쳐 램프된다 — step 이
    아니라 램프로 적용하라는 브리프 지시, dock_lift_handoff_runner_v2.py(467~482줄)
    apply_arms() 와 p4_depth set_arm_targets() 둘 다 동일 관례. DOF 배열은 이
    파일의 기존 휠 관례(art.get_joint_positions()/(...).reshape(-1), 1D)를 그대로
    따른다(_ingress_axle 의 vel_buf 와 동일 패턴)."""
    pos = np.array(art.get_joint_positions(), dtype=np.float32).reshape(-1)
    for name, deg in ARM_TARGETS.items():
        pos[idx[name]] = math.radians(deg * scale)
    art.set_joint_position_targets(pos)


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

    # Task C5: 스윙 암 4개를 위치 드라이브로 설정(configure_hub_drives 와 같은 시점 —
    # 두 곳 다 Articulation.initialize() 이후 UsdPhysics.DriveAPI 를 직접 프림에
    # 걸어 이미 이 파일에서 검증된 관례를 그대로 재사용).
    for robot_id in arts:
        configure_arm_drives(stage, f"{robot_prim_path(robot_id)}/joints")

    from mecanum_drive import WHEEL_JOINTS, cmd_vel_from_wheel_velocities
    from parkbot_motion.wheel_odometry import WheelOdometry

    wheel_idx = {r: {w: arts[r].dof_names.index(j) for w, j in WHEEL_JOINTS.items()}
                 for r in arts}
    arm_idx = {r: {n: arts[r].dof_names.index(n) for n in ARM_TARGETS}
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

    from mecanum_drive import wheel_velocities_from_cmd_vel, slew_twist

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

    # R2: 마지막으로 측정한 바디 twist(vx,vy,wz) — publish_odom 이 Odometry.twist 에
    # 싣는다(속도는 오도메트리 적분과 별개로 매 스텝 새로 측정한 값). 0 으로 시작해
    # step_odometry 가 처음 갱신하기 전에도 publish_odom 이 안전하게 읽을 수 있다.
    last_twist = {r: (0.0, 0.0, 0.0) for r in sm.ROBOTS}

    def publish_odom():
        """--odom 모드에 따라 휠 오도메트리 또는 GT 를 발행한다.

        R2 계약(리팩터 설계서 4절): pose 필드는 이 프로젝트 좌표계 그대로
        (position.x,position.z = 월드 (x,z)[m], position.y 는 항상 0 — 높이는
        휠 오도가 추정하지 않는다; orientation 은 yaw 를 메시지의 z축 회전
        쿼터니언에 인코딩 — ROS map(z-up) 표준이 아니라 이 프로젝트의 yaw=0→+Z
        규약, gt_pose_xz_yaw/WheelOdometry 와 동일, 기존 발행 패턴 그대로
        유지). twist 필드는 REP103 대로 child_frame(base_link) 기준 바디
        twist: linear.x=전진[m/s], linear.y=좌[m/s], angular.z=CCW+[rad/s] —
        read_wheel_twist(cmd_vel_from_wheel_velocities 의 최소자승 역)로 매
        스텝 측정한 값이며 GT 는 전혀 섞이지 않는다.
        """
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
            vx, vy, wz = last_twist[r]
            msg.twist.twist.linear.x = float(vx)
            msg.twist.twist.linear.y = float(vy)
            msg.twist.twist.angular.z = float(wz)
            odom_pub[r].publish(msg)

    def step_odometry(dt):
        """휠 각속도를 읽어 각 로봇 오도메트리를 적분하고, 측정 twist 를 남긴다.

        R3c(taskR3c-report.md): 적분(``odom[r].update``)에는 ``YAW_ODOM_SCALE``
        을 곱한다 — 인프로세스 미션의 ``filt.predict_body(pvx, pvy,
        pwz*YAW_ODOM_SCALE, dt)``(1252/3366행)와 동일한 보정을 휠→물리 회전
        스케일에 적용하는 것이다. 이 보정이 빠져 있던 게 R3b(taskR3b-report.md
        §7.2)가 실측한 "명령 90° → GT 스윕 ≈103°(+14%)" 오버슈트의 원인이었다.
        ``last_twist``(=/odom 의 ``twist`` 필드, 리포팅/측정값 전용)는 보정하지
        않는다 — 원본도 "측정된 바디 속도"와 "적분에 넣는 보정된 값"을 분리해서
        다루고(YAW_ODOM_SCALE 정의부 주석), 현재 이 twist 필드를 예측에 쓰는
        구독자가 없다(marker_localizer_node._on_odom 은 twist 가 아니라 pose
        델타로 predict 한다).

        이중적용 방지: 이 함수(브리지 경로)와 인프로세스 미션의 두 호출부
        (1252행 ``drive_to_pose``, 3366행 DOCKSWEEP류)는 서로 다른 실행
        경로(브리지 vs 인프로세스 미션)라 같은 스텝에서 동시에 돌지 않는다 —
        브리지 모드(``--bridge``)는 미션 코드를 전혀 거치지 않는다(§ "if
        bridge:" 블록은 미션 진입점 이후에 도달).
        """
        for r in _active_robots():
            vx, vy, wz = read_wheel_twist(arts[r], wheel_idx[r])
            last_twist[r] = (vx, vy, wz)
            odom[r].update(vx, vy, wz * YAW_ODOM_SCALE, dt)

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

    # ---- R2: 시뮬 브리지 루프 ----
    # 미션 로직 없음(스테이지 저작 이후 여기서부터는 구독/발행만). 외부
    # ROS2 노드(R3 navigate_action_server 등)가 /cmd_vel·/robot_<id>/lift_cmd 로
    # 로봇을 몰고 /odom·/joint_states 를 구독한다. GT 는 콘솔 하트비트에만
    # 쓴다(제어 입력 절대 아님 — 브리핑 지시).
    from sensor_msgs.msg import JointState
    from std_msgs.msg import Float32, Bool
    from geometry_msgs.msg import Twist

    # ---- R3c 검증용(선택): 로봇을 자기 도크마커 접근선 위로 재배치 ----
    # 기본은 no-op(기존 스폰 좌표 그대로 — 하위호환, R2/R3b 수치와 계속
    # 비교 가능). 로봇의 원래 스폰 자세(entry_lead: x=-3.2,z=2.2,yaw=90)는
    # 도크 마커(z=2.9, marker_visual_center 기준 — read_markers() 의 z 와
    # 0.7m 어긋난다, 211행 marker_visual_center docstring)와 z 가 달라
    # 카메라 정면에 마커가 들어오지 않는다(마커가 진행방향과 수직으로
    # 어긋나 있음). --bridge-marker-pose=<robot_id>:<d> 로 지정하면 그
    # 로봇을 자기 도크 마커에서 d[m] 떨어진 접근선 위(마커와 같은 z, 스폰
    # yaw 유지)로 옮긴다 — --probe=FUSE(1734행)가 검증한 것과 동일한
    # 배치 관례(d_start=2.1~d_end=1.25 구간에서 검출 안정적임, taskR3c
    # 실측 재사용). 재배치 후에는 odom 도 새 GT 로 재시딩한다(오도는
    # "초기 자세만 GT로 정렬"하는 기존 관례, 1358행과 동일 취지).
    for a in sys.argv[1:]:
        if a.startswith("--bridge-marker-pose="):
            _rid, _dstr = a.split("=", 1)[1].split(":")
            if _rid not in arts:
                raise SystemExit(f"--bridge-marker-pose: 알 수 없는 로봇 {_rid!r}")
            _d = float(_dstr)
            _ref_serves = sm.ROBOT_DOCK_MARKER[_rid]
            _mx, _mz = marker_visual_center(stage, _ref_serves)
            _, _spawn_orn = arts[_rid].get_world_poses()
            _spawn_orn = np.asarray(_spawn_orn).reshape(-1)[:4].copy()
            arts[_rid].set_world_poses(
                np.array([[_mx - _d, ROBOT_SPAWN_Y, _mz]]), np.array([_spawn_orn]))
            for _ in range(30):
                app.update()
            _gx, _gz, _gyaw = gt_pose_xz_yaw(arts[_rid])
            odom[_rid] = WheelOdometry(x=_gx, z=_gz, yaw=_gyaw)
            print(f"BRIDGE_TELEPORT robot={_rid} d={_d} "
                  f"pose=({_gx:.3f},{_gz:.3f},{math.degrees(_gyaw):.1f})", flush=True)

    # ---- Task R4 검증용(선택): 로봇을 트럭 진입선(--probe=DEPTH 와 동일
    # 배치)으로 재배치 ----
    # axle_detector_node 스모크는 로봇이 트럭 아래를 완주하며 두 축을
    # 지나가야 한다 — --bridge-marker-pose(위)는 도크 마커 접근선 배치라
    # 용도가 다르다. --probe=DEPTH(1898행 부근)가 쓰는 것과 **완전히 동일한
    # 계산**(트럭 4바퀴 world z 평균으로 중심선 정렬 + spawn_orn 을 180°
    # 반전해 -X 를 보게)을 재사용한다(짐작 없이 --probe=DEPTH 실측 로직
    # 그대로) — 기본 x=-4.5(트럭 밖, 진입 여유), z=z_center_truck(실측
    # 중심선). --bridge-depth-pose=<robot_id>[:<x>] 로 지정하면 그 로봇만
    # 재배치한다(x 생략 시 -4.5 — 하위호환). x 를 명시하면 임의의 x 로 바로
    # 배치할 수 있다 — Task R4 리프트 스모크가 이걸 재사용해 로봇을 실측
    # 축 중심(예: 후축 -6.569)에 직접 놓고 ControlLift 를 시험한다(전체
    # 안무(R5) 없이도 "그 자리에 있으면 리프트가 되는지"만 독립적으로
    # 검증하기 위함).
    for a in sys.argv[1:]:
        if a.startswith("--bridge-depth-pose="):
            _spec = a.split("=", 1)[1]
            if ":" in _spec:
                _rid, _xstr = _spec.split(":", 1)
                _x_start = float(_xstr)
            else:
                _rid, _x_start = _spec, -4.5
            if _rid not in arts:
                raise SystemExit(f"--bridge-depth-pose: 알 수 없는 로봇 {_rid!r}")
            _art = arts[_rid]
            from pxr import UsdGeom as _UsdGeomDepth
            _tc = timeline.get_current_time()

            def _wheel_world_z(name):
                return float(_UsdGeomDepth.Xformable(stage.GetPrimAtPath(
                    f"{HANDOFF_VEHICLE_ROOT}/{name}")).ComputeLocalToWorldTransform(_tc)
                    .ExtractTranslation()[2])

            _front_cz = 0.5 * (_wheel_world_z("FrontLeftWheel") + _wheel_world_z("FrontRightWheel"))
            _rear_cz = 0.5 * (_wheel_world_z("RearLeftWheel") + _wheel_world_z("RearRightWheel"))
            _z_center_truck = 0.5 * (_front_cz + _rear_cz)
            _, _spawn_orn2 = _art.get_world_poses()
            _spawn_orn2 = np.asarray(_spawn_orn2).reshape(-1)[:4].copy()
            _half = math.radians(180.0) * 0.5
            _qy = np.array([math.cos(_half), 0.0, math.sin(_half), 0.0])
            _face_neg_x = _quat_mul(_spawn_orn2, _qy)
            _art.set_world_poses(np.array([[_x_start, ROBOT_SPAWN_Y, _z_center_truck]]),
                                 np.array([_face_neg_x]))
            for _ in range(30):
                app.update()
            _gx, _gz, _gyaw = gt_pose_xz_yaw(_art)
            odom[_rid] = WheelOdometry(x=_gx, z=_gz, yaw=_gyaw)
            print(f"BRIDGE_DEPTH_TELEPORT robot={_rid} z_center_truck={_z_center_truck:.4f} "
                  f"pose=({_gx:.3f},{_gz:.3f},{math.degrees(_gyaw):.1f})", flush=True)

    # ---- R3c gap2-1: 전방카메라 발행(marker_localizer_node 용) ----
    # attach_camera_graph(700행)는 OmniGraph 로 image_raw+camera_info 를 낸다
    # (순수 rclpy 이미지 발행은 Isaac 루프를 죽인다, 그 함수 docstring 참고).
    # 기본은 브리지가 관리하는 로봇 전부(arts, 통상 4대)에 전방캠을 붙인다
    # (R2 의 "4개 로봇 전부 대칭 취급" 관례와 동일) — "--bridge 는 자신이
    # 다루는 로봇(들)의 전방캠을 낸다"를 플래그 없이도 만족시키기 위함.
    # 카메라 렌더 비용 때문에 RTF 가 떨어진다(참고: --probe=C 실측,
    # probe_reports/probe_c_rtf_*cam.json — 0캠 rtf≈8.0, 2캠 rtf≈1.55,
    # 4캠 rtf≈1.12, 그 자체가 다른 부하 조건 실측이라 브리지 루프에 그대로
    # 대입은 못하지만 카메라가 늘수록 RTF 가 준다는 방향은 같다). 한 로봇만
    # 검증할 때 불필요한 렌더 비용을 피하려면 --bridge-cameras=<id[,id...]>
    # (또는 "none")로 부착 대상을 좁힐 수 있다(기본="all").
    #
    # 카메라 높이 보정: marker_localizer_node 의 기본 T_base_cam
    # (_DEFAULT_T_BASE_CAM)은 원시 카메라 마운트가 아니라 cam_h=0.15m 로
    # 높이를 올린 상태에서 자동보정됐다(memory aruco-v4-fusion.md: "T_base_cam
    # 자동보정=X180 6.2mm(카메라 0.15m)", fuse_camera_setup 의 M5/FUSE 계열
    # 프로브가 전부 이 값을 씀, 746행 근처). 브리지가 원시 마운트(0.09m) 그대로
    # 카메라를 붙이면 그 6cm 불일치가 측위에 체계적 편향을 만든다 — 그래서
    # fuse_camera_setup(776-782행)과 동일한 높이 오버셋 로직만 떼어와 재사용한다
    # (그 함수의 나머지: 마커지도/검출기/K 계산은 브리지엔 필요 없다 — 검출은
    # 이 프로세스가 아니라 marker_localizer_node 가 발행된 image_raw 로 한다).
    cam_robots_bridge = list(arts)
    for a in sys.argv[1:]:
        if a.startswith("--bridge-cameras="):
            spec = a.split("=", 1)[1]
            if spec == "all":
                cam_robots_bridge = list(arts)
            elif spec == "none":
                cam_robots_bridge = []
            else:
                cam_robots_bridge = [s for s in spec.split(",") if s]
                unknown = [s for s in cam_robots_bridge if s not in arts]
                if unknown:
                    raise SystemExit(f"--bridge-cameras: 알 수 없는 로봇 {unknown}")
    # 후방캠(role="rear") 발행: sim_bridge 는 Phase B "후방캠 도크점검" 전용이라
    # 기본 ON 이다(러너의 --bridge-rear 옵트인을 기본 켬으로 승격 — 별도 옵션 없음).
    # Phase B 의 marker_localizer_node 가 구독할 /robot_<id>/rear/image_raw 를
    # attach_camera_graph(role="rear") 호출로 낸다(아래, 뎁스캠 셋업 뒤).
    bridge_rear = True
    if cam_robots_bridge:
        from pxr import Gf as _bridge_cam_gf, UsdGeom as _bridge_cam_usdgeom
        BRIDGE_CAM_H = 0.15
        for r in cam_robots_bridge:
            cam_path = find_front_camera(stage, r)
            cam_prim = stage.GetPrimAtPath(cam_path)
            cam_xf = _bridge_cam_usdgeom.Xformable(cam_prim)
            dy_world = BRIDGE_CAM_H - 0.09
            if abs(dy_world) > 1e-9:
                mb = cam_xf.ComputeLocalToWorldTransform(timeline.get_current_time())
                local_delta = mb.GetInverse().TransformDir(
                    _bridge_cam_gf.Vec3d(0.0, dy_world, 0.0))
                cam_xf.AddTranslateOp(_bridge_cam_usdgeom.XformOp.PrecisionDouble,
                                      "camHeightBridge").Set(local_delta)
            attach_camera_graph(r, cam_path, role="front")
        for _ in range(30):
            app.update()
    cam_topics = [f"/robot_{r}/front/image_raw" for r in cam_robots_bridge]
    print(f"BRIDGE_CAMERAS robots={cam_robots_bridge} cam_h=0.15 "
          f"topics={cam_topics}", flush=True)

    # ---- Task R4 Deliverable 1: 측면 뎁스캠 발행(axle_detector_node 용) ----
    # attach_camera_graph(위, image_type 인자 추가됨)를 image_type="depth" 로
    # 호출 — OmniGraph ROS2CameraHelper 의 depth 토큰이 DistanceToImagePlane
    # rendervar 를 그대로 sensor_msgs/Image(encoding "32FC1", 픽셀당 미터)로
    # 낸다(isaacsim.ros2.bridge extension.py 176-189행 실측 확인). 순수
    # rclpy 발행 금지 원칙(위 attach_camera_graph docstring)은 depth 도 동일 —
    # 그래서 OmniGraph 로 낸다(러너 코멘트가 명시한 대로: 파이썬에서 직접
    # 발행하면 Isaac 루프가 죽는다).
    #
    # cam_robots_bridge(위, --bridge-cameras 로 조절되는 로봇 목록)를 그대로
    # 재사용한다 — "브리지는 자신이 다루는 로봇(들)의 카메라를 낸다"는 기존
    # 관례를 전방캠과 동일하게 측면 뎁스캠에도 적용(지시사항: "Add the two
    # side cameras to what --bridge attaches" — 별도 플래그를 새로 만들지
    # 않고 기존 선택 플래그에 얹었다).
    #
    # 카메라 지상고는 건드리지 않는다(원 마운트 0.16m 그대로 — taskC2fix
    # 실측: depth_setup()/--probe=DEPTH 도 측면캠 높이를 보정한 적이 없다,
    # 전방캠만 마커 로컬라이저 T_base_cam 정합을 위해 0.15m 로 올린다).
    # 자기 오클루전(자기 하우징) 은닉은 hide_side_cam_occluder() 로 공유
    # (depth_setup() 이 쓰던 것과 동일 로직, taskC2fix-report.md 근거).
    for r in cam_robots_bridge:
        for side in ("left", "right"):
            side_cam_path = find_side_camera(stage, r, side=side)
            hide_side_cam_occluder(stage, app, side_cam_path)
            attach_camera_graph(r, side_cam_path, role=side, image_type="depth")
    if cam_robots_bridge:
        for _ in range(30):
            app.update()

    # 측면 뎁스캠 게이팅: 뎁스 렌더는 로봇이 트럭 밑 진입을 준비할 때만 필요하다
    # (axle_detector). Phase B 주행 내내 8대(4로봇×2)를 렌더하면 RTF 를 크게 깎고,
    # 게다가 axle_detector 가 주행 중 스테일 트로프를 쌓는다(픽업 미도달 버그의 원인).
    # 그래서 셋업으로 파이프라인(렌더프로덕트+ROS2 writer)은 정상 배선해두되
    # (위 30틱), 곧바로 hydra 업데이트를 pause 한다 — IsaacCreateRenderProduct 의
    # inputs:enabled=False 면 handle 이 이미 있을 때 set_updates_enabled(False) 로
    # 렌더만 멈춘다(OgnIsaacCreateRenderProduct.py:55-61 실측). orchestrator 가
    # 픽업 approach 시점에 /robot_<id>/depth_enable=true 를 쏘면 즉시 resume 한다.
    import omni.graph.core as _og_depth
    depth_state = {}  # r -> bool(현재 뎁스 렌더 on/off) — 전환마다 로그로 확증

    def set_depth_enabled(r, on):
        on = bool(on)
        for side in ("left", "right"):
            _og_depth.Controller.attribute(
                f"/Graphs/cam_{r}_{side}/render.inputs:enabled").set(on)
        if depth_state.get(r) != on:
            depth_state[r] = on
            print(f"DEPTH_RENDER robot={r} enabled={on}", flush=True)

    for r in cam_robots_bridge:
        set_depth_enabled(r, False)

    depth_topics = [f"/robot_{r}/{side}/depth"
                     for r in cam_robots_bridge for side in ("left", "right")]
    print(f"BRIDGE_DEPTH_CAMERAS robots={cam_robots_bridge} "
          f"topics={depth_topics} gated=off(depth_enable 로 켬)", flush=True)

    # ---- 후방캠 발행(기본 ON) ----
    # Phase B "후방캠 도크점검"의 marker_localizer_node 가 구독할
    # /robot_<id>/rear/image_raw 를 낸다. attach_camera_graph(role="rear")
    # 호출로 cam_robots_bridge(위, --bridge-cameras 로 조절되는 로봇 목록)
    # 각각에 후방캠을 붙인다(bridge_rear 는 sim_bridge 에서 항상 True).
    if bridge_rear and cam_robots_bridge:
        for r in cam_robots_bridge:
            rear_cam_path = find_rear_camera(stage, r)
            attach_camera_graph(r, rear_cam_path, role="rear")
        for _ in range(30):
            app.update()
    if bridge_rear:
        rear_topics = [f"/robot_{r}/rear/image_raw" for r in cam_robots_bridge]
        print(f"BRIDGE_REAR_CAMERAS robots={cam_robots_bridge} "
              f"topics={rear_topics}", flush=True)

    joint_pub = {r: ros_node.create_publisher(JointState, f"/robot_{r}/joint_states", 10)
                 for r in arts}
    target_twist = {r: (0.0, 0.0, 0.0) for r in arts}
    applied_twist = {r: (0.0, 0.0, 0.0) for r in arts}
    lift_cmd = {r: 0.0 for r in arts}
    lift_applied = {r: 0.0 for r in arts}
    bridge_vel_buf = {r: np.zeros(np.asarray(arts[r].get_joint_positions()).reshape(-1).shape,
                                  dtype=np.float32) for r in arts}

    def make_cmd_vel_cb(key):
        def cb(msg):
            # ROS(REP103) Twist -> 이 프로젝트 body twist(vx 전진, vy 좌,
            # wz CCW+). 부호반전 없음(dock_lift_handoff_runner_v2.py 의
            # 레거시 브리지는 -msg.angular.z 로 반전했지만, v4 는 검증 결과
            # 다르다 — body_twist_toward(project wz>0 이면 yaw 증가, 즉
            # 전방벡터가 world+Z 에서 world+X 로 도는 방향=body 좌측)와
            # (forward,left,up)=(world Z,X,Y) 가 오른손좌표(Z×X=Y)라서, 이
            # project wz 는 ROS angular.z 의 CCW+(위에서 봤을 때 좌회전
            # 양수) 와 이미 같은 부호다. wheel_velocities_from_cmd_vel 의
            # SIGN_YAW=-1.0/YAW_SCALE 은 그 자체가 "명령된 wz -> 실제
            # 물리 wz" 보정이라 여기 부호와 무관(taskR2-report.md 참고).
            target_twist[key] = (float(msg.linear.x), float(msg.linear.y),
                                 float(msg.angular.z))
        return cb

    def make_lift_cb(key):
        def cb(msg):
            lift_cmd[key] = max(0.0, min(1.0, float(msg.data)))
        return cb

    cmd_sub = {r: ros_node.create_subscription(Twist, f"/robot_{r}/cmd_vel",
                                                make_cmd_vel_cb(r), 10)
               for r in arts}
    lift_sub = {r: ros_node.create_subscription(Float32, f"/robot_{r}/lift_cmd",
                                                 make_lift_cb(r), 10)
                for r in arts}

    # 측면 뎁스캠 on/off: orchestrator 가 픽업 approach~ingress 구간에만 True 를
    # 쏜다(그 밖엔 렌더 pause). set_depth_enabled(위 뎁스 셋업)로 hydra resume/pause.
    def make_depth_enable_cb(r):
        def cb(msg):
            set_depth_enabled(r, bool(msg.data))
        return cb
    depth_en_sub = {r: ros_node.create_subscription(
        Bool, f"/robot_{r}/depth_enable", make_depth_enable_cb(r), 10)
        for r in cam_robots_bridge}

    def publish_joint_states():
        for r in arts:
            art = arts[r]
            pos = np.asarray(art.get_joint_positions()).reshape(-1)
            vel = np.asarray(art.get_joint_velocities()).reshape(-1)
            names, positions, velocities = [], [], []
            for wkey, jname in WHEEL_JOINTS.items():
                i = wheel_idx[r][wkey]
                names.append(jname)
                positions.append(float(pos[i]))
                velocities.append(float(vel[i]))
            for jname in ARM_TARGETS:
                i = arm_idx[r][jname]
                names.append(jname)
                positions.append(float(pos[i]))
                velocities.append(float(vel[i]))
            msg = JointState()
            msg.header.stamp = ros_node.get_clock().now().to_msg()
            msg.name = names
            msg.position = positions
            msg.velocity = velocities
            joint_pub[r].publish(msg)

    # 0..1 초당 램프율 — 리프트가 스텝이 아니라 시간에 걸쳐 걸리게 한다
    # (dock_lift_handoff_runner_v2.py 의 arm_cmd/arm_applied 램프와 같은
    # 취지, dt 스케일만 명시적으로 함).
    LIFT_RAMP_RATE = 1.0

    def _lift_move_toward(cur, tgt, max_delta):
        d = tgt - cur
        if abs(d) <= max_delta:
            return tgt
        return cur + math.copysign(max_delta, d)

    # ---- Task R4 검증용: 트럭 리프트 GT(콘솔 전용, 제어 입력 아님) ----
    # lift_action_server(ControlLift)가 실제로 트럭을 들어올리는지 사람이
    # 육안으로(로그로) 확인할 방법이 브리지엔 없었다(로봇 GT 만 찍었음,
    # 트럭 Y 는 어디에도 없음) — R3/미션 코드의 리프트 판정 로직(3556-3585행
    # 부근, HANDOFF_VEHICLE_WHEELS 4휠 world y 평균)과 동일한 계산을 재사용해
    # 콘솔에만 찍는다(제어 경로에는 절대 안 넣는다 — 기존 GT 원칙 그대로).
    from pxr import UsdGeom as _UsdGeomLift

    def _wheel_y(name):
        return float(_UsdGeomLift.Xformable(stage.GetPrimAtPath(
            f"{HANDOFF_VEHICLE_ROOT}/{name}")).ComputeLocalToWorldTransform(
            timeline.get_current_time()).ExtractTranslation()[1])

    _truck_y0 = sum(_wheel_y(wn) for wn in HANDOFF_VEHICLE_WHEELS) / 4.0

    for _r, _a in arts.items():
        print(f"ROBOT_MASS robot={_r} total_kg={float(np.sum(_a.get_body_masses())):.4f}", flush=True)
    print(f"BRIDGE_READY robots={list(arts)} domain={os.environ.get('ROS_DOMAIN_ID', '0')} "
          f"odom_mode={odom_mode} truck_y0={_truck_y0:.4f}", flush=True)
    prev_sim = timeline.get_current_time()
    last_heartbeat = prev_sim
    last_wall = time.time()   # RTF 실측용(sim시간증분/벽시계증분)
    while app.is_running():
        app.update()
        now_sim = timeline.get_current_time()
        dt = min(0.1, max(0.0, now_sim - prev_sim))
        prev_sim = now_sim
        rclpy.spin_once(ros_node, timeout_sec=0.0)

        for r in arts:
            applied_twist[r] = slew_twist(applied_twist[r], target_twist[r], dt,
                                          linear_accel=LINEAR_ACCEL,
                                          linear_decel=LINEAR_DECEL,
                                          angular_accel=ANGULAR_ACCEL)
            omegas = wheel_velocities_from_cmd_vel(*applied_twist[r])
            buf = bridge_vel_buf[r]
            buf[...] = 0.0
            for w, om in omegas.items():
                buf[wheel_idx[r][w]] = om
            arts[r].set_joint_velocity_targets(buf)

            lift_applied[r] = _lift_move_toward(lift_applied[r], lift_cmd[r],
                                                LIFT_RAMP_RATE * dt)
            deploy_arms(arts[r], arm_idx[r], lift_applied[r])

        step_odometry(dt)
        publish_odom()
        publish_joint_states()

        if now_sim - last_heartbeat >= 2.0:
            now_wall = time.time()
            rtf = (now_sim - last_heartbeat) / max(1e-6, now_wall - last_wall)
            last_heartbeat = now_sim
            last_wall = now_wall
            gt_str = " ".join(
                f"{r}=({x:.2f},{z:.2f},{math.degrees(yaw):.1f})"
                for r, (x, z, yaw) in ((r, gt_pose_xz_yaw(arts[r])) for r in arts))
            cmd_str = " ".join(f"{r}=({vx:.2f},{vy:.2f},{wz:.2f})"
                               for r, (vx, vy, wz) in target_twist.items())
            _truck_y = sum(_wheel_y(wn) for wn in HANDOFF_VEHICLE_WHEELS) / 4.0
            print(f"BRIDGE_ALIVE t={now_sim:.1f} rtf={rtf:.2f} robots={len(arts)} "
                  f"cmd=[{cmd_str}] gt=[{gt_str}] "
                  f"truck_y={_truck_y:.4f} truck_rise={_truck_y - _truck_y0:.4f}",
                  flush=True)
    app.close()
    return


if __name__ == "__main__":
    main()
