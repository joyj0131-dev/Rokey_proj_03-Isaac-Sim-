# 프로젝트 인수인계 — 현재 Isaac Sim 머신 기준

> 최종 확인·갱신: 2026-07-21 (Asia/Seoul)
>
> 이 문서는 `/home/rokey/cobot3_ws`에서 직접 확인한 파일과 실행 결과를 기준으로 한다.
> 이전 인수인계에 적힌 서비스 설계 방향은 유지하되, 이 머신에 없는 코드·문서는
> **이전 트랙 기록**으로 명확히 구분한다.

---

## 0. 프로젝트 정체성

이 프로젝트의 목표는 단순 Isaac Sim 데모가 아니라 **실제 자동발레파킹 서비스**
(Stan/현대위아식 B2B)를 구현하는 것이다. Isaac Sim은 서비스의 기구·인식·측위·관제·ROS 2
연동을 개발하고 검증하는 환경이다.

- 주차 구획, 천장 카메라, 로봇은 운영 인프라이므로 마커 부착과 사전 측량이 가능하다.
- 고객 차량에는 마커를 붙일 수 없으므로 최종 차량 인식은 markerless 방식이어야 한다.
- ArUco/ground truth는 시뮬레이션 부트스트랩과 정답 데이터 생성용으로만 사용한다.
- SLAM, 실제 차량 감지, 바퀴 감지, 정밀 도킹은 최종 시스템의 핵심 모듈이다.

---

## 1. 현재 머신/저장소 상태

| 항목 | 현재 확인값 |
|---|---|
| 워크스페이스 | `/home/rokey/cobot3_ws` |
| Git 브랜치 | `feat/camera` |
| Isaac Sim | 5.1 |
| Isaac Sim 실행 경로 | `/home/rokey/dev_ws/isaac_sim/isaacsim/_build/linux-x86_64/release` |
| 현재 ROS 2 소스 패키지 | `src/M0609`만 확인됨 |
| parking mission 코드 | `parkbot_core`, `parkbot_interfaces`, `robot_agent`, `mock_gwanje` 모두 이 머신에 없음 |
| 이전 설계 문서 | `park.md`, `superpowers/specs/*`, `superpowers/plans/*`, `asset-prompts/*` 이 머신에 없음 |
| Git 상태 주의 | `isaacpjt/*`, `src/M0609` 등 현재 주요 파일이 untracked 상태 |

### 이전 HANDOFF 기록과의 관계

이전 문서에는 `design/two-robot-park-mission` 브랜치, `parkbot_core`/`parkbot_interfaces`,
mock E2E 및 12개 테스트 통과가 기록되어 있었다. 이는 **다른 브랜치·다른 머신 또는 아직
가져오지 않은 작업물의 기록**이다. 현재 워크스페이스에서는 해당 코드와 테스트를 재검증할 수 없다.

통합 작업을 시작하기 전에 해당 브랜치/패키지/설계 문서를 원본 저장소나 팀원 머신에서 가져와야 한다.

---

## 2. 현재 머신에서 확인된 에셋

### 주차장 — **2026-07-20 팀원 제작 버전으로 교체됨**

`parking/` 폴더 전체를 팀원이 만든 버전으로 교체했다. 기존 주차장 에셋은 더 이상 쓰지 않는다.
(교체 과정에서 이전 폴더에 있던 `parking_environment_mechanical_test.usd`는 사라졌으나,
리프트 테스트가 매 실행마다 원본에서 다시 생성하므로 문제되지 않는다.)

- 환경: `isaacpjt/Isaac_envo/parking/parking_environment.usd`
- 생성 스크립트: `isaacpjt/Isaac_envo/parking/build_parking_environment.py` (약 1070줄)
- 통합 필드: `isaacpjt/Isaac_envo/parking/build_integrated_parking_field.py`
- 천장 라이다 데모: `isaacpjt/Isaac_envo/parking/run_ceiling_lidar_demo.py`

실측 명세:

| 항목 | 값 |
|---|---|
| 실내 바닥 | 36.20 × 24.40 m (슬롯 3.40 × 6.60, 통로 9.00) |
| 주차면 | 16개 (A1–A8 / B1–B8). A1·A2는 교통약자 |
| 로봇 도크 | 4개 (양 끝 index 0=대기, 9=충전) |
| 기둥 | 12개, z=±10.86, x 간격 6.8 m (주차칸 **뒤쪽**에 위치) |
| 천장 라이다 | **2대** `/World/Sensors/CeilingLidar{West,East}` (SICK_multiScan136, 온라인 참조) |
| **서측 외부 인계장** | `VehicleHandoffArea` 23.0 × 11.4 m, 수용 6대. 서측 벽 9 m 개구부로 연결 |
| 초기 차량 | **12대** — 실내 6대(`driver_parked` 2 / `robot_parked` 4) + 외부 대기 6대(`driver_dropoff_waiting`) |
| Navigation | `robot:start (-15.3,0,7.8)`, `vehicle:handoff (-21.85,0,2.35)`, `parking:target A5` |

즉 팀원 에셋은 **"운전자가 외부 인계장에 차를 두면 로봇이 받아 실내에 주차한다"**는 서비스 전체
흐름을 씬으로 모델링해 두었다.

> **팀원에게 전달 필요**: `parking:target = A5`인데 그 자리에 `A5_Coupe`가 이미 주차돼 있다.
> 빌더의 `TARGET_INDEX = 5` 잔재로 보이며, 서로 조율되지 않은 상태다.

주의: 차량·PhysicsScene 참조가 **절대경로로 구워져 있어 이 머신에서 전부 끊겨 있었다.**
상대경로로 고쳤다(3절 "팀원 에셋 통합" 참고). 천장 라이다 2대는 여전히 온라인 S3를 참조하므로
오프라인 환경에서는 로컬 에셋으로 교체하는 것이 안전하다.

### 차량

- 차량 묶음: `isaacpjt/Isaac_envo/fab_vehicles.usd`
- 생성 스크립트: `isaacpjt/Isaac_envo/build_fab_vehicles.py`
- 테스트 차량: FAB 차량 묶음의 `Sedan`

10종 실측 제원 (모두 로컬 좌표 기준):

| 차종 | 충돌체 | 타이어반경 | 타이어폭 | 축거 | 윤거 | 질량 |
|---|---|---:|---:|---:|---:|---:|
| Pickup | Cylinder | 0.423 | 0.357 | **3.594** | 1.641 | 2150 |
| Minivan | Cylinder | 0.374 | 0.526 | 3.196 | 1.571 | 1900 |
| SUV | Cylinder | **0.440** | 0.601 | 3.073 | 1.629 | 2050 |
| Wagon | Cylinder | 0.349 | 0.612 | 2.963 | 1.576 | 1550 |
| **Sedan** | Cylinder | 0.343 | **0.256** | 2.951 | 1.569 | 1600 |
| Sport | Cylinder | 0.370 | 0.679 | 2.759 | 1.523 | 1380 |
| Coupe | Cylinder | 0.354 | 0.504 | 2.715 | 1.492 | 1450 |
| Hatchback | Cylinder | 0.341 | 0.580 | 2.705 | 1.557 | 1250 |
| Offroad | Cylinder | 0.419 | 0.571 | 2.611 | 1.654 | 1850 |
| Compact | Cylinder | 0.277 | 0.494 | **2.344** | 1.230 | 1100 |

- **타이어 반경이 0.277~0.440 m로 1.6배** 차이 난다 → arm pivot·roller 간격 일반화 필요(7절 리스크)
- **축거가 2.344~3.594 m로 1.25 m** 차이 난다 → 2로봇 간 거리가 차종마다 달라진다
- Sedan만 타이어 폭 0.256 m로 나머지(0.49~0.68)의 절반이다. 리프트 검증이 Sedan 기준이므로
  다른 차종은 bearing roller와의 접촉 폭이 크게 달라진다.

**휠 충돌체는 10종 전부 `Cylinder`(축 X)이며, 이것이 현재 물리 불안정의 근본 원인이다.**
PhysX에는 원기둥 프리미티브가 없어 면진 볼록체로 근사되므로, 구동계를 제거해 원통이 실제
접촉체가 되면 서스펜션 없는 차체가 면 사이를 넘나들며 영구 진동한다(3절·`DEBUG_LOG.md` 참고).

### 주차로봇

- URDF: `isaacpjt/hwia_parking_robot_final_caster_package/hwia_parking_robot_final_caster.urdf`
- USD: `isaacpjt/hwia_parking_robot_final_caster_package/hwia_parking_robot_final_caster.usd`
- USD 구성 레이어: `isaacpjt/hwia_parking_robot_final_caster_package/configuration/`
- 설명: `isaacpjt/hwia_parking_robot_final_caster_package/hwia_parking_robot_final_caster_README.md`

현재 구조:

- 링크 25개, 조인트 24개
- `base_link`에 직접 연결된 swing arm 4개
- 암 위치 명령: 접힘 0 rad, 전개 약 ±π/2
- arm-tip bearing roller 4개: passive continuous joint
- arm-tip swivel caster 4세트: passive swivel + passive wheel joint
- 구동 휠 4개
- 카메라 링크 4개(전방, QR 하향, 좌/우 depth 위치)

현재 반영된 주요 물리/형상 수정:

- 베이스 폭을 0.73 m로 정리하고 암과의 중첩 제거
- 베이스 하부 충돌체에 약 30 mm 지상고 확보
- 좌/우 depth 카메라가 베이스에 붙도록 위치 수정
- 원통 충돌축 해석 문제를 피하도록 구동 휠·캐스터 충돌 안정화
- bearing roller 접촉 폭을 여러 구형 충돌체로 구성
- 앞/뒤 arm pivot을 차량 타이어 리프트에 맞게 X=±0.27 m로 조정
- 네 swing arm에 position drive 설정
- roller/caster는 drive 없이 passive로 유지

---

## 3. 완료된 1로봇 뒷바퀴 리프트 통합 테스트

관련 파일:

- 테스트 스크립트: `isaacpjt/Isaac_envo/parking_robot_rear_lift_test.py`
- 저장된 테스트 장면: `isaacpjt/Isaac_envo/parking_robot_rear_lift_test.usd`
- 결과 보고서: `isaacpjt/Isaac_envo/parking_robot_rear_lift_test_report.json`

테스트 구성:

- 주차장 **A7** 구역 `(8.5, 0, 7.8)` — 07-20 변경. 팀원 에셋이 A5를 `A5_Coupe`로 점유하기 때문
- Sedan 1대 — 권장 구성은 `--sphere-wheels`(구동계 제거 + 휠 충돌체를 구로 교체)
- 주차로봇 1대 — **메카넘 에셋** `hwia_parking_robot_final_caster_mecha_roller.usd` (07-20 변경)
- 로봇을 Sedan 뒷축 아래로 진입
- 네 swing arm을 점진적으로 전개
- 앞/뒤 바퀴 중심 높이 변화를 측정해 뒷바퀴 리프트 판정

**현재 유효 수치 (2026-07-20, A7 + 메카넘 + `--sphere-wheels`)**

| 검사항목 | 결과 | 기준 |
|---|---:|---|
| 전체 테스트 | **PASS** (`--replay-smoke` 2회 연속) | |
| arrival 오차 | 0.003987 m | < 0.15 |
| 뒷바퀴 평균 상승 | 0.028959 m | ≥ 0.025 (마진 16%) |
| 앞바퀴 평균 상승 | 0.000011 m | 뒤 > 앞 + 0.012 |
| Sedan 진동 (settle 이동경로) | 0.00002 m | — |

<details><summary>과거 수치 (참고용, 조건이 다름)</summary>

| 시점 | 조건 | 뒷축 상승 | arrival |
|---|---|---:|---:|
| ~07-18 | CPU PhysX, A5, 베이스 휠 | 0.048456 | 0.019935 |
| 07-19 | GPU PhysX, A5, 베이스 휠 | 0.031323 | 0.0444 |
| 07-20 | GPU, A7, 베이스 휠, 팀원 에셋 | 0.028208 | 0.062840 |
| 07-20 | GPU, A7, 메카넘, 원통 충돌체 | 0.004779 | 0.6056 → **FAIL** |
| 07-20 | GPU, A7, 메카넘, `--keep-drivetrain` | 0.026486 | 0.001147 |

</details>

### 테스트의 한계

- 현재 진입 구간은 휠을 회전시키면서 로봇 차체를 주차선 중심으로 직선 유도한다.
- 이는 CPU PhysX에서 구형 휠 충돌체의 횡방향 드리프트를 리프트 시험과 분리하기 위한 테스트 장치다.
- 도착 후 유도는 완전히 해제되며, arm 전개와 차량 리프트는 unconstrained PhysX 접촉으로 검증한다.
- 따라서 실제 `/cmd_vel` 기반 자율 진입·조향·정밀 도킹은 아직 검증되지 않았다.
- 현재는 로봇 1대가 뒷축만 드는 시험이며, 차량 앞/뒤에 로봇 2대를 배치한 운반 시험은 아니다.

### GPU PhysX 전환 (2026-07-19)

이 머신은 RTX 5080 Laptop(16GB)이므로 CPU PhysX가 필요 없다. 리프트 테스트의 물리 씬을
GPU로 전환했다: `CreateBroadphaseTypeAttr("GPU")` + `CreateEnableGPUDynamicsAttr(True)`.

- headless 1회 PASS, `--replay-smoke` 2회 모두 PASS(수치 0.031323로 결정론적 동일)
- GPU 접촉 해석 차이로 뒷바퀴 평균 상승 0.048456 → 0.031323 m, arrival 오차 0.0199 → 0.0444 m
- 통과 기준(뒷축 상승 ≥ 0.025, 뒷 > 앞 + 0.012, arrival < 0.15)은 모두 만족
- 위 표의 CPU 수치는 전환 이전 기록이다

### 메카넘(옆이동) 휠 검증 (2026-07-19)

원래 구동 휠 4개는 축 Y 고정 일반 휠이라 횡이동이 불가능했다. 원본 에셋은 수정하지 않고,
재사용 모듈 `isaacpjt/Isaac_envo/mecanum_drive.py`(롤러 authoring + `/cmd_vel`→휠 역기구학 +
허브 드라이브 설정)로 각 구동 휠에 45° passive 롤러 10개(메카넘)를 **테스트 스테이지 override로만**
얹어 GPU PhysX에서 검증했다.

- `mecanum_strafe_test.py`: strafe로 횡이동(world X) 약 4.37 m, 전진 드리프트 0.12 m, 수직 ~0 m. 2회 동일 PASS
- `mecanum_holonomic_test.py`: `/cmd_vel`(vx/vy/wz)로 전진(1.20 m)·좌 strafe(1.18 m, 부호 정확)·제자리 회전 검증, 교차간섭 ~0. PASS
- IK 캘리브레이션: SIGN_FORWARD=+1, SIGN_STRAFE=-1, SIGN_YAW=+1. 제자리 회전은 롤러 슬립 지배라
  YAW_SCALE=1.12로 wz~0.5 동작점을 맞춤(명령 0.5rad/s → 실측 ~0.52rad/s, 89.9°/3s)
- **baked 에셋**: `hwia_parking_robot_final_caster_mecha_roller.usd` — **flatten 자체포함**(원본 참조
  없이 지오메트리+롤러 40개+마찰재가 한 파일에 구워짐, 원본 없어도 열림). 빌더
  `build_mecha_roller_asset.py`(Flatten+Export, self-containment 검증 포함). 원본 에셋은 미수정.
- **2로봇 운반 데모**: `two_robot_carry_demo.py` — 주차장 A5 + Sedan(1600kg) + 메카넘 로봇 2대가
  앞/뒤 축을 잡고 **실제 메카넘 구동으로 전진 1.04 m** 운반(유도 폴백 불필요, 차량 안정). `--gui`로 관람.
  데모 영상(GIF)도 캡처됨: `isaacpjt/Isaac_envo/two_robot_carry.gif`(레코더 `record_carry_demo.py`).
- 남은 것: 롤러를 로봇 소스 에셋에 영구 반영할지 결정(현재는 별도 mecha_roller 에셋으로 분리)

### ROS 2 제어 스택 (2026-07-20) — 검증 완료

외부 머신에서 `/cmd_vel`을 발행해 Isaac 안의 메카넘 로봇을 실제로 주행시키고, 서비스로 팔을
여닫는 것까지 동작 확인했다. 상세 문서: `isaacpjt/Isaac_envo/MECANUM_ROS2_README.md`.

**구성:** Isaac Sim 5.1은 Python 3.11, ROS 2 Humble은 3.10이라 Isaac 안에서는 시스템 `rclpy`를 쓰지
않는다. `/cmd_vel`은 **C++ OmniGraph 브리지**(`ROS2SubscribeTwist`)로 받아 시뮬 루프에서 메카넘
역기구학으로 휠을 구동하고, 팔 서비스만 **Isaac 내부 rclpy(3.11)** 로 띄운다. 외부는 평범한
Humble(3.10)을 쓰며 둘은 DDS로 통신한다.

| 파일 | 역할 |
|---|---|
| `mecanum_ros2_drive.py` | Isaac 드라이버: `/cmd_vel` 구독 → IK → 휠, `/arm_control` 서비스 |
| `run_mecanum_ros2_drive.sh` | 팀 env(도메인/rmw/화이트리스트/내부 humble libs) 세팅 후 실행 |
| `mecanum_teleop_key.py` | 외부(시스템 ROS 2) 방향키 텔레옵: 화살표 주행, `a/d` 회전, `o/c` 팔 |

**환경(양쪽 동일):** `ROS_DOMAIN_ID=126`, `RMW_IMPLEMENTATION=rmw_fastrtps_cpp`,
`FASTRTPS_DEFAULT_PROFILES_FILE=$HOME/.ros/fastdds_whitelist.xml`(폐쇄망 `10.10.0.1~5` + loopback).
Isaac 쪽 터미널은 `/opt/ros`를 소싱하지 않고 내부 Humble 라이브러리를 쓴다(런처가 처리).

**팔 제어:** `/arm_control` (`std_srvs/srv/SetBool`). `data: true`면 네 swing arm을 ±90°로 전개하고
**다 열린 뒤** `success: true, "arms fully opened"`로 응답(접기는 `false` → `"arms folded"`).

**검증:** `/cmd_vel` 전진 0.83 m, 좌 strafe 0.85 m, 제자리 회전 정상. 팔 서비스 open/close 모두
완료 응답 수신.

> 실행 중 막혔던 문제와 해결 과정은 `DEBUG_LOG.md` 참고(특히 Isaac 좀비 프로세스, 브리지 기동 실패).

### 팀원 주차장 에셋 통합 (2026-07-20) — 일부 완료

`parking/` 폴더를 팀원 버전으로 교체하면서 여러 통합 문제가 드러났다. 상세 경과는 `DEBUG_LOG.md`.

**고친 것**

| 문제 | 조치 |
|---|---|
| 차량 12대 물리 유실 (절대경로 참조가 죽어 있음) | 참조 4곳을 `../fab_vehicles.usd` 상대경로로 전환, 폴백 상수 제거 |
| 리프트 타깃 A5에 팀원 차량이 이미 주차 | 테스트 타깃을 **A7 `(8.5, 0, 7.8)`** 로 이동 |
| 천장 라이다 비활성화가 이름 불일치로 무력화 | `CeilingLidar*` 접두사 순회 방식으로 변경 |
| 쇼룸 바닥 디스크 `Cylinder001`이 Y=0에서 Z-fighting | 테스트 스테이지에서 비활성화(원본 미수정) |
| 통합 필드가 폴백으로 로봇 종류를 바꿔치기 | 폴백 제거, `PROJECT_ROOT` 경로 수정 |
| 스크립트마다 로봇 에셋이 제각각 | **3개 모두 `_mecha_roller.usd` + 상대참조로 통일** |

**현재 통과 상태**

| 테스트 | 결과 |
|---|---|
| `build_integrated_parking_field.py --headless-test` | **PASS** (로봇 변위 0.0001 m, 차량 12대 최대 0.0925 m) |
| `parking_robot_rear_lift_test.py --sphere-wheels` | **PASS** (뒷축 0.028959, arrival 0.003987, 2회 재현) |
| `two_robot_carry_demo.py` (기본=구동계 유지, A5_Coupe) | **PASS** (리프트 0.0849 m, 추종률 1.000) |
| `two_robot_carry_demo.py --target B5_Pickup` | **FAIL** — 팔이 타이어를 못 듦(0.4 mm) |
| 플래그 없이 실행 | **FAIL** — 차량 진동으로 로봇이 0.6 m 밀려 앞축을 듦 |

### 2로봇 운반 기본 구성 확정 (2026-07-20 저녁)

`two_robot_carry_demo.py`의 기본값을 **"주차된 차량을 구동계 유지한 채로 집어 옮긴다"**로
바꿨다. 자체 스폰 Sedan 경로는 **삭제**했다(Y=0.0에 놓여 구동계 유지 시 서스펜션이 지면에
파묻힌 채 시작 → 실제 구동 0.056 m, 차량 미상승. 실제 서비스 시나리오도 아님).

```bash
cd /home/rokey/cobot3_ws/isaacpjt/Isaac_envo
python3 two_robot_carry_demo.py                        # 기본: A5_Coupe, 구동계 유지
python3 two_robot_carry_demo.py --gui                  # GUI (Play로 시작/반복)
python3 two_robot_carry_demo.py --gui --show-colliders # 충돌체 와이어프레임까지
python3 two_robot_carry_demo.py --target B5_Pickup     # 대상 차량 변경
```

대상 차량의 축 위치는 상수가 아니라 **실제 휠 좌표에서 계산**한다(차종마다 축거가 다름).
nose-in/nose-out과 무관하게 z가 작은 축을 −Z에서 진입하는 로봇이 맡는다.

**차종별 실측 — 일반화가 안 되어 있다**

| 대상 | 축거 | 리프트 | 추종률 | 판정 |
|---|---:|---:|---:|---|
| **A5_Coupe** | 2.715 | **0.0849 m** | **1.000** | **PASS** |
| B5_Pickup | 3.594 | 0.0004 m | 0.033 | **FAIL** — 못 듦 |
| B3_Minivan | 3.196 | 0.0584 m | 946 | **FAIL** — 운반 실패 |

**검증된 것은 A5_Coupe 한 차종뿐이다.** Pickup(타이어 반경 0.440, 축거 3.594)은 팔이 아예
타이어를 들지 못한다. 7절의 "차량 종류/타이어 지름 변화에 따른 arm pivot 일반화" 리스크가
구체적으로 확인된 것이다.

> **합격 판정 버그 수정**: 이전에는 `total_moved`만 봐서, 실제 구동이 실패해도 **유도 폴백이
> 차를 대신 옮겨 PASS로 찍혔다**(B5_Pickup: 리프트 0.4 mm·추종률 0.033인데 PASS).
> 지금은 `real_drive_no_fallback` / `car_actually_lifted` / `car_followed_robot`을 모두 요구한다.
> 불합격 시 어떤 항목이 걸렸는지 출력한다.

**진단 지표** (리포트 + 콘솔): `car_lift_m`(실제 상승), `carry_follow_ratio`(1.0=함께 이동,
0=두고 감), `grip_jitter_ratio`(파지 중 진동), `parked_max_drift_m`(대상 제외 나머지 11대 변위).

**GUI 충돌체 시각화**: standalone SimulationApp은 물리 디버그 시각화가 꺼진 채 뜨고 뷰포트
메뉴에도 항목이 없다. `--show-colliders`가 carb 설정을 직접 켠다
(`SETTING_DISPLAY_COLLIDERS=VisualizerMode.ALL`, `SETTING_VISUALIZATION_COLLISION_MESH=True`).

**휠 충돌체 대안은 모두 기각됐다**

| 방식 | 결과 |
|---|---|
| 원통(기본, 구동계 제거 시) | 차량 영구 진동 |
| 구(`--sphere-wheels`) | 안정적이나 폭 2.7배 부풀음 |
| Convex Decomposition | **구동계 차량에서 폭발** (변위 2622 m) |
| Convex Hull | **동일하게 폭발** (1627 m) — 조각 개수 문제가 아님 |
| **구동계 유지 (현재 기본)** | **가장 안정. 원통 접촉 자체가 발생하지 않음** |

PhysX Vehicle이 붙은 휠에는 **Mesh 충돌체를 걸 수 없다**(`the sub prim with CollisionAPI
applied has to be a direct child of the wheel attachment prim`, 직속 자식으로 옮겨도 폭발).
따라서 `fab_vehicles.usd`에 볼록체를 굽는 계획은 폐기했다.

---

**`--sphere-wheels` (구동계 제거 시에만 유효)**

구동계는 제거한 채(차가 자유 강체로 남아야 실려 간다) **휠 충돌체만 원통 → 구로 교체**한다.
로봇이 이미 쓰던 방식과 동일하다(구동휠 collision `sphere r=0.060`, 베어링 롤러 `sphere` 3개).
구는 PhysX 네이티브 프리미티브라 면진 근사가 없어 rocking이 원천적으로 발생하지 않는다.

폭이 부풀지만(반경 0.343 → 폭 0.686 vs 타이어 0.256) 이는 휠 중심 높이에서의 최악값이고,
팔 롤러가 실제 작업하는 y=0.045 높이에서는 0.340 m로 원통 대비 **1.33배**에 그친다.
로봇 구동휠도 2.5배 부풀린 채 문제없이 쓰고 있다.

| | 원본(원통) | `--keep-drivetrain` | **`--sphere-wheels`** |
|---|---:|---:|---:|
| Sedan 진동 (settle 이동경로) | 0.632 m | 0.029 m | **0.00002 m** |
| 리프트 뒷축 상승 | 0.004779 ✗ | 0.026486 (마진 6%) | **0.028959 (마진 16%)** |
| 운반 실제 구동 | −2.75 ✗ | 0.056 (폴백 의존) | **1.031 (폴백 불필요)** |
| 운반 차량 리프트 | 0.868 공중부양 ✗ | −0.018 (안 들림) | **+0.0295** |

**`--keep-drivetrain` (리프트 전용 우회책, 비권장)**

PhysX Vehicle 구동계를 **제거하지 않고 살려두는** 옵션. 레이캐스트 서스펜션이 차체를 지지해
원통 접촉이 발생하지 않으므로 리프트 테스트는 통과한다. 그러나 **차가 지면에 묶여 운반이 안 되고**
(실제 구동 0.056 m, 차량 미상승), 리프트가 (a) 타이어가 팔 롤러에 눌린 것인지 (b) 서스펜션 광선이
팔을 맞아 얹힌 것인지 모호하다. 비교·진단용으로만 남긴다.

**정정 기록**: 로봇이 0.6 m 밀려난 것을 처음엔 "메카넘 passive 롤러가 팔 반작용을 못 버티는
설계 한계"로 판단했으나 **틀렸다.** 떨고 있던 차량이 팔을 통해 로봇을 밀어낸 2차 효과였고,
차량이 안정되자 로봇 떨림은 0.0001 m로 측정됐다. 메카넘 자체는 문제가 없다.

**진단 도구**: `isaacpjt/Isaac_envo/diag_jitter.py` — 물체별로 drift(순변위)와 path(이동경로 총합)를
나눠 재서 표류와 진동을 분리한다. 원본 파일을 건드리지 않고 리프트 테스트 스테이지를 재사용한다.

---

## 4. GUI 실행 및 반복 재생

```bash
cd /home/rokey/cobot3_ws/isaacpjt/Isaac_envo
python3 parking_robot_rear_lift_test.py --gui --sphere-wheels
```

스크립트가 자동으로 Isaac python으로 갈아타므로 `python.sh`를 직접 부를 필요는 없다.
`--sphere-wheels` 없이 실행하면 차량이 계속 떨고 로봇이 밀려나 실패하는 모습을 비교로 볼 수 있다.

동작 방식:

1. GUI가 초기 상태로 열리고 정지 상태에서 대기한다.
2. 사용자가 Play 버튼을 누르면 저장된 초기 USD를 다시 연다.
3. 로봇 진입 → arm 전개 → 뒷바퀴 리프트를 실행한다.
4. 완료 후 자동으로 Stop 상태가 된다.
5. Play를 다시 누르면 초기 상태부터 반복한다.
6. 재생 직전 활성 viewport의 카메라 위치·방향을 저장하므로 반복 실행해도 사용자가 보던 시점이 유지된다.

주의: `parking_robot_rear_lift_test.usd`만 직접 열면 Python 제어기가 없으므로 자동 시퀀스와 반복
재생이 동작하지 않는다. 반드시 위 Python 스크립트를 `--gui`로 실행해야 한다.

헤드리스 검증(전부 `isaacpjt/Isaac_envo`에서):

```bash
python3 parking_robot_rear_lift_test.py --sphere-wheels        # 리프트 (PASS)
python3 parking_robot_rear_lift_test.py --replay-smoke --sphere-wheels     # 반복 재생 2회 PASS
python3 two_robot_carry_demo.py --sphere-wheels                 # 2로봇 운반 (PASS, 폴백 불필요)
python3 parking/build_integrated_parking_field.py --headless-test          # 통합 필드 (PASS)
python3 diag_jitter.py [--sphere-wheels|--keep-drivetrain]      # 물체별 떨림 진단
```

주차장 에셋을 다시 구우려면:

```bash
python3 parking/build_parking_environment.py --headless
```

이 머신(RTX 5080)에서는 GPU PhysX로 실행한다. (이전 Codex 헤드리스 환경은 CUDA가 없어 CPU
PhysX로 돌렸던 기록이며, 이 머신엔 해당하지 않는다. 3절 "GPU PhysX 전환" 참고.)

---

## 4-A. Phase 1 ArUco 바닥 마커 설계 (2026-07-20)

Phase 1은 **SLAM/nav2 없이** 바닥 ArUco로 주행한다. 목표는 측위 최적화가 아니라
워크플로 완성이다: 서비스 요청 → 차량 접근 → 하부 진입 → 파지 → 운반 → 지정 칸 하차.
Phase 2에서 SLAM/nav2를 얹어 이동 장애물을 다룬다.

> **가장 중요한 설계 원칙**: Phase 1의 ArUco 주행기를 `NavigateToPose` 인터페이스 뒤에
> 숨길 것. 그래야 Phase 2가 재작성이 아니라 **백엔드 교체**가 된다. 미션 코드가 마커 ID를
> 직접 참조하거나 `/cmd_vel`을 직접 쏘기 시작하면 전부 다시 써야 한다.
> ArUco는 버리는 작업이 아니다 — 0절대로 인프라에는 마커 부착이 가능하고, 나중에
> markerless로 바뀌는 것은 **고객 차량 인식**이지 로봇 측위가 아니다.

### 관련 파일

| 파일 | 역할 |
|---|---|
| `isaacpjt/Isaac_envo/marker_layout.py` | **배치 정의 (단일 출처)**. 순수 파이썬, 의존성 없음 |
| `isaacpjt/Isaac_envo/build_marker_layout.py` | USD override 생성 (원본 비파괴, GPU 불필요) |
| `isaacpjt/Isaac_envo/plot_marker_layout.py` | 평면도 SVG 생성 |
| `isaacpjt/Isaac_envo/marker_layout_plan.svg` | 생성된 평면도 |
| `isaacpjt/Isaac_envo/parking/parking_environment_marker_preview.usd` | 원본 + 마커 override |

```bash
cd /home/rokey/cobot3_ws/isaacpjt/Isaac_envo
python3 marker_layout.py        # 좌표 표로 확인
python3 plot_marker_layout.py   # 평면도 갱신
python3 build_marker_layout.py  # USD override 갱신
```

### 배치 — 총 43장

| 종류 | 수량 | 위치 | 상태 |
|---|---:|---|---|
| 슬롯 마커 | 16 | z = ±2.50, x = A1–A8/B1–B8 중심 | **확정** |
| 도크 마커 | 4 | z = ±2.50, x = ±15.30 | **확정** |
| 게이트 마커 | 2 | x = −18.10, z = ±2.50 (서측 개구부) | 확정 |
| 횡단 마커 | 3 | z = 0, x = −15.3 / 0 / +15.3 | **위치 미확정** |
| 인계 베이 기준점 | 6 | 각 베이 동쪽 끝, z = ±2.35 | 제안 |
| 인계장 우회 차선 | 12 | z = ±4.96, x 3.40 m 간격 | **라우팅 미확정** |

**핵심 통찰**: 슬롯이 x축 3.40 m 등간격으로 붙어 있어, **슬롯 앞에 하나씩만 놓아도
통로를 따라 차선이 자동으로 생긴다.** 별도 차선 마커를 깔 필요가 없다.

**왜 2 m 앞(z=±2.50)인가** — 리드 로봇이 이 열 위를 주행할 때 적재 차량이 옆칸 주차
차량(z=±4.50)과 간섭하면 안 된다.

| 마커 위치 | 적재 차량 앞끝 | 옆칸까지 |
|---|---:|---|
| 슬롯 앞 1 m (z=3.5) | 4.55 | **5 cm 간섭** |
| **슬롯 앞 2 m (z=2.5)** | 3.48 | **1.02 m 여유** |

Sedan 축거 2.951 m(실측) 기준이며, 최장 Pickup(3.594 m)으로 검산해도 0.88 m 남는다.

**왜 3.40 m 간격인가** — 메카넘 오도메트리가 못 버틴다. 자체 측정치로 오차 3~12%
(YAW_SCALE 보정 12%, strafe 드리프트 2.7%):

| 보정 간격 | 구간 누적 드리프트 |
|---|---|
| **3.40 m (슬롯 간격)** | **0.10 ~ 0.41 m** ✓ |
| 36.2 m (무보정) | 1.09 ~ 4.34 m ✗ (슬롯 폭 3.40 m 초과) |

### 카메라 설계 — **아직 미구현** (URDF 그대로)

현재 URDF는 논의 이전 상태다. `cam_qr_down_link`는 바닥 위 5 cm라 시야가 5.8 cm뿐이라
**쓸 수 없고**, `cam_front_link`는 `rpy 0 0 0`으로 수평이다.

| 경사각 | 관측 구간(h=0.09) | 판정 |
|---|---:|---|
| 30° | ∞ | 수평선에 걸려 발산 |
| **45°** | **0.31 m** | **최적** |
| 60° | 0.16 m | 근시안 |
| 90°(하향) | 0.10 m | 최악 |

높이가 각도보다 중요하다 — 0.09 → 0.15 m로 올리면 같은 45°에서 0.31 → **0.52 m**.
경사 시야는 평면 자세 모호성도 해소해 정면보다 pose가 안정적이다. 해상도는 원거리
끝에서도 셀당 약 11 px로 여유가 크다(검출 하한 3~4 px).

**결정된 구성(미구현)**: 전방 45° 하향 + **좌우 45° 외향하향 추가**, `cam_qr_down` 폐기,
카메라 양옆에 LED. 카메라 총수 4 → 5.

좌우를 추가하는 이유는 **메카넘 횡이동에 전방 시야가 없기 때문**이고, 횡이동은 선택이
아니라 필수다 — 주차 슬롯의 차량 길이축(z)과 통로 진행 방향(x)이 **직교**하므로,
통로 운반 구간(최대 30 m)은 차가 옆으로 게걸음쳐야 한다. 회전으로 대체하면 5절의
`차량을 회전시키지 않는 평행이동` 결정을 위반한다. (단 "회전 금지"는 **차량**에 걸린
제약이고, 무적재 로봇은 자유롭게 회전 가능하다.)

### 열린 항목

- 횡단 마커 3장 위치 — A열↔B열 전환을 어디서 할지 미정
- 인계장 우회 차선 라우팅 — 베이 열 사이 x 간격이 **0.70 m**뿐이라 통과 불가.
  남북 바깥 여백(z≈±4.96)을 쓰는 안을 그려뒀으나 검증 안 됨
- 마커 크기·ID 체계·ArUco 사전 미확정 (현재 placeholder 0.25 m, `markerId = -1`)
- 카메라 지상고 0.09 유지 vs 0.15 상향 — 차 하부 여유고를 재야 정할 수 있고,
  격자 간격이 0.31 vs 0.52 m로 갈린다
- 슬롯 진입 구간(z방향 6.6 m)에 마커가 1장뿐이라 드리프트 0.20~0.79 m. 다만 그 구간은
  좌우 depth 카메라의 바퀴 폐루프 정렬이 담당하므로 설계상 구멍은 아니다

---

## 5. 유지할 서비스 설계 결정(이전 트랙에서 인계)

| 항목 | 결정 |
|---|---|
| 로봇 구성 | 동일 로봇 2대가 앞축/뒤축을 각각 담당해 차량 1대 운반 |
| 운반 구동 | 옴니/메카넘 기반, 차량을 회전시키지 않는 평행이동 (메카넘 롤러 횡이동 sim 검증 완료 2026-07-19, 로봇 통합은 진행 중) |
| 조율 | A=다중 로봇 관제, B=로봇 1대 자율주행·인식, C=arm 제어 |
| A↔B | 표준 `NavigateToPose`(approach/carry), 상태 `/robot_state` |
| B→C | 도킹 완료와 바퀴 위치를 `/docking_state`로 전달 |
| arm 명령 | `/joint_command`, 접힘 0, 전개/클램프 약 ±π/2 |
| 바퀴 감지 | 로봇 양옆 depth 카메라 + 옴니 폐루프 정렬 + arm-tip 확인 |
| 진입 | 같은 방향으로 줄지어 진입, 리드=먼 축, 팔로워=가까운 축 |
| 차량 인식 | 최종 markerless segmentation/OBB, 마커는 부트스트랩용 |
| 측위 | ground truth 개발 → SLAM/맵 측위 + 천장 카메라 절대기준 융합 |

`prismatic lift`로 적힌 과거 설계가 있다면 현재 에셋과 다르다. 실제 구조는 별도 리프트 조인트 없이
네 swing arm과 bearing roller가 타이어 아래로 진입해 들어 올리는 방식이다.

---

## 6. 다음 작업 순서

### P0 — 누락된 미션 코드 확보(가장 먼저)

1. `design/two-robot-park-mission` 또는 최신 통합 브랜치 위치 확인
2. `parkbot_core`, `parkbot_interfaces`, `robot_agent`, `mock_gwanje`를 현재 워크스페이스로 가져오기
3. 이전 `park.md`, mission/perception 설계 문서와 구현 계획 가져오기
4. 현재 untracked Isaac 에셋을 백업하고 Git 추적 전략 결정
5. 가져온 코드의 빌드와 기존 테스트를 이 머신에서 다시 실행

### P1 — 2로봇 + ROS 2 Bridge 장면  (상당 부분 완료, 3절 "ROS 2 제어 스택" 참고)

1. ~~주차장 + 차량 1대 + 로봇 2대 장면 구성~~ ✅ `two_robot_carry_demo.py`
2. `robot_front`, `robot_rear` namespace와 articulation 경로 분리 — **articulation 경로는 분리됨,
   ROS 2 네임스페이스 분리는 미완**(현재 드라이버는 로봇 1대 기준 `/cmd_vel`)
3. ~~ROS 2 Bridge 설정~~ ✅ (내부 Humble libs, 도메인 126). clock 설정은 미확인
4. `/cmd_vel` ✅ 검증 / `/odom`·`/scan`·depth 카메라 퍼블리시는 **미구현**
   (`/joint_command` 대신 `/arm_control` 서비스로 팔 제어 ✅)
5. 두 로봇 토픽 충돌 여부 — **미확인**(2로봇 동시 ROS 2 제어가 다음 과제)

### P2 — 실제 시뮬레이터 구동 연결

1. `use_mock_nav=False` 경로를 실제 `/cmd_vel`에 연결
2. 현재 직선 유도 테스트를 제거하고 휠 기반 폐루프 주행 구현
3. 접근·정렬·정지 오차 측정
4. 두 로봇의 회전 없는 평행이동 운반 검증

### P3 이후

1. 천장 인식으로 `/vehicle_pose`, `/slot_status` 생성
2. depth 기반 바퀴 감지와 정밀 도킹 구현
3. A 관제 → B 도킹 → C arm 전개 → 2대 운반 E2E 통합
4. markerless segmentation/OBB와 SLAM/천장 카메라 융합으로 교체

---

## 7. 열린 결정/리스크

- 고객 차량 자세 판정: segmentation과 OBB 중 선택
- 천장 카메라 수·배치 및 다중 카메라 좌표 융합
- SLAM/AMCL/천장 절대 위치 융합 구성
- **차종 일반화 (최우선)** — 2로봇 운반이 **A5_Coupe 한 차종에서만** 검증됐다.
  B5_Pickup(타이어 반경 0.440, 축거 3.594)은 팔이 타이어를 아예 못 들고(0.4 mm),
  B3_Minivan은 운반에 실패한다. arm pivot(X=±0.27)과 roller 간격이 Sedan/Coupe급에
  맞춰져 있어 큰 차량을 못 잡는 것으로 보인다. 10종 전체를 커버하려면 이 기하를 손봐야 한다.
- ~~차량 원통 휠 충돌체~~ — **구동계를 유지하는 것으로 해결**(3절 참고). 구/볼록체 대안은
  모두 기각됐고 `fab_vehicles.usd`를 수정할 필요도 없어졌다.
- 운반 중 **옆칸 차량이 0.38~0.43 m 밀린다**(대상 제외 최대 변위). 실제 서비스에서는
  옆차를 건드리면 안 되므로 원인 규명 필요.
- 리프트 테스트 진입은 여전히 `set_world_poses` 순간이동 유도다(물리 구동 아님). P2에서 교체 대상.
- 두 로봇이 동시에 차량을 들 때 하중 분배와 arm drive torque
- 차량 종류/타이어 지름 변화에 따른 arm pivot·roller 간격 일반화 — 실측 결과 타이어 반경
  0.277~0.440 m(1.6배), 축거 2.344~3.594 m(1.25 m 차) 차이. 현재 검증은 Sedan 1종뿐
- 팀원 에셋의 `parking:target = A5` vs `A5_Coupe` 점유 모순 (팀원과 조율 필요)
- ~~**ArUco 카메라 설계가 미구현**~~ — 팀원 깊이캠 에셋 도착. 전방 깊이캠(로봇좌표 x=0.924,
  높이 0.090, 하향 30°)으로 M4까지 실동작. **단 카메라가 낮아 마커 검출 창이 앞 1.1~1.4 m뿐**
  (실측). 마커 간격 3.4 m는 개루프 드리프트가 작아(3.4 m에 2 cm) 커버되나, 폐루프 전제.
- **인계장 마커가 세로 2대 배치와 불일치** — 환경(`build_parking_environment.py`)은 인계장을
  세로 왼/오 2대로 바꿨으나, `marker_layout.py`의 handoff_bay 마커는 옛 3열×2행 좌표 그대로다.
- **인계장 우회 차선 라우팅 미검증** — 베이 열 사이가 0.70 m라 통과 불가. 바깥 여백
  경로를 제안만 해둔 상태
- 천장 라이다 2대가 온라인 S3 참조 — 오프라인 대비 로컬 에셋화 필요
- HANDOFF가 참조하는 `build_mecha_roller_asset.py`가 현재 워크스페이스에 없다(에셋 자체는 존재)
- 현재 에셋과 미션 코드를 Git에서 어떻게 병합·추적할지

---

## 8. 새 세션 시작 멘트

> 이 프로젝트는 실제 자동발레파킹 서비스다. `/home/rokey/cobot3_ws/HANDOFF.md`를 먼저 읽어라.
> 현재 머신(RTX 5080, Isaac Sim 5.1 = Python 3.11)에서 완료·검증된 것:
> ① GPU PhysX 전환 + 1로봇 뒷바퀴 리프트 테스트(뒷축 0.031323 m, 반복 재생 동일),
> ② 메카넘 롤러 옆이동(strafe 4.37 m) + `/cmd_vel` 역기구학, flatten 자체포함 에셋
>    `hwia_parking_robot_final_caster_mecha_roller.usd`,
> ③ 2로봇이 Sedan(1600 kg) 앞/뒤 축을 잡고 **실제 메카넘 구동으로 전진 1.04 m 운반**,
> ④ 외부 ROS 2(Humble, Python 3.10)에서 `/cmd_vel` 발행 → Isaac 로봇 주행, `/arm_control`
>    서비스로 팔 여닫기까지 검증(도메인 126 + FastDDS 화이트리스트, 방향키 텔레옵 포함).
> ⑤ (07-20) 주차장을 **팀원 제작 에셋으로 교체**하고 통합. 끊긴 절대경로 참조 복구, 리프트 타깃
>    A5→A7 이동, 로봇 에셋 3개 스크립트 모두 메카넘으로 통일, 천장 라이다/쇼룸 잔여물 정리.
> ⑥ (07-20) 차량 휠 충돌체가 `Cylinder`라 PhysX가 면진 볼록체로 근사 → 차량이 영구 진동하고
>    그 진동이 팔을 통해 로봇까지 밀어내던 문제를 규명. 구·볼록체 대안을 모두 실측한 끝에
>    **PhysX Vehicle 구동계를 유지하는 것**이 답이라는 결론(레이캐스트 서스펜션이 지지하므로
>    원통 접촉 자체가 발생하지 않음). `fab_vehicles.usd`는 수정할 필요가 없다.
> ⑦ (07-20) **`two_robot_carry_demo.py` 기본값을 "주차된 차량을 구동계 유지한 채 집어 옮기기"로
>    확정.** 자체 스폰 Sedan 경로는 삭제. 대상 차량의 축 위치는 실제 휠 좌표에서 계산한다.
>    A5_Coupe에서 리프트 0.0849 m, 추종률 1.000, 폴백 불필요로 PASS.
> ⑧ (07-21) **Phase 1 ArUco 측위 파이프라인 M1~M4 + 순수 ROS 2 연동 완성.** (자세히는 ARUCO_PLAN.md)
>    - 바닥 마커 44장(`DICT_5X5_100`), 마커별 월드좌표+yaw를 담은 지도 `marker_map.json`.
>      **이 지도는 순수 ROS 패키지 `src/parkbot_aruco/data/`가 소유**(실제 배포엔 Isaac 없음).
>      생성: `isaacpjt/Isaac_envo/build_marker_textures.py`(레이아웃 소스 `marker_layout.py`).
>    - `src/parkbot_aruco` 신설(colcon OK). `aruco_pose.py`(순수 CV — 시스템 cv2 4.5.4의
>      IPPE_SQUARE 버그 회피 위해 ITERATIVE 폴백), `marker_localizer.py`(좌표변환+오도메트리 융합),
>      `marker_localizer_node.py`(image+camera_info → 검출+측위 → **마커 ID·월드좌표 로그**),
>      `aruco_detector.py`(검출 전용). 합성 자세복원 8/8, 실제 Isaac 렌더 단일측위 **0.6 cm**.
>    - `aruco_sim_bringup.py`(Isaac, M4): 전방 깊이캠 → OmniGraph `ROS2CameraHelper`+
>      `ROS2CameraInfoHelper`로 `/image_raw`+`/camera_info` 발행 + `/cmd_vel` 트리거로 A2→A7 주행.
>    - **2-터미널 실동작 검증**: [A] `python3 aruco_sim_bringup.py` → [B]
>      `ros2 run parkbot_aruco marker_localizer_node` → `/cmd_vel` 1회 pub → 순수 ROS 노드가
>      **A2~A7 6/6 마커 ID·월드좌표 로그**. 도메인 **양쪽 일치 필수**(126 사용) + FastDDS 화이트리스트
>      + 브리지 `LD_LIBRARY_PATH`. camera_info 는 CameraHelper 의 type 이 아니라 별도 노드다.
>    - 로봇 에셋은 깊이캠+메카넘 합본 `hwia_depth_cam_mecha_roller.usd`(캐스터 충돌 제거로 발사 해결).
>    - 발견: 메카넘 휠 오도메트리는 롤러 슬립으로 신뢰 불가(17 m에 9 m 드리프트) → **마커 절대측위 필수**.
> ⑨ (07-21) **주차장 인계장(대기장소) 차량을 세로 방향으로 변경.** 2로봇이 회전이 어려우므로
>    슬롯과 같은 세로(yaw 0)로 세워 회전 없이 옆으로(strafe) 실어 나르게 한다. 현재 요청대로
>    **왼/오 2대만** 배치(가운데 비움). `parking/build_parking_environment.py` 수정 + 검증 로직을
>    대기 2대로 완화. **단, 아래 GPU 이슈로 재빌드 미완** — 디스크의 `parking_environment.usd`는
>    아직 6대(세로) 버전이다. 재부팅 후 `--headless`로 재빌드해야 2대판이 나온다.
> 이전 HANDOFF에 있던 `parkbot_core`와 설계 문서는 여전히 이 워크스페이스에 없으므로 확보가 필요하다.
>
> **지금 막혀 있는 것(07-21, 즉시) — GPU 렌더러 컨텍스트 꼬임.** Isaac 헤드리스가 "app ready"
> 직후 렌더러 초기화에서 멈춘다(Isaac `app.close()`/kit-lock 계열 hang). GUI를 다른 Isaac 앱과
> 겹쳐 띄운 것이 원인. 프로세스 종료·`~/.cache/ov` 락 정리·재시도로 안 풀렸고 **재부팅이 해결책**이다.
> 재부팅 후 첫 할 일: `cd isaacpjt/Isaac_envo/parking && python3 build_parking_environment.py --headless`
> 로 2대(세로) 인계장 환경을 재빌드(코드는 이미 저장됨). 그다음 `render_topdown_parking.py`로 확인.
>
> **막혀 있는 것 — 차종 일반화.** 2로봇 운반은 **A5_Coupe 한 차종에서만** 통과한다.
> B5_Pickup은 팔이 타이어를 아예 못 들고(0.4 mm), B3_Minivan은 운반에 실패한다.
> arm pivot(X=±0.27)과 roller 간격이 Sedan/Coupe급 기하에 맞춰져 있는 것이 원인으로 보인다.
> 7절 첫 항목이 최우선 과제다.
>
> 그다음 할 일(ArUco 트랙): **M5 정확도 관문**(여러 위치 스윕 → 오차 공분산, ARUCO_PLAN 참고)과
> **M6 폐루프 주행**(지금 A2→A7 주행은 Isaac 안 GT 기반 스크립트 — 노드가 낸 측위로 `/cmd_vel`을
> 돌리는 폐루프로 교체). 인계장 세로배치 마커(`marker_layout.py` handoff_bay)는 아직 옛 좌표라
> 환경 2대 세로배치에 맞춰 갱신 필요(이번엔 미변경).
> 운반 트랙: 로봇 2대를 ROS 2 네임스페이스(`robot_front`/`robot_rear`)로 분리해 동시 제어,
> 그다음 천장 인식과 정밀 도킹. 짝 이동은 **가상 강체 링크 제어**(차가 두 로봇을 강체로 묶으므로
> 운반프레임 1개 3-DOF로 제어, 각 로봇 명령은 강체 운동학으로 분배 → 서로 안 싸움) 설계를 검토했다.
> 주의: Isaac은 종료가 걸려 **좀비 프로세스**가 남을 수 있다. 실행 전후로 프로세스를 반드시 확인·정리할 것.
> 주의: USD에 **절대경로를 굽지 말 것**. 팀 간 에셋 교환에서 전부 끊긴다(07-20 사례).

---

## 부록 — M5 정확도 관문과 p95 (2026-07-24)

ArUco 마커 측위 정확도를 **p95(95백분위수)** 로 판정한다. p95 는 오차를 정렬했을 때 "측정의 95%가
이 값보다 작거나 같은" 지점이다. median(절반이 이보다 정확) 보다 엄격하고 max(최악 1개) 보다
덜 가혹해, **가끔 크게 틀리는 꼬리를 잡으면서 이상치엔 안 휘둘린다.** 측위는 5% 확률로 5cm 빗나가면
도킹을 놓칠 수 있어, "평균 정확"이 아니라 "거의 항상 정확"을 보장하려고 p95 를 관문으로 쓴다.

**M5 목표**: 위치 p95 ≤ 2cm, yaw p95 ≤ 1°.

**M5 단일프레임 실측(2026-07-24)**: 위치 median 1.40cm / **p95 4.15cm**, yaw median 0.42° /
**p95 1.68°**. 대부분 자세는 합격선 안인데 먼 거리(1.9m)+비스듬(8°)에서 4~6cm로 튀는 **꼬리**가
p95 를 넘긴다. 오차는 횡방향(월드 Z, σ19mm) 지배 = 평면 마커 비스듬 관측의 기하 한계.

**대응(결정)**: **다중프레임 융합** — 한 마커를 K장 보고 측위 추정을 평균/필터. 랜덤 노이즈는 √K 로
줄어 꼬리를 깎는다. 단 프레임별 랜덤 성분에만 듣고, 고정 편향이면 오도메트리 융합(Phase 3)이 필요.

자세한 원리는 저장소 루트 `p95.md` 참고.

---

## 부록 — 미션 Phase A: 로봇 카메라 4대 + 후방 카메라 실동작 검증 (2026-07-24)

로봇 에셋이 카메라 정확히 4대(front/rear/side_left/side_right) 구성으로 확정됐다
(`hwia_4cam_mecha_roller_lowered.usd`). `cam_rear`는 새로 저작한 게 아니라 에셋에 이미 있던
것을 검증만 했다 — 카메라 **월드 포즈** 기준 `cam_rear = Rz(180°)·T_front`가 오차 0으로 성립
(front pos=(+0.924,0,+0.090) fwd=(+0.866,0,−0.500) ↔ rear pos=(−0.924,0,+0.090)
fwd=(−0.866,0,−0.500)). 카메라가 아니라 빈 링크였던 `cam_qr_down`과 OmniverseKit 에디터
카메라 4개(루트 레이어에 실제로 저장돼 있었음)를 제거해 "정확히 4개" 게이트를 통과시켰다.
러너(`parking_v4_runner.py`)는 이 에셋으로 전환했고, `find_rear_camera(stage, robot_id)` +
`V4_CAMERAS_COUNT` 스모크 토큰(`robot=entry_lead n=4 qr_down=False`)으로 매 스테이지 빌드마다
카메라 구성을 자동 확인한다.

**후방 카메라 실동작(`--probe=REAR`)**: 로봇을 도크 마커 뒤(−X, 후방 카메라 시야)에 세우고,
기존 Probe A 외부 검출 파이프라인(`run_probe_a_detector.sh`, 시스템 ROS 2 Humble + cv2)을
**코드 변경 없이 그대로** 후방 영상(같은 `/robot_entry_lead/image_raw` 토픽)에 돌렸다. 마커
21(D_OUT_1)을 0.6~1.7m 12지점 스윕한 결과 **검출 창 d_min=1.10m ~ d_max=1.70m (7/12 지점,
reproj 대부분 <1.2px)**, 근거리 0.6~1.0m는 미검출 — 전방 카메라가 겪던 근거리 사각(§7의
"1.1~1.4m뿐" M4 수치, 높이 0.09m+하향 30°의 기하 한계)과 **같은 한계까지 대칭**인 패턴이다. 도메인 126 + FastDDS
화이트리스트로 2-터미널 통신 정상, 좀비 프로세스 없이 종료(exit 0). **후방 카메라가 전방과
대칭으로 도크 마커를 검출함을 실측으로 확인 — Phase A 완료.**

---

## 부록 — 미션 Phase B: 도크→XN 융합주행(GT-free) 2로봇 완주 (2026-07-25)

입차 로봇 2대(entry_lead, entry_follow)가 각자 도크에서 나와 인계장 마커 **XN**(id=31) 앞까지
실제 메카넘 바퀴를 굴려 정렬하는 `--mission=B`가 완성됐다. 위치 추정은 **GT를 전혀 쓰지 않고**
ArUco 마커+휠 오도메트리 융합(`PoseFilter.predict_body`+`update`)만으로 이뤄진다 — GT는 카메라
일회성 보정(`calibrate_tbasecam`)과 리포팅(`err_pos_gt`/`err_yaw_gt`)에만 쓰이고 제어 루프
안에는 들어가지 않는다. 두 로봇은 **스태거**(entry_lead가 먼저 완주하고 entry_follow가 이어서
시작)로 움직이며 서로 **충돌 없이**(최종 분리거리 실측 ~1.6m) 완주한다.

**핵심 구성요소**: 순수 제어법 `body_twist_toward(filt.pose(), target)`(`mission_control.py`,
단위테스트+뮤테이션테스트로 검증) → `drive_to_pose`/`rotate_in_place`(`parking_v4_runner.py`)가
이를 예측(휠오도 `predict_body`)·보정(마커 `update`)과 묶어 폐루프 주행으로 감싼다. 회전 오도
보정 상수 `YAW_ODOM_SCALE=1.12`(90°/180° 폐루프 스윕으로 공동 확정), 도크체크·XN정렬 보정은
**위치전용**(`correct_yaw=False` — 마커는 위치만 보정, yaw는 보정된 오도메트리를 신뢰), 그리고
entry_lead도 entry_follow와 동일하게 **전방캠으로 XN을 정면에서** 본다(원래의 후방캠+제자리
180° 재정렬 설계는 폐기 — 아래 DEBUG_LOG 참고).

**실측 결과(2회 반복, 결정적)**: `MISSIONB_RESULT ok=True` 2/2. 최종 XN정렬 오차(GT 대비)
entry_follow **~0.2~0.7cm**, entry_lead(+x 충돌회피 오프셋 포함) **~4.8~6cm**.

**정직한 한계**: (1) 위치전용 보정의 대가로 최종 헤딩 정확도는 마커가 아니라 오도메트리 한계에
묶여 있다(`YAW_ODOM_SCALE` 검증 범위 기준 ~4°). (2) entry_lead의 +x 오프셋 구간은 후반부에 XN이
카메라 시야를 벗어나 순수 오도로 주행하므로 entry_follow보다 종단오차가 크다. (3) 충돌
클리어런스는 로봇 풋프린트를 반경 0.68m 원으로 모델링한 명목 계산이다(실측 미검증) — 오차범위
까지 고려한 GT 최악 분리거리는 **~1.57m**(요구치 ≥1.5m 충족, 클리어런스 ~0.21m)로 여유가 크지
않다.

**Phase C로 넘기는 것**: 차량 배치·차밑 진입·리프트, 2로봇 최종 배치(오프셋 방향·크기)의 실차
재조정, 로봇 풋프린트 반경(0.68m) 실측 검증, 근거리 마커 사각(<1.1m, 검출창 ~1.1~1.7m) 대응.

자세한 설계·교훈은 `docs/concepts/mission-phase-b-fused-drive.md`, 디버깅 과정은 아래
DEBUG_LOG.md "미션 Phase B" 절 참고.
