# 뎁스캠 기반 정지 판단 + 뒷바퀴 리프트 — 설계

> 작성: 2026-07-21. 관련 문서: `HANDOFF.md`(P2 "teleport 유도 제거"), `ARUCO_PLAN.md`(별개 트랙 —
> 이 스크립트는 ArUco 측위 M-시리즈와 무관하다. 뎁스 임계값만으로 정지를 판단하는
> 독립적인 물리/도킹 검증이다).

## 배경

`parking_robot_rear_lift_test.py`는 로봇을 Sedan 뒷축 아래로 **순간이동(teleport,
`set_world_poses`)** 시켜 진입한 뒤 arm을 전개해 리프트를 검증한다. `HANDOFF.md` P2는
이 teleport 유도를 실제 구동 기반 폐루프로 교체하는 것을 다음 과제로 남겨 두었다.

이번 작업은 팀원이 만든 `hwia_depth_cam_mecha_roller.usd`(메카넘 롤러 + 뎁스캠 4대가
이미 결합된 로봇 에셋)를 사용해, 그 교체의 첫 단계 — **전방 뎁스캠으로 차량 하부 진입을
감지해 정지하고 팔을 전개하는 것** — 를 구현한다.

## 범위

- 새 파일: `isaacpjt/Isaac_envo/depth_stop_lift_test.py`
- 원본 에셋 비파괴(기존 관례 유지) — 새 test stage에서만 override.
- ArUco/마커 측위, 좌우 뎁스캠 기반 횡방향 정렬, 2로봇 운반은 범위 밖(추후 별도 작업).

## 아키텍처

`parking_robot_rear_lift_test.py`와 동일한 뼈대(Isaac python 재실행 → 테스트 stage
생성·저장 → SimulationApp 실행 → 리포트 JSON)를 따르되 두 지점을 바꾼다:

1. **로봇 에셋**: `hwia_depth_cam_mecha_roller.usd` 참조. prim 계층이 기존 `_mecha_roller.usd`
   보다 한 단계 얕다 — `verify_depth_cam_mecha.py`의 상수를 그대로 쓴다:
   `ROBOT_ROOT=/World/Robot/base_link`, `ROBOT_JOINTS=/World/Robot/joints`,
   `CAM_FRONT=/World/Robot/cam_front_link/depth_cam_front/Camera_Pseudo_Depth_Front`.
2. **진입 방식**: `robot.set_world_poses()` 순간이동 대신 `mecanum_drive.py`의
   `configure_hub_drives` + `wheel_velocities_from_cmd_vel(vx=0.4, 0, 0)`로 실제 바퀴를
   굴려 전진시킨다(`verify_depth_cam_mecha.py`에서 이미 검증된 SPEED=0.4 재사용).

씬 구성(주차장 A7 + Sedan + 차량 안정화 플래그 `--sphere-wheels`/`--keep-drivetrain`)은
`parking_robot_rear_lift_test.py`를 그대로 재사용한다 — 이미 검증된 물리 조건을 바꾸지 않는다.

로봇은 기존과 같은 시작 위치(Sedan 뒤쪽, `ROBOT_START_Z=4.55`)에서 출발해 뒤에서부터
들어가므로, 뎁스 트리거로 멈추는 지점은 자연히 **뒷축**이 된다. 팔 전개 목표값은 기존
`ARM_TARGETS`(뒷바퀴 리프트 각도)를 그대로 재사용한다.

## 뎁스 정지 판단

- 센서: `isaacsim.sensors.camera.Camera`를 `CAM_FRONT`에 붙이고
  `add_distance_to_image_plane_to_frame()`으로 실제 뎁스 프레임을 받는다. (기존
  스크립트들은 이 카메라로 `get_rgba()`만 썼다 — 뎁스 프레임을 실제로 읽는 것은 이번이
  처음이라 첫 실행에서 센서가 유효한 값을 내는지부터 확인한다.)
- ROI: 이미지 중앙 하단부(가로 중앙 40%, 세로 하단 50%) 패치의 **최소 뎁스**.
- **기준값은 하드코딩하지 않는다.** 진입 시작 직후 N프레임(차량이 아직 시야에 잡히기 전
  구간) 동안 ROI 최소뎁스를 샘플링해 `baseline`을 구한다.
- 정지 조건: `roi_min_depth < baseline - DROP_MARGIN`이 **연속 3프레임** 성립.
  `DROP_MARGIN` 기본 0.05 m, CLI로 조절 가능.
- 안전장치: 최대 주행거리/스텝을 넘도록 트리거가 안 되면 `stop_reason="timeout"`으로
  실패 기록 후 종료(무한루프 방지).
- 매 프레임 `{step, x, z, roi_min_depth}` 궤적 전부를 리포트 JSON에 남긴다 — 트리거가
  안 되거나 오정지할 때 임계값을 튜닝할 근거 데이터.

## 정지 후 처리

`parking_robot_rear_lift_test.py`의 `set_arm_targets` 램프(180스텝에 걸쳐 0→±90도)를
그대로 재사용. 판정 지표도 동일하게 재사용한다:

- 뒷바퀴 평균 상승 ≥ 0.025 m
- 뒤 상승 > 앞 상승 + 0.012 m
- arrival 오차(정지 위치 vs 뒷축 실제 위치) < 0.15 m

리포트에 아래 필드를 추가한다: `depth_baseline_m`, `depth_stop_value_m`, `stop_step`,
`stop_reason`(`"depth_trigger"` | `"timeout"`), `depth_trace`(프레임별 궤적).

## CLI

기존 스크립트들과 통일: `--gui`, `--sphere-wheels`(기본 권장), `--keep-drivetrain`,
`--drop-margin`(기본 0.05).

## 테스트 계획

- 헤드리스 1회 실행 → `DEPTH_STOP_OK=True/False`, `LIFT_OK=True/False` 콘솔 출력 + JSON 리포트.
- `--gui`로 실제 정지 지점과 팔 전개를 육안 확인.
- 뎁스 트리거가 한 번도 안 걸리면(`stop_reason=timeout`) `depth_trace`를 근거로
  ROI/DROP_MARGIN을 조정 — 이번 구현에서 정확한 수치를 못 맞추더라도, 다음 시도에서
  튜닝할 수 있는 데이터를 반드시 남기는 것이 이번 작업의 최소 성공 기준이다.

## 리스크

- `Camera_Pseudo_Depth_Front`가 실제로 유효한 `distance_to_image_plane` 값을 내는지
  미검증(팀원 에셋 첫 실사용). 안 나오면 센서 문제부터 디버깅해야 한다.
- 카메라가 30° 아래를 보므로 뎁스 변화 신호가 기대한 위치(뒷축 부근)에서 정확히
  발생하는지도 미검증 — ROI/마진은 실측 후 조정될 수 있다.
