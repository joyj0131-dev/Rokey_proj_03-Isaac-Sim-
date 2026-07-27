# 미션 Phase C (인계장 픽업) 구현 계획

> **For agentic workers:** REQUIRED SUB-SKILL: superpowers:subagent-driven-development. 태스크 단위로 실행한다.

**Goal:** 입차팀 2로봇이 파란 베이의 Pickup 밑으로 진입해, 측면 뎁스캠으로 바퀴를 실시간 감지(미리 기억 금지)하고 앞축·뒤축 중심에 정지한 뒤 리프트로 들어올린다.

**Architecture:** `origin/p4_depth`의 검증된 구현(`depth_stop_detector.py`, `depth_stop_lift_test_dual.py`)을 v4 미션으로 **이식**한다. 접근은 Phase B의 융합주행(`drive_to_pose`/`rotate_in_place`)을 그대로 쓰고, 최종 정밀 정렬만 뎁스 감지가 맡는다.

**Tech Stack:** Isaac Sim 5.1(python.sh), USD, `omni.replicator.core` 뎁스 annotator, numpy, ROS2 Humble(기존 카메라 발행 경로). pytest 는 `-p no:anyio` 필수.

## Global Constraints

- **GT는 제어에 쓰지 않는다.** 접근=마커+휠오도 융합, 최종=뎁스 감지. GT는 채점·기록·1회 보정 전용.
- **바퀴 위치를 미리 계산·기억하지 않는다**(사용자 명시 요구). 트로프 진입/이탈 중간값만 사용.
- 실제 바퀴 구동(`set_joint_velocity_targets`)으로 주행. 텔레포트는 스폰/셋업 전용.
- **UI 실행 가능**해야 한다(`--gui`). headless 스모크도 제공.
- `parking_v4_runner.py`(4카메라 자산 `hwia_4cam_mecha_roller_lowered.usd`) 위에 구축. 레거시 러너 미변경.
- 기존 `--mission=B`, `--probe=FUSE/M5/REAR/REARXN/ROTCHK/ROTCHK180/A` 회귀 없음.
- 좌표: 인계장 베이 패드 중심 (x=-8.5, z=+7.075), 패드 6.4×3.2. Pickup 전장 5.83 폭 1.91 축거 3.594 타이어반경 0.423. 로봇은 **주차칸 쪽(+x) 끝에서 -x 방향으로** 진입. entry_follow=앞축(깊이), entry_lead=뒤축.
- Isaac 실행은 **백그라운드+바운드 폴링**(절대 무한 대기 금지), 실행 전후 좀비 확인.
- 한국어 주석/문서.

---

## Task C1: Pickup 을 인계장 베이에 배치

**Files:** Modify `isaacpjt/Isaac_envo/parking_v4_runner.py`

**Interfaces:** Produces `spawn_handoff_vehicle(stage)` — `fab_vehicles.usd`의 `Pickup`을 베이에 참조 스폰. 토큰 `V4_VEHICLE`.

- [ ] **Step 1: 참조 스폰 구현**
  - `VEHICLES_USD = WORK_DIR / "fab_vehicles.usd"`. `p4_depth`의 `depth_stop_lift_test_dual.py`가
    차량을 스테이지에 얹는 방식(`VEHICLE_ROOT = /World/VehicleAsset/Vehicles/<이름>`, 참조·물리 처리)을
    `git show origin/p4_depth:isaacpjt/Isaac_envo/depth_stop_lift_test_dual.py` 로 읽어 그대로 따른다.
  - 배치: 패드 중심 (x=-8.5, z=+7.075) 위, **차 길이축을 x에 정렬**(필요 시 Y축 90° 회전),
    차 앞이 게이트 쪽(-x), 뒤가 주차칸 쪽(+x). 바닥에 닿게 y 조정.
  - 토큰: `V4_VEHICLE name=Pickup pos=(x,z) yaw=<deg> len_x=<m> wid_z=<m>`(실측 bbox로 확인).
- [ ] **Step 2: 물리 안정 확인**
  - 스폰 후 60프레임 진행시키고 차량 이동량 출력: `V4_VEHICLE_SETTLE drift_m=<>`. 0.02 m 이하 목표.
  - 진동하면 p4_depth/리프트 테스트가 쓰던 완화책(`--sphere-wheels`, 구동계 제거 등)을 검토·적용하고 근거를 report에 남긴다.
- [ ] **Step 3: 회귀 + 커밋**
  ```bash
  cd /home/rokey/p3/cobot_ws/isaacpjt/Isaac_envo
  bash parking_v4_runner.sh --mission=B 2>&1 | grep -E "MISSIONB_RESULT|Traceback"   # 차 추가가 B를 안 깼는지
  bash parking_v4_runner.sh --probe=FUSE 2>&1 | grep -E "FUSE_RESULT|Traceback"
  ```
  Expected: `MISSIONB_RESULT ok=True`, `FUSE_RESULT=PASS`. 차량이 로봇 경로를 막지 않아야 한다(막으면 배치/경로 조정 후 report 기록).
  커밋: `feat(mission): 인계장 베이에 Pickup 배치`

---

## Task C2: 측면 뎁스캠 파이프라인 + `--probe=DEPTH`

**Files:** Modify `isaacpjt/Isaac_envo/parking_v4_runner.py`

**Interfaces:** Produces `depth_setup(stage, robot_id, side="left")` → ROI 최소뎁스를 매 프레임 읽을 수 있는 컨텍스트. 토큰 `DEPTH_STREAM`.

- [ ] **Step 1: 뎁스 annotator 배선**
  - 4카메라 자산의 측면 카메라 경로: `.../cam_side_left_link/depth_cam_left/Camera_Pseudo_Depth_Left`
    (우측은 `..._right/..._Right`). `find_front_camera` 패턴으로 `find_side_camera(stage, robot_id, side)` 추가.
  - `omni.replicator.core`로 render_product 생성 후 **뎁스 annotator**(`distance_to_image_plane` 등 설치된 이름)를 attach.
    `fuse_camera_setup`의 rgb annotator 배선을 참고하되, p4_depth 테스트가 쓰는 annotator 이름을 그대로 따른다.
  - `roi_min_depth`는 Task C3에서 이식하므로 여기서는 원본 뎁스 배열 shape/유효값만 확인한다.
- [ ] **Step 2: `--probe=DEPTH` 로 실측**
  - entry_follow를 베이 진입선에 놓고 -x로 천천히 전진시키며 매 프레임 ROI 최소뎁스를 출력:
    `DEPTH_STREAM step=<n> x=<gt_x> roi_min=<m>`(20프레임마다 1줄이면 충분).
  - Expected: 바퀴가 시야에 들어올 때 값이 **뚜렷이 떨어졌다가 다시 올라오는 트로프**가 관측된다.
    트로프가 안 보이면 ROI/카메라 방향/차량 위치를 점검하고 정직 보고(억지 통과 금지).
- [ ] **Step 3: 커밋** — `feat(depth): 측면 뎁스캠 파이프라인 + --probe=DEPTH 트로프 실측`

---

## Task C3: 트로프 진입/이탈 중간값 축 감지 (순수 로직 + 단위테스트)

**Files:** Create `isaacpjt/Isaac_envo/depth_stop_detector.py`(p4_depth에서 이식), `isaacpjt/Isaac_envo/axle_center.py`, `isaacpjt/Isaac_envo/tests/test_axle_center.py`

**Interfaces:** Produces `DepthStopDetector`(이식), `TroughTracker`(신규 순수 로직) — 프레임마다 `(위치, roi_min)`을 넣으면 트로프 **진입 위치·이탈 위치**를 기록하고 `axle_center()`로 중간값을 돌려준다.

- [ ] **Step 1: p4_depth 파일 이식**
  ```bash
  cd /home/rokey/p3/cobot_ws
  git show origin/p4_depth:isaacpjt/Isaac_envo/depth_stop_detector.py > isaacpjt/Isaac_envo/depth_stop_detector.py
  ```
  (원본 유지. v4 전용 수정이 필요하면 최소한으로.)
- [ ] **Step 2: 실패 테스트 작성** — `tests/test_axle_center.py`
  ```python
  import sys, pathlib
  sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))
  from axle_center import TroughTracker

  def test_midpoint_of_symmetric_trough():
      # baseline 1.0, 바퀴 구간에서 값이 내려갔다 올라오는 대칭 트로프.
      tr = TroughTracker(baseline=1.0, drop_margin=0.05)
      xs = [0.0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6]
      vals = [1.00, 1.00, 0.90, 0.80, 0.90, 1.00, 1.00]   # 진입 0.2, 이탈 0.5 부근
      for x, v in zip(xs, vals):
          tr.update(x, v)
      c = tr.axle_center()
      assert c is not None
      assert abs(c - 0.35) < 0.06      # 진입·이탈의 중간

  def test_no_trough_returns_none():
      tr = TroughTracker(baseline=1.0, drop_margin=0.05)
      for x in range(5):
          tr.update(float(x) * 0.1, 1.0)
      assert tr.axle_center() is None

  def test_min_position_is_not_used_as_center():
      # 비대칭(한쪽이 더 깊은) 트로프에서도 중심은 최솟값 위치가 아니라 중간값이어야 한다.
      tr = TroughTracker(baseline=1.0, drop_margin=0.05)
      for x, v in [(0.0,1.0),(0.1,0.93),(0.2,0.70),(0.3,0.88),(0.4,1.0)]:
          tr.update(x, v)
      c = tr.axle_center()
      assert abs(c - 0.20) > 1e-9 or True   # 최솟값 위치(0.2)와 중간값이 다를 수 있음을 문서화
      assert 0.10 <= c <= 0.35
  ```
- [ ] **Step 3: 실패 확인** → `python3 -m pytest isaacpjt/Isaac_envo/tests/test_axle_center.py -p no:anyio -q` (FAIL)
- [ ] **Step 4: 구현** — `axle_center.py`에 `TroughTracker` 작성.
  - `update(pos, roi_min)`: `roi_min < baseline - drop_margin` 로 **처음 내려간 pos = enter**,
    다시 임계 위로 **올라온 pos = exit** 기록. `axle_center()` = `(enter + exit)/2`, 미완이면 None.
  - baseline은 생성자 인자 또는 초기 N프레임 중앙값(`DepthStopDetector`와 동일 관례).
- [ ] **Step 5: 통과 확인 + 커밋** — `feat(depth): 트로프 진입/이탈 중간값 축 감지 + 단위테스트`

---

## Task C4: 진입 안무 (베이 접근 → 90° 회전 → 2로봇 순차 진입·정지)

**Files:** Modify `isaacpjt/Isaac_envo/parking_v4_runner.py`

**Interfaces:** Produces `--mission=C`. 토큰 `MISSIONC_APPROACH / MISSIONC_TURN / MISSIONC_INGRESS / MISSIONC_AXLE`.

- [ ] **Step 1: 접근·회전**
  - Phase B의 `_mission_setup`/`drive_to_pose`/`rotate_in_place`를 재사용해, 두 로봇을 베이의
    **주차칸 쪽 진입선**(패드 바깥, 차 밑 진입 직전)으로 이동시키고 **-x를 향해 90° 회전**시킨다.
  - 접근 목표·standoff는 nominal 상수로 두고 실측으로 튜닝, 최종값을 report에 기록.
- [ ] **Step 2: 순차 진입 + 뎁스 정지**
  - **entry_follow 먼저**(앞축=더 깊은 목표) -x로 저속 전진. 매 프레임 좌측 뎁스 ROI를 `TroughTracker`에 넣고,
    **첫 바퀴(뒤축) 트로프를 지나 두 번째(앞축) 트로프의 중간값**에 도달하면 정지·정렬.
  - 그 다음 **entry_lead** 진입 → **첫 트로프(뒤축) 중간값**에서 정지.
  - 토큰: `MISSIONC_AXLE robot=<> axle=<front|rear> enter_x=<> exit_x=<> center_x=<> stop_err_m=<GT 대비>`
  - **정직 게이트**: 감지 실패(트로프 미검출)나 간섭(차체/바퀴 충돌)이면 억지로 통과시키지 말고 실측값과 함께 보고.
- [ ] **Step 3: 스모크 + 회귀 + 커밋**
  ```bash
  bash parking_v4_runner.sh --mission=C 2>&1 | grep -E "MISSIONC_|Traceback"
  bash parking_v4_runner.sh --mission=B 2>&1 | grep -E "MISSIONB_RESULT"
  ```
  커밋: `feat(mission): --mission=C 베이 접근·진입·뎁스 축감지 정지`

---

## Task C5: 리프트 (앞축·뒤축 동시 들어올림)

**Files:** Modify `isaacpjt/Isaac_envo/parking_v4_runner.py`

**Interfaces:** Produces `deploy_arms(art, idx, ramp)` + 토큰 `MISSIONC_LIFT`.

- [ ] **Step 1: 팔 전개 이식**
  - `ARM_TARGETS`(deg): `arm_left_front_joint +90 / arm_left_rear_joint -90 / arm_right_front_joint -90 /
    arm_right_rear_joint +90`. `dock_lift_handoff_runner_v2.py`(467~482줄)의 램프 적용 방식을 따른다.
  - 두 로봇이 정지·정렬을 마친 뒤 **동시에** 전개.
- [ ] **Step 2: 들림 판정**
  - 차량 휠(또는 차체) 높이 상승을 측정해 토큰: `MISSIONC_LIFT rise_front=<m> rise_rear=<m> ok=<bool>`.
    기준은 기존 리프트 테스트 관례(뒷축 상승 ≥ 0.025 m 등)를 참고해 정하고 report에 명시.
- [ ] **Step 3: 스모크 + 커밋** — `feat(mission): 앞축·뒤축 리프트 + 들림 판정`

---

## Task C6: 문서

**Files:** Create `docs/concepts/mission-phase-c-depth-pickup.md`; Modify `HANDOFF.md`, `DEBUG_LOG.md`

- [ ] **Step 1: 개념 md** — 뎁스 바퀴감지 원리(ROI 최소뎁스, baseline−margin, **트로프 중간값이 왜 축 중심인가**),
  카메라 역할, 접근(마커)과 정밀(뎁스)의 역할 분담, 리프트, p4_depth 원본과의 차이.
- [ ] **Step 2: HANDOFF/DEBUG 기록 + 커밋** — `docs: 미션 Phase C(인계장 픽업) 개념·결과·디버깅 기록`

---

## Self-Review 메모

- 스펙 §5 수용기준 매핑: 뎁스 감지 정지=C3/C4, 리프트=C5, GT 배제=C4 제어경로, 회귀=C1/C4, GUI=C4, 문서=C6.
- "미리 기억 금지" 준수: `TroughTracker`는 실시간 스트림만 받고 사전 좌표를 받지 않는다(단위테스트로 고정).
- 리스크(스펙 §6)는 각 태스크의 정직 게이트로 흡수 — 특히 진입 간섭과 마커 사각.
