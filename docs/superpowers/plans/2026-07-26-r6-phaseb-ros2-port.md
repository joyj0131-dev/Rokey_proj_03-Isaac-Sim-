# R6 — Phase B(도크→XN) ROS2 이식 + 러너 미션코드 제거 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 인프로세스 `--mission=B/C`에 갇힌 Phase B(도크→XN 융합주행)를 ROS2 노드/오케스트레이터로 이식해 도크→XN→베이→픽업→리프트 전 구간을 ROS2 경로로 완주시키고, 러너에서 미션 코드를 제거한다.

**Architecture:** `marker_localizer_node`에 (a)ref-마커 하드필터+런타임 전환, (b)위치전용 보정 모드를 추가하고, `--bridge`가 후방캠을 발행하게 한다. 오케스트레이터(`pickup_orchestrator_node`)는 기존 픽업 회랑 안무 **앞에** Phase B 레그(도크 시드→90°회전→후방캠 도크점검→XN 정렬→오프셋)를 스태거링으로 붙인다. 마지막에 러너의 `_run_mission_b_choreo`/`_run_mission_c_choreo`를 제거하고 프로브·브리지만 남긴다.

**Tech Stack:** ROS2 Humble(py3.10), rclpy, nav2_msgs/NavigateToPose·ExecuteParkingTask, Isaac Sim 5.1(py3.11) 브리지, pytest(`-p no:anyio`).

## Global Constraints

- **GT 금지**: 제어는 융합측위(마커+휠오도)+뎁스 감지만. GT는 채점·기록 전용.
- **인프로세스 동등**: ROS2 경로 종단 정확도가 기존 실적 수준(entry_lead ~1.1cm, entry_follow ~0.7cm, truck_rise 0.083m) 이내. 축 오차 상한 5cm.
- **회귀 없음**: 기존 프로브(FUSE/M5/REAR/REARXN/ROTCHK*/YAWSTEP 등), `--bridge`, 139 passing 테스트 유지. GUI 실행 가능.
- **좌표 규약**: yaw=0→월드 +Z(`atan2(fwd_x, fwd_z)`). 목표는 USD 월드 프레임 그대로(map 변환 미배선).
- **위치전용 보정**: 도크·XN 마커는 위치에 강하고 yaw에 약하다 → `correct_yaw=False`(위치만 블렌드, filt yaw는 오도 신뢰). 인프로세스와 동일 근거.
- **회전은 오도로**: 제자리 90° 회전은 융합만으론 불가(단일 전방캠이 회전 중 마커 상실 — R3c §7 실측). 오도 인스턴스(`navigate_to_pose`)로 회전한다.
- **스태거**: entry_lead 먼저 완주 후 entry_follow(회랑/XN 근접 충돌 회피, 분리 ≥1.5m). 기존 `CorridorStaggerGuard` 관례 유지.
- **`ros2` CLI 세그폴트**: 검증은 CLI 대신 rclpy 스크립트/노드 로그로. 테스트는 `PYTHONPATH=install/parking_robot_interfaces/local/lib/python3.10/dist-packages` 필요.
- **되돌릴 수 있게**: 각 태스크 별도 커밋. 러너 미션코드 제거(T6)는 T5 E2E 성공 **이후에만**.

## 진실의 원천 (재발명 금지 — reuse, don't rebuild)

이식 대상 인프로세스 구현은 전부 `isaacpjt/Isaac_envo/parking_v4_runner.py`에 있다. 값·순서는 여기서 **그대로** 가져온다:

- `_run_mission_b_choreo` (2595), `_mission_setup(target, sibling_id, need_front, need_rear, cam_h)` (2715): Phase B 안무 전체.
- `fuse_camera_setup(..., cam_role)` (764): `ctx["ref_id"]` 로 단일마커 하드필터. 도크/XN 전환은 호출부가 `ctx["ref_id"]=xn_id` 로 바꿔치기.
- `drive_to_pose(ctx, art, idx, filt, T, target, correct_yaw=, pos_tol=)` (1157), `rotate_in_place` (1344).
- `--probe=REARXN` (2082): 후방캠 도크→XN ref 전환 + T_base_cam 보정의 검증된 레퍼런스.
- 좌표: 도크 entry_lead=D_OUT_1(id21, -3.2,+2.9)·entry_follow=D_OUT_2(id23, -1.2,+2.9), XN(id31, -2.5,+6.875), 로봇 스폰 z=2.2. 픽업 APPROACH=(x=-3.50, z=7.075, yaw=-90).
- Phase B 종단(스텝5): entry_follow=(xn_x, xn_z−standoff, yaw0), entry_lead=(xn_x+final_x_offset(−1.7), xn_z−standoff, yaw0).

ROS2 쪽 소비자: `marker_localizer_node`(측위 융합), `pose_controller_node`(NavigateToPose, odom/fused 두 인스턴스), `pickup_orchestrator_node`(ExecuteParkingTask), `pickup_choreography`(순수 시퀀싱).

---

### Task 1: marker_localizer_node — ref-마커 하드필터 + 위치전용 보정

**Files:**
- Modify: `src/parkbot_aruco/parkbot_aruco/marker_localizer_node.py`
- Modify: `src/parkbot_aruco/parkbot_aruco/marker_localizer.py` (PoseFilter에 위치전용 보정 메서드가 없으면 추가)
- Test: `src/parkbot_aruco/test/test_marker_localizer_filter.py` (신규)

**Interfaces:**
- Produces: 파라미터 `ref_ids`(int 배열, 기본 `[]`=필터 없음/전체=하위호환), `correct_yaw`(bool, 기본 `True`=기존 동작). 런타임 `set_parameters`로 `ref_ids` 변경 시 즉시 반영(파라미터 콜백). `correct_yaw=False`면 마커 fix의 위치만 블렌드(pos_gain), filt yaw는 predict(오도)만 신뢰.
- Consumes: T3 오케스트레이터가 `set_parameters`로 `ref_ids`를 도크→XN 전환.

**구현 노트(진실의 원천):** 러너 `fuse_camera_setup`의 `hit = [p for p in det if int(p.marker_id) == ctx["ref_id"]]`(960,974행)와 동일한 하드필터. 위치전용 블렌드는 러너 `drive_to_pose` `_apply_fix`의 `filt.set_pose(filt.x + pos_gain*(fix.x-filt.x), filt.z + pos_gain*(fix.z-filt.z), filt.yaw)` 블록과 동일. `PoseFilter`에 `pos_gain` 상수가 이미 있으면 재사용.

- [ ] **Step 1: 실패 테스트 — ref_ids 필터**

```python
# test_marker_localizer_filter.py
def test_ref_ids_filters_detections():
    # ref_ids=[31] 이면 id 21/23 검출은 무시하고 31만 측위에 쓴다
    from parkbot_aruco.marker_localizer_node import filter_detections_by_ref
    dets = [_fake(21), _fake(31), _fake(23)]
    kept = filter_detections_by_ref(dets, [31])
    assert [d.marker_id for d in kept] == [31]
    assert filter_detections_by_ref(dets, []) == dets  # 빈=전체(하위호환)
```

- [ ] **Step 2: 실패 확인** — Run: `python -m pytest src/parkbot_aruco/test/test_marker_localizer_filter.py -p no:anyio -v` → FAIL(함수 없음)

- [ ] **Step 3: `filter_detections_by_ref` 순수함수 추출 + 노드 배선**

`_on_image` 루프 진입 전에 `poses = filter_detections_by_ref(poses, self.ref_ids)` 적용. `ref_ids` 파라미터 선언 + `add_on_set_parameters_callback` 로 런타임 갱신.

- [ ] **Step 4: 실패 테스트 — 위치전용 보정**

```python
def test_position_only_correction_keeps_odom_yaw():
    from parkbot_aruco.marker_localizer import PoseFilter
    f = PoseFilter(); f.set_pose(0.0, 0.0, 90.0)
    f.predict(0.0, 0.0, 5.0)          # 오도가 yaw 를 95°로
    from parkbot_aruco.marker_localizer_node import apply_fix
    apply_fix(f, _fix(x=0.1, z=0.1, yaw_deg=45.0), correct_yaw=False)
    assert abs(f.pose()[2] - 95.0) < 1e-6   # yaw 는 마커(45)에 안 끌려감
    assert f.pose()[0] > 0.0                 # 위치는 마커 쪽으로 이동
```

- [ ] **Step 5: 실패 확인** → FAIL

- [ ] **Step 6: `apply_fix(filt, fix, correct_yaw)` 헬퍼 + 노드 배선**

`_on_image`의 `self.filt.update(fix)` 를 `apply_fix(self.filt, fix, self.correct_yaw)` 로 교체. `correct_yaw` 파라미터 선언.

- [ ] **Step 7: 전체 통과 + 회귀** — Run: `python -m pytest src/parkbot_aruco/test/ -p no:anyio -v` → PASS(신규 포함, 기존 불변)

- [ ] **Step 8: Commit** — `feat(aruco): marker_localizer ref-필터+런타임전환+위치전용 보정`

---

### Task 2: `--bridge` 후방캠 발행

**Files:**
- Modify: `isaacpjt/Isaac_envo/parking_v4_runner.py` (`--bridge` 카메라 셋업, 3976행 부근)

**Interfaces:**
- Produces: `--bridge` 가 `--bridge-rear` 플래그(기본 off, RTF 비용 회피) 지정 시 `/robot_<id>/rear/image_raw`(+camera_info) 발행. 기존 `attach_camera_graph(role="rear")`(700행) 재사용.

**구현 노트:** 러너는 이미 `--probe=REAR`(1569행)에서 `attach_camera_graph(target, cam_path, role="rear")` 로 후방 발행을 한다 — 같은 호출을 `--bridge` 경로에 옵트인으로 추가. 전방/뎁스 셋업(3976,4010행) 옆에 동형으로.

- [ ] **Step 1: `--bridge-rear` 파싱 추가** (`--bridge-cameras` 파싱부 3948행 옆)

- [ ] **Step 2: 후방캠 그래프 부착** — `--bridge-rear` 시 `cam_robots_bridge` 각 로봇에 `attach_camera_graph(r, find_rear_camera(stage, r), role="rear")` + `BRIDGE_REAR_CAMERAS` 로그.

- [ ] **Step 3: 스모크(라이브)** — 브리지 `--bridge --bridge-rear --bridge-cameras=entry_lead` 기동 후 rclpy 스크립트로 `/robot_entry_lead/rear/image_raw` 수신 확인(≥1프레임). 결과를 report에 기록.

- [ ] **Step 4: 회귀** — `--bridge-rear` 없이 기존 `--bridge` 가 전방/뎁스만 내는지(변화 없음) 확인.

- [ ] **Step 5: Commit** — `feat(bridge): --bridge-rear 후방캠 ROS2 발행(옵트인)`

---

### Task 3: Phase B 레그 — 순수 시퀀싱 + 오케스트레이터 배선

**Files:**
- Modify: `src/parkbot_motion/parkbot_motion/pickup_choreography.py` (Phase B 스텝 시퀀스 추가)
- Modify: `src/parkbot_motion/parkbot_motion/pickup_orchestrator_node.py` (Phase B 레그 실행 + ref 전환)
- Test: `src/parkbot_motion/test/test_phase_b_choreography.py` (신규)

**Interfaces:**
- Consumes: T1 `ref_ids`/`correct_yaw`(localizer set_parameters), `pose_controller_node` odom/fused 인스턴스(NavigateToPose), 로봇별 도크/XN 좌표(파라미터).
- Produces: `phase_b_plan(leader_id, follower_id)` → 순수 스텝 리스트(로봇별: SEED_DOCK→ROTATE_90→DOCK_CHECK→XN_ALIGN_X→XN_ALIGN_Z→OFFSET, 스태거: lead 먼저). 오케스트레이터 `_run_phase_b(robot_id, params)` 가 각 스텝을 액션/파라미터 호출로 실행.

**구현 노트(진실의 원천):** 러너 `_mission_setup`+스텝1~5 순서 그대로. SEED_DOCK=localizer `ref_ids=[dock_id]`+첫 fix 시딩. ROTATE_90=odom 인스턴스로 +90°(월드 +Z). DOCK_CHECK=후방캠 localizer, `correct_yaw=False`, 도크좌표 위치보정. XN_ALIGN=`ref_ids=[31]` 전환+전방캠, 2단계(먼저 x=xn_x, 그다음 순수 북진 z=xn_z−standoff), `correct_yaw=False`. OFFSET=entry_lead만 x+=−1.7. 각 목표는 `_navigate_goal`(USD 프레임) 재사용.

- [ ] **Step 1: 실패 테스트 — phase_b_plan 스태거/스텝순서**

```python
def test_phase_b_plan_stagger_and_steps():
    from parkbot_motion.pickup_choreography import phase_b_plan
    steps = phase_b_plan('entry_lead', 'entry_follow')
    ids = [s.robot_id for s in steps]
    # lead 전체가 follow 앞에
    assert ids.index('entry_follow') > max(i for i,r in enumerate(ids) if r=='entry_lead')
    lead = [s.phase for s in steps if s.robot_id=='entry_lead']
    assert lead == ['seed_dock','rotate_90','dock_check','xn_align_x','xn_align_z','offset']
    follow = [s.phase for s in steps if s.robot_id=='entry_follow']
    assert 'offset' not in follow   # follow 는 오프셋 없음(XN 축선)
```

- [ ] **Step 2: 실패 확인** → FAIL

- [ ] **Step 3: `phase_b_plan` + `PhaseBStep` namedtuple 구현**(순수)

- [ ] **Step 4: 통과 확인** → PASS

- [ ] **Step 5: 오케스트레이터 `_run_phase_b` 배선**

로봇별 파라미터(도크 id/좌표, XN 좌표, standoff, offset) 선언. 각 스텝을 localizer `set_parameters`(ref_ids/correct_yaw) + navigate(odom/fused) 호출로 실행. `_on_execute_parking_task` 에서 `run_phase_b_first`(파라미터, 기본 True)면 corridor_plan 앞에 두 로봇 Phase B 레그 실행. 실패 시 FAILED 중단.

- [ ] **Step 6: 오케스트레이터 단위테스트(모의 액션)** — `_run_phase_b` 가 스텝 순서대로 올바른 액션/파라미터를 호출하는지 fake client 로 검증.

- [ ] **Step 7: 전체 테스트 + 회귀** — Run: `PYTHONPATH=install/parking_robot_interfaces/local/lib/python3.10/dist-packages python -m pytest src/parkbot_motion/test/ -p no:anyio -v` → PASS

- [ ] **Step 8: Commit** — `feat(orchestrator): Phase B 레그(도크→XN) 순수 시퀀싱+배선`

---

### Task 3b: marker_localizer_node — 이중카메라·단일필터 (전방+후방)

**계기:** T3 opus 리뷰의 ⚠️. 인프로세스 Phase B는 **하나의 공유 필터**를 후방캠(도크 검출)과 전방캠(XN 검출)이 함께 먹인다(`_run_entry_lead_b`가 같은 `filt`을 `rear_ctx`/`front_ctx`에 번갈아 씀). 충실한 ROS2 이식은 로봇당 localizer 1개가 두 카메라를 구독해 한 `PoseFilter`를 공유하는 것 — 그러면 ref_ids 전환만으로 어느 카메라가 필터를 보정할지 결정되고(도크 보이면 후방, XN 보이면 전방), T3의 단일 fused 액션이 정답이 된다.

**Files:**
- Modify: `src/parkbot_aruco/parkbot_aruco/marker_localizer_node.py`
- Test: `src/parkbot_aruco/test/test_marker_localizer_dual_cam.py` (신규)

**Interfaces:**
- Produces: 파라미터 `rear_image_topic`(기본 `""`=후방 비활성/단일카메라=하위호환), `rear_camera_info_topic`, `rear_t_base_cam`(기본=전방 `t_base_cam`의 Rz(180) 미러 유도값). `rear_image_topic`이 비면 기존 단일카메라 동작 완전 불변.
- Consumes: T4 브링업이 로봇별로 `rear_image_topic:=/robot_<id>/rear/image_raw`(T2 발행)를 준다.

**구현 노트:** 두 카메라의 `_on_image`가 **같은 `self.filt`**을 먹인다. 후방은 자기 `rear_t_base_cam`으로 pose 계산, ref_ids 필터·`apply_fix(correct_yaw)`는 T1 로직 공유. 후방 T_base_cam 기본값은 전방(`_DEFAULT_T_BASE_CAM`)을 base_link 수직축 둘레 180° 회전한 미러(러너 Phase A "cam_rear=front Rz180 미러" + REARXN calib_orn=spawn+180° 근거). 정확값은 T5 라이브에서 검증, 필요 시 파라미터로 오버라이드(러너 `_calibrate_tbasecam_yawchecked` 결과를 T5 로그에서 추출).

- [ ] **Step 1: 실패 테스트** — 후방 이미지 콜백이 ref 마커를 보면 같은 filt이 보정되는지, `rear_image_topic=""`면 후방 구독이 안 생기는지(하위호환).
- [ ] **Step 2: 실패 확인**
- [ ] **Step 3: `rear_t_base_cam` 기본값 유도 헬퍼(전방 Rz180 미러) + 순수 단위테스트**
- [ ] **Step 4: 후방 구독·CameraInfo·`_on_image` 분기 배선**(전방 로직 재사용, 마운트만 rear)
- [ ] **Step 5: 전체 통과 + 회귀**(`python -m pytest src/parkbot_aruco/test/ -p no:anyio -v`)
- [ ] **Step 6: Commit** — `feat(aruco): marker_localizer 이중카메라-단일필터(전방+후방)`

---

### Task 4: 브링업 — Phase B 노드 그래프

**Files:**
- Modify: `isaacpjt/Isaac_envo/bringup_pickup_e2e.sh` (Phase B 노드 추가: 후방캠 localizer 인스턴스, odom nav 인스턴스, 파라미터)
- Modify: `isaacpjt/Isaac_envo/pickup_smoke.py` (도크 스폰에서 시작하도록 goal/검증 확장) 또는 신규 `phase_b_e2e_smoke.py`

**Interfaces:**
- Consumes: T1~T3 노드/파라미터, T2 `--bridge-rear`.
- Produces: 도크 스폰→XN→베이→픽업→리프트 전체를 한 번에 기동하는 스크립트. 로봇별 localizer 인스턴스가 전방/후방 이미지 토픽 구독, 오케스트레이터 `run_phase_b_first:=true`.

**구현 노트:** 기존 bringup 은 Phase B 종단(XN)에서 시작한다고 가정한다 — 스폰을 도크로 바꾸고(러너 미션 스폰과 동일 z=2.2), 후방캠 localizer 인스턴스와 `--bridge-rear` 를 추가. 회전 구간은 odom nav 인스턴스가 담당(이미 존재).

- [ ] **Step 1: 스폰을 도크로** — 러너 미션 스폰 좌표(도크 z=2.2, entry_lead/follow 각 도크마커 앞)를 브링업이 쓰게.

- [ ] **Step 2: 후방캠 localizer 인스턴스 + `--bridge-rear` 배선** — 로봇별 `marker_localizer_node`(전방=XN용, 후방=도크점검용) 또는 단일 인스턴스+ref 전환. docstring 에 선택 근거.

- [ ] **Step 3: 오케스트레이터 `run_phase_b_first:=true`** 로 기동.

- [ ] **Step 4: Commit** — `feat(bringup): Phase B(도크 스폰) 포함 E2E 그래프`

---

### Task 5: E2E 검증 — 도크→XN→베이→픽업→리프트 (라이브)

**Files:**
- Modify: `.superpowers/sdd/progress.md`, `DEBUG_LOG.md`(실측), `HANDOFF.md`(결과)

**구현 노트:** 이 태스크는 코드가 아니라 **라이브 Isaac E2E 검증**이다. RTF≈0.5, 다중노드 기동으로 1회 수분 소요. `bringup_pickup_e2e.sh` 로 전 구간 기동 후 GT 로 채점.

- [ ] **Step 1: 전 구간 라이브 실행** — 브링업 기동, 오케스트레이터 `execute_pickup_choreography` 호출. 로그로 SEED→ROTATE→DOCK_CHECK→XN→APPROACH→ALIGN→INGRESS→LIFT 전이 확인.

- [ ] **Step 2: GT 채점** — entry_lead/entry_follow 종단 위치오차, truck_rise. 기준: 인프로세스 실적(lead ~1.1cm, follow ~0.7cm, rise 0.083m) 이내, 축오차 ≤5cm.

- [ ] **Step 3: 재현 2회** — 결정성 확인(값 편차 기록).

- [ ] **Step 4: 미달 시 재튜닝** — 스펙 §5 위험대로 라이브 값이 인프로세스보다 나쁘면 그 구간(회전 스케일/보정 게인/standoff) 재튜닝하고 DEBUG_LOG 에 기록. **인프로세스 동등 달성이 이 태스크의 수용 기준.**

- [ ] **Step 5: Commit** — `test(e2e): 도크→XN→베이→픽업 ROS2 완주 검증(인프로세스 동등)`

---

### Task 6: 러너 미션코드 제거 + 문서화

**Files:**
- Modify: `isaacpjt/Isaac_envo/parking_v4_runner.py` (`_run_mission_b_choreo`/`_run_mission_c_choreo` 및 전용 헬퍼 제거, `--mission` 처리)
- Create: `docs/concepts/ros2-node-architecture.md` (2프로세스 구조 개념도)
- Modify: `HANDOFF.md`, `DEBUG_LOG.md`, `docs/concepts/mission-phase-b-fused-drive.md`(ROS2 이식 부록)

**전제:** T5 E2E 성공 **이후에만** 착수(미션코드 제거 = Phase B 기능을 ROS2 가 대체 확인한 뒤).

**구현 노트:** 러너에 남을 것 — 씬/로봇/차량/마커 스폰, 카메라 그래프, 프로브 13종, `--bridge`(+`--bridge-rear`). 제거할 것 — `_run_mission_b_choreo`(2595), `_run_mission_c_choreo`(3179), 그 전용 중첩헬퍼(`_mission_setup` 등)와 `main()` 의 `mission=="B"/"C"` 분기(3620,3628). `drive_to_pose`/`rotate_in_place` 가 프로브에서도 쓰이면 유지, 미션 전용이면 제거(사용처 확인 후 결정).

- [ ] **Step 1: 미션코드 사용처 조사** — `drive_to_pose`/`rotate_in_place`/`_mission_setup` 등이 프로브에서 쓰이는지 grep. 프로브 전용/공용은 유지, 미션 전용만 제거 대상 확정.

- [ ] **Step 2: `--mission` 리다이렉트** — `--mission=B/C` 는 제거하고, 호출 시 "ROS2 경로로 이전됨: bringup_pickup_e2e.sh 사용" 안내 출력 후 종료.

- [ ] **Step 3: 미션코드 제거** — Step1 에서 확정한 미션 전용 함수/분기 삭제. 러너 줄 수 대폭 감소.

- [ ] **Step 4: 회귀** — 프로브 대표 3종(예: `--probe=FUSE`, `--probe=ROTCHK`, `--probe=REARXN`) + `--bridge` 기동이 정상인지 확인. 139 테스트 유지.

- [ ] **Step 5: ROS2 아키텍처 개념 문서** — `docs/concepts/ros2-node-architecture.md`: 2프로세스(Isaac 브리지 / ROS2 노드) 구조도, 노드별 책임, 액션 계약, Phase B/C 데이터 흐름. 스펙 §3 목표구조를 실제 구현에 맞게 갱신.

- [ ] **Step 6: HANDOFF/DEBUG 갱신** — R6 완료, Phase B ROS2 이식 결과, 러너 최종 줄 수.

- [ ] **Step 7: Commit** — `refactor(runner): 미션코드 제거(프로브·브리지 잔존)+ROS2 구조 문서화`

---

## Self-Review 메모

- **스펙 커버리지**: 스펙 R6("미션코드 제거+문서 갱신")를 T5(사용자 확정 "Phase B 도 ROS2 로")로 확장 — T1~T4가 Phase B 이식, T5가 동등성 게이트, T6가 제거+문서.
- **위험**: 최대 리스크는 라이브 동등성(T5). 특히 (a)후방캠 T_base_cam 보정(러너 `_calibrate_tbasecam_yawchecked` 레퍼런스), (b)회전 후 XN 검출창 진입(러너 2단계 정렬 재현), (c)async DDS vs 60Hz 동기루프 재튜닝(스펙 §5). 미달 시 T5 Step4 에서 구간 재튜닝.
- **되돌림**: T6 는 T5 성공 후에만. 각 태스크 별도 커밋으로 이전 단계 복귀 가능.
- **범위 밖**: Phase D(운반·주차), 두 NavigateToPose 서버 통합(레거시 정리), 축정렬 반복검증 확대.
