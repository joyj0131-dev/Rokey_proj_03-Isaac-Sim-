# 미션 Phase B (도크→XN 융합 주행) 구현 계획

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax.

**Goal:** 입차팀 2로봇이 GT 없이 마커+휠오도 융합측위로 실제 바퀴를 굴려 도크→인계장 마커 XN 앞까지 주행·정렬한다(`--mission=B`, GUI 실행).

**Architecture:** 러너의 검증된 FUSE 융합주행 루프(실제 바퀴 + predict_body 예측 + 마커 update 보정)를 다로봇·임의 목표자세로 일반화한다. 순수 제어법 `body_twist_toward` 를 단위테스트로 고정하고, 이를 감싼 폐루프 `drive_to_pose`/`rotate_in_place` 로 안무 상태기계를 구동한다. 카메라는 역할별로 발행(전/후 동시).

**Tech Stack:** Isaac Sim 5.1(python.sh), ROS2 Humble(C++ OmniGraph 카메라 발행), OpenCV ArUco(DICT_5X5_100), numpy. pytest 는 `-p no:anyio` 필수.

## Global Constraints

- **GT 는 제어에 사용 금지.** 측위=마커+휠오도 융합(초기화=도크 마커 fix, 예측=휠오도, 보정=마커). GT 는 스모크 채점/기록에만.
- 주행 구간은 **실제 메카넘 바퀴**(`set_joint_velocity_targets`)로. 텔레포트(`set_world_poses`)는 스폰/초기배치에만.
- **UI 실행 가능**: `parking_v4_runner.sh --mission=B --gui`. headless 스모크 변형도 제공.
- `parking_v4_runner.py`(4-카메라 자산) 위에 구축. 레거시 `dock_lift_handoff_runner*.py` 미사용.
- 좌표: yaw=0 → +Z(`atan2(fwd_x, fwd_z)`). 로봇 +X 스폰 → filter-yaw ≈ 90°. body +x=전진(heading), +y=좌(+90°).
- 개체: XN=crossing_N(id 31, x=-2.5, z=+6.875). 도크 entry_lead=D_OUT_1(-3.2,+2.9), entry_follow=D_OUT_2(-1.2,+2.9).
- 역할: entry_follow=앞축(전방캠 XN 정면), entry_lead=뒤축(후방캠 XN, +x 오프셋).
- 기존 `--probe=FUSE/M5/REAR/A` 회귀 없어야 함(전방·측면 미변경).
- 한국어 주석/문서.

---

## File Structure

- `isaacpjt/Isaac_envo/mission_control.py` (신규) — 순수 제어법 `body_twist_toward` + 안무 상태기계 상수/헬퍼(Isaac 비의존 부분). 단위테스트 대상.
- `isaacpjt/Isaac_envo/tests/test_mission_control.py` (신규) — 순수 제어법 단위테스트.
- `isaacpjt/Isaac_envo/parking_v4_runner.py` (수정) — `attach_camera_graph` 역할화, `--mission=B` 분기(폐루프 `drive_to_pose`/`rotate_in_place`, 안무 오케스트레이션).
- `HANDOFF.md` / `DEBUG_LOG.md` (수정) — Phase B 결과·디버깅.
- `docs/concepts/mission-phase-b-fused-drive.md` (신규) — 개념 설명(융합주행·안무·카메라 역할).

---

## Task 0: 카메라 역할 파라미터화

**Files:**
- Modify: `isaacpjt/Isaac_envo/parking_v4_runner.py` (`attach_camera_graph` 및 호출부)

**Interfaces:**
- Consumes: 기존 `attach_camera_graph(robot_id, cam_path, width, height)`, `find_front_camera`, `find_rear_camera`.
- Produces: `attach_camera_graph(robot_id, cam_path, role="front", width=640, height=480)` — 토픽 `/robot_<id>/<role>/image_raw`(+`/camera_info`), 그래프 경로 `/Graphs/cam_<id>_<role>`.

- [ ] **Step 1: `attach_camera_graph` 에 role 추가**

`attach_camera_graph` 서명에 `role="front"` 추가. 함수 내부에서 ROS 네임스페이스와 OmniGraph 경로를 `robot_id` 단독이 아니라 `role` 을 포함해 파생하도록 수정:
- 네임스페이스: `/robot_{robot_id}` → `/robot_{robot_id}/{role}` (토픽이 `/robot_<id>/<role>/image_raw`, `/robot_<id>/<role>/camera_info` 가 되도록).
- OmniGraph 경로: `/Graphs/cam_{robot_id}` → `/Graphs/cam_{robot_id}_{role}`.
기존 호출부(전방)는 `role="front"` 기본값으로 동작이 바뀌지 않게(단, 토픽 경로에 `/front/` 가 추가됨 — 아래 Step 2 에서 검출측/probe 정합 확인).

- [ ] **Step 2: 호출부 갱신 + 토픽 정합**

`main` 의 카메라 부착 루프(전방)와 `--probe=FUSE`/`fuse_camera_setup`/`--probe=REAR` 의 부착 호출이 새 서명을 쓰도록 갱신. `fuse_camera_setup` 이 내부 검출에 쓰는 토픽/그래프 참조도 `/front/` 로 정합. `--probe=REAR` 는 `role="rear"` 로 부착(토픽 `/robot_<id>/rear/image_raw`). REAR 스모크가 여전히 통과하도록 `run_probe_a_detector.sh` 가 구독하는 토픽이 바뀌면 probe 쪽 기본 토픽 인자도 맞춰 갱신(또는 REAR 분기 로그가 새 토픽을 출력).

- [ ] **Step 3: 문법 검사**

Run: `cd /home/rokey/p3/cobot_ws/isaacpjt/Isaac_envo && python3 -m py_compile parking_v4_runner.py && echo OK`
Expected: `OK`

- [ ] **Step 4: 스모크 — 역할별 발행 + FUSE 회귀**

```bash
cd /home/rokey/p3/cobot_ws/isaacpjt/Isaac_envo
ps -eo pid,args --no-headers | grep -E "/kit/kit|python\.sh|isaacsim" | grep -v grep || echo clean
# entry_follow 에 전방+후방 모두 부착되는 임시 확인은 Task 3 에서. 여기선 FUSE 회귀만.
bash parking_v4_runner.sh --probe=FUSE 2>&1 | grep -E "FUSE_TBASECAM_CAL|FUSE_RESULT|Traceback" | head -3
```
Expected: `FUSE_TBASECAM_CAL best=X180 ...` + `FUSE_RESULT=PASS ...`(역할화가 전방 검출을 안 깼음). 좀비 없음.

- [ ] **Step 5: 커밋**

```bash
cd /home/rokey/p3/cobot_ws
git add isaacpjt/Isaac_envo/parking_v4_runner.py
git commit -m "feat(camera): attach_camera_graph 역할 파라미터화(전/후 동시 발행 지원)"
```

---

## Task 1: 순수 제어법 `body_twist_toward` + 단위테스트

**Files:**
- Create: `isaacpjt/Isaac_envo/mission_control.py`
- Test: `isaacpjt/Isaac_envo/tests/test_mission_control.py`

**Interfaces:**
- Consumes: 없음(순수 함수). 자세 규약은 `wheel_odometry.py`/`marker_localizer.predict_body` 와 동일해야 함.
- Produces: `body_twist_toward(cur, tgt, *, pos_gain, yaw_gain, max_lin, max_ang, pos_tol, yaw_tol) -> (vx, vy, wz, done)`. cur/tgt = `(x, z, yaw_deg)` 월드. 반환 twist 는 body frame(+x 전진, +y 좌, wz rad/s CCW).

- [ ] **Step 1: 실패 테스트 작성**

`isaacpjt/Isaac_envo/tests/test_mission_control.py`:
```python
import math, sys, pathlib
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))
from mission_control import body_twist_toward
from wheel_odometry import WheelOdometry

def test_target_ahead_gives_forward():
    # 로봇이 +Z(yaw=0)를 보고 있고 목표가 바로 앞(+Z)이면 전진(vx>0), 횡·회전≈0.
    vx, vy, wz, done = body_twist_toward((0.0, 0.0, 0.0), (0.0, 1.0, 0.0))
    assert vx > 0.1 and abs(vy) < 1e-6 and abs(wz) < 1e-6 and not done

def test_target_left_gives_left_strafe():
    # +Z 를 보는 로봇의 목표가 월드 -X(좌측)면 body +y(좌) 성분이 양(mecanum 횡이동).
    vx, vy, wz, done = body_twist_toward((0.0, 0.0, 0.0), (-1.0, 0.0, 0.0))
    assert vy > 0.1

def test_yaw_error_sign():
    # 목표 yaw 가 현재보다 +면 wz>0(CCW).
    _, _, wz, _ = body_twist_toward((0.0, 0.0, 0.0), (0.0, 0.0, 30.0))
    assert wz > 0

def test_done_within_tolerance():
    _, _, _, done = body_twist_toward((0.0, 0.0, 0.0), (0.005, 0.005, 0.2))
    assert done

def test_roundtrip_reduces_error():
    # 핵심: body_twist_toward 출력을 predict_body 로 적분하면 목표에 가까워져야 한다
    # (제어법 basis 가 오도 적분 basis 와 일치함을 행동으로 검증).
    cur = (0.0, 0.0, 90.0)          # +X 를 보는 로봇
    tgt = (1.0, 0.5, 90.0)
    od = WheelOdometry(x=cur[0], z=cur[1], yaw=cur[2])
    d0 = math.hypot(tgt[0]-cur[0], tgt[1]-cur[1])
    for _ in range(50):
        vx, vy, wz, done = body_twist_toward((od.x, od.z, od.yaw), tgt)
        od.update(vx, vy, wz, 0.1)
        if done: break
    d1 = math.hypot(tgt[0]-od.x, tgt[1]-od.z)
    assert d1 < d0 * 0.2
```

- [ ] **Step 2: 실패 확인**

Run: `cd /home/rokey/p3/cobot_ws && python3 -m pytest isaacpjt/Isaac_envo/tests/test_mission_control.py -p no:anyio -q`
Expected: FAIL (`ModuleNotFoundError: mission_control` 또는 함수 없음).

- [ ] **Step 3: 최소 구현**

`isaacpjt/Isaac_envo/mission_control.py`:
```python
"""미션 안무의 Isaac 비의존 순수 로직(제어법·상수). 단위테스트 대상."""
import math

def body_twist_toward(cur, tgt, *, pos_gain=0.8, yaw_gain=1.2,
                      max_lin=0.25, max_ang=0.6, pos_tol=0.03, yaw_tol=0.5):
    """월드 현재/목표 자세 → body frame twist(vx,vy,wz)+done.

    자세 규약(wheel_odometry 와 동일): yaw=0→+Z. heading 단위벡터
    fwd=(sin yaw, cos yaw), 좌(+90°) left=(cos yaw, -sin yaw). (x,z 평면)
    """
    cx, cz, cyaw = cur
    tx, tz, tyaw = tgt
    dx, dz = tx - cx, tz - cz
    yr = math.radians(cyaw)
    fwd_x, fwd_z = math.sin(yr), math.cos(yr)
    left_x, left_z = math.cos(yr), -math.sin(yr)
    fwd = dx * fwd_x + dz * fwd_z            # body +x(전진)
    left = dx * left_x + dz * left_z         # body +y(좌)
    dyaw = ((tyaw - cyaw + 180.0) % 360.0) - 180.0
    clamp = lambda v, m: max(-m, min(m, v))
    vx = clamp(pos_gain * fwd, max_lin)
    vy = clamp(pos_gain * left, max_lin)
    wz = clamp(yaw_gain * math.radians(dyaw), max_ang)
    done = (math.hypot(dx, dz) <= pos_tol) and (abs(dyaw) <= yaw_tol)
    return vx, vy, wz, done
```
> 주: `left` 부호와 `WheelOdometry.update` 의 `x += fwd*s + left*c` basis 가 일치해야 `test_roundtrip_reduces_error` 가 통과한다. 실패 시 `wheel_odometry.py`/`marker_localizer.predict_body` 의 실제 basis 를 읽어 부호를 맞춰라(Phase 0 오라클 교차검증과 동일 원리). 억지로 테스트를 바꾸지 말 것.

- [ ] **Step 4: 통과 확인**

Run: `cd /home/rokey/p3/cobot_ws && python3 -m pytest isaacpjt/Isaac_envo/tests/test_mission_control.py -p no:anyio -q`
Expected: `5 passed`.

- [ ] **Step 5: 커밋**

```bash
cd /home/rokey/p3/cobot_ws
git add isaacpjt/Isaac_envo/mission_control.py isaacpjt/Isaac_envo/tests/test_mission_control.py
git commit -m "feat(mission): 순수 제어법 body_twist_toward + 단위테스트(오도 basis 교차검증)"
```

---

## Task 2: 폐루프 주행 원시요소 + `--mission=B` 스켈레톤

**Files:**
- Modify: `isaacpjt/Isaac_envo/parking_v4_runner.py`

**Interfaces:**
- Consumes: `body_twist_toward`(Task 1), 기존 FUSE 헬퍼(`fuse_camera_setup`, `calibrate_tbasecam`, `detect_current`, `localize_pose`), `PoseFilter`, `slew_twist`, `wheel_velocities_from_cmd_vel`, `cmd_vel_from_wheel_velocities`, `set_joint_velocity_targets`.
- Produces: 러너 내부 `drive_to_pose(ctx, art, idx, filt, T_base_cam, target_xzyaw, ...) -> bool`(도달 시 True), `rotate_in_place(...)`(회전 목표까지), `--mission=B` 분기(1로봇 짧은 주행 스모크).

- [ ] **Step 1: `--mission=B` 인자 파싱 + 폐루프 헬퍼**

`main` 에 `--mission=` 파싱 추가. `probe==None and mission=="B"` 경로에서, FUSE 루프(러너 865~ 참조)를 함수로 일반화한 `drive_to_pose` 를 구현: 매 스텝
1) 융합 자세 추정(`filt.pose()`; 예측=`predict_body(cmd_vel_from_wheel_velocities(get_joint_velocities), dt)`, 보정=`detect_current→localize_pose→filt.update`),
2) `body_twist_toward(filt.pose_xzyaw, target)` → twist,
3) `slew_twist`→`wheel_velocities_from_cmd_vel`→`set_joint_velocity_targets`,
4) `done` 이면 정지 반환. `max_steps` 안전 상한.
`rotate_in_place(target_yaw)` 는 target=(현재 x,z,yaw) 로 `drive_to_pose` 재사용.

- [ ] **Step 2: 스켈레톤 스모크 — 1로봇 짧은 주행**

`--mission=B` 가 (아직 안무 없이) entry_lead 를 도크에서 **+Z 로 1.0m** 이동시키고 도달 토큰을 출력하도록 임시 배선:
```
MISSIONB_DRIVE robot=entry_lead reached=True err_pos=<m> err_yaw=<deg> steps=<n>
```
Run(백그라운드+폴링, 좀비 확인):
```bash
cd /home/rokey/p3/cobot_ws/isaacpjt/Isaac_envo
bash parking_v4_runner.sh --mission=B 2>&1 | grep -E "MISSIONB_DRIVE|Traceback" | head
```
Expected: `MISSIONB_DRIVE robot=entry_lead reached=True err_pos<0.05 ...`(실제 바퀴로 융합주행해 목표 도달). 검출 안 되는 구간이 있으면 오도 예측만으로도 도달하는지 기록.

- [ ] **Step 3: 문법 검사 + 커밋**

```bash
cd /home/rokey/p3/cobot_ws
python3 -m py_compile isaacpjt/Isaac_envo/parking_v4_runner.py && echo OK
git add isaacpjt/Isaac_envo/parking_v4_runner.py
git commit -m "feat(mission): --mission=B 스켈레톤 + 폐루프 drive_to_pose/rotate_in_place"
```

---

## Task 3: 안무 통합 — 도크 체크·90° 회전·XN 주행·스태거 정렬 (두 로봇)

**Files:**
- Modify: `isaacpjt/Isaac_envo/parking_v4_runner.py`

**Interfaces:**
- Consumes: Task 2 의 `drive_to_pose`/`rotate_in_place`, `attach_camera_graph(role=...)`(Task 0), `find_front_camera`/`find_rear_camera`, `read_markers`/`marker_visual_center`(XN 좌표).
- Produces: `--mission=B` 전체 안무. 완료 토큰 `MISSIONB_DONE robot=<id> role=<front_axle|rear_axle> xn_locked=<bool> pos=(x,z) yaw=<deg>` 로봇당 1줄 + `MISSIONB_RESULT ok=<bool>`.

- [ ] **Step 1: 안무 상태기계 구현**

두 입차 로봇에 대해:
1. **카메라 부착**: entry_lead 는 후방(role="rear"), entry_follow 는 전방+후방 둘 다(전방=XN 정렬, 후방=도크 점검).
2. **DOCK_CHECK+ROTATE90**: 각 로봇 `rotate_in_place`로 +X(yaw≈90°)→+Z(yaw≈0°). 회전 중/후 후방캠으로 도크 마커(entry_lead=D_OUT_1, entry_follow=D_OUT_2) 검출→`filt.set_pose`(도크 마커 실좌표로 초기 fix). 검출 실패 시 로그로 정직 보고.
3. **DRIVE_TO_XN**: XN 좌표(`marker_visual_center`로 crossing_N)를 목표로 `drive_to_pose`(+Z 주행). nominal 목표: entry_follow=(XN.x, XN.z−앞여유, yaw 0°); entry_lead=(XN.x, XN.z−여유, yaw 180°) 로 뒤가 XN 향하게.
4. **ALIGN_XN(스태거)**: entry_lead 먼저 — 후방캠으로 XN 검출·정렬 후 **+x 오프셋**(`OFFSET_X`, nominal 0.6m) 이동. 이어 entry_follow — 전방캠으로 XN 검출·정렬.
5. 각 로봇 `MISSIONB_DONE ...` 출력. 둘 다 성공이면 `MISSIONB_RESULT ok=True`.
nominal 목표 yaw/오프셋은 상단 상수로(GUI 튜닝 대상).

- [ ] **Step 2: headless 스모크 — 두 로봇 완료**

```bash
cd /home/rokey/p3/cobot_ws/isaacpjt/Isaac_envo
ps -eo pid,args --no-headers | grep -E "/kit/kit|python\.sh|isaacsim" | grep -v grep || echo clean
bash parking_v4_runner.sh --mission=B 2>&1 | grep -E "MISSIONB_DONE|MISSIONB_RESULT|Traceback"
```
Expected: `MISSIONB_DONE robot=entry_lead role=rear_axle xn_locked=True ...` + `MISSIONB_DONE robot=entry_follow role=front_axle xn_locked=True ...` + `MISSIONB_RESULT ok=True`. 좀비 없음.
- 마커 미검출/사각으로 xn_locked=False 가 나오면 억지 통과 금지 — 실측 자세·검출 로그를 보고, 근거리 사각(<1.1m)에 목표가 걸리면 nominal 접근거리를 사각 밖으로 조정하고 재실측.

- [ ] **Step 3: GUI 실행 확인(수동 1회)**

Run: `bash parking_v4_runner.sh --mission=B --gui` — 두 로봇이 도크에서 나와 회전·주행·정렬하는지 육안 확인(RTF 낮음, 인내). 결과를 report 에 서술.

- [ ] **Step 4: 커밋**

```bash
cd /home/rokey/p3/cobot_ws
python3 -m py_compile isaacpjt/Isaac_envo/parking_v4_runner.py && echo OK
git add isaacpjt/Isaac_envo/parking_v4_runner.py
git commit -m "feat(mission): Phase B 안무(도크체크·90°회전·XN주행·스태거 정렬) 두 로봇 완주"
```

---

## Task 4: 문서 (HANDOFF/DEBUG + 개념 md)

**Files:**
- Create: `docs/concepts/mission-phase-b-fused-drive.md`
- Modify: `HANDOFF.md`, `DEBUG_LOG.md`

- [ ] **Step 1: 개념 md**

`docs/concepts/mission-phase-b-fused-drive.md` 에: (a) 융합주행 원리(예측=휠오도 predict_body, 보정=마커 update, GT 배제), (b) `body_twist_toward` 제어법과 자세 규약(yaw=0→+Z), (c) 안무 상태기계와 두 로봇 역할(앞축/뒤축, 카메라 역할), (d) 카메라 역할별 발행 토픽 규약, (e) 알려진 한계(근거리 <1.1m 사각). 실측 수치는 Task 3 스모크 결과 인용.

- [ ] **Step 2: HANDOFF/DEBUG 갱신 + 커밋**

`HANDOFF.md` 에 Phase B 결과(두 로봇 XN 정렬 완주, 융합측위, 토큰/수치), `DEBUG_LOG.md` 에 막혔던 것(카메라 역할화 토픽 정합, 제어법 basis, 사각 대응 등) 간결히 기록.
```bash
cd /home/rokey/p3/cobot_ws
git add docs/concepts/mission-phase-b-fused-drive.md HANDOFF.md DEBUG_LOG.md
git commit -m "docs: 미션 Phase B(융합 주행) 개념·결과·디버깅 기록"
```

---

## Self-Review 메모

- 스펙 §5 수용기준 매핑: 융합주행=Task2/3, 두 로봇 XN 정렬·오프셋=Task3, FUSE 회귀=Task0, 단위테스트=Task1, GUI=Task3 Step3, 문서=Task4.
- GT 금지 준수: drive_to_pose 는 `filt.pose`(융합)만 목표비교에 사용. GT(`gt_pose_xz_yaw`)는 스모크 `err_*` 채점·기록에만.
- 미검출 사각(<1.1m)은 Task2/3 에서 정직 보고 + nominal 접근거리로 회피(억지 통과 금지).
- nominal 목표 yaw/오프셋은 상수 — GUI 실측으로 튜닝, 값은 report 에 기록.
