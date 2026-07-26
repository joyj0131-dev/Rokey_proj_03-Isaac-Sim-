# ArUco v4 Phase 3 (주행 중 오도메트리 융합 검증) Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development. Steps use checkbox (`- [ ]`) syntax.

**Goal:** 로봇이 마커로 접근 주행하며 휠 오도(예측)+마커(보정)를 상보필터로 융합해, **접근 종단(도킹 직전)의 융합 위치 오차가 2cm/1° 안에 수렴**함을 검증한다. 단일프레임이 못 넘은 각도 꼬리를 융합이 없애는지 확인한다.

**Architecture:** 핵심 융합 필터 `marker_localizer.PoseFilter`(예측/보정 완비)와 Phase 0 휠 오도, M5 의 검출·측위·T_base_cam 자동보정을 재사용한다. 새 `--probe=FUSE` 모드가 로봇을 메카넘으로 접근 주행시키며, 매 스텝 `PoseFilter.predict_body`(휠 오도 속도)로 예측하고 마커가 잡히면 `PoseFilter.update`(마커 fix)로 보정한다. 융합 위치를 GT 와 비교해 종단 오차와 궤적 p95 를 낸다. 배포 노드(`marker_localizer_node`)의 fuse 배선(현재 스텁)도 완성한다.

**Tech Stack:** Python 3.10/3.11, pxr USD, numpy, OpenCV(aruco), Isaac Sim 5.1, ROS 2 Humble

## Global Constraints

- **융합 게이트: 접근 종단(마지막 검출 지점)의 융합 위치 오차 ≤ 0.02 m 그리고 yaw ≤ 1.0°.** 궤적 p95 도 함께 보고한다.
- GT 는 계측(채점) 기준으로만 허용. **필터 초기화·예측·보정에 GT 를 넣지 않는다** — 초기 자세는 첫 마커 fix 로, 예측은 휠 오도(관절 각속도)로, 보정은 마커로만 한다. GT 는 오직 오차 채점에만 쓴다.
- 좌표/yaw 규약: 로봇 yaw=0 → 월드 +Z (`marker_localizer`/`WheelOdometry` 동일). 이 씬에서 로봇은 월드 +X 를 보므로 필터 yaw ≈ 90°이며, `predict_body(vx,..)` 가 그 yaw 로 전방을 +X 로 옮긴다(규약 자기일관).
- 카메라 높이 0.15 m. 검출 창 앞 1.2~2.1 m(Probe A). 접근은 이 창을 지나며 여러 번 검출된다.
- 재사용 부품(수정 금지): `marker_localizer.PoseFilter`(`set_pose`/`predict_body`/`update`/`pose`), `robot_pose_from_marker`, `rvec_tvec_to_T`, `aruco_pose`, `mecanum_drive`(`wheel_velocities_from_cmd_vel`/`cmd_vel_from_wheel_velocities`/`slew_twist`/`WHEEL_JOINTS`), `site_map_v4`.
- `dock_lift_handoff_runner_v2.*` 는 수정 금지. Phase 3 는 `parking_v4_runner.py` 와 `marker_localizer_node.py` 만 손댄다.
- pytest 는 `-p no:anyio`. Isaac 실행 전후 좀비 확인, `--gui` 는 창 유지·헤드리스만 `app.close()`.

---

## File Structure

| 파일 | 책임 |
|---|---|
| `isaacpjt/Isaac_envo/parking_v4_runner.py` | **수정.** M5 공용 헬퍼를 모듈 함수로 추출(Task 1) + `--probe=FUSE` 주행 융합 검증(Task 2). |
| `src/parkbot_aruco/parkbot_aruco/marker_localizer_node.py` | **수정.** fuse=True 배선 완성(휠 오도 구독→predict, 마커→update) + T_base_cam 파라미터(Task 3). |
| `isaacpjt/Isaac_envo/run_marker_localizer_v4.sh` | **신규.** 배포 측위 노드 런처(v4 지도 + fuse + 보정 마운트). |

---

### Task 1: M5 공용 헬퍼를 모듈 함수로 추출 (DRY, 무행동변화)

M5 분기 안의 T_base_cam 자동보정과 "자세에 놓고 렌더→검출→측위" 로직을 모듈 수준 함수로 빼내
`--probe=FUSE` 가 재사용하게 한다. **M5 의 동작·수치는 바뀌지 않아야 한다**(순수 리팩터).

**Files:**
- Modify: `isaacpjt/Isaac_envo/parking_v4_runner.py`

**Interfaces:**
- Produces (모듈 함수):
  - `fuse_camera_setup(stage, timeline, app, target, cam_h) -> dict` — 카메라 높이 오버라이드 + rgb annotator + K + dist + detector + marker_map(v4) + code_size + ref_id + (mx,mz) + spawn_orn 을 담은 컨텍스트 dict `ctx`.
  - `detect_at_pose(ctx, art, app, d, lat, yaw_deg) -> MarkerPose|None` — 로봇을 마커 앞 (d,lat,yaw)에 놓고 렌더→검출, 대상 마커 pose 반환.
  - `calibrate_tbasecam(ctx, art, app, timeline) -> (T_base_cam, best_name, best_err)` — GT 로 광학 규약 후보 스윕해 T_base_cam 확정.
  - `localize_pose(ctx, pose, T_base_cam) -> RobotFix` — pose→T_cam_marker→robot_pose_from_marker.
- Consumes: 기존 러너의 `read_markers`, `marker_visual_center`, `find_front_camera`, `gt_pose_xz_yaw`, `ROBOT_SPAWN_Y`, `REPO_ROOT`, `sm`.

- [ ] **Step 1: 추출 리팩터 — 모듈 함수 4개 신설, M5 분기가 그것을 호출하도록 교체**

`parking_v4_runner.py` 모듈 수준(함수들 사이, `def find_front_camera` 부근)에 아래 4개 함수를 추가한다.
M5 분기의 인라인 로직과 **동일한 계산**을 함수로 옮긴 것이다(값·순서 보존).

```python
def fuse_camera_setup(stage, timeline, app, target, cam_h):
    """카메라 높이 오버라이드 + rgb annotator + K/detector/marker_map 컨텍스트."""
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
    cam_path = find_front_camera(stage, target)
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
```

그리고 **M5 분기**를 위 함수 호출로 바꾼다: 인라인 카메라 셋업/보정/검출/측위를
`ctx = fuse_camera_setup(...)`, `calibrate_tbasecam(...)`, `detect_at_pose(...)`,
`localize_pose(...)` 호출로 대체한다. **M5 의 스윕·판정·리포트 로직과 출력 토큰은 그대로 둔다.**
(M5 분기가 자체 정의하던 `place_and_capture`/`localize` 는 새 모듈 함수로 대체; 스윕 루프는
`detect_at_pose`/`localize_pose` 를 부르도록 최소 수정.)

- [ ] **Step 2: 문법 검사**

Run: `cd /home/rokey/p3/cobot_ws/isaacpjt/Isaac_envo && python3 -m py_compile parking_v4_runner.py && echo OK`
Expected: `OK`

- [ ] **Step 3: M5 회귀 확인 (동작 불변)**

```bash
cd /home/rokey/p3/cobot_ws/isaacpjt/Isaac_envo
ps -eo pid,args --no-headers | grep -E "/kit/kit|python\.sh|isaacsim" | grep -v grep || echo clean
bash parking_v4_runner.sh --probe=M5 --m5-frames=1 2>&1 | grep -E "M5_TBASECAM_CAL|M5_RESULT|Traceback"
```
Expected: 리팩터 전과 같은 형태의 `M5_TBASECAM_CAL best=X180 verify_pos_err≈0.006m` + `M5_RESULT=FAIL ... full_pos_p95≈4cm`(수치는 렌더 비결정으로 소폭 요동 가능하나 구조 동일). 크래시·토큰 누락 없어야 한다.

- [ ] **Step 4: 커밋**

```bash
cd /home/rokey/p3/cobot_ws
git add isaacpjt/Isaac_envo/parking_v4_runner.py
git commit -m "refactor(probe): M5 카메라셋업/보정/검출/측위를 모듈 함수로 추출 (FUSE 재사용)"
```

---

### Task 2: `--probe=FUSE` — 주행 접근 중 오도+마커 융합 검증 (종단 수렴 게이트)

로봇을 마커로 접근 주행시키며 상보필터로 융합, 종단 융합 오차 ≤ 2cm/1° 를 판정한다.

**Files:**
- Modify: `isaacpjt/Isaac_envo/parking_v4_runner.py` (probe 분기에 `FUSE` 추가)

**Interfaces:**
- Consumes: Task 1 의 `fuse_camera_setup`/`calibrate_tbasecam`/`detect_current`/`detect_at_pose`/`localize_pose`; `PoseFilter`(marker_localizer); `mecanum_drive.wheel_velocities_from_cmd_vel`/`cmd_vel_from_wheel_velocities`/`slew_twist`/`WHEEL_JOINTS`; `gt_pose_xz_yaw`, `arts`, `wheel_idx`, `robot_prim_path`.
- Produces: 러너 인자 `--fuse-speed=`(기본 0.25) `--fuse-dstart=`(기본 2.1) `--fuse-dend=`(기본 1.25) `--fuse-posgain=`(기본 0.5) `--fuse-yawgain=`(기본 0.5); 콘솔 `FUSE_TBASECAM_CAL`, `FUSE_RESULT`; 리포트 `probe_reports/fuse_approach.json`.

- [ ] **Step 1: FUSE 분기 추가**

`parking_v4_runner.py` 의 probe 분기 그룹에 아래를 추가:

```python
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
```

- [ ] **Step 2: 문법 검사**

Run: `cd /home/rokey/p3/cobot_ws/isaacpjt/Isaac_envo && python3 -m py_compile parking_v4_runner.py && echo OK`
Expected: `OK`

- [ ] **Step 3: 좀비 확인 후 헤드리스 실행**

```bash
cd /home/rokey/p3/cobot_ws/isaacpjt/Isaac_envo
ps -eo pid,args --no-headers | grep -E "/kit/kit|python\.sh|isaacsim" | grep -v grep || echo clean
bash parking_v4_runner.sh --probe=FUSE 2>&1 | grep -E "FUSE_TBASECAM_CAL|FUSE_RESULT|Traceback|RuntimeError"
```
Expected(대략): `FUSE_TBASECAM_CAL best=X180 verify_pos_err<0.05` + `FUSE_RESULT=... term_pos=<값>cm ...`.
**해석(구현자):**
- `term_pos ≤ 2cm 및 term_yaw ≤ 1° 이면 PASS` — 융합이 종단에서 수렴한 것.
- 융합이 단일프레임보다 나은지 `fused_traj_p95` vs `single_traj_p95` 로 확인해 보고한다.
- PASS 가 안 나오면 억지로 게인/구간을 바꿔 맞추지 말고, 실제 수치(종단·궤적·단일 비교)를 보고한다.
  게인 민감도는 참고로 `--fuse-posgain`/`--fuse-yawgain` 로 1~2회 스윕해 관찰만 하고 결과를 남긴다.

- [ ] **Step 4: 좀비 정리 확인 + 커밋**

```bash
cd /home/rokey/p3/cobot_ws
ps -eo pid,args --no-headers | grep -E "/kit/kit|python\.sh|isaacsim" | grep -v grep || echo clean
git add isaacpjt/Isaac_envo/parking_v4_runner.py
git commit -m "feat(probe): --probe=FUSE 주행 접근 오도+마커 융합, 종단 수렴 게이트"
```

---

### Task 3: 배포 노드 fuse 배선 완성 + v4 런처

`marker_localizer_node` 의 fuse=True 를 실제로 배선(휠 오도 twist 구독→predict, 마커→update)하고,
보정된 T_base_cam 을 파라미터로 받게 한다. v4 지도로 실행하는 런처를 만든다.

**Files:**
- Modify: `src/parkbot_aruco/parkbot_aruco/marker_localizer_node.py`
- Create: `isaacpjt/Isaac_envo/run_marker_localizer_v4.sh`

**Interfaces:**
- Consumes: `marker_localizer.PoseFilter`, 러너가 발행하는 `/robot_entry_lead/odom`(nav_msgs/Odometry) — 여기서 프레임 간 이동량(월드 dx,dz,dyaw)을 뽑아 `filt.predict(dx,dz,dyaw)` 로 예측.
- Produces: 노드가 `fuse:=true` 일 때 `/robot_<id>/odom` 을 구독해 예측+마커 보정한 `/robot_pose` 발행. `t_base_cam` 파라미터로 마운트 주입. 런처 `run_marker_localizer_v4.sh`.

- [ ] **Step 1: 노드 fuse 배선**

Modify `marker_localizer_node.py` — `__init__` 에서 `fuse=True` 면 `PoseFilter` 를 만들고
`/robot_<id>/odom`(파라미터 `odom_topic`, 기본 `/robot_entry_lead/odom`)을 구독한다. 이미지 콜백은
마커 fix 로 `filt.update` 또는 첫 fix 로 `set_pose` 하고, odom 콜백은 직전 odom 대비 월드 증분으로
`filt.predict`. `fuse=False` 면 기존처럼 마커 단독 발행(회귀).

```python
        self.declare_parameter("odom_topic", "/robot_entry_lead/odom")
        ...
        self.filt = None
        self._last_odom = None
        if self.fuse:
            from parkbot_aruco.marker_localizer import PoseFilter
            self.filt = PoseFilter()
            self.create_subscription(Odometry, self.get_parameter("odom_topic").value,
                                     self._on_odom, qos_profile_sensor_data)
```
odom 콜백(월드 XZ 증분 → predict):
```python
    def _on_odom(self, msg):
        # 러너 odom 은 위치를 (x, ·, z), yaw 를 z/w 쿼터니언으로 담는다(XZ 평면).
        x = msg.pose.pose.position.x
        z = msg.pose.pose.position.z
        qz, qw = msg.pose.pose.orientation.z, msg.pose.pose.orientation.w
        yaw = math.degrees(2.0 * math.atan2(qz, qw))
        if self.filt is not None and self._last_odom is not None and self.filt.x is not None:
            lx, lz, lyaw = self._last_odom
            self.filt.predict(x - lx, z - lz, _wrap := ((yaw - lyaw + 180.0) % 360.0 - 180.0))
        self._last_odom = (x, z, yaw)
```
이미지 콜백에서 fix 산출 후:
```python
        if self.fuse and self.filt is not None:
            if self.filt.x is None:
                self.filt.set_pose(fix.x, fix.z, fix.yaw_deg)
            else:
                self.filt.update(fix)
            px, pz, pyaw = self.filt.pose()
        else:
            px, pz, pyaw = fix.x, fix.z, fix.yaw_deg
        # 이후 px,pz,pyaw 로 PoseStamped 발행(기존 x,z,yaw 자리 대체)
```
`from nav_msgs.msg import Odometry` import 추가. 기존 마커 단독 경로(`fuse=False`)의 출력은 불변.

- [ ] **Step 2: 노드 import·문법 확인 (시스템 ROS)**

```bash
cd /home/rokey/p3/cobot_ws
bash -c 'set +u; source /opt/ros/humble/setup.bash; set -u; export PYTHONPATH=$PWD/src/parkbot_aruco:$PYTHONPATH; python3 -c "from parkbot_aruco import marker_localizer_node; print(\"node import OK\")"'
```
Expected: `node import OK`

- [ ] **Step 3: v4 런처 작성**

Create `isaacpjt/Isaac_envo/run_marker_localizer_v4.sh`:

```bash
#!/bin/bash
# 배포 측위(터미널 B): marker_localizer_node 를 v4 지도 + fuse + 보정 마운트로 실행.
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
PKG_SRC="$(cd -- "$SCRIPT_DIR/../../src/parkbot_aruco" && pwd)"
MAP="$PKG_SRC/data/marker_map_v4.json"
set +u
source /opt/ros/humble/setup.bash
set -u
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-126}"
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE="${FASTRTPS_DEFAULT_PROFILES_FILE:-$HOME/.ros/fastdds_whitelist.xml}"
export PYTHONPATH="$PKG_SRC:${PYTHONPATH:-}"
exec python3 -m parkbot_aruco.marker_localizer_node --ros-args \
  -p image_topic:=/robot_entry_lead/image_raw \
  -p camera_info_topic:=/robot_entry_lead/camera_info \
  -p odom_topic:=/robot_entry_lead/odom \
  -p marker_map:="$MAP" -p fuse:=true -p frame:=usd "$@"
```

- [ ] **Step 4: 실행 권한 + 런처 문법**

```bash
cd /home/rokey/p3/cobot_ws && chmod +x isaacpjt/Isaac_envo/run_marker_localizer_v4.sh
bash -n isaacpjt/Isaac_envo/run_marker_localizer_v4.sh && echo "sh OK"
```
Expected: `sh OK`

- [ ] **Step 5: 커밋**

```bash
cd /home/rokey/p3/cobot_ws
git add src/parkbot_aruco/parkbot_aruco/marker_localizer_node.py \
        isaacpjt/Isaac_envo/run_marker_localizer_v4.sh
git commit -m "feat(aruco): marker_localizer_node fuse 배선 완성(휠오도 predict+마커 update) + v4 런처"
```

---

## Notes for the implementer

- **Isaac 은 느리다.** FUSE 는 접근 주행(수 초 sim) + 스텝마다 렌더/검출이라 수 분 걸린다. 백그라운드+로그폴링.
- **좀비 프로세스** 매 실행 전후 확인·정리. `--gui` 는 창 유지, 헤드리스만 `app.close()`.
- **GT 는 채점 전용.** 필터 초기화(첫 fix)·예측(휠 오도)·보정(마커) 어디에도 GT 를 넣지 않는다. `gt_pose_xz_yaw` 는 오차 계산에만.
- 융합이 종단에서 2cm 를 못 넘으면 억지로 맞추지 말고 수치를 보고한다(게인 스윕은 관찰만).
