# 미션 Phase A (로봇 카메라 4대 정리) Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development. Steps use checkbox (`- [ ]`) syntax.

**Goal:** 로봇 에셋을 정확히 4개 카메라(front, **rear(신규)**, side_left, side_right)로 만든다 — 레거시 `cam_qr_down` 제거 + `cam_rear`(front 미러) 추가 — 그리고 러너가 후방 카메라를 발행·검출할 수 있게 한다.

**Architecture:** pxr 저작 스크립트가 로봇 에셋을 열어 `cam_front` 카메라를 읽고, 그 마운트를 로봇 수직축(base_link Z) 기준 180° 회전한 미러로 `cam_rear` 를 저작하며 `cam_qr_down` 을 제거해 새 4-카메라 에셋으로 저장한다. 러너는 새 에셋을 참조하고 `find_rear_camera` 로 후방 카메라를 타깃한다. 후방 카메라는 RGB ArUco 용(측면 뎁스캠이 바퀴감지를 담당하므로 depth 불필요).

**Tech Stack:** Isaac Sim 5.1(python.sh, pxr/Sdf), numpy, OpenCV(aruco), ROS 2 Humble

## Global Constraints

- 대상 로봇 에셋: `isaacpjt/hwia_parking_robot_final_caster_package/hwia_depth_cam_mecha_roller_lowered.usd`(러너가 쓰는 것). **팀 원본 소스·기존 에셋 미변경** — 새 변형본 `hwia_4cam_mecha_roller_lowered.usd` 를 저장한다.
- 결과 카메라는 **정확히 4개**: `cam_front`, `cam_rear`, `cam_side_left`, `cam_side_right`. `cam_qr_down` 없음.
- `cam_rear` 는 `cam_front` 의 미러: 로봇 base_link Z(수직)축 180° 회전 → 뒤(−X)를 향하고 하향각·화각·intrinsics 동일. **RGB ArUco 용**(depth 미러 불필요).
- 카메라 프림은 이름/경로에 `front`/`rear` 가 들어가 `find_front_camera`/`find_rear_camera`(Camera 타입 중 이름 매칭)로 구분된다. 알려진 구조 패턴: `<root>/cam_<X>_link/depth_cam_<X>/Camera_Pseudo_Depth_<X>`.
- 기존 `cam_front`/`cam_side_*` 및 롤러·물리·지오메트리 미변경. 기존 FUSE/M5(전방) 동작 회귀 없음.
- Isaac 실행 전후 좀비 확인: `ps -eo pid,args --no-headers | grep -E "/kit/kit|python\.sh|isaacsim" | grep -v grep || echo clean`. `--gui` 창 유지, 헤드리스만 `app.close()`.
- 진행·디버깅은 HANDOFF.md / DEBUG_LOG.md 기록(Phase A 완료 시).

---

## File Structure

| 파일 | 책임 |
|---|---|
| `isaacpjt/hwia_parking_robot_final_caster_package/build_rear_camera.py` | **신규.** 로봇 에셋 → cam_rear 저작 + cam_qr_down 제거 → 4-카메라 에셋 저장. Isaac python(pxr). |
| `isaacpjt/hwia_parking_robot_final_caster_package/hwia_4cam_mecha_roller_lowered.usd` | **신규(생성물).** 4-카메라 로봇 에셋. |
| `isaacpjt/Isaac_envo/parking_v4_runner.py` | **수정.** `ROBOT_USD` 4-카메라본 전환 + `find_rear_camera` + 4-카메라 스모크 토큰. |

---

### Task 1: `build_rear_camera.py` — cam_rear 저작 + cam_qr_down 제거

로봇 에셋을 열어 cam_front 를 미러한 cam_rear 를 저작하고 cam_qr_down 을 제거해 4-카메라 에셋을 만든다.

**Files:**
- Create: `isaacpjt/hwia_parking_robot_final_caster_package/build_rear_camera.py`
- Create(생성물): `isaacpjt/hwia_parking_robot_final_caster_package/hwia_4cam_mecha_roller_lowered.usd`

**Interfaces:**
- Produces: 4-카메라 에셋(파일). 검증 토큰 `REARCAM_INSPECT`, `REARCAM_BUILT`, `REARCAM_VERIFY=PASS|FAIL`.

- [ ] **Step 1: 구조 확인용 인스펙트 모드 먼저 작성·실행**

먼저 에셋의 실제 카메라 계층을 찍어 저작 대상 경로·변환을 확정한다(크레이트라 실행 시점에 읽어야 정확). Create `build_rear_camera.py` (인스펙트 부분 먼저):

```python
#!/usr/bin/env python3
"""로봇 에셋의 cam_front 를 미러해 cam_rear 를 저작하고 cam_qr_down 을 제거한다.

측면 뎁스캠이 바퀴감지를 담당하므로 cam_rear 는 RGB ArUco 용이다. cam_front 의
마운트를 base_link 수직축(Z) 180° 회전한 미러로 만든다(뒤를 향하고 하향각 유지).

실행: python.sh build_rear_camera.py --inspect      # 카메라 계층 출력만
      python.sh build_rear_camera.py                # 저작 + 저장 + 검증
"""
import sys
from pathlib import Path

PKG = Path(__file__).resolve().parent
SRC = PKG / "hwia_depth_cam_mecha_roller_lowered.usd"
OUT = PKG / "hwia_4cam_mecha_roller_lowered.usd"
ISAAC_PY = Path("/home/rokey/dev_ws/isaac_sim/isaacsim/_build/linux-x86_64/release/python.sh")


def _boot():
    from isaacsim import SimulationApp
    return SimulationApp({"headless": True})


def inspect(stage):
    from pxr import UsdGeom, Usd
    cams = [p for p in stage.Traverse() if p.GetTypeName() == "Camera"]
    print(f"REARCAM_INSPECT camera_count={len(cams)}", flush=True)
    for p in cams:
        print(f"  CAMERA {p.GetPath()}", flush=True)
    for link in ("cam_front_link", "cam_qr_down_link",
                 "cam_side_left_link", "cam_side_right_link", "cam_rear_link"):
        for x in stage.Traverse():
            if x.GetName() == link:
                m = UsdGeom.Xformable(x).GetLocalTransformation(Usd.TimeCode.Default())
                t = m.ExtractTranslation()
                print(f"  LINK {link} t=({t[0]:+.3f},{t[1]:+.3f},{t[2]:+.3f})", flush=True)


def main():
    app = _boot()
    from pxr import Usd
    stage = Usd.Stage.Open(str(SRC))
    if "--inspect" in sys.argv[1:]:
        inspect(stage)
        app.close()
        return
    build(stage)                    # Step 3 에서 정의
    app.close()


if __name__ == "__main__":
    main()
```

Run: `cd /home/rokey/p3/cobot_ws/isaacpjt/hwia_parking_robot_final_caster_package && /home/rokey/dev_ws/isaac_sim/isaacsim/_build/linux-x86_64/release/python.sh build_rear_camera.py --inspect 2>&1 | grep -E "REARCAM_INSPECT|CAMERA|LINK"`
Expected: `REARCAM_INSPECT camera_count=4` + 4개 CAMERA 경로(front/qr_down/side_left/side_right) + 각 LINK 좌표. **실측 경로·좌표를 Step 3 저작에 사용한다.** (백그라운드+로그폴링, Isaac 부팅 느림.)

- [ ] **Step 2: 저작 요구사항 확정(인스펙트 결과 반영)**

인스펙트로 얻은 것:
- `cam_front_link` 서브트리 구조(`depth_cam_front/Camera_Pseudo_Depth_Front` 등)와 로컬 변환 `T_front`.
- `cam_qr_down_link` 경로.

`cam_rear` 마운트 = **base_link Z축 180° 회전 미러**: `T_rear = Rz(180°) @ T_front`. 이는 위치의
x(,y) 부호를 반전하고 방향을 수직축 기준 180° 돌려 **뒤(−X)를 하향각 유지한 채** 보게 한다.

- [ ] **Step 3: 저작 함수 구현**

`build_rear_camera.py` 에 `build(stage)` 를 추가한다. cam_front_link 서브트리를 Sdf 로 복사해
cam_rear_link 를 만들고, 내부 프림 이름의 `front→rear`(대소문자 유지 규칙: `front`/`Front`) 를 치환,
루트 링크 변환을 `Rz(180)·T_front` 로 덮고, cam_qr_down_link 를 제거한 뒤 새 파일로 export 한다.

```python
def build(stage):
    from pxr import Usd, UsdGeom, Sdf, Gf
    root = stage.GetDefaultPrim()
    def find(name):
        for x in stage.Traverse():
            if x.GetName() == name:
                return x
        return None
    front = find("cam_front_link")
    qr = find("cam_qr_down_link")
    if front is None:
        raise RuntimeError("cam_front_link 없음")
    front_path = front.GetPath()
    rear_path = front_path.GetParentPath().AppendChild("cam_rear_link")

    # 출력 레이어에 원본을 flatten 복사(참조 없는 자체포함)
    flat = stage.Flatten()
    flat.Export(str(OUT))
    out = Usd.Stage.Open(str(OUT))
    from pxr import Sdf as _Sdf
    layer = out.GetRootLayer()

    # 1) cam_front_link -> cam_rear_link 스펙 복사
    _Sdf.CopySpec(layer, front_path, layer, rear_path)

    # 2) rear 서브트리에서 프림 이름/토큰 front->rear 치환
    def rename_tokens(prim):
        for child in list(prim.GetChildren()):
            rename_tokens(child)
        nm = prim.GetName()
        if "front" in nm.lower():
            new = nm.replace("front", "rear").replace("Front", "Rear")
            edit = _Sdf.BatchNamespaceEdit()
            edit.Add(prim.GetPath(), prim.GetPath().GetParentPath().AppendChild(new))
            layer.Apply(edit)
    rename_tokens(out.GetPrimAtPath(rear_path))

    # 3) cam_rear_link 로컬 변환 = Rz(180) @ T_front
    rear_link = out.GetPrimAtPath(rear_path)
    Tf = UsdGeom.Xformable(front).GetLocalTransformation(Usd.TimeCode.Default())
    Rz = Gf.Matrix4d().SetRotate(Gf.Rotation(Gf.Vec3d(0, 0, 1), 180.0))
    Tr = Tf * Rz
    xf = UsdGeom.Xformable(rear_link)
    xf.ClearXformOpOrder()
    xf.AddTransformOp().Set(Tr)

    # 4) cam_qr_down_link 제거
    if qr is not None and out.GetPrimAtPath(qr.GetPath()).IsValid():
        out.RemovePrim(qr.GetPath())

    out.GetRootLayer().Save()
    cams = [p for p in out.Traverse() if p.GetTypeName() == "Camera"]
    names = sorted(str(p.GetPath()) for p in cams)
    print(f"REARCAM_BUILT out={OUT.name} camera_count={len(cams)}", flush=True)
    for n in names:
        print(f"  OUT_CAMERA {n}", flush=True)
```

> **주의(구현자):** Sdf 레벨 rename/copy 는 크레이트 실제 구조에 민감하다. Step 1 인스펙트로 얻은
> 실제 경로·프림 이름에 맞춰 위 코드를 조정하라. 핵심 요구는 결과가 **정확히 4개 카메라(cam_rear 포함,
> cam_qr_down 없음), cam_rear 변환 = Rz(180)·T_front** 이며, 검증(Step 4)이 그것을 강제한다.

- [ ] **Step 4: 검증 로직 추가 + 실행**

`main()` 의 build 뒤에 검증을 붙인다(별도 재오픈):
```python
    out = Usd.Stage.Open(str(OUT))
    cams = [p for p in out.Traverse() if p.GetTypeName() == "Camera"]
    paths = [str(p.GetPath()) for p in cams]
    has_rear = any("rear" in p.lower() for p in paths)
    has_front = any("front" in p.lower() for p in paths if "rear" not in p.lower())
    has_qr = any("qr_down" in p.lower() for p in paths) or \
             any(x.GetName() == "cam_qr_down_link" for x in out.Traverse())
    ok = (len(cams) == 4 and has_rear and has_front and not has_qr)
    print(f"REARCAM_VERIFY={'PASS' if ok else 'FAIL'} n={len(cams)} "
          f"rear={has_rear} front={has_front} qr_down={has_qr} paths={paths}", flush=True)
```

Run: `cd /home/rokey/p3/cobot_ws/isaacpjt/hwia_parking_robot_final_caster_package && /home/rokey/dev_ws/isaac_sim/isaacsim/_build/linux-x86_64/release/python.sh build_rear_camera.py 2>&1 | grep -E "REARCAM_BUILT|OUT_CAMERA|REARCAM_VERIFY"`
Expected: `REARCAM_BUILT ... camera_count=4`, 4개 OUT_CAMERA(front/rear/side_left/side_right), `REARCAM_VERIFY=PASS n=4 rear=True front=True qr_down=False`.

- [ ] **Step 5: 좀비 확인 + 커밋**

```bash
cd /home/rokey/p3/cobot_ws
ps -eo pid,args --no-headers | grep -E "/kit/kit|python\.sh|isaacsim" | grep -v grep || echo clean
git add isaacpjt/hwia_parking_robot_final_caster_package/build_rear_camera.py \
        isaacpjt/hwia_parking_robot_final_caster_package/hwia_4cam_mecha_roller_lowered.usd
git commit -m "feat(asset): cam_rear(front 미러) 저작 + cam_qr_down 제거 -> 4-카메라 로봇 에셋"
```

---

### Task 2: 러너 4-카메라 전환 + find_rear_camera + 스모크

**Files:**
- Modify: `isaacpjt/Isaac_envo/parking_v4_runner.py`

**Interfaces:**
- Consumes: Task 1 의 `hwia_4cam_mecha_roller_lowered.usd`.
- Produces: `find_rear_camera(stage, robot_id) -> str`; 스모크 토큰 `V4_CAMERAS_COUNT`.

- [ ] **Step 1: ROBOT_USD 전환 + find_rear_camera 추가**

Modify `parking_v4_runner.py`:
- `ROBOT_USD` 를 `hwia_4cam_mecha_roller_lowered.usd` 로 변경.
- `find_front_camera` 아래에 대칭 함수 추가:
```python
def find_rear_camera(stage, robot_id):
    """로봇 서브트리에서 후방 카메라 prim 경로를 찾는다(이름/경로에 'rear')."""
    from pxr import Usd
    root = stage.GetPrimAtPath(robot_prim_path(robot_id))
    cams = [p for p in Usd.PrimRange(root) if p.GetTypeName() == "Camera"]
    for p in cams:
        if "rear" in str(p.GetPath()).lower():
            return str(p.GetPath())
    raise RuntimeError(f"{robot_id}: 후방 카메라 prim 을 찾지 못했습니다")
```
- `build_stage` 의 로봇 스폰 뒤(또는 V4_STAGE_READY 부근)에 로봇당 카메라 수를 세어 토큰 출력:
```python
    from pxr import Usd
    _r0 = sm.ROBOTS[0]
    _cams = [p for p in Usd.PrimRange(stage.GetPrimAtPath(robot_prim_path(_r0)))
             if p.GetTypeName() == "Camera"]
    _has_qr = any("qr_down" in str(p.GetPath()).lower() for p in _cams)
    print(f"V4_CAMERAS_COUNT robot={_r0} n={len(_cams)} qr_down={_has_qr}", flush=True)
```

- [ ] **Step 2: 문법 검사**

Run: `cd /home/rokey/p3/cobot_ws/isaacpjt/Isaac_envo && python3 -m py_compile parking_v4_runner.py && echo OK`
Expected: `OK`

- [ ] **Step 3: 4-카메라 스모크 + FUSE 회귀**

```bash
cd /home/rokey/p3/cobot_ws/isaacpjt/Isaac_envo
ps -eo pid,args --no-headers | grep -E "/kit/kit|python\.sh|isaacsim" | grep -v grep || echo clean
bash parking_v4_runner.sh --headless-test 2>&1 | grep -E "V4_CAMERAS_COUNT|V4_PHYSICS_TEST|Traceback"
```
Expected: `V4_CAMERAS_COUNT robot=entry_lead n=4 qr_down=False` + `V4_PHYSICS_TEST=PASS`(물리 회귀 없음).
그다음 전방 카메라 회귀(FUSE 가 여전히 전방 카메라를 찾고 동작):
```bash
bash parking_v4_runner.sh --probe=FUSE 2>&1 | grep -E "FUSE_TBASECAM_CAL|FUSE_RESULT|Traceback" | head -3
```
Expected: `FUSE_TBASECAM_CAL best=X180 ...` + `FUSE_RESULT=...`(전방 카메라 정상 — 4-카메라 전환이 전방 검출을 안 깼음).

- [ ] **Step 4: 좀비 확인 + 커밋**

```bash
cd /home/rokey/p3/cobot_ws
ps -eo pid,args --no-headers | grep -E "/kit/kit|python\.sh|isaacsim" | grep -v grep || echo clean
git add isaacpjt/Isaac_envo/parking_v4_runner.py
git commit -m "feat(isaac): 러너 4-카메라 에셋 전환 + find_rear_camera + 카메라 수 스모크"
```

---

### Task 3: 후방 카메라 실동작 검증(뒤 마커 검출) + 문서

후방 카메라가 전방과 대칭으로 마커를 검출함을 2-터미널로 실측하고, Phase A 결과를 문서화한다.

**Files:**
- Modify: `isaacpjt/Isaac_envo/parking_v4_runner.py`
- Modify: `HANDOFF.md`, `DEBUG_LOG.md`

**Interfaces:**
- Consumes: `find_rear_camera`(Task 2), 기존 `attach_camera_graph`, `probe_a_detector_node`(뒤 마커 검출용).
- Produces: `--probe=REAR` 분기(후방 카메라 발행 + 로봇 뒤에 마커 배치). 콘솔 `REAR_PUBLISHING`.

- [ ] **Step 1: --probe=REAR 분기 추가**

`--probe=A` 분기를 참고해, 후방 카메라를 발행하고 로봇을 **마커 앞이 아니라 마커 뒤**(마커가 후방
카메라 시야에 오도록)에 두는 분기를 추가한다. 핵심 차이만:
```python
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
        attach_camera_graph(target, cam_path)   # /robot_entry_lead/image_raw 로 후방 발행
        for _ in range(30):
            app.update()
        ref_id = read_markers(stage)[ref_serves]["id"]
        print(f"REAR_PUBLISHING image=/robot_{target}/image_raw target_marker_id={ref_id} "
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
```

- [ ] **Step 2: 2-터미널 후방 검출 실측**

터미널 A: `bash parking_v4_runner.sh --probe=REAR`
터미널 B: `bash isaacpjt/Isaac_envo/run_probe_a_detector.sh` (기존 검출 노드, 같은 image 토픽).
Expected: 검출 노드가 후방 카메라 영상을 받아 도크 마커를 검출(거리별 인식 O). **후방 카메라가 전방과
대칭으로 마커를 본다**는 실측. 백그라운드+폴링, 좀비 확인.
- 검출 안 되면 후방 카메라 방향/발행을 재점검하고 정직하게 보고(억지 통과 금지).

- [ ] **Step 3: 문법 검사 + 커밋**

```bash
cd /home/rokey/p3/cobot_ws
python3 -m py_compile isaacpjt/Isaac_envo/parking_v4_runner.py && echo OK
git add isaacpjt/Isaac_envo/parking_v4_runner.py
git commit -m "feat(probe): --probe=REAR 후방 카메라 발행 + 뒤 마커 검출 실동작"
```

- [ ] **Step 4: HANDOFF.md / DEBUG_LOG.md 기록**

`HANDOFF.md` 에 Phase A 결과(카메라 4대 구성, cam_rear=front 미러, 검증 통과)를, `DEBUG_LOG.md` 에
막혔던 것(예: Sdf 저작/좀비/발행 이슈)을 간결히 추가하고 커밋.

```bash
cd /home/rokey/p3/cobot_ws
git add HANDOFF.md DEBUG_LOG.md
git commit -m "docs: 미션 Phase A(카메라 4대) 결과·디버깅 기록"
```

---

## Notes for the implementer

- **Isaac 은 느리다**(부팅 ~30–60s, RTF ~0.3). 모든 Isaac 실행은 백그라운드+로그폴링, 좀비 확인.
- Task 1 의 Sdf 저작은 크레이트 실제 구조에 맞춰 조정하라 — Step 1 인스펙트가 먼저다. 요구는 결과(4 카메라, cam_rear 미러, qr_down 없음)이고 REARCAM_VERIFY 가 강제한다.
- 후방 카메라는 RGB ArUco 용이다(측면 뎁스캠이 바퀴감지). depth 미러는 불필요.
- 전방/측면 카메라·물리 회귀 없어야 한다(FUSE/M5 재실행으로 확인).
