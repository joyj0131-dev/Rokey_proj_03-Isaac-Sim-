# ArUco v4 Phase 1 (지도 생성 + M5 정확도 관문) Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** v4 주차장의 마커 16장으로 측위 지도 `marker_map_v4.json` 을 만들고, "마커로 로봇 월드 자세를 2cm/1° 안에 아는가"를 M5 정확도 관문으로 확정한다.

**Architecture:** 지도는 v4 USD(ASCII)를 순수 파이썬으로 파싱해 생성한다(마커 위치는 실제 데칼 좌표 `xformOp:translate` 를 써서 도크 0.7m 오프셋을 자동 흡수). 측위 노드(`marker_localizer_node.py`)는 이미 마커 전용(`fuse=False`) 측위를 지원하므로 코드 변경 없이 v4 지도만 물려 쓴다. M5 는 Isaac 러너의 새 `--probe=M5` 모드로, 로봇을 여러 자세에 세워 렌더→검출→`robot_pose_from_marker` 로 측위한 값을 GT 와 비교해 p95 오차와 공분산을 낸다.

**Tech Stack:** Python 3.10/3.11, pxr USD(런너 내), numpy, OpenCV(aruco), pytest, Isaac Sim 5.1

## Global Constraints

- 대상 에셋: `isaacpjt/Isaac_envo/parking/parking_environment_v4.usd` (ASCII USD, 마커 16장, `aruco:dictionary="DICT_5X5_100"`, `aruco:codeSize=0.19444445`).
- **마커 월드좌표는 실제 데칼 위치(`xformOp:translate` 의 x,z)를 쓴다.** `aruco:position` 은 도크 마커에서 데칼과 z 로 0.7m 어긋난다(스펙 부록 A/C). 데칼 좌표를 쓰면 특수분기 없이 전 마커가 정확해진다.
- 좌표 규약: 로봇 yaw=0 → 월드 +Z (`marker_localizer.py` 규약, `atan2(fwd_x, fwd_z)`). 입차=z양수/출차=z음수는 `site_map_v4.py` 가 전담(다른 코드는 ENTRY/EXIT 만).
- GT 는 계측(채점) 기준으로만 허용, 제어 입력 금지. M5 는 GT 를 정답으로 쓰는 정확도 시험이다.
- M5 관문 통과 기준: **위치 오차 p95 ≤ 0.02 m 그리고 yaw 오차 p95 ≤ 1.0°**.
- 카메라: 로봇 로컬 전방 x≈0.924, **높이 0.15 m(Phase 0 결정)**, 하향 30°. 카메라 정면(-Z)은 월드 +X 를 본다.
- 기존 러너 `dock_lift_handoff_runner_v2.py`/`.sh`, `mecanum_drive.py`, `marker_localizer.py`, `aruco_pose.py`, `site_map_v4.py` 는 **수정하지 않는다**(M5 는 `parking_v4_runner.py` 에만 추가).
- pytest 는 이 머신의 선행 충돌 때문에 `-p no:anyio` 로 실행한다.
- Isaac 실행 전후 좀비 확인: `ps -eo pid,args --no-headers | grep -E "/kit/kit|python\.sh|isaacsim" | grep -v grep || echo clean`. `--gui` 는 사용자 관람용이라 창을 닫지 말고, 헤드리스에서만 `app.close()`.
- 이번 사이클에서 **하지 않는 것**(Phase 3=주행으로 미룸): `/odom` XZ↔XY 프레임 통일, 오도메트리 융합, 폐루프 NavigateToPose 주행.

---

## File Structure

| 파일 | 책임 |
|---|---|
| `isaacpjt/Isaac_envo/build_marker_map_v4.py` | **신규.** v4 USD(ASCII) 파싱 → `marker_map_v4.json` 생성. 순수 파이썬(Isaac 불필요). |
| `src/parkbot_aruco/data/marker_map_v4.json` | **신규(생성물).** 측위 노드가 소유하는 v4 지도. |
| `src/parkbot_aruco/test/test_build_marker_map_v4.py` | **신규.** 파서 단위테스트. |
| `isaacpjt/Isaac_envo/parking_v4_runner.py` | **수정.** `--probe=M5` 정확도 관문 분기 추가. |

지도 생성기를 `isaacpjt/Isaac_envo/` 에 두는 이유: v4 USD 와 같은 트리에 있어 상대경로가 단순하고, 기존 `build_*` 스크립트들과 위치가 일관된다. 출력 json 은 ROS 패키지 `src/parkbot_aruco/data/` 가 소유한다(실배포엔 Isaac 이 없고 측위 노드가 이 파일을 읽는다).

---

### Task 1: `build_marker_map_v4.py` — v4 USD → marker_map_v4.json

v4 USD 를 파싱해 마커 16장을 지도로 굽는다. 순수 파이썬, Isaac 불필요.

**Files:**
- Create: `isaacpjt/Isaac_envo/build_marker_map_v4.py`
- Create (생성물): `src/parkbot_aruco/data/marker_map_v4.json`
- Test: `src/parkbot_aruco/test/test_build_marker_map_v4.py`

**Interfaces:**
- Consumes: `site_map_v4.role_of(serves) -> "ENTRY"|"EXIT"`, `site_map_v4.SERVES_ROLE` (Task 이전에 이미 존재).
- Produces:
  - `parse_v4_markers(usd_text: str) -> list[dict]` — 각 dict `{"id": int, "serves": str, "kind": str, "yaw": float, "x": float, "z": float}` (x,z 는 데칼 `xformOp:translate`).
  - `build_map(usd_text: str) -> dict` — 최상위 `{"dictionary","code_size_m","source","markers":[...]}`, 각 마커 `{"id","serves","kind","x","z","yaw","role"}`.
  - `main()` — v4 USD 를 읽어 `src/parkbot_aruco/data/marker_map_v4.json` 저장, 콘솔에 마커 수 출력.

- [ ] **Step 1: 실패하는 테스트 작성**

Create `src/parkbot_aruco/test/test_build_marker_map_v4.py`:

```python
"""build_marker_map_v4 파서 단위테스트."""
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(REPO / "isaacpjt" / "Isaac_envo"))
sys.path.insert(0, str(REPO / "src" / "parkbot_aruco"))

import build_marker_map_v4 as B    # noqa: E402


# v4 USD 의 도크 마커 한 블록(발췌). aruco:position(z=2.2)과 데칼 translate(z=2.9)이 다르다.
DOCK_BLOCK = '''
        def Mesh "dock_ROBOT_OUT" (
            prepend apiSchemas = ["MaterialBindingAPI"]
        )
        {
            custom float aruco:codeSize = 0.19444445
            custom string aruco:dictionary = "DICT_5X5_100"
            custom string aruco:kind = "dock"
            custom int aruco:markerId = 21
            custom float3 aruco:position = (-3.2, 0, 2.2)
            custom string aruco:serves = "D_OUT_1"
            custom float aruco:yaw = 0
            double3 xformOp:translate = (-3.2, 0.0012, 2.9000000432133675)
            uniform token[] xformOpOrder = ["xformOp:translate"]
        }
        def Mesh "slot_A1" (
        )
        {
            custom string aruco:dictionary = "DICT_5X5_100"
            custom string aruco:kind = "slot"
            custom int aruco:markerId = 0
            custom float3 aruco:position = (2.8, 0, -6.875)
            custom string aruco:serves = "A1"
            custom float aruco:yaw = 0
            double3 xformOp:translate = (2.8, 0.0012, -6.875)
        }
'''


def test_dock_marker_uses_decal_translate_not_aruco_position():
    """도크 마커 x,z 는 데칼 translate(2.9)여야 한다 — aruco:position(2.2)이 아니라."""
    ms = {m["serves"]: m for m in B.parse_v4_markers(DOCK_BLOCK)}
    d = ms["D_OUT_1"]
    assert d["id"] == 21
    assert d["kind"] == "dock"
    assert abs(d["x"] - (-3.2)) < 1e-6
    assert abs(d["z"] - 2.9) < 1e-3        # 데칼 z, 0.7m 오프셋 반영
    assert abs(d["z"] - 2.2) > 0.5         # aruco:position 이 아님을 확실히


def test_slot_marker_parsed():
    ms = {m["serves"]: m for m in B.parse_v4_markers(DOCK_BLOCK)}
    a1 = ms["A1"]
    assert a1["id"] == 0 and a1["kind"] == "slot"
    assert abs(a1["x"] - 2.8) < 1e-6 and abs(a1["z"] - (-6.875)) < 1e-6


def test_build_map_adds_role_and_top_level():
    m = B.build_map(DOCK_BLOCK)
    assert m["dictionary"] == "DICT_5X5_100"
    assert abs(m["code_size_m"] - 0.19444445) < 1e-6
    by = {x["serves"]: x for x in m["markers"]}
    assert by["D_OUT_1"]["role"] == "ENTRY"    # z 양수 = 입차
    assert by["A1"]["role"] == "EXIT"          # z 음수 = 출차


def test_real_v4_usd_has_16_markers():
    usd = (REPO / "isaacpjt" / "Isaac_envo" / "parking"
           / "parking_environment_v4.usd").read_text(encoding="utf-8")
    markers = B.parse_v4_markers(usd)
    assert len(markers) == 16
    ids = sorted(m["id"] for m in markers)
    assert len(set(ids)) == 16                 # id 중복 없음
```

- [ ] **Step 2: 테스트 실패 확인**

Run: `cd /home/rokey/p3/cobot_ws/src/parkbot_aruco && python3 -m pytest test/test_build_marker_map_v4.py -v -p no:anyio`
Expected: FAIL — `ModuleNotFoundError: No module named 'build_marker_map_v4'`

- [ ] **Step 3: 구현 작성**

Create `isaacpjt/Isaac_envo/build_marker_map_v4.py`:

```python
#!/usr/bin/env python3
"""v4 주차장 USD(ASCII) → marker_map_v4.json 생성 (순수 파이썬, Isaac 불필요).

마커 위치는 **실제 데칼 메시의 xformOp:translate(x,z)** 를 쓴다. aruco:position 속성은
도크 마커에서 데칼과 z 로 0.7m 어긋나 있어(스펙 부록 A/C), 카메라가 실제로 보는 데칼
좌표를 써야 측위가 맞는다. 데칼 좌표를 쓰면 도크/슬롯 구분 없이 전 마커가 정확하다.

손으로 좌표를 옮겨 적지 않는다 — 에셋이 바뀌면 재실행 한 번으로 지도가 갱신된다.
"""
import json
import re
import sys
from pathlib import Path

WORK_DIR = Path(__file__).resolve().parent
REPO_ROOT = WORK_DIR.parent.parent
V4_USD = WORK_DIR / "parking" / "parking_environment_v4.usd"
OUT_JSON = REPO_ROOT / "src" / "parkbot_aruco" / "data" / "marker_map_v4.json"

sys.path.insert(0, str(REPO_ROOT / "src" / "parkbot_aruco"))
from parkbot_aruco import site_map_v4 as sm   # noqa: E402

_NUM = r"[-+]?\d*\.?\d+(?:[eE][-+]?\d+)?"


def parse_v4_markers(usd_text):
    """ASCII USD 에서 aruco 마커 Mesh 를 추출한다.

    마커는 leaf `def Mesh` 프림이라 그 안에 다른 def 가 없다. 'def Mesh "' 로 쪼갠 각
    청크에서 첫 aruco:markerId / serves / kind / yaw 와 첫 xformOp:translate 를 읽는다.
    translate 는 마커 자신의 데칼 위치다.
    """
    out = []
    for chunk in usd_text.split('def Mesh "')[1:]:
        mid = re.search(r"aruco:markerId\s*=\s*(\d+)", chunk)
        if not mid:
            continue
        serves = re.search(r'aruco:serves\s*=\s*"([^"]+)"', chunk)
        kind = re.search(r'aruco:kind\s*=\s*"([^"]+)"', chunk)
        yaw = re.search(rf"aruco:yaw\s*=\s*({_NUM})", chunk)
        tr = re.search(
            rf"xformOp:translate\s*=\s*\(\s*({_NUM})\s*,\s*({_NUM})\s*,\s*({_NUM})\s*\)",
            chunk)
        if not (serves and tr):
            continue
        out.append({
            "id": int(mid.group(1)),
            "serves": serves.group(1),
            "kind": kind.group(1) if kind else "",
            "yaw": float(yaw.group(1)) if yaw else 0.0,
            "x": float(tr.group(1)),      # 데칼 x
            "z": float(tr.group(3)),      # 데칼 z (translate[2])
        })
    return out


def build_map(usd_text):
    """파싱한 마커에 role(ENTRY/EXIT)을 붙이고 지도 dict 를 만든다."""
    markers = parse_v4_markers(usd_text)
    code_size = 0.19444445
    dict_name = "DICT_5X5_100"
    m0 = re.search(r'aruco:codeSize\s*=\s*(' + _NUM + ')', usd_text)
    if m0:
        code_size = float(m0.group(1))
    d0 = re.search(r'aruco:dictionary\s*=\s*"([^"]+)"', usd_text)
    if d0:
        dict_name = d0.group(1)
    out_markers = []
    for m in markers:
        out_markers.append({
            "id": m["id"], "serves": m["serves"], "kind": m["kind"],
            "x": m["x"], "z": m["z"], "yaw": m["yaw"],
            "role": sm.role_of(m["serves"]),
        })
    out_markers.sort(key=lambda x: x["id"])
    return {
        "dictionary": dict_name,
        "code_size_m": code_size,
        "source": "parking_environment_v4.usd (decal xformOp:translate)",
        "note": ("마커 x,z 는 데칼 실좌표(translate). 도크 마커는 aruco:position 과 "
                 "z 로 0.7m 다르므로 데칼을 써야 측위가 맞는다."),
        "markers": out_markers,
    }


def main():
    usd_text = V4_USD.read_text(encoding="utf-8")
    data = build_map(usd_text)
    OUT_JSON.parent.mkdir(parents=True, exist_ok=True)
    OUT_JSON.write_text(json.dumps(data, indent=2, ensure_ascii=False), encoding="utf-8")
    print(f"marker_map_v4.json 저장: {OUT_JSON}  마커 {len(data['markers'])}장 "
          f"(dict={data['dictionary']} code_size={data['code_size_m']})")


if __name__ == "__main__":
    main()
```

- [ ] **Step 4: 테스트 통과 확인**

Run: `cd /home/rokey/p3/cobot_ws/src/parkbot_aruco && python3 -m pytest test/test_build_marker_map_v4.py -v -p no:anyio`
Expected: PASS — 4 passed

- [ ] **Step 5: 실제 지도 생성 + 검증**

Run: `cd /home/rokey/p3/cobot_ws/isaacpjt/Isaac_envo && python3 build_marker_map_v4.py`
Expected: `marker_map_v4.json 저장: ... 마커 16장 (dict=DICT_5X5_100 code_size=0.19444445)`

Run(도크 오프셋 반영 확인):
```bash
python3 -c "import json; d=json.load(open('/home/rokey/p3/cobot_ws/src/parkbot_aruco/data/marker_map_v4.json')); \
m={x['serves']:x for x in d['markers']}; print('D_OUT_1 z =', m['D_OUT_1']['z'], '(2.9 이어야 함)'); \
print('A1 z =', m['A1']['z'], '(-6.875)'); print('roles:', m['D_OUT_1']['role'], m['A1']['role'])"
```
Expected: `D_OUT_1 z = 2.9...`, `A1 z = -6.875`, `roles: ENTRY EXIT`

- [ ] **Step 6: 커밋**

```bash
cd /home/rokey/p3/cobot_ws
git add isaacpjt/Isaac_envo/build_marker_map_v4.py \
        src/parkbot_aruco/data/marker_map_v4.json \
        src/parkbot_aruco/test/test_build_marker_map_v4.py
git commit -m "feat(aruco): v4 마커 지도 생성기 — 데칼 실좌표로 도크 0.7m 오프셋 흡수"
```

---

### Task 2: `--probe=M5` — 마커 측위 정확도 관문

로봇을 여러 자세에 세워 렌더→검출→측위한 값을 GT 와 비교해 p95 오차·공분산을 낸다.
**T_base_cam(카메라 마운트)은 GT 로 자동 보정**한다 — 카메라를 0.15m 로 올렸으므로 v1 기본값을 그대로 쓰면 틀린다.

**Files:**
- Modify: `isaacpjt/Isaac_envo/parking_v4_runner.py` (probe 분기 그룹에 `M5` 추가)

**Interfaces:**
- Consumes:
  - Task 1 결과 `src/parkbot_aruco/data/marker_map_v4.json`
  - `marker_localizer`: `MarkerMap.from_json(data, align_yaw_deg=0.0)`, `rvec_tvec_to_T(rvec, tvec) -> (4,4)`, `robot_pose_from_marker(marker_id, T_cam_marker, T_base_cam, marker_map) -> RobotFix|None` (RobotFix `.x,.z,.yaw_deg`)
  - `aruco_pose.make_detector(dict_name)`, `aruco_pose.detect_and_estimate(gray, detector, code_size_m, K, dist) -> [MarkerPose(.marker_id,.rvec,.tvec,.reproj_err_px)]`
  - 러너 기존: `read_markers`, `marker_visual_center`, `find_front_camera`, `gt_pose_xz_yaw`, `robot_prim_path`, `ROBOT_SPAWN_Y`
- Produces:
  - 콘솔 토큰 `M5_TBASECAM_CAL`(보정 결과), `M5_RESULT`(p95 판정), 리포트 `probe_reports/m5_accuracy.json`

- [ ] **Step 1: M5 분기 추가**

Modify `parking_v4_runner.py` — 다른 probe 분기(`if probe == "A":` 등)와 같은 그룹, `if probe == "M5":` 를 추가한다. 아래는 완전한 분기다.

```python
    if probe == "M5":
        # 마커 측위 정확도 관문. Isaac 안에서 렌더→검출→robot_pose_from_marker 로
        # 월드 자세를 복원해 GT 와 비교한다. T_base_cam(카메라 마운트)은 카메라를
        # 0.15m 로 올렸으므로 v1 기본값을 쓰면 틀린다 → GT 로 자동 보정한다.
        import json
        import cv2
        from pxr import Gf, UsdGeom
        sys.path.insert(0, str(REPO_ROOT / "src" / "parkbot_aruco"))
        from parkbot_aruco import aruco_pose
        from parkbot_aruco import marker_localizer as ML

        cam_h = 0.15
        for a in sys.argv[1:]:
            if a.startswith("--cam-height="):
                cam_h = float(a.split("=", 1)[1])

        target = "entry_lead"
        art = arts[target]
        ref_serves = sm.ROBOT_DOCK_MARKER[target]           # "D_OUT_1"
        markers = read_markers(stage)
        ref_id = markers[ref_serves]["id"]                  # 21
        mx, mz = marker_visual_center(stage, ref_serves)    # 데칼 실좌표

        # 지도 로드(측위가 쓰는 v4 지도)
        map_path = REPO_ROOT / "src" / "parkbot_aruco" / "data" / "marker_map_v4.json"
        mm_json = json.loads(map_path.read_text(encoding="utf-8"))
        marker_map = ML.MarkerMap.from_json(mm_json, align_yaw_deg=0.0)
        code_size_m = float(mm_json["code_size_m"])
        detector = aruco_pose.make_detector(mm_json["dictionary"])

        # 카메라 준비 + 높이 오버라이드(probe A 와 동일: 월드 Y dy 를 로컬로 역변환)
        cam_path = find_front_camera(stage, target)
        cam_prim = stage.GetPrimAtPath(cam_path)
        cam_xf = UsdGeom.Xformable(cam_prim)
        _, spawn_orn = art.get_world_poses()
        spawn_orn = np.asarray(spawn_orn).reshape(-1)[:4].copy()
        dy_world = cam_h - 0.09
        if abs(dy_world) > 1e-9:
            mb = cam_xf.ComputeLocalToWorldTransform(timeline.get_current_time())
            local_delta = mb.GetInverse().TransformDir(Gf.Vec3d(0.0, dy_world, 0.0))
            cam_xf.AddTranslateOp(UsdGeom.XformOp.PrecisionDouble, "camHeightM5").Set(local_delta)
            for _ in range(3):
                app.update()

        import omni.replicator.core as rep
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
        dist = np.zeros((5, 1), dtype=np.float64)

        def usd_to_np(gf_m):
            """Gf.Matrix4d(행벡터 v*M 규약) → 표준 열벡터 4x4(M*v)."""
            m = np.array([[gf_m[i][j] for j in range(4)] for i in range(4)], dtype=np.float64)
            return m.T

        def place_and_capture(d, lat, yaw_deg):
            """로봇을 마커 앞 (d,lat,yaw) 자세에 놓고 렌더 이미지+검출을 돌려준다.

            카메라 정면(-Z)이 월드 +X 를 보므로 마커 앞 d 는 x 축(로봇 중심 mx-d),
            횡오프셋 lat 은 z 축. yaw 는 스폰 자세에 RotateY 를 곱해 준다.
            """
            base = np.array([[mx - d, ROBOT_SPAWN_Y, mz + lat]])
            if abs(yaw_deg) < 1e-9:
                orn = np.array([spawn_orn])
            else:
                half = math.radians(yaw_deg) * 0.5
                qy = np.array([math.cos(half), 0.0, math.sin(half), 0.0])  # (w,x,y,z) about Y
                w0, x0, y0, z0 = spawn_orn
                w1, x1, y1, z1 = qy
                orn = np.array([[
                    w1*w0 - x1*x0 - y1*y0 - z1*z0,
                    w1*x0 + x1*w0 + y1*z0 - z1*y0,
                    w1*y0 - x1*z0 + y1*w0 + z1*x0,
                    w1*z0 + x1*y0 - y1*x0 + z1*w0]])
            art.set_world_poses(base, orn)
            for _ in range(20):
                app.update()
            frame = rgb_annot.get_data()
            img = (np.asarray(frame)[:, :, :3]
                   if frame is not None and len(frame) else None)
            det = []
            if img is not None:
                gray = cv2.cvtColor(np.ascontiguousarray(img), cv2.COLOR_RGB2GRAY)
                det = aruco_pose.detect_and_estimate(gray, detector, code_size_m, K, dist)
            hit = [p for p in det if int(p.marker_id) == ref_id]
            return (hit[0] if hit else None)

        def localize(pose, T_base_cam):
            T_cm = ML.rvec_tvec_to_T(pose.rvec, pose.tvec)
            return ML.robot_pose_from_marker(ref_id, T_cm, T_base_cam, marker_map)

        # ---- T_base_cam 자동 보정 ----
        # T_base_cam = inv(T_world_base) @ T_world_camusd @ C, C 는 USD 카메라→OpenCV 광학
        # 규약 회전. 규약을 손으로 추론하면 틀리기 쉬우므로(프로젝트가 align_yaw 를 스윕해
        # 정한 전례), 후보 C 를 GT 로 스윕해 오차 최소를 고른다.
        pose0 = place_and_capture(1.5, 0.0, 0.0)   # 검증용 기준 자세
        if pose0 is None:
            print("M5_TBASECAM_CAL FAIL: 검증 자세에서 마커 미검출 — 기하 재검토", flush=True)
            if headless:
                app.close()
            raise RuntimeError("M5 T_base_cam 보정 자세에서 마커 미검출")
        # base_link 월드행렬을 Articulation world pose 로 구성
        bpos, born = art.get_world_poses()
        bp = np.asarray(bpos).reshape(-1)[:3]
        bw, bx, by, bz = (float(v) for v in np.asarray(born).reshape(-1)[:4])
        # quat(w,x,y,z) → 회전행렬
        def quat_to_R(w, x, y, z):
            return np.array([
                [1-2*(y*y+z*z), 2*(x*y-w*z),   2*(x*z+w*y)],
                [2*(x*y+w*z),   1-2*(x*x+z*z), 2*(y*z-w*x)],
                [2*(x*z-w*y),   2*(y*z+w*x),   1-2*(x*x+y*y)]], dtype=np.float64)
        T_world_base = np.eye(4)
        T_world_base[:3, :3] = quat_to_R(bw, bx, by, bz)
        T_world_base[:3, 3] = bp
        T_world_camusd = usd_to_np(cam_xf.ComputeLocalToWorldTransform(timeline.get_current_time()))

        candidates = {
            "I":     np.diag([1.0, 1.0, 1.0, 1.0]),
            "X180":  np.diag([1.0, -1.0, -1.0, 1.0]),
            "Y180":  np.diag([-1.0, 1.0, -1.0, 1.0]),
            "Z180":  np.diag([-1.0, -1.0, 1.0, 1.0]),
        }
        gx, gz, gyaw = gt_pose_xz_yaw(art)
        best_name, best_T, best_err = None, None, 1e9
        for name, C in candidates.items():
            T_base_cam = np.linalg.inv(T_world_base) @ T_world_camusd @ C
            fix = localize(pose0, T_base_cam)
            if fix is None:
                continue
            e = math.hypot(fix.x - gx, fix.z - gz)
            if e < best_err:
                best_name, best_T, best_err = name, T_base_cam, e
        print(f"M5_TBASECAM_CAL best={best_name} verify_pos_err={best_err:.4f}m", flush=True)
        if best_T is None or best_err > 0.05:
            print("M5_TBASECAM_CAL FAIL: 어떤 광학 규약도 5cm 안에 못 맞춤 — "
                  "usd_to_np 전치/규약 재검토 필요", flush=True)
            if headless:
                app.close()
            raise RuntimeError(f"M5 T_base_cam 보정 실패(best_err={best_err:.4f})")
        T_base_cam = best_T

        # ---- 정확도 스윕 ----
        results = []
        for d in (1.3, 1.5, 1.7, 1.9):
            for lat in (-0.15, 0.0, 0.15):
                for yaw_deg in (-8.0, 0.0, 8.0):
                    pose = place_and_capture(d, lat, yaw_deg)
                    if pose is None:
                        results.append({"d": d, "lat": lat, "yaw": yaw_deg,
                                        "detected": False})
                        continue
                    fix = localize(pose, T_base_cam)
                    gxx, gzz, gyy = gt_pose_xz_yaw(art)
                    ex, ez = fix.x - gxx, fix.z - gzz
                    dyaw = (fix.yaw_deg - math.degrees(gyy) + 180.0) % 360.0 - 180.0
                    results.append({
                        "d": d, "lat": lat, "yaw": yaw_deg, "detected": True,
                        "pos_err_m": math.hypot(ex, ez), "yaw_err_deg": abs(dyaw),
                        "ex": ex, "ez": ez, "eyaw": dyaw,
                        "reproj_px": float(pose.reproj_err_px)})

        det = [r for r in results if r["detected"]]
        if not det:
            print("M5_RESULT FAIL: 검출 표본 0개", flush=True)
            if headless:
                app.close()
            raise RuntimeError("M5 검출 표본 0")
        pos_errs = sorted(r["pos_err_m"] for r in det)
        yaw_errs = sorted(r["yaw_err_deg"] for r in det)

        def p95(a):
            return a[min(len(a) - 1, int(math.ceil(0.95 * len(a)) - 1))]
        cov = np.cov(np.array([[r["ex"], r["ez"], r["eyaw"]] for r in det]).T).tolist()
        pos_p95, yaw_p95 = p95(pos_errs), p95(yaw_errs)
        ok = (pos_p95 <= 0.02) and (yaw_p95 <= 1.0)
        report = {
            "camera_height_m": cam_h, "tbasecam_convention": best_name,
            "tbasecam_verify_err_m": best_err,
            "n_samples": len(det), "n_total": len(results),
            "pos_err_m": {"mean": sum(pos_errs) / len(pos_errs),
                          "median": pos_errs[len(pos_errs) // 2],
                          "p95": pos_p95, "max": pos_errs[-1]},
            "yaw_err_deg": {"mean": sum(yaw_errs) / len(yaw_errs),
                            "median": yaw_errs[len(yaw_errs) // 2],
                            "p95": yaw_p95, "max": yaw_errs[-1]},
            "cov_ex_ez_eyaw": cov, "samples": results,
        }
        import v4_probes as vp
        path = vp.write_report("m5_accuracy", report)
        print(f"M5_RESULT={'PASS' if ok else 'FAIL'} "
              f"pos_p95={pos_p95*100:.2f}cm yaw_p95={yaw_p95:.2f}deg "
              f"n={len(det)}/{len(results)} conv={best_name} report={path.name}", flush=True)
        if headless:
            app.close()
            return
        while app.is_running():
            app.update()
        app.close()
        return
```

- [ ] **Step 2: 문법 검사**

Run: `cd /home/rokey/p3/cobot_ws/isaacpjt/Isaac_envo && python3 -m py_compile parking_v4_runner.py && echo OK`
Expected: `OK`

- [ ] **Step 3: 좀비 확인 후 M5 헤드리스 실행**

```bash
cd /home/rokey/p3/cobot_ws/isaacpjt/Isaac_envo
ps -eo pid,args --no-headers | grep -E "/kit/kit|python\.sh|isaacsim" | grep -v grep || echo clean
bash parking_v4_runner.sh --probe=M5 2>&1 | grep -E "M5_TBASECAM_CAL|M5_RESULT|Traceback|RuntimeError"
```
Expected(대략): `M5_TBASECAM_CAL best=<규약> verify_pos_err=<0.05 미만>`, 이어서 `M5_RESULT=PASS pos_p95=<2cm 이하> yaw_p95=<1deg 이하> ...`

**해석 규칙(구현자):**
- `M5_TBASECAM_CAL FAIL` 이면 `usd_to_np` 의 전치나 후보 규약 문제다. `T_world_base`/`T_world_camusd` 를 콘솔에 찍어 손검산하고, 필요하면 후보 C 에 90° 회전(예: X±90, Y±90)을 추가한다. 보정이 5cm 안에 들 때까지는 스윕 수치가 의미 없다.
- `M5_RESULT=FAIL` 인데 보정은 통과(<1cm)했다면, 그것이 진짜 측위 한계다(평면 마커 깊이 관측성). 그 경우 수치를 그대로 보고하고 사용자와 상의한다 — 억지로 PASS 로 만들지 않는다.

- [ ] **Step 4: 좀비 정리 확인**

Run: `ps -eo pid,args --no-headers | grep -E "/kit/kit|python\.sh|isaacsim" | grep -v grep || echo clean`
Expected: `clean`

- [ ] **Step 5: 커밋**

```bash
cd /home/rokey/p3/cobot_ws
git add isaacpjt/Isaac_envo/parking_v4_runner.py
git commit -m "feat(probe): M5 정확도 관문 — 마커 측위 vs GT, T_base_cam GT 자동보정"
```

---

### Task 3: 배포 경로 측위 확인 (2-터미널, marker_localizer_node + v4 지도)

M5 는 Isaac 안(cv2 4.11)에서 잰다. 실배포는 외부 노드(시스템 cv2 4.5.4)가 측위한다. 그 경로가
v4 지도로 실제 로봇 월드 자세를 내는지 2-터미널로 확인한다(측위 노드는 이미 존재, 코드 변경 없음).

**Files:**
- Create: `isaacpjt/Isaac_envo/run_marker_localizer_v4.sh` (터미널 B 런처)

**Interfaces:**
- Consumes: 기존 `parkbot_aruco/marker_localizer_node.py` (파라미터 `image_topic`, `camera_info_topic`, `marker_map`, `t_base_cam`, `fuse`, `frame`), Task 1 의 `marker_map_v4.json`, 러너 `--probe=A`(카메라 발행) 또는 `--probe=M5`.
- Produces: `run_marker_localizer_v4.sh` — 도메인 126 + 화이트리스트 + 시스템 ROS 세팅 후 `marker_localizer_node` 를 v4 지도로 실행.

- [ ] **Step 1: 런처 작성**

Create `isaacpjt/Isaac_envo/run_marker_localizer_v4.sh`:

```bash
#!/bin/bash
# 배포 경로 측위(터미널 B): marker_localizer_node 를 v4 지도로 실행.
# /opt/ros setup.bash 는 미설정 변수를 참조하므로 소싱 중에만 set +u.
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

# entry_lead 카메라 토픽 + v4 지도. fuse=False(마커 전용). frame=usd(x,z,yaw 로그).
exec python3 -m parkbot_aruco.marker_localizer_node --ros-args \
  -p image_topic:=/robot_entry_lead/image_raw \
  -p camera_info_topic:=/robot_entry_lead/camera_info \
  -p marker_map:="$MAP" \
  -p fuse:=false -p frame:=usd "$@"
```

- [ ] **Step 2: 실행 권한 + 문법 확인**

```bash
cd /home/rokey/p3/cobot_ws
chmod +x isaacpjt/Isaac_envo/run_marker_localizer_v4.sh
bash -n isaacpjt/Isaac_envo/run_marker_localizer_v4.sh && echo "sh OK"
```
Expected: `sh OK`

- [ ] **Step 3: 2-터미널 실동작 확인**

터미널 A(Isaac, 카메라 발행 + 스윕):
```bash
cd /home/rokey/p3/cobot_ws/isaacpjt/Isaac_envo
bash parking_v4_runner.sh --probe=A --cam-height=0.15 --hold-sec=0.5 --probe-a-loops=30
```
터미널 B(측위 노드):
```bash
cd /home/rokey/p3/cobot_ws
bash isaacpjt/Isaac_envo/run_marker_localizer_v4.sh
```
Expected: 터미널 B 에 `robot_pose_from_marker` 결과(로봇 x,z,yaw)가 마커 검출마다 로그된다.
`marker_localizer_node` 는 `t_base_cam` 기본값(v1 0.09m 마운트)을 쓰므로 **절대 정확도는 M5 만큼
안 맞을 수 있다** — 이 Task 는 "배포 경로가 v4 지도로 측위값을 낸다"는 파이프라인 확인이다.
정확도는 M5(Task 2)가 판정한다.

**주의(구현자):** 두 프로세스는 백그라운드+로그폴링으로 돌린다. Isaac 부팅+디스커버리 ~15초.
좀비 확인/정리 필수. 실제로 측위 로그가 나오는 것을 관찰해 보고한다(못 보면 정직하게 보고).

- [ ] **Step 4: 커밋**

```bash
cd /home/rokey/p3/cobot_ws
git add isaacpjt/Isaac_envo/run_marker_localizer_v4.sh
git commit -m "feat(aruco): 배포 경로 측위 런처 — marker_localizer_node + v4 지도 (2-터미널)"
```

---

## Notes for the implementer

- **Isaac 은 느리다**(RTF ~0.3). M5 는 자동보정 5회 + 36포즈 = 40여 회 렌더라 수 분 걸린다. 백그라운드+로그폴링.
- **좀비 프로세스**를 매 실행 전후로 확인·정리한다. `--gui` 실행은 창을 닫지 말고 유지, 헤드리스만 `app.close()`.
- **T_base_cam 이 M5 의 핵심 리스크다.** 자동보정이 5cm 안에 못 들면 스윕 수치는 의미 없다 — 보정부터 통과시킨다(usd_to_np 전치, 광학 규약 후보). GT 로 검증하는 구조라 눈감고 맞추지 않는다.
- 측위가 GT 를 "정답"으로 쓰는 것은 정확도 채점 목적이며, 로봇 제어에 GT 를 넣는 것이 아니다(규약 준수).

---

### Task 4: M5 다중프레임 융합 (p95 꼬리 감소)

M5 단일프레임이 p95 4.15cm/1.68°(목표 2cm/1°)로 꼬리가 목표를 넘겼다(중앙값은 1.4cm/0.42°).
한 자세에서 **K 프레임을 모아 측위 추정을 평균**해 프레임별 랜덤 노이즈를 줄인다. 효과가 실제로
나오는지(정지 렌더가 결정론적이면 안 날 수 있음)를 **단일 vs 융합 p95 를 나란히 보고**해 정직하게 판정한다.

**Files:**
- Modify: `isaacpjt/Isaac_envo/parking_v4_runner.py` (`--probe=M5` 분기)

**Interfaces:**
- Consumes: 기존 M5 분기의 `place_and_capture(d, lat, yaw_deg) -> MarkerPose|None`, `localize(pose, T_base_cam) -> RobotFix`, `gt_pose_xz_yaw(art)`, 자동보정된 `T_base_cam`.
- Produces:
  - 러너 인자 `--m5-frames=K` (기본 1 = 기존 동작). K>1 이면 자세마다 K 프레임 융합.
  - 콘솔 토큰 `M5_RESULT`(융합 p95 로 판정) + 추가 필드 `single_pos_p95`, `single_yaw_p95`(비교용).
  - 리포트 `m5_accuracy.json` 에 `frames`, `single`(단일프레임 통계), `fused`(융합 통계) 추가.

- [ ] **Step 1: place_and_capture 를 K회 반복 캡처로 확장**

Modify `parking_v4_runner.py` — M5 분기에서 `--m5-frames` 를 파싱하고, 자세마다 K 프레임을
캡처해 각각 측위한 뒤 (x,z 산술평균, yaw 원형평균) 융합한다. 기존 스윕 루프를 아래로 교체한다.

`--cam-height` 파싱 부근에 추가:
```python
        n_frames = 1
        for a in sys.argv[1:]:
            if a.startswith("--m5-frames="):
                n_frames = max(1, int(a.split("=", 1)[1]))
```

`place_and_capture` **정의 바로 뒤**에 K프레임 캡처+융합 헬퍼를 추가:
```python
        def capture_frames(d, lat, yaw_deg, k):
            """같은 자세에서 k 프레임을 잡아 각 프레임의 측위를 리스트로 돌려준다.

            프레임 사이에 app.update() 를 돌려 렌더가 갱신되게 한다. 정지 자세라
            렌더가 결정론적이면 프레임들이 거의 같아 융합 효과가 없다 — 그 경우
            single 과 fused 가 비슷하게 나오며, 그것이 정직한 결과다.
            """
            poses = []
            first = place_and_capture(d, lat, yaw_deg)   # 첫 프레임(자세 세팅 포함)
            gxx, gzz, gyy = gt_pose_xz_yaw(art)           # 이 자세의 GT
            if first is not None:
                poses.append(first)
            for _ in range(k - 1):
                for _ in range(2):
                    app.update()
                frame = rgb_annot.get_data()
                img = (np.asarray(frame)[:, :, :3]
                       if frame is not None and len(frame) else None)
                if img is None:
                    continue
                gray = cv2.cvtColor(np.ascontiguousarray(img), cv2.COLOR_RGB2GRAY)
                det = aruco_pose.detect_and_estimate(gray, detector, code_size_m, K, dist)
                hit = [p for p in det if int(p.marker_id) == ref_id]
                if hit:
                    poses.append(hit[0])
            return poses, (gxx, gzz, gyy)

        def fuse_localize(poses, T_base_cam):
            """여러 프레임의 측위(RobotFix)를 융합: x,z 평균 + yaw 원형평균."""
            fixes = [localize(p, T_base_cam) for p in poses]
            fixes = [f for f in fixes if f is not None]
            if not fixes:
                return None
            xs = sum(f.x for f in fixes) / len(fixes)
            zs = sum(f.z for f in fixes) / len(fixes)
            sy = sum(math.sin(math.radians(f.yaw_deg)) for f in fixes)
            cy = sum(math.cos(math.radians(f.yaw_deg)) for f in fixes)
            yaw = math.degrees(math.atan2(sy, cy))
            return xs, zs, yaw, fixes[0]      # 융합값 + 첫 프레임(단일 비교용)
```

- [ ] **Step 2: 스윕 루프를 단일+융합 동시 측정으로 교체**

M5 분기의 정확도 스윕 루프(`results = []` 부터 `M5_RESULT` 출력 직전까지)를 아래로 교체:
```python
        def err_of(fx, fz, fyaw_deg, gt):
            gxx, gzz, gyy = gt
            ex, ez = fx - gxx, fz - gzz
            dyaw = (fyaw_deg - math.degrees(gyy) + 180.0) % 360.0 - 180.0
            return math.hypot(ex, ez), abs(dyaw), ex, ez, dyaw

        single_pos, single_yaw = [], []
        fused_pos, fused_yaw, fused_vec = [], [], []
        n_total = 0
        for d in (1.3, 1.5, 1.7, 1.9):
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
        s_pos, s_yaw = p95(single_pos), p95(single_yaw)
        f_pos, f_yaw = p95(fused_pos), p95(fused_yaw)
        ok = (f_pos <= 0.02) and (f_yaw <= 1.0)
        report = {
            "camera_height_m": cam_h, "frames": n_frames,
            "tbasecam_convention": best_name, "tbasecam_verify_err_m": best_err,
            "n_samples": len(fused_pos), "n_total": n_total,
            "single": {"pos_err_m": stats(single_pos), "yaw_err_deg": stats(single_yaw)},
            "fused": {"pos_err_m": stats(fused_pos), "yaw_err_deg": stats(fused_yaw)},
            "cov_ex_ez_eyaw": cov,
        }
        import v4_probes as vp
        path = vp.write_report("m5_accuracy", report)
        print(f"M5_RESULT={'PASS' if ok else 'FAIL'} frames={n_frames} "
              f"fused_pos_p95={f_pos*100:.2f}cm fused_yaw_p95={f_yaw:.2f}deg "
              f"single_pos_p95={s_pos*100:.2f}cm single_yaw_p95={s_yaw:.2f}deg "
              f"n={len(fused_pos)}/{n_total} report={path.name}", flush=True)
        if headless:
            app.close()
            return
        while app.is_running():
            app.update()
        app.close()
        return
```

- [ ] **Step 3: 문법 검사**

Run: `cd /home/rokey/p3/cobot_ws/isaacpjt/Isaac_envo && python3 -m py_compile parking_v4_runner.py && echo OK`
Expected: `OK`

- [ ] **Step 4: 단일(K=1) 회귀 + 융합(K=10) 실행 비교**

```bash
cd /home/rokey/p3/cobot_ws/isaacpjt/Isaac_envo
ps -eo pid,args --no-headers | grep -E "/kit/kit|python\.sh|isaacsim" | grep -v grep || echo clean
bash parking_v4_runner.sh --probe=M5 --m5-frames=1  2>&1 | grep -E "M5_TBASECAM_CAL|M5_RESULT|Traceback"
bash parking_v4_runner.sh --probe=M5 --m5-frames=10 2>&1 | grep -E "M5_TBASECAM_CAL|M5_RESULT|Traceback"
```
Expected: K=1 은 `single≈fused`(융합=단일). K=10 은 `fused_pos_p95`/`fused_yaw_p95` 가 single 대비
줄었는지 관찰.
- **fused_p95 ≤ 2cm 및 ≤ 1° 이면 PASS** — 다중프레임이 꼬리를 깎은 것.
- **fused ≈ single(개선 없음)이면**: 정지 렌더가 결정론적이라 정적 융합이 안 듣는 것이다.
  이 사실을 그대로 보고한다(억지로 PASS 만들지 않는다). 진짜 해법은 주행 중 오도메트리 융합
  (Phase 3)이며, 그 판단은 사용자와 한다.

- [ ] **Step 5: 좀비 정리 확인 + 커밋**

```bash
cd /home/rokey/p3/cobot_ws
ps -eo pid,args --no-headers | grep -E "/kit/kit|python\.sh|isaacsim" | grep -v grep || echo clean
git add isaacpjt/Isaac_envo/parking_v4_runner.py
git commit -m "feat(probe): M5 다중프레임 융합 — K프레임 평균으로 p95 꼬리 감소 측정"
```

---

### Task 5: M5 신뢰 운영구간(거리 기반) 판정

다중프레임(Task 4)은 p95 꼬리(먼 거리·비스듬의 고정 편향)를 못 깎았다. 꼬리는 먼 거리에서
발생하고 가까운 구간은 이미 좋다(1.5m 1.1cm). **로봇이 실제로 지킬 수 있는 거리 기반 신뢰
운영구간**(예: 마커까지 [1.4, 1.8] m 일 때만 fix 를 신뢰)을 정하고, 그 구간에서 p95 를 판정한다.

**정직성 규칙**: 통과하는 자세만 임의로 고르는 것이 아니다. **거리(로봇이 알고 행동에 쓸 수 있는
양)로만** 구간을 정하고 그 구간 안 모든 lat·yaw 를 포함한다. yaw/lat 로 골라내지 않는다.
전체 스윕 p95 와 구간 p95 를 **둘 다** 리포트해 꼬리가 어디서 오는지 투명하게 남긴다.

**Files:**
- Modify: `isaacpjt/Isaac_envo/parking_v4_runner.py` (`--probe=M5` 분기)

**Interfaces:**
- Consumes: Task 4 의 M5 스윕 구조(`capture_frames`/`fuse_localize`/`err_of`/`stats`/`p95`), 자동보정 `T_base_cam`.
- Produces:
  - 러너 인자 `--m5-dmin=`(기본 1.4) `--m5-dmax=`(기본 1.8) — 신뢰 거리구간.
  - 콘솔 `M5_RESULT` 를 **구간 p95 로 판정**하고 `window_pos_p95`/`full_pos_p95` 둘 다 출력.
  - `m5_accuracy.json` 에 `trusted_window`(dmin/dmax), `full`(전체), `window`(구간) 통계.

- [ ] **Step 1: 거리 스윕 확장 + 구간 파라미터**

Modify M5 분기 — 거리 스윕을 신뢰구간을 잘 특성화하도록 넓히고 구간 파라미터를 파싱한다.
`--m5-frames` 파싱 부근에 추가:
```python
        dmin, dmax = 1.4, 1.8
        for a in sys.argv[1:]:
            if a.startswith("--m5-dmin="):
                dmin = float(a.split("=", 1)[1])
            if a.startswith("--m5-dmax="):
                dmax = float(a.split("=", 1)[1])
```
스윕 거리 튜플을 넓힌다(기존 `for d in (1.3, 1.5, 1.7, 1.9):` → 아래로):
```python
        for d in (1.3, 1.4, 1.5, 1.6, 1.7, 1.8, 1.9):
```

- [ ] **Step 2: 표본에 거리 기록 + 전체/구간 분리 판정**

스윕에서 각 표본에 `d` 를 함께 저장하도록 누적 리스트를 (err, d) 튜플로 바꾸고, 판정을
전체와 구간으로 나눈다. Task 4 의 판정부(`s_pos, s_yaw = ...` 부터 `M5_RESULT` 출력까지)를
아래로 교체:
```python
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
```
그리고 스윕 루프에서 `dists` 를 채우도록 한다. Task 4 의 스윕 루프 안 `fused_vec.append(...)`
근처에 표본이 채택될 때마다 `dists.append(d)` 를 추가하고, 루프 앞에 `dists = []` 를 선언한다
(single_pos/fused_pos 와 항상 같은 길이가 되도록 채택 지점에서만 append).

- [ ] **Step 3: 문법 검사**

Run: `cd /home/rokey/p3/cobot_ws/isaacpjt/Isaac_envo && python3 -m py_compile parking_v4_runner.py && echo OK`
Expected: `OK`

- [ ] **Step 4: 실행 — 전체 vs 신뢰구간 판정**

```bash
cd /home/rokey/p3/cobot_ws/isaacpjt/Isaac_envo
ps -eo pid,args --no-headers | grep -E "/kit/kit|python\.sh|isaacsim" | grep -v grep || echo clean
bash parking_v4_runner.sh --probe=M5 --m5-frames=1 2>&1 | grep -E "M5_TBASECAM_CAL|M5_RESULT|Traceback"
```
Expected: `M5_RESULT=... window=[1.4,1.8]m window_pos_p95=<값> ... full_pos_p95=<값> ...`
- **window_pos_p95 ≤ 2cm 및 window_yaw_p95 ≤ 1° 이면 PASS** — 신뢰 운영구간에서 정확도 확보.
- 만약 [1.4,1.8] 구간(모든 yaw/lat 포함)에서도 실패하면, 구간을 더 좁히지 말고 그대로 보고한다
  (거리만으로 안 되면 yaw 도 봐야 하는데 그건 별도 결정). n_win 이 너무 작으면(예: <9) 구간이
  비현실적으로 좁다는 신호이므로 함께 보고한다.

- [ ] **Step 5: 좀비 확인 + 커밋**

```bash
cd /home/rokey/p3/cobot_ws
ps -eo pid,args --no-headers | grep -E "/kit/kit|python\.sh|isaacsim" | grep -v grep || echo clean
git add isaacpjt/Isaac_envo/parking_v4_runner.py
git commit -m "feat(probe): M5 신뢰 거리구간 판정 — 운영구간 p95 로 관문, 전체도 함께 보고"
```
