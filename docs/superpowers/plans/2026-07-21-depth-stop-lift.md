# 뎁스캠 기반 정지 판단 + 뒷바퀴 리프트 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** `isaacpjt/Isaac_envo/depth_stop_lift_test.py`를 만든다 — 로봇이 실제 메카넘 구동으로
Sedan 뒤에서 하부로 진입하고, 전방 뎁스캠으로 뒷축 접근을 감지해 정지한 뒤 팔을 전개해
뒷바퀴를 들어올리는 헤드리스/GUI 검증 스크립트.

**Architecture:** 기존 `parking_robot_rear_lift_test.py`의 뼈대(테스트 stage 생성·저장 →
SimulationApp 구동 → 리포트 JSON)를 재사용하되, `robot.set_world_poses()` 순간이동 진입을
`mecanum_drive.py`의 `/cmd_vel` 역기구학 기반 실제 휠 구동으로 교체한다. 정지 판단은 순수
파이썬 모듈(`depth_stop_detector.py`)로 분리해 Isaac 없이 단위 테스트한다. 로봇 에셋은
`hwia_depth_cam_mecha_roller.usd`(뎁스캠 4대 + 메카넘 롤러 내장, prim 계층이 기존 로봇
에셋보다 한 단계 얕음)로 교체한다.

**Tech Stack:** Python 3.11 (Isaac Sim 내장), `pxr` (USD), `isaacsim.core.prims.Articulation`,
`isaacsim.sensors.camera.Camera`, `numpy`. 순수 로직 모듈은 numpy만 의존한다(Isaac 불필요).

## Global Constraints

- 원본 USD 에셋은 절대 수정하지 않는다 — 새 test stage에서만 참조+override(기존 관례).
- USD에 절대경로를 굽지 않는다 — 로봇 참조는 `../hwia_parking_robot_final_caster_package/...`
  상대경로로 authoring (기존 관례, `HANDOFF.md` 주의사항).
- 이 머신(RTX 5080)은 GPU PhysX를 쓴다 — `PhysxSceneAPI`의 `BroadphaseType=GPU`,
  `EnableGPUDynamics=True`, `SolverType=TGS`(기존 검증된 설정 그대로 재사용).
- Isaac Sim 실행 스크립트는 실행 전후 반드시 좀비 프로세스를 확인·정리한다:
  `ps aux | grep -i isaac` 후 필요 시 `kill`. `app.close()`가 걸려 좀비가 남을 수 있다.
- 물리 관련 상수(차량 안정화 플래그, 마찰 재질, GPU 솔버 설정)는 이미 검증된 값이므로
  이번 작업에서 바꾸지 않는다 — `parking_robot_rear_lift_test.py`에서 그대로 가져온다.
- 뎁스 임계값(`DROP_MARGIN` 등)은 실측 전이라 정확한 튜닝을 이번 작업의 성공 기준으로
  삼지 않는다. **최소 성공 기준은 다음번에 튜닝할 수 있는 궤적 데이터(`depth_trace`)를
  리포트에 남기는 것.**

---

## File Structure

- Create: `isaacpjt/Isaac_envo/depth_stop_detector.py` — ROI 추출 + 베이스라인 캘리브레이션
  + 연속-프레임 트리거 판정. 순수 파이썬(numpy만 의존), Isaac 불필요.
- Create: `isaacpjt/Isaac_envo/test_depth_stop_detector.py` — 위 모듈의 단위 테스트.
  `src/parkbot_aruco/test/test_aruco_pose.py`와 같은 관례: `python3` 직접 실행도,
  `pytest`도 가능하게 assert 기반 `test_*` 함수로 작성.
- Create: `isaacpjt/Isaac_envo/probe_depth_cam_stop.py` — 빈 바닥 + 로봇만 있는 최소 씬에서
  `Camera_Pseudo_Depth_Front`가 실제 유효한 `distance_to_image_plane` 값을 내는지, 그리고
  프레임의 축 순서(행=세로/열=가로)가 어느 쪽인지 실측으로 확인하는 probe 스크립트.
- Create: `isaacpjt/Isaac_envo/depth_stop_lift_test.py` — 메인 스크립트. `build_test_stage()`
  (stage 구성, SimulationApp 불필요)와 `run_test()` + `main()`(실제 구동+정지+리프트, 뎁스캠
  실행 필요)으로 나눠 작업한다.

---

### Task 1: 뎁스 정지 판정 순수 로직 (Isaac 불필요)

**Files:**
- Create: `isaacpjt/Isaac_envo/depth_stop_detector.py`
- Test: `isaacpjt/Isaac_envo/test_depth_stop_detector.py`

**Interfaces:**
- Produces: `roi_min_depth(depth_hw: np.ndarray, roi_frac: tuple[float,float,float,float] = (0.30, 0.70, 0.50, 1.00)) -> float`
  — `depth_hw`는 반드시 `(height, width)` 2차원 배열(inf/nan 허용). `roi_frac`은
  `(col_frac_lo, col_frac_hi, row_frac_lo, row_frac_hi)`. 유효 픽셀이 없으면 `math.inf` 반환.
- Produces: `class DepthStopDetector(baseline_frames: int = 30, drop_margin: float = 0.05, confirm_frames: int = 3)`
  with method `update(step: int, roi_value: float) -> bool` (정지 트리거 프레임에 `True`)
  and readable attributes `baseline: float | None`, `threshold: float | None`,
  `triggered: bool`, `trigger_step: int | None`, `trigger_value: float | None`.

- [ ] **Step 1: 실패하는 테스트 작성**

`isaacpjt/Isaac_envo/test_depth_stop_detector.py`:

```python
#!/usr/bin/env python3
"""depth_stop_detector 단위 테스트 (Isaac/ROS 불필요).

실행:
    python3 test_depth_stop_detector.py    # 직접 실행, PASS/FAIL 출력
    pytest test_depth_stop_detector.py     # 동일 검증을 assert 로
"""

import math

import numpy as np

from depth_stop_detector import DepthStopDetector, roi_min_depth


def test_roi_min_depth_extracts_bottom_center_region():
    depth = np.full((100, 200), 5.0, dtype=np.float32)
    # 세로 하단 50%(50~100행), 가로 중앙 40%(80~120열)에만 낮은 값을 심는다.
    depth[70:80, 90:110] = 0.42
    value = roi_min_depth(depth, roi_frac=(0.30, 0.70, 0.50, 1.00))
    assert abs(value - 0.42) < 1e-6

    # 같은 낮은 값을 ROI 밖(상단)에 심으면 잡히면 안 된다.
    depth2 = np.full((100, 200), 5.0, dtype=np.float32)
    depth2[0:10, 90:110] = 0.42
    value2 = roi_min_depth(depth2, roi_frac=(0.30, 0.70, 0.50, 1.00))
    assert abs(value2 - 5.0) < 1e-6


def test_roi_min_depth_ignores_non_finite():
    depth = np.full((100, 200), np.inf, dtype=np.float32)
    depth[60:90, 80:120] = np.nan
    depth[65, 95] = 0.9
    value = roi_min_depth(depth, roi_frac=(0.30, 0.70, 0.50, 1.00))
    assert abs(value - 0.9) < 1e-6


def test_roi_min_depth_all_non_finite_returns_inf():
    depth = np.full((100, 200), np.inf, dtype=np.float32)
    value = roi_min_depth(depth, roi_frac=(0.30, 0.70, 0.50, 1.00))
    assert math.isinf(value)


def test_detector_does_not_trigger_before_calibration():
    det = DepthStopDetector(baseline_frames=5, drop_margin=0.05, confirm_frames=3)
    # 첫 4프레임은 낮은 값이 섞여도(잡음) 베이스라인이 아직 안 잡혔으므로 트리거 금지.
    for step, value in enumerate([2.0, 2.0, 0.1, 2.0], start=1):
        assert det.update(step, value) is False
    assert det.baseline is None


def test_detector_ignores_single_frame_noise():
    det = DepthStopDetector(baseline_frames=3, drop_margin=0.05, confirm_frames=3)
    for step, value in enumerate([2.0, 2.0, 2.0], start=1):
        det.update(step, value)
    assert det.baseline == 2.0
    # 한 프레임만 뚝 떨어지고 바로 회복 -> 트리거되면 안 된다.
    assert det.update(4, 1.0) is False
    assert det.update(5, 2.0) is False
    assert det.triggered is False


def test_detector_triggers_after_confirm_frames():
    det = DepthStopDetector(baseline_frames=3, drop_margin=0.05, confirm_frames=3)
    for step, value in enumerate([2.0, 2.0, 2.0], start=1):
        det.update(step, value)
    assert det.update(4, 1.0) is False
    assert det.update(5, 1.0) is False
    assert det.update(6, 1.0) is True
    assert det.triggered is True
    assert det.trigger_step == 6
    assert det.trigger_value == 1.0


def main():
    tests = [
        test_roi_min_depth_extracts_bottom_center_region,
        test_roi_min_depth_ignores_non_finite,
        test_roi_min_depth_all_non_finite_returns_inf,
        test_detector_does_not_trigger_before_calibration,
        test_detector_ignores_single_frame_noise,
        test_detector_triggers_after_confirm_frames,
    ]
    failed = 0
    for test in tests:
        try:
            test()
            print(f"PASS {test.__name__}")
        except AssertionError as exc:
            failed += 1
            print(f"FAIL {test.__name__}: {exc}")
    print(f"DETECTOR_TEST_OK={failed == 0}")
    if failed:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
```

- [ ] **Step 2: 테스트 실행해서 실패 확인**

Run: `cd /home/rokey/cobot3_ws/isaacpjt/Isaac_envo && python3 test_depth_stop_detector.py`
Expected: `ModuleNotFoundError: No module named 'depth_stop_detector'`

- [ ] **Step 3: 최소 구현 작성**

`isaacpjt/Isaac_envo/depth_stop_detector.py`:

```python
#!/usr/bin/env python3
"""뎁스캠 프레임에서 정지 시점을 판단하는 순수 파이썬 로직.

Isaac/ROS 의존성 없음(numpy만 사용) — depth_stop_lift_test.py와
probe_depth_cam_stop.py가 이 모듈을 가져다 쓴다.

정지 판단은 절대 임계값이 아니라 **런타임 캘리브레이션**이다: 진입 초반
`baseline_frames` 개 프레임 동안 ROI 최소 뎁스를 모아 중앙값을 베이스라인으로
잡고, 그보다 `drop_margin` 이상 낮은 값이 `confirm_frames` 연속으로 나오면
트리거한다. 단일 프레임 잡음으로 오정지하지 않기 위한 디바운스다.
"""

import math

import numpy as np

DEFAULT_ROI_FRAC = (0.30, 0.70, 0.50, 1.00)  # (col_lo, col_hi, row_lo, row_hi), 0~1 비율


def roi_min_depth(depth_hw, roi_frac=DEFAULT_ROI_FRAC):
    """depth_hw: (height, width) 2차원 배열. ROI 내 유효(finite) 최소값을 반환.

    유효 픽셀이 하나도 없으면 math.inf (신호 없음).
    """
    depth_hw = np.asarray(depth_hw)
    if depth_hw.ndim != 2:
        raise ValueError(f"depth_hw must be 2D (height, width), got shape {depth_hw.shape}")
    h, w = depth_hw.shape
    col_lo_f, col_hi_f, row_lo_f, row_hi_f = roi_frac
    row_lo, row_hi = int(h * row_lo_f), max(int(h * row_hi_f), int(h * row_lo_f) + 1)
    col_lo, col_hi = int(w * col_lo_f), max(int(w * col_hi_f), int(w * col_lo_f) + 1)
    patch = np.asarray(depth_hw[row_lo:row_hi, col_lo:col_hi], dtype=np.float64)
    finite = patch[np.isfinite(patch)]
    if finite.size == 0:
        return math.inf
    return float(finite.min())


class DepthStopDetector:
    """진입 구동 중 프레임을 하나씩 넣어 정지 시점을 판단한다."""

    def __init__(self, baseline_frames=30, drop_margin=0.05, confirm_frames=3):
        self.baseline_frames = baseline_frames
        self.drop_margin = drop_margin
        self.confirm_frames = confirm_frames
        self._baseline_samples = []
        self.baseline = None
        self.threshold = None
        self._consecutive = 0
        self.triggered = False
        self.trigger_step = None
        self.trigger_value = None

    def update(self, step, roi_value):
        """한 프레임의 ROI 최소뎁스를 넣는다. 이번 프레임에 트리거되면 True."""
        if self.triggered:
            return True

        if self.baseline is None:
            if math.isfinite(roi_value):
                self._baseline_samples.append(roi_value)
            if len(self._baseline_samples) >= self.baseline_frames:
                self.baseline = float(np.median(self._baseline_samples))
                self.threshold = self.baseline - self.drop_margin
            return False

        if math.isfinite(roi_value) and roi_value < self.threshold:
            self._consecutive += 1
        else:
            self._consecutive = 0

        if self._consecutive >= self.confirm_frames:
            self.triggered = True
            self.trigger_step = step
            self.trigger_value = roi_value
            return True
        return False
```

- [ ] **Step 4: 테스트 실행해서 통과 확인**

Run: `cd /home/rokey/cobot3_ws/isaacpjt/Isaac_envo && python3 test_depth_stop_detector.py`
Expected:
```
PASS test_roi_min_depth_extracts_bottom_center_region
PASS test_roi_min_depth_ignores_non_finite
PASS test_roi_min_depth_all_non_finite_returns_inf
PASS test_detector_does_not_trigger_before_calibration
PASS test_detector_ignores_single_frame_noise
PASS test_detector_triggers_after_confirm_frames
DETECTOR_TEST_OK=True
```

Also run: `cd /home/rokey/cobot3_ws/isaacpjt/Isaac_envo && python3 -m pytest test_depth_stop_detector.py -v`
Expected: 6 passed

- [ ] **Step 5: 커밋**

```bash
cd /home/rokey/cobot3_ws
git add isaacpjt/Isaac_envo/depth_stop_detector.py isaacpjt/Isaac_envo/test_depth_stop_detector.py
git commit -m "feat: add pure-python depth-drop stop detector with unit tests"
```

---

### Task 2: 뎁스캠 실측 probe (Isaac 필요, 차량 없음)

**Files:**
- Create: `isaacpjt/Isaac_envo/probe_depth_cam_stop.py`
- Create (실행 시 생성): `isaacpjt/Isaac_envo/probe_depth_cam_stop_report.json`

**Interfaces:**
- Consumes: `depth_stop_detector.roi_min_depth` (Task 1).
- Consumes: `verify_depth_cam_mecha.py`의 `_restart_with_isaac_python` 패턴, `mecanum_drive.py`의
  `configure_hub_drives`.
- Produces: 콘솔에 `DEPTH_PROBE_OK=<bool>` 출력 + `probe_depth_cam_stop_report.json`
  (`depth_frame_shape`, `rgba_frame_shape`, `depth_axis_order`, `finite_fraction`,
  `depth_min_m`, `depth_max_m`, `depth_mean_m`, `roi_min_depth_m` 키 포함).

이 태스크의 목적은 스펙의 리스크 항목("`Camera_Pseudo_Depth_Front`가 실제로 유효한 값을
내는지 미검증")을 본 구현 전에 확인하는 것이다. **뎁스 프레임의 축 순서를 추측하지 않고
실측으로 정한다** — `get_rgba()`는 `(height, width, 4)`가 확실하므로(기존 스크립트들이
이미 그렇게 써 왔다), 같은 카메라의 `get_depth()` 프레임 shape을 비교해 `(height, width, 1)`
인지 `(width, height, 1)`인지 판정한다.

- [ ] **Step 1: probe 스크립트 작성**

`isaacpjt/Isaac_envo/probe_depth_cam_stop.py`:

```python
#!/usr/bin/env python3
"""hwia_depth_cam_mecha_roller.usd 의 전방 뎁스캠이 실제로 유효한
distance_to_image_plane 값을 내는지, 프레임 축 순서가 어느 쪽인지 실측한다.

빈 바닥 + 로봇만 있는 최소 씬(verify_depth_cam_mecha.py 1단계와 동일한 방식).
차량/주차장은 필요 없다 — 카메라 센서 자체의 유효성만 확인한다.

실행:
    python3 probe_depth_cam_stop.py
"""

import json
import math
import os
import sys
from pathlib import Path

WORK_DIR = Path(__file__).resolve().parent
ROBOT_USD = (WORK_DIR.parent / "hwia_parking_robot_final_caster_package"
             / "hwia_depth_cam_mecha_roller.usd")
REPORT = WORK_DIR / "probe_depth_cam_stop_report.json"
ISAAC_PYTHON = Path(
    "/home/rokey/dev_ws/isaac_sim/isaacsim/_build/linux-x86_64/release/python.sh"
)

ROBOT_XFORM = "/World/Robot"
ROBOT_JOINTS = "/World/Robot/joints"
CAM_FRONT = "/World/Robot/cam_front_link/depth_cam_front/Camera_Pseudo_Depth_Front"
CAM_RES = (640, 480)


def _restart_with_isaac_python():
    if os.environ.get("CARB_APP_PATH"):
        return
    os.execv(str(ISAAC_PYTHON), [str(ISAAC_PYTHON), str(Path(__file__).resolve()), *sys.argv[1:]])


def main():
    _restart_with_isaac_python()

    from isaacsim import SimulationApp
    app = SimulationApp({"headless": True})
    report = {}
    try:
        import numpy as np
        import omni.timeline
        import omni.usd
        from isaacsim.core.api import World
        from isaacsim.sensors.camera import Camera
        from pxr import Gf, Usd, UsdGeom, UsdPhysics

        from depth_stop_detector import roi_min_depth
        from mecanum_drive import configure_hub_drives

        if not ROBOT_USD.is_file():
            raise FileNotFoundError(f"robot asset not found: {ROBOT_USD}")

        stage = Usd.Stage.CreateInMemory()
        UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.y)
        UsdGeom.SetStageMetersPerUnit(stage, 1.0)
        w = UsdGeom.Xform.Define(stage, "/World").GetPrim()
        stage.SetDefaultPrim(w)

        ground = UsdGeom.Cube.Define(stage, "/World/Ground")
        ground.CreateSizeAttr(1.0)
        gx = UsdGeom.Xformable(ground)
        gx.AddTranslateOp().Set(Gf.Vec3d(0.0, -0.5, 0.0))
        gx.AddScaleOp().Set(Gf.Vec3f(40.0, 1.0, 40.0))
        UsdPhysics.CollisionAPI.Apply(ground.GetPrim())

        scene = UsdPhysics.Scene.Define(stage, "/World/PhysicsScene")
        scene.CreateGravityDirectionAttr(Gf.Vec3f(0.0, -1.0, 0.0))
        scene.CreateGravityMagnitudeAttr(9.81)

        robot = stage.DefinePrim(ROBOT_XFORM, "Xform")
        robot.GetReferences().AddReference(str(ROBOT_USD))
        rxf = UsdGeom.Xformable(robot)
        rxf.ClearXformOpOrder()
        rxf.MakeMatrixXform().Set(Gf.Matrix4d(
            0.0, 0.0, 1.0, 0.0,
            1.0, 0.0, 0.0, 0.0,
            0.0, 1.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 1.0,
        ))

        tmp = WORK_DIR / "_probe_depth_cam_stop.usd"
        stage.GetRootLayer().Export(str(tmp))
        ctx = omni.usd.get_context()
        ctx.open_stage(str(tmp))
        for _ in range(30):
            app.update()
        stage = ctx.get_stage()

        configure_hub_drives(stage, ROBOT_JOINTS)
        world = World(stage_units_in_meters=1.0, set_defaults=False)
        omni.timeline.get_timeline_interface().play()
        world.reset()
        for _ in range(30):
            world.step(render=False)

        cam = Camera(prim_path=CAM_FRONT, resolution=CAM_RES)
        cam.initialize()
        cam.add_distance_to_image_plane_to_frame()
        # M1 교훈: 타임라인이 돌아야 렌더 프로덕트가 프레임을 낸다. 몇 스텝 워밍업 필요.
        for _ in range(60):
            world.step(render=True)

        rgba = cam.get_rgba()
        depth = cam.get_depth()

        rgba_shape = tuple(int(v) for v in getattr(rgba, "shape", ()))
        depth_shape = tuple(int(v) for v in getattr(depth, "shape", ()))
        report["rgba_frame_shape"] = rgba_shape
        report["depth_frame_shape"] = depth_shape

        if depth is None or not getattr(depth, "size", 0):
            report["depth_axis_order"] = "unknown"
            report["finite_fraction"] = 0.0
            ok = False
        else:
            depth_arr = np.asarray(depth, dtype=np.float64).squeeze()
            # rgba는 (height, width, 4)가 확정값. depth의 앞 두 축이 rgba와 같은
            # 순서면 (height, width), 뒤집혀 있으면 (width, height)로 판단한다.
            if depth_arr.ndim == 2 and rgba_shape[:2] == depth_arr.shape:
                axis_order = "height_width"
                depth_hw = depth_arr
            elif depth_arr.ndim == 2 and rgba_shape[:2] == depth_arr.shape[::-1]:
                axis_order = "width_height"
                depth_hw = depth_arr.T
            else:
                axis_order = f"unexpected ndim={depth_arr.ndim} shape={depth_arr.shape}"
                depth_hw = depth_arr if depth_arr.ndim == 2 else depth_arr.reshape(-1, 1)
            report["depth_axis_order"] = axis_order

            finite = depth_hw[np.isfinite(depth_hw)]
            report["finite_fraction"] = float(finite.size) / float(depth_hw.size)
            report["depth_min_m"] = float(finite.min()) if finite.size else None
            report["depth_max_m"] = float(finite.max()) if finite.size else None
            report["depth_mean_m"] = float(finite.mean()) if finite.size else None
            report["roi_min_depth_m"] = roi_min_depth(depth_hw)
            ok = finite.size > 0 and math.isfinite(report["roi_min_depth_m"])

        report["ok"] = ok
        REPORT.write_text(json.dumps(report, indent=2, ensure_ascii=False), encoding="utf-8")
        print(f"[probe] rgba shape={rgba_shape} depth shape={depth_shape}", flush=True)
        print(f"[probe] axis_order={report.get('depth_axis_order')}", flush=True)
        print(f"[probe] finite_fraction={report.get('finite_fraction')}", flush=True)
        print(f"[probe] roi_min_depth_m={report.get('roi_min_depth_m')}", flush=True)
        print(f"[probe] 리포트: {REPORT}", flush=True)
        print(f"DEPTH_PROBE_OK={ok}", flush=True)
        tmp.unlink(missing_ok=True)
        if not ok:
            raise SystemExit(1)
    finally:
        app.close()


if __name__ == "__main__":
    main()
```

- [ ] **Step 2: 좀비 프로세스 확인 후 실행**

Run:
```bash
ps aux | grep -i isaac | grep -v grep   # 남은 Isaac 프로세스가 있으면 kill 후 진행
cd /home/rokey/cobot3_ws/isaacpjt/Isaac_envo && python3 probe_depth_cam_stop.py
```
Expected: 마지막 줄 `DEPTH_PROBE_OK=True`, `probe_depth_cam_stop_report.json`이 생성되고
`finite_fraction > 0`, `roi_min_depth_m`이 유한값.

**만약 `DEPTH_PROBE_OK=False`가 나오면** — 이 태스크의 목적이 바로 이걸 미리 잡아내는
것이다. `depth_frame_shape`과 `finite_fraction`을 리포트에서 확인해 원인(카메라 렌더
프로덕트 미설정, clippingRange 문제 등)을 진단하고, Task 3/4로 넘어가기 전에 여기서
고친다. 원인 진단 결과와 조치는 이 태스크의 커밋 메시지 또는 `DEBUG_LOG.md`에 남긴다
(프로젝트 관례).

- [ ] **Step 3: 실행 후 좀비 프로세스 정리**

Run: `ps aux | grep -i isaac | grep -v grep`
Expected: 관련 프로세스가 남아 있지 않음(있으면 `kill <pid>`).

- [ ] **Step 4: 커밋**

```bash
cd /home/rokey/cobot3_ws
git add isaacpjt/Isaac_envo/probe_depth_cam_stop.py isaacpjt/Isaac_envo/probe_depth_cam_stop_report.json
git commit -m "feat: probe depth_cam_front sensor validity + frame axis order"
```

---

### Task 3: 테스트 stage 빌더 (Isaac 필요, SimulationApp 불필요)

**Files:**
- Create: `isaacpjt/Isaac_envo/depth_stop_lift_test.py` (이 태스크에서는 `build_test_stage()`까지만)

**Interfaces:**
- Consumes: `parking_robot_rear_lift_test.py`의 `build_test_stage()` 패턴(주차장/차량/물리씬
  구성 전부), 단 로봇 참조와 prim 경로만 교체.
- Produces: 상수 `ROBOT_ROOT = "/World/Robot/base_link"`, `ROBOT_JOINTS = "/World/Robot/joints"`,
  `ROBOT_WRAP = "/World/Robot"`, `CAM_FRONT`, `ARM_TARGETS`, `WHEEL_JOINTS`(mecanum_drive에서
  import), `SEDAN_ROOT`, `SEDAN_WHEELS`, `OUTPUT_USD`, `REPORT_JSON` — Task 4가 그대로 쓴다.
  함수 `build_test_stage() -> None`(저장된 `OUTPUT_USD`를 만든다. 반환값 없음, 기존 관례와 동일).

이 로봇 에셋의 prim 계층이 기존 `_mecha_roller.usd`보다 한 단계 얕다는 것을 이번 세션
초반 실측으로 확인했다(`verify_depth_cam_mecha.py`가 이미 이 상수들을 쓰고 있다):
- 기존 로봇(`hwia_parking_robot_final_caster_mecha_roller.usd`): `wheel_fl` 등이
  `/World/Robot/base_link/wheel_fl`에 있다.
- 이번 로봇(`hwia_depth_cam_mecha_roller.usd`): `wheel_fl`, `bearing_roller_*`, `arm_*`,
  `cam_*_link`가 전부 `/World/Robot/{name}`에 직접 있고(부모가 `base_link`가 아니라
  defaultPrim), `base_link`는 리지드바디 루트로만 존재한다.

- [ ] **Step 1: `depth_stop_lift_test.py` 골격 + `build_test_stage()` 작성**

`isaacpjt/Isaac_envo/depth_stop_lift_test.py`:

```python
#!/usr/bin/env python3
"""뎁스캠 기반 정지 판단 + 뒷바퀴 리프트 통합 테스트 (Isaac Sim 5.1).

parking_robot_rear_lift_test.py 와 같은 주차장(A7) + Sedan 시나리오를 재사용하되,
로봇을 hwia_depth_cam_mecha_roller.usd(뎁스캠 4대 + 메카넘 롤러 내장)로 교체하고,
진입을 순간이동(set_world_poses) 대신 실제 /cmd_vel 기반 휠 구동으로 한다.
전방 뎁스캠(depth_stop_detector.DepthStopDetector)이 뒷축 접근을 감지하면 정지하고
팔을 전개한다. 원본 로봇/차량/주차장 에셋은 수정하지 않는다.

실행:
    python3 depth_stop_lift_test.py                    # 헤드리스
    python3 depth_stop_lift_test.py --gui               # GUI
    python3 depth_stop_lift_test.py --sphere-wheels      # Sedan 휠 충돌체를 구로(권장)
    python3 depth_stop_lift_test.py --keep-drivetrain    # Sedan PhysX Vehicle 구동계 유지
    python3 depth_stop_lift_test.py --drop-margin 0.08   # 뎁스 정지 마진 조절
"""

import json
import math
import os
import sys
from pathlib import Path

WORK_DIR = Path(__file__).resolve().parent
PARKING_SOURCE_USD = WORK_DIR / "parking" / "parking_environment.usd"
PARKING_USD = WORK_DIR / "parking" / "parking_environment_depth_stop_test.usd"
VEHICLES_USD = WORK_DIR / "fab_vehicles.usd"
ROBOT_PACKAGE = WORK_DIR.parent / "hwia_parking_robot_final_caster_package"
ROBOT_USD = ROBOT_PACKAGE / "hwia_depth_cam_mecha_roller.usd"
ROBOT_REF = f"../{ROBOT_PACKAGE.name}/{ROBOT_USD.name}"

KEEP_DRIVETRAIN = "--keep-drivetrain" in sys.argv[1:]
SPHERE_WHEELS = "--sphere-wheels" in sys.argv[1:]
DROP_MARGIN = 0.05
if "--drop-margin" in sys.argv[1:]:
    DROP_MARGIN = float(sys.argv[sys.argv.index("--drop-margin") + 1])

OUTPUT_USD = WORK_DIR / "depth_stop_lift_test.usd"
REPORT_JSON = WORK_DIR / "depth_stop_lift_test_report.json"

ISAAC_ROOT = Path("/home/rokey/dev_ws/isaac_sim/isaacsim/_build/linux-x86_64/release")
ISAAC_PYTHON = ISAAC_ROOT / "python.sh"

# A7 슬롯 — parking_robot_rear_lift_test.py 와 동일(A5 는 팀원 에셋이 A5_Coupe 로 점유).
PARKING_CENTER = (8.5, 0.0, 7.8)
SEDAN_REAR_AXLE_LOCAL_Z = (-1.5053923 - 1.4878858) * 0.5
ROBOT_START_Z = 4.55
ROBOT_TARGET_Z = PARKING_CENTER[2] + SEDAN_REAR_AXLE_LOCAL_Z

# 이 로봇 에셋(hwia_depth_cam_mecha_roller.usd)은 hwia_parking_robot_final_caster_mecha_roller.usd
# 보다 prim 계층이 한 단계 얕다 — wheel_fl 등이 base_link 밑이 아니라 루트 바로 밑에 있다.
# (verify_depth_cam_mecha.py 에서 이미 같은 상수를 쓴다.)
ROBOT_WRAP = "/World/Robot"
ROBOT_ROOT = "/World/Robot/base_link"
ROBOT_JOINTS = "/World/Robot/joints"
CAM_FRONT = "/World/Robot/cam_front_link/depth_cam_front/Camera_Pseudo_Depth_Front"
CAM_RES = (640, 480)

SEDAN_ROOT = "/World/VehicleAsset/Vehicles/Sedan"
ARM_TARGETS = {
    "arm_left_front_joint": 90.0,
    "arm_left_rear_joint": -90.0,
    "arm_right_front_joint": -90.0,
    "arm_right_rear_joint": 90.0,
}
SEDAN_WHEELS = (
    "FrontLeftWheel",
    "FrontRightWheel",
    "RearLeftWheel",
    "RearRightWheel",
)

DEPTH_SPEED = 0.4          # m/s, verify_depth_cam_mecha.py 에서 검증된 mecanum 전진 속도
DEPTH_BASELINE_FRAMES = 30
DEPTH_CONFIRM_FRAMES = 3
DEPTH_MAX_STEPS = 900       # 15s @ 60Hz 안전 컷오프 — 이보다 오래 걸리면 timeout 처리


def _restart_with_isaac_python():
    if os.environ.get("CARB_APP_PATH"):
        return
    if not ISAAC_PYTHON.is_file():
        raise FileNotFoundError(f"Isaac Sim python.sh not found: {ISAAC_PYTHON}")
    os.execv(str(ISAAC_PYTHON), [str(ISAAC_PYTHON), str(Path(__file__).resolve()), *sys.argv[1:]])


def _replace_matrix_xform(prim, matrix, UsdGeom):
    xform = UsdGeom.Xformable(prim)
    xform.ClearXformOpOrder()
    xform.MakeMatrixXform().Set(matrix)


def build_test_stage():
    from mecanum_drive import WHEEL_JOINTS, configure_hub_drives
    from pxr import Gf, PhysxSchema, Sdf, Usd, UsdGeom, UsdPhysics, UsdShade

    for path in (PARKING_SOURCE_USD, VEHICLES_USD, ROBOT_USD):
        if not path.is_file():
            raise FileNotFoundError(path)

    source_parking_layer = Sdf.Layer.FindOrOpen(str(PARKING_SOURCE_USD))
    if source_parking_layer is None:
        raise RuntimeError(f"unable to read parking layer: {PARKING_SOURCE_USD}")
    parking_layer = (
        Sdf.Layer.FindOrOpen(str(PARKING_USD))
        if PARKING_USD.is_file()
        else Sdf.Layer.CreateNew(str(PARKING_USD))
    )
    parking_layer.TransferContent(source_parking_layer)
    sensors_spec = parking_layer.GetPrimAtPath("/World/Sensors")
    if sensors_spec is not None:
        for child in sensors_spec.nameChildren:
            if child.name.startswith("CeilingLidar"):
                child.referenceList.ClearEdits()
                child.active = False
    parking_layer.Save()

    stage = Usd.Stage.CreateNew(str(OUTPUT_USD))
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.y)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    stage.SetTimeCodesPerSecond(60.0)
    world = UsdGeom.Xform.Define(stage, "/World").GetPrim()
    stage.SetDefaultPrim(world)

    parking = UsdGeom.Xform.Define(stage, "/World/Parking").GetPrim()
    parking.GetReferences().AddReference(str(PARKING_USD))
    vehicle_asset = UsdGeom.Xform.Define(stage, "/World/VehicleAsset").GetPrim()
    vehicle_asset.GetReferences().AddReference(str(VEHICLES_USD))
    robot = UsdGeom.Xform.Define(stage, "/World/Robot").GetPrim()
    robot.GetReferences().AddReference(ROBOT_REF)

    deactivate = [
        "/World/Parking/PhysicsScene",
        "/World/VehicleAsset/PhysicsScene",
        "/World/VehicleAsset/DriveGround",
        "/World/VehicleAsset/FabLighting",
        "/World/VehicleAsset/Cylinder001",
    ]
    parking_sensors = stage.GetPrimAtPath("/World/Parking/Sensors")
    if parking_sensors.IsValid():
        deactivate += [
            str(child.GetPath())
            for child in parking_sensors.GetChildren()
            if child.GetName().startswith("CeilingLidar")
        ]
    for path in deactivate:
        prim = stage.GetPrimAtPath(path)
        if prim.IsValid():
            prim.SetActive(False)

    scene = UsdPhysics.Scene.Define(stage, "/World/PhysicsScene")
    scene.CreateGravityDirectionAttr(Gf.Vec3f(0.0, -1.0, 0.0))
    scene.CreateGravityMagnitudeAttr(9.81)
    physx_scene = PhysxSchema.PhysxSceneAPI.Apply(scene.GetPrim())
    physx_scene.CreateBroadphaseTypeAttr("GPU")
    physx_scene.CreateSolverTypeAttr("TGS")
    physx_scene.CreateEnableCCDAttr(True)
    physx_scene.CreateEnableStabilizationAttr(True)
    physx_scene.CreateEnableGPUDynamicsAttr(True)
    vehicle_context = PhysxSchema.PhysxVehicleContextAPI.Apply(scene.GetPrim())
    vehicle_context.CreateUpdateModeAttr(PhysxSchema.Tokens.velocityChange)
    vehicle_context.CreateVerticalAxisAttr(PhysxSchema.Tokens.posY)
    vehicle_context.CreateLongitudinalAxisAttr(PhysxSchema.Tokens.posZ)

    ground = UsdGeom.Cube.Define(stage, "/World/TestGround")
    ground.CreateSizeAttr(1.0)
    ground.CreateVisibilityAttr(UsdGeom.Tokens.invisible)
    ground_xform = UsdGeom.Xformable(ground)
    ground_xform.AddTranslateOp().Set(Gf.Vec3d(0.0, -0.10, 0.0))
    ground_xform.AddScaleOp().Set(Gf.Vec3f(30.0, 0.20, 30.0))
    UsdPhysics.CollisionAPI.Apply(ground.GetPrim())

    vehicles = stage.GetPrimAtPath("/World/VehicleAsset/Vehicles")
    for vehicle in vehicles.GetChildren():
        if vehicle.GetName() != "Sedan":
            vehicle.SetActive(False)
    sedan = stage.GetPrimAtPath(SEDAN_ROOT)
    sedan_matrix = UsdGeom.Xformable(sedan).GetLocalTransformation()
    sedan_matrix.SetTranslate(Gf.Vec3d(*PARKING_CENTER))
    _replace_matrix_xform(sedan, sedan_matrix, UsdGeom)

    sedan_single_apis = (
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
    if KEEP_DRIVETRAIN:
        print("[depth-lift] --keep-drivetrain: Sedan PhysX Vehicle 구동계 유지", flush=True)
    else:
        for api_schema in sedan_single_apis:
            if sedan.HasAPI(api_schema):
                sedan.RemoveAPI(api_schema)
        for instance_name in (PhysxSchema.Tokens.brakes0, PhysxSchema.Tokens.brakes1):
            if sedan.HasAPI(PhysxSchema.PhysxVehicleBrakesAPI, instance_name):
                sedan.RemoveAPI(PhysxSchema.PhysxVehicleBrakesAPI, instance_name)
        for wheel_name in SEDAN_WHEELS:
            wheel = stage.GetPrimAtPath(f"{SEDAN_ROOT}/{wheel_name}")
            for api_schema in wheel_apis:
                if wheel.HasAPI(api_schema):
                    wheel.RemoveAPI(api_schema)

    if SPHERE_WHEELS:
        radius = None
        for wheel_name in SEDAN_WHEELS:
            wheel_path = f"{SEDAN_ROOT}/{wheel_name}"
            cylinder = stage.GetPrimAtPath(f"{wheel_path}/Collision")
            if not cylinder.IsValid():
                raise RuntimeError(f"휠 충돌체를 찾지 못했습니다: {wheel_path}/Collision")
            radius = float(UsdGeom.Cylinder(cylinder).GetRadiusAttr().Get())
            cylinder.SetActive(False)
            sphere = UsdGeom.Sphere.Define(stage, f"{wheel_path}/CollisionSphere")
            sphere.CreateRadiusAttr(radius)
            sphere.CreatePurposeAttr(UsdGeom.Tokens.guide)
            UsdPhysics.CollisionAPI.Apply(sphere.GetPrim())
        print(f"[depth-lift] --sphere-wheels: 휠 충돌체 4개를 구(r={radius:.4f})로 교체", flush=True)

    sedan_rigid = PhysxSchema.PhysxRigidBodyAPI.Apply(sedan)
    sedan_rigid.GetDisableGravityAttr().Set(False)
    sedan_rigid.CreateEnableCCDAttr(True)
    sedan_rigid.GetSolverPositionIterationCountAttr().Set(16)
    sedan_rigid.GetSolverVelocityIterationCountAttr().Set(8)

    robot_to_world = Gf.Matrix4d(
        0.0, 0.0, 1.0, 0.0,
        1.0, 0.0, 0.0, 0.0,
        0.0, 1.0, 0.0, 0.0,
        PARKING_CENTER[0], 0.0, ROBOT_START_Z, 1.0,
    )
    _replace_matrix_xform(robot, robot_to_world, UsdGeom)

    materials = UsdGeom.Scope.Define(stage, "/World/TestMaterials").GetPath()
    grip = UsdShade.Material.Define(stage, materials.AppendChild("RobotGrip"))
    grip_api = UsdPhysics.MaterialAPI.Apply(grip.GetPrim())
    grip_api.CreateStaticFrictionAttr(1.35)
    grip_api.CreateDynamicFrictionAttr(1.10)
    grip_api.CreateRestitutionAttr(0.01)
    UsdShade.MaterialBindingAPI.Apply(ground.GetPrim()).Bind(
        grip, UsdShade.Tokens.weakerThanDescendants, "physics"
    )
    # 이 에셋은 wheel_fl 등이 base_link 밑이 아니라 /World/Robot 바로 밑에 있다(Task 3 상단 설명).
    for link_name in (
        "wheel_fl", "wheel_fr", "wheel_rl", "wheel_rr",
        "bearing_roller_left_front", "bearing_roller_left_rear",
        "bearing_roller_right_front", "bearing_roller_right_rear",
    ):
        link = stage.GetPrimAtPath(f"{ROBOT_WRAP}/{link_name}")
        UsdShade.MaterialBindingAPI.Apply(link).Bind(
            grip, UsdShade.Tokens.weakerThanDescendants, "physics"
        )
    for wheel_name in SEDAN_WHEELS:
        wheel = stage.GetPrimAtPath(f"{SEDAN_ROOT}/{wheel_name}")
        UsdShade.MaterialBindingAPI.Apply(wheel).Bind(
            grip, UsdShade.Tokens.weakerThanDescendants, "physics"
        )

    configure_hub_drives(stage, ROBOT_JOINTS)
    for name in ARM_TARGETS:
        joint = stage.GetPrimAtPath(f"{ROBOT_JOINTS}/{name}")
        drive = UsdPhysics.DriveAPI.Get(joint, "angular")
        drive.GetStiffnessAttr().Set(1800.0)
        drive.GetDampingAttr().Set(140.0)
        drive.GetMaxForceAttr().Set(5000.0)
        drive.GetTargetPositionAttr().Set(0.0)

    world.SetCustomDataByKey("test", "depth-cam stop-detect + rear-wheel lift")
    world.SetCustomDataByKey("vehicle", "Sedan")
    world.SetCustomDataByKey("parkingBay", "A7")
    world.SetCustomDataByKey("robotTargetZ", ROBOT_TARGET_Z)
    stage.GetRootLayer().Save()


if __name__ == "__main__":
    _restart_with_isaac_python()
    build_test_stage()
    print(f"STAGE_BUILT={OUTPUT_USD}")
```

- [ ] **Step 2: stage 빌드 실행 + 검증**

Run:
```bash
ps aux | grep -i isaac | grep -v grep
cd /home/rokey/cobot3_ws/isaacpjt/Isaac_envo && python3 depth_stop_lift_test.py
```
Expected: 마지막 줄 `STAGE_BUILT=/home/rokey/cobot3_ws/isaacpjt/Isaac_envo/depth_stop_lift_test.usd`,
예외 없음. (이 시점에는 `main()`이 아직 없어 `if __name__` 블록이 `build_test_stage()`만
실행한다 — Task 4에서 이 블록을 실제 `main()`으로 교체한다.)

- [ ] **Step 3: 저장된 stage에 로봇/차량 prim이 기대한 경로에 있는지 확인**

Run:
```bash
cd /home/rokey/cobot3_ws/isaacpjt/Isaac_envo && python3 - << 'EOF'
import os, subprocess, sys
from pathlib import Path
ISAAC_RELEASE = Path("/home/rokey/dev_ws/isaac_sim/isaacsim/_build/linux-x86_64/release")
if "_REEXEC" not in os.environ:
    libs = next((c for c in sorted((ISAAC_RELEASE / "extscache").glob("omni.usd.libs-*")) if (c/"pxr").exists()), None)
    env = dict(os.environ, _REEXEC="1",
        PYTHONPATH=os.pathsep.join([str(libs), os.environ.get("PYTHONPATH","")]).strip(os.pathsep),
        LD_LIBRARY_PATH=os.pathsep.join([str(libs/"bin"), os.environ.get("LD_LIBRARY_PATH","")]).strip(os.pathsep))
    raise SystemExit(subprocess.call([str(ISAAC_RELEASE/"python.sh"), __file__], env=env, cwd=str(ISAAC_RELEASE)))
from pxr import Usd
stage = Usd.Stage.Open("/home/rokey/cobot3_ws/isaacpjt/Isaac_envo/depth_stop_lift_test.usd")
for p in ("/World/Robot/base_link", "/World/Robot/joints/arm_left_front_joint",
          "/World/Robot/wheel_fl", "/World/Robot/cam_front_link/depth_cam_front/Camera_Pseudo_Depth_Front",
          "/World/VehicleAsset/Vehicles/Sedan"):
    prim = stage.GetPrimAtPath(p)
    print(p, "OK" if prim.IsValid() else "MISSING")
EOF
```
Expected: 5줄 전부 `OK`.

- [ ] **Step 4: 좀비 프로세스 정리 + 커밋**

Run: `ps aux | grep -i isaac | grep -v grep` (필요 시 kill)

```bash
cd /home/rokey/cobot3_ws
git add isaacpjt/Isaac_envo/depth_stop_lift_test.py isaacpjt/Isaac_envo/depth_stop_lift_test.usd
git commit -m "feat: build depth-cam-mecha robot + A7 Sedan test stage (no runtime logic yet)"
```

---

### Task 4: 구동 + 뎁스 정지 + 리프트 런타임, CLI

**Files:**
- Modify: `isaacpjt/Isaac_envo/depth_stop_lift_test.py` (Task 3의 `if __name__` 블록을
  `run_test()` + `write_exception_report()` + `main()`으로 교체·확장)

**Interfaces:**
- Consumes: Task 3의 모든 모듈 상수/`build_test_stage()`. Task 1의
  `DepthStopDetector`, `roi_min_depth`. `mecanum_drive.WHEEL_JOINTS`,
  `mecanum_drive.wheel_velocities_from_cmd_vel`. Task 2에서 확인한 `depth_axis_order`
  (probe 리포트가 `height_width`면 `cam.get_depth()`를 그대로 2D로 쓰고, `width_height`면
  `.squeeze().T`로 뒤집어 `roi_min_depth`에 넣는다 — Task 2 리포트 값을 그대로 반영한다).
- Produces: `depth_stop_lift_test_report.json`, 콘솔 `TEST_PASSED=<bool>`,
  `DEPTH_STOP_REASON=<str>`, `REPORT=<path>`.

- [ ] **Step 1: `run_test()` + `main()` 작성 — Task 3의 `if __name__` 블록을 교체**

`depth_stop_lift_test.py`에서 파일 끝의 `if __name__ == "__main__": ...` 블록을 삭제하고
그 자리에 아래를 추가한다. (Task 2에서 실측한 `depth_axis_order`가 `height_width`였다는
전제로 작성했다 — `width_height`였다면 `_depth_to_hw()`의 조건 분기를 그 결과에 맞게
바꾼다.)

```python
def write_exception_report(exc):
    existing = {}
    if REPORT_JSON.is_file():
        try:
            existing = json.loads(REPORT_JSON.read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError):
            existing = {}
    existing.update({
        "passed": False,
        "exception_type": type(exc).__name__,
        "exception": str(exc),
    })
    REPORT_JSON.write_text(json.dumps(existing, indent=2, ensure_ascii=False), encoding="utf-8")


def _depth_to_hw(depth_raw, rgba_hw):
    """카메라 원시 뎁스 프레임을 (height, width) 2D로 정규화한다.

    probe_depth_cam_stop.py 로 실측한 축 순서를 따른다: rgba_hw(=(height,width))와
    앞 두 축이 일치하면 그대로, 뒤집혀 있으면 전치한다.
    """
    import numpy as np

    if depth_raw is None or not getattr(depth_raw, "size", 0):
        return None
    arr = np.asarray(depth_raw, dtype=np.float64).squeeze()
    if arr.ndim != 2:
        return None
    if arr.shape == rgba_hw:
        return arr
    if arr.shape == rgba_hw[::-1]:
        return arr.T
    return arr  # 예상 밖 shape — 호출측에서 roi_min_depth 가 ValueError 로 드러낸다


def run_test(app):
    import numpy as np
    import omni.physx
    import omni.timeline
    import omni.usd
    from isaacsim.core.prims import Articulation
    from isaacsim.sensors.camera import Camera
    from pxr import Usd, UsdGeom

    from depth_stop_detector import DepthStopDetector, roi_min_depth
    from mecanum_drive import WHEEL_JOINTS, wheel_velocities_from_cmd_vel

    context = omni.usd.get_context()
    if not context.open_stage(str(OUTPUT_USD)):
        raise RuntimeError(f"failed to open test stage: {OUTPUT_USD}")
    for _ in range(12):
        app.update()

    stage = context.get_stage()
    timeline = omni.timeline.get_timeline_interface()
    timeline.play()
    for _ in range(12):
        app.update()

    robot = Articulation(ROBOT_ROOT)
    robot.initialize()
    physx = omni.physx.get_physx_interface()

    joint_positions = robot.get_joint_positions()
    command_shape = joint_positions.shape
    wheel_idx = {w: robot.dof_names.index(j) for w, j in WHEEL_JOINTS.items()}
    arm_indices = {name: robot.dof_names.index(name) for name in ARM_TARGETS}
    velocity_targets = np.zeros(command_shape, dtype=np.float32)
    position_targets = np.array(joint_positions, dtype=np.float32, copy=True)

    def rigid_position(path):
        value = physx.get_rigidbody_transformation(path)
        return tuple(float(x) for x in value["position"])

    def wheel_center(name):
        prim = stage.GetPrimAtPath(f"{SEDAN_ROOT}/{name}")
        matrix = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
        return tuple(float(x) for x in matrix.ExtractTranslation())

    def drive(vx, vy, wz):
        for wname, omega in wheel_velocities_from_cmd_vel(vx, vy, wz).items():
            i = wheel_idx[wname]
            if velocity_targets.ndim == 2:
                velocity_targets[0, i] = omega
            else:
                velocity_targets[i] = omega
        robot.set_joint_velocity_targets(velocity_targets)

    def set_arm_targets(scale):
        for name, target in ARM_TARGETS.items():
            index = arm_indices[name]
            target_rad = math.radians(float(target * scale))
            if position_targets.ndim == 2:
                position_targets[0, index] = target_rad
            else:
                position_targets[index] = target_rad
        robot.set_joint_position_targets(position_targets)

    # 정착
    drive(0.0, 0.0, 0.0)
    set_arm_targets(0.0)
    for _ in range(120):
        app.update()

    start_robot = rigid_position(ROBOT_ROOT)
    start_sedan = rigid_position(SEDAN_ROOT)
    initial_wheels = {name: wheel_center(name) for name in SEDAN_WHEELS}

    cam = Camera(prim_path=CAM_FRONT, resolution=CAM_RES)
    cam.initialize()
    cam.add_distance_to_image_plane_to_frame()
    for _ in range(30):
        app.update()
    rgba_hw = tuple(int(v) for v in cam.get_rgba().shape[:2])

    detector = DepthStopDetector(
        baseline_frames=DEPTH_BASELINE_FRAMES,
        drop_margin=DROP_MARGIN,
        confirm_frames=DEPTH_CONFIRM_FRAMES,
    )
    depth_trace = []
    stop_reason = "timeout"
    drive(DEPTH_SPEED, 0.0, 0.0)
    for step in range(1, DEPTH_MAX_STEPS + 1):
        app.update()
        depth_hw = _depth_to_hw(cam.get_depth(), rgba_hw)
        roi_value = roi_min_depth(depth_hw) if depth_hw is not None else math.inf
        p = rigid_position(ROBOT_ROOT)
        depth_trace.append({
            "step": step,
            "x": round(p[0], 4),
            "z": round(p[2], 4),
            "roi_min_depth_m": None if math.isinf(roi_value) else round(roi_value, 4),
        })
        if detector.update(step, roi_value):
            stop_reason = "depth_trigger"
            break
    drive(0.0, 0.0, 0.0)
    for _ in range(60):
        app.update()

    stop_position = rigid_position(ROBOT_ROOT)
    before_lift = {name: wheel_center(name) for name in SEDAN_WHEELS}

    for ramp_step in range(1, 181):
        set_arm_targets(ramp_step / 180.0)
        app.update()
    for _ in range(360):
        app.update()

    after_lift = {name: wheel_center(name) for name in SEDAN_WHEELS}
    final_robot = rigid_position(ROBOT_ROOT)
    final_sedan = rigid_position(SEDAN_ROOT)

    live_positions = robot.get_joint_positions()
    if getattr(live_positions, "ndim", 1) == 2:
        live_positions = live_positions[0]
    arm_angles = {
        name: math.degrees(float(live_positions[robot.dof_names.index(name)]))
        for name in ARM_TARGETS
    }

    rear_lifts = [after_lift[n][1] - before_lift[n][1] for n in ("RearLeftWheel", "RearRightWheel")]
    front_lifts = [after_lift[n][1] - before_lift[n][1] for n in ("FrontLeftWheel", "FrontRightWheel")]
    mean_rear_lift = sum(rear_lifts) * 0.5
    mean_front_lift = sum(front_lifts) * 0.5
    arrival_error = abs(stop_position[2] - ROBOT_TARGET_Z)
    arms_reached = all(abs(arm_angles[n] - t) < 3.0 for n, t in ARM_TARGETS.items())
    lift_pass = mean_rear_lift >= 0.025 and mean_rear_lift > mean_front_lift + 0.012
    depth_stop_ok = stop_reason == "depth_trigger"
    passed = depth_stop_ok and arrival_error < 0.15 and arms_reached and lift_pass

    report = {
        "passed": passed,
        "assets": {
            "parking": str(PARKING_SOURCE_USD),
            "vehicle": str(VEHICLES_USD),
            "robot": str(ROBOT_USD),
            "test_stage": str(OUTPUT_USD),
        },
        "vehicle": "Sedan",
        "keep_drivetrain": KEEP_DRIVETRAIN,
        "sphere_wheels": SPHERE_WHEELS,
        "parking_bay": "A7",
        "robot_start_xyz_m": start_robot,
        "robot_stop_xyz_m": stop_position,
        "robot_final_xyz_m": final_robot,
        "robot_target_z_m": ROBOT_TARGET_Z,
        "arrival_error_m": arrival_error,
        "sedan_start_xyz_m": start_sedan,
        "sedan_final_xyz_m": final_sedan,
        "initial_wheel_centers_m": initial_wheels,
        "before_lift_wheel_centers_m": before_lift,
        "after_lift_wheel_centers_m": after_lift,
        "rear_wheel_lift_m": rear_lifts,
        "front_wheel_lift_m": front_lifts,
        "mean_rear_lift_m": mean_rear_lift,
        "mean_front_lift_m": mean_front_lift,
        "arm_angles_deg": arm_angles,
        "depth_baseline_m": detector.baseline,
        "depth_threshold_m": detector.threshold,
        "depth_drop_margin_m": DROP_MARGIN,
        "depth_stop_value_m": detector.trigger_value,
        "stop_step": detector.trigger_step,
        "stop_reason": stop_reason,
        "depth_trace": depth_trace,
        "checks": {
            "depth_stop_triggered": depth_stop_ok,
            "robot_arrived": arrival_error < 0.15,
            "arms_reached_targets": arms_reached,
            "rear_wheels_lifted": lift_pass,
        },
    }
    REPORT_JSON.write_text(json.dumps(report, indent=2, ensure_ascii=False), encoding="utf-8")

    print(f"TEST_PASSED={passed}", flush=True)
    print(f"DEPTH_STOP_REASON={stop_reason}", flush=True)
    print(f"DEPTH_BASELINE_M={detector.baseline}", flush=True)
    print(f"ROBOT_ARRIVAL_ERROR_M={arrival_error:.6f}", flush=True)
    print(f"MEAN_REAR_LIFT_M={mean_rear_lift:.6f}", flush=True)
    print(f"MEAN_FRONT_LIFT_M={mean_front_lift:.6f}", flush=True)
    print(f"REPORT={REPORT_JSON}", flush=True)

    timeline.stop()
    app.update()
    return report


def main():
    _restart_with_isaac_python()
    from isaacsim import SimulationApp

    gui = "--gui" in sys.argv[1:]
    app = SimulationApp({"headless": not gui})
    try:
        build_test_stage()
        report = run_test(app)
        if gui:
            while app.is_running():
                app.update()
        elif not report["passed"]:
            raise RuntimeError(f"depth-stop lift test failed; see {REPORT_JSON}")
    except Exception as exc:
        write_exception_report(exc)
        print(f"TEST_EXCEPTION={type(exc).__name__}: {exc}", flush=True)
        raise
    finally:
        app.close()


if __name__ == "__main__":
    main()
```

- [ ] **Step 2: 좀비 프로세스 확인 후 헤드리스 실행**

Run:
```bash
ps aux | grep -i isaac | grep -v grep
cd /home/rokey/cobot3_ws/isaacpjt/Isaac_envo && python3 depth_stop_lift_test.py --sphere-wheels
```
Expected: 콘솔에 `TEST_PASSED=`, `DEPTH_STOP_REASON=`, `ROBOT_ARRIVAL_ERROR_M=`,
`MEAN_REAR_LIFT_M=`, `REPORT=...depth_stop_lift_test_report.json` 출력.

**`TEST_PASSED=True`가 나오지 않아도 이 스텝 자체는 실패가 아니다** — 스펙의 최소 성공
기준은 "예외 없이 끝까지 돌고 `depth_trace`가 남는 것"이다. `stop_reason=timeout`이면
`depth_trace`의 `roi_min_depth_m` 값들을 보고 `DROP_MARGIN`이나 ROI를 조정할 근거로 삼는다
(리포트에 원인 분석을 남긴다). `TEST_EXCEPTION`이 나오면(특히 `_depth_to_hw` 관련 shape
오류) Task 2의 `depth_axis_order` 판정을 다시 확인한다.

- [ ] **Step 3: 리포트 JSON에 예상 필드가 모두 있는지 확인**

Run:
```bash
cd /home/rokey/cobot3_ws/isaacpjt/Isaac_envo && python3 -c "
import json
r = json.load(open('depth_stop_lift_test_report.json'))
required = ['passed', 'stop_reason', 'depth_baseline_m', 'depth_stop_value_m',
            'stop_step', 'depth_trace', 'mean_rear_lift_m', 'mean_front_lift_m',
            'arrival_error_m', 'checks']
missing = [k for k in required if k not in r]
print('missing:', missing)
print('depth_trace frames:', len(r['depth_trace']))
print('stop_reason:', r['stop_reason'])
"
```
Expected: `missing: []`, `depth_trace frames:` 1 이상.

- [ ] **Step 4: 좀비 프로세스 정리**

Run: `ps aux | grep -i isaac | grep -v grep` (필요 시 kill)

- [ ] **Step 5: (수동, 선택) GUI로 육안 확인**

Run: `cd /home/rokey/cobot3_ws/isaacpjt/Isaac_envo && python3 depth_stop_lift_test.py --gui --sphere-wheels`

확인할 것: 로봇이 실제로 굴러서 진입하는지, 뒷축 근처에서 멈추는지, 팔이 펴지며 뒷바퀴가
들리는지. 창을 닫아 종료한다.

- [ ] **Step 6: 커밋**

```bash
cd /home/rokey/cobot3_ws
git add isaacpjt/Isaac_envo/depth_stop_lift_test.py isaacpjt/Isaac_envo/depth_stop_lift_test_report.json
git commit -m "feat: drive-and-depth-stop rear-wheel lift test (replaces teleport ingress)"
```

---

## Self-Review Notes

- **스펙 커버리지**: 씬 재사용(Task 3) / cmd_vel 폐루프 진입(Task 4) / ROI+캘리브레이션+
  연속프레임 정지판단(Task 1) / 안전 타임아웃(Task 4 `DEPTH_MAX_STEPS`) / 정지 후 팔 전개·
  판정 지표(Task 4) / `depth_trace` 튜닝 데이터(Task 4) / CLI 4종(Task 3·4) — 스펙의 모든
  섹션에 대응하는 태스크가 있다.
- **센서 리스크**: 스펙이 명시한 "실제로 유효한 값을 내는지 미검증" 리스크를 Task 2에서
  본 구현 전에 격리해 확인하도록 배치했다. Task 4는 Task 2의 실측 결과(`depth_axis_order`)를
  전제로 코드를 쓴다는 점을 단계 설명에 명시했다.
- **타입/시그니처 일관성**: `roi_min_depth(depth_hw, roi_frac=...)`, `DepthStopDetector.update(step, roi_value) -> bool`
  이 Task 1에서 정의된 그대로 Task 2·4에서 쓰인다. `ROBOT_ROOT`/`ROBOT_JOINTS`/`ROBOT_WRAP`/
  `CAM_FRONT` 등 상수는 Task 3에서 정의하고 Task 4가 같은 이름으로 재사용한다.
