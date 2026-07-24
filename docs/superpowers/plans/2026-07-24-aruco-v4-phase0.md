# ArUco v4 Phase 0 (측정) + v4 러너 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** v4 주차장에 로봇 4대를 띄우고, GT가 아닌 **휠 엔코더 오도메트리**를 발행하며, 카메라 높이·마커 밀도·카메라 대수를 사용자가 **GUI로 직접 보고 결정**할 수 있는 probe 3종을 만든다.

**Architecture:** 좌표 규약 반전(입차=z양수 vs 에셋 라벨 반대)을 순수 파이썬 모듈 `site_map_v4.py` 한 곳에 가둔다. Isaac 러너는 이 모듈을 import 해 로봇 4대를 배치하고, 휠 관절 각속도 → `cmd_vel_from_wheel_velocities()` → 적분으로 오도메트리를 만든다. probe는 측정 결과를 씬 바닥에 그려 눈으로 확인 가능하게 한다.

**Tech Stack:** Isaac Sim 5.1 (Python 3.11), ROS 2 Humble (Python 3.10), pxr USD, numpy, pytest, colcon

## Global Constraints

- 좌표 규약: **ENTRY(입차) = z 양수, EXIT(출차) = z 음수.** v4 에셋 라벨(`IN`=z음수)과 반대이며, 반전은 `site_map_v4.py` 에서만 처리한다. 다른 파일은 z 부호나 에셋 라벨을 직접 참조하지 않는다.
- GT는 **제어 입력 금지**, 계측(채점) 기준으로만 허용. 조용한 GT 폴백 금지.
- 대상 에셋: `isaacpjt/Isaac_envo/parking/parking_environment_v4.usd` (ASCII USD, 마커 16장, `DICT_5X5_100`, `aruco:codeSize = 0.19444445`).
- 기존 러너 `dock_lift_handoff_runner_v2.py`(로봇 2대)를 **개조하지 않는다.** v4 전용 러너를 새로 만든다.
- USD에 **절대경로를 굽지 않는다.**
- Isaac 실행 전후로 **좀비 프로세스를 확인·정리**한다: `ps -eo pid,args --no-headers | grep -E "/kit/kit|python\.sh|isaacsim"`
- Isaac 물리는 GPU/TGS/CCD, `PHYSICS_HZ = 120.0`, `RENDER_HZ = 60.0`, 렌더 640x400.
- 차량·로봇 스폰 높이: 실내 바닥 윗면 y=0.0 기준(로봇 y=0.06).

---

## File Structure

| 파일 | 책임 |
|---|---|
| `src/parkbot_aruco/parkbot_aruco/site_map_v4.py` | **신규.** 에셋 라벨↔프로세스 역할 매핑(z 반전), 로봇↔도크 배정, 마커 규약 검증. 순수 파이썬, USD/ROS 의존 없음 |
| `src/parkbot_aruco/test/test_site_map_v4.py` | **신규.** 위 모듈 단위 테스트 (colcon test 에 편입) |
| `isaacpjt/Isaac_envo/parking_v4_runner.py` | **신규.** v4 씬 + 로봇 4대 + 휠 오도메트리 + probe 진입점 |
| `isaacpjt/Isaac_envo/parking_v4_runner.sh` | **신규.** Isaac 내부 Humble libs 환경 런처 |
| `isaacpjt/Isaac_envo/v4_probes.py` | **신규.** Probe A/B/C 구현 + 바닥 시각화 헬퍼 |

`site_map_v4.py` 를 ROS 패키지에 두는 이유: 순수 파이썬이라 Isaac 없이 테스트되고, 이후 측위·주행 노드가 같은 모듈을 쓰며, `colcon test` 의 기존 단위 테스트 묶음에 편입된다. Isaac 러너는 `sys.path` 삽입으로 import 한다(기존에 `mecanum_drive` 를 쓰는 방식과 동일).

---

### Task 1: `site_map_v4.py` — 좌표 규약 매핑

z 반전을 흡수하는 유일한 지점. Isaac 없이 완전히 테스트된다.

**Files:**
- Create: `src/parkbot_aruco/parkbot_aruco/site_map_v4.py`
- Test: `src/parkbot_aruco/test/test_site_map_v4.py`

**Interfaces:**
- Consumes: 없음 (순수 파이썬)
- Produces:
  - `ENTRY: str = "ENTRY"`, `EXIT: str = "EXIT"`
  - `ROBOTS: tuple[str, ...]` — `("entry_lead", "entry_follow", "exit_lead", "exit_follow")`
  - `SERVES_ROLE: dict[str, str]` — 마커 `serves` → 역할
  - `ROBOT_DOCK_MARKER: dict[str, str]` — 로봇 id → 도크 마커 `serves`
  - `role_of(serves: str) -> str`
  - `expected_z_sign(role: str) -> int` — ENTRY→+1, EXIT→−1
  - `team_of(robot_id: str) -> str`
  - `validate_markers(markers: list[dict]) -> list[str]` — 각 dict 는 `{"serves": str, "z": float}`. 문제 설명 문자열 리스트를 반환하며 빈 리스트면 정상

- [ ] **Step 1: 실패하는 테스트 작성**

Create `src/parkbot_aruco/test/test_site_map_v4.py`:

```python
"""site_map_v4 단위 테스트 — z 반전 규약이 무너지면 여기서 잡힌다."""
import pytest

from parkbot_aruco import site_map_v4 as sm


# v4 USD 실측값 (parking_environment_v4.usd 의 aruco:serves / aruco:position.z)
V4_MARKERS = [
    {"serves": "GATE_IN", "z": -7.075}, {"serves": "GATE_OUT", "z": 7.075},
    {"serves": "W_IN", "z": -7.075},    {"serves": "W_OUT", "z": 7.075},
    {"serves": "D_IN_1", "z": -2.2},    {"serves": "D_OUT_1", "z": 2.2},
    {"serves": "D_IN_2", "z": -2.2},    {"serves": "D_OUT_2", "z": 2.2},
    {"serves": "XS", "z": -6.875},      {"serves": "XN", "z": 6.875},
    {"serves": "A1", "z": -6.875},      {"serves": "A1'", "z": 6.875},
    {"serves": "A2", "z": -6.875},      {"serves": "A2'", "z": 6.875},
    {"serves": "A3", "z": -6.875},      {"serves": "A3'", "z": 6.875},
]


def test_entry_is_positive_z_side():
    """입차는 z 양수. 에셋이 OUT 이라 부르는 것이 우리 ENTRY 다."""
    assert sm.role_of("W_OUT") == sm.ENTRY
    assert sm.role_of("D_OUT_1") == sm.ENTRY
    assert sm.expected_z_sign(sm.ENTRY) == 1


def test_exit_is_negative_z_side():
    assert sm.role_of("W_IN") == sm.EXIT
    assert sm.role_of("A1") == sm.EXIT
    assert sm.expected_z_sign(sm.EXIT) == -1


def test_all_16_v4_markers_have_a_role():
    for m in V4_MARKERS:
        assert sm.role_of(m["serves"]) in (sm.ENTRY, sm.EXIT)


def test_v4_markers_match_convention():
    """실측 z 부호가 배정한 역할과 일치해야 한다."""
    assert sm.validate_markers(V4_MARKERS) == []


def test_validate_detects_flipped_marker():
    bad = [{"serves": "W_OUT", "z": -7.075}]     # ENTRY 인데 z 음수
    problems = sm.validate_markers(bad)
    assert len(problems) == 1
    assert "W_OUT" in problems[0]


def test_unknown_serves_raises():
    with pytest.raises(KeyError):
        sm.role_of("NOPE")


def test_four_robots_map_to_distinct_dock_markers():
    assert len(sm.ROBOTS) == 4
    docks = [sm.ROBOT_DOCK_MARKER[r] for r in sm.ROBOTS]
    assert len(set(docks)) == 4
    for d in docks:
        assert d in sm.SERVES_ROLE


def test_team_assignment():
    assert sm.team_of("entry_lead") == sm.ENTRY
    assert sm.team_of("entry_follow") == sm.ENTRY
    assert sm.team_of("exit_lead") == sm.EXIT
    assert sm.team_of("exit_follow") == sm.EXIT
```

- [ ] **Step 2: 테스트가 실패하는지 확인**

Run: `cd /home/rokey/p3/cobot_ws/src/parkbot_aruco && python3 -m pytest test/test_site_map_v4.py -v`
Expected: FAIL — `ModuleNotFoundError: No module named 'parkbot_aruco.site_map_v4'`

- [ ] **Step 3: 최소 구현 작성**

Create `src/parkbot_aruco/parkbot_aruco/site_map_v4.py`:

```python
#!/usr/bin/env python3
"""v4 주차장 사이트 맵 — 에셋 라벨과 우리 프로세스 용어 사이의 **유일한** 변환 지점.

사용자 규약: 입차(ENTRY) = z 양수, 출차(EXIT) = z 음수.

v4 에셋은 정반대로 라벨링돼 있다. `EntryVehicleWait` 에
`parking:center = (-8.5, 0, -5.5)`, `parking:direction = "inbound"`,
`parking:displayLabel = "입차 차량 대기 구역"` 이 박혀 있어 에셋은 "입차 = z 음수"
라고 말한다. 마커 serves 값(W_IN, D_IN_1 ...)도 같은 규약이다.

우리는 사용자 규약을 채택하므로 이 모듈이 반전을 흡수한다. **다른 코드는 ENTRY/EXIT
만 쓰고 z 부호나 에셋 라벨을 직접 참조하지 않는다.** 팀원이 하루 사이 v2->v3->v4 를
냈으므로 앞으로도 에셋 라벨은 계속 우리와 반대로 말할 것이다.
"""

ENTRY = "ENTRY"
EXIT = "EXIT"

# 에셋 serves -> 우리 프로세스. 에셋의 OUT 계열이 우리 ENTRY(입차)다.
SERVES_ROLE = {
    # z 양수 = 입차
    "GATE_OUT": ENTRY, "W_OUT": ENTRY,
    "D_OUT_1": ENTRY, "D_OUT_2": ENTRY,
    "XN": ENTRY, "A1'": ENTRY, "A2'": ENTRY, "A3'": ENTRY,
    # z 음수 = 출차
    "GATE_IN": EXIT, "W_IN": EXIT,
    "D_IN_1": EXIT, "D_IN_2": EXIT,
    "XS": EXIT, "A1": EXIT, "A2": EXIT, "A3": EXIT,
}

ROBOTS = ("entry_lead", "entry_follow", "exit_lead", "exit_follow")

# 로봇 -> 대기 도크 마커. 도크 마커는 도크 중심과 같은 좌표에 있어,
# 러너가 마커 위치에서 스폰 좌표를 얻는다(좌표 손 입력 금지).
ROBOT_DOCK_MARKER = {
    "entry_lead": "D_OUT_1",
    "entry_follow": "D_OUT_2",
    "exit_lead": "D_IN_1",
    "exit_follow": "D_IN_2",
}


def role_of(serves):
    """마커 serves -> ENTRY | EXIT."""
    try:
        return SERVES_ROLE[serves]
    except KeyError:
        raise KeyError(
            f"알 수 없는 마커 serves={serves!r}. v4 에셋이 바뀌었다면 "
            f"SERVES_ROLE 을 갱신할 것. 알려진 값: {sorted(SERVES_ROLE)}")


def expected_z_sign(role):
    """역할 -> 기대되는 z 부호(+1 / -1)."""
    if role == ENTRY:
        return 1
    if role == EXIT:
        return -1
    raise ValueError(f"알 수 없는 role={role!r}")


def team_of(robot_id):
    """로봇 id -> 소속 팀(ENTRY | EXIT)."""
    try:
        return role_of(ROBOT_DOCK_MARKER[robot_id])
    except KeyError:
        raise KeyError(f"알 수 없는 robot_id={robot_id!r}. 알려진 값: {list(ROBOTS)}")


def validate_markers(markers):
    """마커 실측 z 부호가 배정된 역할과 맞는지 검사한다.

    markers: [{"serves": str, "z": float}, ...]
    반환: 문제 설명 문자열 리스트(빈 리스트면 정상).

    에셋이 갱신돼 좌표 규약이 바뀌면 여기서 큰 소리로 잡힌다.
    """
    problems = []
    for m in markers:
        serves = m["serves"]
        z = float(m["z"])
        role = role_of(serves)
        want = expected_z_sign(role)
        if z == 0.0 or (1 if z > 0 else -1) != want:
            problems.append(
                f"마커 {serves}: 역할 {role} 은 z 부호 {want:+d} 를 기대하는데 "
                f"실제 z={z:+.3f}. 에셋 규약이 바뀌었는지 확인할 것.")
    return problems
```

- [ ] **Step 4: 테스트 통과 확인**

Run: `cd /home/rokey/p3/cobot_ws/src/parkbot_aruco && python3 -m pytest test/test_site_map_v4.py -v`
Expected: PASS — 8 passed

- [ ] **Step 5: 커밋**

```bash
cd /home/rokey/p3/cobot_ws
git add src/parkbot_aruco/parkbot_aruco/site_map_v4.py src/parkbot_aruco/test/test_site_map_v4.py
git commit -m "feat(aruco): v4 사이트맵 — 입차=z양수 규약 반전을 한 곳에 가둠"
```

---

### Task 2: v4 러너 — 로봇 4대 배치 + 물리 스모크

**Files:**
- Create: `isaacpjt/Isaac_envo/parking_v4_runner.py`
- Create: `isaacpjt/Isaac_envo/parking_v4_runner.sh`

**Interfaces:**
- Consumes: `site_map_v4.ROBOTS`, `ROBOT_DOCK_MARKER`, `validate_markers` (Task 1); `mecanum_drive.configure_hub_drives`, `WHEEL_JOINTS`
- Produces:
  - `read_markers(stage) -> dict[str, dict]` — `serves` → `{"id": int, "x": float, "z": float, "yaw": float, "kind": str}`
  - `build_stage(app) -> Usd.Stage` — v4 씬 + 로봇 4대
  - `PARKING_USD: Path`, `ROBOT_USD: Path`
  - 콘솔 토큰 `V4_STAGE_READY`, `V4_MARKERS_OK`, `V4_PHYSICS_TEST=PASS|FAIL`

- [ ] **Step 1: 러너 작성**

Create `isaacpjt/Isaac_envo/parking_v4_runner.py`:

```python
#!/usr/bin/env python3
"""v4 주차장 러너 — 로봇 4대(입차팀/출차팀) + 휠 오도메트리 + probe 진입점.

기존 dock_lift_handoff_runner_v2.py(로봇 2대)는 건드리지 않는다. 검증된 2대 구성이
비교 기준으로 남아야 한다.

좌표 규약은 site_map_v4 가 전담한다(입차=z양수, 에셋 라벨과 반대).

실행: parking_v4_runner.sh [--gui] [--headless-test]
"""
import math
import os
import sys
from pathlib import Path

WORK_DIR = Path(__file__).resolve().parent
REPO_ROOT = WORK_DIR.parent.parent
PARKING_USD = WORK_DIR / "parking" / "parking_environment_v4.usd"
ROBOT_USD = (WORK_DIR.parent / "hwia_parking_robot_final_caster_package"
             / "hwia_depth_cam_mecha_roller_lowered.usd")
ISAAC_PYTHON = Path("/home/rokey/dev_ws/isaac_sim/isaacsim/_build/linux-x86_64/release/python.sh")

sys.path.insert(0, str(REPO_ROOT / "src" / "parkbot_aruco"))
from parkbot_aruco import site_map_v4 as sm   # noqa: E402

RENDER_HZ = 60.0
RENDER_WIDTH = 640
RENDER_HEIGHT = 400
PHYSICS_HZ = 120.0
LINEAR_ACCEL = 0.5
LINEAR_DECEL = 0.8
ANGULAR_ACCEL = 0.8
ROBOT_SPAWN_Y = 0.06


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
    vctx = PhysxSchema.PhysxVehicleContextAPI.Apply(sc)
    vctx.CreateUpdateModeAttr(PhysxSchema.Tokens.velocityChange)
    vctx.CreateVerticalAxisAttr(PhysxSchema.Tokens.posY)
    vctx.CreateLongitudinalAxisAttr(PhysxSchema.Tokens.posZ)


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
    for _ in range(30):
        app.update()

    n_lidar = _disable_sensors(stage)

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

    placed = {r: sm.ROBOT_DOCK_MARKER[r] for r in sm.ROBOTS}
    print(f"V4_STAGE_READY robots={placed} disabled_lidar={n_lidar} "
          f"render={RENDER_WIDTH}x{RENDER_HEIGHT}@{RENDER_HZ:.0f}Hz "
          f"physics={PHYSICS_HZ:.0f}Hz", flush=True)
    return stage


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

    arts = {}
    for robot_id in sm.ROBOTS:
        art = Articulation(f"{robot_prim_path(robot_id)}/base_link")
        art.initialize()
        arts[robot_id] = art

    sys.path.insert(0, str(WORK_DIR))
    from mecanum_drive import configure_hub_drives
    for robot_id in sm.ROBOTS:
        configure_hub_drives(stage, f"{robot_prim_path(robot_id)}/joints")

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

    while app.is_running():
        app.update()
    app.close()


if __name__ == "__main__":
    main()
```

- [ ] **Step 2: 런처 작성**

Create `isaacpjt/Isaac_envo/parking_v4_runner.sh`:

```bash
#!/bin/bash
set -u
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REL=$HOME/dev_ws/isaac_sim/isaacsim/_build/linux-x86_64/release
# Isaac 내부 Humble libs 사용. /opt/ros 소싱 금지.
unset PYTHONPATH AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH
unset FASTRTPS_DEFAULT_PROFILES_FILE FASTDDS_DEFAULT_PROFILES_FILE
export ROS_DISTRO=humble
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-126}"
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export LD_LIBRARY_PATH="$REL/exts/isaacsim.ros2.bridge/humble/lib"
exec "$REL/python.sh" "$SCRIPT_DIR/parking_v4_runner.py" "$@"
```

- [ ] **Step 3: 좀비 확인 후 스모크 실행**

```bash
cd /home/rokey/p3/cobot_ws/isaacpjt/Isaac_envo
ps -eo pid,args --no-headers | grep -E "/kit/kit|python\.sh|isaacsim" | grep -v grep || echo "clean"
chmod +x parking_v4_runner.sh
bash parking_v4_runner.sh --headless-test 2>&1 | grep -E "V4_MARKERS_OK|V4_STAGE_READY|V4_PHYSICS_TEST|Traceback"
```

Expected:
```
V4_MARKERS_OK count=16
V4_STAGE_READY robots={'entry_lead': 'D_OUT_1', ...} disabled_lidar=... 
V4_PHYSICS_TEST=PASS robot_disp={'entry_lead': 0.0, ...}
```

- [ ] **Step 4: 좀비 정리 확인**

Run: `ps -eo pid,args --no-headers | grep -E "/kit/kit|python\.sh|isaacsim" | grep -v grep || echo "clean"`
Expected: `clean`

- [ ] **Step 5: 커밋**

```bash
cd /home/rokey/p3/cobot_ws
git add isaacpjt/Isaac_envo/parking_v4_runner.py isaacpjt/Isaac_envo/parking_v4_runner.sh
git commit -m "feat(isaac): v4 러너 — 로봇 4대 배치, 마커 규약 검증, 물리 스모크"
```

---

### Task 3: 휠 엔코더 오도메트리 + `--odom` 플래그

GT 대신 휠 관절 **각속도**를 적분해 오도메트리를 만든다.

> **함정(DEBUG_LOG 2026-07-21):** 관절 *각도* 차분을 쓰면 연속 회전에서 wrap 이 일어나 27.9배로
> 폭주한다. 반드시 **각속도**(`get_joint_velocities()`)를 쓴다.

**Files:**
- Modify: `isaacpjt/Isaac_envo/parking_v4_runner.py`

**Interfaces:**
- Consumes: `mecanum_drive.cmd_vel_from_wheel_velocities(omegas) -> (vx, vy, wz)`, `WHEEL_JOINTS`
- Produces:
  - `class WheelOdometry` — `__init__(self, x=0.0, z=0.0, yaw=0.0)`, `update(self, vx, vy, wz, dt) -> None`, 속성 `x`, `z`, `yaw`
  - `read_wheel_twist(art, wheel_idx) -> tuple[float, float, float]`
  - `gt_pose_xz_yaw(art) -> tuple[float, float, float]`
  - 러너 인자 `--odom=gt|wheel` (기본 `wheel`)

- [ ] **Step 1: 실패하는 테스트 작성**

`WheelOdometry` 는 순수 적분이라 Isaac 없이 테스트한다. Create `src/parkbot_aruco/test/test_wheel_odometry.py`:

```python
"""휠 오도메트리 적분 단위 테스트."""
import math
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(REPO / "isaacpjt" / "Isaac_envo"))

from wheel_odometry import WheelOdometry     # noqa: E402


def test_forward_integrates_along_plus_z_when_yaw_zero():
    """yaw=0 은 월드 +Z 를 향한다(프로젝트 규약)."""
    od = WheelOdometry()
    od.update(1.0, 0.0, 0.0, 1.0)
    assert math.isclose(od.z, 1.0, abs_tol=1e-9)
    assert math.isclose(od.x, 0.0, abs_tol=1e-9)


def test_left_strafe_integrates_along_plus_x_when_yaw_zero():
    """+Z 를 볼 때 로봇 좌측(+vy)은 월드 +X 다.

    근거: marker_localizer.PoseFilter.predict_body 의 규약
    "전방(+X_body)->월드 +Z, 좌(+Y_body)->월드 +X (yaw=0 기준)".
    바디가 X전방/Y좌/Z상 우수계이고 월드가 Y-up 우수계이므로
    좌 = 상 x 전방 = (+Y_world) x (+Z_world) = +X_world 다.
    """
    od = WheelOdometry()
    od.update(0.0, 1.0, 0.0, 1.0)
    assert math.isclose(od.x, 1.0, abs_tol=1e-9)
    assert math.isclose(od.z, 0.0, abs_tol=1e-9)


def test_matches_calibrated_predict_body():
    """이미 GT 로 캘리브된 marker_localizer.PoseFilter.predict_body 와 일치해야 한다.

    독립적인 오라클로 검증한다 — 구현이 스스로를 정당화하지 못하게 한다.
    """
    import sys as _sys
    _sys.path.insert(0, str(REPO / "src" / "parkbot_aruco"))
    from parkbot_aruco.marker_localizer import PoseFilter

    for vx, vy, wz, yaw0 in [(1.0, 0.0, 0.0, 0.0), (0.0, 1.0, 0.0, 0.0),
                             (0.7, -0.4, 0.0, math.radians(35.0)),
                             (-0.3, 0.9, 0.0, math.radians(-110.0))]:
        od = WheelOdometry(yaw=yaw0)
        od.update(vx, vy, wz, 0.5)
        pf = PoseFilter()
        pf.x, pf.z, pf.yaw = 0.0, 0.0, math.degrees(yaw0)
        pf.predict_body(vx, vy, wz, 0.5)
        assert math.isclose(od.x, pf.x, abs_tol=1e-9), (vx, vy, yaw0)
        assert math.isclose(od.z, pf.z, abs_tol=1e-9), (vx, vy, yaw0)


def test_yaw_accumulates_and_wraps():
    od = WheelOdometry()
    od.update(0.0, 0.0, 1.0, math.pi)          # +pi
    assert math.isclose(abs(od.yaw), math.pi, abs_tol=1e-6)
    od.update(0.0, 0.0, 1.0, math.pi)          # 총 2pi -> 0 근처
    assert math.isclose(od.yaw, 0.0, abs_tol=1e-6)


def test_rotated_then_forward():
    od = WheelOdometry(yaw=math.pi / 2)        # +X 를 향함
    od.update(1.0, 0.0, 0.0, 1.0)
    assert math.isclose(od.x, 1.0, abs_tol=1e-9)
    assert math.isclose(od.z, 0.0, abs_tol=1e-9)


def test_zero_dt_is_noop():
    od = WheelOdometry(x=3.0, z=4.0)
    od.update(9.0, 9.0, 9.0, 0.0)
    assert (od.x, od.z) == (3.0, 4.0)
```

- [ ] **Step 2: 테스트 실패 확인**

Run: `cd /home/rokey/p3/cobot_ws/src/parkbot_aruco && python3 -m pytest test/test_wheel_odometry.py -v`
Expected: FAIL — `ModuleNotFoundError: No module named 'wheel_odometry'`

- [ ] **Step 3: `WheelOdometry` 구현**

Create `isaacpjt/Isaac_envo/wheel_odometry.py`:

```python
#!/usr/bin/env python3
"""휠 엔코더 오도메트리 — 로봇 로컬 twist 를 월드 (x, z, yaw) 로 적분한다.

규약(marker_localizer.PoseFilter.predict_body 와 동일해야 한다):
  전방(+X_body) -> 월드 +Z,  좌(+Y_body) -> 월드 +X   (yaw=0 기준)
  nav yaw ψ 에서  전방 = (sinψ,  cosψ),  좌 = (cosψ, -sinψ)   [(x,z) 평면]

바디가 X전방/Y좌/Z상 우수계이고 월드가 Y-up 우수계이므로
좌 = 상 x 전방 = (+Y_world) x (+Z_world) = +X_world 다.
좌 항의 부호를 뒤집으면 메카넘 횡이동이 통째로 좌우 반전되어 적분된다.

순수 파이썬이라 Isaac 없이 테스트된다.
"""
import math


def _wrap(a):
    """각을 (-pi, pi] 로 접는다."""
    return math.atan2(math.sin(a), math.cos(a))


class WheelOdometry:
    """로봇 로컬 twist 적분기. GT 를 쓰지 않는다."""

    def __init__(self, x=0.0, z=0.0, yaw=0.0):
        self.x = float(x)
        self.z = float(z)
        self.yaw = _wrap(float(yaw))

    def update(self, vx, vy, wz, dt):
        """dt 동안 로컬 twist (vx 전진[m/s], vy 좌측[m/s], wz CCW[rad/s]) 적분."""
        dt = float(dt)
        if dt <= 0.0:
            return
        # 구간 중앙 yaw 로 적분해 1차 오차를 줄인다.
        yaw_mid = self.yaw + 0.5 * float(wz) * dt
        s, c = math.sin(yaw_mid), math.cos(yaw_mid)
        fwd = float(vx) * dt
        left = float(vy) * dt
        # 전방=(sinψ, cosψ), 좌=(cosψ, -sinψ). predict_body 와 동일.
        self.x += fwd * s + left * c
        self.z += fwd * c - left * s
        self.yaw = _wrap(self.yaw + float(wz) * dt)
```

- [ ] **Step 4: 테스트 통과 확인**

Run: `cd /home/rokey/p3/cobot_ws/src/parkbot_aruco && python3 -m pytest test/test_wheel_odometry.py -v`
Expected: PASS — 5 passed

- [ ] **Step 5: 러너에 오도메트리 배선**

Modify `isaacpjt/Isaac_envo/parking_v4_runner.py` — `main()` 의 `configure_hub_drives` 루프 **뒤에** 다음을 추가한다:

```python
    from mecanum_drive import WHEEL_JOINTS, cmd_vel_from_wheel_velocities
    from wheel_odometry import WheelOdometry

    wheel_idx = {r: {w: arts[r].dof_names.index(j) for w, j in WHEEL_JOINTS.items()}
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

    odom_mode = "wheel"
    for a in sys.argv[1:]:
        if a.startswith("--odom="):
            odom_mode = a.split("=", 1)[1]
    if odom_mode not in ("gt", "wheel"):
        raise SystemExit(f"--odom 은 gt 또는 wheel 이어야 합니다: {odom_mode!r}")

    odom = {}
    for r in sm.ROBOTS:
        gx, gz, gyaw = gt_pose_xz_yaw(arts[r])
        odom[r] = WheelOdometry(x=gx, z=gz, yaw=gyaw)   # 초기 자세만 GT 로 정렬
    print(f"V4_ODOM_MODE={odom_mode}", flush=True)
```

- [ ] **Step 6: 스모크 재실행**

```bash
cd /home/rokey/p3/cobot_ws/isaacpjt/Isaac_envo
bash parking_v4_runner.sh --headless-test 2>&1 | grep -E "V4_ODOM_MODE|V4_PHYSICS_TEST|Traceback"
```

Expected:
```
V4_ODOM_MODE=wheel
V4_PHYSICS_TEST=PASS robot_disp={...}
```

- [ ] **Step 7: `/robot_<id>/odom` ROS 발행 배선**

측위 노드(Phase 2)가 소비할 오도메트리를 실제로 내보낸다. `--odom=gt` 는 비교·디버깅용이며
`wheel` 이 기본이다.

Modify `parking_v4_runner.py` — Step 5 에서 추가한 `print(f"V4_ODOM_MODE=...")` **뒤에** 추가:

```python
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

    def publish_odom():
        """--odom 모드에 따라 휠 오도메트리 또는 GT 를 발행한다."""
        for r in sm.ROBOTS:
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
            odom_pub[r].publish(msg)

    def step_odometry(dt):
        """휠 각속도를 읽어 각 로봇 오도메트리를 적분한다."""
        for r in sm.ROBOTS:
            vx, vy, wz = read_wheel_twist(arts[r], wheel_idx[r])
            odom[r].update(vx, vy, wz, dt)
```

그리고 `main()` 끝의 유휴 루프를 아래로 교체한다:

```python
    prev_sim = timeline.get_current_time()
    while app.is_running():
        app.update()
        now_sim = timeline.get_current_time()
        dt = min(0.1, max(0.0, now_sim - prev_sim))
        prev_sim = now_sim
        rclpy.spin_once(ros_node, timeout_sec=0.0)
        step_odometry(dt)
        publish_odom()
    app.close()
```

- [ ] **Step 8: odom 토픽 확인**

터미널 [A]: `bash parking_v4_runner.sh` (대기 상태로 유지)

터미널 [B] (시스템 ROS 2, 같은 도메인):
```bash
export ROS_DOMAIN_ID=126 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE=$HOME/.ros/fastdds_whitelist.xml
source /opt/ros/humble/setup.bash
ros2 topic echo /robot_entry_lead/odom --once
```

Expected: `pose.position` 이 entry_lead 도크 좌표(대략 x=-3.2, z=+2.2) 근처인 Odometry 메시지 1건.
확인 후 터미널 [A] 를 Ctrl+C 로 종료하고 좀비를 정리한다.

- [ ] **Step 9: 커밋**

```bash
cd /home/rokey/p3/cobot_ws
git add isaacpjt/Isaac_envo/wheel_odometry.py isaacpjt/Isaac_envo/parking_v4_runner.py \
        src/parkbot_aruco/test/test_wheel_odometry.py
git commit -m "feat(isaac): 휠 엔코더 오도메트리 + /odom 발행 + --odom 플래그"
```

---

### Task 4: Probe B — 오도메트리 드리프트 시각화

GT(흰 선)와 휠 오도메트리(노란 선) 두 궤적을 바닥에 그려 드리프트를 **눈으로** 보인다.

**Files:**
- Create: `isaacpjt/Isaac_envo/v4_probes.py`
- Modify: `isaacpjt/Isaac_envo/parking_v4_runner.py` (`--probe=B` 분기)

**Interfaces:**
- Consumes: Task 3 의 `WheelOdometry`, `read_wheel_twist`, `gt_pose_xz_yaw`
- Produces:
  - `draw_trail(stage, path, points, color, width=0.04, y=0.02) -> None` — `points` 는 `[(x, z), ...]`
  - `write_report(name, payload) -> Path` — `isaacpjt/Isaac_envo/probe_reports/<name>.json`
  - 콘솔 토큰 `PROBE_B_RESULT`

- [ ] **Step 1: 시각화 헬퍼 작성**

Create `isaacpjt/Isaac_envo/v4_probes.py`:

```python
#!/usr/bin/env python3
"""v4 Phase 0 probe 공용 — 바닥 시각화와 리포트 저장.

측정 결과를 씬 안에 그려서 GUI 로 바로 이해되게 하는 것이 목적이다.
"""
import json
from pathlib import Path

REPORT_DIR = Path(__file__).resolve().parent / "probe_reports"

WHITE = (1.0, 1.0, 1.0)
YELLOW = (1.0, 0.85, 0.1)
GREEN = (0.15, 0.85, 0.25)
RED = (0.9, 0.15, 0.15)


def write_report(name, payload):
    """probe 결과 JSON 저장. 반환값은 저장 경로."""
    REPORT_DIR.mkdir(parents=True, exist_ok=True)
    p = REPORT_DIR / f"{name}.json"
    p.write_text(json.dumps(payload, indent=2, ensure_ascii=False), encoding="utf-8")
    return p


def draw_trail(stage, path, points, color, width=0.04, y=0.02):
    """(x, z) 점열을 바닥 위 얇은 큐브 띠로 그린다.

    BasisCurves 대신 큐브를 쓰는 이유: 뷰포트 기본 설정에서 곡선은 두께가
    렌더러 의존이라 잘 안 보인다. 큐브는 어디서나 동일하게 보인다.
    """
    import math
    from pxr import Gf, UsdGeom
    root = UsdGeom.Xform.Define(stage, path)
    for i in range(len(points) - 1):
        x0, z0 = points[i]
        x1, z1 = points[i + 1]
        dx, dz = x1 - x0, z1 - z0
        seg = math.hypot(dx, dz)
        if seg < 1e-6:
            continue
        cube = UsdGeom.Cube.Define(stage, f"{path}/seg_{i:04d}")
        cube.GetSizeAttr().Set(1.0)
        xf = UsdGeom.Xformable(cube)
        xf.ClearXformOpOrder()
        xf.AddTranslateOp().Set(Gf.Vec3d((x0 + x1) * 0.5, y, (z0 + z1) * 0.5))
        xf.AddRotateYOp().Set(math.degrees(math.atan2(dx, dz)))
        xf.AddScaleOp().Set(Gf.Vec3f(width, 0.004, seg))
        cube.GetDisplayColorAttr().Set([Gf.Vec3f(*color)])
    return root


def draw_marker_dot(stage, path, x, z, color, size=0.18, y=0.02):
    """측정점 표식 하나."""
    from pxr import Gf, UsdGeom
    cube = UsdGeom.Cube.Define(stage, path)
    cube.GetSizeAttr().Set(1.0)
    xf = UsdGeom.Xformable(cube)
    xf.ClearXformOpOrder()
    xf.AddTranslateOp().Set(Gf.Vec3d(x, y, z))
    xf.AddScaleOp().Set(Gf.Vec3f(size, 0.004, size))
    cube.GetDisplayColorAttr().Set([Gf.Vec3f(*color)])
    return cube
```

- [ ] **Step 2: 러너에 Probe B 분기 추가**

Modify `parking_v4_runner.py` — Task 3 Step 7 에서 추가한 `step_odometry` 정의 **뒤, 유휴 루프
앞에** 추가(probe 분기는 유휴 루프보다 먼저 와야 한다):

```python
    probe = None
    for a in sys.argv[1:]:
        if a.startswith("--probe="):
            probe = a.split("=", 1)[1]

    if probe == "B":
        import v4_probes as vp
        from mecanum_drive import slew_twist, wheel_velocities_from_cmd_vel

        target = sm.ROBOTS[0]                      # entry_lead 한 대로 측정
        art = arts[target]
        idx = wheel_idx[target]
        vel_buf = np.zeros(np.asarray(art.get_joint_positions()).reshape(-1).shape,
                           dtype=np.float32)

        def drive(tw, steps):
            cur = (0.0, 0.0, 0.0)
            prev = timeline.get_current_time()
            gt_pts, od_pts = [], []
            for _ in range(steps):
                app.update()
                now = timeline.get_current_time()
                dt = min(0.1, max(0.0, now - prev))
                prev = now
                cur = slew_twist(cur, tw, dt, linear_accel=LINEAR_ACCEL,
                                 linear_decel=LINEAR_DECEL, angular_accel=ANGULAR_ACCEL)
                omegas = wheel_velocities_from_cmd_vel(*cur)
                vel_buf[...] = 0.0
                for w, om in omegas.items():
                    vel_buf[idx[w]] = om
                art.set_joint_velocity_targets(vel_buf)
                vx, vy, wz = read_wheel_twist(art, idx)
                odom[target].update(vx, vy, wz, dt)
                gx, gz, _ = gt_pose_xz_yaw(art)
                gt_pts.append((gx, gz))
                od_pts.append((odom[target].x, odom[target].z))
            return gt_pts, od_pts

        gt_all, od_all = [], []
        for tw, steps in (((0.35, 0.0, 0.0), 420), ((0.0, 0.0, 0.0), 60),
                          ((0.0, 0.35, 0.0), 420), ((0.0, 0.0, 0.0), 60)):
            g, o = drive(tw, steps)
            gt_all += g
            od_all += o

        gx, gz = gt_all[-1]
        ox, oz = od_all[-1]
        err = math.hypot(ox - gx, oz - gz)
        gt_len = sum(math.hypot(gt_all[i + 1][0] - gt_all[i][0],
                                gt_all[i + 1][1] - gt_all[i][1])
                     for i in range(len(gt_all) - 1))
        rate = (err / gt_len * 100.0) if gt_len > 1e-6 else 0.0

        vp.draw_trail(stage, "/World/ProbeB/GT", gt_all, vp.WHITE)
        vp.draw_trail(stage, "/World/ProbeB/Odom", od_all, vp.YELLOW)
        path = vp.write_report("probe_b_odom_drift", {
            "robot": target, "gt_path_len_m": gt_len,
            "final_error_m": err, "drift_rate_pct": rate,
            "gt_end": [gx, gz], "odom_end": [ox, oz],
        })
        print(f"PROBE_B_RESULT gt_len={gt_len:.3f}m final_err={err:.3f}m "
              f"drift={rate:.2f}% report={path.name}", flush=True)
        if headless:
            app.close()
            return
```

- [ ] **Step 3: 헤드리스로 Probe B 실행**

```bash
cd /home/rokey/p3/cobot_ws/isaacpjt/Isaac_envo
bash parking_v4_runner.sh --probe=B 2>&1 | grep -E "PROBE_B_RESULT|Traceback"
```

Expected: `PROBE_B_RESULT gt_len=<약 8~12>m final_err=<수치>m drift=<수치>% report=probe_b_odom_drift.json`

- [ ] **Step 4: GUI 로 궤적 확인**

Run: `bash parking_v4_runner.sh --gui --probe=B`
Expected: 흰 선(GT)과 노란 선(휠 오도메트리)이 바닥에 그려지고, 주행이 진행될수록 두 선이 벌어지는 것이 보인다. 창은 열린 채 유지된다.

- [ ] **Step 5: 커밋**

```bash
cd /home/rokey/p3/cobot_ws
git add isaacpjt/Isaac_envo/v4_probes.py isaacpjt/Isaac_envo/parking_v4_runner.py
git commit -m "feat(probe): Probe B — GT vs 휠 오도메트리 궤적을 바닥에 시각화"
```

---

### Task 5: 카메라 발행(OmniGraph) + Probe C — 렌더 부하

**Files:**
- Modify: `isaacpjt/Isaac_envo/parking_v4_runner.py`

**Interfaces:**
- Consumes: `site_map_v4.ROBOTS`, `robot_prim_path`
- Produces:
  - `find_front_camera(stage, robot_id) -> str` — 카메라 prim 경로
  - `attach_camera_graph(robot_id, cam_path, width=640, height=480) -> None` — `/robot_<id>/image_raw`, `/robot_<id>/camera_info` 발행
  - 러너 인자 `--cameras=N` (0 | 2 | 4, 기본 0)
  - 콘솔 토큰 `V4_CAMERAS`, `PROBE_C_RESULT`

- [ ] **Step 1: 카메라 탐색 + OmniGraph 배선 추가**

Modify `parking_v4_runner.py` — `build_stage` 정의 **뒤에**(모듈 수준) 추가:

```python
def find_front_camera(stage, robot_id):
    """로봇 서브트리에서 전방 카메라 prim 경로를 찾는다.

    에셋에 카메라가 4대 있으므로 이름으로 전방을 고른다. 후보가 없으면
    조용히 넘어가지 않고 실패한다.
    """
    root = stage.GetPrimAtPath(robot_prim_path(robot_id))
    cams = [p for p in Usd.PrimRange(root)
            if p.GetTypeName() == "Camera"]
    if not cams:
        raise RuntimeError(f"{robot_id}: 카메라 prim 을 찾지 못했습니다")
    for p in cams:
        if "front" in p.GetName().lower():
            return str(p.GetPath())
    return str(cams[0].GetPath())


def attach_camera_graph(robot_id, cam_path, width=640, height=480):
    """C++ OmniGraph 로 image_raw + camera_info 를 발행한다.

    Python rclpy 로 이미지를 퍼블리시하면 Isaac 루프가 죽는다(ARUCO_PLAN 전제).
    camera_info 는 ROS2CameraHelper 의 type 이 아니라 별도 ROS2CameraInfoHelper 노드다
    (DEBUG_LOG 2026-07-21).
    """
    import omni.graph.core as og
    ns = f"/robot_{robot_id}"
    og.Controller.edit(
        {"graph_path": f"/Graphs/cam_{robot_id}", "evaluator_name": "push"},
        {
            og.Controller.Keys.CREATE_NODES: [
                ("tick", "omni.graph.action.OnPlaybackTick"),
                ("render", "isaacsim.core.nodes.IsaacCreateRenderProduct"),
                ("rgb", "isaacsim.ros2.bridge.ROS2CameraHelper"),
                ("info", "isaacsim.ros2.bridge.ROS2CameraInfoHelper"),
            ],
            og.Controller.Keys.CONNECT: [
                ("tick.outputs:tick", "render.inputs:execIn"),
                ("render.outputs:execOut", "rgb.inputs:execIn"),
                ("render.outputs:execOut", "info.inputs:execIn"),
                ("render.outputs:renderProductPath", "rgb.inputs:renderProductPath"),
                ("render.outputs:renderProductPath", "info.inputs:renderProductPath"),
            ],
            og.Controller.Keys.SET_VALUES: [
                ("render.inputs:cameraPrim", [cam_path]),
                ("render.inputs:width", width),
                ("render.inputs:height", height),
                ("rgb.inputs:type", "rgb"),
                ("rgb.inputs:topicName", f"{ns}/image_raw"),
                ("rgb.inputs:frameId", f"robot_{robot_id}/cam"),
                ("info.inputs:topicName", f"{ns}/camera_info"),
                ("info.inputs:frameId", f"robot_{robot_id}/cam"),
            ],
        },
    )
```

그리고 파일 상단 import 에 `from pxr import Usd` 를 추가한다(모듈 수준에서는 불가하므로 `find_front_camera` 안에서 `from pxr import Usd` 로 지역 import 한다).

- [ ] **Step 2: `--cameras=N` 배선과 Probe C 추가**

Modify `parking_v4_runner.py` — `V4_ODOM_MODE` 출력 **뒤에** 추가:

```python
    n_cams = 0
    for a in sys.argv[1:]:
        if a.startswith("--cameras="):
            n_cams = int(a.split("=", 1)[1])
    if n_cams not in (0, 2, 4):
        raise SystemExit(f"--cameras 는 0, 2, 4 중 하나여야 합니다: {n_cams}")

    # 2대일 때는 팀당 리드에만 단다.
    cam_robots = {0: (), 2: ("entry_lead", "exit_lead"), 4: sm.ROBOTS}[n_cams]
    for r in cam_robots:
        attach_camera_graph(r, find_front_camera(stage, r))
    for _ in range(30):
        app.update()
    print(f"V4_CAMERAS n={n_cams} robots={list(cam_robots)}", flush=True)

    if probe == "C":
        import time as _time
        import v4_probes as vp
        for _ in range(120):                 # 워밍업
            app.update()
        t_wall = _time.monotonic()
        t_sim = timeline.get_current_time()
        for _ in range(600):
            app.update()
        rtf = ((timeline.get_current_time() - t_sim)
               / max(_time.monotonic() - t_wall, 1e-6))
        path = vp.write_report(f"probe_c_rtf_{n_cams}cam", {
            "cameras": n_cams, "robots": list(cam_robots), "rtf": rtf})
        print(f"PROBE_C_RESULT cameras={n_cams} rtf={rtf:.3f} report={path.name}",
              flush=True)
        if headless:
            app.close()
            return
```

- [ ] **Step 3: 카메라 0/2/4 대로 RTF 측정**

```bash
cd /home/rokey/p3/cobot_ws/isaacpjt/Isaac_envo
for n in 0 2 4; do
  bash parking_v4_runner.sh --probe=C --cameras=$n 2>&1 | grep -E "PROBE_C_RESULT|Traceback"
done
```

Expected: 세 줄의 `PROBE_C_RESULT cameras=<n> rtf=<수치>`. 기준선은 로봇 2대·카메라 0대에서 0.305~0.362 였다.

- [ ] **Step 4: 좀비 확인**

Run: `ps -eo pid,args --no-headers | grep -E "/kit/kit|python\.sh|isaacsim" | grep -v grep || echo "clean"`
Expected: `clean`

- [ ] **Step 5: 커밋**

```bash
cd /home/rokey/p3/cobot_ws
git add isaacpjt/Isaac_envo/parking_v4_runner.py
git commit -m "feat(isaac): 카메라 OmniGraph 발행 + Probe C(카메라 대수별 RTF)"
```

---

### Task 6: Probe A — 검출 창 측정 + 바닥 띠 시각화

로봇을 마커 앞 여러 거리에 세워 검출 성공 여부를 재고, 검출 가능 구간을 바닥에 띠로 그린다.

**Files:**
- Modify: `isaacpjt/Isaac_envo/parking_v4_runner.py`
- Modify: `isaacpjt/Isaac_envo/v4_probes.py`

**Interfaces:**
- Consumes: `aruco_pose` (기존 `src/parkbot_aruco/parkbot_aruco/aruco_pose.py`), `v4_probes.draw_marker_dot`, `draw_trail`
- Produces:
  - `v4_probes.draw_band(stage, path, x0, x1, z, color, width=0.6, y=0.015) -> None`
  - 러너 인자 `--probe=A`, `--cam-height=<m>`, `--hold-sec=<s>`
  - 콘솔 토큰 `PROBE_A_RESULT`

- [ ] **Step 1: Isaac 파이썬에서 cv2.aruco 사용 가능한지 확인**

ARUCO_PLAN 은 두 인터프리터의 cv2 aruco API 가 다르다고 기록했다. 먼저 확인한다.

```bash
cd /home/rokey/p3/cobot_ws/isaacpjt/Isaac_envo
/home/rokey/dev_ws/isaac_sim/isaacsim/_build/linux-x86_64/release/python.sh -c \
"import cv2; print('cv2', cv2.__version__); import cv2.aruco as A; \
print('ArucoDetector' if hasattr(A,'ArucoDetector') else 'legacy detectMarkers')"
```

Expected: cv2 버전과 API 종류가 출력된다.
- 출력이 나오면 → Step 2 로 진행(Isaac 내부에서 검출).
- `ModuleNotFoundError` 면 → 이 Task 를 중단하고 사용자에게 보고한다. 대안은 카메라 토픽을 외부 ROS 노드로 보내 검출하는 것이며, 그 경우 Task 6 은 재설계가 필요하다.

- [ ] **Step 2: 띠 그리기 헬퍼 추가**

Modify `isaacpjt/Isaac_envo/v4_probes.py` — 파일 끝에 추가:

```python
def draw_band(stage, path, x0, x1, z, color, width=0.6, y=0.015):
    """검출 가능 구간을 바닥 띠로 그린다. x0~x1 구간, z 중심."""
    from pxr import Gf, UsdGeom
    cube = UsdGeom.Cube.Define(stage, path)
    cube.GetSizeAttr().Set(1.0)
    xf = UsdGeom.Xformable(cube)
    xf.ClearXformOpOrder()
    xf.AddTranslateOp().Set(Gf.Vec3d((x0 + x1) * 0.5, y, z))
    xf.AddScaleOp().Set(Gf.Vec3f(abs(x1 - x0), 0.004, width))
    cube.GetDisplayColorAttr().Set([Gf.Vec3f(*color)])
    return cube
```

- [ ] **Step 3: 러너에 Probe A 추가**

Modify `parking_v4_runner.py` — Probe C 분기 **뒤에** 추가:

```python
    if probe == "A":
        import v4_probes as vp
        sys.path.insert(0, str(REPO_ROOT / "src" / "parkbot_aruco"))
        from parkbot_aruco import aruco_pose

        hold_sec = 3.0
        cam_h = None
        for a in sys.argv[1:]:
            if a.startswith("--hold-sec="):
                hold_sec = float(a.split("=", 1)[1])
            if a.startswith("--cam-height="):
                cam_h = float(a.split("=", 1)[1])

        target = "entry_lead"
        art = arts[target]
        markers = read_markers(stage)
        ref = markers[sm.ROBOT_DOCK_MARKER[target]]      # 자기 도크 마커를 본다

        if cam_h is not None:
            from pxr import Gf, UsdGeom
            cam = stage.GetPrimAtPath(find_front_camera(stage, target))
            UsdGeom.Xformable(cam).AddTranslateOp(
                UsdGeom.XformOp.PrecisionDouble, "camHeightProbe"
            ).Set(Gf.Vec3d(0.0, cam_h - 0.09, 0.0))      # 기준 0.09 에서의 증분

        import omni.replicator.core as rep
        cam_path = find_front_camera(stage, target)
        rp = rep.create.render_product(cam_path, (640, 480))
        rgb_annot = rep.AnnotatorRegistry.get_annotator("rgb")
        rgb_annot.attach([rp])

        results = []
        for i in range(16):
            d = 0.6 + 0.1 * i                            # 마커 앞 0.6~2.1 m
            art.set_world_poses(
                np.array([[ref["x"], ROBOT_SPAWN_Y, ref["z"] - d]]),
                np.array([[1.0, 0.0, 0.0, 0.0]]))
            for _ in range(int(hold_sec * RENDER_HZ)):
                app.update()
            frame = rgb_annot.get_data()
            img = (np.asarray(frame)[:, :, :3]
                   if frame is not None and len(frame) else None)
            det = aruco_pose.detect(img) if img is not None else []
            ok = any(int(x.marker_id) == ref["id"] for x in det)
            results.append({"distance_m": d, "detected": bool(ok)})
            vp.draw_marker_dot(stage, f"/World/ProbeA/dot_{i:02d}",
                               ref["x"], ref["z"] - d,
                               vp.GREEN if ok else vp.RED)

        hits = [r["distance_m"] for r in results if r["detected"]]
        d_min, d_max = (min(hits), max(hits)) if hits else (0.0, 0.0)
        if hits:
            vp.draw_band(stage, "/World/ProbeA/band",
                         ref["x"], ref["x"], ref["z"] - d_max, vp.GREEN)
        path = vp.write_report(
            f"probe_a_window_h{(cam_h or 0.09):.2f}",
            {"camera_height_m": cam_h or 0.09, "marker": ref, "samples": results,
             "d_min": d_min, "d_max": d_max, "window_m": max(0.0, d_max - d_min)})
        print(f"PROBE_A_RESULT h={(cam_h or 0.09):.2f} d_min={d_min:.2f} "
              f"d_max={d_max:.2f} window={max(0.0, d_max - d_min):.2f}m "
              f"report={path.name}", flush=True)
        if headless:
            app.close()
            return
```

> `aruco_pose.detect(img)` 의 반환 원소는 `marker_id` 속성을 가진다. Step 1 에서 확인한 cv2
> API 종류와 무관하게 `aruco_pose` 가 두 API 를 모두 흡수하므로 호출부는 동일하다.

- [ ] **Step 4: 카메라 높이 3종으로 실행**

```bash
cd /home/rokey/p3/cobot_ws/isaacpjt/Isaac_envo
for h in 0.09 0.12 0.15; do
  bash parking_v4_runner.sh --probe=A --cam-height=$h --hold-sec=0.2 2>&1 \
    | grep -E "PROBE_A_RESULT|Traceback"
done
```

Expected: 세 줄의 `PROBE_A_RESULT h=<높이> d_min=.. d_max=.. window=..m`. 기준선은 높이 0.09 에서 창 1.1~1.4 m(폭 0.30 m)였다.

- [ ] **Step 5: GUI 로 확인**

Run: `bash parking_v4_runner.sh --gui --probe=A --cam-height=0.09`
Expected: 마커 앞으로 초록/빨강 측정점이 늘어서고, 검출 가능 구간이 초록 띠로 보인다.

- [ ] **Step 6: 커밋**

```bash
cd /home/rokey/p3/cobot_ws
git add isaacpjt/Isaac_envo/parking_v4_runner.py isaacpjt/Isaac_envo/v4_probes.py
git commit -m "feat(probe): Probe A — 카메라 높이별 검출 창 측정 및 바닥 띠 시각화"
```

---

### Task 7: 결정 게이트 — 사용자와 함께 수치 검토

**Files:** 없음 (측정 결과 검토)

- [ ] **Step 1: 전체 단위 테스트 재실행**

```bash
cd /home/rokey/p3/cobot_ws
colcon test --packages-select parkbot_aruco --event-handlers console_direct+ 2>&1 | tail -20
```

Expected: 실패 0

- [ ] **Step 2: 결정 규칙 적용**

`probe_reports/` 의 JSON 세 종류를 모아 아래를 계산한다.

```
최장 무마커 구간 × 드리프트율  <  검출 창 폭 × 0.5
```

v4 최장 무마커 구간은 **7.2 m**(도크→인계장, `D_OUT_1`→`W_OUT`, Δx=−5.3 / Δz=+4.875)다.

- [ ] **Step 3: 사용자에게 보고하고 결정 받기**

다음 세 가지를 수치와 함께 제시한다.
1. 카메라 높이 (Probe A: 높이별 창 폭)
2. 마커 추가 필요 여부와 위치 (Probe A + B: 창 폭 vs 드리프트)
3. 카메라 대수 (Probe C: RTF)

결정 후 Phase 1(지도 생성 + 필요 시 마커 보강) 계획을 새로 작성한다.

---

## Notes for the implementer

- **Isaac 실행은 느리다**(RTF 약 0.3). 각 probe 실행은 수 분 걸릴 수 있다. 백그라운드로 돌리고 로그를 폴링하라.
- **Isaac 은 종료가 걸려 좀비가 남을 수 있다.** 매 실행 후 확인하라. 좀비가 있으면 이후 모든 실행이 느려지거나 `another kit process is locking it` 으로 실패한다.
- `--gui` 실행은 사용자가 직접 볼 것이므로 창을 닫지 말고 유지한다. 헤드리스에서만 `app.close()` 한다.
- 로그에 "완료" 가 찍혔다고 프로세스가 죽은 것이 아니다(DEBUG_LOG 2026-07-19).
