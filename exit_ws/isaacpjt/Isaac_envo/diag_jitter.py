#!/usr/bin/env python3
"""진단 전용: 리프트 테스트 씬에서 무엇이 떨리는지 물체별로 수치화한다.

눈으로 본 "떨린다"를 두 가지로 분해해서 잰다:
  - drift : 시작→끝 순수 변위 (한 방향으로 밀려남)
  - path  : 프레임별 이동량의 총합 (왔다갔다 한 거리까지 포함)
  - 떨림비 = path / drift. 이 값이 크면 "많이 움직였지만 제자리" = 진동이다.

구간을 나눠서 재기 때문에 언제 떨리기 시작하는지도 드러난다.
  settle : 물리 시작 직후 정착 (팔 안 움직임)
  arm    : 팔 전개 중
  hold   : 팔 전개 완료 후 유지

원본 파일은 건드리지 않고 parking_robot_rear_lift_test 의 스테이지 빌더만 재사용한다.

실행:
    python3 diag_jitter.py
"""

from __future__ import annotations

import json
import math
import os
import sys
from pathlib import Path

WORK_DIR = Path(__file__).resolve().parent
ISAAC_PYTHON = Path(
    "/home/rokey/dev_ws/isaac_sim/isaacsim/_build/linux-x86_64/release/python.sh"
)
OUT_JSON = WORK_DIR / "diag_jitter_report.json"

SETTLE_FRAMES = 120
ARM_FRAMES = 180
HOLD_FRAMES = 180


def _restart_with_isaac_python() -> None:
    if os.environ.get("CARB_APP_PATH"):
        return
    if not ISAAC_PYTHON.is_file():
        raise FileNotFoundError(ISAAC_PYTHON)
    os.execv(
        str(ISAAC_PYTHON),
        [str(ISAAC_PYTHON), str(Path(__file__).resolve()), *sys.argv[1:]],
    )


def _metrics(samples):
    """위치 시계열 -> drift / path / 떨림비."""
    if len(samples) < 2:
        return {"drift_m": 0.0, "path_m": 0.0, "jitter_ratio": 0.0, "max_step_m": 0.0}
    drift = math.dist(samples[0], samples[-1])
    steps = [math.dist(a, b) for a, b in zip(samples, samples[1:])]
    path = sum(steps)
    return {
        "drift_m": drift,
        "path_m": path,
        # drift가 0에 가까우면 비율이 발산하므로 하한을 둔다.
        "jitter_ratio": path / max(drift, 1e-4),
        "max_step_m": max(steps),
    }


def main() -> None:
    _restart_with_isaac_python()

    from isaacsim import SimulationApp

    app = SimulationApp({"headless": True})
    try:
        sys.path.insert(0, str(WORK_DIR))
        import parking_robot_rear_lift_test as lift

        import omni.physx
        import omni.timeline
        import omni.usd
        from isaacsim.core.api import World

        lift.build_test_stage()
        context = omni.usd.get_context()
        context.open_stage(str(lift.OUTPUT_USD))
        for _ in range(30):
            app.update()
        stage = context.get_stage()
        physx = omni.physx.get_physx_interface()

        # 감시 대상: 팀원 주차 차량 12대 / 테스트 Sedan / 로봇 본체
        targets = {}
        for group in ("Parked", "HandoffQueue"):
            root = stage.GetPrimAtPath(f"/World/Parking/ParkingVehicles/{group}")
            if not root.IsValid():
                continue
            for child in root.GetChildren():
                targets[f"teammate/{child.GetName()}"] = str(child.GetPath())
        targets["test/Sedan"] = "/World/VehicleAsset/Vehicles/Sedan"
        targets["robot/base_link"] = lift.ROBOT_ROOT

        def sample():
            out = {}
            for label, path in targets.items():
                try:
                    v = physx.get_rigidbody_transformation(path)
                    if v and v.get("ret_val", True):
                        out[label] = tuple(float(x) for x in v["position"])
                except Exception:
                    pass
            return out

        world = World(stage_units_in_meters=1.0, set_defaults=False)
        timeline = omni.timeline.get_timeline_interface()
        timeline.play()
        world.reset()

        series = {label: [] for label in targets}
        phase_bounds = {}

        def run_phase(name, frames, on_step=None):
            start = len(next(iter(series.values())))
            for i in range(frames):
                if on_step:
                    on_step(i, frames)
                world.step(render=False)
                snap = sample()
                for label in targets:
                    if label in snap:
                        series[label].append(snap[label])
            phase_bounds[name] = (start, len(next(iter(series.values()))))

        # 팔 구동을 위한 articulation 핸들은 리프트 테스트 내부 함수에 묶여 있어
        # 여기서는 재현하지 않는다. 팔을 움직이지 않는 조건에서의 떨림만 본다.
        run_phase("settle", SETTLE_FRAMES)
        run_phase("arm_window", ARM_FRAMES)
        run_phase("hold", HOLD_FRAMES)
        timeline.pause()

        report = {"phases": {}, "note": "팔 미구동 조건. 순수 정착/접촉 떨림만 측정."}
        for phase, (a, b) in phase_bounds.items():
            rows = {}
            for label, vals in series.items():
                if len(vals) >= b:
                    rows[label] = _metrics(vals[a:b])
            report["phases"][phase] = rows

        OUT_JSON.write_text(json.dumps(report, indent=2), encoding="utf-8")

        for phase in ("settle", "arm_window", "hold"):
            rows = report["phases"].get(phase, {})
            if not rows:
                continue
            print(f"\n===== {phase} =====", flush=True)
            print(f"  {'대상':<26} {'drift(m)':>10} {'path(m)':>10} {'떨림비':>9} {'max step':>10}")
            for label, m in sorted(
                rows.items(), key=lambda kv: -kv[1]["path_m"]
            ):
                print(
                    f"  {label:<26} {m['drift_m']:>10.5f} {m['path_m']:>10.5f} "
                    f"{m['jitter_ratio']:>9.1f} {m['max_step_m']:>10.6f}",
                    flush=True,
                )
        print(f"\n[diag] 리포트: {OUT_JSON}", flush=True)
    finally:
        app.close()


if __name__ == "__main__":
    main()
