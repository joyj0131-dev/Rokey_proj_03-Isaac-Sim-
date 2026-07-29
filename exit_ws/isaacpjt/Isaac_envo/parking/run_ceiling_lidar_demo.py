#!/usr/bin/env python3
"""천장 RTX LiDAR 2대의 실제 포인트클라우드 출력과 시각화.

GUI:
    python3 run_ceiling_lidar_demo.py

자동 검증:
    python3 run_ceiling_lidar_demo.py --headless-test
"""

import argparse
import os
import sys
import traceback
from pathlib import Path

from isaac_runtime import restart_with_isaac_python


ROOT = Path(__file__).resolve().parent
STAGE_PATH = ROOT / "parking_environment.usd"
LIDAR_CONFIG = "SICK_multiScan136"
LIDAR_ZONES = ("west", "east")


def point_count_and_summary(payload):
    """Replicator 버전에 따라 dict/array로 오는 실제 XYZ 배열을 찾는다."""
    if payload is None:
        return 0, "None"
    if isinstance(payload, dict):
        summary = {}
        candidates = []
        for key, value in payload.items():
            shape = getattr(value, "shape", None)
            summary[str(key)] = tuple(int(v) for v in shape) if shape is not None else type(value).__name__
            if shape is not None and len(shape) >= 1:
                candidates.append((int(shape[0]), str(key)))
        preferred = [item for item in candidates if item[1].lower() in {"data", "points", "pointcloud", "xyz"}]
        count = max(preferred or candidates, default=(0, ""))[0]
        return count, summary
    shape = getattr(payload, "shape", None)
    count = int(shape[0]) if shape is not None and len(shape) >= 1 else int(len(payload))
    return count, tuple(int(v) for v in shape) if shape is not None else type(payload).__name__


def main() -> None:
    restart_with_isaac_python(Path(__file__))
    parser = argparse.ArgumentParser()
    parser.add_argument("--headless-test", action="store_true")
    args = parser.parse_args()

    from isaacsim import SimulationApp

    app = SimulationApp({
        "headless": args.headless_test,
        "enable_motion_bvh": True,
        "width": 1100,
        "height": 700,
    })
    failure = None
    try:
        import omni.timeline
        import omni.usd
        from isaacsim.core.api import World
        from isaacsim.sensors.rtx import LidarRtx

        if not STAGE_PATH.is_file():
            raise FileNotFoundError(
                f"주차장 USD가 없습니다. build_parking_environment.py --headless를 먼저 실행하세요: {STAGE_PATH}"
            )
        if not omni.usd.get_context().open_stage(str(STAGE_PATH)):
            raise RuntimeError(f"주차장 Stage 열기 실패: {STAGE_PATH}")
        for _ in range(30):
            app.update()
        if not args.headless_test:
            # 새 세션의 기본 뷰포트 카메라는 이 Y-up 주차장 스테이지에 맞춰
            # 있지 않아 화면이 뒤집힌 것처럼 보인다. build_parking_environment.py가
            # 저장해 둔 /World/OverviewCamera로 전환해 바로잡는다.
            from isaacsim.core.utils.viewports import set_active_viewport_camera

            set_active_viewport_camera("/World/OverviewCamera")
            for _ in range(4):
                app.update()
        stage = omni.usd.get_context().get_stage()
        lidar_prims = [
            prim for prim in stage.Traverse()
            if prim.GetTypeName() == "OmniLidar"
            and str(prim.GetPath()).startswith("/World/Sensors/")
        ]
        if len(lidar_prims) != 2:
            raise RuntimeError(
                f"천장 OmniLidar 2대를 찾지 못했습니다: "
                f"{[(str(prim.GetPath()), prim.GetTypeName()) for prim in lidar_prims]}"
            )
        lidar_paths = {}
        for prim in lidar_prims:
            zone_attr = prim.GetAttribute("sensor:coverageZone")
            zone = str(zone_attr.Get()) if zone_attr and zone_attr.Get() else ""
            if zone not in LIDAR_ZONES:
                # 이전 파일 호환: Prim 이름에서 동/서를 판정한다.
                zone = "west" if "west" in str(prim.GetPath()).lower() else "east"
            lidar_paths[zone] = str(prim.GetPath())
        if set(lidar_paths) != set(LIDAR_ZONES):
            raise RuntimeError(f"천장 LiDAR coverageZone 오류: {lidar_paths}")
        print(f"[ceiling-lidar] 실제 OmniLidar 경로: {lidar_paths}", flush=True)

        world = World(stage_units_in_meters=1.0, set_defaults=False)
        lidars = {}
        for zone, path in lidar_paths.items():
            lidars[zone] = world.scene.add(
                LidarRtx(
                    prim_path=path,
                    name=f"ceiling_lidar_{zone}",
                    config_file_name=LIDAR_CONFIG,
                )
            )

        timeline = omni.timeline.get_timeline_interface()
        timeline.play()
        world.reset()
        annotator = "IsaacExtractRTXSensorPointCloudNoAccumulator"
        for lidar in lidars.values():
            lidar.attach_annotator(annotator)
            if not args.headless_test:
                # 뷰포트에 센서 포인트클라우드를 직접 겹쳐 표시한다.
                lidar.enable_visualization()

        for _ in range(240):
            world.step(render=True)

        report = {}
        for zone, lidar in lidars.items():
            frame = lidar.get_current_frame()
            points = frame.get(annotator)
            point_count, payload_summary = point_count_and_summary(points)
            report[zone] = point_count
            print(
                f"[ceiling-lidar] {zone} payload={payload_summary} points={point_count}",
                flush=True,
            )
            if point_count <= 0:
                raise RuntimeError(f"{zone} 천장 LiDAR 포인트 없음: {frame.keys()}")
        print(f"[ceiling-lidar] 실제 포인트클라우드 검증 통과: {report}", flush=True)
        print("[ceiling-lidar] GUI의 색 점들이 바닥·차량·벽에 맞아 생성되는 실제 반환점입니다.", flush=True)

        if args.headless_test:
            os._exit(0)
        print("[ceiling-lidar] 창을 닫으면 종료됩니다.", flush=True)
        while app.is_running():
            world.step(render=True)
    except Exception as exc:
        failure = exc
        print("[ceiling-lidar] 시험 실패", file=sys.stderr, flush=True)
        traceback.print_exc()
    finally:
        app.close(wait_for_replicator=False)
    if failure is not None:
        raise failure


if __name__ == "__main__":
    main()
