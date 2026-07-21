#!/usr/bin/env python3
"""Isaac Sim RTX LiDAR들을 찾아 월드 포인트클라우드를 ``.npy``로 저장한다.

일반 Python으로 실행하면 Isaac Sim의 ``python.sh``로 자동 재실행한다. 설치된
Isaac Sim 5.1 공식 RTX LiDAR 테스트에서 검증하는
``IsaacCreateRTXLidarScanBuffer`` annotator를 사용하고, 버전이 다르면 point-cloud
annotator로 자동 fallback한다.

저장 좌표는 프로젝트의 ROS map 규약이다::

    ros_x = usd_x, ros_y = -usd_z, ros_z = usd_y

실행 예::

    python3 capture_lidar.py --headless
    python3 capture_lidar.py --live
    python3 capture_lidar.py --stage /path/to/parking_environment.usd \
        --output lidar_pointcloud.npy --headless

결과는 Nx3(x, y, z), intensity를 얻을 수 있으면 Nx4(x, y, z, intensity)이며,
같은 이름의 ``.json`` 파일에는 센서 경로/실제 위치/포인트 수를 기록한다.
"""

import argparse
from collections import deque
from datetime import datetime, timezone
import json
import os
import sys
from pathlib import Path

import numpy as np


SCRIPT_DIR = Path(__file__).resolve().parent
REPO_ROOT = SCRIPT_DIR.parents[3]
PKG_ROOT = SCRIPT_DIR.parent.parent
ISAAC_ROOT = Path(
    os.environ.get(
        "ISAAC_SIM_ROOT",
        "/home/rokey/dev_ws/isaac_sim/isaacsim/_build/linux-x86_64/release",
    )
)
ISAAC_PYTHON = ISAAC_ROOT / "python.sh"
DEFAULT_OUTPUT = SCRIPT_DIR / "lidar_pointcloud.npy"
DEFAULT_LIVE_STATUS = SCRIPT_DIR / "live_occupancy.json"
# 실제 통합 씬의 천장/외벽은 차량 점유 판단 대상이 아니다. 원시 캡처에는
# 보존하되, 실시간 슬롯 판정 입력에서만 고정 구조물 영역을 제외한다.
LIVE_MAX_OBJECT_HEIGHT_M = 3.0
LIVE_SLOT_OUTER_Y_LIMIT_M = 10.5
LIVE_STATE_CONFIRMATIONS = 12
DEFAULT_STAGE_CANDIDATES = (
    # build_integrated_parking_field.py의 최종 산출물(주차장+차량+로봇)을
    # 최우선으로 사용한다. 파일이 아직 없으면 기반 주차장으로 fallback한다.
    Path("/home/rokey/Isaac_envo/parking/parking_robot_field.usd"),
    Path("/home/rokey/Isaac_envo/parking/parking_environment.usd"),
    REPO_ROOT / "isaacpjt/Isaac_envo/parking/parking_environment.usd",
)
ANNOTATOR_SPECS = (
    # full-scan buffer는 intensity 출력도 지원한다. 5.1 로컬 공식 테스트에서
    # data/intensity/transform 필드가 검증되는 annotator다.
    ("IsaacCreateRTXLidarScanBuffer", {"outputIntensity": True}),
    ("IsaacExtractRTXSensorPointCloudNoAccumulator", {}),  # 5.x fallback
    ("IsaacExtractRTXSensorPointCloud", {}),  # 구버전 호환
)


def _restart_with_isaac_python() -> None:
    """일반 Python 프로세스를 Isaac Sim Python으로 교체한다."""
    # Isaac python.sh가 C locale로 시작되면 한글 주석이 든 parking_map.yaml을
    # ASCII로 열어 실패하므로 재실행 전에 UTF-8 모드를 명시한다.
    os.environ.setdefault("PYTHONUTF8", "1")
    os.environ.setdefault("PYTHONIOENCODING", "utf-8")
    if os.environ.get("CARB_APP_PATH"):
        return
    if not ISAAC_PYTHON.is_file():
        raise FileNotFoundError(
            "Isaac Sim python.sh를 찾을 수 없습니다: "
            f"{ISAAC_PYTHON} (필요하면 ISAAC_SIM_ROOT를 지정하세요)"
        )
    print(f"[lidar] Isaac Sim Python으로 전환: {ISAAC_PYTHON}", flush=True)
    os.execv(
        str(ISAAC_PYTHON),
        [str(ISAAC_PYTHON), str(Path(__file__).resolve()), *sys.argv[1:]],
    )


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--stage",
        type=Path,
        help=("열 USD 파일. 생략하면 build_integrated_parking_field.py가 생성한 "
              "/home/rokey/Isaac_envo/parking/parking_robot_field.usd 사용"),
    )
    parser.add_argument("-o", "--output", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--headless", action="store_true")
    parser.add_argument(
        "--live", action="store_true",
        help="종료할 때까지 최신 scan으로 슬롯 점유를 반복 판정",
    )
    parser.add_argument(
        "--ros2", action="store_true",
        help="각 RTX LiDAR를 ROS 2 PointCloud2 토픽으로 동시에 발행",
    )
    parser.add_argument(
        "--ros-frame-id", default="map",
        help="PointCloud2 frame_id (기본: map, WORLD/ROS 좌표)",
    )
    parser.add_argument(
        "--live-update-frames", type=int, default=6,
        help="실시간 점유 판정 사이의 렌더 프레임 수 (기본: 6)",
    )
    parser.add_argument(
        "--live-history", type=int, default=6,
        help="점유 판정에 합칠 최신 렌더 scan 수 (기본: 6)",
    )
    parser.add_argument(
        "--status-output", type=Path, default=DEFAULT_LIVE_STATUS,
        help="실시간 슬롯 상태 JSON 경로",
    )
    parser.add_argument(
        "--alternate-facing", action="store_true",
        help=("두 번째, 네 번째... LiDAR를 반대편 외곽 천장으로 옮겨 "
              "비대칭 수직 FOV가 반대편 주차열을 덮게 함"),
    )
    parser.add_argument(
        "--warmup-frames", type=int, default=30,
        help="Play 후 데이터 수집 전 렌더 프레임 수 (기본: 30)",
    )
    parser.add_argument(
        "--capture-frames", type=int, default=3,
        help="센서별로 합칠 비어 있지 않은 프레임 수 (기본: 3)",
    )
    parser.add_argument(
        "--max-frames", type=int, default=300,
        help="데이터를 기다릴 최대 렌더 프레임 수 (기본: 300)",
    )
    parser.add_argument(
        "--min-points", type=int, default=1000,
        help="자체 검증에 필요한 최소 총 포인트 수 (기본: 1000)",
    )
    args, _ = parser.parse_known_args()
    for name in (
        "warmup_frames", "capture_frames", "max_frames", "min_points",
        "live_update_frames", "live_history",
    ):
        if getattr(args, name) < 1:
            parser.error(f"--{name.replace('_', '-')} 값은 1 이상이어야 합니다")
    return args


def _resolve_stage(requested: Path | None) -> Path:
    if requested is not None:
        path = requested.expanduser().resolve()
        if not path.is_file():
            raise FileNotFoundError(f"USD stage를 찾을 수 없습니다: {path}")
        return path
    for candidate in DEFAULT_STAGE_CANDIDATES:
        if candidate.is_file():
            return candidate.resolve()
    tried = "\n  - ".join(str(path) for path in DEFAULT_STAGE_CANDIDATES)
    raise FileNotFoundError(
        "parking_environment.usd를 찾지 못했습니다. --stage로 지정하세요."
        f"\n검색 경로:\n  - {tried}"
    )


def _find_lidar_prims(stage):
    root = stage.GetPrimAtPath("/World/Sensors")
    if not root or not root.IsValid():
        raise RuntimeError("stage에 /World/Sensors prim이 없습니다.")
    lidars = [
        prim for prim in stage.Traverse()
        if str(prim.GetPath()).startswith("/World/Sensors/")
        and prim.GetTypeName() == "OmniLidar"
        and prim.HasAPI("OmniSensorGenericLidarCoreAPI")
    ]
    if not lidars:
        raise RuntimeError("/World/Sensors 아래에서 RTX LiDAR를 찾지 못했습니다.")
    return lidars


def _set_world_output(prim) -> None:
    """RTX sensor가 annotator에 USD 월드 좌표를 내도록 설정한다."""
    from pxr import Sdf

    attr = prim.GetAttribute("omni:sensor:Core:outputFrameOfReference")
    if not attr or not attr.IsValid():
        attr = prim.CreateAttribute(
            "omni:sensor:Core:outputFrameOfReference", Sdf.ValueTypeNames.Token
        )
    attr.Set("WORLD")


def _sensor_pose_ros(prim) -> list[float]:
    from pxr import Usd, UsdGeom

    matrix = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(
        Usd.TimeCode.Default()
    )
    usd_x, usd_y, usd_z = matrix.ExtractTranslation()
    return [float(usd_x), float(-usd_z), float(usd_y)]


def _place_lidar_for_row(prim, world_x: float, world_z: float) -> None:
    """전방 비대칭 스캔이 지정 주차열을 덮도록 월드 위치를 조정한다."""
    from pxr import Gf, Usd, UsdGeom

    xform = UsdGeom.Xformable(prim)
    world = xform.ComputeLocalToWorldTransform(Usd.TimeCode.Default())
    position = world.ExtractTranslation()
    world.SetTranslateOnly(Gf.Vec3d(world_x, float(position[1]), world_z))
    parent = UsdGeom.Xformable(prim.GetParent()).ComputeLocalToWorldTransform(
        Usd.TimeCode.Default()
    )
    local = world * parent.GetInverse()
    xform.ClearXformOpOrder()
    xform.AddTransformOp().Set(local)


def _payload_array(payload, key: str) -> np.ndarray | None:
    """Annotator 결과의 top-level 또는 info 배열을 numpy로 꺼낸다."""
    if not isinstance(payload, dict):
        return None
    value = payload.get(key)
    if value is None and isinstance(payload.get("info"), dict):
        value = payload["info"].get(key)
    if value is None:
        return None
    return np.asarray(value)


def _frame_to_ros(payload) -> np.ndarray:
    """WORLD/USD Y-up annotator frame을 ROS map 좌표 Nx3/Nx4로 바꾼다."""
    points = _payload_array(payload, "data")
    if points is None or points.size == 0:
        return np.empty((0, 3), dtype=np.float32)
    points = np.asarray(points, dtype=np.float32).reshape(-1, 3)
    finite = np.isfinite(points).all(axis=1)
    points = points[finite]
    ros_points = np.column_stack((points[:, 0], -points[:, 2], points[:, 1]))

    intensity = _payload_array(payload, "intensity")
    if intensity is not None:
        intensity = np.asarray(intensity).reshape(-1)
        if intensity.size == finite.size:
            intensity = intensity[finite].astype(np.float32, copy=False)
            ros_points = np.column_stack((ros_points, intensity))
    return np.asarray(ros_points, dtype=np.float32)


def _make_annotator(rep, render_product_path):
    errors = []
    for name, init_params in ANNOTATOR_SPECS:
        try:
            annotator = rep.AnnotatorRegistry.get_annotator(name)
            annotator.initialize(**init_params)
            annotator.attach([render_product_path])
            return name, annotator
        except Exception as exc:  # 버전별 registry 차이를 진단 로그에 보존
            errors.append(f"{name}: {exc}")
    raise RuntimeError("RTX LiDAR point-cloud annotator 연결 실패:\n" + "\n".join(errors))


def verify_capture(output: Path, minimum_points: int) -> np.ndarray:
    """저장 파일을 다시 열어 shape, 유한값, 점 개수를 검증한다."""
    points = np.load(output, allow_pickle=False)
    if points.ndim != 2 or points.shape[1] not in (3, 4):
        raise RuntimeError(f"포인트클라우드 shape 오류: {points.shape}, Nx3/Nx4 필요")
    if len(points) < minimum_points:
        raise RuntimeError(
            f"포인트가 너무 적습니다: {len(points):,} < {minimum_points:,}. "
            "Play/렌더링 또는 LiDAR 방향을 확인하세요."
        )
    if not np.isfinite(points).all():
        raise RuntimeError("포인트클라우드에 NaN 또는 inf가 남아 있습니다.")
    return points


def _create_live_dashboard(slot_ids):
    """Isaac Sim 안에 슬롯 상태가 색으로 바뀌는 실시간 패널을 만든다."""
    import omni.ui as ui

    window = ui.Window("실시간 주차 점유 현황", width=760, height=330)
    slot_widgets = {}
    with window.frame:
        with ui.VStack(spacing=8, style={"background_color": 0xFF181818}):
            summary = ui.Label(
                "LiDAR 데이터 대기 중...",
                height=34,
                alignment=ui.Alignment.CENTER,
                style={"color": 0xFFFFFFFF, "font_size": 20},
            )
            for row_name in ("A", "B"):
                ui.Label(
                    f"{row_name}열",
                    height=24,
                    style={"color": 0xFFCCCCCC, "font_size": 17},
                )
                with ui.HStack(spacing=6, height=86):
                    for slot_id in (s for s in slot_ids if s.startswith(row_name)):
                        with ui.ZStack(width=ui.Fraction(1), height=82):
                            tile = ui.Rectangle(
                                style={"background_color": 0xFF3A3A3A,
                                       "border_radius": 6}
                            )
                            label = ui.Label(
                                f"{slot_id}\n대기 중",
                                alignment=ui.Alignment.CENTER,
                                style={"color": 0xFFFFFFFF, "font_size": 16},
                            )
                        slot_widgets[slot_id] = (tile, label)
    window.visible = True
    print("[live] Isaac Sim 실시간 점유판을 열었습니다.", flush=True)
    return window, summary, slot_widgets


def _update_live_dashboard(dashboard, statuses, results) -> None:
    if dashboard is None:
        return
    _, summary, slot_widgets = dashboard
    occupied = sorted(slot_id for slot_id, status in statuses.items()
                      if status == "OCCUPIED")
    summary.text = (
        f"점유 {len(occupied)}/{len(statuses)}  |  "
        f"{', '.join(occupied) if occupied else '점유 차량 없음'}"
    )
    for slot_id, status in statuses.items():
        tile, label = slot_widgets[slot_id]
        occupied_now = status == "OCCUPIED"
        tile.style = {
            "background_color": 0xFF4849E3 if occupied_now else 0xFF008300,
            "border_radius": 6,
        }
        label.text = (
            f"{slot_id}\n{'점유' if occupied_now else '공석'}\n"
            f"{results[slot_id]['point_count']} pt"
        )


def _run_live_occupancy(args, app, resources, stage_path) -> None:
    """최신 LiDAR scan만 사용해 이미지 없이 슬롯 상태를 계속 판정한다."""
    sys.path.insert(0, str(PKG_ROOT))
    from parking_control.core.graph import ParkingMap
    from parking_control.core.slot_occupancy_detector import detect

    parking_map = ParkingMap.load(PKG_ROOT / "config" / "parking_map.yaml")
    slot_ids = list(parking_map.nodes_of_kind("slot"))
    dashboard = None if args.headless else _create_live_dashboard(slot_ids)
    status_path = args.status_output.expanduser().resolve()
    status_path.parent.mkdir(parents=True, exist_ok=True)
    previous = {}
    stable_statuses = {}
    pending_statuses = {}
    pending_counts = {}
    render_frame = 0
    histories = [deque(maxlen=args.live_history) for _ in resources]

    print(
        "[live] 실시간 점유 판단 시작 — 종료: Ctrl+C / "
        f"상태 파일: {status_path}",
        flush=True,
    )
    while app.is_running():
        app.update()
        render_frame += 1
        frames = [_frame_to_ros(annotator.get_data()) for _, annotator in resources]
        if all(len(frame) for frame in frames):
            for history, frame in zip(histories, frames):
                history.append(frame[:, :3])
        if render_frame % args.live_update_frames:
            continue
        if any(len(history) < args.live_history for history in histories):
            continue

        # 회전식 LiDAR는 렌더 프레임마다 서로 다른 방위각 조각이 오므로
        # 최근 프레임을 한 회전 scan처럼 합쳐 순간 누락을 막는다.
        occupancy_points = np.concatenate(
            [frame for history in histories for frame in history], axis=0
        )
        occupancy_points = occupancy_points[
            (occupancy_points[:, 2] < LIVE_MAX_OBJECT_HEIGHT_M)
            & (np.abs(occupancy_points[:, 1]) < LIVE_SLOT_OUTER_Y_LIMIT_M)
        ]
        results = detect(occupancy_points, parking_map)
        raw_statuses = {
            slot_id: "OCCUPIED" if result["occupied"] else "EMPTY"
            for slot_id, result in results.items()
        }
        for slot_id, raw_status in raw_statuses.items():
            if slot_id not in stable_statuses:
                stable_statuses[slot_id] = raw_status
                continue
            if raw_status == stable_statuses[slot_id]:
                pending_statuses.pop(slot_id, None)
                pending_counts.pop(slot_id, None)
                continue
            if pending_statuses.get(slot_id) == raw_status:
                pending_counts[slot_id] = pending_counts.get(slot_id, 0) + 1
            else:
                pending_statuses[slot_id] = raw_status
                pending_counts[slot_id] = 1
            if pending_counts[slot_id] >= LIVE_STATE_CONFIRMATIONS:
                stable_statuses[slot_id] = raw_status
                pending_statuses.pop(slot_id, None)
                pending_counts.pop(slot_id, None)
        statuses = dict(stable_statuses)
        _update_live_dashboard(dashboard, statuses, results)
        changed = {
            slot_id: status for slot_id, status in statuses.items()
            if previous.get(slot_id) != status
        }
        if changed:
            for slot_id, status in changed.items():
                print(
                    f"[live] {slot_id}: {status} "
                    f"({results[slot_id]['point_count']} points)",
                    flush=True,
                )
            occupied = sorted(k for k, value in statuses.items() if value == "OCCUPIED")
            print(
                f"[live] 점유 {len(occupied)}/{len(statuses)}: "
                f"{', '.join(occupied) if occupied else '없음'}",
                flush=True,
            )
            previous = statuses

        payload = {
            "updated_at": datetime.now(timezone.utc).isoformat(),
            "stage": str(stage_path),
            "point_count": int(len(occupancy_points)),
            "slots": {
                slot_id: {
                    "status": statuses[slot_id],
                    "point_count": int(results[slot_id]["point_count"]),
                }
                for slot_id in sorted(statuses)
            },
        }
        temporary = status_path.with_suffix(status_path.suffix + ".tmp")
        temporary.write_text(
            json.dumps(payload, indent=2, ensure_ascii=False) + "\n",
            encoding="utf-8",
        )
        os.replace(temporary, status_path)


def _capture(args: argparse.Namespace, app) -> None:
    if args.ros2:
        from isaacsim.core.utils.extensions import enable_extension

        enable_extension("isaacsim.ros2.bridge")
        for _ in range(10):
            app.update()
    import omni.replicator.core as rep
    import omni.timeline
    import omni.usd
    from isaacsim.core.utils.stage import is_stage_loading

    stage_path = _resolve_stage(args.stage)
    output = args.output.expanduser().resolve()
    output.parent.mkdir(parents=True, exist_ok=True)

    context = omni.usd.get_context()
    print(f"[lidar] stage 열기: {stage_path}", flush=True)
    context.open_stage(str(stage_path))
    # 원격 참조 자산이 있는 USD도 완전히 로드된 뒤 prim을 순회한다.
    for _ in range(600):
        app.update()
        if not is_stage_loading():
            break
    else:
        raise RuntimeError("USD stage 로딩이 600 update 안에 끝나지 않았습니다.")
    stage = context.get_stage()
    if stage is None:
        raise RuntimeError(f"USD stage 열기 실패: {stage_path}")

    lidar_prims = _find_lidar_prims(stage)
    resources = []
    ros_writers = []
    sensor_records = []
    for index, prim in enumerate(lidar_prims):
        # 기본 실행은 USD에 저장된 센서 위치를 그대로 존중한다. 시험용
        # 재배치는 사용자가 --alternate-facing을 명시했을 때만 적용한다.
        if args.alternate_facing:
            _place_lidar_for_row(
                prim,
                0.0 if index % 2 else 15.0,
                -11.0 if index % 2 else 0.0,
            )
            print(f"[lidar] 주차열 감시 위치로 재배치: {prim.GetPath()}", flush=True)
        _set_world_output(prim)
        render_product = rep.create.render_product(
            str(prim.GetPath()),
            resolution=(128, 128),
            name=f"ParkingLidarCapture_{index}",
            render_vars=["GenericModelOutput", "RtxSensorMetadata"],
        )
        annotator_name, annotator = _make_annotator(rep, render_product.path)
        if args.ros2:
            topic = f"parking/lidar/ceiling_{index + 1:02d}/points_usd"
            writer = rep.writers.get("RtxLidarROS2PublishPointCloud")
            writer.initialize(topicName=topic, frameId=args.ros_frame_id)
            writer.attach([render_product])
            ros_writers.append(writer)
            print(f"[ros2] /{topic}  frame_id={args.ros_frame_id}", flush=True)
        pose = _sensor_pose_ros(prim)
        resources.append((render_product, annotator))
        sensor_records.append({
            "path": str(prim.GetPath()),
            "position_ros_xyz_m": pose,
            "annotator": annotator_name,
            "frames": [],
        })
        print(
            f"[lidar] 발견: {prim.GetPath()} / ROS xyz="
            f"({pose[0]:.3f}, {pose[1]:.3f}, {pose[2]:.3f})",
            flush=True,
        )

    timeline = omni.timeline.get_timeline_interface()
    try:
        # RTX LiDAR는 Play 중 렌더 프레임이 진행되어야 유효한 scan을 낸다.
        app.update()
        timeline.play()
        for _ in range(args.warmup_frames):
            app.update()

        if args.live:
            _run_live_occupancy(args, app, resources, stage_path)
            return

        for frame_index in range(args.max_frames):
            app.update()
            for record, (_, annotator) in zip(sensor_records, resources):
                if len(record["frames"]) >= args.capture_frames:
                    continue
                frame = _frame_to_ros(annotator.get_data())
                if len(frame):
                    record["frames"].append(frame)
            if all(len(record["frames"]) >= args.capture_frames for record in sensor_records):
                break
        else:
            missing = [
                f"{record['path']} ({len(record['frames'])}/{args.capture_frames} frames)"
                for record in sensor_records
                if len(record["frames"]) < args.capture_frames
            ]
            raise RuntimeError(
                f"{args.max_frames} 프레임 안에 LiDAR 데이터를 충분히 받지 못했습니다: "
                + ", ".join(missing)
            )
    finally:
        timeline.stop()
        for writer in ros_writers:
            writer.detach()
        for render_product, annotator in resources:
            annotator.detach()
            render_product.destroy()

    # intensity 제공 여부가 센서/프레임마다 다르면 공통 Nx3 형식으로 맞춘다.
    all_frames = [frame for record in sensor_records for frame in record["frames"]]
    width = 4 if all(frame.shape[1] == 4 for frame in all_frames) else 3
    points = np.concatenate([frame[:, :width] for frame in all_frames], axis=0)
    np.save(output, points, allow_pickle=False)

    metadata = {
        "stage": str(stage_path),
        "coordinate_frame": "ROS map (x=usd_x, y=-usd_z, z=usd_y), metres",
        "shape": list(points.shape),
        "sensors": [],
    }
    for record in sensor_records:
        frames = record.pop("frames")
        record["captured_frame_point_counts"] = [len(frame) for frame in frames]
        record["total_points"] = int(sum(len(frame) for frame in frames))
        metadata["sensors"].append(record)
    metadata_path = output.with_suffix(".json")
    metadata_path.write_text(
        json.dumps(metadata, indent=2, ensure_ascii=False) + "\n", encoding="utf-8"
    )

    verified = verify_capture(output, args.min_points)
    print(f"[lidar] 저장 및 검증 완료: {output}", flush=True)
    print(f"[lidar] shape={verified.shape}, dtype={verified.dtype}", flush=True)
    print(f"[lidar] 메타데이터: {metadata_path}", flush=True)
    lidar_pos = ";".join(
        f"{record['position_ros_xyz_m'][0]:.3f},{record['position_ros_xyz_m'][1]:.3f}"
        for record in metadata["sensors"]
    )
    print(f"[lidar] visualize_lidar.py --lidar-pos \"{lidar_pos}\"", flush=True)


def main() -> None:
    args = _parse_args()
    _restart_with_isaac_python()

    from isaacsim import SimulationApp

    app = SimulationApp({
        "headless": args.headless,
        "enable_motion_bvh": True,
    })
    try:
        _capture(args, app)
    except KeyboardInterrupt:
        print("\n[live] 사용자 요청으로 종료합니다.", flush=True)
    except Exception as exc:
        # 일부 Kit 버전은 app.close() 이후 Python traceback을 표시하지 않는다.
        print(
            f"[lidar] ERROR: {type(exc).__name__}: {exc}",
            file=sys.stderr,
            flush=True,
        )
        raise
    finally:
        app.close()


if __name__ == "__main__":
    main()
