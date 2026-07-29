#!/usr/bin/env python3
"""기존 천장 RTX LiDAR를 ROS2 PointCloud2 토픽으로 송출한다.

1번 센서만 먼저 검증:
    python3 run_ceiling_lidar_ros2.py --sensor ceiling_01

두 센서 동시 송출:
    python3 run_ceiling_lidar_ros2.py --sensor both

LiDAR 2대와 카메라 동시 송출:
    python3 run_ceiling_lidar_ros2.py --sensor both --camera

CI/짧은 구동 확인:
    python3 run_ceiling_lidar_ros2.py --sensor both --camera --headless --test-frames 300
"""

from __future__ import annotations

import argparse
import os
import traceback
from dataclasses import dataclass
from pathlib import Path

from isaac_runtime import restart_with_isaac_python


ROOT = Path(__file__).resolve().parent
STAGE_PATH = ROOT / "parking_environment.usd"
WRITER_NAME = "RtxLidarROS2PublishPointCloud"
CAMERA_PRIM_PATH = "/World/OverviewCamera"
CAMERA_NAMESPACE = "parking/camera/overview"
CAMERA_FRAME_ID = "parking_camera_overview_optical_frame"
CAMERA_RESOLUTION = (1280, 720)


@dataclass(frozen=True)
class SensorContract:
    zone: str
    prim_path: str
    topic: str
    frame_id: str


SENSORS = {
    "ceiling_01": SensorContract(
        zone="west",
        prim_path="/World/Sensors/CeilingLidarWest/sensor",
        topic="parking/lidar/ceiling_01/points",
        frame_id="ceiling_lidar_01_link",
    ),
    "ceiling_02": SensorContract(
        zone="east",
        prim_path="/World/Sensors/CeilingLidarEast/sensor",
        topic="parking/lidar/ceiling_02/points",
        frame_id="ceiling_lidar_02_link",
    ),
}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="천장 RTX LiDAR 2대의 ROS2 PointCloud2 송출"
    )
    parser.add_argument(
        "--sensor",
        choices=("ceiling_01", "ceiling_02", "both", "none"),
        default="both",
        help="검증할 LiDAR. 카메라만 시험할 때는 none을 사용합니다.",
    )
    parser.add_argument(
        "--camera",
        action="store_true",
        help="기존 OverviewCamera의 image_raw와 camera_info도 발행",
    )
    parser.add_argument("--headless", action="store_true", help="GUI 없이 실행")
    parser.add_argument(
        "--test-frames",
        type=int,
        default=0,
        help="0이면 종료할 때까지 실행, 양수이면 해당 프레임 후 정상 종료",
    )
    return parser.parse_args()


def selected_contracts(selection: str) -> list[SensorContract]:
    if selection == "none":
        return []
    if selection == "both":
        return list(SENSORS.values())
    return [SENSORS[selection]]


def create_clock_graph() -> None:
    """모든 센서 메시지와 TF가 공유할 Isaac simulation time을 발행한다."""
    import omni.graph.core as og

    graph_path = "/ParkingSensorsROS2Clock"
    og.Controller.edit(
        {
            "graph_path": graph_path,
            "evaluator_name": "execution",
            "pipeline_stage": og.GraphPipelineStage.GRAPH_PIPELINE_STAGE_SIMULATION,
        },
        {
            og.Controller.Keys.CREATE_NODES: [
                ("OnTick", "omni.graph.action.OnTick"),
                ("ReadSimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
                ("PublishClock", "isaacsim.ros2.bridge.ROS2PublishClock"),
            ],
            og.Controller.Keys.CONNECT: [
                ("OnTick.outputs:tick", "PublishClock.inputs:execIn"),
                (
                    "ReadSimTime.outputs:simulationTime",
                    "PublishClock.inputs:timeStamp",
                ),
            ],
        },
    )


def attach_camera_writers(rep, app):
    """기존 카메라에 RGB Image와 CameraInfo writer를 연결한다."""
    import omni.syntheticdata._syntheticdata as sd
    from isaacsim.ros2.bridge import read_camera_info
    from isaacsim.sensors.camera import Camera

    camera = Camera(
        prim_path=CAMERA_PRIM_PATH,
        name="parking_overview_camera",
        frequency=30,
        resolution=CAMERA_RESOLUTION,
    )
    camera.initialize()
    app.update()
    render_product = camera._render_product_path

    rgb_render_var = sd.SensorType.Rgb.name
    rgb_writer_name = (
        sd.SyntheticData.convert_sensor_type_to_rendervar(rgb_render_var)
        + "ROS2PublishImage"
    )
    rgb_writer = rep.writers.get(rgb_writer_name)
    rgb_writer.initialize(
        frameId=CAMERA_FRAME_ID,
        nodeNamespace=CAMERA_NAMESPACE,
        queueSize=1,
        topicName="image_raw",
    )
    rgb_writer.attach([render_product])

    camera_info, _ = read_camera_info(render_product_path=render_product)
    info_writer = rep.writers.get("ROS2PublishCameraInfo")
    info_writer.initialize(
        frameId=CAMERA_FRAME_ID,
        nodeNamespace=CAMERA_NAMESPACE,
        queueSize=1,
        topicName="camera_info",
        width=camera_info.width,
        height=camera_info.height,
        projectionType=camera_info.distortion_model,
        k=camera_info.k.reshape([1, 9]),
        r=camera_info.r.reshape([1, 9]),
        p=camera_info.p.reshape([1, 12]),
        physicalDistortionModel=camera_info.distortion_model,
        physicalDistortionCoefficients=camera_info.d,
    )
    info_writer.attach([render_product])
    return camera, render_product, [rgb_writer, info_writer]


def main() -> None:
    restart_with_isaac_python(Path(__file__))
    args = parse_args()
    if args.sensor == "none" and not args.camera:
        raise SystemExit("발행할 센서가 없습니다. --camera를 추가하거나 LiDAR를 선택하세요.")

    # ROS2 bridge는 Isaac Sim을 띄우기 전에 선택된 RMW/Domain 환경을 읽는다.
    if not os.environ.get("ROS_DISTRO"):
        print(
            "[ceiling-lidar-ros2] 경고: ROS_DISTRO가 없습니다. "
            "먼저 /opt/ros/<distro>/setup.bash를 source 하세요.",
            flush=True,
        )

    from isaacsim import SimulationApp

    app = SimulationApp(
        {
            "headless": args.headless,
            "enable_motion_bvh": True,
            "width": 1100,
            "height": 700,
        }
    )
    writers = []
    render_products = []
    failure: BaseException | None = None
    try:
        from isaacsim.core.utils.extensions import enable_extension

        enable_extension("isaacsim.ros2.bridge")
        for _ in range(10):
            app.update()

        import omni.replicator.core as rep
        import omni.timeline
        import omni.usd
        from isaacsim.core.api import World

        if not STAGE_PATH.is_file():
            raise FileNotFoundError(
                "주차장 USD가 없습니다. build_parking_environment.py --headless를 "
                f"먼저 실행하세요: {STAGE_PATH}"
            )
        if not omni.usd.get_context().open_stage(str(STAGE_PATH)):
            raise RuntimeError(f"주차장 Stage 열기 실패: {STAGE_PATH}")
        for _ in range(30):
            app.update()

        stage = omni.usd.get_context().get_stage()
        create_clock_graph()
        contracts = selected_contracts(args.sensor)
        for contract in contracts:
            prim = stage.GetPrimAtPath(contract.prim_path)
            if not prim or prim.GetTypeName() != "OmniLidar":
                raise RuntimeError(
                    f"RTX LiDAR Prim을 찾지 못했습니다: {contract.prim_path}"
                )
            zone_attr = prim.GetAttribute("sensor:coverageZone")
            zone = str(zone_attr.Get()) if zone_attr and zone_attr.Get() else ""
            if zone and zone != contract.zone:
                raise RuntimeError(
                    f"센서 구역 계약 불일치: {contract.prim_path} "
                    f"expected={contract.zone}, actual={zone}"
                )

            render_product = rep.create.render_product(
                contract.prim_path,
                (1, 1),
                name=f"{contract.frame_id}_render_product",
            )
            writer = rep.writers.get(WRITER_NAME)
            writer.initialize(
                topicName=contract.topic,
                frameId=contract.frame_id,
            )
            writer.attach([render_product])
            writers.append(writer)
            render_products.append(render_product)

        camera = None
        if args.camera:
            camera_prim = stage.GetPrimAtPath(CAMERA_PRIM_PATH)
            if not camera_prim or camera_prim.GetTypeName() != "Camera":
                raise RuntimeError(f"고정 카메라 Prim을 찾지 못했습니다: {CAMERA_PRIM_PATH}")
            camera, camera_render_product, camera_writers = attach_camera_writers(
                rep, app
            )
            render_products.append(camera_render_product)
            writers.extend(camera_writers)

        world = World(stage_units_in_meters=1.0, set_defaults=False)
        timeline = omni.timeline.get_timeline_interface()
        timeline.play()
        world.reset()

        print(
            "[ceiling-lidar-ros2] READY "
            f"ROS_DOMAIN_ID={os.environ.get('ROS_DOMAIN_ID', '0')}",
            flush=True,
        )
        for contract in contracts:
            print(
                f"  /{contract.topic}  frame_id={contract.frame_id} "
                f"prim={contract.prim_path}",
                flush=True,
            )
        if args.camera:
            print(
                f"  /{CAMERA_NAMESPACE}/image_raw  frame_id={CAMERA_FRAME_ID}",
                flush=True,
            )
            print(
                f"  /{CAMERA_NAMESPACE}/camera_info  frame_id={CAMERA_FRAME_ID}",
                flush=True,
            )
        print("  /clock", flush=True)

        frame = 0
        while app.is_running():
            world.step(render=True)
            frame += 1
            if args.test_frames > 0 and frame >= args.test_frames:
                break
    except KeyboardInterrupt:
        print("\n[ceiling-lidar-ros2] 사용자 요청으로 종료합니다.", flush=True)
    except BaseException as exc:
        failure = exc
        traceback.print_exc()
    finally:
        for writer in writers:
            try:
                writer.detach()
            except Exception:
                pass
        try:
            app.close()
        except Exception:
            pass
    if failure is not None:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
