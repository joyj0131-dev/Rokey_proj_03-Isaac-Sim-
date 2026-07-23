import os

# ROS 2 bridge가 초기화되기 전에 domain을 정해야 한다.
os.environ.setdefault("ROS_DOMAIN_ID", "50")

from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": False})

from isaacsim.core.utils.extensions import enable_extension
enable_extension("isaacsim.ros2.bridge")
simulation_app.update()

from pathlib import Path
import random
import sys
import time

import numpy as np
import omni.usd
from pxr import PhysxSchema, Usd, UsdGeom, UsdPhysics

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32

from isaacsim.core.api import World
from isaacsim.core.api.objects import DynamicCuboid, VisualCuboid
from isaacsim.core.api.tasks import BaseTask
from isaacsim.core.api.materials.physics_material import PhysicsMaterial
from isaacsim.core.prims import SingleGeometryPrim
from isaacsim.robot.manipulators.grippers import ParallelGripper
from isaacsim.robot.manipulators.manipulators import SingleManipulator

_THIS_DIR = Path(__file__).resolve().parent

# rmpflow 인프라 폴더 경로 등록 (인프라 파일 내부 import가 그대로 동작)
RMPFLOW_DIR = str(_THIS_DIR / "rmpflow")
if RMPFLOW_DIR not in sys.path:
    sys.path.insert(0, RMPFLOW_DIR)

from m0609_pick_place_controller import PickPlaceController

# ╔══════════════════════════════════════════════════════════════╗
# ║  A. Task 파라미터                                              ║
# ╚══════════════════════════════════════════════════════════════╝
USD_PATH        = str(_THIS_DIR / "Collected_m0609_camera/Collected_m0609_red_block/m0609_gripper.usd")
ROBOT_PRIM_PATH = "/World/m0609"
EE_LINK_NAME    = "link_6"
GRIPPER_JOINTS  = ["finger_joint", "right_inner_knuckle_joint"]

DRIVE_STIFFNESS = 1e8
DRIVE_DAMPING   = 1e4
DRIVE_MAX_FORCE = 1e8

GRIPPER_OPEN    = [0.0, 0.0]
GRIPPER_CLOSE   = [0.5, 0.5]
GRIPPER_DELTA   = [-0.5, -0.5]

FINGER_STATIC   = 1.8
FINGER_DYNAMIC  = 1.4
CUBE_STATIC     = 1.2
CUBE_DYNAMIC    = 1.0


# ╔══════════════════════════════════════════════════════════════╗
# ║  B. Controller 파라미터                                        ║
# ╚══════════════════════════════════════════════════════════════╝

# ── B-1. 인프라 파일 경로 (RMPFlow가 참조) ────────────────────
M0609_URDF_PATH           = str(_THIS_DIR / "doosan-robot2/urdf/m0609_isaac_sim.urdf")
M0609_DESCRIPTION_PATH    = str(_THIS_DIR / "rmpflow/m0609_description.yaml")
M0609_RMPFLOW_CONFIG_PATH = str(_THIS_DIR / "rmpflow/m0609_rmpflow_common.yaml")

# ── B-2. Pick & Place 동작 파라미터 ───────────────────────────
CUBE_SIZE = 0.05
CUBE_Z = CUBE_SIZE / 2.0

# 큐브 두 개는 먼저 공중 staging 위치에서 대기한다. 매 reset마다 둘 중 하나를
# 선택해 아래 pick 영역의 임의 좌표로 보낸다.
WAIT_POSITIONS = {
    1: np.array([-0.35, 0.55, 0.35]),  # 파란 큐브 대기 위치
    2: np.array([-0.35, 0.40, 0.35]),  # 초록 큐브 대기 위치
}
PICK_X_RANGE = (0.28, 0.42)
PICK_Y_RANGE = (0.28, 0.45)

PLACE_POSITIONS = {
    1: np.array([0.55, -0.35, 0.0]),   # 파란 마커
    2: np.array([0.55, -0.15, 0.0]),   # 초록 마커
}
COLOR_NAMES = {1: "파랑", 2: "초록"}
COLOR_RGB = {
    1: np.array([0.0, 0.0, 1.0]),
    2: np.array([0.0, 1.0, 0.0]),
}

EE_OFFSET = np.array([0.0, 0.0, 0.2])
COLOR_TOPIC = "/color_id"
COLOR_WAIT_EVENT = 4
STAGING_WAIT_SECONDS = 1.5

# ── B-3. 10단계 타이밍 (작을수록 빠름) ────────────────────────
EVENTS_DT = [
    0.008,   # 0. 접근 이동
    0.005,   # 1. 하강
    0.02,    # 2. 그리퍼 닫기 대기
    0.1,     # 3. 그리퍼 닫힘 유지
    0.0025,  # 4. 들어올리기
    0.01,    # 5. Place 위치로 이동
    0.0025,  # 6. 하강
    1,       # 7. 그리퍼 열기 대기
    0.008,   # 8. 상승
    0.08,    # 9. 복귀
]


class ColorIdSubscriber(Node):
    """PC B의 색상 판별 결과(1=파랑, 2=초록)를 받는다."""

    def __init__(self):
        super().__init__("m0609_color_id_subscriber")
        self.color_id = None
        self.accept_messages = False
        self.create_subscription(Int32, COLOR_TOPIC, self._callback, 10)

    def _callback(self, msg):
        if not self.accept_messages:
            return
        if msg.data not in COLOR_NAMES:
            self.get_logger().warning(
                f"유효하지 않은 color_id={msg.data}; 1 또는 2만 허용합니다."
            )
            return
        if self.color_id is None:
            self.color_id = int(msg.data)
            self.get_logger().info(
                f"색상 수신: {COLOR_NAMES[self.color_id]} ({self.color_id})"
            )

    def reset_detection(self):
        self.color_id = None
        self.accept_messages = False


# ============================================================
# 유틸
# ============================================================
def find_prim_path_by_name(root_path: str, name: str):
    stage = omni.usd.get_context().get_stage()
    root_prim = stage.GetPrimAtPath(root_path)
    if not root_prim.IsValid():
        return None
    for prim in Usd.PrimRange(root_prim):
        if prim.GetName() == name:
            return str(prim.GetPath())
    return None


def initialize_robot(robot, world):
    robot.initialize()
    robot.gripper.initialize(
        physics_sim_view=world.physics_sim_view,
        articulation_apply_action_func=robot.apply_action,
        get_joint_positions_func=robot.get_joint_positions,
        set_joint_positions_func=robot.set_joint_positions,
        dof_names=robot.dof_names,
    )
    robot.set_joint_positions(np.zeros(robot.num_dof))


# ============================================================
# Task
# ============================================================
class M0609Task(BaseTask):

    def __init__(self, name):
        super().__init__(name=name, offset=None)
        self._task_achieved = False
        self._active_color_id = None
        self._pick_position = None
        self._pick_ready = False

    def set_up_scene(self, scene):
        super().set_up_scene(scene)
        self._load_usd()
        self._discover_links()
        self._setup_physics()
        self._register_robot(scene)
        self._create_scene(scene)
        print("\n  [완료] 씬 구성 성공!\n")

    def _load_usd(self):
        print("\n" + "=" * 60)
        print("[1.LOAD] USD 로드")
        print("=" * 60)
        stage = omni.usd.get_context().get_stage()
        world_prim = stage.GetPrimAtPath("/World")
        if not world_prim.IsValid():
            world_prim = UsdGeom.Xform.Define(stage, "/World").GetPrim()
        world_prim.GetReferences().AddReference(USD_PATH)
        for _ in range(15):
            simulation_app.update()
        print(f"  [OK] {USD_PATH}")

    def _discover_links(self):
        print("\n" + "=" * 60)
        print("[2.DISCOVER] 링크 경로 탐색")
        print("=" * 60)
        self._ee_path = find_prim_path_by_name(ROBOT_PRIM_PATH, EE_LINK_NAME)
        if self._ee_path is None:
            raise RuntimeError(f"'{EE_LINK_NAME}' not found")
        print(f"  EE ({EE_LINK_NAME}) = {self._ee_path}")
        for jn in GRIPPER_JOINTS:
            print(f"  {jn:<35} = {find_prim_path_by_name(ROBOT_PRIM_PATH, jn)}")

    def _setup_physics(self):
        print("\n" + "=" * 60)
        print("[3.PHYSICS] 물리 설정")
        print("=" * 60)
        stage = omni.usd.get_context().get_stage()

        drive_count = 0
        for prim in Usd.PrimRange(stage.GetPrimAtPath(ROBOT_PRIM_PATH)):
            for dt in ["angular", "linear"]:
                drive = UsdPhysics.DriveAPI.Get(prim, dt)
                if drive:
                    drive.GetStiffnessAttr().Set(DRIVE_STIFFNESS)
                    drive.GetDampingAttr().Set(DRIVE_DAMPING)
                    drive.GetMaxForceAttr().Set(DRIVE_MAX_FORCE)
                    drive_count += 1
        print(f"  [OK] drive updated: {drive_count}")

    def _register_robot(self, scene):
        print("\n" + "=" * 60)
        print("[4.REGISTER] 로봇 등록")
        print("=" * 60)
        gripper = ParallelGripper(
            end_effector_prim_path=self._ee_path,
            joint_prim_names=GRIPPER_JOINTS,
            joint_opened_positions=np.array(GRIPPER_OPEN),
            joint_closed_positions=np.array(GRIPPER_CLOSE),
            action_deltas=np.array(GRIPPER_DELTA),
        )
        self._robot = scene.add(
            SingleManipulator(
                prim_path=ROBOT_PRIM_PATH,
                name="m0609_robot",
                end_effector_prim_path=self._ee_path,
                gripper=gripper,
            )
        )
        print(f"  [OK] SingleManipulator: {ROBOT_PRIM_PATH}")

    def _create_scene(self, scene):
        print("\n" + "=" * 60)
        print("[5.SCENE] 작업 환경 구성")
        print("=" * 60)
        cube_material = PhysicsMaterial(
            prim_path="/World/Physics_Materials/cube_material",
            static_friction=CUBE_STATIC,
            dynamic_friction=CUBE_DYNAMIC,
            restitution=0.0,
        )
        self._cubes = {}
        for color_id, cube_name in ((1, "blue_cube"), (2, "green_cube")):
            cube = scene.add(
                DynamicCuboid(
                    prim_path=f"/World/{cube_name}",
                    name=cube_name,
                    position=WAIT_POSITIONS[color_id],
                    scale=np.array([CUBE_SIZE, CUBE_SIZE, CUBE_SIZE]),
                    color=COLOR_RGB[color_id],
                    mass=0.05,
                    physics_material=cube_material,
                )
            )
            self._cubes[color_id] = cube
            self._set_gravity(cube, disabled=True)
            print(
                f"  [OK] {COLOR_NAMES[color_id]} 큐브 대기 @ "
                f"{WAIT_POSITIONS[color_id]}"
            )

        for color_id, marker_name in ((1, "blue_marker"), (2, "green_marker")):
            scene.add(
                VisualCuboid(
                    prim_path=f"/World/{marker_name}",
                    name=marker_name,
                    position=PLACE_POSITIONS[color_id],
                    scale=np.array([0.07, 0.07, 0.001]),
                    color=COLOR_RGB[color_id],
                )
            )
            print(
                f"  [OK] {COLOR_NAMES[color_id]} 마커 @ "
                f"{PLACE_POSITIONS[color_id]}"
            )
        finger_material = PhysicsMaterial(
            prim_path="/World/Physics_Materials/finger_material",
            static_friction=FINGER_STATIC,
            dynamic_friction=FINGER_DYNAMIC,
            restitution=0.0,
        )
        for link_name in ["left_inner_finger", "right_inner_finger"]:
            link_path = find_prim_path_by_name(ROBOT_PRIM_PATH, link_name)
            if link_path:
                SingleGeometryPrim(
                    prim_path=link_path,
                    name=f"{link_name}_geom",
                ).apply_physics_material(finger_material)
                print(f"  [OK] friction: {link_path}")

    @staticmethod
    def _set_gravity(cube, disabled):
        """공중 대기 큐브만 중력의 영향을 받지 않게 한다."""
        stage = omni.usd.get_context().get_stage()
        prim = stage.GetPrimAtPath(cube.prim_path)
        physx_body = PhysxSchema.PhysxRigidBodyAPI.Apply(prim)
        physx_body.CreateDisableGravityAttr().Set(disabled)

    def prepare_random_pick(self):
        """큐브 하나를 고르고 pick 영역의 임의 위치로 이동한다."""
        self._active_color_id = random.choice(tuple(COLOR_NAMES))
        self._pick_position = np.array(
            [
                random.uniform(*PICK_X_RANGE),
                random.uniform(*PICK_Y_RANGE),
                CUBE_Z,
            ]
        )

        active_cube = self._cubes[self._active_color_id]
        active_cube.set_world_pose(position=self._pick_position)
        self._set_gravity(active_cube, disabled=False)
        self._task_achieved = False
        self._pick_ready = True

        print("\n" + "=" * 60)
        print("[RANDOM PICK]")
        print("=" * 60)
        print(f"  선택 큐브(검증용) = {COLOR_NAMES[self._active_color_id]}")
        print(f"  랜덤 pick 위치    = {self._pick_position}")
        print("  실제 place 분기는 내부 색상이 아니라 /color_id로 결정됩니다.")

    def reset_cubes_to_waiting(self):
        """두 큐브를 중력이 꺼진 공중 staging 위치로 되돌린다."""
        for color_id, cube in self._cubes.items():
            self._set_gravity(cube, disabled=True)
            cube.set_linear_velocity(np.zeros(3))
            cube.set_angular_velocity(np.zeros(3))
            cube.set_world_pose(position=WAIT_POSITIONS[color_id])
        self._active_color_id = None
        self._pick_position = None
        self._pick_ready = False
        self._task_achieved = False

    def get_observations(self):
        active_cube = self._cubes[self._active_color_id]
        cube_pos, _ = active_cube.get_world_pose()
        return {
            self._robot.name: {
                "joint_positions": self._robot.get_joint_positions(),
            },
            "active_cube": {
                "position": cube_pos,
                "expected_color_id": self._active_color_id,
            },
        }

    def pre_step(self, control_index, simulation_time):
        # 성공 판정은 메인 루프에서 실제 /color_id 목표점 기준으로 수행한다.
        pass

    def post_reset(self):
        self._robot.gripper.set_joint_positions(
            self._robot.gripper.joint_opened_positions
        )
        self.reset_cubes_to_waiting()


# ╔══════════════════════════════════════════════════════════════╗
# ║  C. 메인 — Controller 생성 및 실행                              ║
# ╚══════════════════════════════════════════════════════════════╝

def main():
    if os.environ.get("ROS_DOMAIN_ID") != "50":
        print(
            "[주의] ROS_DOMAIN_ID가 50이 아닙니다. PC A/PC B 모두 "
            "`export ROS_DOMAIN_ID=50`로 맞춰 주세요."
        )

    if not rclpy.ok():
        rclpy.init(args=None)
    color_subscriber = ColorIdSubscriber()

    # ── C-1. World + Task ─────────────────────────────────────
    my_world = World(stage_units_in_meters=1.0)
    task = M0609Task(name="m0609_task")
    my_world.add_task(task)
    my_world.reset()

    robot = my_world.scene.get_object("m0609_robot")
    initialize_robot(robot, my_world)

    # 홈 포지션 안정화 대기
    for _ in range(30):
        my_world.step(render=True)

    # ── C-2. Controller 생성 (initialize 이후에만 가능) ───────
    print("\n" + "=" * 60)
    print("[C-2] PickPlaceController 생성")
    print("=" * 60)
    print(f"  URDF        = {M0609_URDF_PATH}")
    print(f"  description = {M0609_DESCRIPTION_PATH}")
    print(f"  rmpflow     = {M0609_RMPFLOW_CONFIG_PATH}")
    print(f"  events_dt   = {EVENTS_DT}")
    print(f"  EE frame    = {EE_LINK_NAME}")

    controller = PickPlaceController(
        name="m0609_pick_place_controller",
        gripper=robot.gripper,
        robot_articulation=robot,
        end_effector_initial_height=0.30,
        events_dt=EVENTS_DT,
        urdf_path=M0609_URDF_PATH,
        robot_description_path=M0609_DESCRIPTION_PATH,
        rmpflow_config_path=M0609_RMPFLOW_CONFIG_PATH,
        end_effector_frame_name=EE_LINK_NAME,
    )
    print("  [OK] Controller 생성 완료")

    # ── C-3. 초기 상태 진단 ───────────────────────────────────
    ee_pos, _ = robot.end_effector.get_world_pose()
    print(f"\n  EE 초기 위치 = {ee_pos}")
    print(f"  랜덤 pick X  = {PICK_X_RANGE}")
    print(f"  랜덤 pick Y  = {PICK_Y_RANGE}")
    print(f"  파란 목표    = {PLACE_POSITIONS[1]}")
    print(f"  초록 목표    = {PLACE_POSITIONS[2]}")
    print(f"  색상 구독    = {COLOR_TOPIC}")

    # ── C-4. Controller 실행 루프 ─────────────────────────────
    print("\n[Pick & Place 시작]\n")
    was_playing = False
    task_done = False
    waiting_message_printed = False
    staging_started_at = None

    try:
        while simulation_app.is_running():
            my_world.step(render=True)
            rclpy.spin_once(color_subscriber, timeout_sec=0.0)
            time.sleep(0.01)
            is_playing = my_world.is_playing()

            # Play 시작 감지 → 새 큐브/랜덤 위치/판별 상태로 리셋
            if is_playing and not was_playing:
                color_subscriber.reset_detection()
                my_world.reset()
                initialize_robot(robot, my_world)
                controller.reset()
                task_done = False
                waiting_message_printed = False
                staging_started_at = time.monotonic()
                print(
                    f"[공중 대기] 파란/초록 큐브가 {STAGING_WAIT_SECONDS:.1f}초 후 "
                    "랜덤 pick 위치로 이동합니다."
                )

            if is_playing and not task_done:
                if not task._pick_ready:
                    if time.monotonic() - staging_started_at < STAGING_WAIT_SECONDS:
                        was_playing = is_playing
                        continue
                    task.prepare_random_pick()

                obs = task.get_observations()
                cube_position = obs["active_cube"]["position"]
                current_joints = obs["m0609_robot"]["joint_positions"]
                event = controller.get_current_event()

                # 접근이 시작된 뒤의 wrist-camera 영상만 색상 판별에 사용한다.
                if event >= 1:
                    color_subscriber.accept_messages = True

                # 큐브를 들어 올린 시점에도 색상 응답이 없으면 controller의
                # 내부 event 진행을 멈추고 /color_id를 기다린다.
                if event >= COLOR_WAIT_EVENT and color_subscriber.color_id is None:
                    if not waiting_message_printed:
                        print(
                            f"[대기] {COLOR_TOPIC} 응답이 필요합니다. "
                            "PC B의 color_detector를 확인하세요."
                        )
                        waiting_message_printed = True
                    was_playing = is_playing
                    continue

                color_id = color_subscriber.color_id
                # event 0~3 동안 place 좌표는 아직 사용되지 않는다. 판별 전에는
                # 임시 좌표를 넣고, 결과가 오는 즉시 올바른 마커로 교체한다.
                placing_position = PLACE_POSITIONS.get(color_id, PLACE_POSITIONS[1])

                actions = controller.forward(
                    picking_position=cube_position,
                    placing_position=placing_position,
                    current_joint_positions=current_joints,
                    end_effector_offset=EE_OFFSET,
                )
                robot.apply_action(actions)

                if controller.is_done():
                    placed_cube = task._cubes[task._active_color_id]
                    placed_position, _ = placed_cube.get_world_pose()
                    place_error = np.linalg.norm(
                        placed_position[:2] - placing_position[:2]
                    )
                    print("\n" + "=" * 60)
                    print("[완료] 색상 기반 Pick & Place 종료")
                    print(f"  감지 색상 = {COLOR_NAMES[color_id]} ({color_id})")
                    print(f"  목표 위치 = {placing_position}")
                    print(f"  실제 위치 = {placed_position}")
                    print(f"  XY 오차   = {place_error:.4f} m")
                    if color_id != task._active_color_id:
                        print(
                            "  [경고] 감지 색상과 선택 큐브 색상이 다릅니다. "
                            "HSV/ROI 설정을 확인하세요."
                        )
                    task_done = True
                    my_world.pause()

                ee_pos, _ = robot.end_effector.get_world_pose()
                print(
                    f"  [event={event}] cube_z={cube_position[2]:.4f} "
                    f"ee_z={ee_pos[2]:.4f} color_id={color_id}"
                )

            was_playing = is_playing
    finally:
        color_subscriber.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        simulation_app.close()


if __name__ == "__main__":
    main()
