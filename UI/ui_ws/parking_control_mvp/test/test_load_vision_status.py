"""협동 적재·ArUco 관제 데이터 계약 회귀 테스트."""

import json
import math
import time

from geometry_msgs.msg import PointStamped, PoseStamped, Twist
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32, String
from parking_robot_interfaces.msg import SafetyState, TaskState

from core.models import (
    ParkingRequest,
    ParkingRequestCreate,
    RequestStatus,
    RequestType,
    Robot,
)
from core.state_store import StateStore
from sources.mock_source import MockDataSource
from sources.ros2_source import Ros2DataSource


def test_mock_load_panel_uses_four_support_points_and_mock_source():
    store = StateStore()
    source = MockDataSource(store)
    request = source.create_request(
        ParkingRequestCreate(
            request_type=RequestType.PARK_IN,
            vehicle_number="55가5555",
        )
    )

    load = source.get_cooperative_load_states()[0]

    assert load.request_id == request.id
    assert load.source == "MOCK"
    assert load.telemetry_age_sec == 0.0
    assert load.telemetry_rate_hz == 10.0
    assert len(load.support_points) == 4
    assert all(len(point.joint_names) == 2 for point in load.support_points)
    assert load.front_alignment_error_mm is None
    assert load.stable is None


def test_mock_lifting_exposes_alignment_support_and_vision_result():
    store = StateStore()
    source = MockDataSource(store)
    request = source.create_request(
        ParkingRequestCreate(
            request_type=RequestType.PARK_IN,
            vehicle_number="55가5555",
        )
    )
    source.advance_request(request.id)  # ROBOT_ASSIGNED -> APPROACHING
    vision = source.get_vision_alignment_states()[0]
    assert vision.marker_detected is True
    assert vision.marker_id == 32
    assert vision.source == "MOCK"

    source.advance_request(request.id)  # APPROACHING -> LIFTING
    source._stage_started[request.id] = (
        time.monotonic() - source._LIFT_HOLD_SEC
    )
    load = source.get_cooperative_load_states()[0]

    assert load.front_alignment_error_mm == 8.0
    assert load.rear_alignment_error_mm == 11.0
    assert load.tire_support_count == 4
    assert load.synchronized is True
    assert load.stable is True


def test_mock_completed_request_records_duration_and_returns_team_to_idle():
    store = StateStore()
    source = MockDataSource(store)
    request = source.create_request(
        ParkingRequestCreate(
            request_type=RequestType.PARK_IN,
            vehicle_number="55가5555",
        )
    )

    for _step in range(5):
        request = source.advance_request(request.id)

    assert request.status == RequestStatus.COMPLETED
    assert request.completed_at is not None
    assert all(
        store.find_robot(robot_id).status == "IDLE"
        for robot_id in request.robot_ids
    )


def test_ros2_joint_pose_and_diagnostics_are_normalized_without_fake_load_data():
    store = StateStore()
    source = Ros2DataSource(store)
    store.requests.append(
        ParkingRequest(
            id=1,
            request_type=RequestType.PARK_IN,
            vehicle_number="55가5555",
            slot_id="A2",
            robot_id="entry_lead",
            robot_ids=["entry_lead", "entry_follow"],
            status=RequestStatus.LIFTING,
            created_at="2026-07-27T10:00:00",
            external_task_id="task-1",
        )
    )

    for robot_id, pose_x, axle_x in (
        ("entry_lead", 1.0, 1.008),
        ("entry_follow", 2.0, 2.011),
    ):
        pose = PoseStamped()
        pose.pose.position.x = pose_x
        source._on_robot_pose(robot_id, pose)
        axle = PointStamped()
        axle.point.x = axle_x
        source._on_axle_center(robot_id, axle)
        joints = JointState()
        joints.name = [
            "arm_left_front_joint",
            "arm_left_rear_joint",
            "arm_right_front_joint",
            "arm_right_rear_joint",
        ]
        joints.position = [
            math.pi / 2,
            -math.pi / 2,
            -math.pi / 2,
            math.pi / 2,
        ]
        source._on_joint_state(robot_id, joints)
        command = Float32()
        command.data = 1.0
        source._on_lift_command(robot_id, command)

    load = source.get_cooperative_load_states()[0]
    assert load.front_alignment_error_mm == 8.0
    assert load.rear_alignment_error_mm == 11.0
    assert load.tire_support_count == 4
    assert load.stable is True
    assert load.vehicle_rise_mm is None
    assert load.pitch_deg is None
    assert load.load_anomaly_suspected is False
    assert load.slip_suspected is None
    assert load.telemetry_age_sec is not None
    assert load.telemetry_rate_hz is not None

    diagnostic = String()
    diagnostic.data = json.dumps(
        {
            "connected": True,
            "detected": True,
            "camera": "front",
            "marker_id": 32,
            "distance_m": 0.84,
            "reprojection_error_px": 0.7,
            "marker_corners": [
                [0.4, 0.3],
                [0.6, 0.3],
                [0.6, 0.6],
                [0.4, 0.6],
            ],
        }
    )
    source._on_vision_alignment("entry_lead", diagnostic)
    vision = source.get_vision_alignment_states()[0]
    assert vision.marker_detected is True
    assert vision.marker_id == 32
    assert vision.source == "ARUCO_FUSED"


def test_ros2_arm_imbalance_and_stalled_motion_raise_estimated_warnings():
    store = StateStore()
    source = Ros2DataSource(store)
    store.requests.append(
        ParkingRequest(
            id=2,
            request_type=RequestType.PARK_IN,
            vehicle_number="77가7777",
            slot_id="A3",
            robot_id="entry_lead",
            robot_ids=["entry_lead", "entry_follow"],
            status=RequestStatus.MOVING_TO_SLOT,
            created_at="2026-07-27T10:00:00",
        )
    )
    for robot_id in ("entry_lead", "entry_follow"):
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.pose.position.x = 1.0
        source._on_robot_pose(robot_id, pose)
        # 두 번째 동일 자세 표본은 명령 대비 실제 이동 0인 상태를 만든다.
        source._on_robot_pose(robot_id, pose)
        command = Twist()
        command.linear.x = 0.2
        source._on_cmd_vel(robot_id, command)
        lift = Float32()
        lift.data = 1.0
        source._on_lift_command(robot_id, lift)
        joints = JointState()
        joints.name = [
            "arm_left_front_joint",
            "arm_left_rear_joint",
            "arm_right_front_joint",
            "arm_right_rear_joint",
        ]
        joints.position = [
            math.pi / 2,
            math.pi / 2,
            math.pi / 2,
            math.pi / 2,
        ]
        if robot_id == "entry_follow":
            joints.position[-1] = math.pi / 4
        source._on_joint_state(robot_id, joints)
        source._robot_slip_since[robot_id] = time.monotonic() - 1.0

    load = source.get_cooperative_load_states()[0]

    assert load.slip_suspected is True
    assert load.load_anomaly_suspected is True
    assert load.synchronized is False
    assert load.stable is False


def test_ros2_safety_state_keeps_affected_robots_out_of_idle():
    store = StateStore()
    source = Ros2DataSource(store)
    store.requests.append(
        ParkingRequest(
            id=3,
            request_type=RequestType.PARK_IN,
            vehicle_number="88가8888",
            slot_id="A2",
            robot_id="entry_lead",
            robot_ids=["entry_lead", "entry_follow"],
            status=RequestStatus.MOVING_TO_SLOT,
            created_at="2026-07-27T10:00:00",
            external_task_id="task-safe-stop",
        )
    )
    store.robots.extend(
        [
            Robot(
                id="entry_lead",
                status="IDLE",
                battery=90,
                x=1.0,
                y=-6.0,
            ),
            Robot(
                id="entry_follow",
                status="IDLE",
                battery=88,
                x=1.0,
                y=-4.0,
            ),
        ]
    )
    failed = TaskState()
    failed.task_id = "task-safe-stop"
    failed.robot_id = "entry_lead"
    failed.state = "FAILED"
    source._on_task_state(failed)
    assert any(
        alert.category.value == "ROBOT_ERROR"
        for alert in store.snapshot()["alerts"]
    )

    stopped = SafetyState()
    stopped.state = "STOPPED_LATCHED"
    stopped.motion_allowed = False
    stopped.stop_epoch = 1
    stopped.reason = "test emergency stop"
    stopped.affected_task_ids = ["task-safe-stop"]
    source._on_safety_state(stopped)

    assert source.recovery_state["status"] == "SAFETY_STOPPED"
    assert not any(
        alert.category.value == "ROBOT_ERROR"
        for alert in store.snapshot()["alerts"]
    )
    assert all(
        store.find_robot(robot_id).status == "SAFETY_STOPPED"
        for robot_id in ("entry_lead", "entry_follow")
    )

    ready = SafetyState()
    ready.state = "READY_FOR_OPERATION"
    ready.motion_allowed = False
    ready.stop_epoch = 1
    ready.affected_task_ids = ["task-safe-stop"]
    source._on_safety_state(ready)

    assert source.recovery_state["status"] == "REQUIRED"
    assert all(
        store.find_robot(robot_id).status == "RECOVERY_REQUIRED"
        for robot_id in ("entry_lead", "entry_follow")
    )
