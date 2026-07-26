import threading
import time
from types import SimpleNamespace

from parking_robot_system.formation_motion import FormationMotion


class _Publisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


class _Logger:
    def __init__(self):
        self.errors = []
        self.warnings = []
        self.infos = []

    def error(self, message):
        self.errors.append(message)

    def warn(self, message):
        self.warnings.append(message)

    def info(self, message):
        self.infos.append(message)


class _Node:
    def __init__(self):
        self.logger = _Logger()

    def get_logger(self):
        return self.logger


def _motion():
    motion = FormationMotion.__new__(FormationMotion)
    motion.node = _Node()
    motion._emergency_stop = False
    motion._team_role = "entry"
    motion._obstacle_paused = False
    motion._operation_cancelled = False
    motion._last_safety_state_at = time.monotonic()
    motion.robots = ("robot",)
    motion.cmd = {"robot": _Publisher()}
    return motion


def test_control_tower_stop_latches_and_forces_zero_velocity():
    motion = _motion()

    motion._on_emergency_stop(SimpleNamespace(
        stop=True,
        source_robot_id="control_tower",
        reason="operator request",
    ))
    motion._pub("robot", 1.0, 2.0, 3.0)

    command = motion.cmd["robot"].messages[-1]
    assert motion._emergency_stop is True
    assert command.linear.x == 0.0
    assert command.linear.y == 0.0
    assert command.angular.z == 0.0
    assert motion.node.logger.errors


def test_non_control_tower_formation_signal_does_not_trigger_global_stop():
    motion = _motion()

    motion._on_emergency_stop(SimpleNamespace(
        stop=True,
        source_robot_id="entry_lead",
        reason="peer stop",
    ))
    motion._pub("robot", 1.0, 0.5, 0.25)

    command = motion.cmd["robot"].messages[-1]
    assert motion._emergency_stop is False
    assert command.linear.x == 1.0
    assert command.linear.y == 0.5
    assert command.angular.z == 0.25


def test_safety_reset_does_not_move_until_operation_is_separately_approved():
    motion = _motion()
    motion._emergency_stop = True
    motion._operation_cancelled = True

    motion._on_safety_state(SimpleNamespace(
        state="READY_FOR_OPERATION",
        motion_allowed=False,
    ))
    motion._pub("robot", 1.0)
    assert motion.cmd["robot"].messages[-1].linear.x == 0.0

    motion._on_safety_state(SimpleNamespace(
        state="NORMAL",
        motion_allowed=True,
    ))
    motion._pub("robot", 1.0)
    assert motion.cmd["robot"].messages[-1].linear.x == 0.0

    # 운영 승인으로 NORMAL이 되어도 중단된 기존 액션은 재개되지 않는다.
    # 이후 관제가 새 작업을 보내 새 액션이 시작될 때만 이동할 수 있다.
    assert motion.begin_operation() is True
    motion._pub("robot", 1.0)
    assert motion.cmd["robot"].messages[-1].linear.x == 1.0


def test_emergency_stopped_operation_cannot_resume_after_normal_state():
    motion = _motion()
    assert motion.begin_operation() is True

    motion._on_safety_state(SimpleNamespace(
        state="STOPPED_LATCHED",
        motion_allowed=False,
    ))
    motion._on_safety_state(SimpleNamespace(
        state="NORMAL",
        motion_allowed=True,
    ))
    motion._pub("robot", 1.0)

    assert motion._operation_cancelled is True
    assert motion.cmd["robot"].messages[-1].linear.x == 0.0


def test_active_motion_loop_exits_instead_of_returning_to_its_target():
    motion = _motion()
    motion.pose = {"robot": (0.0, 0.0, 0.0)}
    assert motion.begin_operation() is True
    outcome = {}

    worker = threading.Thread(
        target=lambda: outcome.setdefault(
            "ok", motion.goto_xz("robot", 10.0, 0.0, timeout=1.0)
        )
    )
    worker.start()
    time.sleep(0.03)
    motion._on_safety_state(SimpleNamespace(
        state="STOPPED_LATCHED",
        motion_allowed=False,
    ))
    motion._on_safety_state(SimpleNamespace(
        state="NORMAL",
        motion_allowed=True,
    ))
    worker.join(timeout=0.5)

    assert worker.is_alive() is False
    assert outcome["ok"] is False
    assert motion.cmd["robot"].messages[-1].linear.x == 0.0


def test_missing_safety_heartbeat_forces_zero_velocity():
    motion = _motion()
    motion._last_safety_state_at = time.monotonic() - 4.0

    motion._pub("robot", 1.0, 0.5, 0.25)

    command = motion.cmd["robot"].messages[-1]
    assert command.linear.x == 0.0
    assert command.linear.y == 0.0
    assert command.angular.z == 0.0


def test_entry_obstacle_pauses_and_clear_resumes_only_entry_motion():
    entry_motion = _motion()
    exit_motion = _motion()
    exit_motion._team_role = "exit"
    detected = SimpleNamespace(
        obstacle_detected=True,
        description="통로 막힘: ZIN03",
        location=SimpleNamespace(y=-6.875),
    )

    entry_motion._on_obstacle_alert(detected)
    exit_motion._on_obstacle_alert(detected)

    assert entry_motion._obstacle_paused is True
    assert exit_motion._obstacle_paused is False
    assert entry_motion.cmd["robot"].messages[-1].linear.x == 0.0

    worker = threading.Thread(
        target=lambda: entry_motion._pub("robot", 1.0)
    )
    worker.start()
    time.sleep(0.03)
    assert worker.is_alive() is True

    cleared = SimpleNamespace(
        obstacle_detected=False,
        description="",
        location=SimpleNamespace(y=0.0),
    )
    entry_motion._on_obstacle_alert(cleared)
    worker.join(timeout=0.5)

    assert worker.is_alive() is False
    assert entry_motion._obstacle_paused is False
    assert entry_motion.cmd["robot"].messages[-1].linear.x == 1.0
