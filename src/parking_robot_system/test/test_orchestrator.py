from types import SimpleNamespace

from parking_robot_system.robot_task_orchestrator import (
    RobotTaskOrchestratorNode,
    TRANSITIONS,
    next_state,
    plan_steps,
)


class _Logger:
    def info(self, _message):
        pass

    def warn(self, _message):
        pass


def test_full_sequence():
    seq = ["SEARCHING", "APPROACHING", "PICKED_UP", "MOVING", "ARRIVED",
           "PARKED", "RETURNING", "DONE"]
    for a, b in zip(seq, seq[1:]):
        assert next_state(a) == b


def test_terminal():
    assert next_state("DONE") == "DONE"


def test_unknown_state_fails():
    assert next_state("BOGUS") == "FAILED"
    assert next_state("FAILED") == "FAILED"


def test_transitions_matches_brief_table():
    assert TRANSITIONS == {
        "SEARCHING": "APPROACHING", "APPROACHING": "PICKED_UP", "PICKED_UP": "MOVING",
        "MOVING": "ARRIVED", "ARRIVED": "PARKED", "PARKED": "RETURNING",
        "RETURNING": "DONE", "DONE": "DONE",
    }


def test_plan_steps_matches_full_sequence():
    seq = ["SEARCHING", "APPROACHING", "PICKED_UP", "MOVING", "ARRIVED",
           "PARKED", "RETURNING", "DONE"]
    assert plan_steps(None) == seq


def test_orchestrator_obstacle_scope_is_team_specific():
    message = SimpleNamespace(
        obstacle_detected=True,
        description="통로 막힘: ZOUT02",
        location=SimpleNamespace(y=7.075),
    )
    entry = SimpleNamespace(
        _team_role="entry",
        _obstacle_paused=False,
        get_logger=lambda: _Logger(),
    )
    exit_team = SimpleNamespace(
        _team_role="exit",
        _obstacle_paused=False,
        get_logger=lambda: _Logger(),
    )

    RobotTaskOrchestratorNode._on_obstacle_alert(entry, message)
    RobotTaskOrchestratorNode._on_obstacle_alert(exit_team, message)

    assert entry._obstacle_paused is False
    assert exit_team._obstacle_paused is True
