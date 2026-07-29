import time
from types import SimpleNamespace

import pytest

from parking_control.sim_orchestrator_node import (
    EmergencyStopTriggered,
    SimOrchestratorNode,
)


class _Logger:
    def info(self, _message):
        pass

    def warn(self, _message):
        pass


def test_stopped_sim_goal_stays_cancelled_after_system_returns_to_normal():
    orchestrator = SimpleNamespace(
        _safety_state="NORMAL",
        _last_safety_state_at=time.monotonic(),
        _emergency_stop=False,
        _operation_cancelled=False,
        get_logger=lambda: _Logger(),
    )

    SimOrchestratorNode._on_safety_state(
        orchestrator,
        SimpleNamespace(state="STOPPED_LATCHED", motion_allowed=False),
    )
    SimOrchestratorNode._on_safety_state(
        orchestrator,
        SimpleNamespace(state="NORMAL", motion_allowed=True),
    )

    assert orchestrator._emergency_stop is False
    assert orchestrator._operation_cancelled is True
    with pytest.raises(EmergencyStopTriggered):
        SimOrchestratorNode._raise_if_emergency_stopped(orchestrator)


def test_entry_obstacle_pauses_only_entry_sim_team_and_clear_resumes():
    message = SimpleNamespace(
        obstacle_detected=True,
        description="통로 막힘: ZIN03",
        location=SimpleNamespace(y=-6.875),
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

    SimOrchestratorNode._on_obstacle_alert(entry, message)
    SimOrchestratorNode._on_obstacle_alert(exit_team, message)

    assert entry._obstacle_paused is True
    assert exit_team._obstacle_paused is False

    message.obstacle_detected = False
    SimOrchestratorNode._on_obstacle_alert(entry, message)
    assert entry._obstacle_paused is False
