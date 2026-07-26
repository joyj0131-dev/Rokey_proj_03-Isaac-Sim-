from parking_robot_system.robot_task_orchestrator import (
    NAVIGATE_RESULT_TIMEOUT, RobotTaskOrchestratorNode, TRANSITIONS,
    next_state, plan_steps,
)


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


class _Future:
    def __init__(self, result=None, done=True):
        self._result = result
        self._done = done

    def done(self):
        return self._done

    def result(self):
        return self._result


class _ChildGoal:
    accepted = True

    def __init__(self):
        self.cancel_count = 0
        self.result_future = _Future(done=False)

    def get_result_async(self):
        return self.result_future

    def cancel_goal_async(self):
        self.cancel_count += 1
        return _Future()


class _Client:
    def __init__(self, child):
        self.child = child

    def wait_for_server(self, timeout_sec):
        return True

    def send_goal_async(self, _goal):
        return _Future(self.child)


class _Logger:
    def warn(self, _message):
        pass


class _Caller:
    def get_logger(self):
        return _Logger()


def test_child_goal_is_canceled_when_result_times_out():
    child = _ChildGoal()
    result, status, reason = RobotTaskOrchestratorNode._call_action(
        _Caller(), _Client(child), object(), label="test",
        wait_timeout=0.0, result_timeout=0.0)

    assert result is None
    assert status is None
    assert "타임아웃" in reason
    assert child.cancel_count == 1


def test_navigation_outer_timeout_covers_exit_three_part_motion():
    assert NAVIGATE_RESULT_TIMEOUT > 420.0 + 90.0 + 420.0
