from parking_robot_system.robot_task_orchestrator import next_state, plan_steps, TRANSITIONS


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
