from pathlib import Path


def test_test_stack_restart_does_not_clear_latched_safety_state():
    script = (
        Path(__file__).resolve().parents[2] / "run_test_stack.sh"
    ).read_text(encoding="utf-8")

    assert "UPDATE safety_state" not in script
    assert "SET state='NORMAL'" not in script
