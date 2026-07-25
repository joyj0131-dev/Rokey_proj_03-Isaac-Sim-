import pytest

from parkbot_motion.pickup_choreography import (
    CorridorStaggerGuard, CorridorStep, corridor_plan, lift_both_succeeded, run_concurrent,
)


# ---- corridor_plan ----------------------------------------------------------------

def test_corridor_plan_follower_completes_before_leader_starts():
    steps = corridor_plan('entry_lead', 'entry_follow')
    assert len(steps) == 6
    follower_steps, leader_steps = steps[:3], steps[3:]
    assert all(s.robot_id == 'entry_follow' for s in follower_steps)
    assert all(s.robot_id == 'entry_lead' for s in leader_steps)


def test_corridor_plan_phase_order_within_each_robot():
    steps = corridor_plan('entry_lead', 'entry_follow')
    follower_phases = [s.phase for s in steps[:3]]
    leader_phases = [s.phase for s in steps[3:]]
    assert follower_phases == ['approach', 'align', 'ingress']
    assert leader_phases == ['approach', 'align', 'ingress']


def test_corridor_plan_default_trough_indices_match_mission_convention():
    # leader = rear axle = trough 0, follower = front axle = trough 1
    # (taskR5a-report.md: entry_lead -> index0(rear), entry_follow -> index1(front))
    steps = corridor_plan('entry_lead', 'entry_follow')
    ingress_by_robot = {s.robot_id: s.trough_index for s in steps if s.phase == 'ingress'}
    assert ingress_by_robot == {'entry_lead': 0, 'entry_follow': 1}


def test_corridor_plan_non_ingress_steps_have_no_trough_index():
    steps = corridor_plan('entry_lead', 'entry_follow')
    for s in steps:
        if s.phase != 'ingress':
            assert s.trough_index is None


def test_corridor_plan_custom_trough_indices():
    steps = corridor_plan('robot_a', 'robot_b', leader_trough_index=5, follower_trough_index=9)
    ingress_by_robot = {s.robot_id: s.trough_index for s in steps if s.phase == 'ingress'}
    assert ingress_by_robot == {'robot_a': 5, 'robot_b': 9}


def test_corridor_step_is_a_plain_namedtuple():
    assert CorridorStep('approach', 'r1', None).phase == 'approach'


# ---- CorridorStaggerGuard ----------------------------------------------------------

def test_stagger_guard_allows_same_robot_reentry():
    g = CorridorStaggerGuard()
    g.enter('entry_follow')
    g.enter('entry_follow')  # approach -> align -> ingress: same robot re-checks in
    assert g.occupant == 'entry_follow'


def test_stagger_guard_blocks_second_robot_before_clear():
    g = CorridorStaggerGuard()
    g.enter('entry_follow')
    with pytest.raises(RuntimeError):
        g.enter('entry_lead')


def test_stagger_guard_allows_second_robot_after_clear():
    g = CorridorStaggerGuard()
    g.enter('entry_follow')
    g.clear('entry_follow')
    g.enter('entry_lead')
    assert g.occupant == 'entry_lead'


def test_stagger_guard_clear_by_non_occupant_is_noop():
    g = CorridorStaggerGuard()
    g.enter('entry_follow')
    g.clear('entry_lead')  # not the occupant -- must not evict entry_follow
    assert g.occupant == 'entry_follow'


def test_stagger_guard_starts_empty():
    g = CorridorStaggerGuard()
    assert g.occupant is None
    g.enter('entry_lead')  # first entrant never conflicts
    assert g.occupant == 'entry_lead'


# ---- run_concurrent (concurrent-lift dispatch) --------------------------------------

def test_run_concurrent_sends_all_before_waiting_any():
    events = []

    def make_pair(name):
        def send():
            events.append(f'send:{name}')
            return name

        def wait(token):
            events.append(f'wait:{token}')
            return f'result:{token}'

        return send, wait

    pairs = [make_pair('leader'), make_pair('follower')]
    results = run_concurrent(pairs)

    assert events == ['send:leader', 'send:follower', 'wait:leader', 'wait:follower']
    assert results == ['result:leader', 'result:follower']


def test_run_concurrent_preserves_pair_order_in_results():
    pairs = [
        (lambda: 'a', lambda tok: tok.upper()),
        (lambda: 'b', lambda tok: tok.upper()),
        (lambda: 'c', lambda tok: tok.upper()),
    ]
    assert run_concurrent(pairs) == ['A', 'B', 'C']


def test_run_concurrent_second_send_not_blocked_by_first_waits_gate():
    # Simulates the real risk this function exists to prevent: if send/wait were
    # interleaved (send1, wait1, send2, wait2), a slow wait1 would delay send2.
    # Here wait() only succeeds once *both* sends have already happened (gate),
    # proving send-all-then-wait-all actually executes in that order.
    gate = {'sent': set()}

    def make_pair(name):
        def send():
            gate['sent'].add(name)
            return name

        def wait(token):
            # If this ever runs before both sends happened, the gate is incomplete.
            assert gate['sent'] == {'leader', 'follower'}
            return True

        return send, wait

    results = run_concurrent([make_pair('leader'), make_pair('follower')])
    assert results == [True, True]


def test_run_concurrent_empty_pairs():
    assert run_concurrent([]) == []


# ---- lift_both_succeeded ------------------------------------------------------------

def test_lift_both_succeeded_true():
    assert lift_both_succeeded([True, True]) is True


def test_lift_both_succeeded_one_failed():
    assert lift_both_succeeded([True, False]) is False


def test_lift_both_succeeded_both_failed():
    assert lift_both_succeeded([False, False]) is False


def test_lift_both_succeeded_empty_is_not_success():
    assert lift_both_succeeded([]) is False


def test_lift_both_succeeded_single_robot_not_enough():
    # A single True is not "both" -- guards against accidentally treating a
    # single-robot call as a full success (taskR4-report.md §3.3 failure mode).
    assert lift_both_succeeded([True]) is False
