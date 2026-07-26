"""Phase B(도크→XN 융합주행) 순수 시퀀싱 + 오케스트레이터 배선 단위테스트 (R6/T3).

앞부분(phase_b_plan/PhaseBStep/phase_b_robot_phases)은 rclpy 무의존 순수 로직이다.
뒷부분(PickupOrchestratorNode._run_phase_b)은 rclpy(Node 서브클래스 정의)를
import-time 에 필요로 하지만, 실제 호출은 노드를 `__new__` 로 만들어(스핀/init
없이) 리프 연산(`_navigate_phase_b`/`_set_localizer_ref`)만 페이크로 갈아끼워
"스텝 순서·목표·ref 전환"이 올바른지 검증한다 — test_pose_controller_node_quat.py
와 동일한 취지(런타임 의존 없는 부분만 순수하게 부른다).
"""
import pytest

from parkbot_motion.pickup_choreography import (
    PhaseBStep, phase_b_plan, phase_b_robot_phases,
)


# ---- 순수 시퀀싱 (brief Step 1) ---------------------------------------------------

def test_phase_b_plan_stagger_and_steps():
    steps = phase_b_plan('entry_lead', 'entry_follow')
    ids = [s.robot_id for s in steps]
    # lead 전체가 follow 앞에
    assert ids.index('entry_follow') > max(i for i, r in enumerate(ids) if r == 'entry_lead')
    lead = [s.phase for s in steps if s.robot_id == 'entry_lead']
    assert lead == ['seed_dock', 'rotate_90', 'dock_check', 'xn_align_x', 'xn_align_z', 'offset']
    follow = [s.phase for s in steps if s.robot_id == 'entry_follow']
    assert 'offset' not in follow   # follow 는 오프셋 없음(XN 축선)


def test_phase_b_plan_leader_all_before_follower():
    steps = phase_b_plan('entry_lead', 'entry_follow')
    assert len(steps) == 11  # leader 6 + follower 5
    assert all(s.robot_id == 'entry_lead' for s in steps[:6])
    assert all(s.robot_id == 'entry_follow' for s in steps[6:])


def test_phase_b_step_is_plain_namedtuple():
    s = PhaseBStep('seed_dock', 'entry_lead')
    assert s.phase == 'seed_dock'
    assert s.robot_id == 'entry_lead'


def test_phase_b_robot_phases_leader_has_offset():
    assert phase_b_robot_phases(True)[-1] == 'offset'
    assert 'offset' not in phase_b_robot_phases(False)


# ---- 오케스트레이터 _run_phase_b (brief Step 6) ------------------------------------

def _make_orch():
    """rclpy.init/스핀 없이 노드 인스턴스만 만들어(리프 연산은 테스트에서 페이크로
    교체) `_run_phase_b` 시퀀싱을 검증한다."""
    from parkbot_motion.pickup_orchestrator_node import PickupOrchestratorNode
    orch = PickupOrchestratorNode.__new__(PickupOrchestratorNode)
    orch.navigate_odom_action = 'navigate_to_pose'
    orch.navigate_fused_action = 'navigate_to_pose_fused'
    return orch


def _leader_params():
    return {
        'is_leader': True,
        'localizer_node': '/robot_entry_lead/marker_localizer_node',
        'dock_id': 21, 'dock_x': -3.2, 'dock_z': 2.2, 'dock_decal_z': 2.9,
        'dockcheck_standoff': 1.4,
        'xn_id': 31, 'xn_x': -2.5, 'xn_z': 6.875, 'xn_standoff': 1.3,
        'final_x_offset': -1.7,
    }


def _follower_params():
    return {
        'is_leader': False,
        'localizer_node': '/robot_entry_follow/marker_localizer_node',
        'dock_id': 23, 'dock_x': -1.2, 'dock_z': 2.2, 'dock_decal_z': 2.9,
        'dockcheck_standoff': 1.4,
        'xn_id': 31, 'xn_x': -2.5, 'xn_z': 6.875, 'xn_standoff': 1.3,
        'final_x_offset': -1.7,
    }


def _instrument(orch, calls):
    def fake_nav(robot_id, action_key, action_suffix, x, z, yaw_deg, label):
        calls.append(('nav', action_key, round(x, 4), round(z, 4), round(yaw_deg, 4)))
        return True, None

    def fake_ref(node_name, ref_ids, correct_yaw, label):
        calls.append(('ref', node_name, list(ref_ids), correct_yaw))
        return True, None

    orch._navigate_phase_b = fake_nav
    orch._set_localizer_ref = fake_ref


def test_run_phase_b_leader_full_sequence():
    orch = _make_orch()
    calls = []
    _instrument(orch, calls)
    ok, reason = orch._run_phase_b('entry_lead', _leader_params())
    assert ok is True and reason is None
    # rotate_90 z = dock_z(스폰/제자리회전, aruco:position) = 2.2
    # dock_check z = dock_decal_z(데칼) + dockcheck_standoff = 2.9 + 1.4 = 4.3
    # xn_align_z / offset z = xn_z - xn_standoff = 6.875 - 1.3 = 5.575
    assert calls == [
        ('ref', '/robot_entry_lead/marker_localizer_node', [21], False),   # seed_dock
        ('nav', 'nav_odom', -3.2, 2.2, 0.0),                               # rotate_90
        ('nav', 'nav_fused', -3.2, 4.3, 0.0),                              # dock_check
        ('ref', '/robot_entry_lead/marker_localizer_node', [31], False),   # xn_align_x: ref 전환
        ('nav', 'nav_fused', -2.5, 4.3, 0.0),                              # xn_align_x: x 정렬(z 유지)
        ('nav', 'nav_fused', -2.5, 5.575, 0.0),                           # xn_align_z: 순수 북진
        ('nav', 'nav_fused', -4.2, 5.575, 0.0),                           # offset: xn_x + (-1.7)
    ]
    # 회귀 가드: dock_z/dock_decal_z 가 다시 뒤바뀌면 이 두 z 값이 같아지거나
    # (스폰 z 를 도크체크에 쓰는 경우) 표준 3.6 로 되돌아간다 — 명시적으로 구분.
    rotate_z = calls[1][3]
    dock_check_z = calls[2][3]
    assert rotate_z == 2.2
    assert dock_check_z == 4.3
    assert rotate_z != dock_check_z


def test_run_phase_b_follower_has_no_offset():
    orch = _make_orch()
    calls = []
    _instrument(orch, calls)
    ok, reason = orch._run_phase_b('entry_follow', _follower_params())
    assert ok is True and reason is None
    # follower: offset 스텝 없음 — 마지막 nav 가 XN 축선(xn_x, 5.575)에서 끝난다.
    assert calls[-1] == ('nav', 'nav_fused', -2.5, 5.575, 0.0)
    # ref 전환은 도크(23) → XN(31) 정확히 두 번.
    ref_calls = [c for c in calls if c[0] == 'ref']
    assert ref_calls == [
        ('ref', '/robot_entry_follow/marker_localizer_node', [23], False),
        ('ref', '/robot_entry_follow/marker_localizer_node', [31], False),
    ]
    assert len(calls) == 6  # seed_dock(ref)+rotate+dock_check+xn_align_x(ref+nav)+xn_align_z


def test_run_phase_b_rotate_uses_odom_all_else_fused():
    orch = _make_orch()
    calls = []
    _instrument(orch, calls)
    orch._run_phase_b('entry_lead', _leader_params())
    nav_keys = [c[1] for c in calls if c[0] == 'nav']
    # 회전만 odom 인스턴스, 나머지 주행은 전부 융합 인스턴스.
    assert nav_keys[0] == 'nav_odom'
    assert all(k == 'nav_fused' for k in nav_keys[1:])


def test_run_phase_b_aborts_on_nav_failure():
    orch = _make_orch()
    calls = []

    def fail_nav(robot_id, action_key, action_suffix, x, z, yaw_deg, label):
        calls.append(('nav', action_key))
        return False, 'boom'

    def ok_ref(node_name, ref_ids, correct_yaw, label):
        calls.append(('ref',))
        return True, None

    orch._navigate_phase_b = fail_nav
    orch._set_localizer_ref = ok_ref
    ok, reason = orch._run_phase_b('entry_lead', _leader_params())
    assert ok is False
    assert 'boom' in reason
    # 첫 nav(rotate_90)에서 실패 → 그 뒤 스텝은 실행되지 않는다.
    assert calls == [('ref',), ('nav', 'nav_odom')]


def test_run_phase_b_aborts_on_ref_failure():
    orch = _make_orch()
    calls = []

    def ok_nav(robot_id, action_key, action_suffix, x, z, yaw_deg, label):
        calls.append(('nav',))
        return True, None

    def fail_ref(node_name, ref_ids, correct_yaw, label):
        calls.append(('ref',))
        return False, 'param rejected'

    orch._navigate_phase_b = ok_nav
    orch._set_localizer_ref = fail_ref
    ok, reason = orch._run_phase_b('entry_lead', _leader_params())
    assert ok is False
    assert 'param rejected' in reason
    # seed_dock(ref) 에서 바로 실패 → 아무 nav 도 안 돈다.
    assert calls == [('ref',)]
