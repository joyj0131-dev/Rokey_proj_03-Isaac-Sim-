"""공동 정지(co-stop) 판단 로직 단위 테스트."""

from parking_control.core.formation_costop import is_stale, should_stop


def test_is_stale_when_never_seen():
    assert is_stale(now_sec=10.0, last_seen_sec=None, timeout_sec=0.5) is True


def test_is_stale_within_timeout():
    assert is_stale(now_sec=10.0, last_seen_sec=9.8, timeout_sec=0.5) is False


def test_is_stale_past_timeout():
    assert is_stale(now_sec=10.0, last_seen_sec=9.0, timeout_sec=0.5) is True


def test_should_stop_all_clear():
    assert should_stop(self_fault=False, peer_signal_stale=False,
                       peer_requested_stop=False) is False


def test_should_stop_on_self_fault_alone():
    assert should_stop(self_fault=True, peer_signal_stale=False,
                       peer_requested_stop=False) is True


def test_should_stop_on_peer_stale_alone():
    assert should_stop(self_fault=False, peer_signal_stale=True,
                       peer_requested_stop=False) is True


def test_should_stop_on_peer_request_alone():
    """한쪽만 멈추면 위험하다는 게 이 프로젝트의 핵심 요구사항 —
    파트너가 멈추라고 방송하면 내가 멀쩡해도 반드시 같이 멈춰야 한다."""
    assert should_stop(self_fault=False, peer_signal_stale=False,
                       peer_requested_stop=True) is True
