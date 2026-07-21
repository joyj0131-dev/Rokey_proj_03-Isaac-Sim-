"""로봇 2대(leader/follower)의 "공동 정지" 판단 로직. 순수 Python.

2026-07-21 논의로 확정: 차량 하나를 로봇 2대가 붙잡고 옮기는 중에는
"한쪽만 멈춤"이 "둘 다 멈춤"이나 "둘 다 이동"보다 훨씬 위험하다(차가
뒤틀리거나 팔에 무리한 힘이 걸림). 그래서 아래 셋 중 하나라도 해당하면
반드시 정지해야 한다:
  1. 자기 자신에게 이상이 생김 (힘 센서, 장애물 감지 등 — 노드 쪽에서 판단)
  2. 파트너 신호(리더면 팔로워 odom, 팔로워면 리더 odom)가 watchdog
     시간 안에 안 옴 — 통신 두절
  3. 파트너가 "나 멈춰야 해" 라고 명시적으로 방송함 (FormationStop.msg)
"""


def is_stale(now_sec: float, last_seen_sec, timeout_sec: float) -> bool:
    """파트너 신호를 아직 한 번도 못 받았거나(last_seen_sec=None), 마지막
    수신 후 timeout_sec 넘게 지났으면 True."""
    if last_seen_sec is None:
        return True
    return (now_sec - last_seen_sec) > timeout_sec


def should_stop(self_fault: bool, peer_signal_stale: bool,
                peer_requested_stop: bool) -> bool:
    """셋 중 하나라도 True면 정지. 어느 조건이 왜 걸렸는지는 호출부가
    로깅해서 원인을 남긴다 — 여기는 최종 판단만 담당."""
    return self_fault or peer_signal_stale or peer_requested_stop
