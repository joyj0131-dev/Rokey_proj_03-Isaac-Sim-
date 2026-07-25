import math
import sys
import pathlib

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))
from axle_center import TroughTracker


# --- 브리프 지정 케이스 (그대로 유지) -----------------------------------

def test_midpoint_of_symmetric_trough():
    # baseline 1.0, 바퀴 구간에서 값이 내려갔다 올라오는 대칭 트로프.
    tr = TroughTracker(baseline=1.0, drop_margin=0.05)
    xs = [0.0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6]
    vals = [1.00, 1.00, 0.90, 0.80, 0.90, 1.00, 1.00]   # 진입 0.2, 이탈 0.5 부근
    for x, v in zip(xs, vals):
        tr.update(x, v)
    c = tr.axle_center()
    assert c is not None
    assert abs(c - 0.35) < 0.06      # 진입·이탈의 중간


def test_no_trough_returns_none():
    tr = TroughTracker(baseline=1.0, drop_margin=0.05)
    for x in range(5):
        tr.update(float(x) * 0.1, 1.0)
    assert tr.axle_center() is None


def test_min_position_is_not_used_as_center():
    # 비대칭(한쪽이 더 깊은) 트로프에서도 중심은 최솟값 위치가 아니라 중간값이어야 한다.
    tr = TroughTracker(baseline=1.0, drop_margin=0.05)
    for x, v in [(0.0, 1.0), (0.1, 0.93), (0.2, 0.70), (0.3, 0.88), (0.4, 1.0)]:
        tr.update(x, v)
    c = tr.axle_center()
    assert abs(c - 0.20) > 1e-9 or True   # 최솟값 위치(0.2)와 중간값이 다를 수 있음을 문서화
    assert 0.10 <= c <= 0.35


# --- 추가 케이스 (Task C3 브리프가 요구) ---------------------------------

def test_two_troughs_rear_then_front_in_order():
    # 로봇이 트럭 아래를 지나며 뒷축(1번 트로프) 다음 앞축(2번 트로프)을 차례로
    # 통과하는 상황을 합성. 각 중간값과 발생 순서를 함께 검증한다 — C4가 "1번째
    # 트로프에서 정지"(rear-axle 로봇) vs "2번째에서 정지"(front-axle 로봇)를
    # 고르려면 순서 보존이 필수다.
    tr = TroughTracker(baseline=1.0, drop_margin=0.05)
    xs = [0.0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0, 1.1, 1.2, 1.3]
    vals = [1.00, 1.00, 0.90, 0.80, 0.90, 1.00, 1.00, 1.00, 1.00, 1.00,
            0.88, 0.75, 0.88, 1.00]
    for x, v in zip(xs, vals):
        tr.update(x, v)

    assert len(tr.troughs) == 2
    c1, c2 = tr.axle_centers()
    assert abs(c1 - 0.35) < 0.06     # 뒷축(트로프1) 진입 0.2·이탈 0.5 중간
    assert abs(c2 - 1.15) < 0.06     # 앞축(트로프2) 진입 1.0·이탈 1.3 중간
    assert c1 < c2                   # 발생 순서(뒤→앞)가 보존돼야 한다
    assert tr.axle_center(0) == c1
    assert tr.axle_center(1) == c2
    assert tr.axle_center() == c2    # 기본은 가장 최근(마지막) 트로프


def test_noisy_signal_no_spurious_trough():
    # baseline 주변 잡음(진폭 ±0.03) + 단일 프레임 스파이크(0.90, 1프레임)가
    # 섞인 스트림. C2 실측 스트림 자체는 매끈했지만(§4 taskC2fix-report),
    # 실전에서는 뎁스 센서 단일 프레임 튐이 있을 수 있으므로 confirm_frames(연속
    # 프레임 요구)로 이런 1프레임 튐을 걸러낼 수 있어야 한다 — 실제 바퀴 트로프는
    # 최소 confirm_frames 프레임 이상 지속된다는 가정에 근거한 디바운스.
    tr = TroughTracker(baseline=1.0, drop_margin=0.05, confirm_frames=3)
    xs = list(range(10))
    vals = [1.00, 0.97, 1.02, 0.90, 1.01, 0.98, 1.03, 0.90, 0.96, 1.00]
    for x, v in zip(xs, vals):
        tr.update(float(x), v)
    assert tr.troughs == []
    assert tr.axle_center() is None


def test_real_c2_shape_midpoint_not_argmin():
    # Task C2 --probe=DEPTH 실측 스트림(rear axle 트로프, taskC2fix-report.md §4)을
    # 그대로 재현. baseline=1.1022(DEPTH_PROBE_BASELINE 실측), 트로프 경계는
    # 로그의 TROUGH1 START x=-6.120 END x=-7.217 와 정확히 일치해야 한다.
    # 비대칭 딥(왼쪽이 짧고 깊게, 오른쪽이 완만하게 회복)에서도 중심은 최솟값
    # 위치(argmin, x≈-6.319)가 아니라 진입·이탈의 중간값이어야 한다.
    tr = TroughTracker(baseline=1.1022, drop_margin=0.05)
    stream = [
        (-6.020, 1.1004),
        (-6.120, 0.3802),   # 진입
        (-6.319, 0.2507),   # 최솟값 부근(0.2507이 딥 전체 최솟값)
        (-6.818, 0.2512),
        (-7.117, 0.5473),   # 아직 임계값 아래(회복 미완)
        (-7.217, 1.1002),   # 이탈
    ]
    for x, v in stream:
        tr.update(x, v)

    c = tr.axle_center()
    assert c is not None
    expected_center = (-6.120 + -7.217) / 2.0
    assert math.isclose(c, expected_center, abs_tol=1e-9)

    argmin_pos = min(stream, key=lambda p: p[1])[0]
    assert argmin_pos == -6.319
    assert abs(c - argmin_pos) > 0.2   # 중심 ≠ 최솟값 위치, 유의미하게 떨어져 있음


def test_baseline_learned_from_initial_samples():
    # baseline 을 생성자 인자로 주지 않으면 DepthStopDetector 와 같은 관례로
    # 초기 N프레임의 중앙값에서 학습해야 한다.
    tr = TroughTracker(baseline=None, baseline_frames=3, drop_margin=0.05)
    xs = [0.0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8]
    vals = [1.00, 1.00, 1.00, 1.00, 0.90, 0.80, 0.90, 1.00, 1.00]
    for x, v in zip(xs, vals):
        tr.update(x, v)

    assert tr.baseline is not None
    assert math.isclose(tr.baseline, 1.0, abs_tol=1e-9)
    c = tr.axle_center()
    assert c is not None
    assert abs(c - 0.55) < 0.06   # 진입 0.4 · 이탈 0.7 의 중간
