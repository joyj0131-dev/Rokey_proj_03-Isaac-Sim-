#!/usr/bin/env python3
"""측면 뎁스캠 ROI-min 스트림에서 "트로프"(값이 baseline 아래로 내려갔다가
다시 올라오는 구간)를 잡아 그 진입/이탈 위치의 **중간값**을 축(axle) 중심
추정치로 낸다.

## 왜 중간값인가 (물리 원리)

로봇이 트럭 아래를 지나가며 바퀴 옆을 스칠 때, ROI 최소 뎁스는 바퀴가
시야에 들어오면서 baseline 아래로 떨어졌다가(진입) 바퀴를 벗어나며
회복된다(이탈). 타이어는 곡면이라 이 딥의 **최솟값(argmin) 위치는 축
중심이 아니다** — 차종마다 다른 타이어 반경/사이드월 프로파일에 좌우되는
보정이 필요해진다. 반면 진입 위치와 이탈 위치의 **중간점**은 딥의 좌우
대칭성만으로 축 중심에 떨어진다(어떤 차종 보정도 불필요). p4_depth 는 이
방식으로 실측 14mm 정확도를 얻었다(Task C3 브리프 참고).

## 하드 요구사항 — 이 모듈은 차종 기하를 전혀 모른다

`TroughTracker` 는 오직 스트리밍되는 `(pos, value)` 쌍만으로 동작한다.
축거(wheelbase), 바퀴 좌표, 차종별 상수 등 **어떤 사전 기억된(pre-remembered)
차량 기하도 이 파일 어디에도 두지 않는다** — 이게 이 방식의 핵심 이점이고,
사용자가 명시적으로 요구한 제약이다. 차종이 바뀌어도 이 모듈은 수정할
필요가 없다.
"""

import math
import statistics


class TroughTracker:
    """`update(pos, roi_min)` 을 프레임마다 호출해 트로프 진입/이탈을 추적한다.

    baseline 은 생성자 인자로 직접 줄 수도 있고(`baseline=...`), 생략하면
    `DepthStopDetector`(depth_stop_detector.py)와 동일한 관례로 처음
    `baseline_frames` 개의 유효(finite) 샘플의 중앙값을 학습해 쓴다 —
    baseline 학습 중에는 트로프 판정을 하지 않는다(신호가 아직 뭘 정상값으로
    봐야 할지 모르므로).

    `confirm_frames` (기본 1, 즉 디바운스 없음)를 1보다 크게 주면 단일 프레임
    잡음(센서 튐)으로 인한 허위 트로프 진입/이탈을 걸러낸다: 값이
    `confirm_frames` 프레임 연속으로 임계값 아래(또는 위)에 있어야 진입(또는
    이탈)로 확정한다. 단, **확정되면 기록되는 위치는 최초로 임계값을 넘은
    프레임의 pos**다(디바운스에 걸린 마지막 프레임이 아니라) — 그래야
    디바운스가 진입/이탈 위치를 뒤로 밀어 중간값 대칭성을 깨뜨리지 않는다.

    로봇이 트럭 아래를 지나며 뒷축→앞축처럼 트로프를 여러 번 지날 수 있으므로
    완료된 트로프를 전부 `self.troughs`(발생 순서 리스트)에 쌓는다.
    `axle_center(index=-1)` 은 기본으로 가장 최근(마지막) 완료 트로프의
    중간값을 돌려주고, `axle_centers()` 는 전체 리스트를 돌려준다 — 호출자가
    "1번째 트로프에서 정지"할지 "2번째에서 정지"할지 고를 수 있게 한다.
    """

    def __init__(self, baseline=None, baseline_frames=30, drop_margin=0.05, confirm_frames=1):
        self.baseline_frames = baseline_frames
        self.drop_margin = drop_margin
        self.confirm_frames = max(1, int(confirm_frames))

        self._baseline_samples = []
        self.baseline = baseline
        self.threshold = (baseline - drop_margin) if baseline is not None else None

        # 트로프 안에 있는 동안의 확정 진입 위치. None 이면 현재 트로프 밖.
        self._enter_pos = None
        # 디바운스용 카운터/후보 위치.
        self._below_count = 0
        self._candidate_enter = None
        self._above_count = 0
        self._candidate_exit = None

        # 완료된 트로프: [{'enter':.., 'exit':.., 'center':..}, ...] 발생 순서.
        self.troughs = []

    def _learn_baseline(self, value):
        if math.isfinite(value):
            self._baseline_samples.append(value)
        if len(self._baseline_samples) >= self.baseline_frames:
            self.baseline = float(statistics.median(self._baseline_samples))
            self.threshold = self.baseline - self.drop_margin

    def update(self, pos, roi_min):
        """한 프레임의 (주행좌표, ROI 최소뎁스)를 넣는다.

        baseline 이 아직 학습 중이면 이번 프레임은 baseline 표본으로만 쓰이고
        트로프 판정에는 참여하지 않는다.
        """
        if self.baseline is None:
            self._learn_baseline(roi_min)
            return

        if self._enter_pos is None:
            # 현재 트로프 밖 — 임계값 아래로 내려가는지 감시.
            if roi_min < self.threshold:
                if self._below_count == 0:
                    self._candidate_enter = pos
                self._below_count += 1
                if self._below_count >= self.confirm_frames:
                    self._enter_pos = self._candidate_enter
                    self._below_count = 0
                    self._candidate_enter = None
                    # 새 트로프 진입 — 이탈 디바운스 상태 초기화.
                    self._above_count = 0
                    self._candidate_exit = None
            else:
                self._below_count = 0
                self._candidate_enter = None
        else:
            # 현재 트로프 안 — 임계값 위로 회복하는지 감시.
            if roi_min >= self.threshold:
                if self._above_count == 0:
                    self._candidate_exit = pos
                self._above_count += 1
                if self._above_count >= self.confirm_frames:
                    enter_pos = self._enter_pos
                    exit_pos = self._candidate_exit
                    center = (enter_pos + exit_pos) / 2.0
                    self.troughs.append({"enter": enter_pos, "exit": exit_pos, "center": center})
                    self._enter_pos = None
                    self._above_count = 0
                    self._candidate_exit = None
            else:
                self._above_count = 0
                self._candidate_exit = None

    def axle_center(self, index=-1):
        """완료된 트로프 중 `index`(기본: 가장 최근)의 중간값. 없으면 None."""
        if not self.troughs:
            return None
        try:
            return self.troughs[index]["center"]
        except IndexError:
            return None

    def axle_centers(self):
        """완료된 트로프 전체의 중간값 리스트(발생 순서)."""
        return [t["center"] for t in self.troughs]
