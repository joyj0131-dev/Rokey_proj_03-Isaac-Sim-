"""LiDAR 포인트클라우드 → 슬롯별 점유 판단. 순수 Python (ROS import 금지).

통로 장애물 감지(core/obstacle_detector.py)와 같은 원리 — 영역 안에
바닥보다 높은 점이 임계값 이상이면 "점유"로 판단한다. 무엇이 점유했는지는
구분하지 않는다(주차 목적에는 있다/없다면 충분).

scripts/lidar/detect_occupancy.py(진단 이미지 생성 CLI 도구)와
slot_occupancy_node.py(실시간 ROS2 노드) 둘 다 이 모듈의 detect()를
공유한다 — 판정 기준이 두 곳에서 따로 관리되면 언젠가 어긋난다.
"""

from collections import deque

import numpy as np


HEIGHT_THRESHOLD_M = 0.15   # 이보다 낮으면 바닥/차선 노이즈로 간주하고 무시
POINT_THRESHOLD = 30        # 이 개수 이상이면 점유로 판단
STABILIZATION_FRAMES = 3

STATUS_WAITING = "WAITING"
STATUS_EMPTY = "EMPTY"
STATUS_OCCUPIED = "OCCUPIED"
STATUS_UNCERTAIN = "UNCERTAIN"


def detect_with_masks(
    points,
    parking_map,
    *,
    height_threshold=HEIGHT_THRESHOLD_M,
    point_threshold=POINT_THRESHOLD,
):
    """필터·슬롯 마스크와 프레임 단위 점유 판정을 한 번에 계산한다.

    RViz용 필터 cloud, 슬롯 집계 cloud, 슬롯 상태 메시지가 모두 이 결과를
    공유한다. 동일한 포인트를 여러 구현에서 다시 판정하지 않도록 한다.
    """
    points = np.asarray(points)
    if points.size == 0:
        points = np.empty((0, 3), dtype=np.float64)
    if points.ndim != 2 or points.shape[1] < 3:
        raise ValueError("points는 (N, 3) 이상의 배열이어야 합니다")

    W = parking_map.meta["params"]["space_width"]
    L = parking_map.meta["params"]["space_length"]
    finite = np.isfinite(points[:, :3]).all(axis=1)
    height_mask = finite & (points[:, 2] > height_threshold)
    any_slot_mask = np.zeros(len(points), dtype=bool)
    results = {}
    for slot_id in parking_map.nodes_of_kind("slot"):
        cx, cy = parking_map.node_pos(slot_id)
        in_box = (
            (points[:, 0] > cx - W / 2) & (points[:, 0] < cx + W / 2) &
            (points[:, 1] > cy - L / 2) & (points[:, 1] < cy + L / 2) &
            height_mask
        )
        any_slot_mask |= in_box
        count = int(in_box.sum())
        results[slot_id] = dict(
            occupied=count >= point_threshold,
            point_count=count,
            x=cx,
            y=cy,
            mask=in_box,
        )
    return results, height_mask, any_slot_mask


def detect(
    points,
    parking_map,
    *,
    height_threshold=HEIGHT_THRESHOLD_M,
    point_threshold=POINT_THRESHOLD,
):
    """기존 호출 호환용 슬롯별 단일 프레임 판정."""
    results, _height_mask, _slot_mask = detect_with_masks(
        points,
        parking_map,
        height_threshold=height_threshold,
        point_threshold=point_threshold,
    )
    return {
        slot_id: {
            key: value
            for key, value in result.items()
            if key != "mask"
        }
        for slot_id, result in results.items()
    }


class SlotDecisionStabilizer:
    """최근 N프레임이 모두 같을 때만 공석/점유를 확정한다."""

    def __init__(self, slot_ids, frames=STABILIZATION_FRAMES):
        if frames < 1:
            raise ValueError("frames는 1 이상이어야 합니다")
        self.frames = int(frames)
        self._history = {
            slot_id: deque(maxlen=self.frames) for slot_id in slot_ids
        }

    def update(self, frame_results):
        stabilized = {}
        for slot_id, result in frame_results.items():
            history = self._history.setdefault(
                slot_id, deque(maxlen=self.frames)
            )
            history.append(bool(result["occupied"]))
            if len(history) < self.frames:
                status = STATUS_WAITING
            elif all(history):
                status = STATUS_OCCUPIED
            elif not any(history):
                status = STATUS_EMPTY
            else:
                status = STATUS_UNCERTAIN
            stabilized[slot_id] = {
                **result,
                "status": status,
                "history_size": len(history),
            }
        return stabilized

    def reset(self):
        for history in self._history.values():
            history.clear()
