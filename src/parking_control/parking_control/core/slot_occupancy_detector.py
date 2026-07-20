"""LiDAR 포인트클라우드 → 슬롯별 점유 판단. 순수 Python (ROS import 금지).

통로 장애물 감지(core/obstacle_detector.py)와 같은 원리 — 영역 안에
바닥보다 높은 점이 임계값 이상이면 "점유"로 판단한다. 무엇이 점유했는지는
구분하지 않는다(주차 목적에는 있다/없다면 충분).

scripts/lidar/detect_occupancy.py(진단 이미지 생성 CLI 도구)와
slot_occupancy_node.py(실시간 ROS2 노드) 둘 다 이 모듈의 detect()를
공유한다 — 판정 기준이 두 곳에서 따로 관리되면 언젠가 어긋난다.
"""

HEIGHT_THRESHOLD_M = 0.15   # 이보다 낮으면 바닥/차선 노이즈로 간주하고 무시
POINT_THRESHOLD = 30        # 이 개수 이상이면 점유로 판단


def detect(points, parking_map):
    """슬롯 id -> {occupied, point_count, x, y} 딕셔너리 반환."""
    W = parking_map.meta["params"]["space_width"]
    L = parking_map.meta["params"]["space_length"]
    results = {}
    for slot_id in parking_map.nodes_of_kind("slot"):
        cx, cy = parking_map.node_pos(slot_id)
        in_box = (
            (points[:, 0] > cx - W / 2) & (points[:, 0] < cx + W / 2) &
            (points[:, 1] > cy - L / 2) & (points[:, 1] < cy + L / 2) &
            (points[:, 2] > HEIGHT_THRESHOLD_M)
        )
        count = int(in_box.sum())
        results[slot_id] = dict(
            occupied=count >= POINT_THRESHOLD, point_count=count, x=cx, y=cy)
    return results
