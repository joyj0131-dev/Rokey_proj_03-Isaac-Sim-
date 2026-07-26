"""통로(존) 장애물 감지 — 순수 Python 판정 로직 (ROS import 금지).

주차 슬롯 점유 판정(scripts/lidar/detect_occupancy.py)과 같은 원리다:
영역 안에 바닥보다 높은 점이 임계값 이상 있으면 "막힘"으로 판단한다.
무엇이 막았는지(사람/카트/기타)는 구분하지 않는다 — ObstacleAlert.msg가
애초에 불리언 하나뿐이라 구분이 필요 없다.

로봇 자신도 물리적으로 존재하므로 LiDAR에는 장애물처럼 찍힌다. 그래서
판정 전에 알려진 로봇 위치 주변의 점을 먼저 제외한다(ROBOT_EXCLUDE_RADIUS_M)
— 이걸 빼먹으면 로봇이 자기 자신을 장애물로 감지해서 스스로 멈추는
사고가 난다.
"""

HEIGHT_THRESHOLD_M = 0.15
POINT_THRESHOLD = 20              # 슬롯(30)보다 낮게 잡음 — 통로가 더 좁아서
ROBOT_EXCLUDE_RADIUS_M = 0.8      # 로봇 풋프린트보다 넉넉하게


def zone_boxes(parking_map):
    """존 id -> 월드 좌표의 축 정렬 감지 영역 ``(x0, x1, y0, y1)``.

    v4 지도는 입·출차 수평 차로와 슬롯으로 이어지는 수직 진입로가 섞여
    있다. 엣지의 실제 두 끝점을 기준으로 박스를 만들고 진행 방향의
    수직축에만 통로 폭을 더한다. 슬롯 직전 ``*_dock`` 구간은 전체 차로
    폭을 적용하면 인접 주차면까지 겹치므로 주차면 폭을 기준으로 제한한다.
    """
    aisle_half = parking_map.meta["params"]["aisle_width"] / 2
    slot_half = parking_map.meta["params"].get(
        "space_width", parking_map.meta["params"]["aisle_width"]
    ) * 0.45
    boxes = {}
    for u, v, data in parking_map.graph.edges(data=True):
        zone_id = data.get("zone")
        if not zone_id:
            continue
        (x1, y1), (x2, y2) = parking_map.node_pos(u), parking_map.node_pos(v)
        half_width = slot_half if zone_id.endswith("_dock") else aisle_half
        dx = abs(x2 - x1)
        dy = abs(y2 - y1)
        if dx >= dy:
            boxes[zone_id] = (
                min(x1, x2),
                max(x1, x2),
                min(y1, y2) - half_width,
                max(y1, y2) + half_width,
            )
        else:
            boxes[zone_id] = (
                min(x1, x2) - half_width,
                max(x1, x2) + half_width,
                min(y1, y2),
                max(y1, y2),
            )
    return boxes


def _exclude_near_robots(points, robot_positions, radius):
    if not robot_positions:
        return points
    keep = None
    for rx, ry in robot_positions:
        dist2 = (points[:, 0] - rx) ** 2 + (points[:, 1] - ry) ** 2
        mask = dist2 > radius * radius
        keep = mask if keep is None else (keep & mask)
    return points[keep]


def detect_blocked_zones(points, boxes, robot_positions=()):
    """존 id -> 막혔는지(bool). robot_positions=[(x,y), ...]는 자기 자신을
    장애물로 오인하지 않도록 미리 빼는 로봇들의 현재 위치."""
    clean = _exclude_near_robots(points, robot_positions, ROBOT_EXCLUDE_RADIUS_M)
    results = {}
    for zone_id, (x0, x1, y0, y1) in boxes.items():
        mask = (
            (clean[:, 0] > x0) & (clean[:, 0] < x1) &
            (clean[:, 1] > y0) & (clean[:, 1] < y1) &
            (clean[:, 2] > HEIGHT_THRESHOLD_M)
        )
        results[zone_id] = bool(int(mask.sum()) >= POINT_THRESHOLD)
    return results
