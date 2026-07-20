"""천장 LiDAR 센서 로컬 좌표 → 저희 주차장 월드 좌표 변환. 순수 Python.

★★★ 검증 안 된 부분이 있다 — 실제 데이터로 반드시 확인할 것 ★★★

rokey님이 만든 run_ceiling_lidar_ros2.py(2026-07-20, 커밋 35617de)가
서쪽/동쪽 LiDAR 2대를 각각 아래 실제 설치값으로 발행한다
(build_parking_environment.py의 add_ceiling_lidars() 기준):

  half_w = 17.0m (space_count=10 * space_width=3.4 / 2, 우리 parking_map.yaml과 동일)
  LIDAR_X_FRACTION = 0.46, CEILING_HEIGHT = 5.60
  서쪽: USD 위치 (-half_w*0.46, CEILING_HEIGHT-0.48, 0) = (-7.82, 5.12, 0)
  동쪽: USD 위치 (+7.82, 5.12, 0)
  방향(둘 다 동일): USD X축 기준 +90도 회전 (로컬 +Z가 천장 아래쪽/-Y를 향하도록)

이 모듈은 위 알려진 설치 정보(회전+이동)로 정적 변환을 계산한다. **다만
포인트클라우드 메시지 안의 raw (x,y,z)가 정확히 "USD 로컬 좌표와 같은
축 라벨링"을 따른다는 가정 하나는 여기서 검증할 수 없었다** — Isaac Sim
ROS2 브릿지가 REP-103(ROS 표준: Z-up, X-forward) 변환을 내부적으로
어떻게 적용하는지는 실제로 돌려봐야 확실하다. 아래 verify_with_known_point()로
반드시 실측 검증할 것 (예: 입구에 서 있는 사람 1명의 변환된 좌표가
entrance 노드 근처(x≈-18.1, y≈0)로 나오는지 확인).

좌표 규약(이 프로젝트 전체와 동일): ros_x = usd_x, ros_y = -usd_z,
ros_z(높이) = usd_y.
"""

import numpy as np

# --- 알려진 설치값 (build_parking_environment.py 기준, 검증됨) ---
LIDAR_X_FRACTION = 0.46
CEILING_HEIGHT_M = 5.60
SENSOR_Y_OFFSET_M = 0.48  # 마운트가 천장에서 살짝 내려온 정도


def sensor_offsets(half_w_m):
    """(서쪽 x오프셋, 동쪽 x오프셋, 공통 높이오프셋) — 전부 usd 단위."""
    x = half_w_m * LIDAR_X_FRACTION
    height = CEILING_HEIGHT_M - SENSOR_Y_OFFSET_M
    return -x, x, height


def transform_to_world(local_points, x_offset_usd, height_offset_usd):
    """센서 로컬 (N,3) 배열 → 월드 ros 좌표 (N,3) 배열.

    회전은 USD X축 +90도 고정(두 센서 공통, build 스크립트의 쿼터니언과
    일치). R_x(90°) = [[1,0,0],[0,0,-1],[0,1,0]] 이므로
    (lx,ly,lz) -> (lx, -lz, ly): world_usd = (lx, -lz, ly) + (x_offset, height_offset, 0).
    (로컬 +Z로 나아갈수록 world_usd_y=높이가 줄어든다 — 천장 센서가
    아래를 보는 것과 일치. 여기서 ly/lz를 바꿔 쓰면 조용히 틀린 좌표가
    나오므로 test_lidar_frame_transform.py로 반드시 회귀 확인한다.)
    그 뒤 프로젝트 공통 규약으로 ros 좌표 변환.
    """
    if local_points.size == 0:
        return local_points
    lx, ly, lz = local_points[:, 0], local_points[:, 1], local_points[:, 2]
    world_usd_x = lx + x_offset_usd
    world_usd_y = -lz + height_offset_usd  # 높이
    world_usd_z = ly                        # (usd z 오프셋은 0)
    ros_x = world_usd_x
    ros_y = -world_usd_z
    ros_z = world_usd_y
    return np.column_stack([ros_x, ros_y, ros_z])


def verify_with_known_point(transformed_points, expected_xy, tolerance_m=2.0):
    """실측 검증용 헬퍼. transformed_points 중 expected_xy 근처(공차 이내)에
    점이 있는지 확인 — 알고 있는 위치(예: 입구에 서 있는 사람)로 좌표
    변환이 대략 맞는지 눈으로/코드로 검증할 때 쓴다."""
    if transformed_points.size == 0:
        return False
    dist = np.hypot(transformed_points[:, 0] - expected_xy[0],
                    transformed_points[:, 1] - expected_xy[1])
    return bool((dist < tolerance_m).any())
