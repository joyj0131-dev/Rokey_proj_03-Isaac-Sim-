"""천장 LiDAR 센서 로컬 좌표 → 저희 주차장 월드 좌표 변환. 순수 Python.

★★★ 검증 안 된 부분이 있다 — 실제 데이터로 반드시 확인할 것 ★★★

2026-07-24: v3.usd(Sensors 스코프) 확인 결과, 천장 LiDAR가 예전(2026-07-20,
서/동 2대) 구성에서 **1대로 통합**됐다(sensor:role="주차장 전체 커버리지(1대
통합)", sensor:coverageZone="all"). 아래는 그 1대의 실측 설치값이다:

  CeilingLidarCenter: USD translate (0.5, 5.12, 0), quatf orient
  (0.70710677, 0.70710677, 0, 0) — 상위 Xform(Sensors/CeilingLidarCenterMount)엔
  별도 translate가 없어 이 값이 곧 월드 기준 절대 위치다.
  회전은 예전 2대와 동일한 컨벤션(쿼터니언 w=x=0.7071 → X축 기준 +90도)이라
  회전 변환식은 그대로 재사용했다.
  높이(5.12m)도 예전 계산식(CEILING_HEIGHT_M - SENSOR_Y_OFFSET_M = 5.60-0.48)과
  정확히 일치 — 천장 높이/마운트 처짐 자체는 안 바뀐 것으로 보인다.

**다만 포인트클라우드 메시지 안의 raw (x,y,z)가 정확히 "USD 로컬 좌표와 같은
축 라벨링"을 따른다는 가정 하나는 여기서 검증할 수 없었다** — Isaac Sim
ROS2 브릿지가 REP-103(ROS 표준: Z-up, X-forward) 변환을 내부적으로
어떻게 적용하는지는 실제로 돌려봐야 확실하다. 아래 verify_with_known_point()로
반드시 실측 검증할 것(예: 슬롯 A2 위에 서 있는 사람 1명의 변환된 좌표가
A2 근처(x≈6.2, y≈0)로 나오는지 확인) — 2026-07-20에 이 파일의 서/동 2대
버전으로 한 번 검증(pytest 회귀 + 시뮬레이션 점군 end-to-end)까지는 했었지만,
그건 그 시점의 2대 배치 기준이었고 지금 1대 통합 배치는 아직 실측 검증 전이다.

좌표 규약(이 프로젝트 전체와 동일): ros_x = usd_x, ros_y = -usd_z,
ros_z(높이) = usd_y.
"""

import numpy as np

# --- 알려진 설치값(v3.usd Sensors 스코프, 2026-07-24 확인) ---
LIDAR_X_OFFSET_M = 0.5     # CeilingLidarCenter translate.x(USD, 곧 월드 x)
CEILING_HEIGHT_M = 5.60
SENSOR_Y_OFFSET_M = 0.48   # 마운트가 천장에서 살짝 내려온 정도(예전과 동일)


def sensor_offset():
    """(x오프셋, 높이오프셋) — 전부 usd 단위. 1대 통합 배치라 인자가 필요 없다
    (예전엔 half_w_m을 받아 서/동 위치를 계산했으나 이제 고정 위치 하나뿐)."""
    height = CEILING_HEIGHT_M - SENSOR_Y_OFFSET_M
    return LIDAR_X_OFFSET_M, height


def transform_to_world(local_points, x_offset_usd, height_offset_usd):
    """센서 로컬 (N,3) 배열 → 월드 ros 좌표 (N,3) 배열.

    회전은 USD X축 +90도 고정(build 스크립트의 쿼터니언과 일치). R_x(90°) =
    [[1,0,0],[0,0,-1],[0,1,0]] 이므로 (lx,ly,lz) -> (lx, -lz, ly):
    world_usd = (lx, -lz, ly) + (x_offset, height_offset, 0).
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
    점이 있는지 확인 — 알고 있는 위치(예: 슬롯 위에 서 있는 사람)로 좌표
    변환이 대략 맞는지 눈으로/코드로 검증할 때 쓴다."""
    if transformed_points.size == 0:
        return False
    dist = np.hypot(transformed_points[:, 0] - expected_xy[0],
                    transformed_points[:, 1] - expected_xy[1])
    return bool((dist < tolerance_m).any())
