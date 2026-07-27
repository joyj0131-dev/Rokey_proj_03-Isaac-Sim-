# LiDAR 슬롯 점유 분석 + RViz2 실행 안내

## 데이터 흐름

```text
Isaac Sim
  /parking/lidar/ceiling_01/points_usd
        ↓ ros_pointcloud_world_relay.py
  /parking/lidar/points_world (PointCloud2, frame=map)
        ↓ safety_monitor
        ├─ /parking/lidar/points_filtered
        ├─ /parking/lidar/points_in_slots
        ├─ /parking/slot_occupancy
        └─ /parking/slot_markers
                  ├─ FastAPI 웹 UI
                  └─ RViz2
```

`safety_monitor`가 높이 필터, 슬롯 집계, 3프레임 안정화를 한 번 수행한다.
웹 UI는 `/parking/slot_occupancy`를 구독하므로 RViz2 marker와 동일한 슬롯별
포인트 수와 판정 상태를 표시한다.

LiDAR 판정은 `parking_slots` DB 상태를 자동으로 덮어쓰지 않는다. 작업·예약
상태와 LiDAR 결과가 다르면 웹 UI에 `상태 불일치` 경고만 표시한다.

## 토픽

| 토픽 | 타입 | 용도 |
| --- | --- | --- |
| `/parking/lidar/points_world` | `sensor_msgs/msg/PointCloud2` | map 좌표 원본 |
| `/parking/lidar/points_filtered` | `sensor_msgs/msg/PointCloud2` | z > 0.15m |
| `/parking/lidar/points_in_slots` | `sensor_msgs/msg/PointCloud2` | A1~A3 집계 사용점 |
| `/parking/slot_occupancy` | `parking_robot_interfaces/msg/SlotOccupancyArray` | 웹과 RViz가 공유하는 판정 |
| `/parking/slot_markers` | `visualization_msgs/msg/MarkerArray` | 슬롯·라벨·L1 위치 |
| `/parking_status_markers` | `visualization_msgs/msg/MarkerArray` | 기존 도구 호환 |

## 빌드

```bash
cd /home/rokey/Desktop/feature_UI/Rokey_proj_03-Isaac-Sim-
source /opt/ros/humble/setup.bash
colcon build --symlink-install \
  --packages-select parking_robot_interfaces parking_control
source install/setup.bash
```

## 통합 실행

```bash
cd /home/rokey/Desktop/feature_UI/Rokey_proj_03-Isaac-Sim-
export ROS_DOMAIN_ID=126
export ROS_LOCALHOST_ONLY=0
./scripts/run_web_with_rviz.sh
```

- 웹 UI: `http://127.0.0.1:8000`
- RViz2 설정: `src/parking_control/config/slot_occupancy.rviz`
- RViz2 창만 닫아도 웹 서버는 유지된다.
- 실행 터미널에서 `Ctrl+C`를 누르면 스크립트가 시작한 자식 프로세스를 정리한다.

이미 Isaac PC가 `/parking/lidar/points_world`를 발행한다면 릴레이 중복을 피한다.

```bash
START_LIDAR_RELAY=0 ./scripts/run_web_with_rviz.sh
```

이미 관제 launch를 별도 터미널에서 실행했다면 다음처럼 중복 실행을 막는다.

```bash
START_CONTROL_TOWER=0 ./scripts/run_web_with_rviz.sh
```

## 확인 명령

```bash
ros2 topic hz /parking/lidar/points_world
ros2 topic hz /parking/lidar/points_filtered
ros2 topic hz /parking/lidar/points_in_slots
ros2 topic echo --once /parking/slot_occupancy
ros2 topic echo --once /parking/slot_markers
```

웹 모달과 `/parking/slot_occupancy`에서 다음 값이 같아야 한다.

- 슬롯별 `point_count`
- `EMPTY`, `OCCUPIED`, `WAITING`, `UNCERTAIN`
- 높이 임계값 0.15m
- 포인트 임계값 30개
- frame `map`

## 실제 데이터와 Mock 구분

- ROS2 모드: `source=ROS2_SHARED_OCCUPANCY`
- Mock 모드: `source=MOCK_SAMPLE`, 화면에 `샘플 데이터 시연` 표시

Mock cloud는 UI 회귀 테스트 전용이며 실제 센서 결과로 표시되지 않는다.
