# 주차장 천장 센서 ROS2 연결

현재 주차장 USD의 고정형 RTX LiDAR 2대를 다음 계약으로 사용한다.

| 구역 | USD Prim | ROS2 topic | `frame_id` |
|---|---|---|---|
| 서쪽(A1~A4/B1~B4) | `/World/Sensors/CeilingLidarWest` | `/parking/lidar/ceiling_01/points` | `ceiling_lidar_01_link` |
| 동쪽(A5~A8/B5~B8) | `/World/Sensors/CeilingLidarEast` | `/parking/lidar/ceiling_02/points` | `ceiling_lidar_02_link` |

고정 카메라는 기존 `/World/OverviewCamera`를 사용한다.

| 데이터 | ROS2 topic | `frame_id` |
|---|---|---|
| RGB 영상 | `/parking/camera/overview/image_raw` | `parking_camera_overview_optical_frame` |
| 카메라 내부 파라미터 | `/parking/camera/overview/camera_info` | `parking_camera_overview_optical_frame` |

모든 센서 메시지는 Isaac simulation time을 사용하며 `/clock`을 함께 발행한다.

## 준비

ROS2 환경과 Isaac Sim 설치 경로를 지정한다. `ISAAC_SIM_PYTHON`은 자동 탐색되는
설치 위치라면 생략할 수 있다.

```bash
source /opt/ros/humble/setup.bash
export ISAAC_SIM_PYTHON=/path/to/isaac-sim/python.sh
```

Isaac Sim 5.1의 ROS2 bridge가 지원하는 ROS 배포판과 RMW를 사용해야 한다. 서로 다른
터미널에서는 `ROS_DOMAIN_ID`도 동일해야 한다.

## 1단계: ceiling_01 단독 검증

```bash
cd isaacpjt/Isaac_envo/parking
python3 run_ceiling_lidar_ros2.py --sensor ceiling_01
```

다른 ROS2 터미널에서 확인한다.

```bash
ros2 topic list | grep /parking/lidar
ros2 topic info /parking/lidar/ceiling_01/points --verbose
ros2 topic hz /parking/lidar/ceiling_01/points
ros2 topic echo /parking/lidar/ceiling_01/points --once --field header
```

메시지 타입은 `sensor_msgs/msg/PointCloud2`, `header.frame_id`는
`ceiling_lidar_01_link`, `width`는 0보다 커야 한다.

## 2단계: 두 센서 동시 검증

```bash
python3 run_ceiling_lidar_ros2.py --sensor both
ros2 topic hz /parking/lidar/ceiling_01/points
ros2 topic hz /parking/lidar/ceiling_02/points
```

두 토픽이 동시에 갱신되고 각 메시지의 `frame_id`가 서로 달라야 한다.

## 3단계: 카메라 및 전체 스트림 검증

카메라만 먼저 확인할 수 있다.

```bash
python3 run_ceiling_lidar_ros2.py --sensor none --camera
ros2 topic hz /parking/camera/overview/image_raw
ros2 topic echo /parking/camera/overview/camera_info --once
```

전체 센서는 다음과 같이 실행한다.

```bash
python3 run_ceiling_lidar_ros2.py --sensor both --camera --headless
```

관제 PC에서는 다섯 토픽을 확인한다.

```bash
ros2 topic hz /parking/lidar/ceiling_01/points
ros2 topic hz /parking/lidar/ceiling_02/points
ros2 topic hz /parking/camera/overview/image_raw
ros2 topic echo /parking/camera/overview/camera_info --once --field header
ros2 topic hz /clock
```

## 다음 단계

PointCloud2 송출을 실제 환경에서 확인한 뒤 이 저장소의 지도 기준 프레임인 `ros_map`에서
각 센서 프레임으로 가는 고정 TF와 `/clock`을 추가한다. RViz에서 두 점군의 위치와
축 방향이 맞는지 검증한 다음 카메라의 `image_raw`, `camera_info`를 연결한다. 외부
관제에서 반드시 `map`이라는 이름이 필요하다면 `ros_map`을 전부 바꾸기보다
`map -> ros_map` 정적 TF를 경계에서 한 번만 제공한다.
