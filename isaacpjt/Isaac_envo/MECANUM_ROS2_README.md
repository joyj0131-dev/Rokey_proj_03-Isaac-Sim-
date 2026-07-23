# 메카넘 로봇 ROS 2 주행 (Isaac Sim ↔ 외부 노트북)

Isaac Sim에서 메카넘 주차 로봇을 띄우고, **다른 노트북에서 `/cmd_vel`을 발행**하면 로봇이
움직인다. 핵심은 **Python 3.11(Isaac) vs 3.10(ROS 2 Humble)** 문제를 우회하는 설계다.

## 왜 Python 버전이 문제인가 (그리고 어떻게 우회했나)

- Isaac Sim 5.1은 **Python 3.11**로 동작한다. ROS 2 Humble은 **Python 3.10**(Ubuntu 22.04)용이다.
  그래서 Isaac의 3.11에서 시스템 `rclpy`(3.10)를 `import`하면 ABI가 안 맞아 깨진다. → **Isaac 안에서
  rclpy를 쓰지 않는다.**
- 대신 **C++ OmniGraph ROS 2 브리지**(`isaacsim.ros2.bridge`)의 **내부 Humble 라이브러리**(3.11용)를 쓴다.
  브리지가 `/cmd_vel`을 DDS로 구독하고, 시뮬 루프에서 그 값을 읽어 **검증된 메카넘 역기구학**
  (`mecanum_drive.wheel_velocities_from_cmd_vel`)으로 4휠 속도를 준다.
- **외부 노트북(3.10)** 은 평범한 ROS 2 Humble로 `/cmd_vel`을 발행한다. 둘은 **DDS 와이어**로 통신하므로
  Python 버전은 무관하다.

```
Isaac 머신 (Python 3.11)                 외부 노트북 (Python 3.10)
┌──────────────────────────────┐        ┌──────────────────────────┐
│ mecanum_ros2_drive.py        │        │ ros2 topic pub /cmd_vel  │
│  OmniGraph ROS2SubscribeTwist│◄──DDS─►│  (또는 teleop / 내 노드)  │
│  → 메카넘 IK → 휠 velocity    │/cmd_vel│                          │
│  (내부 Humble libs, rclpy 안씀)│        │  (시스템 Humble, 3.10)   │
└──────────────────────────────┘        └──────────────────────────┘
```

## ⚠️ 가장 중요한 규칙 (Isaac 5.1)

**Isaac 쪽 터미널에서는 `/opt/ros`를 source 하지 말 것.** 대신 아래 4개 export만 하고 실행한다.
소싱하면 브리지가 3.10 rclpy/rmw를 잡으려다 실패하고, 다음 증상이 난다:

```
Could not import system rclpy: No module named 'rclpy._rclpy_pybind11'
failed to load librmw_fastrtps_cpp.so: cannot open shared object file
OmniGraphError: Could not create node ... 'isaacsim.ros2.bridge.ROS2SubscribeTwist'
```

## ① Isaac 머신에서 로봇 띄우기

에셋 빌드(최초 1회, 이미 되어 있음):
```bash
cd $HOME/dev_ws/isaac_sim/isaacsim/_build/linux-x86_64/release
./python.sh /home/rokey/cobot3_ws/isaacpjt/hwia_parking_robot_final_caster_package/build_mecha_roller_asset.py
```

**팀 표준 export (시스템 ROS 2는 source 하지 않은 새 터미널에서):**
```bash
export ROS_DOMAIN_ID=122                      # 팀에 맞게 수정
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE="$HOME/.ros/fastdds_whitelist.xml"
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$HOME/dev_ws/isaac_sim/isaacsim/_build/linux-x86_64/release/exts/isaacsim.ros2.bridge/humble/lib
```

드라이버 실행:
```bash
cd $HOME/dev_ws/isaac_sim/isaacsim/_build/linux-x86_64/release
./python.sh /home/rokey/cobot3_ws/isaacpjt/Isaac_envo/mecanum_ros2_drive.py --gui
```

또는 위 4개 export를 대신 해주는 **런처 한 줄**(어떤 셸에서든 안전):
```bash
/home/rokey/cobot3_ws/isaacpjt/Isaac_envo/run_mecanum_ros2_drive.sh --gui
# 도메인 바꾸기:  ROS_DOMAIN_ID=7 .../run_mecanum_ros2_drive.sh --gui
```
자동으로 Play되고 `/cmd_vel`을 구독한다. 콘솔에 `ROS2_DRIVE_READY topic=/cmd_vel domain=122 ...`이
뜨면 준비 완료. (인자 없이 실행하면 headless. `--seconds N`으로 자동 종료.)

## FastDDS 화이트리스트 (`~/.ros/fastdds_whitelist.xml`)

팀 폐쇄망만 쓰도록 DDS 인터페이스를 제한한다(현재: `127.0.0.1`, `10.10.0.1~5`,
`useBuiltinTransports=false`). **Isaac 머신과 발행 노트북 양쪽 모두** 같은 화이트리스트를 써야
서로 디스커버리된다. 같은 머신 테스트는 `127.0.0.1` 덕분에 된다. 새 로봇/노트북을 붙이면 그 IP를
화이트리스트에 추가한다.

## ② 외부 노트북(Python 3.10)에서 /cmd_vel 발행

```bash
export ROS_DOMAIN_ID=122                      # Isaac과 동일
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE="$HOME/.ros/fastdds_whitelist.xml"   # 같은 화이트리스트
source /opt/ros/humble/setup.bash             # 발행 쪽은 시스템 ROS 2 사용 OK

# 전진 (linear.x = 로봇 전방)
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.4}}'
# 좌 strafe (linear.y = 로봇 좌측)
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist '{linear: {y: 0.4}}'
# 제자리 회전 (angular.z = yaw, CCW+)
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist '{angular: {z: 0.5}}'

# 키보드 텔레옵
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```
Twist 규약: `linear.x`=전진, `linear.y`=좌, `angular.z`=yaw(CCW+). 메카넘이라 세 축 동시 가능.

## 팔 열기/접기 (서비스)

드라이버는 `/arm_control` **서비스**(`std_srvs/srv/SetBool`)를 연다. `data: true`면 네 swing arm을
±90°로 전개하고, **다 열린 뒤** `success: true, message: "arms fully opened"`로 응답한다(접기는 false).
서비스 서버는 Isaac 내부 rclpy(py3.11)로 돌고, cmd_vel(OmniGraph)과 같은 프로세스에서 공존한다.

```bash
ros2 service call /arm_control std_srvs/srv/SetBool '{data: true}'    # 팔 벌리기 (완료까지 대기)
ros2 service call /arm_control std_srvs/srv/SetBool '{data: false}'   # 팔 접기
```

## 방향키 텔레옵 (`mecanum_teleop_key.py`)

외부 머신(시스템 ROS 2)에서 실행하면 키보드로 주행 + 팔 제어를 한 번에 한다.
```bash
export ROS_DOMAIN_ID=122
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE="$HOME/.ros/fastdds_whitelist.xml"
source /opt/ros/humble/setup.bash
python3 /home/rokey/cobot3_ws/isaacpjt/Isaac_envo/mecanum_teleop_key.py
```
키: `↑↓` 전/후진, `←→` 좌/우 strafe, `a/d` 좌/우 회전, `o/c` 팔 열기/접기, `space` 정지, `x` 종료.
방향키는 누르고 있으면 계속 이동하고 떼면 멈춘다.

## 맞춰야 하는 것 (양쪽)

| 항목 | 값 | 규칙 |
|---|---|---|
| `ROS_DOMAIN_ID` | `122` (팀) | 양쪽 동일 |
| `RMW_IMPLEMENTATION` | `rmw_fastrtps_cpp` | 양쪽 동일 |
| `FASTRTPS_DEFAULT_PROFILES_FILE` | `~/.ros/fastdds_whitelist.xml` | 양쪽 동일(IP 화이트리스트) |
| 네트워크 | 폐쇄망 `10.10.0.x` + loopback | 각 머신 IP가 화이트리스트에 있어야 |

## 디스커버리 확인

```bash
ros2 topic list                 # /cmd_vel 보여야 함
ros2 topic info /cmd_vel -v      # 드라이버가 뜬 상태면 Subscriber(Isaac) 1개
ros2 topic hz /cmd_vel           # 발행 중이면 주기 표시
```

## 파일

- `run_mecanum_ros2_drive.sh` — 팀 env(도메인/rmw/화이트리스트/내부 libs) 세팅 후 드라이버 실행.
- `mecanum_ros2_drive.py` — Isaac 드라이버(브리지 구독 → IK → 휠).
- `mecanum_drive.py` — 롤러 authoring + cmd_vel 역기구학(재사용 모듈).
- `hwia_parking_robot_final_caster_mecha_roller.usd` — flatten 자체포함 메카넘 로봇 에셋.
