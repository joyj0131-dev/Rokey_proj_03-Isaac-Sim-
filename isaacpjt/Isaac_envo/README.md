# isaacpjt/Isaac_envo — Isaac Sim 브리지 & 지원 스크립트

이 폴더는 **Isaac Sim(py3.11) 쪽**만 담는다. 4대 로봇을 실제로 제어·측위·감지·
지휘하는 **ROS2 노드는 여기가 아니라 `src/parkbot_motion`·`src/parkbot_aruco`**
패키지에 있다. 둘은 DDS(domain 126)로 통신한다 — 전체 구조는
[docs/concepts/ros2-node-architecture.md](../../docs/concepts/ros2-node-architecture.md).

## 루트 파일

| 파일 | 역할 |
|---|---|
| `sim_bridge.py` / `.sh` | **Isaac(py3.11) 시뮬 브리지**. 씬/로봇/차량/마커 스폰, /odom·image·depth 발행 + /cmd_vel·/lift_cmd 구독. 인자 없이 = 도크 스폰 + 전후방 카메라(Phase B 모드). `--gui` 로 뷰포트 창. **오직 이것만이 Isaac 쪽 코드다** |
| `build_*.py`, `marker_layout.py` | USD 자산·마커 지도 생성(일회성). 서로 import 하는 응집 그룹 |
| `mecanum_drive.py` | 메카넘 USD 저작 + 기구학(sim_bridge 가 import) |

ROS2 노드 런처(구 `run_*.sh`)와 launch 는 이 폴더를 떠났다 — 이제 ROS2 네이티브
(`ros2 launch` / `ros2 run`)로 돈다. 아래 참고.

## 빠른 실행 (터미널 2개 = 두 컴퓨터)

```bash
# 터미널 A · Isaac(py3.11) 브리지
bash isaacpjt/Isaac_envo/sim_bridge.sh          # 관전하려면 --gui. BRIDGE_READY 대기(~2분)

# 터미널 B · 순수 ROS2(py3.10) 노드 스택 — env 는 launch 가 세팅
source /opt/ros/humble/setup.bash
source install/setup.bash                        # 콜콘 빌드 후 1회
ros2 launch parkbot_motion nodes.launch.py       # marker_localizer×2/pose×4/axle×2/
                                                 #  ingress×2/lift×2/carry/orchestrator
# 정리: 각 터미널 Ctrl-C. 잔여: pkill -9 -f 'sim_bridge.py|parkbot_(aruco|motion)'
```

미션은 orchestrator 의 `auto_start`(launch 기본 true)로 **자율 실행**된다.

개별 노드를 수동 기동/디버그하려면 env 를 export 하고 `ros2 run`. **FastDDS
화이트리스트는 export 하지 말 것** — 켜면 Isaac 브리지(3.11 fastdds)와 discovery 가
깨진다(실측 2026-07-21). 이미 셸에 걸려 있으면 `unset` 하고 실행:

```bash
export ROS_DOMAIN_ID=126 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
unset FASTRTPS_DEFAULT_PROFILES_FILE      # 화이트리스트 걸려 있으면 해제
ros2 run parkbot_motion carry_action_server
```
