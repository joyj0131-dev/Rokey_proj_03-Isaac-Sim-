# isaacpjt/Isaac_envo — Isaac Sim 브리지 & 지원 스크립트

이 폴더는 **Isaac Sim(py3.11) 쪽**만 담는다. 4대 로봇을 실제로 제어·측위·감지·
지휘하는 **ROS2 노드는 여기가 아니라 `src/parkbot_motion`·`src/parkbot_aruco`**
패키지에 있다. 둘은 DDS(domain 126)로 통신한다 — 전체 구조는
[docs/concepts/ros2-node-architecture.md](../../docs/concepts/ros2-node-architecture.md).

## 루트 파일

| 파일 | 역할 |
|---|---|
| `sim_bridge.py` / `.sh` | **Isaac(py3.11) 시뮬 브리지**. 씬/로봇/차량/마커 스폰, /odom·image·depth 발행 + /cmd_vel·/lift_cmd 구독. 인자 없이 = 도크 스폰 + 전후방 카메라(Phase B 모드). 오직 이것만이 Isaac 쪽 코드다 |
| `launch/pickup_mission.launch.py` | **전체 미션 launch**. sim_bridge 기동 → BRIDGE_READY 감지 시 ROS2 노드 스택 기동 → orchestrator 가 자율 실행 |
| `run_*_node.sh`, `run_*_action_server.sh` | 개별 ROS2 노드 런처(시스템 ROS2 Humble 환경 세팅 + `--ros-args` passthrough). launch 가 이걸 감싼다 |
| `run_marker_localizer_node.sh`, `run_probe_a_detector.sh` | parkbot_aruco 노드 런처 |
| `build_*.py`, `marker_layout.py` | USD 자산·마커 지도 생성(일회성). 서로 import 하는 응집 그룹 |
| `mecanum_drive.py` | 메카넘 USD 저작 + 기구학(sim_bridge 가 import) |

## 하위 폴더

- **`smoke/`** — 단계별 스모크 테스트(개발 진단용). `*_smoke.py` + `run_*_smoke.sh`.
- **`legacy/`** — 옛 v2 러너(`dock_lift_handoff_runner*`). 참조용, 현재 미사용.

## 빠른 실행

```bash
# 전체 미션(Isaac 브리지 + ROS2 노드 스택 + 자율 orchestrator)을 launch 하나로
source /opt/ros/humble/setup.bash
ros2 launch isaacpjt/Isaac_envo/launch/pickup_mission.launch.py
# 정리: Ctrl-C (launch 가 자식 시그널). 잔여: pkill -9 -f 'sim_bridge.py|parkbot_(aruco|motion)'
```

미션은 orchestrator 의 `auto_start`(기본 launch 에서 true)로 **자율 실행**된다 — 외부
트리거 스크립트가 필요 없다. 개별 노드를 수동 기동/디버그하려면 `run_*_node.sh` 를
직접 쓰면 된다.
