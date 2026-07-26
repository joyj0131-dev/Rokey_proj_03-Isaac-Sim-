# isaacpjt/Isaac_envo — Isaac Sim 브리지 & 지원 스크립트

이 폴더는 **Isaac Sim(py3.11) 쪽**만 담는다. 4대 로봇을 실제로 제어·측위·감지·
지휘하는 **ROS2 노드는 여기가 아니라 `src/parkbot_motion`·`src/parkbot_aruco`**
패키지에 있다. 둘은 DDS(domain 126)로 통신한다 — 전체 구조는
[docs/concepts/ros2-node-architecture.md](../../docs/concepts/ros2-node-architecture.md).

## 루트 파일

| 파일 | 역할 |
|---|---|
| `parking_v4_runner.py` / `.sh` | **Isaac 시뮬 브리지**. `--bridge` 로 /odom·image·depth 발행 + /cmd_vel·/lift_cmd 구독. `--probe=<X>` 로 진단 프로브 13종. (미션 안무는 R6 에서 ROS2 로 이전됨) |
| `bringup_pickup_e2e.sh` | **전체 미션 기동/정리**(브리지 + ROS2 노드 스택 13개). `up`/`down`/`status` |
| `run_*_node.sh`, `run_*_action_server.sh` | 개별 ROS2 노드 런처(시스템 ROS2 Humble 환경 세팅 후 `src/` 노드 실행) |
| `run_marker_localizer_v4.sh`, `run_probe_a_detector.sh` | parkbot_aruco 노드 런처 |
| `build_*.py`, `marker_layout.py` | USD 자산·마커 지도 생성(일회성). 서로 import 하는 응집 그룹 |
| `mecanum_drive.py` | 메카넘 USD 저작 + 러너용 재익스포트 shim |
| `v4_probes.py` | 프로브 지원 |

## 하위 폴더

- **`smoke/`** — 단계별 스모크 테스트. `*_smoke.py`(클라이언트) + `run_*_smoke.sh`(런처).
  예: `bash smoke/run_pickup_smoke.sh --leader=entry_lead --follower=entry_follow`
- **`legacy/`** — 옛 v2 러너(`dock_lift_handoff_runner*`). 참조용, 현재 미사용.

## 빠른 실행

```bash
bash bringup_pickup_e2e.sh up          # 전체 스택 기동(BRIDGE_READY 까지)
bash smoke/run_pickup_smoke.sh --leader=entry_lead --follower=entry_follow  # 안무 트리거
bash bringup_pickup_e2e.sh down        # 정리
```
