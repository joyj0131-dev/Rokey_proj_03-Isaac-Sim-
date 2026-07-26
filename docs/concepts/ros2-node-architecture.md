# v4 밸릿파킹 ROS2 노드 아키텍처

> R6 정리(2026-07-26). 제어·측위·감지·안무가 어떻게 **두 프로세스가 DDS로
> 통신하며 4대 로봇을 제어**하는지 — "한 파일에 다 몰아넣은 게 아니라"는 것을
> 실행 그래프로 보인다.

## 왜 두 프로세스인가 (버릴 수 없는 제약)

Isaac Sim 5.1 은 **Python 3.11**, ROS2 Humble 은 **Python 3.10** 이라 한
인터프리터에 못 올린다. 그래서 시스템은 항상 두 프로세스로 나뉜다:

- **Isaac 프로세스(py3.11)** — 물리·렌더·센서. `sim_bridge.py`(전용 얇은 브리지).
- **ROS2 프로세스들(py3.10)** — 제어·측위·감지·안무. `src/` 의 노드 패키지.

둘은 **DDS(ROS_DOMAIN_ID=126, FastRTPS)** 토픽/액션으로만 통신한다. Isaac 은
"무엇을 명령할지" 전혀 모른다 — 바퀴 명령을 받고 센서를 내보내는 시뮬 I/O 일 뿐.

## 실행 그래프 (launch/pickup_mission.launch.py 가 띄우는 13개 프로세스)

```
┌─ Isaac 프로세스 (py3.11) ─────────────┐          ┌─ ROS2 노드들 (py3.10, src/) ──────────────┐
│ sim_bridge.py (얇은 전용 브리지)        │  pub →   │ marker_localizer_node × 2                  │
│  (isaacpjt/Isaac_envo/, 시뮬 브리지)   │ /odom    │   (parkbot_aruco) 전방+후방캠→마커융합측위 │
│                                        │ /image   │ pose_controller_node × 4                   │
│  · 씬/로봇/차량/마커 스폰              │ /depth   │   (parkbot_motion) NavigateToPose 주행제어 │
│  · 카메라 그래프(C++ OmniGraph)        │─────────▶│ axle_detector_node × 2                     │
│  · /robot_<id>/odom·image·depth 발행   │          │   (parkbot_motion) 측면뎁스→축중심 검출    │
│  · /joint_states 발행                  │          │ ingress_node × 2                           │
│                                        │  ← sub   │   (parkbot_motion) 차밑 진입+축정지         │
│  · /robot_<id>/cmd_vel 구독→휠속도     │ /cmd_vel │ lift_action_server × 2                     │
│  · /robot_<id>/lift_cmd 구독→팔전개    │ /lift_cmd│   (parkbot_motion) ControlLift 리프트       │
│  (진단 프로브·미션코드 없음 — 순수 브리지)│          │ pickup_orchestrator_node × 1(auto_start)   │
└────────────────────────────────────────┘          │   (parkbot_motion) 전체 안무 지휘           │
                                                     └────────────────────────────────────────────┘
```

로봇별로 노드 인스턴스가 뜬다(entry_lead/entry_follow) — 그래서 pose_controller
4개(로봇당 odom용·융합용 2개), localizer/axle/ingress/lift 각 2개다. **4대 로봇을
제어하는 로직은 전부 ROS2 노드 쪽에 있고, Isaac 파일에는 없다.**

## 각 노드의 책임

| 노드 (패키지) | 입력 | 출력 | 하는 일 |
|---|---|---|---|
| `marker_localizer_node` (parkbot_aruco) | 전/후방 image, /odom | `/robot_<id>/pose` | ArUco 검출→월드측위, 휠오도와 상보필터 융합. ref 마커 하드필터(런타임 전환), 위치전용 보정, 이중카메라-단일필터, seed_pose |
| `pose_controller_node` (parkbot_motion) | /pose 또는 /odom | /cmd_vel | `NavigateToPose` 액션. 목표 자세로 폐루프 주행(순수 정책 `PoseController`) |
| `axle_detector_node` (parkbot_motion) | 측면 depth, 자세 | `/robot_<id>/axle_center` | 뎁스 트로프 진입/이탈 **중간값**으로 축 중심 검출 |
| `ingress_node` (parkbot_motion) | depth, axle_center, 자세 | /cmd_vel | `IngressUnderTruck`. 차밑 진입+뎁스 횡중앙유지+축 정지 |
| `lift_action_server` (parkbot_motion) | — | /lift_cmd | `ControlLift`(UP/DOWN) 램프 전개 |
| `pickup_orchestrator_node` (parkbot_motion) | 위 액션들 | 위 액션 호출 | `ExecuteParkingTask`. Phase B(도크→XN)+픽업(접근→진입→리프트) 안무를 로봇별 액션으로 스태거 지휘 |

## 데이터 흐름 (전체 미션 1회)

```
도크 스폰 ─(orchestrator)─▶ Phase B 레그(로봇별 스태거)
  seed_dock → rotate_90(odom) → dock_check(후방캠) → xn_align(전방캠) → offset
           │  localizer ref_ids 전환(도크21/23→XN31), 위치전용 보정
           ▼
XN 종단 ──▶ 픽업 회랑(스태거)
  approach → align(융합자세) → ingress(뎁스 축감지 정지)
           ▼
양 로봇 동시 LIFT ──▶ 트럭 상승
```

## sim_bridge.py (Isaac 브리지)에 있는 것 / 없는 것

- **있다**: 씬/자산/마커 스폰, 카메라 OmniGraph, 브리지 I/O(cmd_vel·lift_cmd
  구독 / odom·image·depth 발행). 단일 책임 = 시뮬 I/O.
- **없다**: 인프로세스 미션 안무, 진단 프로브 13종, 모드 분기 — 전부 제거됨
  (미션 3151줄 러너 `parking_v4_runner.py` 를 통째로 삭제하고 필수 브리지만
  `sim_bridge.py`(1425줄)로 추출). 미션 제어는 100% ROS2 노드 쪽.
- 공용 순수 로직(메카넘 기구학·휠오도·body_twist·TroughTracker 등)은 `src/`
  패키지에 있고 sim_bridge/노드 양쪽이 import 한다(빌드 없이도 동작).

## 폴더 구조 (isaacpjt/Isaac_envo/)

- 루트: `sim_bridge.py`/`.sh`(Isaac 브리지), `launch/pickup_mission.launch.py`
  (전체 미션 launch), `run_*_node.sh`/`run_*_action_server.sh`/
  `run_marker_localizer_node.sh`(ROS2 노드 런처, launch 가 감쌈),
  `build_*.py`+`marker_layout.py`(자산 생성), `mecanum_drive.py`(기구학).
- `smoke/` — 개발 진단용 스모크 테스트.
- `legacy/` — 옛 v2 러너, 참조용.

## 실행

```bash
# 전체 미션을 launch 하나로: sim_bridge 기동 → BRIDGE_READY 감지 시 ROS2 노드
# 스택 기동 → orchestrator 가 auto_start 로 자율 실행(외부 트리거 없음).
source /opt/ros/humble/setup.bash
ros2 launch isaacpjt/Isaac_envo/launch/pickup_mission.launch.py
# 정리: Ctrl-C. 잔여: pkill -9 -f 'sim_bridge.py|parkbot_(aruco|motion)'
```
