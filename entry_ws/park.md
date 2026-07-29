# PARK.md — 2로봇 발렛파킹 프로젝트 지식 베이스

트럭을 2대의 메카넘 협동로봇이 **밑에서 들어 옮겨** 주차/출차하는 발렛파킹 시스템.
Isaac Sim 가상환경 + ROS2 Humble. 이 문서는 프로젝트 전반의 중요 결정·구조·디버깅
교훈을 한곳에 모은 지식 베이스다. **실행 방법은 [`README.md`](./README.md) 참조.**

- 현재 브랜치: `feat/aruco-v4-fusion`
- 핵심 성과: GT(정답좌표) 제거 → **ArUco 마커 + 휠오도 융합 측위**로 전환, 입차 주차까지
  실측 완주 ✅, 복귀(RETURN) 시퀀스 구현·검증 진행 중.

---

## 1. 기술 스택

| 영역 | 스택 | 메모 |
|---|---|---|
| 로봇 미들웨어 | **ROS2 Humble** (system, py3.10) | 액션 중심 아키텍처 |
| 시뮬 | **Isaac Sim** (번들 py3.11) | `sim_bridge.sh`, headless RTF≈0.20 |
| DDS | **FastDDS** + 화이트리스트 XML | 크로스버전 SHM 비호환 → UDP 강제 (§8-1) |
| 비전 | OpenCV ArUco (바닥 마커) | 전/후방 이중 카메라 |
| 측위 | 휠오도 예측 + 마커 보정 융합 | SLAM 없음, 마커가 유일 절대기준 |
| 구동 | 메카넘 휠 + 롤러 리프트 | 롤러 그립은 병진만 전달, yaw 자유회전 |
| 관제/DB | MySQL (`parking`) | 슬롯·작업·로봇위치 |
| 웹 UI | FastAPI + web (`feature/UI` 브랜치) | PARKING_MODE=ros2 |

---

## 2. 좌표계 · 기하 규약 (반드시 숙지)

- **nav yaw 규약**: `ψ = atan2(fwd_x, fwd_z)`. `0°=+z(북)`, `90°=+x(동)`, `-90/-270=서`, `±180=남`.
  - ⚠️ **±180°는 특이점** — pose_controller가 회전 방향을 못 정해 뒷바퀴 도리도리(§8-8).
- **입차/출차 규약**: **입차=z 양수 / 출차=z 음수.** Isaac 에셋 라벨(IN=z음수)과 **반대**.
  이 반전은 `src/parkbot_aruco/parkbot_aruco/site_map_v4.py` **한 곳에만** 가둠(유일 출처).
- **로봇 4대**(`site_map_v4.py:43`): `entry_lead`, `entry_follow`, `exit_lead`, `exit_follow`.
  - 역할: `*_follow`=앞축(front axle), `*_lead`=뒤축(rear axle).
- **핵심 좌표**(v4, world x/z):
  - 슬롯 A1/A2/A3 레인: x = 2.8 / 6.2 / 9.6, 회랑선 z = **7.075**
  - 입차 도크: `entry_lead`=D_OUT_1(x=-3.2), `entry_follow`=D_OUT_2(x=-1.2), 스폰 z=2.2 (데칼 z는 별도 오프셋)
  - 출차 도크: `exit_lead`(x=-3.2), `exit_follow`(x=-1.2), z=-2.2 (입차의 z 부호반전)
  - 카메라 시선 = 월드 +X (스폰 시 RotateX-90), 높이 **0.15m 상향**(검출창 확보, §8)
- **마커 배치**: 슬롯 레인 마커 A1'/A2'/A3'=id 3/4/5(z=7.075), 회랑 드리프트보정 마커 id 61~64,
  도크 마커(D_OUT_1=21, D_OUT_2=23), crossing XN=id31. **정확한 값은 `site_map_v4.py`가 유일 출처.**
- **도크마커 0.7m 오프셋 주의**: 에셋 `aruco:position`(도크중심)과 실제 바닥 데칼 위치가 0.7m 차 →
  지도 생성 시 **데칼 실좌표**를 써야 함(`marker_map_v4.json`이 이미 흡수).

---

## 3. 시스템 아키텍처 (멀티 PC, 도메인 126)

3대 PC + Isaac이 **동일 ROS_DOMAIN_ID=126**에서 이름공간을 나눠 공존한다.

```
   [웹 UI]  (feature/UI, FastAPI)
      │ dispatch_parking_task (RequestParkingTask: ENTRY|EXIT + vehicle_id)
      ▼
  task_dispatcher (관제 PC, parking_control) ── MySQL / 슬롯배정 / 안전게이트
      │  request_type 분기
      ├─ ENTRY ─▶ /entry/execute_parking_task ─▶ pickup_orchestrator_node      (입차 PC=cobot_ws)
      └─ EXIT  ─▶ /exit/execute_parking_task  ─▶ exit_pickup_orchestrator_node (출차 PC=featureUI)
```

| PC | 워크스페이스 | 런치 | 로봇 |
|---|---|---|---|
| 관제 | featureUI | `parking_control control_tower.launch.py` | — (DB/UI/dispatcher/safety) |
| 입차 | cobot_ws | `parkbot_motion nodes.launch.py` | entry_lead/follow |
| 출차 | featureUI | `parkbot_motion exit_nodes.launch.py` | exit_lead/follow |
| Isaac | GPU PC | `isaacpjt/Isaac_envo/sim_bridge.sh` | 4대 전부 스폰 (1개만!) |

**격리 스킴**: 로봇 네임스페이스 `/robot_entry_*` vs `/robot_exit_*`, 액션 `/entry/*` vs `/exit/*`,
노드명 `*_entry_*`/`carry_action_server`/`pickup_orchestrator_node` vs `*_exit_*`. (배경: 출차팀이
초기에 우리 launch를 복붙하고 robot_id를 안 바꿔 이름 충돌 → UI 요청이 출차로 새서 우리 로봇이
안 움직인 문제가 3회 반복.)

---

## 4. ROS2 노드 구조 (입차 스택, `nodes.launch.py`)

로봇당·전역 노드 구성. FQN 기준.

**측위 레이어** (로봇당 1):
- `/robot_<rid>/marker_localizer_node` (parkbot_aruco) — 전/후방 이중카메라 + 휠오도 융합.
  `fuse=True`, `/odom` predict + 마커 update, `/pose` 발행. `seed_pose`로 스폰자세 시드.

**제어 레이어** (로봇당):
- `pose_controller_odom_<rid>` — `/robot_<rid>/navigate_to_pose` (odom 기반, 제자리 회전용)
- `pose_controller_fused_<rid>` — `/robot_<rid>/navigate_to_pose_fused` (융합 /pose, 정밀 주행)
  - `yaw_tol=0.5°`, `yaw_min_cmd=0.05`(데드밴드 보정), `max_lin=0.15/max_ang=0.35`(슬립 감소)
- `axle_detector_<rid>` — 측면 뎁스캠 트로프-중간값으로 트럭 바퀴(축) 감지
- `ingress_<rid>` — 차 밑 진입/이탈 제어 (SEEK/RETURN/EGRESS/SETTLING/DONE 상태머신)
- `lift_<rid>` — 리프트 액션 서버 (`ramp_wait_sec`)

**전역**:
- `carry_action_server` — Phase D 운반(가상중심 강체제어). 액션 `/entry/carry_to_slot`
- `pickup_orchestrator_node` — 미션 총괄. 액션 `/entry/execute_parking_task`. `auto_start=False`(UI 대기)
- ⚠️ `user_request_gateway_node` — **파일만 있고 launch엔 없음.** 수동 실행 금지 (§9-1)

**관제 스택**(`control_tower.launch.py`): `safety_supervisor`(안전상태·비상정지·`/safety/state`),
`parking_slot_manager`, `task_dispatcher`(UI 접수·라우팅), `robot_position_bridge`(4로봇 odom→DB),
`safety_monitor`(천장 라이다 점유·장애물). ⚠️ `sim_orchestrator`(가짜로봇)는 **띄우지 말 것**(DB 오염).

---

## 5. 미션 파이프라인 (Phase A~D + RETURN)

입차 E2E: **도크 → 인계장 → 픽업 → 운반 → 주차 → 복귀**.

- **Phase A (카메라 정리)** ✅ — 로봇당 정확히 4캠(front/rear/좌/우 뎁스). rear=front의 Rz(180) 미러.
- **Phase B (도크→인계장 주행)** ✅ — 후방캠 도크마커 점검 → 제자리 90° 회전(+Z) → 전방캠 XN(id31)
  정렬 → 마커+휠오도 융합 주행. 스태거(lead 먼저)·충돌회피(~1.6m 분리). entry_follow 오차 ~0.2~0.7cm.
  - **위치전용 보정**(`correct_yaw=False`): 마커 yaw가 약해 filt를 오염 → yaw는 오도 신뢰(정확도 ~4°).
- **Phase C (픽업)** ✅ — 측면 뎁스캠으로 트럭 바퀴 트로프(중간값) 감지 → 차 밑 진입 → 리프트 업(SUPPORTED).
- **Phase D (운반)** ✅ — 가상중심 강체제어로 슬롯까지 운반 → 목표 슬롯 레인마커 게이트로 정지 → 주차.
  - CARRY_LANE(→2.8,7.075,yaw90) → CARRY_SLOT(한 carry 호출이 90°→0° 회전 후 슬롯 -z 진입).
  - 실측 슬롯 안착 center≈(2.4, 0, ~2°). **상세: [carry-virtual-center-control.md](./docs/concepts/carry-virtual-center-control.md)**
- **RETURN (복귀)** — 주차 후 도크로 독립 귀환. follow=마커 이탈, lead=**후진 egress**(회전 불가, §8-9).
  - 완전 병렬(follow·lead 각자), lead egress 속도를 follow와 매칭(0.15)해 충돌 방지 (§8-10).
  - `RETURN_TEST=1`로 주차완료 상태 스폰 → 복귀만 반복 검증.

---

## 6. 측위 레이어 (ArUco 융합)

**GT는 필터 밖.** 초기화=첫 마커fix, 예측=휠오도(`PoseFilter.predict_body`), 보정=마커(`update`).
GT는 채점·1회보정·정지트리거로만 쓰고 제어엔 안 씀.

- **Phase 1 (지도+M5)**: `build_marker_map_v4.py` → `data/marker_map_v4.json`(16장, 도크 0.7m 오프셋 흡수).
  정적 단일프레임 위치 **median 1.4cm / p95 4.15cm**(꼬리는 비스듬 각도 지배, 정적 한계).
  공분산 std ex5.9 / ez20.5mm / yaw0.85° = 융합필터 R.
- **Phase 3 (오도 융합, 핵심 성과)**: 주행 접근 중 예측+보정. **종단 0.29cm / 0.02° PASS**,
  융합>단일(궤적 p95 **1.90 vs 2.52cm**). 배포 노드 `marker_localizer_node fuse=True`.
- **정직성 단서**: 종단 0.29cm는 GT거리로 정지한 근거리 고정자세 정확도지 폐루프 도킹 아님.
  마커 93% 가시라 이득은 주로 시간 평활. 정확도는 궤적 p95로 인용할 것.
- 마커 추가 불필요(기존 16장 충분) — 초기 "추가 필요" 판단은 **충돌 오염된 드리프트 측정값** 탓이었음.

---

## 7. 핵심 제어 철학 (사용자 반복 지시 — 어기지 말 것)

**carry(운반) — 마커 유무 2갈래** (사용자 3회 지시). **상세 수식·구현은
[carry-virtual-center-control.md](./docs/concepts/carry-virtual-center-control.md).**
- **마커 안 보임(기본)**: follow·lead 각자 **경로방향 body-x 순항만**. `vy=0, wz=0 강제`.
- **follow rear가 마커를 봐서 자기 위치·yaw를 실제로 알 때만** yaw·위치(옆) 정렬(가상중심 월드투영).
- **예외 — 슬롯 90° 턴**: 도는 내내 마커를 못 보니 `|eyaw|>align_gate`면 마커 없이 odom 회전(안 그럼 락업).
- **Why**: odom만 믿고 월드투영을 always 돌리면 yaw 조금만 틀어져도 vy/wz가 새어나와 트럭을 흔들고,
  omega always는 마커공백에서 슬립 드리프트를 추종해 발산·락업(실측). ← 내가 이걸 2번 틀렸음.

**가상중심 강체제어**: follow rear /pose=트럭 대표, lead=`follow + L·heading` 강체유도
(L=축간거리, 첫 세그먼트 1회 측정·캐시). 월드 중심 twist를 각 로봇 heading에 투영 → 둘 다 cmd_vel.
회전선행 게이트(|eyaw|>8°면 병진0, 제자리회전 먼저 = 슬롯 클리핑 방지).

**롤러 그립의 물리**: 롤러는 **병진(힘)만 전달, 로봇 yaw는 자유회전**. carry에서 로봇에 omega를 주면
트럭 밑에서 헛돌아 180° 뒤집혀 후진함(실측). → carry 병진은 **순수 전진**, strafe(vy)도 yaw 드리프트
유발하니 0. (`tl=(tl[0],0,0)`)

**ingress**: 깊은 축 미세정렬(RETURN 국면)에서 `wz=0`(끼인 채 회전=바퀴 밀림 차단). 진입 SEEK만 yaw_hold.

---

## 8. 중요 디버깅 교훈 (실측 수치 — 재발 방지)

1. **★ DDS FastDDS 화이트리스트 필수** — Isaac 번들 FastDDS(py3.11)↔시스템 Humble(py3.10)은
   **크로스버전 SHM 데이터전송 비호환**. builtin이면 `topic list`는 보여도 데이터 0(`Publisher count: 0`,
   echo 타임아웃, pose_controller 즉시 `stale_pose` abort). 해결: 화이트리스트 XML
   (`~/.ros/fastdds_whitelist.xml`, `useBuiltinTransports=false`, UDP를 127.0.0.1+10.10.0.x로 강제)을
   **브리지·노드 양쪽에** 걸기. (한때 "화이트리스트가 discovery를 깬다"고 제거했으나 **오판** — 데몬캐시
   착시였음.)
2. **★ install/ 이 복사(비-symlink)면 소스 수정이 런타임에 반영 안 됨** — parkbot_aruco가 복사 설치라
   localizer 수정이 안 먹어 마커 봐도 정지 못하던 진범. `colcon build --symlink-install`로 egg-link화하면
   이후 자동 반영. **교훈: py 수정이 안 먹으면 install/이 복사인지 먼저 확인.**
3. **★ 회전 기구학 부호 오류** — 사용자가 GUI로 "90°가 아니라 270° 돈다" 지적해 발견. `mecanum_drive`의
   `SIGN_YAW/YAW_SCALE`가 물리와 불일치. **-270≡+90, ±540≡180 (mod360)**이라 종단각만 보던 검증이 전부
   통과해 90°/180°에서만 오차가 숨었음. 수정: `SIGN_YAW=-1.0, YAW_SCALE=0.4080`, `YAW_ODOM_SCALE=1.1677`
   (1.0은 롤러슬립 잔차 13~20%로 미션 깨짐). **교훈: 회전 검증은 종단각만으론 부족 — 누적 스윕각 + 360의
   약수 아닌 각도(45°) 필수.**
4. **★ yaw_tol < 메카넘 데드밴드** — `yaw_tol=0.5°`인데 메카넘이 데드밴드(wz≈0.012 rad/s 이하 안 돎)로 0.6°에서
   스톨·실패. 처음엔 `yaw_tol=1.0`으로 완화 → 이후 `yaw_min_cmd=0.05`(최소 회전속도)로 데드밴드 보정해
   0.5°까지 인칭 도달. (진입 직전 정렬 0.5° 넘게 틀어지면 직진 시 트럭 바퀴에 충돌.)
5. **★ final_align 플레이크 = 조기 도달래치 후 재제어 없이 abort** — 회전 중 노이즈 낀 yaw가 한 프레임
   tol에 들면 정지 래치 → settle 중앙값이 tol 밖이면 재제어 없이 `status=6` abort → 미션 전체 사망(매번
   랜덤 지점). **사용자 지시: tol 넓히지 말고 1°될 때까지 계속 제어.** 수정: `PoseController.resume()`
   (래치·twist·settle 리셋) + SETTLING에서 미도달이면 `settle_retries`(기본6)까지 DRIVING 재진입.
6. **★ carry 속도 ~0.03~0.04m/s = 제어 아닌 물리 한계** — 제어방식 2가지 다 같은 속도 → 들어올린 트럭이
   로봇 견인력 대비 무거운 것. 순수 전진으로도 느리면 시뮬 트럭 질량을 낮출 것.
7. **★ 진입 후 두 로봇이 반평행이 아니라 같은 방향(~-87°)** — 메모리의 "lead -90/follow +90 반평행" 전제가
   실제와 달랐음. 하드코딩 반대부호 ±V가 같은방향 로봇을 서로 반대로 밀어 트럭을 회전(찢음)·발산.
   → 세계방향을 각 로봇 heading에 투영(메카넘, 회전 불필요)으로 해결.
8. **±180° 회전 특이점** — pose_controller가 방향을 못 정해 wiggle. 2단계 회전(-90, 0)으로 분해.
9. **★ lead는 트럭 바퀴 사이에서 제자리 회전 물리적 불가** — 실측 lead yaw가 300s에 15°만 돎(-180→-165,
   끼임). RETURN에서 lead 회전을 **완전 제거**, 남향 lead가 **후진(reverse egress)** = 북진 이탈로 대체
   (들어온 경로 되짚기). egress vx 방향을 `cos(yaw)`로 결정.
10. **★ 병렬 RETURN 충돌 = 속도 불일치** — lead 후진 egress(0.4)가 follow 이탈(~0.15)보다 2배 빨라 회랑
    근처(z≈7)에서 20초 앞선 follow를 따라잡아 0.4m까지 붙음. 궤적 로그로 확정. 수정: `return_lead_egress_speed=0.15`
    (follow와 동일) → 간격 유지. (20초 stagger는 정상 작동했으나 속도차가 진범이었음.)
11. **CARRY_SLOT 오버슈트** — 슬롯(yaw≈0) 세그먼트는 `seg_overshoot=0.10`(<goal_pos_tol 0.15)으로 낮춰
    at_goal이 odom에서 발화하게. 레인(yaw≈90)은 1.2 유지.
12. **재시작 위생** — Phase B는 로봇이 도크 스폰에 있다고 가정 → 미션 재실행 시 **브리지도 같이 재시작**.
    launch만 재시작하면 로봇이 도크로 되돌아가는 이상주행.
13. **cold-start pose 흐름** — 액션 보내기 전 `/pose`가 흘러야 함(`ros2 topic hz`). 안 그러면 즉시
    `stale_pose` abort(status=6).
14. **PYTHONNOUSERSITE=1** — cv_bridge가 시스템 NumPy 1.x로 빌드됨. 사용자 site-packages의 NumPy 2.x가
    먼저 잡히면 `_ARRAY_API` import 오류 → 로봇 노드에서 사용자 site 제외(관제 UI의 MySQL 환경은 유지).

---

## 9. 함정 / 하지 말 것

1. **입차 PC에서 `user_request_gateway_node` 수동 실행 금지** — 기본 서비스명 `dispatch_parking_task`가
   관제 task_dispatcher와 **같은 이름 서버 2대** → UI 요청 랜덤 라우팅 → 로봇 안 움직임. `nodes.launch.py`는
   의도적으로 안 띄움. 원천봉쇄하려면 파일 삭제.
2. **한 PC에 cobot_ws + featureUI 워크스페이스 동시 소스 금지** — 같은 `parking_robot_interfaces`인데
   내용이 다름(cobot_ws는 `IngressUnderTruck`에 egress 필드 O·안전 msg 없음 / featureUI는 반대). 어느 쪽도
   상위집합이 아니라 한쪽이 반드시 깨짐. **PC를 나눠 각자 워크스페이스만 소스**하면 됨. 단일 PC 통합
   테스트가 필요하면 인터페이스 병합 빌드 필요.
3. **`sim_orchestrator` 추가 실행 금지** — 가짜 로봇 위치가 DB에서 `robot_position_bridge` 실측과 충돌.
4. **Isaac은 통합 운영 시 1개** — 4로봇이 같은 트럭·주차장을 공유해야 충돌회피·핸드오프 성립. entry용/exit용
   따로 쪼개면 별세계가 됨. (독립 데모끼리는 도메인 분리해서 각자 Isaac 돌리면 OK.)
5. **관제 안전 게이트** — task_dispatcher는 fail-safe라 `safety_supervisor`의 `/safety/state`를 못 받으면
   모든 요청을 막음. dispatcher만 띄우고 안전노드 빼먹으면 "UI 눌러도 조용히 전부 거부".

---

## 10. 참고 파일 맵

- 좌표·마커·로봇ID 유일 출처: `src/parkbot_aruco/parkbot_aruco/site_map_v4.py`
- 마커 지도: `src/parkbot_aruco/data/marker_map_v4.json`
- 융합 측위 노드: `src/parkbot_aruco/parkbot_aruco/marker_localizer_node.py`
- 미션 총괄: `src/parkbot_motion/parkbot_motion/pickup_orchestrator_node.py`
- 운반 제어: `src/parkbot_motion/parkbot_motion/carry_action_server.py` (+ 강체 기하 `formation.py`)
- 운반 제어 상세(가상중심): [`docs/concepts/carry-virtual-center-control.md`](./docs/concepts/carry-virtual-center-control.md)
- 진입/이탈 제어: `src/parkbot_motion/parkbot_motion/ingress_control.py` (+ `ingress_node.py`)
- 자세 제어: `src/parkbot_motion/parkbot_motion/pose_controller_node.py`
- 입차 런치: `src/parkbot_motion/launch/nodes.launch.py`
- Isaac 브리지: `isaacpjt/Isaac_envo/sim_bridge.sh`
- 설계/계획 문서: `docs/superpowers/specs/`, `docs/superpowers/plans/`
- 실행 가이드: [`README.md`](./README.md)
