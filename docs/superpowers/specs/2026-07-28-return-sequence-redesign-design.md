# 복귀(RETURN) 시퀀스 재설계

작성: 2026-07-28 · 대상: `parkbot_motion` (pickup_orchestrator_node, ingress_node)

## 1. 목표·범위

주차 완료 후 두 로봇(entry_lead / entry_follow)을 원래 도크(대기자리)로 되돌리는
복귀 시퀀스를 사용자 명세대로 재설계한다. 핵심 변경 두 가지:

1. 회랑 왕복 이동을 **"90° 회전 후 직진"** 으로 바꾼다(현재는 yaw 유지 홀로노믹 게걸음).
2. lead 가 차 밑에서 나올 때 **측면 뎁스 중앙유지(ingress 와 대칭)** 로 이탈해 바퀴
   충돌을 없앤다(현재는 마커 pose 로 북진 → 좁은 통로에서 바퀴 긁힘).

범위 밖: 입차(Phase B/C)·운반·주차 로직(이미 실측 완주). 출차(exit) 스택.

## 2. 배경 — 현재 복귀와 문제점

현재 `_run_return`(pickup_orchestrator_node.py): 이탈은 순차(follow→lead), 도크
주행은 동시. 각 로봇은 `_return_egress_to_corridor` + `_return_dock` 로 이동하며,
전 구간을 `nav_fused`(pose_controller `NavigateToPose`)로 **yaw 유지한 채 게걸음**한다.

문제:
- **lead 바퀴 충돌**: lead 는 트럭 남쪽 깊숙이(z≈−1.8)에 있어 이탈 시 트럭 밑
  바퀴 사이를 다시 북으로 통과해야 하는데, 마커 pose(nav_fused)의 몇 cm·몇 도
  오차로 바퀴에 긁혀 못 나온다. 진입(ingress)은 측면 뎁스로 바퀴까지 거리를
  실시간 재며 중앙유지해 안 부딪히는데, 이탈은 그 뎁스 중앙유지를 안 쓴다.
- **이동 방식**: 게걸음은 진행방향 카메라 정렬이 어렵고 실로봇에서 부정확. 사용자
  명세는 회전 후 전진(front cam 이 진행방향 마커를 정면으로 봄).

## 3. 확정 결정

| # | 결정 | 선택 |
|---|------|------|
| D1 | 회랑 이동 방식 | **회전(nav_odom) 후 nav_fused** 직진. yaw 는 회전으로 맞추고, 경로 오차만 nav_fused 의 약한 게걸음이 보정 |
| D2 | lead 차밑 이탈 | **ingress_node 에 EGRESS 모드 추가** — 측면 뎁스 중앙유지 + 지정방향 전진 |
| D3 | follow 차밑 이탈 | **마커(slot_ref, 양캠) 북진** 유지(follow 는 북쪽 얕아 통로 짧음) |
| D4 | 대기 최종 자세 | **동향(yaw90) 복원** — 후진 도크진입 후 제자리 90° 회전. 다음 입차 seed(yaw90)와 일치시켜 재시작 정상화 |

## 4. 재설계 시퀀스

좌표(A1 기준): 슬롯 x=2.8 · 회랑선 z=7.075 · 도크 z=2.2.
도크: follow(x=−1.2, 마커23) / lead(x=−3.2, 마커21). 회랑마커: follow=63 / lead=31.

**공통 스텝** (follow=x−1.2 / lead=x−3.2):

| # | 스텝 | 방식·마커 | 목표 pose |
|---|------|-----------|-----------|
| 1 | 차밑 이탈 | follow: 마커 slot_ref[3,65,66] 양캠 북진 (nav_fused) · **lead: EGRESS 뎁스 중앙유지 북진** | (slot_x, 7.075, 0) 회랑 도달 |
| 2 | 서향 회전 | 제자리 90° (nav_odom) | (slot_x, 7.075, −90) 위치유지·yaw만 |
| 3 | 서진 직진 | 회랑마커 [61~64+회랑마커] 양캠, nav_fused | (dock_x, 7.075, −90) |
| 4 | 북향 회전 | 제자리 90° (nav_odom) | (dock_x, 7.075, 0) |
| 5 | 후진 도크진입 | 도크마커[dock_id] 양캠, nav_fused (−z 후진) | (dock_x, 2.2, 0) 대기중앙 |
| 6 | 자세 복원 | 제자리 90° (nav_odom) | (dock_x, 2.2, 90) 동향 대기 |

회전 방향: 이탈 후 북향(yaw0) → 스텝2 시계90°(yaw−90 서향) → 서진 → 스텝4 반시계90°
(yaw0 북향) → 북향에서 후진(−vx=world −z) 남진 → 스텝6 동향(yaw90).

**순서/충돌 회피:** 이탈(스텝1)은 순차 — follow 먼저(같은 x=2.8 라인). 서진(스텝3)부터
x 가 갈리므로(−1.2 vs −3.2) 조율. lead 가 더 서쪽(−3.2)이라 회랑에서 follow(−1.2)를
지나야 하니, 안전하게 **follow 도크 안착까지 끝낸 뒤 lead 진행**(순차) 또는 회랑
z 라인상 앞뒤 간격 확보. 초안은 순차(단순·안전), 느리면 후속 최적화.

## 5. 신규 컴포넌트 — ingress_node EGRESS 모드

ingress_node 는 현재 SEEK(전진+뎁스 중앙유지) → RETURN(후진 정렬) → SETTLING → DONE
상태기계로 **차 밑 진입**만 한다. 나오기(egress)는 없다.

**추가:** IngressUnderTruck goal(또는 노드 파라미터)에 방향/모드 필드를 얹어 EGRESS 모드를
넣는다. EGRESS 동작:
- 측면 좌/우 뎁스가 유효하면 `ingress_control.lateral_centring_vy` **그대로 재사용**해
  바퀴 사이 중앙 유지(vy).
- 지정 방향(북, world +z)으로 전진(vx) — 축 트로프 검출은 **불필요**(중앙유지+전진만).
- 정지: 목표 이동거리/좌표(회랑선 근처) 도달 시. 트럭 밖으로 나오면 측면 뎁스에 벽이
  사라져 중앙유지가 자연히 꺼지고(vy=0 — "둘 다 유효해야 보정" 규칙) 그대로 직진.
- 안전장치(뎁스 유실 전체정지, 포즈 코스팅 워치독)는 SEEK 와 공유.

들어간 방식과 대칭이라 바퀴에 안 부딪힌다. 정확한 진행 부호(회전/후진 여부)와 정지
좌표는 구현 시 실측 로그로 확정(문서화된 단일-미션 부호 관례, ingress_control §주행좌표).

## 6. 변경 파일

- `pickup_orchestrator_node.py` — `_return_egress_to_corridor`/`_return_dock` 을 위
  6-스텝(회전+직진+후진+복원)으로 재작성. lead 이탈은 새 egress 헬퍼(ingress
  EGRESS 액션 호출)로. 회랑마커/도크마커 ref 전환은 기존 `_set_localizer_ref` 재사용.
- `ingress_node.py` / `ingress_control.py` — EGRESS 모드(방향 파라미터 + 상태분기).
  `lateral_centring_vy` 재사용, 자기검증(`__main__`) 에 EGRESS 케이스 1개 추가.
- (필요시) `IngressUnderTruck.action` — mode/direction 필드. 인터페이스 변경이면
  최소화(bool `egress` 또는 int `direction`).
- 신규 파라미터: `return_follow_parked_z`(follow 주차 z), egress 목표 z 등.

## 7. 에러처리·검증

- 각 스텝은 기존 패턴대로 (ok, reason) 반환, 실패 시 즉시 abort + feedback('FAILED').
- 뎁스 유실/포즈 stale 은 ingress_node 기존 워치독이 담당(egress 도 공유).
- **검증(일회용 테스트 코드 금지 — CLAUDE.md)**: 실제 미션(`ros2 action send_goal
  /entry/execute_parking_task {slot_id: A1}`)의 복귀 구간을 Isaac 에서 돌려 lead 가
  바퀴 안 긁고 이탈하는지, 두 로봇이 도크(−1.2/−3.2, 2.2, yaw90)에 정지하는지 실측.
- 순수 로직(egress vx/vy 부호, 회전 방향)은 `ingress_control._self_check` / 해당
  모듈 `__main__` assert 에 케이스 추가로 회귀 가드.

## 8. 미해결/후속

- lead egress 진행 부호·정지 좌표: 구현 시 로그로 확정.
- 서진 순차 vs 동시 최적화: 초안 순차, 느리면 후속.
- 출차(exit) 스택은 별도 — 이 설계 검증 후 동일 패턴 포팅 여부 판단.
