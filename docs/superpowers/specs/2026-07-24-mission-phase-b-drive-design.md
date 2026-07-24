# 미션 Phase B — 도크→인계장(XN) 융합 주행 설계

> 작성: 2026-07-24. 선행: Phase A(4-카메라 완료), ArUco 융합(부록 A~E), FUSE 융합주행 루프.
> 상위: `2026-07-24-mission-e2e-design.md`(§1 개요). UI 실행 필수. 진행·디버깅은 HANDOFF/DEBUG_LOG.

## 1. 목표

입차팀 로봇 2대(entry_lead, entry_follow)가 각자 도크에서 나와, **GT 없이 ArUco 마커+휠오도 융합
측위**로 실제 메카넘 바퀴를 굴려 인계장 마커 **XN(crossing_N)** 앞까지 주행·정렬한다. 차량 배치·차밑
진입·리프트는 Phase C.

## 2. 좌표·개체 (v4 실측)

- 자세 규약: yaw=0 → 월드 +Z(`atan2(fwd_x, fwd_z)`). 로봇은 +X 를 보고 스폰 → filter-yaw ≈ 90°.
- **인계장 마커 XN = crossing_N**: id=31, 월드 (x=-2.5, z=+6.875), 바닥 쿼드.
- **입차 도크**(로봇이 마커 위치에서 스폰): entry_lead=`D_OUT_1`(-3.2, +2.9), entry_follow=`D_OUT_2`(-1.2, +2.9).
- **역할**: entry_follow = 앞축(먼저 차밑 진입), entry_lead = 뒤축(먼저 도크를 빠져나와 +x 자리잡고 나중 진입).

```
 z(북)↑   +6.875 ─ [XN]──────────  A1' A2' A3'
                   x=-2.5
                      │ ~4m (+Z 주행)
          +2.9  ─ [entry_lead] [entry_follow]   ← 도크(+X 정면 스폰)
                   x=-3.2       x=-1.2
```

## 3. 안무 (상태 기계, 로봇별)

공통: `DOCK_CHECK → ROTATE90 → DRIVE_TO_XN → ALIGN_XN → (lead: OFFSET_X) → DONE`. 최종 접근은 **스태거**
(entry_lead 먼저 자리잡고, entry_follow 이어서).

1. **DOCK_CHECK / ROTATE90**: 제자리 90° 회전(+X, yaw≈90° → +Z, yaw≈0°). 빠져나오며 **후방캠으로 도크
   마커** 포착 → 융합필터 초기 fix(도크 마커 실좌표로 자세 고정). GT 아님.
2. **DRIVE_TO_XN**: +Z 로 XN 부근까지 융합주행(예측=휠오도 predict_body, 보정=마커 update).
3. **ALIGN_XN (스태거)**:
   - **entry_lead(뒤축) 먼저**: **후방캠이 XN 을 향하도록** 정렬(최종 yaw≈180°, 뒤가 북/XN) → **+x 오프셋** 이동.
   - **entry_follow(앞축) 이어서**: **전방캠이 XN 정면**(최종 yaw≈0°, 앞이 북/XN)으로 정렬.
   - 최종 자세는 파라미터(nominal yaw/오프셋)로 두고 GUI 실측으로 튜닝.

## 4. 아키텍처 (검증된 원시요소 재사용)

- **주행 원시요소**: 러너의 FUSE 루프(`--probe=FUSE`)가 이미 1로봇용으로 완성 — 실제 바퀴
  `slew_twist→wheel_velocities_from_cmd_vel→set_joint_velocity_targets`, 예측 `predict_body`,
  보정 `detect_current→localize_pose→PoseFilter.update`. Phase B 는 이를 **다로봇·임의 목표자세**로 일반화.
- **주행 컨트롤러(신규)**: 순수 제어법 `body_twist_toward(cur_xzyaw, tgt_xzyaw, gains, max_lin, max_ang)
  → (vx,vy,wz)`(body frame, mecanum 은 x·y·yaw 독립) + 이를 감싼 폐루프 `drive_to_pose`,
  `rotate_in_place`. 순수 함수는 단위테스트, 폐루프는 headless 스모크.
- **카메라 역할 파라미터화(차단 선행)**: `attach_camera_graph(id, cam, role)` → 토픽
  `/robot_<id>/<role>/image_raw`, 그래프 `/Graphs/cam_<id>_<role>`. entry_follow 가 후방(도크점검)+
  전방(XN정렬)을 동시에 써야 하므로 필수. 기존 FUSE(전방) 회귀 없어야 함.
- **실행**: `parking_v4_runner.sh --mission=B [--gui]`. GUI 라이브 실행 + headless 스모크(두 로봇 완료 토큰).

## 5. 수용 기준

- `--mission=B` 로 두 로봇이 도크→XN 을 **융합측위(마커+휠오도)로 실제 주행**해 XN 에 정렬(GT 제어 없음).
- entry_follow 전방캠이 XN 정면, entry_lead 후방캠이 XN 향한 채 +x 오프셋 — 두 로봇 DONE 토큰.
- 기존 FUSE/M5/REAR 회귀 없음. GUI 실행 가능. 순수 제어법 단위테스트 통과.
- 정직성: 도크 fix·XN 정렬은 마커 가시 구간에서만 유효(근거리 <1.1m 사각 존재) — 스모크에 실측 기록.

## 6. 범위 밖 (Phase C/D)

- 차량 배치·측면 뎁스캠 바퀴감지·차밑 진입·리프트(C). 운반·주차(D).
