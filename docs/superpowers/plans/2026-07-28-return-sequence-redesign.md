# 복귀(RETURN) 시퀀스 재설계 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 주차 후 두 로봇을 도크로 되돌리는 복귀 시퀀스를, lead 는 뎁스 중앙유지로 차밑 이탈하고 회랑은 회전+직진으로 이동하며 도크엔 후진 진입 후 동향90 복원하도록 재작성한다.

**Architecture:** ① `ingress_node`/`IngressController` 에 **EGRESS 모드**(축검출 없이 좌우 뎁스 중앙유지 + 지정 목표까지 전진, world-z 축) 추가. ② `pickup_orchestrator_node` 의 `_return_*` 를 6-스텝(이탈→서향회전→서진→북향회전→후진진입→동향복원)으로 재작성. 회랑 이동은 기존 `nav_odom`(회전)+`nav_fused`(직진) 재사용.

**Tech Stack:** ROS2 Humble, rclpy 액션(IngressUnderTruck, NavigateToPose), Isaac Sim. symlink-install(파이썬 소스 편집은 재시작만, **액션 인터페이스 변경만 colcon build 필요**).

## Global Constraints

- 답변·주석 한국어(CLAUDE.md). 직관적 어조.
- **일회용 테스트 파일 금지(CLAUDE.md)**: 순수 로직 검증은 해당 모듈 `if __name__ == "__main__"` self-check 의 assert 에 케이스를 **추가**한다. 통합 검증은 실제 미션(`ros2 action send_goal /entry/execute_parking_task`) 실행으로 한다. 별도 pytest 파일·프레임워크 만들지 않는다.
- 제어 철학(사용자 3회 지시, [[carry-control-philosophy]]): 마커/뎁스 근거 없이 odom 만으로 vy/wz 내지 않는다. EGRESS 도 **블라인드 구간 wz=0**(RETURN 과 동일 — 끼인 채 회전=바퀴 밀림).
- 좌표(A1): slot_x=2.8 · 회랑선 z=7.075 · 도크 z=2.2. follow(dock_x=−1.2, corridor_id=63, dock_id=23) · lead(dock_x=−3.2, corridor_id=31, dock_id=21). lead 주차 z≈−1.8. slot_ref=[3,65,66], lane_markers=[61,62,63,64].
- 부호 관례(문서화된 단일-미션 한계, ingress_control §주행좌표): EGRESS world-z 전진 부호는 구현 중 실측 로그로 확정한다.

---

## 파일 구조

| 파일 | 책임 | 변경 |
|------|------|------|
| `parking_robot_interfaces/action/IngressUnderTruck.action` | ingress/egress 액션 계약 | egress 필드 2개 추가(빌드) |
| `parkbot_motion/ingress_control.py` | 진입/이탈 순수 정책 | `IngressController` EGRESS phase |
| `parkbot_motion/ingress_node.py` | 뎁스·pose 배관, 액션 서버 | egress goal 분기, travel 축 z |
| `parkbot_motion/pickup_orchestrator_node.py` | 미션 오케스트레이션 | `_egress` 헬퍼 + `_return_*` 6스텝 재작성 |

---

## Task 1: IngressController EGRESS 모드 (순수 로직)

**Files:**
- Modify: `src/parkbot_motion/parkbot_motion/ingress_control.py` (`IngressController.__init__`, `.step`, `PHASE_*`, `__main__`)

**Interfaces:**
- Consumes: 기존 `lateral_centring_vy(left, right)`, `DEFAULT_FORWARD_SPEED`, `DEFAULT_SETTLE_FRAMES`.
- Produces: `IngressController(trough_index, *, egress=False, egress_target=None, egress_stop_tol=0.15, ...)`. `egress=True` 면 첫 phase 가 `PHASE_EGRESS`. `step(travel, yaw_deg, left, right, axle_centers, dt)` 는 egress 시 `(±forward_speed, vy, 0.0)` 를 내고 `|egress_target − travel| ≤ egress_stop_tol` 이면 SETTLING→DONE. `final_stop_x` 에 정지 travel 기록.

- [ ] **Step 1: self-check 에 실패 케이스 추가** — `ingress_control.py` 의 `__main__` 끝(‘self-check OK’ print 직전)에:

```python
    # EGRESS: 축검출 없이 목표 travel 까지 전진(중앙유지 vy 는 유지), wz=0(끼임 회전 금지),
    # 목표 도달 시 SETTLING→DONE.
    g = IngressController(0, egress=True, egress_target=7.0, forward_speed=0.4)
    assert g.phase == IngressController.PHASE_EGRESS, g.phase
    vx, vy, wz = g.step(0.0, -70.0, None, None, [], 0.05)   # travel 0 < target 7 → 전진
    assert vx > 0 and wz == 0.0, (vx, wz)                    # yaw 20°틀려도 wz=0
    vx, vy, wz = g.step(7.0, None, None, None, [], 0.05)     # 도달 → SETTLING(정지)
    assert g.phase == IngressController.PHASE_SETTLING and vx == 0.0, (g.phase, vx)
    for _ in range(g.settle_frames):                        # settle 소진 → DONE
        g.step(7.0, None, None, None, [], 0.05)
    assert g.done and abs(g.final_stop_x - 7.0) < 1e-9, (g.phase, g.final_stop_x)
    # EGRESS 중앙유지: 좌우 뎁스 있으면 vy 실린다(SEEK 와 같은 lateral 재사용).
    g2 = IngressController(0, egress=True, egress_target=7.0)
    _vx, vy2, _wz = g2.step(0.0, None, 0.30, 0.10, [], 0.05)  # 좌>우 → +vy
    assert vy2 > 0, vy2
```

- [ ] **Step 2: 실행해서 실패 확인**

Run: `python3 src/parkbot_motion/parkbot_motion/ingress_control.py`
Expected: `AttributeError` 또는 `TypeError`(egress kwarg/PHASE_EGRESS 없음).

- [ ] **Step 3: 최소 구현** — `PHASE_EGRESS = 'EGRESS'` 클래스 상수 추가. `__init__` 시그니처에 `egress=False, egress_target=None, egress_stop_tol=0.15` 추가하고 저장:

```python
        self.egress = bool(egress)
        self.egress_target = None if egress_target is None else float(egress_target)
        self.egress_stop_tol = float(egress_stop_tol)
        self.phase = self.PHASE_EGRESS if self.egress else self.PHASE_SEEK
```

`step()` 의 SEEK 분기 **앞**에 egress 분기 삽입(vy/wz 는 기존 라인에서 이미 계산됨 — egress 는 wz 를 0 으로 덮는다):

```python
        if self.phase == self.PHASE_EGRESS:
            remaining = self.egress_target - travel_x
            if abs(remaining) <= self.egress_stop_tol:
                self.phase = self.PHASE_SETTLING
                self._settle_count = 0
                self.final_stop_x = travel_x
            else:
                vx = math.copysign(self.forward_speed, remaining)  # 목표 향해 등속 전진
                return (vx, vy, 0.0)   # 블라인드 축이동 — 회전 금지(바퀴 밀림, RETURN 동형)
```

- [ ] **Step 4: 실행해서 통과 확인**

Run: `python3 src/parkbot_motion/parkbot_motion/ingress_control.py`
Expected: `ingress_control self-check OK ...` 출력(assert 전부 통과).

- [ ] **Step 5: 커밋**

```bash
git add src/parkbot_motion/parkbot_motion/ingress_control.py
git commit -m "feat(ingress): IngressController EGRESS 모드(축검출 없이 목표까지 중앙유지 전진, wz0)"
```

---

## Task 2: IngressUnderTruck.action egress 필드 + 빌드

**Files:**
- Modify: `src/parking_robot_interfaces/action/IngressUnderTruck.action` (goal 섹션)

**Interfaces:**
- Produces: `IngressUnderTruck.Goal` 에 `bool egress`, `float32 egress_target_z`. 기존 필드(`trough_index/forward_speed/return_speed`) 불변 → ingress 호출부 하위호환.

- [ ] **Step 1: goal 섹션에 필드 추가** — `return_speed` 줄 아래, 첫 `---` 위에:

```
bool egress             # true => 차밑 '나오기' 모드: 축검출 무시, 좌우 뎁스 중앙유지하며
                        #   egress_target_z(world z)까지 전진 후 정지. trough_index 는 무시.
float32 egress_target_z # egress 목표 world z(회랑선 근처). egress=false 면 무시.
```

- [ ] **Step 2: 빌드**

Run: `cd /home/rokey/p3/cobot_ws && colcon build --packages-select parking_robot_interfaces && source install/setup.bash`
Expected: 빌드 성공.

- [ ] **Step 3: 인터페이스 확인**

Run: `ros2 interface show parking_robot_interfaces/action/IngressUnderTruck | grep -E "egress"`
Expected: `bool egress` 와 `float32 egress_target_z` 두 줄 출력.

- [ ] **Step 4: 커밋**

```bash
git add src/parking_robot_interfaces/action/IngressUnderTruck.action
git commit -m "feat(iface): IngressUnderTruck 에 egress/egress_target_z(나오기 모드)"
```

---

## Task 3: ingress_node egress goal 처리 (travel 축 z)

**Files:**
- Modify: `src/parkbot_motion/parkbot_motion/ingress_node.py` (`_on_odom`/`_on_pose_stamped`/`_handle_pose`, `_execute`, `_on_goal`)

**Interfaces:**
- Consumes: Task1 `IngressController(egress=, egress_target=)`, Task2 `goal.egress`/`goal.egress_target_z`.
- Produces: `goal.egress==True` 면 travel=pose.z(북진), `IngressController(egress=True, egress_target=goal.egress_target_z)` 로 구동. 결과/피드백 필드는 재사용(stop_x 에 정지 z 기록).

- [ ] **Step 1: `_handle_pose` 가 z 도 받도록** — `_on_odom`/`_on_pose_stamped` 가 `p.z` 를 함께 넘기고, `_handle_pose(x, z, yaw_deg, stamp)` 로 시그니처 확장. travel 선택:

```python
    def _handle_pose(self, x, z, yaw_deg, stamp):
        x = float(x); z = float(z)
        egress = self._active is not None and self._active.get('egress', False)
        travel = z if egress else x            # egress=북진(world z), ingress=world x
        self._last_travel_x = travel
```
(이하 기존 로직의 `x` 를 `travel` 로 사용. `ctrl.step(travel, self._last_yaw_deg, ...)`.)
호출부 2곳:
```python
    def _on_odom(self, msg):
        p = msg.pose.pose.position; q = msg.pose.pose.orientation
        self._handle_pose(p.x, p.z, odom_quat_to_yaw_deg(q.x, q.y, q.z, q.w), msg.header.stamp)
    def _on_pose_stamped(self, msg):
        p = msg.pose.position; q = msg.pose.orientation
        self._handle_pose(p.x, p.z, odom_quat_to_yaw_deg(q.x, q.y, q.z, q.w), msg.header.stamp)
```

- [ ] **Step 2: `_on_goal` egress 허용** — `trough_index<0` 거부 가드를 egress 면 건너뛴다:

```python
        if not bool(goal_request.egress) and int(goal_request.trough_index) < 0:
            self.get_logger().warn(...)   # 기존 메시지 유지
            return GoalResponse.REJECT
```

- [ ] **Step 3: `_execute` egress 분기** — ctrl 생성 직전에:

```python
        egress = bool(goal.egress)
        if egress:
            ctrl = IngressController(
                0, egress=True, egress_target=float(goal.egress_target_z),
                forward_speed=forward_speed, lat_kp=self.lat_kp, lat_vy_max=self.lat_vy_max,
                lat_deadband=self.lat_deadband, settle_frames=self.settle_frames)
        else:
            ctrl = IngressController(trough_index, forward_speed=forward_speed, ...)  # 기존 그대로
```
`active` dict 에 `'egress': egress` 추가.

- [ ] **Step 4: 노드 임포트/파싱 확인**(통합은 Task6 미션에서)

Run: `source install/setup.bash && python3 -c "import ast; ast.parse(open('src/parkbot_motion/parkbot_motion/ingress_node.py').read()); print('parse OK')"`
Expected: `parse OK`.

- [ ] **Step 5: 커밋**

```bash
git add src/parkbot_motion/parkbot_motion/ingress_node.py
git commit -m "feat(ingress): ingress_node egress goal 처리(travel 축 world z, 축검출 우회)"
```

---

## Task 4: orchestrator 이탈+회랑 재작성 (`_return_egress_to_corridor`)

**Files:**
- Modify: `src/parkbot_motion/parkbot_motion/pickup_orchestrator_node.py` (`_egress` 신규, `_return_egress_to_corridor` 재작성)

**Interfaces:**
- Consumes: 기존 `_navigate_phase_b`, `_set_localizer_ref`, `_client`, `self.ingress_action`, `self.navigate_odom_action`, `self.navigate_fused_action`, `self.phase_b_xn_z`, `self.return_lead_parked_z`, `park_slot_x`, slot/lane 마커 파라미터.
- Produces: `_egress(rid, target_z, forward_speed=0.0)` → IngressUnderTruck egress goal 발행, (ok, reason). `_return_egress_to_corridor(rid, is_leader, dock_x, corridor_id, leader_id)` → 회랑선(dock_x, 7.075, yaw −90) 서향 정렬까지.

- [ ] **Step 1: `_egress` 헬퍼 추가**(`_ingress` 아래):

```python
    def _egress(self, robot_id, target_z, forward_speed=0.0):
        """차밑 나오기: 좌우 뎁스 중앙유지하며 world +z(북)로 target_z 까지 전진."""
        client = self._client(robot_id, 'ingress', IngressUnderTruck, self.ingress_action)
        goal = IngressUnderTruck.Goal()
        goal.egress = True
        goal.egress_target_z = float(target_z)
        goal.forward_speed = float(forward_speed)
        result, _status, reason = self._call_action(
            client, goal, label=f'egress[{robot_id}]', result_timeout=INGRESS_RESULT_TIMEOUT)
        if result is None:
            return False, reason
        if not result.success:
            return False, f'egress[{robot_id}] 실패(stop_reason={result.stop_reason})'
        return True, None
```

- [ ] **Step 2: `_return_egress_to_corridor` 재작성** — lead 는 (남향→북향 제자리회전 후) 뎁스 egress, follow 는 마커 북진(기존). 이후 서향90 회전 + 서진:

```python
    def _return_egress_to_corridor(self, rid, is_leader, dock_x, corridor_id, leader_id):
        """슬롯(차밑)→회랑(dock_x, 7.075) 정렬. lead=뎁스 egress, follow=마커 북진.
        회랑 도달 후 서향(yaw −90)으로 90° 회전(다음 서진 준비)."""
        node = self._return_localizer_node(rid, leader_id)
        slot_x = self.park_slot_x
        corr_z = self.phase_b_xn_z
        slot_ref = [self.park_slot_lane_id, self.park_slot_front_id, self.park_slot_center_id]
        if is_leader:
            # 남향(yaw180)→북향(yaw0) 제자리 회전(odom). 그 뒤 뎁스 중앙유지로 북진 이탈.
            ok, reason = self._navigate_phase_b(
                rid, 'nav_odom', self.navigate_odom_action,
                slot_x, self.return_lead_parked_z, 0.0, f'return:lead-rotate[{rid}]')
            if not ok:
                return False, reason
            ok, reason = self._egress(rid, corr_z)          # 뎁스 egress → (slot_x, ~corr_z)
        else:
            # follow: 얕아서 마커(slot_ref, 양캠)로 북진 이탈.
            ok, reason = self._set_localizer_ref(
                node, slot_ref, True, f'return:egress-ref[{rid}]', use_front=True)
            if ok:
                ok, reason = self._navigate_phase_b(
                    rid, 'nav_fused', self.navigate_fused_action,
                    slot_x, corr_z, 0.0, f'return:egress[{rid}]')
        if not ok:
            return False, reason
        # 서향 90° 회전(제자리, odom) — 위치 유지, yaw 0→−90.
        ok, reason = self._navigate_phase_b(
            rid, 'nav_odom', self.navigate_odom_action,
            slot_x, corr_z, -90.0, f'return:turn-west[{rid}]')
        if not ok:
            return False, reason
        # 회랑 바닥마커(61~64+회랑마커)로 전환 후 서진(양캠, yaw −90 유지).
        ok, reason = self._set_localizer_ref(
            node, self.park_lane_marker_ids + [corridor_id], True,
            f'return:corridor-ref[{rid}]', use_front=True)
        if ok:
            ok, reason = self._navigate_phase_b(
                rid, 'nav_fused', self.navigate_fused_action,
                dock_x, corr_z, -90.0, f'return:corridor-west[{rid}]')
        return ok, reason
```

- [ ] **Step 3: 파싱 확인**

Run: `python3 -c "import ast; ast.parse(open('src/parkbot_motion/parkbot_motion/pickup_orchestrator_node.py').read()); print('parse OK')"`
Expected: `parse OK`.

- [ ] **Step 4: 커밋**

```bash
git add src/parkbot_motion/parkbot_motion/pickup_orchestrator_node.py
git commit -m "feat(return): 이탈 재작성 — lead 뎁스 egress, 회랑 서향회전+서진"
```

---

## Task 5: orchestrator 후진 도크진입 + 동향 복원 (`_return_dock`)

**Files:**
- Modify: `src/parkbot_motion/parkbot_motion/pickup_orchestrator_node.py` (`_return_dock` 재작성)

**Interfaces:**
- Consumes: 기존 `_set_localizer_ref`, `_navigate_phase_b`, dock 파라미터(`phase_b_*_dock_id/x/z`), `self.return_dock_yaw`, `self.phase_b_xn_z`.
- Produces: `_return_dock(rid, is_leader, leader_id)` → 회랑(dock_x, 7.075, −90)에서 북향90 회전 → 후진으로 도크(dock_x, dock_z, 0) 진입 → 동향(yaw 90) 복원.

- [ ] **Step 1: `_return_dock` 재작성**:

```python
    def _return_dock(self, rid, is_leader, leader_id):
        """회랑(dock_x, 7.075, −90)→북향 회전→후진(−z)으로 도크 진입→동향90 복원."""
        node = self._return_localizer_node(rid, leader_id)
        dock_id = self.phase_b_leader_dock_id if is_leader else self.phase_b_follower_dock_id
        dock_x = self.phase_b_leader_dock_x if is_leader else self.phase_b_follower_dock_x
        dock_z = self.phase_b_leader_dock_z if is_leader else self.phase_b_follower_dock_z
        corr_z = self.phase_b_xn_z
        # 북향 90° 회전(제자리, odom): yaw −90→0. 이후 북향에서 후진하면 world −z(남진).
        ok, reason = self._navigate_phase_b(
            rid, 'nav_odom', self.navigate_odom_action,
            dock_x, corr_z, 0.0, f'return:turn-north[{rid}]')
        if not ok:
            return False, reason
        # 도크마커로 ref 전환 후 후진 도크진입(양캠, yaw 0 유지). pose_controller 가 −z 로 몬다.
        ok, reason = self._set_localizer_ref(
            node, [dock_id], True, f'return:dock-ref[{rid}]', use_front=True)
        if ok:
            ok, reason = self._navigate_phase_b(
                rid, 'nav_fused', self.navigate_fused_action,
                dock_x, dock_z, 0.0, f'return:dock[{rid}]')
        if not ok:
            return False, reason
        # 동향(yaw 90) 복원(제자리, odom) — 다음 입차 seed(yaw90)와 자세 일치.
        return self._navigate_phase_b(
            rid, 'nav_odom', self.navigate_odom_action,
            dock_x, dock_z, self.return_dock_yaw, f'return:dock-face[{rid}]')
```

- [ ] **Step 2: 파싱 확인**

Run: `python3 -c "import ast; ast.parse(open('src/parkbot_motion/parkbot_motion/pickup_orchestrator_node.py').read()); print('parse OK')"`
Expected: `parse OK`.

- [ ] **Step 3: 커밋**

```bash
git add src/parkbot_motion/parkbot_motion/pickup_orchestrator_node.py
git commit -m "feat(return): 도크 후진진입 + 동향90 복원"
```

---

## Task 6: 통합 검증 — 실제 A1 복귀 미션

**Files:** (검증 전용, 코드 변경 없음. 필요 시 파라미터 조정만)

**검증 방법(일회용 코드 금지 — 실제 미션):**

- [ ] **Step 1: 노드 self-check 회귀 확인**

Run: `python3 src/parkbot_motion/parkbot_motion/ingress_control.py && python3 src/parkbot_motion/parkbot_motion/carry_action_server.py demo`
Expected: 둘 다 `... OK`.

- [ ] **Step 2: 심링크 반영 위해 파이썬 패키지 빌드**(액션 외 소스는 심링크지만 안전하게)

Run: `cd /home/rokey/p3/cobot_ws && colcon build --packages-select parkbot_motion parking_robot_interfaces && source install/setup.bash`
Expected: 성공.

- [ ] **Step 3: 실제 미션 실행**(터미널1=Isaac, 터미널2=`ros2 launch parkbot_motion nodes.launch.py`, 터미널3):

Run: `ros2 action send_goal /entry/execute_parking_task parking_robot_interfaces/action/ExecuteParkingTask "{slot_id: 'A1'}"`
Expected(복귀 구간): feedback `RETURN_FOLLOW_OUT`→`RETURN_LEAD_OUT`→`RETURN_DOCK`. **lead 가 바퀴 안 긁고 이탈**(INGRESS_DBG phase=EGRESS 로그 확인), 두 로봇이 회랑에서 서향 회전→서진→북향 회전→후진으로 도크(follow −1.2 / lead −3.2, z≈2.2)에 안착 후 동향(yaw≈90).

- [ ] **Step 4: 부호/좌표 튜닝**(필요 시) — EGRESS world-z 전진 부호가 반대면 `ingress_control.py` egress 분기의 `copysign` 인자, 또는 egress 목표 z 를 로그 실측으로 조정. 조정 후 Step1 self-check + Step3 재실행.

- [ ] **Step 5: 최종 커밋**(튜닝이 있었으면)

```bash
git add -A && git commit -m "fix(return): 복귀 부호/좌표 실측 튜닝"
```

---

## Self-Review

**스펙 커버리지:**
- 회전+직진(D1) → Task4(서향회전+서진), Task5(북향회전+후진). ✅
- lead 뎁스 이탈(D2) → Task1~4(EGRESS 모드 + `_egress` 호출). ✅
- follow 마커 이탈(D3) → Task4 else 분기(slot_ref 북진). ✅
- 동향90 복원(D4) → Task5 마지막 `return:dock-face`. ✅
- 순차 이탈/조율 → 기존 `_run_return`(follow→lead 순차) 유지, 변경 불필요. ✅

**Placeholder 스캔:** egress world-z 부호만 "구현 중 확정"으로 남김 — 물리 부호라 실측이 유일 진실원(Global Constraints 에 명시, Task6 Step4 가 해소 절차). 그 외 TBD 없음.

**타입 일관성:** `IngressController(egress=, egress_target=, egress_stop_tol=)`(Task1) = ingress_node 생성부(Task3) = 액션 필드 `egress/egress_target_z`(Task2). `_egress(rid, target_z)`(Task4) → `goal.egress_target_z`. `PHASE_EGRESS` 일관. `_handle_pose(x, z, yaw_deg, stamp)` 4-인자 = 호출부 2곳(Task3 Step1). ✅
