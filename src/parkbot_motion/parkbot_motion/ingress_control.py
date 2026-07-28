#!/usr/bin/env python3
"""ingress_control — R5a: 차 밑 진입 안무의 순수 정책(Isaac/rclpy 비의존).

설계서 ``docs/superpowers/specs/2026-07-25-ros2-node-refactor-design.md`` §3~4(R5a).
``isaacpjt/Isaac_envo/parking_v4_runner.py`` 의 ``_ingress_axle``(3363행 부근,
``--mission=C`` 진입 안무)가 스텝마다 하던 "무엇을 명령할지" 결정만 뽑아낸다
(``PoseController``/``pose_controller.py`` 와 같은 관례: "어떻게 관측/구동할지"는
여전히 ROS2 배관(``ingress_node.py``)에 남는다).

## 행동 보존 — 상수는 러너가 실측 검증한 값 그대로

``FWD_SPEED=0.4``/``RETURN_SPEED=0.15``(러너 3250-3251행), ``LAT_KP=1.2``/
``LAT_VY_MAX=0.15``/``LAT_DEADBAND=0.01``(taskC2fix-report.md §2.2 실측값,
러너 3276행 및 R4 ``axle_smoke.py`` 가 재사용), ``pos_tol=0.02``(러너 3389행)
— 숫자를 바꾸지 않았다.

## 3단계 상태기계 — SEEK -> RETURN -> SETTLING -> DONE

- **SEEK**: ``forward_speed`` 로 전진 + 좌우 뎁스 중앙유지(vy). 요구된
  트로프(``trough_index``, 0-based)가 ``axle_centers``(호출자가 매 틱 넘기는,
  지금까지 확정된 트로프 중심 리스트)에 채워지면 즉시 RETURN 으로 전환한다.
  **하드 제약(브리프 그대로)**: 트로프는 이탈이 확정돼야 완료되므로, 전환
  시점엔 로봇이 이미 그 축을 지나쳐 있다 — 그래서 RETURN(후진 정렬)이
  필요하다(러너 주석 "후진 정렬"과 동일 개념).
- **RETURN**: ``target_x``(방금 확정된 트로프 중심)로 P 제어(게인=1.0,
  ``return_speed`` 로 클립) 복귀, 좌우 중앙유지는 계속 유지한다(러너
  ``_ingress_axle`` 의 return 분기도 ``vy_cmd`` 를 계속 실어 보낸다 — 축과 축
  사이에서도 통로 중앙을 벗어나지 않아야 하므로).
- **SETTLING**: ``pos_tol`` 이내로 들어오면 ``settle_frames`` 틱 동안 0 twist를
  유지하며 정지가 실제로 자리잡을 시간을 준다(``PoseController`` 의 settle
  개념과 같은 취지, 다만 여긴 median 스무딩이 없다 — 1차원 위치 하나만 다루므로
  마지막 표본을 그대로 최종값으로 쓴다).
- **DONE**: 이후 계속 (0,0,0)만 낸다(멱등).

## 무엇을 이 모듈이 모르는가(정직하게)

- **뎁스 신호 완전 유실 시 전체 정지**는 이 모듈의 책임이 아니다 — 호출자
  (``ingress_node.py``)가 ROS 토픽 신선도(마지막 수신 후 경과시간)를 감시해
  ``left_min``/``right_min`` 을 이미 ``None`` 으로 저하시킨 뒤 이 모듈에
  넘겨야 한다(이 모듈은 인자로 받은 값을 신뢰할 뿐 타임스탬프를 전혀 모른다).
  ``left``/``right`` 둘 다 ``None`` 이면 중앙유지는 자연히 vy=0 이 되지만,
  전진(vx) 자체를 멈추는 것은 별도 안전장치(브리프 지시)이므로 노드가 한다.
- **포즈 워치독**(코스팅 방지, R3c 가 실측으로 찾은 버그와 동일 클래스의
  위험)도 이 모듈 밖이다 — 이 모듈은 틱이 안 들어오면 아무것도 안 한다
  (마지막 반환값이 뭐였는지도 기억 안 함, 호출자가 그 값을 무기한 재사용하면
  안 된다).
"""
import math

DEFAULT_FORWARD_SPEED = 0.4    # m/s -- 러너 FWD_SPEED
DEFAULT_RETURN_SPEED = 0.15    # m/s -- 러너 RETURN_SPEED
DEFAULT_LAT_KP = 1.2           # 1/s -- 러너 LAT_KP(taskC2fix 실측)
DEFAULT_LAT_VY_MAX = 0.15      # m/s -- 러너 LAT_VY_MAX
DEFAULT_LAT_DEADBAND = 0.01    # m   -- 러너 LAT_DEADBAND
DEFAULT_POS_TOL = 0.02         # m   -- 러너 _ingress_axle pos_tol
DEFAULT_SETTLE_FRAMES = 10     # 틱  -- 이 모듈 전용(러너엔 대응 없음, PoseController
                                #   settle_frames=30 보다 짧게 잡음 -- 1차원 위치만
                                #   다뤄 median 스무딩이 필요 없으므로 표본 수를
                                #   줄여도 안전하다는 판단, 필요시 파라미터로 늘릴 것)
# 진입 중 yaw 유지(2026-07-28 사용자): 마커융합 /pose yaw 로 목표 헤딩 유지해 똑바로 진입.
DEFAULT_YAW_KP = 0.8            # 진입 yaw유지 게인(pose_controller 1.2 보다 부드럽게)
DEFAULT_YAW_WZ_MAX = 0.3       # rad/s -- 진입 중 회전은 완만히(주행과 동시)
DEFAULT_YAW_DEADBAND_DEG = 1.0  # deg  -- 이 이내면 wz=0(불필요한 미세회전 억제)
DEFAULT_YAW_WZ_MIN = 0.05      # rad/s -- 메카넘 회전 데드밴드 보정(mission_control 동형)


def lateral_centring_vy(left, right, *, kp=DEFAULT_LAT_KP, vy_max=DEFAULT_LAT_VY_MAX,
                         deadband=DEFAULT_LAT_DEADBAND):
    """좌우 ROI-min 뎁스 -> vy 명령[m/s]. 러너 ``_ingress_axle``(3412-3422행)와
    동일 규칙 이식: 둘 다 유효할 때만 보정하고, 한쪽만 유효하거나 둘 다 무효면
    강제로 옆으로 밀지 않는다(vy=0) — "왼쪽/오른쪽 둘 다 유효할 때만 보정"이
    taskC2fix 가 확립한 핵심 안전장치(§2.2)다.

    ``left``/``right`` 는 ``None`` 가능(무효 신호 — 유효 픽셀 없음 또는 토픽
    유실, 호출자가 판정해 넘긴다).
    """
    if left is None or right is None:
        return 0.0
    err = float(left) - float(right)
    if abs(err) < deadband:
        return 0.0
    return max(-vy_max, min(vy_max, kp * err))


def yaw_hold_wz(yaw_deg, hold_yaw_deg, *, kp, wz_max, deadband_deg, wz_min):
    """진입 주행 중 yaw 유지 wz[rad/s]. (사용자 지시 2026-07-28)

    진입 전 Phase B 정렬은 트럭에서 먼 마커에서 하고, 정렬 뒤 트럭까지 wz=0 로
    수 m 주행하는 동안 메카넘 롤러 슬립으로 yaw 가 틀어져 **바퀴에 부딪혔다**.
    진입은 이미 마커융합 /pose 를 쓰므로(회랑마커 61~64 가 진입경로 바닥에 있음)
    그 yaw 로 목표 헤딩(lead −90/follow +90)을 매 틱 잡아 똑바로 들어가게 한다.

    yaw_deg/hold_yaw_deg=None 이면 0(비활성, 하위호환). err 는 최단각[-180,180].
    데드밴드 밖인데 비례 wz 가 wz_min 보다 작으면 메카넘 회전 데드밴드 보정으로 깐다.
    부호·규약은 pose_controller 와 동일(project yaw, angular.z 그대로, 브리지가 SIGN_YAW).
    """
    if hold_yaw_deg is None or yaw_deg is None:
        return 0.0
    err = ((hold_yaw_deg - yaw_deg + 180.0) % 360.0) - 180.0
    if abs(err) < deadband_deg:
        return 0.0
    wz = max(-wz_max, min(wz_max, kp * math.radians(err)))
    if abs(wz) < wz_min:
        wz = math.copysign(wz_min, err)
    return wz


def return_phase_vx(remaining, return_speed, drive_sign=1.0):
    """RETURN 단계 vx: 러너 3441행 ``spd = clip(-1.0*remaining, ±RETURN_SPEED)``
    그대로. ``remaining = target_x - travel_x``(호출자가 계산해 넘긴다).

    ``drive_sign``: 로컬 forward(+vx)가 world x 를 감소시키면 +1(서향 로봇,
    APPROACH_YAW=-90°, 기존 관례). 동향(+90°) 로봇이 **후진**으로 진입하면
    forward 가 world x 를 증가시키므로 -1 — SEEK/RETURN 두 vx 부호를 함께
    뒤집는다(2026-07-27 재안무: follow 가 동쪽 보며 -x 로 진입).
    """
    return max(-return_speed, min(return_speed, drive_sign * -1.0 * float(remaining)))


def pick_target_axle(axle_centers, trough_index):
    """``trough_index``(0-based) 번째 트로프가 이미 확정됐으면 그 중심 x, 아직
    이면 ``None``. 순수 리스트 인덱싱 — ``axle_detector_node`` 의 0-based
    ``axle_index`` 규약과 동일(그 노드가 완료 순서대로 ``axle_centers`` 리스트를
    채운다고 가정, 호출자가 매 틱 그 시점까지의 스냅샷을 넘긴다).
    """
    if trough_index < 0:
        raise ValueError(f"trough_index 는 0 이상이어야 합니다: {trough_index!r}")
    if len(axle_centers) > trough_index:
        return axle_centers[trough_index]
    return None


class IngressController:
    """1차원 진입 안무 정책: SEEK(전진+중앙유지) -> RETURN(후진 정렬) ->
    SETTLING -> DONE.

    사용법(``ingress_node.py`` 의 포즈 콜백이 매 틱 하는 일):
    ``vx, vy, wz = ctrl.step(travel_x, left_min, right_min, axle_centers, dt)``
    를 불러 이번 틱에 명령할 바디 twist 를 얻고 그대로 ``/cmd_vel`` 에 싣는다.
    ``ctrl.done`` 이 True 가 되면 ``ctrl.final_stop_x``/``ctrl.target_x`` 를
    결과에 담아 액션을 종료한다.
    """

    PHASE_EGRESS = 'EGRESS'
    PHASE_SEEK = 'SEEK'
    PHASE_RETURN = 'RETURN'
    PHASE_SETTLING = 'SETTLING'
    PHASE_DONE = 'DONE'

    def __init__(self, trough_index, *, forward_speed=DEFAULT_FORWARD_SPEED,
                 return_speed=DEFAULT_RETURN_SPEED, lat_kp=DEFAULT_LAT_KP,
                 lat_vy_max=DEFAULT_LAT_VY_MAX, lat_deadband=DEFAULT_LAT_DEADBAND,
                 pos_tol=DEFAULT_POS_TOL, settle_frames=DEFAULT_SETTLE_FRAMES,
                 drive_sign=1.0, hold_yaw_deg=None, yaw_kp=DEFAULT_YAW_KP,
                 yaw_wz_max=DEFAULT_YAW_WZ_MAX, yaw_deadband_deg=DEFAULT_YAW_DEADBAND_DEG,
                 yaw_wz_min=DEFAULT_YAW_WZ_MIN,
                 egress=False, egress_target=None, egress_stop_tol=0.15):
        if trough_index < 0:
            raise ValueError(f"trough_index 는 0 이상이어야 합니다: {trough_index!r}")
        self.trough_index = trough_index
        # +1: forward=world -x(서향, 기존). -1: 동향 로봇 후진진입(vx 부호 반전).
        self.drive_sign = 1.0 if drive_sign >= 0 else -1.0
        self.forward_speed = forward_speed
        self.return_speed = return_speed
        self.lat_kp = lat_kp
        self.lat_vy_max = lat_vy_max
        self.lat_deadband = lat_deadband
        self.pos_tol = pos_tol
        self.settle_frames = max(1, int(settle_frames))
        # 진입 중 유지할 목표 헤딩[deg]. None 이면 yaw 유지 비활성(기존 wz=0).
        self.hold_yaw_deg = None if hold_yaw_deg is None else float(hold_yaw_deg)
        self.yaw_kp = yaw_kp
        self.yaw_wz_max = yaw_wz_max
        self.yaw_deadband_deg = yaw_deadband_deg
        self.yaw_wz_min = yaw_wz_min
        # EGRESS(차밑 나오기): 축검출 없이 좌우 뎁스 중앙유지하며 egress_target(주행좌표)
        # 까지 전진, |남은거리|<=egress_stop_tol 이면 정지. drive_sign/trough 무관.
        self.egress = bool(egress)
        self.egress_target = None if egress_target is None else float(egress_target)
        self.egress_stop_tol = float(egress_stop_tol)

        self.phase = self.PHASE_EGRESS if self.egress else self.PHASE_SEEK
        self.target_x = None
        self.final_stop_x = None
        self._settle_count = 0

        # 진단용(GT 아님) — 명령 vy 를 시간적분한 횡위치 추정치의 최대 절대값.
        # 실제 횡편차 검증은 GT 를 가진 외부 스모크가 한다(이 모듈/노드는 GT를
        # 전혀 모른다 — 브리프의 "GT는 콘솔전용, 제어입력 아님" 원칙과 같은
        # 이유로 여기 있는 값도 어디까지나 "명령이 얼마나 옆으로 밀었다고
        # 주장하는가"의 추정치일 뿐, 실제 물리 편차와 다를 수 있다).
        self._lat_pos_est = 0.0
        self.max_lat_dev_est = 0.0

    @property
    def done(self):
        return self.phase == self.PHASE_DONE

    def step(self, travel_x, yaw_deg, left_min, right_min, axle_centers, dt):
        """한 틱: (주행좌표, 현재 yaw[deg], 좌뎁스, 우뎁스, 지금까지 확정된 트로프
        중심 리스트, dt) -> (vx, vy, wz).

        ``wz`` 는 **SEEK(전진 진입 주행) 에서만** 목표 헤딩(hold_yaw_deg) 유지로 낸다
        (2026-07-28: 진입 전 먼 마커 정렬만으론 주행 중 요 드리프트로 바퀴에 부딪혀,
        마커융합 /pose yaw 로 매 틱 잡는다). **RETURN 은 wz=0**(2026-07-28 정정): 깊은
        축에서의 미세 x정렬 구간인데, 거기선 로봇이 트럭 바퀴 사이에 끼어 있고 rear 캠이
        마커를 완전히 잃어(실측 x<-6.8 블라인드) /pose yaw 가 순수 odom 이라, 그 드리프트로
        wz 를 내면 끼인 채 회전해 바퀴를 밀어버린다(트럭 밀림 실측). SETTLING/DONE 도 wz=0.
        hold_yaw_deg=None 이면 SEEK 도 wz=0(기존 동작).
        """
        vy = lateral_centring_vy(left_min, right_min, kp=self.lat_kp,
                                  vy_max=self.lat_vy_max, deadband=self.lat_deadband)
        self._lat_pos_est += vy * dt
        self.max_lat_dev_est = max(self.max_lat_dev_est, abs(self._lat_pos_est))
        wz = yaw_hold_wz(yaw_deg, self.hold_yaw_deg, kp=self.yaw_kp,
                         wz_max=self.yaw_wz_max, deadband_deg=self.yaw_deadband_deg,
                         wz_min=self.yaw_wz_min)

        if self.phase == self.PHASE_EGRESS:
            remaining = self.egress_target - travel_x
            if abs(remaining) <= self.egress_stop_tol:
                self.phase = self.PHASE_SETTLING
                self._settle_count = 0
                self.final_stop_x = travel_x
            else:
                vx = math.copysign(self.forward_speed, remaining)  # 목표 향해 등속 전진
                return (vx, vy, 0.0)   # 블라인드 축이동 — 회전 금지(바퀴 밀림, RETURN 동형)

        if self.phase == self.PHASE_SEEK:
            target = pick_target_axle(axle_centers, self.trough_index)
            if target is None:
                return (self.drive_sign * self.forward_speed, vy, wz)
            self.target_x = float(target)
            self.phase = self.PHASE_RETURN
            # 같은 틱에 RETURN 을 곧바로 평가한다(축이 확정된 그 순간부터
            # 되돌아가기 시작 -- 러너도 트로프 완료를 감지한 스텝에서 바로
            # phase 전환 후 같은 스텝 루프 흐름으로 이어간다).

        if self.phase == self.PHASE_RETURN:
            remaining = self.target_x - travel_x
            if abs(remaining) <= self.pos_tol:
                self.phase = self.PHASE_SETTLING
                self._settle_count = 0
            else:
                vx = return_phase_vx(remaining, self.return_speed, self.drive_sign)
                return (vx, vy, 0.0)   # RETURN: 블라인드 축정렬 — 회전 금지(바퀴 밀림)

        if self.phase == self.PHASE_SETTLING:
            self._settle_count += 1
            if self._settle_count >= self.settle_frames:
                self.phase = self.PHASE_DONE
                self.final_stop_x = travel_x
            return (0.0, 0.0, 0.0)

        return (0.0, 0.0, 0.0)  # PHASE_DONE


if __name__ == "__main__":
    # drive_sign 자기검증(회귀 가드): 동향 후진(-1)이면 SEEK/RETURN vx 둘 다 서향(+1)과
    # 정확히 반대 부호여야 한다. 축은 아직 미검출이라 SEEK 국면.
    w = IngressController(0, drive_sign=1.0)   # 서향(기존)
    e = IngressController(0, drive_sign=-1.0)  # 동향(후진 진입)
    vw = w.step(0.0, None, None, None, [], 0.05)[0]  # SEEK vx (yaw_deg=None → wz=0)
    ve = e.step(0.0, None, None, None, [], 0.05)[0]
    assert vw > 0 and ve < 0 and abs(vw + ve) < 1e-9, (vw, ve)
    # RETURN vx 도 부호 반전(같은 remaining 부호에서 서향 음수/동향 양수).
    assert return_phase_vx(0.5, 0.15, 1.0) < 0 < return_phase_vx(0.5, 0.15, -1.0)

    # yaw 유지 자기검증: hold_yaw_deg=None 이면 wz=0(하위호환), 지정 시 오차 줄이는 방향.
    assert yaw_hold_wz(85.0, None, kp=0.8, wz_max=0.3, deadband_deg=1.0, wz_min=0.05) == 0.0
    # follow 목표 +90°, 현재 85°(부족) → +방향 회전(err=+5° >0). 서로 반대편 오차는 반대부호.
    lo = yaw_hold_wz(85.0, 90.0, kp=0.8, wz_max=0.3, deadband_deg=1.0, wz_min=0.05)
    hi = yaw_hold_wz(95.0, 90.0, kp=0.8, wz_max=0.3, deadband_deg=1.0, wz_min=0.05)
    assert lo > 0 and hi < 0 and abs(lo + hi) < 1e-9, (lo, hi)
    # 데드밴드 안(0.5°<1°) → 0. 큰 오차는 wz_max 로 포화.
    assert yaw_hold_wz(89.5, 90.0, kp=0.8, wz_max=0.3, deadband_deg=1.0, wz_min=0.05) == 0.0
    assert abs(yaw_hold_wz(0.0, 90.0, kp=0.8, wz_max=0.3, deadband_deg=1.0, wz_min=0.05)) == 0.3
    # ±180 경계 최단각: 179 → -179 목표면 err=+2°(양수), -178° 로 안 돈다.
    assert yaw_hold_wz(179.0, -179.0, kp=0.8, wz_max=0.3, deadband_deg=1.0, wz_min=0.05) > 0
    # SEEK 에서 hold_yaw 주면 wz 가 실린다(yaw 부족 → 회전 명령).
    h = IngressController(0, drive_sign=-1.0, hold_yaw_deg=90.0)
    _vx, _vy, wz = h.step(0.0, 85.0, None, None, [], 0.05)
    assert wz > 0, wz
    # RETURN(블라인드 축정렬)은 yaw 오차가 커도 wz=0 — 끼인 채 회전 금지(바퀴 밀림).
    r = IngressController(0, drive_sign=1.0, hold_yaw_deg=-90.0)
    _vx, _vy, wz = r.step(0.0, -70.0, None, None, [0.5], 0.05)  # 축확정→RETURN, yaw 20°틀림
    assert r.phase == IngressController.PHASE_RETURN and wz == 0.0, (r.phase, wz)

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

    print("ingress_control self-check OK (drive_sign + yaw_hold + RETURN wz0 + EGRESS)")
