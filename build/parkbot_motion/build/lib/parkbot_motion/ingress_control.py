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


def return_phase_vx(remaining, return_speed):
    """RETURN 단계 vx: 러너 3441행 ``spd = clip(-1.0*remaining, ±RETURN_SPEED)``
    그대로. ``remaining = target_x - travel_x``(호출자가 계산해 넘긴다) — 이
    부호 관계는 로봇의 로컬 forward 가 주행좌표(world x)를 감소시키는 이 미션의
    실측 관례(APPROACH_YAW=-90°, taskC2fix/§)에서만 성립한다(문서화된 가정,
    ``ingress_node.py`` docstring 참고).
    """
    return max(-return_speed, min(return_speed, -1.0 * float(remaining)))


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

    PHASE_SEEK = 'SEEK'
    PHASE_RETURN = 'RETURN'
    PHASE_SETTLING = 'SETTLING'
    PHASE_DONE = 'DONE'

    def __init__(self, trough_index, *, forward_speed=DEFAULT_FORWARD_SPEED,
                 return_speed=DEFAULT_RETURN_SPEED, lat_kp=DEFAULT_LAT_KP,
                 lat_vy_max=DEFAULT_LAT_VY_MAX, lat_deadband=DEFAULT_LAT_DEADBAND,
                 pos_tol=DEFAULT_POS_TOL, settle_frames=DEFAULT_SETTLE_FRAMES):
        if trough_index < 0:
            raise ValueError(f"trough_index 는 0 이상이어야 합니다: {trough_index!r}")
        self.trough_index = trough_index
        self.forward_speed = forward_speed
        self.return_speed = return_speed
        self.lat_kp = lat_kp
        self.lat_vy_max = lat_vy_max
        self.lat_deadband = lat_deadband
        self.pos_tol = pos_tol
        self.settle_frames = max(1, int(settle_frames))

        self.phase = self.PHASE_SEEK
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

    def step(self, travel_x, left_min, right_min, axle_centers, dt):
        """한 틱: (주행좌표, 좌뎁스, 우뎁스, 지금까지 확정된 트로프 중심 리스트,
        dt) -> (vx, vy, wz). ``wz`` 는 이 안무에서 항상 0.0(요 보정 없음 —
        브리프의 하드 요구사항: 순수 ``linear.x`` 만으로는 요 드리프트가 나서
        충돌한다는 R4 의 실측을 반영해, 이 컨트롤러는 wz=0 을 내되 호출자가
        중앙유지(vy)와 결합해 드리프트를 억제하는 전체 설계에 의존한다 — 요
        자체를 능동 보정하는 루프는 taskC2fix 가 검증 표본 부재로 보류한
        것과 같은 이유로 여기서도 넣지 않았다).
        """
        vy = lateral_centring_vy(left_min, right_min, kp=self.lat_kp,
                                  vy_max=self.lat_vy_max, deadband=self.lat_deadband)
        self._lat_pos_est += vy * dt
        self.max_lat_dev_est = max(self.max_lat_dev_est, abs(self._lat_pos_est))

        if self.phase == self.PHASE_SEEK:
            target = pick_target_axle(axle_centers, self.trough_index)
            if target is None:
                return (self.forward_speed, vy, 0.0)
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
                vx = return_phase_vx(remaining, self.return_speed)
                return (vx, vy, 0.0)

        if self.phase == self.PHASE_SETTLING:
            self._settle_count += 1
            if self._settle_count >= self.settle_frames:
                self.phase = self.PHASE_DONE
                self.final_stop_x = travel_x
            return (0.0, 0.0, 0.0)

        return (0.0, 0.0, 0.0)  # PHASE_DONE
