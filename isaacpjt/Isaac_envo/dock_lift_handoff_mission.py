#!/usr/bin/env python3
"""인계장 도킹·리프트·슬롯 주차 오케스트레이터 (외부 ROS2 Humble).

/dock_lift(Trigger) 한 번에:
  1. 접근: 두 로봇이 도크에서 Coupe 앞(+z 아일 쪽) 정렬 위치로 옴니 이동(회전 없이)
  2. 뒷축 로봇 회전(느리게, GT 감시) → 차 밑으로 진입 → 뒷축 정지
  3. 앞축 로봇 회전 → 진입 → 앞축 정지 (순차)
  4. 파지·리프트
  5. 슬롯까지 운반: 통로 정렬(x, carry_speed) → 슬롯 진입(z, slot_entry_speed),
     매 틱 mod-180도 회전 보정(_carry_axis 내부)만으로 진행 — 슬롯 앞 명시적
     정렬(_rotate_car_to_axis)을 넣어봤는데 정렬 자체가 오래 걸리고(실측 39초)
     그 이후에도 편차가 계속 커지는 문제가 있어 뺐다. 목표는 target_slot_x/
     target_slot_z/target_axis_rad 파라미터(기본값 B1, pi/2) — 재시작 없이
     `ros2 param set`으로 바꿀 수 있고, 관제 쪽은 isaac_parking_bridge_node가
     ExecuteParkingTask goal의 slot_pose를 받아 이 값들을 설정해준다.
  6. 하차: 파지 해제(차량 착지) → 두 로봇 차체 밑에서 이탈(후진)
  7. 도크 복귀: 빈 로봇 각자 실제 도크 위치(home_pose, 1번 시작 전 실측 기록)로
     이동 → 원래 방위(yaw=0)로 회전

테스트 반복용: 러너의 /sim_checkpoint_staged(Trigger) 호출로 로봇을 게이트 통과 직후
대기 위치까지 순간이동시킨 뒤 /dock_lift_from_staged(Trigger)를 부르면 1(게이트 통과)만
건너뛰고 2~7(회전·진입·파지·운반·하차·복귀)은 그대로 실제로 실행한다 — 차 밑까지 직접
순간이동시키면 차체와 겹쳐 물리가 튕겨나가는 문제가 있어 이렇게 나눔.
/sim_reset(러너)은 씬을 도크 초기 상태로 되돌린다(재시작 불필요).

좌표: world XZ, 차 길이축=z, Coupe center_x≈-29.6. 로봇은 도크에서 +X 향함(yaw≈0),
차 밑 진입 방향은 -z(yaw=+pi/2). 회전은 GT yaw 폐루프(느림)로 안정화. 슬롯 좌표는
marker_map.json 과 같은 world 프레임(도크 x=-15.3 과 일치)이라 변환 없이 그대로 쓴다.
"""
import math
import sys
import time

import rclpy
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import SetParametersResult
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_srvs.srv import Trigger, SetBool

ROBOTS = ("robot_rear", "robot_front")
CONTROL_HZ = 20.0
STEP_TIMEOUT = 90.0
POS_TOL = 0.10
CORNER_TOL = 0.40   # 중간 웨이포인트는 이 반경에서 통과(정지 안 함) → 코너 부드럽게
YAW_TOL = math.radians(4.0)
FACE_MZ = math.pi / 2    # -z 를 향하는 yaw (odom 규약: atan2(-fwd_z, fwd_x)).
# 한 방향 진입: 두 로봇 다 개구부(북)쪽에서 -z 로 들어간다(둘 다 FACE_MZ).
# 실측: 메카넘 실속 = 0.30 × 지령(선형, 슬립비 일정). 접근은 지령 0.6(실속~0.18)으로
# 빠르게, 진입·운반은 0.30(실속~0.09)으로 젠틀·정밀하게.
K_LIN, MAX_LIN = 0.8, 0.6   # MAX_LIN은 회전 정렬(_rotate_car_to_axis) 전용 상수로 남김(그대로 보수적).
TRAVEL_SPEED = 0.6   # 일반 주행(게이트 통과·정렬 이동·도크 복귀 — 차 안 들고 있음)용.
                      # 차 밑 진입/슬롯 진입과 무관해서 별도로 크게 열어둔다.
K_STRAFE = 0.8
K_YAW, MAX_YAW = 0.5, 0.15   # 회전은 느리게(Plan 2: wz<=0.15 결정적, 발레 방지)
INGRESS_SPEED = 0.45   # 시연용 상향(0.30→0.40→0.45). 차 밑 진입이라 정밀도 필요 —
                        # 더 올리려면 dock_motion_check.py로 축 정렬 놓치는지 확인 권장.
CARRY_SPEED = 0.60     # 통로 횡단(x축, 개활지 직선)용. 슬롯 진입만큼 정밀도가
                        # 필요 없어 상한을 크게 열어둔다.
SLOT_ENTRY_SPEED = 0.60   # 슬롯 진입(z축)용 — carry_speed와 분리. 옆 슬롯/기둥에
                           # 가까워지는 정밀 구간이라 carry처럼 확 열지 않는다.
# 기본값일 뿐 — 런타임에 ROS2 파라미터로 재설정 가능(speed_control_ui.py).
# 실측: ingress_speed를 실수로 3.0까지 올렸더니 축 정렬을 놓쳐 차 밑 진입을
# 실패함(언더바디 0.243m - 로봇 0.18m = 6.3cm 여유뿐인 정밀 구간이라 빠르면
# 보정이 못 따라감). 그래서 3단계로 상한을 분리한다:
#   ingress(차 밑 진입)   — 가장 좁게, 정밀도 최우선
#   slot_entry(슬롯 진입)  — 중간, 옆 슬롯/기둥 근접이라 어느 정도는 조심
#   travel(빈 로봇 주행)   — 넓게, 차를 안 들고 있어 개활지만큼 안전
#   carry(통로 횡단, 차 실음) — 가장 넓게, 개활지 직선이라 빨라도 안전
SPEED_MIN = 0.10
INGRESS_SPEED_MAX = 0.70
SLOT_ENTRY_SPEED_MAX = 1.50
TRAVEL_SPEED_MAX = 8.0
CARRY_SPEED_MAX = 8.0
CARRY_DIST = 1.0

# 게이트 통과 안무: 서쪽 벽 개구부는 z∈[-4.5,4.5](폭 9m). 도크는 z=±7.8 로 개구부
# 양옆(벽 뒤)이라 서진 직진하면 WestNorth/WestSouth 벽에 박는다. 먼저 개구부 통로 z 로
# 옆이동한 뒤 서진해 벽(x≈-18.1)을 통과한다. 두 로봇 통로 z 를 벌려 상호 회피.
LANE_Z_REAR = -1.5      # 개구부 통과 통로(z∈[-4.5,4.5] 내부, 남쪽)
LANE_Z_FRONT = 1.5      # 개구부 통과 통로(북쪽)
NORTH_STAGE_Z = 4.0     # Pickup 차체 북쪽 끝(+2.91) 밖. 여기서 -z 로 진입(둘 다 사용).
# rear 가 먼저 여기서 -z 로 뒷축(-1.93)까지 깊이 통과, 그 다음 front 가 앞축(+1.66).
# (Pickup 언더바디 0.243m > 로봇 0.18m 라 차체 밑 관통 가능 — Coupe(0.163)는 불가였음.)
WALL_CLEAR_X = -20.0    # 서쪽 벽(-18.1) 서쪽, 인계장 바닥 안
DOCK_X = -15.3          # West 도크 x (robot:dockPose)
# 실측: 메카넘이 바닥에서 ~70% 슬립 → 지령 0.35 라도 실속 ~0.10 m/s(지령 더 올리면
# 슬립 악화로 오히려 느려짐). 그래서 속도는 그대로 두고 두 로봇 접근을 병렬화한다.
APPROACH_TIMEOUT = 300.0

# B1 슬롯 — 실제 슬롯 중심(marker_layout.py HANDOFF_BAY_Z/row_center와 동일 계산:
# AISLE_WIDTH*0.5 + SPACE_LENGTH*0.5 = 4.5+3.3 = 7.8). x는 marker_map.json B1과 동일
# (슬롯 인덱스 1 기준 -half_w+(1+0.5)*SPACE_WIDTH = -11.9, 이건 실제로 맞음).
# ⚠ marker_map.json 의 B1 z=-2.5 는 슬롯 중심이 아니라 "슬롯 입구(z=4.5)에서 통로
# 쪽으로 2m 앞"에 둔 카메라 인식용 마커 위치(LANE_Z) — 그 좌표로 가면 슬롯 밖(통로)에서
# 멈춘다(실측 확인됨). 슬롯에 실제로 들어가려면 z=-7.8(B 행은 음수)이어야 한다.
SLOT_B1_X = -11.9
SLOT_B1_Z = -7.8
SLOT_CARRY_TIMEOUT = 300.0   # 이동 거리가 길어(~25m) 기존 STEP_TIMEOUT(90s)로는 부족.
RETREAT_DIST = 4.0           # 하차 후 앞축 로봇이 차체 밑에서 벗어나는 후진 거리.
                              # 뒷축은 여기에 축간거리(front_axle-rear_axle)를 더 간다(_retreat).
AISLE_CENTER_Z = 0.0         # 중앙 통로 z=0 (AISLE_WIDTH=9.0m 기준 중심, marker_layout.py와 동일).
SIDE_K, SIDE_MAX = 0.8, 0.30  # 부축(주 이동축 아닌 쪽) 유지 보정 게인. 0.15로는 실측상
                               # 계속 포화 상태인데도 편차를 못 줄여서(비대칭 하중 토크가
                               # 더 셈) 0.30으로 올림 — 그래도 carry_speed 기본값(0.6)보다 작게.
CARRY_WZ_MAX = 0.15   # _carry_axis 회전 보정 상한. 원래 0.10이었는데 실측상 계속 포화된
                       # 채로 목표(90도)와 55도씩 차이 나는 걸 못 줄여서, 발레 방지용으로
                       # 이미 검증된 다른 상한(MAX_YAW)과 맞춰 0.15로 올림.


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


class HandoffMission(Node):
    def __init__(self):
        super().__init__("dock_lift_handoff_mission")
        p = self.declare_parameter
        # Pickup 인계장 중앙(z=0). fab wheel offset: front=+1.66, rear=-1.93(휠베이스 3.59m).
        # 러너 DOCK_STAGE_READY 의 front_z/rear_z 와 일치해야 함(불일치 시 파라미터로 덮기).
        p("center_x", -29.6); p("rear_axle_z", -1.93); p("front_axle_z", 1.66)
        # 진입/운반 속도 — 재시작 없이 `ros2 param set`(또는 speed_control_ui.py)으로 조절.
        p("ingress_speed", INGRESS_SPEED); p("carry_speed", CARRY_SPEED)
        p("slot_entry_speed", SLOT_ENTRY_SPEED); p("travel_speed", TRAVEL_SPEED)
        # 목표 슬롯이 요구하는 축(mod pi, 코/꼬리 무관) — parking_map.yaml의
        # slot_axis_rad와 같은 값. 이 데모 지도는 통로=X축, 슬롯=Z(깊이)방향이라
        # 기본이 pi/2다. 실제로는 task_dispatcher의 ExecuteParkingTask.slot_pose
        # 에서 받아와야 하지만, 이 스크립트는 아직 그 파이프라인과 안 이어져
        # 있어 파라미터로 받는다.
        p("target_axis_rad", math.pi / 2)
        # 목표 슬롯 좌표(world XZ) — 재시작 없이 재설정 가능. 기본값은 그동안 테스트해온
        # B1(marker_layout.py HANDOFF_BAY_Z 기준). 관제(isaac_parking_bridge_node)가
        # ExecuteParkingTask.slot_pose를 받으면 이 값을 덮어써서 다른 슬롯으로도
        # 갈 수 있게 한다 — world_x=slot_pose.position.x, world_z=-slot_pose.position.y
        # (parking_map.yaml y축과 world z축 부호가 반대인 걸 B1/A1로 실측 확인함).
        p("target_slot_x", SLOT_B1_X); p("target_slot_z", SLOT_B1_Z)
        g = lambda k: self.get_parameter(k).value
        self.cx = g("center_x")
        self.rear_axle = g("rear_axle_z")
        self.front_axle = g("front_axle_z")
        self.ingress_speed = g("ingress_speed")
        self.carry_speed = g("carry_speed")
        self.slot_entry_speed = g("slot_entry_speed")
        self.travel_speed = g("travel_speed")
        self.target_slot_x = g("target_slot_x")
        self.target_slot_z = g("target_slot_z")
        self.add_on_set_parameters_callback(self._on_param)
        self.target_axis_rad = float(g("target_axis_rad"))
        self.pose = {r: None for r in ROBOTS}   # (x, z, yaw)
        self.veh_x = self.veh_y = self.veh_z = self.veh_yaw = None
        self.home_pose = None   # {rid: (x, z)} — /dock_lift 시작 시점(실제 도크) 기록, 복귀용
        grp = ReentrantCallbackGroup()
        for r in ROBOTS:
            self.create_subscription(Odometry, f"/{r}/odom",
                                     lambda m, rid=r: self._odom(rid, m), 10, callback_group=grp)
        self.create_subscription(PoseStamped, "/vehicle/pose", self._veh, 10, callback_group=grp)
        self.cmd = {r: self.create_publisher(Twist, f"/{r}/cmd_vel", 10) for r in ROBOTS}
        self.arm = {r: self.create_client(SetBool, f"/{r}/arm_control", callback_group=grp)
                    for r in ROBOTS}
        self.create_service(Trigger, "/dock_lift", self._on_dock_lift, callback_group=grp)
        self.create_service(Trigger, "/dock_lift_from_staged", self._on_dock_lift_from_staged,
                            callback_group=grp)
        self.get_logger().info("dock_lift_handoff_mission 준비 — /dock_lift 대기 "
                               "(/dock_lift_from_staged 로 게이트 통과만 건너뛰고 재테스트 가능)")

    def _on_param(self, params):
        """범위 밖 값은 거부(clamp 아님) — 조용히 잘라내면 ros2 param get에는 원래
        요청값이 남아 실제 동작(clamp된 값)과 어긋나 보이는 문제가 있었다(실측)."""
        for prm in params:
            if prm.name == "ingress_speed" and not (SPEED_MIN <= prm.value <= INGRESS_SPEED_MAX):
                return SetParametersResult(
                    successful=False,
                    reason=f"ingress_speed 허용 범위 {SPEED_MIN}~{INGRESS_SPEED_MAX} (정밀 진입 구간)")
            if prm.name == "carry_speed" and not (SPEED_MIN <= prm.value <= CARRY_SPEED_MAX):
                return SetParametersResult(
                    successful=False,
                    reason=f"carry_speed 허용 범위 {SPEED_MIN}~{CARRY_SPEED_MAX}")
            if prm.name == "slot_entry_speed" and not (SPEED_MIN <= prm.value <= SLOT_ENTRY_SPEED_MAX):
                return SetParametersResult(
                    successful=False,
                    reason=f"slot_entry_speed 허용 범위 {SPEED_MIN}~{SLOT_ENTRY_SPEED_MAX} (슬롯 진입 구간)")
            if prm.name == "travel_speed" and not (SPEED_MIN <= prm.value <= TRAVEL_SPEED_MAX):
                return SetParametersResult(
                    successful=False,
                    reason=f"travel_speed 허용 범위 {SPEED_MIN}~{TRAVEL_SPEED_MAX}")
        for prm in params:
            if prm.name == "ingress_speed":
                self.ingress_speed = prm.value
            elif prm.name == "carry_speed":
                self.carry_speed = prm.value
            elif prm.name == "slot_entry_speed":
                self.slot_entry_speed = prm.value
            elif prm.name == "travel_speed":
                self.travel_speed = prm.value
            elif prm.name == "target_slot_x":
                self.target_slot_x = prm.value
            elif prm.name == "target_slot_z":
                self.target_slot_z = prm.value
        return SetParametersResult(successful=True)

    def _odom(self, rid, m):
        q = m.pose.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        self.pose[rid] = (m.pose.pose.position.x, m.pose.pose.position.z, yaw)

    def _veh(self, m):
        q = m.pose.orientation
        self.veh_x = m.pose.position.x
        self.veh_y = m.pose.position.y
        self.veh_z = m.pose.position.z
        # 지금까지는 position만 썼다 — 회전 판단은 로봇 자기 yaw 평균이 아니라
        # 차량 자체의 실측 yaw로 하는 게 더 정확하다(그립 슬립이 있어도 차가
        # 실제로 얼마나 돌았는지는 이 값이 answer). 시뮬레이션 GT라 가능한
        # 값이고, 실물에서는 천장 LiDAR로 차량 헤딩을 재는 쪽이 이 자리를 대신해야 한다.
        self.veh_yaw = math.atan2(
            2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))

    def _pub(self, rid, vx, vy=0.0, wz=0.0):
        t = Twist()
        t.linear.x, t.linear.y, t.angular.z = float(vx), float(vy), float(wz)
        self.cmd[rid].publish(t)

    def _stop_all(self):
        for r in ROBOTS:
            self._pub(r, 0.0)

    def _settle(self, secs=0.2):
        """단계 경계에서 정지 후 잠깐 멈춤 → 관성 흡수 + 동작이 또렷한 단계로 보이게.
        시연 속도용으로 단축(0.5→0.2). 사이클당 9회 호출되니 누적 효과가 크다."""
        self._stop_all()
        end = time.time() + secs
        while time.time() < end:
            self._stop_all()
            time.sleep(1.0 / CONTROL_HZ)

    def _clamp(self, v, m):
        return max(-m, min(m, v))

    def _wait_data(self, timeout=15.0):
        end = time.time() + timeout
        while time.time() < end and (any(self.pose[r] is None for r in ROBOTS)
                                     or self.veh_z is None):
            time.sleep(0.1)
        return all(self.pose[r] is not None for r in ROBOTS) and self.veh_z is not None

    def _omni_step(self, rid, tx, tz, tol=POS_TOL):
        """world (tx,tz)로 향하는 옴니 지령 한 틱 발행. tol 반경 도달 시 True.

        world 오차(ex,ez)를 로봇 body 지령(vx=forward, vy=left)으로 변환.
        odom 규약: forward_world=(cosθ,-sinθ), up=+Y 와의 외적으로 left_world=(-sinθ,-cosθ).
        역행렬: vx=ex·c-ez·s, vy=-(ex·s+ez·c). (+vy 가 world -Z 이므로 strafe 부호 반전.)"""
        x, z, yaw = self.pose[rid]
        ex, ez = tx - x, tz - z
        if math.hypot(ex, ez) < tol:
            return True
        c, s = math.cos(yaw), math.sin(yaw)
        fwd = ex * c - ez * s
        left = -(ex * s + ez * c)
        self._pub(rid, self._clamp(K_LIN * fwd, self.travel_speed),
                  self._clamp(K_STRAFE * left, self.travel_speed), 0.0)
        return False

    def _goto_xz(self, rid, tx, tz, timeout=STEP_TIMEOUT):
        """현재 yaw 유지한 채 world (tx,tz)로 옴니 이동(vx,vy). 회전 없음."""
        end = time.time() + timeout
        while time.time() < end:
            if self._omni_step(rid, tx, tz):
                break
            time.sleep(1.0 / CONTROL_HZ)
        self._pub(rid, 0.0)
        return math.hypot(tx - self.pose[rid][0], tz - self.pose[rid][1]) < POS_TOL * 2

    def _approach_parallel(self, routes, timeout=APPROACH_TIMEOUT):
        """routes: {rid: [(x,z),...]} 두 로봇을 동시에 웨이포인트 체인 따라 옴니 이동.
        중간 웨이포인트는 CORNER_TOL 반경에서 통과(정지 없이 코너를 돌아 부드럽게),
        마지막 웨이포인트만 POS_TOL 로 정밀 정지."""
        idx = {rid: 0 for rid in routes}
        end = time.time() + timeout
        while time.time() < end:
            for rid, wps in routes.items():
                if idx[rid] >= len(wps):
                    self._pub(rid, 0.0)
                    continue
                tx, tz = wps[idx[rid]]
                last = idx[rid] == len(wps) - 1
                if self._omni_step(rid, tx, tz, POS_TOL if last else CORNER_TOL):
                    idx[rid] += 1
            if all(idx[rid] >= len(routes[rid]) for rid in routes):
                break
            time.sleep(1.0 / CONTROL_HZ)
        for rid in routes:
            self._pub(rid, 0.0)
        return all(idx[rid] >= len(routes[rid]) for rid in routes)

    def _rotate_to(self, rid, target_yaw, timeout=90.0):
        """GT yaw 폐루프 회전(느림). 회전 방향 비신뢰라 작은 wz로 수렴.
        인플레이스 회전은 롤러 슬립이 커 느리므로 타임아웃 넉넉히."""
        end = time.time() + timeout
        while time.time() < end:
            yaw = self.pose[rid][2]
            e = wrap(target_yaw - yaw)
            if abs(e) < YAW_TOL:
                break
            self._pub(rid, 0.0, 0.0, self._clamp(K_YAW * e, MAX_YAW))
            time.sleep(1.0 / CONTROL_HZ)
        self._pub(rid, 0.0)
        return abs(wrap(target_yaw - self.pose[rid][2])) < YAW_TOL * 3

    def _axis_alignment_rotation(self, current_yaw, target_axis_rad):
        """차량을 슬롯 축(mod pi, 코/꼬리 무관)에 맞추는 최소 회전량(rad, signed).

        parking_control/core/pivot_rotate_controller.py의 axis_alignment_rotation과
        같은 식이다. 이 파일은 x,z 평면 규약(_omni_step의 world<->body 변환)이
        그 모듈의 x,y 규약과 달라, import로 섞어 쓰다 부호를 잘못 옮길 위험을
        피하려고 이 파일 자체 규약으로 다시 짰다 — 값은 동일하다.
        """
        diff = (target_axis_rad - current_yaw) % math.pi
        if diff > math.pi / 2:
            diff -= math.pi
        return diff

    def _rotate_car_to_axis(self, target_axis_rad, timeout=120.0):
        """차량을 든 채로 두 로봇이 차량 중심(두 로봇 중점) 기준 반대 방향으로
        피벗 회전해서 target_axis_rad(mod pi)에 맞춘다.

        판단 근거는 로봇 각자 yaw 평균이 아니라 /vehicle/pose 의 실측
        veh_yaw다 — 그립에 슬립이 있어도 차가 실제로 얼마나 돌았는지는 이
        값이 정답이다(실물에서는 이 자리를 천장 LiDAR 기반 차량 헤딩 추정이
        대신해야 한다 — GT는 시뮬레이션 검증용).

        강체 운동학: 중심에서 오프셋 r인 점의 접선속도는 이 파일의 yaw
        규약(forward=(cosθ,-sinθ))에서 rot90(r)=(r_z,-r_x)이므로
        world velocity = omega*(r_z, -r_x). 두 로봇은 중심을 사이에 두고
        반대편에 있어 이 식 하나로 선속도가 자동으로 반대 방향이 되고,
        angular.z(=omega)는 항상 같게 나온다 — pivot_rotate_controller.py와
        동일한 설계, 좌표 규약만 이 파일 것으로.
        """
        end = time.time() + timeout
        while time.time() < end:
            if self.veh_yaw is None:
                self._stop_all()
                return False
            diff = self._axis_alignment_rotation(self.veh_yaw, target_axis_rad)
            if abs(diff) < YAW_TOL:
                break
            omega = self._clamp(K_YAW * diff, MAX_YAW)
            cx = (self.pose["robot_rear"][0] + self.pose["robot_front"][0]) / 2.0
            cz = (self.pose["robot_rear"][1] + self.pose["robot_front"][1]) / 2.0
            for rid in ROBOTS:
                x, z, yaw = self.pose[rid]
                rx, rz = x - cx, z - cz
                c, s = math.cos(yaw), math.sin(yaw)
                fwd = omega * (rz * c + rx * s)
                left = omega * (rx * c - rz * s)
                self._pub(rid, self._clamp(fwd, MAX_LIN),
                         self._clamp(left, MAX_LIN), omega)
            time.sleep(1.0 / CONTROL_HZ)
        self._stop_all()
        if self.veh_yaw is None:
            return False
        return abs(self._axis_alignment_rotation(self.veh_yaw, target_axis_rad)) < YAW_TOL * 3

    def _ingress_to(self, rid, target_z, face_yaw, timeout=STEP_TIMEOUT):
        """차 밑으로 진입하며 축(target_z)에 정렬. 중심선(x=cx)과 방위(face_yaw)를
        폐루프 유지 → 진입 중 드리프트로 바퀴에 걸리는 것을 방지(실측: 개루프는 드리프트로
        축 못 미치고 걸림). 진입 방향은 target_z 부호가 알아서 결정(옴니)."""
        end = time.time() + timeout
        while time.time() < end:
            x, z, yaw = self.pose[rid]
            if abs(z - target_z) < POS_TOL and abs(x - self.cx) < POS_TOL * 2:
                break
            ex, ez = self.cx - x, target_z - z
            c, s = math.cos(yaw), math.sin(yaw)
            fwd = ex * c - ez * s
            left = -(ex * s + ez * c)
            eyaw = wrap(face_yaw - yaw)
            self._pub(rid, self._clamp(K_LIN * fwd, self.ingress_speed),
                      self._clamp(K_STRAFE * left, self.ingress_speed),  # 중심선 보정도 젠틀히
                      self._clamp(0.6 * eyaw, 0.10))   # 완만한 방위 유지
            time.sleep(1.0 / CONTROL_HZ)
        self._pub(rid, 0.0)
        return abs(self.pose[rid][1] - target_z) < POS_TOL * 3

    def _call_arms(self, opening):
        for r in ROBOTS:
            if not self.arm[r].wait_for_service(timeout_sec=5.0):
                return False
        futs = [self.arm[r].call_async(SetBool.Request(data=opening)) for r in ROBOTS]
        end = time.time() + 6.0
        while time.time() < end and not all(f.done() for f in futs):
            time.sleep(0.05)
        return all(f.done() and f.result() and f.result().success for f in futs)

    def _grip_lift(self):
        y0 = self.veh_y
        if not self._call_arms(True):
            return 0.0
        end = time.time() + 12.0
        while time.time() < end:
            time.sleep(0.05)
        return (self.veh_y - y0) if (self.veh_y is not None and y0 is not None) else 0.0

    def _omni_carry(self):
        """파지 후 오미 운반: 앞(-z) → 뒤(+z) → 옆(-x) 각 1m.

        두 로봇 다 -z 향함(FACE_MZ) → 같은 body 지령이면 같은 world 방향(편대 유지).
          forward(+vx)=world -z, back(-vx)=+z, left(+vy)=world -x (yaw=+π/2 기하)."""
        def move(vx, vy, getter, dist):
            start = getter()
            end = time.time() + STEP_TIMEOUT
            while time.time() < end:
                for r in ROBOTS:
                    self._pub(r, vx, vy, 0.0)
                time.sleep(1.0 / CONTROL_HZ)
                if start is not None and getter() is not None and abs(getter() - start) >= dist:
                    break
            self._stop_all()
            return abs(getter() - start) if (start is not None and getter() is not None) else 0.0

        S = self.carry_speed
        fwd = move(S, 0.0, lambda: self.veh_z, CARRY_DIST)      # 차량 -z (전진)
        self.get_logger().info(f"앞으로(-z) {fwd:.2f}m")
        time.sleep(0.5)
        back = move(-S, 0.0, lambda: self.veh_z, CARRY_DIST)    # 차량 +z (후진)
        self.get_logger().info(f"뒤로(+z) {back:.2f}m")
        time.sleep(0.5)
        side = move(0.0, S, lambda: self.veh_x, CARRY_DIST)     # 차량 -x (옆)
        self.get_logger().info(f"옆으로(-x) {side:.2f}m")
        return fwd, back, side

    def _carry_axis(self, mode, target, hold, speed, tol=POS_TOL, timeout=SLOT_CARRY_TIMEOUT):
        """차량의 world x(mode='x') 또는 z(mode='z')를 target 으로 맞추면서, 반대편
        축(부축)은 hold 값으로 살짝 유지한다. 부축을 그냥 속도 0으로만 두면 회전(wz)
        보정 도중 슬쩍 새서 통로 중앙(또는 슬롯 x)을 벗어나 한쪽으로 치우쳐 이동하는
        게 실측 확인됨(_ingress_to가 진입 중 중심선 x=cx를 같이 유지하는 것과 동일한
        이유·방식). 회전 보정은 차량 실측 yaw(veh_yaw) 기준 — 로봇 각자 자기 yaw를
        보면 로봇은 똑바른데 차량만 비뚤어지는 문제가 있어(두 로봇 미세 속도차) 이걸로 바꿈.
        speed는 호출부가 구간(통로 횡단 vs 슬롯 진입)에 맞는 값을 넘긴다 — 둘을 같은
        속도로 묶어두면 "직선 구간은 빠르게, 진입 구간은 느리게"가 안 돼서 분리함."""
        end = time.time() + timeout
        ok = False
        last_log = time.time()
        while time.time() < end:
            cur = self.veh_z if mode == "z" else self.veh_x
            side = self.veh_x if mode == "z" else self.veh_z
            if cur is None or side is None:
                time.sleep(1.0 / CONTROL_HZ)
                continue
            err = target - cur
            if abs(err) < tol:
                ok = True
                break
            if time.time() - last_log >= 2.0:
                self.get_logger().info(
                    f"슬롯 이동({mode}) 진행 중: 남은 거리 {abs(err):.2f}m, "
                    f"부축 편차 {side - hold:+.2f}m")
                last_log = time.time()
            axis_speed = speed if err < 0 else -speed
            side_speed = self._clamp(-SIDE_K * (hold - side), SIDE_MAX)
            veh_yaw = self.veh_yaw if self.veh_yaw is not None else FACE_MZ
            # mod-pi로 비교해야 한다 — _rotate_car_to_axis가 "코/꼬리 무관"으로
            # 180도 뒤집힌 해에 수렴할 수 있는데, 여기서 plain wrap()(360도 기준)을
            # 쓰면 그 상태를 180도짜리 진짜 오차로 착각해서 wz를 계속 최대로 걸어
            # 긴 이동거리 동안 차가 휘어져 나가는 문제가 실측 확인됨(A1에서 발견).
            eyaw = self._axis_alignment_rotation(veh_yaw, FACE_MZ)
            wz = self._clamp(0.6 * eyaw, CARRY_WZ_MAX)
            vx, vy = (axis_speed, side_speed) if mode == "z" else (side_speed, axis_speed)
            for r in ROBOTS:
                self._pub(r, vx, vy, wz)
            time.sleep(1.0 / CONTROL_HZ)
        self._stop_all()
        return ok

    def _carry_to_slot(self, slot_x, slot_z):
        """파지 후 슬롯까지 2단계 직선: 통로 중앙(z=AISLE_CENTER_Z)을 유지하며 x 정렬
        (슬롯 열까지 이동, carry_speed — 개활지라 빠르게) → 그대로 x=slot_x를 유지하며
        z로 직진해 슬롯 안으로 진입(slot_entry_speed — 옆 슬롯/기둥 근접이라 더 느리게).

        중간에 명시적 회전 정렬(_rotate_car_to_axis)을 넣어봤는데, 정렬 자체가
        오래 걸리고(실측 39초) 그 이후에도 부축 편차가 계속 커지는 문제가 있어
        도로 뺐다 — _carry_axis의 매 틱 mod-180도 보정만으로 진행한다."""
        self.get_logger().info(f"슬롯 이동: 통로 정렬(x→{slot_x:.2f}, 통로중앙 z 유지)")
        if not self._carry_axis("x", slot_x, hold=AISLE_CENTER_Z, speed=self.carry_speed):
            return False
        self._settle()
        self.get_logger().info(f"슬롯 이동: 슬롯 진입(z→{slot_z:.2f}, x={slot_x:.2f} 유지)")
        if not self._carry_axis("z", slot_z, hold=slot_x, speed=self.slot_entry_speed):
            return False
        self._settle()
        return True

    def _retreat_one(self, rid, dist):
        """로봇 한 대만 후진 이탈(world +z = body -vx, 통로/북쪽 방향)."""
        start = self.pose[rid][1]
        end = time.time() + STEP_TIMEOUT
        while time.time() < end:
            self._pub(rid, -self.ingress_speed, 0.0, 0.0)
            time.sleep(1.0 / CONTROL_HZ)
            if abs(self.pose[rid][1] - start) >= dist:
                break
        self._pub(rid, 0.0)

    def _retreat(self, dist=RETREAT_DIST):
        """파지 해제 후 두 로봇이 차체 밑에서 순차 이탈, 둘 다 통로(북쪽) 방향.
        앞축이 먼저 빠져 길을 비운 뒤 뒷축이 뒤따라 나간다 — 둘 다 동시에 같은
        방향으로 가면 더 깊이 들어가 있던 뒷축이 앞축 자리를 그대로 통과해야 해서
        부딪힌다(둘 다 같은 x 선상에 있음). 뒷축은 앞축보다 축간 거리만큼 더
        가야 차 밑을 완전히 벗어난다."""
        self.get_logger().info("이탈: 앞축 로봇 먼저(통로 방향)")
        self._retreat_one("robot_front", dist)
        self._settle()
        rear_dist = dist + (self.front_axle - self.rear_axle)
        self.get_logger().info(f"이탈: 뒷축 로봇(통로 방향, {rear_dist:.2f}m)")
        self._retreat_one("robot_rear", rear_dist)
        self._stop_all()

    def _return_to_dock(self):
        """하차 후 두 로봇을 원래 도크 위치(self.home_pose — /dock_lift 시작 시점에
        실측 기록한 실제 좌표)로 되돌리고, 원래 방위(yaw=0, +X 향함 —
        _place_robot_dock 초기값과 동일)로 되돌린다. 빈 로봇이라 편대 유지가
        필요 없어 각자 독립 이동(_approach_parallel 재사용, 로봇당 웨이포인트 1개).

        예전엔 DOCK_X/LANE_Z_* 근사값으로 갔는데, 그 지점이 실제 도크와 달라서
        두 로봇이 도크가 아니라 서로 가까운 엉뚱한 한 지점에 몰리는 문제가
        실측 확인됨 — home_pose로 바꿔서 정확히 원래 자리로 가게 고침.
        home_pose가 없으면(예: 체크포인트 테스트 경로) 예전 근사값으로 대체."""
        self.get_logger().info("도크로 복귀: 이동")
        if self.home_pose:
            home = {rid: [self.home_pose[rid]] for rid in ROBOTS}
        else:
            home = {
                "robot_rear": [(DOCK_X, LANE_Z_REAR)],
                "robot_front": [(DOCK_X, LANE_Z_FRONT)],
            }
        if not self._approach_parallel(home):
            self._stop_all()
            return False
        self._settle()
        self.get_logger().info("도크로 복귀: 방위 원위치(yaw=0)")
        for rid in ROBOTS:
            if not self._rotate_to(rid, 0.0):
                self._stop_all()
                return False
        self._settle()
        return True

    def _on_dock_lift(self, req, resp):
        if not self._wait_data():
            resp.success = False; resp.message = "데이터 미수신"; return resp
        # 실제 도크 위치 기록(복귀용) — 아직 한 발짝도 안 움직인 지금이 진짜 도크다.
        self.home_pose = {rid: self.pose[rid][:2] for rid in ROBOTS}
        # 접근(한 방향): 두 로봇이 개구부(북)쪽에서 한 줄로 -z 진입.
        # 1) 게이트 통과는 병렬(rear 남쪽 통로 -1.5, front 북쪽 통로 +1.5 로 분리 → 벽 서쪽).
        gate = {
            "robot_rear":  [(DOCK_X, LANE_Z_REAR), (WALL_CLEAR_X, LANE_Z_REAR)],
            "robot_front": [(DOCK_X, LANE_Z_FRONT), (WALL_CLEAR_X, LANE_Z_FRONT)],
        }
        self.get_logger().info("접근: 게이트 통과(병렬)")
        if not self._approach_parallel(gate):
            self._stop_all(); resp.success = False
            resp.message = "게이트 통과 타임아웃"; return resp
        self._settle()
        return self._ingress_both(resp)

    def _ingress_both(self, resp):
        """뒷축→앞축 순차 진입(회전+진입) 후 파지·운반까지. 두 로봇이 이미 게이트
        통과 직후 대기 위치(rear: cx,NORTH_STAGE_Z / front: WALL_CLEAR_X,LANE_Z_FRONT)에
        있다고 가정 — 실제 주행으로 왔든(_on_dock_lift) 체크포인트 순간이동으로 왔든
        (_on_dock_lift_from_staged) 공통으로 쓴다."""
        # rear 가 먼저 북쪽 스테이징(cx, NORTH_STAGE_Z)으로 → -z 회전 → 뒷축(-1.93)까지 깊이 진입.
        # front 는 벽 서쪽(-20,+1.5)에서 대기(rear 가 스테이징 비운 뒤 진입) → 상호 회피.
        self.get_logger().info("뒷축 로봇: 북쪽 정렬 → 진입(깊이)")
        self._goto_xz("robot_rear", self.cx, NORTH_STAGE_Z)
        self._settle()
        if not self._rotate_to("robot_rear", FACE_MZ):
            self._stop_all(); resp.success = False; resp.message = "rear 회전 실패"; return resp
        self._settle()                                  # 조준 후 잠깐 멈췄다 진입
        self._ingress_to("robot_rear", self.rear_axle, FACE_MZ, timeout=140.0)
        self._settle()
        # front 가 같은 북쪽 스테이징으로(이제 rear 는 깊이 들어가 비어 있음) → 앞축(+1.66) 진입.
        self.get_logger().info("앞축 로봇: 북쪽 정렬 → 진입")
        self._goto_xz("robot_front", self.cx, NORTH_STAGE_Z)
        self._settle()
        if not self._rotate_to("robot_front", FACE_MZ):
            self._stop_all(); resp.success = False; resp.message = "front 회전 실패"; return resp
        self._settle()
        self._ingress_to("robot_front", self.front_axle, FACE_MZ)
        self._settle()
        return self._grip_carry_release(resp)

    def _grip_carry_release(self, resp):
        """파지·리프트 → B1 운반 → 하차 → 이탈. 로봇이 이미 차 밑(축 정렬)에 있어야 한다.

        방향 정렬(_axis_alignment_rotation/_rotate_car_to_axis)은 일부러 뺐다 —
        주차(입고) 먼저 안정화하고, 회전은 나중에 출차 흐름에서 붙이기로 함
        (메서드 자체는 남겨뒀으니 그때 그대로 재사용)."""
        self.get_logger().info("파지·리프트")
        lift = self._grip_lift()
        if lift < 0.02:
            self._stop_all(); resp.success = False
            resp.message = f"리프트 실패 {lift:.4f}m"; return resp
        self.get_logger().info(
            f"리프트 {lift:.3f}m — 슬롯으로 운반(x={self.target_slot_x:.2f}, z={self.target_slot_z:.2f})")
        if not self._carry_to_slot(self.target_slot_x, self.target_slot_z):
            self._stop_all(); resp.success = False
            resp.message = "슬롯 이동 타임아웃"; return resp
        self.get_logger().info(f"도착(x={self.veh_x:.2f}, z={self.veh_z:.2f}) — 하차")
        if not self._call_arms(False):
            self._stop_all(); resp.success = False
            resp.message = "파지 해제 실패"; return resp
        self._settle(0.4)   # 하차 직후라 완전히 줄이진 않음(차량 착지 안정화 여유)
        self._retreat()
        if not self._return_to_dock():
            self._stop_all(); resp.success = False
            resp.message = "도크 복귀 타임아웃"; return resp
        resp.success = True
        resp.message = (f"완료: 리프트 {lift:.3f}m, 주차(x={self.veh_x:.2f}, z={self.veh_z:.2f}), "
                        f"도크 복귀")
        self.get_logger().info(resp.message)
        return resp

    def _on_dock_lift_from_staged(self, req, resp):
        """테스트 반복용: /sim_checkpoint_staged(러너)로 로봇을 게이트 통과 직후
        대기 위치까지 순간이동시킨 뒤 이 서비스를 호출하면, 느린 게이트 통과 구간만
        건너뛰고 회전·진입·파지·B1운반·하차는 그대로 실제 주행/동작으로 실행한다."""
        if not self._wait_data():
            resp.success = False; resp.message = "데이터 미수신"; return resp
        return self._ingress_both(resp)


def main():
    rclpy.init()
    node = HandoffMission()
    ex = MultiThreadedExecutor(num_threads=4)
    ex.add_node(node)
    try:
        ex.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
