#!/usr/bin/env python3
"""인계장 도킹·리프트·오미 운반 오케스트레이터 (외부 ROS2 Humble).

/dock_lift(Trigger) 한 번에:
  1. 접근: 두 로봇이 도크에서 Coupe 앞(+z 아일 쪽) 정렬 위치로 옴니 이동(회전 없이)
  2. 뒷축 로봇 회전(느리게, GT 감시) → 차 밑으로 진입 → 뒷축 정지
  3. 앞축 로봇 회전 → 진입 → 앞축 정지 (순차)
  4. 파지·리프트
  5. 오미 운반: 앞 → 뒤 → 옆(strafe) 순차

좌표: world XZ, 차 길이축=z, Coupe center_x≈-29.6. 로봇은 도크에서 +X 향함(yaw≈0),
차 밑 진입 방향은 -z(yaw=+pi/2). 회전은 GT yaw 폐루프(느림)로 안정화.
"""
import math
import sys
import time

import rclpy
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
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
K_LIN, MAX_LIN = 0.8, 0.6
K_STRAFE = 0.8
K_YAW, MAX_YAW = 0.5, 0.15   # 회전은 느리게(Plan 2: wz<=0.15 결정적, 발레 방지)
INGRESS_SPEED = 0.30
CARRY_SPEED = 0.30
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


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


class HandoffMission(Node):
    def __init__(self):
        super().__init__("dock_lift_handoff_mission")
        p = self.declare_parameter
        # Pickup 인계장 중앙(z=0). fab wheel offset: front=+1.66, rear=-1.93(휠베이스 3.59m).
        # 러너 DOCK_STAGE_READY 의 front_z/rear_z 와 일치해야 함(불일치 시 파라미터로 덮기).
        p("center_x", -29.6); p("rear_axle_z", -1.93); p("front_axle_z", 1.66)
        g = lambda k: self.get_parameter(k).value
        self.cx = g("center_x")
        self.rear_axle = g("rear_axle_z")
        self.front_axle = g("front_axle_z")
        self.pose = {r: None for r in ROBOTS}   # (x, z, yaw)
        self.veh_x = self.veh_y = self.veh_z = None
        grp = ReentrantCallbackGroup()
        for r in ROBOTS:
            self.create_subscription(Odometry, f"/{r}/odom",
                                     lambda m, rid=r: self._odom(rid, m), 10, callback_group=grp)
        self.create_subscription(PoseStamped, "/vehicle/pose", self._veh, 10, callback_group=grp)
        self.cmd = {r: self.create_publisher(Twist, f"/{r}/cmd_vel", 10) for r in ROBOTS}
        self.arm = {r: self.create_client(SetBool, f"/{r}/arm_control", callback_group=grp)
                    for r in ROBOTS}
        self.create_service(Trigger, "/dock_lift", self._on_dock_lift, callback_group=grp)
        self.get_logger().info("dock_lift_handoff_mission 준비 — /dock_lift 대기")

    def _odom(self, rid, m):
        q = m.pose.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        self.pose[rid] = (m.pose.pose.position.x, m.pose.pose.position.z, yaw)

    def _veh(self, m):
        self.veh_x = m.pose.position.x
        self.veh_y = m.pose.position.y
        self.veh_z = m.pose.position.z

    def _pub(self, rid, vx, vy=0.0, wz=0.0):
        t = Twist()
        t.linear.x, t.linear.y, t.angular.z = float(vx), float(vy), float(wz)
        self.cmd[rid].publish(t)

    def _stop_all(self):
        for r in ROBOTS:
            self._pub(r, 0.0)

    def _settle(self, secs=0.5):
        """단계 경계에서 정지 후 잠깐 멈춤 → 관성 흡수 + 동작이 또렷한 단계로 보이게."""
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
        self._pub(rid, self._clamp(K_LIN * fwd, MAX_LIN),
                  self._clamp(K_STRAFE * left, MAX_LIN), 0.0)
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
            self._pub(rid, self._clamp(K_LIN * fwd, INGRESS_SPEED),
                      self._clamp(K_STRAFE * left, INGRESS_SPEED),  # 중심선 보정도 젠틀히
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

        S = CARRY_SPEED
        fwd = move(S, 0.0, lambda: self.veh_z, CARRY_DIST)      # 차량 -z (전진)
        self.get_logger().info(f"앞으로(-z) {fwd:.2f}m")
        time.sleep(0.5)
        back = move(-S, 0.0, lambda: self.veh_z, CARRY_DIST)    # 차량 +z (후진)
        self.get_logger().info(f"뒤로(+z) {back:.2f}m")
        time.sleep(0.5)
        side = move(0.0, S, lambda: self.veh_x, CARRY_DIST)     # 차량 -x (옆)
        self.get_logger().info(f"옆으로(-x) {side:.2f}m")
        return fwd, back, side

    def _on_dock_lift(self, req, resp):
        if not self._wait_data():
            resp.success = False; resp.message = "데이터 미수신"; return resp
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
        # 2) rear 가 먼저 북쪽 스테이징(cx, NORTH_STAGE_Z)으로 → -z 회전 → 뒷축(-1.93)까지 깊이 진입.
        #    front 는 벽 서쪽(-20,+1.5)에서 대기(rear 가 스테이징 비운 뒤 진입) → 상호 회피.
        self.get_logger().info("뒷축 로봇: 북쪽 정렬 → 진입(깊이)")
        self._goto_xz("robot_rear", self.cx, NORTH_STAGE_Z)
        self._settle()
        if not self._rotate_to("robot_rear", FACE_MZ):
            self._stop_all(); resp.success = False; resp.message = "rear 회전 실패"; return resp
        self._settle()                                  # 조준 후 잠깐 멈췄다 진입
        self._ingress_to("robot_rear", self.rear_axle, FACE_MZ, timeout=140.0)
        self._settle()
        # 3) front 가 같은 북쪽 스테이징으로(이제 rear 는 깊이 들어가 비어 있음) → 앞축(+1.66) 진입.
        self.get_logger().info("앞축 로봇: 북쪽 정렬 → 진입")
        self._goto_xz("robot_front", self.cx, NORTH_STAGE_Z)
        self._settle()
        if not self._rotate_to("robot_front", FACE_MZ):
            self._stop_all(); resp.success = False; resp.message = "front 회전 실패"; return resp
        self._settle()
        self._ingress_to("robot_front", self.front_axle, FACE_MZ)
        self._settle()
        self.get_logger().info("파지·리프트")
        lift = self._grip_lift()
        if lift < 0.02:
            self._stop_all(); resp.success = False
            resp.message = f"리프트 실패 {lift:.4f}m"; return resp
        self.get_logger().info(f"리프트 {lift:.3f}m — 오미 운반")
        fwd, back, side = self._omni_carry()
        resp.success = True
        resp.message = (f"완료: 리프트 {lift:.3f}m, 앞 {fwd:.2f}m, 뒤 {back:.2f}m, 옆 {side:.2f}m")
        self.get_logger().info(resp.message)
        return resp


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
