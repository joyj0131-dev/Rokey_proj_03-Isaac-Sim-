#!/usr/bin/env python3
"""ROS2 촉발 도킹·리프트·운반 오케스트레이터 (외부 ROS2 Humble).

/dock_lift(Trigger) 요청 하나로 순차 진입 → 파지 → 운반. Isaac 러너의
cmd_vel/odom/arm_control 만 쓴다. 축 좌표는 파라미터(러너 DOCK_STAGE_READY 값).

주의: 서비스 콜백 안에서 오래 도는 시퀀스가 odom(구독) 데이터를 읽어야 하므로
MultiThreadedExecutor + ReentrantCallbackGroup 을 쓴다. 단일 스레드에서 콜백 안
spin_once 는 구독 콜백을 못 돌린다(self.z 가 안 갱신됨 — 실측).
"""
import sys
import time

import rclpy
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_srvs.srv import Trigger, SetBool

sys.path.insert(0, "/home/rokey/cobot3_ws/src/parkbot_aruco")
from parkbot_aruco.dock_lift_state import DockLiftPlan, CARRY_SPEED

ROBOTS = ("robot_rear", "robot_front")
CONTROL_HZ = 20.0
STEP_TIMEOUT = 60.0


class DockLiftMission(Node):
    def __init__(self):
        super().__init__("dock_lift_mission")
        self.declare_parameter("rear_target_z", -1.36)
        self.declare_parameter("front_target_z", 1.36)
        self.declare_parameter("center_x", 0.0)
        g = lambda k: self.get_parameter(k).value
        self.plan = DockLiftPlan(g("rear_target_z"), g("front_target_z"),
                                 g("center_x"), carry_distance=1.0)
        self.z = {r: None for r in ROBOTS}
        self.veh_y = None   # 차량 세로 높이(리프트)
        self.veh_z = None   # 차량 운반 진행축
        grp = ReentrantCallbackGroup()
        for r in ROBOTS:
            self.create_subscription(Odometry, f"/{r}/odom",
                                     lambda m, rid=r: self._odom(rid, m), 10,
                                     callback_group=grp)
        self.create_subscription(PoseStamped, "/vehicle/pose", self._veh, 10,
                                 callback_group=grp)
        self.cmd = {r: self.create_publisher(Twist, f"/{r}/cmd_vel", 10) for r in ROBOTS}
        self.arm = {r: self.create_client(SetBool, f"/{r}/arm_control",
                                          callback_group=grp) for r in ROBOTS}
        self.create_service(Trigger, "/dock_lift", self._on_dock_lift, callback_group=grp)
        self.get_logger().info("dock_lift_mission 준비 — /dock_lift 대기")

    def _odom(self, rid, m):
        self.z[rid] = m.pose.pose.position.z

    def _veh(self, m):
        self.veh_y = m.pose.position.y   # world Y = 세로 높이(리프트)
        self.veh_z = m.pose.position.z   # world Z = 운반 진행축

    def _call_arms(self, opening):
        for r in ROBOTS:
            if not self.arm[r].wait_for_service(timeout_sec=5.0):
                return False
        futs = []
        for r in ROBOTS:
            req = SetBool.Request(); req.data = opening
            futs.append(self.arm[r].call_async(req))
        end = time.time() + 6.0
        while time.time() < end and not all(f.done() for f in futs):
            time.sleep(0.05)   # 응답은 다른 스레드가 채움
        return all(f.done() and f.result() and f.result().success for f in futs)

    def _grip_and_check(self):
        """팔 전개 서비스 호출 후, 팔 램프+리프트가 일어날 시간을 준 뒤 상승량 측정."""
        y0 = self.veh_y
        if not self._call_arms(True):
            return 0.0
        end = time.time() + 12.0   # 램프(0.02/틱)+정착 대기
        while time.time() < end:
            time.sleep(0.05)
        return (self.veh_y - y0) if (self.veh_y is not None and y0 is not None) else 0.0

    def _carry(self):
        """편대 직진 운반: 두 로봇이 차량을 world +Z 로 밀어 이동.
        rear facing +1 -> forward=+Z, front facing -1 -> forward=-Z 이므로
        차량을 +Z 로 밀려면 rear 는 forward(+), front 는 backward(-)."""
        z0 = self.veh_z
        end = time.time() + STEP_TIMEOUT
        while time.time() < end:
            self._pub("robot_rear", CARRY_SPEED)
            self._pub("robot_front", -CARRY_SPEED)
            time.sleep(1.0 / CONTROL_HZ)
            carried = (self.veh_z - z0) if (self.veh_z is not None and z0 is not None) else 0.0
            if carried >= self.plan.carry_distance:
                break
        self._stop_all()
        return (self.veh_z - z0) if (self.veh_z is not None and z0 is not None) else 0.0

    def _pub(self, rid, vx):
        t = Twist(); t.linear.x = float(vx)   # 로컬 forward; 러너가 메카넘 IK 처리
        self.cmd[rid].publish(t)

    def _stop_all(self):
        for r in ROBOTS:
            self._pub(r, 0.0)

    def _wait_odom(self, timeout=15.0):
        end = time.time() + timeout
        while time.time() < end and any(self.z[r] is None for r in ROBOTS):
            time.sleep(0.1)   # odom 은 다른 스레드가 갱신
        return all(self.z[r] is not None for r in ROBOTS)

    def _run_ingress(self):
        """순차 진입: ingress_rear -> ingress_front. 상태기계로 명령 결정."""
        phase = "ingress_rear"
        end = time.time() + STEP_TIMEOUT
        last_log = 0.0
        while phase in ("ingress_rear", "ingress_front") and time.time() < end:
            cmd = self.plan.ingress_cmd(phase, self.z["robot_rear"], self.z["robot_front"])
            for r in ROBOTS:
                self._pub(r, cmd[r])
            time.sleep(1.0 / CONTROL_HZ)
            phase = self.plan.next_phase(phase, self.z["robot_rear"],
                                         self.z["robot_front"], 0.0, 0.0)
            if time.time() - last_log > 1.0:
                self.get_logger().info(
                    f"[{phase}] rear_z={self.z['robot_rear']:.2f} "
                    f"front_z={self.z['robot_front']:.2f}")
                last_log = time.time()
        self._stop_all()
        return phase == "grip"

    def _on_dock_lift(self, req, resp):
        if not self._wait_odom():
            resp.success = False
            resp.message = "odom 미수신"
            return resp
        self.get_logger().info("순차 진입 시작")
        if not self._run_ingress():
            self._stop_all()
            resp.success = False
            resp.message = "진입 타임아웃"
            return resp
        self.get_logger().info("파지·리프트 시작")
        car_lift = self._grip_and_check()
        if self.plan.next_phase("grip", self.z["robot_rear"], self.z["robot_front"],
                                car_lift, 0.0) != "carry":
            self._stop_all()
            resp.success = False
            resp.message = f"리프트 실패 car_lift={car_lift:.4f}m"
            self.get_logger().warn(resp.message)
            return resp
        self.get_logger().info(f"리프트 {car_lift:.3f}m — 운반 시작")
        carried = self._carry()
        resp.success = carried >= self.plan.carry_distance * 0.8
        resp.message = f"완료: 리프트 {car_lift:.3f}m, 운반 {carried:.3f}m"
        self.get_logger().info(resp.message)
        return resp


def main():
    rclpy.init()
    node = DockLiftMission()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
