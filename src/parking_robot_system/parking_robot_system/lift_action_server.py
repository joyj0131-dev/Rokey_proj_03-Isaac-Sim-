#!/usr/bin/env python3
"""lift_action_server (리프트): 리프트 액션 서버.

control_lift(액션 서버): 원본 isaacpjt/Isaac_envo/dock_lift_handoff_mission.py의
HandoffMission._call_arms(L207-215)를 로직 변경 없이 이식. UP -> SetBool(data=True),
DOWN -> SetBool(data=False)를 robot_rear/robot_front 양쪽 /{robot}/arm_control
(std_srvs/SetBool)에 비동기 호출하고, 둘 다 서비스 기동+응답 success 여야 전체 성공.

서비스/액션 콜백 안에서 rclpy.spin_until_future_complete로 재진입 spin하면 콜백과
클라이언트가 같은 콜백그룹·실행자를 공유할 때 클라이언트 응답 콜백이 영원히 스케줄되지
않는 데드락이 난다(Task 7에서 발견된 Critical 결함과 동일 원인). 이를 피하기 위해
Task 7/8에서 확립된 교정 패턴을 그대로 따른다:
  - 액션서버 + 두 arm_control 클라이언트를 모두 같은 ReentrantCallbackGroup에 배치.
  - future 대기는 spin_until_future_complete 대신 time.monotonic() 데드라인 +
    sleep(0.02) non-respin 폴링.
  - main()은 rclpy.spin 대신 MultiThreadedExecutor(num_threads=4)로 스핀.
"""
import time

import rclpy
from rclpy.action import ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_srvs.srv import SetBool

from parking_robot_interfaces.action import ControlLift

ROBOTS = ("robot_rear", "robot_front")
ARM_SERVICE_WAIT_TIMEOUT = 5.0   # 원본 _call_arms의 wait_for_service(timeout_sec=5.0) 그대로
ARM_CALL_DEADLINE = 6.0          # 원본 _call_arms의 future 대기 상한(6.0s) 그대로

# --- UP(파지·리프트) 물리 완료 대기용 (사용자 보고: 팔이 다 올라오기 전에 운반이 시작됨) ---
# arm_control 서비스는 runner에서 "팔 목표만 설정"하고 즉시 반환한다(팔은 이후 틱마다 램프
# 업). 그래서 _call_arms 성공 != 리프트 완료다. 원본 미션 _grip_lift는 arm 지령 후 12초를
# 기다려 /vehicle/pose Y 상승을 측정했는데, Task 10a 이식 때 이 대기가 빠졌다. 여기서 복원한다.
LIFT_RISE_MIN = 0.015     # m — 차량 Y가 이만큼 오르면 "들림"으로 판정
LIFT_WAIT_TIMEOUT = 15.0  # s — 물리 리프트 완료 대기 상한
LIFT_SETTLE = 2.0         # s — 상승 감지 후 안정 확인(이 시간 유지되면 완료)
DOWN_SETTLE = 4.0         # s — DOWN(해제) 후 차량 안착 대기(veh_y 관측 불가 시 폴백용)
ARM_FOLD_TIME = 6.0       # s — 차량 안착 '후' 팔이 완전히 접힐 때까지 추가 대기.
# 사용자 요구 순서: ① 차량 완전 안착(veh_y 멈춤) → ② 팔 완전 접힘(이 시간) → ③ 복귀.
# arm_control은 목표만 설정하고 즉시 반환하며 팔은 이후 틱마다 램프로 접히므로, 팔이 차
# 밑에서 완전히 빠질 시간을 명시적으로 확보해야 복귀가 차를 안 건드린다. (넉넉히 6s.)


class LiftActionServerNode(Node):

    def __init__(self):
        super().__init__('lift_action_server')

        grp = ReentrantCallbackGroup()
        self.arm = {r: self.create_client(SetBool, f'/{r}/arm_control', callback_group=grp)
                    for r in ROBOTS}

        # 차량 높이(Y) 관찰용 — 물리 리프트 완료 판정에 사용(같은 ReentrantCallbackGroup이라
        # 블로킹 대기 도중에도 다른 스레드가 이 콜백을 돌려 self.veh_y를 갱신한다).
        self.veh_y = None
        self.create_subscription(
            PoseStamped, '/vehicle/pose', self._veh, 10, callback_group=grp)

        self._action_server = ActionServer(
            self, ControlLift, 'control_lift', self._on_control_lift, callback_group=grp)

        self.get_logger().info('lift_action_server started')

    def _veh(self, m):
        self.veh_y = m.pose.position.y

    def _wait_lift_complete(self):
        """arm UP 지령 후 /vehicle/pose Y가 실제로 상승·안정될 때까지 대기.
        이게 있어야 orchestrator가 '차가 다 들린 뒤'에 운반을 시작한다(사용자 요구:
        완벽 정렬 → 바퀴 다 리프트 → 그다음 이동). 상승 미검출 시 False(리프트 실패)."""
        t0 = time.monotonic()
        while self.veh_y is None and time.monotonic() - t0 < 3.0:
            time.sleep(0.05)
        if self.veh_y is None:
            return False
        y0 = self.veh_y
        deadline = time.monotonic() + LIFT_WAIT_TIMEOUT
        risen_at = None
        while time.monotonic() < deadline:
            if self.veh_y is not None and (self.veh_y - y0) >= LIFT_RISE_MIN:
                if risen_at is None:
                    risen_at = time.monotonic()
                elif time.monotonic() - risen_at >= LIFT_SETTLE:
                    return True   # 충분히 상승 + 안정 유지 → 리프트 완료
            time.sleep(0.05)
        return self.veh_y is not None and (self.veh_y - y0) >= LIFT_RISE_MIN

    @staticmethod
    def _wait(secs):
        end = time.monotonic() + secs
        while time.monotonic() < end:
            time.sleep(0.05)

    def _wait_settled(self, timeout=12.0, stable_for=1.5, eps=0.004, min_time=1.0):
        """DOWN 후 차량이 완전히 안착할 때까지(=/vehicle/pose Y가 stable_for초 동안 eps 이내로
        유지) 대기한다. 팔 접힘 대기는 호출부에서 별도(ARM_FOLD_TIME)로 처리한다.
        Y 관측 불가 시 max(DOWN_SETTLE, min_time) 고정 대기로 폴백."""
        t0 = time.monotonic()
        while self.veh_y is None and time.monotonic() - t0 < 3.0:
            time.sleep(0.05)
        start = time.monotonic()
        if self.veh_y is None:
            self._wait(max(DOWN_SETTLE, min_time))
            return
        last = self.veh_y
        stable_since = time.monotonic()
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            cur = self.veh_y
            if cur is not None and abs(cur - last) > eps:
                last = cur
                stable_since = time.monotonic()
            elif (time.monotonic() - stable_since >= stable_for
                  and time.monotonic() - start >= min_time):
                return   # 안착 + 팔 완전 접힘 보장 시간 경과
            time.sleep(0.05)

    def _call_arms(self, opening):
        """원본 HandoffMission._call_arms 이식(로직 동일, 대기만 non-respin으로 교정).

        robot_rear/robot_front 양쪽 arm_control(SetBool)에 opening을 비동기 호출.
        두 서비스 모두 기동 확인 + 두 응답 모두 success=True 여야 전체 True.
        """
        for r in ROBOTS:
            if not self.arm[r].wait_for_service(timeout_sec=ARM_SERVICE_WAIT_TIMEOUT):
                return False
        futs = [self.arm[r].call_async(SetBool.Request(data=opening)) for r in ROBOTS]
        deadline = time.monotonic() + ARM_CALL_DEADLINE
        while time.monotonic() < deadline and not all(f.done() for f in futs):
            time.sleep(0.02)
        return all(f.done() and f.result() and f.result().success for f in futs)

    def _on_control_lift(self, goal_handle):
        # command: "UP" -> arm_control(True)(파지/지지), 그 외("DOWN" 등) -> arm_control(False)(해제)
        opening = goal_handle.request.command == 'UP'
        ok = self._call_arms(opening)
        if ok and opening:
            # 팔 지령만으론 아직 안 들렸다 — 차량이 실제로 올라와 안정될 때까지 대기.
            ok = self._wait_lift_complete()
        elif ok and not opening:
            # 사용자 요구 순서: ① 차량 완전 안착 → ② 팔 완전 접힘 → (반환 후 orchestrator가) ③ 복귀.
            self._wait_settled()        # ① 차량이 바닥에 완전히 내려앉을 때까지(veh_y 멈춤)
            self._wait(ARM_FOLD_TIME)   # ② 팔이 차 밑에서 완전히 접혀 빠질 때까지
        result = ControlLift.Result()
        result.success = ok
        result.support_state = 'SUPPORTED' if (opening and ok) else 'RELEASED'
        goal_handle.succeed()
        return result


def main(args=None):
    rclpy.init(args=args)
    node = LiftActionServerNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
