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
from std_srvs.srv import SetBool

from parking_robot_interfaces.action import ControlLift

ROBOTS = ("robot_rear", "robot_front")
ARM_SERVICE_WAIT_TIMEOUT = 5.0   # 원본 _call_arms의 wait_for_service(timeout_sec=5.0) 그대로
ARM_CALL_DEADLINE = 6.0          # 원본 _call_arms의 future 대기 상한(6.0s) 그대로


class LiftActionServerNode(Node):

    def __init__(self):
        super().__init__('lift_action_server')

        grp = ReentrantCallbackGroup()
        self.arm = {r: self.create_client(SetBool, f'/{r}/arm_control', callback_group=grp)
                    for r in ROBOTS}

        self._action_server = ActionServer(
            self, ControlLift, 'control_lift', self._on_control_lift, callback_group=grp)

        self.get_logger().info('lift_action_server started')

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
