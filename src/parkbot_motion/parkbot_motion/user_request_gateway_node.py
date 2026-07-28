#!/usr/bin/env python3
"""user_request_gateway_node — 관제 UI 입차 요청을 우리 픽업 안무 액션으로 잇는 게이트웨이.

다른 컴퓨터의 웹 UI(parking_control_mvp, git feature/UI)가 ROS_DOMAIN_ID 126 으로 보내는
입차 요청을 받아 슬롯을 배정하고, 우리 orchestrator 의 ``execute_pickup_choreography``
(ExecuteParkingTask) 액션을 띄운다. MySQL/관제탑(sim_orchestrator·task_dispatcher) 없이
UI ↔ 우리 로봇스택을 **직접** 잇는다(사용자 지시 2026-07-28).

받는 인터페이스(둘 다 서빙):
  · ``dispatch_parking_task`` (parking_robot_interfaces/srv/RequestParkingTask)
      request_type=ENTRY/EXIT, vehicle_id → accepted, task_id, message.
      UI 의 ``ros2_source.py``(PARKING_MODE=ros2, run_prs.sh)가 실제 호출하는 서비스.
      UI 는 슬롯을 안 보내므로(vehicle_id 만) 여기서 순환 배정한다.
  · ``/park_in_slot`` (parking_robot_interfaces/srv/ParkInSlot)
      slot_id → accepted, task_id, message. UI 의 ``ros2_prs_source.py``(PRS 모드)용.

동작: ENTRY → 슬롯(A1→A2→A3 순환 또는 요청 slot_id) → 액션 goal **비동기** 발행 →
즉시 (accepted, task_id) 응답(주차는 수 분이라 응답을 안 기다림). 로봇 2대가 한 팀이라
**한 번에 한 작업** — 진행 중엔 새 요청 거절. EXIT 는 출차 안무가 없어 미지원(거절).

크로스머신 주의: UI 컴퓨터와 이 컴퓨터가 같은 domain(126)+호환 DDS 여야 서비스가 보인다.
FastDDS 화이트리스트를 쓰면 **UI 머신 IP 도 화이트리스트에 포함**돼야 한다(안 그러면 discovery
는 돼도 호출이 안 붙음). nodes.launch.py 가 이 노드에도 domain/whitelist env 를 준다.
"""
import threading

import rclpy
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from parking_robot_interfaces.action import ExecuteParkingTask
from parking_robot_interfaces.srv import ParkInSlot, RequestParkingTask


def next_slot(slots, idx):
    """순환 슬롯 배정: (slot, next_idx). slots 비면 (None, idx)."""
    if not slots:
        return None, idx
    return slots[idx % len(slots)], idx + 1


class UserRequestGateway(Node):
    def __init__(self):
        super().__init__('user_request_gateway')
        self.declare_parameter('slots', ['A1', 'A2', 'A3'])   # 순환 배정 슬롯
        self.declare_parameter('leader_robot_id', 'entry_lead')
        self.declare_parameter('follower_robot_id', 'entry_follow')
        self.declare_parameter('action_name', 'execute_pickup_choreography')
        self.declare_parameter('dispatch_service', 'dispatch_parking_task')
        self.declare_parameter('park_in_slot_service', '/park_in_slot')
        self.declare_parameter('action_wait_sec', 5.0)

        gp = lambda n: self.get_parameter(n).value           # noqa: E731
        self._slots = [str(s).strip().upper() for s in gp('slots')]
        self._leader = gp('leader_robot_id')
        self._follower = gp('follower_robot_id')
        self._action_wait = float(gp('action_wait_sec'))

        self._cbg = ReentrantCallbackGroup()
        self._action = ActionClient(self, ExecuteParkingTask, gp('action_name'),
                                    callback_group=self._cbg)
        self._lock = threading.Lock()
        self._busy = None        # 진행 중 task_id 또는 None(유휴)
        self._slot_idx = 0
        self._seq = 0

        self.create_service(RequestParkingTask, gp('dispatch_service'),
                            self._on_dispatch, callback_group=self._cbg)
        self.create_service(ParkInSlot, gp('park_in_slot_service'),
                            self._on_park_in_slot, callback_group=self._cbg)
        self.get_logger().info(
            f"user_request_gateway 시작: dispatch='{gp('dispatch_service')}' "
            f"park_in_slot='{gp('park_in_slot_service')}' → 액션 '{gp('action_name')}' "
            f"(슬롯 순환 {self._slots}, 팀 {self._leader}+{self._follower})")

    # ---- 서비스 핸들러 ----

    def _on_dispatch(self, request, response):
        """UI dispatch_parking_task: ENTRY 만 처리(슬롯 순환배정), EXIT/기타 거절."""
        rt = (request.request_type or '').strip().upper()
        if rt == 'EXIT':
            response.accepted, response.message = False, '출차(EXIT)는 미지원입니다(입차만).'
            return response
        if rt != 'ENTRY':
            response.accepted, response.message = False, f'알 수 없는 request_type={rt!r}'
            return response
        response.accepted, response.task_id, response.message = self._start(
            None, request.vehicle_id or '')          # slot=None → 순환 배정
        return response

    def _on_park_in_slot(self, request, response):
        """UI /park_in_slot: 지정 slot_id 로 입차."""
        slot = (request.slot_id or '').strip().upper()
        if slot not in self._slots:
            response.accepted, response.message = False, (
                f'알 수 없는 slot_id={slot!r} (지원: {self._slots})')
            return response
        response.accepted, response.task_id, response.message = self._start(slot, '')
        return response

    # ---- 액션 발행 ----

    def _start(self, slot, vehicle_id):
        """입차 안무 액션 goal 을 **비동기** 발행. slot=None 이면 순환배정.
        반환 (accepted, task_id, message). 진행 중이면 거절."""
        with self._lock:
            if self._busy is not None:
                return False, '', f'이미 작업 중({self._busy}) — 완료 후 다시 요청하세요.'
            if slot is None:
                slot, self._slot_idx = next_slot(self._slots, self._slot_idx)
            if slot is None:
                return False, '', '배정 가능한 슬롯이 없습니다(slots 파라미터 비어있음).'
            self._seq += 1
            task_id = f'{slot}-{vehicle_id or "req"}-{self._seq}'
            self._busy = task_id                      # 래치: 다음 요청 거절

        if not self._action.wait_for_server(timeout_sec=self._action_wait):
            self._clear_busy()
            return False, '', ('execute_pickup_choreography 액션서버가 없습니다 '
                               '— 이 컴퓨터의 로봇 스택(nodes.launch.py)이 떠 있는지 확인.')

        goal = ExecuteParkingTask.Goal()
        goal.task_id = task_id
        goal.request_type = 'ENTRY'
        goal.vehicle_id = vehicle_id
        goal.slot_id = slot
        goal.leader_robot_id = self._leader
        goal.follower_robot_id = self._follower
        self._action.send_goal_async(goal).add_done_callback(self._on_goal_response)
        self.get_logger().info(f"[{task_id}] '{slot}' 입차 안무 시작 (차량 {vehicle_id!r})")
        return True, task_id, f'{slot} 입차 시작 (task {task_id})'

    def _on_goal_response(self, future):
        gh = future.result()
        if not gh.accepted:
            self.get_logger().warn('입차 안무 goal 이 orchestrator 에서 거절됨')
            self._clear_busy()
            return
        gh.get_result_async().add_done_callback(self._on_result)

    def _on_result(self, future):
        try:
            res = future.result().result
            self.get_logger().info(
                f'입차 안무 종료: success={res.success} message={res.message}')
        finally:
            self._clear_busy()

    def _clear_busy(self):
        with self._lock:
            self._busy = None


def main():
    rclpy.init()
    node = UserRequestGateway()
    ex = MultiThreadedExecutor()
    ex.add_node(node)
    try:
        ex.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()


def _demo():
    """순환 슬롯 배정 자기검증(노드 없이)."""
    slots = ['A1', 'A2', 'A3']
    idx = 0
    got = []
    for _ in range(7):
        s, idx = next_slot(slots, idx)
        got.append(s)
    assert got == ['A1', 'A2', 'A3', 'A1', 'A2', 'A3', 'A1'], got
    assert next_slot([], 5) == (None, 5)
    print('user_request_gateway _demo OK')


if __name__ == '__main__':
    import sys
    if len(sys.argv) > 1 and sys.argv[1] == 'demo':
        _demo()
    else:
        main()
