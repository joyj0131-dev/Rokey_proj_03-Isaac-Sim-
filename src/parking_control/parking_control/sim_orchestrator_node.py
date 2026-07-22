#!/usr/bin/env python3
"""sim_orchestrator: 테스트용 가짜 로봇 (B/C의 실제 구현 전 대역).

robot_task_orchestrator와 같은 execute_parking_task 액션을 제공하지만,
실제 로봇 대신 pathfinder 경로를 따라 DB의 robots.x/y를 조금씩 옮기며
"움직이는 척"만 한다. 완료되면 parking_slots.status를 실제로
OCCUPIED/EMPTY로 바꾸므로, 대시보드(dashboard.py)와 웹 UI 양쪽에서
입고/출차가 눈에 보이는 변화로 확인된다.

작업 흐름은 사람이 읽기 좋은 7단계로 세분화했다 (ENTRY/EXIT 공통 구조):
  SEARCHING → APPROACHING → PICKED_UP → MOVING → ARRIVED
  → PARKED/UNPARKED → RETURNING(대기 장소 또는 충전 도크로 복귀) → DONE

이동하는 동안에는 robots.target_node에 지금 향하는 노드를 기록한다.
대시보드가 이 값 + 현재 좌표로 "가야 할 경로"를 실시간 계산해서 보여줄 수
있게 하기 위함이다 (닿으면 다시 비운다).

주의: parking_robot_system의 진짜 robot_task_orchestrator와 동시에
띄우지 않는다 (같은 액션 이름을 두 노드가 동시에 서비스하면 어느 쪽이
응답할지 불명확하다). 테스트할 때는 이 노드가 그 자리를 대신한다.
"""

import math
import time

import rclpy
from rclpy.action import ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from parking_robot_interfaces.action import ExecuteParkingTask
from parking_robot_interfaces.msg import TaskState

from parking_control.core.db import ParkingDB
from parking_control.core.graph import ParkingMap
from parking_control.core.pathfinder import PathFinder
from parking_control.parking_slot_manager_node import _default_map_yaml


class SimOrchestratorNode(Node):

    def __init__(self):
        super().__init__("sim_orchestrator")

        self.declare_parameter("db_host", "localhost")
        self.declare_parameter("db_user", "parking")
        self.declare_parameter("db_password", "parking1234")
        self.declare_parameter("db_name", "parking")
        self.declare_parameter("map_yaml", _default_map_yaml())
        self.declare_parameter("robot_id", "robot_1")
        self.declare_parameter("move_step_sec", 0.15)   # 좌표 갱신(DB write) 주기
        self.declare_parameter("move_speed_mps", 4.0)   # 이동 속도 (mock과 동일하게 맞춤)
        self.declare_parameter("stage_pause_sec", 0.5)  # 인식/픽업/도착 등 정지 시간

        p = self.get_parameter
        self._db = ParkingDB(
            host=p("db_host").value, user=p("db_user").value,
            password=p("db_password").value, database=p("db_name").value)
        self._map = ParkingMap.load(p("map_yaml").value)
        self._pathfinder = PathFinder(self._map)
        self._robot_id = p("robot_id").value

        self._task_state_pub = self.create_publisher(TaskState, "task_state", 10)
        self._server = ActionServer(
            self, ExecuteParkingTask, "execute_parking_task",
            execute_callback=self._execute,
            callback_group=ReentrantCallbackGroup())

        self.get_logger().info(
            f"sim_orchestrator 시작 (robot_id={self._robot_id}) — "
            "테스트 전용: 실제 로봇 대신 좌표만 흉내 내어 이동합니다")

    # ---- 이동 시뮬레이션 ----

    def _current_node(self):
        pos = self._db.get_robot_position(self._robot_id)
        if pos is None:
            return "dock_wait_A"
        return self._map.nearest_node(*pos)

    def _walk_segment(self, from_x, from_y, to_x, to_y, tick, speed):
        """한 구간을 등속 직선(수평 또는 수직)으로 이동하며 좌표를 갱신한다."""
        dist = math.hypot(to_x - from_x, to_y - from_y)
        if dist == 0:
            return
        duration = dist / speed if speed > 0 else 0.0
        steps = max(1, round(duration / tick))
        for step in range(1, steps + 1):
            ratio = step / steps
            self._db.update_robot_position(
                self._robot_id,
                from_x + (to_x - from_x) * ratio,
                from_y + (to_y - from_y) * ratio,
            )
            time.sleep(tick)

    def _move_to(self, target_node):
        """target_node까지 경로를 따라 좌표를 조금씩 갱신한다.

        웨이포인트당 고정 시간이 아니라 실제 구간 거리 ÷ 속도로 이동 시간을
        계산해서 등속으로 움직인다 — 그래야 웹 대시보드가 1.5초 간격으로
        DB를 폴링해도 중간 경로(꺾이는 지점)를 놓치지 않고 잡아낸다.
        이동 중에는 robots.target_node를 채워 대시보드가 "가야 할 경로"를
        계산할 수 있게 하고, 도착하면 비운다.

        그래프 인접 노드끼리도 좌표가 대각선으로 놓인 경우가 있는데, 실제
        로봇은 대각선으로 못 움직이므로 x를 먼저 맞추고(통로 이동) 그 다음
        y로 진입하는 두 구간으로 쪼갠다 — 주차면 진입 방식(먼저 통로를 타고
        칸 앞까지 간 뒤 직각으로 들어감)과 같은 규칙이다.
        """
        start = self._current_node()
        path = self._pathfinder.find_path(start, target_node)
        if path is None:
            self.get_logger().warn(f"경로 없음: {start} → {target_node}")
            return
        self._db.update_robot_target(self._robot_id, target_node)

        tick = self.get_parameter("move_step_sec").value
        speed = self.get_parameter("move_speed_mps").value
        current_x, current_y = path.waypoints[0]

        for next_x, next_y in path.waypoints[1:]:
            if current_x != next_x and current_y != next_y:
                self._walk_segment(current_x, current_y, next_x, current_y, tick, speed)
                self._walk_segment(next_x, current_y, next_x, next_y, tick, speed)
            else:
                self._walk_segment(current_x, current_y, next_x, next_y, tick, speed)
            current_x, current_y = next_x, next_y

        self._db.update_robot_target(self._robot_id, None)

    def _nearest_dock(self):
        """가장 가까운 로봇 대기/충전 도크와 그 역할(waiting/charging)."""
        start = self._current_node()
        best = None
        for dock in self._map.nodes_of_kind("dock"):
            path = self._pathfinder.find_path(start, dock)
            if path is not None and (best is None or path.length < best[1]):
                best = (dock, path.length)
        dock = best[0] if best else "dock_wait_A"
        role = self._map.graph.nodes[dock].get("role", "waiting")
        return dock, role

    def _publish_state(self, task_id, state, step):
        msg = TaskState()
        msg.robot_id = self._robot_id
        msg.task_id = task_id
        msg.state = state
        msg.current_step = step
        self._task_state_pub.publish(msg)
        self.get_logger().info(f"[{task_id[:8]}] {state}: {step}")

    def _pause(self):
        time.sleep(self.get_parameter("stage_pause_sec").value)

    def _return_to_dock(self, task_id):
        dock, role = self._nearest_dock()
        label = "충전 도크" if role == "charging" else "대기 장소"
        self._publish_state(task_id, "RETURNING", f"{label}로 이동 중")
        self._move_to(dock)

    # ---- 액션 콜백 ----

    def _execute(self, goal_handle):
        goal = goal_handle.request
        task_id, slot_id = goal.task_id, goal.slot_id
        result = ExecuteParkingTask.Result()

        if goal.request_type == "ENTRY":
            self._publish_state(task_id, "SEARCHING", "입고 예정 차량을 찾는 중")
            self._pause()
            self._publish_state(task_id, "APPROACHING", "차량 하부로 진입 중")
            self._move_to("entrance")
            self._pause()  # 정렬 + 리프트 (내부 동작, 별도 발행 없이 픽업완료로 묶음)
            self._publish_state(task_id, "PICKED_UP", "차량 픽업 완료")
            self._publish_state(task_id, "MOVING", f"{slot_id} 칸으로 이동 중")
            self._move_to(slot_id)
            self._pause()
            self._publish_state(task_id, "ARRIVED", "목적지에 도착")
            self._db.set_slot_status(slot_id, "OCCUPIED")
            self._publish_state(task_id, "PARKED", "차량 입고 완료")
        else:  # EXIT
            self._publish_state(task_id, "SEARCHING", "해당 차량을 찾는 중")
            self._pause()
            self._publish_state(task_id, "APPROACHING", "차량 하부로 진입 중")
            self._move_to(slot_id)
            self._pause()
            self._publish_state(task_id, "PICKED_UP", "차량 픽업 완료")
            self._publish_state(task_id, "MOVING", "출차 위치로 이동 중")
            self._move_to("entrance")
            self._pause()
            self._publish_state(task_id, "ARRIVED", "목적지에 도착")
            self._db.set_slot_status(slot_id, "EMPTY")
            self._publish_state(task_id, "UNPARKED", "차량 출차 완료")

        self._return_to_dock(task_id)
        self._publish_state(task_id, "DONE", "작업 완료")
        goal_handle.succeed()
        result.success = True
        result.message = "OK (sim)"
        return result

    def destroy_node(self):
        self._db.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SimOrchestratorNode()
    executor = MultiThreadedExecutor(num_threads=2)
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
