#!/usr/bin/env python3
"""sim_orchestrator: 테스트용 가짜 로봇 (B/C의 실제 구현 전 대역).

robot_task_orchestrator와 같은 execute_parking_task 액션을 제공하지만,
실제 로봇 대신 pathfinder 경로를 따라 DB의 robots.x/y를 조금씩 옮기며
"움직이는 척"만 한다. 완료되면 parking_slots.status를 실제로
OCCUPIED/EMPTY로 바꾸므로, 대시보드(dashboard.py)와 웹 UI 양쪽에서
입고/출차가 눈에 보이는 변화로 확인된다.

주의: parking_robot_system의 진짜 robot_task_orchestrator와 동시에
띄우지 않는다 (같은 액션 이름을 두 노드가 동시에 서비스하면 어느 쪽이
응답할지 불명확하다). 테스트할 때는 이 노드가 그 자리를 대신한다.
"""

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
        self.declare_parameter("move_step_sec", 0.15)   # 이동 한 칸(waypoint)당 시간
        self.declare_parameter("stage_pause_sec", 0.5)  # 인식/정렬/리프트 정지 시간

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

    def _move_to(self, target_node):
        """target_node까지 경로를 따라 좌표를 조금씩 갱신한다 (대시보드에 실시간 반영)."""
        start = self._current_node()
        path = self._pathfinder.find_path(start, target_node)
        if path is None:
            self.get_logger().warn(f"경로 없음: {start} → {target_node}")
            return
        delay = self.get_parameter("move_step_sec").value
        for x, y in path.waypoints[1:]:
            self._db.update_robot_position(self._robot_id, x, y)
            time.sleep(delay)

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

    # ---- 액션 콜백 ----

    def _execute(self, goal_handle):
        goal = goal_handle.request
        task_id, slot_id = goal.task_id, goal.slot_id
        result = ExecuteParkingTask.Result()

        if goal.request_type == "ENTRY":
            self._publish_state(task_id, "DETECTING", "차량 인식 중")
            self._pause()
            self._publish_state(task_id, "NAVIGATING", "차량 위치로 이동 중")
            self._move_to("entrance")
            self._publish_state(task_id, "ALIGNING", "차량 하부로 정렬 중")
            self._pause()
            self._publish_state(task_id, "LIFTING", "차량을 들어올리는 중")
            self._pause()
            self._publish_state(task_id, "NAVIGATING", f"{slot_id} 칸으로 이동 중")
            self._move_to(slot_id)
            self._db.set_slot_status(slot_id, "OCCUPIED")
        else:  # EXIT
            self._publish_state(task_id, "NAVIGATING", f"{slot_id} 칸으로 이동 중")
            self._move_to(slot_id)
            self._publish_state(task_id, "ALIGNING", "차량 하부로 정렬 중")
            self._pause()
            self._publish_state(task_id, "LIFTING", "차량을 들어올리는 중")
            self._pause()
            self._publish_state(task_id, "NAVIGATING", "출차 위치로 이동 중")
            self._move_to("entrance")
            self._db.set_slot_status(slot_id, "EMPTY")

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
