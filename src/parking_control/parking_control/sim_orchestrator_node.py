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
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from parking_robot_interfaces.action import ExecuteParkingTask
from parking_robot_interfaces.msg import FormationStop, SafetyState, TaskState

from parking_control.core.db import ParkingDB
from parking_control.core.graph import ParkingMap
from parking_control.core.pathfinder import PathFinder
from parking_control.parking_slot_manager_node import _default_map_yaml


# 실제 formation_gap_controller의 기본 편대 간격과 맞춘다. UI에서도 92px
# 로봇 카드 두 장이 한 아이콘처럼 겹치지 않고 리더/팔로워가 구분된다.
SIM_FORMATION_GAP_M = 2.9


class EmergencyStopTriggered(RuntimeError):
    """관제탑 비상정지로 테스트 이동을 즉시 중단한다."""


def formation_positions(previous_center, center, robot_count=2,
                        gap_m=SIM_FORMATION_GAP_M):
    """경로 진행축을 따라 리더/팔로워의 화면용 편대 좌표를 계산한다.

    첫 번째 좌표가 리더(진행 방향 앞), 두 번째가 팔로워(뒤)다. 테스트용
    시뮬레이터이므로 실제 제어기를 흉내 내지는 않지만, 두 로봇이 같은 차량을
    운반하는 편대로 함께 이동하는 모습은 유지한다.
    """
    if robot_count <= 0:
        return []
    if robot_count == 1:
        return [center]

    dx = center[0] - previous_center[0]
    dy = center[1] - previous_center[1]
    length = math.hypot(dx, dy)
    if length < 1e-9:
        dx, dy, length = 1.0, 0.0, 1.0
    ux, uy = dx / length, dy / length
    start_offset = gap_m * (robot_count - 1) / 2.0
    return [
        (
            center[0] + ux * (start_offset - index * gap_m),
            center[1] + uy * (start_offset - index * gap_m),
        )
        for index in range(robot_count)
    ]


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
        self.declare_parameter("stage_pause_sec", 0.5)  # 인식/픽업/도착 등 정지 시간

        p = self.get_parameter
        self._db = ParkingDB(
            host=p("db_host").value, user=p("db_user").value,
            password=p("db_password").value, database=p("db_name").value)
        self._map = ParkingMap.load(p("map_yaml").value)
        self._pathfinder = PathFinder(self._map)
        self._robot_id = p("robot_id").value
        self._emergency_stop = True
        self._operation_cancelled = True
        self._safety_state = "UNKNOWN"
        self._last_safety_state_at = None

        self._task_state_pub = self.create_publisher(TaskState, "task_state", 10)
        self.create_subscription(
            FormationStop, "/formation_stop", self._on_emergency_stop, 10
        )
        safety_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(
            SafetyState, "/safety/state", self._on_safety_state, safety_qos
        )
        self._server = ActionServer(
            self, ExecuteParkingTask, "execute_parking_task",
            execute_callback=self._execute,
            callback_group=ReentrantCallbackGroup())

        self.get_logger().info(
            f"sim_orchestrator 시작 (robot_id={self._robot_id}) — "
            "테스트 전용: 실제 로봇 대신 좌표만 흉내 내어 이동합니다")

    # ---- 이동 시뮬레이션 ----

    def _on_emergency_stop(self, msg):
        if msg.stop and msg.source_robot_id == "control_tower":
            self._emergency_stop = True
            self._operation_cancelled = True
            self.get_logger().error(
                f"관제탑 비상정지 수신: {msg.reason or '사유 없음'}"
            )

    def _on_safety_state(self, msg):
        previous = self._safety_state
        self._safety_state = msg.state
        self._last_safety_state_at = time.monotonic()
        self._emergency_stop = not msg.motion_allowed
        if self._emergency_stop:
            self._operation_cancelled = True
        if previous != msg.state:
            self.get_logger().info(
                f"중앙 안전 상태={msg.state}, motion_allowed={msg.motion_allowed}"
            )

    def _raise_if_emergency_stopped(self):
        heartbeat_stale = (
            self._last_safety_state_at is None
            or time.monotonic() - self._last_safety_state_at > 3.0
        )
        if self._emergency_stop or self._operation_cancelled or heartbeat_stale:
            reason = (
                "중앙 안전 관리자 heartbeat 두절"
                if heartbeat_stale
                else "관제 UI 전체 비상정지"
            )
            raise EmergencyStopTriggered(reason)

    def _current_node(self, robot_id=None):
        pos = self._db.get_robot_position(robot_id or self._robot_id)
        if pos is None:
            return "dock_wait_A"
        return self._map.nearest_node(*pos)

    def _move_to(self, target_node, robot_ids):
        """target_node까지 경로를 따라 좌표를 조금씩 갱신한다.

        리더만 움직이던 기존 테스트 대역과 달리 goal에 배정된 리더/팔로워를
        경로 진행축 앞뒤의 편대로 배치하고, 한 SQL 문으로 함께 갱신한다.
        이동 중에는 두 로봇 모두 target_node를 채우고 도착하면 비운다.
        """
        robot_ids = [robot_id for robot_id in robot_ids if robot_id]
        if not robot_ids:
            return
        start = self._current_node(robot_ids[0])
        path = self._pathfinder.find_path(start, target_node)
        if path is None:
            self.get_logger().warn(f"경로 없음: {start} → {target_node}")
            return
        for robot_id in robot_ids:
            self._db.update_robot_target(robot_id, target_node)
        delay = self.get_parameter("move_step_sec").value
        previous = path.waypoints[0]
        for center in path.waypoints[1:]:
            self._raise_if_emergency_stopped()
            positions = formation_positions(
                previous, center, robot_count=len(robot_ids))
            self._db.update_robot_positions(
                (robot_id, x, y)
                for robot_id, (x, y) in zip(robot_ids, positions)
            )
            time.sleep(delay)
            previous = center
        for robot_id in robot_ids:
            self._db.update_robot_target(robot_id, None)

    def _nearest_dock(self, robot_id):
        """가장 가까운 로봇 대기/충전 도크와 그 역할(waiting/charging)."""
        start = self._current_node(robot_id)
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
        end = time.monotonic() + self.get_parameter("stage_pause_sec").value
        while time.monotonic() < end:
            self._raise_if_emergency_stopped()
            time.sleep(0.05)

    def _return_to_dock(self, task_id, robot_ids):
        dock, role = self._nearest_dock(robot_ids[0])
        label = "충전 도크" if role == "charging" else "대기 장소"
        self._publish_state(task_id, "RETURNING", f"{label}로 이동 중")
        self._move_to(dock, robot_ids)

        # 마지막에는 각 로봇을 자기 팀의 실제 도크에 정렬한다. 같은 도크
        # 중심에 두 아이콘이 겹치거나 다른 슬롯 위에 남는 것을 방지한다.
        team_docks = sorted(
            node_id
            for node_id in self._map.nodes_of_kind("dock")
            if self._map.graph.nodes[node_id].get("role") == role
        )
        if len(team_docks) >= len(robot_ids):
            self._db.update_robot_positions(
                (
                    robot_id,
                    *self._map.node_pos(dock_id),
                )
                for robot_id, dock_id in zip(robot_ids, team_docks)
            )

    # ---- 액션 콜백 ----

    def _execute(self, goal_handle):
        heartbeat_stale = (
            self._last_safety_state_at is None
            or time.monotonic() - self._last_safety_state_at > 3.0
        )
        if self._emergency_stop or heartbeat_stale:
            goal_handle.abort()
            result = ExecuteParkingTask.Result()
            result.success = False
            result.message = (
                "중앙 안전 상태로 새 작업을 시작할 수 없습니다."
            )
            return result
        # 새 goal만 이전 작업 취소 래치를 해제할 수 있다. 비상정지 뒤
        # NORMAL이 오더라도 현재 실행 중인 goal은 계속 취소 상태다.
        self._operation_cancelled = False
        try:
            return self._execute_task(goal_handle)
        except EmergencyStopTriggered as exc:
            goal = goal_handle.request
            for robot_id in {
                goal.leader_robot_id or self._robot_id,
                goal.follower_robot_id,
            }:
                if robot_id:
                    self._db.update_robot_target(robot_id, None)
            self._publish_state(goal.task_id, "FAILED", str(exc))
            goal_handle.abort()
            result = ExecuteParkingTask.Result()
            result.success = False
            result.message = str(exc)
            return result

    def _execute_task(self, goal_handle):
        goal = goal_handle.request
        task_id, slot_id = goal.task_id, goal.slot_id
        leader_id = goal.leader_robot_id or self._robot_id
        robot_ids = list(dict.fromkeys(
            robot_id
            for robot_id in (leader_id, goal.follower_robot_id)
            if robot_id
        ))
        result = ExecuteParkingTask.Result()

        if goal.request_type == "ENTRY":
            self._publish_state(task_id, "SEARCHING", "입고 예정 차량을 찾는 중")
            self._pause()
            self._publish_state(task_id, "APPROACHING", "차량 하부로 진입 중")
            self._move_to("entry_wait", robot_ids)
            self._pause()  # 정렬 + 리프트 (내부 동작, 별도 발행 없이 픽업완료로 묶음)
            self._publish_state(task_id, "PICKED_UP", "차량 픽업 완료")
            self._publish_state(task_id, "MOVING", f"{slot_id} 칸으로 이동 중")
            self._move_to(slot_id, robot_ids)
            self._pause()
            self._publish_state(task_id, "ARRIVED", "목적지에 도착")
            self._db.set_slot_status(slot_id, "OCCUPIED")
            self._publish_state(task_id, "PARKED", "차량 입고 완료")
        else:  # EXIT
            self._publish_state(task_id, "SEARCHING", "해당 차량을 찾는 중")
            self._pause()
            self._publish_state(task_id, "APPROACHING", "차량 하부로 진입 중")
            self._move_to(slot_id, robot_ids)
            self._pause()
            self._publish_state(task_id, "PICKED_UP", "차량 픽업 완료")
            self._publish_state(task_id, "MOVING", "출차 위치로 이동 중")
            self._move_to("exit_wait", robot_ids)
            self._pause()
            self._publish_state(task_id, "ARRIVED", "목적지에 도착")
            self._db.set_slot_status(slot_id, "EMPTY")
            self._publish_state(task_id, "UNPARKED", "차량 출차 완료")

        self._return_to_dock(task_id, robot_ids)
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
