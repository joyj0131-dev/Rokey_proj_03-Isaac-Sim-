#!/usr/bin/env python3
"""task_dispatcher: 작업 접수 → 로봇 선택 → 슬롯 확보 → goal 전달 → DB 기록.

제공 서비스
  - dispatch_parking_task (RequestParkingTask): 작업 접수. 접수 즉시 응답하고
    이후 파이프라인은 비동기로 진행한다 (find_empty_slot → ExecuteParkingTask).
  - acquire_zones / release_zones (AcquireZones/ReleaseZones): 존 락.
    zone_lock_mode 파라미터가 'stub'이면 무조건 승인(로봇 1대 MVP),
    'db'이면 zone_locks 테이블 INSERT 성패로 판정 (다중로봇 단계).
    요청자는 robot_id(로봇 개인, 통로 구간용) 또는 task_id(로봇 2대 팀,
    슬롯처럼 함께 점유해야 하는 zone용) 중 정확히 하나를 채운다.

로봇 선택은 Allocator 전략(allocator 파라미터: nearest | hungarian)에 위임한다.

작업이 시작되면(goal 전송 시점) formation_assignment 토픽으로 리더/팔로워
로봇 각각에게 역할·파트너를 배정해 formation_gap_controller(간격유지+공동
정지)를 활성화하고, 작업이 끝나거나 실패하면 같은 토픽으로 배정을
해제한다(각 로봇은 idle로 복귀).
"""

import uuid

import rclpy
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node

from parking_robot_interfaces.action import ExecuteParkingTask
from parking_robot_interfaces.msg import FormationAssignment
from parking_robot_interfaces.srv import AcquireZones, FindEmptySlot, \
    ReleaseZones, RequestParkingTask

from parking_control.core.allocator import (
    RobotState, TaskRequest, make_allocator, pick_follower,
)
from parking_control.core.db import ParkingDB
from parking_control.core.graph import ParkingMap
from parking_control.core.pathfinder import PathFinder
from parking_control.parking_slot_manager_node import _default_map_yaml


class TaskDispatcherNode(Node):

    def __init__(self):
        super().__init__("task_dispatcher")

        self.declare_parameter("db_host", "localhost")
        self.declare_parameter("db_user", "parking")
        self.declare_parameter("db_password", "parking1234")
        self.declare_parameter("db_name", "parking")
        self.declare_parameter("map_yaml", _default_map_yaml())
        self.declare_parameter("allocator", "nearest")
        self.declare_parameter("zone_lock_mode", "stub")   # stub | db
        self.declare_parameter("zone_retry_sec", 1.0)

        p = self.get_parameter
        self._db = ParkingDB(
            host=p("db_host").value, user=p("db_user").value,
            password=p("db_password").value, database=p("db_name").value)
        self._map = ParkingMap.load(p("map_yaml").value)
        self._pathfinder = PathFinder(self._map)
        self._allocator = make_allocator(p("allocator").value)
        self._stub_held = {}   # robot_id -> set(zone_ids), stub 모드 전용

        group = ReentrantCallbackGroup()
        self._find_slot_client = self.create_client(
            FindEmptySlot, "find_empty_slot", callback_group=group)
        self._execute_client = ActionClient(
            self, ExecuteParkingTask, "execute_parking_task",
            callback_group=group)
        self._formation_pub = self.create_publisher(
            FormationAssignment, "formation_assignment", 10)

        self.create_service(RequestParkingTask, "dispatch_parking_task",
                            self._handle_dispatch, callback_group=group)
        self.create_service(AcquireZones, "acquire_zones",
                            self._handle_acquire, callback_group=group)
        self.create_service(ReleaseZones, "release_zones",
                            self._handle_release, callback_group=group)

        self.get_logger().info(
            f"task_dispatcher 시작 (allocator={p('allocator').value}, "
            f"zone_lock_mode={p('zone_lock_mode').value})")

    # ---- 작업 접수 ----

    def _handle_dispatch(self, request, response):
        response.accepted = False
        response.task_id = ""
        if request.request_type not in ("ENTRY", "EXIT"):
            response.message = f"알 수 없는 request_type: {request.request_type}"
            return response

        # EXIT는 "빈 슬롯 찾기"가 아니라 "이 차가 지금 어느 칸에 있는지" 조회다.
        exit_slot_id = None
        if request.request_type == "EXIT":
            exit_slot_id = self._db.find_vehicle_slot(request.vehicle_id)
            if exit_slot_id is None:
                response.message = (
                    f"{request.vehicle_id}의 주차 기록을 찾을 수 없습니다 "
                    "(입고 완료된 차량만 출차할 수 있습니다)")
                return response

        robots = [RobotState(r["robot_id"], float(r["x"] or 0), float(r["y"] or 0))
                  for r in self._db.idle_robots()]
        if len(robots) < 2:
            response.message = (
                f"가용(IDLE) 로봇 2대 필요 (현재 {len(robots)}대) — "
                "차량 1대는 로봇 2대(front/rear)가 함께 옮깁니다")
            return response

        task_id = str(uuid.uuid4())
        target_node = exit_slot_id or "entrance"
        task = TaskRequest(task_id=task_id, target_node=target_node)
        assignments = self._allocator.assign(robots, [task], self._cost)
        if not assignments:
            response.message = "도달 가능한 로봇 없음"
            return response
        leader_id = assignments[0].robot_id
        leader = next(r for r in robots if r.robot_id == leader_id)
        follower = pick_follower(leader, robots)
        if follower is None:
            response.message = "팔로워로 배정할 로봇이 없음"
            return response
        follower_id = follower.robot_id

        self._db.upsert_vehicle(request.vehicle_id)
        self._db.create_task(task_id, request.request_type, request.vehicle_id)
        self._db.update_task(task_id, robot_id=leader_id,
                             follower_robot_id=follower_id)
        self._db.set_robot_status(leader_id, "BUSY")
        self._db.set_robot_status(follower_id, "BUSY")

        if exit_slot_id is not None:
            # 슬롯을 이미 알고 있으니(EXIT) find_empty_slot을 건너뛰고 바로 진행.
            self._db.update_task(task_id, slot_id=exit_slot_id, state="PROCESSING")
            x, y = self._map.node_pos(exit_slot_id)
            self._send_execute_goal(
                task_id, request, leader_id, follower_id, exit_slot_id, x, y)
        else:
            # 접수 응답은 여기서 끝. 슬롯 확보부터는 비동기 파이프라인.
            future = self._find_slot_client.call_async(FindEmptySlot.Request())
            future.add_done_callback(
                lambda f: self._on_slot_found(
                    f, task_id, request, leader_id, follower_id))

        response.accepted = True
        response.task_id = task_id
        response.message = (
            f"리더 {leader_id}(거리 {assignments[0].cost:.2f}m) / "
            f"팔로워 {follower_id} 배정")
        self.get_logger().info(f"작업 접수 {task_id[:8]}: {response.message}")
        return response

    def _publish_formation(self, task_id, leader_id, follower_id, active):
        """로봇 2대의 formation_gap_controller에게 역할/파트너를 배정(또는
        해제)한다. 각 로봇이 자기 robot_id로 필터링해서 받아간다."""
        pairs = ((leader_id, "leader", follower_id),
                (follower_id, "follower", leader_id))
        for robot_id, role, partner_id in pairs:
            self._formation_pub.publish(FormationAssignment(
                robot_id=robot_id, task_id=task_id if active else "",
                role=role if active else "", partner_robot_id=partner_id,
                active=active))

    def _cost(self, robot, task):
        start = self._map.nearest_node(robot.x, robot.y)
        path = self._pathfinder.find_path(start, task.target_node)
        return None if path is None else path.length

    def _on_slot_found(self, future, task_id, request, leader_id, follower_id):
        result = future.result()
        if result is None or not result.success:
            self._fail_task(task_id, leader_id, follower_id, "빈 슬롯 확보 실패")
            return
        self._db.update_task(task_id, slot_id=result.slot_id,
                             state="PROCESSING")
        self._send_execute_goal(
            task_id, request, leader_id, follower_id, result.slot_id,
            result.slot_pose.position.x, result.slot_pose.position.y)

    def _send_execute_goal(self, task_id, request, leader_id, follower_id,
                           slot_id, x, y):
        goal = ExecuteParkingTask.Goal()
        goal.task_id = task_id
        goal.request_type = request.request_type
        goal.vehicle_id = request.vehicle_id
        goal.slot_id = slot_id
        goal.slot_pose.position.x = float(x)
        goal.slot_pose.position.y = float(y)
        goal.slot_pose.orientation.w = 1.0
        goal.leader_robot_id = leader_id
        goal.follower_robot_id = follower_id
        self._publish_formation(task_id, leader_id, follower_id, active=True)
        self.get_logger().info(
            f"작업 {task_id[:8]}: 슬롯 {slot_id} → goal 전송 "
            f"(리더 {leader_id}, 팔로워 {follower_id})")
        send_future = self._execute_client.send_goal_async(goal)
        send_future.add_done_callback(
            lambda f: self._on_goal_response(f, task_id, leader_id, follower_id))

    def _on_goal_response(self, future, task_id, leader_id, follower_id):
        goal_handle = future.result()
        if goal_handle is None or not goal_handle.accepted:
            self._fail_task(task_id, leader_id, follower_id, "orchestrator가 goal 거부")
            return
        goal_handle.get_result_async().add_done_callback(
            lambda f: self._on_task_result(f, task_id, leader_id, follower_id))

    def _on_task_result(self, future, task_id, leader_id, follower_id):
        result = future.result().result
        state = "DONE" if result.success else "FAILED"
        self._db.update_task(task_id, state=state)
        self._db.set_robot_status(leader_id, "IDLE")
        self._db.set_robot_status(follower_id, "IDLE")
        self._publish_formation(task_id, leader_id, follower_id, active=False)
        self.get_logger().info(
            f"작업 {task_id[:8]} 종료: {state} ({result.message})")

    def _fail_task(self, task_id, leader_id, follower_id, reason):
        self._db.update_task(task_id, state="FAILED")
        self._db.set_robot_status(leader_id, "IDLE")
        self._db.set_robot_status(follower_id, "IDLE")
        self._publish_formation(task_id, leader_id, follower_id, active=False)
        self.get_logger().warn(f"작업 {task_id[:8]} 실패: {reason}")

    # ---- 존 락 ----

    def _owner(self, request):
        """robot_id(로봇 개인)/task_id(로봇 2대 팀) 중 정확히 하나를 뽑는다.
        both/neither면 (None, None)을 반환해 호출부가 거부하게 한다."""
        owner_robot = request.robot_id or None
        owner_task = request.task_id or None
        if (owner_robot is None) == (owner_task is None):
            return None, None
        return owner_robot, owner_task

    def _handle_acquire(self, request, response):
        mode = self.get_parameter("zone_lock_mode").value
        zone_ids = list(request.zone_ids)
        owner_robot, owner_task = self._owner(request)
        owner_key = owner_robot or owner_task
        if owner_key is None or zone_ids != sorted(zone_ids):
            # robot_id/task_id 둘 다(또는 둘 다 아님) 왔거나, 오름차순이 아님
            # (오름차순 규칙은 데드락 방지용)
            response.granted = False
            response.retry_after_sec = 0.0
            return response

        if mode == "stub":
            held = self._stub_held.setdefault(owner_key, set())
            held.update(zone_ids)
            response.granted = True
            response.held_zones = sorted(held)
            return response

        acquired = []
        for zone_id in zone_ids:
            if self._db.try_acquire_zone(
                    zone_id, robot_id=owner_robot, task_id=owner_task):
                acquired.append(zone_id)
            else:
                # 전부 못 잡으면 잡은 것도 되돌린다 (부분 보유 대기 = 데드락 씨앗)
                self._db.release_zones(
                    robot_id=owner_robot, task_id=owner_task, zone_ids=acquired)
                response.granted = False
                response.retry_after_sec = float(
                    self.get_parameter("zone_retry_sec").value)
                return response
        response.granted = True
        response.held_zones = zone_ids
        return response

    def _handle_release(self, request, response):
        mode = self.get_parameter("zone_lock_mode").value
        zone_ids = list(request.zone_ids)
        owner_robot, owner_task = self._owner(request)
        owner_key = owner_robot or owner_task
        if owner_key is None:
            response.success = False
            return response

        if mode == "stub":
            held = self._stub_held.setdefault(owner_key, set())
            held.difference_update(zone_ids or set(held))
        else:
            self._db.release_zones(
                robot_id=owner_robot, task_id=owner_task, zone_ids=zone_ids or None)
        response.success = True
        return response

    def destroy_node(self):
        self._db.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = TaskDispatcherNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
