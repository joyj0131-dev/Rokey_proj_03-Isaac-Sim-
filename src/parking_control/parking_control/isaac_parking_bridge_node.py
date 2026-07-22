#!/usr/bin/env python3
"""isaac_parking_bridge: 관제(task_dispatcher)의 execute_parking_task 액션을 받아
Isaac Sim 쪽 실제 로봇(dock_lift_handoff_mission)을 실행시키는 다리.

관제 쪽엔 로봇을 직접 실행하는 노드가 없었다 — execute_parking_task 액션
서버가 아예 없어서(action info로 실측 확인: 클라이언트 1개, 서버 0개) 관제가
명령을 보내도 Isaac이 못 받았다. 이 노드가 그 서버 역할을 한다.

ENTRY: goal.slot_pose(맵 좌표 x,y)를 Isaac world 좌표로 변환해
dock_lift_handoff_mission의 target_slot_x/target_slot_z 파라미터로 설정한 뒤
/dock_lift(Trigger)를 호출, 끝날 때까지 기다렸다가 결과를 액션 result로 돌려준다.

좌표 변환(실측 확인, parking_map.yaml vs dock_lift_handoff_mission.py 상수 대조):
  B1: 맵(x=-11.9, y=+7.8)  ↔ world(x=-11.9, z=-7.8)
  A1: 맵(x=-11.9, y=-7.8)  ↔ world(x=-11.9, z=+7.8)
  → world_x = slot_pose.position.x,  world_z = -slot_pose.position.y
  (맵 y축과 Isaac world z축이 부호 반대. x축은 그대로.)

EXIT(출차): 아직 미구현 — dock_lift_handoff_mission 쪽에 출차 로직이 없어서
실패로 즉시 응답한다. 회전 정렬(_axis_alignment_rotation 등)은 출차 흐름에서
쓰기로 이미 코드에 남겨둔 상태(사용은 안 함).

실행: ros2 run parking_control isaac_parking_bridge_node
   (colcon build 전이면) PYTHONPATH에 src/parking_control 넣고
   python3 -m parking_control.isaac_parking_bridge_node
"""
import time

import rclpy
from rcl_interfaces.msg import Parameter as ParamMsg
from rcl_interfaces.msg import ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
from rclpy.action import ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_srvs.srv import Trigger

from parking_robot_interfaces.action import ExecuteParkingTask

MISSION_NODE = "dock_lift_handoff_mission"
DOCK_LIFT_SERVICE = "/dock_lift"
SET_PARAMS_SERVICE = f"/{MISSION_NODE}/set_parameters"


def _double_param(name, value):
    return ParamMsg(name=name, value=ParameterValue(
        type=ParameterType.PARAMETER_DOUBLE, double_value=float(value)))


class IsaacParkingBridge(Node):
    def __init__(self):
        super().__init__("isaac_parking_bridge")
        grp = ReentrantCallbackGroup()
        self._dock_lift_client = self.create_client(
            Trigger, DOCK_LIFT_SERVICE, callback_group=grp)
        self._param_client = self.create_client(
            SetParameters, SET_PARAMS_SERVICE, callback_group=grp)
        self._action_server = ActionServer(
            self, ExecuteParkingTask, "execute_parking_task",
            self._on_execute, callback_group=grp)
        self.get_logger().info(
            "isaac_parking_bridge 준비 — execute_parking_task 대기 (ENTRY만 지원)")

    def _set_target_slot(self, world_x, world_z, timeout=5.0):
        if not self._param_client.wait_for_service(timeout_sec=timeout):
            return False, f"{SET_PARAMS_SERVICE} 서비스 없음 — {MISSION_NODE} 떠 있는지 확인"
        req = SetParameters.Request(parameters=[
            _double_param("target_slot_x", world_x),
            _double_param("target_slot_z", world_z),
        ])
        future = self._param_client.call_async(req)
        end = time.time() + timeout
        while not future.done() and time.time() < end:
            time.sleep(0.05)
        if not future.done():
            return False, "파라미터 설정 타임아웃"
        results = future.result().results
        if not all(r.successful for r in results):
            reasons = "; ".join(r.reason for r in results if not r.successful)
            return False, f"파라미터 거부됨: {reasons}"
        return True, ""

    def _on_execute(self, goal_handle):
        goal = goal_handle.request
        result = ExecuteParkingTask.Result()

        if goal.request_type != "ENTRY":
            self.get_logger().warn(f"request_type={goal.request_type} 미구현")
            goal_handle.abort()
            result.success = False
            result.message = f"request_type={goal.request_type} 미구현(ENTRY만 지원) — 출차는 추후 작업"
            return result

        world_x = goal.slot_pose.position.x
        world_z = -goal.slot_pose.position.y
        self.get_logger().info(
            f"ENTRY 수신: task_id={goal.task_id} slot_id={goal.slot_id} "
            f"맵(x={world_x:.2f}, y={goal.slot_pose.position.y:.2f}) "
            f"→ world(x={world_x:.2f}, z={world_z:.2f})")

        ok, err = self._set_target_slot(world_x, world_z)
        if not ok:
            self.get_logger().error(f"목표 슬롯 설정 실패: {err}")
            goal_handle.abort()
            result.success = False
            result.message = f"목표 슬롯 설정 실패: {err}"
            return result

        if not self._dock_lift_client.wait_for_service(timeout_sec=5.0):
            goal_handle.abort()
            result.success = False
            result.message = f"{DOCK_LIFT_SERVICE} 서비스 없음 — {MISSION_NODE} 떠 있는지 확인"
            return result

        self.get_logger().info(f"{DOCK_LIFT_SERVICE} 트리거 (slot_id={goal.slot_id})")
        future = self._dock_lift_client.call_async(Trigger.Request())
        while not future.done():
            time.sleep(0.1)
        dock_resp = future.result()

        goal_handle.succeed()
        result.success = bool(dock_resp and dock_resp.success)
        result.message = dock_resp.message if dock_resp else f"{DOCK_LIFT_SERVICE} 응답 없음"
        self.get_logger().info(f"완료: success={result.success} message={result.message}")
        return result


def main():
    rclpy.init()
    node = IsaacParkingBridge()
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
