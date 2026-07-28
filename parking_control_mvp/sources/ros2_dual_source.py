"""dual 모드: 입차/출차 요청을 서로 다른 로봇 그룹으로 분리 라우팅.

전제(2026-07-28 사용자 지시):
  - 입차로봇 팀(entry_lead/entry_follow)과 출차로봇 팀(exit_lead/exit_follow)이
    서로 다른 물리 PC에서 각자의 Isaac Sim + parkbot_motion 노드 스택
    (src/parkbot_motion/launch/nodes.launch.py 계열)을 띄운다.
  - 두 PC 모두 같은 ROS_DOMAIN_ID(126)를 공유한다 — 그래서 두 그룹이 같은
    서비스 이름을 쓰면 충돌하므로, 그룹마다 다른 dispatch_service 이름을 쓴다
    (라우팅 표는 core/db.py의 robot_groups 테이블, 기본값은 entry=
    'dispatch_parking_task', exit='dispatch_parking_task_exit').
  - user_request_gateway_node.py는 이미 dispatch_service 이름을 launch
    파라미터로 받으므로, 출차 PC는 그 파라미터 값만 다르게 launch하면 된다 —
    로봇 스택 소스 코드 변경 없이 관제(이 파일)+DB만으로 라우팅이 완성된다.
  - 출차 안무 자체는 아직 없다(user_request_gateway_node가 request_type=EXIT를
    무조건 거절하도록 만들어져 있음). 그래도 이 소스는 출차 그룹의 서비스로
    정상적으로 요청을 "전달"한다 — 거절 응답이 오면 그 사유를 그대로 사용자에게
    보여준다. 즉 여기서 검증되는 것은 "올바른 Isaac Sim으로 라우팅되는가"이고,
    실제 출차 모션은 로봇 스택 쪽에 안무가 추가되는 별도 작업이다.

받는 텔레메트리(있는 그룹만): /parking_slots(JSON), /{robot_id}/odom, task_state.
현재는 입차 그룹만 실제로 이 토픽들을 발행하므로 출차 그룹은 슬롯/좌표 없이
서비스 연결 상태만 표시된다.
"""

import json
import math
import threading
from datetime import datetime

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from std_msgs.msg import String
from nav_msgs.msg import Odometry
from parking_robot_interfaces.msg import TaskState
from parking_robot_interfaces.srv import RequestParkingTask

import config
from core import db
from core.datasource import DataSource, DataSourceError
from core.models import (
    Alert, AlertCategory, AlertLevel,
    ParkingRequest, ParkingSlot, Robot, RequestStatus, RequestType,
)

_REQUEST_TYPE_TO_ROS = {RequestType.PARK_IN: "ENTRY", RequestType.PARK_OUT: "EXIT"}

# 팀원 TaskState.state(8단계) → UI RequestStatus(6단계) 매핑. ros2_prs_source.py와 동일.
_STATE_MAP = {
    "SEARCHING":   RequestStatus.ROBOT_ASSIGNED,
    "APPROACHING": RequestStatus.APPROACHING,
    "PICKED_UP":   RequestStatus.LIFTING,
    "MOVING":      RequestStatus.MOVING_TO_SLOT,
    "ARRIVED":     RequestStatus.MOVING_TO_SLOT,
    "PARKED":      RequestStatus.RETURNING,
    "UNPARKED":    RequestStatus.RETURNING,
    "RETURNING":   RequestStatus.RETURNING,
    "DONE":        RequestStatus.COMPLETED,
    "FAILED":      RequestStatus.CANCELLED,
}
_ACTIVE_STATES = {"SEARCHING", "APPROACHING", "PICKED_UP", "MOVING", "ARRIVED"}


def _now() -> str:
    return datetime.now().strftime("%H:%M:%S")


class Ros2DualDataSource(DataSource):
    """입차/출차 로봇 그룹 분리 라우팅용 실시스템 소스 (Mock 제어 없음)."""

    supports_mock_controls = False
    mock_auto_advance = False

    def __init__(self, store) -> None:
        super().__init__(store)
        self._node: Node | None = None
        self._executor = None
        self._thread = None
        self._own_rclpy = False
        #: group_id -> RequestParkingTask 클라이언트
        self._dispatch_clients: dict = {}
        #: request_type(str) -> robot_groups Row (DB 스냅샷, start() 시점에 로드)
        self._groups_by_request_type: dict = {}
        #: group_id -> robot_groups Row
        self._groups_by_id: dict = {}

    # ------------------------------------------------------------------
    # 기동/정리
    # ------------------------------------------------------------------
    def start(self) -> None:
        db.init_db()
        groups = db.list_robot_groups()
        self._groups_by_id = {g["group_id"]: g for g in groups}
        self._groups_by_request_type = {g["request_type"]: g for g in groups}

        if not rclpy.ok():
            rclpy.init()
            self._own_rclpy = True
        self._node = Node("parking_control_dual_bridge")
        grp = ReentrantCallbackGroup()

        for group in groups:
            self._dispatch_clients[group["group_id"]] = self._node.create_client(
                RequestParkingTask, group["dispatch_service"], callback_group=grp)

        self._node.create_subscription(String, "/parking_slots", self._on_slots, 10,
                                       callback_group=grp)
        self._node.create_subscription(TaskState, "task_state", self._on_task_state, 20,
                                       callback_group=grp)

        robot_ids = set()
        for group in groups:
            for rid in (group["leader_robot_id"], group["follower_robot_id"]):
                if rid:
                    robot_ids.add(rid)
        for rid in robot_ids:
            self._node.create_subscription(
                Odometry, f"/{rid}/odom",
                lambda m, r=rid: self._on_odom(r, m), 10, callback_group=grp)

        self._executor = MultiThreadedExecutor(num_threads=4)
        self._executor.add_node(self._node)
        self._thread = threading.Thread(target=self._executor.spin, daemon=True)
        self._thread.start()

    def stop(self) -> None:
        if self._executor is not None:
            self._executor.shutdown()
        if self._node is not None:
            self._node.destroy_node()
        if self._own_rclpy and rclpy.ok():
            rclpy.shutdown()

    # ------------------------------------------------------------------
    # 구독 콜백 → StateStore 갱신 (ros2_prs_source.py와 동일한 방식)
    # ------------------------------------------------------------------
    def _on_slots(self, msg) -> None:
        try:
            data = json.loads(msg.data)
        except Exception:
            return
        slots = [
            ParkingSlot(
                id=s["slot_id"],
                status="OCCUPIED" if s.get("occupied") else "EMPTY",
                vehicle_number=None,
                x=s.get("x"), y=s.get("y"),
                is_accessible=bool(s.get("is_accessible")),
            )
            for s in data
        ]
        with self.store.lock:
            self.store.parking_slots = slots

    def _on_odom(self, rid, m) -> None:
        p = m.pose.pose.position
        x, y = round(p.x, 2), round(-p.z, 2)   # odom(x,z) → 지도(x, y=-z)
        with self.store.lock:
            robot = self.store.find_robot(rid)
            if robot is None:
                self.store.robots.append(Robot(id=rid, status="IDLE", battery=100, x=x, y=y))
            else:
                robot.x, robot.y = x, y

    def _on_task_state(self, msg) -> None:
        status = _STATE_MAP.get(msg.state)
        with self.store.lock:
            req = next((r for r in self.store.requests
                        if r.external_task_id and r.external_task_id == msg.task_id), None)
            if req is not None:
                if status is not None:
                    req.status = status
                if msg.robot_id:
                    req.robot_id = msg.robot_id
                    if msg.robot_id not in req.robot_ids:
                        req.robot_ids.append(msg.robot_id)
            if msg.robot_id:
                robot = self.store.find_robot(msg.robot_id)
                if robot is not None:
                    if msg.state in _ACTIVE_STATES or msg.state == "RETURNING":
                        robot.status = "BUSY"
                        robot.current_task_id = req.id if req else None
                    elif msg.state in ("DONE", "PARKED", "UNPARKED"):
                        robot.status = "IDLE"
                        robot.current_task_id = None
                    elif msg.state == "FAILED":
                        robot.status = "ERROR"
                        robot.error_message = msg.current_step or "작업 실패"
            if msg.state == "FAILED":
                self.store.alerts.append(Alert(
                    id=self.store.next_alert_id(),
                    level=AlertLevel.ERROR, category=AlertCategory.ROBOT_ERROR,
                    message=f"{msg.robot_id or '로봇'} 작업 실패"
                            f"{f' ({msg.current_step})' if msg.current_step else ''}",
                    robot_id=msg.robot_id or None, created_at=_now(),
                ))

    # ------------------------------------------------------------------
    # 요청 등록 → 요청 유형(PARK_IN/PARK_OUT)에 맞는 로봇 그룹으로 라우팅
    # ------------------------------------------------------------------
    def create_request(self, payload) -> ParkingRequest:
        vehicle_number = payload.vehicle_number.strip()
        if not vehicle_number:
            raise DataSourceError("차량 번호를 입력해주세요.", status_code=400)

        ros_type = _REQUEST_TYPE_TO_ROS[payload.request_type]
        group = self._groups_by_request_type.get(ros_type)
        if group is None:
            raise DataSourceError(
                f"{ros_type} 요청을 처리할 로봇 그룹이 라우팅 표(DB)에 없습니다.",
                status_code=500,
            )
        group_id = group["group_id"]
        client = self._dispatch_clients.get(group_id)

        if client is None or not client.wait_for_service(
            timeout_sec=config.DUAL_DISPATCH_TIMEOUT_SEC
        ):
            message = (
                f"'{group['display_name']}'({group['dispatch_service']}) 서비스에 연결할 수 "
                "없습니다. 해당 Isaac Sim/로봇 스택(nodes.launch.py 계열)이 이 도메인에 "
                "떠 있는지 확인해주세요."
            )
            db.log_dispatch(
                task_id=None, group_id=group_id, request_type=ros_type,
                vehicle_number=vehicle_number, accepted=False, message=message,
            )
            raise DataSourceError(message, status_code=503)

        request = RequestParkingTask.Request()
        request.request_type = ros_type
        request.vehicle_id = vehicle_number

        done = threading.Event()
        outcome: dict = {}

        def _on_done(future) -> None:
            outcome["response"] = future.result()
            outcome["exception"] = future.exception()
            done.set()

        future = client.call_async(request)
        future.add_done_callback(_on_done)

        if not done.wait(timeout=config.DUAL_DISPATCH_TIMEOUT_SEC):
            message = f"'{group['display_name']}' 응답이 시간 내에 오지 않았습니다."
            db.log_dispatch(
                task_id=None, group_id=group_id, request_type=ros_type,
                vehicle_number=vehicle_number, accepted=False, message=message,
            )
            raise DataSourceError(message, status_code=504)

        if outcome.get("exception") is not None:
            message = f"'{group['display_name']}' 호출 중 오류: {outcome['exception']}"
            db.log_dispatch(
                task_id=None, group_id=group_id, request_type=ros_type,
                vehicle_number=vehicle_number, accepted=False, message=message,
            )
            raise DataSourceError(message, status_code=502)

        response = outcome["response"]
        if not response.accepted:
            message = response.message or f"'{group['display_name']}'이(가) 요청을 거절했습니다."
            db.log_dispatch(
                task_id=response.task_id or None, group_id=group_id, request_type=ros_type,
                vehicle_number=vehicle_number, accepted=False, message=message,
            )
            raise DataSourceError(message, status_code=409)

        db.log_dispatch(
            task_id=response.task_id, group_id=group_id, request_type=ros_type,
            vehicle_number=vehicle_number, accepted=True, message=response.message,
        )

        with self.store.lock:
            parking_request = ParkingRequest(
                id=self.store.next_request_id(),
                request_type=payload.request_type,
                vehicle_number=vehicle_number,
                slot_id=None,
                robot_id=None,
                robot_ids=[],
                status=RequestStatus.WAITING,
                created_at=_now(),
                external_task_id=response.task_id or None,
                robot_group=group_id,
            )
            self.store.requests.append(parking_request)
        return parking_request

    # ------------------------------------------------------------------
    # 로봇 그룹 상태 (관제 UI 패널용)
    # ------------------------------------------------------------------
    def get_robot_group_status(self) -> list[dict]:
        statuses = []
        for group_id, group in self._groups_by_id.items():
            client = self._dispatch_clients.get(group_id)
            connected = bool(client is not None and client.service_is_ready())
            statuses.append({
                "group_id": group_id,
                "display_name": group["display_name"],
                "request_type": group["request_type"],
                "dispatch_service": group["dispatch_service"],
                "connected": connected,
                "leader_robot_id": group["leader_robot_id"],
                "follower_robot_id": group["follower_robot_id"],
                "last_dispatch_at": db.last_dispatch_at(group_id),
            })
        return statuses

    # ------------------------------------------------------------------
    # Mock 전용 — 실시스템에서는 미지원
    # ------------------------------------------------------------------
    def advance_request(self, request_id: int) -> ParkingRequest:
        raise DataSourceError(
            "실시스템 모드에서는 단계를 수동 진행할 수 없습니다 (dispatcher가 제어).",
            status_code=403)

    def reset(self) -> None:
        raise DataSourceError("실시스템 모드에서는 초기화를 지원하지 않습니다.", status_code=403)
