"""parking_robot_system(feat/camera) 연동 데이터 소스.

dongsoo의 ros2_source.py는 parking_control(MySQL + /dispatch_parking_task)에 묶여
있어, 팀원의 parking_robot_system(토픽 + 슬롯지정 모델)에는 안 맞는다. 이 소스는
같은 DataSource 인터페이스를 구현하되 팀원 시스템에 직접 배선한다:

  요청 : /park_in_slot · /exit_slot (parking_robot_interfaces/srv/ParkInSlot, slot_id)
         프론트가 슬롯을 안 보내면(vehicle_number만) 서버가 슬롯을 고른다
         (입차=빈 슬롯, 출차=점유 슬롯).
  슬롯 : /parking_slots (std_msgs/String JSON) 구독
  로봇 : /robot_*/odom (nav_msgs/Odometry) 구독 (지도 y = -odom.z)
  진행 : task_state (parking_robot_interfaces/msg/TaskState) 구독 → 요청 상태 갱신

PARKING_MODE=prs 로 선택. MySQL/parking_control 의존 없음.
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
from parking_robot_interfaces.srv import ParkInSlot

from core.datasource import DataSource, DataSourceError
from core.models import (
    Alert, AlertCategory, AlertLevel,
    ParkingRequest, ParkingSlot, Robot, RequestStatus, RequestType,
)

ROBOT_IDS = ("robot_rear", "robot_front")

# 팀원 TaskState.state(8단계) → UI RequestStatus(6단계) 매핑.
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


def _yaw_deg(q) -> float:
    return math.degrees(math.atan2(2 * (q.w * q.z + q.x * q.y),
                                   1 - 2 * (q.y * q.y + q.z * q.z)))


class Ros2PrsDataSource(DataSource):
    """parking_robot_system 백엔드용 실시스템 소스 (Mock 제어 없음)."""

    supports_mock_controls = False
    mock_auto_advance = False

    def __init__(self, store) -> None:
        super().__init__(store)
        self._node = None
        self._executor = None
        self._thread = None
        self._own_rclpy = False

    # ------------------------------------------------------------------
    # 기동/정리
    # ------------------------------------------------------------------
    def start(self) -> None:
        if not rclpy.ok():
            rclpy.init()
            self._own_rclpy = True
        self._node = Node("parking_ui_prs")
        grp = ReentrantCallbackGroup()
        self._node.create_subscription(String, "/parking_slots", self._on_slots, 10,
                                       callback_group=grp)
        for rid in ROBOT_IDS:
            self._node.create_subscription(
                Odometry, f"/{rid}/odom",
                lambda m, r=rid: self._on_odom(r, m), 10, callback_group=grp)
        self._node.create_subscription(TaskState, "task_state", self._on_task_state, 20,
                                       callback_group=grp)
        self._park_cli = self._node.create_client(ParkInSlot, "/park_in_slot",
                                                  callback_group=grp)
        self._exit_cli = self._node.create_client(ParkInSlot, "/exit_slot",
                                                  callback_group=grp)
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
    # 구독 콜백 → StateStore 갱신
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
            # 로봇 상태 반영
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
    # 슬롯 결정 헬퍼 (프론트가 슬롯 미지정 시 서버가 고른다)
    # ------------------------------------------------------------------
    def _pick_slot(self, request_type, given) -> str | None:
        if given:
            return given
        with self.store.lock:
            want = "EMPTY" if request_type == RequestType.PARK_IN else "OCCUPIED"
            # 입차는 장애인석을 뒤로 미룸(일반석 우선), 그 외 순서대로
            cands = [s for s in self.store.parking_slots if s.status == want]
            if request_type == RequestType.PARK_IN:
                cands.sort(key=lambda s: (s.is_accessible, s.id))
            else:
                cands.sort(key=lambda s: s.id)
            return cands[0].id if cands else None

    # ------------------------------------------------------------------
    # 요청 등록 (FastAPI 워커 스레드에서 호출)
    # ------------------------------------------------------------------
    def create_request(self, payload) -> ParkingRequest:
        vehicle_number = payload.vehicle_number.strip()
        if not vehicle_number:
            raise DataSourceError("차량 번호를 입력해주세요.", status_code=400)

        slot_id = self._pick_slot(payload.request_type, payload.slot_id)
        if not slot_id:
            kind = "빈" if payload.request_type == RequestType.PARK_IN else "점유된"
            raise DataSourceError(
                f"가능한 {kind} 슬롯이 없습니다. (러너/parking_slots 발행 확인)",
                status_code=409)

        cli = self._park_cli if payload.request_type == RequestType.PARK_IN else self._exit_cli
        name = "/park_in_slot" if payload.request_type == RequestType.PARK_IN else "/exit_slot"
        if not cli.wait_for_service(timeout_sec=5.0):
            raise DataSourceError(
                f"{name} 서비스에 연결할 수 없습니다. (user_request_gateway 노드 확인)",
                status_code=503)

        done = threading.Event()
        outcome: dict = {}

        def _on_done(fut):
            outcome["res"] = fut.result()
            outcome["err"] = fut.exception()
            done.set()

        fut = cli.call_async(ParkInSlot.Request(slot_id=slot_id))
        fut.add_done_callback(_on_done)
        if not done.wait(timeout=8.0):
            raise DataSourceError("게이트웨이 응답 시간 초과", status_code=504)
        if outcome.get("err") is not None:
            raise DataSourceError(f"서비스 호출 오류: {outcome['err']}", status_code=502)

        res = outcome["res"]
        if not res.accepted:
            raise DataSourceError(res.message or "요청이 거절되었습니다.", status_code=409)

        with self.store.lock:
            req = ParkingRequest(
                id=self.store.next_request_id(),
                request_type=payload.request_type,
                vehicle_number=vehicle_number,
                slot_id=slot_id,
                robot_id=None, robot_ids=[],
                status=RequestStatus.WAITING,
                created_at=_now(),
                external_task_id=res.task_id or None,
            )
            self.store.requests.append(req)
        return req

    # ------------------------------------------------------------------
    # Mock 전용 — 실시스템에서는 미지원
    # ------------------------------------------------------------------
    def advance_request(self, request_id: int) -> ParkingRequest:
        raise DataSourceError(
            "실시스템 모드에서는 단계를 수동 진행할 수 없습니다 (dispatcher가 제어).",
            status_code=403)

    def reset(self) -> None:
        raise DataSourceError("실시스템 모드에서는 초기화를 지원하지 않습니다.", status_code=403)
