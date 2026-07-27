#!/usr/bin/env python3
"""비상정지와 관제 복구 승인 상태의 단일 권위 노드.

안전 상태는 MySQL에 영속 저장하고 `/safety/state`에 transient-local QoS로
발행한다. 비상정지 해제 요청은 모션을 재개하지 않고 READY_FOR_OPERATION
상태까지만 전환한다. 별도의 운영 복귀 승인이 있어야 NORMAL이 된다.
"""

import threading

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from parking_robot_interfaces.msg import FormationStop, SafetyState
from parking_robot_interfaces.srv import (
    ActivateEmergencyStop,
    ApproveOperation,
    RequestSafetyReset,
)

from parking_control.core.db import ParkingDB

NORMAL = "NORMAL"
STOPPED_LATCHED = "STOPPED_LATCHED"
READY_FOR_OPERATION = "READY_FOR_OPERATION"


class SafetySupervisorNode(Node):

    def __init__(self):
        super().__init__("safety_supervisor")
        self.declare_parameter("db_host", "localhost")
        self.declare_parameter("db_user", "parking")
        self.declare_parameter("db_password", "parking1234")
        self.declare_parameter("db_name", "parking")

        p = self.get_parameter
        self._db = ParkingDB(
            host=p("db_host").value,
            user=p("db_user").value,
            password=p("db_password").value,
            database=p("db_name").value,
        )
        self._db.ensure_safety_schema()
        self._lock = threading.RLock()
        self._state = self._db.get_safety_state()

        state_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._state_pub = self.create_publisher(
            SafetyState, "/safety/state", state_qos
        )
        self._formation_stop_pub = self.create_publisher(
            FormationStop, "/formation_stop", 10
        )
        self.create_service(
            ActivateEmergencyStop,
            "/safety/activate_emergency_stop",
            self._activate,
        )
        self.create_service(
            RequestSafetyReset,
            "/safety/request_reset",
            self._request_reset,
        )
        self.create_service(
            ApproveOperation,
            "/safety/approve_operation",
            self._approve_operation,
        )

        # 상태 토픽은 재기동 노드용, 1 Hz 재발행은 구독/네트워크 이상 복구용이다.
        self.create_timer(1.0, self._heartbeat)
        self._publish_state()
        if self._state["state"] != NORMAL:
            self._publish_legacy_stop("영속 비상정지 상태 복원")
        self.get_logger().info(
            f"safety_supervisor 시작: state={self._state['state']} "
            f"epoch={self._state['stop_epoch']}"
        )

    def _message(self):
        return SafetyState(
            state=self._state["state"],
            motion_allowed=self._state["state"] == NORMAL,
            stop_epoch=int(self._state["stop_epoch"]),
            reason=self._state.get("reason") or "",
            operator_id=self._state.get("operator_id") or "",
            inspection_note=self._state.get("inspection_note") or "",
            affected_task_ids=list(self._state.get("affected_task_ids") or []),
            blockers=list(self._state.get("blockers") or []),
            updated_at=self._state.get("updated_at") or "",
        )

    def _publish_state(self):
        self._state = self._db.get_safety_state()
        self._state_pub.publish(self._message())

    def _publish_legacy_stop(self, reason):
        self._formation_stop_pub.publish(
            FormationStop(
                task_id="",
                source_robot_id="control_tower",
                stop=True,
                reason=reason,
            )
        )

    def _heartbeat(self):
        with self._lock:
            self._publish_state()
            if self._state["state"] != NORMAL:
                self._publish_legacy_stop(
                    f"중앙 안전 상태 유지: {self._state['state']}"
                )

    def _write_state(
        self,
        *,
        state,
        stop_epoch,
        reason,
        operator_id,
        inspection_note,
        affected_task_ids,
        blockers,
        event_type,
        event_note,
    ):
        self._db.set_safety_state(
            state=state,
            stop_epoch=stop_epoch,
            reason=reason,
            operator_id=operator_id,
            inspection_note=inspection_note,
            affected_task_ids=affected_task_ids,
            blockers=blockers,
        )
        self._db.add_safety_event(
            stop_epoch=stop_epoch,
            event_type=event_type,
            state=state,
            operator_id=operator_id,
            note=event_note,
            details={
                "reason": reason,
                "affected_task_ids": list(affected_task_ids),
                "blockers": list(blockers),
            },
        )
        self._publish_state()

    def _activate(self, request, response):
        with self._lock:
            current = self._db.get_safety_state()
            if current["state"] == STOPPED_LATCHED:
                response.accepted = True
                response.state = current["state"]
                response.stop_epoch = int(current["stop_epoch"])
                response.message = "비상정지가 이미 유지되고 있습니다."
                return response
            if current["state"] not in {NORMAL, READY_FOR_OPERATION}:
                response.accepted = False
                response.state = current["state"]
                response.stop_epoch = int(current["stop_epoch"])
                response.message = "현재 안전 상태에서는 비상정지를 갱신할 수 없습니다."
                return response

            operator_id = request.operator_id.strip() or "control_ui"
            reason = request.reason.strip() or "관제 UI 전체 비상정지"
            epoch = int(current["stop_epoch"]) + 1
            affected = list(
                dict.fromkeys(
                    [
                        *current["affected_task_ids"],
                        *request.affected_task_ids,
                    ]
                )
            )
            self._write_state(
                state=STOPPED_LATCHED,
                stop_epoch=epoch,
                reason=reason,
                operator_id=operator_id,
                inspection_note="",
                affected_task_ids=affected,
                blockers=["현장 안전 점검 및 관제 해제 승인 필요"],
                event_type="EMERGENCY_STOP",
                event_note=reason,
            )
            self._publish_legacy_stop(reason)
            response.accepted = True
            response.state = STOPPED_LATCHED
            response.stop_epoch = epoch
            response.message = (
                "비상정지가 작동했습니다. 현장 점검 후 관제 해제 요청이 필요합니다."
            )
            self.get_logger().error(
                f"비상정지 epoch={epoch}, operator={operator_id}: {reason}"
            )
            return response

    def _request_reset(self, request, response):
        with self._lock:
            current = self._db.get_safety_state()
            blockers = []
            operator_id = request.operator_id.strip()
            inspection_note = request.inspection_note.strip()
            checks = (
                (request.area_clear, "작업 구역 안전 확인 필요"),
                (request.robots_stopped, "전체 로봇 정지 확인 필요"),
                (request.load_secured, "차량·리프트 상태 확인 필요"),
                (request.sensors_checked, "센서·통신 상태 확인 필요"),
            )
            if current["state"] != STOPPED_LATCHED:
                blockers.append("현재 상태에서는 해제 요청을 접수할 수 없습니다.")
            if not operator_id:
                blockers.append("확인한 관제 담당자 ID가 필요합니다.")
            if len(inspection_note) < 5:
                blockers.append("점검 결과를 5자 이상 입력해주세요.")
            blockers.extend(message for checked, message in checks if not checked)
            active_tasks = self._db.active_task_ids()
            if active_tasks:
                blockers.append(
                    f"종료 처리되지 않은 작업 {len(active_tasks)}건이 남아 있습니다."
                )

            if blockers:
                response.accepted = False
                response.state = current["state"]
                response.stop_epoch = int(current["stop_epoch"])
                response.message = "안전 해제 조건을 충족하지 못했습니다."
                response.blockers = blockers
                return response

            self._write_state(
                state=READY_FOR_OPERATION,
                stop_epoch=int(current["stop_epoch"]),
                reason=current["reason"],
                operator_id=operator_id,
                inspection_note=inspection_note,
                affected_task_ids=current["affected_task_ids"],
                blockers=[],
                event_type="RESET_VERIFIED",
                event_note=inspection_note,
            )
            response.accepted = True
            response.state = READY_FOR_OPERATION
            response.stop_epoch = int(current["stop_epoch"])
            response.message = (
                "점검 결과가 승인되었습니다. 로봇은 계속 정지 상태이며 "
                "영향 로봇은 제한 안전 복귀를 먼저 완료해야 합니다."
            )
            response.blockers = []
            return response

    def _approve_operation(self, request, response):
        with self._lock:
            current = self._db.get_safety_state()
            blockers = []
            operator_id = request.operator_id.strip()
            approval_note = request.approval_note.strip()
            if current["state"] != READY_FOR_OPERATION:
                blockers.append("점검 완료 상태에서만 운영 복귀를 승인할 수 있습니다.")
            if not operator_id:
                blockers.append("승인한 관제 담당자 ID가 필요합니다.")
            if len(approval_note) < 5:
                blockers.append("운영 복귀 사유를 5자 이상 입력해주세요.")
            active_tasks = self._db.active_task_ids()
            if active_tasks:
                blockers.append(
                    f"종료 처리되지 않은 작업 {len(active_tasks)}건이 남아 있습니다."
                )
            unsafe_robots = self._db.unsafe_robot_statuses()
            if unsafe_robots:
                detail = ", ".join(
                    f"{row['robot_id']}={row['status']}" for row in unsafe_robots
                )
                blockers.append(f"운영 복귀 불가 로봇 상태: {detail}")

            if blockers:
                response.accepted = False
                response.state = current["state"]
                response.stop_epoch = int(current["stop_epoch"])
                response.message = "운영 복귀 조건을 충족하지 못했습니다."
                response.blockers = blockers
                return response

            self._write_state(
                state=NORMAL,
                stop_epoch=int(current["stop_epoch"]),
                reason="",
                operator_id=operator_id,
                inspection_note=current["inspection_note"],
                affected_task_ids=[],
                blockers=[],
                event_type="OPERATION_APPROVED",
                event_note=approval_note,
            )
            response.accepted = True
            response.state = NORMAL
            response.stop_epoch = int(current["stop_epoch"])
            response.message = (
                "정상 운영 복귀가 승인되었습니다. 기존 취소 작업은 재개되지 않으며 "
                "새 작업만 접수합니다."
            )
            response.blockers = []
            self.get_logger().info(
                f"운영 복귀 epoch={current['stop_epoch']}, operator={operator_id}"
            )
            return response

    def destroy_node(self):
        self._db.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SafetySupervisorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
