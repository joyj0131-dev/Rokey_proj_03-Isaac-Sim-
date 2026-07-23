#!/usr/bin/env python3
"""pedestrian_cue: task_state를 보고 있다가 Isaac Sim 쪽 "사람 걷기" 연출에 필요한
순간에 간단한 신호(/pedestrian_cue, std_msgs/String)를 쏜다.

왜 이 노드가 따로 있나: 사람 캐릭터를 안 보이게/물리 끄기/순간이동시키는 건
Isaac Sim 프로세스 안에서 USD를 직접 만져야 하는 일이라, 우리 로봇 제어처럼
"외부에서 ROS2로 명령만 보내는" 패턴(dock_lift_handoff_mission.py)이 안 통한다.
그래서 판단(언제 연출할지)은 여기(관제 컴퓨터, 보통 system rclpy)서 하고,
실제 연출(Isaac 쪽 USD 조작)은 Isaac Sim 프로세스 안에서 도는 별도 스크립트가
이 신호만 받아서 하도록 역할을 나눴다.

트리거 판단:
  - ENTRY 시작(entry_depart): 차가 인계장에 이미 있는 채로 로봇이 픽업을
    시작하는 시점(SEARCHING). "차가 실제로 주행해서 들어오는" 이벤트는 지금
    시스템에 없다(차가 처음부터 인계장에 고정 배치돼 있음, VEHICLE_POS) —
    그래서 이 시점을 "손님이 막 도착해서 내리는 순간"의 대역으로 쓴다.
    SEARCHING은 ENTRY/EXIT 둘 다 있어서 task_id로 DB를 조회해 ENTRY만 가른다.
  - EXIT 완료(exit_arrive): orchestrator가 실제로 이렇게 발행한다
    (robot_task_orchestrator.py L397-398): state=='DONE'이고
    current_step=='출차 완료'. DB 조회 없이 문자열만으로 이미 유일하게 EXIT를
    가리킨다.

각 (task_id, state)는 orchestrator 상태머신이 정확히 한 번씩만 발행하므로
같은 신호를 중복으로 쏘는 걸 막는 별도 dedup은 필요 없다.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from parking_robot_interfaces.msg import TaskState

from parking_control.core.db import ParkingDB
from parking_control.parking_slot_manager_node import _default_map_yaml  # noqa: F401  (map_yaml 파라미터 관례 통일용)


class PedestrianCueNode(Node):

    def __init__(self):
        super().__init__("pedestrian_cue")

        self.declare_parameter("db_host", "localhost")
        self.declare_parameter("db_user", "parking")
        self.declare_parameter("db_password", "parking1234")
        self.declare_parameter("db_name", "parking")

        p = self.get_parameter
        self._db = ParkingDB(
            host=p("db_host").value, user=p("db_user").value,
            password=p("db_password").value, database=p("db_name").value)

        self._cue_pub = self.create_publisher(String, "pedestrian_cue", 10)
        self.create_subscription(TaskState, "task_state", self._on_task_state, 20)

        self.get_logger().info("pedestrian_cue 시작 — task_state 감시 중")

    def _on_task_state(self, msg: TaskState) -> None:
        if msg.state == "SEARCHING":
            task = self._db.get_task(msg.task_id)
            if task is not None and task.get("request_type") == "ENTRY":
                self._publish_cue("entry_depart", msg.task_id)
        elif msg.state == "DONE" and msg.current_step == "출차 완료":
            self._publish_cue("exit_arrive", msg.task_id)

    def _publish_cue(self, cue: str, task_id: str) -> None:
        self._cue_pub.publish(String(data=cue))
        self.get_logger().info(f"pedestrian_cue 발행: {cue} (task_id={task_id[:8]})")

    def destroy_node(self):
        self._db.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = PedestrianCueNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
