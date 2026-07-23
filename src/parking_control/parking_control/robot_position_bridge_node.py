#!/usr/bin/env python3
"""robot_position_bridge: 로봇 실측 odom(/robot_rear/odom, /robot_front/odom)을
DB(robots.x/y)에 기록해 웹 대시보드에 실시간 위치가 보이게 하는 다리.

지금까지 이 다리가 없어서 DB에는 sim_orchestrator(가짜 로봇 데모용)가 채우는
값만 있었고, 실제 Isaac Sim 로봇이 움직여도 웹 화면에는 반영되지 않았다.

좌표 변환은 isaac_parking_bridge_node가 이미 실측으로 검증해둔 규약을 그대로
쓴다: map_x = world_x, map_y = -world_z (맵 y축과 Isaac world z축이 반대,
B1/A1 위치로 대조 확인됨).

odom은 초당 수십 번 오므로 매번 DB에 쓰지 않고 ROBOT_UPDATE_INTERVAL_SEC
간격으로 스로틀한다 — 대시보드도 1.5~2초 간격 폴링이라 그보다 자주 쓸 필요가
없다(DB 부하만 늘어남).
"""
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node

from parking_control.core.db import ParkingDB

ROBOTS = ("robot_rear", "robot_front")
ROBOT_UPDATE_INTERVAL_SEC = 0.5


class RobotPositionBridgeNode(Node):

    def __init__(self):
        super().__init__("robot_position_bridge")

        self.declare_parameter("db_host", "localhost")
        self.declare_parameter("db_user", "parking")
        self.declare_parameter("db_password", "parking1234")
        self.declare_parameter("db_name", "parking")

        p = self.get_parameter
        self._db = ParkingDB(
            host=p("db_host").value, user=p("db_user").value,
            password=p("db_password").value, database=p("db_name").value)

        self._last_write = {rid: 0.0 for rid in ROBOTS}
        for rid in ROBOTS:
            self._db.upsert_robot(rid)
            self.create_subscription(
                Odometry, f"/{rid}/odom",
                lambda msg, r=rid: self._on_odom(r, msg), 10)

        self.get_logger().info(
            f"robot_position_bridge 시작 — {', '.join(ROBOTS)} odom → DB")

    def _on_odom(self, robot_id, msg):
        now = time.monotonic()
        if now - self._last_write[robot_id] < ROBOT_UPDATE_INTERVAL_SEC:
            return
        self._last_write[robot_id] = now
        x_map = msg.pose.pose.position.x
        y_map = -msg.pose.pose.position.z
        self._db.update_robot_position(robot_id, x_map, y_map)


def main(args=None):
    rclpy.init(args=args)
    node = RobotPositionBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
