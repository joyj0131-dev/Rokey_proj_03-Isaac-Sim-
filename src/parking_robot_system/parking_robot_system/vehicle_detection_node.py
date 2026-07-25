#!/usr/bin/env python3
"""vehicle_detection_node (인식): 위치·크기 인식.

detect_vehicle(액션 서버) — P1 스텁: 실제 카메라/LiDAR 인식(SR-01, SR-04, P5) 대신
알려진 인계 위치를 고정 반환한다. 이 노드는 입차 세트에서만 실제로 쓰인다(출차는
SEARCHING에서 이미 알고 있는 슬롯 좌표를 그대로 쓰고 detect_vehicle을 안 부른다).
2026-07-24: 좌표(USD x,z)를 파라미터화 — 기본값은 v3 레이아웃 입차 인계지점
(vehicle:entryWait). request.trigger는 이 스텁에서는 사용하지 않는다(항상 동일 응답).
"""
import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node

from parking_robot_interfaces.action import DetectVehicle
from parking_robot_system.frame_transform import usd_to_map

# Pickup 대략값(P1 스텁; 실제 인식은 P5에서 카메라/LiDAR로 대체).
PICKUP_LENGTH_M = 4.8
PICKUP_WIDTH_M = 1.9
PICKUP_HEIGHT_M = 1.5
PICKUP_Y_USD = 0.035   # 차체 바닥 높이(USD +Y), 위치와 무관하게 고정


class VehicleDetectionNode(Node):

    def __init__(self):
        super().__init__('vehicle_detection_node')

        self.declare_parameter("pickup_x_usd", -8.5)
        self.declare_parameter("pickup_z_usd", -5.5)
        self._pickup_x = self.get_parameter("pickup_x_usd").value
        self._pickup_z = self.get_parameter("pickup_z_usd").value

        self._action_server = ActionServer(
            self, DetectVehicle, 'detect_vehicle', self._on_detect_vehicle)

        self.get_logger().info('vehicle_detection_node started')

    def _on_detect_vehicle(self, goal_handle):
        # TODO(SR-01, SR-04, P5): 카메라/LiDAR로 실제 차량 위치·방향·크기 인식.
        # P1: 알려진 인계 위치를 고정 반환.
        goal_handle.succeed()
        x_map, y_map = usd_to_map(self._pickup_x, self._pickup_z)
        y_usd = PICKUP_Y_USD

        result = DetectVehicle.Result()
        info = result.vehicle_info
        info.pose.position.x = x_map
        info.pose.position.y = y_map
        info.pose.position.z = y_usd   # USD +Y(상방) 높이를 그대로 전달
        info.pose.orientation.w = 1.0  # 회전 없음(기본 Quaternion은 정규화되지 않은 0벡터)
        info.length = PICKUP_LENGTH_M
        info.width = PICKUP_WIDTH_M
        info.height = PICKUP_HEIGHT_M
        result.success = True
        return result


def main(args=None):
    rclpy.init(args=args)
    node = VehicleDetectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
