#!/usr/bin/env python3
"""vehicle_detection_node (인식): 위치·크기 인식.

detect_vehicle(액션 서버) — P1 스텁: 실제 카메라/LiDAR 인식(SR-01, SR-04, P5) 대신
알려진 Pickup 인계 위치를 고정 반환한다. 원본 러너
isaacpjt/Isaac_envo/dock_lift_handoff_runner.py의 VEHICLE_POS(USD x,y,z) =
(-29.6, 0.035, 0.0)를 map 프레임으로 변환(frame_transform 규약: x_map=x_usd,
y_map=-z_usd)해 VehicleInfo.pose를 채우고, length/width/height는 Pickup 대략값을 채워
success=True로 응답한다. request.trigger는 이 스텁에서는 사용하지 않는다(항상 동일 응답).
"""
import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node

from parking_robot_interfaces.action import DetectVehicle
from parking_robot_system.frame_transform import usd_to_map

# 원본 dock_lift_handoff_runner.VEHICLE_POS(x_usd, y_usd(상방), z_usd) 그대로.
PICKUP_POS_USD = (-29.6, 0.035, 0.0)
# Pickup 대략값(P1 스텁; 실제 인식은 P5에서 카메라/LiDAR로 대체).
PICKUP_LENGTH_M = 4.8
PICKUP_WIDTH_M = 1.9
PICKUP_HEIGHT_M = 1.5


class VehicleDetectionNode(Node):

    def __init__(self):
        super().__init__('vehicle_detection_node')

        self._action_server = ActionServer(
            self, DetectVehicle, 'detect_vehicle', self._on_detect_vehicle)

        self.get_logger().info('vehicle_detection_node started')

    def _on_detect_vehicle(self, goal_handle):
        # TODO(SR-01, SR-04, P5): 카메라/LiDAR로 실제 차량 위치·방향·크기 인식.
        # P1: 알려진 Pickup 인계 위치를 고정 반환.
        goal_handle.succeed()
        x_usd, y_usd, z_usd = PICKUP_POS_USD
        x_map, y_map = usd_to_map(x_usd, z_usd)

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
