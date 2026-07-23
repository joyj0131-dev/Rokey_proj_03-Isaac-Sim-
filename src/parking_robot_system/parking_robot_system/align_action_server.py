#!/usr/bin/env python3
"""align_action_server (정렬): AlignVehicle 액션 서버 — Isaac 편대모션 브리지.

원본 dock_lift_handoff_mission.HandoffMission._on_dock_lift(L254-286)의 접근+진입(픽업)
시퀀스를 FormationMotion.pickup_sequence()(formation_motion.py — 로직 변경 없이 이식)로
그대로 실행하고 final_error를 반환한다.

goal.target_pose는 AlignVehicle.action 인터페이스 계약상 수신하지만 P1에서는 사용하지 않는다:
pickup_sequence()는 미션 상수(DOCK_X/LANE_Z_REAR/LANE_Z_FRONT/NORTH_STAGE_Z/FACE_MZ 및
FormationMotion 생성자 기본값 center_x=-29.6·rear_axle_z=-1.93·front_axle_z=1.66)로 고정된
인계베이 Pickup 축 정렬만 수행한다 — 임의 target_pose로 일반화하는 것은 이번 태스크 지시가
명시한 대로 Task 12 범위(사용자 결정: "검증된 미션 편대 모션 재사용").

★동시성 패턴(필수, lift_action_server/Task 7-8과 동일 교정 패턴): pickup_sequence()는 최대
수분 블로킹되는 폐루프(게이트 통과 → rear 진입 → front 진입)다. odom/vehicle 구독
(FormationMotion 내부)과 이 액션서버 콜백을 모두 같은 ReentrantCallbackGroup에 두고 main()을
MultiThreadedExecutor(num_threads=4)로 스핀해야 블로킹 루프 도중에도 odom 콜백이 돌아
self.pose가 갱신된다(안 그러면 폐루프가 수렴 못 함 — 치명적).
"""
import rclpy
from rclpy.action import ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from parking_robot_interfaces.action import AlignVehicle
from parking_robot_system.formation_motion import FormationMotion
from parking_robot_system.frame_transform import map_to_usd

# 인계베이 픽업 판정 임계 x(USD). 베이 Pickup은 x≈-29.6, 슬롯은 x≥-15.3이라 그 사이로 가른다.
BAY_X_THRESHOLD = -25.0


class AlignActionServerNode(Node):

    def __init__(self):
        super().__init__('align_action_server')

        grp = ReentrantCallbackGroup()
        self.formation = FormationMotion(self, callback_group=grp)

        self._action_server = ActionServer(
            self, AlignVehicle, 'align_vehicle', self._on_align_vehicle, callback_group=grp)

        self.get_logger().info('align_action_server started')

    def _final_error(self):
        """두 로봇의 축 정렬 잔차 중 최댓값(ingress_to의 수렴 판정과 동일 축 — z, 원본 L192-193
        대조). odom이 아직 없으면 -1.0(관측 불가) 센티널을 반환한다."""
        errors = []
        axle_targets = (
            ('robot_rear', self.formation.rear_axle),
            ('robot_front', self.formation.front_axle),
        )
        for rid, target_z in axle_targets:
            pose = self.formation.pose.get(rid)
            if pose is None:
                return -1.0
            errors.append(abs(pose[1] - target_z))
        return max(errors)

    def _on_align_vehicle(self, goal_handle):
        # target_pose(map)로 픽업 위치를 판단: 인계베이(x≈-29.6)면 입차 픽업(검증된
        # pickup_sequence), 그 외(슬롯)면 출차 픽업(pickup_at_slot). 좌표 변환은 frame_transform.
        p = goal_handle.request.target_pose.position
        tx_usd, tz_usd = map_to_usd(p.x, p.y)
        if tx_usd <= BAY_X_THRESHOLD:
            ok, message = self.formation.pickup_sequence()             # 입차: 인계베이 픽업
        else:
            ok, message = self.formation.pickup_at_slot(tx_usd, tz_usd)  # 출차: 슬롯 픽업
        self.get_logger().info(f'align_vehicle: {message}')

        result = AlignVehicle.Result()
        result.success = ok
        result.final_error = float(self._final_error())
        goal_handle.succeed()
        return result


def main(args=None):
    rclpy.init(args=args)
    node = AlignActionServerNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
