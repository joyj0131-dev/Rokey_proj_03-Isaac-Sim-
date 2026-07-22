#!/usr/bin/env python3
"""navigate_action_server (이동): Isaac 편대모션 브리지, nav2_msgs/action/NavigateToPose 재사용.

goal.pose(geometry_msgs/PoseStamped, map 프레임)를 parking_robot_system.frame_transform로
USD 프레임으로 변환한 뒤, goal.behavior_tree 문자열을 모드로 사용해 FormationMotion(검증된
dock_lift_handoff_mission 폐루프 이식, formation_motion.py 참고)에 위임한다:
    "carry"                     -> FormationMotion.carry_to(tx,tz)        편대(둘 다) 운반 이동
    "rotate"                    -> FormationMotion.carry_rotate_to(yaw)   편대(둘 다) 회전(정렬)
    "rear"|"front"              -> FormationMotion.goto_xz(rid, tx, tz)   로봇 1대 개별 직선 이동
    "return_both"               -> FormationMotion.return_both_to_docks() 두 로봇 동시 초기도크 복귀
그 외 문자열(빈 문자열 포함)은 알 수 없는 모드로 간주해 abort한다 — 조용히 기본값으로
넘어가면(예: 빈 문자열을 "carry"로 취급) 호출부의 설정 누락을 숨기게 되므로 의도적으로 엄격하다.

NavigateToPose.Result 에는 success 필드가 없다(std_msgs/Empty result 뿐 — nav2 표준 계약).
그래서 성공/실패는 오직 액션 상태(goal_handle.succeed()/abort())로만 전달한다: "수렴 시
succeed"(Task 10b 지시), 그 외(데이터 미수신·알 수 없는 모드·폐루프 타임아웃)는 abort.
(align/lift 액션서버는 자체 success bool 필드가 있어 항상 succeed() 후 그 필드로 성패를
전달하는 다른 관례를 쓴다 — 이건 인터페이스 계약 차이지 실수가 아니다.)

★동시성 패턴(필수, lift_action_server/Task 7-8과 동일 교정 패턴): FormationMotion의 폐루프는
최대 수분 블로킹된다. odom/vehicle 구독(FormationMotion 내부)과 이 액션서버 콜백을 모두 같은
ReentrantCallbackGroup에 두고 main()을 MultiThreadedExecutor(num_threads=4)로 스핀해야
블로킹 루프 도중에도 odom 콜백이 돌아 self.pose가 갱신된다(안 그러면 폐루프가 수렴 못 함 — 치명적).
"""
import math

import rclpy
from rclpy.action import ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from nav2_msgs.action import NavigateToPose

from parking_robot_system.formation_motion import FormationMotion
from parking_robot_system.frame_transform import map_to_usd, map_to_usd_yaw_deg

# carry 모드 L자 경로용 통로 z(USD). 중앙 통로 중심이자 서쪽 벽 개구부(z∈[-4.5,4.5]) 중심.
# 인계베이(x≈-29.6)→슬롯까지 이 z를 따라 동진하면 개구부를 통과해 벽을 관통하지 않는다.
AISLE_Z = 0.0


def _yaw_from_quaternion(q):
    """geometry_msgs/Quaternion -> yaw(rad). formation_motion.FormationMotion._odom과 동일
    공식(원본 dock_lift_handoff_mission.HandoffMission._odom, L88)을 그대로 사용."""
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class NavigateActionServerNode(Node):

    def __init__(self):
        super().__init__('navigate_action_server')

        grp = ReentrantCallbackGroup()
        self.formation = FormationMotion(self, callback_group=grp)

        self._action_server = ActionServer(
            self, NavigateToPose, 'navigate_to_pose', self._on_navigate_to_pose,
            callback_group=grp)

        self.get_logger().info('navigate_action_server started')

    def _on_navigate_to_pose(self, goal_handle):
        goal = goal_handle.request
        mode = goal.behavior_tree

        if not self.formation.wait_data():
            self.get_logger().warn('navigate_to_pose: 데이터 미수신(odom/vehicle pose)')
            goal_handle.abort()
            return NavigateToPose.Result()

        x_map = goal.pose.pose.position.x
        y_map = goal.pose.pose.position.y
        tx_usd, tz_usd = map_to_usd(x_map, y_map)

        if mode == 'carry':
            # 사용자 보고: 슬롯으로 곧장 직선 이동하면 서쪽 벽(x≈-18.1)을 관통한다.
            # 픽업하러 갈 때 쓴 통로(중앙 통로 z≈0 + 개구부)를 따라가도록 L자 경로로 나눈다:
            #   ① 슬롯 x열까지 통로(z=AISLE_Z)를 따라 동진(개구부 통과) → ② 슬롯으로 진입.
            # (이미 통로에 있으면 ①은 사실상 짧게 끝난다.)
            ok = self.formation.carry_to(tx_usd, AISLE_Z)
            if ok:
                ok = self.formation.carry_to(tx_usd, tz_usd)
        elif mode == 'rotate':
            yaw_map_rad = _yaw_from_quaternion(goal.pose.pose.orientation)
            yaw_usd_deg = map_to_usd_yaw_deg(math.degrees(yaw_map_rad))
            ok = self.formation.carry_rotate_to(math.radians(yaw_usd_deg))
        elif mode in ('rear', 'front'):
            ok = self.formation.goto_xz(f'robot_{mode}', tx_usd, tz_usd)
        elif mode == 'return_both':
            # 복귀: 두 로봇 동시에 초기 대기 도크로(슬롯→앞→통로→도크). 도크 좌표는
            # FormationMotion에 내장이라 goal.pose는 사용하지 않는다.
            ok = self.formation.return_both_to_docks()
        else:
            self.get_logger().warn(f'navigate_to_pose: 알 수 없는 behavior_tree 모드 {mode!r}')
            goal_handle.abort()
            return NavigateToPose.Result()

        if ok:
            goal_handle.succeed()
        else:
            goal_handle.abort()
        return NavigateToPose.Result()


def main(args=None):
    rclpy.init(args=args)
    node = NavigateActionServerNode()
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
