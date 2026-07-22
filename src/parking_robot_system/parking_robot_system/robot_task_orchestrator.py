#!/usr/bin/env python3
"""robot_task_orchestrator (제어): SR-11 상태머신 실행 — execute_parking_task 액션서버.

`execute_parking_task`(ExecuteParkingTask) goal(slot_id/slot_pose/leader·follower_robot_id)을
받아 아래 순수 전이 테이블(TRANSITIONS/next_state)을 따라 4개 액션서버(detect_vehicle/
align_vehicle/control_lift/navigate_to_pose(nav2))를 순차 호출하고, 매 전이마다 `task_state`
(TaskState) 토픽을 발행한다(design 문서 "오케스트레이터 상태머신" 절 그대로):

    SEARCHING    detect_vehicle(trigger=True)                    → Pickup 좌표 확보
    APPROACHING  align_vehicle(target_pose=탐지된 차량 pose)       → 두 로봇 차 밑 진입(픽업)
    PICKED_UP    control_lift(command="UP")                      → 리프트 상승
    MOVING       navigate_to_pose(goal.slot_pose, "carry")        → 슬롯으로 편대 운반
    ARRIVED      navigate_to_pose(goal.slot_pose, "rotate")       → 슬롯 목표 yaw로 편대 회전
    PARKED       control_lift(command="DOWN")                    → 안착
    RETURNING    navigate_to_pose(도크, "rear") → ("front")       → West 도크로 개별 복귀
    DONE         result.success=True

임의 단계 실패 시 상태 FAILED로 전이하고 result.success=False + task_state에 사유를 담아
발행한다(그 시점 이후 더 이상 액션을 호출하지 않음).

## 로봇 정지(best-effort) — orchestrator에는 직접적인 정지 채널이 없음
orchestrator 자신은 cmd_vel을 직접 publish하지 않는다(그 권한은 하위 4개 액션서버가
formation_motion.FormationMotion을 통해 갖는다). 대신 아래 두 가지로 "정지"를 실질적으로
확보한다:
  1. formation_motion.py의 모든 폐루프(goto_xz/approach_parallel/rotate_to/ingress_to/
     carry_to/carry_rotate_to/pickup_sequence)는 성공/실패 관계없이 반환 직전에 항상
     `_stop_all()`/`_pub(rid, 0.0)`을 호출한다 — 즉 orchestrator가 실패를 감지해 다음
     액션 호출을 멈추는 시점에는 이미 로봇이 정지해 있다(하위 서버가 스스로 멎지 않는
     한). 이것이 "정지 시도"의 실질적 메커니즘이다.
  2. `_on_execute_parking_task`는 실패 즉시 이후 단계를 절대 호출하지 않는다(추가 이동
     지령 없음).
하위 액션서버는 취소(cancel)를 지원하지 않는다(Task 10b 리포트 우려사항 #5 — 로직 변경
없이 이식된 원본에도 취소 개념이 없었음). 따라서 하위 서버가 자신의 내부 타임아웃보다
먼저 진짜로 "멎어"(hang) 있다면 orchestrator가 능동적으로 멈춰 세울 방법은 없다 — 이 경우
`_call_action`의 result_timeout(아래)이 지나서야 FAILED로 전이한다(진짜 하드웨어 정지가
아니라 "더 이상 지령을 보내지 않음"에 그침). 이 한계는 안전상 후속(P4, safety_monitor
정식화)에서 다뤄야 한다.

## ★동시성 패턴(필수, Task 7/8/10과 동일 교정 패턴)
이 노드는 액션 서버(execute_parking_task)이면서 그 콜백 안에서 4개 액션 클라이언트를
순차 호출·대기한다 — 재진입 spin 데드락 위험(Task 7에서 발견된 Critical 결함과 동일
원인: 콜백 안에서 재진입 spin_until_future_complete를 쓰면, 콜백과 클라이언트가 같은
콜백그룹·실행자를 공유할 때 클라이언트 응답 콜백이 영원히 스케줄되지 않는다). 이를
피하기 위해:
  - 액션서버(execute_parking_task) + 4개 액션 클라이언트(detect_vehicle/navigate_to_pose/
    align_vehicle/control_lift)를 모두 같은 ReentrantCallbackGroup에 배치.
  - 액션 결과 대기는 spin_until_future_complete류의 재진입 spin 없이, send_goal_async→
    goal_handle→get_result_async의 각 future를 time.monotonic() 데드라인 + sleep(0.05)
    non-respin 폴링으로 기다린다(`_call_action`, 아래).
  - main()은 rclpy.spin 대신 MultiThreadedExecutor(num_threads=4)로 스핀 — 이 스레드가
    폴링(블로킹)하는 동안 다른 실행자 스레드가 클라이언트 응답 콜백을 처리할 수 있어야
    future가 실제로 완료된다.
  - 각 액션 서버는 wait_for_server(timeout)로 기동 확인 후 없으면 그 단계를 실패로
    처리(FAILED)한다. wait_for_server/wait_for_service 자체는 그래프 이벤트 기반 자체
    폴링이라 재진입 spin이 필요 없다(lift_action_server의 기존 관례와 동일 — Task 10a
    리포트 참고).

## 결과 대기 상한(안전망)
`_call_action`의 `result_timeout`은 "정상적으로는 걸리지 않아야 하는" 안전망이다 — 하위
액션서버(align/navigate)가 완전히 멎었을 때만 발동해야 하므로, 각 하위 서버가 내부적으로
블로킹될 수 있는 최대 시간(formation_motion.py/lift_action_server.py의 타임아웃 상수) 이상
여유 있게 잡는다(아래 상수 정의부에 근거 명시).

## RETURNING 도크 pose — best-effort(TODO Task 12)
West 도크 복귀 목표(map 프레임)는 미션 상수(formation_motion.py: DOCK_X=-15.3,
LANE_Z_REAR=-1.5, LANE_Z_FRONT=1.5, USD 프레임)를 frame_transform 규약(y_map=-z_usd)
그대로 환산한 근사값이다(아래 DOCK_* 상수 정의부 참고) — carry_to/carry_rotate_to와 동일한
"신규 best-effort" 성격이라 실제 정위치는 Task 12에서 Isaac GUI로 사람이 확인해야 한다.
"""
import time

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Pose
from rclpy.action import ActionClient, ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from nav2_msgs.action import NavigateToPose

from parking_robot_interfaces.action import (
    AlignVehicle, ControlLift, DetectVehicle, ExecuteParkingTask,
)
from parking_robot_interfaces.msg import ObstacleAlert, TaskState

# ---- 순수 전이 테이블(TDD 대상) ----------------------------------------------------
TRANSITIONS = {
    "SEARCHING": "APPROACHING", "APPROACHING": "PICKED_UP", "PICKED_UP": "MOVING",
    "MOVING": "ARRIVED", "ARRIVED": "PARKED", "PARKED": "RETURNING",
    "RETURNING": "DONE", "DONE": "DONE",
}


def next_state(current):
    """TRANSITIONS 조회. DONE은 고정점, 미정의(FAILED 포함) 상태는 모두 FAILED로 수렴."""
    return TRANSITIONS.get(current, "FAILED")


# TRANSITIONS를 SEARCHING부터 따라간 전체 시퀀스(고정, 8단계) — 액션이 실제로 호출되는
# 전이는 SEARCHING..RETURNING의 7개, DONE은 종료 마커.
PLAN_STEPS = ("SEARCHING", "APPROACHING", "PICKED_UP", "MOVING", "ARRIVED",
              "PARKED", "RETURNING", "DONE")


def plan_steps(slot_pose):
    """P1 계획은 고정 8단계라 어느 슬롯이든 동일 시퀀스를 반환한다. slot_pose는 인터페이스
    계약(향후 슬롯/요청유형별 분기 확장 여지 — 예: EXIT 플로우가 다른 시퀀스를 쓰게 될
    경우)을 위해 받되 P1(ENTRY 전용)에서는 사용하지 않는다."""
    return list(PLAN_STEPS)


# ---- task_state에 실릴 사람이 읽는 진행 문장(DONE/FAILED는 _on_execute_parking_task가
# 별도로 채움 — DONE="주차 완료", FAILED=구체 실패 사유) ----
_STEP_MESSAGES = {
    "SEARCHING": "차량 탐색 중",
    "APPROACHING": "차량 하부 진입 중(픽업 정렬)",
    "PICKED_UP": "리프트 상승 완료(픽업)",
    "MOVING": "슬롯으로 이동 중",
    "ARRIVED": "슬롯 방향 정렬 중",
    "PARKED": "안착 완료(리프트 하강)",
    "RETURNING": "대기 도크로 복귀 중",
}

# ---- ★동시성 패턴 상수 ----
POLL_INTERVAL = 0.05                # 브리프 지시: non-respin 폴링 간격
ACTION_SERVER_WAIT_TIMEOUT = 5.0    # wait_for_server 상한(task_dispatcher/lift_action_server와 동일 관례)
SEND_GOAL_TIMEOUT = 10.0            # send_goal_async 수락/거부 핸드셰이크 상한(정상은 즉시 응답)

# get_result_async 대기 상한 — 하위 서버의 내부 최대 블로킹 시간 이상으로 여유 있게:
DETECT_RESULT_TIMEOUT = 15.0        # vehicle_detection_node: 즉시 반환(P1 스텁) + 여유
LIFT_RESULT_TIMEOUT = 20.0          # lift_action_server: wait_for_service 5s + 응답대기 6s = 11s + 여유
# formation_motion: carry_to 300s(CARRY_TO_TIMEOUT) 또는 goto_xz/carry_rotate_to
# 90s(STEP_TIMEOUT) 중 최댓값 + 여유.
NAVIGATE_RESULT_TIMEOUT = 330.0
# formation_motion.pickup_sequence 내부 단계별 타임아웃 총합(게이트 300 +
# goto_xz/rotate/ingress 각 최대치 ×2단계 ≈ 890s 최악치) + 여유.
ALIGN_RESULT_TIMEOUT = 900.0

# ---- RETURNING: West 도크 복귀 목표(map 프레임) — best-effort 근사(TODO Task 12) ----
# formation_motion.py 미션 상수 DOCK_X=-15.3, LANE_Z_REAR=-1.5, LANE_Z_FRONT=1.5(USD
# 프레임)를 frame_transform 규약(map_to_usd: x_usd=x_map, z_usd=-y_map)의 역으로 map
# 프레임에 맞춰 두면, navigate_action_server가 다시 map_to_usd로 되돌릴 때 정확히 미션
# 도크 좌표로 상쇄되어 수렴한다(rear: x_usd=-15.3,z_usd=-(1.5)=-1.5=LANE_Z_REAR / front:
# x_usd=-15.3,z_usd=-(-1.5)=1.5=LANE_Z_FRONT). 다만 이 좌표가 실제 도크 "정위치"(안착
# 자세·장애물 없는 진입 여부)까지 보장하지는 않는다 — carry_to와 동일한 best-effort
# 한계이며, Isaac GUI에서 사람이 관찰·조정해야 한다(Task 12).
DOCK_X_MAP = -15.3
DOCK_Y_REAR_MAP = 1.5      # rear 차로: LANE_Z_REAR=-1.5(USD) → y_map = -(-1.5) = 1.5
DOCK_Y_FRONT_MAP = -1.5    # front 차로: LANE_Z_FRONT=1.5(USD) → y_map = -(1.5) = -1.5


def _dock_pose(y_map):
    pose = Pose()
    pose.position.x = DOCK_X_MAP
    pose.position.y = y_map
    pose.orientation.w = 1.0
    return pose


class RobotTaskOrchestratorNode(Node):

    def __init__(self):
        super().__init__('robot_task_orchestrator')

        grp = ReentrantCallbackGroup()

        self._task_state_pub = self.create_publisher(TaskState, 'task_state', 10)
        self._obstacle_alert_sub = self.create_subscription(
            ObstacleAlert, 'obstacle_alert', self._on_obstacle_alert, 10, callback_group=grp)

        self._execute_task_server = ActionServer(
            self, ExecuteParkingTask, 'execute_parking_task', self._on_execute_parking_task,
            callback_group=grp)

        self._detect_client = ActionClient(
            self, DetectVehicle, 'detect_vehicle', callback_group=grp)
        self._navigate_client = ActionClient(
            self, NavigateToPose, 'navigate_to_pose', callback_group=grp)
        self._align_client = ActionClient(
            self, AlignVehicle, 'align_vehicle', callback_group=grp)
        self._lift_client = ActionClient(
            self, ControlLift, 'control_lift', callback_group=grp)

        self._emergency_stop = False

        self.get_logger().info('robot_task_orchestrator node started')

    def _on_obstacle_alert(self, msg):
        # TODO(SR-10/P4): 긴급정지 상태 반영, 장애물 해소 시 작업 재개 신호 처리는
        # safety_monitor 정식화(P4) 범위 — 이번 태스크(P1 상태머신 골격)는 플래그 보관만.
        self._emergency_stop = msg.obstacle_detected

    # ---- ★동시성 패턴 핵심: 재진입 spin 없는 액션 호출 ----
    def _call_action(self, client, goal, *, label, wait_timeout, result_timeout):
        """send_goal_async → goal_handle → get_result_async를 non-respin 폴링으로 대기.

        반환 (result, status, reason):
          - 성공: (액션별 Result 메시지, GoalStatus 정수, None)
          - 실패(서버 미기동/거부/타임아웃): (None, None, 사유 문자열)
        """
        if not client.wait_for_server(timeout_sec=wait_timeout):
            reason = f'{label} 서버 미기동({wait_timeout:.0f}s)'
            self.get_logger().warn(reason)
            return None, None, reason

        send_fut = client.send_goal_async(goal)
        deadline = time.monotonic() + SEND_GOAL_TIMEOUT
        while not send_fut.done() and time.monotonic() < deadline:
            time.sleep(POLL_INTERVAL)
        if not send_fut.done():
            reason = f'{label} goal 전송 타임아웃'
            self.get_logger().warn(reason)
            return None, None, reason

        goal_handle = send_fut.result()
        if goal_handle is None or not goal_handle.accepted:
            reason = f'{label} goal 거부'
            self.get_logger().warn(reason)
            return None, None, reason

        result_fut = goal_handle.get_result_async()
        deadline = time.monotonic() + result_timeout
        while not result_fut.done() and time.monotonic() < deadline:
            time.sleep(POLL_INTERVAL)
        if not result_fut.done():
            reason = f'{label} 결과 타임아웃({result_timeout:.0f}s)'
            self.get_logger().warn(reason)
            return None, None, reason

        response = result_fut.result()
        return response.result, response.status, None

    # ---- 단계별 액션 호출 래퍼: 모두 (ok, payload, reason) 3-tuple로 통일 ----
    def _call_detect_vehicle(self):
        result, _status, reason = self._call_action(
            self._detect_client, DetectVehicle.Goal(trigger=True),
            label='detect_vehicle',
            wait_timeout=ACTION_SERVER_WAIT_TIMEOUT, result_timeout=DETECT_RESULT_TIMEOUT)
        if result is None:
            return False, None, reason
        if not result.success:
            return False, None, 'detect_vehicle 실패(success=False)'
        return True, result.vehicle_info.pose, None

    def _call_align_vehicle(self, target_pose):
        result, _status, reason = self._call_action(
            self._align_client, AlignVehicle.Goal(target_pose=target_pose),
            label='align_vehicle',
            wait_timeout=ACTION_SERVER_WAIT_TIMEOUT, result_timeout=ALIGN_RESULT_TIMEOUT)
        if result is None:
            return False, None, reason
        if not result.success:
            return False, None, f'align_vehicle 실패(final_error={result.final_error:.3f})'
        return True, None, None

    def _call_control_lift(self, command):
        result, _status, reason = self._call_action(
            self._lift_client, ControlLift.Goal(command=command),
            label=f'control_lift[{command}]',
            wait_timeout=ACTION_SERVER_WAIT_TIMEOUT, result_timeout=LIFT_RESULT_TIMEOUT)
        if result is None:
            return False, None, reason
        if not result.success:
            return False, None, f'control_lift[{command}] 실패(support_state={result.support_state})'
        return True, None, None

    def _navigate_goal(self, pose, mode):
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose = pose
        goal.behavior_tree = mode
        return goal

    def _call_navigate(self, pose, mode):
        # NavigateToPose.Result에는 success 필드가 없다(std_msgs/Empty뿐, nav2 표준
        # 계약) — 성패는 액션 종단 상태(GoalStatus)로만 판정(navigate_action_server가
        # 수렴 시 succeed()/그 외 abort()로 인코딩, Task 10b 관례 그대로).
        result, status, reason = self._call_action(
            self._navigate_client, self._navigate_goal(pose, mode),
            label=f'navigate_to_pose[{mode}]',
            wait_timeout=ACTION_SERVER_WAIT_TIMEOUT, result_timeout=NAVIGATE_RESULT_TIMEOUT)
        if result is None:
            return False, None, reason
        if status != GoalStatus.STATUS_SUCCEEDED:
            return False, None, f'navigate_to_pose[{mode}] 실패(status={status})'
        return True, None, None

    def _call_return_to_dock(self):
        """RETURNING: rear → front 순으로 개별 West 도크 복귀(순차 — 동시 진입 시 충돌 회피).

        'return_rear'/'return_front' 모드는 navigate_action_server에서 통로 경유 L자 경로
        (슬롯→통로→도크)로 처리한다 — 슬롯에서 도크로 직선 이동하면 차량/벽을 관통하기 때문."""
        ok, _, reason = self._call_navigate(_dock_pose(DOCK_Y_REAR_MAP), 'return_rear')
        if not ok:
            return False, None, f'rear 복귀 실패: {reason}'
        ok, _, reason = self._call_navigate(_dock_pose(DOCK_Y_FRONT_MAP), 'return_front')
        if not ok:
            return False, None, f'front 복귀 실패: {reason}'
        return True, None, None

    # ---- task_state / feedback 발행 ----
    def _publish_task_state(self, robot_id, task_id, state, current_step):
        msg = TaskState()
        msg.robot_id = robot_id
        msg.task_id = task_id
        msg.state = state
        msg.current_step = current_step
        self._task_state_pub.publish(msg)

    def _publish_feedback(self, goal_handle, state, idx, total):
        fb = ExecuteParkingTask.Feedback()
        fb.current_step = state
        fb.progress = float(idx) / float(total) if total else 1.0
        goal_handle.publish_feedback(fb)

    # ---- 상태머신 실행부 ----
    def _on_execute_parking_task(self, goal_handle):
        goal = goal_handle.request
        robot_id = goal.leader_robot_id or 'robot_rear'
        steps = plan_steps(goal.slot_pose)   # ("SEARCHING", ..., "DONE") 고정 8단계
        total = len(steps) - 1               # 실제 액션 호출 전이 수(7)
        vehicle_pose = None
        idx = 0
        state = steps[0]                     # "SEARCHING"

        self.get_logger().info(
            f'execute_parking_task 시작: task_id={goal.task_id} slot_id={goal.slot_id}')

        # goal/vehicle_pose(가변)를 클로저로 참조 — 매 호출(goal_handle)마다 새로 만들어져
        # 동시 실행 중인 다른 goal과 상태를 공유하지 않는다(인스턴스 속성에 담지 않음).
        handlers = {
            "SEARCHING": self._call_detect_vehicle,
            "APPROACHING": lambda: self._call_align_vehicle(vehicle_pose),
            "PICKED_UP": lambda: self._call_control_lift('UP'),
            "MOVING": lambda: self._call_navigate(goal.slot_pose, 'carry'),
            "ARRIVED": lambda: self._call_navigate(goal.slot_pose, 'rotate'),
            "PARKED": lambda: self._call_control_lift('DOWN'),
            "RETURNING": self._call_return_to_dock,
        }

        fail_reason = ''
        while state not in ('DONE', 'FAILED'):
            self._publish_task_state(robot_id, goal.task_id, state, _STEP_MESSAGES[state])
            self._publish_feedback(goal_handle, state, idx, total)

            ok, payload, reason = handlers[state]()
            if state == 'SEARCHING' and ok:
                vehicle_pose = payload

            if ok:
                idx += 1
                state = next_state(state)
            else:
                fail_reason = f'{state}: {reason}'
                state = 'FAILED'

        final_message = '주차 완료' if state == 'DONE' else fail_reason
        self._publish_task_state(robot_id, goal.task_id, state, final_message)
        self._publish_feedback(goal_handle, state, idx, total)
        self.get_logger().info(
            f'execute_parking_task 종료: task_id={goal.task_id} state={state} '
            f'message={final_message}')

        # align/lift/detect 액션서버와 동일 관례: 액션 자체는 항상 succeed()하고
        # 성패는 result.success 필드로 전달한다(원 스텁도 이 관례를 이미 따르고 있었음).
        goal_handle.succeed()
        result = ExecuteParkingTask.Result()
        result.success = (state == 'DONE')
        result.message = final_message
        return result


def main(args=None):
    rclpy.init(args=args)
    node = RobotTaskOrchestratorNode()
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
