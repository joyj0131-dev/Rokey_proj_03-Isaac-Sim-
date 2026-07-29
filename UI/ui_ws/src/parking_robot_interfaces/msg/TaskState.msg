# [초안] robot_task_orchestrator가 발행하는 진행 상황. task_state 토픽에서 사용.
#
# state 값 (2026-07-20 세분화, ENTRY/EXIT 공통):
#   SEARCHING    대상 차량을 찾는 중 (입고: 입고 예정 차량 / 출차: 해당 차량)
#   APPROACHING  차량 하부로 진입 중
#   PICKED_UP    차량 픽업 완료
#   MOVING       차량 이동 중
#   ARRIVED      목적지에 도착
#   PARKED       차량 입고 완료 (ENTRY 전용)
#   UNPARKED     차량 출차 완료 (EXIT 전용)
#   RETURNING    대기 장소 또는 충전 도크로 복귀 이동 중
#   DONE / FAILED 작업 종료
#
# state는 문자열 필드라 새 값 추가가 다른 구독자를 깨뜨리지 않는다(모르는
# 값은 조용히 무시됨). current_step은 화면에 그대로 보여줄 사람이 읽는 문장.
string robot_id
string task_id
string state
string current_step
