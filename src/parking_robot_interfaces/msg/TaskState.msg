# [초안] robot_task_orchestrator가 발행하는 진행 상황. task_state 토픽에서 사용.
string robot_id
string task_id
string state          # DETECTING, NAVIGATING, ALIGNING, LIFTING, DONE, FAILED
string current_step
