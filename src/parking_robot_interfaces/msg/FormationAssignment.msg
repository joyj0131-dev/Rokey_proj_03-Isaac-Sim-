# [초안, Team A 2026-07-21] task_dispatcher -> 로봇별 formation_gap_controller 배정.
# formation_assignment 토픽(공용, 네임스페이스 없음)에서 사용한다.
#
# formation_gap_controller는 로봇마다 하나씩 항상 떠 있는 상시 노드다(실제
# 로봇의 온보드 소프트웨어처럼). 작업이 없을 때는 idle 상태로 아무것도
# 발행하지 않고 대기하다가, 이 메시지로 자기 robot_id에 해당하는 배정이
# 오면 그 순간부터 active=true인 동안만 간격유지/공동정지 로직을 켠다.
#
# 파트너 odom 토픽 이름은 "/<partner_robot_id>/odom" 컨벤션을 그대로
# 따른다고 가정한다 — 로봇마다 자기 네임스페이스 아래 odom/cmd_vel을
# 갖는다는 게 이 프로젝트의 다중로봇 토픽 규칙이다.
string robot_id          # 이 배정의 대상 로봇
string task_id           # active=false일 때는 의미 없음
string role              # leader | follower
string partner_robot_id
bool active               # false면 이 로봇은 idle로 복귀
