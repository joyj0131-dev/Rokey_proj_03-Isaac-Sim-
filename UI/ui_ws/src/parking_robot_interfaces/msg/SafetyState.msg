# 중앙 safety_supervisor가 소유하는 관제 안전 상태.
# transient-local QoS로 발행하여 재기동한 노드도 마지막 상태를 즉시 받는다.
string state
bool motion_allowed
uint64 stop_epoch
string reason
string operator_id
string inspection_note
string[] affected_task_ids
string[] blockers
string updated_at
