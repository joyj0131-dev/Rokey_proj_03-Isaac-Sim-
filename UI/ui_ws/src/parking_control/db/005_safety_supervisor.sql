-- 중앙 안전 상태와 관제 승인 이력.
-- 비상정지는 프로세스 메모리가 아니라 이 단일 행에 영속 저장한다.

CREATE TABLE IF NOT EXISTS safety_state (
    singleton_id       TINYINT PRIMARY KEY,
    state              VARCHAR(32) NOT NULL DEFAULT 'NORMAL',
    stop_epoch         BIGINT UNSIGNED NOT NULL DEFAULT 0,
    reason             VARCHAR(500) NOT NULL DEFAULT '',
    operator_id        VARCHAR(64) NOT NULL DEFAULT '',
    inspection_note    VARCHAR(1000) NOT NULL DEFAULT '',
    affected_task_ids  JSON NOT NULL,
    blockers           JSON NOT NULL,
    updated_at         TIMESTAMP NOT NULL DEFAULT CURRENT_TIMESTAMP
                       ON UPDATE CURRENT_TIMESTAMP,
    CONSTRAINT chk_safety_singleton CHECK (singleton_id = 1)
);

INSERT INTO safety_state (
    singleton_id, state, affected_task_ids, blockers
) VALUES (
    1, 'NORMAL', JSON_ARRAY(), JSON_ARRAY()
) ON DUPLICATE KEY UPDATE singleton_id = singleton_id;

CREATE TABLE IF NOT EXISTS safety_events (
    event_id      BIGINT UNSIGNED AUTO_INCREMENT PRIMARY KEY,
    stop_epoch    BIGINT UNSIGNED NOT NULL,
    event_type    VARCHAR(32) NOT NULL,
    state         VARCHAR(32) NOT NULL,
    operator_id   VARCHAR(64) NOT NULL DEFAULT '',
    note          VARCHAR(1000) NOT NULL DEFAULT '',
    details       JSON NOT NULL,
    created_at    TIMESTAMP NOT NULL DEFAULT CURRENT_TIMESTAMP
);
