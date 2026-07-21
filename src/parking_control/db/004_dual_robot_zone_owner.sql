-- 이미 001_schema.sql로 만들어둔 기존 DB에 "로봇 2대 팀 작업" 지원 컬럼을 추가한다.
-- (001_schema.sql 자체도 함께 갱신해 두었으므로, 새로 스키마를 만드는
--  경우에는 이 파일을 실행할 필요가 없다.)
-- 적용: mysql -u parking -p parking < 004_dual_robot_zone_owner.sql
--
-- 배경: hwia_parking_robot_final_caster는 팔 4개로 차량 바퀴를 집어 올리는
-- 구조라 구조상 로봇 2대(front/rear)가 항상 팀으로 일해야 차를 옮길 수 있다.
-- tasks에는 팔로워 로봇을, zone_locks에는 "팀(task) 단위 소유"를 추가한다.

ALTER TABLE tasks
    ADD COLUMN follower_robot_id VARCHAR(16) NULL AFTER robot_id,
    ADD FOREIGN KEY (follower_robot_id) REFERENCES robots(robot_id);

ALTER TABLE zone_locks
    MODIFY COLUMN robot_id VARCHAR(16) NULL,
    ADD COLUMN task_id VARCHAR(36) NULL AFTER robot_id,
    ADD FOREIGN KEY (task_id) REFERENCES tasks(task_id),
    ADD CONSTRAINT chk_zone_lock_owner CHECK (
        (robot_id IS NOT NULL AND task_id IS NULL) OR
        (robot_id IS NULL AND task_id IS NOT NULL)
    );
