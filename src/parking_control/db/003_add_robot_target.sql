-- 이미 001_schema.sql로 만들어둔 기존 DB에 target_node 컬럼을 추가한다.
-- (001_schema.sql 자체도 함께 갱신해 두었으므로, 새로 스키마를 만드는
--  경우에는 이 파일을 실행할 필요가 없다.)
-- 적용: mysql -u parking -p parking < 003_add_robot_target.sql

ALTER TABLE robots
    ADD COLUMN target_node VARCHAR(16) NULL
    AFTER battery_percent;
