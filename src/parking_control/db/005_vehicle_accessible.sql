-- 이미 001_schema.sql로 만들어둔 기존 DB에 "교통약자 배려 차량 여부" 컬럼을 추가한다.
-- (001_schema.sql 자체도 함께 갱신해 두었으므로, 새로 스키마를 만드는
--  경우에는 이 파일을 실행할 필요가 없다.)
-- 적용: mysql -u parking -p parking < 005_vehicle_accessible.sql
--
-- 배경: 웹 UI에서 "교통약자 배려 차량" 체크박스를 켜고 입차 요청하면
-- vehicle_type='ACCESSIBLE'로 저장하고, 그 차량은 배려석(A1/A2)으로만,
-- 나머지(STANDARD)는 일반 슬롯으로만 배정한다(서로 안 섞임).
-- requires_accessible_slot 같은 별도 bool은 안 둔다 — vehicle_type 하나로
-- 이미 판단 가능한 값을 두 컬럼에 중복 저장하면 나중에 어긋날 위험만 커진다.
-- 우선순위(service_priority)도 실제로 쓰는 로직이 없어 지금은 넣지 않는다.

ALTER TABLE vehicles
    ADD COLUMN vehicle_type ENUM('STANDARD', 'ACCESSIBLE') NOT NULL DEFAULT 'STANDARD';
