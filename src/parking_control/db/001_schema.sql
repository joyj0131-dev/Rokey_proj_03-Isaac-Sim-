-- 주차로봇 시스템 관제 DB 스키마 (MySQL 8.0)
-- 적용: mysql -u parking -p parking < 001_schema.sql
-- 생성 순서 = 외래키 의존성 순서. 상태 ENUM 철자는 ROS 인터페이스와 동일하게 유지한다.

CREATE TABLE IF NOT EXISTS zones (
    zone_id     VARCHAR(16) PRIMARY KEY,
    description VARCHAR(100) NULL
);

CREATE TABLE IF NOT EXISTS parking_slots (
    slot_id       VARCHAR(8) PRIMARY KEY,   -- 'A1'~'B8' (USD 라벨·그래프 노드 id와 동일 문자열)
    x             DECIMAL(7,3) NOT NULL,    -- ROS map 프레임 좌표 (m)
    y             DECIMAL(7,3) NOT NULL,
    is_accessible BOOLEAN NOT NULL DEFAULT FALSE,
    status        ENUM('EMPTY','RESERVED','OCCUPIED') NOT NULL DEFAULT 'EMPTY',
    updated_at    TIMESTAMP NOT NULL DEFAULT CURRENT_TIMESTAMP ON UPDATE CURRENT_TIMESTAMP
);

CREATE TABLE IF NOT EXISTS robots (
    robot_id        VARCHAR(16) PRIMARY KEY,
    status          ENUM('IDLE','BUSY','CHARGING','ERROR','OFFLINE') NOT NULL DEFAULT 'OFFLINE',
    x               DECIMAL(7,3) NULL,
    y               DECIMAL(7,3) NULL,
    battery_percent DECIMAL(5,2) NULL,
    target_node     VARCHAR(16) NULL,  -- 지금 이동 중인 다리(leg)의 목적지 노드. 대시보드가
                                        -- 이 값 + 현재 좌표로 "가야 할 경로"를 실시간 계산한다.
    updated_at      TIMESTAMP NOT NULL DEFAULT CURRENT_TIMESTAMP ON UPDATE CURRENT_TIMESTAMP
);

CREATE TABLE IF NOT EXISTS vehicles (
    vehicle_id    VARCHAR(32) PRIMARY KEY,
    length_m      DECIMAL(5,2) NULL,        -- VehicleInfo.msg의 인식 결과
    width_m       DECIMAL(5,2) NULL,
    height_m      DECIMAL(5,2) NULL,
    registered_at TIMESTAMP NOT NULL DEFAULT CURRENT_TIMESTAMP
);

-- vehicles/robots/parking_slots를 잇는 조인 테이블. 시스템의 작업 이력 원장.
CREATE TABLE IF NOT EXISTS tasks (
    task_id      VARCHAR(36) PRIMARY KEY,
    request_type ENUM('ENTRY','EXIT') NOT NULL,               -- RequestParkingTask.srv와 일치
    state        ENUM('WAITING','PROCESSING','DONE','FAILED') -- GetTaskStatus.srv와 일치
                 NOT NULL DEFAULT 'WAITING',
    vehicle_id   VARCHAR(32) NOT NULL,
    robot_id     VARCHAR(16) NULL,          -- 할당 전에는 NULL
    slot_id      VARCHAR(8)  NULL,          -- 슬롯 확정 전에는 NULL
    created_at   TIMESTAMP NOT NULL DEFAULT CURRENT_TIMESTAMP,
    updated_at   TIMESTAMP NOT NULL DEFAULT CURRENT_TIMESTAMP ON UPDATE CURRENT_TIMESTAMP,
    FOREIGN KEY (vehicle_id) REFERENCES vehicles(vehicle_id),
    FOREIGN KEY (robot_id)   REFERENCES robots(robot_id),
    FOREIGN KEY (slot_id)    REFERENCES parking_slots(slot_id)
);

-- 그래프 엣지 (참고·모니터링용 사본. 경로 계산의 원천은 parking_map.yaml)
CREATE TABLE IF NOT EXISTS parking_lot_edges (
    u       VARCHAR(16) NOT NULL,
    v       VARCHAR(16) NOT NULL,
    dist_m  DECIMAL(7,3) NOT NULL,
    zone_id VARCHAR(16) NULL,
    PRIMARY KEY (u, v),
    FOREIGN KEY (zone_id) REFERENCES zones(zone_id)
);

-- 존 락: zone_id가 PK라서 INSERT 성공 = 락 획득, 1062 중복 에러 = 락 실패.
-- 락 메커니즘 자체가 DB 제약조건이므로 코드 버그로 이중 획득이 불가능하다.
CREATE TABLE IF NOT EXISTS zone_locks (
    zone_id   VARCHAR(16) PRIMARY KEY,
    robot_id  VARCHAR(16) NOT NULL,
    locked_at TIMESTAMP NOT NULL DEFAULT CURRENT_TIMESTAMP,
    FOREIGN KEY (zone_id)  REFERENCES zones(zone_id),
    FOREIGN KEY (robot_id) REFERENCES robots(robot_id)
);
