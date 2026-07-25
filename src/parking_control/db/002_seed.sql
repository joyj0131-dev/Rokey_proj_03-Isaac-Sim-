-- generate_map.py가 자동 생성한 시드. 손으로 편집하지 말 것.
-- 적용: mysql -u parking -p parking < 002_seed.sql

INSERT INTO zones (zone_id) VALUES ('ZIN01') ON DUPLICATE KEY UPDATE zone_id = zone_id;
INSERT INTO zones (zone_id) VALUES ('ZIN02') ON DUPLICATE KEY UPDATE zone_id = zone_id;
INSERT INTO zones (zone_id) VALUES ('ZIN03') ON DUPLICATE KEY UPDATE zone_id = zone_id;
INSERT INTO zones (zone_id) VALUES ('ZIN_A1') ON DUPLICATE KEY UPDATE zone_id = zone_id;
INSERT INTO zones (zone_id) VALUES ('ZIN_A1_dock') ON DUPLICATE KEY UPDATE zone_id = zone_id;
INSERT INTO zones (zone_id) VALUES ('ZIN_A2') ON DUPLICATE KEY UPDATE zone_id = zone_id;
INSERT INTO zones (zone_id) VALUES ('ZIN_A2_dock') ON DUPLICATE KEY UPDATE zone_id = zone_id;
INSERT INTO zones (zone_id) VALUES ('ZIN_A3') ON DUPLICATE KEY UPDATE zone_id = zone_id;
INSERT INTO zones (zone_id) VALUES ('ZIN_A3_dock') ON DUPLICATE KEY UPDATE zone_id = zone_id;
INSERT INTO zones (zone_id) VALUES ('ZIN_CROSS') ON DUPLICATE KEY UPDATE zone_id = zone_id;
INSERT INTO zones (zone_id) VALUES ('ZOUT01') ON DUPLICATE KEY UPDATE zone_id = zone_id;
INSERT INTO zones (zone_id) VALUES ('ZOUT02') ON DUPLICATE KEY UPDATE zone_id = zone_id;
INSERT INTO zones (zone_id) VALUES ('ZOUT03') ON DUPLICATE KEY UPDATE zone_id = zone_id;
INSERT INTO zones (zone_id) VALUES ('ZOUT_A1') ON DUPLICATE KEY UPDATE zone_id = zone_id;
INSERT INTO zones (zone_id) VALUES ('ZOUT_A1_dock') ON DUPLICATE KEY UPDATE zone_id = zone_id;
INSERT INTO zones (zone_id) VALUES ('ZOUT_A2') ON DUPLICATE KEY UPDATE zone_id = zone_id;
INSERT INTO zones (zone_id) VALUES ('ZOUT_A2_dock') ON DUPLICATE KEY UPDATE zone_id = zone_id;
INSERT INTO zones (zone_id) VALUES ('ZOUT_A3') ON DUPLICATE KEY UPDATE zone_id = zone_id;
INSERT INTO zones (zone_id) VALUES ('ZOUT_A3_dock') ON DUPLICATE KEY UPDATE zone_id = zone_id;
INSERT INTO zones (zone_id) VALUES ('ZOUT_CROSS') ON DUPLICATE KEY UPDATE zone_id = zone_id;

INSERT INTO parking_slots (slot_id, x, y, is_accessible) VALUES ('A1', 2.8, -0.0, FALSE) ON DUPLICATE KEY UPDATE x = VALUES(x), y = VALUES(y), is_accessible = VALUES(is_accessible);
INSERT INTO parking_slots (slot_id, x, y, is_accessible) VALUES ('A2', 6.2, -0.0, FALSE) ON DUPLICATE KEY UPDATE x = VALUES(x), y = VALUES(y), is_accessible = VALUES(is_accessible);
INSERT INTO parking_slots (slot_id, x, y, is_accessible) VALUES ('A3', 9.6, -0.0, FALSE) ON DUPLICATE KEY UPDATE x = VALUES(x), y = VALUES(y), is_accessible = VALUES(is_accessible);

INSERT INTO parking_lot_edges (u, v, dist_m, zone_id) VALUES ('entry_gate', 'entry_outer', 8.0, 'ZIN01') ON DUPLICATE KEY UPDATE dist_m = VALUES(dist_m), zone_id = VALUES(zone_id);
INSERT INTO parking_lot_edges (u, v, dist_m, zone_id) VALUES ('entry_gate', 'entry_wait', 4.5, 'ZIN02') ON DUPLICATE KEY UPDATE dist_m = VALUES(dist_m), zone_id = VALUES(zone_id);
INSERT INTO parking_lot_edges (u, v, dist_m, zone_id) VALUES ('entry_j0', 'entry_wait', 4.305, 'ZIN03') ON DUPLICATE KEY UPDATE dist_m = VALUES(dist_m), zone_id = VALUES(zone_id);
INSERT INTO parking_lot_edges (u, v, dist_m, zone_id) VALUES ('entry_a1', 'entry_j0', 7.0, 'ZIN_A1') ON DUPLICATE KEY UPDATE dist_m = VALUES(dist_m), zone_id = VALUES(zone_id);
INSERT INTO parking_lot_edges (u, v, dist_m, zone_id) VALUES ('A1', 'entry_a1', 5.3, 'ZIN_A1_dock') ON DUPLICATE KEY UPDATE dist_m = VALUES(dist_m), zone_id = VALUES(zone_id);
INSERT INTO parking_lot_edges (u, v, dist_m, zone_id) VALUES ('entry_a2', 'entry_j0', 10.4, 'ZIN_A2') ON DUPLICATE KEY UPDATE dist_m = VALUES(dist_m), zone_id = VALUES(zone_id);
INSERT INTO parking_lot_edges (u, v, dist_m, zone_id) VALUES ('A2', 'entry_a2', 5.3, 'ZIN_A2_dock') ON DUPLICATE KEY UPDATE dist_m = VALUES(dist_m), zone_id = VALUES(zone_id);
INSERT INTO parking_lot_edges (u, v, dist_m, zone_id) VALUES ('entry_a3', 'entry_j0', 13.8, 'ZIN_A3') ON DUPLICATE KEY UPDATE dist_m = VALUES(dist_m), zone_id = VALUES(zone_id);
INSERT INTO parking_lot_edges (u, v, dist_m, zone_id) VALUES ('A3', 'entry_a3', 5.3, 'ZIN_A3_dock') ON DUPLICATE KEY UPDATE dist_m = VALUES(dist_m), zone_id = VALUES(zone_id);
INSERT INTO parking_lot_edges (u, v, dist_m, zone_id) VALUES ('exit_j0', 'exit_wait', 4.305, 'ZOUT01') ON DUPLICATE KEY UPDATE dist_m = VALUES(dist_m), zone_id = VALUES(zone_id);
INSERT INTO parking_lot_edges (u, v, dist_m, zone_id) VALUES ('exit_gate', 'exit_wait', 4.5, 'ZOUT02') ON DUPLICATE KEY UPDATE dist_m = VALUES(dist_m), zone_id = VALUES(zone_id);
INSERT INTO parking_lot_edges (u, v, dist_m, zone_id) VALUES ('exit_gate', 'exit_outer', 8.0, 'ZOUT03') ON DUPLICATE KEY UPDATE dist_m = VALUES(dist_m), zone_id = VALUES(zone_id);
INSERT INTO parking_lot_edges (u, v, dist_m, zone_id) VALUES ('exit_a1', 'exit_j0', 7.0, 'ZOUT_A1') ON DUPLICATE KEY UPDATE dist_m = VALUES(dist_m), zone_id = VALUES(zone_id);
INSERT INTO parking_lot_edges (u, v, dist_m, zone_id) VALUES ('A1', 'exit_a1', 5.3, 'ZOUT_A1_dock') ON DUPLICATE KEY UPDATE dist_m = VALUES(dist_m), zone_id = VALUES(zone_id);
INSERT INTO parking_lot_edges (u, v, dist_m, zone_id) VALUES ('exit_a2', 'exit_j0', 10.4, 'ZOUT_A2') ON DUPLICATE KEY UPDATE dist_m = VALUES(dist_m), zone_id = VALUES(zone_id);
INSERT INTO parking_lot_edges (u, v, dist_m, zone_id) VALUES ('A2', 'exit_a2', 5.3, 'ZOUT_A2_dock') ON DUPLICATE KEY UPDATE dist_m = VALUES(dist_m), zone_id = VALUES(zone_id);
INSERT INTO parking_lot_edges (u, v, dist_m, zone_id) VALUES ('exit_a3', 'exit_j0', 13.8, 'ZOUT_A3') ON DUPLICATE KEY UPDATE dist_m = VALUES(dist_m), zone_id = VALUES(zone_id);
INSERT INTO parking_lot_edges (u, v, dist_m, zone_id) VALUES ('A3', 'exit_a3', 5.3, 'ZOUT_A3_dock') ON DUPLICATE KEY UPDATE dist_m = VALUES(dist_m), zone_id = VALUES(zone_id);
INSERT INTO parking_lot_edges (u, v, dist_m, zone_id) VALUES ('crossing_entry', 'entry_wait', 6.156, 'ZIN_CROSS') ON DUPLICATE KEY UPDATE dist_m = VALUES(dist_m), zone_id = VALUES(zone_id);
INSERT INTO parking_lot_edges (u, v, dist_m, zone_id) VALUES ('crossing_entry', 'dock_entry_1', 4.727, NULL) ON DUPLICATE KEY UPDATE dist_m = VALUES(dist_m), zone_id = VALUES(zone_id);
INSERT INTO parking_lot_edges (u, v, dist_m, zone_id) VALUES ('crossing_entry', 'dock_entry_2', 4.852, NULL) ON DUPLICATE KEY UPDATE dist_m = VALUES(dist_m), zone_id = VALUES(zone_id);
INSERT INTO parking_lot_edges (u, v, dist_m, zone_id) VALUES ('crossing_exit', 'exit_wait', 6.156, 'ZOUT_CROSS') ON DUPLICATE KEY UPDATE dist_m = VALUES(dist_m), zone_id = VALUES(zone_id);
INSERT INTO parking_lot_edges (u, v, dist_m, zone_id) VALUES ('crossing_exit', 'dock_exit_1', 4.727, NULL) ON DUPLICATE KEY UPDATE dist_m = VALUES(dist_m), zone_id = VALUES(zone_id);
INSERT INTO parking_lot_edges (u, v, dist_m, zone_id) VALUES ('crossing_exit', 'dock_exit_2', 4.852, NULL) ON DUPLICATE KEY UPDATE dist_m = VALUES(dist_m), zone_id = VALUES(zone_id);
