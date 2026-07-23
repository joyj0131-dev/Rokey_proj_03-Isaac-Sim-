"""순수 Python 코어 — ROS import 금지.

이 패키지 안의 모듈(graph, pathfinder, allocator, zone_lock, db)은
rclpy 없이 pytest만으로 검증 가능해야 한다.
"""
