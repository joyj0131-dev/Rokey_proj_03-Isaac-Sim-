"""실행 모드 설정.

환경변수 PARKING_MODE 로 데이터 소스를 선택한다.
  - mock (기본): 인메모리 시뮬레이션
  - ros2        : task_dispatcher(feature/parking-control) 연동

예)
  PARKING_MODE=mock python -m uvicorn main:app --port 8000
  PARKING_MODE=ros2 python -m uvicorn main:app --port 8000
"""

import os

PARKING_MODE = os.getenv("PARKING_MODE", "mock").lower()

VALID_MODES = {"mock", "ros2", "prs"}  # prs = parking_robot_system(feat/camera) 연동

if PARKING_MODE not in VALID_MODES:
    raise ValueError(
        f"PARKING_MODE 값이 잘못되었습니다: {PARKING_MODE!r} "
        f"(가능한 값: {', '.join(sorted(VALID_MODES))})"
    )

# ---------------------------------------------------------------------
# ros2 모드 전용 설정. task_dispatcher(Team A) 쪽 기본값과 동일하게 맞춘다.
# ---------------------------------------------------------------------
DISPATCH_SERVICE_NAME = os.getenv("PARKING_DISPATCH_SERVICE", "dispatch_parking_task")
OBSTACLE_ALERT_TOPIC = os.getenv("PARKING_OBSTACLE_TOPIC", "obstacle_alert")
TASK_STATE_TOPIC = os.getenv("PARKING_TASK_STATE_TOPIC", "task_state")
DISPATCH_SERVICE_TIMEOUT_SEC = float(os.getenv("PARKING_DISPATCH_TIMEOUT_SEC", "5.0"))

# task_dispatcher가 쓰는 MySQL과 동일한 DB를 읽기 전용으로 폴링한다
# (로봇/슬롯/작업 목록을 조회하는 ROS2 서비스가 아직 없음 — dashboard.py와 동일한 방식).
DB_HOST = os.getenv("PARKING_DB_HOST", "localhost")
DB_USER = os.getenv("PARKING_DB_USER", "parking")
DB_PASSWORD = os.getenv("PARKING_DB_PASSWORD", "parking1234")
DB_NAME = os.getenv("PARKING_DB_NAME", "parking")
DB_POLL_INTERVAL_SEC = float(os.getenv("PARKING_DB_POLL_INTERVAL_SEC", "1.5"))
