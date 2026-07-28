"""실행 모드 설정.

환경변수 PARKING_MODE 로 데이터 소스를 선택한다.
  - mock (기본): 인메모리 시뮬레이션
  - ros2        : task_dispatcher(feature/parking-control) 연동

예)
  PARKING_MODE=mock python -m uvicorn main:app --port 8000
  PARKING_MODE=ros2 python -m uvicorn main:app --port 8000
"""

import os
from pathlib import Path

PARKING_MODE = os.getenv("PARKING_MODE", "mock").lower()

# prs  = parking_robot_system(feat/camera) 연동 (로봇 그룹 구분 없음, 단일 팀)
# dual = prs와 같은 로봇 스택에 붙되, 입차/출차 요청을 서로 다른 로봇 그룹
#        (= 서로 다른 Isaac Sim PC에 뜬 user_request_gateway_node 서비스)으로
#        분리 라우팅한다. 라우팅 표는 core/db.py(SQLite)에서 관리.
VALID_MODES = {"mock", "ros2", "prs", "dual"}

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
SLOT_OCCUPANCY_TOPIC = os.getenv(
    "PARKING_SLOT_OCCUPANCY_TOPIC", "/parking/slot_occupancy"
)
DISPATCH_SERVICE_TIMEOUT_SEC = float(os.getenv("PARKING_DISPATCH_TIMEOUT_SEC", "5.0"))

# task_dispatcher가 쓰는 MySQL과 동일한 DB를 읽기 전용으로 폴링한다
# (로봇/슬롯/작업 목록을 조회하는 ROS2 서비스가 아직 없음 — dashboard.py와 동일한 방식).
DB_HOST = os.getenv("PARKING_DB_HOST", "localhost")
DB_USER = os.getenv("PARKING_DB_USER", "parking")
DB_PASSWORD = os.getenv("PARKING_DB_PASSWORD", "parking1234")
DB_NAME = os.getenv("PARKING_DB_NAME", "parking")
DB_POLL_INTERVAL_SEC = float(os.getenv("PARKING_DB_POLL_INTERVAL_SEC", "0.25"))

# ---------------------------------------------------------------------
# dual 모드 전용 설정. MySQL(위 DB_*)과는 무관한 별개의 경량 SQLite로,
# task_dispatcher DB가 아니라 "입차/출차 요청을 어느 로봇 그룹으로 보낼지"
# 라우팅 표 + 전달 이력만 담는다 (core/db.py).
# ---------------------------------------------------------------------
DB_SQLITE_PATH = os.getenv(
    "PARKING_SQLITE_PATH", str(Path(__file__).resolve().parent / "parking_control.db")
)
#: dispatch_parking_task 서비스 응답 대기 시간 (그룹별 공통 기본값).
DUAL_DISPATCH_TIMEOUT_SEC = float(os.getenv("PARKING_DUAL_DISPATCH_TIMEOUT_SEC", "5.0"))
