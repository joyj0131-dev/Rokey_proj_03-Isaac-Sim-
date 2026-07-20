"""실행 모드 설정.

환경변수 PARKING_MODE 로 데이터 소스를 선택한다.
  - mock (기본): 인메모리 시뮬레이션
  - ros2        : ROS2 Bridge (task_dispatcher 스펙 확정 후 구현 예정)

예)
  PARKING_MODE=mock python -m uvicorn main:app --port 8000
"""

import os

PARKING_MODE = os.getenv("PARKING_MODE", "mock").lower()

VALID_MODES = {"mock", "ros2"}

if PARKING_MODE not in VALID_MODES:
    raise ValueError(
        f"PARKING_MODE 값이 잘못되었습니다: {PARKING_MODE!r} "
        f"(가능한 값: {', '.join(sorted(VALID_MODES))})"
    )
