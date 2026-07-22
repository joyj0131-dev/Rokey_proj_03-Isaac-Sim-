# 주차로봇 관제 웹 UI

## v0.3 변경 사항 (구조 리팩터링 + 에러/장애물 UI)

### 구조
- 비즈니스 로직을 웹 계층에서 분리 (ROS2 연동 대비)
  - `core/models.py` — 공용 데이터 모델
  - `core/state_store.py` — 락으로 보호되는 스레드 안전 상태 저장소
  - `core/datasource.py` — DataSource 추상 인터페이스
  - `sources/mock_source.py` — Mock 구현체 (기존 로직 이관)
  - `config.py` — `PARKING_MODE` 환경변수 (mock / ros2)
  - `main.py` — FastAPI 라우팅만 담당
- ros2 모드는 task_dispatcher 인터페이스 확정 후 `sources/ros2_source.py`로 추가 예정

### 기능
- 작업 단계를 dispatcher 예상 흐름에 맞춰 6단계로 확장:
  요청 대기 → 로봇 할당 → 차량 접근 → 차량 리프트 → 주차 위치 이동 → 완료
- 에러 · 장애물 감지 패널 추가 (활성 알림 표시, 해제 버튼)
- Mock 이벤트 시뮬레이션: 장애물 감지 / 로봇 오류 버튼
- 로봇 오류 상태에서는 해당 작업 진행 불가, 알림 해제 시 복구
- 헤더 시스템 상태 배지 동적화 (정상 / 주의 필요 / 시스템 오류)
- ROS2 모드에서는 Mock 제어 버튼이 자동으로 숨겨지는 구조

### 신규 API
- `GET  /api/system` — 실행 모드, Mock 제어 여부, 시스템 상태
- `POST /api/mock/obstacle` — 장애물 감지 이벤트 발생 (mock 전용)
- `POST /api/mock/robot-error` — 로봇 오류 이벤트 발생 (mock 전용)
- `POST /api/alerts/{id}/resolve` — 알림 해제

## 이전 변경 사항

- 전체 화면을 밝은 회색 및 흰색 중심으로 변경
- 헤더와 시스템 상태 배지 정리
- 요약 카드에 상태별 강조 요소 추가
- 입력 폼과 버튼 가독성 개선
- 로봇 카드에 아이콘, 배터리 정보, 현재 작업 정보 정리
- 주차면 카드에 상태별 좌측 색상 표시 추가
- 작업 요청이 없을 때 빈 상태 안내 화면 추가
- 반응형 화면 개선
- 실행 위치와 무관하게 정적 파일 경로를 찾도록 `main.py` 보완

## Windows 실행

```powershell
cd parking_control_mvp_light
python -m venv .venv
.\.venv\Scripts\Activate.ps1
pip install -r requirements.txt
python -m uvicorn main:app --reload --host 0.0.0.0 --port 8000
```

PowerShell 실행 정책 오류가 발생하면:

```powershell
Set-ExecutionPolicy -Scope Process -ExecutionPolicy Bypass
.\.venv\Scripts\Activate.ps1
```

브라우저:

```text
http://127.0.0.1:8000
```

API 문서:

```text
http://127.0.0.1:8000/docs
```

## Ubuntu 실행

```bash
cd parking_control_mvp_light
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
python3 -m uvicorn main:app --reload --host 0.0.0.0 --port 8000
```
