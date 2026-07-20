# Hyundai WIA-style Parking Robot — Bearing Roller + Arm-tip Caster Revision

최종 수정 버전입니다.

## 핵심 구조
- 4개의 swing arm은 `base_link`에 직접 연결됩니다.
- 각 팔은 평소 0 rad에서 본체와 나란히 접혀 있습니다.
- 차량 바퀴를 잡을 때 4개 팔이 Z축 revolute joint로 펼쳐집니다.
- 각 팔의 타이어 접촉부에는 자유회전 `bearing_roller_*`가 있어 타이어 표면을 따라 굴러갑니다.
- 각 팔의 맨 끝에는 의자 바퀴 형태의 **passive swivel caster**가 추가되었습니다.

## 캐스터 구조
각 팔마다 캐스터 1개가 있고, 캐스터마다 2개 passive joint가 있습니다.

1. `caster_*_swivel_joint`
   - 팔 끝에 연결
   - Z축 `continuous`
   - 캐스터가 진행 방향에 맞춰 자유롭게 방향을 돌림

2. `caster_*_wheel_joint`
   - 캐스터 포크와 작은 지지 바퀴 연결
   - Y축 `continuous`
   - 지면 위에서 수동으로 굴러감

## 목적
캐스터는 차량 타이어를 들어 올리는 장치가 아닙니다. 타이어는 기존의 bearing roller와 swing arm의 파지 동작으로 올라갑니다. 캐스터는 팔 끝을 지면에서 지지해 차량 하중 일부를 바닥으로 전달하고, 팔 자체와 팔 pivot에 걸리는 굽힘 모멘트를 줄이기 위한 보조 지지 바퀴입니다.

## 기본 치수
- 캐스터 휠 반지름: 0.028 m
- 캐스터 휠 폭: 0.022 m
- 팔 pivot 기준 캐스터 swivel 위치: local X = 0.50 m
- nominal wheel center height: 약 0.028 m

## Isaac Sim 권장 설정
- `caster_*_swivel_joint`: drive 없음, 낮은 damping/friction
- `caster_*_wheel_joint`: drive 없음, 낮은 damping/friction
- caster wheel ↔ floor: 미끄러지지 않으면서 잘 구를 수 있도록 중간 정도 마찰부터 튜닝
- bearing roller ↔ tire: 롤러가 타이어를 따라 회전하면서 하단으로 진입하도록 접촉 마찰 튜닝
- 팔을 펼친 상태에서 캐스터 휠이 지면에 아주 약하게 접촉하도록 높이를 맞추는 것이 중요

## 검증
- XML parse: PASS
- Links: 25
- Joints: 24
- Bearing roller passive joints: 4
- Caster swivel passive joints: 4
- Caster wheel passive joints: 4
