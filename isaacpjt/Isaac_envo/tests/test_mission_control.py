import math, sys, pathlib
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))
from mission_control import body_twist_toward
from wheel_odometry import WheelOdometry

def test_target_ahead_gives_forward():
    # 로봇이 +Z(yaw=0)를 보고 있고 목표가 바로 앞(+Z)이면 전진(vx>0), 횡·회전≈0.
    vx, vy, wz, done = body_twist_toward((0.0, 0.0, 0.0), (0.0, 1.0, 0.0))
    assert vx > 0.1 and abs(vy) < 1e-6 and abs(wz) < 1e-6 and not done

def test_target_left_gives_left_strafe():
    # 자세규약 좌=(cosψ,-sinψ): ψ=0(+Z 향함)일 때 로봇의 좌측은 월드 +X.
    # 목표가 월드 +X 면 body +y(좌) 성분이 양(mecanum 횡이동).
    vx, vy, wz, done = body_twist_toward((0.0, 0.0, 0.0), (1.0, 0.0, 0.0))
    assert vy > 0.1

def test_yaw_error_sign():
    # 목표 yaw 가 현재보다 +면 wz>0(CCW).
    _, _, wz, _ = body_twist_toward((0.0, 0.0, 0.0), (0.0, 0.0, 30.0))
    assert wz > 0

def test_done_within_tolerance():
    _, _, _, done = body_twist_toward((0.0, 0.0, 0.0), (0.005, 0.005, 0.2))
    assert done

def test_roundtrip_reduces_error():
    # 핵심: body_twist_toward 출력을 predict_body 로 적분하면 목표에 가까워져야 한다
    # (제어법 basis 가 오도 적분 basis 와 일치함을 행동으로 검증).
    # 단위 브리징: WheelOdometry.yaw 는 라디안(update 가 wz[rad/s]*dt 를 그대로
    # 누적, wheel_odometry.py 어디에도 deg->rad 변환이 없음 - parking_v4_runner.py
    # 의 실사용도 atan2 결과(라디안)를 그대로 넘김으로 확인됨), 반면 body_twist_toward
    # 의 cur/tgt 3번째 항은 yaw_deg(도, marker_localizer.PoseFilter 규약과 동일).
    # 두 단위계 경계에서 명시적으로 변환해야 하며, 이는 기존
    # src/parkbot_aruco/test/test_wheel_odometry.py::test_matches_calibrated_predict_body
    # 가 WheelOdometry(라디안)<->PoseFilter(도) 를 math.degrees()/math.radians() 로
    # 브리징하는 것과 동일한 패턴이다.
    cur = (0.0, 0.0, 90.0)          # +X 를 보는 로봇
    tgt = (1.0, 0.5, 90.0)
    od = WheelOdometry(x=cur[0], z=cur[1], yaw=math.radians(cur[2]))
    d0 = math.hypot(tgt[0]-cur[0], tgt[1]-cur[1])
    for _ in range(50):
        vx, vy, wz, done = body_twist_toward((od.x, od.z, math.degrees(od.yaw)), tgt)
        od.update(vx, vy, wz, 0.1)
        if done: break
    d1 = math.hypot(tgt[0]-od.x, tgt[1]-od.z)
    assert d1 < d0 * 0.2
