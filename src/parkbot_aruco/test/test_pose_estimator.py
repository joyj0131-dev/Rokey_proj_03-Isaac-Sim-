"""데드레커닝 + 마커 보정 융합 (ROS 불필요)."""
import math

from parkbot_aruco.pose_estimator import PoseEstimator


def test_predict_integrates_in_heading_frame():
    est = PoseEstimator(x=0.0, y=0.0, yaw=math.pi / 2)   # +y를 바라봄
    est.predict(vx=1.0, vy=0.0, wz=0.0, dt=0.5)          # 로컬 전진 0.5m
    x, y, yaw = est.pose
    assert abs(x) < 1e-9 and abs(y - 0.5) < 1e-9


def test_correct_pulls_toward_fix():
    est = PoseEstimator(x=1.0, y=0.0, yaw=0.0)
    est.correct(x=2.0, y=0.0, yaw=0.0, gain=0.5)
    assert abs(est.pose[0] - 1.5) < 1e-9


def test_yaw_correct_wraps():
    est = PoseEstimator(x=0.0, y=0.0, yaw=math.pi - 0.05)
    est.correct(x=0.0, y=0.0, yaw=-math.pi + 0.05, gain=1.0)
    assert abs(abs(est.pose[2]) - math.pi) < 0.06
