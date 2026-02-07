"""
Hand-Eye Calibration Service Module

독립 실행 가능한 Hand-Eye 캘리브레이션 모듈.
Eye-in-Hand 구성 (카메라가 로봇 엔드이펙터에 장착)을 위한 캘리브레이션 수행.

Usage:
    from services.hand_eye_calibration import HandEyeCalibrator, Algorithm

    calibrator = HandEyeCalibrator(algorithm=Algorithm.TSAI)
    num_samples = calibrator.load_data('calibration/hand_eye_data/')
    result = calibrator.calibrate()
    print(f"Translation: {result.t_cam2gripper.flatten()}")
"""

from .calibrator import HandEyeCalibrator
from .algorithms import Algorithm
from .models import RobotPose, CameraPose, CalibrationResult, EvaluationMetrics
from .transforms import euler_to_rotation_matrix, rotation_matrix_to_euler

__all__ = [
    'HandEyeCalibrator',
    'Algorithm',
    'RobotPose',
    'CameraPose',
    'CalibrationResult',
    'EvaluationMetrics',
    'euler_to_rotation_matrix',
    'rotation_matrix_to_euler',
]

__version__ = '1.0.0'
