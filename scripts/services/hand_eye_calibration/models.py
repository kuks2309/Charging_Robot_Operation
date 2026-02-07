"""
Hand-Eye Calibration Data Models

데이터 클래스 정의: RobotPose, CameraPose, CalibrationResult, EvaluationMetrics
"""

from dataclasses import dataclass, field
from typing import Optional
import numpy as np
import cv2

from .algorithms import Algorithm


@dataclass
class RobotPose:
    """
    로봇 TCP 포즈 (Eye-in-Hand에서 gripper 포즈)

    Attributes:
        x, y, z: 위치 (mm)
        rx, ry, rz: Euler angles (degrees)
    """
    x: float
    y: float
    z: float
    rx: float  # Euler angle (deg)
    ry: float
    rz: float

    def to_rotation_matrix(self) -> np.ndarray:
        """
        3x3 Rotation Matrix 반환 (Rz @ Ry @ Rx 순서)

        기존 tab_eye_in_hand.py와 동일한 ZYX Euler 순서 사용.
        """
        import math

        rx_rad = math.radians(self.rx)
        ry_rad = math.radians(self.ry)
        rz_rad = math.radians(self.rz)

        Rx = np.array([
            [1, 0, 0],
            [0, math.cos(rx_rad), -math.sin(rx_rad)],
            [0, math.sin(rx_rad), math.cos(rx_rad)]
        ], dtype=np.float64)

        Ry = np.array([
            [math.cos(ry_rad), 0, math.sin(ry_rad)],
            [0, 1, 0],
            [-math.sin(ry_rad), 0, math.cos(ry_rad)]
        ], dtype=np.float64)

        Rz = np.array([
            [math.cos(rz_rad), -math.sin(rz_rad), 0],
            [math.sin(rz_rad), math.cos(rz_rad), 0],
            [0, 0, 1]
        ], dtype=np.float64)

        return Rz @ Ry @ Rx

    def to_translation_vector(self) -> np.ndarray:
        """3x1 Translation Vector 반환 (mm)"""
        return np.array([[self.x], [self.y], [self.z]], dtype=np.float64)

    def to_homogeneous_matrix(self) -> np.ndarray:
        """4x4 Homogeneous Transformation Matrix 반환"""
        H = np.eye(4, dtype=np.float64)
        H[:3, :3] = self.to_rotation_matrix()
        H[:3, 3] = [self.x, self.y, self.z]
        return H


@dataclass
class CameraPose:
    """
    카메라에서 본 타겟(체스보드) 포즈

    Attributes:
        rvec: Rodrigues rotation vector (3,)
        tvec: Translation vector (mm) (3,)
    """
    rvec: np.ndarray  # Rodrigues rotation vector
    tvec: np.ndarray  # Translation vector (mm)

    def __post_init__(self):
        """numpy array 변환 보장"""
        self.rvec = np.asarray(self.rvec, dtype=np.float64).flatten()
        self.tvec = np.asarray(self.tvec, dtype=np.float64).flatten()

    def to_rotation_matrix(self) -> np.ndarray:
        """3x3 Rotation Matrix 반환 (via cv2.Rodrigues)"""
        R, _ = cv2.Rodrigues(self.rvec.reshape(3, 1))
        return R

    def to_translation_vector(self) -> np.ndarray:
        """3x1 Translation Vector 반환"""
        return self.tvec.reshape(3, 1)

    def to_homogeneous_matrix(self) -> np.ndarray:
        """4x4 Homogeneous Transformation Matrix 반환"""
        H = np.eye(4, dtype=np.float64)
        H[:3, :3] = self.to_rotation_matrix()
        H[:3, 3] = self.tvec
        return H


@dataclass
class CalibrationResult:
    """
    Hand-Eye 캘리브레이션 결과

    Eye-in-Hand 구성에서 Camera → Gripper 변환 행렬.

    Attributes:
        R_cam2gripper: 3x3 rotation matrix
        t_cam2gripper: 3x1 translation vector (mm)
        algorithm: 사용된 알고리즘
        num_samples: 사용된 샘플 수
    """
    R_cam2gripper: np.ndarray
    t_cam2gripper: np.ndarray
    algorithm: Algorithm
    num_samples: int

    def to_homogeneous_matrix(self) -> np.ndarray:
        """4x4 Transformation Matrix 반환"""
        H = np.eye(4, dtype=np.float64)
        H[:3, :3] = self.R_cam2gripper
        H[:3, 3] = self.t_cam2gripper.flatten()
        return H

    def __str__(self) -> str:
        t = self.t_cam2gripper.flatten()
        return (f"CalibrationResult({self.algorithm.name}, "
                f"T=[{t[0]:.2f}, {t[1]:.2f}, {t[2]:.2f}] mm, "
                f"samples={self.num_samples})")


@dataclass
class EvaluationMetrics:
    """
    캘리브레이션 평가 지표

    Attributes:
        reprojection_error_mean: 평균 재투영 오차 (pixels)
        reprojection_error_std: 재투영 오차 표준편차
        reprojection_error_max: 최대 재투영 오차
        rotation_error_deg: 회전 오차 (degrees)
        translation_error_mm: 변환 오차 (mm)
    """
    reprojection_error_mean: float = 0.0
    reprojection_error_std: float = 0.0
    reprojection_error_max: float = 0.0
    rotation_error_deg: Optional[float] = None
    translation_error_mm: Optional[float] = None

    def __str__(self) -> str:
        return (f"Metrics(reproj={self.reprojection_error_mean:.3f}±{self.reprojection_error_std:.3f} px, "
                f"max={self.reprojection_error_max:.3f} px)")
