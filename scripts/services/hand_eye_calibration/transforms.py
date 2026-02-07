"""
Coordinate Transform Utilities

Euler ↔ Rotation Matrix 변환 유틸리티.
순서: Rz @ Ry @ Rx (ZYX Euler angles) - 기존 tab_eye_in_hand.py와 동일
"""

import math
import numpy as np
import cv2


def euler_to_rotation_matrix(rx: float, ry: float, rz: float) -> np.ndarray:
    """
    Euler angles (degrees) → 3x3 Rotation Matrix

    순서: Rz @ Ry @ Rx (ZYX Euler angles)

    Args:
        rx, ry, rz: Euler angles in degrees

    Returns:
        3x3 rotation matrix
    """
    rx_rad = math.radians(rx)
    ry_rad = math.radians(ry)
    rz_rad = math.radians(rz)

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


def rotation_matrix_to_euler(R: np.ndarray) -> tuple:
    """
    3x3 Rotation Matrix → Euler angles (degrees)

    순서: Rz @ Ry @ Rx (ZYX Euler angles)

    Args:
        R: 3x3 rotation matrix

    Returns:
        (rx, ry, rz) in degrees
    """
    sy = math.sqrt(R[0, 0] ** 2 + R[1, 0] ** 2)

    singular = sy < 1e-6

    if not singular:
        rx = math.atan2(R[2, 1], R[2, 2])
        ry = math.atan2(-R[2, 0], sy)
        rz = math.atan2(R[1, 0], R[0, 0])
    else:
        rx = math.atan2(-R[1, 2], R[1, 1])
        ry = math.atan2(-R[2, 0], sy)
        rz = 0

    return (math.degrees(rx), math.degrees(ry), math.degrees(rz))


def rodrigues_to_rotation_matrix(rvec: np.ndarray) -> np.ndarray:
    """
    Rodrigues vector → 3x3 Rotation Matrix

    Args:
        rvec: Rodrigues rotation vector (3,) or (3,1)

    Returns:
        3x3 rotation matrix
    """
    rvec = np.asarray(rvec, dtype=np.float64).reshape(3, 1)
    R, _ = cv2.Rodrigues(rvec)
    return R


def rotation_matrix_to_rodrigues(R: np.ndarray) -> np.ndarray:
    """
    3x3 Rotation Matrix → Rodrigues vector

    Args:
        R: 3x3 rotation matrix

    Returns:
        Rodrigues rotation vector (3,)
    """
    rvec, _ = cv2.Rodrigues(R)
    return rvec.flatten()


def create_homogeneous_matrix(R: np.ndarray, t: np.ndarray) -> np.ndarray:
    """
    3x3 Rotation + 3x1 Translation → 4x4 Homogeneous Matrix

    Args:
        R: 3x3 rotation matrix
        t: 3x1 or (3,) translation vector

    Returns:
        4x4 homogeneous transformation matrix
    """
    H = np.eye(4, dtype=np.float64)
    H[:3, :3] = R
    H[:3, 3] = np.asarray(t).flatten()
    return H


def invert_homogeneous_matrix(H: np.ndarray) -> np.ndarray:
    """
    4x4 Homogeneous Matrix 역변환

    Args:
        H: 4x4 homogeneous transformation matrix

    Returns:
        Inverse of H
    """
    R = H[:3, :3]
    t = H[:3, 3]

    H_inv = np.eye(4, dtype=np.float64)
    H_inv[:3, :3] = R.T
    H_inv[:3, 3] = -R.T @ t
    return H_inv
