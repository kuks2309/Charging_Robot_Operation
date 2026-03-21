"""
Hand-Eye 캘리브레이션 서비스.
cv2.solvePnP, cv2.Rodrigues, cv2.calibrateHandEye 호출을 탭 레이어에서 분리.
"""
import math

import numpy as np
import cv2
from PyQt5.QtCore import QObject, pyqtSignal


class HandEyeCalibrationService(QObject):
    """Hand-Eye 캘리브레이션 및 포즈 추정 서비스."""

    calibration_completed = pyqtSignal(bool, object)

    def __init__(self, parent=None):
        super().__init__(parent)

    def solve_pnp_and_store(self, obj_points, corners, camera_matrix, dist_coeffs=None) -> dict | None:
        """cv2.solvePnP + Rodrigues 실행.

        Returns dict with {success, rvec, tvec, R_cam} or None on failure.
        """
        try:
            success, rvec, tvec = cv2.solvePnP(
                obj_points, corners, camera_matrix, dist_coeffs
            )
            if not success:
                return None
            R_cam, _ = cv2.Rodrigues(rvec)
            return {
                "success": True,
                "rvec": rvec,
                "tvec": tvec,
                "R_cam": R_cam,
            }
        except Exception:
            return None

    def calibrate(self, robot_poses: list, camera_poses: list,
                  method=None) -> dict:
        """cv2.calibrateHandEye 실행.

        robot_poses: [(x,y,z,rx,ry,rz), ...] euler 각도 리스트
        camera_poses: [(R_cam, tvec), ...] 리스트
        Returns dict with {success, R_cam2gripper, t_cam2gripper, hand_eye_matrix, error_message}
        """
        try:
            R_gripper2base_list = []
            t_gripper2base_list = []

            for pose in robot_poses:
                x, y, z, rx, ry, rz = pose
                R = self._euler_to_rotation_matrix(rx, ry, rz)
                t = np.array([[x], [y], [z]], dtype=np.float64)
                R_gripper2base_list.append(R)
                t_gripper2base_list.append(t)

            R_target2cam_list = [p[0] for p in camera_poses]
            t_target2cam_list = [p[1] for p in camera_poses]

            calib_method = method if method is not None else cv2.CALIB_HAND_EYE_TSAI

            R_cam2gripper, t_cam2gripper = cv2.calibrateHandEye(
                R_gripper2base_list, t_gripper2base_list,
                R_target2cam_list, t_target2cam_list,
                method=calib_method,
            )

            hand_eye_matrix = np.eye(4)
            hand_eye_matrix[:3, :3] = R_cam2gripper
            hand_eye_matrix[:3, 3] = t_cam2gripper.flatten()

            return {
                "success": True,
                "R_cam2gripper": R_cam2gripper,
                "t_cam2gripper": t_cam2gripper,
                "hand_eye_matrix": hand_eye_matrix,
                "error_message": None,
            }
        except Exception as e:
            return {
                "success": False,
                "R_cam2gripper": None,
                "t_cam2gripper": None,
                "hand_eye_matrix": None,
                "error_message": str(e),
            }

    @staticmethod
    def _euler_to_rotation_matrix(rx_deg, ry_deg, rz_deg) -> np.ndarray:
        """Euler angles (degrees) -> 3x3 rotation matrix. ZYX convention."""
        rx_rad = math.radians(rx_deg)
        ry_rad = math.radians(ry_deg)
        rz_rad = math.radians(rz_deg)

        Rx = np.array([
            [1, 0, 0],
            [0, math.cos(rx_rad), -math.sin(rx_rad)],
            [0, math.sin(rx_rad), math.cos(rx_rad)]
        ])

        Ry = np.array([
            [math.cos(ry_rad), 0, math.sin(ry_rad)],
            [0, 1, 0],
            [-math.sin(ry_rad), 0, math.cos(ry_rad)]
        ])

        Rz = np.array([
            [math.cos(rz_rad), -math.sin(rz_rad), 0],
            [math.sin(rz_rad), math.cos(rz_rad), 0],
            [0, 0, 1]
        ])

        return Rz @ Ry @ Rx
