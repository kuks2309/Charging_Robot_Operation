"""
카메라 캘리브레이션 서비스.
cv2.calibrateCamera 호출을 탭 레이어에서 분리.
"""
import numpy as np
import cv2
from PyQt5.QtCore import QObject, pyqtSignal


class CameraCalibrationService(QObject):
    """카메라 내부 파라미터 캘리브레이션 서비스."""

    calibration_completed = pyqtSignal(bool, object)  # (success, result_dict)

    def __init__(self, parent=None):
        super().__init__(parent)

    def run_calibration(self,
                        points_3d: list,
                        points_2d: list,
                        image_size: tuple) -> dict:
        """카메라 캘리브레이션 실행.

        Args:
            points_3d: 3D 객체 점 리스트 (각 요소: Nx3 float32 array)
            points_2d: 2D 이미지 점 리스트 (각 요소: Nx1x2 float32 array)
            image_size: (width, height) 튜플

        Returns:
            dict with keys:
                success (bool): 캘리브레이션 성공 여부
                camera_matrix (np.ndarray | None): 3x3 내부 파라미터 행렬
                dist_coeffs (np.ndarray | None): 왜곡 계수
                rms_error (float | None): RMS 재투영 오차
                rvecs (list | None): 회전 벡터 리스트
                tvecs (list | None): 이동 벡터 리스트
                error_message (str | None): 실패 시 오류 메시지
        """
        if len(points_3d) < 3:
            result = {
                'success': False,
                'camera_matrix': None,
                'dist_coeffs': None,
                'rms_error': None,
                'rvecs': None,
                'tvecs': None,
                'error_message': f'최소 3개 이미지 필요 (현재: {len(points_3d)}개)'
            }
            self.calibration_completed.emit(False, result)
            return result

        try:
            rms, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(
                points_3d, points_2d, image_size, None, None
            )
            result = {
                'success': True,
                'camera_matrix': camera_matrix,
                'dist_coeffs': dist_coeffs,
                'rms_error': rms,
                'rvecs': rvecs,
                'tvecs': tvecs,
                'error_message': None
            }
            self.calibration_completed.emit(True, result)
            return result
        except Exception as e:
            result = {
                'success': False,
                'camera_matrix': None,
                'dist_coeffs': None,
                'rms_error': None,
                'rvecs': None,
                'tvecs': None,
                'error_message': str(e)
            }
            self.calibration_completed.emit(False, result)
            return result
