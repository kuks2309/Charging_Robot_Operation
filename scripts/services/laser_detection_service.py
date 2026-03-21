"""
레이저 검출 서비스.
extract_laser_center, fit_laser_line 호출을 탭 레이어에서 분리.
"""
import numpy as np
from Sensor.laser.extract_laser_center import (
    extract_laser_center,
    extract_laser_center_conv,
    fit_laser_line,
    extract_red_mask,
    fit_multiple_lines,
)


class LaserDetectionService:
    """레이저 중심 추출 + 라인 피팅 통합 서비스 (Stateless)."""

    @staticmethod
    def detect_laser_center(frame: np.ndarray, **kwargs) -> dict | None:
        """extract_laser_center 래퍼.

        Returns dict with {cols, centers_y, mask, est_width} or None.
        """
        cols, centers_y, mask, est_width = extract_laser_center(frame, **kwargs)
        if len(cols) == 0:
            return None
        return {
            'cols': cols,
            'centers_y': centers_y,
            'mask': mask,
            'est_width': est_width,
        }

    @staticmethod
    def detect_laser_center_conv(frame: np.ndarray, **kwargs) -> dict | None:
        """extract_laser_center_conv 래퍼.

        Returns dict with {cols, centers_y, est_width} or None.
        """
        cols, centers_y, est_width = extract_laser_center_conv(frame, **kwargs)
        if len(cols) == 0:
            return None
        return {
            'cols': cols,
            'centers_y': centers_y,
            'est_width': est_width,
        }

    @staticmethod
    def detect_and_fit(frame: np.ndarray,
                       fit_kwargs: dict | None = None,
                       **kwargs) -> dict | None:
        """extract_laser_center_conv + fit_laser_line 통합.

        fit_kwargs: fit_laser_line에 전달할 추가 인자 (예: mad_scale).
        Returns dict with {cols, centers_y, inlier_cols, inlier_y, coeffs, est_width} or None.
        """
        cols, centers_y, est_width = extract_laser_center_conv(frame, **kwargs)
        if len(cols) == 0:
            return None
        coeffs, inlier_cols, inlier_y = fit_laser_line(cols, centers_y, **(fit_kwargs or {}))
        if coeffs is None or len(coeffs) < 2:
            return None
        return {
            'cols': cols,
            'centers_y': centers_y,
            'inlier_cols': inlier_cols,
            'inlier_y': inlier_y,
            'coeffs': coeffs,
            'est_width': est_width,
        }

    @staticmethod
    def fit_laser_centers(cols: np.ndarray,
                          centers_y: np.ndarray,
                          **fit_kwargs) -> dict | None:
        """fit_laser_line 래퍼.

        Returns dict with {coeffs, inlier_cols, inlier_y} or None.
        """
        coeffs, inlier_cols, inlier_y = fit_laser_line(cols, centers_y, **fit_kwargs)
        if coeffs is None or len(coeffs) < 2:
            return None
        return {
            'coeffs': coeffs,
            'inlier_cols': inlier_cols,
            'inlier_y': inlier_y,
        }

    @staticmethod
    def detect_in_roi(frame: np.ndarray, roi_rect: tuple) -> dict | None:
        """ROI 영역 crop → detect → 원본 좌표로 변환.

        roi_rect: (x0, y0, x1, y1)
        tab_laser_scan._detect_laser_in_roi 로직 이전.
        Returns dict with {cols, centers_y, inlier_cols, inlier_y, coeffs, angle_deg, est_width} or None.
        """
        x0, y0, x1, y1 = roi_rect
        if (x1 - x0) < 10 or (y1 - y0) < 10:
            return None

        cropped = frame[y0:y1, x0:x1]
        cols, centers_y, est_width = extract_laser_center_conv(cropped)

        if len(cols) == 0:
            return None

        # crop 좌표 → 원본 프레임 좌표
        cols = cols + x0
        centers_y = centers_y + y0

        coeffs, inlier_cols, inlier_y = fit_laser_line(cols, centers_y)
        if coeffs is None or len(coeffs) < 2:
            return None

        angle_deg = float(np.degrees(np.arctan(coeffs[0])))
        return {
            'cols': cols,
            'centers_y': centers_y,
            'inlier_cols': inlier_cols,
            'inlier_y': inlier_y,
            'coeffs': coeffs,
            'angle_deg': angle_deg,
            'est_width': est_width,
        }

    @staticmethod
    def extract_red_mask(frame: np.ndarray, **kwargs) -> np.ndarray:
        """extract_red_mask 래퍼. RGB 채널 차이 기반 레이저 마스크 반환."""
        return extract_red_mask(frame, **kwargs)

    @staticmethod
    def fit_multiple_lines(cols: np.ndarray, centers_y: np.ndarray, **kwargs) -> list:
        """fit_multiple_lines 래퍼. 다중 직선 피팅 결과 반환."""
        return fit_multiple_lines(cols, centers_y, **kwargs)

    @staticmethod
    def get_laser_y_at_center(frame: np.ndarray, **kwargs) -> float | None:
        """프레임 중심 X에서의 레이저 Y 좌표.

        tab_laser_calibration.get_current_laser_y 로직 이전.
        Returns float (pixel Y) or None.
        """
        cols, centers_y, est_width = extract_laser_center_conv(frame, **kwargs)
        if len(cols) == 0:
            return None
        coeffs, inlier_cols, inlier_y = fit_laser_line(cols, centers_y)
        if coeffs is None or len(coeffs) < 2:
            return None
        center_x = frame.shape[1] // 2
        return float(np.polyval(coeffs, center_x))
