"""
단위 테스트: scripts/services/laser_detection_service.py

테스트 대상:
- LaserDetectionService.detect_in_roi: 작은/빈 ROI → None
- LaserDetectionService.detect_and_fit: 순수 검정 이미지 → None
- LaserDetectionService.get_laser_y_at_center: 순수 검정 이미지 → None
- LaserDetectionService.detect_laser_center_conv: 검정 이미지 → None
"""
import numpy as np
import pytest


# conftest.py가 sys.path를 설정한다
from services.laser_detection_service import LaserDetectionService


# ---------------------------------------------------------------------------
# 헬퍼
# ---------------------------------------------------------------------------

def _black_bgr(h=100, w=100):
    """레이저가 전혀 없는 순수 검정 BGR 이미지."""
    return np.zeros((h, w, 3), dtype=np.uint8)


def _tiny_bgr(h=5, w=5):
    """ROI 최소 크기(10px) 미만의 이미지."""
    return np.zeros((h, w, 3), dtype=np.uint8)


# ---------------------------------------------------------------------------
# detect_in_roi
# ---------------------------------------------------------------------------

class TestDetectInRoi:
    """LaserDetectionService.detect_in_roi 동작 검증."""

    def test_returns_none_when_roi_width_is_too_small(self):
        """ROI 폭이 10px 미만이면 None을 반환해야 한다."""
        frame = _black_bgr()
        # (x0=0, y0=0, x1=5, y1=80): 폭 5 < 10
        result = LaserDetectionService.detect_in_roi(frame, roi_rect=(0, 0, 5, 80))

        assert result is None

    def test_returns_none_when_roi_height_is_too_small(self):
        """ROI 높이가 10px 미만이면 None을 반환해야 한다."""
        frame = _black_bgr()
        # (x0=0, y0=0, x1=80, y1=5): 높이 5 < 10
        result = LaserDetectionService.detect_in_roi(frame, roi_rect=(0, 0, 80, 5))

        assert result is None

    def test_returns_none_when_roi_is_exactly_minimum_minus_one(self):
        """ROI 크기가 정확히 10px 미만(경계)이면 None을 반환해야 한다."""
        frame = _black_bgr(50, 50)
        result = LaserDetectionService.detect_in_roi(frame, roi_rect=(0, 0, 9, 9))

        assert result is None

    def test_returns_none_on_black_image_with_valid_roi(self):
        """유효한 ROI 크기라도 레이저가 없는 검정 이미지면 None을 반환해야 한다."""
        frame = _black_bgr(100, 100)
        result = LaserDetectionService.detect_in_roi(frame, roi_rect=(0, 0, 80, 80))

        assert result is None


# ---------------------------------------------------------------------------
# detect_and_fit
# ---------------------------------------------------------------------------

class TestDetectAndFit:
    """LaserDetectionService.detect_and_fit 동작 검증."""

    def test_returns_none_on_pure_black_image(self):
        """레이저가 없는 순수 검정 이미지에서 None을 반환해야 한다."""
        frame = _black_bgr()
        result = LaserDetectionService.detect_and_fit(frame)

        assert result is None

    def test_returns_none_on_small_black_image(self):
        """작은 검정 이미지에서도 None을 반환해야 한다."""
        frame = _black_bgr(20, 20)
        result = LaserDetectionService.detect_and_fit(frame)

        assert result is None


# ---------------------------------------------------------------------------
# get_laser_y_at_center
# ---------------------------------------------------------------------------

class TestGetLaserYAtCenter:
    """LaserDetectionService.get_laser_y_at_center 동작 검증."""

    def test_returns_none_on_pure_black_image(self):
        """레이저가 없는 이미지에서 None을 반환해야 한다."""
        frame = _black_bgr()
        result = LaserDetectionService.get_laser_y_at_center(frame)

        assert result is None

    def test_returns_none_on_large_black_image(self):
        """큰 검정 이미지에서도 None을 반환해야 한다."""
        frame = _black_bgr(480, 640)
        result = LaserDetectionService.get_laser_y_at_center(frame)

        assert result is None


# ---------------------------------------------------------------------------
# detect_laser_center_conv
# ---------------------------------------------------------------------------

class TestDetectLaserCenterConv:
    """LaserDetectionService.detect_laser_center_conv 동작 검증."""

    def test_returns_none_on_black_image(self):
        """검정 이미지에서 cols가 없으므로 None을 반환해야 한다."""
        frame = _black_bgr()
        result = LaserDetectionService.detect_laser_center_conv(frame)

        assert result is None

    def test_returns_none_on_large_black_image(self):
        """큰 검정 이미지에서도 None을 반환해야 한다."""
        frame = _black_bgr(480, 640)
        result = LaserDetectionService.detect_laser_center_conv(frame)

        assert result is None
