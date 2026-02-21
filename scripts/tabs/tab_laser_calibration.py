#!/usr/bin/env python3
"""
레이저 캘리브레이션 탭
ArduCam 카메라로 라인 레이저 중심선을 추출하여 핑크색으로 표시
"""

import os
import cv2
import numpy as np
from PyQt5 import uic
from PyQt5.QtWidgets import QWidget
from PyQt5.QtCore import pyqtSignal

from utils.common import display_frame_on_label
from Sensor.laser.extract_laser_center import (
    extract_laser_center,
    fit_laser_line,
)


# UI 파일 경로
UI_DIR = os.path.join(os.path.dirname(__file__), '..', '..', 'ui')
TAB_LASER_CALIBRATION_UI = os.path.join(UI_DIR, 'tab_laser_calibration.ui')


class TabLaserCalibration(QWidget):
    """레이저 캘리브레이션 탭 (ArduCam 전용)"""

    log_message = pyqtSignal(str)
    camera_start_requested = pyqtSignal()
    camera_stop_requested = pyqtSignal()
    arducam_required = pyqtSignal()

    def __init__(self, parent=None):
        super().__init__(parent)

        # UI 로드
        uic.loadUi(TAB_LASER_CALIBRATION_UI, self)

        self.camera_manager = None
        self.current_frame = None
        self.show_laser_overlay = False

        # 마지막 추출 결과 캐시
        self._last_coeffs = None
        self._last_angle_deg = None
        self._last_inlier_count = 0
        self._last_total_count = 0

        self._connect_signals()

    def _connect_signals(self):
        """시그널 연결"""
        self.btnStartCamera.clicked.connect(self._on_start_camera)
        self.btnStopCamera.clicked.connect(self.camera_stop_requested.emit)
        self.btnToggleLaser.toggled.connect(self._on_toggle_laser)
        self.btnSaveImage.clicked.connect(self._on_save_image)

    def _on_start_camera(self):
        """카메라 시작 — ArduCam 강제 선택 후 시작"""
        self.arducam_required.emit()
        self.camera_start_requested.emit()

    def _on_toggle_laser(self, checked):
        """레이저 오버레이 토글"""
        self.show_laser_overlay = checked
        self.btnToggleLaser.setText("레이저 표시 OFF" if checked else "레이저 표시 ON")
        if not checked:
            self._clear_result_labels()

    def _clear_result_labels(self):
        """결과 라벨 초기화"""
        self.labelAngle.setText("-")
        self.labelPoints.setText("-")
        self.labelYRange.setText("-")
        self.labelCoeffs.setText("-")
        self._last_coeffs = None
        self._last_angle_deg = None

    def set_camera_manager(self, camera_manager):
        self.camera_manager = camera_manager

    def update_frame(self, frame):
        """카메라 프레임 업데이트 — 레이저 오버레이 적용"""
        if frame is None:
            return

        self.current_frame = frame.copy()
        display = frame.copy()

        if self.show_laser_overlay:
            display = self._draw_laser_overlay(display)

        display_frame_on_label(display, self.labelCameraView)

    def _draw_laser_overlay(self, frame):
        """레이저 중심선 추출 후 핑크색으로 오버레이"""
        min_w = int(self.spinMinStripe.value())
        max_w = int(self.spinMaxStripe.value())
        mad_scale = self.spinMadScale.value()

        cols, centers_y, mask = extract_laser_center(
            frame, min_stripe_width=min_w, max_stripe_width=max_w,
        )

        if len(cols) == 0:
            self._clear_result_labels()
            return frame

        coeffs, inlier_cols, inlier_y = fit_laser_line(
            cols, centers_y, mad_scale=mad_scale,
        )

        # 핑크색 (BGR: 180, 105, 255)
        PINK = (180, 105, 255)

        # inlier 포인트를 핑크 점으로 표시
        for cx, cy in zip(inlier_cols, inlier_y):
            cv2.circle(frame, (int(cx), int(round(cy))), 2, PINK, -1)

        # 피팅 직선 표시
        if len(coeffs) >= 2 and len(inlier_cols) > 0:
            x_start = int(inlier_cols.min())
            x_end = int(inlier_cols.max())
            y_start = int(round(np.polyval(coeffs, x_start)))
            y_end = int(round(np.polyval(coeffs, x_end)))
            cv2.line(frame, (x_start, y_start), (x_end, y_end), PINK, 2)

        # 결과 업데이트
        angle_deg = np.degrees(np.arctan(coeffs[0])) if len(coeffs) >= 2 else None
        self._last_coeffs = coeffs
        self._last_angle_deg = angle_deg
        self._last_inlier_count = len(inlier_cols)
        self._last_total_count = len(cols)

        self.labelAngle.setText(f"{angle_deg:.3f}°" if angle_deg is not None else "-")
        self.labelPoints.setText(f"{len(inlier_cols)} / {len(cols)}")
        if len(inlier_y) > 0:
            self.labelYRange.setText(f"{inlier_y.min():.1f} ~ {inlier_y.max():.1f} px")
        if len(coeffs) >= 2:
            self.labelCoeffs.setText(f"[{coeffs[0]:.6f}, {coeffs[1]:.2f}]")

        return frame

    def _get_save_dir(self):
        """저장 폴더 반환 (~/images/laser)"""
        save_dir = os.path.expanduser("~/images/laser")
        os.makedirs(save_dir, exist_ok=True)
        return save_dir

    def _make_filename(self):
        """저장 파일명 생성 (laser_날짜시간.jpg)"""
        from datetime import datetime
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        return f"laser_{timestamp}.jpg"

    def _on_save_image(self):
        """이미지 저장 (오버레이 ON이면 오버레이 포함, OFF면 원본)"""
        if self.current_frame is None:
            return

        save_frame = self.current_frame.copy()
        if self.show_laser_overlay:
            save_frame = self._draw_laser_overlay(save_frame)

        filepath = os.path.join(self._get_save_dir(), self._make_filename())
        cv2.imwrite(filepath, save_frame)
        self.log_message.emit(f"이미지 저장: {filepath}")

    def _log(self, msg):
        self.log_message.emit(msg)
