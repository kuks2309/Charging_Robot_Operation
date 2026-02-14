#!/usr/bin/env python3
"""
레이저 캘리브레이션 탭
ArduCam 카메라로 라인 레이저 중심선을 추출하여 핑크색으로 표시
"""

import cv2
import numpy as np
from PyQt5.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QLabel, QPushButton,
    QGroupBox, QGridLayout, QDoubleSpinBox,
)
from PyQt5.QtCore import Qt, pyqtSignal

from utils.common import display_frame_on_label
from Sensor.laser.extract_laser_center import (
    extract_laser_center,
    fit_laser_line,
)


class TabLaserCalibration(QWidget):
    """레이저 캘리브레이션 탭 (ArduCam 전용)"""

    log_message = pyqtSignal(str)
    camera_start_requested = pyqtSignal()
    camera_stop_requested = pyqtSignal()
    # ArduCam 강제 선택 시그널 (main_window에서 처리)
    arducam_required = pyqtSignal()

    def __init__(self, parent=None):
        super().__init__(parent)

        self.camera_manager = None
        self.current_frame = None
        self.show_laser_overlay = False

        # 마지막 추출 결과 캐시
        self._last_coeffs = None
        self._last_angle_deg = None
        self._last_inlier_count = 0
        self._last_total_count = 0

        self._setup_ui()
        self._connect_signals()

    def _setup_ui(self):
        """UI 구성: 왼쪽 카메라 뷰 + 오른쪽 컨트롤"""
        main_layout = QHBoxLayout(self)

        # === 왼쪽: 카메라 뷰 ===
        self.labelCameraView = QLabel("카메라 미연결")
        self.labelCameraView.setAlignment(Qt.AlignCenter)
        self.labelCameraView.setMinimumSize(640, 480)
        self.labelCameraView.setStyleSheet("background-color: #1a1a2e; color: #aaa; border: 1px solid #333;")
        main_layout.addWidget(self.labelCameraView, stretch=3)

        # === 오른쪽: 컨트롤 패널 ===
        control_panel = QVBoxLayout()

        # 카메라 (ArduCam 전용 표시)
        cam_label = QLabel("ArduCam 전용")
        cam_label.setStyleSheet("font-weight: bold; color: #666; padding: 4px;")
        control_panel.addWidget(cam_label)

        # 카메라 시작/정지
        cam_btn_layout = QHBoxLayout()
        self.btnStartCamera = QPushButton("카메라 시작")
        self.btnStopCamera = QPushButton("카메라 정지")
        self.btnStartCamera.setStyleSheet("padding: 6px;")
        self.btnStopCamera.setStyleSheet("padding: 6px;")
        cam_btn_layout.addWidget(self.btnStartCamera)
        cam_btn_layout.addWidget(self.btnStopCamera)
        control_panel.addLayout(cam_btn_layout)

        # 레이저 오버레이 그룹
        group_laser = QGroupBox("레이저 직선 추출")
        laser_layout = QVBoxLayout(group_laser)

        # 토글 버튼
        self.btnToggleLaser = QPushButton("레이저 표시 ON")
        self.btnToggleLaser.setCheckable(True)
        self.btnToggleLaser.setStyleSheet(
            "QPushButton { padding: 10px; font-weight: bold; font-size: 14px; }"
            "QPushButton:checked { background-color: #ff69b4; color: white; }"
        )
        laser_layout.addWidget(self.btnToggleLaser)

        # 파라미터
        param_grid = QGridLayout()
        param_grid.addWidget(QLabel("Min stripe width:"), 0, 0)
        self.spinMinStripe = QDoubleSpinBox()
        self.spinMinStripe.setRange(1, 50)
        self.spinMinStripe.setValue(3)
        self.spinMinStripe.setDecimals(0)
        self.spinMinStripe.setSuffix(" px")
        param_grid.addWidget(self.spinMinStripe, 0, 1)

        param_grid.addWidget(QLabel("Max stripe width:"), 1, 0)
        self.spinMaxStripe = QDoubleSpinBox()
        self.spinMaxStripe.setRange(10, 200)
        self.spinMaxStripe.setValue(80)
        self.spinMaxStripe.setDecimals(0)
        self.spinMaxStripe.setSuffix(" px")
        param_grid.addWidget(self.spinMaxStripe, 1, 1)

        param_grid.addWidget(QLabel("MAD scale:"), 2, 0)
        self.spinMadScale = QDoubleSpinBox()
        self.spinMadScale.setRange(1.0, 10.0)
        self.spinMadScale.setValue(3.0)
        self.spinMadScale.setDecimals(1)
        param_grid.addWidget(self.spinMadScale, 2, 1)

        laser_layout.addLayout(param_grid)

        # 결과 표시
        group_result = QGroupBox("추출 결과")
        result_grid = QGridLayout(group_result)
        result_grid.addWidget(QLabel("기울기:"), 0, 0)
        self.labelAngle = QLabel("-")
        self.labelAngle.setStyleSheet("font-weight: bold; font-size: 16px;")
        result_grid.addWidget(self.labelAngle, 0, 1)

        result_grid.addWidget(QLabel("포인트:"), 1, 0)
        self.labelPoints = QLabel("-")
        result_grid.addWidget(self.labelPoints, 1, 1)

        result_grid.addWidget(QLabel("Y 범위:"), 2, 0)
        self.labelYRange = QLabel("-")
        result_grid.addWidget(self.labelYRange, 2, 1)

        result_grid.addWidget(QLabel("Coeffs:"), 3, 0)
        self.labelCoeffs = QLabel("-")
        self.labelCoeffs.setWordWrap(True)
        result_grid.addWidget(self.labelCoeffs, 3, 1)

        laser_layout.addWidget(group_result)
        control_panel.addWidget(group_laser)

        # 스냅샷 버튼
        self.btnSnapshot = QPushButton("스냅샷 저장")
        self.btnSnapshot.setStyleSheet("padding: 6px;")
        control_panel.addWidget(self.btnSnapshot)

        control_panel.addStretch()
        main_layout.addLayout(control_panel, stretch=1)

    def _connect_signals(self):
        """시그널 연결"""
        self.btnStartCamera.clicked.connect(self._on_start_camera)
        self.btnStopCamera.clicked.connect(self.camera_stop_requested.emit)
        self.btnToggleLaser.toggled.connect(self._on_toggle_laser)
        self.btnSnapshot.clicked.connect(self._on_snapshot)

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

    def _on_snapshot(self):
        """현재 프레임 스냅샷 저장"""
        if self.current_frame is None:
            return

        from datetime import datetime
        import os

        save_dir = os.path.expanduser("~/Project/Charging_Robot_Operation/data/laser")
        os.makedirs(save_dir, exist_ok=True)
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filepath = os.path.join(save_dir, f"laser_{timestamp}.png")

        # 오버레이 적용된 프레임 저장
        save_frame = self.current_frame.copy()
        if self.show_laser_overlay:
            save_frame = self._draw_laser_overlay(save_frame)

        cv2.imwrite(filepath, save_frame)
        self.log_message.emit(f"스냅샷 저장: {filepath}")

    def _log(self, msg):
        self.log_message.emit(msg)
