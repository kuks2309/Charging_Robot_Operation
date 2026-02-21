#!/usr/bin/env python3
"""
레이저 캘리브레이션 탭
ArduCam 카메라로 라인 레이저 중심선을 추출하여 핑크색으로 표시
"""

import os
import cv2
import yaml
import numpy as np
from datetime import datetime
from PyQt5 import uic
from PyQt5.QtWidgets import QWidget
from PyQt5.QtCore import pyqtSignal

from utils.common import display_frame_on_label
from tabs.jog_mixin import JogMixin
from Sensor.laser.extract_laser_center import (
    extract_laser_center,
    extract_laser_center_conv,
    extract_red_mask,
    fit_laser_line,
    fit_multiple_lines,
)


# UI 파일 경로
UI_DIR = os.path.join(os.path.dirname(__file__), '..', '..', 'ui')
TAB_LASER_CALIBRATION_UI = os.path.join(UI_DIR, 'tab_laser_calibration.ui')

# 캘리브레이션 파일 경로
CONFIG_DIR = os.path.join(os.path.dirname(__file__), '..', '..', 'config')
ARDUCAM_CALIB_FILE = os.path.join(CONFIG_DIR, 'arducam_calibration.yaml')


class TabLaserCalibration(QWidget, JogMixin):
    """레이저 캘리브레이션 탭 (ArduCam 전용)"""

    log_message = pyqtSignal(str)
    camera_start_requested = pyqtSignal()
    camera_stop_requested = pyqtSignal()
    arducam_required = pyqtSignal()
    jog_move_requested = pyqtSignal(str, float)
    jog_rotate_requested = pyqtSignal(str, float)

    def __init__(self, parent=None):
        super().__init__(parent)

        # UI 로드
        uic.loadUi(TAB_LASER_CALIBRATION_UI, self)

        # === 레이아웃 고정 (내용 변화에도 UI 안정) ===
        self.labelCameraView.setFixedSize(640, 360)      # 카메라 1280x720의 1/2
        self.widgetLeft.setFixedWidth(710)                # 좌측 패널 폭 고정
        self.widgetRight.setFixedWidth(450)               # 우측 패널 폭 고정
        self.groupResult.setFixedHeight(220)              # 추출 결과 높이 고정
        self.labelAngle.setWordWrap(True)                 # 긴 기울기 텍스트 줄바꿈

        self.camera_manager = None
        self.current_frame = None       # 원본 프레임
        self.display_frame = None       # 표시용 프레임 (original 또는 undistorted)
        # 표시 모드: None, 'laser', 'conv', 'lines', 'hsv'
        self.display_type = None

        # 캘리브레이션
        self.camera_matrix = None
        self.dist_coeffs = None
        self._calibration_loaded = False

        # 마지막 추출 결과 캐시
        self._last_coeffs = None
        self._last_angle_deg = None
        self._last_inlier_count = 0
        self._last_total_count = 0

        self._load_calibration()
        self._connect_signals()

    def _connect_signals(self):
        """시그널 연결"""
        self.btnStartCamera.clicked.connect(self._on_start_camera)
        self.btnStopCamera.clicked.connect(self.camera_stop_requested.emit)
        self.btnToggleLaser.toggled.connect(self._on_toggle_laser)
        self.btnConvCenter.toggled.connect(self._on_toggle_conv)
        self.btnFitLines.toggled.connect(self._on_toggle_fit_lines)
        self.btnRgbMask.toggled.connect(self._on_toggle_rgb_mask)
        self.btnSaveImage.clicked.connect(self._on_save_image)
        self._connect_jog_buttons()

    def _load_calibration(self):
        """ArduCam 캘리브레이션 파일 로드"""
        if self._calibration_loaded:
            return

        try:
            with open(ARDUCAM_CALIB_FILE, 'r') as f:
                data = yaml.safe_load(f)

            cam_data = data['camera_matrix']
            self.camera_matrix = np.array(cam_data['data'], dtype=np.float64).reshape(
                cam_data['rows'], cam_data['cols']
            )

            dist_data = data['distortion_coefficients']
            self.dist_coeffs = np.array(dist_data['data'], dtype=np.float64).reshape(
                dist_data['rows'], dist_data['cols']
            )

            self._calibration_loaded = True

        except Exception as e:
            self.camera_matrix = None
            self.dist_coeffs = None
            self.log_message.emit(f"캘리브레이션 로드 실패: {e}")

    def _on_start_camera(self):
        """카메라 시작 — ArduCam 강제 선택 후 시작"""
        self.arducam_required.emit()
        self.camera_start_requested.emit()

    # 버튼 ↔ display_type 매핑
    _DISPLAY_BUTTONS = {
        'laser': ('btnToggleLaser', '레이저 표시 ON', '레이저 표시 OFF'),
        'conv':  ('btnConvCenter',  '레이저 중심 추출', '중심 추출 OFF'),
        'lines': ('btnFitLines',    '직선 추출',       '직선 추출 OFF'),
        'rgb':   ('btnRgbMask',     'RGB 레이저 추출', 'RGB 추출 OFF'),
    }

    def _on_display_toggle(self, dtype: str, checked: bool):
        """표시 모드 토글 — 단일 display_type으로 상호배제 처리"""
        if checked:
            self.display_type = dtype
            # 다른 버튼 모두 해제
            for key, (btn_name, _, _) in self._DISPLAY_BUTTONS.items():
                if key != dtype:
                    getattr(self, btn_name).setChecked(False)
        else:
            if self.display_type == dtype:
                self.display_type = None
                self._clear_result_labels()

        # 버튼 텍스트 갱신
        btn_name, text_off, text_on = self._DISPLAY_BUTTONS[dtype]
        getattr(self, btn_name).setText(text_on if checked else text_off)

    def _on_toggle_laser(self, checked):
        self._on_display_toggle('laser', checked)

    def _on_toggle_conv(self, checked):
        self._on_display_toggle('conv', checked)

    def _on_toggle_fit_lines(self, checked):
        self._on_display_toggle('lines', checked)

    def _on_toggle_rgb_mask(self, checked):
        self._on_display_toggle('rgb', checked)

    def _clear_result_labels(self):
        """결과 라벨 초기화"""
        self.labelAngle.setText("-")
        self.labelPoints.setText("-")
        self.labelYRange.setText("-")
        self.labelCoeffs.setText("-")
        self.labelWidth.setText("-")
        self.labelLineCount.setText("-")
        self._last_coeffs = None
        self._last_angle_deg = None

    def set_camera_manager(self, camera_manager):
        self.camera_manager = camera_manager

    def update_frame(self, frame):
        """카메라 프레임 업데이트 — undistort 적용 후 레이저 오버레이"""
        if frame is None:
            return

        self.current_frame = frame.copy()

        # Undistorted 선택 시 보정 적용
        if self.radioUndistorted.isChecked() and self.camera_matrix is not None:
            base_frame = cv2.undistort(frame, self.camera_matrix, self.dist_coeffs)
        else:
            base_frame = frame.copy()

        # 표시용 프레임 저장 (이미지 저장 시 사용)
        self.display_frame = base_frame.copy()

        display = base_frame.copy()
        _draw = {
            'rgb':   self._draw_rgb_overlay,
            'lines': self._draw_line_overlay,
            'conv':  self._draw_conv_overlay,
            'laser': self._draw_laser_overlay,
        }
        if self.display_type in _draw:
            display = _draw[self.display_type](display)

        display_frame_on_label(display, self.labelCameraView)

    def _draw_rgb_overlay(self, frame):
        """RGB 채널 차이 기반 레이저 마스크를 표시 (dilation 포함)"""
        mask = extract_red_mask(frame)
        # 마스크 영역을 노란색으로 오버레이 (빨간 레이저와 구분)
        overlay = frame.copy()
        overlay[mask > 0] = (0, 255, 255)
        display = cv2.addWeighted(frame, 0.6, overlay, 0.4, 0)
        n_pixels = int(np.count_nonzero(mask))
        self.labelPoints.setText(f"{n_pixels} px")
        return display

    def _draw_laser_overlay(self, frame):
        """레이저 중심선 추출 후 핑크색으로 오버레이 (열 스캔, 수직 검출)"""
        min_w = int(self.spinMinStripe.value())
        max_w = int(self.spinMaxStripe.value())
        mad_scale = self.spinMadScale.value()

        # 각 열(column)에서 수직 방향 y-중심 추출 + 두께 자동 검출
        cols, centers_y, mask, est_width = extract_laser_center(
            frame, min_stripe_width=min_w, max_stripe_width=max_w,
        )

        if len(cols) == 0:
            self._clear_result_labels()
            return frame

        coeffs, inlier_cols, inlier_y = fit_laser_line(
            cols, centers_y, mad_scale=mad_scale,
        )

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
        self.labelWidth.setText(f"{est_width:.1f} px" if est_width > 0 else "-")

        return frame

    def _draw_conv_overlay(self, frame):
        """Conv 마스크 기반 레이저 중심 추출 — 핑크 점만 표시 (라인 피팅 없음)"""
        min_hw = max(1, int(self.spinMinStripe.value()) // 2)
        max_hw = max(min_hw + 1, int(self.spinMaxStripe.value()) // 2)

        cols, centers_y, est_width = extract_laser_center_conv(
            frame,
            min_half_width=min_hw,
            max_half_width=max_hw,
        )

        if len(cols) == 0:
            self._clear_result_labels()
            return frame

        BLUE = (255, 100, 0)

        for cx, cy in zip(cols, centers_y):
            cv2.circle(frame, (int(cx), int(round(cy))), 2, BLUE, -1)

        self.labelAngle.setText("-")
        self.labelCoeffs.setText("-")
        self.labelPoints.setText(f"{len(cols)}")
        if len(centers_y) > 0:
            self.labelYRange.setText(
                f"{centers_y.min():.1f} ~ {centers_y.max():.1f} px"
            )
        self.labelWidth.setText(f"{est_width:.1f} px" if est_width > 0 else "-")

        return frame

    def _draw_line_overlay(self, frame):
        """Conv 중심 추출 후 RANSAC 다중 직선 피팅"""
        min_hw = max(1, int(self.spinMinStripe.value()) // 2)
        max_hw = max(min_hw + 1, int(self.spinMaxStripe.value()) // 2)

        cols, centers_y, est_width = extract_laser_center_conv(
            frame, min_half_width=min_hw, max_half_width=max_hw,
        )

        if len(cols) == 0:
            self._clear_result_labels()
            return frame

        # 중심점을 파란 점으로 표시
        BLUE = (255, 100, 0)
        for cx, cy in zip(cols, centers_y):
            cv2.circle(frame, (int(cx), int(round(cy))), 1, BLUE, -1)

        # 다중 직선 피팅 (필터 최소화 — 50px 이상 모두 표시)
        threshold = self.spinMadScale.value()
        lines = fit_multiple_lines(
            cols, centers_y,
            residual_threshold=threshold,
            min_inliers=50,
            max_lines=10,
        )

        if not lines:
            self.labelLineCount.setText("0")
            self.labelAngle.setText("-")
            self.labelCoeffs.setText("-")
            self.labelPoints.setText(f"{len(cols)}")
            if len(centers_y) > 0:
                self.labelYRange.setText(
                    f"{centers_y.min():.1f} ~ {centers_y.max():.1f} px"
                )
            self.labelWidth.setText(f"{est_width:.1f} px" if est_width > 0 else "-")
            return frame

        # 각 직선을 다른 색으로 표시
        LINE_COLORS = [
            (0, 255, 0),      # Green
            (0, 255, 255),    # Yellow
            (255, 0, 255),    # Magenta
            (0, 165, 255),    # Orange
            (0, 0, 255),      # Red
        ]

        total_inliers = 0
        angle_strs = []
        coeff_strs = []

        for i, line in enumerate(lines):
            color = LINE_COLORS[i % len(LINE_COLORS)]
            inlier_cols = line['inlier_cols']
            coeffs = line['coeffs']

            # 직선 그리기 — inlier가 연속된 구간에서만
            sorted_x = np.sort(inlier_cols)
            GAP_THRESHOLD = 10  # px — 이보다 큰 간격이면 별도 세그먼트
            gaps = np.where(np.diff(sorted_x) > GAP_THRESHOLD)[0]
            segments = np.split(sorted_x, gaps + 1)
            valid_segments = [seg for seg in segments if len(seg) >= 2]
            for seg in valid_segments:
                x0, x1 = int(seg[0]), int(seg[-1])
                y0 = int(round(np.polyval(coeffs, x0)))
                y1 = int(round(np.polyval(coeffs, x1)))
                cv2.line(frame, (x0, y0), (x1, y1), color, 2)
                # 세그먼트 시작/끝점 원 표시
                cv2.circle(frame, (x0, y0), 6, color, 2)
                cv2.circle(frame, (x1, y1), 6, color, 2)

            # inlier 점을 해당 색으로 표시
            for cx, cy in zip(inlier_cols, line['inlier_y']):
                cv2.circle(frame, (int(cx), int(round(cy))), 2, color, -1)

            # 직선 번호 라벨 — 가장 긴 세그먼트 중심에 배치
            if valid_segments:
                longest_seg = max(valid_segments, key=len)
                label_x = int((longest_seg[0] + longest_seg[-1]) / 2)
            else:
                label_x = int((inlier_cols.min() + inlier_cols.max()) / 2)
            label_y = int(round(np.polyval(coeffs, label_x))) - 15
            cv2.putText(
                frame, f"L{i+1}", (label_x, label_y),
                cv2.FONT_HERSHEY_SIMPLEX, 1.0, color, 2,
            )

            total_inliers += len(inlier_cols)
            angle_strs.append(f"L{i+1}: {line['angle_deg']:.2f}")
            coeff_strs.append(
                f"L{i+1}: [{coeffs[0]:.5f}, {coeffs[1]:.1f}]"
            )

        # 결과 라벨 업데이트
        self.labelLineCount.setText(str(len(lines)))
        self.labelAngle.setText(" | ".join(angle_strs) + " deg")
        self.labelCoeffs.setText("\n".join(coeff_strs))
        self.labelPoints.setText(f"{total_inliers} / {len(cols)}")
        if len(centers_y) > 0:
            self.labelYRange.setText(
                f"{centers_y.min():.1f} ~ {centers_y.max():.1f} px"
            )
        self.labelWidth.setText(f"{est_width:.1f} px" if est_width > 0 else "-")

        return frame

    def _get_save_dir(self):
        """저장 폴더 반환 (프로젝트/images/laser)"""
        project_root = os.path.join(os.path.dirname(__file__), '..', '..')
        save_dir = os.path.join(os.path.abspath(project_root), 'images', 'laser')
        os.makedirs(save_dir, exist_ok=True)
        return save_dir

    def _on_save_image(self):
        """이미지 저장 (오버레이 ON이면 오버레이 포함, OFF면 표시 중인 프레임)"""
        if self.display_frame is None:
            self.log_message.emit("이미지 저장 실패: 카메라 프레임 없음")
            return

        save_frame = self.display_frame.copy()
        if self.show_line_overlay:
            save_frame = self._draw_line_overlay(save_frame)
        elif self.show_conv_overlay:
            save_frame = self._draw_conv_overlay(save_frame)
        elif self.show_laser_overlay:
            save_frame = self._draw_laser_overlay(save_frame)

        save_dir = self._get_save_dir()
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filepath = os.path.join(save_dir, f"laser_{timestamp}.png")

        success = cv2.imwrite(filepath, save_frame)
        if success and os.path.exists(filepath) and os.path.getsize(filepath) > 0:
            self.log_message.emit(f"이미지 저장: {filepath}")
        else:
            self.log_message.emit(f"이미지 저장 실패: {filepath}")

    def _log(self, msg):
        self.log_message.emit(msg)
