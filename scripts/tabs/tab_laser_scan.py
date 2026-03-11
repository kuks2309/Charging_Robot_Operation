#!/usr/bin/env python3
"""
레이저 스캔 탭
로봇 Z축 스캔으로 대상물 각도 측정
"""

import os
import json
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
    extract_laser_center_conv,
    fit_laser_line,
)


# UI 파일 경로
UI_DIR = os.path.join(os.path.dirname(__file__), '..', '..', 'ui')
TAB_LASER_SCAN_UI = os.path.join(UI_DIR, 'tab_laser_scan.ui')

# 캘리브레이션 파일 경로
CONFIG_DIR = os.path.join(os.path.dirname(__file__), '..', '..', 'config')
ARDUCAM_CALIB_FILE = os.path.join(CONFIG_DIR, 'arducam_calibration.yaml')
LASER_SCAN_ROI_FILE = os.path.join(CONFIG_DIR, 'laser_scan_roi.json')


class TabLaserScan(QWidget, JogMixin):
    """레이저 스캔 탭 (ArduCam 전용)"""

    log_message = pyqtSignal(str)
    camera_start_requested = pyqtSignal()
    camera_stop_requested = pyqtSignal()
    arducam_required = pyqtSignal()
    jog_move_requested = pyqtSignal(str, float)
    jog_rotate_requested = pyqtSignal(str, float)
    scan_start_requested = pyqtSignal(float, float)    # (step_mm, total_distance)
    scan_cancel_requested = pyqtSignal()

    def __init__(self, parent=None):
        super().__init__(parent)

        uic.loadUi(TAB_LASER_SCAN_UI, self)

        # 레이아웃 고정
        self.labelCameraView.setFixedSize(640, 360)
        self.widgetLeft.setFixedWidth(710)

        self.camera_manager = None
        self.current_frame = None
        self.display_frame = None

        # 캘리브레이션
        self.camera_matrix = None
        self.dist_coeffs = None
        self._calibration_loaded = False

        self._laser_detect_on = False
        self._scanning = False
        self._last_scan_results = None

        self._load_calibration()
        self._roi_config = None
        self._load_roi_config()
        self._connect_signals()

    def _connect_signals(self):
        """시그널 연결"""
        self.btnStartCamera.clicked.connect(self._on_start_camera)
        self.btnStopCamera.clicked.connect(self.camera_stop_requested.emit)
        self.btnSaveImage.clicked.connect(self._on_save_image)
        self.btnToggleLaser.toggled.connect(self._on_toggle_laser)
        self._connect_jog_buttons()

        # Z 스캔
        self.btnScanStart.clicked.connect(self._on_scan_start)
        self.btnScanStop.clicked.connect(self.scan_cancel_requested.emit)
        self.spinScanStep.valueChanged.connect(self._update_step_count_label)
        self.spinScanDistance.valueChanged.connect(self._update_step_count_label)
        self.btnSaveScanData.clicked.connect(self._on_save_scan_data)

    def _load_calibration(self):
        """ArduCam 캘리브레이션 파일 로드"""
        if self._calibration_loaded:
            return
        try:
            with open(ARDUCAM_CALIB_FILE, 'r') as f:
                data = yaml.safe_load(f)
            cam_data = data['camera_matrix']
            self.camera_matrix = np.array(cam_data['data'], dtype=np.float64).reshape(
                cam_data['rows'], cam_data['cols'])
            dist_data = data['distortion_coefficients']
            self.dist_coeffs = np.array(dist_data['data'], dtype=np.float64).reshape(
                dist_data['rows'], dist_data['cols'])
            self._calibration_loaded = True
        except Exception as e:
            self.camera_matrix = None
            self.dist_coeffs = None
            self.log_message.emit(f"캘리브레이션 로드 실패: {e}")

    def _load_roi_config(self):
        """ROI 설정 파일 로드"""
        try:
            with open(LASER_SCAN_ROI_FILE, 'r') as f:
                self._roi_config = json.load(f)
        except Exception as e:
            self._roi_config = None
            self.log_message.emit(f"ROI 설정 로드 실패: {e}")

    @staticmethod
    def _compute_roi_rects(frame_w, frame_h, roi_cfg):
        """좌우 ROI 사각형 좌표 계산 (픽셀 단위)"""
        cx = frame_w // 2
        hw = roi_cfg['roi_width'] // 2
        y0 = roi_cfg['y_offset']
        y1 = y0 + roi_cfg['roi_height']

        lx0 = cx - roi_cfg['offset_x'] - hw
        lx1 = cx - roi_cfg['offset_x'] + hw
        rx0 = cx + roi_cfg['offset_x'] - hw
        rx1 = cx + roi_cfg['offset_x'] + hw

        lx0 = int(max(0, lx0))
        lx1 = int(min(frame_w, lx1))
        rx0 = int(max(0, rx0))
        rx1 = int(min(frame_w, rx1))
        y0 = int(max(0, y0))
        y1 = int(min(frame_h, y1))

        return [(lx0, y0, lx1, y1), (rx0, y0, rx1, y1)]

    def _draw_roi_boxes(self, frame):
        """프레임에 ROI 박스 오버레이"""
        if self._roi_config is None:
            return
        roi_cfg = self._roi_config['roi']
        h, w = frame.shape[:2]
        rects = TabLaserScan._compute_roi_rects(w, h, roi_cfg)
        color = tuple(roi_cfg.get('color', [0, 0, 255]))
        thickness = roi_cfg.get('thickness', 2)
        for (x0, y0, x1, y1) in rects:
            cv2.rectangle(frame, (x0, y0), (x1, y1), color, thickness)

    @staticmethod
    def _detect_laser_in_roi(frame, roi_rect):
        """ROI 영역을 crop하여 레이저 라인 검출. 결과는 원본 좌표."""
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

    # 좌=Cyan(BGR), 우=Magenta(BGR)
    _ROI_COLORS = [(255, 255, 0), (255, 0, 255)]

    def _draw_laser_results(self, display, frame):
        """좌/우 ROI에서 레이저 검출 후 결과를 display에 오버레이"""
        if self._roi_config is None:
            return

        roi_cfg = self._roi_config['roi']
        h, w = frame.shape[:2]
        rects = TabLaserScan._compute_roi_rects(w, h, roi_cfg)
        labels = [
            (self.labelLeftAngle, self.labelLeftPoints, self.labelLeftYRange),
            (self.labelRightAngle, self.labelRightPoints, self.labelRightYRange),
        ]
        angles = []

        for i, rect in enumerate(rects):
            lbl_angle, lbl_points, lbl_yrange = labels[i]
            color = self._ROI_COLORS[i]

            result = TabLaserScan._detect_laser_in_roi(frame, rect)
            if result is None:
                lbl_angle.setText("-")
                lbl_points.setText("-")
                lbl_yrange.setText("-")
                angles.append(None)
                continue

            # inlier 포인트 표시
            for cx, cy in zip(result['inlier_cols'], result['inlier_y']):
                cv2.circle(display, (int(cx), int(round(cy))), 2, color, -1)

            # 피팅 직선 표시
            coeffs = result['coeffs']
            ic = result['inlier_cols']
            if len(ic) > 0:
                x_start, x_end = int(ic.min()), int(ic.max())
                y_start = int(round(np.polyval(coeffs, x_start)))
                y_end = int(round(np.polyval(coeffs, x_end)))
                cv2.line(display, (x_start, y_start), (x_end, y_end), color, 2)

            # 라벨 업데이트
            lbl_angle.setText(f"{result['angle_deg']:.3f} deg")
            lbl_points.setText(f"{len(result['inlier_cols'])} / {len(result['cols'])}")
            iy = result['inlier_y']
            if len(iy) > 0:
                lbl_yrange.setText(f"{iy.min():.1f} ~ {iy.max():.1f} px")
            angles.append(result['angle_deg'])

        # 좌우 각도 차이
        if angles[0] is not None and angles[1] is not None:
            diff = abs(angles[0] - angles[1])
            self.labelAngleDiff.setText(f"|L-R| = {diff:.3f} deg")
        else:
            self.labelAngleDiff.setText("-")

    def _on_toggle_laser(self, checked):
        """레이저 검출 토글"""
        self._laser_detect_on = checked
        self.btnToggleLaser.setText("레이저 검출 ON" if checked else "레이저 검출 OFF")
        if not checked:
            self._clear_result_labels()

    def _clear_result_labels(self):
        """결과 라벨 초기화"""
        self.labelLeftAngle.setText("-")
        self.labelLeftPoints.setText("-")
        self.labelLeftYRange.setText("-")
        self.labelRightAngle.setText("-")
        self.labelRightPoints.setText("-")
        self.labelRightYRange.setText("-")
        self.labelAngleDiff.setText("-")

    # ── Z축 스캔 ──

    def _on_scan_start(self):
        """스캔 시작 요청"""
        step_mm = self.spinScanStep.value()
        total_distance = self.spinScanDistance.value()
        self.scan_start_requested.emit(step_mm, total_distance)

    def _update_step_count_label(self):
        """스텝 수 라벨 업데이트"""
        step = self.spinScanStep.value()
        dist = self.spinScanDistance.value()
        if step > 0:
            count = int(round(dist / step))
            self.labelScanStepCount.setText(f"{count} 스텝")

    def set_scanning(self, active):
        """스캔 상태 설정 — 버튼 활성화/비활성화"""
        self._scanning = active
        self.btnScanStart.setEnabled(not active)
        self.btnScanStop.setEnabled(active)
        self.btnStartCamera.setEnabled(not active)
        self.btnStopCamera.setEnabled(not active)
        # 조그 버튼 비활성화 (스캔 중 이동 방지)
        for btn_name in ['btnJogXPlus', 'btnJogXMinus', 'btnJogYPlus', 'btnJogYMinus',
                         'btnJogZPlus', 'btnJogZMinus', 'btnJogRxPlus', 'btnJogRxMinus',
                         'btnJogRyPlus', 'btnJogRyMinus', 'btnJogRzPlus', 'btnJogRzMinus']:
            btn = getattr(self, btn_name, None)
            if btn:
                btn.setEnabled(not active)
        # 스캔 시작 시 이전 결과 초기화 + 레이저 검출 자동 ON
        if active:
            self._last_scan_results = None
            self.btnSaveScanData.setEnabled(False)
            self.tableScanData.setRowCount(0)
        if active and not self._laser_detect_on:
            self.btnToggleLaser.setChecked(True)

    def update_scan_status(self, text):
        """스캔 상태 텍스트 업데이트 (서비스 시그널 슬롯)"""
        self.labelScanStatus.setText(text)

    def update_scan_progress(self, current, total):
        """스캔 진행바 업데이트 (서비스 시그널 슬롯)"""
        self.progressScan.setMaximum(total)
        self.progressScan.setValue(current)

    def add_scan_data_row(self, step_idx, z_mm, left_result, right_result):
        """스캔 데이터 행 추가 (서비스 시그널 슬롯)"""
        row = self.tableScanData.rowCount()
        self.tableScanData.insertRow(row)

        from PyQt5.QtWidgets import QTableWidgetItem
        self.tableScanData.setItem(row, 0, QTableWidgetItem(str(step_idx)))
        self.tableScanData.setItem(row, 1, QTableWidgetItem(f"{z_mm:.1f}"))

        if left_result is not None:
            self.tableScanData.setItem(row, 2, QTableWidgetItem(f"{left_result['y_mean']:.1f}"))
            self.tableScanData.setItem(row, 4, QTableWidgetItem(f"{left_result['angle_deg']:.2f}"))
        else:
            self.tableScanData.setItem(row, 2, QTableWidgetItem("-"))
            self.tableScanData.setItem(row, 4, QTableWidgetItem("-"))

        if right_result is not None:
            self.tableScanData.setItem(row, 3, QTableWidgetItem(f"{right_result['y_mean']:.1f}"))
            self.tableScanData.setItem(row, 5, QTableWidgetItem(f"{right_result['angle_deg']:.2f}"))
        else:
            self.tableScanData.setItem(row, 3, QTableWidgetItem("-"))
            self.tableScanData.setItem(row, 5, QTableWidgetItem("-"))

        self.tableScanData.scrollToBottom()

    def show_scan_results(self, results):
        """스캔 최종 결과 표시 (서비스 시그널 슬롯)"""
        self._last_scan_results = results
        self.btnSaveScanData.setEnabled(True)
        left = results.get('left')
        right = results.get('right')

        if left and left.get('slope_px_per_mm') is not None:
            self.labelLeftSlope.setText(f"{left['slope_px_per_mm']:.3f} px/mm (R²={left['r_squared']:.4f})")
        else:
            self.labelLeftSlope.setText("- px/mm")

        if right and right.get('slope_px_per_mm') is not None:
            self.labelRightSlope.setText(f"{right['slope_px_per_mm']:.3f} px/mm (R²={right['r_squared']:.4f})")
        else:
            self.labelRightSlope.setText("- px/mm")

        # 삼각측량 각도 추정 표시
        angle_est = results.get('angle_estimate')
        if angle_est and angle_est.get('estimated_deg') is not None:
            deg = angle_est['estimated_deg']
            left_deg = angle_est.get('left_deg')
            right_deg = angle_est.get('right_deg')
            parts = [f"추정 각도: {deg:.2f}°"]
            if left_deg is not None and right_deg is not None:
                parts.append(f"(L={left_deg:.2f}° R={right_deg:.2f}°)")
            self.labelTiltEstimate.setText(" ".join(parts))
        else:
            err = angle_est.get('error', '') if angle_est else 'no_calibration'
            if err == 'no_calibration':
                self.labelTiltEstimate.setText("각도 추정: 캘리브레이션 없음")
            else:
                self.labelTiltEstimate.setText("각도 추정: -")

    def reset_scan_ui(self):
        """스캔 UI 초기화 (진행바/상태만 — 저장 버튼은 결과 유지)"""
        self.progressScan.setValue(0)
        self.labelScanStatus.setText("-")

    def _on_save_scan_data(self):
        """수동 스캔 데이터 저장 (data/laser_scan/)"""
        if self._last_scan_results is None:
            self.log_message.emit("저장할 스캔 데이터가 없습니다.")
            return

        project_root = os.path.join(os.path.dirname(__file__), '..', '..')
        save_dir = os.path.join(os.path.abspath(project_root), 'data', 'laser_scan')
        os.makedirs(save_dir, exist_ok=True)

        ts = datetime.now().strftime('%Y%m%d_%H%M%S')
        results = self._last_scan_results

        # JSON 저장
        json_data = {
            'timestamp': datetime.now().isoformat(),
            'parameters': results.get('parameters'),
            'left': results.get('left'),
            'right': results.get('right'),
            'angle_estimate': results.get('angle_estimate'),
            'raw_data': results.get('raw_data', []),
        }
        json_path = os.path.join(save_dir, f'scan_{ts}.json')
        try:
            with open(json_path, 'w', encoding='utf-8') as f:
                json.dump(json_data, f, indent=2, ensure_ascii=False, default=str)
        except Exception as e:
            self.log_message.emit(f"JSON 저장 실패: {e}")
            return

        # CSV 저장
        csv_path = os.path.join(save_dir, f'scan_{ts}.csv')
        try:
            import csv
            with open(csv_path, 'w', newline='', encoding='utf-8') as f:
                writer = csv.writer(f)
                writer.writerow([
                    'step_index', 'z_offset_mm',
                    'left_y_mean', 'left_angle_deg',
                    'right_y_mean', 'right_angle_deg',
                ])
                for entry in results.get('raw_data', []):
                    left = entry.get('left_roi')
                    right = entry.get('right_roi')
                    writer.writerow([
                        entry.get('step_index', ''),
                        f"{entry.get('z_offset_mm', 0):.3f}",
                        f"{left['y_mean']:.3f}" if left else '',
                        f"{left['angle_deg']:.4f}" if left else '',
                        f"{right['y_mean']:.3f}" if right else '',
                        f"{right['angle_deg']:.4f}" if right else '',
                    ])
        except Exception as e:
            self.log_message.emit(f"CSV 저장 실패: {e}")
            return

        self.log_message.emit(f"스캔 데이터 저장: {json_path}")
        self.log_message.emit(f"CSV 저장: {csv_path}")

    def _on_start_camera(self):
        """카메라 시작 — ArduCam 강제 선택 후 시작"""
        self.arducam_required.emit()
        self.camera_start_requested.emit()

    def set_camera_manager(self, camera_manager):
        self.camera_manager = camera_manager

    def update_frame(self, frame):
        """카메라 프레임 업데이트"""
        if frame is None:
            return

        self.current_frame = frame.copy()

        if self.radioUndistorted.isChecked() and self.camera_matrix is not None:
            base_frame = cv2.undistort(frame, self.camera_matrix, self.dist_coeffs)
        else:
            base_frame = frame.copy()

        self.display_frame = base_frame.copy()  # clean copy for saving
        display = base_frame.copy()  # separate copy for display overlays
        self._draw_roi_boxes(display)
        if self._laser_detect_on:
            self._draw_laser_results(display, base_frame)
        display_frame_on_label(display, self.labelCameraView)

    def deactivate(self):
        """탭 비활성화 시 초기화"""
        if self._scanning:
            self.scan_cancel_requested.emit()
        self._laser_detect_on = False
        self.btnToggleLaser.setChecked(False)
        self._clear_result_labels()

    def _on_save_image(self):
        """이미지 저장"""
        if self.display_frame is None:
            self.log_message.emit("이미지 저장 실패: 카메라 프레임 없음")
            return

        project_root = os.path.join(os.path.dirname(__file__), '..', '..')
        save_dir = os.path.join(os.path.abspath(project_root), 'images', 'laser_scan')
        os.makedirs(save_dir, exist_ok=True)

        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filepath = os.path.join(save_dir, f"scan_{timestamp}.png")

        success = cv2.imwrite(filepath, self.display_frame)
        if success and os.path.exists(filepath) and os.path.getsize(filepath) > 0:
            self.log_message.emit(f"이미지 저장: {filepath}")
        else:
            self.log_message.emit(f"이미지 저장 실패: {filepath}")

    def _log(self, msg):
        self.log_message.emit(msg)
