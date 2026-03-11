#!/usr/bin/env python3
"""
레이저 캘리브레이션 탭
ArduCam 카메라로 라인 레이저 중심선을 추출하여 핑크색으로 표시
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
from utils.overlay import draw_image_center_crosshair
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
LASER_CALIB_FILE = os.path.join(CONFIG_DIR, 'laser_calibration.json')


class TabLaserCalibration(QWidget, JogMixin):
    """레이저 캘리브레이션 탭 (ArduCam 전용)"""

    log_message = pyqtSignal(str)
    camera_start_requested = pyqtSignal()
    camera_stop_requested = pyqtSignal()
    arducam_required = pyqtSignal()
    jog_move_requested = pyqtSignal(str, float)
    jog_rotate_requested = pyqtSignal(str, float)
    calib_align_aruco_requested = pyqtSignal()
    calib_adjust_z_requested = pyqtSignal()
    calib_save_pos_requested = pyqtSignal()
    calib_move_x_adjust_z_requested = pyqtSignal()
    calib_save_compare_requested = pyqtSignal()
    calib_auto_requested = pyqtSignal()
    calib_cancel_requested = pyqtSignal()

    def __init__(self, parent=None):
        super().__init__(parent)

        # UI 로드
        uic.loadUi(TAB_LASER_CALIBRATION_UI, self)

        # === 레이아웃 고정 (내용 변화에도 UI 안정) ===
        self.labelCameraView.setFixedSize(640, 360)      # 카메라 1920x1080의 1/3
        self.widgetLeft.setFixedWidth(710)                # 좌측 패널 폭 고정
        # widgetRight, groupResult 크기는 UI 파일에서 관리
        self.labelAngle.setWordWrap(True)                 # 긴 기울기 텍스트 줄바꿈

        self.robot = None
        self.camera_manager = None
        self.current_frame = None       # 원본 프레임
        self._raw_frame = None          # 오버레이 없는 순수 원본 프레임 (ArUco 검출 전용)
        self.display_frame = None       # 표시용 프레임 (original 또는 undistorted)
        # 표시 모드: None, 'laser', 'conv', 'lines', 'hsv'
        self.display_type = None

        # 캘리브레이션
        self.camera_matrix = None
        self.dist_coeffs = None
        self._calibration_loaded = False

        # 캘리브레이션 데이터
        self._calib_data = []
        self._calib_pos1 = None
        self._calib_step = 0
        self._z_adjust_cancel = False
        self._auto_calib_running = False
        self._current_pose = None
        self._show_aruco_overlay = False

        # ArUco 기반 레이저 ROI
        self._marker_list = []  # detect_marker_centers 결과
        self._roi_mask = None   # 프레임 크기 bool mask

        # 마지막 추출 결과 캐시
        self._last_coeffs = None
        self._last_angle_deg = None
        self._last_inlier_count = 0
        self._last_total_count = 0

        self._load_calibration()
        self._connect_signals()
        self._init_calib_table()

    def _connect_signals(self):
        """시그널 연결"""
        self.btnStartCamera.clicked.connect(self._on_start_camera)
        self.btnStopCamera.clicked.connect(self.camera_stop_requested.emit)
        self.btnToggleLaser.toggled.connect(self._on_toggle_laser)
        self.btnConvCenter.toggled.connect(self._on_toggle_conv)
        self.btnFitLines.toggled.connect(self._on_toggle_fit_lines)
        self.btnRgbMask.toggled.connect(self._on_toggle_rgb_mask)
        self.btnSaveImage.clicked.connect(self._on_save_image)
        self.btnSetDetectionPose.clicked.connect(self._on_set_detection_pose)
        self._connect_jog_buttons()
        # 캘리브레이션 버튼
        self.btnAlignAruco.clicked.connect(self._on_btn_align_aruco)
        self.btnAdjustZ.clicked.connect(self._on_btn_adjust_z)
        self.btnSavePosition.clicked.connect(self.calib_save_pos_requested.emit)
        self.btnMoveXAdjustZ.clicked.connect(self.calib_move_x_adjust_z_requested.emit)
        self.btnSaveCompare.clicked.connect(self.calib_save_compare_requested.emit)
        self.btnAutoCalib.clicked.connect(self._on_btn_auto_calib)
        self.btnClearCalibData.clicked.connect(self._clear_calib_data)
        self.btnSaveCalibResult.clicked.connect(self._save_calib_result)

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

    # 버튼 ↔ display_type 매핑 (UI 버튼 텍스트와 일치)
    _DISPLAY_BUTTONS = {
        'laser': ('btnToggleLaser', '레이저 표시',  '레이저 표시 OFF'),
        'conv':  ('btnConvCenter',  '중심 추출',    '중심 추출 OFF'),
        'lines': ('btnFitLines',    '직선 추출',    '직선 추출 OFF'),
        'rgb':   ('btnRgbMask',     'RGB 추출',     'RGB 추출 OFF'),
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

    def deactivate(self):
        """탭 비활성화 시 표시 모드 토글 버튼 초기화"""
        self.display_type = None
        for key, (btn_name, text_off, _) in self._DISPLAY_BUTTONS.items():
            btn = getattr(self, btn_name, None)
            if btn:
                btn.setChecked(False)
                btn.setText(text_off)
        self._clear_result_labels()

    def set_robot(self, robot):
        self.robot = robot

    def _on_set_detection_pose(self):
        """set_rz.py 방식: TF3 전환 → 현재 XYZ + 목표 RxRyRz movel → TF4 복귀"""
        if self.robot is None:
            from PyQt5.QtWidgets import QMessageBox
            QMessageBox.warning(self, "경고", "로봇이 연결되지 않았습니다.")
            return

        import time
        from PyQt5.QtWidgets import QApplication, QMessageBox

        try:
            self._log("Detection Pose: TF3으로 전환")
            success, msg = self.robot.send_set_toolframe(3, wait=True)
            if not success:
                self._log(f"TF3 전환 실패: {msg}")
                return
            time.sleep(0.5)

            pose = self.robot.read_current_pose()
            if pose is None:
                QMessageBox.warning(self, "경고", "TCP 좌표를 읽을 수 없습니다.")
                return

            x, y, z = pose[0], pose[1], pose[2]
            tgt_rx, tgt_ry, tgt_rz = 90.0, 0.0, 90.0

            self._log(f"현재: X={x:.1f} Y={y:.1f} Z={z:.1f} Rx={pose[3]:.1f} Ry={pose[4]:.1f} Rz={pose[5]:.1f}")
            self._log(f"목표: X={x:.1f} Y={y:.1f} Z={z:.1f} Rx={tgt_rx:.1f} Ry={tgt_ry:.1f} Rz={tgt_rz:.1f}")

            to_int16 = self.robot.to_uint16
            regs = [
                to_int16(int(round(x * 10))),
                to_int16(int(round(y * 10))),
                to_int16(int(round(z * 10))),
                to_int16(int(round(tgt_rx * 10))),
                to_int16(int(round(tgt_ry * 10))),
                to_int16(int(round(tgt_rz * 10))),
            ]
            self.robot.write_registers(self.robot.REGISTER_POSE_MAIN, regs)
            self.robot.write_command(self.robot.CMD_MOVE_TO_POSE)

            success, msg = self.robot.wait_for_done_motion_aware(
                process_events_callback=QApplication.processEvents
            )
            if not success:
                self._log(f"Detection Pose 이동 실패: {msg}")
                return

            self._log("Detection Pose 이동 완료")
            time.sleep(0.5)

            success, msg = self.robot.send_set_toolframe(4, wait=True)
            if success:
                self._log("TF4 복귀 완료")
            else:
                self._log(f"TF4 복귀 실패: {msg}")

        except Exception as e:
            from PyQt5.QtWidgets import QMessageBox
            QMessageBox.warning(self, "오류", f"Detection Pose 실패: {e}")

    def set_camera_manager(self, camera_manager):
        self.camera_manager = camera_manager

    def set_markers(self, markers: list):
        """ArUco 마커 검출 결과 저장 + ROI 마스크 갱신.

        Args:
            markers: detect_marker_centers 결과 list[dict]
                     각 dict에 'corners' (4x2 ndarray) 포함
        """
        self._marker_list = markers if markers else []

    def compute_roi_mask(self, frame_shape):
        """마커 corners 기반 레이저 검출 ROI 마스크 생성.

        각 마커 아래쪽에 ROI 박스를 설정:
          - 상단: 마커 하단 가장자리
          - 하단: 프레임 하단
          - 좌/우: 마커 좌우 가장자리 ± 마커 폭 만큼 확장

        Returns:
            np.ndarray (bool): ROI mask (H, W), 마커 미검출 시 전체 True
        """
        h, w = frame_shape[:2]
        mask = np.zeros((h, w), dtype=bool)

        if not self._marker_list:
            return mask  # 마커 없으면 전체 False → 검출 안함

        for m in self._marker_list:
            corners = m['corners']
            if len(corners.shape) == 3:
                corners = corners[0]
            # 마커 경계
            x_min_m = corners[:, 0].min()
            x_max_m = corners[:, 0].max()
            y_max_m = corners[:, 1].max()  # 마커 하단
            marker_w = x_max_m - x_min_m

            # ROI: 마커 아래, 마커 폭 × 마커 폭 (정사각형)
            roi_x0 = max(0, int(x_min_m))
            roi_x1 = min(w, int(x_max_m))
            roi_y0 = int(y_max_m)
            roi_y1 = min(h, int(y_max_m + marker_w))
            mask[roi_y0:roi_y1, roi_x0:roi_x1] = True

        self._roi_mask = mask
        return mask

    def _draw_roi_boxes(self, frame):
        """ROI 영역을 빨간 박스로 표시"""
        h, w = frame.shape[:2]
        for m in self._marker_list:
            corners = m['corners']
            if len(corners.shape) == 3:
                corners = corners[0]
            x_min_m = corners[:, 0].min()
            x_max_m = corners[:, 0].max()
            y_max_m = corners[:, 1].max()
            marker_w = x_max_m - x_min_m
            roi_x0 = max(0, int(x_min_m))
            roi_x1 = min(w, int(x_max_m))
            roi_y0 = int(y_max_m)
            roi_y1 = min(h, int(y_max_m + marker_w))
            cv2.rectangle(frame, (roi_x0, roi_y0), (roi_x1, roi_y1), (0, 255, 255), 2)

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

        # ROI 마스크 갱신 (매 프레임 ArUco 위치 반영)
        self.compute_roi_mask(base_frame.shape)

        display = base_frame.copy()

        # ROI 영역 시각화 (빨간 박스)
        if self._roi_mask is not None and self._marker_list:
            self._draw_roi_boxes(display)

        _draw = {
            'rgb':   self._draw_rgb_overlay,
            'lines': self._draw_line_overlay,
            'conv':  self._draw_conv_overlay,
            'laser': self._draw_laser_overlay,
        }
        if self.display_type in _draw:
            display = _draw[self.display_type](display)

        # target_y 참조선 + laser_y 현재 위치 표시
        h, w = display.shape[:2]
        target_y = self.get_target_y()
        if target_y is not None:
            ty = int(round(target_y))
            if 0 <= ty < h:
                cv2.line(display, (0, ty), (w, ty), (0, 255, 0), 1)
                cv2.putText(display, f"target={target_y:.0f}px (offset={self.spinTargetLaserY.value()})",
                            (w - 420, ty - 8),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.68, (0, 255, 0), 2)

        # 현재 laser_y 측정 (get_current_laser_y 재사용)
        laser_y = self.get_current_laser_y(self.current_frame)
        if laser_y is not None:
            ly = int(round(laser_y))
            if 0 <= ly < h:
                cv2.line(display, (0, ly), (w, ly), (0, 0, 255), 1)
                cv2.putText(display, f"laser={laser_y:.1f}px", (w - 160, ly + 15),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 255), 1)

        # 이미지 중심 십자선 (항상 표시)
        draw_image_center_crosshair(display)

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

    def _filter_by_roi(self, cols, centers_y):
        """ROI 마스크로 점 필터링. 마스크 없으면 전부 통과."""
        if self._roi_mask is None or not self._marker_list:
            return cols, centers_y
        h, w = self._roi_mask.shape
        keep = []
        for i, (cx, cy) in enumerate(zip(cols, centers_y)):
            ix, iy = int(round(cx)), int(round(cy))
            if 0 <= iy < h and 0 <= ix < w and self._roi_mask[iy, ix]:
                keep.append(i)
        if not keep:
            return np.array([]), np.array([])
        keep = np.array(keep)
        return cols[keep], centers_y[keep]

    def _draw_conv_overlay(self, frame):
        """Conv 마스크 기반 레이저 중심 추출 — 핑크 점만 표시 (라인 피팅 없음)"""
        min_hw = max(1, int(self.spinMinStripe.value()) // 2)
        max_hw = max(min_hw + 1, int(self.spinMaxStripe.value()) // 2)

        cols, centers_y, est_width = extract_laser_center_conv(
            frame,
            min_half_width=min_hw,
            max_half_width=max_hw,
        )

        # ROI 필터링
        cols, centers_y = self._filter_by_roi(cols, centers_y)

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

        # ROI 필터링
        cols, centers_y = self._filter_by_roi(cols, centers_y)

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
        if self.display_type == 'lines':
            save_frame = self._draw_line_overlay(save_frame)
        elif self.display_type == 'conv':
            save_frame = self._draw_conv_overlay(save_frame)
        elif self.display_type == 'laser':
            save_frame = self._draw_laser_overlay(save_frame)
        elif self.display_type == 'rgb':
            save_frame = self._draw_rgb_overlay(save_frame)

        save_dir = self._get_save_dir()
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filepath = os.path.join(save_dir, f"laser_{timestamp}.png")

        success = cv2.imwrite(filepath, save_frame)
        if success and os.path.exists(filepath) and os.path.getsize(filepath) > 0:
            self.log_message.emit(f"이미지 저장: {filepath}")
        else:
            self.log_message.emit(f"이미지 저장 실패: {filepath}")

    # ==================== 캘리브레이션 메서드 ====================

    def _init_calib_table(self):
        """캘리브레이션 테이블 초기화"""
        table = self.tableCalibData
        table.setColumnCount(7)
        table.setHorizontalHeaderLabels(['#', 'X1(mm)', 'Z1(mm)', 'X2(mm)', 'Z2(mm)', 'ΔX(mm)', 'ΔZ(mm)'])
        table.setColumnWidth(0, 30)
        for col in range(1, 7):
            table.setColumnWidth(col, 65)
        table.setRowCount(0)
        # 프로그레스바 초기화
        self.progressCalib.setMaximum(self.spinRepeatCount.value())
        self.progressCalib.setValue(0)

    def _on_btn_align_aruco(self):
        """ArUco 정렬 버튼 → ArUco 검출 + Ry/Y 보정 + Z 조정 (재클릭=취소)"""
        if self._calib_step > 0:
            # 이미 실행 중이면 취소
            self._z_adjust_cancel = True
            self._update_calib_step(0, "취소 중...")
            return
        self._z_adjust_cancel = False
        self._update_calib_step(1, "ArUco 정렬 요청...")
        self.calib_align_aruco_requested.emit()

    def _on_btn_adjust_z(self):
        """Z 조정 버튼 토글 (실행/취소)"""
        if self._calib_step == 0:
            self._z_adjust_cancel = False
            self._update_calib_step(2, "Z 조정 요청 중...")
            self.calib_adjust_z_requested.emit()
        else:
            self._z_adjust_cancel = True
            self._update_calib_step(0, "Z 조정 취소됨")

    def _on_btn_auto_calib(self):
        """자동 캘리브레이션 버튼 토글"""
        if not self._auto_calib_running:
            self._auto_calib_running = True
            self._z_adjust_cancel = False
            self.progressCalib.setMaximum(self.spinRepeatCount.value())
            self.progressCalib.setValue(0)
            self.btnAutoCalib.setText("자동 캘리브레이션 취소")
            self.btnAutoCalib.setStyleSheet(
                "QPushButton { padding: 8px; font-weight: bold; font-size: 14px; "
                "background-color: #C62828; color: white; border-radius: 4px; }"
                "QPushButton:hover { background-color: #E53935; }")
            self.calib_auto_requested.emit()
        else:
            self._auto_calib_running = False
            self._z_adjust_cancel = True
            self.calib_cancel_requested.emit()
            self._reset_auto_calib_ui()

    def _reset_auto_calib_ui(self):
        """자동 캘리브레이션 UI 초기 상태로 복원"""
        self._auto_calib_running = False
        self.btnAutoCalib.setText("자동 캘리브레이션")
        self.btnAutoCalib.setStyleSheet(
            "QPushButton { padding: 8px; font-weight: bold; font-size: 14px; "
            "background-color: #1565C0; color: white; border-radius: 4px; }"
            "QPushButton:hover { background-color: #1976D2; }"
            "QPushButton:pressed { background-color: #0D47A1; }"
            "QPushButton:disabled { background-color: #90A4AE; color: #eceff1; }")

    def _update_calib_step(self, step_num, message):
        """캘리브레이션 단계 라벨 업데이트"""
        self._calib_step = step_num
        self.labelCalibStep.setText(message)

    def set_current_pose(self, x, y, z, rx, ry, rz):
        """main_window에서 TCP 위치 전달받기"""
        self._current_pose = (x, y, z, rx, ry, rz)

    def set_z_adjust_status(self, running):
        """Z 조정 상태 → 버튼 텍스트 토글"""
        if running:
            self.btnAdjustZ.setText("2. Z 조정 취소")
            self._z_adjust_cancel = False
        else:
            self.btnAdjustZ.setText("2. Z 조정")
            self._z_adjust_cancel = False
            self._calib_step = 0

    def get_marker_center_y(self):
        """검출된 ArUco 마커들의 평균 중심 Y 반환. 마커 없으면 None."""
        if not self._marker_list:
            return None
        ys = [m['center'][1] for m in self._marker_list if 'center' in m]
        if not ys:
            return None
        return float(np.mean(ys))

    def get_target_y(self):
        """마커 중심 Y + 오프셋 = 레이저 목표 Y (절대 픽셀). 마커 없으면 None."""
        marker_cy = self.get_marker_center_y()
        if marker_cy is None:
            return None
        return marker_cy + self.spinTargetLaserY.value()

    def get_current_laser_y(self, frame):
        """프레임에서 레이저 라인 Y 위치 측정 (main_window에서 호출)"""
        if frame is None:
            return None
        min_hw = max(1, int(self.spinMinStripe.value()) // 2)
        max_hw = max(min_hw + 1, int(self.spinMaxStripe.value()) // 2)
        cols, centers_y, est_width = extract_laser_center_conv(
            frame, min_half_width=min_hw, max_half_width=max_hw)
        if len(cols) == 0:
            return None
        coeffs, inlier_cols, inlier_y = fit_laser_line(
            cols, centers_y, mad_scale=self.spinMadScale.value())
        if coeffs is None or len(coeffs) < 2:
            return None
        center_x = frame.shape[1] // 2
        laser_y = float(np.polyval(coeffs, center_x))
        return laser_y

    def _add_calib_row(self, x1, z1, x2, z2):
        """캘리브레이션 데이터 행 추가"""
        dx = x2 - x1
        dz = z2 - z1
        self._calib_data.append({
            'x1': x1, 'z1': z1, 'x2': x2, 'z2': z2, 'dx': dx, 'dz': dz
        })
        row = self.tableCalibData.rowCount()
        self.tableCalibData.insertRow(row)
        from PyQt5.QtWidgets import QTableWidgetItem
        values = [str(row + 1), f"{x1:.2f}", f"{z1:.2f}", f"{x2:.2f}", f"{z2:.2f}", f"{dx:.2f}", f"{dz:.2f}"]
        for col, val in enumerate(values):
            self.tableCalibData.setItem(row, col, QTableWidgetItem(val))

    def _clear_calib_data(self):
        """캘리브레이션 데이터 초기화"""
        self._calib_data.clear()
        self._calib_pos1 = None
        self._calib_step = 0
        self.tableCalibData.setRowCount(0)
        self.progressCalib.setValue(0)
        self._update_calib_step(0, "대기 중")
        self._log("캘리브레이션 데이터 초기화")

    def _save_calib_result(self):
        """캘리브레이션 결과 JSON 저장"""
        if not self._calib_data:
            self._log("저장할 캘리브레이션 데이터가 없습니다")
            return
        os.makedirs(os.path.dirname(LASER_CALIB_FILE), exist_ok=True)
        result = {
            'timestamp': datetime.now().isoformat(),
            'marker_y_offset_px': self.spinTargetLaserY.value(),
            'marker_center_y_px': self.get_marker_center_y(),
            'target_laser_y_px': self.get_target_y(),
            'x_move_step_mm': self.spinXMoveStep.value(),
            'data': self._calib_data,
            'summary': {
                'count': len(self._calib_data),
                'avg_dx': sum(d['dx'] for d in self._calib_data) / len(self._calib_data),
                'avg_dz': sum(d['dz'] for d in self._calib_data) / len(self._calib_data),
            }
        }
        with open(LASER_CALIB_FILE, 'w') as f:
            json.dump(result, f, indent=2, ensure_ascii=False)
        self._log(f"캘리브레이션 결과 저장: {LASER_CALIB_FILE}")

    def _log(self, msg):
        self.log_message.emit(msg)
