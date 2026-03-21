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
from PyQt5.QtCore import pyqtSignal, QTimer

from utils.common import display_frame_on_label
from utils.overlay import draw_image_center_crosshair
from utils.image_processing import undistort_frame, draw_roi_box, draw_laser_fit_line, compute_roi_rects
from tabs.jog_mixin import JogMixin
from services.laser_detection_service import LaserDetectionService


# UI 파일 경로
UI_DIR = os.path.join(os.path.dirname(__file__), '..', '..', 'ui')
TAB_LASER_CALIBRATION_UI = os.path.join(UI_DIR, 'tab_laser_calibration.ui')

# 캘리브레이션 파일 경로
CONFIG_DIR = os.path.join(os.path.dirname(__file__), '..', '..', 'config')
ARDUCAM_CALIB_FILE = os.path.join(CONFIG_DIR, 'calibration', 'arducam', 'arducam_calibration.yaml')
LASER_CALIB_FILE = os.path.join(CONFIG_DIR, 'laser_calibration.json')
LASER_VERT_SCAN_ROI_FILE = os.path.join(CONFIG_DIR, 'laser', 'vertical', 'laser_vertical_scan_roi.json')
LASER_HORIZ_SCAN_ROI_FILE = os.path.join(CONFIG_DIR, 'laser', 'horizontal', 'laser_horizontal_scan_roi.json')
JIG_POSITIONS_FILE = os.path.join(CONFIG_DIR, 'laser', 'vertical', 'laser_jig_positions.json')
LASER_JIG_STEP_CONFIG_FILE = os.path.join(CONFIG_DIR, 'laser', 'vertical', 'laser_jig_step_config.json')
LASER_VERT_CALIB_FILE = os.path.join(CONFIG_DIR, 'laser', 'vertical', 'laser_vertical_triangulation_calib.json')
LASER_DETECTION_POSE_FILE = os.path.join(CONFIG_DIR, 'laser', 'laser_detection_pose.json')
LASER_VERT_CALIB_DATA_DIR = os.path.join(
    os.path.dirname(__file__), '..', '..', 'data', 'laser_scan', 'calibration', 'vertical'
)


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
    calib_auto_vert_scan_requested = pyqtSignal()
    calib_auto_vert_scan_cancel_requested = pyqtSignal()

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
        self._auto_vert_scan_running = False
        self._current_pose = None
        self._show_aruco_overlay = False

        # 지그 초기 위치 (TF5 기준)
        self._jig_pos_vertical = None
        self._jig_pos_horizontal = None

        # 수직 스캔 데이터
        self._vert_scan_data = []       # list of {'step': int, 'y_px': float} — 최근 캡처
        self._multi_vert_captures = []  # list of {'z_tcp': float, 'steps': [...]} — 누적
        self._vert_scan_base_data = []  # list of {'y_px': float} — base ROI 데이터
        self._step_labels = []          # labelStep1..5
        self._step_res_labels = []      # labelStepRes1..5

        # 캘리브레이션 지그 ROI / Laser 검출
        self._calib_roi_active = False
        self._jig_detect_active = False
        self._roi_cfg = {}
        self._roi2_cfg = None
        self._all_rois = []  # [(name, config_dict), ...]
        self._frame_cx = 960
        self._frame_cy = 540

        # ArUco 기반 레이저 ROI
        self._marker_list = []  # detect_marker_centers 결과
        self._roi_mask = None   # 프레임 크기 bool mask

        # 마지막 추출 결과 캐시
        self._last_coeffs = None
        self._last_angle_deg = None
        self._last_inlier_count = 0
        self._last_total_count = 0

        self._load_calibration()
        self._load_calib_roi_config(LASER_VERT_SCAN_ROI_FILE)
        self._load_jig_positions()
        self._connect_signals()
        self._init_calib_table()
        self._init_vert_scan_ui()

        # 로봇 상태 업데이트 타이머 (1초 주기, 탭 생존 기간 동안 유지)
        self._robot_status_timer = QTimer()
        self._robot_status_timer.timeout.connect(self._update_robot_status_ui)
        self._robot_status_timer.start(1000)

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
        self.btnCreateROIVertical.clicked.connect(self._on_create_roi_vertical)
        self.btnCreateROIHorizontal.clicked.connect(self._on_create_roi_horizontal)
        self.btnLaserDetect.clicked.connect(self._on_toggle_jig_detect)
        self.btnSaveJigPosVertical.clicked.connect(self._on_save_jig_pos_vertical)
        self.btnMoveJigPosVertical.clicked.connect(self._on_move_jig_pos_vertical)
        self.btnSaveJigPosHorizontal.clicked.connect(self._on_save_jig_pos_horizontal)
        self.btnMoveJigPosHorizontal.clicked.connect(self._on_move_jig_pos_horizontal)
        self.btnFitVertModel.clicked.connect(self._on_fit_vert_model)
        self.btnAutoVertScan.clicked.connect(self._on_btn_auto_vert_scan)
        self.btnCancelAutoScan.clicked.connect(self.calib_auto_vert_scan_cancel_requested.emit)
        self.btnCancelAutoScan.setVisible(False)
        self.btnHorizLaserOn.toggled.connect(self._on_toggle_horiz_laser)
        self.btnVertLaserOn.toggled.connect(self._on_toggle_vert_laser)

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

    def _load_calib_roi_config(self, roi_file):
        """레이저 캘리브레이션 ROI 설정 로드 — config 키 이름을 동적으로 읽음."""
        try:
            with open(roi_file, 'r') as f:
                data = json.load(f)
            # config에서 dict 값인 키만 ROI로 인식 (_note 등 문자열 제외)
            self._all_rois = [
                (key, val) for key, val in data.items()
                if isinstance(val, dict) and not key.startswith('_')
            ]
            # 첫 번째 ROI = primary (검출/필터링용), 나머지 = secondary
            if self._all_rois:
                self._roi_cfg = self._all_rois[0][1]
                self._roi2_cfg = self._all_rois[1][1] if len(self._all_rois) > 1 else None
            else:
                self._roi_cfg = {}
                self._roi2_cfg = None
            # 로드된 ROI 목록 로그
            for name, cfg in self._all_rois:
                mode = cfg.get('mode', 'symmetric')
                rects_n = 1 if mode == 'single' else 2
                self.log_message.emit(
                    f"[ROI] {name}: mode={mode}({rects_n}rects) "
                    f"w={cfg.get('width_px', 0)} h={cfg.get('height_px', 0)} "
                    f"color={cfg.get('color', [])}"
                )
        except Exception as e:
            self._roi_cfg = {}
            self._roi2_cfg = None
            self._all_rois = []
            self.log_message.emit(f"[ROI] config 로드 실패: {e}")

    def _draw_calib_roi_overlay(self, frame):
        """이미지 중심 기준 직사각형 ROI 오버레이 — config 키 이름으로 표시"""
        if not self._calib_roi_active:
            return
        h, w = frame.shape[:2]
        for roi_idx, (roi_name, roi_cfg) in enumerate(self._all_rois):
            rects = compute_roi_rects(w, h, roi_cfg)
            color = tuple(roi_cfg.get('color', (0, 0, 255)))
            thickness = roi_cfg.get('thickness', 2)
            alpha = 0.30 if roi_idx == 0 else 0.15
            side_labels = ['(L)', '(R)'] if len(rects) >= 2 else ['']
            for i, (x0, y0, x1, y1) in enumerate(rects):
                side = side_labels[i] if i < len(side_labels) else ''
                cx = (x0 + x1) // 2
                cy = (y0 + y1) // 2
                lbl = f"{roi_name}{side}  cx={cx} cy={cy}  {x1-x0}x{y1-y0}px"
                draw_roi_box(frame, x0, y0, x1, y1, color, thickness,
                             alpha=alpha, min_thickness=8 if roi_idx == 0 else 0,
                             label=lbl)

    def _filter_by_calib_roi(self, cols, centers_y):
        """캘리브 ROI 활성 시 범위 밖 점 제거. 비활성이면 전부 통과."""
        if not self._calib_roi_active or len(cols) == 0:
            return cols, centers_y
        w, h = self._frame_cx * 2, self._frame_cy * 2
        rects = compute_roi_rects(w, h, self._roi_cfg)
        keep = np.zeros(len(cols), dtype=bool)
        for x0, y0, x1, y1 in rects:
            keep |= (cols >= x0) & (cols <= x1) & (centers_y >= y0) & (centers_y <= y1)
        return cols[keep], centers_y[keep]

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

    def _on_toggle_horiz_laser(self, checked):
        """수평레이저 ON/OFF 토글 (하드웨어 미연결 — 버튼 텍스트만 변경)"""
        self.btnHorizLaserOn.setText("수평레이저 OFF" if checked else "수평레이저 ON")
        self._log(f"수평레이저 {'ON' if checked else 'OFF'}")

    def _on_toggle_vert_laser(self, checked):
        """수직레이저 ON/OFF 토글 (하드웨어 미연결 — 버튼 텍스트만 변경)"""
        self.btnVertLaserOn.setText("수직레이저 OFF" if checked else "수직레이저 ON")
        self._log(f"수직레이저 {'ON' if checked else 'OFF'}")

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
        self._calib_roi_active = False
        self.btnCreateROIVertical.setChecked(False)
        self.btnCreateROIHorizontal.setChecked(False)
        self._jig_detect_active = False
        self.btnLaserDetect.setChecked(False)
        self.labelJigDetectResult.setText("-")
        self._on_clear_vert_scan()

    def set_robot(self, robot):
        self.robot = robot

    def _on_set_detection_pose(self):
        """set_rz.py 방식: TF3 전환 → 현재 XYZ + 목표 RxRyRz movel → TF5 복귀"""
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

            # config에서 목표 RxRyRz 읽기 (매번 핫 리로드)
            tgt_rx, tgt_ry, tgt_rz = 90.0, 0.0, 90.0  # 기본값
            try:
                with open(LASER_DETECTION_POSE_FILE, 'r', encoding='utf-8') as f:
                    dp_cfg = json.load(f)
                tgt_rx = dp_cfg.get('target_rx', 90.0)
                tgt_ry = dp_cfg.get('target_ry', 0.0)
                tgt_rz = dp_cfg.get('target_rz', 90.0)
            except Exception as e:
                self._log(f"Detection Pose config 로드 실패 (기본값 사용): {e}")

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

            success, msg = self.robot.send_set_toolframe(5, wait=True)
            if success:
                self._log("TF5 복귀 완료")
            else:
                self._log(f"TF5 복귀 실패: {msg}")

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
            cv2.rectangle(frame, (roi_x0, roi_y0), (roi_x1, roi_y1), (0, 0, 255), 2)

    def update_frame(self, frame):
        """카메라 프레임 업데이트 — undistort 적용 후 레이저 오버레이"""
        if frame is None:
            return

        self.current_frame = frame.copy()

        # Undistorted 선택 시 보정 적용
        if self.radioUndistorted.isChecked() and self.camera_matrix is not None:
            base_frame = undistort_frame(frame, self.camera_matrix, self.dist_coeffs)
        else:
            base_frame = frame.copy()

        # 표시용 프레임 저장 (이미지 저장 시 사용)
        self.display_frame = base_frame.copy()

        # 프레임 수평 중심 갱신
        self._frame_cx = base_frame.shape[1] // 2
        self._frame_cy = base_frame.shape[0] // 2

        # ROI 마스크 갱신 (매 프레임 ArUco 위치 반영)
        self.compute_roi_mask(base_frame.shape)

        display = base_frame.copy()

        # ROI 영역 시각화 (빨간 박스)
        if self._roi_mask is not None and self._marker_list:
            self._draw_roi_boxes(display)

        # 캘리브레이션 지그 Laser 검출 오버레이
        self._draw_jig_laser_detect(display)

        _draw = {
            'rgb':   self._draw_rgb_overlay,
            'lines': self._draw_line_overlay,
            'conv':  self._draw_conv_overlay,
            'laser': self._draw_laser_overlay,
        }
        if self.display_type in _draw:
            display = _draw[self.display_type](display)

        # 캘리브레이션 지그 ROI 오버레이 (display-type 처리 이후 — 덮어씌워지지 않도록)
        self._draw_calib_roi_overlay(display)

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
        """RGB 채널 차이 기반 레이저 마스크를 표시 (calib ROI 범위 내에서만).
        검출은 오버레이 없는 self.display_frame으로 수행 — overlay 간섭 방지.
        """
        # 검출: 오버레이 없는 원본 프레임 사용
        src = self.display_frame if self.display_frame is not None else frame
        mask = LaserDetectionService.extract_red_mask(src)
        # calib ROI 활성 시 해당 범위 밖 마스크 제거
        if self._calib_roi_active:
            h, w = src.shape[:2]
            rects = compute_roi_rects(w, h, self._roi_cfg)
            roi_mask = np.zeros_like(mask)
            for x0, y0, x1, y1 in rects:
                roi_mask[y0:y1, x0:x1] = mask[y0:y1, x0:x1]
            mask = roi_mask
        # 시각화: display 프레임(frame)에 노란색 오버레이
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

        # 검출: 오버레이 없는 클린 프레임 사용
        src = self.display_frame if self.display_frame is not None else frame
        laser_result = LaserDetectionService.detect_laser_center(
            src, min_stripe_width=min_w, max_stripe_width=max_w,
        )

        if laser_result is None:
            self._clear_result_labels()
            return frame

        cols = laser_result['cols']
        centers_y = laser_result['centers_y']
        mask = laser_result['mask']
        est_width = laser_result['est_width']

        # ROI 필터 적용
        cols, centers_y = self._filter_by_calib_roi(cols, centers_y)
        if len(cols) == 0:
            self._clear_result_labels()
            return frame

        fit_result = LaserDetectionService.fit_laser_centers(
            cols, centers_y, mad_scale=mad_scale,
        )
        if fit_result is None:
            self._clear_result_labels()
            return frame
        coeffs = fit_result['coeffs']
        inlier_cols = fit_result['inlier_cols']
        inlier_y = fit_result['inlier_y']

        PINK = (180, 105, 255)

        # inlier 포인트를 핑크 점으로 표시
        for cx, cy in zip(inlier_cols, inlier_y):
            cv2.circle(frame, (int(cx), int(round(cy))), 2, PINK, -1)

        # 피팅 직선 표시
        draw_laser_fit_line(frame, coeffs, inlier_cols, PINK)

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

        conv_result = LaserDetectionService.detect_laser_center_conv(
            frame,
            min_half_width=min_hw,
            max_half_width=max_hw,
        )

        if conv_result is None:
            self._clear_result_labels()
            return frame

        cols = conv_result['cols']
        centers_y = conv_result['centers_y']
        est_width = conv_result['est_width']

        # ROI 필터링
        cols, centers_y = self._filter_by_roi(cols, centers_y)
        cols, centers_y = self._filter_by_calib_roi(cols, centers_y)

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

        line_result = LaserDetectionService.detect_laser_center_conv(
            frame, min_half_width=min_hw, max_half_width=max_hw,
        )

        if line_result is None:
            self._clear_result_labels()
            return frame

        cols = line_result['cols']
        centers_y = line_result['centers_y']
        est_width = line_result['est_width']

        # ROI 필터링
        cols, centers_y = self._filter_by_roi(cols, centers_y)
        cols, centers_y = self._filter_by_calib_roi(cols, centers_y)

        if len(cols) == 0:
            self._clear_result_labels()
            return frame

        # 중심점을 파란 점으로 표시
        BLUE = (255, 100, 0)
        for cx, cy in zip(cols, centers_y):
            cv2.circle(frame, (int(cx), int(round(cy))), 1, BLUE, -1)

        # 다중 직선 피팅 (필터 최소화 — 50px 이상 모두 표시)
        threshold = self.spinMadScale.value()
        lines = LaserDetectionService.fit_multiple_lines(
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
        table.setHorizontalHeaderLabels(['#', 'Y1(mm)', 'Z1(mm)', 'Y2(mm)', 'Z2(mm)', 'ΔY(mm)', 'ΔZ(mm)'])
        table.setColumnWidth(0, 30)
        for col in range(1, 7):
            table.setColumnWidth(col, 65)
        table.setRowCount(0)
        # 프로그레스바 초기화
        self.progressCalib.setMaximum(self.spinRepeatCount.value())
        self.progressCalib.setValue(0)

    def _on_create_roi_vertical(self):
        """Laser Vertical ROI config 재로드 + 활성화."""
        self._load_calib_roi_config(LASER_VERT_SCAN_ROI_FILE)
        self._calib_roi_active = True
        self.btnCreateROIVertical.setChecked(True)
        self.btnCreateROIHorizontal.setChecked(False)
        cx_parts = []
        if 'center_x_px' in self._roi_cfg:
            cx_parts.append(f"cx(abs)={self._roi_cfg['center_x_px']}")
        if 'center_x_offset_px' in self._roi_cfg:
            cx_parts.append(f"cx(offset)={self._roi_cfg['center_x_offset_px']}")
        cy_parts = []
        if 'center_y_px' in self._roi_cfg:
            cy_parts.append(f"cy(abs)={self._roi_cfg['center_y_px']}")
        if 'center_y_offset_px' in self._roi_cfg:
            cy_parts.append(f"cy(offset)={self._roi_cfg['center_y_offset_px']}")
        self._log(
            f"Laser Vertical ROI 갱신: {' '.join(cx_parts)}  {' '.join(cy_parts)}"
            f"  {self._roi_cfg.get('width_px', 0)}x{self._roi_cfg.get('height_px', 0)}px"
        )

    def _on_create_roi_horizontal(self):
        """Laser Horizontal ROI config 재로드 + 활성화."""
        self._load_calib_roi_config(LASER_HORIZ_SCAN_ROI_FILE)
        self._calib_roi_active = True
        self.btnCreateROIHorizontal.setChecked(True)
        self.btnCreateROIVertical.setChecked(False)
        cx_parts = []
        if 'center_x_px' in self._roi_cfg:
            cx_parts.append(f"cx(abs)={self._roi_cfg['center_x_px']}")
        if 'center_x_offset_px' in self._roi_cfg:
            cx_parts.append(f"cx(offset)={self._roi_cfg['center_x_offset_px']}")
        cy_parts = []
        if 'center_y_px' in self._roi_cfg:
            cy_parts.append(f"cy(abs)={self._roi_cfg['center_y_px']}")
        if 'center_y_offset_px' in self._roi_cfg:
            cy_parts.append(f"cy(offset)={self._roi_cfg['center_y_offset_px']}")
        self._log(
            f"Laser Horizontal ROI 갱신: {' '.join(cx_parts)}  {' '.join(cy_parts)}"
            f"  {self._roi_cfg.get('width_px', 0)}x{self._roi_cfg.get('height_px', 0)}px"
        )

    def _on_toggle_jig_detect(self):
        """Laser 검출 토글"""
        self._jig_detect_active = not self._jig_detect_active
        self.btnLaserDetect.setChecked(self._jig_detect_active)
        if self._jig_detect_active:
            self._log("Laser 검출 활성")
        else:
            self._log("Laser 검출 비활성")
            self.labelJigDetectResult.setText("-")

    def _draw_jig_laser_detect(self, frame):
        """캘리브레이션 지그 Laser 검출 오버레이 (단일 직선 피팅)"""
        if not self._jig_detect_active:
            return
        # conv/lines 모드 활성 시 이중 convolution 방지
        if self.display_type in ('conv', 'lines'):
            return

        min_hw = max(1, int(self.spinMinStripe.value()) // 2)
        max_hw = max(min_hw + 1, int(self.spinMaxStripe.value()) // 2)

        # 검출: 오버레이 없는 클린 프레임 사용
        src = self.display_frame if self.display_frame is not None else frame
        conv_result = LaserDetectionService.detect_laser_center_conv(
            src, min_half_width=min_hw, max_half_width=max_hw,
        )
        if conv_result is None:
            self.labelJigDetectResult.setText("검출 없음")
            return

        cols, centers_y = self._filter_by_calib_roi(
            conv_result['cols'], conv_result['centers_y']
        )
        if len(cols) == 0:
            self.labelJigDetectResult.setText("ROI 내 검출 없음")
            return

        fit_result = LaserDetectionService.fit_laser_centers(
            cols, centers_y, mad_scale=self.spinMadScale.value()
        )
        if fit_result is None:
            self.labelJigDetectResult.setText("직선 피팅 실패")
            return

        coeffs = fit_result['coeffs']
        inlier_cols = fit_result['inlier_cols']
        inlier_y = fit_result['inlier_y']

        CYAN = (255, 255, 0)

        # inlier 포인트 표시
        for cx, cy in zip(inlier_cols, inlier_y):
            cv2.circle(frame, (int(cx), int(round(cy))), 2, CYAN, -1)

        # 피팅 직선 표시
        draw_laser_fit_line(frame, coeffs, inlier_cols, CYAN)

        # 결과 라벨 업데이트
        angle_deg = np.degrees(np.arctan(coeffs[0])) if len(coeffs) >= 2 else None
        if angle_deg is not None and len(inlier_y) > 0:
            y_center = float(np.polyval(coeffs, self._frame_cx))
            self.labelJigDetectResult.setText(
                f"각도: {angle_deg:.2f}°  Y: {y_center:.1f}px  점: {len(inlier_cols)}"
            )

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

    def get_current_laser_y(self, frame) -> float | None:
        """프레임에서 레이저 라인 Y 위치 측정 (main_window에서 호출)"""
        if frame is None:
            return None
        min_hw = max(1, int(self.spinMinStripe.value()) // 2)
        max_hw = max(min_hw + 1, int(self.spinMaxStripe.value()) // 2)
        return LaserDetectionService.get_laser_y_at_center(
            frame, min_half_width=min_hw, max_half_width=max_hw)

    def _add_calib_row(self, y1, z1, y2, z2):
        """캘리브레이션 데이터 행 추가"""
        dy = y2 - y1
        dz = z2 - z1
        self._calib_data.append({
            'y1': y1, 'z1': z1, 'y2': y2, 'z2': z2, 'dy': dy, 'dz': dz
        })
        row = self.tableCalibData.rowCount()
        self.tableCalibData.insertRow(row)
        from PyQt5.QtWidgets import QTableWidgetItem
        values = [str(row + 1), f"{y1:.2f}", f"{z1:.2f}", f"{y2:.2f}", f"{z2:.2f}", f"{dy:.2f}", f"{dz:.2f}"]
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

    # ==================== 지그 초기 위치 ====================

    def _load_jig_positions(self):
        """지그 초기 위치 파일 로드"""
        try:
            with open(JIG_POSITIONS_FILE, 'r') as f:
                data = json.load(f)
            if data.get('vertical'):
                self._jig_pos_vertical = data['vertical']
                self._update_jig_pos_label('vertical', data['vertical'])
            if data.get('horizontal'):
                self._jig_pos_horizontal = data['horizontal']
                self._update_jig_pos_label('horizontal', data['horizontal'])
        except Exception:
            pass

    def _save_jig_positions(self):
        """지그 초기 위치 파일 저장"""
        data = {}
        if self._jig_pos_vertical:
            data['vertical'] = self._jig_pos_vertical
        if self._jig_pos_horizontal:
            data['horizontal'] = self._jig_pos_horizontal
        try:
            with open(JIG_POSITIONS_FILE, 'w') as f:
                json.dump(data, f, indent=2, ensure_ascii=False)
        except Exception as e:
            self._log(f"위치 파일 저장 실패: {e}")

    def _update_jig_pos_label(self, orientation, pose):
        """위치 라벨 업데이트"""
        label = self.labelJigPosVertical if orientation == 'vertical' else self.labelJigPosHorizontal
        label.setText(
            f"X={pose['x']:.1f} Y={pose['y']:.1f} Z={pose['z']:.1f} "
            f"Rx={pose['rx']:.1f} Ry={pose['ry']:.1f} Rz={pose['rz']:.1f}"
        )

    def _on_save_jig_pos(self, orientation):
        """현재 위치를 수직/수평 초기 위치로 저장 (TF5 기준)"""
        if self.robot is None:
            from PyQt5.QtWidgets import QMessageBox
            QMessageBox.warning(self, "경고", "로봇이 연결되지 않았습니다.")
            return

        try:
            pose = self.robot.read_current_pose()
            if pose is None:
                self._log("TCP 좌표 읽기 실패")
                return

            pos_dict = {
                'x': float(pose[0]), 'y': float(pose[1]), 'z': float(pose[2]),
                'rx': float(pose[3]), 'ry': float(pose[4]), 'rz': float(pose[5]),
            }
            if orientation == 'vertical':
                self._jig_pos_vertical = pos_dict
            else:
                self._jig_pos_horizontal = pos_dict

            self._update_jig_pos_label(orientation, pos_dict)
            self._save_jig_positions()
            label = '수직' if orientation == 'vertical' else '수평'
            self._log(f"지그 {label} 위치 저장 (TF5): X={pos_dict['x']:.1f} Y={pos_dict['y']:.1f} "
                      f"Z={pos_dict['z']:.1f} Rx={pos_dict['rx']:.1f} Ry={pos_dict['ry']:.1f} Rz={pos_dict['rz']:.1f}")
        except Exception as e:
            self._log(f"위치 저장 오류: {e}")

    def _on_save_jig_pos_vertical(self):
        self._on_save_jig_pos('vertical')

    def _on_save_jig_pos_horizontal(self):
        self._on_save_jig_pos('horizontal')

    def _on_move_jig_pos(self, orientation):
        """수직/수평 초기 위치로 이동 (TF5 유지)"""
        pos = self._jig_pos_vertical if orientation == 'vertical' else self._jig_pos_horizontal
        label = '수직' if orientation == 'vertical' else '수평'
        if pos is None:
            self._log(f"저장된 {label} 위치 없음 — 먼저 저장하세요")
            return
        if self.robot is None:
            from PyQt5.QtWidgets import QMessageBox
            QMessageBox.warning(self, "경고", "로봇이 연결되지 않았습니다.")
            return

        from PyQt5.QtWidgets import QApplication
        try:
            to_int16 = self.robot.to_uint16
            regs = [
                to_int16(int(round(pos['x']  * 10))),
                to_int16(int(round(pos['y']  * 10))),
                to_int16(int(round(pos['z']  * 10))),
                to_int16(int(round(pos['rx'] * 10))),
                to_int16(int(round(pos['ry'] * 10))),
                to_int16(int(round(pos['rz'] * 10))),
            ]
            self.robot.write_registers(self.robot.REGISTER_POSE_MAIN, regs)
            self.robot.write_command(self.robot.CMD_MOVE_TO_POSE)

            success, msg = self.robot.wait_for_done_motion_aware(
                process_events_callback=QApplication.processEvents
            )
            if not success:
                self._log(f"지그 {label} 위치 이동 실패: {msg}")
                return
            self._log(f"지그 {label} 위치 이동 완료 (TF5)")
        except Exception as e:
            self._log(f"이동 오류: {e}")

    def _on_move_jig_pos_vertical(self):
        self._on_move_jig_pos('vertical')

    def _on_move_jig_pos_horizontal(self):
        self._on_move_jig_pos('horizontal')

    # ==================== 수직 캘리브레이션 (5단 지그) ====================

    def _init_vert_scan_ui(self):
        """수직 스캔 UI 초기화"""
        self._step_labels = [
            self.labelStep1, self.labelStep2, self.labelStep3,
            self.labelStep4, self.labelStep5,
        ]
        self._step_res_labels = [
            self.labelStepRes1, self.labelStepRes2, self.labelStepRes3,
            self.labelStepRes4, self.labelStepRes5,
        ]
        try:
            with open(LASER_JIG_STEP_CONFIG_FILE, 'r') as f:
                cfg = json.load(f)
            self.spinStepInterval.setValue(cfg.get('step_interval_mm', 2.0))
        except Exception:
            pass
        try:
            with open(os.path.join(CONFIG_DIR, 'laser', 'vertical', 'laser_jig_auto_scan.json'), 'r') as f:
                scan_cfg = json.load(f)
            self.spinScanRange.setValue(scan_cfg.get('scan_range_mm', 70.0))
        except Exception:
            pass
        self._reset_step_indicator()

    def _reset_step_indicator(self):
        """스텝 인디케이터 초기화 (회색)"""
        _idle = (
            "border: 1px solid #888; border-radius: 4px; padding: 2px;"
            "background: #555; color: #ccc; font-size: 11px;"
        )
        for i, lbl in enumerate(self._step_labels):
            lbl.setText(f"Step {i + 1}")
            lbl.setStyleSheet(_idle)
        for i, lbl in enumerate(self._step_res_labels):
            lbl.setText(f"Step {i + 1}: -")
            lbl.setStyleSheet("font-size: 11px; color: #aaa; padding: 1px 4px;")

    def _set_step_state(self, idx, state, y_px=None):
        """스텝 인디케이터 상태 업데이트.

        state: 'idle' | 'detected' | 'error'
        """
        if idx < 0 or idx >= len(self._step_labels):
            return
        lbl = self._step_labels[idx]
        res_lbl = self._step_res_labels[idx]

        if state == 'detected':
            lbl.setStyleSheet(
                "border: 1px solid #2E7D32; border-radius: 4px; padding: 2px;"
                "background: #1B5E20; color: #A5D6A7; font-size: 11px; font-weight: bold;"
            )
            lbl.setText(f"Step {idx + 1}\n✓")
            if y_px is not None:
                z_rel = idx * self.spinStepInterval.value()
                res_lbl.setText(f"Step {idx + 1}: Z+{z_rel:.0f}mm  Y={y_px:.1f}px")
                res_lbl.setStyleSheet("font-size: 11px; color: #A5D6A7; padding: 1px 4px;")
        elif state == 'error':
            lbl.setStyleSheet(
                "border: 1px solid #C62828; border-radius: 4px; padding: 2px;"
                "background: #B71C1C; color: #FFCDD2; font-size: 11px;"
            )
            lbl.setText(f"Step {idx + 1}\n✗")
            res_lbl.setText(f"Step {idx + 1}: 검출 실패")
            res_lbl.setStyleSheet("font-size: 11px; color: #FFCDD2; padding: 1px 4px;")
        else:
            lbl.setStyleSheet(
                "border: 1px solid #888; border-radius: 4px; padding: 2px;"
                "background: #555; color: #ccc; font-size: 11px;"
            )
            lbl.setText(f"Step {idx + 1}")
            res_lbl.setText(f"Step {idx + 1}: -")
            res_lbl.setStyleSheet("font-size: 11px; color: #aaa; padding: 1px 4px;")

    def _detect_roi_lines(self, frame, rect, max_lines=5):
        """단일 ROI rect 에서 레이저 직선을 추출한다.

        Args:
            frame: 전체 프레임 (undistort 적용 후)
            rect: (x0, y0, x1, y1) ROI 좌표
            max_lines: 최대 직선 수

        Returns:
            (lines, raw_pixels): lines=list of line dicts, raw_pixels=dict{'cols','centers_y'}
            실패 시 ([], None)
        """
        x0, y0, x1, y1 = rect
        roi_frame = frame[y0:y1, x0:x1]
        conv_result = LaserDetectionService.detect_laser_center_conv(roi_frame)
        if conv_result is None:
            return [], None
        cols = conv_result['cols'] + x0
        centers_y = conv_result['centers_y'] + y0
        raw_pixels = {'cols': cols.copy(), 'centers_y': centers_y.copy()}
        lines = LaserDetectionService.fit_multiple_lines(
            cols, centers_y,
            residual_threshold=3.0,
            min_inliers=30,
            max_lines=max_lines,
        )
        return (lines if lines else []), raw_pixels

    def _run_vert_scan(self):
        """현재 프레임에서 5단 지그 수직 스캔 — ROI별 step 데이터 반환.

        Returns:
            dict {'center': list[{'step','y_px'}],
                  'left':   list[{'step','y_px'}],
                  'right':  list[{'step','y_px'}]}
            실패 시 {} (빈 dict)
        """
        if self.current_frame is None:
            self._log("수직 스캔: 카메라 프레임 없음")
            self.labelVertScanStatus.setText("카메라 없음")
            return {}

        if self.radioUndistorted.isChecked() and self.camera_matrix is not None:
            frame = undistort_frame(self.current_frame, self.camera_matrix, self.dist_coeffs)
        else:
            frame = self.current_frame.copy()

        h_f, w_f = frame.shape[:2]
        center_x = w_f // 2

        # -- ROI rect 목록 구성 (config 키 이름 사용) --
        roi_entries = []  # list of (label, rect)
        if self._calib_roi_active and self._all_rois:
            side_suffixes = {1: [''], 2: ['(L)', '(R)']}
            for roi_name, roi_cfg in self._all_rois:
                rects = compute_roi_rects(w_f, h_f, roi_cfg)
                suffixes = side_suffixes.get(len(rects), [f'({i})' for i in range(len(rects))])
                for i, rect in enumerate(rects):
                    label = f"{roi_name}{suffixes[i] if i < len(suffixes) else ''}"
                    roi_entries.append((label, rect))

        # ROI 미설정 시 전체 프레임 사용 (기존 fallback)
        if not roi_entries:
            roi_entries.append(('full_frame', (0, 0, w_f, h_f)))

        # -- 각 ROI별 독립 레이저 직선 추출 --
        result = {}
        self._last_scan_raw_pixels = {}  # ROI별 원본 레이저 픽셀 데이터
        for label, rect in roi_entries:
            lines, raw_pixels = self._detect_roi_lines(frame, rect, max_lines=5)
            if raw_pixels is not None:
                # R ROI edge 오염 필터링 (col > 1300 제거)
                col_limit = rect[2]  # ROI 오른쪽 경계
                if 'base' in label and col_limit > 1300:
                    cols = raw_pixels['cols']
                    mask = cols <= 1300
                    raw_pixels = {
                        'cols': cols[mask],
                        'centers_y': raw_pixels['centers_y'][mask],
                    }
                self._last_scan_raw_pixels[label] = raw_pixels
            if lines:
                lines_sorted = sorted(
                    lines,
                    key=lambda l: float(np.polyval(l['coeffs'], center_x)),
                    reverse=True,
                )
                step_data = []
                for i, line in enumerate(lines_sorted[:5]):
                    y_px = float(np.polyval(line['coeffs'], center_x))
                    step_data.append({'step': i, 'y_px': y_px})
                result[label] = step_data
            else:
                result[label] = []
            self._log(f"수직 스캔 {label} ROI: {len(result[label])}개 직선 검출")

        # -- 첫 번째 ROI 결과로 UI 업데이트 --
        primary_name = self._all_rois[0][0] if self._all_rois else 'full_frame'
        center_data = result.get(primary_name, [])
        self._vert_scan_data = list(center_data)
        self._reset_step_indicator()

        detected = len(center_data)
        for i in range(detected):
            self._set_step_state(i, 'detected', center_data[i]['y_px'])
        for i in range(detected, 5):
            self._set_step_state(i, 'error')

        # base ROI 데이터 저장 (single distance fallback용)
        base_data = []
        for key, steps in result.items():
            if key.startswith('roi_laser_base') and steps:
                for s in steps:
                    base_data.append({'y_px': s['y_px']})
        self._vert_scan_base_data = base_data

        self.labelVertScanStatus.setText(f"{detected}/5 스텝 검출")
        return result

    def _on_clear_vert_scan(self):
        """수직 스캔 데이터 초기화"""
        self._vert_scan_data = []
        self._multi_vert_captures = []
        self._vert_scan_base_data = []
        self._reset_step_indicator()
        if hasattr(self, 'labelCaptureCount'):
            self.labelCaptureCount.setText("캡처: 0회")
        if hasattr(self, 'labelVertScanStatus'):
            self.labelVertScanStatus.setText("-")
        if hasattr(self, 'labelVertFitResult'):
            self.labelVertFitResult.setText("-")
        if hasattr(self, 'btnFitVertModel'):
            self.btnFitVertModel.setEnabled(False)

    def _on_fit_vert_model(self):
        """다중 거리 캡처 데이터로 h, alpha, delta_z 피팅 & 저장.

        2회 이상 캡처 시 다중거리 피팅 (권장),
        1회 이하 시 단일거리 fallback.
        """
        valid_caps = [c for c in self._multi_vert_captures if len(c['steps']) >= 1]
        if len(valid_caps) >= 2:
            self._fit_multi_distance(valid_caps)
        elif len(self._vert_scan_data) >= 3:
            self._log("캡처 1회 — 단일거리 fallback으로 피팅 (정확도 낮음)")
            self._fit_single_distance()
        else:
            self._log("모델 피팅: 데이터 부족 (캡처 2회 이상 권장)")

    def _fit_multi_distance(self, valid_caps):
        """다중 거리 캡처: h_base, alpha, delta_z 동시 피팅.

        모델: y = cy + fy * tan(arctan(H_abs / Z_abs) - alpha)
        Z_abs = z_tcp + delta_z + step * tread_mm   (depth 증분)
        H_abs = h_base + step * rise_mm              (height 증분)
        미지수: h_base(mm), alpha(deg), delta_z(mm)
        """
        try:
            from scipy.optimize import minimize as sp_minimize

            if self.camera_matrix is not None:
                fy = float(self.camera_matrix[1, 1])
                cy_cam = float(self.camera_matrix[1, 2])
            else:
                fy = 5347.3
                cy_cam = 478.2

            step_tread = self.spinStepInterval.value()
            try:
                with open(LASER_JIG_STEP_CONFIG_FILE, 'r') as f:
                    _jig = json.load(f)
                step_rise = float(_jig.get('step_rise_mm', step_tread))
            except Exception:
                step_rise = step_tread

            # 데이터 수집: (z_tcp, z_rel, h_rel, y_px) + base ROI
            z_tcps, z_rels, h_rels, y_pixs, is_base = [], [], [], [], []
            for cap in valid_caps:
                for d in cap['steps']:
                    z_tcps.append(cap['z_tcp'])
                    z_rels.append(d['step'] * step_tread)
                    h_rels.append(d['step'] * step_rise)
                    y_pixs.append(d['y_px'])
                    is_base.append(False)
                # base ROI 데이터 (h_rel=0, z_rel=0)
                for key in cap:
                    if key.startswith('roi_laser_base') and len(cap[key]) > 0:
                        for d in cap[key]:
                            z_tcps.append(cap['z_tcp'])
                            z_rels.append(0.0)
                            h_rels.append(0.0)
                            y_pixs.append(d['y_px'])
                            is_base.append(True)
            z_tcps = np.array(z_tcps)
            z_rels = np.array(z_rels)
            h_rels = np.array(h_rels)
            y_pixs = np.array(y_pixs)
            is_base = np.array(is_base)

            # 초기값 (기존 파일 우선)
            h0, alpha0, delta_z0 = 70.0, 13.0, -200.0
            existing = {}
            try:
                with open(LASER_VERT_CALIB_FILE, 'r') as f:
                    existing = json.load(f)
                h0 = existing.get('h_mm', h0)
                alpha0 = existing.get('alpha_deg', alpha0)
                delta_z0 = existing.get('delta_z_mm', delta_z0)
            except Exception:
                pass

            def cost(params):
                h_base, alpha_deg, delta_z = params
                if h_base <= 0:
                    return 1e9
                Z_abs = z_tcps + delta_z + z_rels
                H_abs = h_base + h_rels
                if np.any(Z_abs <= 0):
                    return 1e9
                try:
                    y_pred = cy_cam + fy * np.tan(
                        np.arctan(H_abs / Z_abs) - np.radians(alpha_deg)
                    )
                    return float(np.sum((y_pred - y_pixs) ** 2))
                except Exception:
                    return 1e9

            result = sp_minimize(
                cost, x0=[h0, alpha0, delta_z0],
                method='L-BFGS-B',
                bounds=[(1, 500), (0, 30), (-2000, 2000)],
                options={'ftol': 0.01, 'maxiter': 20000},
            )

            h_fit, alpha_fit, delta_z_fit = result.x
            Z_abs_fit = z_tcps + delta_z_fit + z_rels
            H_abs_fit = h_fit + h_rels
            y_pred = cy_cam + fy * np.tan(
                np.arctan(H_abs_fit / Z_abs_fit) - np.radians(alpha_fit)
            )
            rmse = float(np.sqrt(np.mean((y_pixs - y_pred) ** 2)))
            step_mask = ~is_base
            base_mask = is_base
            step_rmse = float(np.sqrt(np.mean((y_pixs[step_mask] - y_pred[step_mask]) ** 2))) if np.any(step_mask) else 0.0
            base_rmse = float(np.sqrt(np.mean((y_pixs[base_mask] - y_pred[base_mask]) ** 2))) if np.any(base_mask) else 0.0
            n_base = int(np.sum(base_mask))
            self._log(f"피팅: base={n_base}pts step={int(np.sum(step_mask))}pts | RMSE total={rmse:.2f} step={step_rmse:.2f} base={base_rmse:.2f}")

            calib_out = {
                'h_mm': float(h_fit),
                'Bx_mm': float(existing.get('Bx_mm', -30.0)),
                'alpha_deg': float(alpha_fit),
                'delta_z_mm': float(delta_z_fit),
                'rmse_px': float(rmse),
                'step_rmse_px': float(step_rmse),
                'base_rmse_px': float(base_rmse),
                'num_base_points': n_base,
                'num_captures': len(valid_caps),
                'num_points': len(y_pixs),
                'step_tread_mm': float(step_tread),
                'step_rise_mm': float(step_rise),
                'step_interval_mm': float(step_tread),
                'calibration_date': datetime.now().strftime('%Y-%m-%d'),
                'note': (
                    f'Multi-distance vertical calibration. '
                    f'fy={fy:.1f}, cy={cy_cam:.1f}. '
                    f'{len(valid_caps)} captures, {len(y_pixs)} points. L-BFGS-B.'
                ),
            }

            os.makedirs(os.path.dirname(LASER_VERT_CALIB_FILE), exist_ok=True)
            with open(LASER_VERT_CALIB_FILE, 'w') as f:
                json.dump(calib_out, f, indent=4, ensure_ascii=False)
            _ts = datetime.now().strftime('%Y%m%d_%H%M%S')
            _hist = os.path.join(LASER_VERT_CALIB_DATA_DIR, f'calib_vert_{_ts}.json')
            os.makedirs(LASER_VERT_CALIB_DATA_DIR, exist_ok=True)
            with open(_hist, 'w') as f:
                json.dump(calib_out, f, indent=4, ensure_ascii=False)

            result_text = (
                f"h={h_fit:.1f}mm  α={alpha_fit:.3f}°  "
                f"δZ={delta_z_fit:.1f}mm  RMSE={rmse:.2f}px  "
                f"({len(valid_caps)}캡처/{len(y_pixs)}pts)"
            )
            self.labelVertFitResult.setText(result_text)
            self._log(f"다중거리 수직 캘리브 저장 완료: {result_text}")

        except Exception as e:
            self._log(f"모델 피팅 오류: {e}")
            self.labelVertFitResult.setText(f"오류: {e}")

    def _fit_single_distance(self):
        """단일 거리 캡처: h_base, alpha, z_ref 피팅 (레거시 fallback)."""
        try:
            from scipy.optimize import minimize as sp_minimize

            if self.camera_matrix is not None:
                fy = float(self.camera_matrix[1, 1])
                cy_cam = float(self.camera_matrix[1, 2])
            else:
                fy = 5347.3
                cy_cam = 478.2

            step_tread = self.spinStepInterval.value()
            try:
                with open(LASER_JIG_STEP_CONFIG_FILE, 'r') as f:
                    _jig = json.load(f)
                step_rise = float(_jig.get('step_rise_mm', step_tread))
            except Exception:
                step_rise = step_tread

            steps = [d['step'] for d in self._vert_scan_data]
            y_vals = [d['y_px'] for d in self._vert_scan_data]
            is_base_list = [False] * len(steps)
            z_rel_list = [s * step_tread for s in steps]
            h_rel_list = [s * step_rise for s in steps]
            # base ROI 데이터 (h_rel=0, z_rel=0)
            for bd in self._vert_scan_base_data:
                steps.append(0)
                y_vals.append(bd['y_px'])
                is_base_list.append(True)
                z_rel_list.append(0.0)
                h_rel_list.append(0.0)
            z_rel = np.array(z_rel_list)
            h_rel = np.array(h_rel_list)
            y_pix = np.array(y_vals)
            is_base = np.array(is_base_list)

            def model_y(Z_abs, H_abs, alpha_rad):
                return cy_cam + fy * np.tan(np.arctan(H_abs / Z_abs) - alpha_rad)

            def cost(params):
                h_base, alpha_deg, z_ref = params
                if h_base <= 0 or z_ref <= 0:
                    return 1e9
                try:
                    y_pred = model_y(z_ref + z_rel, h_base + h_rel, np.radians(alpha_deg))
                    return float(np.sum((y_pred - y_pix) ** 2))
                except Exception:
                    return 1e9

            h0, alpha0, bx0 = 70.0, 13.0, -30.0
            existing = {}
            try:
                with open(LASER_VERT_CALIB_FILE, 'r') as f:
                    existing = json.load(f)
                h0 = existing.get('h_mm', h0)
                alpha0 = existing.get('alpha_deg', alpha0)
                bx0 = existing.get('Bx_mm', bx0)
            except Exception:
                pass

            result = sp_minimize(
                cost, x0=[h0, alpha0, 300.0],
                method='L-BFGS-B',
                bounds=[(1, 500), (0, 30), (1, 2000)],
                options={'ftol': 0.01, 'maxiter': 20000},
            )

            h_fit, alpha_fit, z_ref_fit = result.x
            y_pred = model_y(z_ref_fit + z_rel, h_fit + h_rel, np.radians(alpha_fit))
            rmse = float(np.sqrt(np.mean((y_pix - y_pred) ** 2)))
            step_mask = ~is_base
            base_mask = is_base
            step_rmse = float(np.sqrt(np.mean((y_pix[step_mask] - y_pred[step_mask]) ** 2))) if np.any(step_mask) else 0.0
            base_rmse = float(np.sqrt(np.mean((y_pix[base_mask] - y_pred[base_mask]) ** 2))) if np.any(base_mask) else 0.0
            n_base = int(np.sum(base_mask))
            self._log(f"피팅: base={n_base}pts step={int(np.sum(step_mask))}pts | RMSE total={rmse:.2f} step={step_rmse:.2f} base={base_rmse:.2f}")

            calib_out = {
                'h_mm': float(h_fit),
                'Bx_mm': float(bx0),
                'alpha_deg': float(alpha_fit),
                'z_ref_mm': float(z_ref_fit),
                'rmse_px': float(rmse),
                'step_rmse_px': float(step_rmse),
                'base_rmse_px': float(base_rmse),
                'num_base_points': n_base,
                'num_steps': len(self._vert_scan_data),
                'step_tread_mm': float(step_tread),
                'step_rise_mm': float(step_rise),
                'step_interval_mm': float(step_tread),
                'calibration_date': datetime.now().strftime('%Y-%m-%d'),
                'note': (
                    f'Single-distance fallback. '
                    f'fy={fy:.1f}, cy={cy_cam:.1f}. L-BFGS-B.'
                ),
            }

            os.makedirs(os.path.dirname(LASER_VERT_CALIB_FILE), exist_ok=True)
            with open(LASER_VERT_CALIB_FILE, 'w') as f:
                json.dump(calib_out, f, indent=4, ensure_ascii=False)
            _ts = datetime.now().strftime('%Y%m%d_%H%M%S')
            _hist = os.path.join(LASER_VERT_CALIB_DATA_DIR, f'calib_vert_{_ts}.json')
            os.makedirs(LASER_VERT_CALIB_DATA_DIR, exist_ok=True)
            with open(_hist, 'w') as f:
                json.dump(calib_out, f, indent=4, ensure_ascii=False)

            result_text = (
                f"h={h_fit:.1f}mm  α={alpha_fit:.3f}°  "
                f"Z_ref={z_ref_fit:.1f}mm  RMSE={rmse:.2f}px"
            )
            self.labelVertFitResult.setText(result_text)
            self._log(f"수직 캘리브 저장 완료 (단일거리): {result_text}")

        except Exception as e:
            self._log(f"모델 피팅 오류: {e}")
            self.labelVertFitResult.setText(f"오류: {e}")

    def _on_btn_auto_vert_scan(self):
        """자동 수직 스캔 버튼 클릭 — main_window로 시그널 전달."""
        self.calib_auto_vert_scan_requested.emit()

    def _set_auto_scan_ui_state(self, running: bool):
        """자동 스캔 중 버튼 활성/비활성 전환."""
        self.btnAutoVertScan.setEnabled(not running)
        self.btnFitVertModel.setEnabled(not running)
        self.btnCancelAutoScan.setVisible(running)
        self._auto_vert_scan_running = running

    def _update_auto_scan_progress(self, text: str):
        """자동 스캔 진행 상태 레이블 업데이트."""
        if hasattr(self, 'labelAutoScanProgress'):
            self.labelAutoScanProgress.setText(text)

    def _update_robot_status_ui(self):
        """현재 로봇 TF/위치/자세 UI 업데이트 (1초 주기)"""
        if not hasattr(self, 'labelTFValue'):
            return

        if self.robot is None:
            self.labelTFValue.setText("로봇 미연결")
            self.labelPosValue.setText("-")
            self.labelOriValue.setText("-")
            return

        try:
            tf = self.robot.read_current_toolframe()
            self.labelTFValue.setText(f"TF{tf}" if tf is not None else "읽기 실패")

            pose = self.robot.read_camera_pose()
            if pose is not None:
                x, y, z, rx, ry, rz = pose
                self.labelPosValue.setText(f"X={x:.2f}, Y={y:.2f}, Z={z:.2f} mm")
                self.labelOriValue.setText(f"Rx={rx:.2f}, Ry={ry:.2f}, Rz={rz:.2f}°")
            else:
                self.labelPosValue.setText("읽기 실패")
                self.labelOriValue.setText("-")
        except Exception as e:
            self.labelTFValue.setText(f"오류: {e}")
            self.labelPosValue.setText("-")
            self.labelOriValue.setText("-")

    def _log(self, msg):
        self.log_message.emit(msg)
