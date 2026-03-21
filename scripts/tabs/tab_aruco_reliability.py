#!/usr/bin/env python3
"""
ArUco Tag 신뢰성 검증 탭
ArUco 태그의 위치/자세 검출 신뢰성을 반복 측정하여 통계적으로 분석
"""

import os
import json
import csv
import cv2
import yaml
import numpy as np
from datetime import datetime
from PyQt5 import uic
from PyQt5.QtWidgets import QWidget, QFileDialog, QMessageBox, QVBoxLayout, QButtonGroup, QTabWidget
from PyQt5.QtCore import Qt, pyqtSignal, QTimer

from .jog_mixin import JogMixin
from PyQt5.QtGui import QPixmap, QImage
from utils.image_processing import undistort_frame, rvec_to_euler_deg

# matplotlib 통합
import matplotlib
matplotlib.use('Qt5Agg')
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure
import matplotlib.pyplot as plt

# 공통 유틸리티
from utils.common import (
    display_frame_on_label,
    require_camera_running,
    Messages,
)

# 좌표 변환 유틸리티
from utils.ar_to_base_tf import camera_to_vision

# 평면 추출 유틸리티
from services.plane_extractor import PlaneExtractor
from services.dual_aruco_detector import DualArucoDetector, PlanePose
from services.tcp_corrector import TCPCorrector
from Sensor.aruco.aruco_detector import compute_dual_alignment, draw_dual_marker_overlay


# UI 파일 경로
UI_DIR = os.path.join(os.path.dirname(__file__), '..', '..', 'ui')
TAB_ARUCO_RELIABILITY_UI = os.path.join(UI_DIR, 'tab_aruco_reliability.ui')

# 캘리브레이션 파일 경로
CONFIG_DIR = os.path.join(os.path.dirname(__file__), '..', '..', 'config')
DS435_CALIB_FILE = os.path.join(CONFIG_DIR, 'calibration', 'ds435', 'ds435_calibration.yaml')
ARDUCAM_CALIB_FILE = os.path.join(CONFIG_DIR, 'calibration', 'arducam', 'arducam_calibration.yaml')


class TabArucoReliability(QWidget, JogMixin):
    """ArUco Tag 신뢰성 검증 탭 클래스"""

    # 시그널 정의
    log_message = pyqtSignal(str)
    camera_start_requested = pyqtSignal()
    camera_stop_requested = pyqtSignal()
    jog_move_requested = pyqtSignal(str, float)  # axis, distance(mm)
    jog_rotate_requested = pyqtSignal(str, float)  # axis, angle(deg)
    align_parallel_requested = pyqtSignal(float, float, float)  # dRx, dRy, dRz
    align_single_axis_requested = pyqtSignal(str, float)  # axis('rx'/'ry'/'rz'), angle(deg)
    align_base_ry_requested = pyqtSignal(float)  # angle(deg) - base 기준 Ry movel 보정
    align_base_rz_requested = pyqtSignal(float, float)  # angle(deg), distance(mm) - base 기준 Rz + Y보정
    align_base_y_requested = pyqtSignal(float)  # dY (px) - base Y 위치 보정
    align_aruco_y_requested = pyqtSignal()        # 통합 Y 정렬 (Ry + BaseY)
    align_aruco_x_requested = pyqtSignal()        # 통합 X 정렬 (Rz)
    align_aruco_combined_requested = pyqtSignal()  # 통합 정렬 (Y + X)

    def __init__(self, parent=None):
        super().__init__(parent)

        # UI 로드
        uic.loadUi(TAB_ARUCO_RELIABILITY_UI, self)

        # rightTabWidget, widgetArucoAlign, widgetArTagTcpAlign 등은 .ui에서 로드됨

        # 카메라/비전 매니저 참조
        self.camera_manager = None
        self.vision_manager = None
        self.robot = None

        # 현재 프레임
        self.current_frame = None
        self.undistorted_frame = None  # undistort 적용된 프레임

        # 캘리브레이션 데이터
        self.camera_matrix = None
        self.dist_coeffs = None
        self._current_camera_type = None  # 'ds435' or 'arducam'

        # 캡처 상태
        self.is_capturing = False
        self.capture_count = 0
        self.max_captures = 50

        # 이미지 저장 설정
        self.save_images = False
        self.save_folder = None

        # 수집된 데이터
        self.collected_data = []  # List of dicts: {timestamp, tag_id, detected, tvec, rvec, euler}

        # matplotlib 그래프 설정 (서브탭: Raw Data / Outlier 제거)
        self.graph_tab_widget = QTabWidget()

        # Raw Data 탭
        self.figure_raw = Figure(figsize=(6, 3))
        self.canvas_raw = FigureCanvas(self.figure_raw)
        raw_widget = QWidget()
        raw_layout = QVBoxLayout(raw_widget)
        raw_layout.setContentsMargins(0, 0, 0, 0)
        raw_layout.addWidget(self.canvas_raw)
        self.graph_tab_widget.addTab(raw_widget, "Raw Data")

        # Outlier 제거 탭
        self.figure_filtered = Figure(figsize=(6, 3))
        self.canvas_filtered = FigureCanvas(self.figure_filtered)
        filtered_widget = QWidget()
        filtered_layout = QVBoxLayout(filtered_widget)
        filtered_layout.setContentsMargins(0, 0, 0, 0)
        filtered_layout.addWidget(self.canvas_filtered)
        self.graph_tab_widget.addTab(filtered_widget, "Outlier 제거")

        # 그래프 캔버스를 UI에 추가
        layout = QVBoxLayout(self.widgetGraphCanvas)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.addWidget(self.graph_tab_widget)

        # 캡처 타이머
        self.capture_timer = QTimer()
        self.capture_timer.timeout.connect(self._on_capture_next)

        # 카메라 선택 버튼 그룹 (DS435/ArduCam)
        self.camera_select_button_group = QButtonGroup(self)
        self.camera_select_button_group.addButton(self.radioDS435, 0)
        self.camera_select_button_group.addButton(self.radioArduCam, 1)

        # 그래프 마커 선택 버튼 그룹 (ID0/ID1)
        self.graph_marker_button_group = QButtonGroup(self)
        self.graph_marker_button_group.addButton(self.radioGraphID0, 0)
        self.graph_marker_button_group.addButton(self.radioGraphID1, 1)
        self.graph_marker_button_group.buttonClicked.connect(self._on_graph_marker_changed)

        # Disambiguation UI elements는 .ui 파일에서 로드됨

        # 시그널 연결
        self._connect_signals()

        # Detection Pose config 로드
        self._load_detection_pose_config()

        # 초기화
        self._init_ui()

    def _connect_signals(self):
        """내부 시그널-슬롯 연결"""
        # 카메라 버튼
        self.btnStartCamera.clicked.connect(self._on_start_camera)
        self.btnStopCamera.clicked.connect(self._on_stop_camera)

        # 제어 버튼
        self.btnStartCapture.clicked.connect(self._on_start_capture)
        self.btnStopCapture.clicked.connect(self._on_stop_capture)
        self.btnReset.clicked.connect(self._on_reset)

        # 내보내기 버튼
        self.btnLoadCSV.clicked.connect(self._on_load_csv)
        self.btnExportCSV.clicked.connect(self._on_export_csv)
        self.btnExportGraph.clicked.connect(self._on_export_graph)

        # Detection Pose 버튼
        self.btnSetDetectionPose.clicked.connect(self._on_set_detection_pose)
        if hasattr(self, 'btnSetRobotPosition'):
            self.btnSetRobotPosition.clicked.connect(self._on_set_robot_position)

        # 조그 이동 (JogMixin)
        self._connect_jog_buttons()

        # aruco 통합 정렬 버튼
        self.btnAlignArucoY.clicked.connect(self._on_align_aruco_y)
        self.btnAlignArucoX.clicked.connect(self._on_align_aruco_x)
        self.btnAlignArucoCombined.clicked.connect(self._on_align_aruco_combined)

        # ar tag tcp align 탭 버튼
        self.btnAlignRx.clicked.connect(lambda: self._on_align_single_axis('rx'))
        self.btnAlignRy.clicked.connect(self._on_align_ry_from_angle)
        self.btnAlignRz.clicked.connect(lambda: self._on_align_single_axis('rz'))
        self.btnAlignParallel.clicked.connect(self._on_align_parallel)

    def _load_detection_pose_config(self):
        """config/charging/charging_gun_coupling.json에서 detection_pose 로드"""
        config_path = os.path.join(CONFIG_DIR, 'charging', 'charging_gun_coupling.json')
        try:
            with open(config_path, 'r') as f:
                config = json.load(f)
            pose = config.get('reference_positions', {}).get('detection_pose', {})
            if pose:
                self.spinDetPoseRx.setValue(pose.get('rx', 90.0))
                self.spinDetPoseRy.setValue(pose.get('ry', 0.0))
                self.spinDetPoseRz.setValue(pose.get('rz', 90.0))
                self._detection_pose_xyz = (
                    pose.get('x'), pose.get('y'), pose.get('z')
                )
                self._detection_pose_use_xyz = pose.get('use_xyz', False)
        except Exception as e:
            print(f"[TabArucoReliability] detection_pose config 로드 실패: {e}")
            self._detection_pose_xyz = (None, None, None)
            self._detection_pose_use_xyz = False

    def _init_ui(self):
        """UI 초기화"""
        self.progressBar.setValue(0)
        self.textLog.setPlainText("신뢰성 검증을 시작하려면 '캡처 시작' 버튼을 누르세요.")
        self._update_statistics_ui_m0()
        self._update_statistics_ui_m1()

    def set_camera_manager(self, camera_manager):
        """카메라 매니저 설정"""
        self.camera_manager = camera_manager

    def set_vision_manager(self, vision_manager):
        """비전 매니저 설정"""
        self.vision_manager = vision_manager

    def set_robot(self, robot):
        """로봇 클라이언트 설정"""
        self.robot = robot

    def _load_calibration(self, camera_type: str):
        """카메라 캘리브레이션 파일 로드"""
        if camera_type == self._current_camera_type and self.camera_matrix is not None:
            return  # 이미 로드됨

        calib_file = DS435_CALIB_FILE if camera_type == 'ds435' else ARDUCAM_CALIB_FILE

        try:
            with open(calib_file, 'r') as f:
                data = yaml.safe_load(f)

            # OpenCV YAML 형식 파싱 (rows, cols, data 구조)
            cam_data = data['camera_matrix']
            self.camera_matrix = np.array(cam_data['data'], dtype=np.float64).reshape(
                cam_data['rows'], cam_data['cols']
            )

            dist_data = data['distortion_coefficients']
            self.dist_coeffs = np.array(dist_data['data'], dtype=np.float64).reshape(
                dist_data['rows'], dist_data['cols']
            )

            self._current_camera_type = camera_type
            self._log(f"캘리브레이션 로드: {calib_file}")

        except Exception as e:
            self._log(f"캘리브레이션 로드 실패: {e}")
            self.camera_matrix = None
            self.dist_coeffs = None

    def update_frame(self, frame):
        """카메라 프레임 업데이트 (Dual ArUco 지원)"""
        if frame is None:
            return

        self.current_frame = frame.copy()

        # 카메라 타입에 따른 캘리브레이션 로드
        camera_type = 'arducam' if self.radioArduCam.isChecked() else 'ds435'
        self._load_calibration(camera_type)

        # undistort 저장 (표시용)
        self.undistorted_frame = undistort_frame(frame, self.camera_matrix, self.dist_coeffs)

        # 마커 중심 좌표 검출 (서비스 레이어 호출)
        tag_id1 = self.spinTagID1.value()
        tag_id2 = self.spinTagID2.value()
        colors = {tag_id1: (0, 255, 0), tag_id2: (255, 0, 0)}

        marker1_info = None
        marker2_info = None
        alignment = None

        if self.vision_manager:
            h, w = frame.shape[:2]
            markers = self.vision_manager.detect_marker_centers(
                frame, self.camera_matrix, self.dist_coeffs, estimate_pose=True
            )

            # Compute alignment
            alignment = compute_dual_alignment(markers, tag_id1, tag_id2, w, h)

            # Draw overlay (in-place on frame)
            frame = draw_dual_marker_overlay(frame, markers, tag_id1, tag_id2, alignment, colors, show_info=False)

            # Build marker_info dicts for _update_align_tab_display() (needs depth from DS435)
            if alignment is not None:
                depth1 = depth2 = None
                if self.camera_manager and hasattr(self.camera_manager, 'get_distance_at'):
                    depth1 = self.camera_manager.get_distance_at(
                        int(round(alignment.marker1_cx)), int(round(alignment.marker1_cy)), from_color=True)
                    depth2 = self.camera_manager.get_distance_at(
                        int(round(alignment.marker2_cx)), int(round(alignment.marker2_cy)), from_color=True)
                marker1_info = {
                    'cx': alignment.marker1_cx, 'cy': alignment.marker1_cy,
                    'tvec': alignment.marker1_tvec, 'depth': depth1
                }
                marker2_info = {
                    'cx': alignment.marker2_cx, 'cy': alignment.marker2_cy,
                    'tvec': alignment.marker2_tvec, 'depth': depth2
                }

        # Extract values for display
        angle_2d = alignment.angle_2d if alignment else None
        angle_3d = alignment.angle_3d if alignment else None
        angle_rx = alignment.angle_rx if alignment else None
        offset_y = alignment.offset_y if alignment else None
        offset_z = alignment.offset_z if alignment else None

        self._update_align_tab_display(marker1_info, marker2_info, angle_2d, angle_3d, angle_rx, offset_y, offset_z)
        display_frame_on_label(frame, self.labelCameraView)

    def _update_align_tab_display(self, marker1_info, marker2_info, angle_2d=None, angle_3d=None, angle_rx=None, offset_y=None, offset_z=None):
        """aruco 정렬 탭의 마커 중심 좌표 및 각도 라벨 업데이트"""
        if marker1_info:
            self.labelM1CenterX.setText(f"{marker1_info['cx']:.1f}")
            self.labelM1CenterY.setText(f"{marker1_info['cy']:.1f}")
            d1 = marker1_info.get('depth')
            self.labelM1Depth.setText(f"{d1:.0f}" if d1 is not None else "-")
            t1 = marker1_info.get('tvec')
            self.labelM1ZAr.setText(f"{t1[2]*1000:.1f}" if t1 is not None else "-")
        else:
            self.labelM1CenterX.setText("-")
            self.labelM1CenterY.setText("-")
            self.labelM1Depth.setText("-")
            self.labelM1ZAr.setText("-")

        if marker2_info:
            self.labelM2CenterX.setText(f"{marker2_info['cx']:.1f}")
            self.labelM2CenterY.setText(f"{marker2_info['cy']:.1f}")
            d2 = marker2_info.get('depth')
            self.labelM2Depth.setText(f"{d2:.0f}" if d2 is not None else "-")
            t2 = marker2_info.get('tvec')
            self.labelM2ZAr.setText(f"{t2[2]*1000:.1f}" if t2 is not None else "-")
        else:
            self.labelM2CenterX.setText("-")
            self.labelM2CenterY.setText("-")
            self.labelM2Depth.setText("-")
            self.labelM2ZAr.setText("-")

        # Ry: 2D / 3D
        self.labelMarkerAngle.setText(f"{angle_2d:.2f}" if angle_2d is not None else "-")
        self.labelMarkerAngle3D.setText(f"{angle_3d:.2f}" if angle_3d is not None else "-")
        # Rx: 3D only (Z차이 기반)
        self.labelMarkerRx.setText(f"{angle_rx:.2f}" if angle_rx is not None else "-")

        # Ry 캐시 (3D 우선, 없으면 2D)
        active_ry = angle_3d if angle_3d is not None else angle_2d
        if active_ry is not None:
            self._last_marker_angle = active_ry
        else:
            self._last_marker_angle = None

        # Rz 캐시 + 평균 마커 거리 저장
        if angle_rx is not None:
            self._last_marker_rz_angle = angle_rx
            t1 = marker1_info.get('tvec') if marker1_info else None
            t2 = marker2_info.get('tvec') if marker2_info else None
            if t1 is not None and t2 is not None:
                self._last_marker_distance = (t1[2] + t2[2]) / 2.0 * 1000  # mm
            else:
                self._last_marker_distance = None
        else:
            self._last_marker_rz_angle = None
            self._last_marker_distance = None

        # 중심 오프셋 표시
        self.labelOffsetY.setText(f"{offset_y:.1f}" if offset_y is not None else "-")
        self.labelOffsetZ.setText(f"{offset_z:.1f}" if offset_z is not None else "-")
        if offset_y is not None:
            self._last_offset_y_px = offset_y
        else:
            self._last_offset_y_px = None

        # Ry 정렬 버튼 활성화 (라이브 마커 각도 기반)
        if active_ry is not None:
            self.btnAlignRy.setEnabled(True)
            self.btnAlignRy.setText(f"Ry={active_ry:.1f}°")
        else:
            self.btnAlignRy.setEnabled(False)
            self.btnAlignRy.setText("Ry 정렬")

        # 통합 정렬 버튼 활성화
        has_ry = active_ry is not None
        has_y = offset_y is not None
        has_rz = angle_rx is not None and self._last_marker_distance is not None
        self.btnAlignArucoY.setEnabled(has_ry or has_y)
        self.btnAlignArucoX.setEnabled(has_rz)
        self.btnAlignArucoCombined.setEnabled(has_ry or has_y or has_rz)

    def _on_start_camera(self):
        """카메라 시작"""
        self.camera_start_requested.emit()
        self._log("카메라 시작 요청")

    def _on_stop_camera(self):
        """카메라 정지"""
        self.camera_stop_requested.emit()
        self._log("카메라 정지 요청")

    def _on_start_capture(self):
        """캡처 시작"""
        if not self.camera_manager or not self.camera_manager.is_running:
            QMessageBox.warning(self, "경고", "카메라가 실행 중이지 않습니다.")
            return

        if not self.vision_manager:
            QMessageBox.warning(self, "경고", "비전 매니저가 설정되지 않았습니다.")
            return

        # 초기화
        self.collected_data = []
        self.capture_count = 0
        self.max_captures = self.spinRepeatCount.value()
        self.is_capturing = True

        # 이미지 저장 설정
        self.save_images = hasattr(self, 'checkSaveImages') and self.checkSaveImages.isChecked()
        self.save_folder = None
        if self.save_images:
            # 타임스탬프 폴더 생성
            base_dir = os.path.join(os.path.dirname(__file__), '..', '..', 'images', 'aruco_mark')
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            self.save_folder = os.path.join(base_dir, timestamp)
            os.makedirs(self.save_folder, exist_ok=True)
            self._log(f"이미지 저장 폴더: {self.save_folder}")

        # UI 업데이트
        self.btnStartCapture.setEnabled(False)
        self.btnStopCapture.setEnabled(True)
        self.spinTagID1.setEnabled(False)
        self.spinTagID2.setEnabled(False)
        self.spinRepeatCount.setEnabled(False)
        self.spinStabilizationDelay.setEnabled(False)
        self.progressBar.setValue(0)
        self.progressBar.setMaximum(self.max_captures)
        self.textLog.clear()
        self._log(f"Dual ArUco (ID:{self.spinTagID1.value()}, ID:{self.spinTagID2.value()}) 신뢰성 검증 시작 ({self.max_captures}회)")

        # 캡처 타이머 시작
        delay_ms = self.spinStabilizationDelay.value()
        self.capture_timer.start(delay_ms)

    def _on_stop_capture(self):
        """캡처 중지"""
        self.is_capturing = False
        self.capture_timer.stop()

        # UI 업데이트
        self.btnStartCapture.setEnabled(True)
        self.btnStopCapture.setEnabled(False)
        self.spinTagID1.setEnabled(True)
        self.spinTagID2.setEnabled(True)
        self.spinRepeatCount.setEnabled(True)
        self.spinStabilizationDelay.setEnabled(True)

        self._log(f"캡처 중지됨 (총 {self.capture_count}/{self.max_captures}회)")

    def _on_capture_next(self):
        """다음 캡처 수행 (Dual ArUco 검출, Disambiguation 지원)"""
        if not self.is_capturing or self.capture_count >= self.max_captures:
            self._on_capture_complete()
            return

        tag_id1 = self.spinTagID1.value()
        tag_id2 = self.spinTagID2.value()
        timestamp = datetime.now().isoformat()

        if self.current_frame is None:
            self._log(f"[{self.capture_count + 1}/{self.max_captures}] 프레임 없음")
            self.capture_count += 1
            self.progressBar.setValue(self.capture_count)
            return

        # Disambiguation 체크 여부 확인
        use_disambiguation = hasattr(self, 'checkUseDisambiguation') and self.checkUseDisambiguation.isChecked()
        current_distance_error = None  # Track for logging

        if use_disambiguation:
            # Known distance 가져오기 (mm -> m)
            known_distance_m = self.spinKnownDistance.value() / 1000.0
            self._log(f"[Capture] Using disambiguation: True, known_distance={known_distance_m * 1000:.2f}mm")

            # Disambiguation 방식으로 검출 - 원본 프레임 사용 (solvePnP가 왜곡 처리)
            vis_frame, marker1_pose, marker2_pose, measured_distance, distance_error = \
                self.vision_manager.detect_markers_disambiguated(
                    self.current_frame,
                    (tag_id1, tag_id2),
                    known_distance_m
                )

            if marker1_pose is not None and marker2_pose is not None:
                markers = [marker1_pose, marker2_pose]
                current_distance_error = distance_error
                # Distance error 표시
                if hasattr(self, 'labelDistanceError'):
                    self.labelDistanceError.setText(f"Distance Error: {distance_error * 1000:.2f} mm")
            else:
                markers = []
                if hasattr(self, 'labelDistanceError'):
                    self.labelDistanceError.setText("Distance Error: -- mm")
        else:
            # 기존 방식으로 검출 - 원본 프레임 사용 (solvePnP가 왜곡 처리)
            _, markers = self.vision_manager.detect_markers(self.current_frame)

        # Dual 마커 검출
        marker1_data = None
        marker2_data = None

        if markers:
            for marker in markers:
                if marker['id'] == tag_id1:
                    marker1_data = self._extract_marker_data(marker)
                elif marker['id'] == tag_id2:
                    marker2_data = self._extract_marker_data(marker)

        # 두 마커 모두 검출 시 평면 정보 계산
        both_detected = marker1_data is not None and marker2_data is not None

        # 데이터 저장 (마커1 기준, 마커2 정보도 포함)
        data_entry = {
            'timestamp': timestamp,
            'tag_id': tag_id1,
            'tag_id2': tag_id2,
            'detected': marker1_data is not None,
            'detected2': marker2_data is not None,
            'both_detected': both_detected,
            'tvec': marker1_data['tvec'] if marker1_data else None,
            'rvec': marker1_data['rvec'] if marker1_data else None,
            'euler': marker1_data['euler'] if marker1_data else None,
            'tvec2': marker2_data['tvec'] if marker2_data else None,
            'rvec2': marker2_data['rvec'] if marker2_data else None,
            'euler2': marker2_data['euler'] if marker2_data else None,
        }
        self.collected_data.append(data_entry)

        # 이미지 저장
        if self.save_images and self.save_folder and self.undistorted_frame is not None:
            import cv2
            img_filename = f"frame_{self.capture_count + 1:04d}.jpg"
            img_path = os.path.join(self.save_folder, img_filename)
            cv2.imwrite(img_path, self.undistorted_frame)

        # 로그
        status1 = "O" if marker1_data else "X"
        status2 = "O" if marker2_data else "X"
        dist_err_str = f" distErr={current_distance_error*1000:.2f}mm" if current_distance_error is not None else ""
        self._log(f"[{self.capture_count + 1}/{self.max_captures}] ID{tag_id1}:{status1} ID{tag_id2}:{status2}{dist_err_str}")

        self.capture_count += 1
        self.progressBar.setValue(self.capture_count)

    def _extract_marker_data(self, marker):
        """마커에서 tvec, rvec, euler 추출"""
        tvec_cam = marker.get('tvec', None)
        rvec_cam = marker.get('rvec', None)
        if tvec_cam is None or rvec_cam is None:
            return None
        tvec, rvec = camera_to_vision(tvec_cam, rvec_cam)
        euler = self._rvec_to_euler(rvec)
        return {'tvec': tvec, 'rvec': rvec, 'euler': euler}

    def _on_capture_complete(self):
        """캡처 완료"""
        self.is_capturing = False
        self.capture_timer.stop()

        # UI 복원
        self.btnStartCapture.setEnabled(True)
        self.btnStopCapture.setEnabled(False)
        self.spinTagID1.setEnabled(True)
        self.spinTagID2.setEnabled(True)
        self.spinRepeatCount.setEnabled(True)
        self.spinStabilizationDelay.setEnabled(True)

        self._log(f"\n캡처 완료: 총 {self.capture_count}회")

        # 통계 계산 및 표시
        self._calculate_and_display_statistics()

        # 그래프 그리기
        self._plot_graphs()

        # 이미지 저장 모드일 때 CSV도 같은 폴더에 자동 저장
        if self.save_images and self.save_folder:
            self._auto_save_csv_to_folder()

    def _on_reset(self):
        """초기화"""
        self.collected_data = []
        self.capture_count = 0
        self.progressBar.setValue(0)
        self.textLog.clear()
        self._log("초기화 완료")
        self._update_statistics_ui_m0()
        self._update_statistics_ui_m1()
        self._update_plane_result_ui()
        self._clear_graphs()
        # 정렬 UI 초기화
        self._last_tcp_correction = None
        self.labelAlignCorrection.setText("TCP 보정값: 검증 미완료")
        self.labelAlignCorrection.setStyleSheet("color: gray;")
        self.btnAlignRx.setText("Rx 정렬")
        self.btnAlignRy.setText("Ry 정렬")
        self.btnAlignRz.setText("Rz 정렬")
        self.btnAlignRx.setEnabled(False)
        self.btnAlignRy.setEnabled(False)
        self.btnAlignRz.setEnabled(False)
        self.btnAlignParallel.setEnabled(False)
        self.btnAlignArucoY.setEnabled(False)
        self.btnAlignArucoX.setEnabled(False)
        self.btnAlignArucoCombined.setEnabled(False)
        self.txtAlignDebug.clear()

    def _on_load_csv(self):
        """CSV 파일 읽기"""
        # 파일 선택 대화상자
        file_path, _ = QFileDialog.getOpenFileName(
            self, "CSV 읽기", "", "CSV Files (*.csv)"
        )

        if not file_path:
            return

        try:
            # 기존 데이터 초기화
            self.collected_data = []
            self.capture_count = 0
            self.progressBar.setValue(0)

            with open(file_path, 'r', encoding='utf-8') as f:
                reader = csv.reader(f)

                # 헤더 읽기
                header = next(reader)

                # 헤더 검증
                expected_header = ['Timestamp', 'Tag_ID', 'Detected', 'X_TF1(mm)', 'Y_TF1(mm)', 'Z_TF1(mm)', 'Rx_TF1(deg)', 'Ry_TF1(deg)', 'Rz_TF1(deg)']
                if header != expected_header:
                    QMessageBox.warning(
                        self,
                        "경고",
                        f"CSV 파일 형식이 올바르지 않습니다.\n예상 헤더: {expected_header}\n실제 헤더: {header}"
                    )
                    return

                # 데이터 읽기 (17컬럼: Dual 마커 포맷, 9컬럼: 레거시 단일 마커 포맷)
                loaded_count = 0
                for row in reader:
                    if len(row) == 17:
                        # 신규 Dual 마커 포맷
                        timestamp = row[0]
                        tag_id1 = int(row[1])
                        detected1 = row[2].lower() in ('true', '1', 'yes')
                        tag_id2 = int(row[9])
                        detected2 = row[10].lower() in ('true', '1', 'yes')

                        tvec, euler = None, None
                        tvec2, euler2 = None, None

                        if detected1:
                            try:
                                tvec = np.array([float(row[3]) / 1000.0, float(row[4]) / 1000.0, float(row[5]) / 1000.0])
                                euler = [float(row[6]), float(row[7]), float(row[8])]
                            except (ValueError, IndexError):
                                detected1, tvec, euler = False, None, None

                        if detected2:
                            try:
                                tvec2 = np.array([float(row[11]) / 1000.0, float(row[12]) / 1000.0, float(row[13]) / 1000.0])
                                euler2 = [float(row[14]), float(row[15]), float(row[16])]
                            except (ValueError, IndexError):
                                detected2, tvec2, euler2 = False, None, None

                        data_entry = {
                            'timestamp': timestamp,
                            'tag_id': tag_id1, 'tag_id2': tag_id2,
                            'detected': detected1, 'detected2': detected2,
                            'both_detected': detected1 and detected2,
                            'tvec': tvec, 'rvec': None, 'euler': euler,
                            'tvec2': tvec2, 'rvec2': None, 'euler2': euler2,
                        }
                    elif len(row) == 9:
                        # 레거시 단일 마커 포맷
                        timestamp = row[0]
                        tag_id = int(row[1])
                        detected = row[2].lower() in ('true', '1', 'yes')
                        tvec, euler = None, None

                        if detected:
                            try:
                                tvec = np.array([float(row[3]) / 1000.0, float(row[4]) / 1000.0, float(row[5]) / 1000.0])
                                euler = [float(row[6]), float(row[7]), float(row[8])]
                            except (ValueError, IndexError):
                                detected, tvec, euler = False, None, None

                        data_entry = {
                            'timestamp': timestamp,
                            'tag_id': tag_id,
                            'detected': detected,
                            'tvec': tvec, 'rvec': None, 'euler': euler,
                        }
                    else:
                        continue

                    self.collected_data.append(data_entry)
                    loaded_count += 1

                self._log(f"CSV 파일 읽기 완료: {file_path}")
                self._log(f"총 {loaded_count}개 데이터 로드됨")

                # Tag ID 업데이트 (첫 번째 데이터의 Tag ID 사용)
                if self.collected_data:
                    first_tag_id = self.collected_data[0]['tag_id']
                    self.spinTagID1.setValue(first_tag_id)
                    # tag_id2가 있으면 설정
                    if 'tag_id2' in self.collected_data[0]:
                        self.spinTagID2.setValue(self.collected_data[0]['tag_id2'])

                # 진행률 바 업데이트
                self.progressBar.setValue(loaded_count)
                self.progressBar.setMaximum(loaded_count)

                # 통계 계산 및 표시
                self._calculate_and_display_statistics()

                # 그래프 그리기
                self._plot_graphs()

                QMessageBox.information(
                    self,
                    "성공",
                    f"CSV 파일을 성공적으로 읽었습니다.\n로드된 데이터: {loaded_count}개"
                )

        except Exception as e:
            self._log(f"CSV 읽기 오류: {str(e)}")
            QMessageBox.critical(self, "오류", f"CSV 파일 읽기 실패:\n{str(e)}")

    def _on_export_csv(self):
        """CSV 내보내기"""
        if not self.collected_data:
            QMessageBox.warning(self, "경고", "저장할 데이터가 없습니다.")
            return

        # 파일 저장 대화상자 (Dual ArUco)
        tag_id1 = self.spinTagID1.value()
        tag_id2 = self.spinTagID2.value()
        default_name = f"aruco_dual_id{tag_id1}_{tag_id2}_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"

        # 이미지 저장 폴더가 있으면 해당 폴더 사용, 없으면 기본 images/aruco_mark 폴더
        if hasattr(self, 'save_folder') and self.save_folder and os.path.exists(self.save_folder):
            default_dir = self.save_folder
        else:
            project_root = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
            default_dir = os.path.join(project_root, "images", "aruco_mark")
            os.makedirs(default_dir, exist_ok=True)
        default_path = os.path.join(default_dir, default_name)

        file_path, _ = QFileDialog.getSaveFileName(
            self, "CSV 저장", default_path, "CSV Files (*.csv)"
        )

        if not file_path:
            return

        try:
            with open(file_path, 'w', newline='', encoding='utf-8') as f:
                writer = csv.writer(f)

                # 헤더 (TF1 좌표계, 위치: mm, 회전: deg) - Dual 마커 지원
                writer.writerow([
                    'Timestamp',
                    'Tag_ID1', 'Detected1', 'X1_TF1(mm)', 'Y1_TF1(mm)', 'Z1_TF1(mm)', 'Rx1_TF1(deg)', 'Ry1_TF1(deg)', 'Rz1_TF1(deg)',
                    'Tag_ID2', 'Detected2', 'X2_TF1(mm)', 'Y2_TF1(mm)', 'Z2_TF1(mm)', 'Rx2_TF1(deg)', 'Ry2_TF1(deg)', 'Rz2_TF1(deg)'
                ])

                # 데이터
                for entry in self.collected_data:
                    row = [entry['timestamp']]

                    # 마커 1 데이터
                    row.append(entry.get('tag_id', tag_id1))
                    row.append(entry.get('detected', False))
                    if entry.get('detected') and entry.get('tvec') is not None:
                        tvec_flat = entry['tvec'].flatten()
                        row.extend([
                            tvec_flat[0] * 1000.0,
                            tvec_flat[1] * 1000.0,
                            tvec_flat[2] * 1000.0,
                        ])
                        if entry.get('euler') is not None:
                            row.extend(entry['euler'])
                        else:
                            row.extend([None, None, None])
                    else:
                        row.extend([None, None, None, None, None, None])

                    # 마커 2 데이터
                    row.append(entry.get('tag_id2', tag_id2))
                    row.append(entry.get('detected2', False))
                    if entry.get('detected2') and entry.get('tvec2') is not None:
                        tvec2_flat = entry['tvec2'].flatten()
                        row.extend([
                            tvec2_flat[0] * 1000.0,
                            tvec2_flat[1] * 1000.0,
                            tvec2_flat[2] * 1000.0,
                        ])
                        if entry.get('euler2') is not None:
                            row.extend(entry['euler2'])
                        else:
                            row.extend([None, None, None])
                    else:
                        row.extend([None, None, None, None, None, None])

                    writer.writerow(row)

            self._log(f"CSV 저장 완료: {file_path}")
            QMessageBox.information(self, "성공", f"CSV 파일이 저장되었습니다.\n{file_path}")

        except Exception as e:
            self._log(f"CSV 저장 실패: {str(e)}")
            QMessageBox.critical(self, "오류", f"CSV 저장 중 오류 발생:\n{str(e)}")

    def _auto_save_csv_to_folder(self):
        """이미지 저장 폴더에 CSV 자동 저장"""
        if not self.collected_data or not self.save_folder:
            return

        try:
            tag_id1 = self.spinTagID1.value()
            tag_id2 = self.spinTagID2.value()
            csv_filename = f"aruco_dual_id{tag_id1}_{tag_id2}.csv"
            file_path = os.path.join(self.save_folder, csv_filename)

            with open(file_path, 'w', newline='', encoding='utf-8') as f:
                writer = csv.writer(f)

                # 헤더
                writer.writerow([
                    'Timestamp',
                    'Tag_ID1', 'Detected1', 'X1_TF1(mm)', 'Y1_TF1(mm)', 'Z1_TF1(mm)', 'Rx1_TF1(deg)', 'Ry1_TF1(deg)', 'Rz1_TF1(deg)',
                    'Tag_ID2', 'Detected2', 'X2_TF1(mm)', 'Y2_TF1(mm)', 'Z2_TF1(mm)', 'Rx2_TF1(deg)', 'Ry2_TF1(deg)', 'Rz2_TF1(deg)'
                ])

                # 데이터
                for entry in self.collected_data:
                    row = [entry['timestamp']]

                    # 마커 1
                    row.append(entry.get('tag_id', tag_id1))
                    row.append(entry.get('detected', False))
                    if entry.get('detected') and entry.get('tvec') is not None:
                        tvec_flat = entry['tvec'].flatten()
                        row.extend([tvec_flat[0] * 1000.0, tvec_flat[1] * 1000.0, tvec_flat[2] * 1000.0])
                        row.extend(entry['euler'] if entry.get('euler') is not None else [None, None, None])
                    else:
                        row.extend([None, None, None, None, None, None])

                    # 마커 2
                    row.append(entry.get('tag_id2', tag_id2))
                    row.append(entry.get('detected2', False))
                    if entry.get('detected2') and entry.get('tvec2') is not None:
                        tvec2_flat = entry['tvec2'].flatten()
                        row.extend([tvec2_flat[0] * 1000.0, tvec2_flat[1] * 1000.0, tvec2_flat[2] * 1000.0])
                        row.extend(entry['euler2'] if entry.get('euler2') is not None else [None, None, None])
                    else:
                        row.extend([None, None, None, None, None, None])

                    writer.writerow(row)

            self._log(f"CSV 자동 저장 완료: {file_path}")

        except Exception as e:
            self._log(f"CSV 자동 저장 실패: {str(e)}")

    def _on_export_graph(self):
        """그래프 이미지 저장"""
        if not self.collected_data:
            QMessageBox.warning(self, "경고", "저장할 그래프가 없습니다.")
            return

        # 파일 저장 대화상자
        default_name = f"aruco_reliability_graph_{datetime.now().strftime('%Y%m%d_%H%M%S')}.png"
        file_path, _ = QFileDialog.getSaveFileName(
            self, "그래프 저장", default_name, "PNG Files (*.png);;PDF Files (*.pdf)"
        )

        if not file_path:
            return

        try:
            active_figure = self.figure_raw if self.graph_tab_widget.currentIndex() == 0 else self.figure_filtered
            active_figure.savefig(file_path, dpi=300, bbox_inches='tight')
            self._log(f"그래프 저장 완료: {file_path}")
            QMessageBox.information(self, "성공", f"그래프가 저장되었습니다.\n{file_path}")

        except Exception as e:
            self._log(f"그래프 저장 실패: {str(e)}")
            QMessageBox.critical(self, "오류", f"그래프 저장 중 오류 발생:\n{str(e)}")

    def _calculate_and_display_statistics(self):
        """통계 계산 및 UI 업데이트 (2σ Outlier 필터링 포함, Dual 마커 지원)"""
        if not self.collected_data:
            self._update_statistics_ui_m0()
            self._update_statistics_ui_m1()
            return

        # Dual 마커 통계 각각 계산
        self._log(f"\n{'='*50}")
        self._log(f"=== Dual ArUco 신뢰성 분석 리포트 ===")
        self._log(f"{'='*50}")

        # 마커1 통계
        detected_data1 = [d for d in self.collected_data if d['detected'] and d['tvec'] is not None]
        stats1 = self._calculate_marker_statistics(detected_data1, "마커1")

        # 마커2 통계
        detected_data2 = [d for d in self.collected_data if d.get('detected2') and d.get('tvec2') is not None]
        stats2 = self._calculate_marker_statistics(detected_data2, "마커2", key_suffix='2')

        # 둘 다 검출된 경우
        both_detected = [d for d in self.collected_data if d.get('both_detected')]
        self._log(f"\n양쪽 마커 동시 검출: {len(both_detected)}/{len(self.collected_data)}개 ({len(both_detected)/len(self.collected_data)*100:.1f}%)")

        # UI 업데이트: 마커1과 마커2 각각
        if stats1:
            detection_rate1 = len(detected_data1) / len(self.collected_data) * 100
            self._update_statistics_ui_m0(
                stats1['x_mean'], stats1['y_mean'], stats1['z_mean'],
                stats1['x_std'], stats1['y_std'], stats1['z_std'],
                stats1.get('rx_mean', 0), stats1.get('ry_mean', 0), stats1.get('rz_mean', 0),
                stats1.get('rx_std', 0), stats1.get('ry_std', 0), stats1.get('rz_std', 0),
                detection_rate1,
                stats1.get('x_mean_raw', 0), stats1.get('x_std_raw', 0),
                stats1.get('y_mean_raw', 0), stats1.get('y_std_raw', 0),
                stats1.get('z_mean_raw', 0), stats1.get('z_std_raw', 0),
                stats1.get('rx_mean_raw', 0), stats1.get('rx_std_raw', 0),
                stats1.get('ry_mean_raw', 0), stats1.get('ry_std_raw', 0),
                stats1.get('rz_mean_raw', 0), stats1.get('rz_std_raw', 0)
            )
        else:
            self._update_statistics_ui_m0()

        if stats2:
            self._update_statistics_ui_m1(
                stats2['x_mean'], stats2['y_mean'], stats2['z_mean'],
                stats2['x_std'], stats2['y_std'], stats2['z_std'],
                stats2.get('rx_mean', 0), stats2.get('ry_mean', 0), stats2.get('rz_mean', 0),
                stats2.get('rx_std', 0), stats2.get('ry_std', 0), stats2.get('rz_std', 0),
                stats2.get('x_mean_raw', 0), stats2.get('x_std_raw', 0),
                stats2.get('y_mean_raw', 0), stats2.get('y_std_raw', 0),
                stats2.get('z_mean_raw', 0), stats2.get('z_std_raw', 0),
                stats2.get('rx_mean_raw', 0), stats2.get('rx_std_raw', 0),
                stats2.get('ry_mean_raw', 0), stats2.get('ry_std_raw', 0),
                stats2.get('rz_mean_raw', 0), stats2.get('rz_std_raw', 0)
            )
        else:
            self._update_statistics_ui_m1()

        # 평면 결과 계산
        self._calculate_plane_from_data()

        # UI는 마커1 기준으로 표시 (기존 호환성)
        detected_data = detected_data1

        if not detected_data:
            self._log("검출 성공한 데이터가 없습니다.")
            self._update_statistics_ui_m0()
            return

        # tvec + euler 데이터 추출 (aligned arrays)
        tvec_array, euler_array = self._extract_aligned_arrays(detected_data)

        # 디버깅: Euler 데이터 확인
        self._log(f"\n검출된 데이터: {len(detected_data)}개")

        # === Outlier 필터링 전 통계 (Before) ===
        x_mm_before = tvec_array[:, 0] * 1000.0
        y_mm_before = tvec_array[:, 1] * 1000.0
        z_mm_before = tvec_array[:, 2] * 1000.0

        x_mean_before = np.mean(x_mm_before)
        y_mean_before = np.mean(y_mm_before)
        z_mean_before = np.mean(z_mm_before)
        x_std_before = np.std(x_mm_before, ddof=1)
        y_std_before = np.std(y_mm_before, ddof=1)
        z_std_before = np.std(z_mm_before, ddof=1)

        # === Outlier 필터링 (unified) ===
        sigma = self.spinSigmaMultiplier.value()
        min_samples = self.spinMinSamples.value()
        tvec_filtered, euler_filtered, valid_mask, removal_info = self._remove_outliers(
            tvec_array, euler_array, sigma=sigma, min_samples=min_samples)
        num_outliers = removal_info['total_removed']
        outlier_indices = np.where(~valid_mask)[0]

        # === Outlier 필터링 후 통계 (After) ===
        x_mean = np.mean(tvec_filtered[:, 0]) * 1000.0
        y_mean = np.mean(tvec_filtered[:, 1]) * 1000.0
        z_mean = np.mean(tvec_filtered[:, 2]) * 1000.0

        x_std = np.std(tvec_filtered[:, 0], ddof=1) * 1000.0
        y_std = np.std(tvec_filtered[:, 1], ddof=1) * 1000.0
        z_std = np.std(tvec_filtered[:, 2], ddof=1) * 1000.0

        detection_rate = len(detected_data) / len(self.collected_data) * 100

        # Euler 각도 통계 (필터링 전)
        rx_mean_raw = ry_mean_raw = rz_mean_raw = 0.0
        rx_std_raw = ry_std_raw = rz_std_raw = 0.0
        if len(euler_array) > 0:
            rx_mean_raw = np.nanmean(euler_array[:, 0])
            ry_mean_raw = np.nanmean(euler_array[:, 1])
            rz_mean_raw = np.nanmean(euler_array[:, 2])
            rx_std_raw = np.nanstd(euler_array[:, 0], ddof=1)
            ry_std_raw = np.nanstd(euler_array[:, 1], ddof=1)
            rz_std_raw = np.nanstd(euler_array[:, 2], ddof=1)

        # Euler 각도 통계 (필터링 후)
        rx_mean = ry_mean = rz_mean = 0.0
        rx_std = ry_std = rz_std = 0.0
        if len(euler_filtered) > 0:
            rx_mean = np.nanmean(euler_filtered[:, 0])
            ry_mean = np.nanmean(euler_filtered[:, 1])
            rz_mean = np.nanmean(euler_filtered[:, 2])
            rx_std = np.nanstd(euler_filtered[:, 0], ddof=1)
            ry_std = np.nanstd(euler_filtered[:, 1], ddof=1)
            rz_std = np.nanstd(euler_filtered[:, 2], ddof=1)

        # UI 업데이트 (필터링 후 + 전 값)
        self._update_statistics_ui_m0(
            x_mean, y_mean, z_mean, x_std, y_std, z_std,
            rx_mean, ry_mean, rz_mean, rx_std, ry_std, rz_std,
            detection_rate,
            x_mean_before, x_std_before, y_mean_before, y_std_before,
            z_mean_before, z_std_before,
            rx_mean_raw, rx_std_raw, ry_mean_raw, ry_std_raw,
            rz_mean_raw, rz_std_raw
        )

        # 로그 출력 - Before/After 비교 리포트
        self._log(f"\n{'='*50}")
        self._log(f"=== ArUco 신뢰성 분석 리포트 ===")
        self._log(f"{'='*50}")
        self._log(f"총 샘플: {len(self.collected_data)}개")
        self._log(f"검출 성공: {len(detected_data)}개 ({detection_rate:.1f}%)")
        self._log(f"Outlier: {num_outliers}개 ({num_outliers/len(detected_data)*100:.1f}%)")
        if num_outliers > 0:
            self._log(f"Outlier 인덱스: {outlier_indices.tolist()}")

        self._log(f"\n--- Outlier 필터링 전 (Before) ---")
        self._log(f"위치: X={x_mean_before:.2f}±{x_std_before:.2f}, Y={y_mean_before:.2f}±{y_std_before:.2f}, Z={z_mean_before:.2f}±{z_std_before:.2f} mm")

        self._log(f"\n--- Outlier 필터링 후 (After, {sigma}σ) ---")
        self._log(f"유효 샘플: {np.sum(valid_mask)}개")
        self._log(f"위치: X={x_mean:.2f}±{x_std:.2f}, Y={y_mean:.2f}±{y_std:.2f}, Z={z_mean:.2f}±{z_std:.2f} mm")
        self._log(f"회전: Rx={rx_mean:.2f}±{rx_std:.2f}, Ry={ry_mean:.2f}±{ry_std:.2f}, Rz={rz_mean:.2f}±{rz_std:.2f}°")

        # 개선율 계산
        if x_std_before > 0:
            x_improve = (1 - x_std / x_std_before) * 100
            y_improve = (1 - y_std / y_std_before) * 100
            z_improve = (1 - z_std / z_std_before) * 100
            self._log(f"\n--- 정밀도 개선율 ---")
            self._log(f"X: {x_improve:.1f}%, Y: {y_improve:.1f}%, Z: {z_improve:.1f}%")

        self._log(f"{'='*50}")

    # --- Unified outlier removal helpers ---
    # Replaces 4 separate outlier removal paths (A: L935 inline, B: L1174 3-sigma, C: L1120 iterative, D: L1628 inline)

    def _extract_aligned_arrays(self, detected_data, key_suffix=''):
        """
        Extract aligned tvec and euler arrays from detected data.
        Guarantees tvec_array and euler_array have the same number of rows
        by replacing None euler entries with [NaN, NaN, NaN].

        Args:
            detected_data: list of dicts with 'tvec'/'euler' (or 'tvec2'/'euler2') keys
            key_suffix: '' for marker1, '2' for marker2

        Returns:
            tuple: (tvec_array, euler_array)
                   tvec_array: np.ndarray shape (N, 3) in meters
                   euler_array: np.ndarray shape (N, 3) in degrees, with NaN for missing entries
        """
        tvec_key = 'tvec' + key_suffix
        euler_key = 'euler' + key_suffix

        tvec_array = np.array([d[tvec_key].flatten() for d in detected_data])

        euler_list = [d.get(euler_key) for d in detected_data]
        if any(e is not None for e in euler_list):
            euler_array = np.array([
                e if e is not None else [np.nan, np.nan, np.nan]
                for e in euler_list
            ])
        else:
            euler_array = np.full((len(detected_data), 3), np.nan)

        return tvec_array, euler_array

    def _remove_outliers(self, tvec_array, euler_array,
                         sigma=2.0, max_iterations=5, min_samples=5):
        """
        Unified iterative outlier removal across all 6 DOF.

        Args:
            tvec_array: np.ndarray shape (N, 3) in meters
            euler_array: np.ndarray shape (N, 3) in degrees (may contain NaN)
            sigma: float, number of standard deviations for threshold (default 2.0)
            max_iterations: int, max removal passes (default 5)
            min_samples: int, stop if fewer samples remain (default 5)

        Returns:
            tuple: (filtered_tvec, filtered_euler, valid_mask, removal_info)
            valid_mask: boolean array relative to original input
            removal_info: dict with keys 'total_removed', 'iterations_used',
                          'per_iteration_removed', 'initial_count', 'final_count'
        """
        n = len(tvec_array)
        valid_mask = np.ones(n, dtype=bool)
        per_iteration_removed = []

        for iteration in range(max_iterations):
            current_tvec = tvec_array[valid_mask]
            current_euler = euler_array[valid_mask]
            current_n = len(current_tvec)

            if current_n <= min_samples:
                break

            # Build outlier mask for this iteration (True = keep)
            iter_mask = np.ones(current_n, dtype=bool)

            # Position axes (X, Y, Z)
            for i in range(3):
                col = current_tvec[:, i]
                mean = np.mean(col)
                std = np.std(col, ddof=1)
                if std > 0:
                    iter_mask &= (np.abs(col - mean) <= sigma * std)

            # Rotation axes (Rx, Ry, Rz) - skip if all NaN
            for i in range(3):
                col = current_euler[:, i]
                if np.all(np.isnan(col)):
                    continue
                mean = np.nanmean(col)
                std = np.nanstd(col, ddof=1)
                if std > 0:
                    # For NaN entries, keep them (don't mark as outlier)
                    axis_mask = np.isnan(col) | (np.abs(col - mean) <= sigma * std)
                    iter_mask &= axis_mask

            # Check minimum samples guard
            remaining = np.sum(iter_mask)
            if remaining < min_samples:
                # Don't apply this iteration's removal
                per_iteration_removed.append(0)
                break

            removed_this_iter = current_n - remaining
            per_iteration_removed.append(removed_this_iter)

            if removed_this_iter == 0:
                break

            # Map iter_mask back to valid_mask
            current_indices = np.where(valid_mask)[0]
            for j, idx in enumerate(current_indices):
                if not iter_mask[j]:
                    valid_mask[idx] = False

        filtered_tvec = tvec_array[valid_mask]
        filtered_euler = euler_array[valid_mask]
        total_removed = n - np.sum(valid_mask)

        removal_info = {
            'total_removed': total_removed,
            'iterations_used': len(per_iteration_removed),
            'per_iteration_removed': per_iteration_removed,
            'initial_count': n,
            'final_count': np.sum(valid_mask)
        }

        return filtered_tvec, filtered_euler, valid_mask, removal_info

    def _calculate_marker_statistics(self, detected_data, marker_name, key_suffix=''):
        """개별 마커의 통계 계산 및 로그 출력 (이상치 제거 전후 비교)"""
        if not detected_data:
            self._log(f"\n--- {marker_name} ---")
            self._log(f"검출된 데이터 없음")
            return None

        # tvec + euler 데이터 추출 (aligned arrays)
        tvec_array, euler_array = self._extract_aligned_arrays(detected_data, key_suffix)

        # mm 단위 변환
        x_mm_all = tvec_array[:, 0] * 1000.0
        y_mm_all = tvec_array[:, 1] * 1000.0
        z_mm_all = tvec_array[:, 2] * 1000.0

        # === 진짜 raw 통계 (모든 필터링 전) ===
        x_mean_raw = np.mean(x_mm_all)
        y_mean_raw = np.mean(y_mm_all)
        z_mean_raw = np.mean(z_mm_all)
        x_std_raw = np.std(x_mm_all, ddof=1)
        y_std_raw = np.std(y_mm_all, ddof=1)
        z_std_raw = np.std(z_mm_all, ddof=1)

        # Euler raw 통계 (모든 필터링 전)
        if len(euler_array) > 0:
            rx_mean_raw = np.nanmean(euler_array[:, 0])
            ry_mean_raw = np.nanmean(euler_array[:, 1])
            rz_mean_raw = np.nanmean(euler_array[:, 2])
            rx_std_raw = np.nanstd(euler_array[:, 0], ddof=1)
            ry_std_raw = np.nanstd(euler_array[:, 1], ddof=1)
            rz_std_raw = np.nanstd(euler_array[:, 2], ddof=1)
        else:
            rx_mean_raw = ry_mean_raw = rz_mean_raw = 0
            rx_std_raw = ry_std_raw = rz_std_raw = 0

        # === 이상치 제거 전 통계 ===
        x_mean_before = np.mean(x_mm_all)
        y_mean_before = np.mean(y_mm_all)
        z_mean_before = np.mean(z_mm_all)
        x_std_before = np.std(x_mm_all, ddof=1)
        y_std_before = np.std(y_mm_all, ddof=1)
        z_std_before = np.std(z_mm_all, ddof=1)

        # === Outlier 필터링 (unified) ===
        sigma = self.spinSigmaMultiplier.value()
        min_samples = self.spinMinSamples.value()
        tvec_filtered, euler_filtered, valid_mask, removal_info = self._remove_outliers(
            tvec_array, euler_array, sigma=sigma, min_samples=min_samples)
        outlier_indices = np.where(~valid_mask)[0]

        # === 이상치 제거 후 데이터 ===
        x_mm = tvec_filtered[:, 0] * 1000.0
        y_mm = tvec_filtered[:, 1] * 1000.0
        z_mm = tvec_filtered[:, 2] * 1000.0

        # === 이상치 제거 후 통계 ===
        x_mean = np.mean(x_mm)
        y_mean = np.mean(y_mm)
        z_mean = np.mean(z_mm)
        x_std = np.std(x_mm, ddof=1)
        y_std = np.std(y_mm, ddof=1)
        z_std = np.std(z_mm, ddof=1)

        # === 로그 출력 ===
        self._log(f"\n{'='*60}")
        self._log(f"[{marker_name}]")
        self._log(f"{'='*60}")

        # 이상치 제거 전
        self._log(f"\n📊 이상치 제거 전 (샘플 수: {len(x_mm_all)})")
        self._log(f"{'-'*60}")
        self._log(f"  X: Mean={x_mean_before:8.2f}mm  Std={x_std_before:6.3f}mm")
        self._log(f"  Y: Mean={y_mean_before:8.2f}mm  Std={y_std_before:6.3f}mm")
        self._log(f"  Z: Mean={z_mean_before:8.2f}mm  Std={z_std_before:6.3f}mm")

        # 이상치 정보
        if len(outlier_indices) > 0:
            self._log(f"\n⚠️  이상치 감지: {len(outlier_indices)}개")
            self._log(f"{'-'*60}")
            for idx in outlier_indices[:5]:  # 최대 5개만 표시
                self._log(f"  Index {idx}: X={x_mm_all[idx]:7.2f}mm, Y={y_mm_all[idx]:7.2f}mm, Z={z_mm_all[idx]:7.2f}mm")
            if len(outlier_indices) > 5:
                self._log(f"  ... 외 {len(outlier_indices)-5}개")
        else:
            self._log(f"\n✅ 이상치 없음")

        # 이상치 제거 후
        self._log(f"\n✨ 이상치 제거 후 (샘플 수: {len(x_mm)})")
        self._log(f"{'-'*60}")
        self._log(f"  X: Mean={x_mean:8.2f}mm  Std={x_std:6.3f}mm")
        self._log(f"  Y: Mean={y_mean:8.2f}mm  Std={y_std:6.3f}mm")
        self._log(f"  Z: Mean={z_mean:8.2f}mm  Std={z_std:6.3f}mm")

        # 개선 효과
        if len(outlier_indices) > 0:
            self._log(f"\n📈 개선 효과")
            self._log(f"{'-'*60}")

            for axis_name, std_before, std_after in [
                ('X', x_std_before, x_std),
                ('Y', y_std_before, y_std),
                ('Z', z_std_before, z_std)
            ]:
                improvement = (std_before - std_after) / std_before * 100 if std_before > 0 else 0
                status = "✅" if std_after < 1.0 else "⚠️" if std_after < 3.0 else "❌"
                self._log(f"  {axis_name} Std: {std_before:6.3f}mm → {std_after:6.3f}mm  (개선: {improvement:5.1f}%) {status}")

        stats = {
            'x_mean': x_mean, 'x_std': x_std,
            'y_mean': y_mean, 'y_std': y_std,
            'z_mean': z_mean, 'z_std': z_std,
            # 이상치 제거 전 값 (진짜 raw - 모든 필터링 전)
            'x_mean_raw': x_mean_raw, 'x_std_raw': x_std_raw,
            'y_mean_raw': y_mean_raw, 'y_std_raw': y_std_raw,
            'z_mean_raw': z_mean_raw, 'z_std_raw': z_std_raw,
        }

        # Euler 각도 통계 (NaN 무시, 이상치 제거 후)
        if len(euler_filtered) > 0:
            rx_mean = np.nanmean(euler_filtered[:, 0])
            ry_mean = np.nanmean(euler_filtered[:, 1])
            rz_mean = np.nanmean(euler_filtered[:, 2])
            rx_std = np.nanstd(euler_filtered[:, 0], ddof=1)
            ry_std = np.nanstd(euler_filtered[:, 1], ddof=1)
            rz_std = np.nanstd(euler_filtered[:, 2], ddof=1)

            self._log(f"\n회전 (이상치 제거 후):")
            self._log(f"  Rx={rx_mean:6.2f}±{rx_std:5.2f}°, Ry={ry_mean:6.2f}±{ry_std:5.2f}°, Rz={rz_mean:6.2f}±{rz_std:5.2f}°")

            stats.update({
                'rx_mean': rx_mean, 'rx_std': rx_std,
                'ry_mean': ry_mean, 'ry_std': ry_std,
                'rz_mean': rz_mean, 'rz_std': rz_std,
                # 이상치 제거 전 회전 값 (raw)
                'rx_mean_raw': rx_mean_raw, 'rx_std_raw': rx_std_raw,
                'ry_mean_raw': ry_mean_raw, 'ry_std_raw': ry_std_raw,
                'rz_mean_raw': rz_mean_raw, 'rz_std_raw': rz_std_raw,
            })

        return stats

    def _calculate_plane_from_data(self):
        """수집된 Dual 마커 데이터로 평면 계산"""
        # 양쪽 마커 모두 검출된 데이터만 추출
        both_detected = [d for d in self.collected_data
                        if d.get('both_detected') and d.get('tvec') is not None and d.get('tvec2') is not None]

        if len(both_detected) < 3:
            self._log(f"평면 계산 불가: 유효 샘플 {len(both_detected)}개 (최소 3개 필요)")
            self._update_plane_result_ui()
            return

        # 중심점 계산 (두 마커의 중간점)
        centers = []
        for d in both_detected:
            tvec1 = d['tvec'].flatten()
            tvec2 = d['tvec2'].flatten()
            center = (tvec1 + tvec2) / 2.0
            centers.append(center)

        centers = np.array(centers)

        # 평균 중심점 (mm 단위로 변환)
        avg_center = np.mean(centers, axis=0) * 1000.0  # m -> mm

        # 수평 벡터 (마커1 -> 마커2)
        horizontals = []
        for d in both_detected:
            tvec1 = d['tvec'].flatten()
            tvec2 = d['tvec2'].flatten()
            h = tvec2 - tvec1
            h = h / np.linalg.norm(h)
            horizontals.append(h)

        avg_horizontal = np.mean(horizontals, axis=0)
        avg_horizontal = avg_horizontal / np.linalg.norm(avg_horizontal)

        # 법선 벡터 계산 (Euler 각도에서 추출)
        # 두 마커의 평균 자세를 사용하여 법선 계산
        from services.plane_utils import normal_horizontal_to_euler

        # 마커들의 평균 자세에서 법선 추정 (간단히 Z축 방향 사용)
        # 실제로는 rvec에서 계산해야 하지만, 데이터에 rvec이 없을 수 있으므로
        # Euler 각도에서 역으로 법선 추정
        euler1_list = [d['euler'] for d in both_detected if d.get('euler') is not None]
        euler2_list = [d['euler2'] for d in both_detected if d.get('euler2') is not None]

        if euler1_list and euler2_list:
            # 평균 Euler 각도
            avg_euler1 = np.mean(euler1_list, axis=0)
            avg_euler2 = np.mean(euler2_list, axis=0)
            avg_euler = (avg_euler1 + avg_euler2) / 2.0

            rx, ry, rz = avg_euler[0], avg_euler[1], avg_euler[2]
        else:
            # Euler 데이터가 없으면 수평 벡터에서 추정
            rx, ry, rz = normal_horizontal_to_euler(
                np.array([0, 0, -1]),  # 기본 법선 (카메라 방향)
                avg_horizontal
            )

        # 표준편차 계산
        center_std = np.std(centers, axis=0, ddof=1) * 1000.0  # mm

        # PlanePose 생성
        plane_pose = PlanePose(
            x=avg_center[0],
            y=avg_center[1],
            z=avg_center[2],
            rx=rx,
            ry=ry,
            rz=rz,
            valid_samples=len(both_detected),
            total_attempts=len(self.collected_data),
            std_position_mm=(center_std[0], center_std[1], center_std[2])
        )

        # TCP 보정값 계산
        corrector = TCPCorrector(target_rx=0, target_ry=0, target_rz=0)
        correction = corrector.compute_correction(plane_pose)
        self._last_tcp_correction = (correction.delta_rx, correction.delta_ry, correction.delta_rz)

        # 정렬 버튼 활성화 및 보정값 표시
        if hasattr(self, 'btnAlignParallel'):
            self.btnAlignParallel.setEnabled(True)
            self.btnAlignRx.setEnabled(True)
            self.btnAlignRz.setEnabled(True)
            self.btnAlignRx.setText(f"Rx={correction.delta_rx:.1f}°")
            self.btnAlignRz.setText(f"Rz={correction.delta_rz:.1f}°")
            self.labelAlignCorrection.setText(
                f"TCP 보정: dRx={correction.delta_rx:.2f}, dRy={correction.delta_ry:.2f}, dRz={correction.delta_rz:.2f}°")
            self.labelAlignCorrection.setStyleSheet("color: #e91e63; font-weight: bold;")

        # 로그 출력
        self._log(f"\n{'='*60}")
        self._log(f"📐 평면 결과 (Dual ArUco)")
        self._log(f"{'='*60}")
        self._log(f"유효 샘플: {len(both_detected)}/{len(self.collected_data)}개")
        self._log(f"평면 중심: X={plane_pose.x:.2f}, Y={plane_pose.y:.2f}, Z={plane_pose.z:.2f} mm")
        self._log(f"평면 자세: Rx={plane_pose.rx:.2f}, Ry={plane_pose.ry:.2f}, Rz={plane_pose.rz:.2f}°")
        self._log(f"위치 Std: X={center_std[0]:.3f}, Y={center_std[1]:.3f}, Z={center_std[2]:.3f} mm")
        self._log(f"TCP 보정: dRx={correction.delta_rx:.2f}, dRy={correction.delta_ry:.2f}, dRz={correction.delta_rz:.2f}°")

        # UI 업데이트
        self._update_plane_result_ui(plane_pose, correction)

    def _update_plane_result_ui(self, plane_pose=None, correction=None):
        """평면 결과 UI 업데이트"""
        if not hasattr(self, 'labelPlanePosition'):
            return

        if plane_pose is None:
            self.labelPlanePosition.setText("-")
            self.labelPlaneOrientation.setText("-")
            self.labelTCPCorrection.setText("-")
        else:
            self.labelPlanePosition.setText(
                f"X={plane_pose.x:.2f}, Y={plane_pose.y:.2f}, Z={plane_pose.z:.2f} mm"
            )
            self.labelPlaneOrientation.setText(
                f"Rx={plane_pose.rx:.2f}, Ry={plane_pose.ry:.2f}, Rz={plane_pose.rz:.2f}°"
            )
            if correction:
                self.labelTCPCorrection.setText(
                    f"dRx={correction.delta_rx:.2f}, dRy={correction.delta_ry:.2f}, dRz={correction.delta_rz:.2f}°"
                )

        # 현재 로봇 포즈 표시
        self._update_robot_pose_ui()

    def _on_align_ry_from_angle(self):
        """aruco 정렬 탭 - 마커 기울기 기반 TCP Ry movel 보정"""
        angle = getattr(self, '_last_marker_angle', None)
        if angle is None:
            QMessageBox.warning(self, "경고", "마커 기울기 값이 없습니다.")
            return
        self._log(f"TCP Ry 보정 요청: {angle:.2f}°")
        self.align_base_ry_requested.emit(angle)

    def _on_align_rz_from_angle(self):
        """aruco 정렬 탭 - 마커 Z차이 기반 TCP Rz + Y보정"""
        angle = getattr(self, '_last_marker_rz_angle', None)
        distance = getattr(self, '_last_marker_distance', None)
        if angle is None:
            QMessageBox.warning(self, "경고", "Rz 값이 없습니다.")
            return
        if distance is None:
            QMessageBox.warning(self, "경고", "마커 거리 값이 없습니다.")
            return
        self._log(f"TCP Rz 보정 요청: {angle:.2f}°, D={distance:.0f}mm")
        self.align_base_rz_requested.emit(angle, distance)

    def _on_align_base_y(self):
        """aruco 정렬 탭 - 이미지 중심 기준 Base Y 위치 보정"""
        dy_px = getattr(self, '_last_offset_y_px', None)
        if dy_px is None:
            QMessageBox.warning(self, "경고", "dY 값이 없습니다.")
            return
        self._log(f"Base Y 보정 요청: dY={dy_px:.1f}px")
        self.align_base_y_requested.emit(dy_px)

    def _on_align_aruco_y(self):
        """통합 ArUco Y 정렬 요청 (Ry + Base Y)"""
        self._log("ArUco 정렬 Y 요청")
        self.align_aruco_y_requested.emit()

    def _on_align_aruco_x(self):
        """통합 ArUco X 정렬 요청 (Rz)"""
        self._log("ArUco 정렬 X 요청")
        self.align_aruco_x_requested.emit()

    def _on_align_aruco_combined(self):
        """통합 ArUco 정렬 요청 (Y + X)"""
        self._log("통합 ArUco 정렬 요청")
        self.align_aruco_combined_requested.emit()

    def _on_align_single_axis(self, axis: str):
        """개별 축 정렬 버튼 핸들러"""
        if not hasattr(self, '_last_tcp_correction') or self._last_tcp_correction is None:
            QMessageBox.warning(self, "경고", "TCP 보정값이 없습니다.\n먼저 신뢰성 검증을 실행하세요.")
            return
        drx, dry, drz = self._last_tcp_correction
        angle_map = {'rx': drx, 'ry': dry, 'rz': drz}
        angle = angle_map.get(axis, 0)
        self._log(f"개별 축 정렬 요청: {axis.upper()}={angle:.2f}°")
        self.align_single_axis_requested.emit(axis, angle)

    def _on_align_parallel(self):
        """마커 평행 정렬 버튼 핸들러 - TF5 기준 tool.rot 회전"""
        if not hasattr(self, '_last_tcp_correction') or self._last_tcp_correction is None:
            QMessageBox.warning(self, "경고", "TCP 보정값이 없습니다.\n먼저 신뢰성 검증을 실행하세요.")
            return

        drx, dry, drz = self._last_tcp_correction
        self._log(f"마커 평행 정렬 요청: dRx={drx:.2f}, dRy={dry:.2f}, dRz={drz:.2f}°")
        self.align_parallel_requested.emit(drx, dry, drz)

    def _on_set_detection_pose(self):
        """set_rz.py 방식: TF5 전환 → 현재XYZ+목표RxRyRz movel → TF5 유지"""
        if self.robot is None:
            QMessageBox.warning(self, "경고", "로봇이 연결되지 않았습니다.")
            return

        # 버튼 클릭 시 config 다시 읽기
        self._load_detection_pose_config()

        import time
        from PyQt5.QtWidgets import QApplication

        try:
            # 1) TF5으로 전환
            self._log("Detection Pose: TF5으로 전환")
            success, msg = self.robot.send_set_toolframe(5, wait=True)
            if not success:
                self._log(f"TF5 전환 실패: {msg}")
                return
            time.sleep(0.5)

            # 2) 현재 위치 읽기 (set_rz.py와 동일: reg 158~169, float32)
            pose = self.robot.read_current_pose()
            if pose is None:
                QMessageBox.warning(self, "경고", "TCP 좌표를 읽을 수 없습니다.")
                return

            x, y, z = pose[0], pose[1], pose[2]
            tgt_rx = self.spinDetPoseRx.value()
            tgt_ry = self.spinDetPoseRy.value()
            tgt_rz = self.spinDetPoseRz.value()

            self._log(f"현재: X={x:.1f} Y={y:.1f} Z={z:.1f} Rx={pose[3]:.1f} Ry={pose[4]:.1f} Rz={pose[5]:.1f}")
            self._log(f"목표: X={x:.1f} Y={y:.1f} Z={z:.1f} Rx={tgt_rx:.1f} Ry={tgt_ry:.1f} Rz={tgt_rz:.1f}")

            # 3) 레지스터 직접 쓰기 (set_rz.py 방식 그대로)
            to_int16 = self.robot.to_uint16
            regs = [
                to_int16(int(x * 10)),
                to_int16(int(y * 10)),
                to_int16(int(z * 10)),
                to_int16(int(tgt_rx * 10)),
                to_int16(int(tgt_ry * 10)),
                to_int16(int(tgt_rz * 10)),
            ]
            self.robot.write_registers(self.robot.REGISTER_POSE_MAIN, regs)
            self.robot.write_command(self.robot.CMD_MOVE_TO_POSE)

            # 4) 완료 대기
            success, msg = self.robot.wait_for_done_motion_aware(
                process_events_callback=QApplication.processEvents
            )
            if not success:
                self._log(f"Detection Pose 이동 실패: {msg}")
                return

            self._log("Detection Pose 이동 완료")
            time.sleep(0.5)

            # 5) TF5 유지 확인
            success, msg = self.robot.send_set_toolframe(5, wait=True)
            if success:
                self._log("TF5 확인 완료")
            else:
                self._log(f"TF5 설정 실패: {msg}")

        except Exception as e:
            QMessageBox.warning(self, "오류", f"Detection Pose 실패: {e}")

    def _on_set_robot_position(self):
        """Config의 detection_pose XYZ + RxRyRz로 절대 이동 (TF5→movel→TF5 유지)"""
        if self.robot is None:
            QMessageBox.warning(self, "경고", "로봇이 연결되지 않았습니다.")
            return

        # 버튼 클릭 시 config 다시 읽기
        self._load_detection_pose_config()

        # config에서 XYZ 확인
        if not hasattr(self, '_detection_pose_xyz') or None in self._detection_pose_xyz:
            QMessageBox.warning(self, "경고", "Config에 detection_pose XYZ 값이 없습니다.")
            return

        import time
        from PyQt5.QtWidgets import QApplication

        x, y, z = self._detection_pose_xyz
        tgt_rx = self.spinDetPoseRx.value()
        tgt_ry = self.spinDetPoseRy.value()
        tgt_rz = self.spinDetPoseRz.value()

        reply = QMessageBox.question(
            self, "확인",
            f"로봇을 다음 위치로 이동합니다:\n"
            f"X={x:.1f} Y={y:.1f} Z={z:.1f}\n"
            f"Rx={tgt_rx:.1f} Ry={tgt_ry:.1f} Rz={tgt_rz:.1f}\n\n"
            f"진행하시겠습니까?",
            QMessageBox.Yes | QMessageBox.No
        )
        if reply != QMessageBox.Yes:
            return

        try:
            # 1) TF5 전환
            self._log("Set Robot Position: TF5으로 전환")
            success, msg = self.robot.send_set_toolframe(5, wait=True)
            if not success:
                self._log(f"TF5 전환 실패: {msg}")
                return
            time.sleep(0.5)

            # 2) 절대 좌표 이동 (config XYZ + UI RxRyRz)
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

            # 3) 완료 대기
            success, msg = self.robot.wait_for_done_motion_aware(
                process_events_callback=QApplication.processEvents
            )
            if not success:
                self._log(f"Set Robot Position 이동 실패: {msg}")
                return

            self._log("Set Robot Position 이동 완료")
            time.sleep(0.5)

            # 4) TF5 유지 확인
            success, msg = self.robot.send_set_toolframe(5, wait=True)
            if success:
                self._log("TF5 확인 완료")
            else:
                self._log(f"TF5 설정 실패: {msg}")

        except Exception as e:
            QMessageBox.warning(self, "오류", f"Set Robot Position 실패: {e}")

    def _update_robot_pose_ui(self):
        """현재 로봇 포즈 UI 업데이트"""
        if not hasattr(self, 'labelRobotPosition'):
            return

        if self.robot is None:
            self.labelRobotPosition.setText("로봇 미연결")
            self.labelRobotOrientation.setText("-")
            return

        try:
            pose = self.robot.read_camera_pose()
            if pose is not None:
                x, y, z, rx, ry, rz = pose
                self.labelRobotPosition.setText(
                    f"X={x:.2f}, Y={y:.2f}, Z={z:.2f} mm"
                )
                self.labelRobotOrientation.setText(
                    f"Rx={rx:.2f}, Ry={ry:.2f}, Rz={rz:.2f}°"
                )
            else:
                self.labelRobotPosition.setText("읽기 실패")
                self.labelRobotOrientation.setText("-")
        except Exception as e:
            self.labelRobotPosition.setText(f"오류: {e}")
            self.labelRobotOrientation.setText("-")

    def _update_statistics_ui_m0(self, x_mean=None, y_mean=None, z_mean=None,
                               x_std=None, y_std=None, z_std=None,
                               rx_mean=None, ry_mean=None, rz_mean=None,
                               rx_std=None, ry_std=None, rz_std=None,
                               detection_rate=None,
                               x_mean_raw=None, x_std_raw=None,
                               y_mean_raw=None, y_std_raw=None,
                               z_mean_raw=None, z_std_raw=None,
                               rx_mean_raw=None, rx_std_raw=None,
                               ry_mean_raw=None, ry_std_raw=None,
                               rz_mean_raw=None, rz_std_raw=None):
        """통계 UI 업데이트 (마커1) - filtered / raw(outlier) 형식"""
        if x_mean is None:
            # 초기화
            self.labelXMeanValue.setText("-")
            self.labelYMeanValue.setText("-")
            self.labelZMeanValue.setText("-")
            self.labelRxMeanValue.setText("-")
            self.labelRyMeanValue.setText("-")
            self.labelRzMeanValue.setText("-")
            self.labelDetectionRateValue.setText("-")
        else:
            # 값 설정: filtered / raw(outlier) 형식
            # 색상: filtered mean=#4CAF50(녹색), filtered std=#FF9800, raw=#9E9E9E (회색)
            self.labelXMeanValue.setText(
                f'<span style="color:#2196F3">{x_mean:.2f}</span> <span style="color:#FF9800">± {x_std:.2f}</span> '
                f'<span style="color:#9E9E9E">/ {x_mean_raw:.2f} ± {x_std_raw:.2f}</span> mm')
            self.labelYMeanValue.setText(
                f'<span style="color:#2196F3">{y_mean:.2f}</span> <span style="color:#FF9800">± {y_std:.2f}</span> '
                f'<span style="color:#9E9E9E">/ {y_mean_raw:.2f} ± {y_std_raw:.2f}</span> mm')
            self.labelZMeanValue.setText(
                f'<span style="color:#2196F3">{z_mean:.2f}</span> <span style="color:#FF9800">± {z_std:.2f}</span> '
                f'<span style="color:#9E9E9E">/ {z_mean_raw:.2f} ± {z_std_raw:.2f}</span> mm')
            # 회전: 강조색 (녹색 #4CAF50)
            self.labelRxMeanValue.setText(
                f'<span style="color:#4CAF50">{rx_mean:.2f}</span> <span style="color:#FF9800">± {rx_std:.2f}</span> '
                f'<span style="color:#9E9E9E">/ {rx_mean_raw:.2f} ± {rx_std_raw:.2f}</span>°')
            self.labelRyMeanValue.setText(
                f'<span style="color:#4CAF50">{ry_mean:.2f}</span> <span style="color:#FF9800">± {ry_std:.2f}</span> '
                f'<span style="color:#9E9E9E">/ {ry_mean_raw:.2f} ± {ry_std_raw:.2f}</span>°')
            self.labelRzMeanValue.setText(
                f'<span style="color:#4CAF50">{rz_mean:.2f}</span> <span style="color:#FF9800">± {rz_std:.2f}</span> '
                f'<span style="color:#9E9E9E">/ {rz_mean_raw:.2f} ± {rz_std_raw:.2f}</span>°')
            self.labelDetectionRateValue.setText(f"{detection_rate:.1f}%")

    def _update_statistics_ui_m1(self, x_mean=None, y_mean=None, z_mean=None,
                                  x_std=None, y_std=None, z_std=None,
                                  rx_mean=None, ry_mean=None, rz_mean=None,
                                  rx_std=None, ry_std=None, rz_std=None,
                                  x_mean_raw=None, x_std_raw=None,
                                  y_mean_raw=None, y_std_raw=None,
                                  z_mean_raw=None, z_std_raw=None,
                                  rx_mean_raw=None, rx_std_raw=None,
                                  ry_mean_raw=None, ry_std_raw=None,
                                  rz_mean_raw=None, rz_std_raw=None):
        """통계 UI 업데이트 (마커2) - filtered / raw(outlier) 형식"""
        if x_mean is None:
            # 초기화
            self.labelXMeanValueM2.setText("-")
            self.labelYMeanValueM2.setText("-")
            self.labelZMeanValueM2.setText("-")
            self.labelRxMeanValueM2.setText("-")
            self.labelRyMeanValueM2.setText("-")
            self.labelRzMeanValueM2.setText("-")
        else:
            # 값 설정: filtered / raw(outlier) 형식
            self.labelXMeanValueM2.setText(
                f'<span style="color:#2196F3">{x_mean:.2f}</span> <span style="color:#FF9800">± {x_std:.2f}</span> '
                f'<span style="color:#9E9E9E">/ {x_mean_raw:.2f} ± {x_std_raw:.2f}</span> mm')
            self.labelYMeanValueM2.setText(
                f'<span style="color:#2196F3">{y_mean:.2f}</span> <span style="color:#FF9800">± {y_std:.2f}</span> '
                f'<span style="color:#9E9E9E">/ {y_mean_raw:.2f} ± {y_std_raw:.2f}</span> mm')
            self.labelZMeanValueM2.setText(
                f'<span style="color:#2196F3">{z_mean:.2f}</span> <span style="color:#FF9800">± {z_std:.2f}</span> '
                f'<span style="color:#9E9E9E">/ {z_mean_raw:.2f} ± {z_std_raw:.2f}</span> mm')
            # 회전: 강조색 (녹색 #4CAF50)
            self.labelRxMeanValueM2.setText(
                f'<span style="color:#4CAF50">{rx_mean:.2f}</span> <span style="color:#FF9800">± {rx_std:.2f}</span> '
                f'<span style="color:#9E9E9E">/ {rx_mean_raw:.2f} ± {rx_std_raw:.2f}</span>°')
            self.labelRyMeanValueM2.setText(
                f'<span style="color:#4CAF50">{ry_mean:.2f}</span> <span style="color:#FF9800">± {ry_std:.2f}</span> '
                f'<span style="color:#9E9E9E">/ {ry_mean_raw:.2f} ± {ry_std_raw:.2f}</span>°')
            self.labelRzMeanValueM2.setText(
                f'<span style="color:#4CAF50">{rz_mean:.2f}</span> <span style="color:#FF9800">± {rz_std:.2f}</span> '
                f'<span style="color:#9E9E9E">/ {rz_mean_raw:.2f} ± {rz_std_raw:.2f}</span>°')

    def _on_graph_marker_changed(self):
        """그래프 마커 선택 변경 시 그래프 다시 그리기"""
        self._plot_graphs()

    def _draw_distribution(self, figure, canvas, tvec_array_mm, marker_label):
        """scatter + histogram 2x3 서브플롯을 주어진 figure에 그림"""
        figure.clear()

        ax1 = figure.add_subplot(2, 3, 1)
        ax2 = figure.add_subplot(2, 3, 2)
        ax3 = figure.add_subplot(2, 3, 3)
        ax4 = figure.add_subplot(2, 3, 4)
        ax5 = figure.add_subplot(2, 3, 5)
        ax6 = figure.add_subplot(2, 3, 6)

        labels = ['X', 'Y', 'Z']
        scatter_axes = [ax1, ax2, ax3]
        hist_axes = [ax4, ax5, ax6]

        for i, (sax, hax, label) in enumerate(zip(scatter_axes, hist_axes, labels)):
            sax.scatter(range(len(tvec_array_mm)), tvec_array_mm[:, i], alpha=0.5, s=10)
            sax.axhline(np.mean(tvec_array_mm[:, i]), color='r', linestyle='--', linewidth=1)
            sax.set_ylabel(f'{label} (mm)')
            sax.set_title(f'{label} Position ({marker_label})')
            sax.grid(True, alpha=0.3)

            hax.hist(tvec_array_mm[:, i], bins=20, alpha=0.7, edgecolor='black')
            hax.set_xlabel(f'{label} (mm)')
            hax.set_ylabel('Count')
            hax.grid(True, alpha=0.3)

        figure.tight_layout()
        canvas.draw()

    def _plot_graphs(self):
        """분포 그래프 그리기 (Raw Data + Outlier 제거)"""
        if not self.collected_data:
            self._clear_graphs()
            return

        # 선택된 마커 데이터 추출
        selected_marker = self.graph_marker_button_group.checkedId()
        if selected_marker == 0:
            detected_data = [d for d in self.collected_data if d['detected'] and d['tvec'] is not None]
            marker_label = f"ID {self.spinTagID1.value()}"
        else:
            detected_data = [d for d in self.collected_data if d.get('detected2') and d.get('tvec2') is not None]
            marker_label = f"ID {self.spinTagID2.value()}"

        if not detected_data:
            self._clear_graphs()
            return

        # tvec + euler 추출 (aligned arrays)
        key_suffix = '' if selected_marker == 0 else '2'
        tvec_array, euler_array = self._extract_aligned_arrays(detected_data, key_suffix)
        tvec_array_mm = tvec_array * 1000.0

        # === Raw Data 탭 ===
        self._draw_distribution(self.figure_raw, self.canvas_raw, tvec_array_mm, marker_label)

        # === Outlier 제거 탭 ===
        sigma = self.spinSigmaMultiplier.value()
        min_samples = self.spinMinSamples.value()
        filtered_tvec, filtered_euler, valid_mask, removal_info = self._remove_outliers(
            tvec_array, euler_array, sigma=sigma, min_samples=min_samples)
        filtered_tvec_mm = filtered_tvec * 1000.0
        removed_count = removal_info['total_removed']

        if len(filtered_tvec_mm) > 0:
            self._draw_distribution(self.figure_filtered, self.canvas_filtered,
                                    filtered_tvec_mm, f"{marker_label} (filtered)")
        else:
            self.figure_filtered.clear()
            self.canvas_filtered.draw()

        # 탭 이름에 제거 수 표시
        self.graph_tab_widget.setTabText(1, f"Outlier 제거 ({removed_count}개 제거)")

    def _clear_graphs(self):
        """그래프 초기화"""
        self.figure_raw.clear()
        self.canvas_raw.draw()
        self.figure_filtered.clear()
        self.canvas_filtered.draw()
        self.graph_tab_widget.setTabText(1, "Outlier 제거")

    def _rvec_to_euler(self, rvec):
        """Rotation vector를 Euler 각도로 변환 (Rx, Ry, Rz in degrees)"""
        return rvec_to_euler_deg(rvec)


    def _log(self, message):
        """로그 출력"""
        self.textLog.appendPlainText(message)
        print(f"[ArUco Reliability] {message}")
