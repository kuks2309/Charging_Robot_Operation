#!/usr/bin/env python3
"""
카메라 캘리브레이션 탭
Chessboard 패턴을 사용한 카메라 내부 파라미터 캘리브레이션
"""

import os
import cv2
import numpy as np
from datetime import datetime
from PyQt5 import uic
from PyQt5.QtWidgets import QWidget, QFileDialog, QMessageBox, QApplication, QButtonGroup
from PyQt5.QtCore import Qt, pyqtSignal
from PyQt5.QtGui import QPixmap, QImage

# 공통 유틸리티
from utils.common import (
    display_frame_on_label,
    save_snapshot,
    require_robot_connection,
    require_camera_running,
    Messages,
)
from utils.chessboard_detector import ChessboardDetector
from utils.overlay import draw_crosshair, draw_center_marker
from utils.camera_calib_position_generator import (
    generate_planar_positions_vision_tf,
    generate_base_positions_with_rotation,
    format_position_label,
    format_position_label_base,
)
from services.chessboard_alignment_service import ChessboardAlignmentService
from services.camera_calibration_service import CameraCalibrationService
from utils.image_processing import undistort_frame


# UI 파일 경로
UI_DIR = os.path.join(os.path.dirname(__file__), '..', '..', 'ui')
TAB_CALIBRATION_UI = os.path.join(UI_DIR, 'tab_calibration.ui')


class TabCalibration(QWidget):
    """카메라 캘리브레이션 탭 클래스"""

    # 시그널 정의
    log_message = pyqtSignal(str)
    camera_start_requested = pyqtSignal()
    camera_stop_requested = pyqtSignal()

    # 자동 캘리브레이션 시그널
    auto_calib_start_requested = pyqtSignal(int)  # num_positions
    auto_calib_stop_requested = pyqtSignal()

    def __init__(self, parent=None):
        super().__init__(parent)

        # UI 로드
        uic.loadUi(TAB_CALIBRATION_UI, self)

        # 로봇 참조
        self.robot = None

        # 캘리브레이션 데이터
        self.calib_images = []  # 캘리브레이션용 이미지들
        self.calib_points_3d = []  # 3D 객체 점들 (chessboard corners)
        self.calib_points_2d = []  # 2D 이미지 점들 (detected corners)

        # 캘리브레이션 결과
        self.camera_matrix = None
        self.dist_coeffs = None
        self.rms_error = None

        # 현재 프레임 (캡처용)
        self.current_frame = None
        self.current_depth_frame = None  # Depth 프레임

        # 카메라 매니저 참조 (depth 프레임용)
        self.camera_manager = None

        # 체스보드 감지기
        self.chessboard_detector = ChessboardDetector(cols=10, rows=7, square_size=22.0)

        # 자동 캘리브레이션 상태
        self.auto_calib_running = False
        self.auto_calib_positions = []  # 생성된 위치 리스트 [(dx, dy, dz), ...]
        self.auto_calib_base_pos = None  # 기준 좌표 (X, Y, Z) - 체스보드 중심 정렬 시 저장

        # 중심 정렬 시 저장된 체스보드 거리 (mm)
        self.aligned_distance = None
        # 중심 정렬 시 저장된 Rx, Ry, Rz 각도 (deg)
        self.aligned_rx = None
        self.aligned_ry = None
        self.aligned_rz = None
        # 중심 정렬 시 저장된 기준 X, Y, Z 좌표 (mm)
        self.aligned_x = None
        self.aligned_y = None
        self.aligned_z = None

        # 체스보드 정렬 서비스
        self.alignment_service = ChessboardAlignmentService()
        self.alignment_service.set_log_callback(self._log)

        # 카메라 캘리브레이션 서비스
        self._calib_service = CameraCalibrationService()

        # 스텝 버튼 그룹 생성
        self.calib_step_button_group = QButtonGroup(self)
        self.calib_step_button_group.addButton(self.radioCalibStep1, 1)
        self.calib_step_button_group.addButton(self.radioCalibStep5, 5)
        self.calib_step_button_group.addButton(self.radioCalibStep10, 10)

        # 카메라 선택 버튼 그룹 (DS435/ArduCam)
        self.camera_select_button_group = QButtonGroup(self)
        self.camera_select_button_group.addButton(self.radioDS435, 0)
        self.camera_select_button_group.addButton(self.radioArduCam, 1)

        # 이미지 타입 버튼 그룹 (Color/Depth)
        self.image_type_button_group = QButtonGroup(self)
        self.image_type_button_group.addButton(self.radioColorImage, 0)
        self.image_type_button_group.addButton(self.radioDepthImage, 1)

        # 시그널 연결
        self._connect_signals()

        # 초기화
        self._init_ui()

    def _connect_signals(self):
        """내부 시그널-슬롯 연결"""
        # 카메라 버튼
        self.btnCalibStartCamera.clicked.connect(self._on_start_camera)
        self.btnCalibStopCamera.clicked.connect(self._on_stop_camera)
        self.btnCalibSnapshot.clicked.connect(self._on_snapshot)

        # 체스보드 포즈 버튼
        self.btnFindChessboardPose.clicked.connect(self._on_find_chessboard_pose)

        # 캘리브레이션 버튼
        self.btnCaptureCalibImage.clicked.connect(self._on_capture_calib_image)
        self.btnClearCalibImages.clicked.connect(self._on_clear_calib_images)
        self.btnRunCalibration.clicked.connect(self._on_run_calibration)
        self.btnLoadCalibration.clicked.connect(self._on_load_calibration)

        # 체스보드 로봇 정렬 버튼
        self.btnAlignCenter.clicked.connect(self._on_align_center)

        # 로봇 이동 버튼 (베이스 좌표계)
        self.btnCalibXMinus.clicked.connect(lambda: self._on_calib_base_move('x', -1))
        self.btnCalibXPlus.clicked.connect(lambda: self._on_calib_base_move('x', 1))
        self.btnCalibYMinus.clicked.connect(lambda: self._on_calib_base_move('y', -1))
        self.btnCalibYPlus.clicked.connect(lambda: self._on_calib_base_move('y', 1))
        self.btnCalibZMinus.clicked.connect(lambda: self._on_calib_base_move('z', -1))
        self.btnCalibZPlus.clicked.connect(lambda: self._on_calib_base_move('z', 1))
        self.btnCalibRxMinus.clicked.connect(lambda: self._on_calib_base_rotate('rx', -1))
        self.btnCalibRxPlus.clicked.connect(lambda: self._on_calib_base_rotate('rx', 1))
        self.btnCalibRyMinus.clicked.connect(lambda: self._on_calib_base_rotate('ry', -1))
        self.btnCalibRyPlus.clicked.connect(lambda: self._on_calib_base_rotate('ry', 1))
        self.btnCalibRzMinus.clicked.connect(lambda: self._on_calib_base_rotate('rz', -1))
        self.btnCalibRzPlus.clicked.connect(lambda: self._on_calib_base_rotate('rz', 1))

        # TCP 정렬 버튼
        self.btnTcpAlign.clicked.connect(self._on_tcp_align)
        self.btnTcpAlignRead.clicked.connect(self._on_tcp_align_read)
        self.btnTcpAlignCenter.clicked.connect(self._on_tcp_align_center)

        # Rx 프리셋 버튼 (카메라 캘리브레이션용)
        self.btnRxPreset70.clicked.connect(lambda: self._on_rx_preset(70))
        self.btnRxPreset80.clicked.connect(lambda: self._on_rx_preset(80))
        self.btnRxPreset90.clicked.connect(lambda: self._on_rx_preset(90))
        self.btnRxPreset100.clicked.connect(lambda: self._on_rx_preset(100))
        self.btnRxPreset110.clicked.connect(lambda: self._on_rx_preset(110))

        # Ry 프리셋 버튼 (카메라 캘리브레이션용)
        self.btnRyPresetM20.clicked.connect(lambda: self._on_ry_preset(-20))
        self.btnRyPresetM10.clicked.connect(lambda: self._on_ry_preset(-10))
        self.btnRyPreset0.clicked.connect(lambda: self._on_ry_preset(0))
        self.btnRyPreset10.clicked.connect(lambda: self._on_ry_preset(10))
        self.btnRyPreset20.clicked.connect(lambda: self._on_ry_preset(20))

        # Rz 프리셋 버튼 (카메라 캘리브레이션용)
        self.btnRzPreset70.clicked.connect(lambda: self._on_rz_preset(70))
        self.btnRzPreset80.clicked.connect(lambda: self._on_rz_preset(80))
        self.btnRzPreset90.clicked.connect(lambda: self._on_rz_preset(90))
        self.btnRzPreset100.clicked.connect(lambda: self._on_rz_preset(100))
        self.btnRzPreset110.clicked.connect(lambda: self._on_rz_preset(110))

        # 자동 캘리브레이션 위치 생성 버튼
        self.btnGeneratePositions.clicked.connect(self._on_generate_positions)
        self.btnClearPositions.clicked.connect(self._on_clear_positions)
        self.btnMoveToSelectedBase.clicked.connect(self._on_move_to_selected_base)
        self.btnSequentialMove.clicked.connect(self._on_sequential_move)
        self.btnRunAutoCapture.clicked.connect(self._on_run_auto_capture)
        self.btnStopAutoCapture.clicked.connect(self._on_stop_auto_capture)

        # 서브탭 변경 시그널
        self.tabWidgetCalibSub.currentChanged.connect(self._on_calib_sub_tab_changed)

    def _on_calib_sub_tab_changed(self, index: int):
        """캘리브레이션 서브탭 변경 시 호출"""
        tab_names = {0: "Chessboard", 1: "자동 캘리브레이션"}
        tab_name = tab_names.get(index, f"Tab{index}")
        print(f"[TabCalibration] Sub-tab changed: {index} - {tab_name}")

        # 로봇 연결 시 TF1 설정
        if self.robot and self.robot.is_connected:
            try:
                success, msg = self.robot.send_set_toolframe(1, wait=True)
                print(f"[TabCalibration] Set TF1: success={success}, msg={msg}")

                # 실제 TF 값 읽기
                toolframe = self.robot.read_current_toolframe()
                print(f"[TabCalibration] Read TF from robot: {toolframe}")

                if toolframe is not None:
                    self.update_current_toolframe(toolframe)
            except Exception as e:
                print(f"[TabCalibration] TF setting error: {e}")

    def _init_ui(self):
        """UI 초기화"""
        self._update_captured_count()
        self._clear_results()
        self._clear_chessboard_pose()
        # 중심 정렬 자동 보정 체크박스 초기 비활성화
        self.checkAutoCorrection.setEnabled(False)
        self.checkAutoCorrection.setChecked(False)

    def _log(self, message: str):
        """로그 메시지 출력"""
        self.log_message.emit(message)
        print(f"[Calibration] {message}")

    def get_chessboard_size(self) -> tuple:
        """체스보드 크기 (cols, rows) 반환 및 감지기 업데이트"""
        cols = self.spinChessboardCols.value()
        rows = self.spinChessboardRows.value()
        self.chessboard_detector.set_board_size(cols, rows)
        return (cols, rows)

    def get_square_size(self) -> float:
        """체스보드 정사각형 크기 (mm) 반환 및 감지기 업데이트"""
        size = self.spinSquareSize.value()
        self.chessboard_detector.set_square_size(size)
        return size

    def is_chessboard_detect_enabled(self) -> bool:
        """체스보드 감지 활성화 여부"""
        return self.checkChessboardDetect.isChecked()

    def is_depth_mode(self) -> bool:
        """Depth 이미지 모드 여부"""
        return self.radioDepthImage.isChecked()

    def set_camera_manager(self, camera_manager):
        """카메라 매니저 설정"""
        self.camera_manager = camera_manager

    def set_current_frame(self, frame: np.ndarray):
        """현재 프레임 설정 (외부에서 호출)"""
        self.current_frame = frame.copy() if frame is not None else None
        # Depth 프레임도 함께 가져오기
        if self.camera_manager:
            depth_frame = self.camera_manager.get_depth_frame()
            self.current_depth_frame = depth_frame.copy() if depth_frame is not None else None

    def process_frame(self, frame: np.ndarray) -> np.ndarray:
        """프레임 처리 - 체스보드 감지 및 표시"""
        if not self.is_chessboard_detect_enabled():
            return frame

        # 체스보드 설정 업데이트
        self.get_chessboard_size()
        self.get_square_size()

        # 체스보드 감지 (ChessboardDetector 사용)
        result = self.chessboard_detector.detect(frame)

        if result is not None:
            corners, center, angle = result

            # 코너 그리기
            frame = self.chessboard_detector.draw_corners(frame, corners)

            # 중심 및 이미지 중심 표시
            frame = self.chessboard_detector.draw_pose(frame, center)

            # 상태 텍스트
            cx, cy = int(center[0]), int(center[1])
            cv2.putText(frame, "Chessboard Detected", (10, 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            cv2.putText(frame, f"Center: ({cx}, {cy})", (10, 60),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            cv2.putText(frame, f"Angle: {angle:.1f} deg", (10, 85),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

            # 체스보드 중심 거리 측정 및 표시 (Color→Depth 좌표 변환)
            if self.camera_manager:
                distance = self.camera_manager.get_distance_at(cx, cy, from_color=True)
                if distance is not None and distance > 0:
                    cv2.putText(frame, f"Distance: {distance:.0f} mm", (10, 110),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 255), 2)
                    self._update_distance_display(distance)
                else:
                    cv2.putText(frame, "Distance: N/A", (10, 110),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.6, (128, 128, 128), 2)
                    self._update_distance_display(None)

            # UI 라벨 업데이트 (중심, 각도)
            self.labelChessboardCenterValue.setText(f"({cx}, {cy})")
            self.labelChessboardAngleValue.setText(f"{angle:.1f}°")
        else:
            cv2.putText(frame, "Chessboard Not Found", (10, 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
            # UI 라벨 초기화
            self.labelChessboardCenterValue.setText("-")
            self.labelChessboardAngleValue.setText("-")
            self._update_distance_display(None)

        return frame

    @property
    def last_chessboard_result(self):
        """체스보드 감지 결과 (호환성 유지)"""
        if self.chessboard_detector.last_center is None:
            return None
        return (self.chessboard_detector.last_center, self.chessboard_detector.last_angle)

    def _update_distance_display(self, distance: float):
        """UI에 거리 표시 업데이트"""
        if distance is not None and distance > 0:
            self.labelChessboardDistanceValue.setText(f"{distance:.0f} mm")
        else:
            self.labelChessboardDistanceValue.setText("-")

    def display_frame(self, frame: np.ndarray):
        """프레임을 QLabel에 표시"""
        if frame is None:
            return

        # Depth 모드일 때 중앙 거리 표시
        display_frame = frame.copy()
        if self.is_depth_mode() and self.camera_manager:
            distance = self.camera_manager.get_distance_at_center()
            h, w = display_frame.shape[:2]
            cx, cy = w // 2, h // 2

            # 중심점 십자선 표시
            draw_crosshair(display_frame, cx, cy, (255, 255, 255), thickness=2, full_frame=False, arm_length=20)
            cv2.circle(display_frame, (cx, cy), 5, (0, 255, 255), -1)

            # 거리 텍스트 표시
            if distance is not None and distance > 0:
                text = f"Distance: {distance:.0f} mm ({distance/1000:.3f} m)"
                cv2.putText(display_frame, text, (10, 30),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
                cv2.putText(display_frame, text, (10, 30),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 1)
            else:
                cv2.putText(display_frame, "Distance: N/A", (10, 30),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)

        # 좌표축 표시
        self._draw_coordinate_axes(display_frame)

        # 해상도 표시 (우상단)
        h, w = display_frame.shape[:2]
        resolution_text = f"{w}x{h}"
        cv2.putText(display_frame, resolution_text, (w - 100, 25),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        cv2.putText(display_frame, resolution_text, (w - 100, 25),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 1)

        # 공통 유틸 사용
        display_frame_on_label(display_frame, self.labelCalibCameraView)

    def _draw_coordinate_axes(self, frame: np.ndarray):
        """
        화면에 좌표축 표시
        - 좌하단: 이미지 좌표계 (u: 빨강 →, v: 초록 ↓)
        - 우하단: Vision 좌표계 (X: 빨강 →, Y: 초록 ↑)
        """
        h, w = frame.shape[:2]
        axis_length = 50  # 축 길이 (픽셀)
        margin = 20       # 가장자리 여백
        arrow_tip = 8     # 화살표 팁 크기

        # ===== 좌하단: 이미지 좌표계 (u, v) =====
        # OpenCV 이미지 좌표계: 원점이 좌상단, u→오른쪽, v↓아래쪽
        img_origin_x = margin + 10
        img_origin_y = h - margin - axis_length - 10  # 하단에서 위로

        # u축 (빨강, 오른쪽 →)
        cv2.arrowedLine(frame,
                        (img_origin_x, img_origin_y),
                        (img_origin_x + axis_length, img_origin_y),
                        (0, 0, 255), 2, tipLength=0.2)
        cv2.putText(frame, "u", (img_origin_x + axis_length + 5, img_origin_y + 5),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

        # v축 (초록, 아래쪽 ↓) - OpenCV 좌표계는 아래가 +
        cv2.arrowedLine(frame,
                        (img_origin_x, img_origin_y),
                        (img_origin_x, img_origin_y + axis_length),
                        (0, 255, 0), 2, tipLength=0.2)
        cv2.putText(frame, "v", (img_origin_x - 15, img_origin_y + axis_length + 5),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        # "Image" 라벨
        cv2.putText(frame, "Image", (img_origin_x - 5, img_origin_y + axis_length + 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)

        # ===== 우하단: Vision 좌표계 (X, Y) =====
        # 원점 위치
        vision_origin_x = w - margin - axis_length - 20
        vision_origin_y = h - margin - 10

        # X축 (빨강, 오른쪽 →)
        cv2.arrowedLine(frame,
                        (vision_origin_x, vision_origin_y),
                        (vision_origin_x + axis_length, vision_origin_y),
                        (0, 0, 255), 2, tipLength=0.2)
        cv2.putText(frame, "X", (vision_origin_x + axis_length + 5, vision_origin_y + 5),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

        # Y축 (초록, 위쪽 ↑) - Vision 좌표계는 위가 +
        cv2.arrowedLine(frame,
                        (vision_origin_x, vision_origin_y),
                        (vision_origin_x, vision_origin_y - axis_length),
                        (0, 255, 0), 2, tipLength=0.2)
        cv2.putText(frame, "Y", (vision_origin_x - 5, vision_origin_y - axis_length - 5),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        # "Vision" 라벨
        cv2.putText(frame, "Vision", (vision_origin_x - 5, vision_origin_y + 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)

    # ==================== 카메라 버튼 핸들러 ====================

    def _on_start_camera(self):
        """카메라 시작"""
        self.camera_start_requested.emit()

    def _on_stop_camera(self):
        """카메라 정지"""
        self.camera_stop_requested.emit()

    @require_camera_running
    def _on_snapshot(self):
        """스냅샷 저장"""
        save_snapshot(self.current_frame, self, "snapshot_calib", self._log)

    # ==================== 체스보드 포즈 ====================

    @require_camera_running
    def _on_find_chessboard_pose(self):
        """체스보드 포즈 찾기 (중심 좌표 및 회전 각도)"""
        # 체스보드 설정 업데이트
        self.get_chessboard_size()

        result = self.chessboard_detector.detect(self.current_frame)

        if result is None:
            QMessageBox.warning(self, "경고", Messages.CHESSBOARD_NOT_DETECTED)
            self._clear_chessboard_pose()
            return

        corners, center, angle = result
        self._update_chessboard_pose(center, angle)
        self._log(f"체스보드 포즈 - 중심: ({center[0]:.1f}, {center[1]:.1f}), 각도: {angle:.2f}°")

    def find_chessboard_pose(self, frame: np.ndarray) -> tuple:
        """
        체스보드의 이미지 중심 좌표와 회전 각도 계산 (ChessboardDetector 사용)

        Args:
            frame: 입력 이미지

        Returns:
            ((cx, cy), angle) 또는 None (감지 실패 시)
        """
        self.get_chessboard_size()
        result = self.chessboard_detector.detect(frame)

        if result is None:
            return None

        corners, center, angle = result
        return (center, angle)

    def _update_chessboard_pose(self, center: tuple, angle: float):
        """체스보드 포즈 UI 업데이트"""
        self.labelChessboardCenterValue.setText(f"({center[0]:.1f}, {center[1]:.1f})")
        self.labelChessboardAngleValue.setText(f"{angle:.2f}°")

    def _clear_chessboard_pose(self):
        """체스보드 포즈 UI 초기화"""
        self.labelChessboardCenterValue.setText("-")
        self.labelChessboardAngleValue.setText("-")

    # ==================== 캘리브레이션 핸들러 ====================

    @require_camera_running
    def _on_capture_calib_image(self):
        """캘리브레이션 이미지 캡처"""
        frame = self.current_frame.copy()

        # 체스보드 설정 업데이트 및 감지
        self.get_chessboard_size()
        self.get_square_size()

        result = self.chessboard_detector.detect(frame)

        if result is None:
            QMessageBox.warning(self, "경고", Messages.CHESSBOARD_NOT_DETECTED)
            return

        corners, center, angle = result

        # 3D 객체 점 생성 (ChessboardDetector 사용)
        objp = self.chessboard_detector.get_object_points()

        # 데이터 저장
        self.calib_images.append(frame)
        self.calib_points_3d.append(objp)
        self.calib_points_2d.append(corners)

        self._update_captured_count()
        self._log(f"캘리브레이션 이미지 캡처 완료 ({len(self.calib_images)}개)")

    def _on_clear_calib_images(self):
        """캡처된 이미지 초기화"""
        reply = QMessageBox.question(
            self, "초기화 확인",
            "캡처된 모든 이미지를 삭제하시겠습니까?",
            QMessageBox.Yes | QMessageBox.No,
            QMessageBox.No
        )

        if reply == QMessageBox.Yes:
            self.calib_images.clear()
            self.calib_points_3d.clear()
            self.calib_points_2d.clear()
            self._update_captured_count()
            self._clear_results()
            self._log("캘리브레이션 이미지 초기화")

    def _on_run_calibration(self):
        """캘리브레이션 실행"""
        if len(self.calib_images) < 3:
            QMessageBox.warning(self, "경고", "최소 3개 이상의 이미지가 필요합니다.")
            return

        self._log("캘리브레이션 실행 중...")

        # 이미지 크기
        img_shape = self.calib_images[0].shape[:2][::-1]  # (width, height)

        # 캘리브레이션 실행
        result = self._calib_service.run_calibration(
            self.calib_points_3d,
            self.calib_points_2d,
            img_shape,
        )

        if not result['success']:
            QMessageBox.critical(self, "오류", "캘리브레이션에 실패했습니다.")
            return

        # 결과 저장
        self.camera_matrix = result['camera_matrix']
        self.dist_coeffs = result['dist_coeffs']
        self.rms_error = result['rms_error']

        # UI 업데이트
        self._update_results()

        self._log(f"캘리브레이션 완료! RMS 오차: {ret:.4f}")

    def _on_load_calibration(self):
        """캘리브레이션 결과 로드"""
        filepath, _ = QFileDialog.getOpenFileName(
            self, "캘리브레이션 로드", "", "NumPy 파일 (*.npz *.npy);;All Files (*)"
        )

        if not filepath:
            return

        try:
            data = np.load(filepath)
            self.camera_matrix = data['camera_matrix']
            self.dist_coeffs = data['dist_coeffs']
            self.rms_error = float(data.get('rms_error', 0))

            self._update_results()
            self._log(f"캘리브레이션 로드: {filepath}")
            QMessageBox.information(self, "완료", "캘리브레이션이 로드되었습니다.")

        except Exception as e:
            QMessageBox.critical(self, "오류", f"로드 실패: {str(e)}")
            self._log(f"캘리브레이션 로드 실패: {e}")

    # ==================== UI 업데이트 ====================

    def _update_captured_count(self):
        """캡처된 이미지 수 업데이트"""
        count = len(self.calib_images)
        self.labelCapturedCount.setText(str(count))

    def _update_results(self):
        """캘리브레이션 결과 로그 출력"""
        if self.camera_matrix is None:
            return

        # 카메라 매트릭스 값
        fx = self.camera_matrix[0, 0]
        fy = self.camera_matrix[1, 1]
        cx = self.camera_matrix[0, 2]
        cy = self.camera_matrix[1, 2]

        self._log(f"캘리브레이션 결과: fx={fx:.2f}, fy={fy:.2f}, cx={cx:.2f}, cy={cy:.2f}")
        if self.rms_error is not None:
            self._log(f"RMS 오차: {self.rms_error:.4f}")

    def _clear_results(self):
        """결과 초기화 (UI 라벨 삭제됨 - 로그만 출력)"""
        pass

    # ==================== 외부 인터페이스 ====================

    def get_calibration_data(self) -> dict:
        """캘리브레이션 데이터 반환"""
        return {
            'camera_matrix': self.camera_matrix,
            'dist_coeffs': self.dist_coeffs,
            'rms_error': self.rms_error
        }

    def set_calibration_data(self, data: dict):
        """캘리브레이션 데이터 설정"""
        self.camera_matrix = data.get('camera_matrix')
        self.dist_coeffs = data.get('dist_coeffs')
        self.rms_error = data.get('rms_error')
        self._update_results()

    # ==================== 로봇 인터페이스 ====================

    def set_robot(self, robot):
        """로봇 참조 설정"""
        self.robot = robot
        self.alignment_service.set_robot(robot)

    def update_robot_position(self, x: float, y: float, z: float,
                               rx: float, ry: float, rz: float):
        """로봇 좌표 업데이트 (외부에서 호출)"""
        self.editCalibX.setText(f"{x:.2f}")
        self.editCalibY.setText(f"{y:.2f}")
        self.editCalibZ.setText(f"{z:.2f}")
        self.editCalibRx.setText(f"{rx:.2f}")
        self.editCalibRy.setText(f"{ry:.2f}")
        self.editCalibRz.setText(f"{rz:.2f}")

    def update_current_toolframe(self, toolframe: int):
        """현재 툴프레임 업데이트 (외부에서 호출)"""
        tf_names = {0: "TF0 (Eye-in-Hand)", 1: "TF1 (비전)", 2: "TF2 (충전)", 3: "TF3 (충전)", 4: "TF4", 5: "TF5 (Hand-Eye)"}
        tf_colors = {0: "#666666", 1: "#2196F3", 2: "#FF9800", 3: "#9C27B0", 4: "#4CAF50", 5: "#E91E63"}
        name = tf_names.get(toolframe, f"TF{toolframe}")
        color = tf_colors.get(toolframe, "#000000")
        self.labelCalibToolframeValue.setText(name)
        self.labelCalibToolframeValue.setStyleSheet(f"color: {color}; font-weight: bold;")

    # ==================== 체스보드 로봇 정렬 ====================

    # 픽셀당 mm 변환 비율 (카메라 캘리브레이션 후 조정 필요)
    # 1920x1080 기준, fy 비율(4003.2/5347.3=0.7486) 이론값 — 현장 측정 후 확정
    PIXEL_TO_MM = 0.187  # 0.25 × (fy_old/fy_new) = 0.25 × 0.7486

    @require_robot_connection
    @require_camera_running
    def _on_align_center(self, checked=False):
        """중심 정렬 - 체스보드 중심을 이미지 중심으로 이동 (베이스 프레임 기준)"""

        # 체스보드 감지가 활성화되어 있으면 캐시된 결과 사용
        if self.is_chessboard_detect_enabled() and self.last_chessboard_result is not None:
            result = self.last_chessboard_result
        else:
            # 캐시된 결과가 없으면 새로 감지
            result = self.find_chessboard_pose(self.current_frame.copy())

        # 결과가 있으면 화면에 표시
        if result is not None:
            frame_copy = self.current_frame.copy()
            center, angle = result
            cx, cy = int(center[0]), int(center[1])

            # 체스보드 중심점 표시 (빨간색 - 화면 전체 라인)
            h, w = frame_copy.shape[:2]
            draw_center_marker(frame_copy, cx, cy, (0, 0, 255), radius=8, thickness=-1)

            # 이미지 중심점 표시 (파란색 - 화면 전체 라인)
            draw_center_marker(frame_copy, w // 2, h // 2, (255, 0, 0), radius=8, thickness=2)

            # 정보 텍스트
            cv2.putText(frame_copy, "Chessboard Detected", (10, 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            cv2.putText(frame_copy, f"Center: ({cx}, {cy})", (10, 60),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            cv2.putText(frame_copy, f"Angle: {angle:.1f} deg", (10, 85),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

            # 화면에 표시
            self.display_frame(frame_copy)

        if result is None:
            QMessageBox.warning(self, "경고", "체스보드를 감지하지 못했습니다.")
            return

        center, angle = result

        # 이미지 중심 좌표
        frame_height, frame_width = self.current_frame.shape[:2]
        image_center_x = frame_width / 2
        image_center_y = frame_height / 2

        # 체스보드 중심과 이미지 중심의 차이 (픽셀)
        dx_pixel = center[0] - image_center_x
        dy_pixel = center[1] - image_center_y

        # 픽셀을 mm로 변환 (둘 다 부호 반대)
        dx_mm = -dx_pixel * self.PIXEL_TO_MM  # 로봇 X 이동량 (카메라 X → 로봇 X, 반대)
        dy_mm = -dy_pixel * self.PIXEL_TO_MM  # 로봇 Y 이동량 (카메라 Y → 로봇 Y, 반대)

        self._log(f"중심 오프셋: dx={dx_pixel:.1f}px, dy={dy_pixel:.1f}px")
        self._log(f"로봇 이동량: X={dx_mm:.2f}mm, Y={dy_mm:.2f}mm")

        # 로봇 연결 확인
        if not self.robot or not self.robot.is_connected:
            self._log("로봇 미연결 - 계산 결과만 표시")
            return

        # 이동 전 로봇 좌표 읽기 (현재 위치 - 레지스터 158~169)
        pose_before = self.robot.read_current_pose()
        if pose_before:
            self._log(f"[이동 전] X={pose_before[0]:.1f}, Y={pose_before[1]:.1f}, Z={pose_before[2]:.1f}, "
                     f"Rx={pose_before[3]:.1f}, Ry={pose_before[4]:.1f}, Rz={pose_before[5]:.1f}")
        else:
            self._log("[이동 전] 로봇 좌표 읽기 실패")

        # 툴프레임 1로 변경 (비전 프레임) - wait=False로 빠르게 처리
        import time

        # 툴프레임 1로 변경 (비전 프레임)
        self._log("툴프레임 1 설정 중...")
        self.robot.send_set_toolframe(1, wait=False)
        # 툴프레임 설정 완료 대기 (상태가 0이 될 때까지)
        for _ in range(50):
            time.sleep(0.1)
            status = self.robot.read_status()
            if status == 0:
                break
        time.sleep(0.5)  # 추가 대기
        self._log("툴프레임 1 설정 완료")
        self.update_current_toolframe(1)

        # XY 동시 이동 (툴 좌표계 기준, command 13)
        self._log(f"XY 이동 중: X={dx_mm:.2f}mm, Y={dy_mm:.2f}mm")
        success, msg = self.robot.send_tcp_linear('xyz', (dx_mm, dy_mm, 0), process_events_callback=QApplication.processEvents)
        if not success:
            self._log(f"XY 이동 실패: {msg}")
            return
        self._log(f"XY 이동 완료: {msg}")

        # 이동 완료 후 잠시 대기
        import time
        time.sleep(0.5)

        # 이동 후 로봇 좌표 읽기 (현재 위치 - 레지스터 158~169)
        pose_after = self.robot.read_current_pose()
        if pose_after:
            self._log(f"[이동 후] X={pose_after[0]:.1f}, Y={pose_after[1]:.1f}, Z={pose_after[2]:.1f}, "
                     f"Rx={pose_after[3]:.1f}, Ry={pose_after[4]:.1f}, Rz={pose_after[5]:.1f}")
            # 이동량 계산
            if pose_before:
                dx_actual = pose_after[0] - pose_before[0]
                dy_actual = pose_after[1] - pose_before[1]
                dz_actual = pose_after[2] - pose_before[2]
                self._log(f"[실제 이동량] dX={dx_actual:.1f}mm, dY={dy_actual:.1f}mm, dZ={dz_actual:.1f}mm")
        else:
            self._log("[이동 후] 로봇 좌표 읽기 실패")

        # 중심 정렬 완료 후 카메라 중심 거리 측정 및 저장 (보정용 고정값)
        self._log("중심 정렬 완료")
        if self.camera_manager:
            # 카메라 이미지 중심 거리 측정 (보정 시 사용할 고정 거리)
            distance = self.camera_manager.get_distance_at_center()
            if distance and distance > 0:
                self._log(f"[보정용] 카메라 중심 거리: {distance:.0f} mm (고정)")
                self._update_distance_display(distance)
                # 자동 보정용 거리 저장 (이 값을 보정 계산에 사용)
                self.aligned_distance = distance
            else:
                self._log("거리 측정 실패")
                self.aligned_distance = None

        # 현재 Rx, Ry, Rz, X, Y, Z 저장 (자동 보정용)
        if pose_after:
            self.aligned_rx = pose_after[3]
            self.aligned_ry = pose_after[4]
            self.aligned_rz = pose_after[5]
            self.aligned_x = pose_after[0]  # 기준 X 좌표 저장
            self.aligned_y = pose_after[1]  # 기준 Y 좌표 저장
            self.aligned_z = pose_after[2]  # 기준 Z 좌표 저장
            self._log(f"[보정용] 기준 저장: X={self.aligned_x:.1f}mm, Y={self.aligned_y:.1f}mm, Z={self.aligned_z:.1f}mm, Rx={self.aligned_rx:.1f}°, Ry={self.aligned_ry:.1f}°, Rz={self.aligned_rz:.1f}°, D={self.aligned_distance}mm")

            # Base 좌표 저장 (6축 전체) 및 UI 표시
            self.auto_calib_base_pos = pose_after  # (X, Y, Z, Rx, Ry, Rz)
            base_pos_text = f"기준 좌표: X={pose_after[0]:.1f}, Y={pose_after[1]:.1f}, Z={pose_after[2]:.1f} mm"
            self.labelAlignedBasePos.setText(base_pos_text)
            self.labelAlignedBasePosAutoTab.setText(base_pos_text)
            self._log(f"기준 Base 좌표 저장: X={pose_after[0]:.1f}, Y={pose_after[1]:.1f}, Z={pose_after[2]:.1f}")

        # 중심 정렬 성공 - 자동 보정 체크박스 활성화 및 체크
        self.checkAutoCorrection.setEnabled(True)
        self.checkAutoCorrection.setChecked(True)
        self._log("자동 보정 체크박스 활성화")

        # 정렬 오차 확인 및 재정렬 (최대 4회 시도: 초기 1회 + 재정렬 3회)
        RETRY_THRESHOLD_PX = 2.0
        MAX_RETRY_COUNT = 3

        time.sleep(0.3)  # 카메라 프레임 갱신 대기

        for retry_count in range(MAX_RETRY_COUNT + 1):  # 0, 1, 2, 3 (총 4회)
            if self.current_frame is None:
                break

            result_after = self.find_chessboard_pose(self.current_frame.copy())
            if result_after is None:
                self._log(f"[시도 {retry_count + 1}/{MAX_RETRY_COUNT + 1}] 체스보드 감지 실패")
                break

            center_after, angle_after = result_after
            frame_height, frame_width = self.current_frame.shape[:2]
            error_x = center_after[0] - frame_width / 2
            error_y = center_after[1] - frame_height / 2
            error_mm_x = error_x * self.PIXEL_TO_MM
            error_mm_y = error_y * self.PIXEL_TO_MM

            self._log(f"[시도 {retry_count + 1}/{MAX_RETRY_COUNT + 1}] 정렬 오차: X={error_x:.1f}px ({error_mm_x:.2f}mm), Y={error_y:.1f}px ({error_mm_y:.2f}mm)")

            # UI 라벨 업데이트
            error_text = f"정렬 오차: X={error_x:.1f}px ({error_mm_x:.2f}mm), Y={error_y:.1f}px ({error_mm_y:.2f}mm)"
            self.labelAlignError.setText(error_text)

            # 오차가 임계값 미만이면 성공
            if abs(error_x) < RETRY_THRESHOLD_PX and abs(error_y) < RETRY_THRESHOLD_PX:
                self._log(f"[정렬 성공] 오차 {RETRY_THRESHOLD_PX}px 미만 - 정렬 완료")
                break

            # 마지막 시도였으면 종료
            if retry_count >= MAX_RETRY_COUNT:
                self._log(f"[정렬 실패] {MAX_RETRY_COUNT + 1}회 시도 후에도 오차 {RETRY_THRESHOLD_PX}px 이상")
                break

            # 재정렬 시도
            self._log(f"오차 {RETRY_THRESHOLD_PX}px 이상 - 재정렬 시도 {retry_count + 1}/{MAX_RETRY_COUNT}")
            dx_mm_retry = -error_x * self.PIXEL_TO_MM
            dy_mm_retry = -error_y * self.PIXEL_TO_MM
            success_retry, msg_retry = self.robot.send_tcp_linear(
                'xyz', (dx_mm_retry, dy_mm_retry, 0),
                process_events_callback=QApplication.processEvents
            )

            if not success_retry:
                self._log(f"재정렬 실패: {msg_retry}")
                break

            self._log(f"재정렬 완료: {msg_retry}")
            time.sleep(0.3)  # 다음 시도 전 대기

        return

        # TODO: 로봇 이동 활성화 시 아래 주석 해제
        # 이동량이 너무 작으면 스킵
        if abs(dx_mm) < 0.5 and abs(dy_mm) < 0.5:
            self._log("이미 정렬됨 (이동량 < 0.5mm)")
            QMessageBox.information(self, "완료", "이미 중심에 정렬되어 있습니다.")
            return

        # 현재 로봇 위치 읽기
        current_pose = self.robot.read_camera_pose()
        if current_pose is None:
            QMessageBox.warning(self, "오류", "현재 로봇 위치를 읽을 수 없습니다.")
            return

        x, y, z, rx, ry, rz = current_pose

        # 베이스 프레임 기준 새 위치 계산
        new_x = x + dx_mm
        new_y = y + dy_mm

        self._log(f"현재 위치: X={x:.2f}, Y={y:.2f}")
        self._log(f"목표 위치: X={new_x:.2f}, Y={new_y:.2f}")

        # 로봇 이동 (툴프레임 1 비전, 절대 좌표)
        try:
            # 먼저 툴프레임 1(비전 프레임)으로 전환
            success, msg = self.robot.send_set_toolframe(1, wait=True)
            if not success:
                self._log(f"툴프레임 전환 실패: {msg}")
                QMessageBox.warning(self, "오류", f"툴프레임 전환 실패: {msg}")
                return

            # 절대 좌표 이동
            success, msg = self.robot.send_move_to_pose(new_x, new_y, z, rx, ry, rz, wait=True)
            if not success:
                self._log(f"이동 실패: {msg}")
                QMessageBox.warning(self, "오류", f"이동 실패: {msg}")
                return

            self._log("중심 정렬 완료")

        except Exception as e:
            self._log(f"중심 정렬 오류: {e}")
            QMessageBox.critical(self, "오류", f"중심 정렬 중 오류 발생: {e}")

    # ==================== 로봇 이동 (베이스 좌표계) ====================

    def get_calib_step_size(self) -> float:
        """캘리브레이션 탭의 스텝 크기 반환"""
        return float(self.calib_step_button_group.checkedId())

    @require_robot_connection
    def _on_calib_base_move(self, axis: str, direction: int):
        """베이스 좌표계 직선 이동"""
        step = self.get_calib_step_size() * direction

        self._log(f"베이스 {axis.upper()} {'+' if direction > 0 else ''}{step}mm 이동 중...")

        try:
            success, msg = self.robot.send_base_linear(axis, step, wait=True,
                                                       process_events_callback=QApplication.processEvents)
            if success:
                self._log(f"이동 완료")
            else:
                self._log(f"이동 실패: {msg}")
                QMessageBox.warning(self, "오류", f"이동 실패: {msg}")
        except Exception as e:
            self._log(f"이동 오류: {e}")
            QMessageBox.critical(self, "오류", f"이동 오류: {e}")

    @require_robot_connection
    def _on_calib_base_rotate(self, axis: str, direction: int):
        """베이스 좌표계 회전 이동"""
        step = self.get_calib_step_size() * direction

        self._log(f"베이스 {axis.upper()} {'+' if direction > 0 else ''}{step}deg 회전 중...")

        try:
            success, msg = self.robot.send_base_rotate(axis, step, wait=True,
                                                       process_events_callback=QApplication.processEvents)
            if success:
                self._log(f"회전 완료")
            else:
                self._log(f"회전 실패: {msg}")
                QMessageBox.warning(self, "오류", f"회전 실패: {msg}")
        except Exception as e:
            self._log(f"회전 오류: {e}")
            QMessageBox.critical(self, "오류", f"회전 오류: {e}")

    # ==================== TCP 정렬 ====================

    @require_robot_connection
    def _on_tcp_align(self, checked=False):
        """TCP 자세 정렬 - 목표 Rx, Ry, Rz로 이동"""
        # 목표 자세 읽기
        try:
            target_rx = float(self.editTcpAlignRx.text())
            target_ry = float(self.editTcpAlignRy.text())
            target_rz = float(self.editTcpAlignRz.text())
        except ValueError:
            QMessageBox.warning(self, "경고", "Rx, Ry, Rz 값을 올바르게 입력해주세요.")
            return

        # 현재 위치 읽기
        current_pose = self.robot.read_current_pose()
        if current_pose is None:
            QMessageBox.warning(self, "오류", "현재 로봇 위치를 읽을 수 없습니다.")
            return

        x, y, z, rx, ry, rz = current_pose

        self._log(f"TCP 정렬: 현재 Rx={rx:.2f}, Ry={ry:.2f}, Rz={rz:.2f}")
        self._log(f"TCP 정렬: 목표 Rx={target_rx:.2f}, Ry={target_ry:.2f}, Rz={target_rz:.2f}")

        # 절대 좌표 이동 (현재 위치 유지, 자세만 변경)
        try:
            success, msg = self.robot.send_move_to_pose(
                x, y, z, target_rx, target_ry, target_rz,
                wait=True, process_events_callback=QApplication.processEvents
            )
            if success:
                self._log(f"TCP 정렬 완료")
            else:
                self._log(f"TCP 정렬 실패: {msg}")
                QMessageBox.warning(self, "오류", f"TCP 정렬 실패: {msg}")
        except Exception as e:
            self._log(f"TCP 정렬 오류: {e}")
            QMessageBox.critical(self, "오류", f"TCP 정렬 오류: {e}")

    @require_robot_connection
    def _on_tcp_align_read(self, checked=False):
        """현재 TCP 자세를 읽어서 편집창에 표시"""
        # 현재 위치 읽기
        current_pose = self.robot.read_current_pose()
        if current_pose is None:
            QMessageBox.warning(self, "오류", "현재 로봇 위치를 읽을 수 없습니다.")
            return

        x, y, z, rx, ry, rz = current_pose

        # 편집창에 표시
        self.editTcpAlignRx.setText(f"{rx:.2f}")
        self.editTcpAlignRy.setText(f"{ry:.2f}")
        self.editTcpAlignRz.setText(f"{rz:.2f}")

        self._log(f"현재 TCP 자세 읽기: Rx={rx:.2f}, Ry={ry:.2f}, Rz={rz:.2f}")

    def _on_tcp_align_center(self):
        """중앙 정렬 프리셋 (Rx=90, Ry=0, Rz=90)"""
        self._on_rx_preset(90)

    def _on_rx_preset(self, rx: int):
        """Rx 프리셋 적용 (Ry=0, Rz=90 고정) + 자동 Z 보정"""
        self.editTcpAlignRx.setText(str(rx))
        self.editTcpAlignRy.setText("0")
        self.editTcpAlignRz.setText("90")

        # 자동 보정이 활성화되어 있고, 기준 데이터가 있으면 Z 보정
        if (self.checkAutoCorrection.isChecked() and
            self.aligned_distance is not None and
            self.aligned_rx is not None and
            self.aligned_z is not None):
            self._on_tcp_align_with_z_correction(rx)
        else:
            self._on_tcp_align()

    @require_robot_connection
    def _on_tcp_align_with_z_correction(self, target_rx: int):
        """
        Rx 변경 시 Z축 자동 보정을 포함한 TCP 정렬

        Rx가 변경되면 카메라가 X축을 중심으로 회전하면서 Z축 위치가 변함.
        (Rx=90°에서 카메라가 아래를 바라보는 상태 기준)
        보정 공식:
        - ΔRx = target_rx - aligned_rx (각도 변화량, deg)
        - ΔZ = D × tan(ΔRx)  (Z축 보정량)
        - new_z = aligned_z + ΔZ (기준 Z에서 절대 계산)
        """
        import math

        D = self.aligned_distance  # 체스보드까지 거리 (mm)
        delta_rx = target_rx - self.aligned_rx  # 각도 변화량 (deg)
        delta_rx_rad = math.radians(delta_rx)  # 라디안 변환

        # 보정량 계산 (Z축만)
        dz = D * math.tan(delta_rx_rad)  # Z축 보정

        self._log(f"[자동 보정] 기준 Rx={self.aligned_rx:.1f}°, 목표 Rx={target_rx}°, ΔRx={delta_rx:.1f}°")
        self._log(f"[자동 보정] 거리 D={D:.0f}mm, ΔZ={dz:.2f}mm")

        # 현재 위치 읽기
        current_pose = self.robot.read_current_pose()
        if current_pose is None:
            QMessageBox.warning(self, "오류", "현재 로봇 위치를 읽을 수 없습니다.")
            return

        x, y, z, rx, ry, rz = current_pose

        # 새 위치 계산 (기준 Z에서 절대적으로 계산)
        new_x = x
        new_y = y
        new_z = self.aligned_z + dz  # 기준 Z + 보정량 (절대 계산)
        new_rx = target_rx
        new_ry = 0
        new_rz = 90

        self._log(f"현재: X={x:.1f}, Y={y:.1f}, Z={z:.1f}, Rx={rx:.1f}")
        self._log(f"목표: X={new_x:.1f}, Y={new_y:.1f}, Z={new_z:.1f}, Rx={new_rx} (기준Z={self.aligned_z:.1f})")

        # 절대 좌표 이동
        try:
            success, msg = self.robot.send_move_to_pose(
                new_x, new_y, new_z, new_rx, new_ry, new_rz,
                wait=True, process_events_callback=QApplication.processEvents
            )
            if success:
                self._log(f"TCP 정렬 (Z 보정 포함) 완료")
            else:
                self._log(f"TCP 정렬 실패: {msg}")
                QMessageBox.warning(self, "오류", f"TCP 정렬 실패: {msg}")
        except Exception as e:
            self._log(f"TCP 정렬 오류: {e}")
            QMessageBox.critical(self, "오류", f"TCP 정렬 오류: {e}")

    def _on_ry_preset(self, ry: int):
        """Ry 프리셋 적용 (Rx=90, Rz=90 고정)"""
        self.editTcpAlignRx.setText("90")
        self.editTcpAlignRy.setText(str(ry))
        self.editTcpAlignRz.setText("90")
        self._on_tcp_align()

    def _on_rz_preset(self, rz: int):
        """Rz 프리셋 적용 (Rx=90, Ry=0 고정) + 자동 Y 보정"""
        self.editTcpAlignRx.setText("90")
        self.editTcpAlignRy.setText("0")
        self.editTcpAlignRz.setText(str(rz))

        # 자동 보정 조건 확인
        auto_correction_enabled = self.checkAutoCorrection.isChecked()
        has_distance = self.aligned_distance is not None
        has_rz = self.aligned_rz is not None
        has_y = self.aligned_y is not None

        self._log(f"[Rz 보정 체크] 자동보정={auto_correction_enabled}, 거리={has_distance}, Rz={has_rz}, Y={has_y}")
        if has_distance:
            self._log(f"[Rz 보정 체크] aligned_distance={self.aligned_distance:.0f}mm")
        if has_rz:
            self._log(f"[Rz 보정 체크] aligned_rz={self.aligned_rz:.1f}°")
        if has_y:
            self._log(f"[Rz 보정 체크] aligned_y={self.aligned_y:.1f}mm")

        # 자동 보정이 활성화되어 있고, 기준 데이터가 있으면 Y 보정
        if auto_correction_enabled and has_distance and has_rz and has_y:
            self._on_tcp_align_with_y_correction_rz(rz)
        else:
            self._log(f"[Rz 보정] 조건 미충족 - 일반 정렬 수행")
            self._on_tcp_align()

    @require_robot_connection
    def _on_tcp_align_with_y_correction_rz(self, target_rz: int):
        """
        Rz 변경 시 Y축 자동 보정을 포함한 TCP 정렬

        Rz가 변경되면 Y축 위치가 변함 (부호 반대 적용).
        보정 공식:
        - ΔRz = target_rz - aligned_rz (각도 변화량, deg)
        - ΔY = D × tan(ΔRz)  (Y축 보정량)
        - new_y = aligned_y - ΔY (기준 Y에서 절대 계산, 부호 반대)
        """
        import math

        D = self.aligned_distance  # 체스보드까지 거리 (mm)
        delta_rz = target_rz - self.aligned_rz  # 각도 변화량 (deg)
        delta_rz_rad = math.radians(delta_rz)  # 라디안 변환

        # 보정량 계산 (Y축만, 부호 반대)
        dy = D * math.tan(delta_rz_rad)  # Y축 보정

        self._log(f"[자동 보정] 기준 Rz={self.aligned_rz:.1f}°, 목표 Rz={target_rz}°, ΔRz={delta_rz:.1f}°")
        self._log(f"[자동 보정] 거리 D={D:.0f}mm, ΔY={dy:.2f}mm (부호 반대)")

        # 현재 위치 읽기
        current_pose = self.robot.read_current_pose()
        if current_pose is None:
            QMessageBox.warning(self, "오류", "현재 로봇 위치를 읽을 수 없습니다.")
            return

        x, y, z, rx, ry, rz = current_pose

        # 새 위치 계산 (기준 Y에서 절대적으로 계산, 부호 반대)
        new_x = x
        new_y = self.aligned_y - dy  # 기준 Y - 보정량 (부호 반대)
        new_z = z
        new_rx = 90
        new_ry = 0
        new_rz = target_rz

        self._log(f"현재: X={x:.1f}, Y={y:.1f}, Z={z:.1f}, Rz={rz:.1f}")
        self._log(f"목표: X={new_x:.1f}, Y={new_y:.1f}, Z={new_z:.1f}, Rz={new_rz} (기준Y={self.aligned_y:.1f})")

        # 절대 좌표 이동
        try:
            success, msg = self.robot.send_move_to_pose(
                new_x, new_y, new_z, new_rx, new_ry, new_rz,
                wait=True, process_events_callback=QApplication.processEvents
            )
            if success:
                self._log(f"TCP 정렬 (Y 보정 포함) 완료")
            else:
                self._log(f"TCP 정렬 실패: {msg}")
                QMessageBox.warning(self, "오류", f"TCP 정렬 실패: {msg}")
        except Exception as e:
            self._log(f"TCP 정렬 오류: {e}")
            QMessageBox.critical(self, "오류", f"TCP 정렬 오류: {e}")

    @require_robot_connection
    def _on_tcp_align_with_y_correction(self, target_ry: int):
        """
        비전 Ry 변경 시 Y축 자동 보정을 포함한 TCP 정렬

        비전 TF1의 Ry가 변경되면 베이스 Rz가 회전하고 Y축 위치가 변함.
        보정 공식:
        - 비전 Ry → 베이스 Rz 매핑
        - ΔRy = target_ry - aligned_ry (각도 변화량, deg)
        - ΔY = D × tan(ΔRy)  (Y축 보정량)
        - new_y = aligned_y + ΔY (기준 Y에서 절대 계산)
        """
        import math

        D = self.aligned_distance  # 체스보드까지 거리 (mm)
        delta_ry = target_ry - self.aligned_ry  # 각도 변화량 (deg)
        delta_ry_rad = math.radians(delta_ry)  # 라디안 변환

        # 보정량 계산 (Y축만)
        dy = D * math.tan(delta_ry_rad)  # Y축 보정

        self._log(f"[자동 보정] 기준 비전Ry={self.aligned_ry:.1f}°, 목표 비전Ry={target_ry}°, Δ={delta_ry:.1f}°")
        self._log(f"[자동 보정] 거리 D={D:.0f}mm, ΔY={dy:.2f}mm")

        # 현재 위치 읽기
        current_pose = self.robot.read_current_pose()
        if current_pose is None:
            QMessageBox.warning(self, "오류", "현재 로봇 위치를 읽을 수 없습니다.")
            return

        x, y, z, rx, ry, rz = current_pose

        # 새 위치 계산 (기준 Y에서 절대적으로 계산)
        # 비전 Ry → 베이스 Rz 매핑
        new_x = x
        new_y = self.aligned_y + dy  # 기준 Y + 보정량 (절대 계산)
        new_z = z
        new_rx = 90
        new_ry = 0          # 베이스 Ry 고정
        new_rz = target_ry  # 비전 Ry → 베이스 Rz

        self._log(f"현재: X={x:.1f}, Y={y:.1f}, Z={z:.1f}, 베이스Rz={rz:.1f}")
        self._log(f"목표: X={new_x:.1f}, Y={new_y:.1f}, Z={new_z:.1f}, 베이스Rz={new_rz} (기준Y={self.aligned_y:.1f})")

        # 절대 좌표 이동
        try:
            success, msg = self.robot.send_move_to_pose(
                new_x, new_y, new_z, new_rx, new_ry, new_rz,
                wait=True, process_events_callback=QApplication.processEvents
            )
            if success:
                self._log(f"TCP 정렬 (Y 보정 포함) 완료")
            else:
                self._log(f"TCP 정렬 실패: {msg}")
                QMessageBox.warning(self, "오류", f"TCP 정렬 실패: {msg}")
        except Exception as e:
            self._log(f"TCP 정렬 오류: {e}")
            QMessageBox.critical(self, "오류", f"TCP 정렬 오류: {e}")

    # ==================== 자동 캘리브레이션 위치 생성 ====================
    # 좌표계: TF1 (Tool Frame 1 - 비전 프레임)
    # - X: 카메라 시야 좌우 방향
    # - Y: 카메라 시야 상하 방향
    # - Z: 카메라와 타겟 간 거리 (광축 방향)

    def _on_generate_positions(self):
        """자동 캘리브레이션 위치 생성 - Base 절대 좌표 (회전 포함)"""
        xy_step = self.spinXYStep.value()
        z_step = self.spinZStep.value()

        # 현재 로봇 위치 읽기 (기준 좌표로 저장 - 6축 전체)
        if self.robot and self.robot.is_connected:
            current_pose = self.robot.read_current_pose()
            if current_pose:
                self.auto_calib_base_pos = current_pose  # (X, Y, Z, Rx, Ry, Rz)
                x, y, z, rx, ry, rz = current_pose
                self._log(f"기준 좌표 저장: X={x:.1f}, Y={y:.1f}, Z={z:.1f}, Rx={rx:.1f}, Ry={ry:.1f}, Rz={rz:.1f}")

                # Base 절대 좌표로 위치 생성 (회전 포함, Rx/Rz 보정 적용)
                self.auto_calib_positions = generate_base_positions_with_rotation(
                    current_pose, xy_step, z_step,
                    aligned_distance=self.aligned_distance,
                    aligned_y=self.aligned_y,
                    aligned_z=self.aligned_z,
                    aligned_rx=self.aligned_rx,
                    aligned_rz=self.aligned_rz
                )
                self._update_position_list()

                # 위치 수 계산 (9개 자세 × 32개 XYZ = 288개)
                xyz_count = 32
                rotation_count = 9  # Rx 3개 + Ry 3개 + Rz 3개 (독립)
                total_count = len(self.auto_calib_positions)

                self._log(f"위치 생성 완료: {total_count}개 ({rotation_count}개 자세 × {xyz_count}개 XYZ)")
                self._log(f"  - XY 간격: {xy_step}mm, Z 간격: {z_step}mm")
                self._log(f"  - 독립 회전: Rx만(90/80/100°), Ry만(-10/0/10°), Rz만(80/90/100°)")

                # Rx/Rz 보정 상태 표시
                rx_applied = self.aligned_distance and self.aligned_z is not None and self.aligned_rx is not None
                rz_applied = self.aligned_distance and self.aligned_y is not None and self.aligned_rz is not None

                if rx_applied and rz_applied:
                    self._log(f"  - Rx/Rz 보정 적용: D={self.aligned_distance:.0f}mm, 기준Rx={self.aligned_rx:.1f}°, 기준Rz={self.aligned_rz:.1f}°")
                elif rz_applied:
                    self._log(f"  - Rz 보정 적용: D={self.aligned_distance:.0f}mm, 기준Rz={self.aligned_rz:.1f}°")
                    self._log(f"  - Rx 보정 미적용 (aligned_z 또는 aligned_rx 누락)")
                elif rx_applied:
                    self._log(f"  - Rx 보정 적용: D={self.aligned_distance:.0f}mm, 기준Rx={self.aligned_rx:.1f}°")
                    self._log(f"  - Rz 보정 미적용 (aligned_y 또는 aligned_rz 누락)")
                else:
                    self._log(f"  - Rx/Rz 보정 미적용 (중심 정렬 필요)")
            else:
                self._log("로봇 좌표 읽기 실패 - 위치 생성 불가")
        else:
            self._log("로봇 미연결 - 위치 생성 불가")

    def _on_clear_positions(self):
        """위치 리스트 초기화"""
        self.auto_calib_positions.clear()
        self.listAutoCalibPositions.clear()
        self._log("위치 리스트 초기화")

    def _update_position_list(self):
        """위치 리스트 UI 업데이트 - 상대 + Base 절대 좌표"""
        self.listAutoCalibPositions.clear()

        total_count = len(self.auto_calib_positions)
        for i, pos in enumerate(self.auto_calib_positions):
            label = format_position_label_base(i, pos, total_count, self.auto_calib_base_pos)
            self.listAutoCalibPositions.addItem(label)

    @require_robot_connection
    def _on_sequential_move(self, checked=False):
        """
        순차 이동 - 0번부터 마지막 위치까지 순차적으로 이동

        각 위치 이동 후 로봇 상태 확인하여 이동 완료 확인
        """
        import time

        if not self.auto_calib_positions:
            QMessageBox.warning(self, "경고", "먼저 위치를 생성해주세요.")
            return

        if self.auto_calib_running:
            QMessageBox.warning(self, "경고", "이미 작업이 실행 중입니다.")
            return

        # 순차 이동 시작 - 중지 버튼 활성화
        self.auto_calib_running = True
        self.btnSequentialMove.setEnabled(False)
        self.btnStopAutoCapture.setEnabled(True)

        total = len(self.auto_calib_positions)
        self._log(f"순차 이동 시작: {total}개 위치")

        try:
            for index in range(total):
                # 중지 요청 확인
                if not self.auto_calib_running:
                    self._log("사용자 요청으로 순차 이동 중지")
                    break

                # 현재 위치 선택 (UI 표시)
                self.listAutoCalibPositions.setCurrentRow(index)
                QApplication.processEvents()

                # 목표 좌표 (Base 절대 좌표)
                target_x, target_y, target_z, target_rx, target_ry, target_rz = self.auto_calib_positions[index]

                self._log(f"[{index+1}/{total}] 이동 중: X={target_x:.1f}, Y={target_y:.1f}, Z={target_z:.1f}")

                # Base 좌표계 절대 이동 (중지 콜백 포함)
                success, msg = self.robot.send_move_to_pose(
                    target_x, target_y, target_z,
                    target_rx, target_ry, target_rz,
                    wait=True,
                    process_events_callback=QApplication.processEvents,
                    stop_flag_callback=lambda: not self.auto_calib_running
                )

                if not success:
                    # 사용자 중지인 경우 에러 메시지 없이 중단
                    if msg == "사용자 중지":
                        self._log(f"[{index+1}/{total}] 사용자 중지 요청")
                        break
                    # 그 외 오류는 에러 메시지 표시
                    self._log(f"[{index+1}/{total}] 이동 실패: {msg}")
                    QMessageBox.critical(self, "오류", f"위치 [{index+1}] 이동 실패: {msg}")
                    break

                self._log(f"[{index+1}/{total}] 이동 완료")

                # 다음 이동 전 짧은 대기
                time.sleep(0.3)
                QApplication.processEvents()

            # 완료 또는 중지 메시지
            if self.auto_calib_running:
                self._log("순차 이동 완료")
                QMessageBox.information(self, "완료", f"순차 이동 완료: {total}개 위치")
            else:
                self._log("순차 이동 중지됨")

        except Exception as e:
            self._log(f"순차 이동 오류: {e}")
            QMessageBox.critical(self, "오류", f"순차 이동 오류: {e}")

        finally:
            # 버튼 상태 복원
            self.auto_calib_running = False
            self.btnSequentialMove.setEnabled(True)
            self.btnStopAutoCapture.setEnabled(False)

    @require_robot_connection
    def _on_move_to_selected_base(self, checked=False):
        """
        선택된 위치로 이동 (Base 좌표계 절대 이동)

        저장된 Base 절대 좌표로 직접 이동
        """
        import time

        # 위치 리스트 확인
        if not self.auto_calib_positions:
            QMessageBox.warning(self, "경고", "먼저 위치를 생성해주세요.")
            return

        current_item = self.listAutoCalibPositions.currentItem()
        if current_item is None:
            QMessageBox.warning(self, "경고", "이동할 위치를 선택해주세요.")
            return

        index = self.listAutoCalibPositions.currentRow()
        if not (0 <= index < len(self.auto_calib_positions)):
            return

        # 선택된 Base 절대 좌표 (X, Y, Z, Rx, Ry, Rz)
        target_x, target_y, target_z, target_rx, target_ry, target_rz = self.auto_calib_positions[index]

        self._log(f"[Base 이동] 목표: X={target_x:.1f}, Y={target_y:.1f}, Z={target_z:.1f}, Rx={target_rx:.1f}, Ry={target_ry:.1f}, Rz={target_rz:.1f}")

        try:
            # 절대 좌표로 이동 (Base 좌표계)
            self._log(f"Base 좌표계 절대 이동 중...")
            success, msg = self.robot.send_move_to_pose(
                target_x, target_y, target_z,
                target_rx, target_ry, target_rz,
                wait=True, process_events_callback=QApplication.processEvents
            )
            if not success:
                self._log(f"이동 실패: {msg}")
                return

            self._log(f"위치 [{index}] Base 이동 완료")

            # 이동 후 좌표 확인
            time.sleep(0.3)
            pose_after = self.robot.read_current_pose()
            if pose_after:
                self._log(f"[이동 후] X={pose_after[0]:.1f}, Y={pose_after[1]:.1f}, Z={pose_after[2]:.1f}")

        except Exception as e:
            self._log(f"이동 오류: {e}")
            QMessageBox.critical(self, "오류", f"이동 오류: {e}")

    def _on_run_auto_capture(self):
        """자동 캡처 실행 - 모든 위치로 이동하며 이미지 자동 저장"""
        import time

        if not self.auto_calib_positions:
            QMessageBox.warning(self, "경고", "먼저 위치를 생성해주세요.")
            return

        if self.auto_calib_running:
            QMessageBox.warning(self, "경고", "이미 자동 캡처가 실행 중입니다.")
            return

        # 저장 디렉토리 생성
        home_dir = os.path.expanduser("~")
        base_save_dir = os.path.join(home_dir, "Project", "Charging_Robot_Operation", "calibration", "camera")
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        save_dir = os.path.join(base_save_dir, f"auto_{timestamp}")

        try:
            os.makedirs(save_dir, exist_ok=True)
            self._log(f"저장 경로: {save_dir}")
        except Exception as e:
            QMessageBox.critical(self, "오류", f"디렉토리 생성 실패: {e}")
            return

        # 자동 캡처 시작
        self.auto_calib_running = True
        self.btnRunAutoCapture.setEnabled(False)
        self.btnStopAutoCapture.setEnabled(True)

        total = len(self.auto_calib_positions)
        stabilization_ms = self.spinStabilizationDelay.value()
        stabilization_sec = stabilization_ms / 1000.0

        self._log(f"자동 캡처 시작: {total}개 위치, 안정화 시간: {stabilization_ms}ms")

        try:
            for index in range(total):
                # 중지 요청 확인
                if not self.auto_calib_running:
                    self._log("사용자 요청으로 자동 캡처 중지")
                    break

                # 현재 위치 선택 (UI 표시)
                self.listAutoCalibPositions.setCurrentRow(index)
                QApplication.processEvents()

                # 목표 좌표 (Base 절대 좌표)
                target_x, target_y, target_z, target_rx, target_ry, target_rz = self.auto_calib_positions[index]

                self._log(f"[{index+1}/{total}] 이동 중: X={target_x:.1f}, Y={target_y:.1f}, Z={target_z:.1f}, Rx={target_rx:.1f}, Ry={target_ry:.1f}, Rz={target_rz:.1f}")

                # Base 좌표계 절대 이동 (중지 콜백 포함)
                success, msg = self.robot.send_move_to_pose(
                    target_x, target_y, target_z,
                    target_rx, target_ry, target_rz,
                    wait=True,
                    process_events_callback=QApplication.processEvents,
                    stop_flag_callback=lambda: not self.auto_calib_running
                )

                if not success:
                    # 사용자 중지인 경우 에러 메시지 없이 중단
                    if msg == "사용자 중지":
                        self._log(f"[{index+1}/{total}] 사용자 중지 요청")
                        break
                    # 그 외 오류는 에러 메시지 표시
                    self._log(f"[{index+1}/{total}] 이동 실패: {msg}")
                    QMessageBox.critical(self, "오류", f"위치 [{index+1}] 이동 실패: {msg}")
                    break

                self._log(f"[{index+1}/{total}] 이동 완료, 안정화 대기 중...")

                # 안정화 대기
                time.sleep(stabilization_sec)
                QApplication.processEvents()

                # 이미지 캡처
                if self.current_frame is None:
                    self._log(f"[{index+1}/{total}] 경고: 프레임 없음, 캡처 건너뜀")
                    continue

                # 파일명: 인덱스_좌표.png
                filename = f"{index:04d}_X{target_x:.1f}_Y{target_y:.1f}_Z{target_z:.1f}_Rx{target_rx:.1f}_Ry{target_ry:.1f}_Rz{target_rz:.1f}.png"
                filepath = os.path.join(save_dir, filename)

                try:
                    cv2.imwrite(filepath, self.current_frame)
                    self._log(f"[{index+1}/{total}] 캡처 완료: {filename}")
                except Exception as e:
                    self._log(f"[{index+1}/{total}] 캡처 실패: {e}")

                QApplication.processEvents()

            # 완료 또는 중지 메시지
            if self.auto_calib_running:
                self._log(f"자동 캡처 완료: {total}개 위치")
                QMessageBox.information(self, "완료", f"자동 캡처 완료\n저장 위치: {save_dir}")
            else:
                self._log("자동 캡처 중지됨")

        except Exception as e:
            self._log(f"자동 캡처 오류: {e}")
            QMessageBox.critical(self, "오류", f"자동 캡처 오류: {e}")

        finally:
            # 버튼 상태 복원
            self.auto_calib_running = False
            self.btnRunAutoCapture.setEnabled(True)
            self.btnStopAutoCapture.setEnabled(False)

    def _on_stop_auto_capture(self):
        """자동 캡처 중지"""
        if not self.auto_calib_running:
            return

        self._log("자동 캡처 중지 요청")
        self.auto_calib_running = False
        self.btnRunAutoCapture.setEnabled(True)
        self.btnStopAutoCapture.setEnabled(False)
        self._log("자동 캡처 중지됨")
