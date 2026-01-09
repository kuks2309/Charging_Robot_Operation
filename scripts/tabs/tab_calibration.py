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
from PyQt5.QtWidgets import QWidget, QFileDialog, QMessageBox
from PyQt5.QtCore import Qt, pyqtSignal
from PyQt5.QtGui import QPixmap, QImage


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

        # 자동 캘리브레이션 상태
        self.auto_calib_running = False

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
        self.btnSaveCalibration.clicked.connect(self._on_save_calibration)
        self.btnLoadCalibration.clicked.connect(self._on_load_calibration)

        # 자동 캘리브레이션 버튼
        self.btnStartAutoCalib.clicked.connect(self._on_start_auto_calib)
        self.btnStopAutoCalib.clicked.connect(self._on_stop_auto_calib)

        # 체스보드 로봇 정렬 버튼
        self.btnAlignCenter.clicked.connect(self._on_align_center)
        self.btnAlignAngle.clicked.connect(self._on_align_angle)
        self.btnAlignBoth.clicked.connect(self._on_align_both)

    def _init_ui(self):
        """UI 초기화"""
        self._update_captured_count()
        self._clear_results()
        self._clear_chessboard_pose()

    def _log(self, message: str):
        """로그 메시지 출력"""
        self.log_message.emit(message)
        print(f"[Calibration] {message}")

    def get_chessboard_size(self) -> tuple:
        """체스보드 크기 (cols, rows) 반환"""
        cols = self.spinChessboardCols.value()
        rows = self.spinChessboardRows.value()
        return (cols, rows)

    def get_square_size(self) -> float:
        """체스보드 정사각형 크기 (mm) 반환"""
        return self.spinSquareSize.value()

    def is_chessboard_detect_enabled(self) -> bool:
        """체스보드 감지 활성화 여부"""
        return self.checkChessboardDetect.isChecked()

    def set_current_frame(self, frame: np.ndarray):
        """현재 프레임 설정 (외부에서 호출)"""
        self.current_frame = frame.copy() if frame is not None else None

    def process_frame(self, frame: np.ndarray) -> np.ndarray:
        """프레임 처리 - 체스보드 감지 및 표시"""
        if not self.is_chessboard_detect_enabled():
            return frame

        # 체스보드 코너 감지
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        board_size = self.get_chessboard_size()

        ret, corners = cv2.findChessboardCorners(
            gray, board_size,
            cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE
        )

        # 감지되면 코너 표시
        if ret:
            # 코너 서브픽셀 정밀화
            criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
            corners = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)

            # 코너 그리기
            cv2.drawChessboardCorners(frame, board_size, corners, ret)

            # 상태 텍스트
            cv2.putText(frame, "Chessboard Detected", (10, 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        else:
            cv2.putText(frame, "Chessboard Not Found", (10, 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)

        return frame

    def display_frame(self, frame: np.ndarray):
        """프레임을 QLabel에 표시"""
        if frame is None:
            return

        # BGR -> RGB 변환
        rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        h, w, ch = rgb_frame.shape
        bytes_per_line = ch * w

        # QImage로 변환
        q_image = QImage(rgb_frame.data, w, h, bytes_per_line, QImage.Format_RGB888)

        # QLabel 크기에 맞게 스케일링
        pixmap = QPixmap.fromImage(q_image)
        scaled_pixmap = pixmap.scaled(
            self.labelCalibCameraView.size(),
            Qt.KeepAspectRatio,
            Qt.SmoothTransformation
        )
        self.labelCalibCameraView.setPixmap(scaled_pixmap)

    # ==================== 카메라 버튼 핸들러 ====================

    def _on_start_camera(self):
        """카메라 시작"""
        self.camera_start_requested.emit()

    def _on_stop_camera(self):
        """카메라 정지"""
        self.camera_stop_requested.emit()

    def _on_snapshot(self):
        """스냅샷 저장"""
        if self.current_frame is None:
            QMessageBox.warning(self, "경고", "카메라가 실행되지 않았습니다.")
            return

        # 저장 경로
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        default_name = f"snapshot_calib_{timestamp}.png"

        filepath, _ = QFileDialog.getSaveFileName(
            self, "스냅샷 저장", default_name, "PNG Files (*.png);;All Files (*)"
        )

        if filepath:
            cv2.imwrite(filepath, self.current_frame)
            self._log(f"스냅샷 저장: {filepath}")

    # ==================== 체스보드 포즈 ====================

    def _on_find_chessboard_pose(self):
        """체스보드 포즈 찾기 (중심 좌표 및 회전 각도)"""
        if self.current_frame is None:
            QMessageBox.warning(self, "경고", "카메라가 실행되지 않았습니다.")
            return

        frame = self.current_frame.copy()
        result = self.find_chessboard_pose(frame)

        if result is None:
            QMessageBox.warning(self, "경고", "체스보드를 감지하지 못했습니다.")
            self._clear_chessboard_pose()
            return

        center, angle = result
        self._update_chessboard_pose(center, angle)
        self._log(f"체스보드 포즈 - 중심: ({center[0]:.1f}, {center[1]:.1f}), 각도: {angle:.2f}°")

    def find_chessboard_pose(self, frame: np.ndarray) -> tuple:
        """
        체스보드의 이미지 중심 좌표와 회전 각도 계산

        Args:
            frame: 입력 이미지

        Returns:
            ((cx, cy), angle) 또는 None (감지 실패 시)
            - center: 체스보드 중심의 이미지 좌표 (픽셀)
            - angle: 체스보드의 회전 각도 (도, degree)
        """
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        board_size = self.get_chessboard_size()

        # 체스보드 코너 감지
        ret, corners = cv2.findChessboardCorners(
            gray, board_size,
            cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE
        )

        if not ret:
            return None

        # 코너 서브픽셀 정밀화
        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
        corners = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)

        # 중심 좌표 계산 (모든 코너의 평균)
        corners_2d = corners.reshape(-1, 2)
        center_x = np.mean(corners_2d[:, 0])
        center_y = np.mean(corners_2d[:, 1])

        # 회전 각도 계산 (모든 행의 방향 벡터 평균)
        cols, rows = board_size
        angles = []

        for row_idx in range(rows):
            # 각 행의 첫 번째와 마지막 코너
            first_corner = corners_2d[row_idx * cols]
            last_corner = corners_2d[row_idx * cols + (cols - 1)]

            dx = last_corner[0] - first_corner[0]
            dy = last_corner[1] - first_corner[1]
            row_angle = np.arctan2(dy, dx)
            angles.append(row_angle)

        # 평균 각도 계산 (라디안 -> 도)
        avg_angle = np.degrees(np.mean(angles))

        return ((center_x, center_y), avg_angle)

    def _update_chessboard_pose(self, center: tuple, angle: float):
        """체스보드 포즈 UI 업데이트"""
        self.labelChessboardCenterValue.setText(f"({center[0]:.1f}, {center[1]:.1f})")
        self.labelChessboardAngleValue.setText(f"{angle:.2f}°")

    def _clear_chessboard_pose(self):
        """체스보드 포즈 UI 초기화"""
        self.labelChessboardCenterValue.setText("-")
        self.labelChessboardAngleValue.setText("-")

    # ==================== 캘리브레이션 핸들러 ====================

    def _on_capture_calib_image(self):
        """캘리브레이션 이미지 캡처"""
        if self.current_frame is None:
            QMessageBox.warning(self, "경고", "카메라가 실행되지 않았습니다.")
            return

        frame = self.current_frame.copy()
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        board_size = self.get_chessboard_size()

        # 체스보드 코너 감지
        ret, corners = cv2.findChessboardCorners(
            gray, board_size,
            cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE
        )

        if not ret:
            QMessageBox.warning(self, "경고", "체스보드를 감지하지 못했습니다.\n카메라 위치를 조정해주세요.")
            return

        # 코너 서브픽셀 정밀화
        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
        corners = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)

        # 3D 객체 점 생성
        square_size = self.get_square_size()
        objp = np.zeros((board_size[0] * board_size[1], 3), np.float32)
        objp[:, :2] = np.mgrid[0:board_size[0], 0:board_size[1]].T.reshape(-1, 2)
        objp *= square_size

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
        ret, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(
            self.calib_points_3d,
            self.calib_points_2d,
            img_shape,
            None, None
        )

        if not ret:
            QMessageBox.critical(self, "오류", "캘리브레이션에 실패했습니다.")
            return

        # 결과 저장
        self.camera_matrix = camera_matrix
        self.dist_coeffs = dist_coeffs
        self.rms_error = ret

        # UI 업데이트
        self._update_results()

        self._log(f"캘리브레이션 완료! RMS 오차: {ret:.4f}")

    def _on_save_calibration(self):
        """캘리브레이션 결과 저장"""
        if self.camera_matrix is None:
            QMessageBox.warning(self, "경고", "저장할 캘리브레이션 데이터가 없습니다.")
            return

        # 저장 경로
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        default_name = f"camera_calib_{timestamp}.npz"

        filepath, _ = QFileDialog.getSaveFileName(
            self, "캘리브레이션 저장", default_name, "NumPy 파일 (*.npz);;All Files (*)"
        )

        if filepath:
            np.savez(
                filepath,
                camera_matrix=self.camera_matrix,
                dist_coeffs=self.dist_coeffs,
                rms_error=self.rms_error
            )
            self._log(f"캘리브레이션 저장: {filepath}")
            QMessageBox.information(self, "완료", f"캘리브레이션이 저장되었습니다.\n{filepath}")

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
        """캘리브레이션 결과 표시"""
        if self.camera_matrix is None:
            self._clear_results()
            return

        # RMS 오차
        if self.rms_error is not None:
            self.labelRMSErrorValue.setText(f"{self.rms_error:.4f}")

        # 카메라 매트릭스 값
        fx = self.camera_matrix[0, 0]
        fy = self.camera_matrix[1, 1]
        cx = self.camera_matrix[0, 2]
        cy = self.camera_matrix[1, 2]

        self.labelFxValue.setText(f"{fx:.2f}")
        self.labelFyValue.setText(f"{fy:.2f}")
        self.labelCxValue.setText(f"{cx:.2f}")
        self.labelCyValue.setText(f"{cy:.2f}")

    def _clear_results(self):
        """결과 표시 초기화"""
        self.labelRMSErrorValue.setText("-")
        self.labelFxValue.setText("-")
        self.labelFyValue.setText("-")
        self.labelCxValue.setText("-")
        self.labelCyValue.setText("-")

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

    def update_robot_position(self, x: float, y: float, z: float,
                               rx: float, ry: float, rz: float):
        """로봇 좌표 업데이트 (외부에서 호출)"""
        self.editCalibX.setText(f"{x:.2f}")
        self.editCalibY.setText(f"{y:.2f}")
        self.editCalibZ.setText(f"{z:.2f}")
        self.editCalibRx.setText(f"{rx:.2f}")
        self.editCalibRy.setText(f"{ry:.2f}")
        self.editCalibRz.setText(f"{rz:.2f}")

    # ==================== 체스보드 로봇 정렬 ====================

    # 픽셀당 mm 변환 비율 (카메라 캘리브레이션 후 조정 필요)
    PIXEL_TO_MM = 0.5  # 기본값: 1픽셀 = 0.5mm (카메라 높이에 따라 달라짐)

    def _on_align_center(self):
        """중심 정렬 - 체스보드 중심을 이미지 중심으로 이동 (베이스 프레임 기준)"""
        if self.robot is None or not self.robot.is_connected:
            QMessageBox.warning(self, "경고", "로봇이 연결되지 않았습니다.")
            return

        if self.current_frame is None:
            QMessageBox.warning(self, "경고", "카메라가 실행되지 않았습니다.")
            return

        # 체스보드 포즈 감지
        result = self.find_chessboard_pose(self.current_frame.copy())
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

        # 픽셀을 mm로 변환
        # 카메라 이미지 좌표계 -> 로봇 베이스 좌표계 변환
        # 카메라 X축 = 로봇 Y축, 카메라 Y축 = 로봇 X축 (카메라 설치 방향에 따라 조정)
        dx_mm = -dy_pixel * self.PIXEL_TO_MM  # 로봇 X 이동량
        dy_mm = -dx_pixel * self.PIXEL_TO_MM  # 로봇 Y 이동량

        self._log(f"중심 오프셋: dx={dx_pixel:.1f}px, dy={dy_pixel:.1f}px")
        self._log(f"로봇 이동량: X={dx_mm:.2f}mm, Y={dy_mm:.2f}mm")

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

        # 로봇 이동 (베이스 프레임, 절대 좌표)
        try:
            success, msg = self.robot.send_move_to_pose(new_x, new_y, z, rx, ry, rz, wait=True)
            if not success:
                self._log(f"이동 실패: {msg}")
                QMessageBox.warning(self, "오류", f"이동 실패: {msg}")
                return

            self._log("중심 정렬 완료")

        except Exception as e:
            self._log(f"중심 정렬 오류: {e}")
            QMessageBox.critical(self, "오류", f"중심 정렬 중 오류 발생: {e}")

    def _on_align_angle(self):
        """각도 정렬 - TODO: 구현 필요"""
        pass

    def _on_align_both(self):
        """전체 정렬 (중심 + 각도) - TODO: 구현 필요"""
        pass

    # ==================== 자동 캘리브레이션 (TODO) ====================

    def _on_start_auto_calib(self):
        """자동 캘리브레이션 시작 - TODO: 구현 필요"""
        pass

    def _on_stop_auto_calib(self):
        """자동 캘리브레이션 중지 - TODO: 구현 필요"""
        pass
