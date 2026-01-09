#!/usr/bin/env python3
"""
Eye in Hand 캘리브레이션 탭
로봇 엔드이펙터에 장착된 카메라의 Hand-Eye 캘리브레이션
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
TAB_EYE_IN_HAND_UI = os.path.join(UI_DIR, 'tab_eye_in_hand.ui')


class TabEyeInHand(QWidget):
    """Eye in Hand 캘리브레이션 탭 클래스"""

    # 시그널 정의
    log_message = pyqtSignal(str)
    camera_start_requested = pyqtSignal()
    camera_stop_requested = pyqtSignal()

    def __init__(self, parent=None):
        super().__init__(parent)

        # UI 로드
        uic.loadUi(TAB_EYE_IN_HAND_UI, self)

        # Hand-Eye 캘리브레이션 데이터
        self.robot_poses = []  # 로봇 TCP 포즈 (4x4 행렬)
        self.camera_poses = []  # 카메라에서 본 타겟 포즈 (4x4 행렬)

        # 캘리브레이션 결과
        self.hand_eye_matrix = None  # 카메라 -> TCP 변환 행렬

        # 현재 프레임 (캡처용)
        self.current_frame = None

        # 로봇 참조
        self.robot = None
        self.camera_calibration = None  # 카메라 내부 파라미터

        # 시그널 연결
        self._connect_signals()

        # 초기화
        self._init_ui()

    def _connect_signals(self):
        """내부 시그널-슬롯 연결"""
        # 카메라 버튼
        self.btnEyeInHandStartCamera.clicked.connect(self._on_start_camera)
        self.btnEyeInHandStopCamera.clicked.connect(self._on_stop_camera)
        self.btnEyeInHandSnapshot.clicked.connect(self._on_snapshot)

    def _init_ui(self):
        """UI 초기화"""
        pass

    def _log(self, message: str):
        """로그 메시지 출력"""
        self.log_message.emit(message)
        print(f"[EyeInHand] {message}")

    def set_robot(self, robot):
        """로봇 참조 설정"""
        self.robot = robot

    def set_camera_calibration(self, camera_matrix, dist_coeffs):
        """카메라 캘리브레이션 데이터 설정"""
        self.camera_calibration = {
            'camera_matrix': camera_matrix,
            'dist_coeffs': dist_coeffs
        }

    def set_current_frame(self, frame: np.ndarray):
        """현재 프레임 설정 (외부에서 호출)"""
        self.current_frame = frame.copy() if frame is not None else None

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
            self.labelEyeInHandCameraView.size(),
            Qt.KeepAspectRatio,
            Qt.SmoothTransformation
        )
        self.labelEyeInHandCameraView.setPixmap(scaled_pixmap)

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
        default_name = f"snapshot_eyeinhand_{timestamp}.png"

        filepath, _ = QFileDialog.getSaveFileName(
            self, "스냅샷 저장", default_name, "PNG Files (*.png);;All Files (*)"
        )

        if filepath:
            cv2.imwrite(filepath, self.current_frame)
            self._log(f"스냅샷 저장: {filepath}")

    # ==================== Hand-Eye 캘리브레이션 (추후 구현) ====================

    def capture_pose_pair(self) -> bool:
        """
        로봇 포즈와 카메라 포즈 쌍 캡처
        Returns:
            성공 여부
        """
        if self.robot is None:
            self._log("로봇이 연결되지 않았습니다.")
            return False

        if self.camera_calibration is None:
            self._log("카메라 캘리브레이션이 필요합니다.")
            return False

        if self.current_frame is None:
            self._log("카메라 프레임이 없습니다.")
            return False

        # TODO: 구현 예정
        # 1. 로봇 TCP 포즈 읽기
        # 2. 타겟(Aruco 또는 Chessboard)에서 카메라 포즈 계산
        # 3. 포즈 쌍 저장

        self._log("포즈 쌍 캡처 (미구현)")
        return False

    def run_hand_eye_calibration(self) -> bool:
        """
        Hand-Eye 캘리브레이션 실행
        Returns:
            성공 여부
        """
        if len(self.robot_poses) < 3:
            self._log("최소 3개 이상의 포즈 쌍이 필요합니다.")
            return False

        # TODO: cv2.calibrateHandEye() 호출
        # 여러 메서드 지원:
        # - cv2.CALIB_HAND_EYE_TSAI
        # - cv2.CALIB_HAND_EYE_PARK
        # - cv2.CALIB_HAND_EYE_HORAUD
        # - cv2.CALIB_HAND_EYE_ANDREFF
        # - cv2.CALIB_HAND_EYE_DANIILIDIS

        self._log("Hand-Eye 캘리브레이션 (미구현)")
        return False

    def save_hand_eye_calibration(self, filepath: str) -> bool:
        """Hand-Eye 캘리브레이션 결과 저장"""
        if self.hand_eye_matrix is None:
            return False

        try:
            np.savez(
                filepath,
                hand_eye_matrix=self.hand_eye_matrix
            )
            self._log(f"Hand-Eye 캘리브레이션 저장: {filepath}")
            return True
        except Exception as e:
            self._log(f"저장 실패: {e}")
            return False

    def load_hand_eye_calibration(self, filepath: str) -> bool:
        """Hand-Eye 캘리브레이션 결과 로드"""
        try:
            data = np.load(filepath)
            self.hand_eye_matrix = data['hand_eye_matrix']
            self._log(f"Hand-Eye 캘리브레이션 로드: {filepath}")
            return True
        except Exception as e:
            self._log(f"로드 실패: {e}")
            return False

    def clear_pose_pairs(self):
        """캡처된 포즈 쌍 초기화"""
        self.robot_poses.clear()
        self.camera_poses.clear()
        self._log("포즈 쌍 초기화")

    # ==================== 외부 인터페이스 ====================

    def get_hand_eye_matrix(self) -> np.ndarray:
        """Hand-Eye 변환 행렬 반환"""
        return self.hand_eye_matrix

    def get_pose_pair_count(self) -> int:
        """캡처된 포즈 쌍 수 반환"""
        return len(self.robot_poses)
