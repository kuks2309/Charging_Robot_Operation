#!/usr/bin/env python3
"""
Eye in Hand 캘리브레이션 탭
로봇 엔드이펙터에 장착된 카메라의 Hand-Eye 캘리브레이션

특징:
- TF0 (기본 프레임) 사용
- DS435 카메라 + undistort 적용
- Chessboard 기반 포즈 추정
"""

import os
import cv2
import yaml
import numpy as np
from datetime import datetime
from PyQt5 import uic
from PyQt5.QtWidgets import QWidget, QMessageBox, QButtonGroup, QApplication, QFileDialog
from PyQt5.QtCore import pyqtSignal

from utils.common import (
    display_frame_on_label,
    save_snapshot,
    require_robot_connection,
    require_camera_running,
)
from utils.chessboard_detector import ChessboardDetector
from tabs.calibration_mixin import CalibrationMixin


# UI 파일 경로
UI_DIR = os.path.join(os.path.dirname(__file__), '..', '..', 'ui')
TAB_EYE_IN_HAND_UI = os.path.join(UI_DIR, 'tab_eye_in_hand.ui')

# DS435 캘리브레이션 파일 경로
CONFIG_DIR = os.path.join(os.path.dirname(__file__), '..', '..', 'config')
DS435_CALIB_FILE = os.path.join(CONFIG_DIR, 'ds435_calibration.yaml')


class TabEyeInHand(CalibrationMixin, QWidget):
    """Eye in Hand 캘리브레이션 탭 클래스"""

    # 시그널 정의
    log_message = pyqtSignal(str)
    camera_start_requested = pyqtSignal()
    camera_stop_requested = pyqtSignal()

    def __init__(self, parent=None):
        super().__init__(parent)

        # UI 로드
        uic.loadUi(TAB_EYE_IN_HAND_UI, self)

        # Mixin 초기화
        self.init_mixin()

        # 로봇 참조
        self.robot = None

        # 카메라 매니저 참조
        self.camera_manager = None

        # 현재 프레임 (캡처용)
        self.current_frame = None
        self.undistorted_frame = None  # undistort 적용된 프레임

        # DS435 캘리브레이션 데이터 로드
        self.camera_matrix = None
        self.dist_coeffs = None
        self._load_ds435_calibration()

        # 체스보드 감지기
        self.chessboard_detector = ChessboardDetector(cols=10, rows=7, square_size=22.0)

        # Hand-Eye 캘리브레이션 데이터
        self.robot_poses = []  # 로봇 TCP 포즈 리스트 [(R, t), ...]
        self.camera_poses = []  # 카메라에서 본 타겟 포즈 리스트 [(R, t), ...]

        # 캘리브레이션 결과
        self.hand_eye_matrix = None  # 4x4 변환 행렬

        # 버튼 그룹
        self._setup_button_groups()

        # 시그널 연결
        self._connect_signals()

        # 초기화
        self._init_ui()

    def _load_ds435_calibration(self):
        """DS435 캘리브레이션 파일 로드"""
        try:
            with open(DS435_CALIB_FILE, 'r') as f:
                data = yaml.safe_load(f)

            self.camera_matrix = np.array(data['camera_matrix'])
            self.dist_coeffs = np.array(data['distortion_coefficients'])
            print(f"[EyeInHand] DS435 캘리브레이션 로드: {DS435_CALIB_FILE}")
            print(f"  - camera_matrix: {self.camera_matrix.shape}")
            print(f"  - dist_coeffs: {self.dist_coeffs.shape}")

        except Exception as e:
            print(f"[EyeInHand] DS435 캘리브레이션 로드 실패: {e}")
            self.camera_matrix = None
            self.dist_coeffs = None

    def _setup_button_groups(self):
        """버튼 그룹 설정"""
        # 카메라 선택 버튼 그룹 (DS435/ArduCam)
        self.camera_select_button_group = QButtonGroup(self)
        self.camera_select_button_group.addButton(self.radioDS435, 0)
        self.camera_select_button_group.addButton(self.radioArduCam, 1)

        # 스텝 버튼 그룹
        self.step_button_group = QButtonGroup(self)
        self.step_button_group.addButton(self.radioEyeInHandStep1, 1)
        self.step_button_group.addButton(self.radioEyeInHandStep5, 5)
        self.step_button_group.addButton(self.radioEyeInHandStep10, 10)

    def _connect_signals(self):
        """내부 시그널-슬롯 연결"""
        # 카메라 버튼
        self.btnEyeInHandStartCamera.clicked.connect(self._on_start_camera)
        self.btnEyeInHandStopCamera.clicked.connect(self._on_stop_camera)
        self.btnEyeInHandSnapshot.clicked.connect(self._on_snapshot)

        # 체스보드 포즈 버튼
        self.btnFindChessboardPose.clicked.connect(self._on_find_chessboard_pose)

        # 중심 정렬 버튼
        self.btnAlignCenter.clicked.connect(self._on_align_center)

        # 조그 이동 버튼 (베이스 좌표계)
        self.btnEyeInHandXMinus.clicked.connect(lambda: self._on_base_move('x', -1))
        self.btnEyeInHandXPlus.clicked.connect(lambda: self._on_base_move('x', 1))
        self.btnEyeInHandYMinus.clicked.connect(lambda: self._on_base_move('y', -1))
        self.btnEyeInHandYPlus.clicked.connect(lambda: self._on_base_move('y', 1))
        self.btnEyeInHandZMinus.clicked.connect(lambda: self._on_base_move('z', -1))
        self.btnEyeInHandZPlus.clicked.connect(lambda: self._on_base_move('z', 1))
        self.btnEyeInHandRxMinus.clicked.connect(lambda: self._on_base_rotate('rx', -1))
        self.btnEyeInHandRxPlus.clicked.connect(lambda: self._on_base_rotate('rx', 1))
        self.btnEyeInHandRyMinus.clicked.connect(lambda: self._on_base_rotate('ry', -1))
        self.btnEyeInHandRyPlus.clicked.connect(lambda: self._on_base_rotate('ry', 1))
        self.btnEyeInHandRzMinus.clicked.connect(lambda: self._on_base_rotate('rz', -1))
        self.btnEyeInHandRzPlus.clicked.connect(lambda: self._on_base_rotate('rz', 1))

        # TCP 정렬 버튼
        self.btnTcpAlign.clicked.connect(self._on_tcp_align)
        self.btnTcpAlignRead.clicked.connect(self._on_tcp_align_read)
        self.btnTcpAlignCenter.clicked.connect(self._on_tcp_align_center)

        # Rx 프리셋 버튼
        self.btnRxPreset70.clicked.connect(lambda: self._on_rx_preset(70))
        self.btnRxPreset80.clicked.connect(lambda: self._on_rx_preset(80))
        self.btnRxPreset90.clicked.connect(lambda: self._on_rx_preset(90))
        self.btnRxPreset100.clicked.connect(lambda: self._on_rx_preset(100))
        self.btnRxPreset110.clicked.connect(lambda: self._on_rx_preset(110))

        # Ry 프리셋 버튼
        self.btnRyPresetM20.clicked.connect(lambda: self._on_ry_preset(-20))
        self.btnRyPresetM10.clicked.connect(lambda: self._on_ry_preset(-10))
        self.btnRyPreset0.clicked.connect(lambda: self._on_ry_preset(0))
        self.btnRyPreset10.clicked.connect(lambda: self._on_ry_preset(10))
        self.btnRyPreset20.clicked.connect(lambda: self._on_ry_preset(20))

        # Rz 프리셋 버튼
        self.btnRzPreset70.clicked.connect(lambda: self._on_rz_preset(70))
        self.btnRzPreset80.clicked.connect(lambda: self._on_rz_preset(80))
        self.btnRzPreset90.clicked.connect(lambda: self._on_rz_preset(90))
        self.btnRzPreset100.clicked.connect(lambda: self._on_rz_preset(100))
        self.btnRzPreset110.clicked.connect(lambda: self._on_rz_preset(110))

        # 자동 캘리브레이션 위치 버튼 (Mixin 메서드 사용)
        self.btnGeneratePositions.clicked.connect(self._on_generate_positions)
        self.btnClearPositions.clicked.connect(self._on_clear_positions)
        self.btnMoveToSelectedBase.clicked.connect(self._on_move_to_selected_base)
        self.btnSequentialMove.clicked.connect(self._on_sequential_move)
        self.btnRunAutoCapture.clicked.connect(self._on_run_auto_capture)
        self.btnStopAutoCapture.clicked.connect(self._on_stop_auto_capture)

        # Hand-Eye 캘리브레이션 버튼
        self.btnRunHandEyeCalib.clicked.connect(self._on_run_hand_eye_calib)
        self.btnSaveHandEyeCalib.clicked.connect(self._on_save_hand_eye_calib)
        self.btnLoadHandEyeCalib.clicked.connect(self._on_load_hand_eye_calib)

    def _init_ui(self):
        """UI 초기화"""
        self._update_pose_pair_count()
        self.checkAutoCorrection.setEnabled(False)
        self.checkAutoCorrection.setChecked(False)

    def _log(self, message: str):
        """로그 메시지 출력"""
        self.log_message.emit(message)
        print(f"[EyeInHand] {message}")

    # ==================== Mixin 오버라이드 ====================

    def get_step_size(self) -> float:
        """스텝 크기 반환"""
        return float(self.step_button_group.checkedId())

    def get_tcp_align_values(self) -> tuple:
        """TCP 정렬 목표값 반환"""
        rx = float(self.editTcpAlignRx.text())
        ry = float(self.editTcpAlignRy.text())
        rz = float(self.editTcpAlignRz.text())
        return (rx, ry, rz)

    def set_tcp_align_values(self, rx: float, ry: float, rz: float):
        """TCP 정렬 목표값 설정"""
        self.editTcpAlignRx.setText(f"{rx:.2f}")
        self.editTcpAlignRy.setText(f"{ry:.2f}")
        self.editTcpAlignRz.setText(f"{rz:.2f}")

    def _is_auto_correction_enabled(self) -> bool:
        """자동 보정 활성화 여부"""
        return self.checkAutoCorrection.isChecked()

    def get_save_dir_prefix(self) -> str:
        """저장 디렉토리 접두사"""
        return "hand_eye"

    def on_capture_at_position(self, index: int, total: int, save_dir: str,
                                target_pose: tuple) -> bool:
        """각 위치에서의 캡처 - 로봇 포즈(TF0) + 카메라 포즈 저장"""
        if self.undistorted_frame is None:
            self._log(f"[{index+1}/{total}] 프레임 없음")
            return False

        # TF0 고정 확인 (Hand-Eye 캘리브레이션은 기본 TCP 기준)
        current_tf = self.robot.read_current_toolframe()
        if current_tf != 0:
            self._log(f"[{index+1}/{total}] TF0으로 전환 중...")
            self.robot.send_set_toolframe(0, wait=True)

        # 체스보드 감지
        result = self.chessboard_detector.detect(self.undistorted_frame)
        if result is None:
            self._log(f"[{index+1}/{total}] 체스보드 감지 실패")
            return False

        corners, center, angle = result

        # 로봇 포즈 저장
        robot_pose = self.robot.read_current_pose()
        if robot_pose is None:
            self._log(f"[{index+1}/{total}] 로봇 포즈 읽기 실패")
            return False

        # solvePnP로 카메라 포즈 계산
        obj_points = self.chessboard_detector.get_object_points()
        success, rvec, tvec = cv2.solvePnP(
            obj_points, corners,
            self.camera_matrix, None  # undistort된 이미지이므로 distCoeffs=None
        )

        if not success:
            self._log(f"[{index+1}/{total}] solvePnP 실패")
            return False

        # 포즈 쌍 저장
        R_cam, _ = cv2.Rodrigues(rvec)
        self.robot_poses.append(robot_pose)
        self.camera_poses.append((R_cam, tvec))

        # 이미지 저장
        target_x, target_y, target_z, target_rx, target_ry, target_rz = target_pose
        filename = f"{index:04d}_X{target_x:.1f}_Y{target_y:.1f}_Z{target_z:.1f}_Rx{target_rx:.1f}_Ry{target_ry:.1f}_Rz{target_rz:.1f}.png"
        filepath = os.path.join(save_dir, filename)

        try:
            cv2.imwrite(filepath, self.undistorted_frame)
            self._log(f"[{index+1}/{total}] 캡처 완료 (포즈 쌍 {len(self.robot_poses)}개)")
            self._update_pose_pair_count()
            return True
        except Exception as e:
            self._log(f"[{index+1}/{total}] 저장 실패: {e}")
            return False

    def _on_run_auto_capture(self):
        """자동 캡처 실행 - TF0 고정 후 시작"""
        if not self.robot or not self.robot.is_connected:
            QMessageBox.warning(self, "경고", "로봇이 연결되지 않았습니다.")
            return

        # TF0으로 고정 (Hand-Eye 캘리브레이션은 기본 TCP 기준)
        self._log("Hand-Eye 캘리브레이션: TF0 (기본 TCP)으로 고정")
        self.robot.send_set_toolframe(0, wait=True)
        self.update_current_toolframe(0)

        # 부모 클래스 메서드 호출
        super()._on_run_auto_capture()

    # ==================== 카메라 ====================

    def _on_start_camera(self):
        """카메라 시작"""
        self.camera_start_requested.emit()

    def _on_stop_camera(self):
        """카메라 정지"""
        self.camera_stop_requested.emit()

    @require_camera_running
    def _on_snapshot(self):
        """스냅샷 저장"""
        save_snapshot(self.undistorted_frame, self, "eyeinhand", self._log)

    # ==================== 프레임 처리 ====================

    def set_camera_manager(self, camera_manager):
        """카메라 매니저 설정"""
        self.camera_manager = camera_manager

    def set_current_frame(self, frame: np.ndarray):
        """현재 프레임 설정 + undistort 적용"""
        if frame is None:
            self.current_frame = None
            self.undistorted_frame = None
            return

        self.current_frame = frame.copy()

        # undistort 적용
        if self.camera_matrix is not None and self.dist_coeffs is not None:
            self.undistorted_frame = cv2.undistort(
                frame, self.camera_matrix, self.dist_coeffs
            )
        else:
            self.undistorted_frame = frame.copy()

    def process_frame(self, frame: np.ndarray) -> np.ndarray:
        """프레임 처리 - undistort + 체스보드 감지"""
        # undistort 적용
        if self.camera_matrix is not None and self.dist_coeffs is not None:
            processed = cv2.undistort(frame, self.camera_matrix, self.dist_coeffs)
        else:
            processed = frame.copy()

        # 체스보드 감지
        if self.is_chessboard_detect_enabled():
            self.get_chessboard_size()
            self.get_square_size()

            result = self.chessboard_detector.detect(processed)
            if result is not None:
                corners, center, angle = result
                processed = self.chessboard_detector.draw_corners(processed, corners)
                processed = self.chessboard_detector.draw_pose(processed, center)

                # 상태 텍스트
                cx, cy = int(center[0]), int(center[1])
                cv2.putText(processed, "Chessboard Detected", (10, 30),
                           cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                cv2.putText(processed, f"Center: ({cx}, {cy})", (10, 60),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                cv2.putText(processed, f"Angle: {angle:.1f} deg", (10, 85),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

                # 거리 측정
                if self.camera_manager:
                    distance = self.camera_manager.get_distance_at(cx, cy, from_color=True)
                    if distance and distance > 0:
                        cv2.putText(processed, f"Distance: {distance:.0f} mm", (10, 110),
                                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 255), 2)

                # UI 업데이트
                self.labelChessboardCenterValue.setText(f"({cx}, {cy})")
                self.labelChessboardAngleValue.setText(f"{angle:.1f}°")
            else:
                cv2.putText(processed, "Chessboard Not Found", (10, 30),
                           cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
                self.labelChessboardCenterValue.setText("-")
                self.labelChessboardAngleValue.setText("-")

        # undistort 표시
        cv2.putText(processed, "Undistorted", (10, processed.shape[0] - 10),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)

        return processed

    def display_frame(self, frame: np.ndarray):
        """프레임을 QLabel에 표시"""
        if frame is not None:
            display_frame_on_label(frame, self.labelEyeInHandCameraView)

    # ==================== 체스보드 설정 ====================

    def get_chessboard_size(self) -> tuple:
        """체스보드 크기 반환 및 감지기 업데이트"""
        cols = self.spinChessboardCols.value()
        rows = self.spinChessboardRows.value()
        self.chessboard_detector.set_board_size(cols, rows)
        return (cols, rows)

    def get_square_size(self) -> float:
        """체스보드 정사각형 크기 반환"""
        size = self.spinSquareSize.value()
        self.chessboard_detector.set_square_size(size)
        return size

    def is_chessboard_detect_enabled(self) -> bool:
        """체스보드 감지 활성화 여부"""
        return self.checkChessboardDetect.isChecked()

    @require_camera_running
    def _on_find_chessboard_pose(self):
        """체스보드 포즈 찾기"""
        self.get_chessboard_size()

        result = self.chessboard_detector.detect(self.undistorted_frame)
        if result is None:
            QMessageBox.warning(self, "경고", "체스보드를 감지하지 못했습니다.")
            return

        corners, center, angle = result
        self._log(f"체스보드 포즈 - 중심: ({center[0]:.1f}, {center[1]:.1f}), 각도: {angle:.2f}°")

    # ==================== 중심 정렬 ====================

    @require_robot_connection
    @require_camera_running
    def _on_align_center(self, checked=False):
        """중심 정렬 - 체스보드 중심을 이미지 중심으로 이동"""
        import time

        result = self.chessboard_detector.detect(self.undistorted_frame)
        if result is None:
            QMessageBox.warning(self, "경고", "체스보드를 감지하지 못했습니다.")
            return

        center, angle = result[1], result[2]

        # 이미지 중심
        h, w = self.undistorted_frame.shape[:2]
        image_center_x = w / 2
        image_center_y = h / 2

        # 오프셋 계산
        dx_pixel = center[0] - image_center_x
        dy_pixel = center[1] - image_center_y
        dx_mm = -dx_pixel * self.PIXEL_TO_MM
        dy_mm = -dy_pixel * self.PIXEL_TO_MM

        self._log(f"중심 오프셋: dx={dx_pixel:.1f}px, dy={dy_pixel:.1f}px")
        self._log(f"로봇 이동량: X={dx_mm:.2f}mm, Y={dy_mm:.2f}mm")

        # TF0 설정
        self._log("툴프레임 0 설정 중...")
        self.robot.send_set_toolframe(0, wait=False)
        for _ in range(50):
            time.sleep(0.1)
            if self.robot.read_status() == 0:
                break
        time.sleep(0.5)

        # XY 이동
        success, msg = self.robot.send_tcp_linear(
            'xyz', (dx_mm, dy_mm, 0),
            process_events_callback=QApplication.processEvents
        )

        if not success:
            self._log(f"이동 실패: {msg}")
            return

        time.sleep(0.5)

        # 정렬 후 기준 좌표 저장
        pose_after = self.robot.read_current_pose()
        if pose_after:
            self.aligned_x = pose_after[0]
            self.aligned_y = pose_after[1]
            self.aligned_z = pose_after[2]
            self.aligned_rx = pose_after[3]
            self.aligned_ry = pose_after[4]
            self.aligned_rz = pose_after[5]

            # 거리 저장
            if self.camera_manager:
                distance = self.camera_manager.get_distance_at_center()
                if distance and distance > 0:
                    self.aligned_distance = distance

            # Base 좌표 저장
            self.auto_calib_base_pos = pose_after
            base_text = f"기준 좌표: X={pose_after[0]:.1f}, Y={pose_after[1]:.1f}, Z={pose_after[2]:.1f} mm"
            self.labelAlignedBasePos.setText(base_text)
            self.labelAlignedBasePosAutoTab.setText(base_text)

            self._log(f"기준 좌표 저장: {base_text}")

        # 자동 보정 활성화
        self.checkAutoCorrection.setEnabled(True)
        self.checkAutoCorrection.setChecked(True)
        self._log("중심 정렬 완료")

    # ==================== 로봇 인터페이스 ====================

    def set_robot(self, robot):
        """로봇 참조 설정"""
        self.robot = robot

    def update_robot_position(self, x: float, y: float, z: float,
                               rx: float, ry: float, rz: float):
        """로봇 좌표 업데이트"""
        self.editEyeInHandX.setText(f"{x:.2f}")
        self.editEyeInHandY.setText(f"{y:.2f}")
        self.editEyeInHandZ.setText(f"{z:.2f}")
        self.editEyeInHandRx.setText(f"{rx:.2f}")
        self.editEyeInHandRy.setText(f"{ry:.2f}")
        self.editEyeInHandRz.setText(f"{rz:.2f}")

    def update_current_toolframe(self, toolframe: int):
        """현재 툴프레임 업데이트"""
        tf_names = {0: "TF0 (기본)", 1: "TF1 (비전)", 2: "TF2", 3: "TF3"}
        tf_colors = {0: "#666666", 1: "#2196F3", 2: "#FF9800", 3: "#9C27B0"}
        name = tf_names.get(toolframe, f"TF{toolframe}")
        color = tf_colors.get(toolframe, "#000000")
        self.labelEyeInHandToolframeValue.setText(name)
        self.labelEyeInHandToolframeValue.setStyleSheet(f"color: {color}; font-weight: bold;")

    # ==================== Hand-Eye 캘리브레이션 ====================

    def _update_pose_pair_count(self):
        """포즈 쌍 수 업데이트"""
        count = len(self.robot_poses)
        self.labelPosePairCountValue.setText(str(count))

    def _on_run_hand_eye_calib(self):
        """Hand-Eye 캘리브레이션 실행"""
        if len(self.robot_poses) < 3:
            QMessageBox.warning(self, "경고", "최소 3개 이상의 포즈 쌍이 필요합니다.")
            return

        self._log(f"Hand-Eye 캘리브레이션 시작 ({len(self.robot_poses)}개 포즈 쌍)")

        try:
            # 로봇 포즈를 R, t로 변환
            R_gripper2base_list = []
            t_gripper2base_list = []

            for pose in self.robot_poses:
                x, y, z, rx, ry, rz = pose
                # Euler angles to rotation matrix
                R = self._euler_to_rotation_matrix(rx, ry, rz)
                t = np.array([[x], [y], [z]], dtype=np.float64)
                R_gripper2base_list.append(R)
                t_gripper2base_list.append(t)

            # 카메라 포즈 (이미 R, t 형태)
            R_target2cam_list = [p[0] for p in self.camera_poses]
            t_target2cam_list = [p[1] for p in self.camera_poses]

            # cv2.calibrateHandEye 호출
            R_cam2gripper, t_cam2gripper = cv2.calibrateHandEye(
                R_gripper2base_list, t_gripper2base_list,
                R_target2cam_list, t_target2cam_list,
                method=cv2.CALIB_HAND_EYE_TSAI
            )

            # 4x4 변환 행렬 생성
            self.hand_eye_matrix = np.eye(4)
            self.hand_eye_matrix[:3, :3] = R_cam2gripper
            self.hand_eye_matrix[:3, 3] = t_cam2gripper.flatten()

            # 결과 표시
            t_mm = t_cam2gripper.flatten()
            result_text = f"T: [{t_mm[0]:.1f}, {t_mm[1]:.1f}, {t_mm[2]:.1f}] mm"
            self.labelHandEyeResult.setText(result_text)

            self._log(f"Hand-Eye 캘리브레이션 완료")
            self._log(f"  Translation: X={t_mm[0]:.2f}, Y={t_mm[1]:.2f}, Z={t_mm[2]:.2f} mm")

            QMessageBox.information(self, "완료", f"Hand-Eye 캘리브레이션 완료\n{result_text}")

        except Exception as e:
            self._log(f"Hand-Eye 캘리브레이션 오류: {e}")
            QMessageBox.critical(self, "오류", f"캘리브레이션 실패: {e}")

    def _euler_to_rotation_matrix(self, rx: float, ry: float, rz: float) -> np.ndarray:
        """Euler angles (deg) to rotation matrix"""
        import math

        rx_rad = math.radians(rx)
        ry_rad = math.radians(ry)
        rz_rad = math.radians(rz)

        Rx = np.array([
            [1, 0, 0],
            [0, math.cos(rx_rad), -math.sin(rx_rad)],
            [0, math.sin(rx_rad), math.cos(rx_rad)]
        ])

        Ry = np.array([
            [math.cos(ry_rad), 0, math.sin(ry_rad)],
            [0, 1, 0],
            [-math.sin(ry_rad), 0, math.cos(ry_rad)]
        ])

        Rz = np.array([
            [math.cos(rz_rad), -math.sin(rz_rad), 0],
            [math.sin(rz_rad), math.cos(rz_rad), 0],
            [0, 0, 1]
        ])

        return Rz @ Ry @ Rx

    def _on_save_hand_eye_calib(self):
        """Hand-Eye 캘리브레이션 결과 저장"""
        if self.hand_eye_matrix is None:
            QMessageBox.warning(self, "경고", "저장할 캘리브레이션 결과가 없습니다.")
            return

        filepath, _ = QFileDialog.getSaveFileName(
            self, "Hand-Eye 캘리브레이션 저장", "", "NumPy 파일 (*.npz)"
        )

        if not filepath:
            return

        if not filepath.endswith('.npz'):
            filepath += '.npz'

        try:
            np.savez(filepath, hand_eye_matrix=self.hand_eye_matrix)
            self._log(f"저장 완료: {filepath}")
            QMessageBox.information(self, "완료", f"저장 완료: {filepath}")
        except Exception as e:
            self._log(f"저장 실패: {e}")
            QMessageBox.critical(self, "오류", f"저장 실패: {e}")

    def _on_load_hand_eye_calib(self):
        """Hand-Eye 캘리브레이션 결과 로드"""
        filepath, _ = QFileDialog.getOpenFileName(
            self, "Hand-Eye 캘리브레이션 로드", "", "NumPy 파일 (*.npz)"
        )

        if not filepath:
            return

        try:
            data = np.load(filepath)
            self.hand_eye_matrix = data['hand_eye_matrix']

            t_mm = self.hand_eye_matrix[:3, 3]
            result_text = f"T: [{t_mm[0]:.1f}, {t_mm[1]:.1f}, {t_mm[2]:.1f}] mm"
            self.labelHandEyeResult.setText(result_text)

            self._log(f"로드 완료: {filepath}")
            QMessageBox.information(self, "완료", f"로드 완료\n{result_text}")
        except Exception as e:
            self._log(f"로드 실패: {e}")
            QMessageBox.critical(self, "오류", f"로드 실패: {e}")

    # ==================== 외부 인터페이스 ====================

    def get_hand_eye_matrix(self) -> np.ndarray:
        """Hand-Eye 변환 행렬 반환"""
        return self.hand_eye_matrix

    def get_pose_pair_count(self) -> int:
        """캡처된 포즈 쌍 수 반환"""
        return len(self.robot_poses)

    def clear_pose_pairs(self):
        """포즈 쌍 초기화"""
        self.robot_poses.clear()
        self.camera_poses.clear()
        self._update_pose_pair_count()
        self._log("포즈 쌍 초기화")
