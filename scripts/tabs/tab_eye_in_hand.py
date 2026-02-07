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
import csv
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
from utils.camera_calib_position_generator import (
    generate_planar_positions_vision_tf,
    generate_base_positions_with_rotation,
    format_position_label,
    format_position_label_base,
)
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

            # OpenCV YAML 형식 파싱 (rows, cols, data 구조)
            cam_data = data['camera_matrix']
            self.camera_matrix = np.array(cam_data['data'], dtype=np.float64).reshape(
                cam_data['rows'], cam_data['cols']
            )

            dist_data = data['distortion_coefficients']
            self.dist_coeffs = np.array(dist_data['data'], dtype=np.float64).reshape(
                dist_data['rows'], dist_data['cols']
            )

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
                                target_pose: tuple, csv_path: str = None) -> bool:
        """
        각 위치에서의 캡처 - 로봇 포즈 + 카메라 포즈 + CSV 저장

        워크플로우:
        1. 이미지는 이미 획득됨 (undistorted_frame 사용)
        2. TF4 설정 후 TCP 좌표 읽기
        3. 체스보드 감지, solvePnP, CSV 저장

        Args:
            index: 현재 인덱스
            total: 총 위치 수
            save_dir: 저장 디렉토리
            target_pose: 목표 포즈 (x, y, z, rx, ry, rz) - Robot Base 좌표계
            csv_path: CSV 파일 경로 (None이면 CSV 저장 안함)

        Returns:
            True: 캡처 성공 (체스보드 미감지여도 로봇 좌표는 저장)
        """
        timestamp = datetime.now().isoformat()
        chessboard_detected = False
        rvec_values = (0.0, 0.0, 0.0)
        tvec_values = (0.0, 0.0, 0.0)

        if self.undistorted_frame is None:
            self._log(f"[{index+1}/{total}] 프레임 없음")
            return False

        # TF4 설정 후 TCP(플랜지) 좌표 읽기
        # TF4는 오프셋 0 = 플랜지 위치 (Hand-Eye 캘리브레이션으로 카메라 오프셋 계산)
        self.robot.send_set_toolframe(4, wait=True)
        tcp_pose = self.robot.read_current_pose()
        if tcp_pose is None:
            self._log(f"[{index+1}/{total}] 로봇 TCP 포즈 읽기 실패")
            return False


        # 이미지 파일명 생성
        image_filename = f"{index:04d}_pose.png"
        filepath = os.path.join(save_dir, image_filename)

        # 체스보드 감지 시도
        result = self.chessboard_detector.detect(self.undistorted_frame)
        if result is not None:
            corners, center, angle = result
            chessboard_detected = True

            # solvePnP로 카메라 포즈 계산
            obj_points = self.chessboard_detector.get_object_points()
            success, rvec, tvec = cv2.solvePnP(
                obj_points, corners,
                self.camera_matrix, None  # undistort된 이미지이므로 distCoeffs=None
            )

            if success:
                rvec_values = (rvec[0][0], rvec[1][0], rvec[2][0])
                tvec_values = (tvec[0][0], tvec[1][0], tvec[2][0])

                # 포즈 쌍 저장 (Hand-Eye 캘리브레이션용)
                R_cam, _ = cv2.Rodrigues(rvec)
                self.robot_poses.append(tcp_pose)
                self.camera_poses.append((R_cam, tvec))
                self._update_pose_pair_count()
            else:
                self._log(f"[{index+1}/{total}] solvePnP 실패")
                chessboard_detected = False
        else:
            self._log(f"[{index+1}/{total}] 체스보드 미감지 - 로봇 좌표만 저장")

        # 이미지 저장
        try:
            cv2.imwrite(filepath, self.undistorted_frame)
        except Exception as e:
            self._log(f"[{index+1}/{total}] 이미지 저장 실패: {e}")
            return False

        # CSV 행 추가
        if csv_path:
            csv_data = {
                'timestamp': timestamp,
                'index': index,
                'image_filename': image_filename,
                'tcp_pose': tcp_pose,
                'chessboard_detected': chessboard_detected,
                'rvec': rvec_values,
                'tvec': tvec_values
            }
            self._append_csv_row(csv_path, csv_data)

        status = "체스보드 감지" if chessboard_detected else "좌표만 저장"
        self._log(f"[{index+1}/{total}] 캡처 완료 ({status}, 포즈 쌍 {len(self.robot_poses)}개)")
        return True

    def _on_run_auto_capture(self):
        """자동 캡처 실행 - Robot Base 좌표계로 이동, TF4로 좌표 읽기 + CSV 파일 초기화 포함"""
        if not self.robot or not self.robot.is_connected:
            QMessageBox.warning(self, "경고", "로봇이 연결되지 않았습니다.")
            return

        if not self.auto_calib_positions:
            QMessageBox.warning(self, "경고", "먼저 위치를 생성해주세요.")
            return

        if self.auto_calib_running:
            return

        # 이동은 Robot Base 좌표계 사용 (movel 명령은 Base 좌표계 기준)
        # 좌표 읽기는 TF4 사용
        self._log("Hand-Eye 캘리브레이션: Robot Base 좌표계로 이동, TF4(TCP/플랜지)로 좌표 읽기")

        # 저장 디렉토리 생성 (타임스탬프 포함)
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        save_dir = os.path.join(
            os.path.dirname(__file__), '..', '..', 'calibration',
            f"{self.get_save_dir_prefix()}_{timestamp}"
        )

        try:
            os.makedirs(save_dir, exist_ok=True)
            self._log(f"저장 경로: {save_dir}")
        except Exception as e:
            QMessageBox.critical(self, "오류", f"디렉토리 생성 실패: {e}")
            return

        # CSV 파일 초기화
        self._current_csv_path = self._init_csv_file(save_dir)

        self.auto_calib_running = True
        self.btnRunAutoCapture.setEnabled(False)
        self.btnStopAutoCapture.setEnabled(True)

        total = len(self.auto_calib_positions)
        stabilization_sec = self.spinStabilizationDelay.value() / 1000.0

        self._log(f"자동 캡처 시작: {total}개")

        import time
        try:
            for index in range(total):
                if not self.auto_calib_running:
                    self._log("자동 캡처 중지")
                    break

                self.listAutoCalibPositions.setCurrentRow(index)
                QApplication.processEvents()

                target = self.auto_calib_positions[index]
                self._log(f"[{index+1}/{total}] 이동 중...")

                success, msg = self.robot.send_move_to_pose(
                    *target,
                    wait=True,
                    process_events_callback=QApplication.processEvents,
                    stop_flag_callback=lambda: not self.auto_calib_running
                )

                if not success:
                    if msg != "사용자 중지":
                        self._log(f"이동 실패: {msg}")
                    break

                time.sleep(stabilization_sec)
                QApplication.processEvents()

                # CSV 경로 포함하여 캡처
                self.on_capture_at_position(index, total, save_dir, target, self._current_csv_path)

            if self.auto_calib_running:
                self._log(f"자동 캡처 완료 - CSV: {self._current_csv_path}")
                QMessageBox.information(self, "완료", f"저장 위치: {save_dir}")

        finally:
            self.auto_calib_running = False
            self.btnRunAutoCapture.setEnabled(True)
            self.btnStopAutoCapture.setEnabled(False)

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
                        self.labelChessboardDistanceValue.setText(f"{distance:.0f} mm")
                    else:
                        self.labelChessboardDistanceValue.setText("-")

                # UI 업데이트
                self.labelChessboardCenterValue.setText(f"({cx}, {cy})")
                self.labelChessboardAngleValue.setText(f"{angle:.1f}°")
            else:
                cv2.putText(processed, "Chessboard Not Found", (10, 30),
                           cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
                self.labelChessboardCenterValue.setText("-")
                self.labelChessboardAngleValue.setText("-")
                self.labelChessboardDistanceValue.setText("-")

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
        """중심 정렬 - 체스보드 중심을 이미지 중심으로 이동 (베이스 프레임 기준)"""

        # 체스보드 감지
        result = self.chessboard_detector.detect(self.undistorted_frame)

        # 결과가 있으면 화면에 표시
        if result is not None:
            frame_copy = self.undistorted_frame.copy()
            center, angle = result[1], result[2]
            cx, cy = int(center[0]), int(center[1])

            # 체스보드 중심점 표시 (빨간색 - 화면 전체 라인)
            h, w = frame_copy.shape[:2]
            cv2.line(frame_copy, (0, cy), (w, cy), (0, 0, 255), 1)
            cv2.line(frame_copy, (cx, 0), (cx, h), (0, 0, 255), 1)
            cv2.circle(frame_copy, (cx, cy), 8, (0, 0, 255), -1)

            # 이미지 중심점 표시 (파란색 - 화면 전체 라인)
            img_cx, img_cy = w // 2, h // 2
            cv2.line(frame_copy, (0, img_cy), (w, img_cy), (255, 0, 0), 1)
            cv2.line(frame_copy, (img_cx, 0), (img_cx, h), (255, 0, 0), 1)
            cv2.circle(frame_copy, (img_cx, img_cy), 8, (255, 0, 0), 2)

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

        center, angle = result[1], result[2]

        # 이미지 중심 좌표
        frame_height, frame_width = self.undistorted_frame.shape[:2]
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

        # 자동 보정 활성화
        self.checkAutoCorrection.setEnabled(True)
        self.checkAutoCorrection.setChecked(True)
        self._log("자동 보정 체크박스 활성화")

        # 정렬 오차 확인 및 재정렬 (최대 4회 시도: 초기 1회 + 재정렬 3회)
        RETRY_THRESHOLD_PX = 2.0
        MAX_RETRY_COUNT = 3

        time.sleep(0.3)  # 카메라 프레임 갱신 대기

        for retry_count in range(MAX_RETRY_COUNT + 1):  # 0, 1, 2, 3 (총 4회)
            if self.undistorted_frame is None:
                break

            result_after = self.chessboard_detector.detect(self.undistorted_frame)
            if result_after is None:
                self._log(f"[시도 {retry_count + 1}/{MAX_RETRY_COUNT + 1}] 체스보드 감지 실패")
                break

            corners_after, center_after, angle_after = result_after
            frame_height, frame_width = self.undistorted_frame.shape[:2]
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

        self._log("중심 정렬 완료")

    # ==================== 자동 캘리브레이션 위치 생성 ====================

    def _on_generate_positions(self):
        """자동 캘리브레이션 위치 생성 - Base 절대 좌표 (회전 포함)"""
        xy_step = self.spinXYStep.value()
        z_step = self.spinZStep.value()

        # Camera Calibration의 기준 좌표 사용 (없으면 현재 로봇 위치)
        if self.robot and self.robot.is_connected:
            # 1. Camera Calibration의 기준 좌표 확인
            base_pose = None
            if hasattr(self.parent(), 'tabCalibration'):
                calib_tab = self.parent().tabCalibration
                if hasattr(calib_tab, 'auto_calib_base_pos') and calib_tab.auto_calib_base_pos:
                    base_pose = calib_tab.auto_calib_base_pos
                    self._log(f"Camera Calibration 기준 좌표 사용")

            # 2. Camera Calibration 좌표가 없으면 현재 로봇 위치 사용
            if base_pose is None:
                base_pose = self.robot.read_current_pose()
                self._log(f"현재 로봇 위치를 기준 좌표로 사용")

            if base_pose:
                self.auto_calib_base_pos = base_pose  # (X, Y, Z, Rx, Ry, Rz)
                x, y, z, rx, ry, rz = base_pose
                self._log(f"기준 좌표 저장: X={x:.1f}, Y={y:.1f}, Z={z:.1f}, Rx={rx:.1f}, Ry={ry:.1f}, Rz={rz:.1f}")

                # Camera Calibration의 정렬 데이터 사용 (있으면)
                aligned_distance = self.aligned_distance
                aligned_y = self.aligned_y
                aligned_z = self.aligned_z
                aligned_rx = self.aligned_rx
                aligned_rz = self.aligned_rz

                if hasattr(self.parent(), 'tabCalibration'):
                    calib_tab = self.parent().tabCalibration
                    if hasattr(calib_tab, 'aligned_distance') and calib_tab.aligned_distance:
                        aligned_distance = calib_tab.aligned_distance
                        aligned_y = calib_tab.aligned_y
                        aligned_z = calib_tab.aligned_z
                        aligned_rx = calib_tab.aligned_rx
                        aligned_rz = calib_tab.aligned_rz
                        self._log(f"Camera Calibration 정렬 데이터 사용")

                # Base 절대 좌표로 위치 생성 (회전 포함, Rx/Rz 보정 적용)
                self.auto_calib_positions = generate_base_positions_with_rotation(
                    base_pose, xy_step, z_step,
                    aligned_distance=aligned_distance,
                    aligned_y=aligned_y,
                    aligned_z=aligned_z,
                    aligned_rx=aligned_rx,
                    aligned_rz=aligned_rz
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
                rx_applied = aligned_distance and aligned_z is not None and aligned_rx is not None
                rz_applied = aligned_distance and aligned_y is not None and aligned_rz is not None

                if rx_applied and rz_applied:
                    self._log(f"  - Rx/Rz 보정 적용: D={aligned_distance:.0f}mm, 기준Rx={aligned_rx:.1f}°, 기준Rz={aligned_rz:.1f}°")
                elif rz_applied:
                    self._log(f"  - Rz 보정 적용: D={aligned_distance:.0f}mm, 기준Rz={aligned_rz:.1f}°")
                    self._log(f"  - Rx 보정 미적용 (aligned_z 또는 aligned_rx 누락)")
                elif rx_applied:
                    self._log(f"  - Rx 보정 적용: D={aligned_distance:.0f}mm, 기준Rx={aligned_rx:.1f}°")
                    self._log(f"  - Rz 보정 미적용 (aligned_y 또는 aligned_rz 누락)")
                else:
                    self._log(f"  - Rx/Rz 보정 미적용 (중심 정렬 필요)")
            else:
                self._log("로봇 좌표 읽기 실패 - 위치 생성 불가")
        else:
            self._log("로봇 미연결 - 위치 생성 불가")

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

    # ==================== 회전 프리셋 (Camera Calibration과 동일) ====================

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
        tf_names = {0: "TF0 (Eye-in-Hand)", 1: "TF1 (비전)", 2: "TF2 (충전)", 3: "TF3 (충전)", 4: "TF4", 5: "TF5 (Hand-Eye)"}
        tf_colors = {0: "#666666", 1: "#2196F3", 2: "#FF9800", 3: "#9C27B0", 4: "#4CAF50", 5: "#E91E63"}
        name = tf_names.get(toolframe, f"TF{toolframe}")
        color = tf_colors.get(toolframe, "#000000")
        self.labelEyeInHandToolframeValue.setText(name)
        self.labelEyeInHandToolframeValue.setStyleSheet(f"color: {color}; font-weight: bold;")

    # ==================== Hand-Eye 캘리브레이션 ====================

    def _update_pose_pair_count(self):
        """포즈 쌍 수 업데이트"""
        count = len(self.robot_poses)
        self.labelPosePairCountValue.setText(str(count))

    # ==================== CSV 저장 ====================

    def _init_csv_file(self, save_dir: str) -> str:
        """
        CSV 파일 초기화 및 헤더 작성

        CSV columns:
        timestamp,index,image_filename,base_x,base_y,base_z,base_rx,base_ry,base_rz,
        tcp_x,tcp_y,tcp_z,tcp_rx,tcp_ry,tcp_rz,chessboard_detected,
        rvec_x,rvec_y,rvec_z,tvec_x,tvec_y,tvec_z

        Args:
            save_dir: 저장 디렉토리

        Returns:
            생성된 CSV 파일 경로
        """
        csv_filename = "hand_eye_data.csv"
        csv_path = os.path.join(save_dir, csv_filename)

        headers = [
            "timestamp", "index", "image_filename",
            "tcp_x", "tcp_y", "tcp_z", "tcp_rx", "tcp_ry", "tcp_rz",
            "chessboard_detected",
            "rvec_x", "rvec_y", "rvec_z",
            "tvec_x", "tvec_y", "tvec_z"
        ]

        try:
            with open(csv_path, 'w', newline='', encoding='utf-8') as f:
                writer = csv.writer(f)
                writer.writerow(headers)
            self._log(f"CSV 파일 생성: {csv_path}")
            return csv_path
        except Exception as e:
            self._log(f"CSV 파일 생성 실패: {e}")
            return None

    def _append_csv_row(self, csv_path: str, data: dict):
        """
        CSV 파일에 행 추가

        Args:
            csv_path: CSV 파일 경로
            data: 저장할 데이터 딕셔너리
                - timestamp: ISO 형식 타임스탬프
                - index: 캡처 인덱스
                - image_filename: 이미지 파일명
                - base_pose: (x, y, z, rx, ry, rz) 베이스 좌표
                - tcp_pose: (x, y, z, rx, ry, rz) TCP 좌표
                - chessboard_detected: 체스보드 감지 여부
                - rvec: (x, y, z) 회전 벡터 (없으면 0)
                - tvec: (x, y, z) 변환 벡터 (없으면 0)
        """
        try:
            tcp_pose = data.get('tcp_pose', (0, 0, 0, 0, 0, 0))
            rvec = data.get('rvec', (0, 0, 0))
            tvec = data.get('tvec', (0, 0, 0))

            row = [
                data.get('timestamp', ''),
                data.get('index', 0),
                data.get('image_filename', ''),
                tcp_pose[0], tcp_pose[1], tcp_pose[2],
                tcp_pose[3], tcp_pose[4], tcp_pose[5],
                data.get('chessboard_detected', False),
                rvec[0], rvec[1], rvec[2],
                tvec[0], tvec[1], tvec[2]
            ]

            with open(csv_path, 'a', newline='', encoding='utf-8') as f:
                writer = csv.writer(f)
                writer.writerow(row)

        except Exception as e:
            self._log(f"CSV 행 추가 실패: {e}")

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
