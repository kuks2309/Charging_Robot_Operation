#!/usr/bin/env python3
"""
Charging Robot Task Manager - Main Window
"""

import json
import os
from datetime import datetime
import time
import cv2
import numpy as np
from Sensor.aruco.aruco_detector import compute_dual_alignment, draw_dual_marker_overlay
from PyQt5 import uic
from PyQt5.QtWidgets import (
    QMainWindow, QMessageBox, QFileDialog, QTableWidgetItem, QApplication
)
from PyQt5.QtCore import QTimer

from Robot import ModbusClient, RobotController, PoseManager
from services import CameraManager, ArduCamManager, VisionManager, AlignmentService, DataCollector, PoseService
from job_types import JOB_TYPES

# 카메라 타입 상수
CAMERA_DS435 = "DS435"
CAMERA_ARDUCAM = "ArduCam"
from tabs import TabTaskEdit, TabVision, TabCalibration, TabArucoReliability, TabEyeInHand, TabMotionTest, TabLaserCalibration, TabStereoCalibration, TabLaserScan

# UI 파일 경로
UI_DIR = os.path.join(os.path.dirname(__file__), '..', 'ui')
UI_FILE = os.path.join(UI_DIR, 'main_window.ui')


class MainWindow(QMainWindow):
    """메인 윈도우 클래스"""

    # Job 타입 정의 (별도 파일에서 import)
    JOB_TYPES = JOB_TYPES

    def __init__(self):
        super().__init__()

        # UI 로드
        uic.loadUi(UI_FILE, self)

        # 로봇 Modbus 클라이언트
        self.robot: ModbusClient = None

        # 로봇 컨트롤러 및 포즈 매니저 (탭 로드 전에 초기화)
        self.pose_manager = PoseManager()
        self.robot_controller: RobotController = None

        # 태스크 시퀀스 데이터
        self.task_sequence = []
        self.current_task_index = -1

        # 분리된 탭 클래스 로드 및 추가
        self._load_separated_tabs()

        # 현재 선택된 카메라 타입
        self._current_camera_type = CAMERA_DS435

        # DS435 카메라 매니저 초기화
        self.ds435_camera_manager = CameraManager()
        self.ds435_camera_manager.set_log_callback(self._log)
        self.ds435_camera_manager.frame_ready.connect(self._on_camera_frame)

        # ArduCam 카메라 매니저 초기화
        self.arducam_manager = ArduCamManager(device_index=6, color_resolution=(1920, 1080))
        self.arducam_manager.set_log_callback(self._log)
        self.arducam_manager.frame_ready.connect(self._on_camera_frame)

        # 현재 활성 카메라 매니저 (기본: DS435)
        self.camera_manager = self.ds435_camera_manager

        # 탭에 카메라 매니저 전달
        self.tabCalibration.set_camera_manager(self.camera_manager)
        self.tabEyeInHand.set_camera_manager(self.camera_manager)

        # Vision 매니저 초기화
        self.vision_manager = VisionManager(self.camera_manager)
        self.vision_manager.set_log_callback(self._log)

        # ArUco 신뢰성 검증 탭에 매니저 전달
        self.tabArucoReliability.set_camera_manager(self.camera_manager)
        self.tabArucoReliability.set_vision_manager(self.vision_manager)

        # 스테레오 캘리브레이션 탭에 양쪽 카메라 매니저 전달
        self.tabStereoCalibration.set_camera_managers(
            self.ds435_camera_manager, self.arducam_manager)

        # 테스트 탭 카메라 프레임 시그널
        self.ds435_camera_manager.frame_ready.connect(self._on_test_ds435_frame)
        self.arducam_manager.frame_ready.connect(self._on_test_arducam_frame)

        # 정렬 서비스 초기화
        self.alignment_service = AlignmentService(self.vision_manager)
        self.alignment_service.set_log_callback(self._log)
        self.alignment_service.status_changed.connect(self._update_align_status)

        # 데이터 수집 서비스 초기화
        self.data_collector = DataCollector(self.vision_manager)
        self.data_collector.set_log_callback(self._log)

        # 포즈 서비스 초기화
        self.pose_service = PoseService(self.pose_manager)
        self.pose_service.set_log_callback(self._log)

        # 초기화
        self._connect_signals()
        self._init_status()

        # 타이머 설정
        self._setup_timers()

    def _connect_signals(self):
        """시그널-슬롯 연결 (메인윈도우 UI 위젯들만)"""
        # 실행 모니터 탭
        self.btnRun.clicked.connect(self._on_run)
        self.btnPause.clicked.connect(self._on_pause)
        self.btnStop.clicked.connect(self._on_stop)
        self.btnStep.clicked.connect(self._on_step)
        self.btnEmergencyStop.clicked.connect(self._on_emergency_stop)
        self.btnClearLog.clicked.connect(self._on_clear_log)
        self.btnSaveLog.clicked.connect(self._on_save_log)

        # 설정 탭
        self.btnTestConnection.clicked.connect(self._on_test_connection)
        self.btnSaveSettings.clicked.connect(self._on_save_settings)
        self.btnLoadSettings.clicked.connect(self._on_load_settings)
        self.btnLoadCalib.clicked.connect(self._on_load_calibration)
        self.btnNewCalib.clicked.connect(self._on_new_calibration)
        self.btnClearDebugLog.clicked.connect(self._on_clear_debug_log)

        # 메뉴 액션
        self.actionNew.triggered.connect(self._on_new_recipe)
        self.actionOpen.triggered.connect(self._on_open_recipe)
        self.actionSave.triggered.connect(self._on_save_recipe)
        self.actionSaveAs.triggered.connect(self._on_save_recipe_as)
        self.actionExit.triggered.connect(self.close)
        self.actionConnect.triggered.connect(self._on_connect)
        self.actionDisconnect.triggered.connect(self._on_disconnect)
        self.actionStartCamera.triggered.connect(self._on_start_camera)
        self.actionStopCamera.triggered.connect(self._on_stop_camera)
        self.actionEmergencyStop.triggered.connect(self._on_emergency_stop)
        self.actionAbout.triggered.connect(self._on_about)

        # 탭 변경 시그널
        self.tabWidget.currentChanged.connect(self._on_tab_changed)

    def _load_separated_tabs(self):
        """분리된 탭 클래스들을 인스턴스화하여 탭위젯에 추가"""
        # Task 편집 탭 (인덱스 0에 삽입)
        self.tabTaskEdit = TabTaskEdit(JOB_TYPES, self.pose_manager, self)
        self.tabWidget.insertTab(0, self.tabTaskEdit, "Task 편집")

        # 비전 탭 (인덱스 1에 삽입)
        self.tabVision = TabVision(self)
        self.tabWidget.insertTab(1, self.tabVision, "비전")

        # 카메라 캘리브레이션 탭 (인덱스 2에 삽입)
        self.tabCalibration = TabCalibration(self)
        self.tabWidget.insertTab(2, self.tabCalibration, "카메라 캘리브레이션")

        # ArUco 신뢰성 검증 탭 (인덱스 3에 삽입)
        self.tabArucoReliability = TabArucoReliability(self)
        self.tabWidget.insertTab(3, self.tabArucoReliability, "ArUco 신뢰성 검증")

        # Eye in Hand 탭 (인덱스 4에 삽입)
        self.tabEyeInHand = TabEyeInHand(self)
        self.tabWidget.insertTab(4, self.tabEyeInHand, "Eye in Hand")

        # Motion Test 탭 (인덱스 5에 삽입)
        self.tabMotionTest = TabMotionTest(self)
        self.tabWidget.insertTab(5, self.tabMotionTest, "모션 테스트")

        # 레이저 캘리브레이션 탭 (인덱스 6에 삽입)
        self.tabLaserCalibration = TabLaserCalibration(self)
        self.tabWidget.insertTab(6, self.tabLaserCalibration, "Laser Calibration")

        # 스테레오 캘리브레이션 탭 (인덱스 7에 삽입)
        self.tabStereoCalibration = TabStereoCalibration(self)
        self.tabWidget.insertTab(7, self.tabStereoCalibration, "Stereo Calibration")

        # 레이저 스캔 탭 (인덱스 8에 삽입)
        self.tabLaserScan = TabLaserScan(self)
        self.tabWidget.insertTab(8, self.tabLaserScan, "Laser Scan")

        # 테스트 탭 내용 교체 (충전건 결합)
        self._rebuild_test_tab()

        # 탭 시그널 연결
        self._connect_tab_signals()

        # 첫 번째 탭 선택
        self.tabWidget.setCurrentIndex(0)

        # 초기 카메라 타입으로 모든 탭의 라디오 버튼 동기화
        self._sync_camera_radio_buttons(CAMERA_DS435)

    def _rebuild_test_tab(self):
        """테스트 탭 내용을 충전건 결합 UI로 교체"""
        from PyQt5.QtWidgets import QWidget, QGroupBox, QVBoxLayout, QHBoxLayout, QPushButton, QLabel
        from PyQt5.QtCore import Qt

        # 기존 레이아웃/위젯 제거
        old_layout = self.tabTest.layout()
        if old_layout:
            while old_layout.count():
                item = old_layout.takeAt(0)
                w = item.widget()
                if w:
                    w.deleteLater()

            # 기존 레이아웃을 tabTest에서 분리
            QWidget().setLayout(old_layout)

        # 새 레이아웃: 상단 카메라 뷰 + 하단 컨트롤
        outer_layout = QVBoxLayout(self.tabTest)

        # ── 상단: 카메라 뷰 2개 (좌: DS435, 우: ArduCam) ──
        camera_row = QHBoxLayout()

        ds435_layout = QVBoxLayout()
        ds435_title = QLabel("DS435")
        ds435_title.setAlignment(Qt.AlignCenter)
        ds435_layout.addWidget(ds435_title)
        self.labelTestDS435View = QLabel()
        self.labelTestDS435View.setFixedSize(640, 360)
        self.labelTestDS435View.setAlignment(Qt.AlignCenter)
        self.labelTestDS435View.setStyleSheet("background-color: #222;")
        ds435_layout.addWidget(self.labelTestDS435View)
        camera_row.addLayout(ds435_layout)

        arducam_layout = QVBoxLayout()
        arducam_title = QLabel("ArduCam")
        arducam_title.setAlignment(Qt.AlignCenter)
        arducam_layout.addWidget(arducam_title)
        self.labelTestArduCamView = QLabel()
        self.labelTestArduCamView.setFixedSize(640, 360)
        self.labelTestArduCamView.setAlignment(Qt.AlignCenter)
        self.labelTestArduCamView.setStyleSheet("background-color: #222;")
        arducam_layout.addWidget(self.labelTestArduCamView)
        camera_row.addLayout(arducam_layout)

        camera_row.addStretch()
        outer_layout.addLayout(camera_row)

        # ── 하단: 충전건 결합 컨트롤 ──
        control_row = QHBoxLayout()

        self.groupCharging = QGroupBox("충전건 결합")
        self.groupCharging.setFixedHeight(100)
        charging_layout = QHBoxLayout(self.groupCharging)

        self.btnTestToggleCameras = QPushButton("카메라 구동")
        self.btnTestToggleCameras.setCheckable(True)
        self.btnTestToggleCameras.clicked.connect(self._on_test_toggle_cameras)
        charging_layout.addWidget(self.btnTestToggleCameras)

        self.btnTestAlignDS435 = QPushButton("DS435 Aruco 정렬")
        self.btnTestAlignDS435.clicked.connect(self._on_test_align_ds435)
        charging_layout.addWidget(self.btnTestAlignDS435)

        self.btnTestHandoffDS435 = QPushButton("DS435 → ArduCam 핸드오프")
        self.btnTestHandoffDS435.clicked.connect(self._on_test_handoff_ds435_to_arducam)
        charging_layout.addWidget(self.btnTestHandoffDS435)

        self.btnTestDepthAdjust = QPushButton("Depth 보정")
        self.btnTestDepthAdjust.clicked.connect(self._on_test_depth_adjust)
        charging_layout.addWidget(self.btnTestDepthAdjust)

        self.btnTestLaserScan = QPushButton("레이저 스캔")
        self.btnTestLaserScan.clicked.connect(self._on_test_laser_scan)
        charging_layout.addWidget(self.btnTestLaserScan)

        self.btnTestApplyRy = QPushButton("Rx 보정 적용")
        self.btnTestApplyRy.setEnabled(False)
        self.btnTestApplyRy.clicked.connect(self._on_test_apply_ry)
        charging_layout.addWidget(self.btnTestApplyRy)

        self.btnTestMoveEntrance = QPushButton("입구 이동")
        self.btnTestMoveEntrance.clicked.connect(self._on_test_move_to_entrance)
        charging_layout.addWidget(self.btnTestMoveEntrance)

        self.btnTestTcpZLinear = QPushButton("TCP Z 전진")
        self.btnTestTcpZLinear.clicked.connect(self._on_test_tcp_z_linear)
        charging_layout.addWidget(self.btnTestTcpZLinear)

        self.btnTestReleaseReturn = QPushButton("해제 복귀")
        self.btnTestReleaseReturn.clicked.connect(self._on_test_release_return)
        charging_layout.addWidget(self.btnTestReleaseReturn)

        self._test_laser_angle_deg = None  # 레이저 스캔 각도 저장

        self.labelTestAlignStatus = QLabel("")
        self.labelTestAlignStatus.setWordWrap(True)
        charging_layout.addWidget(self.labelTestAlignStatus, 1)

        control_row.addWidget(self.groupCharging)
        outer_layout.addLayout(control_row)

        outer_layout.addStretch()

    def _connect_tab_signals(self):
        """탭 클래스들의 시그널을 메인윈도우 슬롯에 연결"""
        # Task 편집 탭 시그널
        self.tabTaskEdit.log_message.connect(self._log)
        self.tabTaskEdit.connect_requested.connect(self._on_connect_from_tab)
        self.tabTaskEdit.read_current_position_requested.connect(self._on_read_current_position_from_tab)
        self.tabTaskEdit.execute_current_task_requested.connect(self._on_execute_current_task_from_tab)
        self.tabTaskEdit.jog_move_requested.connect(self._on_jog_move_from_tab)
        self.tabTaskEdit.jog_rotate_requested.connect(self._on_jog_rotate_from_tab)
        self.tabTaskEdit.task_sequence_changed.connect(self._on_task_sequence_changed)

        # 비전 탭 시그널
        self.tabVision.log_message.connect(self._log)
        self.tabVision.camera_start_requested.connect(self._on_start_camera)
        self.tabVision.camera_stop_requested.connect(self._on_stop_camera)
        self.tabVision.align_center_requested.connect(self._on_align_center_from_tab)
        self.tabVision.align_pose_requested.connect(self._on_align_pose_from_tab)
        self.tabVision.align_full_requested.connect(self._on_align_full_from_tab)

        # 캘리브레이션 탭 시그널
        self.tabCalibration.log_message.connect(self._log)
        self.tabCalibration.camera_start_requested.connect(self._on_start_camera)
        self.tabCalibration.camera_stop_requested.connect(self._on_stop_camera)

        # ArUco 신뢰성 검증 탭 시그널
        self.tabArucoReliability.log_message.connect(self._log)
        self.tabArucoReliability.camera_start_requested.connect(self._on_start_camera)
        self.tabArucoReliability.camera_stop_requested.connect(self._on_stop_camera)
        self.tabArucoReliability.jog_move_requested.connect(self._on_jog_move_from_tab)
        self.tabArucoReliability.jog_rotate_requested.connect(self._on_jog_rotate_from_tab)
        self.tabArucoReliability.align_parallel_requested.connect(self._on_ar_tag_align_parallel)
        self.tabArucoReliability.align_single_axis_requested.connect(self._on_ar_tag_align_single_axis)
        self.tabArucoReliability.align_base_ry_requested.connect(self._on_ar_tag_align_base_ry)
        self.tabArucoReliability.align_aruco_y_requested.connect(self._on_aruco_align_y)
        self.tabArucoReliability.align_aruco_x_requested.connect(self._on_aruco_align_x)
        self.tabArucoReliability.align_aruco_combined_requested.connect(self._on_aruco_align_combined)

        # Eye in Hand 탭 시그널
        self.tabEyeInHand.log_message.connect(self._log)
        self.tabEyeInHand.camera_start_requested.connect(self._on_start_camera)
        self.tabEyeInHand.camera_stop_requested.connect(self._on_stop_camera)

        # Motion Test 탭 시그널
        self.tabMotionTest.log_message.connect(self._log)

        # 레이저 캘리브레이션 탭 시그널
        self.tabLaserCalibration.log_message.connect(self._log)
        self.tabLaserCalibration.camera_start_requested.connect(self._on_start_camera)
        self.tabLaserCalibration.camera_stop_requested.connect(self._on_stop_camera)
        self.tabLaserCalibration.jog_move_requested.connect(self._on_jog_move_from_tab)
        self.tabLaserCalibration.jog_rotate_requested.connect(self._on_jog_rotate_from_tab)
        self.tabLaserCalibration.arducam_required.connect(
            lambda: self._on_camera_type_changed(CAMERA_ARDUCAM)
        )
        self.tabLaserCalibration.calib_align_aruco_requested.connect(self._on_calib_align_aruco)
        self.tabLaserCalibration.calib_adjust_z_requested.connect(self._on_calib_adjust_z)
        self.tabLaserCalibration.calib_save_pos_requested.connect(self._on_calib_save_pos)
        self.tabLaserCalibration.calib_move_x_adjust_z_requested.connect(self._on_calib_move_x_adjust_z)
        self.tabLaserCalibration.calib_save_compare_requested.connect(self._on_calib_save_compare)
        self.tabLaserCalibration.calib_auto_requested.connect(self._on_calib_auto)
        self.tabLaserCalibration.calib_cancel_requested.connect(self._on_calib_cancel)

        # 레이저 스캔 탭 시그널
        self.tabLaserScan.log_message.connect(self._log)
        self.tabLaserScan.camera_start_requested.connect(self._on_start_camera)
        self.tabLaserScan.camera_stop_requested.connect(self._on_stop_camera)
        self.tabLaserScan.jog_move_requested.connect(self._on_jog_move_from_tab)
        self.tabLaserScan.jog_rotate_requested.connect(self._on_jog_rotate_from_tab)
        self.tabLaserScan.arducam_required.connect(
            lambda: self._on_camera_type_changed(CAMERA_ARDUCAM)
        )
        self.tabLaserScan.scan_start_requested.connect(self._on_laser_scan_start)
        self.tabLaserScan.scan_cancel_requested.connect(self._on_laser_scan_cancel)

        # 스테레오 캘리브레이션 탭 시그널
        self.tabStereoCalibration.log_message.connect(self._log)
        self.tabStereoCalibration.calib_align_aruco_requested.connect(
            self._on_stereo_calib_align_aruco)
        self.tabStereoCalibration.calib_align_aruco_x_requested.connect(
            self._on_stereo_calib_align_aruco_x)
        self.tabStereoCalibration.calib_align_aruco_combined_requested.connect(
            self._on_stereo_calib_align_aruco_combined)
        self.tabStereoCalibration.calib_align_ds435_requested.connect(
            self._on_stereo_calib_align_ds435)
        self.tabStereoCalibration.handoff_ds435_to_arducam_requested.connect(
            self._on_stereo_calib_handoff_ds435_to_arducam)
        self.tabStereoCalibration.sweep_start_requested.connect(
            self._on_sweep_start)
        self.tabStereoCalibration.sweep_cancel_requested.connect(
            self._on_sweep_cancel)

        # 스윕 캘리브레이션 서비스
        self._sweep_service = None

        # 레이저 스캔 서비스
        self._laser_scan_service = None

        # 자동 캘리브레이션 상태 변수 초기화
        self._auto_calib_state = None
        self._auto_calib_iteration = 0
        self._auto_calib_total = 0
        self._auto_calib_pos1 = None

        # 카메라 선택 라디오 버튼 시그널 연결
        self._connect_camera_selection_signals()

    def _init_status(self):
        """상태 초기화"""
        self.statusbar.showMessage("준비됨")
        self._log("Charging Robot Task Manager 시작")

    def _setup_timers(self):
        """타이머 설정"""
        # 상태 업데이트 타이머 (100ms)
        self.status_timer = QTimer()
        self.status_timer.timeout.connect(self._update_robot_status)
        # 연결 후 시작됨

        # Heartbeat 연결 감시 변수
        self._heartbeat_failures: int = 0
        self._heartbeat_max_failures: int = 5  # 연속 5사이클(~500ms) 실패 시 끊김 판정
        self._handling_connection_lost: bool = False

    # ==================== 유틸리티 헬퍼 ====================

    def _require_robot(self) -> bool:
        """로봇 연결 확인. 미연결 시 경고 표시 후 False 반환"""
        if not self.robot or not self.robot.is_connected:
            QMessageBox.warning(self, "오류", "로봇이 연결되지 않았습니다.")
            return False
        return True

    # ==================== 탭 시그널 핸들러 ====================

    def _on_task_sequence_changed(self, sequence: list):
        """태스크 시퀀스 변경 시 (TabTaskEdit 시그널 핸들러)"""
        self.task_sequence = sequence

    def _on_connect_from_tab(self, ip: str, port: int):
        """로봇 연결 요청 (TabTaskEdit 시그널 핸들러)"""
        self._connect_robot(ip, port)

    def _on_read_current_position_from_tab(self):
        """현재 위치 읽기 요청 (TabTaskEdit 시그널 핸들러)"""
        if not self._require_robot():
            return

        current_pose = self.robot.read_current_pose()
        if current_pose is None:
            QMessageBox.warning(self, "오류", "현재 로봇 위치를 읽을 수 없습니다.")
            return

        x, y, z, rx, ry, rz = current_pose
        self.tabTaskEdit.fill_current_position_to_params(x, y, z, rx, ry, rz)

    def _on_execute_current_task_from_tab(self, task: dict):
        """현재 Task 실행 요청 (TabTaskEdit 시그널 핸들러)"""
        task_type = task.get('type')
        params = task.get('params', {})

        # Vision Task는 로봇 연결 불필요 (detect_dual_aruco_plane은 최종 TCP 계산에 로봇 필요)
        vision_tasks = ['detect_aruco']

        if task_type not in vision_tasks:
            if not self._require_robot():
                return

        self._log(f"Task 실행: {task_type}")

        try:
            # Task 타입별 실행
            if task_type == 'go_home':
                self._execute_go_home()
            elif task_type == 'move_to_pose':
                self._execute_move_to_pose(params)
            elif task_type == 'tcp_linear_x':
                self._execute_tcp_linear(params, 'x')
            elif task_type == 'tcp_linear_y':
                self._execute_tcp_linear(params, 'y')
            elif task_type == 'tcp_linear_z':
                self._execute_tcp_linear(params, 'z')
            elif task_type == 'tcp_rotate_rx':
                self._execute_tcp_rotate(params, 'rx')
            elif task_type == 'tcp_rotate_ry':
                self._execute_tcp_rotate(params, 'ry')
            elif task_type == 'tcp_rotate_rz':
                self._execute_tcp_rotate(params, 'rz')
            elif task_type == 'tcp_linear_xyz':
                self._execute_tcp_linear_xyz(params)
            elif task_type == 'tcp_rotate_rxryrz':
                self._execute_tcp_rotate_rxryrz(params)
            elif task_type == 'toolframe':
                self._execute_toolframe(params)
            elif task_type == 'detect_aruco':
                self._execute_detect_aruco(params)
            elif task_type == 'detect_dual_aruco_plane':
                self._execute_detect_dual_aruco_plane(params)
            elif task_type == 'align_aruco_pose':
                self._execute_align_aruco_pose(params)
            else:
                QMessageBox.information(self, "알림", f"'{task_type}' Task 실행은 아직 구현되지 않았습니다.")
                return

            self._log(f"Task 실행 완료: {task_type}")

        except Exception as e:
            self._log(f"Task 실행 오류: {e}")
            QMessageBox.critical(self, "오류", f"Task 실행 중 오류 발생:\n{e}")

    def _execute_go_home(self):
        """GO HOME 실행"""
        success, message = self.robot.send_go_home()
        if not success:
            raise Exception(f"GO HOME 실패: {message}")

    def _execute_move_to_pose(self, params: dict):
        """절대 위치 이동 실행"""
        x = params.get('x', 0.0)
        y = params.get('y', 0.0)
        z = params.get('z', 0.0)
        rx = params.get('rx', 0.0)
        ry = params.get('ry', 0.0)
        rz = params.get('rz', 0.0)

        success, message = self.robot.send_move_to_pose(x, y, z, rx, ry, rz)
        if not success:
            raise Exception(f"위치 이동 실패: {message}")

    def _execute_tcp_linear(self, params: dict, axis: str):
        """TCP 직선 이동 실행"""
        distance = params.get('distance', 0.0)
        coordinate = params.get('coordinate', 'TF1')
        self._log(f"[DEBUG] params={params}, coordinate={coordinate}")

        # 이동 전 위치 기록
        before_pose = self.robot.read_current_pose()
        if before_pose:
            self._log(f"[이동 전] X={before_pose[0]:.2f}, Y={before_pose[1]:.2f}, Z={before_pose[2]:.2f}, Rx={before_pose[3]:.2f}, Ry={before_pose[4]:.2f}, Rz={before_pose[5]:.2f}")
        else:
            self._log("[경고] 이동 전 위치 읽기 실패")

        if coordinate == 'Base':
            success, message = self.robot.send_base_linear(axis, distance)
        else:
            # TF1, TF2, TF3, TF4 - 툴프레임 설정 후 tool.trans 실행
            tf_num = int(coordinate[2])  # 'TF1' -> 1, 'TF2' -> 2, etc.
            tf_success, tf_msg = self.robot.send_set_toolframe(tf_num, wait=True)
            if not tf_success:
                raise Exception(f"툴프레임 {tf_num} 설정 실패: {tf_msg}")
            self._log(f"툴프레임 {tf_num} 설정 완료")
            self._update_statusbar()
            success, message = self.robot.send_tcp_linear(axis, distance)

        if not success:
            raise Exception(f"{coordinate} {axis.upper()} 이동 실패: {message}")

        # 이동 후 위치 기록 및 검증
        after_pose = self.robot.read_current_pose()
        if after_pose and before_pose:
            dx = after_pose[0] - before_pose[0]
            dy = after_pose[1] - before_pose[1]
            dz = after_pose[2] - before_pose[2]
            self._log(f"[이동 후] X={after_pose[0]:.2f}, Y={after_pose[1]:.2f}, Z={after_pose[2]:.2f}, Rx={after_pose[3]:.2f}, Ry={after_pose[4]:.2f}, Rz={after_pose[5]:.2f}")
            drx = after_pose[3] - before_pose[3]
            dry = after_pose[4] - before_pose[4]
            drz = after_pose[5] - before_pose[5]
            self._log(f"[변화량] dX={dx:.2f}, dY={dy:.2f}, dZ={dz:.2f}mm, dRx={drx:.2f}, dRy={dry:.2f}, dRz={drz:.2f}deg")

    def _execute_tcp_rotate(self, params: dict, axis: str):
        """TCP 회전 이동 실행"""
        angle = params.get('angle', 0.0)

        success, message = self.robot.send_tcp_rotate(axis, angle)
        if not success:
            raise Exception(f"TCP {axis.upper()} 회전 실패: {message}")

    def _execute_tcp_linear_xyz(self, params: dict):
        """TCP XYZ 직선 이동 실행"""
        x = params.get('x', 0.0)
        y = params.get('y', 0.0)
        z = params.get('z', 0.0)
        coordinate = params.get('coordinate', 'TF1')

        # 이동 전 위치 기록
        before_pose = self.robot.read_current_pose()
        if before_pose:
            self._log(f"[이동 전] X={before_pose[0]:.2f}, Y={before_pose[1]:.2f}, Z={before_pose[2]:.2f}, Rx={before_pose[3]:.2f}, Ry={before_pose[4]:.2f}, Rz={before_pose[5]:.2f}")
        else:
            self._log("[경고] 이동 전 위치 읽기 실패")

        if coordinate == 'Base':
            success, message = self.robot.send_base_linear('xyz', (x, y, z))
        else:
            # TF1, TF2, TF3, TF4 - 툴프레임 설정 후 tool.trans 실행
            tf_num = int(coordinate[2])  # 'TF1' -> 1, 'TF2' -> 2, etc.
            tf_success, tf_msg = self.robot.send_set_toolframe(tf_num, wait=True)
            if not tf_success:
                raise Exception(f"툴프레임 {tf_num} 설정 실패: {tf_msg}")
            self._log(f"툴프레임 {tf_num} 설정 완료")
            self._update_statusbar()
            success, message = self.robot.send_tcp_linear('xyz', (x, y, z))

        if not success:
            raise Exception(f"{coordinate} XYZ 이동 실패: {message}")

        # 이동 후 위치 기록 및 검증
        after_pose = self.robot.read_current_pose()
        if after_pose and before_pose:
            dx = after_pose[0] - before_pose[0]
            dy = after_pose[1] - before_pose[1]
            dz = after_pose[2] - before_pose[2]
            self._log(f"[이동 후] X={after_pose[0]:.2f}, Y={after_pose[1]:.2f}, Z={after_pose[2]:.2f}, Rx={after_pose[3]:.2f}, Ry={after_pose[4]:.2f}, Rz={after_pose[5]:.2f}")
            drx = after_pose[3] - before_pose[3]
            dry = after_pose[4] - before_pose[4]
            drz = after_pose[5] - before_pose[5]
            self._log(f"[변화량] dX={dx:.2f}, dY={dy:.2f}, dZ={dz:.2f}mm, dRx={drx:.2f}, dRy={dry:.2f}, dRz={drz:.2f}deg")

    def _execute_tcp_rotate_rxryrz(self, params: dict):
        """TCP RxRyRz 회전 이동 실행"""
        rx = params.get('rx', 0.0)
        ry = params.get('ry', 0.0)
        rz = params.get('rz', 0.0)

        success, message = self.robot.send_tcp_rotate('rxryrz', (rx, ry, rz))
        if not success:
            raise Exception(f"TCP RxRyRz 회전 실패: {message}")

    def _execute_toolframe(self, params: dict):
        """툴프레임 변경 실행"""
        frame = params.get('frame', 0)

        success, message = self.robot.send_set_toolframe(frame, wait=True)
        if not success:
            raise Exception(f"툴프레임 {frame} 변경 실패: {message}")

        self._log(f"툴프레임 {frame}으로 변경 완료")

    def _execute_detect_aruco(self, params: dict):
        """Aruco Tag 인식 실행 (카메라 자동 켜기 포함)"""
        tag_id = params.get('tag_id', 0)
        timeout = params.get('timeout', 10.0)

        # 카메라가 꺼져 있으면 자동으로 켜기
        if not self.camera_manager.is_running:
            self._log("카메라가 꺼져 있습니다. 자동으로 카메라를 시작합니다...")
            success, msg = self.camera_manager.start()
            if not success:
                raise Exception(f"카메라 시작 실패: {msg}")
            self._log("카메라 시작 완료")

        # Aruco Tag 감지
        self._log(f"Aruco Tag {tag_id} 감지 시작 (timeout: {timeout}초)...")
        result = self.detect_aruco_tag(tag_id, timeout)

        if result is None:
            raise Exception(f"Aruco Tag {tag_id}를 {timeout}초 안에 감지하지 못했습니다.")

        self._log(f"Aruco Tag {tag_id} 감지 성공!")
        QMessageBox.information(self, "성공", f"Aruco Tag {tag_id}를 감지했습니다.")

    def _execute_detect_dual_aruco_plane(self, params: dict):
        """듀얼 ArUco 평면 추출 실행"""
        tag_id1 = params.get('tag_id1', 0)
        tag_id2 = params.get('tag_id2', 1)
        num_samples = params.get('num_samples', 20)
        delay_ms = params.get('delay_ms', 100)
        timeout = params.get('timeout', 30.0)

        # 카메라 매니저 확인
        if not self.camera_manager:
            raise Exception("카메라가 초기화되지 않았습니다. 비전 탭에서 카메라를 선택해주세요.")

        # 카메라가 꺼져 있으면 자동으로 켜기
        if not self.camera_manager.is_running:
            self._log("카메라가 꺼져 있습니다. 자동으로 카메라를 시작합니다...")
            success, msg = self.camera_manager.start()
            if not success:
                raise Exception(f"카메라 시작 실패: {msg}")
            self._log("카메라 시작 완료")

        # DualArucoDetector import
        try:
            from services.dual_aruco_detector import DualArucoDetector
        except ImportError:
            raise Exception("DualArucoDetector 모듈을 찾을 수 없습니다.")

        # DualArucoDetector 초기화
        detector = DualArucoDetector(
            vision_manager=self.vision_manager,
            marker_id1=tag_id1,
            marker_id2=tag_id2
        )
        detector.set_log_callback(self._log)

        # PlaneExtractor와 TCPCorrector import
        from services.plane_extractor import PlaneExtractor
        from services.tcp_corrector import TCPCorrector

        # PlaneExtractor 초기화
        extractor = PlaneExtractor(
            dual_detector=detector,
            target_samples=num_samples,
            max_attempts=num_samples * 3
        )

        # 평면 추출 실행
        self._log(f"듀얼 ArUco 평면 추출 시작 (Tag {tag_id1}, {tag_id2}, {num_samples}회, delay={delay_ms}ms, timeout={timeout}초)...")

        def get_frame():
            if delay_ms > 0:
                time.sleep(delay_ms / 1000.0)
            return self.camera_manager.get_frame()

        valid, attempts = extractor.collect_samples(
            frame_source=get_frame,
            on_progress=lambda curr, total: self._log(f"샘플 수집: {curr}/{total}")
        )

        plane_pose = extractor.get_plane_pose()

        if plane_pose is None:
            raise Exception(f"듀얼 ArUco 평면 추출 실패: 유효 샘플 부족 ({valid}/{attempts})")

        # TCP 보정값 계산
        corrector = TCPCorrector(target_rx=0, target_ry=0, target_rz=0)
        correction = corrector.compute_correction(plane_pose)

        self._log(f"평면 추출 성공: {valid}/{attempts}회 유효")
        self._log(f"평면 중심: X={plane_pose.x:.2f}, Y={plane_pose.y:.2f}, Z={plane_pose.z:.2f} mm")
        self._log(f"평면 자세: Rx={plane_pose.rx:.2f}, Ry={plane_pose.ry:.2f}, Rz={plane_pose.rz:.2f}°")
        self._log(f"TCP 보정: dRx={correction.delta_rx:.2f}, dRy={correction.delta_ry:.2f}, dRz={correction.delta_rz:.2f}°")

        # 현재 로봇 TCP 가져오기
        current_tcp = None
        if self.robot_controller:
            current_tcp = self.robot_controller.get_current_camera_pose()

        # 결과 표시 업데이트
        self.tabTaskEdit.update_plane_result(plane_pose, correction, current_tcp)

        QMessageBox.information(self, "성공",
            f"듀얼 ArUco 평면 추출 완료!\n"
            f"유효 샘플: {valid}/{attempts}\n"
            f"평면 중심: X={plane_pose.x:.2f}, Y={plane_pose.y:.2f}, Z={plane_pose.z:.2f} mm\n"
            f"평면 자세: Rx={plane_pose.rx:.2f}°, Ry={plane_pose.ry:.2f}°, Rz={plane_pose.rz:.2f}°")

    def _execute_align_aruco_pose(self, params: dict):
        """듀얼 ArUco 기반 자세 정렬 실행"""
        tag_id1 = params.get('tag_id1', 0)
        tag_id2 = params.get('tag_id2', 1)
        num_samples = params.get('num_samples', 20)
        target_rx = params.get('target_rx', 0.0)
        target_ry = params.get('target_ry', 0.0)
        target_rz = params.get('target_rz', 0.0)
        timeout = params.get('timeout', 30.0)

        # 로봇 연결 확인 (자세 정렬은 로봇 이동 필요)
        if not self.robot_controller or not self.robot_controller.is_connected:
            raise Exception("로봇이 연결되지 않았습니다. 자세 정렬에는 로봇 연결이 필요합니다.")

        # 카메라 매니저 확인
        if not self.camera_manager:
            raise Exception("카메라가 초기화되지 않았습니다. 비전 탭에서 카메라를 선택해주세요.")

        # 카메라가 꺼져 있으면 자동으로 켜기
        if not self.camera_manager.is_running:
            self._log("카메라가 꺼져 있습니다. 자동으로 카메라를 시작합니다...")
            success, msg = self.camera_manager.start()
            if not success:
                raise Exception(f"카메라 시작 실패: {msg}")
            self._log("카메라 시작 완료")

        # 모듈 import
        from services.dual_aruco_detector import DualArucoDetector
        from services.plane_extractor import PlaneExtractor
        from services.tcp_corrector import TCPCorrector

        # DualArucoDetector 초기화
        detector = DualArucoDetector(
            vision_manager=self.vision_manager,
            marker_id1=tag_id1,
            marker_id2=tag_id2
        )
        detector.set_log_callback(self._log)

        # PlaneExtractor 초기화
        extractor = PlaneExtractor(
            dual_detector=detector,
            target_samples=num_samples,
            max_attempts=num_samples * 3
        )

        self._log(f"자세 정렬 시작 (Tag {tag_id1}, {tag_id2}, {num_samples}회)...")
        self._log(f"목표 자세: Rx={target_rx:.2f}°, Ry={target_ry:.2f}°, Rz={target_rz:.2f}°")

        # 평면 추출
        def get_frame():
            time.sleep(0.1)  # 100ms delay
            return self.camera_manager.get_frame()

        valid, attempts = extractor.collect_samples(
            frame_source=get_frame,
            on_progress=lambda curr, total: self._log(f"샘플 수집: {curr}/{total}")
        )

        plane_pose = extractor.get_plane_pose()
        if plane_pose is None:
            raise Exception(f"평면 추출 실패: 유효 샘플 부족 ({valid}/{attempts})")

        # TCP 보정값 계산
        current_tcp = self.robot_controller.get_current_camera_pose()
        if current_tcp is None:
            raise Exception("현재 TCP 조회 실패")

        corrector = TCPCorrector(target_rx=target_rx, target_ry=target_ry, target_rz=target_rz)
        correction = corrector.compute_correction(plane_pose, current_tcp=current_tcp)

        self._log(f"평면 자세: Rx={plane_pose.rx:.2f}°, Ry={plane_pose.ry:.2f}°, Rz={plane_pose.rz:.2f}°")
        self._log(f"보정량: dRx={correction.delta_rx:.2f}°, dRy={correction.delta_ry:.2f}°, dRz={correction.delta_rz:.2f}°")

        # 최종 TCP 계산
        final_tcp = corrector.compute_final_tcp(current_tcp, correction)
        self._log(f"최종 TCP: Rx={final_tcp[3]:.2f}°, Ry={final_tcp[4]:.2f}°, Rz={final_tcp[5]:.2f}°")

        # 로봇 이동 (위치: mm→meter 변환, 회전: deg 그대로)
        x_m = final_tcp[0] / 1000.0
        y_m = final_tcp[1] / 1000.0
        z_m = final_tcp[2] / 1000.0
        result = self.robot_controller.move_to_pose(x_m, y_m, z_m, final_tcp[3], final_tcp[4], final_tcp[5])
        if not result.success:
            raise Exception(f"로봇 이동 실패: {result.message}")

        self._log("자세 정렬 완료!")

        # 결과 표시 업데이트
        self.tabTaskEdit.update_plane_result(plane_pose, correction, current_tcp)

        QMessageBox.information(self, "성공",
            f"자세 정렬 완료!\n"
            f"보정량: dRx={correction.delta_rx:.2f}°, dRy={correction.delta_ry:.2f}°, dRz={correction.delta_rz:.2f}°")

    def _on_align_center_from_tab(self, tag_id: int, num_samples: int):
        """중심 정렬 요청 (TabVision 시그널 핸들러)"""
        result = self.alignment_service.align_center(tag_id, num_samples)
        if not result.success:
            QMessageBox.warning(self, "경고", result.message)

    def _on_align_pose_from_tab(self, tag_id: int, num_samples: int):
        """자세 정렬 요청 (TabVision 시그널 핸들러)"""
        result = self.alignment_service.align_pose(tag_id, num_samples)
        if not result.success:
            QMessageBox.warning(self, "경고", result.message)

    def _on_align_full_from_tab(self, tag_id: int, num_samples: int):
        """전체 정렬 요청 (TabVision 시그널 핸들러)"""
        result = self.alignment_service.align_full(tag_id, num_samples)
        if not result.success:
            QMessageBox.warning(self, "경고", result.message)

    # ==================== 로봇 연결 ====================

    def _on_connect(self):
        """로봇 연결 토글 (메뉴/설정 탭에서 호출)"""
        # 설정 탭에서 IP/Port 가져오기
        ip = self.editSettingsRobotIP.text() if hasattr(self, 'editSettingsRobotIP') else "192.168.1.150"
        port = self.spinSettingsPort.value() if hasattr(self, 'spinSettingsPort') else 502
        self._connect_robot(ip, port)

    def _connect_robot(self, ip: str, port: int):
        """로봇 연결 공통 로직"""
        # 이미 연결된 경우 연결 해제
        if self.robot and self.robot.is_connected:
            self._on_disconnect()
            return

        self._log(f"연결 시도: {ip}:{port}")

        # 새 연결
        self.robot = ModbusClient(ip=ip, port=port, timeout=1.0)
        success, message = self.robot.connect()

        if success:
            self._log(message)

            # 공통 연결 설정
            self._setup_robot_connection()

            # 연결 후 현재 TCP 좌표 표시
            try:
                pose = self.robot.read_camera_pose()
                if pose:
                    x, y, z, rx, ry, rz = pose
                    self._log(f"현재 TCP 좌표: X={x:.1f} Y={y:.1f} Z={z:.1f} Rx={rx:.1f} Ry={ry:.1f} Rz={rz:.1f}")
            except Exception:
                pass
        else:
            self.tabTaskEdit.update_connection_status(False)
            self.statusbar.showMessage(message)
            self._log(message)
            QMessageBox.warning(self, "연결 실패", message)

    def _setup_robot_connection(self):
        """로봇 연결 후 공통 설정"""
        # Heartbeat 카운터 초기화
        self._heartbeat_failures = 0
        self._handling_connection_lost = False

        # RobotController 초기화
        self.robot_controller = RobotController(self.robot, self.pose_manager)
        self.robot_controller.set_on_error(lambda msg: self._log(f"[ERROR] {msg}"))

        # AlignmentService에 로봇 설정
        self.alignment_service.set_robot(self.robot)

        # PoseService에 로봇 설정
        self.pose_service.set_robot(self.robot, self.robot_controller)

        # Eye in Hand 탭에 로봇 설정
        self.tabEyeInHand.set_robot(self.robot)

        # Calibration 탭에 로봇 설정
        self.tabCalibration.set_robot(self.robot)

        # Motion Test 탭에 로봇 설정
        self.tabMotionTest.set_robot(self.robot)

        # ArUco 신뢰성 검증 탭에 로봇 설정
        self.tabArucoReliability.set_robot(self.robot)

        # Task 편집 탭의 연결 상태 업데이트
        self.tabTaskEdit.update_connection_status(True)

        # 상태 업데이트 타이머 시작
        self.status_timer.start(100)

        # 상태바에 연결 정보 및 Tool Frame 표시
        self._update_statusbar()

        # 현재 탭에 따라 Tool Frame 설정 (비전/캘리브 → TF1, ArUco → TF4)
        current_tab = self.tabWidget.currentIndex()
        if current_tab in [1, 2, 3]:
            try:
                tf = 4 if current_tab == 3 else 1
                success, msg = self.robot.send_set_toolframe(tf, wait=True)
                tab_name = "Vision" if current_tab == 1 else ("캘리브레이션" if current_tab == 2 else "ArUco 신뢰성 검증")
                if success:
                    self._log(f"{tab_name} 탭: Tool Frame {tf}으로 설정 완료")
                    if current_tab == 2:
                        self.tabCalibration.update_current_toolframe(tf)
                    # 상태바 업데이트
                    self._update_statusbar()
                else:
                    self._log(f"Tool Frame 설정 실패: {msg}")
            except Exception as e:
                self._log(f"Tool Frame 설정 오류: {e}")

    def _update_statusbar(self):
        """상태바 업데이트 - 연결 정보 및 Tool Frame 표시"""
        if not self.robot or not self.robot.is_connected:
            self.statusbar.showMessage("연결 안 됨")
            return

        # 현재 Tool Frame 읽기
        try:
            toolframe = self.robot.read_current_toolframe()
            if toolframe is not None:
                self.statusbar.showMessage(f"연결 성공: {self.robot.ip}:{self.robot.port} | TF: {toolframe}")
            else:
                self.statusbar.showMessage(f"연결 성공: {self.robot.ip}:{self.robot.port}")
        except Exception as e:
            self.statusbar.showMessage(f"연결 성공: {self.robot.ip}:{self.robot.port}")

    def _on_disconnect(self, reason="manual"):
        """로봇 연결 해제

        Args:
            reason: "manual" (수동 해제) 또는 "connection_lost" (네트워크 끊김)
        """
        if self.robot:
            if reason == "manual":
                success, message = self.robot.disconnect()
                self._log(message)
            # connection_lost인 경우 이미 mark_connection_lost() 호출됨

        # RobotController 해제
        self.robot_controller = None

        # AlignmentService 로봇 해제
        self.alignment_service.set_robot(None)

        # PoseService 로봇 해제
        self.pose_service.set_robot(None, None)

        # Eye in Hand 탭 로봇 해제
        self.tabEyeInHand.set_robot(None)

        # Calibration 탭 로봇 해제
        self.tabCalibration.set_robot(None)

        # Motion Test 탭 로봇 해제
        self.tabMotionTest.set_robot(None)

        # ArUco 신뢰성 검증 탭 로봇 해제
        self.tabArucoReliability.set_robot(None)

        # 타이머 정지
        self.status_timer.stop()

        # TabTaskEdit 연결 상태 업데이트
        self.tabTaskEdit.update_connection_status(False)

        # 상태바 메시지 구분
        if reason == "connection_lost":
            self.statusbar.showMessage("연결 끊김 감지 - 재연결 필요")
        else:
            self.statusbar.showMessage("연결 해제됨")

    def _ensure_toolframe(self, tf: int) -> bool:
        """지정 툴프레임 확인/설정. 성공 시 True 반환. PRS 클린업 대기 포함."""
        if not self.robot or not self.robot.is_connected:
            return False
        success, msg = self.robot.send_set_toolframe(tf, wait=True)
        if not success:
            self._log(f"TF{tf} 설정 실패: {msg}")
            return False
        time.sleep(0.2)  # PRS 클린업 대기 (레이스 컨디션 방지)
        return True

    def _on_jog_move_from_tab(self, axis: str, distance: float):
        """조그 이동 요청 (TabTaskEdit 시그널 핸들러)"""
        if not self._require_robot():
            return

        if axis not in ('x', 'y', 'z'):
            self._log(f"알 수 없는 축: {axis}")
            return

        # 베이스 좌표계 이동 (processEvents 콜백으로 UI 블로킹 방지)
        success, message = self.robot.send_base_linear(
            axis, distance,
            process_events_callback=QApplication.processEvents)
        if not success:
            self._log(f"조그 이동 실패: {message}")
            QMessageBox.warning(self, "오류", f"조그 이동 실패:\n{message}")
        else:
            self._log(f"조그 이동 완료: {axis.upper()} {'+' if distance > 0 else ''}{distance}mm")

    def _on_jog_rotate_from_tab(self, axis: str, angle: float):
        """조그 회전 요청 (TabTaskEdit 시그널 핸들러)"""
        if not self._require_robot():
            return

        # 베이스 좌표계 회전 (processEvents 콜백으로 UI 블로킹 방지)
        success, message = self.robot.send_base_rotate(
            axis, angle,
            process_events_callback=QApplication.processEvents)
        if not success:
            self._log(f"조그 회전 실패: {message}")
            QMessageBox.warning(self, "오류", f"조그 회전 실패:\n{message}")
        else:
            self._log(f"조그 회전 완료: {axis.upper()} {'+' if angle > 0 else ''}{angle}deg")

    def _on_ar_tag_align_base_ry(self, angle: float):
        """ArUco 정렬 탭 - Robot base 기준 Ry movel 보정"""
        if not self._require_robot():
            return
        try:
            self._log(f"[ArUco] Base Ry 보정: {angle:.2f}°")
            success, msg = self.robot.send_base_rotate(
                'ry', angle, wait=True,
                process_events_callback=QApplication.processEvents)
            if success:
                self._log(f"[ArUco] Base Ry 보정 완료")
            else:
                self._log(f"[ArUco] Base Ry 보정 실패: {msg}")
            self._update_statusbar()
        except Exception as e:
            self._log(f"[ArUco] Base Ry 보정 오류: {e}")

    def _on_ar_tag_align_base_rz(self, angle: float, distance: float):
        """ArUco 정렬 탭 - Robot base 기준 Rz movel + Y 보정"""
        if not self._require_robot():
            return
        try:
            import math

            current_pose = self.robot.read_current_pose()
            if current_pose is None:
                self._log("[ArUco] 현재 자세 읽기 실패")
                return

            x, y, z, rx, ry, rz = current_pose
            new_rz = rz + angle
            # Y 보정: ΔY = D × tan(ΔRz)
            dy = distance * math.tan(math.radians(angle))
            new_y = y - dy

            self._log(f"[ArUco] Rz 보정: ΔRz={angle:.2f}°, D={distance:.0f}mm, ΔY={dy:.2f}mm")
            self._log(f"[ArUco] 현재: Y={y:.1f}, Rz={rz:.1f} → 목표: Y={new_y:.1f}, Rz={new_rz:.1f}")

            success, msg = self.robot.send_move_to_pose(
                x, new_y, z, rx, ry, new_rz,
                wait=True, process_events_callback=QApplication.processEvents)
            if success:
                self._log(f"[ArUco] Rz + Y 보정 완료")
            else:
                self._log(f"[ArUco] Rz + Y 보정 실패: {msg}")
            self._update_statusbar()
        except Exception as e:
            self._log(f"[ArUco] Rz + Y 보정 오류: {e}")

    def _on_ar_tag_align_base_y(self, dy_px: float):
        """ArUco 정렬 탭 - 적응형 Base Y 위치 보정

        1단계: +5mm 테스트 이동 → px/mm 비율 산출 (부호 자동 결정)
        2단계: 비율 기반 보정 이동 (fine translate, 0.1mm 해상도)
        3단계: 재측정 검증

        Note:
            _ds435_adaptive_align()과 알고리즘 구조가 유사하나
            MAX_CORRECTION 초과 시 처리가 다름 (이쪽: 안전 중단,
            DS435: 2단계 분할 이동). 측정 함수도 상이하여 통합 보류.
        """
        if not self._require_robot():
            return

        DEAD_ZONE_PX = 3
        TEST_MM = 5.0
        MAX_CORRECTION_MM = 50.0

        if abs(dy_px) < DEAD_ZONE_PX:
            self._log("[ArUco] Base Y: 오프셋 3px 미만, 보정 불필요")
            return

        try:
            d0 = dy_px
            # 오프셋 반대 방향으로 테스트 (d0>0: 마커 오른쪽 → Y-, d0<0: 마커 왼쪽 → Y+)
            test_cmd = -TEST_MM if d0 > 0 else TEST_MM
            self._log(f"[ArUco] Base Y 보정 시작: d0={d0:.1f}px")

            # --- 1단계: 테스트 이동으로 px/mm 비율 산출 ---
            # 이동 전 로봇 Y 좌표 기록
            pose_before = self.robot.read_current_pose()
            if pose_before is None:
                self._log("[ArUco] 현재 포즈 읽기 실패")
                return
            y_before = pose_before[1]

            self._log(f"[ArUco] 1단계: {test_cmd:+.1f}mm 테스트 이동")
            success, msg = self.robot.send_base_linear(
                'y', test_cmd, wait=True,
                process_events_callback=QApplication.processEvents)
            if not success:
                self._log(f"[ArUco] 테스트 이동 실패: {msg}")
                return

            # 위치 안정화 대기: 이동 후 위치가 변하지 않을 때까지 폴링 (최대 10초)
            _, final_y = self._wait_for_position_stable(
                axis_idx=1, reference_val=y_before)
            actual_mm = final_y - y_before
            self._log(f"[ArUco] 실제 이동: {actual_mm:.2f}mm (명령: {test_cmd:+.1f}mm)")

            if abs(actual_mm) < 0.5:
                self._log("[ArUco] 실제 이동 < 0.5mm, 로봇 이동 불가. 복귀")
                self.robot.send_base_linear(
                    'y', -test_cmd, wait=True,
                    process_events_callback=QApplication.processEvents)
                return

            d1 = self._measure_marker_dy_px()
            if d1 is None:
                self._log("[ArUco] 테스트 후 마커 감지 실패, 복귀")
                self.robot.send_base_linear(
                    'y', -test_cmd, wait=True,
                    process_events_callback=QApplication.processEvents)
                return

            delta_px = d1 - d0
            self._log(f"[ArUco] 테스트 결과: d1={d1:.1f}px, 변화={delta_px:.1f}px")

            if abs(delta_px) < 2:
                self._log("[ArUco] 픽셀 변화 < 2px, 측정 불안정. 복귀")
                self.robot.send_base_linear(
                    'y', -test_cmd, wait=True,
                    process_events_callback=QApplication.processEvents)
                return

            # 실제 이동량 기반 비율 산출
            px_per_mm = delta_px / actual_mm
            self._log(f"[ArUco] 비율: {px_per_mm:.2f} px/mm ({abs(1.0/px_per_mm):.3f} mm/px)")

            # --- 2단계: 비율 기반 보정 이동 ---
            correction_mm = -d1 / px_per_mm
            self._log(f"[ArUco] 2단계: 보정 {correction_mm:.1f}mm")

            if abs(correction_mm) > MAX_CORRECTION_MM:
                self._log(f"[ArUco] 보정 과대 ({correction_mm:.1f}mm > {MAX_CORRECTION_MM}mm), 안전 중단")
                return

            # 0.1mm 해상도 정밀 이동 사용
            success, msg = self.robot.send_base_linear(
                'y', correction_mm, wait=True,
                process_events_callback=QApplication.processEvents)
            if not success:
                self._log(f"[ArUco] 보정 이동 실패: {msg}")
                return

            total_mm = test_cmd + correction_mm

            # --- 3단계: 이동완료 대기 후 검증 ---
            self._settle(0.5)

            d_final = self._measure_marker_dy_px()
            if d_final is not None:
                self._log(f"[ArUco] Base Y 보정 완료: 총 {total_mm:.1f}mm, 잔여={d_final:.1f}px")
            else:
                self._log(f"[ArUco] Base Y 보정 완료: 총 {total_mm:.1f}mm (검증 측정 실패)")

            self._update_statusbar()
        except Exception as e:
            self._log(f"[ArUco] Base Y 보정 오류: {e}")

    def _get_target_tag_ids(self):
        """ArUco 탭에서 타겟 마커 ID 쌍 반환"""
        return (self.tabArucoReliability.spinTagID1.value(),
                self.tabArucoReliability.spinTagID2.value())

    def _set_buttons_enabled(self, tab, button_names, enabled: bool):
        """탭의 버튼들 활성화/비활성화"""
        for btn_name in button_names:
            btn = getattr(tab, btn_name, None)
            if btn:
                btn.setEnabled(enabled)

    def _settle(self, delay=0.3):
        """이동 후 안정화 대기 + UI 이벤트 처리"""
        time.sleep(delay)
        QApplication.processEvents()

    def _get_effective_ry(self, alignment):
        """alignment에서 유효 Ry 반환 (3D 우선, fallback 2D)"""
        if alignment.angle_3d is not None:
            return alignment.angle_3d
        return alignment.angle_2d if alignment.angle_2d is not None else 0.0

    def _find_dual_marker_centers(self, markers, tag_id1, tag_id2):
        """마커 리스트에서 두 ID 검색 후 중심점 반환. 미발견 시 None"""
        m1, m2 = None, None
        for m in markers:
            if m['id'] == tag_id1:
                m1 = m
            elif m['id'] == tag_id2:
                m2 = m
        if m1 is None or m2 is None:
            return None
        c1 = m1['corners'][0].mean(axis=0) if len(m1['corners'].shape) == 3 else m1['corners'].mean(axis=0)
        c2 = m2['corners'][0].mean(axis=0) if len(m2['corners'].shape) == 3 else m2['corners'].mean(axis=0)
        return c1, c2

    def _get_arducam_intrinsics(self):
        """ArduCam intrinsics → (camera_matrix, dist_coeffs) numpy 배열 반환"""
        if not self.arducam_manager or not self.arducam_manager.intrinsics:
            return None, None
        intr = self.arducam_manager.intrinsics
        camera_matrix = np.array([
            [intr.fx, 0, intr.ppx],
            [0, intr.fy, intr.ppy],
            [0, 0, 1]
        ], dtype=np.float64)
        dist_coeffs = np.array(intr.coeffs, dtype=np.float64).reshape(1, -1)
        return camera_matrix, dist_coeffs

    def _detect_dual_alignment(self, frame, display_label=None, max_retries=1,
                               display_func=None, frame_source=None,
                               camera_matrix=None, dist_coeffs=None):
        """통합 ArUco 듀얼 마커 검출 + 오버레이 표시

        Args:
            frame: 입력 프레임 (None이면 None 반환)
            display_label: 오버레이 표시 대상 QLabel (None이면 표시 안함)
            max_retries: 검출 재시도 횟수
            display_func: 오버레이 표시 함수 (frame, label). None이면 display_frame_on_label 사용
            frame_source: 재시도 시 최신 프레임 획득 callable (None이면 최초 frame 재사용)
            camera_matrix: 카메라 행렬 (None이면 ArUco 탭 설정 사용)
            dist_coeffs: 왜곡 계수 (None이면 ArUco 탭 설정 사용)

        Returns:
            DualMarkerAlignmentResult or None
        """
        from utils.common import display_frame_on_label as _default_display

        if display_func is None:
            display_func = _default_display

        if camera_matrix is None:
            camera_matrix = self.tabArucoReliability.camera_matrix
        if dist_coeffs is None:
            dist_coeffs = self.tabArucoReliability.dist_coeffs
        tag_id1, tag_id2 = self._get_target_tag_ids()

        for attempt in range(max_retries):
            try:
                if attempt > 0:
                    QApplication.processEvents()
                cur_frame = frame_source() if (attempt > 0 and frame_source) else frame
                if cur_frame is None:
                    self._log("[Detect] 검출 실패: 프레임 없음")
                    if attempt < max_retries - 1:
                        time.sleep(0.2)
                        continue
                    return None
                work = cur_frame.copy()

                markers = self.vision_manager.detect_marker_centers(
                    work, camera_matrix, dist_coeffs, estimate_pose=True
                )

                h, w = work.shape[:2]
                alignment = compute_dual_alignment(markers, tag_id1, tag_id2, w, h)

                # 오버레이 표시
                if display_label is not None:
                    display = work.copy()
                    draw_dual_marker_overlay(display, markers, tag_id1, tag_id2, alignment)
                    display_func(display, display_label)

                if alignment is not None:
                    return alignment

                if attempt < max_retries - 1:
                    self._log(f"[Detect] 마커 미검출 (시도 {attempt+1}/{max_retries}), 재시도...")
                    time.sleep(0.3)
                else:
                    self._log(f"[Detect] 마커 {tag_id1}/{tag_id2} 미검출"
                              + (f" ({max_retries}회 실패)" if max_retries > 1 else ""))

            except Exception as e:
                self._log(f"[Detect] 검출 오류: {e}")
                return None

        return None

    def _wait_for_position_stable(self, axis_idx, reference_val=None,
                                  threshold=0.05, min_delta=0.1,
                                  stable_count_needed=3, timeout=10.0,
                                  cancel_callback=None):
        """로봇 이동 후 위치 안정화 대기

        Args:
            axis_idx: pose 배열 인덱스 (0=X, 1=Y, 2=Z)
            reference_val: 이동 전 좌표값 (None이면 첫 측정값 사용)
            threshold: 연속 측정 간 변화 임계값 (mm)
            min_delta: 이동 전 대비 최소 변화량 (mm, 0이면 변화량 체크 안함)
            stable_count_needed: 안정 판정 연속 횟수
            timeout: 최대 대기 시간 (초)
            cancel_callback: 취소 확인 callable (True 반환 시 중단)

        Returns:
            (stable: bool, final_value: float)
        """
        start = time.time()
        stable_count = 0
        pose = self.robot.read_current_pose()
        if pose is None:
            return False, 0.0
        last_val = pose[axis_idx]
        if reference_val is None:
            reference_val = last_val

        while time.time() - start < timeout:
            time.sleep(0.2)
            QApplication.processEvents()
            if cancel_callback and cancel_callback():
                return False, last_val
            pose_now = self.robot.read_current_pose()
            if pose_now is None:
                continue
            current_val = pose_now[axis_idx]
            if abs(current_val - last_val) < threshold:
                stable_count += 1
                if stable_count >= stable_count_needed:
                    if min_delta == 0 or abs(current_val - reference_val) > min_delta:
                        return True, current_val
            else:
                stable_count = 0
            last_val = current_val

        return False, last_val

    # ==================== ArUco 신뢰성 탭 통합 정렬 핸들러 ====================

    def _aruco_tab_detect_alignment(self):
        """ArUco 신뢰성 탭용 마커 검출.

        Returns:
            DualMarkerAlignmentResult or None
        """
        frame = getattr(self.tabArucoReliability, 'current_frame', None)
        label = getattr(self.tabArucoReliability, 'labelCameraView', None)
        return self._detect_dual_alignment(frame, label)

    def _set_aruco_align_buttons_enabled(self, enabled: bool):
        """통합 정렬 버튼 활성화/비활성화"""
        self._set_buttons_enabled(
            self.tabArucoReliability,
            ('btnAlignArucoY', 'btnAlignArucoX', 'btnAlignArucoCombined'),
            enabled)

    def _on_aruco_align_y(self):
        """ArUco 정렬 Y: Ry 보정 + Base Y 보정 (detect-correct-redetect 패턴)"""
        if not self._require_robot():
            return

        self._set_aruco_align_buttons_enabled(False)
        try:
            QApplication.processEvents()

            # 1) 검출
            alignment = self._aruco_tab_detect_alignment()
            if alignment is None:
                self._log("[ArUco Y] 정렬 실패: 마커 미검출")
                return

            # 2) Ry 보정 (>=0.5deg)
            active_ry = self._get_effective_ry(alignment)
            if active_ry is not None and abs(active_ry) >= 0.5:
                self._log(f"[ArUco Y] Ry 보정: {active_ry:.2f}°")
                self._on_ar_tag_align_base_ry(active_ry)
                self._settle()
            else:
                self._log(f"[ArUco Y] Ry 보정 불필요: {active_ry}°")

            # 3) 재검출 + Base Y 보정 (>=5px)
            alignment2 = self._aruco_tab_detect_alignment()
            if alignment2 is not None and alignment2.offset_y is not None:
                if abs(alignment2.offset_y) >= 5.0:
                    self._log(f"[ArUco Y] Y 보정: {alignment2.offset_y:.1f}px")
                    self._on_ar_tag_align_base_y(alignment2.offset_y)
                    self._settle()
                else:
                    self._log(f"[ArUco Y] Y 보정 불필요: {alignment2.offset_y:.1f}px")

            # 4) 최종 검출 + 결과
            alignment3 = self._aruco_tab_detect_alignment()
            if alignment3 is not None:
                ry_f = self._get_effective_ry(alignment3)
                self._log(f"[ArUco Y] 정렬 완료: Ry={ry_f:.2f}°, offset_y={alignment3.offset_y:.1f}px")
            else:
                self._log("[ArUco Y] 최종 검출 실패")

        except Exception as e:
            self._log(f"[ArUco Y] 오류: {e}")
        finally:
            self._set_aruco_align_buttons_enabled(True)

    def _on_aruco_align_x(self):
        """ArUco 정렬 X: Rz 보정 (세로축 깊이 차이 기반)"""
        if not self._require_robot():
            return

        self._set_aruco_align_buttons_enabled(False)
        try:
            QApplication.processEvents()

            # 1) 검출
            alignment = self._aruco_tab_detect_alignment()
            if alignment is None:
                self._log("[ArUco X] 정렬 실패: 마커 미검출")
                return

            # 2) Rz 보정 (>=0.3deg)
            if alignment.angle_rx is not None and abs(alignment.angle_rx) >= 0.3:
                tab = self.tabArucoReliability
                distance = getattr(tab, '_last_marker_distance', None)
                if distance is None:
                    self._log("[ArUco X] 마커 거리 정보 없음, 기본값 360mm 사용")
                    distance = 360.0

                self._log(f"[ArUco X] Rz 보정: {alignment.angle_rx:.2f}°, D={distance:.0f}mm")
                self._on_ar_tag_align_base_rz(alignment.angle_rx, distance)
                self._settle()
            else:
                rx_disp = alignment.angle_rx if alignment.angle_rx is not None else 0
                self._log(f"[ArUco X] Rz 보정 불필요: {rx_disp:.2f}°")

            # 3) 최종 검출 + 결과
            alignment2 = self._aruco_tab_detect_alignment()
            if alignment2 is not None:
                rx2 = alignment2.angle_rx if alignment2.angle_rx is not None else 0
                self._log(f"[ArUco X] 정렬 완료: Rz={rx2:.2f}°")
            else:
                self._log("[ArUco X] 최종 검출 실패")

        except Exception as e:
            self._log(f"[ArUco X] 오류: {e}")
        finally:
            self._set_aruco_align_buttons_enabled(True)

    def _on_aruco_align_combined(self):
        """통합 ArUco 정렬: Y축 먼저 → X축 (수평 정렬이 Rz 정확도에 영향)"""
        if not self._require_robot():
            return

        self._set_aruco_align_buttons_enabled(False)
        try:
            self._log("[통합] ArUco 통합 정렬 시작 (Y → X)")
            QApplication.processEvents()

            # 1) Y 정렬 (Ry + Base Y)
            alignment = self._aruco_tab_detect_alignment()
            if alignment is not None:
                active_ry = self._get_effective_ry(alignment)
                if active_ry is not None and abs(active_ry) >= 0.5:
                    self._log(f"[통합] Ry 보정: {active_ry:.2f}°")
                    self._on_ar_tag_align_base_ry(active_ry)
                    self._settle()

                alignment2 = self._aruco_tab_detect_alignment()
                if alignment2 is not None and alignment2.offset_y is not None and abs(alignment2.offset_y) >= 5.0:
                    self._log(f"[통합] Y 보정: {alignment2.offset_y:.1f}px")
                    self._on_ar_tag_align_base_y(alignment2.offset_y)
                    self._settle()

            # 2) X 정렬 (Rz)
            alignment3 = self._aruco_tab_detect_alignment()
            if alignment3 is not None and alignment3.angle_rx is not None and abs(alignment3.angle_rx) >= 0.3:
                tab = self.tabArucoReliability
                distance = getattr(tab, '_last_marker_distance', None) or 360.0
                self._log(f"[통합] Rz 보정: {alignment3.angle_rx:.2f}°, D={distance:.0f}mm")
                self._on_ar_tag_align_base_rz(alignment3.angle_rx, distance)
                self._settle()

            # 3) 최종 결과
            final = self._aruco_tab_detect_alignment()
            if final is not None:
                ry_f = self._get_effective_ry(final)
                rx_f = final.angle_rx if final.angle_rx is not None else 0
                oy_f = final.offset_y if final.offset_y is not None else 0
                self._log(f"[통합] 정렬 완료: Ry={ry_f:.2f}°, Rz={rx_f:.2f}°, offset_y={oy_f:.1f}px")
            else:
                self._log("[통합] 최종 검출 실패")

        except Exception as e:
            self._log(f"[통합] 오류: {e}")
        finally:
            self._set_aruco_align_buttons_enabled(True)

    def _measure_marker_dy_px(self):
        """현재 카메라 프레임에서 마커 중점의 dY 픽셀 오프셋 측정"""
        try:
            frame = self.camera_manager.get_frame()
            if frame is None:
                return None

            tag_id1, tag_id2 = self._get_target_tag_ids()

            markers = self.vision_manager.detect_marker_centers(frame)
            m1, m2 = None, None
            for m in markers:
                if m['id'] == tag_id1:
                    m1 = m
                elif m['id'] == tag_id2:
                    m2 = m

            if m1 is None or m2 is None:
                return None

            h, w = frame.shape[:2]
            mid_px_x = (m1['center'][0] + m2['center'][0]) / 2.0
            return mid_px_x - w / 2.0  # 양수=오른쪽
        except Exception as e:
            self._log(f"[ArUco] 마커 측정 오류: {e}")
            return None

    def _fine_align_axis(self, axis, px_per_mm, measure_fn, log_prefix="[FineAlign]"):
        """ArduCam 미세 정렬: 0.1mm씩 이동하며 수렴.

        coarse 보정 후 잔여 오프셋이 3px 이내일 때 호출.
        3-샘플 평균 측정, 포스트-무브 부호 반전 감지로 발산 방지.

        Args:
            axis: 'y' 또는 'z'
            px_per_mm: coarse 단계에서 산출된 px/mm 비율 (부호 포함)
            measure_fn: () -> Optional[float] — 현재 오프셋(px) 반환
            log_prefix: 로그 접두사

        Returns:
            최종 잔여 오프셋(px), 또는 None (측정 실패)

        Note:
            px/mm < 8 인 경우 (DS435 등) 미세 정렬 효과 없으므로 건너뜀.
        """
        STEP_MM = 0.1
        CONVERGE_PX = 1.5
        ENTRY_PX = 10.0
        MAX_ITER = 20
        TIMEOUT_S = 30.0
        N_SAMPLES = 3
        SAMPLE_DELAY = 0.15  # ArduCam 10fps → 100ms/frame, 150ms로 fresh frame 보장

        # px/mm 하한 검사 (DS435 등 저해상도 보호)
        if abs(px_per_mm) < 8.0:
            self._log(f"{log_prefix} px/mm={px_per_mm:.1f} < 8, 미세 정렬 건너뜀")
            return None

        def averaged_measure():
            """N-샘플 평균 측정"""
            samples = []
            for _ in range(N_SAMPLES):
                QApplication.processEvents()
                time.sleep(SAMPLE_DELAY)
                val = measure_fn()
                if val is not None:
                    samples.append(val)
            if not samples:
                return None
            return sum(samples) / len(samples)

        # 초기 측정
        offset = averaged_measure()
        if offset is None:
            self._log(f"{log_prefix} 초기 측정 실패")
            return None

        if abs(offset) > ENTRY_PX:
            self._log(f"{log_prefix} 잔여={offset:.1f}px > {ENTRY_PX}px, 미세 정렬 불필요")
            return offset

        self._log(f"{log_prefix} 미세 정렬 시작: 잔여={offset:.1f}px, px/mm={px_per_mm:.2f}")

        # 이동 방향: offset > 0 이고 px_per_mm > 0 이면 음의 방향으로 이동
        direction = -1.0 if (offset * px_per_mm) > 0 else 1.0

        prev_sign = None
        reversal_count = 0
        start_time = time.time()

        for i in range(MAX_ITER):
            if time.time() - start_time > TIMEOUT_S:
                self._log(f"{log_prefix} 타임아웃 ({TIMEOUT_S}s)")
                break

            if abs(offset) <= CONVERGE_PX:
                self._log(f"{log_prefix} 수렴 완료: {offset:.1f}px (반복 {i})")
                return offset

            # 0.1mm 이동 (미세 이동은 PRS가 ~50ms 내 완료 → Running 감지 불가)
            # wait=False + 고정 sleep으로 대체
            move_mm = direction * STEP_MM
            success, msg = self.robot.send_base_linear(
                axis, move_mm, wait=False)
            if not success:
                self._log(f"{log_prefix} 이동 실패: {msg}")
                break
            # 포스트-무브 측정 (0.1mm 이동은 PRS ~50ms 완료 → 0.3s 여유)
            self._settle(0.3)
            new_offset = averaged_measure()
            if new_offset is None:
                self._log(f"{log_prefix} 측정 실패 (반복 {i+1})")
                break

            # 부호 반전 감지 (발산 방지)
            curr_sign = 1 if new_offset > 0 else (-1 if new_offset < 0 else 0)
            if curr_sign != 0 and prev_sign is not None and prev_sign != 0:
                if curr_sign != prev_sign:
                    reversal_count += 1
                    if reversal_count >= 2:
                        self._log(f"{log_prefix} 부호 반전 2회, 발산 중단: {new_offset:.1f}px")
                        return new_offset
                else:
                    reversal_count = 0
            prev_sign = curr_sign

            # 방향 갱신: 오프셋이 줄었으면 같은 방향 유지, 늘었으면 반전
            if abs(new_offset) > abs(offset):
                direction = -direction
                self._log(f"{log_prefix} [{i+1}] 오프셋 증가 {offset:.1f}→{new_offset:.1f}px, 방향 반전")

            offset = new_offset

        self._log(f"{log_prefix} 미세 정렬 종료: 잔여={offset:.1f}px (반복 {i+1})")
        return offset

    def _on_ar_tag_align_single_axis(self, axis: str, angle: float):
        """AR Tag TCP Align - 개별 축 tool.rot 테스트"""
        if not self._require_robot():
            return
        tf_changed = False
        try:
            dbg = self.tabArucoReliability.txtAlignDebug

            # Vision→Robot 축 매핑 (카메라 마운트 기준)
            axis_map = {'rx': 'ry', 'ry': 'rz', 'rz': 'rx'}
            robot_axis = axis_map.get(axis.lower(), axis)

            # TF4로 변경
            success, msg = self.robot.send_set_toolframe(4, wait=True)
            if not success:
                self._log(f"[AR Tag] TF4 설정 실패: {msg}")
                return
            tf_changed = True
            self._log("[AR Tag] TF4 설정 완료")
            time.sleep(0.2)

            # 정렬 전 자세
            before = self.robot.read_camera_pose()
            if before:
                dbg.append(f"── Vision {axis.upper()} → Robot {robot_axis.upper()} ({angle:.2f}°) ──")
                dbg.append(f"  전) Rx={before[3]:.2f}, Ry={before[4]:.2f}, Rz={before[5]:.2f}")

            self._log(f"[AR Tag] Vision {axis.upper()} → tool.rot{robot_axis[-1]}({angle:.2f}°) 실행")
            success, msg = self.robot.send_tcp_rotate(
                robot_axis, angle, wait=True,
                process_events_callback=QApplication.processEvents)
            if not success:
                dbg.append(f"  실패: {msg}")
                self._log(f"[AR Tag] {axis.upper()} 실패: {msg}")
                return

            # 정렬 후 자세
            after = self.robot.read_camera_pose()
            if after and before:
                dbg.append(f"  후) Rx={after[3]:.2f}, Ry={after[4]:.2f}, Rz={after[5]:.2f}")
                dbg.append(f"  Δ ) dRx={after[3]-before[3]:.2f}, dRy={after[4]-before[4]:.2f}, dRz={after[5]-before[5]:.2f}")

            self._log(f"[AR Tag] {axis.upper()} 완료")
        except Exception as e:
            self._log(f"[AR Tag] 오류: {e}")
        finally:
            if tf_changed:
                self._ensure_toolframe(3)
            self._update_statusbar()

    def _on_ar_tag_align_parallel(self, drx: float, dry: float, drz: float):
        """AR Tag TCP Align - TF4 기준 tool.rot 회전으로 마커 평행 정렬"""
        if not self._require_robot():
            return

        tf_changed = False
        try:
            dbg = self.tabArucoReliability.txtAlignDebug

            # TF4로 변경
            success, msg = self.robot.send_set_toolframe(4, wait=True)
            if not success:
                self._log(f"[AR Tag] TF4 설정 실패: {msg}")
                QMessageBox.warning(self, "오류", f"TF4 설정 실패:\n{msg}")
                return
            tf_changed = True
            self._log("[AR Tag] TF4 설정 완료")
            time.sleep(0.2)  # PRS 클린업 대기

            # 정렬 전 자세
            before = self.robot.read_camera_pose()
            if before:
                dbg.append(f"══ 전체 정렬 (dRx={drx:.2f}, dRy={dry:.2f}, dRz={drz:.2f}) ══")
                dbg.append(f"  전) X={before[0]:.1f}, Y={before[1]:.1f}, Z={before[2]:.1f}")
                dbg.append(f"      Rx={before[3]:.2f}, Ry={before[4]:.2f}, Rz={before[5]:.2f}")

            # Vision→Robot 축 매핑 적용 후 tool.rot 순차 실행
            axis_map = {'rx': 'ry', 'ry': 'rz', 'rz': 'rx'}
            self._log(f"[AR Tag] 회전 보정 실행: dRx={drx:.2f}, dRy={dry:.2f}, dRz={drz:.2f}°")
            for vision_axis, angle in [('rx', drx), ('ry', dry), ('rz', drz)]:
                if abs(angle) < 0.5:
                    dbg.append(f"  Vision {vision_axis.upper()}={angle:.2f}° → 스킵 (<0.5)")
                    continue
                robot_axis = axis_map[vision_axis]
                success, msg = self.robot.send_tcp_rotate(
                    robot_axis, angle, wait=True,
                    process_events_callback=QApplication.processEvents)
                if not success:
                    dbg.append(f"  {vision_axis.upper()} 실패: {msg}")
                    self._log(f"[AR Tag] {vision_axis.upper()} 회전 실패: {msg}")
                    QMessageBox.warning(self, "오류", f"{vision_axis.upper()} 회전 실패:\n{msg}")
                    return
                dbg.append(f"  {vision_axis.upper()} → {round(angle, 1)}° 완료")
                time.sleep(0.2)  # PRS 클린업 대기

            # 정렬 후 자세
            after = self.robot.read_camera_pose()
            if after and before:
                dbg.append(f"  후) X={after[0]:.1f}, Y={after[1]:.1f}, Z={after[2]:.1f}")
                dbg.append(f"      Rx={after[3]:.2f}, Ry={after[4]:.2f}, Rz={after[5]:.2f}")
                dbg.append(f"  Δ ) dRx={after[3]-before[3]:.2f}, dRy={after[4]-before[4]:.2f}, dRz={after[5]-before[5]:.2f}")
                dbg.append("")

            self._log("[AR Tag] 마커 평행 정렬 완료")
        except Exception as e:
            self._log(f"[AR Tag] 정렬 오류: {e}")
            QMessageBox.warning(self, "오류", f"정렬 실패:\n{e}")
        finally:
            if tf_changed:
                self._ensure_toolframe(3)
            self._update_statusbar()

    def _update_robot_status(self):
        """로봇 상태 업데이트 (TCP 위치, 레지스터 등) + heartbeat 감시"""
        if not self.robot or not self.robot.is_connected:
            return

        # --- heartbeat: 사이클 단위 실패 감지 ---
        any_success = False

        # 카메라 포즈 읽기 (158~169)
        cam_pose = self.robot.read_camera_pose()
        if cam_pose:
            any_success = True
            x, y, z, rx, ry, rz = cam_pose
            # TabTaskEdit에 TCP 위치 업데이트
            self.tabTaskEdit.update_tcp_position(x, y, z, rx, ry, rz)
            # TabCalibration에 로봇 좌표 업데이트
            self.tabCalibration.update_robot_position(x, y, z, rx, ry, rz)
            # TabMotionTest에 로봇 좌표 업데이트
            self.tabMotionTest.update_robot_position(x, y, z, rx, ry, rz)

        # 현재 툴프레임 읽기 (레지스터 219)
        toolframe = self.robot.read_current_toolframe()
        if toolframe is not None:
            any_success = True
            self.tabMotionTest.update_current_toolframe(toolframe)
            self.tabTaskEdit.update_current_toolframe(toolframe)
            self.tabCalibration.update_current_toolframe(toolframe)

        # 커맨드/응답 레지스터 읽기
        cmd = self.robot.read_command()
        resp = self.robot.read_response()
        if cmd is not None or resp is not None:
            any_success = True

        # Modbus 모니터 테이블 업데이트 (실행 모니터 탭)
        if hasattr(self, 'tableModbusRegisters'):
            # Command row
            if cmd is not None:
                self.tableModbusRegisters.setItem(2, 1,
                    QTableWidgetItem(str(cmd)))
            # Response row
            if resp is not None:
                self.tableModbusRegisters.setItem(3, 1,
                    QTableWidgetItem(str(resp)))

        # --- heartbeat: 사이클 결과 판정 ---
        if any_success:
            self._heartbeat_failures = 0
        else:
            self._heartbeat_failures += 1
            if self._heartbeat_failures >= self._heartbeat_max_failures:
                self._handle_connection_lost()
                return

        # --- keepalive: 60초간 명령 없으면 현재 위치로 movel ---
        if (any_success and cam_pose and
                time.time() - self.robot._last_command_time > 60):
            if resp == 0:  # Idle
                regs = [self.robot.to_uint16(int(round(v * 10))) for v in cam_pose]
                self.robot.write_registers(self.robot.REGISTER_POSE_MAIN, regs)
                self.robot.write_command(self.robot.CMD_MOVE_TO_POSE)
                self._log("[Keepalive] 현재 위치로 movel 전송 (idle 60초 경과)")

    def _handle_connection_lost(self):
        """네트워크 연결 끊김 감지 시 처리 (재진입 방지)"""
        if self._handling_connection_lost:
            return
        self._handling_connection_lost = True

        try:
            # 1. status_timer 즉시 정지 (추가 _update_robot_status 호출 차단)
            self.status_timer.stop()

            # 2. ModbusClient에 연결 끊김 알림
            if self.robot:
                self.robot.mark_connection_lost()

            # 3. 로그 기록
            self._log("[경고] 로봇 연결 끊김 감지! (연속 5사이클 통신 실패)")

            # 4. 기존 disconnect 로직 재사용 (reason 구분)
            self._on_disconnect(reason="connection_lost")

            # 5. 비차단 알림 (QTimer.singleShot으로 현재 이벤트 루프 밖에서 실행)
            QTimer.singleShot(0, self._show_connection_lost_warning)
        finally:
            self._handling_connection_lost = False

    def _show_connection_lost_warning(self):
        """연결 끊김 경고 (비차단, 이벤트 루프 밖에서 실행)"""
        QMessageBox.warning(self, "연결 끊김",
            "로봇과의 Modbus TCP 연결이 끊어졌습니다.\n"
            "네트워크 상태를 확인하고 다시 연결해주세요.")

    # ==================== 카메라 선택 ====================

    def _connect_camera_selection_signals(self):
        """각 탭의 카메라 선택 라디오 버튼 시그널 연결"""
        # TabVision
        if hasattr(self.tabVision, 'radioDS435'):
            self.tabVision.radioDS435.toggled.connect(
                lambda checked: self._on_camera_type_changed(CAMERA_DS435) if checked else None
            )
        if hasattr(self.tabVision, 'radioArduCam'):
            self.tabVision.radioArduCam.toggled.connect(
                lambda checked: self._on_camera_type_changed(CAMERA_ARDUCAM) if checked else None
            )

        # TabCalibration
        if hasattr(self.tabCalibration, 'radioDS435'):
            self.tabCalibration.radioDS435.toggled.connect(
                lambda checked: self._on_camera_type_changed(CAMERA_DS435) if checked else None
            )
        if hasattr(self.tabCalibration, 'radioArduCam'):
            self.tabCalibration.radioArduCam.toggled.connect(
                lambda checked: self._on_camera_type_changed(CAMERA_ARDUCAM) if checked else None
            )

        # TabArucoReliability
        if hasattr(self.tabArucoReliability, 'radioDS435'):
            self.tabArucoReliability.radioDS435.toggled.connect(
                lambda checked: self._on_camera_type_changed(CAMERA_DS435) if checked else None
            )
        if hasattr(self.tabArucoReliability, 'radioArduCam'):
            self.tabArucoReliability.radioArduCam.toggled.connect(
                lambda checked: self._on_camera_type_changed(CAMERA_ARDUCAM) if checked else None
            )

        # TabEyeInHand
        if hasattr(self.tabEyeInHand, 'radioDS435'):
            self.tabEyeInHand.radioDS435.toggled.connect(
                lambda checked: self._on_camera_type_changed(CAMERA_DS435) if checked else None
            )
        if hasattr(self.tabEyeInHand, 'radioArduCam'):
            self.tabEyeInHand.radioArduCam.toggled.connect(
                lambda checked: self._on_camera_type_changed(CAMERA_ARDUCAM) if checked else None
            )


    def _on_camera_type_changed(self, camera_type: str):
        """카메라 타입 변경 시 호출"""
        if camera_type == self._current_camera_type:
            return

        # 현재 카메라가 실행 중이면 정지
        was_running = self.camera_manager.is_running
        if was_running:
            self.camera_manager.stop()

        # 카메라 타입 변경
        self._current_camera_type = camera_type

        # 새 카메라 매니저로 전환
        if camera_type == CAMERA_DS435:
            self.camera_manager = self.ds435_camera_manager
        else:
            self.camera_manager = self.arducam_manager

        # 탭들에 새 카메라 매니저 전달
        self.tabCalibration.set_camera_manager(self.camera_manager)
        self.tabEyeInHand.set_camera_manager(self.camera_manager)
        self.tabArucoReliability.set_camera_manager(self.camera_manager)

        # Vision 매니저에 새 카메라 매니저 설정
        self.vision_manager.set_camera_manager(self.camera_manager)

        self._log(f"카메라 변경: {camera_type}")

        # 모든 탭의 라디오 버튼 동기화
        self._sync_camera_radio_buttons(camera_type)

        # 이전에 실행 중이었으면 새 카메라 시작
        if was_running:
            self._on_start_camera()

    def _sync_camera_radio_buttons(self, camera_type: str):
        """모든 탭의 카메라 선택 라디오 버튼 동기화"""
        is_ds435 = camera_type == CAMERA_DS435

        # 시그널 블로킹하여 무한 루프 방지
        tabs = [self.tabVision, self.tabCalibration, self.tabArucoReliability, self.tabEyeInHand]

        for tab in tabs:
            if hasattr(tab, 'radioDS435') and hasattr(tab, 'radioArduCam'):
                tab.radioDS435.blockSignals(True)
                tab.radioArduCam.blockSignals(True)

                tab.radioDS435.setChecked(is_ds435)
                tab.radioArduCam.setChecked(not is_ds435)

                tab.radioDS435.blockSignals(False)
                tab.radioArduCam.blockSignals(False)

    # ==================== 비전 ====================

    def _on_start_camera(self):
        """카메라 시작"""
        if not self.camera_manager.is_available:
            self._log("RealSense 라이브러리가 설치되지 않았습니다.")
            QMessageBox.warning(self, "경고", "pyrealsense2가 설치되지 않았습니다.")
            return

        success, msg = self.camera_manager.start()
        if not success:
            QMessageBox.critical(self, "오류", msg)

    def _on_stop_camera(self):
        """카메라 정지"""
        self.camera_manager.stop()

    # ==================== 레이저 캘리브레이션 핸들러 ====================

    def _on_calib_align_aruco(self):
        """ArUco 정렬 버튼: 검출 → Ry 보정 → Y 중심 정렬"""
        self._cached_error_per_mm = None  # 정렬 변경 → 감도 캐시 무효화
        if not self._require_robot():
            return

        tab = self.tabLaserCalibration
        tab._update_calib_step(1, "ArUco 마커 검출 중...")

        try:
            # 1) 검출
            result = self._calib_detect_aruco_alignment()
            if result is None:
                tab._update_calib_step(0, "ArUco 검출 실패 - 마커를 확인하세요")
                return
            angle_ry, offset_y = result

            # 2) Ry 보정 (0.5° 이상)
            if abs(angle_ry) >= 0.5:
                tab._update_calib_step(1, f"Ry 보정 중: {angle_ry:.2f}°")
                self._on_ar_tag_align_base_ry(angle_ry)
                self._settle()
                self._log(f"[Calib] Ry 보정 완료: {angle_ry:.2f}°")
            else:
                self._log(f"[Calib] Ry 보정 불필요: {angle_ry:.2f}°")

            # 3) 재검출 후 Y 중심 정렬 (5px 이상)
            result2 = self._calib_detect_aruco_alignment()
            if result2 is not None:
                _, offset_y2 = result2
                if abs(offset_y2) >= 5.0:
                    tab._update_calib_step(1, f"Y 보정 중: {offset_y2:.1f}px")
                    self._on_ar_tag_align_base_y(offset_y2)
                    self._settle()
                    self._log(f"[Calib] Y 보정 완료: {offset_y2:.1f}px")
                else:
                    self._log(f"[Calib] Y 보정 불필요: {offset_y2:.1f}px")

            # 4) ArUco 오버레이 ON + 최종 결과 표시
            tab._show_aruco_overlay = True
            result3 = self._calib_detect_aruco_alignment()
            if result3 is not None:
                ry_f, oy_f = result3
                msg = f"정렬 완료: Ry={ry_f:.2f}°, offset_y={oy_f:.1f}px"
                self._log(f"[Calib] {msg}")
                tab._update_calib_step(0, msg)
            else:
                tab._update_calib_step(0, "정렬 후 재검출 실패")

        except Exception as e:
            self._log(f"[Calib] ArUco 정렬 오류: {e}")
            tab._update_calib_step(0, f"오류: {e}")

    def _calib_detect_aruco_alignment(self):
        """자동 캘리브레이션용 ArUco 마커 정렬값 검출 + 카메라 뷰에 시각화.

        Returns:
            (angle_ry, offset_y) tuple, or None if detection fails.
            angle_ry: Ry 보정 각도 (°), offset_y: 이미지 중심 대비 마커 중점 X 오프셋 (px)
        """
        frame = getattr(self.tabLaserCalibration, '_raw_frame', None)
        if frame is None:
            frame = getattr(self.tabLaserCalibration, 'current_frame', None)
        label = getattr(self.tabLaserCalibration, 'labelCameraView', None)
        ac_mat, ac_dist = self._get_arducam_intrinsics()
        result = self._detect_dual_alignment(frame, label,
                                             camera_matrix=ac_mat, dist_coeffs=ac_dist)
        if result is None:
            return None
        active_ry = self._get_effective_ry(result)
        return (active_ry, result.offset_y)

    # ==================== 스테레오 캘리브레이션 핸들러 ====================

    def _stereo_align_z_core(self, tab, detect_func=None, status_func=None):
        """스테레오 Z축 세로 정렬 코어 로직 (버튼/예외 관리 없음)

        테스트 이동 → px/mm 산출 → 보정 이동 → 검증.

        Args:
            tab: 탭 객체 (status_func 미지정 시 tab._update_calib_step 사용)
            detect_func: 정렬 검출 함수 (기본: _stereo_detect_full_alignment)
            status_func: 상태 표시 함수 (step, msg) (기본: tab._update_calib_step)

        Returns:
            (success: bool, msg: str)
        """
        if detect_func is None:
            detect_func = self._stereo_detect_full_alignment
        if status_func is None:
            status_func = tab._update_calib_step

        DEAD_ZONE_PX = 5
        TEST_MM = 3.0
        MAX_CORRECTION_MM = 30.0

        status_func(1, "ArUco 정렬 Y: 마커 검출 중...")

        alignment = detect_func()
        if alignment is None:
            return (False, "ArUco 검출 실패")

        # offset_z = mid_y - img_cy (이미지 세로 오프셋, 양수=아래)
        pz0 = alignment.offset_z
        if pz0 is None or abs(pz0) < DEAD_ZONE_PX:
            pz_disp = pz0 if pz0 is not None else 0
            self._log(f"[StereoCalib Y] Z 보정 불필요: offset_z={pz_disp:.1f}px")
            return (True, f"Y 정렬 완료 (offset_z {pz_disp:.1f}px < {DEAD_ZONE_PX}px)")

        self._log(f"[StereoCalib Y] 세로 오프셋: offset_z={pz0:.1f}px, "
                  f"offset_y={alignment.offset_y:.1f}px, "
                  f"mid=({alignment.mid_x:.0f},{alignment.mid_y:.0f})")

        # 1) 테스트 이동: Z +TEST_MM
        status_func(1, f"테스트 이동: Z +{TEST_MM:.1f}mm")
        success, msg = self.robot.send_base_linear(
            'z', TEST_MM, wait=True,
            process_events_callback=QApplication.processEvents)
        if not success:
            self._log(f"[StereoCalib Y] 테스트 이동 실패: {msg}")
            return (False, f"이동 실패: {msg}")
        # 프레임 갱신 대기 (ArduCam 새 프레임 보장)
        for _ in range(10):
            time.sleep(0.1)
            QApplication.processEvents()

        # 2) 재검출 → px/mm 비율 산출
        alignment2 = detect_func()
        if alignment2 is None or alignment2.offset_z is None:
            self._log("[StereoCalib Y] 재검출 실패, 원위치 복귀")
            self.robot.send_base_linear('z', -TEST_MM, wait=True,
                process_events_callback=QApplication.processEvents)
            return (False, "재검출 실패")

        pz1 = alignment2.offset_z
        delta_pz = pz1 - pz0
        delta_py = alignment2.offset_y - alignment.offset_y
        self._log(f"[StereoCalib Y] 테스트 후: offset_z={pz1:.1f}px (delta={delta_pz:.1f}), "
                  f"offset_y={alignment2.offset_y:.1f}px (delta={delta_py:.1f}), "
                  f"mid=({alignment2.mid_x:.0f},{alignment2.mid_y:.0f})")

        if abs(delta_pz) < 1.0:
            self._log(f"[StereoCalib Y] offset_z 변화 미미 ({delta_pz:.1f}px), 원위치 복귀")
            self.robot.send_base_linear('z', -TEST_MM, wait=True,
                process_events_callback=QApplication.processEvents)
            return (False, f"Z 감도 부족 (delta_z={delta_pz:.1f}, delta_y={delta_py:.1f})")

        px_per_mm = delta_pz / TEST_MM
        correction_mm = -pz1 / px_per_mm

        if abs(correction_mm) > MAX_CORRECTION_MM:
            correction_mm = MAX_CORRECTION_MM if correction_mm > 0 else -MAX_CORRECTION_MM

        self._log(f"[StereoCalib Y] px/mm={px_per_mm:.2f}, 보정: {correction_mm:.2f}mm")

        # 3) 보정 이동
        status_func(1, f"Z 보정: {correction_mm:.1f}mm")
        success2, msg2 = self.robot.send_base_linear(
            'z', correction_mm, wait=True,
            process_events_callback=QApplication.processEvents)
        if not success2:
            self._log(f"[StereoCalib Y] 보정 이동 실패: {msg2}")
        for _ in range(5):
            time.sleep(0.1)
            QApplication.processEvents()

        # 4) 최종 결과
        final = detect_func()
        if final is not None:
            oz_f = final.offset_z if final.offset_z is not None else 0
            result_msg = f"Y 정렬 완료: offset_z={oz_f:.1f}px"
            self._log(f"[StereoCalib Y] {result_msg}")
            return (True, result_msg)
        else:
            return (False, "최종 검출 실패")

    def _on_stereo_calib_align_aruco(self):
        """스테레오 탭 ArUco 정렬 Y: Z축 이동으로 이미지 세로(mid_y) 중심 정렬."""
        if not self._require_robot():
            return

        tab = self.tabStereoCalibration
        self._set_stereo_align_buttons_enabled(False)
        try:
            QApplication.processEvents()
            success, msg = self._stereo_align_z_core(tab)
            tab._update_calib_step(0, msg)
        except Exception as e:
            self._log(f"[StereoCalib Y] 오류: {e}")
            tab._update_calib_step(0, f"오류: {e}")
        finally:
            self._set_stereo_align_buttons_enabled(True)

    def _stereo_detect_aruco_alignment(self):
        """스테레오 탭 ArduCam 프레임에서 ArUco 정렬값 검출 + 시각화.

        Returns:
            (angle_ry, offset_y) tuple, or None if detection fails.
        """
        frame = getattr(self.tabStereoCalibration, 'current_frame', None)
        label = getattr(self.tabStereoCalibration, 'labelArduCamView', None)
        ac_mat, ac_dist = self._get_arducam_intrinsics()
        result = self._detect_dual_alignment(
            frame, label,
            display_func=self.tabStereoCalibration._display_fixed,
            camera_matrix=ac_mat, dist_coeffs=ac_dist)
        if result is None:
            return None
        active_ry = self._get_effective_ry(result)
        return (active_ry, result.offset_y)

    def _stereo_detect_full_alignment(self, max_retries=3):
        """스테레오 탭 ArduCam 프레임에서 전체 정렬 결과 반환.

        Args:
            max_retries: 검출 실패 시 재시도 횟수 (기본 3회)

        Returns:
            DualMarkerAlignmentResult or None
        """
        frame = getattr(self.tabStereoCalibration, 'current_frame', None)
        label = getattr(self.tabStereoCalibration, 'labelArduCamView', None)
        ac_mat, ac_dist = self._get_arducam_intrinsics()
        return self._detect_dual_alignment(
            frame, label, max_retries=max_retries,
            display_func=self.tabStereoCalibration._display_fixed,
            frame_source=lambda: getattr(self.tabStereoCalibration, 'current_frame', None),
            camera_matrix=ac_mat, dist_coeffs=ac_dist)

    def _set_stereo_align_buttons_enabled(self, enabled: bool):
        """스테레오 탭 정렬 버튼 활성화/비활성화"""
        self._set_buttons_enabled(
            self.tabStereoCalibration,
            ('btnAlignArucoY', 'btnAlignArucoX', 'btnAlignArucoCombined',
             'btnAlignDS435', 'btnHandoffDS435ToArduCam'),
            enabled)

    def _stereo_align_y_core(self, tab, detect_func=None, status_func=None):
        """스테레오 Y축 가로 정렬 코어 로직 (Ry 보정 포함, 버튼/예외 관리 없음)

        Ry 회전 보정 → Base Y 이동 보정 → 검증.

        Args:
            tab: 탭 객체 (status_func 미지정 시 tab._update_calib_step 사용)
            detect_func: 정렬 검출 함수 (기본: _stereo_detect_full_alignment)
            status_func: 상태 표시 함수 (step, msg) (기본: tab._update_calib_step)

        Returns:
            (success: bool, msg: str)
        """
        if detect_func is None:
            detect_func = self._stereo_detect_full_alignment
        if status_func is None:
            status_func = tab._update_calib_step

        status_func(1, "ArUco 정렬 X: 마커 검출 중...")

        alignment = detect_func()
        if alignment is None:
            return (False, "ArUco 검출 실패")

        # 1) Ry 보정 (마커 기울기 → 수평 회전)
        active_ry = self._get_effective_ry(alignment)
        if active_ry is not None and abs(active_ry) >= 0.5:
            status_func(1, f"Ry 보정 중: {active_ry:.2f}°")
            self._log(f"[StereoCalib X] Ry 보정: {active_ry:.2f}°")
            self._on_ar_tag_align_base_ry(active_ry)
            self._settle()
        else:
            ry_disp = active_ry if active_ry is not None else 0
            self._log(f"[StereoCalib X] Ry 보정 불필요: {ry_disp:.2f}°")

        # 2) 재검출 → Base Y 보정 (이미지 가로 중심 오프셋)
        alignment2 = detect_func()
        if alignment2 is not None and alignment2.offset_y is not None and abs(alignment2.offset_y) >= 5.0:
            status_func(1, f"Base Y 보정 중: {alignment2.offset_y:.1f}px")
            self._log(f"[StereoCalib X] Base Y 보정: {alignment2.offset_y:.1f}px")
            self._on_ar_tag_align_base_y(alignment2.offset_y)
            self._settle()
        elif alignment2 is not None:
            oy_disp = alignment2.offset_y if alignment2.offset_y is not None else 0
            self._log(f"[StereoCalib X] Y 보정 불필요: {oy_disp:.1f}px")

        # 3) 최종 결과
        final = detect_func()
        if final is not None:
            ry_f = self._get_effective_ry(final)
            oy_f = final.offset_y if final.offset_y is not None else 0
            result_msg = f"X 정렬 완료: Ry={ry_f:.2f}°, dY={oy_f:.1f}px"
            self._log(f"[StereoCalib X] {result_msg}")
            return (True, result_msg)
        else:
            return (False, "정렬 후 재검출 실패")

    def _on_stereo_calib_align_aruco_x(self):
        """스테레오 탭 ArUco 정렬 X: Ry 회전 + Base Y 이동으로 이미지 가로 중심 정렬"""
        if not self._require_robot():
            return

        tab = self.tabStereoCalibration
        self._set_stereo_align_buttons_enabled(False)
        try:
            QApplication.processEvents()
            success, msg = self._stereo_align_y_core(tab)
            tab._update_calib_step(0, msg)
        except Exception as e:
            self._log(f"[StereoCalib X] 오류: {e}")
            tab._update_calib_step(0, f"오류: {e}")
        finally:
            self._set_stereo_align_buttons_enabled(True)

    def _on_stereo_calib_align_aruco_combined(self):
        """스테레오 탭 통합 정렬: Y축(Z이동 세로중심) → X축(Ry+BaseY 가로중심) 순차 실행"""
        if not self._require_robot():
            return

        tab = self.tabStereoCalibration
        self._set_stereo_align_buttons_enabled(False)
        try:
            self._log("[StereoCalib 통합] 통합 정렬 시작 (Y:Z축 → X:Ry+BaseY)")
            QApplication.processEvents()

            # ── 1) Y 정렬: Z축 이동으로 이미지 세로(offset_z) 중심 정렬 ──
            tab._update_calib_step(1, "통합 정렬: Y축(세로) 보정 중...")
            z_ok, z_msg = self._stereo_align_z_core(tab)
            self._log(f"[StereoCalib 통합] Y: {z_msg}")

            # ── 2) X 정렬: Ry 회전 + Base Y 이동으로 이미지 가로 중심 정렬 ──
            tab._update_calib_step(1, "통합 정렬: X축(가로) 보정 중...")
            y_ok, y_msg = self._stereo_align_y_core(tab)
            self._log(f"[StereoCalib 통합] X: {y_msg}")

            # ── 3) 미세 정렬 (ArduCam 전용, 모든 coarse 완료 후) ──
            pre_fine = self._stereo_detect_full_alignment()
            if pre_fine is not None:
                ARDUCAM_PX_PER_MM = 13.0  # ArduCam 근사값, 방향 자동 보정

                def _fine_z_measure():
                    a = self._stereo_detect_full_alignment(max_retries=1)
                    return a.offset_z if a is not None and a.offset_z is not None else None

                def _fine_y_measure():
                    a = self._stereo_detect_full_alignment(max_retries=1)
                    return a.offset_y if a is not None and a.offset_y is not None else None

                # Z 미세 정렬
                oz_pre = pre_fine.offset_z if pre_fine.offset_z is not None else 0
                if abs(oz_pre) > 1.5:
                    tab._update_calib_step(1, f"미세 정렬 Z: {oz_pre:.1f}px")
                    self._fine_align_axis('z', ARDUCAM_PX_PER_MM, _fine_z_measure,
                                          log_prefix="[ArduCam Fine Z]")

                # Y 미세 정렬
                pre_fine2 = self._stereo_detect_full_alignment(max_retries=1)
                oy_pre = pre_fine2.offset_y if pre_fine2 is not None and pre_fine2.offset_y is not None else 0
                if abs(oy_pre) > 1.5:
                    tab._update_calib_step(1, f"미세 정렬 Y: {oy_pre:.1f}px")
                    self._fine_align_axis('y', ARDUCAM_PX_PER_MM, _fine_y_measure,
                                          log_prefix="[ArduCam Fine Y]")

            # ── 4) 최종 결과 ──
            final = self._stereo_detect_full_alignment()
            if final is not None:
                ry_f = self._get_effective_ry(final)
                oy_f = final.offset_y if final.offset_y is not None else 0
                oz_f = final.offset_z if final.offset_z is not None else 0
                msg = f"통합 정렬 완료: Ry={ry_f:.2f}°, dY={oy_f:.1f}px, dZ={oz_f:.1f}px"
                self._log(f"[StereoCalib 통합] {msg}")
                tab._update_calib_step(0, msg)
            else:
                tab._update_calib_step(0, "최종 검출 실패")

        except Exception as e:
            self._log(f"[StereoCalib 통합] 오류: {e}")
            tab._update_calib_step(0, f"오류: {e}")
        finally:
            self._set_stereo_align_buttons_enabled(True)

    # ==================== DS435 ArUco 센터링 ====================

    def _on_stereo_calib_align_ds435(self):
        """스테레오 탭 DS435 ArUco 정렬: DS435 프레임으로 검출 → Y/Z 센터링"""
        if not self._require_robot():
            return

        tab = self.tabStereoCalibration

        # DS435 정렬용 고정 TCP 자세 (Rx=90, Ry=0, Rz=90)
        TARGET_RX, TARGET_RY, TARGET_RZ = 90.0, 0.0, 90.0

        tab._update_ds435_calib_step(1, "Detection Pose로 이동 중...")
        try:
            # 현재 TF 그대로 pose 읽기 → Rx/Ry/Rz만 변경 후 movel
            current_pose = self.robot.read_current_pose()
            if not current_pose:
                tab._update_ds435_calib_step(0, "현재 자세 읽기 실패")
                return

            rx, ry, rz = current_pose[3], current_pose[4], current_pose[5]
            need_move = (abs(rx - TARGET_RX) > 0.5 or
                         abs(ry - TARGET_RY) > 0.5 or
                         abs(rz - TARGET_RZ) > 0.5)

            if need_move:
                self._log(f"[DS435Calib] Detection Pose 이동: "
                          f"Rx={rx:.1f}→{TARGET_RX}, Ry={ry:.1f}→{TARGET_RY}, Rz={rz:.1f}→{TARGET_RZ}")

                target = list(current_pose)
                target[3] = TARGET_RX
                target[4] = TARGET_RY
                target[5] = TARGET_RZ

                regs = [self.robot.to_uint16(int(round(v * 10))) for v in target]
                self.robot.write_registers(self.robot.REGISTER_POSE_MAIN, regs)
                self.robot.write_command(self.robot.CMD_MOVE_TO_POSE)

                result = self.robot.wait_for_done(
                    process_events_callback=QApplication.processEvents)
                if not result[0]:
                    tab._update_ds435_calib_step(0, f"Detection Pose 이동 실패: {result[1]}")
                    return
                self._settle(0.5)
                self._log("[DS435Calib] Detection Pose 이동 완료")
            else:
                self._log("[DS435Calib] Detection Pose 이동 불필요 (이미 목표 자세)")

        except Exception as e:
            self._log(f"[DS435Calib] Detection Pose 이동 오류: {e}")
            tab._update_ds435_calib_step(0, f"Detection Pose 이동 오류: {e}")
            return

        tab._update_ds435_calib_step(1, "ArUco 마커 검출 중...")

        try:
            result = self._stereo_detect_ds435_aruco_alignment()
            if result is None:
                tab._update_ds435_calib_step(0, "ArUco 검출 실패 - 마커를 확인하세요")
                return
            offset_y, offset_z = result

            # Y 센터링 (horizontal, 5px 이상)
            if abs(offset_y) >= 5.0:
                tab._update_ds435_calib_step(1, f"Y 보정 중: {offset_y:.1f}px")
                self._ds435_adaptive_align('y', offset_y)
                self._settle()
            else:
                self._log(f"[DS435Calib] Y 보정 불필요: {offset_y:.1f}px")

            # 재검출 후 Z 센터링 (vertical, 5px 이상)
            result2 = self._stereo_detect_ds435_aruco_alignment()
            if result2 is not None:
                _, offset_z2 = result2
                if abs(offset_z2) >= 5.0:
                    tab._update_ds435_calib_step(1, f"Z 보정 중: {offset_z2:.1f}px")
                    self._ds435_adaptive_align('z', offset_z2)
                    self._settle()
                else:
                    self._log(f"[DS435Calib] Z 보정 불필요: {offset_z2:.1f}px")

            # 최종 결과 표시
            result3 = self._stereo_detect_ds435_aruco_alignment()
            if result3 is not None:
                oy_f, oz_f = result3
                msg = f"정렬 완료: dY={oy_f:.1f}px, dZ={oz_f:.1f}px"
                self._log(f"[DS435Calib] {msg}")
                tab._update_ds435_calib_step(0, msg)
            else:
                tab._update_ds435_calib_step(0, "정렬 후 재검출 실패")

        except Exception as e:
            self._log(f"[DS435Calib] ArUco 정렬 오류: {e}")
            tab._update_ds435_calib_step(0, f"오류: {e}")

    def _stereo_detect_ds435_aruco_alignment(self):
        """DS435 프레임에서 ArUco 정렬값 검출 + 시각화.

        탭의 자체 ArUco estimator를 사용 (화면 표시와 동일한 검출기).

        Returns:
            (offset_y, offset_z) tuple, or None if detection fails.
        """
        try:
            tab = self.tabStereoCalibration
            frame = tab.current_ds435_frame
            if frame is None:
                self._log("[DS435Calib] 프레임 없음")
                return None

            intrinsics = self.ds435_camera_manager.intrinsics
            if intrinsics is None:
                self._log("[DS435Calib] DS435 intrinsics 없음")
                return None

            # 탭의 자체 estimator 사용 (화면 표시와 동일)
            markers = tab._aruco_estimator.detect_and_estimate_pose(
                frame.copy(), intrinsics)
            if not markers:
                self._log("[DS435Calib] 마커 미검출")
                return None

            tag_id1, tag_id2 = self._get_target_tag_ids()

            result = self._find_dual_marker_centers(markers, tag_id1, tag_id2)
            if result is None:
                detected_ids = [m['id'] for m in markers]
                self._log(f"[DS435Calib] 대상 마커 미검출: 필요={tag_id1},{tag_id2}, 검출={detected_ids}")
                return None
            c1, c2 = result

            h, w = frame.shape[:2]
            mid_x = (c1[0] + c2[0]) / 2.0
            mid_y = (c1[1] + c2[1]) / 2.0
            offset_y = mid_x - w / 2.0   # horizontal: positive = right
            offset_z = mid_y - h / 2.0   # vertical: positive = below

            # 오버레이 표시
            distances = tab._get_marker_distances(markers)
            overlay = tab._draw_markers(frame, markers, distances)
            tab._display_fixed(overlay, tab.labelDS435View)

            self._log(f"[DS435Calib] offset_y={offset_y:.1f}px, offset_z={offset_z:.1f}px")
            return (offset_y, offset_z)

        except Exception as e:
            self._log(f"[DS435Calib] 검출 오류: {e}")
            return None

    def _ds435_adaptive_align(self, axis: str, d0_px: float, measure_func=None):
        """DS435 기반 적응형 센터링 (Y 또는 Z 축)

        1단계: 테스트 이동 → px/mm 비율 산출
        2단계: 비율 기반 보정 이동
        3단계: 재측정 검증

        Args:
            axis: 'y' (horizontal) or 'z' (vertical)
            d0_px: 현재 오프셋 (px)
            measure_func: 마커 오프셋 측정 함수 (기본: _measure_ds435_marker_offset)

        Note:
            _on_ar_tag_align_base_y()와 알고리즘 구조가 유사하나
            MAX_CORRECTION 초과 시 처리가 다름 (이쪽: 2단계 분할 이동,
            ArUco탭: 안전 중단). 측정 함수도 상이하여 통합 보류.
        """
        if measure_func is None:
            measure_func = self._measure_ds435_marker_offset
        DEAD_ZONE_PX = 3
        TEST_MM = 5.0
        MAX_CORRECTION_MM = 50.0

        if abs(d0_px) < DEAD_ZONE_PX:
            self._log(f"[DS435Calib] Base {axis.upper()}: 오프셋 {DEAD_ZONE_PX}px 미만, 보정 불필요")
            return

        try:
            test_cmd = -TEST_MM if d0_px > 0 else TEST_MM
            self._log(f"[DS435Calib] Base {axis.upper()} 보정 시작: d0={d0_px:.1f}px")

            # --- 1단계: 테스트 이동으로 px/mm 비율 산출 ---
            pose_before = self.robot.read_current_pose()
            if pose_before is None:
                self._log("[DS435Calib] 현재 포즈 읽기 실패")
                return
            axis_idx = 1 if axis == 'y' else 2  # Y=1, Z=2
            val_before = pose_before[axis_idx]

            self._log(f"[DS435Calib] 1단계: {test_cmd:+.1f}mm 테스트 이동 ({axis.upper()})")
            success, msg = self.robot.send_base_linear(
                axis, test_cmd, wait=True,
                process_events_callback=QApplication.processEvents)
            if not success:
                self._log(f"[DS435Calib] 테스트 이동 실패: {msg}")
                return

            # 위치 안정화 대기
            _, final_val = self._wait_for_position_stable(
                axis_idx=axis_idx, reference_val=val_before)
            actual_mm = final_val - val_before
            self._log(f"[DS435Calib] 실제 이동: {actual_mm:.2f}mm (명령: {test_cmd:+.1f}mm)")

            if abs(actual_mm) < 0.5:
                self._log(f"[DS435Calib] 실제 이동 < 0.5mm, 복귀")
                self.robot.send_base_linear(
                    axis, -test_cmd, wait=True,
                    process_events_callback=QApplication.processEvents)
                return

            # 재측정
            self._settle()
            d1 = measure_func(axis)
            if d1 is None:
                self._log(f"[DS435Calib] 테스트 후 마커 감지 실패, 복귀")
                self.robot.send_base_linear(
                    axis, -test_cmd, wait=True,
                    process_events_callback=QApplication.processEvents)
                return

            delta_px = d1 - d0_px
            self._log(f"[DS435Calib] 테스트 결과: d1={d1:.1f}px, 변화={delta_px:.1f}px")

            if abs(delta_px) < 2:
                self._log(f"[DS435Calib] 픽셀 변화 < 2px, 측정 불안정. 복귀")
                self.robot.send_base_linear(
                    axis, -test_cmd, wait=True,
                    process_events_callback=QApplication.processEvents)
                return

            px_per_mm = delta_px / actual_mm
            self._log(f"[DS435Calib] 비율: {px_per_mm:.2f} px/mm ({abs(1.0/px_per_mm):.3f} mm/px)")

            # --- 2단계: 비율 기반 보정 이동 ---
            correction_mm = -d1 / px_per_mm
            self._log(f"[DS435Calib] 2단계: 보정 {correction_mm:.1f}mm")

            if abs(correction_mm) > MAX_CORRECTION_MM:
                # 2단계 분할 이동: 절반 이동 → 재측정 → 나머지 보정
                half_mm = correction_mm / 2.0
                self._log(f"[DS435Calib] 보정 과대 ({correction_mm:.1f}mm > {MAX_CORRECTION_MM}mm), 2단계 분할: {half_mm:.1f}mm + 나머지")

                # 분할 1차 이동
                success, msg = self.robot.send_base_linear(
                    axis, half_mm, wait=True,
                    process_events_callback=QApplication.processEvents)
                if not success:
                    self._log(f"[DS435Calib] 분할 1차 이동 실패: {msg}")
                    return

                self._settle(0.5)

                # 재측정
                d_mid = measure_func(axis)
                if d_mid is not None and abs(d_mid) >= DEAD_ZONE_PX:
                    correction2 = -d_mid / px_per_mm
                    correction2 = max(-MAX_CORRECTION_MM, min(MAX_CORRECTION_MM, correction2))
                    self._log(f"[DS435Calib] 분할 2차: 잔여={d_mid:.1f}px → 보정 {correction2:.1f}mm")

                    success2, msg2 = self.robot.send_base_linear(
                        axis, correction2, wait=True,
                        process_events_callback=QApplication.processEvents)
                    if not success2:
                        self._log(f"[DS435Calib] 분할 2차 이동 실패: {msg2}")

                    total_mm = test_cmd + half_mm + correction2
                else:
                    total_mm = test_cmd + half_mm
                    self._log(f"[DS435Calib] 분할 1차로 충분 (잔여={d_mid}px)")
            else:
                success, msg = self.robot.send_base_linear(
                    axis, correction_mm, wait=True,
                    process_events_callback=QApplication.processEvents)
                if not success:
                    self._log(f"[DS435Calib] 보정 이동 실패: {msg}")
                    return

                total_mm = test_cmd + correction_mm

            # --- 3단계: 검증 ---
            self._settle(0.5)

            d_final = measure_func(axis)
            if d_final is not None:
                self._log(f"[DS435Calib] Base {axis.upper()} 보정 완료: 총 {total_mm:.1f}mm, 잔여={d_final:.1f}px")
            else:
                self._log(f"[DS435Calib] Base {axis.upper()} 보정 완료: 총 {total_mm:.1f}mm (검증 측정 실패)")

        except Exception as e:
            self._log(f"[DS435Calib] Base {axis.upper()} 보정 오류: {e}")

    def _measure_ds435_marker_offset(self, axis: str):
        """DS435 프레임에서 마커 중점의 오프셋 측정

        Args:
            axis: 'y' (horizontal offset) or 'z' (vertical offset)

        Returns:
            float offset in pixels, or None if detection fails.
        """
        try:
            tab = self.tabStereoCalibration
            frame = tab.current_ds435_frame
            if frame is None:
                return None

            intrinsics = self.ds435_camera_manager.intrinsics
            if intrinsics is None:
                return None

            markers = tab._aruco_estimator.detect_and_estimate_pose(
                frame.copy(), intrinsics)
            if not markers:
                return None

            tag_id1, tag_id2 = self._get_target_tag_ids()

            result = self._find_dual_marker_centers(markers, tag_id1, tag_id2)
            if result is None:
                return None
            c1, c2 = result

            h, w = frame.shape[:2]
            if axis == 'y':
                mid_px = (c1[0] + c2[0]) / 2.0
                return mid_px - w / 2.0
            else:  # z
                mid_px = (c1[1] + c2[1]) / 2.0
                return mid_px - h / 2.0
        except Exception as e:
            self._log(f"[DS435Calib] 마커 측정 오류: {e}")
            return None

    # ==================== DS435 → ArduCam 핸드오프 ====================

    def _check_arducam_marker_visible(self):
        """ArduCam 프레임에서 마커 검출 시도.

        Returns:
            (offset_y_px, offset_z_px) tuple if detected, None otherwise.
        """
        try:
            tab = self.tabStereoCalibration
            frame = tab.current_frame  # ArduCam 최신 프레임
            if frame is None:
                self._log("[Handoff] ArduCam 프레임 없음")
                return None

            if not self.arducam_manager:
                self._log("[Handoff] ArduCam 매니저 없음")
                return None

            intrinsics = self.arducam_manager.intrinsics
            if intrinsics is None:
                self._log("[Handoff] ArduCam intrinsics 없음")
                return None

            markers = tab._aruco_estimator.detect_and_estimate_pose(
                frame.copy(), intrinsics)
            if not markers:
                return None

            tag_id1, tag_id2 = self._get_target_tag_ids()

            result = self._find_dual_marker_centers(markers, tag_id1, tag_id2)
            if result is None:
                return None
            c1, c2 = result

            h, w = frame.shape[:2]
            mid_x = (c1[0] + c2[0]) / 2.0
            mid_y = (c1[1] + c2[1]) / 2.0
            offset_y = mid_x - w / 2.0
            offset_z = mid_y - h / 2.0

            # 오버레이 표시
            overlay = tab._draw_markers(frame, markers)
            tab._display_fixed(overlay, tab.labelArduCamView)

            return (offset_y, offset_z)

        except Exception as e:
            self._log(f"[Handoff] ArduCam 검출 오류: {e}")
            return None

    def _measure_ds435_marker_depth(self):
        """DS435에서 마커 중심점의 depth 측정 (mm).

        Returns:
            depth in mm, or None if measurement fails.
        """
        try:
            tab = self.tabStereoCalibration
            frame = tab.current_ds435_frame
            if frame is None:
                return None

            intrinsics = self.ds435_camera_manager.intrinsics
            if intrinsics is None:
                return None

            markers = tab._aruco_estimator.detect_and_estimate_pose(
                frame.copy(), intrinsics)
            if not markers:
                return None

            tag_id1, tag_id2 = self._get_target_tag_ids()

            result = self._find_dual_marker_centers(markers, tag_id1, tag_id2)
            if result is None:
                return None
            c1, c2 = result
            mid = ((c1[0] + c2[0]) / 2.0, (c1[1] + c2[1]) / 2.0)

            depth = self.ds435_camera_manager.get_distance_at(
                int(mid[0]), int(mid[1]), from_color=True)
            return depth
        except Exception as e:
            self._log(f"[Handoff] depth 측정 오류: {e}")
            return None

    def _on_stereo_calib_handoff_ds435_to_arducam(self):
        """DS435 → ArduCam 핸드오프: 4단계 순차 실행.

        1단계: DS435로 마커 중심 정렬 (이미지 중앙)
        2단계: X축 이동으로 거리 380mm 유지
        3단계: camera_offset_mm 적용하여 Y,Z 이동 (ArduCam FOV로)
        4단계: ArduCam 통합 정렬
        """
        if not self._require_robot():
            return

        tab = self.tabStereoCalibration
        self._set_stereo_align_buttons_enabled(False)

        try:
            from services.stereo_offset_calculator import StereoOffsetCalculator

            TARGET_DEPTH_MM = 370.0
            DEPTH_TOLERANCE_MM = 5.0

            # --- sweep 데이터 로드 ---
            tab._update_ds435_calib_step(1, "① sweep 데이터 로드 중...")
            QApplication.processEvents()

            data_dir = os.path.join(os.path.dirname(__file__), '..', 'data')
            sweep_path = StereoOffsetCalculator.find_latest_sweep(data_dir)
            if sweep_path is None:
                tab._update_ds435_calib_step(0, "sweep 데이터 없음 - 스윕 먼저 실행하세요")
                self._log("[Handoff] data/stereo/에 sweep JSON 파일 없음")
                return

            calc = StereoOffsetCalculator(sweep_path, self.arducam_manager.intrinsics)
            if not calc.is_valid:
                tab._update_ds435_calib_step(0, "sweep 데이터 카메라 불일치 - 스윕 재실행 필요")
                self._log("[Handoff] sweep 데이터가 현재 카메라와 불일치. 스윕 재실행 필요.")
                return
            raw_y, raw_z = calc.camera_offset_mm
            # 부호 반전: 이미지 좌표→로봇 이동 방향 변환
            cam_offset_y = -raw_y
            cam_offset_z = -raw_z
            self._log(f"[Handoff] camera_offset(raw): dY={raw_y:.2f}mm, dZ={raw_z:.2f}mm → 적용: dY={cam_offset_y:.2f}mm, dZ={cam_offset_z:.2f}mm")

            # ============================================================
            # 0단계: Detection Pose 이동 (Rx=90, Ry=0, Rz=90)
            # ============================================================
            TARGET_RX, TARGET_RY, TARGET_RZ = 90.0, 0.0, 90.0
            tab._update_ds435_calib_step(1, "⓪ Detection Pose 이동 중...")
            QApplication.processEvents()

            current_pose = self.robot.read_current_pose()
            if not current_pose:
                tab._update_ds435_calib_step(0, "현재 자세 읽기 실패")
                return

            rx, ry, rz = current_pose[3], current_pose[4], current_pose[5]
            need_move = (abs(rx - TARGET_RX) > 0.5 or
                         abs(ry - TARGET_RY) > 0.5 or
                         abs(rz - TARGET_RZ) > 0.5)

            if need_move:
                self._log(f"[Handoff] Detection Pose 이동: "
                          f"Rx={rx:.1f}→{TARGET_RX}, Ry={ry:.1f}→{TARGET_RY}, Rz={rz:.1f}→{TARGET_RZ}")
                target = list(current_pose)
                target[3] = TARGET_RX
                target[4] = TARGET_RY
                target[5] = TARGET_RZ

                regs = [self.robot.to_uint16(int(round(v * 10))) for v in target]
                self.robot.write_registers(self.robot.REGISTER_POSE_MAIN, regs)
                self.robot.write_command(self.robot.CMD_MOVE_TO_POSE)

                result = self.robot.wait_for_done(
                    process_events_callback=QApplication.processEvents)
                if not result[0]:
                    tab._update_ds435_calib_step(0, f"Detection Pose 이동 실패: {result[1]}")
                    return
                self._settle(0.5)
                self._log("[Handoff] Detection Pose 이동 완료")
            else:
                self._log("[Handoff] Detection Pose 이동 불필요 (이미 목표 자세)")

            # ============================================================
            # 1단계: DS435 마커 중심 정렬
            # ============================================================
            tab._update_ds435_calib_step(1, "① DS435 마커 중심 정렬 중...")
            QApplication.processEvents()
            self._log("[Handoff] === 1단계: DS435 마커 중심 정렬 ===")

            result = self._stereo_detect_ds435_aruco_alignment()
            if result is None:
                tab._update_ds435_calib_step(0, "DS435 마커 검출 실패")
                return

            offset_y, offset_z = result
            self._log(f"[Handoff] DS435 초기 오프셋: dY={offset_y:.1f}px, dZ={offset_z:.1f}px")

            # Y 센터링
            if abs(offset_y) >= 5.0:
                tab._update_ds435_calib_step(1, f"① DS435 Y 보정: {offset_y:.1f}px")
                self._ds435_adaptive_align('y', offset_y)
                self._settle()

            # 재검출 후 Z 센터링
            result2 = self._stereo_detect_ds435_aruco_alignment()
            if result2 is not None:
                _, offset_z2 = result2
                if abs(offset_z2) >= 5.0:
                    tab._update_ds435_calib_step(1, f"① DS435 Z 보정: {offset_z2:.1f}px")
                    self._ds435_adaptive_align('z', offset_z2)
                    self._settle()

            # 1단계 결과 확인
            result3 = self._stereo_detect_ds435_aruco_alignment()
            if result3 is not None:
                fy, fz = result3
                self._log(f"[Handoff] 1단계 완료: dY={fy:.1f}px, dZ={fz:.1f}px")
            else:
                self._log("[Handoff] 1단계 후 재검출 실패, 계속 진행")

            # ============================================================
            # 2단계: X축 거리 380mm 유지
            # ============================================================
            tab._update_ds435_calib_step(1, "② X축 거리 380mm 조정 중...")
            QApplication.processEvents()
            self._log("[Handoff] === 2단계: X축 거리 380mm 조정 ===")

            # depth 측정 (최대 3회 시도)
            depth = None
            for attempt in range(3):
                self._settle()
                depth = self._measure_ds435_marker_depth()
                if depth is not None and depth > 0:
                    break
                self._log(f"[Handoff] depth 측정 실패 (시도 {attempt+1}/3)")

            if depth is None or depth <= 0:
                self._log("[Handoff] depth 측정 불가, 2단계 건너뜀")
            else:
                depth_error = depth - TARGET_DEPTH_MM
                self._log(f"[Handoff] 현재 depth: {depth:.1f}mm, 목표: {TARGET_DEPTH_MM:.0f}mm, 차이: {depth_error:.1f}mm")

                if abs(depth_error) >= DEPTH_TOLERANCE_MM:
                    # depth가 크면 마커에 가까워져야 → X+ 이동 (로봇이 전진)
                    # depth가 작으면 마커에서 멀어져야 → X- 이동 (로봇이 후진)
                    x_move = depth_error  # depth 큰만큼 전진
                    if abs(x_move) > 100.0:
                        self._log(f"[Handoff] X 이동 과대 ({x_move:.1f}mm > 100mm), 안전 중단")
                    else:
                        tab._update_ds435_calib_step(1, f"② X 이동: {x_move:.1f}mm")
                        success, msg = self.robot.send_base_linear(
                            'x', x_move, wait=True,
                            process_events_callback=QApplication.processEvents)
                        if success:
                            self._settle(0.5)
                            # depth 재측정 확인
                            depth2 = self._measure_ds435_marker_depth()
                            if depth2 is not None:
                                self._log(f"[Handoff] X 이동 후 depth: {depth2:.1f}mm")
                            else:
                                self._log("[Handoff] X 이동 후 depth 재측정 실패")
                        else:
                            self._log(f"[Handoff] X 이동 실패: {msg}")
                else:
                    self._log(f"[Handoff] depth 오차 {abs(depth_error):.1f}mm < {DEPTH_TOLERANCE_MM}mm, X 이동 불필요")

            # ============================================================
            # 3단계: camera_offset_mm 적용 (Y, Z 이동)
            # ============================================================
            tab._update_ds435_calib_step(1, f"③ 카메라 오프셋 적용: Y={cam_offset_y:.1f}mm, Z={cam_offset_z:.1f}mm")
            QApplication.processEvents()
            self._log(f"[Handoff] === 3단계: 카메라 오프셋 적용 Y={cam_offset_y:.1f}mm, Z={cam_offset_z:.1f}mm ===")

            # Y 이동
            if abs(cam_offset_y) >= 0.5:
                tab._update_ds435_calib_step(1, f"③ Y 이동: {cam_offset_y:.1f}mm")
                success, msg = self.robot.send_base_linear(
                    'y', cam_offset_y, wait=True,
                    process_events_callback=QApplication.processEvents)
                if success:
                    self._log(f"[Handoff] Y 이동 완료: {cam_offset_y:.1f}mm")
                else:
                    self._log(f"[Handoff] Y 이동 실패: {msg}")

                self._settle()

            # Z 이동
            if abs(cam_offset_z) >= 0.5:
                tab._update_ds435_calib_step(1, f"③ Z 이동: {cam_offset_z:.1f}mm")
                success, msg = self.robot.send_base_linear(
                    'z', cam_offset_z, wait=True,
                    process_events_callback=QApplication.processEvents)
                if success:
                    self._log(f"[Handoff] Z 이동 완료: {cam_offset_z:.1f}mm")
                else:
                    self._log(f"[Handoff] Z 이동 실패: {msg}")

                self._settle()

            # ArduCam 검출 확인 (3단계 결과 — 실패해도 4단계 진행)
            self._settle(0.5)

            arducam_check = self._check_arducam_marker_visible()
            if arducam_check is not None:
                ar_oy, ar_oz = arducam_check
                self._log(f"[Handoff] 3단계 후 ArduCam 검출 성공: dY={ar_oy:.1f}px, dZ={ar_oz:.1f}px")
            else:
                self._log("[Handoff] 3단계 후 ArduCam 즉시 검출 실패, 4단계에서 재시도")

            # ============================================================
            # 4단계: ArduCam 통합 정렬 (반복 수렴)
            # ============================================================
            tab._update_ds435_calib_step(1, "④ ArduCam 통합 정렬 중...")
            QApplication.processEvents()
            self._log("[Handoff] === 4단계: ArduCam 통합 정렬 ===")

            CONVERGE_PX = 10.0
            MAX_ITER = 3

            for iteration in range(MAX_ITER):
                self._log(f"[Handoff] 통합 정렬 반복 {iteration+1}/{MAX_ITER}")
                tab._update_ds435_calib_step(1, f"④ 통합 정렬 ({iteration+1}/{MAX_ITER})...")

                # 버튼 복원 후 기존 통합 정렬 호출 (자체 버튼 관리)
                self._set_stereo_align_buttons_enabled(True)
                self._on_stereo_calib_align_aruco_combined()

                # 수렴 확인
                self._settle()
                check = self._stereo_detect_full_alignment()
                if check is not None:
                    oy = abs(check.offset_y) if check.offset_y is not None else 0
                    oz = abs(check.offset_z) if check.offset_z is not None else 0
                    self._log(f"[Handoff] 반복 {iteration+1} 결과: dY={oy:.1f}px, dZ={oz:.1f}px")
                    if oy < CONVERGE_PX and oz < CONVERGE_PX:
                        self._log(f"[Handoff] 수렴 완료 (반복 {iteration+1})")
                        break
                else:
                    self._log(f"[Handoff] 반복 {iteration+1} 후 검출 실패")

            # 최종 결과
            self._set_stereo_align_buttons_enabled(False)
            final = self._stereo_detect_full_alignment()
            if final is not None:
                ry_f = self._get_effective_ry(final)
                oy_f = final.offset_y if final.offset_y is not None else 0
                oz_f = final.offset_z if final.offset_z is not None else 0
                msg = f"핸드오프 완료! Ry={ry_f:.2f}°, dY={oy_f:.1f}px, dZ={oz_f:.1f}px"
            else:
                msg = "핸드오프 완료 (최종 검출 실패)"

            self._log(f"[Handoff] {msg}")
            tab._update_ds435_calib_step(0, msg)

        except Exception as e:
            self._log(f"[Handoff] 오류: {e}")
            import traceback
            self._log(f"[Handoff] {traceback.format_exc()}")
            tab._update_ds435_calib_step(0, f"오류: {e}")
        finally:
            self._set_stereo_align_buttons_enabled(True)

    # ==================== Sweep Calibration ====================

    def _on_sweep_start(self, step_mm: float, count: int):
        """스윕 캘리브레이션 시작 핸들러"""
        if not self.robot or not self.robot.is_connected:
            self._log("[Sweep] 로봇 미연결")
            self.tabStereoCalibration.reset_sweep_ui()
            return

        # 안전 확인 대화상자
        total_mm = step_mm * count
        msg = (f"로봇-픽셀 관계 분석을 시작합니다.\n\n"
               f"스텝: {step_mm}mm × {count}회 = {total_mm:.0f}mm\n"
               f"축: Z(-{total_mm:.0f}mm), X(+{total_mm:.0f}mm), Y(+{total_mm:.0f}mm)\n\n"
               f"로봇이 현재 위치에서 각 축 방향으로 이동합니다.\n"
               f"주변에 장애물이 없는지 확인하세요.")
        reply = QMessageBox.question(
            self, "스윕 캘리브레이션",
            msg, QMessageBox.Yes | QMessageBox.No, QMessageBox.No)
        if reply != QMessageBox.Yes:
            self.tabStereoCalibration.reset_sweep_ui()
            return

        # 양쪽 카메라 자동 시작 (꺼져 있으면)
        cameras_started = False
        if not self.ds435_camera_manager.is_running:
            self._log("[Sweep] DS435 카메라 자동 시작")
            self.ds435_camera_manager.start()
            cameras_started = True
        if not self.arducam_manager.is_running:
            self._log("[Sweep] ArduCam 카메라 자동 시작")
            self.arducam_manager.start()
            cameras_started = True
        if cameras_started:
            # 카메라 안정화 대기 + 탭 버튼 상태 동기화
            self._settle(1.0)
            tab = self.tabStereoCalibration
            tab.btnStartCameras.setEnabled(False)
            tab.btnStopCameras.setEnabled(True)

        # intrinsics 확인
        if self.ds435_camera_manager.intrinsics is None:
            self._log("[Sweep] DS435 intrinsics 없음 — 카메라 확인 필요")
            self.tabStereoCalibration.reset_sweep_ui()
            return
        if self.arducam_manager.intrinsics is None:
            self._log("[Sweep] ArduCam intrinsics 없음 — 캘리브레이션 파일 확인 필요")
            self.tabStereoCalibration.reset_sweep_ui()
            return

        # 기존 서비스 정리 (orphan signal 방지)
        if self._sweep_service is not None:
            try:
                self._sweep_service.status_updated.disconnect()
                self._sweep_service.progress_updated.disconnect()
                self._sweep_service.log_message.disconnect()
                self._sweep_service.sweep_error.disconnect()
                self._sweep_service.sweep_finished.disconnect()
                self._sweep_service.data_captured.disconnect()
            except RuntimeError:
                pass
            self._sweep_service.setParent(None)
            self._sweep_service.deleteLater()
            self._sweep_service = None

        # 서비스 생성
        from services.sweep_calibration_service import SweepCalibrationService
        tab = self.tabStereoCalibration

        self._sweep_service = SweepCalibrationService(
            robot=self.robot,
            ds435_manager=self.ds435_camera_manager,
            arducam_manager=self.arducam_manager,
            aruco_estimator=tab._aruco_estimator,
            parent=self,
        )

        # 시그널 연결
        self._sweep_service.status_updated.connect(tab.update_sweep_status)
        self._sweep_service.progress_updated.connect(tab.update_sweep_progress)
        self._sweep_service.log_message.connect(self._log)
        self._sweep_service.sweep_error.connect(self._on_sweep_error)
        self._sweep_service.sweep_finished.connect(self._on_sweep_finished)
        self._sweep_service.data_captured.connect(tab.add_sweep_data_row)

        self._sweep_service.start(step_mm, count)

    def _on_sweep_cancel(self):
        """스윕 취소 핸들러"""
        if self._sweep_service and self._sweep_service.is_running:
            self._sweep_service.cancel()

    def _on_sweep_error(self, error_msg: str):
        """스윕 오류 핸들러"""
        self._log(f"[Sweep] 오류: {error_msg}")
        tab = self.tabStereoCalibration
        tab.update_sweep_status(f"오류: {error_msg}")
        tab.reset_sweep_ui()

    def _on_sweep_finished(self, results: dict):
        """스윕 완료 핸들러"""
        tab = self.tabStereoCalibration
        tab.reset_sweep_ui()
        self._log("[Sweep] 스윕 완료")

    # ==================== Z축 레이저 스캔 ====================

    def _on_laser_scan_start(self, step_mm: float, total_distance: float):
        """Z축 레이저 스캔 시작 핸들러"""
        from PyQt5.QtWidgets import QMessageBox

        # 로봇 연결 확인
        if self.robot is None or not self.robot.is_connected:
            self._log("[LaserScan] 로봇 미연결")
            return

        # ArduCam 카메라 확인/시작
        if self.arducam_manager is None or not self.arducam_manager.is_running:
            self._log("[LaserScan] ArduCam 시작 중...")
            self._on_camera_type_changed(CAMERA_ARDUCAM)
            self._on_start_camera()
            import time
            time.sleep(1.0)

        # 확인 다이얼로그
        n_steps = int(round(total_distance / step_mm))
        reply = QMessageBox.question(
            self, "Z축 레이저 스캔",
            f"Z축 레이저 스캔을 시작합니다.\n\n"
            f"스텝: {step_mm}mm, 총 거리: {total_distance}mm ({n_steps}스텝)\n"
            f"로봇이 Z축 아래로 {total_distance}mm 이동합니다.\n"
            f"주변 장애물을 확인하세요.",
            QMessageBox.Ok | QMessageBox.Cancel,
            QMessageBox.Cancel,
        )
        if reply != QMessageBox.Ok:
            return

        # 기존 서비스 정리
        self._cleanup_laser_scan_service()

        # 서비스 생성
        from services.laser_scan_service import LaserScanService
        tab = self.tabLaserScan

        self._laser_scan_service = LaserScanService(
            robot=self.robot,
            arducam_manager=self.arducam_manager,
            roi_config=tab._roi_config,
            camera_matrix=tab.camera_matrix,
            dist_coeffs=tab.dist_coeffs,
            parent=self,
        )

        # 시그널 연결
        self._laser_scan_service.status_updated.connect(tab.update_scan_status)
        self._laser_scan_service.progress_updated.connect(tab.update_scan_progress)
        self._laser_scan_service.step_data_captured.connect(tab.add_scan_data_row)
        self._laser_scan_service.scan_finished.connect(self._on_laser_scan_finished)
        self._laser_scan_service.scan_error.connect(self._on_laser_scan_error)
        self._laser_scan_service.log_message.connect(self._log)

        tab.set_scanning(True)
        self._laser_scan_service.start(step_mm, total_distance)

    def _on_laser_scan_cancel(self):
        """Z축 레이저 스캔 취소 핸들러"""
        if self._laser_scan_service and self._laser_scan_service.is_running:
            self._laser_scan_service.cancel()

    def _on_laser_scan_finished(self, results: dict):
        """Z축 레이저 스캔 완료 핸들러"""
        tab = self.tabLaserScan
        tab.show_scan_results(results)
        tab.set_scanning(False)
        tab.reset_scan_ui()
        self._log("[LaserScan] 스캔 완료")
        self._cleanup_laser_scan_service()

    def _on_laser_scan_error(self, error_msg: str):
        """Z축 레이저 스캔 오류 핸들러"""
        self._log(f"[LaserScan] 오류: {error_msg}")
        tab = self.tabLaserScan
        tab.update_scan_status(f"오류: {error_msg}")
        tab.set_scanning(False)
        tab.reset_scan_ui()
        self._cleanup_laser_scan_service()

    def _cleanup_laser_scan_service(self):
        """레이저 스캔 서비스 정리"""
        if self._laser_scan_service is not None:
            try:
                self._laser_scan_service.status_updated.disconnect()
                self._laser_scan_service.progress_updated.disconnect()
                self._laser_scan_service.step_data_captured.disconnect()
                self._laser_scan_service.scan_finished.disconnect()
                self._laser_scan_service.scan_error.disconnect()
                self._laser_scan_service.log_message.disconnect()
            except (RuntimeError, TypeError):
                pass
            self._laser_scan_service.setParent(None)
            self._laser_scan_service.deleteLater()
            self._laser_scan_service = None

    # ==================== 레이저 캘리브레이션 Z 조정 ====================

    def _on_calib_adjust_z(self):
        """Z 조정 버튼: 레이저 Y 측정 + Z축 이동으로 목표 도달"""
        tab = self.tabLaserCalibration
        tab._show_aruco_overlay = True
        tab.set_z_adjust_status(True)

        try:
            result = self._calib_adjust_z_to_target()
            if result is True:
                tab._update_calib_step(0, "Z 조정 완료")
            elif result is False and not tab._z_adjust_cancel:
                tab._update_calib_step(0, "Z 조정 실패")
            # result is None → 감도 측정 완료 (메시지 이미 설정됨)
        except Exception as e:
            self._log(f"[Calib] Z 조정 오류: {e}")
            tab._update_calib_step(0, f"오류: {e}")
        finally:
            tab.set_z_adjust_status(False)

    def _calib_measure_error_per_mm(self):
        """error_per_mm 측정 (테스트 2mm + 원위치 복귀).

        error = laser_y - target_y 이므로,
        마커/레이저 모두 Z에 따라 이동해도 error 변화율이 정확.
        """
        tab = self.tabLaserCalibration
        tab._update_calib_step(2, "감도 측정 중 (2mm 테스트)...")

        # 이동 전 측정 (laser+target 동시 median)
        laser_y, target_y = self._calib_measure_state()
        if target_y is None or laser_y is None:
            self._log("[Calib] 감도 측정 실패: 검출 불가")
            return None
        error_before = laser_y - target_y

        # 테스트 이동
        test_move = -2.0
        if not self._calib_move_z(test_move, tab):
            return None

        # 이동 후 측정 (laser+target 동시 median)
        laser_y2, target_y2 = self._calib_measure_state()
        if target_y2 is None or laser_y2 is None:
            self._log("[Calib] 감도 측정: 이동 후 검출 실패 → 원위치 복귀")
            self._calib_move_z(-test_move, tab)
            return None

        error_after = laser_y2 - target_y2
        delta = error_after - error_before

        # 원위치 복귀
        if not self._calib_move_z(-test_move, tab):
            self._log("[Calib] 원위치 복귀 실패 — Z 위치 오프셋됨")
            return None

        if abs(delta) < 0.3:
            self._log(f"[Calib] 감도 측정: error 변화 미미 ({delta:.2f}px)")
            return None

        rate = delta / test_move
        if abs(rate) < 0.5:
            self._log(f"[Calib] 감도 너무 낮음: {rate:.2f} px/mm → 위치 변경 필요")
            return None

        self._log(f"[Calib] 감도 측정 완료: {rate:.2f} px/mm")
        return rate

    def _calib_adjust_z_to_target(self):
        """레이저 라인을 목표 Y에 맞추는 Z 조정.

        첫 호출: 감도 측정 (테스트+복귀) → 캐시 저장. 이동 없음.
        이후 호출: 캐시된 감도로 1회 이동만 수행.
        """
        tab = self.tabLaserCalibration
        max_move = 20.0  # 1회 최대 이동

        # ── 감도 캐시 확인 ──
        if not hasattr(self, '_cached_error_per_mm') or self._cached_error_per_mm is None:
            rate = self._calib_measure_error_per_mm()
            if rate is None:
                tab._update_calib_step(0, "감도 측정 실패")
                return False
            self._cached_error_per_mm = rate
            self._log(f"[Calib] error_per_mm={rate:.2f} 캐시됨. 다시 눌러서 이동하세요.")
            tab._update_calib_step(0, f"감도={rate:.2f}px/mm. 다시 눌러 이동")
            return None  # 감도 측정 완료 (실패가 아님)

        # ── 현재 error 측정 (laser+target 동시 median) ──
        laser_y, target_y = self._calib_measure_state()
        if target_y is None or laser_y is None:
            self._log("[Calib] 검출 실패 (마커 또는 레이저)")
            tab._update_calib_step(0, "검출 실패")
            return False

        error = laser_y - target_y
        self._log(f"[Calib] error={error:.1f}px (laser={laser_y:.1f}, target={target_y:.1f})")

        if abs(error) <= 1:
            self._log("[Calib] 목표 범위 내 (≤1px)")
            return True

        # ── 1회 이동 ──
        move_z = -error / self._cached_error_per_mm
        if abs(move_z) > max_move:
            move_z = max_move if move_z > 0 else -max_move
            self._log(f"[Calib] 이동 제한: {move_z:.1f}mm (최대 {max_move}mm)")

        if abs(move_z) < 0.1:
            self._log(f"[Calib] 이동량 미미: {move_z:.2f}mm")
            return abs(error) <= 2

        tab._update_calib_step(2, f"Z {move_z:.1f}mm 이동 중...")
        self._log(f"[Calib] Z 이동: {move_z:.1f}mm")
        if not self._calib_move_z(move_z, tab):
            return False
        self._cached_error_per_mm = None  # Z 위치 변경 → 감도 재측정 필요

        # ── 결과 확인 (laser+target 동시 median) ──
        laser_f, target_f = self._calib_measure_state()
        if target_f is not None and laser_f is not None:
            final_error = laser_f - target_f
            self._log(f"[Calib] 결과: error={final_error:.1f}px (laser={laser_f:.1f}, target={target_f:.1f})")
            return abs(final_error) <= 2
        self._log("[Calib] 이동 후 검출 실패")
        return False

    def _calib_refresh_markers(self, frame):
        """프레임에서 ArUco 마커 검출 → tab._marker_list 갱신"""
        try:
            tab_ar = self.tabArucoReliability
            markers = self.vision_manager.detect_marker_centers(
                frame, tab_ar.camera_matrix, tab_ar.dist_coeffs, estimate_pose=True)
            self.tabLaserCalibration.set_markers(markers)
        except Exception:
            self.tabLaserCalibration.set_markers([])

    def _calib_measure_laser_y(self, n_samples=3):
        """레이저 Y 위치 측정 + ArUco 마커 갱신 (다중 프레임 median)"""
        laser_y, _ = self._calib_measure_state(n_samples)
        return laser_y

    def _calib_measure_state(self, n_samples=3):
        """laser_y, target_y 동시 측정 (다중 프레임 median, 노이즈 대칭 저감)"""
        tab = self.tabLaserCalibration
        laser_vals, target_vals = [], []
        for _ in range(n_samples):
            frame = self.camera_manager.get_frame()
            if frame is None:
                continue
            self._calib_refresh_markers(frame)
            ly = tab.get_current_laser_y(frame)
            ty = tab.get_target_y()
            if ly is not None and ty is not None:
                laser_vals.append(ly)
                target_vals.append(ty)
            QApplication.processEvents()
            time.sleep(0.1)
        if not laser_vals:
            return None, None
        return float(np.median(laser_vals)), float(np.median(target_vals))

    def _calib_move_z(self, distance, tab):
        """Z축 이동 + 대기 + 안정화. 취소 시 False 반환"""
        self.robot.send_base_linear('z', distance,
            wait=False,
            process_events_callback=QApplication.processEvents)
        done_ok, done_msg = self.robot.wait_for_done(
            process_events_callback=QApplication.processEvents,
            stop_flag_callback=lambda: tab._z_adjust_cancel)
        if not done_ok:
            if tab._z_adjust_cancel:
                self._log("[Calib] Z 이동 중 취소")
                return False
            self._log(f"[Calib] Z 이동 대기 실패: {done_msg}")
            return False
        time.sleep(0.2)  # PRS 클린업 대기
        self._calib_wait_for_position_stable('z')
        return not tab._z_adjust_cancel

    def _calib_wait_for_position_stable(self, axis='z', timeout=10.0):
        """이동 후 위치 안정화 폴링 (연속 3회 < 0.05mm)"""
        axis_idx = {'x': 0, 'y': 1, 'z': 2}[axis]
        stable, _ = self._wait_for_position_stable(
            axis_idx=axis_idx, min_delta=0, timeout=timeout,
            cancel_callback=lambda: self.tabLaserCalibration._z_adjust_cancel)
        return stable

    def _on_calib_save_pos(self):
        """TCP 위치 저장 (pos1)"""
        if not self._require_robot():
            return

        pose = self.robot.read_current_pose()
        if pose is None:
            self._log("[Calib] 위치 읽기 실패")
            return

        tab = self.tabLaserCalibration
        tab.set_current_pose(*pose[:6])
        tab._calib_pos1 = (pose[0], pose[2])  # x, z
        tab._update_calib_step(3, f"위치 1 저장: X={pose[0]:.2f}, Z={pose[2]:.2f}")
        self._log(f"[Calib] 위치 1 저장: X={pose[0]:.2f}, Z={pose[2]:.2f}")

    def _on_calib_move_x_adjust_z(self):
        """X 이동 + Z 재조정"""
        if not self._require_robot():
            return

        tab = self.tabLaserCalibration
        x_step = tab.spinXMoveStep.value()
        tab._update_calib_step(4, f"X {x_step}mm 이동 중...")
        self._log(f"[Calib] X {x_step}mm 이동 시작")

        try:
            # X 이동
            success, msg = self.robot.send_base_linear(
                'x', x_step,
                process_events_callback=QApplication.processEvents)
            if not success:
                self._log(f"[Calib] X 이동 실패: {msg}")
                tab._update_calib_step(0, f"X 이동 실패: {msg}")
                return

            self._calib_wait_for_position_stable('x')
            time.sleep(0.2)  # PRS 클린업 대기

            # Z 재조정
            tab._update_calib_step(4, "Z 재조정 중...")
            tab.set_z_adjust_status(True)
            z_result = self._calib_adjust_z_to_target()
            if z_result is None:  # 감도 측정됨 → 바로 재시도
                z_result = self._calib_adjust_z_to_target()
            tab.set_z_adjust_status(False)

            if z_result:
                tab._update_calib_step(4, "X 이동 + Z 재조정 완료")
                self._log("[Calib] X 이동 + Z 재조정 완료")
            else:
                tab._update_calib_step(0, "Z 재조정 실패")
                self._log("[Calib] Z 재조정 실패")
        except Exception as e:
            self._log(f"[Calib] X 이동 + Z 재조정 오류: {e}")
            tab._update_calib_step(0, f"오류: {e}")
            tab.set_z_adjust_status(False)

    def _on_calib_save_compare(self):
        """비교 저장 (pos2 저장 + 테이블에 추가)"""
        if not self._require_robot():
            return

        tab = self.tabLaserCalibration
        if tab._calib_pos1 is None:
            self._log("[Calib] 위치 1이 저장되지 않았습니다. 먼저 '3. 위치 저장'을 실행하세요.")
            QMessageBox.warning(self, "오류", "위치 1이 저장되지 않았습니다.")
            return

        pose = self.robot.read_current_pose()
        if pose is None:
            self._log("[Calib] 위치 읽기 실패")
            return

        x1, z1 = tab._calib_pos1
        x2, z2 = pose[0], pose[2]
        tab._add_calib_row(x1, z1, x2, z2)

        dx = x2 - x1
        dz = z2 - z1
        tab._update_calib_step(5, f"비교 저장: ΔX={dx:.2f}, ΔZ={dz:.2f}")
        self._log(f"[Calib] 비교 저장: X1={x1:.2f}, Z1={z1:.2f}, X2={x2:.2f}, Z2={z2:.2f}, ΔX={dx:.2f}, ΔZ={dz:.2f}")

        # 다음 반복을 위해 pos1 초기화
        tab._calib_pos1 = None

    def _on_calib_auto(self):
        """자동 캘리브레이션 시작"""
        if not self.robot or not self.robot.is_connected:
            QMessageBox.warning(self, "오류", "로봇이 연결되지 않았습니다.")
            self.tabLaserCalibration._reset_auto_calib_ui()
            return

        tab = self.tabLaserCalibration
        self._auto_calib_state = 'ALIGNING'
        self._auto_calib_iteration = 0
        self._auto_calib_total = tab.spinRepeatCount.value()
        self._auto_calib_pos1 = None

        self._log(f"[Calib] 자동 캘리브레이션 시작 ({self._auto_calib_total}회)")
        QTimer.singleShot(100, self._auto_calib_step)

    def _auto_calib_step(self):
        """자동 캘리브레이션 상태 머신"""
        tab = self.tabLaserCalibration
        state = self._auto_calib_state

        if not tab._auto_calib_running:
            self._auto_calib_state = 'CANCELLED'
            tab._update_calib_step(0, "자동 캘리브레이션 취소됨")
            tab._reset_auto_calib_ui()
            self._log("[Calib] 자동 캘리브레이션 취소됨")
            return

        try:
            if state == 'ALIGNING':
                tab._update_calib_step(1, f"[{self._auto_calib_iteration+1}/{self._auto_calib_total}] ArUco 정렬 중...")
                result = self._calib_detect_aruco_alignment()
                if result is not None:
                    angle_ry, offset_y = result
                    # Ry 보정 (임계값 0.5° 이상일 때만)
                    if abs(angle_ry) >= 0.5:
                        self._log(f"[Calib] Ry 보정 실행: {angle_ry:.2f}°")
                        self._on_ar_tag_align_base_ry(angle_ry)
                        self._settle()
                    else:
                        self._log(f"[Calib] Ry 보정 불필요: {angle_ry:.2f}°")
                    # 재측정 후 Y 보정 (임계값 5px 이상일 때만)
                    result2 = self._calib_detect_aruco_alignment()
                    if result2 is not None:
                        _, offset_y2 = result2
                        if abs(offset_y2) >= 5.0:
                            self._log(f"[Calib] Y 보정 실행: {offset_y2:.1f}px")
                            self._on_ar_tag_align_base_y(offset_y2)
                            self._settle()
                        else:
                            self._log(f"[Calib] Y 보정 불필요: {offset_y2:.1f}px")
                else:
                    self._log("[Calib] ArUco 정렬 스킵 (마커 미검출) - Z 조정으로 진행")
                self._auto_calib_state = 'Z_ADJUSTING'
                QTimer.singleShot(500, self._auto_calib_step)

            elif state == 'Z_ADJUSTING':
                tab._update_calib_step(2, f"[{self._auto_calib_iteration+1}/{self._auto_calib_total}] Z 조정 중...")
                tab.set_z_adjust_status(True)
                success = self._calib_adjust_z_to_target()
                if success is None:  # 감도 측정됨 → 바로 재시도
                    success = self._calib_adjust_z_to_target()
                tab.set_z_adjust_status(False)

                if success is False and not tab._z_adjust_cancel:
                    self._auto_calib_state = 'ERROR'
                    tab._update_calib_step(0, "오류: Z 조정 실패")
                    tab._reset_auto_calib_ui()
                    self._log("[Calib] 자동 캘리브레이션 중단: Z 조정 실패")
                    return
                if tab._z_adjust_cancel:
                    tab._reset_auto_calib_ui()
                    return

                self._auto_calib_state = 'SAVING_POS1'
                QTimer.singleShot(100, self._auto_calib_step)

            elif state == 'SAVING_POS1':
                tab._update_calib_step(3, f"[{self._auto_calib_iteration+1}/{self._auto_calib_total}] 위치 1 저장...")
                pose = self.robot.read_current_pose()
                if pose is None:
                    self._auto_calib_state = 'ERROR'
                    tab._update_calib_step(0, "오류: 위치 읽기 실패")
                    tab._reset_auto_calib_ui()
                    return
                self._auto_calib_pos1 = (pose[0], pose[2])
                tab.set_current_pose(*pose[:6])
                self._log(f"[Calib] Auto 위치 1: X={pose[0]:.2f}, Z={pose[2]:.2f}")
                self._auto_calib_state = 'MOVING_X'
                QTimer.singleShot(100, self._auto_calib_step)

            elif state == 'MOVING_X':
                x_step = tab.spinXMoveStep.value()
                tab._update_calib_step(4, f"[{self._auto_calib_iteration+1}/{self._auto_calib_total}] X {x_step}mm 이동 중...")
                success, msg = self.robot.send_base_linear(
                    'x', x_step,
                    process_events_callback=QApplication.processEvents)
                if not success:
                    self._auto_calib_state = 'ERROR'
                    tab._update_calib_step(0, f"오류: X 이동 실패 - {msg}")
                    tab._reset_auto_calib_ui()
                    return
                self._calib_wait_for_position_stable('x')
                time.sleep(0.2)
                self._auto_calib_state = 'Z_READJUSTING'
                QTimer.singleShot(100, self._auto_calib_step)

            elif state == 'Z_READJUSTING':
                tab._update_calib_step(5, f"[{self._auto_calib_iteration+1}/{self._auto_calib_total}] Z 재조정 중...")
                tab.set_z_adjust_status(True)
                success = self._calib_adjust_z_to_target()
                if success is None:  # 감도 측정됨 → 바로 재시도
                    success = self._calib_adjust_z_to_target()
                tab.set_z_adjust_status(False)

                if success is False and not tab._z_adjust_cancel:
                    self._auto_calib_state = 'ERROR'
                    tab._update_calib_step(0, "오류: Z 재조정 실패")
                    tab._reset_auto_calib_ui()
                    self._log("[Calib] 자동 캘리브레이션 중단: Z 재조정 실패")
                    return
                if tab._z_adjust_cancel:
                    tab._reset_auto_calib_ui()
                    return

                self._auto_calib_state = 'SAVING_POS2'
                QTimer.singleShot(100, self._auto_calib_step)

            elif state == 'SAVING_POS2':
                pose = self.robot.read_current_pose()
                if pose is None:
                    self._auto_calib_state = 'ERROR'
                    tab._update_calib_step(0, "오류: 위치 읽기 실패")
                    tab._reset_auto_calib_ui()
                    return

                pos2 = (pose[0], pose[2])
                tab._add_calib_row(
                    self._auto_calib_pos1[0], self._auto_calib_pos1[1],
                    pos2[0], pos2[1])

                dx = pos2[0] - self._auto_calib_pos1[0]
                dz = pos2[1] - self._auto_calib_pos1[1]
                self._log(f"[Calib] Auto 반복 {self._auto_calib_iteration+1}: ΔX={dx:.2f}, ΔZ={dz:.2f}")

                self._auto_calib_iteration += 1
                tab.progressCalib.setValue(self._auto_calib_iteration)

                if self._auto_calib_iteration < self._auto_calib_total:
                    tab._update_calib_step(0, f"반복 {self._auto_calib_iteration}/{self._auto_calib_total} 완료")
                    self._auto_calib_state = 'ALIGNING'
                    QTimer.singleShot(500, self._auto_calib_step)
                else:
                    self._auto_calib_state = 'COMPLETE'
                    tab._auto_calib_running = False
                    tab._reset_auto_calib_ui()
                    tab._update_calib_step(0, f"캘리브레이션 완료 ({self._auto_calib_total}회)")
                    self._log(f"[Calib] 자동 캘리브레이션 완료 ({self._auto_calib_total}회)")

        except Exception as e:
            self._log(f"[Calib] 자동 캘리브레이션 오류: {e}")
            tab._update_calib_step(0, f"오류: {e}")
            tab._reset_auto_calib_ui()

    def _on_calib_cancel(self):
        """캘리브레이션 취소"""
        tab = self.tabLaserCalibration
        tab._z_adjust_cancel = True
        tab._auto_calib_running = False
        tab._reset_auto_calib_ui()
        tab._update_calib_step(0, "취소됨")
        self._log("[Calib] 캘리브레이션 취소")

    def _on_camera_frame(self, frame: np.ndarray):
        """카메라 프레임 수신 시 호출 (CameraManager signal)"""
        # 현재 활성 탭 인덱스
        current_tab = self.tabWidget.currentIndex()

        # 비전 탭이 활성화된 경우
        if current_tab == 1:  # 비전 탭
            # 비전 프로세싱
            processed_frame = frame.copy()

            # Aruco 감지 (체크박스 활성화 시에만)
            if self.tabVision.is_aruco_detect_enabled():
                processed_frame, markers = self.vision_manager.detect_markers(processed_frame)

                # AR 태그 포즈를 UI에 표시
                if markers and len(markers) > 0:
                    marker = markers[0]  # 첫 번째 마커

                    # 1. Camera 좌표: 카메라 기준 마커 위치 (tvec)
                    tvec = marker['tvec']
                    tvec_euler = self._rotation_matrix_to_euler(marker['rotation_matrix'])
                    self.tabVision.update_pose_camera(
                        tvec[0] * 1000,  # m -> mm
                        tvec[1] * 1000,
                        tvec[2] * 1000,
                        tvec_euler[0], tvec_euler[1], tvec_euler[2]
                    )

                    # 2. World 좌표: Tool Frame 1 기준 로봇 포즈 표시
                    if self.robot and self.robot.is_connected:
                        try:
                            robot_pose = self.robot.read_current_pose()
                            if robot_pose:
                                # Tool Frame 1 (카메라) 기준 로봇 TCP 포즈 표시
                                self.tabVision.update_pose_world(
                                    robot_pose[0], robot_pose[1], robot_pose[2],
                                    robot_pose[3], robot_pose[4], robot_pose[5]
                                )
                        except Exception as e:
                            print(f"[Vision] 로봇 포즈 읽기 오류: {e}")

                    # ArUco ID 업데이트
                    self.tabVision.update_detection_result(tag_id=marker['id'])
                else:
                    self.tabVision.clear_pose_display()
                    self.tabVision.update_detection_result()

            # 탭에 프레임 전달
            self.tabVision.set_current_frame(frame)
            self.tabVision.display_frame(processed_frame)

        # 캘리브레이션 탭이 활성화된 경우
        elif current_tab == 2:  # 카메라 캘리브레이션 탭
            self.tabCalibration.set_current_frame(frame)
            # Depth 모드이면 depth 프레임 표시
            if self.tabCalibration.is_depth_mode():
                depth_frame = self.tabCalibration.current_depth_frame
                if depth_frame is not None:
                    self.tabCalibration.display_frame(depth_frame)
            else:
                processed_frame = self.tabCalibration.process_frame(frame)
                self.tabCalibration.display_frame(processed_frame)

        # ArUco 신뢰성 검증 탭이 활성화된 경우
        elif current_tab == 3:  # ArUco 신뢰성 검증 탭
            self.tabArucoReliability.update_frame(frame)

        # Eye in Hand 탭이 활성화된 경우
        elif current_tab == 4:  # Eye in Hand 탭
            self.tabEyeInHand.set_current_frame(frame)
            processed_frame = self.tabEyeInHand.process_frame(frame)
            self.tabEyeInHand.display_frame(processed_frame)

        # 레이저 캘리브레이션 탭이 활성화된 경우
        elif current_tab == 6:  # 레이저 캘리브레이션 탭
            # ArUco 마커 검출 → 레이저 ROI 설정 + 오버레이
            try:
                tab_ar = self.tabArucoReliability
                markers = self.vision_manager.detect_marker_centers(
                    frame, tab_ar.camera_matrix, tab_ar.dist_coeffs, estimate_pose=True)
                self.tabLaserCalibration.set_markers(markers)
                # 오버레이 적용 전 순수 원본 저장 (ArUco 검출 전용)
                self.tabLaserCalibration._raw_frame = frame.copy()
                # ArUco 오버레이 표시 (정렬 후 결과 확인용)
                if self.tabLaserCalibration._show_aruco_overlay:
                    tag_id1 = tab_ar.spinTagID1.value()
                    tag_id2 = tab_ar.spinTagID2.value()
                    alignment = compute_dual_alignment(markers, tag_id1, tag_id2, frame.shape[1], frame.shape[0])
                    frame = draw_dual_marker_overlay(frame, markers, tag_id1, tag_id2, alignment)
            except Exception:
                self.tabLaserCalibration.set_markers([])
            self.tabLaserCalibration.update_frame(frame)

        # 레이저 스캔 탭이 활성화된 경우
        elif current_tab == 8:  # 레이저 스캔 탭
            self.tabLaserScan.update_frame(frame)

    def detect_aruco_tag(self, tag_id: int, timeout: float = 10.0, num_samples: int = 10):
        """특정 Aruco 태그 감지 (VisionManager 위임)"""
        return self.vision_manager.detect_tag(tag_id, timeout, num_samples)

    def _update_align_status(self, status: str):
        """정렬 상태 업데이트 (AlignmentService signal 핸들러)"""
        self.tabVision.update_align_status(status)

    # ==================== 실행 모니터 ====================

    def _on_run(self):
        """실행"""
        self._log("실행 시작")
        self.labelExecutionState.setText("실행 중")
        self.labelExecutionState.setStyleSheet("color: green; font-weight: bold;")
        # TODO: Job 실행

    def _on_pause(self):
        """일시정지"""
        self._log("일시정지")
        self.labelExecutionState.setText("일시정지")
        self.labelExecutionState.setStyleSheet("color: orange; font-weight: bold;")

    def _on_stop(self):
        """정지"""
        self._log("정지")
        self.labelExecutionState.setText("정지됨")
        self.labelExecutionState.setStyleSheet("color: red; font-weight: bold;")

    def _on_step(self):
        """스텝 실행"""
        self._log("스텝 실행")
        # TODO: 단일 Job 실행

    def _on_emergency_stop(self):
        """비상 정지"""
        self._log("비상 정지!")
        self.labelExecutionState.setText("비상 정지")
        self.labelExecutionState.setStyleSheet("color: red; font-weight: bold;")
        QMessageBox.warning(self, "비상 정지", "비상 정지가 활성화되었습니다.")

    def _on_clear_log(self):
        """로그 지우기"""
        self.textExecutionLog.clear()

    def _on_save_log(self):
        """로그 저장"""
        filename, _ = QFileDialog.getSaveFileName(
            self, "로그 저장", "", "텍스트 파일 (*.txt)")
        if filename:
            with open(filename, 'w', encoding='utf-8') as f:
                f.write(self.textExecutionLog.toPlainText())
            self._log(f"로그 저장: {filename}")

    # ==================== 테스트 탭 (충전건 결합) ====================

    def _on_test_toggle_cameras(self, checked):
        """테스트 탭 - 양쪽 카메라 토글"""
        if checked:
            if not self.ds435_camera_manager.is_running:
                self.ds435_camera_manager.start()
            if not self.arducam_manager.is_running:
                self.arducam_manager.start()
            self.btnTestToggleCameras.setText("카메라 정지")
            self._log("[Test] DS435 + ArduCam 카메라 시작")
        else:
            if self.ds435_camera_manager.is_running:
                self.ds435_camera_manager.stop()
            if self.arducam_manager.is_running:
                self.arducam_manager.stop()
            self.btnTestToggleCameras.setText("카메라 구동")
            self._log("[Test] DS435 + ArduCam 카메라 정지")

    _test_ds435_frame_count = 0
    _test_arducam_frame_count = 0
    _test_ds435_last_markers = []
    _test_arducam_last_markers = []

    def _test_display_fixed(self, frame, label):
        """테스트 탭 프레임을 640x360 고정 크기로 라벨에 표시"""
        if frame is None:
            return
        resized = cv2.resize(frame, (640, 360), interpolation=cv2.INTER_AREA)
        rgb = cv2.cvtColor(resized, cv2.COLOR_BGR2RGB)
        h, w, ch = rgb.shape
        from PyQt5.QtGui import QImage, QPixmap
        q_image = QImage(rgb.data, w, h, ch * w, QImage.Format_RGB888)
        label.setPixmap(QPixmap.fromImage(q_image))

    def _on_test_ds435_frame(self, frame):
        """테스트 탭 DS435 프레임: ArUco 검출 + depth + 오버레이"""
        if not self.tabTest.isVisible():
            return
        self._test_ds435_frame_count += 1
        if self._test_ds435_frame_count % 3 == 0:
            intrinsics = self.ds435_camera_manager.intrinsics
            if intrinsics is not None:
                if not hasattr(self, '_test_aruco_estimator'):
                    from Sensor.aruco.aruco_detector import ArucoCameraPoseEstimator
                    self._test_aruco_estimator = ArucoCameraPoseEstimator()
                results = self._test_aruco_estimator.detect_and_estimate_pose(
                    frame, intrinsics)
                self._test_ds435_last_markers = results if results else []
        distances = None
        if self._test_ds435_last_markers:
            distances = []
            for m in self._test_ds435_last_markers:
                corners = m['corners']
                crn = corners[0] if len(corners.shape) == 3 else corners
                center = np.mean(crn, axis=0).astype(int)
                dist = self.ds435_camera_manager.get_distance_at(
                    int(center[0]), int(center[1]), from_color=True)
                distances.append(dist)
        overlay = self.tabStereoCalibration._draw_markers(
            frame, self._test_ds435_last_markers, distances)
        self._test_display_fixed(overlay, self.labelTestDS435View)

    def _on_test_arducam_frame(self, frame):
        """테스트 탭 ArduCam 프레임: ArUco 검출 + 오버레이"""
        if not self.tabTest.isVisible():
            return
        self._test_arducam_frame_count += 1
        if self._test_arducam_frame_count % 3 == 0:
            intrinsics = self.arducam_manager.intrinsics if self.arducam_manager else None
            if intrinsics is not None:
                if not hasattr(self, '_test_aruco_estimator'):
                    from Sensor.aruco.aruco_detector import ArucoCameraPoseEstimator
                    self._test_aruco_estimator = ArucoCameraPoseEstimator()
                results = self._test_aruco_estimator.detect_and_estimate_pose(
                    frame, intrinsics)
                self._test_arducam_last_markers = results if results else []
        overlay = self.tabStereoCalibration._draw_markers(
            frame, self._test_arducam_last_markers)
        self._test_display_fixed(overlay, self.labelTestArduCamView)

    def _test_detect_ds435_aruco_alignment(self):
        """테스트 탭 전용: DS435 프레임에서 ArUco 정렬값 검출.

        스테레오 탭의 isVisible() 제약 없이 ds435_camera_manager.last_frame을 직접 사용.

        Returns:
            (offset_y, offset_z) tuple, or None if detection fails.
        """
        try:
            frame = self.ds435_camera_manager.last_frame
            if frame is None:
                self._log("[Test] DS435 프레임 없음")
                return None

            intrinsics = self.ds435_camera_manager.intrinsics
            if intrinsics is None:
                self._log("[Test] DS435 intrinsics 없음")
                return None

            if not hasattr(self, '_test_aruco_estimator'):
                from Sensor.aruco.aruco_detector import ArucoCameraPoseEstimator
                self._test_aruco_estimator = ArucoCameraPoseEstimator()

            work = frame.copy()
            markers = self._test_aruco_estimator.detect_and_estimate_pose(
                work, intrinsics)

            # 오버레이 표시 (마커 검출 여부 무관)
            if markers and hasattr(self, 'labelTestDS435View'):
                overlay = work.copy()
                for m in markers:
                    corners = m['corners'].reshape(-1, 2).astype(int)
                    cv2.polylines(overlay, [corners], True, (0, 255, 0), 2)
                    cx, cy = corners.mean(axis=0).astype(int)
                    cv2.putText(overlay, str(m['id']), (cx, cy - 10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                from utils.common import display_frame_on_label
                display_frame_on_label(overlay, self.labelTestDS435View)

            if not markers:
                self._log("[Test] 마커 미검출")
                return None

            tag_id1, tag_id2 = self._get_target_tag_ids()

            result = self._find_dual_marker_centers(markers, tag_id1, tag_id2)
            if result is None:
                detected_ids = [m['id'] for m in markers]
                self._log(f"[Test] 대상 마커 미검출: 필요={tag_id1},{tag_id2}, 검출={detected_ids}")
                return None
            c1, c2 = result

            h, w = frame.shape[:2]
            mid_x = (c1[0] + c2[0]) / 2.0
            mid_y = (c1[1] + c2[1]) / 2.0
            offset_y = mid_x - w / 2.0
            offset_z = mid_y - h / 2.0

            self._log(f"[Test] offset_y={offset_y:.1f}px, offset_z={offset_z:.1f}px")
            return (offset_y, offset_z)

        except Exception as e:
            self._log(f"[Test] 검출 오류: {e}")
            return None

    def _test_measure_ds435_marker_offset(self, axis: str):
        """테스트 탭 전용: DS435 마커 오프셋 측정 (stereo tab 비의존)"""
        try:
            frame = self.ds435_camera_manager.last_frame
            if frame is None:
                return None
            intrinsics = self.ds435_camera_manager.intrinsics
            if intrinsics is None:
                return None
            if not hasattr(self, '_test_aruco_estimator'):
                from Sensor.aruco.aruco_detector import ArucoCameraPoseEstimator
                self._test_aruco_estimator = ArucoCameraPoseEstimator()
            markers = self._test_aruco_estimator.detect_and_estimate_pose(
                frame.copy(), intrinsics)
            if not markers:
                return None
            tag_id1, tag_id2 = self._get_target_tag_ids()
            result = self._find_dual_marker_centers(markers, tag_id1, tag_id2)
            if result is None:
                return None
            c1, c2 = result
            h, w = frame.shape[:2]
            if axis == 'y':
                mid_px = (c1[0] + c2[0]) / 2.0
                return mid_px - w / 2.0
            else:
                mid_px = (c1[1] + c2[1]) / 2.0
                return mid_px - h / 2.0
        except Exception as e:
            self._log(f"[Test] 마커 측정 오류: {e}")
            return None

    def _test_measure_ds435_marker_depth(self):
        """테스트 탭 전용: DS435 마커 depth 측정 (stereo tab 비의존)

        DS435 depth 센서는 마커 중심에서 유효 depth가 없는 경우가 많으므로
        중심 → 마커1 → 마커2 → 주변 오프셋 순으로 다중 샘플링.
        """
        try:
            frame = self.ds435_camera_manager.last_frame
            if frame is None:
                self._log("[Test] depth 측정: DS435 프레임 없음")
                return None
            intrinsics = self.ds435_camera_manager.intrinsics
            if intrinsics is None:
                self._log("[Test] depth 측정: DS435 intrinsics 없음")
                return None
            if not hasattr(self, '_test_aruco_estimator'):
                from Sensor.aruco.aruco_detector import ArucoCameraPoseEstimator
                self._test_aruco_estimator = ArucoCameraPoseEstimator()
            markers = self._test_aruco_estimator.detect_and_estimate_pose(
                frame.copy(), intrinsics)
            if not markers:
                self._log("[Test] depth 측정: 마커 검출 실패")
                return None
            tag_id1, tag_id2 = self._get_target_tag_ids()
            result = self._find_dual_marker_centers(markers, tag_id1, tag_id2)
            if result is None:
                detected_ids = [m['id'] for m in markers]
                self._log(f"[Test] depth 측정: 타겟 마커({tag_id1},{tag_id2}) 미발견, 검출된 ID: {detected_ids}")
                return None
            c1, c2 = result
            mid_x = (c1[0] + c2[0]) / 2.0
            mid_y = (c1[1] + c2[1]) / 2.0

            # 다중 샘플 포인트: 중심 → 마커1 → 마커2 → 주변 오프셋
            sample_points = [
                (int(mid_x), int(mid_y)),
                (int(c1[0]), int(c1[1])),
                (int(c2[0]), int(c2[1])),
            ]
            for dx, dy in [(0, -20), (0, 20), (-20, 0), (20, 0)]:
                sample_points.append((int(mid_x + dx), int(mid_y + dy)))

            for sx, sy in sample_points:
                depth = self.ds435_camera_manager.get_distance_at(sx, sy, from_color=True)
                if depth is not None and depth > 0:
                    self._log(f"[Test] depth 측정: {depth:.1f}mm (sample=({sx},{sy}))")
                    return depth

            self._log(f"[Test] depth 측정: 모든 샘플 포인트 depth=None (mid=({int(mid_x)},{int(mid_y)}))")
            return None
        except Exception as e:
            self._log(f"[Test] depth 측정 오류: {e}")
            return None

    def _test_detect_full_alignment(self, max_retries=3):
        """테스트 탭 전용: ArduCam 전체 정렬 검출 (stereo tab 비의존)"""
        frame = self.arducam_manager.last_frame if self.arducam_manager else None
        ac_mat, ac_dist = self._get_arducam_intrinsics()
        return self._detect_dual_alignment(
            frame, None, max_retries=max_retries,
            frame_source=lambda: self.arducam_manager.last_frame if self.arducam_manager else None,
            camera_matrix=ac_mat, dist_coeffs=ac_dist)

    def _test_align_aruco_combined(self):
        """테스트 탭 전용: ArduCam 통합 정렬 (stereo tab 비의존)"""
        if not self._require_robot():
            return

        detect = self._test_detect_full_alignment
        status = lambda step, msg: self.labelTestAlignStatus.setText(msg)

        try:
            self._log("[Test 통합] 통합 정렬 시작 (Y:Z축 → X:Ry+BaseY)")
            QApplication.processEvents()

            # 1) Y 정렬: Z축 이동으로 이미지 세로 중심 정렬
            status(1, "통합 정렬: Y축(세로) 보정 중...")
            z_ok, z_msg = self._stereo_align_z_core(
                None, detect_func=detect, status_func=status)
            self._log(f"[Test 통합] Y: {z_msg}")

            # 2) X 정렬: Ry 회전 + Base Y 이동으로 가로 중심 정렬
            status(1, "통합 정렬: X축(가로) 보정 중...")
            y_ok, y_msg = self._stereo_align_y_core(
                None, detect_func=detect, status_func=status)
            self._log(f"[Test 통합] X: {y_msg}")

            # 3) 미세 정렬
            pre_fine = detect()
            if pre_fine is not None:
                ARDUCAM_PX_PER_MM = 13.0

                def _fine_z_measure():
                    a = detect(max_retries=1)
                    return a.offset_z if a is not None and a.offset_z is not None else None

                def _fine_y_measure():
                    a = detect(max_retries=1)
                    return a.offset_y if a is not None and a.offset_y is not None else None

                FINE_THRESHOLD_PX = 0.5

                oz_pre = pre_fine.offset_z if pre_fine.offset_z is not None else 0
                if abs(oz_pre) > FINE_THRESHOLD_PX:
                    status(1, f"미세 정렬 Z: {oz_pre:.1f}px")
                    self._fine_align_axis('z', ARDUCAM_PX_PER_MM, _fine_z_measure,
                                          log_prefix="[Test Fine Z]")
                else:
                    self._log(f"[Test Fine Z] 이미 수렴: {oz_pre:.1f}px")

                pre_fine2 = detect(max_retries=1)
                oy_pre = pre_fine2.offset_y if pre_fine2 is not None and pre_fine2.offset_y is not None else 0
                if abs(oy_pre) > FINE_THRESHOLD_PX:
                    status(1, f"미세 정렬 Y: {oy_pre:.1f}px")
                    self._fine_align_axis('y', ARDUCAM_PX_PER_MM, _fine_y_measure,
                                          log_prefix="[Test Fine Y]")
                else:
                    self._log(f"[Test Fine Y] 이미 수렴: {oy_pre:.1f}px")

            # 4) 최종 결과
            final = detect()
            if final is not None:
                ry_f = self._get_effective_ry(final)
                oy_f = final.offset_y if final.offset_y is not None else 0
                oz_f = final.offset_z if final.offset_z is not None else 0
                msg = f"통합 정렬 완료: Ry={ry_f:.2f}°, dY={oy_f:.1f}px, dZ={oz_f:.1f}px"
                self._log(f"[Test 통합] {msg}")
                status(0, msg)
            else:
                status(0, "최종 검출 실패")

        except Exception as e:
            self._log(f"[Test 통합] 오류: {e}")
            status(0, f"오류: {e}")

    def _on_test_align_ds435(self):
        """테스트 탭 DS435 ArUco 정렬"""
        if not self._require_robot():
            self.labelTestAlignStatus.setText("로봇 미연결")
            return

        TARGET_RX, TARGET_RY, TARGET_RZ = 90.0, 0.0, 90.0

        self.labelTestAlignStatus.setText("Detection Pose로 이동 중...")
        try:
            current_pose = self.robot.read_current_pose()
            if not current_pose:
                self.labelTestAlignStatus.setText("현재 자세 읽기 실패")
                return

            rx, ry, rz = current_pose[3], current_pose[4], current_pose[5]
            need_move = (abs(rx - TARGET_RX) > 0.5 or
                         abs(ry - TARGET_RY) > 0.5 or
                         abs(rz - TARGET_RZ) > 0.5)

            if need_move:
                self._log(f"[Test] Detection Pose 이동: "
                          f"Rx={rx:.1f}→{TARGET_RX}, Ry={ry:.1f}→{TARGET_RY}, Rz={rz:.1f}→{TARGET_RZ}")
                target = list(current_pose)
                target[3] = TARGET_RX
                target[4] = TARGET_RY
                target[5] = TARGET_RZ

                regs = [self.robot.to_uint16(int(round(v * 10))) for v in target]
                self.robot.write_registers(self.robot.REGISTER_POSE_MAIN, regs)
                self.robot.write_command(self.robot.CMD_MOVE_TO_POSE)

                result = self.robot.wait_for_done(
                    process_events_callback=QApplication.processEvents)
                if not result[0]:
                    self.labelTestAlignStatus.setText(f"Detection Pose 이동 실패: {result[1]}")
                    return
                self._settle(0.5)
                self._log("[Test] Detection Pose 이동 완료")
            else:
                self._log("[Test] Detection Pose 이동 불필요 (이미 목표 자세)")

        except Exception as e:
            self._log(f"[Test] Detection Pose 이동 오류: {e}")
            self.labelTestAlignStatus.setText(f"Detection Pose 이동 오류: {e}")
            return

        self.labelTestAlignStatus.setText("ArUco 마커 검출 중...")
        try:
            result = self._test_detect_ds435_aruco_alignment()
            if result is None:
                self.labelTestAlignStatus.setText("ArUco 검출 실패 - 마커를 확인하세요")
                return
            offset_y, offset_z = result

            if abs(offset_y) >= 5.0:
                self.labelTestAlignStatus.setText(f"Y 보정 중: {offset_y:.1f}px")
                self._ds435_adaptive_align('y', offset_y,
                    measure_func=self._test_measure_ds435_marker_offset)
                self._settle()
            else:
                self._log(f"[Test] Y 보정 불필요: {offset_y:.1f}px")

            result2 = self._test_detect_ds435_aruco_alignment()
            if result2 is not None:
                _, offset_z2 = result2
                if abs(offset_z2) >= 5.0:
                    self.labelTestAlignStatus.setText(f"Z 보정 중: {offset_z2:.1f}px")
                    self._ds435_adaptive_align('z', offset_z2,
                        measure_func=self._test_measure_ds435_marker_offset)
                    self._settle()
                else:
                    self._log(f"[Test] Z 보정 불필요: {offset_z2:.1f}px")

            result3 = self._test_detect_ds435_aruco_alignment()
            if result3 is not None:
                oy_f, oz_f = result3
                msg = f"정렬 완료: dY={oy_f:.1f}px, dZ={oz_f:.1f}px"
                self._log(f"[Test] {msg}")
                self.labelTestAlignStatus.setText(msg)
            else:
                self.labelTestAlignStatus.setText("정렬 후 재검출 실패")

        except Exception as e:
            self._log(f"[Test] ArUco 정렬 오류: {e}")
            self.labelTestAlignStatus.setText(f"오류: {e}")

    def _on_test_handoff_ds435_to_arducam(self):
        """테스트 탭 DS435 → ArduCam 핸드오프"""
        if not self._require_robot():
            self.labelTestAlignStatus.setText("로봇 미연결")
            return

        try:
            from services.stereo_offset_calculator import StereoOffsetCalculator

            cfg = self._load_charging_config()
            TARGET_DEPTH_MM = cfg.get('handoff', {}).get('target_depth_mm', 370.0)
            DEPTH_TOLERANCE_MM = cfg.get('handoff', {}).get('depth_tolerance_mm', 5.0)

            # --- sweep 데이터 로드 ---
            self.labelTestAlignStatus.setText("① sweep 데이터 로드 중...")
            QApplication.processEvents()

            data_dir = os.path.join(os.path.dirname(__file__), '..', 'data')
            sweep_path = StereoOffsetCalculator.find_latest_sweep(data_dir)
            if sweep_path is None:
                self.labelTestAlignStatus.setText("sweep 데이터 없음 - 스윕 먼저 실행하세요")
                self._log("[Test Handoff] data/stereo/에 sweep JSON 파일 없음")
                return

            calc = StereoOffsetCalculator(sweep_path, self.arducam_manager.intrinsics)
            if not calc.is_valid:
                self.labelTestAlignStatus.setText("sweep 데이터 카메라 불일치 - 스윕 재실행 필요")
                self._log("[Test Handoff] sweep 데이터가 현재 카메라와 불일치")
                return
            raw_y, raw_z = calc.camera_offset_mm
            cam_offset_y = -raw_y
            cam_offset_z = -raw_z
            self._log(f"[Test Handoff] camera_offset(raw): dY={raw_y:.2f}mm, dZ={raw_z:.2f}mm → 적용: dY={cam_offset_y:.2f}mm, dZ={cam_offset_z:.2f}mm")

            # 0단계: Detection Pose 이동 (Rx=90, Ry=0, Rz=90)
            TARGET_RX, TARGET_RY, TARGET_RZ = 90.0, 0.0, 90.0
            self.labelTestAlignStatus.setText("⓪ Detection Pose 이동 중...")
            QApplication.processEvents()

            current_pose = self.robot.read_current_pose()
            if not current_pose:
                self.labelTestAlignStatus.setText("현재 자세 읽기 실패")
                return

            rx, ry, rz = current_pose[3], current_pose[4], current_pose[5]
            need_move = (abs(rx - TARGET_RX) > 0.5 or
                         abs(ry - TARGET_RY) > 0.5 or
                         abs(rz - TARGET_RZ) > 0.5)

            if need_move:
                self._log(f"[Test Handoff] Detection Pose 이동: "
                          f"Rx={rx:.1f}→{TARGET_RX}, Ry={ry:.1f}→{TARGET_RY}, Rz={rz:.1f}→{TARGET_RZ}")
                target = list(current_pose)
                target[3] = TARGET_RX
                target[4] = TARGET_RY
                target[5] = TARGET_RZ

                regs = [self.robot.to_uint16(int(round(v * 10))) for v in target]
                self.robot.write_registers(self.robot.REGISTER_POSE_MAIN, regs)
                self.robot.write_command(self.robot.CMD_MOVE_TO_POSE)

                result = self.robot.wait_for_done(
                    process_events_callback=QApplication.processEvents)
                if not result[0]:
                    self.labelTestAlignStatus.setText(f"Detection Pose 이동 실패: {result[1]}")
                    return
                self._settle(0.5)
                self._log("[Test Handoff] Detection Pose 이동 완료")
            else:
                self._log("[Test Handoff] Detection Pose 이동 불필요 (이미 목표 자세)")

            # 1단계: DS435 마커 중심 정렬
            self.labelTestAlignStatus.setText("① DS435 마커 중심 정렬 중...")
            QApplication.processEvents()
            self._log("[Test Handoff] === 1단계: DS435 마커 중심 정렬 ===")

            result = self._test_detect_ds435_aruco_alignment()
            if result is None:
                self.labelTestAlignStatus.setText("DS435 마커 검출 실패")
                return

            offset_y, offset_z = result
            if abs(offset_y) >= 5.0:
                self.labelTestAlignStatus.setText(f"① DS435 Y 보정: {offset_y:.1f}px")
                self._ds435_adaptive_align('y', offset_y,
                    measure_func=self._test_measure_ds435_marker_offset)
                self._settle()

            result2 = self._test_detect_ds435_aruco_alignment()
            if result2 is not None:
                _, offset_z2 = result2
                if abs(offset_z2) >= 5.0:
                    self.labelTestAlignStatus.setText(f"① DS435 Z 보정: {offset_z2:.1f}px")
                    self._ds435_adaptive_align('z', offset_z2,
                        measure_func=self._test_measure_ds435_marker_offset)
                    self._settle()

            result3 = self._test_detect_ds435_aruco_alignment()
            if result3 is not None:
                fy, fz = result3
                self._log(f"[Test Handoff] 1단계 완료: dY={fy:.1f}px, dZ={fz:.1f}px")

            # 2단계: X축 거리 유지
            self.labelTestAlignStatus.setText(f"② X축 거리 {TARGET_DEPTH_MM:.0f}mm 조정 중...")
            QApplication.processEvents()
            self._log(f"[Test Handoff] === 2단계: X축 거리 {TARGET_DEPTH_MM:.0f}mm 조정 ===")

            depth = None
            for attempt in range(3):
                self._settle()
                depth = self._test_measure_ds435_marker_depth()
                self._log(f"[Test Handoff] depth 시도 {attempt+1}/3: {depth}")
                if depth is not None and depth > 0:
                    break

            if depth is not None and depth > 0:
                FINE_TOLERANCE_MM = 0.1
                MAX_DEPTH_ITER = 5
                DAMPING = 0.7  # 진동 방지 감쇠 계수
                NUM_SAMPLES = 3  # 평균 측정 횟수

                def _avg_depth():
                    """다중 측정 평균 (노이즈 감소)"""
                    readings = []
                    for _ in range(NUM_SAMPLES):
                        d = self._test_measure_ds435_marker_depth()
                        if d is not None and d > 0:
                            readings.append(d)
                        self._settle(0.15)
                    return sum(readings) / len(readings) if readings else None

                for d_iter in range(MAX_DEPTH_ITER):
                    depth_error = depth - TARGET_DEPTH_MM
                    abs_err = abs(depth_error)
                    self._log(f"[Test Handoff] depth iter {d_iter+1}/{MAX_DEPTH_ITER}: "
                              f"{depth:.1f}mm, 목표: {TARGET_DEPTH_MM:.0f}mm, 오차: {depth_error:+.1f}mm")

                    if abs_err < FINE_TOLERANCE_MM:
                        self._log(f"[Test Handoff] 수렴 완료 (오차 {abs_err:.1f}mm < {FINE_TOLERANCE_MM}mm)")
                        self.labelTestAlignStatus.setText(
                            f"② 수렴: {depth:.1f}mm (오차 {depth_error:+.1f}mm)")
                        break

                    x_move = depth_error * DAMPING
                    if abs(x_move) > 100.0:
                        self._log(f"[Test Handoff] X 이동 거리 초과: {x_move:.1f}mm (±100mm 제한)")
                        self.labelTestAlignStatus.setText(f"② X 이동 거리 초과: {x_move:.1f}mm")
                        break

                    self.labelTestAlignStatus.setText(
                        f"② X 이동: {x_move:+.1f}mm ({d_iter+1}/{MAX_DEPTH_ITER})")
                    QApplication.processEvents()
                    success, msg = self.robot.send_base_linear(
                        'x', x_move, wait=True,
                        process_events_callback=QApplication.processEvents)
                    if not success:
                        self._log(f"[Test Handoff] X 이동 실패: {msg}")
                        break

                    self._settle(0.5)

                    # 다중 측정 평균
                    depth = _avg_depth()
                    if depth is None:
                        self._log("[Test Handoff] 재측정 실패, 반복 중단")
                        break
                else:
                    self._log(f"[Test Handoff] {MAX_DEPTH_ITER}회 반복 후 오차: {abs(depth - TARGET_DEPTH_MM):.1f}mm")
            else:
                self._log("[Test Handoff] depth 측정 실패 — 2단계 건너뜀")
                self.labelTestAlignStatus.setText("② depth 측정 실패 — 건너뜀")

            # 3단계: camera_offset_mm 적용
            self.labelTestAlignStatus.setText(f"③ 카메라 오프셋 적용: Y={cam_offset_y:.1f}mm, Z={cam_offset_z:.1f}mm")
            QApplication.processEvents()
            self._log(f"[Test Handoff] === 3단계: 카메라 오프셋 Y={cam_offset_y:.1f}mm, Z={cam_offset_z:.1f}mm ===")

            if abs(cam_offset_y) >= 0.5:
                self.robot.send_base_linear(
                    'y', cam_offset_y, wait=True,
                    process_events_callback=QApplication.processEvents)
                self._settle()

            if abs(cam_offset_z) >= 0.5:
                self.robot.send_base_linear(
                    'z', cam_offset_z, wait=True,
                    process_events_callback=QApplication.processEvents)
                self._settle()

            self._settle(0.5)

            # 4단계: ArduCam 통합 정렬
            self.labelTestAlignStatus.setText("④ ArduCam 통합 정렬 중...")
            QApplication.processEvents()
            self._log("[Test Handoff] === 4단계: ArduCam 통합 정렬 ===")

            CONVERGE_PX = 10.0
            MAX_ITER = 3

            for iteration in range(MAX_ITER):
                self.labelTestAlignStatus.setText(f"④ 통합 정렬 ({iteration+1}/{MAX_ITER})...")
                self._test_align_aruco_combined()
                self._settle()
                check = self._test_detect_full_alignment()
                if check is not None:
                    oy = abs(check.offset_y) if check.offset_y is not None else 0
                    oz = abs(check.offset_z) if check.offset_z is not None else 0
                    if oy < CONVERGE_PX and oz < CONVERGE_PX:
                        self._log(f"[Test Handoff] 수렴 완료 (반복 {iteration+1})")
                        break

            final = self._test_detect_full_alignment()
            if final is not None:
                ry_f = self._get_effective_ry(final)
                oy_f = final.offset_y if final.offset_y is not None else 0
                oz_f = final.offset_z if final.offset_z is not None else 0
                msg = f"핸드오프 완료! Ry={ry_f:.2f}°, dY={oy_f:.1f}px, dZ={oz_f:.1f}px"
            else:
                msg = "핸드오프 완료 (최종 검출 실패)"

            self._log(f"[Test Handoff] {msg}")
            self.labelTestAlignStatus.setText(msg)

        except Exception as e:
            self._log(f"[Test Handoff] 오류: {e}")
            import traceback
            self._log(f"[Test Handoff] {traceback.format_exc()}")
            self.labelTestAlignStatus.setText(f"오류: {e}")

    # ==================== 테스트 탭 레이저 스캔 ====================

    _CHARGING_CONFIG_FILE = os.path.join(
        os.path.dirname(__file__), '..', 'config', 'charging_coupling_config.json')

    def _load_charging_config(self) -> dict:
        """충전건 결합 설정 로드"""
        try:
            with open(self._CHARGING_CONFIG_FILE, 'r', encoding='utf-8') as f:
                return json.load(f)
        except Exception as e:
            self._log(f"[Test] 충전건 설정 로드 실패: {e}")
            return {"laser_scan": {"step_mm": 2.0, "num_steps": 10}}

    def _on_test_laser_scan(self):
        """테스트 탭 레이저 스캔 시작"""
        from PyQt5.QtWidgets import QMessageBox

        if not self._require_robot():
            self.labelTestAlignStatus.setText("로봇 미연결")
            return

        # ArduCam 확인/시작
        if self.arducam_manager is None or not self.arducam_manager.is_running:
            self._log("[Test LaserScan] ArduCam 시작 중...")
            self._on_camera_type_changed(CAMERA_ARDUCAM)
            self._on_start_camera()
            import time
            time.sleep(1.0)

        # 설정 로드
        config = self._load_charging_config()
        scan_cfg = config.get('laser_scan', {})
        step_mm = scan_cfg.get('step_mm', 2.0)
        num_steps = scan_cfg.get('num_steps', 10)
        total_distance = step_mm * num_steps

        # 확인 다이얼로그
        reply = QMessageBox.question(
            self, "레이저 스캔",
            f"Z축 레이저 스캔을 시작합니다.\n\n"
            f"스텝: {step_mm}mm × {num_steps}회 = {total_distance}mm\n"
            f"로봇이 Z축 아래로 이동합니다.\n"
            f"주변 장애물을 확인하세요.",
            QMessageBox.Ok | QMessageBox.Cancel,
            QMessageBox.Cancel,
        )
        if reply != QMessageBox.Ok:
            return

        # 기존 서비스 정리
        self._cleanup_laser_scan_service()

        # 서비스 생성 (레이저 스캔 탭의 roi/calib 재사용)
        from services.laser_scan_service import LaserScanService
        tab_ls = self.tabLaserScan

        self._laser_scan_service = LaserScanService(
            robot=self.robot,
            arducam_manager=self.arducam_manager,
            roi_config=tab_ls._roi_config,
            camera_matrix=tab_ls.camera_matrix,
            dist_coeffs=tab_ls.dist_coeffs,
            parent=self,
        )

        # 시그널 연결
        self._laser_scan_service.status_updated.connect(
            lambda msg: self.labelTestAlignStatus.setText(f"[스캔] {msg}"))
        self._laser_scan_service.progress_updated.connect(
            lambda cur, tot: self.labelTestAlignStatus.setText(
                f"[스캔] {cur}/{tot} 스텝"))
        self._laser_scan_service.scan_finished.connect(self._on_test_laser_scan_finished)
        self._laser_scan_service.scan_error.connect(self._on_test_laser_scan_error)
        self._laser_scan_service.log_message.connect(self._log)

        self.btnTestLaserScan.setEnabled(False)
        self.labelTestAlignStatus.setText("[스캔] 시작...")
        self._laser_scan_service.start(step_mm, total_distance)

    def _on_test_laser_scan_finished(self, results: dict):
        """테스트 탭 레이저 스캔 완료"""
        self.btnTestLaserScan.setEnabled(True)
        left = results.get('left', {})
        right = results.get('right', {})
        angle = results.get('angle_estimate', {})
        slope_l = left.get('slope_px_per_mm', 0) or 0
        slope_r = right.get('slope_px_per_mm', 0) or 0
        r2_l = left.get('r_squared', 0) or 0
        r2_r = right.get('r_squared', 0) or 0
        avg_angle = angle.get('estimated_deg', None)
        self._test_laser_angle_deg = avg_angle
        self.btnTestApplyRy.setEnabled(avg_angle is not None)
        angle_str = f", 각도={avg_angle:.1f}°" if avg_angle is not None else ""
        msg = (f"스캔 완료: L={slope_l:.3f}px/mm (R²={r2_l:.4f}), "
               f"R={slope_r:.3f}px/mm (R²={r2_r:.4f}){angle_str}")
        self.labelTestAlignStatus.setText(msg)
        self._log(f"[Test LaserScan] {msg}")
        self._cleanup_laser_scan_service()

    def _on_test_laser_scan_error(self, error_msg: str):
        """테스트 탭 레이저 스캔 오류"""
        self.btnTestLaserScan.setEnabled(True)
        self.labelTestAlignStatus.setText(f"스캔 오류: {error_msg}")
        self._log(f"[Test LaserScan] 오류: {error_msg}")
        self._cleanup_laser_scan_service()

    def _on_test_depth_adjust(self):
        """독립 depth 370mm 보정 (핸드오프 없이 단독 실행)"""
        if not self._require_robot():
            self.labelTestAlignStatus.setText("로봇 미연결")
            return

        try:
            cfg = self._load_charging_config()
            TARGET_DEPTH_MM = cfg.get('handoff', {}).get('target_depth_mm', 370.0)
            FINE_TOLERANCE_MM = 0.1
            MAX_DEPTH_ITER = 10
            DAMPING = 0.7
            NUM_SAMPLES = 3

            self.labelTestAlignStatus.setText(f"Depth 보정: 목표 {TARGET_DEPTH_MM:.0f}mm")
            QApplication.processEvents()
            self._log(f"[Test Depth] === Depth 보정 시작: 목표 {TARGET_DEPTH_MM:.0f}mm ===")

            def _avg_depth():
                readings = []
                for _ in range(NUM_SAMPLES):
                    d = self._test_measure_ds435_marker_depth()
                    if d is not None and d > 0:
                        readings.append(d)
                    self._settle(0.15)
                return sum(readings) / len(readings) if readings else None

            depth = _avg_depth()
            if depth is None:
                self.labelTestAlignStatus.setText("Depth 측정 실패")
                self._log("[Test Depth] 측정 실패")
                return

            for d_iter in range(MAX_DEPTH_ITER):
                depth_error = depth - TARGET_DEPTH_MM
                abs_err = abs(depth_error)
                self._log(f"[Test Depth] iter {d_iter+1}/{MAX_DEPTH_ITER}: "
                          f"{depth:.1f}mm, 오차: {depth_error:+.1f}mm")

                if abs_err < FINE_TOLERANCE_MM:
                    self._log(f"[Test Depth] 수렴 완료 (오차 {abs_err:.1f}mm)")
                    self.labelTestAlignStatus.setText(
                        f"Depth 수렴: {depth:.1f}mm (오차 {depth_error:+.1f}mm)")
                    break

                x_total = depth_error * DAMPING
                # 100mm 단위 분할 이동
                remaining = x_total
                move_ok = True
                while abs(remaining) > 0.05:
                    step = max(-100.0, min(100.0, remaining))
                    self.labelTestAlignStatus.setText(
                        f"Depth X 이동: {step:+.1f}mm (잔여: {remaining:+.1f}mm)")
                    QApplication.processEvents()
                    success, msg = self.robot.send_base_linear(
                        'x', step, wait=True,
                        process_events_callback=QApplication.processEvents)
                    if not success:
                        self._log(f"[Test Depth] X 이동 실패: {msg}")
                        move_ok = False
                        break
                    self._settle(0.3)
                    remaining -= step
                if not move_ok:
                    break

                self._settle(0.5)
                depth = _avg_depth()
                if depth is None:
                    self._log("[Test Depth] 재측정 실패")
                    break
            else:
                self._log(f"[Test Depth] {MAX_DEPTH_ITER}회 후 오차: {abs(depth - TARGET_DEPTH_MM):.1f}mm")

        except Exception as e:
            self._log(f"[Test Depth] 오류: {e}")
            self.labelTestAlignStatus.setText(f"Depth 오류: {e}")

    def _on_test_apply_ry(self):
        """레이저 스캔 각도를 Rx에 적용"""
        if self._test_laser_angle_deg is None:
            self.labelTestAlignStatus.setText("스캔 결과 없음 — 레이저 스캔 먼저 실행")
            return
        if not self._require_robot():
            self.labelTestAlignStatus.setText("로봇 미연결")
            return

        angle = self._test_laser_angle_deg
        try:
            current_pose = self.robot.read_current_pose()
            if not current_pose:
                self.labelTestAlignStatus.setText("현재 자세 읽기 실패")
                return

            current_rx = current_pose[3]
            new_rx = current_rx + angle
            self._log(f"[Test Rx] 현재 Rx={current_rx:.2f}°, 보정={angle:+.2f}°, 목표={new_rx:.2f}°")
            self.labelTestAlignStatus.setText(
                f"Rx 보정: {current_rx:.2f}° → {new_rx:.2f}° ({angle:+.2f}°)")
            QApplication.processEvents()

            success, msg = self.robot.send_base_rotate(
                'rx', angle, wait=True,
                process_events_callback=QApplication.processEvents)

            if success:
                self._settle(0.3)
                after_pose = self.robot.read_current_pose()
                after_rx = after_pose[3] if after_pose else None
                self._log(f"[Test Rx] 보정 완료: Rx={after_rx:.2f}°" if after_rx is not None
                          else "[Test Rx] 보정 완료 (자세 읽기 실패)")
                self.labelTestAlignStatus.setText(
                    f"Rx 보정 완료: {after_rx:.2f}°" if after_rx is not None
                    else "Rx 보정 완료")
            else:
                self._log(f"[Test Rx] 보정 실패: {msg}")
                self.labelTestAlignStatus.setText(f"Rx 보정 실패: {msg}")
        except Exception as e:
            self._log(f"[Test Rx] 오류: {e}")
            self.labelTestAlignStatus.setText(f"Rx 보정 오류: {e}")

    _COUPLING_CONFIG_FILE = os.path.join(
        os.path.dirname(__file__), '..', 'config', 'charging_gun_coupling.json')

    def _stop_cameras_for_movement(self):
        """이동 전 카메라 정지 및 버튼 상태 동기화"""
        self._stop_all_cameras()
        self.btnTestToggleCameras.setChecked(False)
        self.btnTestToggleCameras.setText("카메라 구동")

    def _on_test_move_to_entrance(self):
        """현재 위치 + delta → 입구 절대 좌표 계산 후 movel"""
        if not self._require_robot():
            self.labelTestAlignStatus.setText("로봇 미연결")
            return

        self._stop_cameras_for_movement()

        try:
            with open(self._COUPLING_CONFIG_FILE, 'r', encoding='utf-8') as f:
                cfg = json.load(f)
        except Exception as e:
            self.labelTestAlignStatus.setText(f"config 로드 실패: {e}")
            return

        d = cfg.get('offsets', {}).get('corrected_to_entrance', {})
        if not d:
            self.labelTestAlignStatus.setText("config에 corrected_to_entrance 없음")
            return

        try:
            pose = self.robot.read_current_pose()
            if not pose or len(pose) < 6:
                self.labelTestAlignStatus.setText("현재 자세 읽기 실패")
                return

            x, y, z, rx, ry, rz = pose[:6]
            tx = x + d.get('x', 0)
            ty = y + d.get('y', 0)
            tz = z + d.get('z', 0)
            trx = rx + d.get('rx', 0)
            t_ry = ry + d.get('ry', 0)
            trz = rz + d.get('rz', 0)

            self._log(f"[입구이동] 현재: ({x:.1f}, {y:.1f}, {z:.1f}) → "
                      f"목표: ({tx:.1f}, {ty:.1f}, {tz:.1f})")
            self.labelTestAlignStatus.setText(
                f"입구 이동 중... → ({tx:.1f}, {ty:.1f}, {tz:.1f})")
            QApplication.processEvents()

            success, msg = self.robot.send_move_to_pose(
                tx, ty, tz, trx, t_ry, trz,
                wait=True,
                process_events_callback=QApplication.processEvents)

            if success:
                self._settle(0.3)
                after = self.robot.read_current_pose()
                if after:
                    self._log(f"[입구이동] 완료: X={after[0]:.1f} Y={after[1]:.1f} Z={after[2]:.1f} "
                              f"Rx={after[3]:.1f} Ry={after[4]:.1f} Rz={after[5]:.1f}")
                    self.labelTestAlignStatus.setText(
                        f"입구 도착: ({after[0]:.1f}, {after[1]:.1f}, {after[2]:.1f})")
                else:
                    self.labelTestAlignStatus.setText("입구 이동 완료")
            else:
                self._log(f"[입구이동] 실패: {msg}")
                self.labelTestAlignStatus.setText(f"입구 이동 실패: {msg}")

        except Exception as e:
            self._log(f"[입구이동] 오류: {e}")
            self.labelTestAlignStatus.setText(f"입구 이동 오류: {e}")

    def _on_test_tcp_z_linear(self):
        """TCP Z축 전진 (config에서 거리 읽기)"""
        if not self._require_robot():
            self.labelTestAlignStatus.setText("로봇 미연결")
            return

        self._stop_cameras_for_movement()

        try:
            with open(self._COUPLING_CONFIG_FILE, 'r', encoding='utf-8') as f:
                cfg = json.load(f)
        except Exception as e:
            self.labelTestAlignStatus.setText(f"config 로드 실패: {e}")
            return

        step = cfg.get('offsets', {}).get('entrance_to_coupling', {}).get('tool_z_step_mm', 50.0)

        try:
            self._log(f"[TCP Z] 전진: {step}mm")
            self.labelTestAlignStatus.setText(f"TCP Z 전진 중... {step}mm")
            QApplication.processEvents()

            success, msg = self.robot.send_tcp_linear(
                'z', step, wait=True,
                process_events_callback=QApplication.processEvents)

            if success:
                self._settle(0.3)
                after = self.robot.read_current_pose()
                if after:
                    self._log(f"[TCP Z] 완료: X={after[0]:.1f} Y={after[1]:.1f} Z={after[2]:.1f}")
                    self.labelTestAlignStatus.setText(
                        f"TCP Z 완료: ({after[0]:.1f}, {after[1]:.1f}, {after[2]:.1f})")
                else:
                    self.labelTestAlignStatus.setText(f"TCP Z 전진 완료 ({step}mm)")
            else:
                self._log(f"[TCP Z] 실패: {msg}")
                self.labelTestAlignStatus.setText(f"TCP Z 실패: {msg}")

        except Exception as e:
            self._log(f"[TCP Z] 오류: {e}")
            self.labelTestAlignStatus.setText(f"TCP Z 오류: {e}")

    def _on_test_release_return(self):
        """충전건 해제 복귀: TCP Z 후퇴 → 역 병진 변환으로 복귀"""
        if not self._require_robot():
            self.labelTestAlignStatus.setText("로봇 미연결")
            return

        self._stop_cameras_for_movement()

        try:
            with open(self._COUPLING_CONFIG_FILE, 'r', encoding='utf-8') as f:
                cfg = json.load(f)
        except Exception as e:
            self.labelTestAlignStatus.setText(f"config 로드 실패: {e}")
            return

        retract = cfg.get('offsets', {}).get('release_return', {}).get('tcp_z_retract_mm', 200.0)
        d = cfg.get('offsets', {}).get('corrected_to_entrance', {})
        if not d:
            self.labelTestAlignStatus.setText("config에 corrected_to_entrance 없음")
            return

        try:
            # 1단계: TCP Z 후퇴
            self._log(f"[해제복귀] TCP Z 후퇴: -{retract}mm")
            self.labelTestAlignStatus.setText(f"TCP Z 후퇴 중... -{retract}mm")
            QApplication.processEvents()

            self.robot.send_tcp_linear('z', -retract, wait=False)
            success, msg = self.robot.wait_for_done_motion_aware(
                idle_timeout=10.0, max_timeout=120.0,
                process_events_callback=QApplication.processEvents)

            if not success:
                self._log(f"[해제복귀] TCP Z 후퇴 실패: {msg}")
                self.labelTestAlignStatus.setText(f"TCP Z 후퇴 실패: {msg}")
                return

            self._settle(0.3)

            # 2단계: 역 병진 변환 (corrected_to_entrance의 역방향)
            pose = self.robot.read_current_pose()
            if not pose or len(pose) < 6:
                self.labelTestAlignStatus.setText("현재 자세 읽기 실패")
                return

            x, y, z, rx, ry, rz = pose[:6]
            tx = x - d.get('x', 0)
            ty = y - d.get('y', 0)
            tz = z - d.get('z', 0)

            self._log(f"[해제복귀] 역 병진 이동: ({x:.1f}, {y:.1f}, {z:.1f}) → "
                      f"({tx:.1f}, {ty:.1f}, {tz:.1f})")
            self.labelTestAlignStatus.setText(
                f"복귀 이동 중... ({tx:.1f}, {ty:.1f}, {tz:.1f})")
            QApplication.processEvents()

            self.robot.send_move_to_pose(tx, ty, tz, rx, ry, rz, wait=False)
            success, msg = self.robot.wait_for_done_motion_aware(
                idle_timeout=10.0, max_timeout=120.0,
                process_events_callback=QApplication.processEvents)

            if success:
                self._settle(0.3)
                after = self.robot.read_current_pose()
                if after:
                    self._log(f"[해제복귀] 완료: X={after[0]:.1f} Y={after[1]:.1f} Z={after[2]:.1f} "
                              f"Rx={after[3]:.1f} Ry={after[4]:.1f} Rz={after[5]:.1f}")
                    self.labelTestAlignStatus.setText(
                        f"복귀 완료: ({after[0]:.1f}, {after[1]:.1f}, {after[2]:.1f})")
                else:
                    self.labelTestAlignStatus.setText("복귀 완료")
            else:
                self._log(f"[해제복귀] 복귀 이동 실패: {msg}")
                self.labelTestAlignStatus.setText(f"복귀 이동 실패: {msg}")

        except Exception as e:
            self._log(f"[해제복귀] 오류: {e}")
            self.labelTestAlignStatus.setText(f"해제 복귀 오류: {e}")

    # ==================== 설정 ====================

    def _on_test_connection(self):
        """설정 탭에서 연결 테스트"""
        ip = self.editSettingsRobotIP.text()
        port = self.spinSettingsPort.value()
        timeout = self.spinTimeout.value()

        self._debug(f"연결 시도: {ip}:{port} (timeout={timeout}s)")

        # 기존 연결 해제
        if self.robot and self.robot.is_connected:
            self.robot.disconnect()
            self._debug("기존 연결 해제")

        # 새 연결
        self.robot = ModbusClient(ip=ip, port=port, timeout=timeout)
        success, message = self.robot.connect()

        if success:
            self.labelSettingsConnStatus.setText("연결됨")
            self.labelSettingsConnStatus.setStyleSheet("color: green; font-weight: bold;")
            self._debug(f"연결 성공: {message}")

            # 공통 연결 설정
            self._setup_robot_connection()

            self._debug("상태 업데이트 타이머 시작 (100ms)")

            # 테스트 읽기
            self._test_read_registers()
        else:
            self.labelSettingsConnStatus.setText("연결 실패")
            self.labelSettingsConnStatus.setStyleSheet("color: red; font-weight: bold;")
            self._debug(f"연결 실패: {message}")

    def _test_read_registers(self):
        """연결 후 레지스터 테스트 읽기"""
        if not self.robot or not self.robot.is_connected:
            return

        # 커맨드/응답 레지스터
        cmd = self.robot.read_command()
        resp = self.robot.read_response()
        self._debug(f"CMD(351)={cmd}, RESP(352)={resp}")

        # 카메라 포즈
        cam_pose = self.robot.read_camera_pose()
        if cam_pose:
            x, y, z, rx, ry, rz = cam_pose
            self._debug(f"Camera Pose: X={x:.2f}, Y={y:.2f}, Z={z:.2f}")
            self._debug(f"Camera Pose: Rx={rx:.2f}, Ry={ry:.2f}, Rz={rz:.2f}")
        else:
            self._debug("Camera Pose: 읽기 실패")

    def _on_clear_debug_log(self):
        """디버그 로그 지우기"""
        if hasattr(self, 'textDebugLog'):
            self.textDebugLog.clear()

    def _debug(self, message):
        """디버그 로그 출력"""
        timestamp = datetime.now().strftime("%H:%M:%S.%f")[:-3]
        if hasattr(self, 'textDebugLog'):
            self.textDebugLog.append(f"[{timestamp}] {message}")
        print(f"[DEBUG {timestamp}] {message}")  # 콘솔에도 출력

    def _on_save_settings(self):
        """설정 저장"""
        self._log("설정 저장")
        # TODO: YAML 저장

    def _on_load_settings(self):
        """설정 불러오기"""
        self._log("설정 불러오기")
        # TODO: YAML 로드

    def _on_load_calibration(self):
        """캘리브레이션 로드"""
        filename, _ = QFileDialog.getOpenFileName(
            self, "캘리브레이션 로드", "", "NumPy 파일 (*.npy *.npz)")
        if filename:
            self._log(f"캘리브레이션 로드: {filename}")
            # TODO: 캘리브레이션 로드

    def _on_new_calibration(self):
        """새 캘리브레이션"""
        self._log("새 캘리브레이션 시작")
        # TODO: 캘리브레이션 프로세스

    # ==================== 메뉴 ====================

    def _on_new_recipe(self):
        """새 레시피"""
        self.task_sequence.clear()
        self._refresh_task_list()
        self._log("새 레시피")

    def _on_open_recipe(self):
        """레시피 열기"""
        filename, _ = QFileDialog.getOpenFileName(
            self, "레시피 열기", "", "YAML 파일 (*.yaml *.yml)")
        if filename:
            self._log(f"레시피 열기: {filename}")
            # TODO: YAML 로드

    def _on_save_recipe(self):
        """레시피 저장"""
        self._log("레시피 저장")
        # TODO: YAML 저장

    def _on_save_recipe_as(self):
        """다른 이름으로 저장"""
        filename, _ = QFileDialog.getSaveFileName(
            self, "레시피 저장", "", "YAML 파일 (*.yaml)")
        if filename:
            self._log(f"레시피 저장: {filename}")
            # TODO: YAML 저장

    def _on_about(self):
        """정보"""
        QMessageBox.about(
            self,
            "Charging Robot Task Manager",
            "Charging Robot Task Manager v1.0\n\n"
            "자동 충전건 포지셔닝 및 그래핑 시스템\n\n"
            "KAIST"
        )

    # ==================== 포즈 저장/이동 (PoseService 위임) ====================

    def _on_save_current_pose(self):
        """현재 위치 저장 (PoseService 위임)"""
        from PyQt5.QtWidgets import QInputDialog

        name, ok = QInputDialog.getText(self, "포즈 저장", "포즈 이름:")
        if not ok or not name.strip():
            return

        pose_types = PoseService.get_pose_types()
        pose_type, ok = QInputDialog.getItem(self, "포즈 타입", "타입 선택:", pose_types, 0, False)
        if not ok:
            return

        result = self.pose_service.save_current_pose(name.strip(), pose_type)
        if not result.success:
            QMessageBox.warning(self, "오류", result.message)

    def _on_delete_pose(self):
        """저장된 포즈 삭제 (PoseService 위임)"""
        if not hasattr(self, 'listSavedPoses'):
            return
        current_item = self.listSavedPoses.currentItem()
        if not current_item:
            QMessageBox.warning(self, "경고", "삭제할 포즈를 선택하세요.")
            return

        name = current_item.data(Qt.UserRole)
        reply = QMessageBox.question(self, "삭제 확인", f"'{name}' 포즈를 삭제하시겠습니까?",
                                     QMessageBox.Yes | QMessageBox.No, QMessageBox.No)
        if reply == QMessageBox.Yes:
            result = self.pose_service.delete_pose(name)
            if not result.success:
                QMessageBox.warning(self, "오류", result.message)

    def _on_move_to_saved_pose(self):
        """저장된 위치로 이동 (PoseService 위임)"""
        if not hasattr(self, 'listSavedPoses'):
            return
        current_item = self.listSavedPoses.currentItem()
        if not current_item:
            QMessageBox.warning(self, "경고", "이동할 포즈를 선택하세요.")
            return

        name = current_item.data(Qt.UserRole)
        reply = QMessageBox.question(self, "이동 확인", f"'{name}' 위치로 이동하시겠습니까?",
                                     QMessageBox.Yes | QMessageBox.No, QMessageBox.Yes)
        if reply == QMessageBox.Yes:
            result = self.pose_service.move_to_pose(name)
            if not result.success:
                QMessageBox.warning(self, "오류", result.message)

    def _on_approach_pose(self):
        """저장된 위치의 어프로치 위치로 이동 (PoseService 위임)"""
        from PyQt5.QtWidgets import QInputDialog

        if not hasattr(self, 'listSavedPoses'):
            return
        current_item = self.listSavedPoses.currentItem()
        if not current_item:
            QMessageBox.warning(self, "경고", "이동할 포즈를 선택하세요.")
            return

        name = current_item.data(Qt.UserRole)
        distance, ok = QInputDialog.getDouble(self, "어프로치 거리", "거리 (meter):", 0.2, 0.01, 1.0, 2)
        if not ok:
            return

        result = self.pose_service.approach_pose(name, distance)
        if not result.success:
            QMessageBox.warning(self, "오류", result.message)

    # ==================== 탭 변경 핸들러 ====================

    def _stop_all_cameras(self):
        """모든 카메라 정지 및 탭 UI 초기화"""
        stopped = False
        # 글로벌 카메라 매니저 정지
        if self.camera_manager.is_running:
            self.camera_manager.stop()
            stopped = True
        # 비활성 카메라 매니저도 정지 (스테레오 캘리브레이션 등에서 독립 실행 가능)
        if self.camera_manager is not self.ds435_camera_manager and self.ds435_camera_manager.is_running:
            self.ds435_camera_manager.stop()
            stopped = True
        if self.camera_manager is not self.arducam_manager and self.arducam_manager.is_running:
            self.arducam_manager.stop()
            stopped = True
        # 각 탭의 카메라 관련 UI 초기화
        self.tabStereoCalibration.deactivate()
        self.tabLaserCalibration.deactivate()
        self.tabLaserScan.deactivate()
        if stopped:
            self._log("탭 전환: 카메라 자동 정지")

    def _on_tab_changed(self, index: int):
        """탭 변경 시 호출"""
        # 탭 전환 시 이전 탭의 카메라 기능 정지 및 버튼 초기화
        self._stop_all_cameras()

        # Vision 탭 (인덱스 1), 캘리브레이션 탭 (인덱스 2) → TF1
        # ArUco 신뢰성 검증 탭 (인덱스 3) → TF4 (TF5는 TCP 오프셋이 커서 보호정지)
        if index in [1, 2, 3]:
            if self.robot and self.robot.is_connected:
                try:
                    tf = 4 if index == 3 else 1
                    success, msg = self.robot.send_set_toolframe(tf, wait=True)
                    tab_name = "Vision" if index == 1 else ("캘리브레이션" if index == 2 else "ArUco 신뢰성 검증")
                    if success:
                        self._log(f"{tab_name} 탭 선택: Tool Frame {tf}으로 설정 완료")
                        if index == 2:
                            self.tabCalibration.update_current_toolframe(tf)
                        self._update_statusbar()
                    else:
                        self._log(f"Tool Frame 설정 실패: {msg}")
                except Exception as e:
                    self._log(f"Tool Frame 설정 오류: {e}")
        # Eye in Hand 탭 (인덱스 4)이 선택되면 Tool Frame 4로 설정
        elif index == 4:
            if self.robot and self.robot.is_connected:
                try:
                    success, msg = self.robot.send_set_toolframe(4, wait=True)
                    if success:
                        self._log("Eye in Hand 탭 선택: Tool Frame 4로 설정 완료")
                        self.tabEyeInHand.update_current_toolframe(4)
                        self._update_statusbar()
                    else:
                        self._log(f"Tool Frame 설정 실패: {msg}")
                except Exception as e:
                    self._log(f"Tool Frame 설정 오류: {e}")
        # 레이저 캘리브레이션 탭 (인덱스 6) → ArduCam 강제 전환
        elif index == 6:
            self._on_camera_type_changed(CAMERA_ARDUCAM)
        # 테스트 탭 (충전건 결합) 진입 → 카메라 자동 시작
        elif index == self.tabWidget.indexOf(self.tabTest):
            if not self.ds435_camera_manager.is_running:
                self.ds435_camera_manager.start()
            if not self.arducam_manager.is_running:
                self.arducam_manager.start()
            self.btnTestToggleCameras.setChecked(True)
            self.btnTestToggleCameras.setText("카메라 정지")
            self._log("[Test] 탭 진입: DS435 + ArduCam 카메라 자동 시작")
        else:
            # 다른 탭으로 변경 시에도 상태바 업데이트
            if self.robot and self.robot.is_connected:
                self._update_statusbar()

    # ==================== 유틸리티 ====================

    def _rotation_matrix_to_euler(self, R):
        """회전 행렬을 오일러 각도 (roll, pitch, yaw)로 변환 (도 단위)"""
        sy = np.sqrt(R[0, 0] * R[0, 0] + R[1, 0] * R[1, 0])
        singular = sy < 1e-6

        if not singular:
            x = np.arctan2(R[2, 1], R[2, 2])  # roll
            y = np.arctan2(-R[2, 0], sy)       # pitch
            z = np.arctan2(R[1, 0], R[0, 0])   # yaw
        else:
            x = np.arctan2(-R[1, 2], R[1, 1])  # roll
            y = np.arctan2(-R[2, 0], sy)       # pitch
            z = 0                              # yaw

        return np.degrees([x, y, z])

    def _log(self, message):
        """로그 메시지 추가"""
        timestamp = datetime.now().strftime("%H:%M:%S")
        log_msg = f"[{timestamp}] {message}"
        self.textExecutionLog.append(log_msg)
        print(f"[MainWindow] {log_msg}")

    def closeEvent(self, event):
        """종료 이벤트"""
        reply = QMessageBox.question(
            self, '종료 확인',
            '정말 종료하시겠습니까?',
            QMessageBox.Yes | QMessageBox.No,
            QMessageBox.No
        )

        if reply == QMessageBox.Yes:
            # 모든 카메라 정지
            if self.ds435_camera_manager.is_running:
                self.ds435_camera_manager.stop()
            if self.arducam_manager.is_running:
                self.arducam_manager.stop()

            # 타이머 정지
            if hasattr(self, 'status_timer'):
                self.status_timer.stop()

            # 로봇 연결 해제
            if self.robot and self.robot.is_connected:
                self.robot.disconnect()

            event.accept()
        else:
            event.ignore()


if __name__ == '__main__':
    import sys
    from PyQt5.QtWidgets import QApplication

    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec_())
