#!/usr/bin/env python3
"""
Charging Robot Task Manager - Main Window
"""

import os
from datetime import datetime
import numpy as np
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
from tabs import TabTaskEdit, TabVision, TabCalibration, TabArucoReliability, TabEyeInHand, TabMotionTest, TabLaserCalibration

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
        self.arducam_manager = ArduCamManager(device_index=6)
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

        # 테스트 탭
        self.btnStartTest.clicked.connect(self._on_start_test)
        self.btnStopTest.clicked.connect(self._on_stop_test)
        self.btnExportCSV.clicked.connect(self._on_export_csv)

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

        # 탭 시그널 연결
        self._connect_tab_signals()

        # 첫 번째 탭 선택
        self.tabWidget.setCurrentIndex(0)

        # 초기 카메라 타입으로 모든 탭의 라디오 버튼 동기화
        self._sync_camera_radio_buttons(CAMERA_DS435)

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
        self.tabArucoReliability.align_base_rz_requested.connect(self._on_ar_tag_align_base_rz)
        self.tabArucoReliability.align_base_y_requested.connect(self._on_ar_tag_align_base_y)

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

    # ==================== 탭 시그널 핸들러 ====================

    def _on_task_sequence_changed(self, sequence: list):
        """태스크 시퀀스 변경 시 (TabTaskEdit 시그널 핸들러)"""
        self.task_sequence = sequence

    def _on_connect_from_tab(self, ip: str, port: int):
        """로봇 연결 요청 (TabTaskEdit 시그널 핸들러)"""
        self._connect_robot(ip, port)

    def _on_read_current_position_from_tab(self):
        """현재 위치 읽기 요청 (TabTaskEdit 시그널 핸들러)"""
        if not self.robot or not self.robot.is_connected:
            QMessageBox.warning(self, "오류", "로봇이 연결되지 않았습니다.")
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
            if not self.robot or not self.robot.is_connected:
                QMessageBox.warning(self, "오류", "로봇이 연결되지 않았습니다.")
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
        import time

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
        import time

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

    def _on_disconnect(self):
        """로봇 연결 해제"""
        if self.robot:
            success, message = self.robot.disconnect()
            self._log(message)

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
        self.statusbar.showMessage("연결 해제됨")

    def _ensure_toolframe(self, tf: int) -> bool:
        """지정 툴프레임 확인/설정. 성공 시 True 반환. PRS 클린업 대기 포함."""
        if not self.robot or not self.robot.is_connected:
            return False
        success, msg = self.robot.send_set_toolframe(tf, wait=True)
        if not success:
            self._log(f"TF{tf} 설정 실패: {msg}")
            return False
        import time
        time.sleep(0.2)  # PRS 클린업 대기 (레이스 컨디션 방지)
        return True

    def _on_jog_move_from_tab(self, axis: str, distance: float):
        """조그 이동 요청 (TabTaskEdit 시그널 핸들러)"""
        if not self.robot or not self.robot.is_connected:
            QMessageBox.warning(self, "오류", "로봇이 연결되지 않았습니다.")
            return

        # Command 매핑
        cmd_map = {
            'x': self.robot.CMD_BASE_LINEAR_X,
            'y': self.robot.CMD_BASE_LINEAR_Y,
            'z': self.robot.CMD_BASE_LINEAR_Z,
        }

        cmd = cmd_map.get(axis)
        if cmd is None:
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
        if not self.robot or not self.robot.is_connected:
            QMessageBox.warning(self, "오류", "로봇이 연결되지 않았습니다.")
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
        if not self.robot or not self.robot.is_connected:
            QMessageBox.warning(self, "오류", "로봇이 연결되지 않았습니다.")
            return
        try:
            from PyQt5.QtWidgets import QApplication
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
        if not self.robot or not self.robot.is_connected:
            QMessageBox.warning(self, "오류", "로봇이 연결되지 않았습니다.")
            return
        try:
            import math
            from PyQt5.QtWidgets import QApplication

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
        """
        if not self.robot or not self.robot.is_connected:
            QMessageBox.warning(self, "오류", "로봇이 연결되지 않았습니다.")
            return

        DEAD_ZONE_PX = 3
        TEST_MM = 5.0
        MAX_CORRECTION_MM = 50.0

        if abs(dy_px) < DEAD_ZONE_PX:
            self._log("[ArUco] Base Y: 오프셋 3px 미만, 보정 불필요")
            return

        try:
            import time
            from PyQt5.QtWidgets import QApplication

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
            actual_mm = 0.0
            last_y = y_before
            stable_count = 0
            deadline = time.time() + 10.0
            while time.time() < deadline:
                time.sleep(0.2)
                QApplication.processEvents()
                pose_now = self.robot.read_current_pose()
                if pose_now is None:
                    continue
                current_y = pose_now[1]
                if abs(current_y - last_y) < 0.05:
                    stable_count += 1
                    if stable_count >= 3 and abs(current_y - y_before) > 0.1:
                        break  # 3회 연속 안정 + 실제 이동 있음
                else:
                    stable_count = 0
                last_y = current_y
            actual_mm = last_y - y_before
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
            time.sleep(0.5)
            QApplication.processEvents()

            d_final = self._measure_marker_dy_px()
            if d_final is not None:
                self._log(f"[ArUco] Base Y 보정 완료: 총 {total_mm:.1f}mm, 잔여={d_final:.1f}px")
            else:
                self._log(f"[ArUco] Base Y 보정 완료: 총 {total_mm:.1f}mm (검증 측정 실패)")

            self._update_statusbar()
        except Exception as e:
            self._log(f"[ArUco] Base Y 보정 오류: {e}")

    def _measure_marker_dy_px(self):
        """현재 카메라 프레임에서 마커 중점의 dY 픽셀 오프셋 측정"""
        try:
            frame = self.camera_manager.get_frame()
            if frame is None:
                return None

            tag_id1 = self.tabArucoReliability.spinTagID1.value()
            tag_id2 = self.tabArucoReliability.spinTagID2.value()

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

    def _on_ar_tag_align_single_axis(self, axis: str, angle: float):
        """AR Tag TCP Align - 개별 축 tool.rot 테스트"""
        if not self.robot or not self.robot.is_connected:
            QMessageBox.warning(self, "오류", "로봇이 연결되지 않았습니다.")
            return
        try:
            dbg = self.tabArucoReliability.txtAlignDebug
            import time
            from PyQt5.QtWidgets import QApplication

            # Vision→Robot 축 매핑 (카메라 마운트 기준)
            axis_map = {'rx': 'ry', 'ry': 'rz', 'rz': 'rx'}
            robot_axis = axis_map.get(axis.lower(), axis)

            # TF4로 변경
            success, msg = self.robot.send_set_toolframe(4, wait=True)
            if not success:
                self._log(f"[AR Tag] TF4 설정 실패: {msg}")
                return
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
            # TF3 복원 (조그용 기본 툴프레임)
            self._ensure_toolframe(3)
            self._update_statusbar()
        except Exception as e:
            self._log(f"[AR Tag] 오류: {e}")

    def _on_ar_tag_align_parallel(self, drx: float, dry: float, drz: float):
        """AR Tag TCP Align - TF4 기준 tool.rot 회전으로 마커 평행 정렬"""
        if not self.robot or not self.robot.is_connected:
            QMessageBox.warning(self, "오류", "로봇이 연결되지 않았습니다.")
            return

        try:
            dbg = self.tabArucoReliability.txtAlignDebug
            import time
            from PyQt5.QtWidgets import QApplication

            # TF4로 변경
            success, msg = self.robot.send_set_toolframe(4, wait=True)
            if not success:
                self._log(f"[AR Tag] TF4 설정 실패: {msg}")
                QMessageBox.warning(self, "오류", f"TF4 설정 실패:\n{msg}")
                return
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
                    dbg.append(f"  {axis.upper()} 실패: {msg}")
                    self._log(f"[AR Tag] {axis.upper()} 회전 실패: {msg}")
                    QMessageBox.warning(self, "오류", f"{axis.upper()} 회전 실패:\n{msg}")
                    return
                dbg.append(f"  {axis.upper()} → {round(angle, 1)}° 완료")
                time.sleep(0.2)  # PRS 클린업 대기

            # 정렬 후 자세
            after = self.robot.read_camera_pose()
            if after and before:
                dbg.append(f"  후) X={after[0]:.1f}, Y={after[1]:.1f}, Z={after[2]:.1f}")
                dbg.append(f"      Rx={after[3]:.2f}, Ry={after[4]:.2f}, Rz={after[5]:.2f}")
                dbg.append(f"  Δ ) dRx={after[3]-before[3]:.2f}, dRy={after[4]-before[4]:.2f}, dRz={after[5]-before[5]:.2f}")
                dbg.append("")

            self._log("[AR Tag] 마커 평행 정렬 완료")
            # TF3 복원 (조그용 기본 툴프레임)
            self._ensure_toolframe(3)
            self._update_statusbar()

        except Exception as e:
            self._log(f"[AR Tag] 정렬 오류: {e}")
            QMessageBox.warning(self, "오류", f"정렬 실패:\n{e}")

    def _update_robot_status(self):
        """로봇 상태 업데이트 (TCP 위치, 레지스터 등)"""
        if not self.robot or not self.robot.is_connected:
            return

        # 카메라 포즈 읽기 (158~169)
        cam_pose = self.robot.read_camera_pose()
        if cam_pose:
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
            self.tabMotionTest.update_current_toolframe(toolframe)
            self.tabTaskEdit.update_current_toolframe(toolframe)
            self.tabCalibration.update_current_toolframe(toolframe)

        # 커맨드/응답 레지스터 읽기
        cmd = self.robot.read_command()
        resp = self.robot.read_response()

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
            self.tabLaserCalibration.update_frame(frame)

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

    # ==================== 테스트 ====================

    def _on_start_test(self):
        """테스트 시작"""
        repeat_count = self.spinRepeatCount.value()
        interval = self.spinInterval.value()
        self._log(f"테스트 시작: {repeat_count}회, {interval}ms 간격")
        # TODO: 테스트 실행

    def _on_stop_test(self):
        """테스트 중지"""
        self._log("테스트 중지")

    def _on_export_csv(self):
        """CSV 내보내기"""
        filename, _ = QFileDialog.getSaveFileName(
            self, "CSV 내보내기", "", "CSV 파일 (*.csv)")
        if filename:
            self._log(f"CSV 내보내기: {filename}")
            # TODO: 통계 데이터 CSV 저장

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

    def _on_tab_changed(self, index: int):
        """탭 변경 시 호출"""
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
            print(f"[DEBUG] Eye in Hand 탭 선택됨 (index={index})")
            if self.robot and self.robot.is_connected:
                print(f"[DEBUG] 로봇 연결 상태: {self.robot.is_connected}")
                try:
                    success, msg = self.robot.send_set_toolframe(4, wait=True)
                    print(f"[DEBUG] send_set_toolframe(4) 결과: success={success}, msg={msg}")
                    if success:
                        self._log("Eye in Hand 탭 선택: Tool Frame 4로 설정 완료")
                        self.tabEyeInHand.update_current_toolframe(4)
                        self._update_statusbar()
                    else:
                        self._log(f"Tool Frame 설정 실패: {msg}")
                except Exception as e:
                    print(f"[DEBUG] 예외 발생: {e}")
                    self._log(f"Tool Frame 설정 오류: {e}")
            else:
                print(f"[DEBUG] 로봇 미연결 - robot={self.robot}, is_connected={self.robot.is_connected if self.robot else 'N/A'}")
        # 레이저 캘리브레이션 탭 (인덱스 6) → ArduCam 강제 전환
        elif index == 6:
            self._on_camera_type_changed(CAMERA_ARDUCAM)
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
