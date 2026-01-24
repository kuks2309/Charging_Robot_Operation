#!/usr/bin/env python3
"""
Charging Robot Task Manager - Main Window
"""

import os
from datetime import datetime
import numpy as np
from PyQt5 import uic
from PyQt5.QtWidgets import (
    QMainWindow, QMessageBox, QFileDialog, QTableWidgetItem
)
from PyQt5.QtCore import QTimer

from Robot import ModbusClient, RobotController, PoseManager
from services import CameraManager, VisionManager, AlignmentService, DataCollector, PoseService
from job_types import JOB_TYPES
from tabs import TabTaskEdit, TabVision, TabCalibration, TabEyeInHand, TabMotionTest

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

        # 카메라 매니저 초기화
        self.camera_manager = CameraManager()
        self.camera_manager.set_log_callback(self._log)
        self.camera_manager.frame_ready.connect(self._on_camera_frame)

        # 탭에 카메라 매니저 전달
        self.tabCalibration.set_camera_manager(self.camera_manager)

        # Vision 매니저 초기화
        self.vision_manager = VisionManager(self.camera_manager)
        self.vision_manager.set_log_callback(self._log)

        # 정렬 서비스 초기화
        self.alignment_service = AlignmentService(self.vision_manager)
        self.alignment_service.set_log_callback(self._log)
        self.alignment_service.status_changed.connect(self._update_align_status)

        # 데이터 수집 서비스 초기화
        self.data_collector = DataCollector(self.vision_manager)
        self.data_collector.set_log_callback(self._log)
        self.data_collector.sample_collected.connect(self._on_sample_collected)
        self.data_collector.collection_completed.connect(self._on_collection_completed)

        # 포즈 서비스 초기화
        self.pose_service = PoseService(self.pose_manager)
        self.pose_service.set_log_callback(self._log)
        self.pose_service.pose_list_changed.connect(self._refresh_saved_poses_list)

        # 초기화
        self._connect_signals()
        self._init_status()

        # 타이머 설정
        self._setup_timers()

    def _refresh_saved_poses_list(self):
        """저장된 포즈 리스트 갱신 (PoseService 시그널 핸들러)"""
        self.tabTaskEdit.refresh_poses()

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
        self.actionGoHome.triggered.connect(self._on_go_home)
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

        # Eye in Hand 탭 (인덱스 3에 삽입)
        self.tabEyeInHand = TabEyeInHand(self)
        self.tabWidget.insertTab(3, self.tabEyeInHand, "Eye in Hand")

        # Motion Test 탭 (인덱스 4에 삽입)
        self.tabMotionTest = TabMotionTest(self)
        self.tabWidget.insertTab(4, self.tabMotionTest, "모션 테스트")

        # 탭 시그널 연결
        self._connect_tab_signals()

        # 첫 번째 탭 선택
        self.tabWidget.setCurrentIndex(0)

    def _connect_tab_signals(self):
        """탭 클래스들의 시그널을 메인윈도우 슬롯에 연결"""
        # Task 편집 탭 시그널
        self.tabTaskEdit.log_message.connect(self._log)
        self.tabTaskEdit.connect_requested.connect(self._on_connect_from_tab)
        self.tabTaskEdit.go_home_requested.connect(self._on_go_home)
        self.tabTaskEdit.set_home_requested.connect(self._on_set_home)
        self.tabTaskEdit.save_pose_requested.connect(self._on_save_pose_from_tab)
        self.tabTaskEdit.delete_pose_requested.connect(self._on_delete_pose_from_tab)
        self.tabTaskEdit.move_to_pose_requested.connect(self._on_move_to_pose_from_tab)
        self.tabTaskEdit.approach_pose_requested.connect(self._on_approach_pose_from_tab)
        self.tabTaskEdit.task_sequence_changed.connect(self._on_task_sequence_changed)

        # 비전 탭 시그널
        self.tabVision.log_message.connect(self._log)
        self.tabVision.camera_start_requested.connect(self._on_start_camera)
        self.tabVision.camera_stop_requested.connect(self._on_stop_camera)
        self.tabVision.gamma_changed.connect(self._on_gamma_changed_from_tab)
        self.tabVision.align_center_requested.connect(self._on_align_center_from_tab)
        self.tabVision.align_pose_requested.connect(self._on_align_pose_from_tab)
        self.tabVision.align_full_requested.connect(self._on_align_full_from_tab)
        self.tabVision.collect_start_requested.connect(self._on_start_collect_from_tab)
        self.tabVision.collect_stop_requested.connect(self._on_stop_collect)
        self.tabVision.collect_save_requested.connect(self._on_save_collect)

        # 캘리브레이션 탭 시그널
        self.tabCalibration.log_message.connect(self._log)
        self.tabCalibration.camera_start_requested.connect(self._on_start_camera)
        self.tabCalibration.camera_stop_requested.connect(self._on_stop_camera)

        # Eye in Hand 탭 시그널
        self.tabEyeInHand.log_message.connect(self._log)
        self.tabEyeInHand.camera_start_requested.connect(self._on_start_camera)
        self.tabEyeInHand.camera_stop_requested.connect(self._on_stop_camera)

        # Motion Test 탭 시그널
        self.tabMotionTest.log_message.connect(self._log)

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

    def _on_save_pose_from_tab(self, name: str, pose_type: str):
        """포즈 저장 요청 (TabTaskEdit 시그널 핸들러)"""
        result = self.pose_service.save_current_pose(name, pose_type)
        if not result.success:
            QMessageBox.warning(self, "오류", result.message)
        else:
            self.tabTaskEdit.refresh_poses()

    def _on_delete_pose_from_tab(self, name: str):
        """포즈 삭제 요청 (TabTaskEdit 시그널 핸들러)"""
        result = self.pose_service.delete_pose(name)
        if not result.success:
            QMessageBox.warning(self, "오류", result.message)
        else:
            self.tabTaskEdit.refresh_poses()

    def _on_move_to_pose_from_tab(self, name: str):
        """포즈로 이동 요청 (TabTaskEdit 시그널 핸들러)"""
        result = self.pose_service.move_to_pose(name)
        if not result.success:
            QMessageBox.warning(self, "오류", result.message)

    def _on_approach_pose_from_tab(self, name: str, distance: float):
        """어프로치 위치로 이동 요청 (TabTaskEdit 시그널 핸들러)"""
        result = self.pose_service.approach_pose(name, distance)
        if not result.success:
            QMessageBox.warning(self, "오류", result.message)

    def _on_gamma_changed_from_tab(self, gamma: float):
        """감마 값 변경 (TabVision 시그널 핸들러)"""
        self.camera_manager.set_gamma(gamma)

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

    def _on_start_collect_from_tab(self, tag_id: int, target_count: int):
        """데이터 수집 시작 요청 (TabVision 시그널 핸들러)"""
        if not self.data_collector.start(tag_id, target_count):
            QMessageBox.warning(self, "경고", "먼저 카메라를 시작하세요.")
            return

        self.tabVision.set_collect_buttons_enabled(True)

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
            self.statusbar.showMessage(message)
            self._log(message)

            # 공통 연결 설정
            self._setup_robot_connection()
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

        # Task 편집 탭의 연결 상태 업데이트
        self.tabTaskEdit.update_connection_status(True)

        # 상태 업데이트 타이머 시작
        self.status_timer.start(100)

        # 상태바에 연결 정보 및 Tool Frame 표시
        self._update_statusbar()

        # 현재 탭이 비전/캘리브레이션 탭이면 Tool Frame 1로 설정
        current_tab = self.tabWidget.currentIndex()
        if current_tab in [1, 2]:
            try:
                success, msg = self.robot.send_set_toolframe(1, wait=True)
                tab_name = "Vision" if current_tab == 1 else "캘리브레이션"
                if success:
                    self._log(f"{tab_name} 탭: Tool Frame 1 (비전)으로 설정 완료")
                    # 캘리브레이션 탭 UI 업데이트
                    if current_tab == 2:
                        self.tabCalibration.update_current_toolframe(1)
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

        # 타이머 정지
        self.status_timer.stop()

        # TabTaskEdit 연결 상태 업데이트
        self.tabTaskEdit.update_connection_status(False)
        self.statusbar.showMessage("연결 해제됨")

    def _on_go_home(self):
        """HOME 이동"""
        self._log("HOME으로 이동")
        # TODO: 로봇 HOME 이동 명령

    def _on_set_home(self):
        """현재 위치를 HOME으로 설정"""
        self._log("현재 위치를 HOME으로 설정")
        # TODO: HOME 위치 저장

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

            # 데이터 수집 중이면 샘플 저장
            if self.data_collector.is_collecting:
                self.data_collector.collect_sample()

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

        # Eye in Hand 탭이 활성화된 경우
        elif current_tab == 3:  # Eye in Hand 탭
            self.tabEyeInHand.set_current_frame(frame)
            self.tabEyeInHand.display_frame(frame)

    def detect_aruco_tag(self, tag_id: int, timeout: float = 10.0, num_samples: int = 10):
        """특정 Aruco 태그 감지 (VisionManager 위임)"""
        return self.vision_manager.detect_tag(tag_id, timeout, num_samples)

    def _update_align_status(self, status: str):
        """정렬 상태 업데이트 (AlignmentService signal 핸들러)"""
        self.tabVision.update_align_status(status)

    # ==================== 데이터 수집 (노이즈 분석용) ====================

    def _on_stop_collect(self):
        """데이터 수집 중지 (DataCollector 위임)"""
        self.data_collector.stop()

        # 통계 출력
        stats_str = self.data_collector.print_statistics()
        for line in stats_str.split('\n'):
            self._log(line)

    def _on_sample_collected(self, current: int, target: int):
        """샘플 수집 시그널 핸들러 (DataCollector signal)"""
        self.tabVision.update_collect_status(current, target)

    def _on_collection_completed(self, count: int):
        """수집 완료 시그널 핸들러 (DataCollector signal)"""
        self.tabVision.set_collect_buttons_enabled(False)
        self.tabVision.set_save_button_enabled(count > 0)

    def _on_save_collect(self):
        """수집된 데이터를 CSV로 저장 (DataCollector 위임)"""
        if self.data_collector.collected_count == 0:
            QMessageBox.warning(self, "경고", "저장할 데이터가 없습니다.")
            return

        # 파일 저장 다이얼로그
        default_name = self.data_collector.get_default_filename()
        filepath, _ = QFileDialog.getSaveFileName(
            self, "데이터 저장", default_name, "CSV Files (*.csv)"
        )

        if not filepath:
            return

        if self.data_collector.save_to_csv(filepath):
            QMessageBox.information(self, "완료", f"데이터가 저장되었습니다.\n{filepath}")
        else:
            QMessageBox.critical(self, "오류", "저장 실패")

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
        # Vision 탭 (인덱스 1) 또는 캘리브레이션 탭 (인덱스 2)이 선택되면 Tool Frame 1로 설정
        if index in [1, 2]:
            if self.robot and self.robot.is_connected:
                try:
                    success, msg = self.robot.send_set_toolframe(1, wait=True)
                    tab_name = "Vision" if index == 1 else "캘리브레이션"
                    if success:
                        self._log(f"{tab_name} 탭 선택: Tool Frame 1 (비전)으로 설정 완료")
                        # 캘리브레이션 탭 UI 업데이트
                        if index == 2:
                            self.tabCalibration.update_current_toolframe(1)
                        # 상태바 업데이트
                        self._update_statusbar()
                    else:
                        self._log(f"Tool Frame 설정 실패: {msg}")
                except Exception as e:
                    self._log(f"Tool Frame 설정 오류: {e}")
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
        self.textExecutionLog.append(f"[{timestamp}] {message}")

    def closeEvent(self, event):
        """종료 이벤트"""
        reply = QMessageBox.question(
            self, '종료 확인',
            '정말 종료하시겠습니까?',
            QMessageBox.Yes | QMessageBox.No,
            QMessageBox.No
        )

        if reply == QMessageBox.Yes:
            # 카메라 정지
            if self.camera_manager.is_running:
                self.camera_manager.stop()

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
