#!/usr/bin/env python3
"""
Charging Robot Task Manager - Main Window
"""

import os
import sys
import socket
from datetime import datetime
from PyQt5 import uic
from PyQt5.QtWidgets import (
    QMainWindow, QMessageBox, QFileDialog, QTreeWidgetItem,
    QListWidgetItem, QDoubleSpinBox, QSpinBox, QCheckBox,
    QLineEdit, QComboBox, QLabel, QFormLayout, QWidget,
    QTableWidgetItem
)
from PyQt5.QtCore import QTimer, Qt
from PyQt5.QtGui import QPixmap, QImage

from Robot import ModbusClient, RobotController, PoseManager, PoseType

# UI 파일 경로
UI_FILE = os.path.join(os.path.dirname(__file__), '..', 'ui', 'main_window.ui')


class MainWindow(QMainWindow):
    """메인 윈도우 클래스"""

    # Job 타입 정의 (단순화)
    JOB_TYPES = {
        # Motion
        'move_to_pose': {
            'name': '위치 이동',
            'category': 'Motion',
            'params': {
                'pose_name': {'type': 'pose_select', 'default': '', 'description': '목표 위치'},
            }
        },
        'approach_and_move': {
            'name': '어프로치 후 이동',
            'category': 'Motion',
            'params': {
                'pose_name': {'type': 'pose_select', 'default': '', 'description': '목표 위치'},
                'approach_distance': {'type': 'float', 'default': 0.2, 'unit': 'm', 'description': '어프로치 거리'},
            }
        },

        # Vision
        'detect_object': {
            'name': '객체 감지',
            'category': 'Vision',
            'params': {
                'target': {'type': 'str', 'default': 'gun', 'options': ['gun', 'port', 'car_port']},
                'timeout': {'type': 'float', 'default': 10.0, 'unit': 'sec'},
            }
        },

        # Gripper
        'gripper': {
            'name': '그리퍼',
            'category': 'Gripper',
            'params': {
                'action': {'type': 'str', 'default': 'close', 'options': ['open', 'close']},
                'delay': {'type': 'float', 'default': 0.5, 'unit': 'sec'},
            }
        },

        # Modbus
        'write_modbus': {
            'name': 'Modbus 쓰기',
            'category': 'Modbus',
            'params': {
                'register': {'type': 'int', 'default': 351},
                'value': {'type': 'int', 'default': 0},
            }
        },
        'send_response': {
            'name': '응답 전송',
            'category': 'Modbus',
            'params': {
                'response': {'type': 'str', 'default': 'pose_back', 'options': ['pose_back', 'pose_main']},
            }
        },

        # Control
        'wait': {
            'name': '대기',
            'category': 'Control',
            'params': {
                'duration': {'type': 'float', 'default': 1.0, 'unit': 'sec'},
            }
        },
    }

    def __init__(self):
        super().__init__()

        # UI 로드
        uic.loadUi(UI_FILE, self)

        # 로봇 Modbus 클라이언트
        self.robot: ModbusClient = None

        # 로봇 컨트롤러 및 포즈 매니저
        self.pose_manager = PoseManager()
        self.robot_controller: RobotController = None

        # 태스크 시퀀스 데이터
        self.task_sequence = []
        self.current_task_index = -1

        # 파라미터 위젯 저장
        self.param_widgets = {}

        # 초기화
        self._init_available_tasks()
        self._init_saved_poses_list()
        self._connect_signals()
        self._init_status()

        # 타이머 설정
        self._setup_timers()

    def _init_available_tasks(self):
        """Available Tasks 트리 초기화"""
        self.treeAvailableTasks.clear()

        # 카테고리별로 그룹화
        categories = {}
        for job_type, info in self.JOB_TYPES.items():
            category = info['category']
            if category not in categories:
                categories[category] = []
            categories[category].append((job_type, info['name']))

        # 트리에 추가
        for category, jobs in categories.items():
            category_item = QTreeWidgetItem([category])
            category_item.setFlags(category_item.flags() & ~Qt.ItemIsSelectable)

            for job_type, job_name in jobs:
                job_item = QTreeWidgetItem([job_name])
                job_item.setData(0, Qt.UserRole, job_type)
                category_item.addChild(job_item)

            self.treeAvailableTasks.addTopLevelItem(category_item)

        # 모든 카테고리 펼치기
        self.treeAvailableTasks.expandAll()

    def _init_saved_poses_list(self):
        """저장된 포즈 리스트 초기화"""
        self._refresh_saved_poses_list()

    def _refresh_saved_poses_list(self):
        """저장된 포즈 리스트 갱신"""
        if not hasattr(self, 'listSavedPoses'):
            return

        self.listSavedPoses.clear()
        for name in self.pose_manager.get_all_names():
            saved_pose = self.pose_manager.get_pose(name)
            if saved_pose:
                pose = saved_pose.pose
                item_text = f"{name} ({saved_pose.pose_type})"
                item = QListWidgetItem(item_text)
                item.setData(Qt.UserRole, name)
                item.setToolTip(
                    f"X: {pose.x:.2f}, Y: {pose.y:.2f}, Z: {pose.z:.2f}\n"
                    f"Rx: {pose.rx:.2f}, Ry: {pose.ry:.2f}, Rz: {pose.rz:.2f}\n"
                    f"{saved_pose.description}"
                )
                self.listSavedPoses.addItem(item)

    def _connect_signals(self):
        """시그널-슬롯 연결"""
        # Task 편집 탭
        self.btnAddTask.clicked.connect(self._on_add_task)
        self.btnDeleteTask.clicked.connect(self._on_delete_task)
        self.btnMoveUp.clicked.connect(self._on_move_up)
        self.btnMoveDown.clicked.connect(self._on_move_down)
        self.listTaskSequence.currentRowChanged.connect(self._on_task_selected)
        self.treeAvailableTasks.itemDoubleClicked.connect(self._on_add_task)
        self.btnApplyParams.clicked.connect(self._on_apply_params)
        self.btnTeachPosition.clicked.connect(self._on_teach_position)

        # 로봇 연결
        self.btnConnect.clicked.connect(self._on_connect)
        self.btnGoHome.clicked.connect(self._on_go_home)
        self.btnSetHome.clicked.connect(self._on_set_home)

        # 비전 탭
        self.btnStartCamera.clicked.connect(self._on_start_camera)
        self.btnStopCamera.clicked.connect(self._on_stop_camera)
        self.btnSnapshot.clicked.connect(self._on_snapshot)
        self.sliderGamma.valueChanged.connect(self._on_gamma_changed)

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

        # 포즈 저장/이동 (Task 편집 탭)
        if hasattr(self, 'btnSavePose'):
            self.btnSavePose.clicked.connect(self._on_save_current_pose)
        if hasattr(self, 'btnDeletePose'):
            self.btnDeletePose.clicked.connect(self._on_delete_pose)
        if hasattr(self, 'btnMoveToPose'):
            self.btnMoveToPose.clicked.connect(self._on_move_to_saved_pose)
        if hasattr(self, 'btnApproachPose'):
            self.btnApproachPose.clicked.connect(self._on_approach_pose)
        if hasattr(self, 'listSavedPoses'):
            self.listSavedPoses.itemDoubleClicked.connect(self._on_move_to_saved_pose)

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

    def _init_status(self):
        """상태 초기화"""
        self.statusbar.showMessage("준비됨")
        self._log("Charging Robot Task Manager 시작")

        # PC IP 표시
        pc_ip = self._get_pc_ip()
        if hasattr(self, 'labelPCIPValue'):
            self.labelPCIPValue.setText(pc_ip)
        self._log(f"PC IP: {pc_ip}")

    def _get_pc_ip(self) -> str:
        """PC의 IP 주소 가져오기"""
        try:
            # 외부 연결용 소켓으로 로컬 IP 확인
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            s.connect(("8.8.8.8", 80))
            ip = s.getsockname()[0]
            s.close()
            return ip
        except Exception:
            try:
                # 대체: hostname으로 IP 가져오기
                return socket.gethostbyname(socket.gethostname())
            except Exception:
                return "Unknown"

    def _setup_timers(self):
        """타이머 설정"""
        # 상태 업데이트 타이머 (100ms)
        self.status_timer = QTimer()
        self.status_timer.timeout.connect(self._update_robot_status)
        # 연결 후 시작됨

    # ==================== Task 편집 ====================

    def _on_add_task(self):
        """태스크 추가"""
        current_item = self.treeAvailableTasks.currentItem()
        if not current_item or not current_item.parent():
            return

        job_type = current_item.data(0, Qt.UserRole)
        if job_type not in self.JOB_TYPES:
            return

        job_info = self.JOB_TYPES[job_type]

        # 기본 파라미터로 태스크 생성
        task = {
            'id': len(self.task_sequence) + 1,
            'type': job_type,
            'name': job_info['name'],
            'params': {k: v['default'] for k, v in job_info.get('params', {}).items()}
        }

        self.task_sequence.append(task)
        self._refresh_task_list()
        self._log(f"태스크 추가: {task['name']}")

    def _on_delete_task(self):
        """태스크 삭제"""
        row = self.listTaskSequence.currentRow()
        if row < 0:
            return

        task = self.task_sequence[row]
        del self.task_sequence[row]
        self._update_task_ids()
        self._refresh_task_list()
        self._log(f"태스크 삭제: {task['name']}")

    def _on_move_up(self):
        """태스크 위로 이동"""
        row = self.listTaskSequence.currentRow()
        if row <= 0:
            return

        self.task_sequence[row], self.task_sequence[row-1] = \
            self.task_sequence[row-1], self.task_sequence[row]
        self._update_task_ids()
        self._refresh_task_list()
        self.listTaskSequence.setCurrentRow(row - 1)

    def _on_move_down(self):
        """태스크 아래로 이동"""
        row = self.listTaskSequence.currentRow()
        if row < 0 or row >= len(self.task_sequence) - 1:
            return

        self.task_sequence[row], self.task_sequence[row+1] = \
            self.task_sequence[row+1], self.task_sequence[row]
        self._update_task_ids()
        self._refresh_task_list()
        self.listTaskSequence.setCurrentRow(row + 1)

    def _on_task_selected(self, row):
        """태스크 선택 시"""
        if row < 0 or row >= len(self.task_sequence):
            self.labelSelectedTask.setText("선택된 Task 없음")
            self.labelTaskType.setText("")
            self._clear_param_widgets()
            return

        task = self.task_sequence[row]
        self.labelSelectedTask.setText(f"{task['id']}. {task['name']}")
        self.labelTaskType.setText(f"Type: {task['type']}")

        # 파라미터 위젯 생성
        self._create_param_widgets(task)

    def _create_param_widgets(self, task):
        """파라미터 편집 위젯 생성"""
        self._clear_param_widgets()

        job_type = task['type']
        if job_type not in self.JOB_TYPES:
            return

        job_info = self.JOB_TYPES[job_type]
        params_def = job_info.get('params', {})

        layout = self.formLayoutParams

        # Task 이름 편집 필드 (맨 위에 추가)
        name_label = QLabel("이름")
        name_label.setStyleSheet("font-weight: bold;")
        name_edit = QLineEdit()
        name_edit.setText(task.get('name', ''))
        name_edit.setPlaceholderText("Task 이름을 입력하세요")
        layout.addRow(name_label, name_edit)
        self.param_widgets['_task_name'] = name_edit

        # 파라미터 위젯들
        for param_name, param_info in params_def.items():
            param_type = param_info['type']
            default = param_info.get('default')
            current_value = task['params'].get(param_name, default)
            unit = param_info.get('unit', '')
            description = param_info.get('description', param_name)

            label_text = description if description else param_name
            if unit:
                label_text += f" ({unit})"

            label = QLabel(label_text)

            # 타입별 위젯 생성
            if param_type == 'float':
                widget = QDoubleSpinBox()
                widget.setRange(-10000, 10000)
                widget.setDecimals(2)
                widget.setValue(current_value if current_value else 0.0)
            elif param_type == 'int':
                widget = QSpinBox()
                widget.setRange(-10000, 10000)
                widget.setValue(current_value if current_value else 0)
            elif param_type == 'bool':
                widget = QCheckBox()
                widget.setChecked(current_value if current_value else False)
            elif param_type == 'pose_select':
                # 저장된 포즈 선택 콤보박스
                widget = QComboBox()
                widget.addItem("(선택 안함)", "")
                for pose_name in self.pose_manager.get_all_names():
                    saved_pose = self.pose_manager.get_pose(pose_name)
                    if saved_pose:
                        widget.addItem(f"{pose_name} ({saved_pose.pose_type})", pose_name)
                # 현재 값 선택
                if current_value:
                    idx = widget.findData(current_value)
                    if idx >= 0:
                        widget.setCurrentIndex(idx)
            elif param_type == 'str' and 'options' in param_info:
                widget = QComboBox()
                widget.addItems(param_info['options'])
                if current_value in param_info['options']:
                    widget.setCurrentText(current_value)
            else:
                widget = QLineEdit()
                widget.setText(str(current_value) if current_value else '')

            layout.addRow(label, widget)
            self.param_widgets[param_name] = widget

    def _clear_param_widgets(self):
        """파라미터 위젯 제거"""
        while self.formLayoutParams.count():
            item = self.formLayoutParams.takeAt(0)
            if item.widget():
                item.widget().deleteLater()
        self.param_widgets.clear()

    def _on_apply_params(self):
        """파라미터 적용"""
        row = self.listTaskSequence.currentRow()
        if row < 0:
            return

        task = self.task_sequence[row]
        name_changed = False

        for param_name, widget in self.param_widgets.items():
            # Task 이름 필드 처리
            if param_name == '_task_name':
                new_name = widget.text().strip()
                if new_name and new_name != task.get('name', ''):
                    task['name'] = new_name
                    name_changed = True
                continue

            # 일반 파라미터 처리
            if isinstance(widget, QDoubleSpinBox):
                task['params'][param_name] = widget.value()
            elif isinstance(widget, QSpinBox):
                task['params'][param_name] = widget.value()
            elif isinstance(widget, QCheckBox):
                task['params'][param_name] = widget.isChecked()
            elif isinstance(widget, QComboBox):
                # pose_select 등 data가 있는 콤보박스는 data 사용
                data = widget.currentData()
                if data is not None:
                    task['params'][param_name] = data
                else:
                    task['params'][param_name] = widget.currentText()
            elif isinstance(widget, QLineEdit):
                task['params'][param_name] = widget.text()

        # 이름이 변경되면 리스트 갱신
        if name_changed:
            self._refresh_task_list()
            self.listTaskSequence.setCurrentRow(row)

        self._log(f"파라미터 적용: {task['name']}")

    def _on_teach_position(self):
        """현재 위치 입력"""
        # TODO: 로봇에서 현재 위치 읽어서 파라미터에 입력
        self._log("현재 위치 입력 (미구현)")

    def _update_task_ids(self):
        """태스크 ID 재정렬"""
        for i, task in enumerate(self.task_sequence):
            task['id'] = i + 1

    def _refresh_task_list(self):
        """태스크 리스트 갱신"""
        current_row = self.listTaskSequence.currentRow()
        self.listTaskSequence.clear()

        for task in self.task_sequence:
            item = QListWidgetItem(f"{task['id']}. {task['name']}")
            item.setData(Qt.UserRole, task)
            self.listTaskSequence.addItem(item)

        if current_row >= 0 and current_row < len(self.task_sequence):
            self.listTaskSequence.setCurrentRow(current_row)

    # ==================== 로봇 연결 ====================

    def _on_connect(self):
        """로봇 연결 토글"""
        # 이미 연결된 경우 연결 해제
        if self.robot and self.robot.is_connected:
            self._on_disconnect()
            return

        ip = self.editRobotIP.text()
        port = self.spinModbusPort.value()
        self._log(f"연결 시도: {ip}:{port}")

        # 새 연결
        self.robot = ModbusClient(ip=ip, port=port, timeout=1.0)
        success, message = self.robot.connect()

        if success:
            self.labelConnectionStatusValue.setText("연결됨")
            self.labelConnectionStatusValue.setStyleSheet("color: green;")
            self.statusbar.showMessage(message)
            self._log(message)

            # RobotController 초기화
            self.robot_controller = RobotController(self.robot, self.pose_manager)
            self.robot_controller.set_on_error(lambda msg: self._log(f"[ERROR] {msg}"))

            # 상태 업데이트 타이머 시작
            self.status_timer.start(100)

            # 연결 버튼 텍스트 변경
            self.btnConnect.setText("연결 해제")
        else:
            self.labelConnectionStatusValue.setText("연결 실패")
            self.labelConnectionStatusValue.setStyleSheet("color: red;")
            self.statusbar.showMessage(message)
            self._log(message)
            QMessageBox.warning(self, "연결 실패", message)

    def _on_disconnect(self):
        """로봇 연결 해제"""
        if self.robot:
            success, message = self.robot.disconnect()
            self._log(message)

        # RobotController 해제
        self.robot_controller = None

        # 타이머 정지
        self.status_timer.stop()

        self.labelConnectionStatusValue.setText("연결 안됨")
        self.labelConnectionStatusValue.setStyleSheet("color: red;")
        self.statusbar.showMessage("연결 해제됨")
        self.btnConnect.setText("연결")

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
            self.editX.setText(f"{x:.2f}")
            self.editY.setText(f"{y:.2f}")
            self.editZ.setText(f"{z:.2f}")
            self.editRx.setText(f"{rx:.2f}")
            self.editRy.setText(f"{ry:.2f}")
            self.editRz.setText(f"{rz:.2f}")

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
        self._log("카메라 시작")
        # TODO: RealSense 카메라 시작

    def _on_stop_camera(self):
        """카메라 정지"""
        self._log("카메라 정지")
        # TODO: 카메라 정지

    def _on_snapshot(self):
        """스냅샷 저장"""
        filename = f"snapshot_{datetime.now().strftime('%Y%m%d_%H%M%S')}.png"
        self._log(f"스냅샷 저장: {filename}")
        # TODO: 이미지 저장

    def _on_gamma_changed(self, value):
        """감마 값 변경"""
        gamma = value / 100.0
        self.labelGammaValue.setText(f"{gamma:.1f}")

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

            # Task 편집 탭의 연결 상태도 업데이트
            self.labelConnectionStatusValue.setText("연결됨")
            self.labelConnectionStatusValue.setStyleSheet("color: green;")
            self.editRobotIP.setText(ip)
            self.spinModbusPort.setValue(port)
            self.btnConnect.setText("연결 해제")

            # 상태 업데이트 타이머 시작
            self.status_timer.start(100)
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

    # ==================== 포즈 저장/이동 ====================

    def _on_save_current_pose(self):
        """현재 위치 저장"""
        if not self.robot or not self.robot.is_connected:
            QMessageBox.warning(self, "경고", "로봇에 연결되지 않았습니다.")
            return

        # 현재 카메라 포즈 읽기
        cam_pose = self.robot.read_camera_pose()
        if cam_pose is None:
            QMessageBox.warning(self, "경고", "현재 위치를 읽을 수 없습니다.")
            return

        # 이름 입력 다이얼로그
        from PyQt5.QtWidgets import QInputDialog
        name, ok = QInputDialog.getText(self, "포즈 저장", "포즈 이름:")
        if not ok or not name.strip():
            return

        name = name.strip()

        # 타입 선택
        pose_types = ["ar_tag", "home", "charging_gun", "charging_port", "approach", "custom"]
        pose_type, ok = QInputDialog.getItem(
            self, "포즈 타입", "타입 선택:", pose_types, 0, False
        )
        if not ok:
            return

        # 저장
        success = self.pose_manager.save_pose_from_tuple(
            name, cam_pose, pose_type,
            description=f"X:{cam_pose[0]:.2f}, Y:{cam_pose[1]:.2f}, Z:{cam_pose[2]:.2f}"
        )

        if success:
            self._log(f"포즈 저장: {name} ({pose_type})")
            self._refresh_saved_poses_list()
        else:
            QMessageBox.warning(self, "오류", "포즈 저장에 실패했습니다.")

    def _on_delete_pose(self):
        """저장된 포즈 삭제"""
        if not hasattr(self, 'listSavedPoses'):
            return

        current_item = self.listSavedPoses.currentItem()
        if not current_item:
            QMessageBox.warning(self, "경고", "삭제할 포즈를 선택하세요.")
            return

        name = current_item.data(Qt.UserRole)

        reply = QMessageBox.question(
            self, "삭제 확인",
            f"'{name}' 포즈를 삭제하시겠습니까?",
            QMessageBox.Yes | QMessageBox.No,
            QMessageBox.No
        )

        if reply == QMessageBox.Yes:
            if self.pose_manager.delete_pose(name):
                self._log(f"포즈 삭제: {name}")
                self._refresh_saved_poses_list()
            else:
                QMessageBox.warning(self, "오류", "포즈 삭제에 실패했습니다.")

    def _on_move_to_saved_pose(self):
        """저장된 위치로 이동"""
        if not self.robot_controller:
            QMessageBox.warning(self, "경고", "로봇에 연결되지 않았습니다.")
            return

        if not hasattr(self, 'listSavedPoses'):
            return

        current_item = self.listSavedPoses.currentItem()
        if not current_item:
            QMessageBox.warning(self, "경고", "이동할 포즈를 선택하세요.")
            return

        name = current_item.data(Qt.UserRole)

        reply = QMessageBox.question(
            self, "이동 확인",
            f"'{name}' 위치로 이동하시겠습니까?",
            QMessageBox.Yes | QMessageBox.No,
            QMessageBox.Yes
        )

        if reply == QMessageBox.Yes:
            result = self.robot_controller.move_to_saved_pose(name)
            if result.success:
                self._log(f"이동 명령: {result.message}")
            else:
                self._log(f"이동 실패: {result.message}")
                QMessageBox.warning(self, "오류", result.message)

    def _on_approach_pose(self):
        """저장된 위치의 어프로치 위치로 이동"""
        if not self.robot_controller:
            QMessageBox.warning(self, "경고", "로봇에 연결되지 않았습니다.")
            return

        if not hasattr(self, 'listSavedPoses'):
            return

        current_item = self.listSavedPoses.currentItem()
        if not current_item:
            QMessageBox.warning(self, "경고", "이동할 포즈를 선택하세요.")
            return

        name = current_item.data(Qt.UserRole)

        # 어프로치 거리 입력
        from PyQt5.QtWidgets import QInputDialog
        distance, ok = QInputDialog.getDouble(
            self, "어프로치 거리", "거리 (meter):",
            0.2, 0.01, 1.0, 2
        )
        if not ok:
            return

        result = self.robot_controller.approach_pose(name, approach_distance=distance)
        if result.success:
            self._log(f"어프로치 이동: {result.message}")
        else:
            self._log(f"어프로치 실패: {result.message}")
            QMessageBox.warning(self, "오류", result.message)

    # ==================== 유틸리티 ====================

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
            # 타이머 정지
            if hasattr(self, 'status_timer'):
                self.status_timer.stop()

            # 로봇 연결 해제
            if self.robot and self.robot.is_connected:
                self.robot.disconnect()

            event.accept()
        else:
            event.ignore()
