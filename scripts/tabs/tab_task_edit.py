#!/usr/bin/env python3
"""
Task 편집 탭
태스크 시퀀스 관리, 파라미터 편집, 로봇 상태 표시
"""

import os
import socket
import netifaces
from PyQt5 import uic
from PyQt5.QtWidgets import (
    QWidget, QTreeWidgetItem, QListWidgetItem,
    QDoubleSpinBox, QSpinBox, QCheckBox, QLineEdit,
    QComboBox, QLabel, QPushButton, QMessageBox, QInputDialog
)
from PyQt5.QtCore import Qt, pyqtSignal


# UI 파일 경로
UI_DIR = os.path.join(os.path.dirname(__file__), '..', '..', 'ui')
TAB_TASK_EDIT_UI = os.path.join(UI_DIR, 'tab_task_edit.ui')


class TabTaskEdit(QWidget):
    """Task 편집 탭 클래스"""

    # 시그널 정의
    log_message = pyqtSignal(str)
    connect_requested = pyqtSignal(str, int)  # ip, port
    disconnect_requested = pyqtSignal()

    # 현재 위치 읽기 시그널
    read_current_position_requested = pyqtSignal()

    # 현재 Task 실행 시그널
    execute_current_task_requested = pyqtSignal(dict)  # task 정보

    # 조그 이동 시그널 (베이스 좌표계)
    jog_move_requested = pyqtSignal(str, float)  # axis, distance
    jog_rotate_requested = pyqtSignal(str, float)  # axis, angle

    # 태스크 시퀀스 변경 시그널
    task_sequence_changed = pyqtSignal(list)

    def __init__(self, job_types: dict, pose_manager=None, parent=None):
        super().__init__(parent)

        # UI 로드
        uic.loadUi(TAB_TASK_EDIT_UI, self)

        # Job 타입 정의
        self.JOB_TYPES = job_types

        # 포즈 매니저 참조
        self.pose_manager = pose_manager

        # 태스크 시퀀스 데이터
        self.task_sequence = []

        # 파라미터 위젯 저장
        self.param_widgets = {}
        self.param_labels = {}

        # 시그널 연결
        self._connect_signals()

        # 초기화
        self._init_ui()

    def _connect_signals(self):
        """내부 시그널-슬롯 연결"""
        # 태스크 관리
        self.btnAddTask.clicked.connect(self._on_add_task)
        self.btnDeleteTask.clicked.connect(self._on_delete_task)
        self.btnMoveUp.clicked.connect(self._on_move_up)
        self.btnMoveDown.clicked.connect(self._on_move_down)
        self.listTaskSequence.currentRowChanged.connect(self._on_task_selected)
        self.treeAvailableTasks.itemDoubleClicked.connect(self._on_add_task)

        # 파라미터
        self.btnApplyParams.clicked.connect(self._on_apply_params)
        self.btnTeachPosition.clicked.connect(self._on_teach_position)

        # 로봇 연결
        self.btnConnect.clicked.connect(self._on_connect)

        # 조그 이동 (베이스 좌표계)
        self.btnJogXMinus.clicked.connect(lambda: self._on_jog_move('x', -1))
        self.btnJogXPlus.clicked.connect(lambda: self._on_jog_move('x', 1))
        self.btnJogYMinus.clicked.connect(lambda: self._on_jog_move('y', -1))
        self.btnJogYPlus.clicked.connect(lambda: self._on_jog_move('y', 1))
        self.btnJogZMinus.clicked.connect(lambda: self._on_jog_move('z', -1))
        self.btnJogZPlus.clicked.connect(lambda: self._on_jog_move('z', 1))
        self.btnJogRxMinus.clicked.connect(lambda: self._on_jog_rotate('rx', -1))
        self.btnJogRxPlus.clicked.connect(lambda: self._on_jog_rotate('rx', 1))
        self.btnJogRyMinus.clicked.connect(lambda: self._on_jog_rotate('ry', -1))
        self.btnJogRyPlus.clicked.connect(lambda: self._on_jog_rotate('ry', 1))
        self.btnJogRzMinus.clicked.connect(lambda: self._on_jog_rotate('rz', -1))
        self.btnJogRzPlus.clicked.connect(lambda: self._on_jog_rotate('rz', 1))

    def _init_ui(self):
        """UI 초기화"""
        self._init_available_tasks()
        self._init_pc_ip_combo()

    def _log(self, message: str):
        """로그 메시지 출력"""
        self.log_message.emit(message)
        print(f"[TaskEdit] {message}")

    # ==================== Available Tasks 초기화 ====================

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

    # ==================== 네트워크 초기화 ====================

    def _init_pc_ip_combo(self):
        """PC 네트워크 인터페이스 목록으로 콤보박스 초기화"""
        self.comboPCIP.clear()
        interfaces = self._get_network_interfaces()

        for iface_name, ip_addr in interfaces:
            self.comboPCIP.addItem(f"{ip_addr} ({iface_name})", ip_addr)

        # 192.168.0.x 대역 자동 선택
        for i in range(self.comboPCIP.count()):
            ip = self.comboPCIP.itemData(i)
            if ip and ip.startswith("192.168.0."):
                self.comboPCIP.setCurrentIndex(i)
                break

    def _get_network_interfaces(self) -> list:
        """모든 네트워크 인터페이스와 IP 주소 목록 반환"""
        result = []
        try:
            for iface in netifaces.interfaces():
                addrs = netifaces.ifaddresses(iface)
                if netifaces.AF_INET in addrs:
                    for addr_info in addrs[netifaces.AF_INET]:
                        ip = addr_info.get('addr')
                        if ip and ip != '127.0.0.1':
                            result.append((iface, ip))
        except Exception as e:
            self._log(f"네트워크 인터페이스 조회 실패: {e}")

        # 결과가 없으면 기본 방식으로 시도
        if not result:
            try:
                s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                s.connect(("8.8.8.8", 80))
                ip = s.getsockname()[0]
                s.close()
                result.append(("default", ip))
            except Exception:
                pass

        return result

    # ==================== 태스크 관리 ====================

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
        self.task_sequence_changed.emit(self.task_sequence)

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
        self.task_sequence_changed.emit(self.task_sequence)

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
        self.task_sequence_changed.emit(self.task_sequence)

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
        self.task_sequence_changed.emit(self.task_sequence)

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

    def _update_task_ids(self):
        """태스크 ID 재정렬"""
        for i, task in enumerate(self.task_sequence):
            task['id'] = i + 1

    def _get_task_display_name(self, task: dict) -> str:
        """파라미터 기반 태스크 표시 이름 생성"""
        base_name = task.get('name', '')
        task_type = task.get('type', '')
        params = task.get('params', {})

        # 그리퍼: action 값에 따라 이름 변경
        if task_type == 'gripper':
            action = params.get('action', '')
            if action:
                return f"{base_name} {action}"

        # TCP Linear: 좌표계, 모드와 거리 표시
        elif task_type.startswith('tcp_linear_'):
            mode = params.get('mode', '상대')
            coord = params.get('coordinate', 'TF1')
            if task_type == 'tcp_linear_xyz':
                x = params.get('x', 0)
                y = params.get('y', 0)
                z = params.get('z', 0)
                return f"{base_name} [{coord}] ({mode}) [{x},{y},{z}]"
            else:
                dist = params.get('distance', 0)
                return f"{base_name} [{coord}] ({mode}) {dist}mm"

        # TCP Rotate: 모드와 각도 표시
        elif task_type.startswith('tcp_rotate_'):
            mode = params.get('mode', '상대')
            if task_type == 'tcp_rotate_rxryrz':
                rx = params.get('rx', 0)
                ry = params.get('ry', 0)
                rz = params.get('rz', 0)
                return f"{base_name} ({mode}) [{rx},{ry},{rz}]"
            else:
                angle = params.get('angle', 0)
                return f"{base_name} ({mode}) {angle}°"

        # 위치 이동: 좌표 표시
        elif task_type == 'move_to_pose':
            x = params.get('x', 0)
            y = params.get('y', 0)
            z = params.get('z', 0)
            return f"{base_name} ({x:.1f},{y:.1f},{z:.1f})"

        return base_name

    def _refresh_task_list(self):
        """태스크 리스트 갱신"""
        current_row = self.listTaskSequence.currentRow()
        self.listTaskSequence.clear()

        for task in self.task_sequence:
            display_name = self._get_task_display_name(task)
            item = QListWidgetItem(f"{task['id']}. {display_name}")
            item.setData(Qt.UserRole, task)
            self.listTaskSequence.addItem(item)

        if current_row >= 0 and current_row < len(self.task_sequence):
            self.listTaskSequence.setCurrentRow(current_row)

    # ==================== 파라미터 편집 ====================

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

        # 현재 모드 확인 (절대/상대)
        current_mode = task['params'].get('mode', '상대')

        # 파라미터 위젯들
        for param_name, param_info in params_def.items():
            param_type = param_info['type']
            default = param_info.get('default')
            current_value = task['params'].get(param_name, default)
            unit = param_info.get('unit', '')

            # 모드에 따라 description 선택
            if current_mode == '절대' and 'description_absolute' in param_info:
                description = param_info.get('description_absolute', param_name)
            else:
                description = param_info.get('description', param_name)

            label_text = description if description else param_name
            if unit:
                label_text += f" ({unit})"

            label = QLabel(label_text)
            self.param_labels[param_name] = (label, param_info)

            # 타입별 위젯 생성
            if param_type == 'float':
                widget = QDoubleSpinBox()
                widget.setRange(-10000, 10000)
                widget.setDecimals(2)
                widget.setValue(current_value if current_value else 0.0)
            elif param_type == 'int':
                widget = QSpinBox()
                min_val = param_info.get('min', -10000)
                max_val = param_info.get('max', 10000)
                step_val = param_info.get('step', 1)
                widget.setRange(min_val, max_val)
                widget.setSingleStep(step_val)
                widget.setValue(current_value if current_value else param_info.get('default', 0))
            elif param_type == 'bool':
                widget = QCheckBox()
                widget.setChecked(current_value if current_value else False)
            elif param_type == 'pose_select':
                # 저장된 포즈 선택 콤보박스
                widget = QComboBox()
                widget.addItem("(선택 안함)", "")
                if self.pose_manager:
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
                # 모드 콤보박스인 경우 변경 시 라벨 업데이트 연결
                if param_name == 'mode':
                    widget.currentTextChanged.connect(self._on_mode_changed)
            else:
                widget = QLineEdit()
                widget.setText(str(current_value) if current_value else '')

            layout.addRow(label, widget)
            self.param_widgets[param_name] = widget

        # "현재 위치 읽기" 버튼 추가 (has_read_position 플래그가 있는 경우)
        if job_info.get('has_read_position', False):
            read_btn = QPushButton("현재 위치 읽기")
            read_btn.clicked.connect(self._on_read_current_position)
            layout.addRow("", read_btn)

    def _on_mode_changed(self, mode: str):
        """모드 변경 시 라벨 텍스트 업데이트"""
        for param_name, (label, param_info) in self.param_labels.items():
            if param_name == 'mode':
                continue

            unit = param_info.get('unit', '')

            # 모드에 따라 description 선택
            if mode == '절대' and 'description_absolute' in param_info:
                description = param_info.get('description_absolute', param_name)
            else:
                description = param_info.get('description', param_name)

            label_text = description if description else param_name
            if unit:
                label_text += f" ({unit})"

            label.setText(label_text)

    def _clear_param_widgets(self):
        """파라미터 위젯 제거"""
        while self.formLayoutParams.count():
            item = self.formLayoutParams.takeAt(0)
            if item.widget():
                item.widget().deleteLater()
        self.param_widgets.clear()
        self.param_labels.clear()

    def _on_apply_params(self):
        """파라미터 적용"""
        row = self.listTaskSequence.currentRow()
        if row < 0:
            return

        task = self.task_sequence[row]

        for param_name, widget in self.param_widgets.items():
            # Task 이름 필드 처리
            if param_name == '_task_name':
                new_name = widget.text().strip()
                if new_name and new_name != task.get('name', ''):
                    task['name'] = new_name
                continue

            # 일반 파라미터 처리
            if isinstance(widget, QDoubleSpinBox):
                task['params'][param_name] = widget.value()
            elif isinstance(widget, QSpinBox):
                task['params'][param_name] = widget.value()
            elif isinstance(widget, QCheckBox):
                task['params'][param_name] = widget.isChecked()
            elif isinstance(widget, QComboBox):
                data = widget.currentData()
                if data is not None:
                    task['params'][param_name] = data
                else:
                    task['params'][param_name] = widget.currentText()
            elif isinstance(widget, QLineEdit):
                task['params'][param_name] = widget.text()

        self._refresh_task_list()
        self.listTaskSequence.setCurrentRow(row)

        self._log(f"파라미터 적용: {self._get_task_display_name(task)}")
        self.task_sequence_changed.emit(self.task_sequence)

    def _on_teach_position(self):
        """현재 선택된 Task 실행"""
        # 현재 선택된 태스크 가져오기
        current_row = self.listTaskSequence.currentRow()
        if current_row < 0 or current_row >= len(self.task_sequence):
            QMessageBox.warning(self, "경고", "실행할 Task를 선택해주세요.")
            return

        task = self.task_sequence[current_row]
        task_name = self._get_task_display_name(task)

        self._log(f"현 Task 실행 요청: {task_name}")
        self.execute_current_task_requested.emit(task)

    def _on_read_current_position(self):
        """로봇의 현재 위치를 읽어서 파라미터에 입력"""
        self._log("현재 위치 읽기 요청")
        self.read_current_position_requested.emit()

    # ==================== 로봇 연결 ====================

    def _on_connect(self):
        """로봇 연결/해제 토글"""
        ip = self.editRobotIP.text()
        port = self.spinModbusPort.value()
        self.connect_requested.emit(ip, port)

    # ==================== 조그 이동 ====================

    def _on_jog_move(self, axis: str, direction: int):
        """베이스 좌표계 조그 이동"""
        # 스텝 크기 가져오기
        if axis == 'x':
            step = self.spinJogStepX.value()
        elif axis == 'y':
            step = self.spinJogStepY.value()
        elif axis == 'z':
            step = self.spinJogStepZ.value()
        else:
            return

        distance = step * direction
        self._log(f"조그 이동: {axis.upper()} {'+' if direction > 0 else ''}{distance}mm")
        self.jog_move_requested.emit(axis, distance)

    def _on_jog_rotate(self, axis: str, direction: int):
        """베이스 좌표계 조그 회전"""
        # 스텝 크기 가져오기
        if axis == 'rx':
            step = self.spinJogStepRx.value()
        elif axis == 'ry':
            step = self.spinJogStepRy.value()
        elif axis == 'rz':
            step = self.spinJogStepRz.value()
        else:
            return

        angle = step * direction
        self._log(f"조그 회전: {axis.upper()} {'+' if direction > 0 else ''}{angle}deg")
        self.jog_rotate_requested.emit(axis, angle)

    # ==================== 외부 인터페이스 ====================

    def set_pose_manager(self, pose_manager):
        """포즈 매니저 설정"""
        self.pose_manager = pose_manager

    def update_connection_status(self, connected: bool):
        """연결 상태 업데이트"""
        if connected:
            self.labelConnectionStatusValue.setText("연결됨")
            self.labelConnectionStatusValue.setStyleSheet("color: green;")
            self.btnConnect.setText("연결 해제")
        else:
            self.labelConnectionStatusValue.setText("연결 안됨")
            self.labelConnectionStatusValue.setStyleSheet("color: red;")
            self.btnConnect.setText("연결")

    def update_tcp_position(self, x, y, z, rx, ry, rz):
        """TCP 위치 업데이트"""
        self.editX.setText(f"{x:.2f}")
        self.editY.setText(f"{y:.2f}")
        self.editZ.setText(f"{z:.2f}")
        self.editRx.setText(f"{rx:.2f}")
        self.editRy.setText(f"{ry:.2f}")
        self.editRz.setText(f"{rz:.2f}")

    def update_joint_position(self, j1, j2, j3, j4, j5, j6):
        """조인트 위치 업데이트"""
        self.editJ1.setText(f"{j1:.2f}")
        self.editJ2.setText(f"{j2:.2f}")
        self.editJ3.setText(f"{j3:.2f}")
        self.editJ4.setText(f"{j4:.2f}")
        self.editJ5.setText(f"{j5:.2f}")
        self.editJ6.setText(f"{j6:.2f}")

    def fill_current_position_to_params(self, x, y, z, rx, ry, rz):
        """현재 로봇 위치를 Task Parameters에 채우기"""
        # param_widgets 딕셔너리의 위젯에 값 설정
        param_map = {
            'x': x,
            'y': y,
            'z': z,
            'rx': rx,
            'ry': ry,
            'rz': rz
        }

        for param_name, value in param_map.items():
            if param_name in self.param_widgets:
                widget = self.param_widgets[param_name]
                if isinstance(widget, (QDoubleSpinBox, QSpinBox)):
                    widget.setValue(value)
                elif isinstance(widget, QLineEdit):
                    widget.setText(f"{value:.2f}")

        self._log(f"현재 위치 입력 완료: X={x:.2f}, Y={y:.2f}, Z={z:.2f}, Rx={rx:.2f}, Ry={ry:.2f}, Rz={rz:.2f}")

    def update_current_toolframe(self, toolframe: int):
        """현재 툴프레임 업데이트 (placeholder)"""
        pass

    def get_task_sequence(self) -> list:
        """태스크 시퀀스 반환"""
        return self.task_sequence

    def set_task_sequence(self, sequence: list):
        """태스크 시퀀스 설정"""
        self.task_sequence = sequence
        self._refresh_task_list()
