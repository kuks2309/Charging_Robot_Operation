#!/usr/bin/env python3
"""
Charging Robot Task Manager - Main Window
"""

import os
import sys
import socket
import netifaces
from datetime import datetime
import numpy as np
import cv2
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
from Sensor import ArucoCameraPoseEstimator
from services import CameraManager, VisionManager

# UI 파일 경로
UI_FILE = os.path.join(os.path.dirname(__file__), '..', 'ui', 'main_window.ui')


class MainWindow(QMainWindow):
    """메인 윈도우 클래스"""

    # Job 타입 정의 (단순화)
    JOB_TYPES = {
        # Motion
        'go_home': {
            'name': 'Go Home',
            'category': 'Motion',
            'params': {}
        },
        'move_to_pose': {
            'name': '위치 이동',
            'category': 'Motion',
            'params': {
                'x': {'type': 'float', 'default': 0.0, 'unit': 'mm', 'description': 'X 위치'},
                'y': {'type': 'float', 'default': 0.0, 'unit': 'mm', 'description': 'Y 위치'},
                'z': {'type': 'float', 'default': 0.0, 'unit': 'mm', 'description': 'Z 위치'},
                'rx': {'type': 'float', 'default': 0.0, 'unit': 'deg', 'description': 'Rx 회전'},
                'ry': {'type': 'float', 'default': 0.0, 'unit': 'deg', 'description': 'Ry 회전'},
                'rz': {'type': 'float', 'default': 0.0, 'unit': 'deg', 'description': 'Rz 회전'},
            },
            'has_read_position': True
        },
        'tcp_linear_x': {
            'name': 'TCP Linear X',
            'category': 'Motion',
            'params': {
                'mode': {'type': 'str', 'default': '상대', 'options': ['절대', '상대'], 'description': '이동 모드'},
                'distance': {'type': 'float', 'default': 0.0, 'unit': 'mm', 'description': 'X 이동 거리', 'description_absolute': 'X 목표 위치'},
            }
        },
        'tcp_linear_y': {
            'name': 'TCP Linear Y',
            'category': 'Motion',
            'params': {
                'mode': {'type': 'str', 'default': '상대', 'options': ['절대', '상대'], 'description': '이동 모드'},
                'distance': {'type': 'float', 'default': 0.0, 'unit': 'mm', 'description': 'Y 이동 거리', 'description_absolute': 'Y 목표 위치'},
            }
        },
        'tcp_linear_z': {
            'name': 'TCP Linear Z',
            'category': 'Motion',
            'params': {
                'mode': {'type': 'str', 'default': '상대', 'options': ['절대', '상대'], 'description': '이동 모드'},
                'distance': {'type': 'float', 'default': 0.0, 'unit': 'mm', 'description': 'Z 이동 거리', 'description_absolute': 'Z 목표 위치'},
            }
        },
        'tcp_linear_xyz': {
            'name': 'TCP Linear XYZ',
            'category': 'Motion',
            'params': {
                'mode': {'type': 'str', 'default': '상대', 'options': ['절대', '상대'], 'description': '이동 모드'},
                'x': {'type': 'float', 'default': 0.0, 'unit': 'mm', 'description': 'X 이동 거리', 'description_absolute': 'X 목표 위치'},
                'y': {'type': 'float', 'default': 0.0, 'unit': 'mm', 'description': 'Y 이동 거리', 'description_absolute': 'Y 목표 위치'},
                'z': {'type': 'float', 'default': 0.0, 'unit': 'mm', 'description': 'Z 이동 거리', 'description_absolute': 'Z 목표 위치'},
            }
        },
        'tcp_rotate_rx': {
            'name': 'TCP Rotate Rx',
            'category': 'Motion',
            'params': {
                'mode': {'type': 'str', 'default': '상대', 'options': ['절대', '상대'], 'description': '회전 모드'},
                'angle': {'type': 'float', 'default': 0.0, 'unit': 'deg', 'description': 'Rx 회전 각도', 'description_absolute': 'Rx 목표 각도'},
            }
        },
        'tcp_rotate_ry': {
            'name': 'TCP Rotate Ry',
            'category': 'Motion',
            'params': {
                'mode': {'type': 'str', 'default': '상대', 'options': ['절대', '상대'], 'description': '회전 모드'},
                'angle': {'type': 'float', 'default': 0.0, 'unit': 'deg', 'description': 'Ry 회전 각도', 'description_absolute': 'Ry 목표 각도'},
            }
        },
        'tcp_rotate_rz': {
            'name': 'TCP Rotate Rz',
            'category': 'Motion',
            'params': {
                'mode': {'type': 'str', 'default': '상대', 'options': ['절대', '상대'], 'description': '회전 모드'},
                'angle': {'type': 'float', 'default': 0.0, 'unit': 'deg', 'description': 'Rz 회전 각도', 'description_absolute': 'Rz 목표 각도'},
            }
        },
        'tcp_rotate_rxryrz': {
            'name': 'TCP Rotate RxRyRz',
            'category': 'Motion',
            'params': {
                'mode': {'type': 'str', 'default': '상대', 'options': ['절대', '상대'], 'description': '회전 모드'},
                'rx': {'type': 'float', 'default': 0.0, 'unit': 'deg', 'description': 'Rx 회전 각도', 'description_absolute': 'Rx 목표 각도'},
                'ry': {'type': 'float', 'default': 0.0, 'unit': 'deg', 'description': 'Ry 회전 각도', 'description_absolute': 'Ry 목표 각도'},
                'rz': {'type': 'float', 'default': 0.0, 'unit': 'deg', 'description': 'Rz 회전 각도', 'description_absolute': 'Rz 목표 각도'},
            }
        },

        # Vision
        'detect_aruco': {
            'name': 'Aruco Tag 인식',
            'category': 'Vision',
            'params': {
                'tag_id': {'type': 'int', 'default': 0, 'description': 'Tag ID'},
                'timeout': {'type': 'float', 'default': 10.0, 'unit': 'sec'},
            }
        },

        # 정렬 (Alignment)
        'align_aruco_center': {
            'name': 'Aruco Tag 중심 정렬',
            'category': '정렬',
            'params': {
                'tag_id': {'type': 'int', 'default': 0, 'description': 'Tag ID'},
                'offset_x': {'type': 'float', 'default': 0.0, 'unit': 'mm', 'description': 'X 오프셋'},
                'offset_y': {'type': 'float', 'default': 0.0, 'unit': 'mm', 'description': 'Y 오프셋'},
            }
        },
        'align_aruco_pose': {
            'name': 'Aruco Tag 자세 정렬',
            'category': '정렬',
            'params': {
                'tag_id': {'type': 'int', 'default': 0, 'description': 'Tag ID'},
            }
        },
        'align_aruco_full': {
            'name': 'Aruco Tag 전체 정렬',
            'category': '정렬',
            'params': {
                'tag_id': {'type': 'int', 'default': 0, 'description': 'Tag ID'},
                'offset_x': {'type': 'float', 'default': 0.0, 'unit': 'mm', 'description': 'X 오프셋'},
                'offset_y': {'type': 'float', 'default': 0.0, 'unit': 'mm', 'description': 'Y 오프셋'},
            }
        },
        'tcp_offset_move': {
            'name': 'TCP 오프셋 이동',
            'category': '정렬',
            'params': {
                'offset_x': {'type': 'float', 'default': 0.0, 'unit': 'mm', 'description': 'X 오프셋'},
                'offset_y': {'type': 'float', 'default': 0.0, 'unit': 'mm', 'description': 'Y 오프셋'},
                'offset_z': {'type': 'float', 'default': 0.0, 'unit': 'mm', 'description': 'Z 오프셋'},
            }
        },

        # Gripper
        'gripper': {
            'name': '그리퍼',
            'category': 'Gripper',
            'params': {
                'action': {'type': 'str', 'default': 'close', 'options': ['open', 'close', 'home']},
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
                'duration': {'type': 'int', 'default': 500, 'unit': 'msec', 'min': 10, 'max': 10000, 'step': 10},
            }
        },

        # Frame
        'toolframe': {
            'name': '툴프레임 변경',
            'category': 'Frame',
            'params': {
                'frame': {'type': 'int', 'default': 0, 'min': 0, 'max': 3, 'description': '툴프레임 번호 (0-3)'},
            }
        },
        'base': {
            'name': '베이스프레임 초기화',
            'category': 'Frame',
            'params': {}
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

        # 카메라 매니저 초기화
        self.camera_manager = CameraManager()
        self.camera_manager.set_log_callback(self._log)
        self.camera_manager.frame_ready.connect(self._on_camera_frame)

        # Vision 매니저 초기화
        self.vision_manager = VisionManager(self.camera_manager)
        self.vision_manager.set_log_callback(self._log)

        # 데이터 수집 초기화
        self._init_data_collect()

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

        # Aruco 정렬 테스트 버튼
        if hasattr(self, 'btnAlignCenter'):
            self.btnAlignCenter.clicked.connect(self._on_align_center)
        if hasattr(self, 'btnAlignPose'):
            self.btnAlignPose.clicked.connect(self._on_align_pose)
        if hasattr(self, 'btnAlignFull'):
            self.btnAlignFull.clicked.connect(self._on_align_full)

        # 데이터 수집 버튼
        if hasattr(self, 'btnStartCollect'):
            self.btnStartCollect.clicked.connect(self._on_start_collect)
        if hasattr(self, 'btnStopCollect'):
            self.btnStopCollect.clicked.connect(self._on_stop_collect)
        if hasattr(self, 'btnSaveCollect'):
            self.btnSaveCollect.clicked.connect(self._on_save_collect)

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

        # PC IP 콤보박스 초기화
        self._init_pc_ip_combo()

    def _init_pc_ip_combo(self):
        """PC 네트워크 인터페이스 목록으로 콤보박스 초기화"""
        if not hasattr(self, 'comboPCIP'):
            return

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

        selected_ip = self.comboPCIP.currentData() or "Unknown"
        self._log(f"PC IP: {selected_ip}")

        # 설정 탭의 레이블도 업데이트
        if hasattr(self, 'labelPCIPValue'):
            self.labelPCIPValue.setText(selected_ip)

        # 콤보박스 변경 시 설정 탭 레이블 동기화
        self.comboPCIP.currentIndexChanged.connect(self._on_pc_ip_changed)

    def _on_pc_ip_changed(self, index):
        """PC IP 콤보박스 변경 시"""
        ip = self.comboPCIP.currentData()
        if hasattr(self, 'labelPCIPValue') and ip:
            self.labelPCIPValue.setText(ip)
        self._log(f"PC IP 변경: {ip}")

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
            ip = self._get_pc_ip_fallback()
            if ip != "Unknown":
                result.append(("default", ip))

        return result

    def _get_pc_ip_fallback(self) -> str:
        """PC의 IP 주소 가져오기 (fallback)"""
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            s.connect(("8.8.8.8", 80))
            ip = s.getsockname()[0]
            s.close()
            return ip
        except Exception:
            try:
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

        # 라벨 저장용 딕셔너리 (모드 변경 시 라벨 업데이트용)
        self.param_labels = {}

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
            self.param_labels[param_name] = (label, param_info)  # 라벨과 정보 저장

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
            from PyQt5.QtWidgets import QPushButton
            read_btn = QPushButton("현재 위치 읽기")
            read_btn.clicked.connect(self._on_read_current_position)
            layout.addRow("", read_btn)

    def _on_mode_changed(self, mode: str):
        """모드 변경 시 라벨 텍스트 업데이트"""
        if not hasattr(self, 'param_labels'):
            return

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

    def _on_read_current_position(self):
        """로봇의 현재 위치를 읽어서 파라미터에 입력"""
        if not self.robot or not self.robot.is_connected:
            self._log("로봇이 연결되지 않았습니다")
            return

        # 로봇에서 현재 TCP 위치 읽기
        pose = self.robot.read_camera_pose()  # 또는 다른 포즈 읽기 함수
        if pose is None:
            self._log("현재 위치 읽기 실패")
            return

        x, y, z, rx, ry, rz = pose

        # 파라미터 위젯에 값 설정
        if 'x' in self.param_widgets:
            self.param_widgets['x'].setValue(x)
        if 'y' in self.param_widgets:
            self.param_widgets['y'].setValue(y)
        if 'z' in self.param_widgets:
            self.param_widgets['z'].setValue(z)
        if 'rx' in self.param_widgets:
            self.param_widgets['rx'].setValue(rx)
        if 'ry' in self.param_widgets:
            self.param_widgets['ry'].setValue(ry)
        if 'rz' in self.param_widgets:
            self.param_widgets['rz'].setValue(rz)

        self._log(f"현재 위치 읽기 완료: X={x:.2f}, Y={y:.2f}, Z={z:.2f}, Rx={rx:.2f}, Ry={ry:.2f}, Rz={rz:.2f}")

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

        # 파라미터 변경 시 항상 리스트 갱신 (표시 이름 업데이트)
        self._refresh_task_list()
        self.listTaskSequence.setCurrentRow(row)

        self._log(f"파라미터 적용: {self._get_task_display_name(task)}")

    def _on_teach_position(self):
        """현재 위치 입력"""
        # TODO: 로봇에서 현재 위치 읽어서 파라미터에 입력
        self._log("현재 위치 입력 (미구현)")

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

        # TCP Linear: 모드와 거리 표시
        elif task_type.startswith('tcp_linear_'):
            mode = params.get('mode', '상대')
            if task_type == 'tcp_linear_xyz':
                x = params.get('x', 0)
                y = params.get('y', 0)
                z = params.get('z', 0)
                return f"{base_name} ({mode}) [{x},{y},{z}]"
            else:
                dist = params.get('distance', 0)
                return f"{base_name} ({mode}) {dist}mm"

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

    def _init_data_collect(self):
        """데이터 수집 초기화"""
        # 데이터 수집 관련
        self.collecting_data = False
        self.collected_data = []
        self.collect_target_count = 100
        self.collect_tag_id = 0

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
        # Aruco 감지 (체크박스가 활성화된 경우)
        if hasattr(self, 'checkArucoDetect') and self.checkArucoDetect.isChecked():
            frame, _ = self.vision_manager.detect_markers(frame)

            # 데이터 수집 중이면 샘플 저장
            if self.collecting_data and self.vision_manager.last_result:
                self._collect_sample()

        # QLabel에 표시
        self._display_frame(frame)

    def detect_aruco_tag(self, tag_id: int, timeout: float = 10.0, num_samples: int = 10):
        """특정 Aruco 태그 감지 (VisionManager 위임)"""
        return self.vision_manager.detect_tag(tag_id, timeout, num_samples)

    def _display_frame(self, frame):
        """프레임을 QLabel에 표시"""
        if not hasattr(self, 'labelCameraView'):
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
            self.labelCameraView.size(),
            Qt.KeepAspectRatio,
            Qt.SmoothTransformation
        )
        self.labelCameraView.setPixmap(scaled_pixmap)

    def _on_snapshot(self):
        """스냅샷 저장"""
        filepath = self.camera_manager.snapshot()
        if filepath is None:
            QMessageBox.warning(self, "경고", "스냅샷 저장에 실패했습니다.")

    def _on_gamma_changed(self, value):
        """감마 값 변경"""
        gamma = value / 100.0
        self.labelGammaValue.setText(f"{gamma:.1f}")
        self.camera_manager.set_gamma(gamma)

    # ==================== Aruco 정렬 테스트 ====================

    def _get_num_samples(self) -> int:
        """UI에서 샘플 수 가져오기"""
        if hasattr(self, 'spinNumSamples'):
            return self.spinNumSamples.value()
        return 10  # 기본값

    def _on_align_center(self):
        """Aruco Tag 중심 정렬 테스트"""
        tag_id = self.spinTargetTagId.value() if hasattr(self, 'spinTargetTagId') else 0
        num_samples = self._get_num_samples()
        self._log(f"Aruco Tag {tag_id} 중심 정렬 시작 ({num_samples}회 측정)")
        self._update_align_status("중심 정렬 중...")

        # 카메라 체크
        if not self.camera_manager.is_running:
            self._update_align_status("카메라 미연결")
            QMessageBox.warning(self, "경고", "먼저 카메라를 시작하세요.")
            return

        # 태그 감지 (n회 평균)
        marker = self.detect_aruco_tag(tag_id, timeout=10.0, num_samples=num_samples)
        if marker is None:
            self._update_align_status("태그 감지 실패")
            QMessageBox.warning(self, "경고", f"Tag ID {tag_id}를 찾을 수 없습니다.")
            return

        # 중심 오프셋 계산 (카메라 중심에서 태그까지)
        # marker['tvec'] = [x, y, z] in camera frame (meters)
        tvec = marker['tvec']
        offset_x = tvec[0] * 1000  # m to mm
        offset_y = tvec[1] * 1000  # m to mm

        self._log(f"중심 오프셋: X={offset_x:.2f}mm, Y={offset_y:.2f}mm")

        # 로봇 이동 (상대 이동)
        if self.robot and self.robot.is_connected:
            # TCP Linear XYZ 상대 이동으로 보정 (X, Y만 이동)
            success, msg = self.robot.send_tcp_linear(
                axis='xyz',
                distance=(-offset_x, -offset_y, 0),
                absolute=False,
                wait=True
            )
            if success:
                self._update_align_status(f"중심 정렬 완료 (X:{-offset_x:.1f}, Y:{-offset_y:.1f})")
            else:
                self._update_align_status(f"이동 실패: {msg}")
        else:
            self._update_align_status("로봇 미연결")
            QMessageBox.warning(self, "경고", "로봇에 연결되어 있지 않습니다.")

    def _on_align_pose(self):
        """Aruco Tag 자세 정렬 테스트"""
        tag_id = self.spinTargetTagId.value() if hasattr(self, 'spinTargetTagId') else 0
        num_samples = self._get_num_samples()
        self._log(f"Aruco Tag {tag_id} 자세 정렬 시작 ({num_samples}회 측정)")
        self._update_align_status("자세 정렬 중...")

        # 카메라 체크
        if not self.camera_manager.is_running:
            self._update_align_status("카메라 미연결")
            QMessageBox.warning(self, "경고", "먼저 카메라를 시작하세요.")
            return

        # 태그 감지 (n회 평균)
        marker = self.detect_aruco_tag(tag_id, timeout=10.0, num_samples=num_samples)
        if marker is None:
            self._update_align_status("태그 감지 실패")
            QMessageBox.warning(self, "경고", f"Tag ID {tag_id}를 찾을 수 없습니다.")
            return

        # 회전 오프셋 계산
        # marker['camera_rotation'] = rotation matrix
        euler_angles = self.vision_manager.aruco_detector._rotation_matrix_to_euler(marker['camera_rotation'])
        rx, ry, rz = euler_angles

        self._log(f"자세 오프셋: Rx={rx:.2f}°, Ry={ry:.2f}°, Rz={rz:.2f}°")

        # 로봇 회전 (상대 이동)
        if self.robot and self.robot.is_connected:
            # TCP Rotate로 보정
            success, msg = self.robot.send_tcp_rotate(
                axis='rxryrz',
                angle=(-rx, -ry, -rz),
                absolute=False,
                wait=True
            )
            if success:
                self._update_align_status(f"자세 정렬 완료 (Rx:{-rx:.1f}, Ry:{-ry:.1f}, Rz:{-rz:.1f})")
            else:
                self._update_align_status(f"회전 실패: {msg}")
        else:
            self._update_align_status("로봇 미연결")
            QMessageBox.warning(self, "경고", "로봇에 연결되어 있지 않습니다.")

    def _on_align_full(self):
        """Aruco Tag 전체 정렬 테스트 (중심 + 자세)"""
        tag_id = self.spinTargetTagId.value() if hasattr(self, 'spinTargetTagId') else 0
        num_samples = self._get_num_samples()
        self._log(f"Aruco Tag {tag_id} 전체 정렬 시작 ({num_samples}회 측정)")
        self._update_align_status("전체 정렬 중...")

        # 카메라 체크
        if not self.camera_manager.is_running:
            self._update_align_status("카메라 미연결")
            QMessageBox.warning(self, "경고", "먼저 카메라를 시작하세요.")
            return

        # 로봇 체크
        if not self.robot or not self.robot.is_connected:
            self._update_align_status("로봇 미연결")
            QMessageBox.warning(self, "경고", "로봇에 연결되어 있지 않습니다.")
            return

        # 태그 감지 (n회 평균)
        marker = self.detect_aruco_tag(tag_id, timeout=10.0, num_samples=num_samples)
        if marker is None:
            self._update_align_status("태그 감지 실패")
            QMessageBox.warning(self, "경고", f"Tag ID {tag_id}를 찾을 수 없습니다.")
            return

        # 중심 오프셋 계산
        tvec = marker['tvec']
        offset_x = tvec[0] * 1000  # m to mm
        offset_y = tvec[1] * 1000  # m to mm

        # 회전 오프셋 계산
        euler_angles = self.vision_manager.aruco_detector._rotation_matrix_to_euler(marker['camera_rotation'])
        rx, ry, rz = euler_angles

        self._log(f"중심: X={offset_x:.2f}mm, Y={offset_y:.2f}mm")
        self._log(f"자세: Rx={rx:.2f}°, Ry={ry:.2f}°, Rz={rz:.2f}°")

        # 1. 먼저 자세 정렬
        self._update_align_status("자세 정렬 중...")
        success, msg = self.robot.send_tcp_rotate(
            axis='rxryrz',
            angle=(-rx, -ry, -rz),
            absolute=False,
            wait=True
        )
        if not success:
            self._update_align_status(f"자세 정렬 실패: {msg}")
            return
        self._log("자세 정렬 완료")

        # 2. 중심 정렬
        self._update_align_status("중심 정렬 중...")
        success, msg = self.robot.send_tcp_linear(
            axis='xyz',
            distance=(-offset_x, -offset_y, 0),
            absolute=False,
            wait=True
        )
        if not success:
            self._update_align_status(f"중심 정렬 실패: {msg}")
            return

        self._update_align_status("전체 정렬 완료")
        self._log("전체 정렬 완료")

    def _update_align_status(self, status: str):
        """정렬 상태 업데이트"""
        if hasattr(self, 'labelAlignStatus'):
            self.labelAlignStatus.setText(f"상태: {status}")
        self._log(status)

    # ==================== 데이터 수집 (노이즈 분석용) ====================

    def _on_start_collect(self):
        """데이터 수집 시작"""
        if not self.camera_manager.is_running:
            QMessageBox.warning(self, "경고", "먼저 카메라를 시작하세요.")
            return

        # 설정 가져오기
        self.collect_tag_id = self.spinCollectTagId.value() if hasattr(self, 'spinCollectTagId') else 0
        self.collect_target_count = self.spinCollectCount.value() if hasattr(self, 'spinCollectCount') else 100

        # 초기화
        self.collected_data = []
        self.collecting_data = True

        # UI 업데이트
        if hasattr(self, 'btnStartCollect'):
            self.btnStartCollect.setEnabled(False)
        if hasattr(self, 'btnStopCollect'):
            self.btnStopCollect.setEnabled(True)
        if hasattr(self, 'btnSaveCollect'):
            self.btnSaveCollect.setEnabled(False)
        if hasattr(self, 'progressCollect'):
            self.progressCollect.setValue(0)

        self._log(f"데이터 수집 시작: Tag ID={self.collect_tag_id}, 목표={self.collect_target_count}회")

    def _on_stop_collect(self):
        """데이터 수집 중지"""
        self.collecting_data = False

        # UI 업데이트
        if hasattr(self, 'btnStartCollect'):
            self.btnStartCollect.setEnabled(True)
        if hasattr(self, 'btnStopCollect'):
            self.btnStopCollect.setEnabled(False)
        if hasattr(self, 'btnSaveCollect'):
            self.btnSaveCollect.setEnabled(len(self.collected_data) > 0)

        self._log(f"데이터 수집 중지: {len(self.collected_data)}개 수집됨")

        # 간단한 통계 출력
        if len(self.collected_data) > 0:
            self._print_collect_statistics()

    def _collect_sample(self):
        """현재 프레임에서 샘플 수집"""
        if not self.collecting_data or not self.vision_manager.last_result:
            return

        import time

        # 타겟 태그 찾기
        for marker in self.vision_manager.last_result:
            if marker['id'] == self.collect_tag_id:
                sample = {
                    'timestamp': time.time(),
                    'tag_id': marker['id'],
                    'tvec_x': marker['tvec'][0],
                    'tvec_y': marker['tvec'][1],
                    'tvec_z': marker['tvec'][2],
                    'rvec_x': marker['rvec'][0],
                    'rvec_y': marker['rvec'][1],
                    'rvec_z': marker['rvec'][2],
                }

                # 회전 행렬에서 오일러 각도 계산
                euler = self.vision_manager.aruco_detector._rotation_matrix_to_euler(marker['camera_rotation'])
                sample['euler_rx'] = euler[0]
                sample['euler_ry'] = euler[1]
                sample['euler_rz'] = euler[2]

                self.collected_data.append(sample)

                # UI 업데이트
                count = len(self.collected_data)
                if hasattr(self, 'labelCollectStatus'):
                    self.labelCollectStatus.setText(f"수집: {count} / {self.collect_target_count}")
                if hasattr(self, 'progressCollect'):
                    progress = int(100 * count / self.collect_target_count)
                    self.progressCollect.setValue(min(progress, 100))

                # 목표 도달 시 자동 중지
                if count >= self.collect_target_count:
                    self._on_stop_collect()

                break

    def _print_collect_statistics(self):
        """수집된 데이터의 통계 출력"""
        if len(self.collected_data) == 0:
            return

        # numpy 배열로 변환
        tvec_x = np.array([d['tvec_x'] for d in self.collected_data])
        tvec_y = np.array([d['tvec_y'] for d in self.collected_data])
        tvec_z = np.array([d['tvec_z'] for d in self.collected_data])
        euler_rx = np.array([d['euler_rx'] for d in self.collected_data])
        euler_ry = np.array([d['euler_ry'] for d in self.collected_data])
        euler_rz = np.array([d['euler_rz'] for d in self.collected_data])

        self._log("=" * 50)
        self._log(f"수집 통계 (n={len(self.collected_data)})")
        self._log("-" * 50)
        self._log("위치 (mm):")
        self._log(f"  X: 평균={tvec_x.mean()*1000:.3f}, 표준편차={tvec_x.std()*1000:.3f}")
        self._log(f"  Y: 평균={tvec_y.mean()*1000:.3f}, 표준편차={tvec_y.std()*1000:.3f}")
        self._log(f"  Z: 평균={tvec_z.mean()*1000:.3f}, 표준편차={tvec_z.std()*1000:.3f}")
        self._log("-" * 50)
        self._log("회전 (deg):")
        self._log(f"  Rx: 평균={euler_rx.mean():.3f}, 표준편차={euler_rx.std():.3f}")
        self._log(f"  Ry: 평균={euler_ry.mean():.3f}, 표준편차={euler_ry.std():.3f}")
        self._log(f"  Rz: 평균={euler_rz.mean():.3f}, 표준편차={euler_rz.std():.3f}")
        self._log("=" * 50)

    def _on_save_collect(self):
        """수집된 데이터를 CSV로 저장"""
        if len(self.collected_data) == 0:
            QMessageBox.warning(self, "경고", "저장할 데이터가 없습니다.")
            return

        # 파일 저장 다이얼로그
        default_name = f"aruco_data_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
        filepath, _ = QFileDialog.getSaveFileName(
            self, "데이터 저장", default_name, "CSV Files (*.csv)"
        )

        if not filepath:
            return

        try:
            import csv

            with open(filepath, 'w', newline='') as f:
                writer = csv.DictWriter(f, fieldnames=self.collected_data[0].keys())
                writer.writeheader()
                writer.writerows(self.collected_data)

            self._log(f"데이터 저장 완료: {filepath}")
            QMessageBox.information(self, "완료", f"데이터가 저장되었습니다.\n{filepath}")

        except Exception as e:
            self._log(f"데이터 저장 실패: {e}")
            QMessageBox.critical(self, "오류", f"저장 실패: {e}")

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
