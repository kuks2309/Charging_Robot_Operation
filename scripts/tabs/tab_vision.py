#!/usr/bin/env python3
"""
비전 탭
Aruco 감지, 포즈 추정, 정렬 테스트 기능
"""

import os
import cv2
import numpy as np
from PyQt5 import uic
from PyQt5.QtWidgets import QWidget, QMessageBox, QButtonGroup
from PyQt5.QtCore import pyqtSignal

from utils import display_frame_on_label, save_snapshot, require_camera_running


# UI 파일 경로
UI_DIR = os.path.join(os.path.dirname(__file__), '..', '..', 'ui')
TAB_VISION_UI = os.path.join(UI_DIR, 'tab_vision.ui')


class TabVision(QWidget):
    """비전 탭 클래스"""

    # 시그널 정의
    log_message = pyqtSignal(str)
    camera_start_requested = pyqtSignal()
    camera_stop_requested = pyqtSignal()

    # 정렬 시그널
    align_center_requested = pyqtSignal(int, int)  # tag_id, num_samples
    align_pose_requested = pyqtSignal(int, int)
    align_full_requested = pyqtSignal(int, int)

    def __init__(self, parent=None):
        super().__init__(parent)

        # UI 로드
        uic.loadUi(TAB_VISION_UI, self)

        # 현재 프레임
        self.current_frame = None

        # 감지 결과
        self.detected_markers = []
        self.current_pose_camera = None
        self.current_pose_world = None

        # 카메라 선택 버튼 그룹 (DS435/ArduCam)
        self.camera_select_button_group = QButtonGroup(self)
        self.camera_select_button_group.addButton(self.radioDS435, 0)
        self.camera_select_button_group.addButton(self.radioArduCam, 1)

        # Vision Processor 버튼 그룹 (Gun/Port/Standard)
        self.processor_button_group = QButtonGroup(self)
        self.processor_button_group.addButton(self.radioProcessorGun, 0)
        self.processor_button_group.addButton(self.radioProcessorPort, 1)
        self.processor_button_group.addButton(self.radioProcessorStandard, 2)

        # 시그널 연결
        self._connect_signals()

        # 초기화
        self._init_ui()

    def _connect_signals(self):
        """내부 시그널-슬롯 연결"""
        # 카메라 버튼
        self.btnStartCamera.clicked.connect(self._on_start_camera)
        self.btnStopCamera.clicked.connect(self._on_stop_camera)
        self.btnSnapshot.clicked.connect(self._on_snapshot)

        # 정렬 버튼
        self.btnAlignCenter.clicked.connect(self._on_align_center)
        self.btnAlignPose.clicked.connect(self._on_align_pose)
        self.btnAlignFull.clicked.connect(self._on_align_full)

    def _init_ui(self):
        """UI 초기화"""
        pass

    def _log(self, message: str):
        """로그 메시지 출력"""
        self.log_message.emit(message)
        print(f"[Vision] {message}")

    def set_current_frame(self, frame: np.ndarray):
        """현재 프레임 설정 (외부에서 호출)"""
        self.current_frame = frame.copy() if frame is not None else None

    def display_frame(self, frame: np.ndarray):
        """프레임을 QLabel에 표시"""
        if frame is not None:
            display_frame_on_label(frame, self.labelCameraView)

    # ==================== UI 설정 접근자 ====================

    def is_aruco_detect_enabled(self) -> bool:
        """Aruco 감지 활성화 여부"""
        return self.checkArucoDetect.isChecked()

    def get_target_tag_id(self) -> int:
        """타겟 Tag ID 반환"""
        return self.spinTargetTagId.value()

    def get_num_samples(self) -> int:
        """샘플 수 반환"""
        return self.spinNumSamples.value()

    def is_pose_axes_enabled(self) -> bool:
        """포즈 축 표시 여부"""
        return self.checkShowPoseAxes.isChecked()

    def is_bounding_box_enabled(self) -> bool:
        """바운딩 박스 표시 여부"""
        return self.checkShowBoundingBox.isChecked()

    def is_keypoints_enabled(self) -> bool:
        """키포인트 표시 여부"""
        return self.checkShowKeypoints.isChecked()

    def get_selected_processor(self) -> str:
        """선택된 Vision Processor 반환"""
        if self.radioProcessorGun.isChecked():
            return 'gun'
        elif self.radioProcessorPort.isChecked():
            return 'port'
        elif self.radioProcessorStandard.isChecked():
            return 'standard'
        return 'gun'

    # ==================== 카메라 버튼 핸들러 ====================

    def _on_start_camera(self):
        """카메라 시작"""
        self.camera_start_requested.emit()

    def _on_stop_camera(self):
        """카메라 정지"""
        self.camera_stop_requested.emit()

    @require_camera_running
    def _on_snapshot(self):
        """스냅샷 저장"""
        save_snapshot(self.current_frame, self, "vision", self._log)

    # ==================== 정렬 버튼 핸들러 ====================

    def _on_align_center(self):
        """중심 정렬"""
        tag_id = self.get_target_tag_id()
        num_samples = self.get_num_samples()
        self.align_center_requested.emit(tag_id, num_samples)

    def _on_align_pose(self):
        """자세 정렬"""
        tag_id = self.get_target_tag_id()
        num_samples = self.get_num_samples()
        self.align_pose_requested.emit(tag_id, num_samples)

    def _on_align_full(self):
        """전체 정렬"""
        tag_id = self.get_target_tag_id()
        num_samples = self.get_num_samples()
        self.align_full_requested.emit(tag_id, num_samples)

    # ==================== 외부에서 호출하는 UI 업데이트 ====================

    def update_detection_result(self, tag_id: int = None, confidence: float = None, convergence: float = None):
        """감지 결과 업데이트"""
        if tag_id is not None:
            self.labelArUcoIDValue.setText(str(tag_id))
        else:
            self.labelArUcoIDValue.setText("-")

        if confidence is not None:
            self.progressConfidence.setValue(int(confidence * 100))
        else:
            self.progressConfidence.setValue(0)

        if convergence is not None:
            self.progressConvergence.setValue(int(convergence * 100))
        else:
            self.progressConvergence.setValue(0)

    def update_pose_camera(self, x: float, y: float, z: float, rx: float, ry: float, rz: float):
        """카메라 좌표계 포즈 업데이트"""
        self.editCamX.setText(f"{x:.2f}")
        self.editCamY.setText(f"{y:.2f}")
        self.editCamZ.setText(f"{z:.2f}")
        self.editCamRx.setText(f"{rx:.2f}")
        self.editCamRy.setText(f"{ry:.2f}")
        self.editCamRz.setText(f"{rz:.2f}")

    def update_pose_world(self, x: float, y: float, z: float, rx: float, ry: float, rz: float):
        """월드 좌표계 포즈 업데이트"""
        self.editWorldX.setText(f"{x:.2f}")
        self.editWorldY.setText(f"{y:.2f}")
        self.editWorldZ.setText(f"{z:.2f}")
        self.editWorldRx.setText(f"{rx:.2f}")
        self.editWorldRy.setText(f"{ry:.2f}")
        self.editWorldRz.setText(f"{rz:.2f}")

    def clear_pose_display(self):
        """포즈 표시 초기화"""
        for edit in [self.editCamX, self.editCamY, self.editCamZ,
                     self.editCamRx, self.editCamRy, self.editCamRz,
                     self.editWorldX, self.editWorldY, self.editWorldZ,
                     self.editWorldRx, self.editWorldRy, self.editWorldRz]:
            edit.setText("")

    def update_align_status(self, status: str):
        """정렬 상태 업데이트"""
        self.labelAlignStatus.setText(f"상태: {status}")
