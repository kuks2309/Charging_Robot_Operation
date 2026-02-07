#!/usr/bin/env python3
"""
AR Tag TCP Align 탭
ArUco 마커를 이용한 TCP 정렬 기능
- 마커 감지 후 TCP 보정값(dRx, dRy, dRz) 계산
- TF5 기준 tool.trans 상대 회전으로 마커와 평행 정렬
"""

import os
from PyQt5 import uic
from PyQt5.QtWidgets import QWidget, QMessageBox
from PyQt5.QtCore import pyqtSignal

from .jog_mixin import JogMixin


# UI 파일 경로
UI_DIR = os.path.join(os.path.dirname(__file__), '..', '..', 'ui')
TAB_AR_TAG_TCP_ALIGN_UI = os.path.join(UI_DIR, 'tab_ar_tag_tcp_align.ui')


class TabArTagTcpAlign(QWidget, JogMixin):
    """AR Tag TCP Align 탭 클래스"""

    # 시그널 정의
    log_message = pyqtSignal(str)
    camera_start_requested = pyqtSignal()
    camera_stop_requested = pyqtSignal()
    jog_move_requested = pyqtSignal(str, float)   # axis, distance(mm)
    jog_rotate_requested = pyqtSignal(str, float)  # axis, angle(deg)
    detect_requested = pyqtSignal()                # ArUco 감지 요청
    align_parallel_requested = pyqtSignal(float, float, float)  # dRx, dRy, dRz

    def __init__(self, parent=None):
        super().__init__(parent)

        # UI 로드
        uic.loadUi(TAB_AR_TAG_TCP_ALIGN_UI, self)

        # 로봇 참조
        self.robot = None

        # 카메라 매니저 참조
        self.camera_manager = None

        # 현재 프레임
        self.current_frame = None

        # 최근 TCP 보정값 (dRx, dRy, dRz)
        self._tcp_correction = None

        # UI 초기화
        self._setup_ui()

    def _setup_ui(self):
        """UI 초기화"""
        # 카메라 버튼 시그널
        self.btnStartCamera.clicked.connect(lambda: self.camera_start_requested.emit())
        self.btnStopCamera.clicked.connect(lambda: self.camera_stop_requested.emit())

        # 정렬 제어 버튼
        self.btnDetect.clicked.connect(self._on_detect)
        self.btnAlign.clicked.connect(self._on_align_parallel)

        # 조그 이동 좌표계 표시
        self.groupJogMove.setTitle("조그 이동 (베이스기준)")

        # 조그 이동 버튼 시그널 - JogMixin
        self._connect_jog_buttons()

    def set_robot(self, robot):
        """로봇 참조 설정"""
        self.robot = robot

    def set_camera_manager(self, camera_manager):
        """카메라 매니저 참조 설정"""
        self.camera_manager = camera_manager

    def update_frame(self, frame):
        """카메라 프레임 업데이트"""
        self.current_frame = frame

    def update_tcp_correction(self, drx: float, dry: float, drz: float):
        """TCP 보정값 업데이트 (외부에서 ArUco 감지 결과 전달)"""
        self._tcp_correction = (drx, dry, drz)
        self.labelStatusTCPValue.setText(
            f"dRx={drx:.2f}°, dRy={dry:.2f}°, dRz={drz:.2f}°")
        self._log(f"TCP 보정값 갱신: dRx={drx:.2f}, dRy={dry:.2f}, dRz={drz:.2f}°")

    def _on_detect(self):
        """감지 버튼 핸들러 - ArUco 감지 요청"""
        if self.current_frame is None:
            QMessageBox.warning(self, "경고", "카메라가 실행되지 않았습니다.")
            return
        self._log("ArUco 마커 감지 요청...")
        self.detect_requested.emit()

    def _on_align_parallel(self):
        """정렬 실행 버튼 핸들러 - TF5 기준 tool.trans 회전 보정"""
        if self._tcp_correction is None:
            QMessageBox.warning(self, "경고",
                "TCP 보정값이 없습니다.\n먼저 '감지' 버튼으로 마커를 감지하세요.")
            return

        drx, dry, drz = self._tcp_correction
        self._log(f"마커 평행 정렬 실행: dRx={drx:.2f}, dRy={dry:.2f}, dRz={drz:.2f}°")
        self.align_parallel_requested.emit(drx, dry, drz)

    def _log(self, message):
        """로그 메시지 출력"""
        self.textLog.append(message)
        self.log_message.emit(message)
