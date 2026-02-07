#!/usr/bin/env python3
"""
모션 테스트 탭
Base/Tool 좌표계 기준 이동 테스트
"""

import os
import numpy as np
from PyQt5 import uic
from PyQt5.QtWidgets import QWidget, QMessageBox, QButtonGroup, QVBoxLayout
from PyQt5.QtCore import pyqtSignal
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure
from mpl_toolkits.mplot3d import Axes3D

from utils import require_robot_connection


# UI 파일 경로
UI_DIR = os.path.join(os.path.dirname(__file__), '..', '..', 'ui')
TAB_MOTION_TEST_UI = os.path.join(UI_DIR, 'tab_motion_test.ui')


class TabMotionTest(QWidget):
    """모션 테스트 탭 클래스"""

    # 시그널 정의
    log_message = pyqtSignal(str)

    def __init__(self, parent=None):
        super().__init__(parent)

        # UI 로드
        uic.loadUi(TAB_MOTION_TEST_UI, self)

        # 로봇 참조
        self.robot = None

        # 버튼 그룹 생성
        self.toolframe_button_group = QButtonGroup(self)
        self.toolframe_button_group.addButton(self.radioTF1, 1)
        self.toolframe_button_group.addButton(self.radioTF2, 2)
        self.toolframe_button_group.addButton(self.radioTF3, 3)
        self.toolframe_button_group.addButton(self.radioTF4, 4)
        self.toolframe_button_group.addButton(self.radioTF5, 5)
        self.radioTF1.setChecked(True)

        self.step_button_group = QButtonGroup(self)
        self.step_button_group.addButton(self.radioStep01, 1)   # 0.1 -> id=1
        self.step_button_group.addButton(self.radioStep05, 5)   # 0.5 -> id=5
        self.step_button_group.addButton(self.radioStep1, 10)   # 1 -> id=10
        self.step_button_group.addButton(self.radioStep5, 50)   # 5 -> id=50
        self.step_button_group.addButton(self.radioStep10, 100) # 10 -> id=100
        self.step_button_group.addButton(self.radioStep50, 500) # 50 -> id=500

        # 3D 그래프 초기화
        self._init_3d_plot()

        # 시그널 연결
        self._connect_signals()

    def _connect_signals(self):
        """내부 시그널-슬롯 연결"""
        # 프레임 설정
        self.toolframe_button_group.buttonClicked.connect(self._on_toolframe_changed)
        self.btnResetBaseframe.clicked.connect(self._on_reset_baseframe)

        # 툴 좌표계 이동
        self.btnToolXMinus.clicked.connect(lambda: self._on_tool_linear_move('x', -1))
        self.btnToolXPlus.clicked.connect(lambda: self._on_tool_linear_move('x', 1))
        self.btnToolYMinus.clicked.connect(lambda: self._on_tool_linear_move('y', -1))
        self.btnToolYPlus.clicked.connect(lambda: self._on_tool_linear_move('y', 1))
        self.btnToolZMinus.clicked.connect(lambda: self._on_tool_linear_move('z', -1))
        self.btnToolZPlus.clicked.connect(lambda: self._on_tool_linear_move('z', 1))

        self.btnToolRxMinus.clicked.connect(lambda: self._on_tool_rotate_move('rx', -1))
        self.btnToolRxPlus.clicked.connect(lambda: self._on_tool_rotate_move('rx', 1))
        self.btnToolRyMinus.clicked.connect(lambda: self._on_tool_rotate_move('ry', -1))
        self.btnToolRyPlus.clicked.connect(lambda: self._on_tool_rotate_move('ry', 1))
        self.btnToolRzMinus.clicked.connect(lambda: self._on_tool_rotate_move('rz', -1))
        self.btnToolRzPlus.clicked.connect(lambda: self._on_tool_rotate_move('rz', 1))

        # 베이스 좌표계 이동
        self.btnBaseXMinus.clicked.connect(lambda: self._on_base_linear_move('x', -1))
        self.btnBaseXPlus.clicked.connect(lambda: self._on_base_linear_move('x', 1))
        self.btnBaseYMinus.clicked.connect(lambda: self._on_base_linear_move('y', -1))
        self.btnBaseYPlus.clicked.connect(lambda: self._on_base_linear_move('y', 1))
        self.btnBaseZMinus.clicked.connect(lambda: self._on_base_linear_move('z', -1))
        self.btnBaseZPlus.clicked.connect(lambda: self._on_base_linear_move('z', 1))

        self.btnBaseRxMinus.clicked.connect(lambda: self._on_base_rotate_move('rx', -1))
        self.btnBaseRxPlus.clicked.connect(lambda: self._on_base_rotate_move('rx', 1))
        self.btnBaseRyMinus.clicked.connect(lambda: self._on_base_rotate_move('ry', -1))
        self.btnBaseRyPlus.clicked.connect(lambda: self._on_base_rotate_move('ry', 1))
        self.btnBaseRzMinus.clicked.connect(lambda: self._on_base_rotate_move('rz', -1))
        self.btnBaseRzPlus.clicked.connect(lambda: self._on_base_rotate_move('rz', 1))

    def _log(self, message: str):
        """로그 메시지 출력"""
        self.log_message.emit(message)
        print(f"[MotionTest] {message}")

    def set_robot(self, robot):
        """로봇 참조 설정"""
        self.robot = robot

    def _init_3d_plot(self):
        """3D 그래프 초기화"""
        # Matplotlib Figure 생성
        self.fig = Figure(figsize=(5, 5), dpi=100)
        self.canvas = FigureCanvas(self.fig)

        # 3D axes 생성
        self.ax = self.fig.add_subplot(111, projection='3d')

        # widget3DPlot에 canvas 추가
        layout = QVBoxLayout(self.widget3DPlot)
        layout.addWidget(self.canvas)
        layout.setContentsMargins(0, 0, 0, 0)

        # 초기 좌표계 설정
        self._setup_3d_axes()

        # 초기 TCP 위치
        self.current_tcp_pos = [0, 0, 0]
        self.current_tcp_rot = [0, 0, 0]

        # 줌 레벨 (1.0 = 기본)
        self.zoom_level = 1.0

        # 마우스 휠 이벤트 연결
        self.canvas.mpl_connect('scroll_event', self._on_scroll)

        # 초기 그리기
        self._update_3d_plot()

    def _on_scroll(self, event):
        """마우스 휠 스크롤 이벤트 처리 (확대/축소)"""
        if event.inaxes != self.ax:
            return

        # 줌 비율 설정
        zoom_factor = 1.15
        if event.button == 'up':
            # 휠 위로 = 확대
            self.zoom_level /= zoom_factor
        elif event.button == 'down':
            # 휠 아래로 = 축소
            self.zoom_level *= zoom_factor

        # 줌 레벨 제한 (0.2 ~ 5.0)
        self.zoom_level = max(0.2, min(5.0, self.zoom_level))

        # 축 범위 업데이트
        self._update_3d_plot()

    def _setup_3d_axes(self, preserve_view=False):
        """3D axes 설정"""
        # 현재 view angle 저장 (preserve_view=True일 때)
        if preserve_view:
            elev = self.ax.elev
            azim = self.ax.azim

        self.ax.clear()

        # 축 범위 설정 (mm 단위) - 줌 레벨 적용
        base_limit = 500
        limit = base_limit * getattr(self, 'zoom_level', 1.0)
        self.ax.set_xlim([-limit, limit])
        self.ax.set_ylim([-limit, limit])
        self.ax.set_zlim([0, limit * 2])

        # 축 라벨
        self.ax.set_xlabel('X (mm)', fontsize=10)
        self.ax.set_ylabel('Y (mm)', fontsize=10)
        self.ax.set_zlabel('Z (mm)', fontsize=10)

        # 뷰 각도 설정
        if preserve_view:
            # 저장된 view angle 복원
            self.ax.view_init(elev=elev, azim=azim)
        else:
            # 초기 view angle 설정
            self.ax.view_init(elev=20, azim=45)

        # 그리드
        self.ax.grid(True, alpha=0.3)

        # 축 비율 설정
        self.ax.set_box_aspect([1, 1, 2])  # X:Y:Z = 1:1:2

        # 베이스 좌표계 그리기 (원점)
        self._draw_coordinate_frame([0, 0, 0], [0, 0, 0], scale=100, alpha=0.6)

    def _draw_coordinate_frame(self, position, rotation, scale=100, alpha=1.0):
        """좌표 프레임 그리기 (RGB = XYZ), 회전 적용"""
        x, y, z = position
        rx, ry, rz = rotation  # degrees

        # 회전 행렬 계산 (Rx * Ry * Rz 순서)
        rx_rad = np.radians(rx)
        ry_rad = np.radians(ry)
        rz_rad = np.radians(rz)

        # Rotation matrix around X axis
        Rx = np.array([
            [1, 0, 0],
            [0, np.cos(rx_rad), -np.sin(rx_rad)],
            [0, np.sin(rx_rad), np.cos(rx_rad)]
        ])
        # Rotation matrix around Y axis
        Ry = np.array([
            [np.cos(ry_rad), 0, np.sin(ry_rad)],
            [0, 1, 0],
            [-np.sin(ry_rad), 0, np.cos(ry_rad)]
        ])
        # Rotation matrix around Z axis
        Rz = np.array([
            [np.cos(rz_rad), -np.sin(rz_rad), 0],
            [np.sin(rz_rad), np.cos(rz_rad), 0],
            [0, 0, 1]
        ])

        # Combined rotation matrix (Rz * Ry * Rx)
        R = Rz @ Ry @ Rx

        # 기본 축 벡터
        x_axis = R @ np.array([scale, 0, 0])
        y_axis = R @ np.array([0, scale, 0])
        z_axis = R @ np.array([0, 0, scale])

        # X축 (빨강)
        self.ax.quiver(x, y, z, x_axis[0], x_axis[1], x_axis[2],
                      color='red', arrow_length_ratio=0.2, alpha=alpha, linewidth=2)
        # Y축 (초록)
        self.ax.quiver(x, y, z, y_axis[0], y_axis[1], y_axis[2],
                      color='green', arrow_length_ratio=0.2, alpha=alpha, linewidth=2)
        # Z축 (파랑)
        self.ax.quiver(x, y, z, z_axis[0], z_axis[1], z_axis[2],
                      color='blue', arrow_length_ratio=0.2, alpha=alpha, linewidth=2)

    def _update_3d_plot(self):
        """3D 그래프 업데이트"""
        # axes 재설정 (view angle 보존)
        self._setup_3d_axes(preserve_view=True)

        # TCP 좌표계 그리기
        self._draw_coordinate_frame(
            self.current_tcp_pos,
            self.current_tcp_rot,
            scale=80,
            alpha=1.0
        )

        # TCP 위치에 점 표시
        self.ax.scatter(
            [self.current_tcp_pos[0]],
            [self.current_tcp_pos[1]],
            [self.current_tcp_pos[2]],
            color='black', s=50, marker='o'
        )

        # 캔버스 업데이트
        self.canvas.draw()

    def get_step_size(self) -> float:
        """현재 스텝 크기 반환"""
        # id를 실제 스텝 값으로 변환 (id는 10배로 저장)
        return float(self.step_button_group.checkedId()) / 10.0

    def is_wait_complete(self) -> bool:
        """완료 대기 여부 반환"""
        return self.checkWaitComplete.isChecked()

    # ==================== 프레임 설정 ====================

    @require_robot_connection
    def _on_toolframe_changed(self, button):
        """툴프레임 변경"""
        frame = self.toolframe_button_group.checkedId()
        self._log(f"툴프레임 {frame} 설정 중...")

        try:
            success, msg = self.robot.send_set_toolframe(frame, wait=True)
            if success:
                self._log(f"툴프레임 {frame} 설정 완료")
                self.labelCurrentToolframeValue.setText(f"TF{frame}")
            else:
                self._log(f"툴프레임 설정 실패: {msg}")
                QMessageBox.warning(self, "오류", f"툴프레임 설정 실패: {msg}")
        except Exception as e:
            self._log(f"툴프레임 설정 오류: {e}")
            QMessageBox.critical(self, "오류", f"툴프레임 설정 오류: {e}")

    @require_robot_connection
    def _on_reset_baseframe(self, checked=False):
        """베이스프레임 초기화"""
        self._log("베이스프레임 초기화 중...")

        try:
            success, msg = self.robot.send_reset_base(wait=True)
            if success:
                self._log("베이스프레임 초기화 완료")
            else:
                self._log(f"베이스프레임 초기화 실패: {msg}")
                QMessageBox.warning(self, "오류", f"베이스프레임 초기화 실패: {msg}")
        except Exception as e:
            self._log(f"베이스프레임 초기화 오류: {e}")
            QMessageBox.critical(self, "오류", f"베이스프레임 초기화 오류: {e}")

    # ==================== 툴 좌표계 이동 ====================

    @require_robot_connection
    def _on_tool_linear_move(self, axis: str, direction: int):
        """툴 좌표계 직선 이동"""
        step = self.get_step_size() * direction
        wait = self.is_wait_complete()

        self._log(f"툴 좌표계 {axis.upper()} {'+' if direction > 0 else ''}{step}mm 이동 중...")

        try:
            success, msg = self.robot.send_tcp_linear(axis, step, wait=wait)
            if success:
                self._log(f"이동 명령 전송 완료")
                self._update_position_display()
            else:
                self._log(f"이동 실패: {msg}")
                QMessageBox.warning(self, "오류", f"이동 실패: {msg}")
        except Exception as e:
            self._log(f"이동 오류: {e}")
            QMessageBox.critical(self, "오류", f"이동 오류: {e}")

    @require_robot_connection
    def _on_tool_rotate_move(self, axis: str, direction: int):
        """툴 좌표계 회전 이동"""
        step = self.get_step_size() * direction
        wait = self.is_wait_complete()

        self._log(f"툴 좌표계 {axis.upper()} {'+' if direction > 0 else ''}{step}deg 회전 중...")

        try:
            success, msg = self.robot.send_tcp_rotate(axis, step, wait=wait)
            if success:
                self._log(f"회전 명령 전송 완료")
                self._update_position_display()
            else:
                self._log(f"회전 실패: {msg}")
                QMessageBox.warning(self, "오류", f"회전 실패: {msg}")
        except Exception as e:
            self._log(f"회전 오류: {e}")
            QMessageBox.critical(self, "오류", f"회전 오류: {e}")

    # ==================== 베이스 좌표계 이동 ====================

    @require_robot_connection
    def _on_base_linear_move(self, axis: str, direction: int):
        """베이스 좌표계 직선 이동"""
        step = self.get_step_size() * direction
        wait = self.is_wait_complete()

        self._log(f"베이스 좌표계 {axis.upper()} {'+' if direction > 0 else ''}{step}mm 이동 중...")

        try:
            success, msg = self.robot.send_base_linear(axis, step, wait=wait)
            if success:
                self._log(f"이동 명령 전송 완료")
                self._update_position_display()
            else:
                self._log(f"이동 실패: {msg}")
                QMessageBox.warning(self, "오류", f"이동 실패: {msg}")
        except Exception as e:
            self._log(f"이동 오류: {e}")
            QMessageBox.critical(self, "오류", f"이동 오류: {e}")

    @require_robot_connection
    def _on_base_rotate_move(self, axis: str, direction: int):
        """베이스 좌표계 회전 이동"""
        step = self.get_step_size() * direction
        wait = self.is_wait_complete()

        self._log(f"베이스 좌표계 {axis.upper()} {'+' if direction > 0 else ''}{step}deg 회전 중...")

        try:
            success, msg = self.robot.send_base_rotate(axis, step, wait=wait)
            if success:
                self._log(f"회전 명령 전송 완료")
                self._update_position_display()
            else:
                self._log(f"회전 실패: {msg}")
                QMessageBox.warning(self, "오류", f"회전 실패: {msg}")
        except Exception as e:
            self._log(f"회전 오류: {e}")
            QMessageBox.critical(self, "오류", f"회전 오류: {e}")

    # ==================== 상태 업데이트 ====================

    def _update_position_display(self):
        """현재 위치 표시 업데이트"""
        if self.robot is None or not self.robot.is_connected:
            return

        try:
            pose = self.robot.read_camera_pose()
            if pose is not None:
                x, y, z, rx, ry, rz = pose
                self.labelCurrentPosValue.setText(f"X={x:.1f}, Y={y:.1f}, Z={z:.1f} mm")
                self.labelCurrentRotValue.setText(f"Rx={rx:.1f}, Ry={ry:.1f}, Rz={rz:.1f} deg")

            # 현재 툴프레임 읽기
            toolframe = self.robot.read_current_toolframe()
            if toolframe is not None:
                self.labelCurrentToolframeValue.setText(f"TF{toolframe}")
        except Exception as e:
            self._log(f"위치 읽기 오류: {e}")

    def update_robot_position(self, x: float, y: float, z: float,
                               rx: float, ry: float, rz: float):
        """로봇 좌표 업데이트 (외부에서 호출)"""
        # 상단 현재 상태
        self.labelCurrentPosValue.setText(f"X={x:.1f}, Y={y:.1f}, Z={z:.1f} mm")
        self.labelCurrentRotValue.setText(f"Rx={rx:.1f}, Ry={ry:.1f}, Rz={rz:.1f} deg")

        # 툴 좌표계 이동 섹션 (2줄 분리)
        self.labelToolCurrentPosValue.setText(f"X={x:.1f}, Y={y:.1f}, Z={z:.1f} mm")
        self.labelToolCurrentRotValue.setText(f"Rx={rx:.1f}, Ry={ry:.1f}, Rz={rz:.1f} deg")

        # 베이스 좌표계 이동 섹션 (2줄 분리)
        self.labelBaseCurrentPosValue.setText(f"X={x:.1f}, Y={y:.1f}, Z={z:.1f} mm")
        self.labelBaseCurrentRotValue.setText(f"Rx={rx:.1f}, Ry={ry:.1f}, Rz={rz:.1f} deg")

        # 3D 그래프 업데이트
        self.current_tcp_pos = [x, y, z]
        self.current_tcp_rot = [rx, ry, rz]
        self._update_3d_plot()

    def update_current_toolframe(self, toolframe: int):
        """현재 툴프레임 업데이트 (외부에서 호출)"""
        self.labelCurrentToolframeValue.setText(f"TF{toolframe}")
