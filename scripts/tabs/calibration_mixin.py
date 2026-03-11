#!/usr/bin/env python3
"""
캘리브레이션 공통 Mixin
TabCalibration과 TabEyeInHand에서 공유하는 로직
"""

import os
import time
import cv2
import numpy as np
from datetime import datetime
from PyQt5.QtWidgets import QMessageBox, QApplication

from utils.common import require_robot_connection
from utils.camera_calib_position_generator import (
    generate_base_positions_with_rotation,
    format_position_label_base,
)


class CalibrationMixin:
    """
    캘리브레이션 탭 공통 Mixin

    제공 기능:
    - 조그 이동 (X/Y/Z/Rx/Ry/Rz ±)
    - TCP 정렬 프리셋
    - 위치 생성/순차 이동/자동 캡처
    """

    PIXEL_TO_MM = 0.187  # 1920x1080 기준, fy 비율(4003.2/5347.3=0.7486) 이론값, 현장 측정 후 확정

    def init_mixin(self):
        """Mixin 초기화"""
        self.auto_calib_running = False
        self.auto_calib_positions = []
        self.auto_calib_base_pos = None

        self.aligned_distance = None
        self.aligned_rx = None
        self.aligned_ry = None
        self.aligned_rz = None
        self.aligned_x = None
        self.aligned_y = None
        self.aligned_z = None

    # ==================== 로봇 이동 ====================

    def get_step_size(self) -> float:
        """스텝 크기 반환 - 서브클래스에서 구현"""
        return 1.0

    @require_robot_connection
    def _on_base_move(self, axis: str, direction: int):
        """베이스 좌표계 직선 이동"""
        step = self.get_step_size() * direction
        self._log(f"베이스 {axis.upper()} {'+' if direction > 0 else ''}{step}mm 이동 중...")

        try:
            success, msg = self.robot.send_base_linear(
                axis, step, wait=True,
                process_events_callback=QApplication.processEvents
            )
            if success:
                self._log("이동 완료")
            else:
                self._log(f"이동 실패: {msg}")
        except Exception as e:
            self._log(f"이동 오류: {e}")

    @require_robot_connection
    def _on_base_rotate(self, axis: str, direction: int):
        """베이스 좌표계 회전 이동"""
        step = self.get_step_size() * direction
        self._log(f"베이스 {axis.upper()} {'+' if direction > 0 else ''}{step}deg 회전 중...")

        try:
            success, msg = self.robot.send_base_rotate(
                axis, step, wait=True,
                process_events_callback=QApplication.processEvents
            )
            if success:
                self._log("회전 완료")
            else:
                self._log(f"회전 실패: {msg}")
        except Exception as e:
            self._log(f"회전 오류: {e}")

    # ==================== TCP 정렬 ====================

    def get_tcp_align_values(self) -> tuple:
        """TCP 정렬 목표값 - 서브클래스에서 구현"""
        return (90.0, 0.0, 90.0)

    def set_tcp_align_values(self, rx: float, ry: float, rz: float):
        """TCP 정렬 목표값 설정 - 서브클래스에서 구현"""
        pass

    @require_robot_connection
    def _on_tcp_align(self, checked=False):
        """TCP 자세 정렬"""
        try:
            target_rx, target_ry, target_rz = self.get_tcp_align_values()
        except ValueError:
            QMessageBox.warning(self, "경고", "Rx, Ry, Rz 값을 올바르게 입력해주세요.")
            return

        current_pose = self.robot.read_current_pose()
        if current_pose is None:
            QMessageBox.warning(self, "오류", "현재 로봇 위치를 읽을 수 없습니다.")
            return

        x, y, z, rx, ry, rz = current_pose
        self._log(f"TCP 정렬: 목표 Rx={target_rx:.2f}, Ry={target_ry:.2f}, Rz={target_rz:.2f}")

        try:
            success, msg = self.robot.send_move_to_pose(
                x, y, z, target_rx, target_ry, target_rz,
                wait=True, process_events_callback=QApplication.processEvents
            )
            if success:
                self._log("TCP 정렬 완료")
            else:
                self._log(f"TCP 정렬 실패: {msg}")
        except Exception as e:
            self._log(f"TCP 정렬 오류: {e}")

    @require_robot_connection
    def _on_tcp_align_read(self, checked=False):
        """현재 TCP 자세 읽기"""
        current_pose = self.robot.read_current_pose()
        if current_pose is None:
            QMessageBox.warning(self, "오류", "현재 로봇 위치를 읽을 수 없습니다.")
            return

        x, y, z, rx, ry, rz = current_pose
        self.set_tcp_align_values(rx, ry, rz)
        self._log(f"현재 TCP 자세: Rx={rx:.2f}, Ry={ry:.2f}, Rz={rz:.2f}")

    def _on_tcp_align_center(self):
        """중앙 정렬 프리셋"""
        self._on_rx_preset(90)

    # ==================== 프리셋 ====================

    def _is_auto_correction_enabled(self) -> bool:
        """자동 보정 활성화 여부 - 서브클래스에서 구현"""
        return False

    def _on_rx_preset(self, rx: int):
        """Rx 프리셋"""
        self.set_tcp_align_values(rx, 0, 90)

        if (self._is_auto_correction_enabled() and
            self.aligned_distance is not None and
            self.aligned_rx is not None and
            self.aligned_z is not None):
            self._on_tcp_align_with_z_correction(rx)
        else:
            self._on_tcp_align()

    def _on_ry_preset(self, ry: int):
        """Ry 프리셋"""
        self.set_tcp_align_values(90, ry, 90)
        self._on_tcp_align()

    def _on_rz_preset(self, rz: int):
        """Rz 프리셋"""
        self.set_tcp_align_values(90, 0, rz)

        if (self._is_auto_correction_enabled() and
            self.aligned_distance is not None and
            self.aligned_rz is not None and
            self.aligned_y is not None):
            self._on_tcp_align_with_y_correction_rz(rz)
        else:
            self._on_tcp_align()

    # ==================== Z/Y 보정 ====================

    @require_robot_connection
    def _on_tcp_align_with_z_correction(self, target_rx: int):
        """Rx 변경 시 Z축 자동 보정"""
        import math

        D = self.aligned_distance
        delta_rx = target_rx - self.aligned_rx
        dz = D * math.tan(math.radians(delta_rx))

        self._log(f"[자동 보정] ΔRx={delta_rx:.1f}°, ΔZ={dz:.2f}mm")

        current_pose = self.robot.read_current_pose()
        if current_pose is None:
            return

        x, y, z, rx, ry, rz = current_pose
        new_z = self.aligned_z + dz

        try:
            self.robot.send_move_to_pose(
                x, y, new_z, target_rx, 0, 90,
                wait=True, process_events_callback=QApplication.processEvents
            )
            self._log("TCP 정렬 (Z 보정) 완료")
        except Exception as e:
            self._log(f"TCP 정렬 오류: {e}")

    @require_robot_connection
    def _on_tcp_align_with_y_correction_rz(self, target_rz: int):
        """Rz 변경 시 Y축 자동 보정"""
        import math

        D = self.aligned_distance
        delta_rz = target_rz - self.aligned_rz
        dy = D * math.tan(math.radians(delta_rz))

        self._log(f"[자동 보정] ΔRz={delta_rz:.1f}°, ΔY={dy:.2f}mm")

        current_pose = self.robot.read_current_pose()
        if current_pose is None:
            return

        x, y, z, rx, ry, rz = current_pose
        new_y = self.aligned_y - dy

        try:
            self.robot.send_move_to_pose(
                x, new_y, z, 90, 0, target_rz,
                wait=True, process_events_callback=QApplication.processEvents
            )
            self._log("TCP 정렬 (Y 보정) 완료")
        except Exception as e:
            self._log(f"TCP 정렬 오류: {e}")

    # ==================== 위치 생성 ====================

    def _on_generate_positions(self):
        """자동 캘리브레이션 위치 생성"""
        xy_step = self.spinXYStep.value()
        z_step = self.spinZStep.value()

        if not self.robot or not self.robot.is_connected:
            self._log("로봇 미연결 - 위치 생성 불가")
            return

        current_pose = self.robot.read_current_pose()
        if not current_pose:
            self._log("로봇 좌표 읽기 실패")
            return

        self.auto_calib_base_pos = current_pose
        x, y, z, rx, ry, rz = current_pose
        self._log(f"기준 좌표: X={x:.1f}, Y={y:.1f}, Z={z:.1f}")

        self.auto_calib_positions = generate_base_positions_with_rotation(
            current_pose, xy_step, z_step,
            aligned_distance=self.aligned_distance,
            aligned_y=self.aligned_y,
            aligned_z=self.aligned_z,
            aligned_rx=self.aligned_rx,
            aligned_rz=self.aligned_rz
        )

        self._update_position_list()
        self._log(f"위치 생성 완료: {len(self.auto_calib_positions)}개")

    def _on_clear_positions(self):
        """위치 리스트 초기화"""
        self.auto_calib_positions.clear()
        self.listAutoCalibPositions.clear()
        self._log("위치 리스트 초기화")

    def _update_position_list(self):
        """위치 리스트 UI 업데이트"""
        self.listAutoCalibPositions.clear()
        total = len(self.auto_calib_positions)
        for i, pos in enumerate(self.auto_calib_positions):
            label = format_position_label_base(i, pos, total, self.auto_calib_base_pos)
            self.listAutoCalibPositions.addItem(label)

    # ==================== 이동 ====================

    @require_robot_connection
    def _on_move_to_selected_base(self, checked=False):
        """선택된 위치로 이동"""
        if not self.auto_calib_positions:
            QMessageBox.warning(self, "경고", "먼저 위치를 생성해주세요.")
            return

        if self.listAutoCalibPositions.currentItem() is None:
            QMessageBox.warning(self, "경고", "이동할 위치를 선택해주세요.")
            return

        index = self.listAutoCalibPositions.currentRow()
        if not (0 <= index < len(self.auto_calib_positions)):
            return

        target = self.auto_calib_positions[index]
        self._log(f"[{index}] 이동 중...")

        try:
            success, msg = self.robot.send_move_to_pose(
                *target,
                wait=True, process_events_callback=QApplication.processEvents
            )
            if success:
                self._log(f"이동 완료")
            else:
                self._log(f"이동 실패: {msg}")
        except Exception as e:
            self._log(f"이동 오류: {e}")

    @require_robot_connection
    def _on_sequential_move(self, checked=False):
        """순차 이동"""
        if not self.auto_calib_positions:
            QMessageBox.warning(self, "경고", "먼저 위치를 생성해주세요.")
            return

        if self.auto_calib_running:
            return

        self.auto_calib_running = True
        self.btnSequentialMove.setEnabled(False)
        self.btnStopAutoCapture.setEnabled(True)

        total = len(self.auto_calib_positions)
        self._log(f"순차 이동 시작: {total}개")

        try:
            for index in range(total):
                if not self.auto_calib_running:
                    self._log("순차 이동 중지")
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

                time.sleep(0.3)

            if self.auto_calib_running:
                self._log("순차 이동 완료")

        finally:
            self.auto_calib_running = False
            self.btnSequentialMove.setEnabled(True)
            self.btnStopAutoCapture.setEnabled(False)

    # ==================== 자동 캡처 ====================

    def get_save_dir_prefix(self) -> str:
        """저장 디렉토리 접두사"""
        return "auto"

    def on_capture_at_position(self, index: int, total: int, save_dir: str,
                                target_pose: tuple) -> bool:
        """각 위치에서의 캡처 - 서브클래스에서 오버라이드 가능"""
        if self.current_frame is None:
            self._log(f"[{index+1}/{total}] 프레임 없음")
            return False

        target_x, target_y, target_z, target_rx, target_ry, target_rz = target_pose
        filename = f"{index:04d}_X{target_x:.1f}_Y{target_y:.1f}_Z{target_z:.1f}_Rx{target_rx:.1f}_Ry{target_ry:.1f}_Rz{target_rz:.1f}.png"
        filepath = os.path.join(save_dir, filename)

        try:
            cv2.imwrite(filepath, self.current_frame)
            self._log(f"[{index+1}/{total}] 캡처: {filename}")
            return True
        except Exception as e:
            self._log(f"[{index+1}/{total}] 캡처 실패: {e}")
            return False

    def _on_run_auto_capture(self):
        """자동 캡처 실행"""
        if not self.auto_calib_positions:
            QMessageBox.warning(self, "경고", "먼저 위치를 생성해주세요.")
            return

        if self.auto_calib_running:
            return

        # 저장 디렉토리
        home_dir = os.path.expanduser("~")
        base_dir = os.path.join(home_dir, "Project", "Charging_Robot_Operation", "calibration")
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        save_dir = os.path.join(base_dir, f"{self.get_save_dir_prefix()}_{timestamp}")

        try:
            os.makedirs(save_dir, exist_ok=True)
            self._log(f"저장 경로: {save_dir}")
        except Exception as e:
            QMessageBox.critical(self, "오류", f"디렉토리 생성 실패: {e}")
            return

        self.auto_calib_running = True
        self.btnRunAutoCapture.setEnabled(False)
        self.btnStopAutoCapture.setEnabled(True)

        total = len(self.auto_calib_positions)
        stabilization_sec = self.spinStabilizationDelay.value() / 1000.0

        self._log(f"자동 캡처 시작: {total}개")

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

                self.on_capture_at_position(index, total, save_dir, target)

            if self.auto_calib_running:
                self._log("자동 캡처 완료")
                QMessageBox.information(self, "완료", f"저장 위치: {save_dir}")

        finally:
            self.auto_calib_running = False
            self.btnRunAutoCapture.setEnabled(True)
            self.btnStopAutoCapture.setEnabled(False)

    def _on_stop_auto_capture(self):
        """자동 캡처 중지"""
        if self.auto_calib_running:
            self._log("중지 요청")
            self.auto_calib_running = False
            self.btnRunAutoCapture.setEnabled(True)
            self.btnStopAutoCapture.setEnabled(False)
