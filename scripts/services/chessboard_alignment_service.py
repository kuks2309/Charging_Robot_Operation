#!/usr/bin/env python3
"""
ChessboardAlignmentService - 체스보드 기반 중심 정렬 서비스

tab_calibration.py에서 분리된 정렬 로직:
- align_center: 체스보드 중심 정렬 (오차 5px 이상시 재정렬)
"""

import time
from typing import Optional, Callable, Tuple
from dataclasses import dataclass

from PyQt5.QtCore import QObject, pyqtSignal


@dataclass
class AlignmentResult:
    """정렬 결과"""
    success: bool
    message: str
    error_x_px: float = 0.0
    error_y_px: float = 0.0
    error_x_mm: float = 0.0
    error_y_mm: float = 0.0
    retried: bool = False


class ChessboardAlignmentService(QObject):
    """체스보드 기반 중심 정렬 서비스"""

    # Qt Signals
    status_changed = pyqtSignal(str)  # 상태 변경 로그
    alignment_completed = pyqtSignal(bool, str, float, float)  # success, msg, error_x, error_y

    # 상수 (1920x1080 기준, fy 비율 이론값 — 현장 측정 후 확정)
    PIXEL_TO_MM = 0.0468  # 0.0625 × (fy_old/fy_new) = 0.0625 × (4003.2/5347.3)
    RETRY_THRESHOLD_PX = 5.0  # 재정렬 임계값 (픽셀)

    def __init__(self, robot_client=None):
        """
        Args:
            robot_client: ModbusClient 인스턴스 (로봇 이동용)
        """
        super().__init__()
        self.robot = robot_client
        self._log_callback: Optional[Callable[[str], None]] = None

    def set_log_callback(self, callback: Callable[[str], None]):
        """로그 콜백 설정"""
        self._log_callback = callback

    def set_robot(self, robot_client):
        """로봇 클라이언트 설정"""
        self.robot = robot_client

    def _log(self, message: str):
        """로그 출력"""
        if self._log_callback:
            self._log_callback(message)
        self.status_changed.emit(message)

    def align_center(
        self,
        current_frame,
        find_pose_callback: Callable,
        process_events_callback: Optional[Callable] = None
    ) -> AlignmentResult:
        """
        체스보드 중심 정렬 수행
        - 오차가 5px 이상이면 자동 재정렬 (1회)

        Args:
            current_frame: 현재 카메라 프레임
            find_pose_callback: 체스보드 포즈 검출 콜백 (frame) -> (center, angle) or None
            process_events_callback: Qt 이벤트 처리 콜백 (QApplication.processEvents)

        Returns:
            AlignmentResult: 정렬 결과
        """
        if current_frame is None:
            return AlignmentResult(False, "프레임 없음")

        # 로봇 연결 확인
        if not self.robot or not self.robot.is_connected:
            return AlignmentResult(False, "로봇 미연결")

        # 1차 정렬
        result = self._perform_alignment(
            current_frame, find_pose_callback, process_events_callback
        )
        if not result.success:
            return result

        # 오차 확인 및 재정렬
        if abs(result.error_x_px) >= self.RETRY_THRESHOLD_PX or abs(result.error_y_px) >= self.RETRY_THRESHOLD_PX:
            self._log(f"오차 {self.RETRY_THRESHOLD_PX}px 이상 - 재정렬 시도")

            # 재정렬 이동
            dx_mm = -result.error_x_px * self.PIXEL_TO_MM
            dy_mm = -result.error_y_px * self.PIXEL_TO_MM

            success, msg = self._move_xy(dx_mm, dy_mm, process_events_callback)
            if not success:
                self._log(f"재정렬 이동 실패: {msg}")
                return AlignmentResult(
                    success=True,
                    message="1차 정렬 완료 (재정렬 실패)",
                    error_x_px=result.error_x_px,
                    error_y_px=result.error_y_px,
                    error_x_mm=result.error_x_mm,
                    error_y_mm=result.error_y_mm,
                    retried=True
                )

            self._log(f"재정렬 완료: {msg}")

            # 재정렬 후 오차 확인
            time.sleep(0.3)
            error_x, error_y = self._check_alignment_error(
                current_frame, find_pose_callback
            )
            error_x_mm = error_x * self.PIXEL_TO_MM
            error_y_mm = error_y * self.PIXEL_TO_MM

            self._log(f"[재정렬 후 오차] X={error_x:.1f}px ({error_x_mm:.2f}mm), Y={error_y:.1f}px ({error_y_mm:.2f}mm)")

            return AlignmentResult(
                success=True,
                message="재정렬 완료",
                error_x_px=error_x,
                error_y_px=error_y,
                error_x_mm=error_x_mm,
                error_y_mm=error_y_mm,
                retried=True
            )

        return result

    def _perform_alignment(
        self,
        current_frame,
        find_pose_callback: Callable,
        process_events_callback: Optional[Callable]
    ) -> AlignmentResult:
        """
        단일 정렬 수행

        Args:
            current_frame: 현재 프레임
            find_pose_callback: 포즈 검출 콜백
            process_events_callback: Qt 이벤트 처리 콜백

        Returns:
            AlignmentResult: 정렬 결과
        """
        # 체스보드 검출
        result = find_pose_callback(current_frame.copy())
        if result is None:
            return AlignmentResult(False, "체스보드 감지 실패")

        center, angle = result
        frame_height, frame_width = current_frame.shape[:2]
        image_center_x = frame_width / 2
        image_center_y = frame_height / 2

        # 오프셋 계산
        dx_pixel = center[0] - image_center_x
        dy_pixel = center[1] - image_center_y
        dx_mm = -dx_pixel * self.PIXEL_TO_MM
        dy_mm = -dy_pixel * self.PIXEL_TO_MM

        self._log(f"중심 오프셋: dx={dx_pixel:.1f}px, dy={dy_pixel:.1f}px")
        self._log(f"로봇 이동량: X={dx_mm:.2f}mm, Y={dy_mm:.2f}mm")

        # 로봇 이동
        success, msg = self._move_xy(dx_mm, dy_mm, process_events_callback)
        if not success:
            return AlignmentResult(False, f"이동 실패: {msg}")

        self._log(f"XY 이동 완료: {msg}")

        # 이동 완료 후 오차 확인
        time.sleep(0.3)
        error_x, error_y = self._check_alignment_error(current_frame, find_pose_callback)
        error_x_mm = error_x * self.PIXEL_TO_MM
        error_y_mm = error_y * self.PIXEL_TO_MM

        self._log(f"[정렬 오차] X={error_x:.1f}px ({error_x_mm:.2f}mm), Y={error_y:.1f}px ({error_y_mm:.2f}mm)")

        return AlignmentResult(
            success=True,
            message="정렬 완료",
            error_x_px=error_x,
            error_y_px=error_y,
            error_x_mm=error_x_mm,
            error_y_mm=error_y_mm,
            retried=False
        )

    def _move_xy(
        self,
        dx_mm: float,
        dy_mm: float,
        process_events_callback: Optional[Callable]
    ) -> Tuple[bool, str]:
        """
        XY 이동 (툴 좌표계 기준)

        Args:
            dx_mm: X 이동량 (mm)
            dy_mm: Y 이동량 (mm)
            process_events_callback: Qt 이벤트 처리 콜백

        Returns:
            (success, message)
        """
        if not self.robot or not self.robot.is_connected:
            return False, "로봇 미연결"

        return self.robot.send_tcp_linear(
            'xyz',
            (dx_mm, dy_mm, 0),
            process_events_callback=process_events_callback
        )

    def _check_alignment_error(
        self,
        current_frame,
        find_pose_callback: Callable
    ) -> Tuple[float, float]:
        """
        정렬 오차 확인

        Args:
            current_frame: 현재 프레임
            find_pose_callback: 포즈 검출 콜백

        Returns:
            (error_x_px, error_y_px): 오차 (픽셀)
        """
        if current_frame is None:
            return 0.0, 0.0

        result = find_pose_callback(current_frame.copy())
        if result is None:
            return 0.0, 0.0

        center, _ = result
        frame_height, frame_width = current_frame.shape[:2]

        error_x = center[0] - frame_width / 2
        error_y = center[1] - frame_height / 2

        return error_x, error_y
