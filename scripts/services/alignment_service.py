#!/usr/bin/env python3
"""
AlignmentService - Aruco 태그 기반 정렬 서비스

MainWindow에서 분리된 정렬 로직:
- align_center: 태그 중심 정렬
- align_pose: 태그 자세 정렬
- align_full: 전체 정렬 (중심 + 자세)
"""

from typing import Optional, Callable, Tuple
from dataclasses import dataclass
from enum import Enum

from PyQt5.QtCore import QObject, pyqtSignal

from .vision_manager import VisionManager


class AlignmentStep(Enum):
    """정렬 단계"""
    IDLE = "idle"
    DETECTING = "detecting"
    ALIGNING_CENTER = "center"
    ALIGNING_POSE = "pose"
    COMPLETED = "completed"
    FAILED = "failed"


@dataclass
class AlignmentResult:
    """정렬 결과"""
    success: bool
    message: str
    offset_x: float = 0.0
    offset_y: float = 0.0
    offset_rx: float = 0.0
    offset_ry: float = 0.0
    offset_rz: float = 0.0


class AlignmentService(QObject):
    """Aruco 태그 기반 정렬 서비스"""

    # Qt Signals
    status_changed = pyqtSignal(str)  # 상태 변경
    alignment_completed = pyqtSignal(bool, str)  # 완료 (성공여부, 메시지)

    def __init__(self, vision_manager: VisionManager, robot_client=None):
        """
        Args:
            vision_manager: VisionManager 인스턴스
            robot_client: ModbusClient 인스턴스 (로봇 이동용)
        """
        super().__init__()

        self.vision_manager = vision_manager
        self.robot = robot_client

        # 현재 상태
        self._current_step = AlignmentStep.IDLE

        # 로그 콜백
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

    def _update_status(self, status: str):
        """상태 업데이트 및 시그널 발생"""
        self._log(status)
        self.status_changed.emit(status)

    @property
    def current_step(self) -> AlignmentStep:
        """현재 정렬 단계"""
        return self._current_step

    def align_center(self, tag_id: int, num_samples: int = 10,
                     timeout: float = 10.0) -> AlignmentResult:
        """
        Aruco Tag 중심 정렬

        Args:
            tag_id: 타겟 태그 ID
            num_samples: 평균 측정 횟수
            timeout: 타임아웃 (초)

        Returns:
            AlignmentResult
        """
        self._current_step = AlignmentStep.DETECTING
        self._update_status("중심 정렬 중...")

        # 카메라 체크
        if not self.vision_manager.camera_manager.is_running:
            self._current_step = AlignmentStep.FAILED
            return AlignmentResult(
                success=False,
                message="카메라가 실행 중이 아닙니다."
            )

        # 태그 감지 (n회 평균)
        self._log(f"Aruco Tag {tag_id} 감지 중... ({num_samples}회 측정)")
        marker = self.vision_manager.detect_tag(tag_id, timeout, num_samples)

        if marker is None:
            self._current_step = AlignmentStep.FAILED
            self._update_status("태그 감지 실패")
            return AlignmentResult(
                success=False,
                message=f"Tag ID {tag_id}를 찾을 수 없습니다."
            )

        # 중심 오프셋 계산 (카메라 중심에서 태그까지)
        self._current_step = AlignmentStep.ALIGNING_CENTER
        tvec = marker['tvec']
        offset_x = tvec[0] * 1000  # m to mm
        offset_y = tvec[1] * 1000  # m to mm

        self._log(f"중심 오프셋: X={offset_x:.2f}mm, Y={offset_y:.2f}mm")

        # 로봇 이동
        if self.robot and self.robot.is_connected:
            success, msg = self.robot.send_tcp_linear(
                axis='xyz',
                distance=(-offset_x, -offset_y, 0),
                absolute=False,
                wait=True
            )

            if success:
                self._current_step = AlignmentStep.COMPLETED
                result_msg = f"중심 정렬 완료 (X:{-offset_x:.1f}, Y:{-offset_y:.1f})"
                self._update_status(result_msg)
                self.alignment_completed.emit(True, result_msg)
                return AlignmentResult(
                    success=True,
                    message=result_msg,
                    offset_x=-offset_x,
                    offset_y=-offset_y
                )
            else:
                self._current_step = AlignmentStep.FAILED
                self._update_status(f"이동 실패: {msg}")
                return AlignmentResult(success=False, message=f"이동 실패: {msg}")
        else:
            self._current_step = AlignmentStep.FAILED
            return AlignmentResult(
                success=False,
                message="로봇에 연결되어 있지 않습니다.",
                offset_x=offset_x,
                offset_y=offset_y
            )

    def align_pose(self, tag_id: int, num_samples: int = 10,
                   timeout: float = 10.0) -> AlignmentResult:
        """
        Aruco Tag 자세 정렬

        Args:
            tag_id: 타겟 태그 ID
            num_samples: 평균 측정 횟수
            timeout: 타임아웃 (초)

        Returns:
            AlignmentResult
        """
        self._current_step = AlignmentStep.DETECTING
        self._update_status("자세 정렬 중...")

        # 카메라 체크
        if not self.vision_manager.camera_manager.is_running:
            self._current_step = AlignmentStep.FAILED
            return AlignmentResult(
                success=False,
                message="카메라가 실행 중이 아닙니다."
            )

        # 태그 감지 (n회 평균)
        self._log(f"Aruco Tag {tag_id} 자세 감지 중... ({num_samples}회 측정)")
        marker = self.vision_manager.detect_tag(tag_id, timeout, num_samples)

        if marker is None:
            self._current_step = AlignmentStep.FAILED
            self._update_status("태그 감지 실패")
            return AlignmentResult(
                success=False,
                message=f"Tag ID {tag_id}를 찾을 수 없습니다."
            )

        # 회전 오프셋 계산
        self._current_step = AlignmentStep.ALIGNING_POSE
        euler_angles = self.vision_manager.aruco_detector._rotation_matrix_to_euler(
            marker['camera_rotation']
        )
        rx, ry, rz = euler_angles

        self._log(f"자세 오프셋: Rx={rx:.2f}°, Ry={ry:.2f}°, Rz={rz:.2f}°")

        # 로봇 회전
        if self.robot and self.robot.is_connected:
            success, msg = self.robot.send_tcp_rotate(
                axis='rxryrz',
                angle=(-rx, -ry, -rz),
                absolute=False,
                wait=True
            )

            if success:
                self._current_step = AlignmentStep.COMPLETED
                result_msg = f"자세 정렬 완료 (Rx:{-rx:.1f}, Ry:{-ry:.1f}, Rz:{-rz:.1f})"
                self._update_status(result_msg)
                self.alignment_completed.emit(True, result_msg)
                return AlignmentResult(
                    success=True,
                    message=result_msg,
                    offset_rx=-rx,
                    offset_ry=-ry,
                    offset_rz=-rz
                )
            else:
                self._current_step = AlignmentStep.FAILED
                self._update_status(f"회전 실패: {msg}")
                return AlignmentResult(success=False, message=f"회전 실패: {msg}")
        else:
            self._current_step = AlignmentStep.FAILED
            return AlignmentResult(
                success=False,
                message="로봇에 연결되어 있지 않습니다.",
                offset_rx=rx,
                offset_ry=ry,
                offset_rz=rz
            )

    def align_full(self, tag_id: int, num_samples: int = 10,
                   timeout: float = 10.0) -> AlignmentResult:
        """
        Aruco Tag 전체 정렬 (자세 -> 중심)

        Args:
            tag_id: 타겟 태그 ID
            num_samples: 평균 측정 횟수
            timeout: 타임아웃 (초)

        Returns:
            AlignmentResult
        """
        self._current_step = AlignmentStep.DETECTING
        self._update_status("전체 정렬 중...")

        # 카메라 체크
        if not self.vision_manager.camera_manager.is_running:
            self._current_step = AlignmentStep.FAILED
            return AlignmentResult(
                success=False,
                message="카메라가 실행 중이 아닙니다."
            )

        # 로봇 체크
        if not self.robot or not self.robot.is_connected:
            self._current_step = AlignmentStep.FAILED
            return AlignmentResult(
                success=False,
                message="로봇에 연결되어 있지 않습니다."
            )

        # 태그 감지 (n회 평균)
        self._log(f"Aruco Tag {tag_id} 전체 정렬 시작 ({num_samples}회 측정)")
        marker = self.vision_manager.detect_tag(tag_id, timeout, num_samples)

        if marker is None:
            self._current_step = AlignmentStep.FAILED
            self._update_status("태그 감지 실패")
            return AlignmentResult(
                success=False,
                message=f"Tag ID {tag_id}를 찾을 수 없습니다."
            )

        # 오프셋 계산
        tvec = marker['tvec']
        offset_x = tvec[0] * 1000  # m to mm
        offset_y = tvec[1] * 1000  # m to mm

        euler_angles = self.vision_manager.aruco_detector._rotation_matrix_to_euler(
            marker['camera_rotation']
        )
        rx, ry, rz = euler_angles

        self._log(f"중심: X={offset_x:.2f}mm, Y={offset_y:.2f}mm")
        self._log(f"자세: Rx={rx:.2f}°, Ry={ry:.2f}°, Rz={rz:.2f}°")

        # 1. 자세 정렬
        self._current_step = AlignmentStep.ALIGNING_POSE
        self._update_status("자세 정렬 중...")

        success, msg = self.robot.send_tcp_rotate(
            axis='rxryrz',
            angle=(-rx, -ry, -rz),
            absolute=False,
            wait=True
        )

        if not success:
            self._current_step = AlignmentStep.FAILED
            self._update_status(f"자세 정렬 실패: {msg}")
            return AlignmentResult(success=False, message=f"자세 정렬 실패: {msg}")

        self._log("자세 정렬 완료")

        # 2. 중심 정렬
        self._current_step = AlignmentStep.ALIGNING_CENTER
        self._update_status("중심 정렬 중...")

        success, msg = self.robot.send_tcp_linear(
            axis='xyz',
            distance=(-offset_x, -offset_y, 0),
            absolute=False,
            wait=True
        )

        if not success:
            self._current_step = AlignmentStep.FAILED
            self._update_status(f"중심 정렬 실패: {msg}")
            return AlignmentResult(success=False, message=f"중심 정렬 실패: {msg}")

        # 완료
        self._current_step = AlignmentStep.COMPLETED
        result_msg = "전체 정렬 완료"
        self._update_status(result_msg)
        self.alignment_completed.emit(True, result_msg)

        return AlignmentResult(
            success=True,
            message=result_msg,
            offset_x=-offset_x,
            offset_y=-offset_y,
            offset_rx=-rx,
            offset_ry=-ry,
            offset_rz=-rz
        )

    def get_offset(self, tag_id: int, num_samples: int = 10,
                   timeout: float = 10.0) -> Tuple[Optional[float], Optional[float],
                                                    Optional[float], Optional[float], Optional[float]]:
        """
        정렬 오프셋만 계산 (로봇 이동 없이)

        Args:
            tag_id: 타겟 태그 ID
            num_samples: 평균 측정 횟수
            timeout: 타임아웃 (초)

        Returns:
            (offset_x, offset_y, rx, ry, rz) 또는 모두 None
        """
        if not self.vision_manager.camera_manager.is_running:
            self._log("카메라가 실행 중이 아닙니다.")
            return None, None, None, None, None

        marker = self.vision_manager.detect_tag(tag_id, timeout, num_samples)

        if marker is None:
            self._log(f"Tag ID {tag_id}를 찾을 수 없습니다.")
            return None, None, None, None, None

        # 위치 오프셋
        tvec = marker['tvec']
        offset_x = tvec[0] * 1000  # m to mm
        offset_y = tvec[1] * 1000  # m to mm

        # 회전 오프셋
        euler_angles = self.vision_manager.aruco_detector._rotation_matrix_to_euler(
            marker['camera_rotation']
        )
        rx, ry, rz = euler_angles

        return offset_x, offset_y, rx, ry, rz
