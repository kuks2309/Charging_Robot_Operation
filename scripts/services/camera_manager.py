#!/usr/bin/env python3
"""
CameraManager - RealSense 카메라 관리 서비스

MainWindow에서 분리된 카메라 관련 로직:
- 카메라 초기화/시작/정지
- 프레임 캡처
- 스냅샷 저장
"""

import os
from datetime import datetime
from typing import Optional, Tuple, Callable
import numpy as np
import cv2

from PyQt5.QtCore import QObject, QTimer, pyqtSignal

# RealSense 카메라 (선택적 import)
try:
    import pyrealsense2 as rs
    REALSENSE_AVAILABLE = True
except ImportError:
    REALSENSE_AVAILABLE = False


class Intrinsics:
    """카메라 내부 파라미터"""
    def __init__(self, fx: float, fy: float, ppx: float, ppy: float, coeffs: list):
        self.fx = fx
        self.fy = fy
        self.ppx = ppx
        self.ppy = ppy
        self.coeffs = coeffs


class CameraManager(QObject):
    """RealSense 카메라 관리 클래스"""

    # Qt Signals
    frame_ready = pyqtSignal(np.ndarray)  # 새 프레임 준비됨
    camera_started = pyqtSignal()
    camera_stopped = pyqtSignal()
    error_occurred = pyqtSignal(str)

    def __init__(self, resolution: Tuple[int, int] = (640, 480), fps: int = 30):
        """
        Args:
            resolution: 카메라 해상도 (width, height)
            fps: 프레임 레이트
        """
        super().__init__()

        self.resolution = resolution
        self.fps = fps

        # RealSense 관련
        self._pipeline = None
        self._config = None
        self._running = False

        # 카메라 intrinsics
        self._intrinsics: Optional[Intrinsics] = None

        # 프레임 업데이트 타이머
        self._timer: Optional[QTimer] = None
        self._frame_interval = int(1000 / fps)  # ms

        # 마지막 프레임
        self._last_frame: Optional[np.ndarray] = None

        # 감마 보정
        self._gamma = 1.0

        # 로그 콜백
        self._log_callback: Optional[Callable[[str], None]] = None

    @property
    def is_available(self) -> bool:
        """RealSense 라이브러리 사용 가능 여부"""
        return REALSENSE_AVAILABLE

    @property
    def is_running(self) -> bool:
        """카메라 실행 상태"""
        return self._running

    @property
    def intrinsics(self) -> Optional[Intrinsics]:
        """카메라 내부 파라미터"""
        return self._intrinsics

    @property
    def last_frame(self) -> Optional[np.ndarray]:
        """마지막 캡처된 프레임"""
        return self._last_frame

    def set_log_callback(self, callback: Callable[[str], None]):
        """로그 콜백 설정"""
        self._log_callback = callback

    def _log(self, message: str):
        """로그 출력"""
        if self._log_callback:
            self._log_callback(message)

    def start(self) -> Tuple[bool, str]:
        """
        카메라 시작

        Returns:
            (성공 여부, 메시지)
        """
        if not REALSENSE_AVAILABLE:
            msg = "RealSense 라이브러리가 설치되지 않았습니다."
            self.error_occurred.emit(msg)
            return False, msg

        if self._running:
            return False, "카메라가 이미 실행 중입니다."

        try:
            self._pipeline = rs.pipeline()
            self._config = rs.config()

            # 해상도 설정
            width, height = self.resolution
            self._config.enable_stream(rs.stream.color, width, height, rs.format.bgr8, self.fps)

            # 파이프라인 시작
            profile = self._pipeline.start(self._config)

            # intrinsics 가져오기
            color_stream = profile.get_stream(rs.stream.color)
            rs_intrinsics = color_stream.as_video_stream_profile().get_intrinsics()
            self._intrinsics = Intrinsics(
                fx=rs_intrinsics.fx,
                fy=rs_intrinsics.fy,
                ppx=rs_intrinsics.ppx,
                ppy=rs_intrinsics.ppy,
                coeffs=list(rs_intrinsics.coeffs)
            )

            # 카메라 안정화 대기
            for _ in range(30):
                self._pipeline.wait_for_frames()

            self._running = True

            # 프레임 업데이트 타이머 시작
            self._timer = QTimer()
            self._timer.timeout.connect(self._update_frame)
            self._timer.start(self._frame_interval)

            self._log("카메라 시작됨")
            self.camera_started.emit()
            return True, "카메라 시작됨"

        except Exception as e:
            msg = f"카메라 시작 실패: {e}"
            self._log(msg)
            self.error_occurred.emit(msg)
            return False, msg

    def stop(self) -> Tuple[bool, str]:
        """
        카메라 정지

        Returns:
            (성공 여부, 메시지)
        """
        if not self._running:
            return False, "카메라가 실행 중이 아닙니다."

        try:
            if self._timer:
                self._timer.stop()
                self._timer = None

            if self._pipeline:
                self._pipeline.stop()
                self._pipeline = None

            self._running = False
            self._last_frame = None

            self._log("카메라 정지됨")
            self.camera_stopped.emit()
            return True, "카메라 정지됨"

        except Exception as e:
            msg = f"카메라 정지 오류: {e}"
            self._log(msg)
            self.error_occurred.emit(msg)
            return False, msg

    def _update_frame(self):
        """프레임 업데이트 (타이머에서 호출)"""
        if not self._running or not self._pipeline:
            return

        try:
            frames = self._pipeline.wait_for_frames()
            color_frame = frames.get_color_frame()

            if not color_frame:
                return

            # numpy 배열로 변환
            frame = np.asanyarray(color_frame.get_data())

            # 감마 보정 적용
            if self._gamma != 1.0:
                frame = self._apply_gamma(frame, self._gamma)

            self._last_frame = frame

            # 프레임 준비 시그널 발생
            self.frame_ready.emit(frame)

        except Exception as e:
            self._log(f"프레임 업데이트 오류: {e}")

    def get_frame(self) -> Optional[np.ndarray]:
        """
        현재 프레임 가져오기 (동기)

        Returns:
            BGR 프레임 또는 None
        """
        if not self._running or not self._pipeline:
            return None

        try:
            frames = self._pipeline.wait_for_frames()
            color_frame = frames.get_color_frame()

            if color_frame:
                frame = np.asanyarray(color_frame.get_data())
                if self._gamma != 1.0:
                    frame = self._apply_gamma(frame, self._gamma)
                return frame

        except Exception as e:
            self._log(f"프레임 가져오기 오류: {e}")

        return None

    def set_gamma(self, gamma: float):
        """
        감마 값 설정

        Args:
            gamma: 감마 값 (1.0 = 보정 없음)
        """
        self._gamma = max(0.1, min(3.0, gamma))

    def _apply_gamma(self, frame: np.ndarray, gamma: float) -> np.ndarray:
        """감마 보정 적용"""
        inv_gamma = 1.0 / gamma
        table = np.array([((i / 255.0) ** inv_gamma) * 255
                          for i in np.arange(0, 256)]).astype("uint8")
        return cv2.LUT(frame, table)

    def snapshot(self, save_dir: Optional[str] = None) -> Optional[str]:
        """
        스냅샷 저장

        Args:
            save_dir: 저장 디렉토리 (None이면 기본 snapshots 폴더)

        Returns:
            저장된 파일 경로 또는 None
        """
        if not self._running:
            self._log("카메라가 실행 중이 아닙니다.")
            return None

        frame = self.get_frame()
        if frame is None:
            self._log("프레임을 가져올 수 없습니다.")
            return None

        try:
            # 저장 디렉토리 설정
            if save_dir is None:
                save_dir = os.path.join(os.path.dirname(__file__), '..', '..', 'snapshots')

            os.makedirs(save_dir, exist_ok=True)

            # 파일명 생성
            filename = f"snapshot_{datetime.now().strftime('%Y%m%d_%H%M%S')}.png"
            filepath = os.path.join(save_dir, filename)

            # 저장
            cv2.imwrite(filepath, frame)
            self._log(f"스냅샷 저장: {filepath}")
            return filepath

        except Exception as e:
            self._log(f"스냅샷 저장 실패: {e}")
            return None

    def __del__(self):
        """소멸자"""
        if self._running:
            self.stop()
