#!/usr/bin/env python3
"""
ArduCamManager - USB UVC 카메라 (ArduCam) 관리 서비스

CameraManager와 동일한 인터페이스를 제공하여 쉽게 교체 가능
"""

import os
import yaml
from datetime import datetime
from typing import Optional, Tuple, Callable
import numpy as np
import cv2

from PyQt5.QtCore import QObject, QTimer, pyqtSignal

# 기본 캘리브레이션 파일 경로
DEFAULT_CALIBRATION_FILE = os.path.join(
    os.path.dirname(__file__), '..', '..', 'config', 'arducam_calibration.yaml'
)


class Intrinsics:
    """카메라 내부 파라미터 (CameraManager와 동일)"""
    def __init__(self, fx: float, fy: float, ppx: float, ppy: float, coeffs: list,
                 width: int = 1920, height: int = 1080):
        self.fx = fx
        self.fy = fy
        self.ppx = ppx
        self.ppy = ppy
        self.coeffs = coeffs
        self.width = width
        self.height = height

    @classmethod
    def from_yaml(cls, filepath: str) -> Optional['Intrinsics']:
        """YAML 캘리브레이션 파일에서 로드 (camera_matrix.data 형식 지원)"""
        try:
            with open(filepath, 'r') as f:
                data = yaml.safe_load(f)

            if 'camera_matrix' in data and 'data' in data['camera_matrix']:
                cam = data['camera_matrix']['data']
                fx, fy = cam[0], cam[4]
                ppx, ppy = cam[2], cam[5]
            else:
                fx = data['fx']
                fy = data['fy']
                ppx = data['cx']
                ppy = data['cy']

            if 'distortion_coefficients' in data and 'data' in data['distortion_coefficients']:
                coeffs = data['distortion_coefficients']['data']
            else:
                coeffs = [
                    data.get('k1', 0.0),
                    data.get('k2', 0.0),
                    data.get('p1', 0.0),
                    data.get('p2', 0.0),
                    data.get('k3', 0.0)
                ]

            return cls(
                fx=fx, fy=fy, ppx=ppx, ppy=ppy,
                coeffs=coeffs,
                width=data.get('image_width', 1920),
                height=data.get('image_height', 1080)
            )
        except Exception as e:
            print(f"Failed to load calibration: {e}")
            return None

    def to_matrix(self) -> np.ndarray:
        """3x3 카메라 행렬 반환"""
        return np.array([
            [self.fx, 0, self.ppx],
            [0, self.fy, self.ppy],
            [0, 0, 1]
        ], dtype=np.float32)

    def to_dist_coeffs(self) -> np.ndarray:
        """왜곡 계수 배열 반환"""
        return np.array(self.coeffs, dtype=np.float32)


class ArduCamManager(QObject):
    """ArduCam (USB UVC) 카메라 관리 클래스 - CameraManager와 동일한 인터페이스"""

    # Qt Signals (CameraManager와 동일)
    frame_ready = pyqtSignal(np.ndarray)
    camera_started = pyqtSignal()
    camera_stopped = pyqtSignal()
    error_occurred = pyqtSignal(str)

    def __init__(self, device_index: int = 0,
                 color_resolution: Tuple[int, int] = (1920, 1080),
                 depth_resolution: Tuple[int, int] = (640, 480),
                 fps: int = 30,
                 calibration_file: str = None):
        """
        Args:
            device_index: USB 카메라 장치 인덱스 (0, 1, 2, ...)
            color_resolution: Color 카메라 해상도 (width, height)
            depth_resolution: Depth 카메라 해상도 (미사용, 호환성용)
            fps: 프레임 레이트
            calibration_file: 캘리브레이션 YAML 파일 경로
        """
        super().__init__()

        self.device_index = device_index
        self.color_resolution = color_resolution
        self.depth_resolution = depth_resolution  # 호환성용
        self.fps = fps

        # OpenCV VideoCapture
        self._capture: Optional[cv2.VideoCapture] = None
        self._running = False

        # 캘리브레이션 로드
        self._calibration_file = calibration_file or DEFAULT_CALIBRATION_FILE
        self._intrinsics: Optional[Intrinsics] = None
        self._use_calibration_file = False

        if os.path.exists(self._calibration_file):
            self._intrinsics = Intrinsics.from_yaml(self._calibration_file)
            if self._intrinsics:
                self._use_calibration_file = True
                print(f"[ArduCamManager] Calibration loaded: {self._calibration_file}")

        # 프레임 업데이트 타이머
        self._timer: Optional[QTimer] = None
        self._frame_interval = int(1000 / fps)

        # 마지막 프레임
        self._last_frame: Optional[np.ndarray] = None
        self._last_depth_frame: Optional[np.ndarray] = None  # 호환성용 (항상 None)
        self._last_depth_raw: Optional[np.ndarray] = None  # 호환성용 (항상 None)

        # 감마 보정
        self._gamma = 1.0

        # 로그 콜백
        self._log_callback: Optional[Callable[[str], None]] = None

    @property
    def is_available(self) -> bool:
        """카메라 사용 가능 여부 (항상 True)"""
        return True

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

    @staticmethod
    def list_available_cameras(max_index: int = 10) -> list:
        """사용 가능한 USB 카메라 목록"""
        available = []
        for i in range(max_index):
            cap = cv2.VideoCapture(i)
            if cap.isOpened():
                sysfs_name_path = f"/sys/class/video4linux/video{i}/name"
                try:
                    with open(sysfs_name_path) as _f:
                        device_name = _f.read().strip()
                except OSError:
                    device_name = f"USB Camera {i}"
                available.append({
                    'index': i,
                    'name': device_name,
                    'width': int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)),
                    'height': int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
                })
                cap.release()
        return available

    def start(self) -> Tuple[bool, str]:
        """카메라 시작"""
        if self._running:
            return False, "카메라가 이미 실행 중입니다."

        try:
            self._capture = cv2.VideoCapture(self.device_index)

            if not self._capture.isOpened():
                msg = f"ArduCam 장치 {self.device_index}을(를) 열 수 없습니다."
                self.error_occurred.emit(msg)
                return False, msg

            # 해상도 및 FPS 설정
            width, height = self.color_resolution
            self._capture.set(cv2.CAP_PROP_FRAME_WIDTH, width)
            self._capture.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
            self._capture.set(cv2.CAP_PROP_FPS, self.fps)

            # 실제 설정된 값 확인
            actual_width = int(self._capture.get(cv2.CAP_PROP_FRAME_WIDTH))
            actual_height = int(self._capture.get(cv2.CAP_PROP_FRAME_HEIGHT))
            actual_fps = self._capture.get(cv2.CAP_PROP_FPS)

            self._log(f"ArduCam 해상도: {actual_width}x{actual_height}, FPS: {actual_fps}")

            # 캘리브레이션 파일이 없으면 기본값
            if not self._use_calibration_file:
                self._intrinsics = Intrinsics(
                    fx=float(actual_width),
                    fy=float(actual_width),
                    ppx=float(actual_width) / 2,
                    ppy=float(actual_height) / 2,
                    coeffs=[0.0, 0.0, 0.0, 0.0, 0.0],
                    width=actual_width,
                    height=actual_height
                )
                self._log("기본 intrinsics 사용 (캘리브레이션 필요)")

            # 안정화
            for _ in range(10):
                self._capture.read()

            self._running = True

            # 타이머 시작
            self._timer = QTimer()
            self._timer.timeout.connect(self._update_frame)
            self._timer.start(self._frame_interval)

            self._log(f"ArduCam 시작됨 (장치 {self.device_index})")
            self.camera_started.emit()
            return True, "ArduCam 시작됨"

        except Exception as e:
            msg = f"ArduCam 시작 실패: {e}"
            self._log(msg)
            self.error_occurred.emit(msg)
            return False, msg

    def stop(self) -> Tuple[bool, str]:
        """카메라 정지"""
        if not self._running:
            return False, "카메라가 실행 중이 아닙니다."

        try:
            if self._timer:
                self._timer.stop()
                self._timer = None

            if self._capture:
                self._capture.release()
                self._capture = None

            self._running = False
            self._last_frame = None

            self._log("ArduCam 정지됨")
            self.camera_stopped.emit()
            return True, "ArduCam 정지됨"

        except Exception as e:
            msg = f"ArduCam 정지 오류: {e}"
            self._log(msg)
            self.error_occurred.emit(msg)
            return False, msg

    def _update_frame(self):
        """프레임 업데이트"""
        if not self._running or not self._capture:
            return

        try:
            ret, frame = self._capture.read()

            if not ret or frame is None:
                return

            if self._gamma != 1.0:
                frame = self._apply_gamma(frame, self._gamma)

            self._last_frame = frame
            self.frame_ready.emit(frame)

        except Exception as e:
            self._log(f"프레임 업데이트 오류: {e}")

    def get_frame(self) -> Optional[np.ndarray]:
        """현재 프레임 가져오기"""
        if not self._running or not self._capture:
            return None

        try:
            ret, frame = self._capture.read()
            if ret and frame is not None:
                if self._gamma != 1.0:
                    frame = self._apply_gamma(frame, self._gamma)
                return frame
        except Exception as e:
            self._log(f"프레임 가져오기 오류: {e}")

        return None

    def get_depth_frame(self) -> Optional[np.ndarray]:
        """Depth 프레임 (ArduCam은 미지원)"""
        return None

    def get_depth_raw(self) -> Optional[np.ndarray]:
        """Raw Depth 프레임 (ArduCam은 미지원)"""
        return None

    def get_distance_at(self, x: int, y: int, from_color: bool = False) -> Optional[float]:
        """특정 픽셀 거리 (ArduCam은 미지원)"""
        return None

    def get_distance_at_center(self) -> Optional[float]:
        """중심 거리 (ArduCam은 미지원)"""
        return None

    def set_gamma(self, gamma: float):
        """감마 값 설정"""
        self._gamma = max(0.1, min(3.0, gamma))

    def _apply_gamma(self, frame: np.ndarray, gamma: float) -> np.ndarray:
        """감마 보정"""
        inv_gamma = 1.0 / gamma
        table = np.array([((i / 255.0) ** inv_gamma) * 255
                          for i in np.arange(0, 256)]).astype("uint8")
        return cv2.LUT(frame, table)

    def snapshot(self, save_dir: Optional[str] = None) -> Optional[str]:
        """스냅샷 저장"""
        if not self._running:
            self._log("카메라가 실행 중이 아닙니다.")
            return None

        frame = self.get_frame()
        if frame is None:
            self._log("프레임을 가져올 수 없습니다.")
            return None

        try:
            if save_dir is None:
                save_dir = os.path.join(os.path.dirname(__file__), '..', '..', 'snapshots')

            os.makedirs(save_dir, exist_ok=True)

            filename = f"arducam_snapshot_{datetime.now().strftime('%Y%m%d_%H%M%S')}.png"
            filepath = os.path.join(save_dir, filename)

            cv2.imwrite(filepath, frame)
            self._log(f"스냅샷 저장: {filepath}")
            return filepath

        except Exception as e:
            self._log(f"스냅샷 저장 실패: {e}")
            return None

