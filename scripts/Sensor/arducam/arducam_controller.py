#!/usr/bin/env python3
"""
ArduCam USB (UVC) Camera Controller

USB UVC 방식 ArduCam 카메라 제어를 위한 클래스
D435Controller와 동일한 인터페이스 제공
"""

import os
import yaml
import numpy as np
import cv2
from dataclasses import dataclass
from typing import Optional, Tuple, List


@dataclass
class CameraIntrinsics:
    """카메라 내부 파라미터"""
    fx: float  # focal length x
    fy: float  # focal length y
    ppx: float  # principal point x (cx)
    ppy: float  # principal point y (cy)
    coeffs: list  # distortion coefficients [k1, k2, p1, p2, k3]
    width: int = 1920
    height: int = 1080

    @classmethod
    def from_yaml(cls, filepath: str) -> 'CameraIntrinsics':
        """YAML 캘리브레이션 파일에서 로드"""
        with open(filepath, 'r') as f:
            data = yaml.safe_load(f)

        return cls(
            fx=data['fx'],
            fy=data['fy'],
            ppx=data['cx'],
            ppy=data['cy'],
            coeffs=[
                data.get('k1', 0.0),
                data.get('k2', 0.0),
                data.get('p1', 0.0),
                data.get('p2', 0.0),
                data.get('k3', 0.0)
            ],
            width=data.get('image_width', 1920),
            height=data.get('image_height', 1080)
        )

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


class ArduCamController:
    """ArduCam USB (UVC) 카메라 컨트롤러"""

    # 기본 캘리브레이션 파일 경로
    DEFAULT_CALIBRATION = os.path.join(
        os.path.dirname(__file__), '..', '..', '..', 'config', 'arducam_calibration.yaml'
    )

    def __init__(self, device_index: int = 0, calibration_file: str = None):
        """
        ArduCam 컨트롤러 초기화

        Args:
            device_index: USB 카메라 장치 인덱스 (0, 1, 2, ...)
            calibration_file: 캘리브레이션 YAML 파일 경로 (None이면 기본값 사용)
        """
        self.device_index = device_index
        self.capture: Optional[cv2.VideoCapture] = None
        self.is_running = False

        # 캘리브레이션 로드
        calib_path = calibration_file or self.DEFAULT_CALIBRATION
        if os.path.exists(calib_path):
            self.intrinsics = CameraIntrinsics.from_yaml(calib_path)
            self._use_calibration_file = True
            print(f"[ArduCam] Calibration loaded: {calib_path}")
        else:
            # 기본값 사용 (이상적인 핀홀 카메라)
            self.intrinsics = CameraIntrinsics(
                fx=1920.0, fy=1920.0,
                ppx=960.0, ppy=540.0,
                coeffs=[0.0, 0.0, 0.0, 0.0, 0.0],
                width=1920, height=1080
            )
            self._use_calibration_file = False
            print(f"[ArduCam] Using default calibration (file not found: {calib_path})")

        # 설정
        self.width = self.intrinsics.width
        self.height = self.intrinsics.height
        self.fps = 30

    def start(self) -> Tuple[bool, str]:
        """
        카메라 시작

        Returns:
            (success, message) 튜플
        """
        if self.is_running:
            return True, "Camera already running"

        try:
            self.capture = cv2.VideoCapture(self.device_index)

            if not self.capture.isOpened():
                return False, f"Failed to open camera device {self.device_index}"

            # 해상도 및 FPS 설정
            self.capture.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
            self.capture.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
            self.capture.set(cv2.CAP_PROP_FPS, self.fps)

            # 실제 설정된 값 확인
            actual_width = int(self.capture.get(cv2.CAP_PROP_FRAME_WIDTH))
            actual_height = int(self.capture.get(cv2.CAP_PROP_FRAME_HEIGHT))
            actual_fps = self.capture.get(cv2.CAP_PROP_FPS)

            # 실제 해상도로 업데이트
            self.width = actual_width
            self.height = actual_height

            # 캘리브레이션 파일을 사용하지 않는 경우 intrinsics 업데이트
            if not self._use_calibration_file:
                self.intrinsics = CameraIntrinsics(
                    fx=float(actual_width),
                    fy=float(actual_width),
                    ppx=float(actual_width) / 2,
                    ppy=float(actual_height) / 2,
                    coeffs=[0.0, 0.0, 0.0, 0.0, 0.0],
                    width=actual_width,
                    height=actual_height
                )

            # 카메라 안정화 (처음 몇 프레임 버리기)
            for _ in range(10):
                self.capture.read()

            self.is_running = True

            return True, f"ArduCam connected: Device {self.device_index} ({actual_width}x{actual_height} @ {actual_fps}fps)"

        except Exception as e:
            self.is_running = False
            return False, f"Failed to start ArduCam: {str(e)}"

    def stop(self) -> Tuple[bool, str]:
        """
        카메라 중지

        Returns:
            (success, message) 튜플
        """
        if not self.is_running:
            return True, "Camera not running"

        try:
            if self.capture:
                self.capture.release()
                self.capture = None
            self.is_running = False
            return True, "ArduCam stopped"
        except Exception as e:
            return False, f"Failed to stop ArduCam: {str(e)}"

    def get_frame(self) -> Optional[np.ndarray]:
        """
        현재 프레임 가져오기

        Returns:
            BGR 이미지 (numpy array) 또는 None
        """
        if not self.is_running or not self.capture:
            return None

        try:
            ret, frame = self.capture.read()

            if not ret or frame is None:
                return None

            return frame

        except Exception as e:
            print(f"[ArduCam] Frame capture error: {e}")
            return None

    def get_intrinsics(self) -> CameraIntrinsics:
        """카메라 내부 파라미터 반환"""
        return self.intrinsics

    def is_connected(self) -> bool:
        """카메라 연결 상태"""
        return self.is_running

    @staticmethod
    def list_devices(max_index: int = 10) -> List[dict]:
        """
        사용 가능한 USB 카메라 목록

        Args:
            max_index: 검사할 최대 인덱스

        Returns:
            사용 가능한 카메라 정보 목록
        """
        devices = []

        for i in range(max_index):
            cap = cv2.VideoCapture(i)
            if cap.isOpened():
                # 카메라 정보 가져오기
                width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
                height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
                fps = cap.get(cv2.CAP_PROP_FPS)

                # 백엔드 정보
                backend = cap.getBackendName()

                devices.append({
                    'index': i,
                    'name': f"USB Camera {i}",
                    'resolution': f"{width}x{height}",
                    'fps': fps,
                    'backend': backend
                })
                cap.release()

        return devices



# 테스트 코드
if __name__ == "__main__":
    print("ArduCam Camera Test")
    print("=" * 40)

    # 연결된 디바이스 확인
    devices = ArduCamController.list_devices()
    print(f"Found {len(devices)} USB camera(s)")
    for d in devices:
        print(f"  - {d['name']} ({d['resolution']} @ {d['fps']}fps) [{d['backend']}]")

    if not devices:
        print("No devices found!")
        exit(1)

    # 컨트롤러 생성 및 시작
    controller = ArduCamController(device_index=0)
    success, msg = controller.start()
    print(f"\nStart: {msg}")

    if success:
        print(f"Resolution: {controller.width}x{controller.height}")
        print(f"Intrinsics: fx={controller.intrinsics.fx:.2f}, fy={controller.intrinsics.fy:.2f}")

        print("\nPress 'q' to quit")

        while True:
            frame = controller.get_frame()
            if frame is not None:
                cv2.imshow("ArduCam", frame)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        controller.stop()
        cv2.destroyAllWindows()
