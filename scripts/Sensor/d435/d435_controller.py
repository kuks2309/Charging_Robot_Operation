#!/usr/bin/env python3
"""
Intel RealSense D435 Camera Controller

D435 카메라 제어 및 ArUco/ChArUco 마커 감지를 위한 클래스
"""

import os
import yaml
import numpy as np
from dataclasses import dataclass
from typing import Optional, Tuple, Dict, Any

try:
    import pyrealsense2 as rs
    REALSENSE_AVAILABLE = True
except ImportError:
    REALSENSE_AVAILABLE = False
    print("Warning: pyrealsense2 not installed. D435 camera will not be available.")


@dataclass
class CameraIntrinsics:
    """카메라 내부 파라미터"""
    fx: float  # focal length x
    fy: float  # focal length y
    ppx: float  # principal point x (cx)
    ppy: float  # principal point y (cy)
    coeffs: list  # distortion coefficients [k1, k2, p1, p2, k3]
    width: int = 640
    height: int = 480

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
            width=data.get('image_width', 640),
            height=data.get('image_height', 480)
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


class D435Controller:
    """Intel RealSense D435 카메라 컨트롤러"""

    # 기본 캘리브레이션 파일 경로
    DEFAULT_CALIBRATION = os.path.join(
        os.path.dirname(__file__), '..', '..', '..', 'config', 'calibration', 'ds435', 'ds435_calibration.yaml'
    )

    def __init__(self, calibration_file: str = None):
        """
        D435 컨트롤러 초기화

        Args:
            calibration_file: 캘리브레이션 YAML 파일 경로 (None이면 기본값 사용)
        """
        if not REALSENSE_AVAILABLE:
            raise RuntimeError("pyrealsense2 library is not installed")

        self.pipeline: Optional[rs.pipeline] = None
        self.config: Optional[rs.config] = None
        self.is_running = False

        # 캘리브레이션 로드
        calib_path = calibration_file or self.DEFAULT_CALIBRATION
        if os.path.exists(calib_path):
            self.intrinsics = CameraIntrinsics.from_yaml(calib_path)
            print(f"[D435] Calibration loaded: {calib_path}")
        else:
            # 기본값 사용
            self.intrinsics = CameraIntrinsics(
                fx=563.62, fy=566.41,
                ppx=330.62, ppy=259.12,
                coeffs=[0.0886, -0.0985, 0.0, 0.0, 0.0],
                width=640, height=480
            )
            print(f"[D435] Using default calibration (file not found: {calib_path})")

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
            self.pipeline = rs.pipeline()
            self.config = rs.config()

            # 컬러 스트림 설정
            self.config.enable_stream(
                rs.stream.color,
                self.width, self.height,
                rs.format.bgr8,
                self.fps
            )

            # 파이프라인 시작
            profile = self.pipeline.start(self.config)

            # 카메라 안정화 (처음 몇 프레임 버리기)
            for _ in range(30):
                self.pipeline.wait_for_frames()

            self.is_running = True

            # 디바이스 정보
            device = profile.get_device()
            device_name = device.get_info(rs.camera_info.name)
            serial = device.get_info(rs.camera_info.serial_number)

            return True, f"D435 connected: {device_name} (S/N: {serial})"

        except Exception as e:
            self.is_running = False
            return False, f"Failed to start D435: {str(e)}"

    def stop(self) -> Tuple[bool, str]:
        """
        카메라 중지

        Returns:
            (success, message) 튜플
        """
        if not self.is_running:
            return True, "Camera not running"

        try:
            if self.pipeline:
                self.pipeline.stop()
            self.is_running = False
            return True, "D435 stopped"
        except Exception as e:
            return False, f"Failed to stop D435: {str(e)}"

    def get_frame(self) -> Optional[np.ndarray]:
        """
        현재 프레임 가져오기

        Returns:
            BGR 이미지 (numpy array) 또는 None
        """
        if not self.is_running:
            return None

        try:
            frames = self.pipeline.wait_for_frames(timeout_ms=1000)
            color_frame = frames.get_color_frame()

            if not color_frame:
                return None

            return np.asanyarray(color_frame.get_data())

        except Exception as e:
            print(f"[D435] Frame capture error: {e}")
            return None

    def get_intrinsics(self) -> CameraIntrinsics:
        """카메라 내부 파라미터 반환"""
        return self.intrinsics

    def get_runtime_intrinsics(self) -> Optional[CameraIntrinsics]:
        """
        RealSense SDK에서 런타임 내부 파라미터 가져오기
        (캘리브레이션 파일 대신 카메라에서 직접 읽음)
        """
        if not self.is_running:
            return None

        try:
            profile = self.pipeline.get_active_profile()
            color_profile = profile.get_stream(rs.stream.color)
            intr = color_profile.as_video_stream_profile().get_intrinsics()

            return CameraIntrinsics(
                fx=intr.fx,
                fy=intr.fy,
                ppx=intr.ppx,
                ppy=intr.ppy,
                coeffs=list(intr.coeffs),
                width=intr.width,
                height=intr.height
            )
        except Exception as e:
            print(f"[D435] Failed to get runtime intrinsics: {e}")
            return None

    def is_connected(self) -> bool:
        """카메라 연결 상태"""
        return self.is_running

    @staticmethod
    def list_devices() -> list:
        """연결된 RealSense 디바이스 목록"""
        if not REALSENSE_AVAILABLE:
            return []

        ctx = rs.context()
        devices = []

        for dev in ctx.query_devices():
            devices.append({
                'name': dev.get_info(rs.camera_info.name),
                'serial': dev.get_info(rs.camera_info.serial_number),
                'firmware': dev.get_info(rs.camera_info.firmware_version)
            })

        return devices

    def __del__(self):
        """소멸자"""
        if self.is_running:
            self.stop()


# 테스트 코드
if __name__ == "__main__":
    import cv2

    print("D435 Camera Test")
    print("=" * 40)

    # 연결된 디바이스 확인
    devices = D435Controller.list_devices()
    print(f"Found {len(devices)} RealSense device(s)")
    for d in devices:
        print(f"  - {d['name']} (S/N: {d['serial']})")

    if not devices:
        print("No devices found!")
        exit(1)

    # 컨트롤러 생성 및 시작
    controller = D435Controller()
    success, msg = controller.start()
    print(f"\nStart: {msg}")

    if success:
        print(f"Resolution: {controller.width}x{controller.height}")
        print(f"Intrinsics: fx={controller.intrinsics.fx:.2f}, fy={controller.intrinsics.fy:.2f}")

        print("\nPress 'q' to quit")

        while True:
            frame = controller.get_frame()
            if frame is not None:
                cv2.imshow("D435", frame)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        controller.stop()
        cv2.destroyAllWindows()
