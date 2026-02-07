#!/usr/bin/env python3
"""
VisionManager - Aruco 태그 감지 서비스

MainWindow에서 분리된 Vision 관련 로직:
- Aruco 마커 감지
- 태그 추적
- 평균 측정
"""

import time
from typing import Optional, Dict, List, Callable
import numpy as np
import cv2

from PyQt5.QtCore import QObject, pyqtSignal
from PyQt5.QtWidgets import QApplication

from Sensor import ArucoCameraPoseEstimator
from .camera_manager import CameraManager


class VisionManager(QObject):
    """Aruco 태그 감지 관리 클래스"""

    # Qt Signals
    marker_detected = pyqtSignal(list)  # 마커 감지됨
    tag_found = pyqtSignal(int, dict)   # 특정 태그 발견 (tag_id, marker_info)

    def __init__(self, camera_manager: CameraManager,
                 marker_size_meters: float = 0.03,
                 dictionary_type: int = cv2.aruco.DICT_5X5_50):
        """
        Args:
            camera_manager: CameraManager 인스턴스
            marker_size_meters: 마커 크기 (미터)
            dictionary_type: Aruco 사전 타입
        """
        super().__init__()

        self.camera_manager = camera_manager
        self.marker_size = marker_size_meters

        # Aruco 검출기 초기화
        self.aruco_detector = ArucoCameraPoseEstimator(
            marker_size_meters=marker_size_meters,
            dictionary_type=dictionary_type
        )

        # 마지막 검출 결과
        self._last_result: Optional[List[Dict]] = None

        # 로그 콜백
        self._log_callback: Optional[Callable[[str], None]] = None

    @property
    def last_result(self) -> Optional[List[Dict]]:
        """마지막 감지 결과"""
        return self._last_result

    def set_log_callback(self, callback: Callable[[str], None]):
        """로그 콜백 설정"""
        self._log_callback = callback

    def set_camera_manager(self, camera_manager):
        """카메라 매니저 변경"""
        self.camera_manager = camera_manager

    def _log(self, message: str):
        """로그 출력"""
        if self._log_callback:
            self._log_callback(message)

    def detect_markers(self, frame: np.ndarray) -> tuple:
        """
        프레임에서 Aruco 마커 감지

        Args:
            frame: BGR 이미지

        Returns:
            (시각화된 프레임, 마커 리스트)
        """
        intrinsics = self.camera_manager.intrinsics
        if intrinsics is None:
            return frame, None

        # Aruco 감지
        marker_poses = self.aruco_detector.detect_and_estimate_pose(
            frame, intrinsics
        )

        if marker_poses:
            self._last_result = marker_poses
            self.marker_detected.emit(marker_poses)

            # 시각화
            vis_frame = self.aruco_detector.visualize_markers(
                frame, intrinsics, marker_poses
            )
            return vis_frame, marker_poses
        else:
            self._last_result = None
            return frame, None

    def detect_markers_disambiguated(
        self,
        frame: np.ndarray,
        target_ids: tuple,
        known_distance_m: float
    ) -> tuple:
        """
        Detect dual markers with pose disambiguation using known inter-marker distance.

        Args:
            frame: BGR image frame
            target_ids: Tuple of two marker IDs (e.g., (0, 1))
            known_distance_m: Known distance between markers in meters

        Returns:
            Tuple of (vis_frame, marker1_dict, marker2_dict, measured_distance, distance_error)
            marker_dict contains: {'id', 'tvec', 'rvec', 'corners'} or None if not detected
        """
        if self.aruco_detector is None or self.camera_manager.intrinsics is None:
            return (frame, None, None, 0, float('inf'))

        # Debug logging: method called
        print(f"[VisionManager] Disambiguation called for IDs {target_ids}, known_distance={known_distance_m*1000:.2f}mm")

        # Call the disambiguation method
        result = self.aruco_detector.detect_and_estimate_pose_dual_disambiguated(
            frame,
            self.camera_manager.intrinsics,
            target_ids,
            known_distance_m
        )

        marker1_pose, marker2_pose, measured_distance, distance_error, combo_idx = result

        if marker1_pose is None or marker2_pose is None:
            print("[VisionManager] Disambiguation result: fail, markers not detected")
            return (frame, None, None, 0, float('inf'))

        # Debug logging: result summary
        print(f"[VisionManager] Disambiguation result: success, error={distance_error*1000:.2f}mm")

        # Create visualization
        vis_frame = self.aruco_detector.visualize_markers(
            frame,
            self.camera_manager.intrinsics,
            [marker1_pose, marker2_pose],
            show_info=True
        )

        return (vis_frame, marker1_pose, marker2_pose, measured_distance, distance_error)

    def detect_tag(self, tag_id: int, timeout: float = 10.0,
                   num_samples: int = 10) -> Optional[Dict]:
        """
        특정 Aruco 태그 감지 (n번 측정 평균)

        Args:
            tag_id: 찾을 태그 ID
            timeout: 타임아웃 (초)
            num_samples: 평균을 낼 샘플 수

        Returns:
            태그 정보 (평균값) 또는 None
        """
        if not self.camera_manager.is_running:
            self._log("카메라가 실행 중이 아닙니다.")
            return None

        start_time = time.time()
        samples = []

        self._log(f"Aruco Tag {tag_id} 감지 중... ({num_samples}회 측정)")

        while time.time() - start_time < timeout:
            if self._last_result:
                for marker in self._last_result:
                    if marker['id'] == tag_id:
                        # 샘플 수집
                        samples.append({
                            'tvec': marker['tvec'].copy(),
                            'rvec': marker['rvec'].copy(),
                            'camera_rotation': marker['camera_rotation'].copy(),
                            'camera_position': marker['camera_position'].copy(),
                        })

                        if len(samples) >= num_samples:
                            # 평균 계산
                            avg_marker = self._calculate_average_marker(marker, samples)
                            self._log(f"Aruco Tag {tag_id} 감지 완료 ({len(samples)}회 평균)")
                            self.tag_found.emit(tag_id, avg_marker)
                            return avg_marker

            # UI 이벤트 처리
            QApplication.processEvents()
            time.sleep(0.05)  # 50ms 간격으로 샘플링

        # 타임아웃 시 수집된 샘플이 있으면 평균 반환
        if len(samples) > 0:
            self._log(f"Aruco Tag {tag_id} 부분 감지 ({len(samples)}회 평균)")
            if self._last_result:
                for marker in self._last_result:
                    if marker['id'] == tag_id:
                        avg_marker = self._calculate_average_marker(marker, samples)
                        self.tag_found.emit(tag_id, avg_marker)
                        return avg_marker

        self._log(f"Aruco Tag {tag_id} 감지 실패 (타임아웃)")
        return None

    def _calculate_average_marker(self, base_marker: Dict, samples: List[Dict]) -> Dict:
        """
        여러 샘플의 평균 마커 정보 계산

        Args:
            base_marker: 기본 마커 정보 (id, corners 등 포함)
            samples: tvec, rvec 등이 담긴 샘플 리스트

        Returns:
            평균화된 마커 정보
        """
        n = len(samples)

        # tvec 평균
        avg_tvec = np.mean([s['tvec'] for s in samples], axis=0)

        # rvec 평균
        avg_rvec = np.mean([s['rvec'] for s in samples], axis=0)

        # camera_rotation 평균
        avg_camera_rotation = np.mean([s['camera_rotation'] for s in samples], axis=0)

        # camera_position 평균
        avg_camera_position = np.mean([s['camera_position'] for s in samples], axis=0)

        # 표준편차 계산 (정밀도 확인용)
        std_tvec = np.std([s['tvec'] for s in samples], axis=0)
        self._log(f"  tvec 표준편차: X={std_tvec[0]*1000:.3f}mm, Y={std_tvec[1]*1000:.3f}mm, Z={std_tvec[2]*1000:.3f}mm")

        # 평균 마커 생성
        avg_marker = base_marker.copy()
        avg_marker['tvec'] = avg_tvec
        avg_marker['rvec'] = avg_rvec
        avg_marker['camera_rotation'] = avg_camera_rotation
        avg_marker['camera_position'] = avg_camera_position
        avg_marker['num_samples'] = n
        avg_marker['std_tvec'] = std_tvec

        return avg_marker

    def get_marker_by_id(self, tag_id: int) -> Optional[Dict]:
        """
        마지막 결과에서 특정 ID의 마커 가져오기

        Args:
            tag_id: 태그 ID

        Returns:
            마커 정보 또는 None
        """
        if self._last_result:
            for marker in self._last_result:
                if marker['id'] == tag_id:
                    return marker
        return None

    def update_marker_size(self, size_meters: float):
        """마커 크기 업데이트"""
        self.marker_size = size_meters
        self.aruco_detector = ArucoCameraPoseEstimator(
            marker_size_meters=size_meters,
            dictionary_type=cv2.aruco.DICT_5X5_50
        )
