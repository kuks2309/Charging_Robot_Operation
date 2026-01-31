#!/usr/bin/env python3
"""
DualArucoDetector - 2개의 ArUco 마커를 동시 검출하여 평면 정의

충전 포트 정렬을 위한 Dual ArUco 마커 검출 서비스:
- 2개 ArUco 마커 동시 검출
- 평면 법선 벡터 계산
- Outlier 필터링 적용
"""

from dataclasses import dataclass
from typing import Optional, Tuple, List, Dict, Callable
import numpy as np
import cv2

from PyQt5.QtCore import QObject, pyqtSignal

from .vision_manager import VisionManager


@dataclass
class ArucoResult:
    """ArUco 마커 검출 결과"""
    id: int
    detected: bool
    tvec: Optional[np.ndarray] = None  # Vision 좌표계 (TF1/TF5)
    rvec: Optional[np.ndarray] = None
    corners: Optional[np.ndarray] = None

    def to_dict(self) -> Dict:
        """딕셔너리로 변환"""
        return {
            'id': self.id,
            'detected': self.detected,
            'tvec': self.tvec,
            'rvec': self.rvec,
            'corners': self.corners,
        }


@dataclass
class PlaneResult:
    """평면 계산 결과"""
    valid: bool
    center: Optional[np.ndarray] = None      # 두 마커 중심점 (Vision 좌표계)
    normal: Optional[np.ndarray] = None      # 평면 법선 벡터
    horizontal: Optional[np.ndarray] = None  # 수평 벡터 (마커1 → 마커2)
    marker1: Optional[ArucoResult] = None
    marker2: Optional[ArucoResult] = None
    distance_mm: float = 0.0                 # 두 마커 간 거리 (mm)


@dataclass
class PlanePose:
    """평면 자세 (위치 + 방향) - 기존 Pose 규약 준수"""
    x: float      # mm (NOT meters)
    y: float      # mm
    z: float      # mm
    rx: float     # degrees (rotation around X)
    ry: float     # degrees (rotation around Y)
    rz: float     # degrees (rotation around Z)

    # 메타 정보
    valid_samples: int = 0
    total_attempts: int = 0
    std_position_mm: Optional[Tuple[float, float, float]] = None  # (std_x, std_y, std_z)
    std_rotation_deg: Optional[Tuple[float, float, float]] = None  # (std_rx, std_ry, std_rz)

    def to_dict(self) -> Dict:
        return {
            'x': self.x, 'y': self.y, 'z': self.z,
            'rx': self.rx, 'ry': self.ry, 'rz': self.rz,
            'valid_samples': self.valid_samples,
            'total_attempts': self.total_attempts,
        }

    @classmethod
    def from_dict(cls, d: Dict) -> 'PlanePose':
        return cls(**{k: d[k] for k in ['x', 'y', 'z', 'rx', 'ry', 'rz', 'valid_samples', 'total_attempts']})


class DualArucoDetector(QObject):
    """
    Dual ArUco 마커 검출기

    2개의 ArUco 마커를 동시에 검출하고 평면을 정의합니다.
    충전 포트 정렬에 사용됩니다.
    """

    # Signals
    both_detected = pyqtSignal(object)  # PlaneResult
    detection_failed = pyqtSignal(str)  # 에러 메시지

    def __init__(self,
                 vision_manager: VisionManager,
                 marker_id1: int = 0,
                 marker_id2: int = 1,
                 known_distance_m: float = 0.118):
        """
        Args:
            vision_manager: VisionManager 인스턴스
            marker_id1: 첫 번째 마커 ID (기본: 0)
            marker_id2: 두 번째 마커 ID (기본: 1)
            known_distance_m: 두 마커 간 알려진 거리 (meter, 기본: 0.118m = 118mm)
        """
        super().__init__()

        self.vision_manager = vision_manager
        self.marker_id1 = marker_id1
        self.marker_id2 = marker_id2
        self.known_distance_m = known_distance_m
        self.use_disambiguation = True  # 기본으로 disambiguation 사용

        # Outlier 필터링을 위한 샘플 버퍼
        self._samples: List[PlaneResult] = []
        self._max_samples = 50

        # 마지막 검출 결과
        self._last_result: Optional[PlaneResult] = None

        # 로그 콜백
        self._log_callback: Optional[Callable[[str], None]] = None

    @property
    def marker_ids(self) -> Tuple[int, int]:
        """마커 ID 튜플 반환"""
        return (self.marker_id1, self.marker_id2)

    @marker_ids.setter
    def marker_ids(self, ids: Tuple[int, int]):
        """마커 ID 설정"""
        self.marker_id1, self.marker_id2 = ids
        self._samples.clear()  # 샘플 초기화

    @property
    def last_result(self) -> Optional[PlaneResult]:
        """마지막 검출 결과"""
        return self._last_result

    def set_log_callback(self, callback: Callable[[str], None]):
        """로그 콜백 설정"""
        self._log_callback = callback

    def _log(self, message: str):
        """로그 출력"""
        if self._log_callback:
            self._log_callback(message)

    def detect(self, frame: np.ndarray) -> PlaneResult:
        """
        프레임에서 두 개의 ArUco 마커 검출

        Args:
            frame: BGR 이미지

        Returns:
            PlaneResult: 평면 계산 결과
        """
        marker1_data = None
        marker2_data = None

        # Disambiguation 사용 여부에 따라 검출 방식 선택
        if self.use_disambiguation:
            # Disambiguation 방식: 두 마커 간 알려진 거리로 자세 모호성 해결
            try:
                result = self.vision_manager.detect_markers_disambiguated(
                    frame,
                    target_ids=(self.marker_id1, self.marker_id2),
                    known_distance_m=self.known_distance_m
                )
                if result is not None:
                    markers, distance_error = result
                    if markers:
                        for marker in markers:
                            if marker['id'] == self.marker_id1:
                                marker1_data = self._extract_aruco_result(marker, self.marker_id1)
                            elif marker['id'] == self.marker_id2:
                                marker2_data = self._extract_aruco_result(marker, self.marker_id2)
            except Exception as e:
                self._log(f"Disambiguation 실패, 일반 검출로 fallback: {e}")
                self.use_disambiguation = False

        # Fallback: 일반 검출
        if marker1_data is None or marker2_data is None:
            _, markers = self.vision_manager.detect_markers(frame)
            if markers:
                for marker in markers:
                    if marker['id'] == self.marker_id1 and marker1_data is None:
                        marker1_data = self._extract_aruco_result(marker, self.marker_id1)
                    elif marker['id'] == self.marker_id2 and marker2_data is None:
                        marker2_data = self._extract_aruco_result(marker, self.marker_id2)

        # 마커1 미검출
        if marker1_data is None:
            marker1_data = ArucoResult(id=self.marker_id1, detected=False)

        # 마커2 미검출
        if marker2_data is None:
            marker2_data = ArucoResult(id=self.marker_id2, detected=False)

        # 두 마커 모두 검출된 경우 평면 계산
        if marker1_data.detected and marker2_data.detected:
            result = self._calculate_plane(marker1_data, marker2_data)
            self._last_result = result

            # 샘플 버퍼에 추가
            self._add_sample(result)

            self.both_detected.emit(result)
            return result
        else:
            # 검출 실패
            result = PlaneResult(
                valid=False,
                marker1=marker1_data,
                marker2=marker2_data,
            )
            self._last_result = result

            missing = []
            if not marker1_data.detected:
                missing.append(f"ID{self.marker_id1}")
            if not marker2_data.detected:
                missing.append(f"ID{self.marker_id2}")

            self.detection_failed.emit(f"마커 미검출: {', '.join(missing)}")
            return result

    def _extract_aruco_result(self, marker: Dict, marker_id: int) -> ArucoResult:
        """
        마커 딕셔너리에서 ArucoResult 추출

        좌표계 변환: Camera → Vision (TF1)
        - tvec: X=X, Y=-Y, Z=-Z (위치 변환)
        - rvec: 원본 유지 (Rodrigues 벡터는 회전 행렬 추출에 사용)
        """
        tvec_cam = marker.get('tvec', None)
        rvec_cam = marker.get('rvec', None)
        corners = marker.get('corners', None)

        if tvec_cam is None or rvec_cam is None:
            return ArucoResult(id=marker_id, detected=False)

        # tvec: Camera → Vision 좌표 변환
        tvec_vision = self._camera_to_vision(tvec_cam)

        # rvec: 원본 유지 (Rodrigues 벡터, 회전 행렬 추출에 사용)
        rvec = np.array(rvec_cam).flatten()

        return ArucoResult(
            id=marker_id,
            detected=True,
            tvec=tvec_vision,
            rvec=rvec,
            corners=corners,
        )

    def _camera_to_vision(self, vec: np.ndarray) -> np.ndarray:
        """
        Camera 좌표계 → Vision 좌표계 변환

        변환: X=X, Y=-Y, Z=-Z
        """
        vec = np.array(vec).flatten()
        return np.array([vec[0], -vec[1], -vec[2]])

    def _calculate_plane(self, marker1: ArucoResult, marker2: ArucoResult) -> PlaneResult:
        """
        두 마커로부터 평면 계산

        평면 정의:
        - 중심점: (P1 + P2) / 2
        - 수평 벡터: normalize(P2 - P1)
        - 법선 벡터: 마커 Z축 평균 (마커 → 카메라 방향의 반대 = 포트 표면 바깥)
        """
        p1 = marker1.tvec
        p2 = marker2.tvec

        # 중심점 계산
        center = (p1 + p2) / 2.0

        # 수평 벡터 (마커1 → 마커2 방향)
        horizontal = p2 - p1
        distance_mm = np.linalg.norm(horizontal) * 1000.0  # m → mm
        horizontal = horizontal / np.linalg.norm(horizontal)  # 정규화

        # 마커 rvec에서 법선 벡터 추출 (마커 Z축 = 마커에서 카메라 방향)
        # 충전 포트 법선 = 마커 Z축의 반대 방향 (포트 표면에서 바깥으로)
        normal1 = self._get_marker_z_axis(marker1.rvec)
        normal2 = self._get_marker_z_axis(marker2.rvec)

        # 두 마커 법선 평균 (더 안정적)
        normal = (normal1 + normal2) / 2.0
        normal = normal / np.linalg.norm(normal)  # 정규화

        # 포트 표면 바깥 방향 = 마커 Z축의 반대
        normal = -normal

        return PlaneResult(
            valid=True,
            center=center,
            normal=normal,
            horizontal=horizontal,
            marker1=marker1,
            marker2=marker2,
            distance_mm=distance_mm,
        )

    def _get_marker_z_axis(self, rvec: np.ndarray) -> np.ndarray:
        """
        마커 rvec에서 Z축(법선) 방향 추출

        마커 Z축: 마커 평면에서 카메라 방향
        """
        # Rodrigues 변환: rvec → 회전 행렬
        R, _ = cv2.Rodrigues(rvec)
        # Z축은 회전 행렬의 세 번째 열
        z_axis = R[:, 2]
        return z_axis

    def _add_sample(self, result: PlaneResult):
        """샘플 버퍼에 추가"""
        if not result.valid:
            return

        self._samples.append(result)

        # 최대 샘플 수 초과 시 오래된 샘플 제거
        if len(self._samples) > self._max_samples:
            self._samples.pop(0)

    def get_filtered_result(self, num_samples: int = 10) -> Optional[PlaneResult]:
        """
        Outlier 필터링된 평균 결과 반환

        IQR 방식으로 outlier 제거 후 평균 계산

        Args:
            num_samples: 사용할 최근 샘플 수

        Returns:
            필터링된 평균 PlaneResult 또는 None
        """
        if len(self._samples) < 3:
            self._log(f"샘플 부족: {len(self._samples)}/3")
            return self._last_result

        # 최근 n개 샘플 사용
        samples = self._samples[-min(num_samples, len(self._samples)):]

        # 중심점 Z 값으로 outlier 탐지 (가장 민감)
        z_values = np.array([s.center[2] for s in samples])

        # IQR 방식
        q1 = np.percentile(z_values, 25)
        q3 = np.percentile(z_values, 75)
        iqr = q3 - q1
        lower_bound = q1 - 1.5 * iqr
        upper_bound = q3 + 1.5 * iqr

        # 유효한 샘플 필터링
        valid_samples = [
            s for s, z in zip(samples, z_values)
            if lower_bound <= z <= upper_bound
        ]

        num_outliers = len(samples) - len(valid_samples)
        if num_outliers > 0:
            self._log(f"Outlier 제거: {num_outliers}개")

        if len(valid_samples) == 0:
            self._log("유효한 샘플 없음")
            return self._last_result

        # 평균 계산
        avg_center = np.mean([s.center for s in valid_samples], axis=0)
        avg_normal = np.mean([s.normal for s in valid_samples], axis=0)
        avg_normal = avg_normal / np.linalg.norm(avg_normal)  # 재정규화
        avg_horizontal = np.mean([s.horizontal for s in valid_samples], axis=0)
        avg_horizontal = avg_horizontal / np.linalg.norm(avg_horizontal)
        avg_distance = np.mean([s.distance_mm for s in valid_samples])

        # 표준편차 계산 (정밀도 확인)
        std_center = np.std([s.center for s in valid_samples], axis=0) * 1000.0  # mm
        self._log(f"필터링 후 중심점 std: X={std_center[0]:.3f}mm, Y={std_center[1]:.3f}mm, Z={std_center[2]:.3f}mm")

        return PlaneResult(
            valid=True,
            center=avg_center,
            normal=avg_normal,
            horizontal=avg_horizontal,
            marker1=valid_samples[-1].marker1,
            marker2=valid_samples[-1].marker2,
            distance_mm=avg_distance,
        )

    def get_plane_normal(self) -> Optional[np.ndarray]:
        """평면 법선 벡터 반환"""
        if self._last_result and self._last_result.valid:
            return self._last_result.normal
        return None

    def get_plane_center(self) -> Optional[np.ndarray]:
        """두 마커 중심점 반환"""
        if self._last_result and self._last_result.valid:
            return self._last_result.center
        return None

    def clear_samples(self):
        """샘플 버퍼 초기화"""
        self._samples.clear()
        self._last_result = None

    def get_sample_count(self) -> int:
        """현재 샘플 수 반환"""
        return len(self._samples)
