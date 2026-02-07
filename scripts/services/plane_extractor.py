"""
PlaneExtractor - 듀얼 ArUco 마커 기반 평면 추출
반복 샘플링 + outlier 재시도 + Euler 변환
"""
from typing import Optional, Callable, Tuple, List
import numpy as np

from .dual_aruco_detector import DualArucoDetector, PlaneResult, PlanePose
from .plane_utils import normal_horizontal_to_euler, ensure_normal_positive_z


class PlaneExtractor:
    """
    듀얼 ArUco 마커 기반 평면 추출기

    목표 샘플 수 도달까지 자동 재시도하며,
    outlier 제거 후 평면 자세(x,y,z,rx,ry,rz)를 반환

    단위: 위치 mm, 각도 degrees
    """

    def __init__(self,
                 dual_detector: DualArucoDetector,
                 target_samples: int = 20,
                 max_attempts: int = 50,
                 min_valid_ratio: float = 0.6):
        """
        Args:
            dual_detector: DualArucoDetector 인스턴스
            target_samples: 목표 유효 샘플 수
            max_attempts: 최대 시도 횟수
            min_valid_ratio: 최소 유효 샘플 비율 (outlier 제거 후)
        """
        self.dual_detector = dual_detector
        self.target_samples = target_samples
        self.max_attempts = max_attempts
        self.min_valid_ratio = min_valid_ratio

        self._collected_samples: List[PlaneResult] = []
        self._total_attempts = 0

    def collect_samples(self,
                        frame_source: Callable[[], np.ndarray],
                        on_progress: Optional[Callable[[int, int], None]] = None
                       ) -> Tuple[int, int]:
        """
        목표 샘플 수까지 샘플 수집

        Args:
            frame_source: 프레임 획득 함수 (예: lambda: camera_manager.get_frame())
            on_progress: 진행 콜백 (current_valid, target)

        Returns:
            (valid_samples, total_attempts)
        """
        self._collected_samples.clear()
        self.dual_detector.clear_samples()
        self._total_attempts = 0

        while self._total_attempts < self.max_attempts:
            frame = frame_source()
            if frame is None:
                self._total_attempts += 1
                continue

            result = self.dual_detector.detect(frame)
            self._total_attempts += 1

            if result.valid:
                self._collected_samples.append(result)

            if on_progress:
                on_progress(len(self._collected_samples), self.target_samples)

            # 목표 도달 확인
            if len(self._collected_samples) >= self.target_samples:
                break

        return (len(self._collected_samples), self._total_attempts)

    def get_plane_pose(self) -> Optional[PlanePose]:
        """
        필터링된 평면 자세 반환

        Returns:
            PlanePose(x,y,z,rx,ry,rz) 또는 None (단위: mm, degrees)
        """
        filtered = self.dual_detector.get_filtered_result(
            num_samples=len(self._collected_samples)
        )

        if filtered is None or not filtered.valid:
            return None

        # Normal 방향 보정 (+Z 방향으로)
        normal = ensure_normal_positive_z(filtered.normal)

        # Euler 각도 계산
        try:
            rx, ry, rz = normal_horizontal_to_euler(normal, filtered.horizontal)
        except ValueError as e:
            print(f"[ERROR] Euler 변환 실패: {e}")
            return None

        # 위치: meters → mm 변환
        center_mm = filtered.center * 1000.0

        return PlanePose(
            x=center_mm[0],
            y=center_mm[1],
            z=center_mm[2],
            rx=rx,
            ry=ry,
            rz=rz,
            valid_samples=len(self._collected_samples),
            total_attempts=self._total_attempts,
        )

    def reset(self):
        """샘플 버퍼 초기화"""
        self._collected_samples.clear()
        self.dual_detector.clear_samples()
        self._total_attempts = 0


def extract_plane_and_correction(
    dual_detector: DualArucoDetector,
    frame_source: Callable[[], np.ndarray],
    target_samples: int = 20,
    max_attempts: int = 50,
    target_orientation: Tuple[float, float, float] = (0.0, 0.0, 0.0),
    current_tcp: Optional[Tuple[float, float, float, float, float, float]] = None,
    on_progress: Optional[Callable[[int, int], None]] = None
) -> Tuple[Optional[PlanePose], Optional['TCPCorrection']]:
    """
    원스톱 평면 추출 및 TCP 보정 계산

    Args:
        dual_detector: DualArucoDetector 인스턴스
        frame_source: 프레임 획득 함수
        target_samples: 목표 샘플 수
        max_attempts: 최대 시도 횟수
        target_orientation: 목표 (rx, ry, rz) degrees
        current_tcp: 현재 TCP (x, y, z, rx, ry, rz) in mm, degrees. 제공 시 correction.final_rx/ry/rz에 최종 절대 자세 포함
        on_progress: 진행 콜백

    Returns:
        (PlanePose, TCPCorrection) 또는 (None, None)

    Example:
        from scripts.services.dual_aruco_detector import DualArucoDetector
        from scripts.services.vision_manager import VisionManager

        vm = VisionManager(...)
        detector = DualArucoDetector(vm, marker_id1=0, marker_id2=1)

        pose, correction = extract_plane_and_correction(
            dual_detector=detector,
            frame_source=lambda: vm.camera_manager.get_frame(),
            target_samples=20,
        )

        if pose:
            print(f"평면 위치: ({pose.x:.1f}, {pose.y:.1f}, {pose.z:.1f}) mm")
            print(f"평면 자세: ({pose.rx:.1f}, {pose.ry:.1f}, {pose.rz:.1f}) deg")
            print(f"TCP 보정: ({correction.delta_rx:.1f}, {correction.delta_ry:.1f}, {correction.delta_rz:.1f}) deg")
    """
    from .tcp_corrector import TCPCorrector, TCPCorrection

    extractor = PlaneExtractor(
        dual_detector=dual_detector,
        target_samples=target_samples,
        max_attempts=max_attempts,
    )

    valid, attempts = extractor.collect_samples(
        frame_source=frame_source,
        on_progress=on_progress,
    )

    plane_pose = extractor.get_plane_pose()

    if plane_pose is None:
        return (None, None)

    corrector = TCPCorrector(
        target_rx=target_orientation[0],
        target_ry=target_orientation[1],
        target_rz=target_orientation[2],
    )

    correction = corrector.compute_correction(plane_pose, current_tcp=current_tcp)

    return (plane_pose, correction)
