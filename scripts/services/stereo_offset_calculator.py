#!/usr/bin/env python3
"""
DS435-ArduCam 카메라 간 mm 오프셋 계산기

Sweep 캘리브레이션 JSON 데이터를 분석하여 두 카메라 광축 간의
물리적 오프셋(mm)을 계산한다. 이 오프셋을 활용하면 DS435에서
마커를 검출한 후 ArduCam FOV 중심으로 로봇을 이동시킬 수 있다.
"""

import json
import glob
import os
import logging
from typing import Optional, Tuple

logger = logging.getLogger(__name__)


class StereoOffsetCalculator:
    """Sweep 데이터 기반 DS435-ArduCam mm 오프셋 계산기"""

    # 이미지 중심 (1280x720)
    IMAGE_CENTER_X = 640.0
    IMAGE_CENTER_Y = 360.0

    def __init__(self, sweep_json_path: str):
        """sweep JSON 로드 및 카메라 간 오프셋 계산.

        Args:
            sweep_json_path: sweep 캘리브레이션 JSON 파일 경로
        """
        self._path = sweep_json_path
        self._data = None
        self._camera_offset_y_mm = 0.0
        self._camera_offset_z_mm = 0.0
        self._origin_pose_z = 0.0
        self._ds435_px_per_mm = {'y': 0.0, 'z': 0.0}
        self._arducam_px_per_mm = {'y': 0.0, 'z': 0.0}

        self._load_and_compute(sweep_json_path)

    def _load_and_compute(self, path: str):
        """JSON 로드 및 오프셋 계산"""
        with open(path, 'r') as f:
            self._data = json.load(f)

        results = self._data['results']
        origin_pose = self._data['origin_pose']
        self._origin_pose_z = origin_pose[2]

        # px/mm 비율 추출
        # Y축(가로): 로봇 Y 이동 시 이미지 X방향 변화 → px_per_mm_x from y-sweep
        # Z축(세로): 로봇 Z 이동 시 이미지 Y방향 변화 → px_per_mm_y from z-sweep
        self._ds435_px_per_mm = {
            'y': results['y']['ds435']['px_per_mm_x'],
            'z': results['z']['ds435']['px_per_mm_y'],
        }
        self._arducam_px_per_mm = {
            'y': results['y']['arducam']['px_per_mm_x'],
            'z': results['z']['arducam']['px_per_mm_y'],
        }

        # 원점(robot_mm=0.0)에서의 마커 midpoint 추출
        ds435_mid_x = results['y']['ds435']['mid_x'][0]  # Y-sweep 원점
        ds435_mid_y = results['z']['ds435']['mid_y'][0]   # Z-sweep 원점
        arducam_mid_x = results['y']['arducam']['mid_x'][0]
        arducam_mid_y = results['z']['arducam']['mid_y'][0]

        # 각 카메라에서 마커가 이미지 중심으로부터 떨어진 거리를 mm로 변환
        # "마커를 이미지 중심에 놓으려면 로봇을 얼마나 이동해야 하는가"
        ds_dy_mm = (ds435_mid_x - self.IMAGE_CENTER_X) / self._ds435_px_per_mm['y']
        ds_dz_mm = (ds435_mid_y - self.IMAGE_CENTER_Y) / self._ds435_px_per_mm['z']
        ar_dy_mm = (arducam_mid_x - self.IMAGE_CENTER_X) / self._arducam_px_per_mm['y']
        ar_dz_mm = (arducam_mid_y - self.IMAGE_CENTER_Y) / self._arducam_px_per_mm['z']

        # 카메라 간 오프셋 = ArduCam 광축 오프셋 - DS435 광축 오프셋
        # 의미: DS435 중심에 마커가 있을 때, ArduCam 중심으로 옮기기 위한 추가 이동량
        self._camera_offset_y_mm = ar_dy_mm - ds_dy_mm
        self._camera_offset_z_mm = ar_dz_mm - ds_dz_mm

        logger.info(
            f"[StereoOffset] 로드 완료: {os.path.basename(path)}\n"
            f"  DS435 원점: ({ds435_mid_x:.1f}, {ds435_mid_y:.1f})px, "
            f"ArduCam 원점: ({arducam_mid_x:.1f}, {arducam_mid_y:.1f})px\n"
            f"  DS435 px/mm: Y={self._ds435_px_per_mm['y']:.3f}, Z={self._ds435_px_per_mm['z']:.3f}\n"
            f"  ArduCam px/mm: Y={self._arducam_px_per_mm['y']:.3f}, Z={self._arducam_px_per_mm['z']:.3f}\n"
            f"  카메라 오프셋: dY={self._camera_offset_y_mm:.2f}mm, dZ={self._camera_offset_z_mm:.2f}mm\n"
            f"  sweep 원점 Z: {self._origin_pose_z:.1f}mm"
        )

    @property
    def camera_offset_mm(self) -> Tuple[float, float]:
        """카메라 간 물리적 오프셋 (dy_mm, dz_mm).

        DS435 이미지 중심에 마커가 있을 때 ArduCam 중심으로 옮기기 위한 로봇 이동량.
        """
        return (self._camera_offset_y_mm, self._camera_offset_z_mm)

    @property
    def origin_pose_z(self) -> float:
        """sweep 수행 시의 원점 Z 좌표 (mm)"""
        return self._origin_pose_z

    @property
    def ds435_px_per_mm(self) -> dict:
        """DS435 px/mm 비율 {'y': float, 'z': float}"""
        return self._ds435_px_per_mm.copy()

    @property
    def arducam_px_per_mm(self) -> dict:
        """ArduCam px/mm 비율 {'y': float, 'z': float}"""
        return self._arducam_px_per_mm.copy()

    def validate_current_z(self, current_z_mm: float) -> Optional[str]:
        """현재 Z가 sweep 원점 Z와 30mm 이상 차이 시 경고 반환.

        Returns:
            경고 문자열 또는 None (정상 범위)
        """
        delta = abs(current_z_mm - self._origin_pose_z)
        if delta > 30.0:
            return (
                f"현재 Z={current_z_mm:.1f}mm이 sweep 원점 Z={self._origin_pose_z:.1f}mm과 "
                f"{delta:.1f}mm 차이 (>30mm). 오프셋 정확도 저하 가능."
            )
        return None

    @staticmethod
    def find_latest_sweep(data_dir: str) -> Optional[str]:
        """data/stereo/에서 가장 최근 sweep JSON 파일 경로 반환.

        Args:
            data_dir: 프로젝트 루트의 data 디렉토리 경로

        Returns:
            가장 최근 sweep JSON 경로, 없으면 None
        """
        stereo_dir = os.path.join(data_dir, 'stereo')
        if not os.path.isdir(stereo_dir):
            return None

        pattern = os.path.join(stereo_dir, 'sweep_*.json')
        files = sorted(glob.glob(pattern))
        if not files:
            return None

        return files[-1]  # 파일명에 타임스탬프 포함, 정렬 → 최신
