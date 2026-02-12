#!/usr/bin/env python3
"""
DataCollector - Aruco 태그 데이터 수집 서비스

MainWindow에서 분리된 데이터 수집 로직:
- 2개 태그의 위치/자세 데이터 동시 수집
- 통계 계산 (평균, 표준편차)
- CSV 저장
"""

import time
import csv
from datetime import datetime
from typing import Optional, Callable, List, Dict, Tuple

from dataclasses import dataclass, field

import numpy as np
from PyQt5.QtCore import QObject, pyqtSignal

from .vision_manager import VisionManager


@dataclass
class CollectStatistics:
    """수집 통계 (태그 1개분)"""
    tag_id: int = 0
    count: int = 0
    tvec_mean: np.ndarray = field(default_factory=lambda: np.zeros(3))
    tvec_std: np.ndarray = field(default_factory=lambda: np.zeros(3))
    euler_mean: np.ndarray = field(default_factory=lambda: np.zeros(3))
    euler_std: np.ndarray = field(default_factory=lambda: np.zeros(3))


class DataCollector(QObject):
    """Aruco 태그 데이터 수집 서비스 (2개 태그 동시 수집)"""

    # Qt Signals
    sample_collected = pyqtSignal(int, int)  # (현재 수집 수, 목표 수)
    collection_completed = pyqtSignal(int)    # 최종 수집 수
    statistics_ready = pyqtSignal(object)     # list of CollectStatistics

    def __init__(self, vision_manager: VisionManager):
        super().__init__()

        self.vision_manager = vision_manager

        # 수집 상태
        self._collecting = False
        self._target_tag_ids: Tuple[int, int] = (0, 1)
        self._target_count = 100
        self._collected_data: List[Dict] = []

        # 로그 콜백
        self._log_callback: Optional[Callable[[str], None]] = None

    @property
    def is_collecting(self) -> bool:
        return self._collecting

    @property
    def collected_count(self) -> int:
        return len(self._collected_data)

    @property
    def target_count(self) -> int:
        return self._target_count

    @property
    def collected_data(self) -> List[Dict]:
        return self._collected_data

    def set_log_callback(self, callback: Callable[[str], None]):
        self._log_callback = callback

    def _log(self, message: str):
        if self._log_callback:
            self._log_callback(message)

    def start(self, tag_id_1: int, tag_id_2: int, target_count: int = 100) -> bool:
        """
        데이터 수집 시작 (2개 태그 동시)

        Args:
            tag_id_1: 첫 번째 태그 ID
            tag_id_2: 두 번째 태그 ID
            target_count: 목표 수집 수
        """
        if not self.vision_manager.camera_manager.is_running:
            self._log("카메라가 실행 중이 아닙니다.")
            return False

        if self._collecting:
            self._log("이미 수집 중입니다.")
            return False

        self._target_tag_ids = (tag_id_1, tag_id_2)
        self._target_count = target_count
        self._collected_data = []
        self._collecting = True

        self._log(f"데이터 수집 시작: Tag ID={tag_id_1}, {tag_id_2}, 목표={target_count}회")
        return True

    def stop(self) -> int:
        self._collecting = False
        count = len(self._collected_data)

        self._log(f"데이터 수집 중지: {count}개 수집됨")

        if count > 0:
            stats = self.get_statistics()
            self.statistics_ready.emit(stats)

        self.collection_completed.emit(count)
        return count

    def _extract_marker_data(self, marker, prefix: str) -> Dict:
        """마커에서 데이터 추출 (prefix로 키 구분)"""
        data = {
            f'{prefix}_tag_id': marker['id'],
            f'{prefix}_tvec_x': marker['tvec'][0],
            f'{prefix}_tvec_y': marker['tvec'][1],
            f'{prefix}_tvec_z': marker['tvec'][2],
            f'{prefix}_rvec_x': marker['rvec'][0],
            f'{prefix}_rvec_y': marker['rvec'][1],
            f'{prefix}_rvec_z': marker['rvec'][2],
        }

        euler = self.vision_manager.aruco_detector._rotation_matrix_to_euler(
            marker['camera_rotation']
        )
        data[f'{prefix}_euler_rx'] = euler[0]
        data[f'{prefix}_euler_ry'] = euler[1]
        data[f'{prefix}_euler_rz'] = euler[2]

        return data

    def collect_sample(self) -> bool:
        """
        현재 프레임에서 샘플 수집
        두 태그가 모두 검출된 경우에만 저장
        """
        if not self._collecting or not self.vision_manager.last_result:
            return False

        # 두 태그 모두 찾기
        marker_1 = None
        marker_2 = None
        for marker in self.vision_manager.last_result:
            if marker['id'] == self._target_tag_ids[0]:
                marker_1 = marker
            elif marker['id'] == self._target_tag_ids[1]:
                marker_2 = marker

        if marker_1 is None or marker_2 is None:
            return False

        sample = {'timestamp': time.time()}
        sample.update(self._extract_marker_data(marker_1, 'm1'))
        sample.update(self._extract_marker_data(marker_2, 'm2'))

        self._collected_data.append(sample)

        count = len(self._collected_data)
        self.sample_collected.emit(count, self._target_count)

        if count >= self._target_count:
            self.stop()

        return True

    def get_statistics(self, tag_prefix: str = None) -> Optional[List[CollectStatistics]]:
        """
        수집된 데이터의 통계 계산

        Returns:
            [m1 통계, m2 통계] 리스트 또는 None
        """
        if len(self._collected_data) == 0:
            return None

        result = []
        for prefix, tag_id in [('m1', self._target_tag_ids[0]),
                                ('m2', self._target_tag_ids[1])]:
            tvec_x = np.array([d[f'{prefix}_tvec_x'] for d in self._collected_data])
            tvec_y = np.array([d[f'{prefix}_tvec_y'] for d in self._collected_data])
            tvec_z = np.array([d[f'{prefix}_tvec_z'] for d in self._collected_data])
            euler_rx = np.array([d[f'{prefix}_euler_rx'] for d in self._collected_data])
            euler_ry = np.array([d[f'{prefix}_euler_ry'] for d in self._collected_data])
            euler_rz = np.array([d[f'{prefix}_euler_rz'] for d in self._collected_data])

            stats = CollectStatistics(
                tag_id=tag_id,
                count=len(self._collected_data),
                tvec_mean=np.array([tvec_x.mean(), tvec_y.mean(), tvec_z.mean()]),
                tvec_std=np.array([tvec_x.std(), tvec_y.std(), tvec_z.std()]),
                euler_mean=np.array([euler_rx.mean(), euler_ry.mean(), euler_rz.mean()]),
                euler_std=np.array([euler_rx.std(), euler_ry.std(), euler_rz.std()])
            )
            result.append(stats)

        return result

    def print_statistics(self) -> str:
        stats_list = self.get_statistics()
        if stats_list is None:
            return "수집된 데이터가 없습니다."

        lines = []
        for stats in stats_list:
            lines += [
                "=" * 50,
                f"Tag ID={stats.tag_id} 수집 통계 (n={stats.count})",
                "-" * 50,
                "위치 (mm):",
                f"  X: 평균={stats.tvec_mean[0]*1000:.3f}, 표준편차={stats.tvec_std[0]*1000:.3f}",
                f"  Y: 평균={stats.tvec_mean[1]*1000:.3f}, 표준편차={stats.tvec_std[1]*1000:.3f}",
                f"  Z: 평균={stats.tvec_mean[2]*1000:.3f}, 표준편차={stats.tvec_std[2]*1000:.3f}",
                "-" * 50,
                "회전 (deg):",
                f"  Rx: 평균={stats.euler_mean[0]:.3f}, 표준편차={stats.euler_std[0]:.3f}",
                f"  Ry: 평균={stats.euler_mean[1]:.3f}, 표준편차={stats.euler_std[1]:.3f}",
                f"  Rz: 평균={stats.euler_mean[2]:.3f}, 표준편차={stats.euler_std[2]:.3f}",
                "=" * 50,
                ""
            ]

        return "\n".join(lines)

    def save_to_csv(self, filepath: str) -> bool:
        if len(self._collected_data) == 0:
            self._log("저장할 데이터가 없습니다.")
            return False

        try:
            with open(filepath, 'w', newline='') as f:
                writer = csv.DictWriter(f, fieldnames=self._collected_data[0].keys())
                writer.writeheader()
                writer.writerows(self._collected_data)

            self._log(f"데이터 저장 완료: {filepath}")
            return True

        except Exception as e:
            self._log(f"데이터 저장 실패: {e}")
            return False

    def get_default_filename(self) -> str:
        return f"aruco_data_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"

    def clear(self):
        self._collected_data = []
        self._collecting = False
        self._log("데이터 초기화됨")
