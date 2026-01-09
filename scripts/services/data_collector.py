#!/usr/bin/env python3
"""
DataCollector - Aruco 태그 데이터 수집 서비스

MainWindow에서 분리된 데이터 수집 로직:
- 특정 태그의 위치/자세 데이터 수집
- 통계 계산 (평균, 표준편차)
- CSV 저장
"""

import time
import csv
from datetime import datetime
from typing import Optional, Callable, List, Dict
from dataclasses import dataclass, field

import numpy as np
from PyQt5.QtCore import QObject, pyqtSignal

from .vision_manager import VisionManager


@dataclass
class CollectStatistics:
    """수집 통계"""
    count: int = 0
    tvec_mean: np.ndarray = field(default_factory=lambda: np.zeros(3))
    tvec_std: np.ndarray = field(default_factory=lambda: np.zeros(3))
    euler_mean: np.ndarray = field(default_factory=lambda: np.zeros(3))
    euler_std: np.ndarray = field(default_factory=lambda: np.zeros(3))


class DataCollector(QObject):
    """Aruco 태그 데이터 수집 서비스"""

    # Qt Signals
    sample_collected = pyqtSignal(int, int)  # (현재 수집 수, 목표 수)
    collection_completed = pyqtSignal(int)    # 최종 수집 수
    statistics_ready = pyqtSignal(object)     # CollectStatistics

    def __init__(self, vision_manager: VisionManager):
        """
        Args:
            vision_manager: VisionManager 인스턴스
        """
        super().__init__()

        self.vision_manager = vision_manager

        # 수집 상태
        self._collecting = False
        self._target_tag_id = 0
        self._target_count = 100
        self._collected_data: List[Dict] = []

        # 로그 콜백
        self._log_callback: Optional[Callable[[str], None]] = None

    @property
    def is_collecting(self) -> bool:
        """수집 중 여부"""
        return self._collecting

    @property
    def collected_count(self) -> int:
        """수집된 샘플 수"""
        return len(self._collected_data)

    @property
    def target_count(self) -> int:
        """목표 수집 수"""
        return self._target_count

    @property
    def collected_data(self) -> List[Dict]:
        """수집된 데이터"""
        return self._collected_data

    def set_log_callback(self, callback: Callable[[str], None]):
        """로그 콜백 설정"""
        self._log_callback = callback

    def _log(self, message: str):
        """로그 출력"""
        if self._log_callback:
            self._log_callback(message)

    def start(self, tag_id: int, target_count: int = 100) -> bool:
        """
        데이터 수집 시작

        Args:
            tag_id: 수집할 태그 ID
            target_count: 목표 수집 수

        Returns:
            시작 성공 여부
        """
        if not self.vision_manager.camera_manager.is_running:
            self._log("카메라가 실행 중이 아닙니다.")
            return False

        if self._collecting:
            self._log("이미 수집 중입니다.")
            return False

        self._target_tag_id = tag_id
        self._target_count = target_count
        self._collected_data = []
        self._collecting = True

        self._log(f"데이터 수집 시작: Tag ID={tag_id}, 목표={target_count}회")
        return True

    def stop(self) -> int:
        """
        데이터 수집 중지

        Returns:
            수집된 샘플 수
        """
        self._collecting = False
        count = len(self._collected_data)

        self._log(f"데이터 수집 중지: {count}개 수집됨")

        if count > 0:
            stats = self.get_statistics()
            self.statistics_ready.emit(stats)

        self.collection_completed.emit(count)
        return count

    def collect_sample(self) -> bool:
        """
        현재 프레임에서 샘플 수집
        (프레임 업데이트 시 호출)

        Returns:
            샘플 수집 성공 여부
        """
        if not self._collecting or not self.vision_manager.last_result:
            return False

        # 타겟 태그 찾기
        for marker in self.vision_manager.last_result:
            if marker['id'] == self._target_tag_id:
                sample = {
                    'timestamp': time.time(),
                    'tag_id': marker['id'],
                    'tvec_x': marker['tvec'][0],
                    'tvec_y': marker['tvec'][1],
                    'tvec_z': marker['tvec'][2],
                    'rvec_x': marker['rvec'][0],
                    'rvec_y': marker['rvec'][1],
                    'rvec_z': marker['rvec'][2],
                }

                # 회전 행렬에서 오일러 각도 계산
                euler = self.vision_manager.aruco_detector._rotation_matrix_to_euler(
                    marker['camera_rotation']
                )
                sample['euler_rx'] = euler[0]
                sample['euler_ry'] = euler[1]
                sample['euler_rz'] = euler[2]

                self._collected_data.append(sample)

                # 시그널 발생
                count = len(self._collected_data)
                self.sample_collected.emit(count, self._target_count)

                # 목표 도달 시 자동 중지
                if count >= self._target_count:
                    self.stop()

                return True

        return False

    def get_statistics(self) -> Optional[CollectStatistics]:
        """
        수집된 데이터의 통계 계산

        Returns:
            CollectStatistics 또는 None
        """
        if len(self._collected_data) == 0:
            return None

        # numpy 배열로 변환
        tvec_x = np.array([d['tvec_x'] for d in self._collected_data])
        tvec_y = np.array([d['tvec_y'] for d in self._collected_data])
        tvec_z = np.array([d['tvec_z'] for d in self._collected_data])
        euler_rx = np.array([d['euler_rx'] for d in self._collected_data])
        euler_ry = np.array([d['euler_ry'] for d in self._collected_data])
        euler_rz = np.array([d['euler_rz'] for d in self._collected_data])

        stats = CollectStatistics(
            count=len(self._collected_data),
            tvec_mean=np.array([tvec_x.mean(), tvec_y.mean(), tvec_z.mean()]),
            tvec_std=np.array([tvec_x.std(), tvec_y.std(), tvec_z.std()]),
            euler_mean=np.array([euler_rx.mean(), euler_ry.mean(), euler_rz.mean()]),
            euler_std=np.array([euler_rx.std(), euler_ry.std(), euler_rz.std()])
        )

        return stats

    def print_statistics(self) -> str:
        """
        통계 문자열 생성

        Returns:
            통계 문자열
        """
        stats = self.get_statistics()
        if stats is None:
            return "수집된 데이터가 없습니다."

        lines = [
            "=" * 50,
            f"수집 통계 (n={stats.count})",
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
            "=" * 50
        ]

        return "\n".join(lines)

    def save_to_csv(self, filepath: str) -> bool:
        """
        수집된 데이터를 CSV로 저장

        Args:
            filepath: 저장 경로

        Returns:
            저장 성공 여부
        """
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
        """기본 파일명 생성"""
        return f"aruco_data_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"

    def clear(self):
        """수집된 데이터 초기화"""
        self._collected_data = []
        self._collecting = False
        self._log("데이터 초기화됨")
