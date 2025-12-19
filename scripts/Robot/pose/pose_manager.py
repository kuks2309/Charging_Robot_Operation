#!/usr/bin/env python3
"""
Pose Manager - 로봇 위치 저장/로드/관리
"""

import os
import json
from typing import Optional, Dict, List, Tuple
from dataclasses import dataclass, asdict
from datetime import datetime

from .constants import PoseType


@dataclass
class Pose:
    """로봇 포즈 데이터"""
    x: float        # mm (meter 단위로 저장)
    y: float        # mm
    z: float        # mm
    rx: float       # deg
    ry: float       # deg
    rz: float       # deg

    def to_tuple(self) -> Tuple[float, float, float, float, float, float]:
        """튜플로 변환"""
        return (self.x, self.y, self.z, self.rx, self.ry, self.rz)

    def to_dict(self) -> dict:
        """딕셔너리로 변환"""
        return asdict(self)

    @classmethod
    def from_tuple(cls, t: Tuple[float, float, float, float, float, float]) -> 'Pose':
        """튜플에서 생성"""
        return cls(x=t[0], y=t[1], z=t[2], rx=t[3], ry=t[4], rz=t[5])

    @classmethod
    def from_dict(cls, d: dict) -> 'Pose':
        """딕셔너리에서 생성"""
        return cls(**d)


@dataclass
class SavedPose:
    """저장된 포즈 (메타데이터 포함)"""
    name: str
    pose: Pose
    pose_type: str
    description: str = ""
    created_at: str = ""

    def to_dict(self) -> dict:
        """딕셔너리로 변환"""
        return {
            'name': self.name,
            'pose': self.pose.to_dict(),
            'pose_type': self.pose_type,
            'description': self.description,
            'created_at': self.created_at
        }

    @classmethod
    def from_dict(cls, d: dict) -> 'SavedPose':
        """딕셔너리에서 생성"""
        return cls(
            name=d['name'],
            pose=Pose.from_dict(d['pose']),
            pose_type=d.get('pose_type', PoseType.CUSTOM),
            description=d.get('description', ''),
            created_at=d.get('created_at', '')
        )


class PoseManager:
    """위치 저장/로드 관리자"""

    DEFAULT_FILE = "saved_poses.json"

    def __init__(self, save_dir: Optional[str] = None):
        """
        Args:
            save_dir: 저장 디렉토리 (기본: scripts/robot/data)
        """
        if save_dir is None:
            save_dir = os.path.join(os.path.dirname(__file__), 'data')

        self.save_dir = save_dir
        self._poses: Dict[str, SavedPose] = {}

        # 디렉토리 생성
        os.makedirs(self.save_dir, exist_ok=True)

        # 기존 데이터 로드
        self.load()

    @property
    def poses(self) -> Dict[str, SavedPose]:
        """저장된 포즈 목록"""
        return self._poses.copy()

    def save_pose(self, name: str, pose: Pose, pose_type: str = PoseType.CUSTOM,
                  description: str = "") -> bool:
        """
        포즈 저장

        Args:
            name: 포즈 이름 (고유 키)
            pose: Pose 객체
            pose_type: 포즈 타입 (PoseType 상수)
            description: 설명

        Returns:
            성공 여부
        """
        saved_pose = SavedPose(
            name=name,
            pose=pose,
            pose_type=pose_type,
            description=description,
            created_at=datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        )

        self._poses[name] = saved_pose
        return self._save_to_file()

    def save_pose_from_tuple(self, name: str,
                              pose_tuple: Tuple[float, float, float, float, float, float],
                              pose_type: str = PoseType.CUSTOM,
                              description: str = "") -> bool:
        """
        튜플에서 포즈 저장

        Args:
            name: 포즈 이름
            pose_tuple: (x, y, z, rx, ry, rz) - meter/deg 단위
            pose_type: 포즈 타입
            description: 설명
        """
        pose = Pose.from_tuple(pose_tuple)
        return self.save_pose(name, pose, pose_type, description)

    def get_pose(self, name: str) -> Optional[SavedPose]:
        """
        포즈 가져오기

        Args:
            name: 포즈 이름

        Returns:
            SavedPose 또는 None
        """
        return self._poses.get(name)

    def get_pose_tuple(self, name: str) -> Optional[Tuple[float, float, float, float, float, float]]:
        """
        포즈 튜플로 가져오기

        Args:
            name: 포즈 이름

        Returns:
            (x, y, z, rx, ry, rz) 또는 None
        """
        saved_pose = self.get_pose(name)
        if saved_pose:
            return saved_pose.pose.to_tuple()
        return None

    def delete_pose(self, name: str) -> bool:
        """
        포즈 삭제

        Args:
            name: 포즈 이름

        Returns:
            성공 여부
        """
        if name in self._poses:
            del self._poses[name]
            return self._save_to_file()
        return False

    def rename_pose(self, old_name: str, new_name: str) -> bool:
        """
        포즈 이름 변경

        Args:
            old_name: 기존 이름
            new_name: 새 이름

        Returns:
            성공 여부
        """
        if old_name not in self._poses or new_name in self._poses:
            return False

        saved_pose = self._poses[old_name]
        saved_pose.name = new_name
        self._poses[new_name] = saved_pose
        del self._poses[old_name]

        return self._save_to_file()

    def get_poses_by_type(self, pose_type: str) -> List[SavedPose]:
        """
        타입별 포즈 목록

        Args:
            pose_type: 포즈 타입

        Returns:
            SavedPose 리스트
        """
        return [p for p in self._poses.values() if p.pose_type == pose_type]

    def get_all_names(self) -> List[str]:
        """모든 포즈 이름 목록"""
        return list(self._poses.keys())

    def clear_all(self) -> bool:
        """모든 포즈 삭제"""
        self._poses.clear()
        return self._save_to_file()

    # ==================== 파일 I/O ====================

    def _get_file_path(self) -> str:
        """저장 파일 경로"""
        return os.path.join(self.save_dir, self.DEFAULT_FILE)

    def _save_to_file(self) -> bool:
        """파일에 저장"""
        try:
            data = {name: sp.to_dict() for name, sp in self._poses.items()}
            with open(self._get_file_path(), 'w', encoding='utf-8') as f:
                json.dump(data, f, indent=2, ensure_ascii=False)
            return True
        except Exception as e:
            print(f"Failed to save poses: {e}")
            return False

    def load(self) -> bool:
        """파일에서 로드"""
        file_path = self._get_file_path()
        if not os.path.exists(file_path):
            return True  # 파일 없으면 빈 상태로 시작

        try:
            with open(file_path, 'r', encoding='utf-8') as f:
                data = json.load(f)

            self._poses.clear()
            for name, pose_dict in data.items():
                self._poses[name] = SavedPose.from_dict(pose_dict)

            return True
        except Exception as e:
            print(f"Failed to load poses: {e}")
            return False

    def export_to_file(self, file_path: str) -> bool:
        """다른 파일로 내보내기"""
        try:
            data = {name: sp.to_dict() for name, sp in self._poses.items()}
            with open(file_path, 'w', encoding='utf-8') as f:
                json.dump(data, f, indent=2, ensure_ascii=False)
            return True
        except Exception as e:
            print(f"Failed to export poses: {e}")
            return False

    def import_from_file(self, file_path: str, merge: bool = True) -> bool:
        """
        파일에서 가져오기

        Args:
            file_path: 파일 경로
            merge: True면 병합, False면 덮어쓰기
        """
        try:
            with open(file_path, 'r', encoding='utf-8') as f:
                data = json.load(f)

            if not merge:
                self._poses.clear()

            for name, pose_dict in data.items():
                self._poses[name] = SavedPose.from_dict(pose_dict)

            return self._save_to_file()
        except Exception as e:
            print(f"Failed to import poses: {e}")
            return False
