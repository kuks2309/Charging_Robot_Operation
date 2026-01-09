#!/usr/bin/env python3
"""
PoseService - 포즈 저장/이동 서비스

MainWindow에서 분리된 포즈 관리 로직:
- 현재 위치 저장
- 저장된 포즈로 이동
- 어프로치 이동
"""

from typing import Optional, Callable, List, Tuple
from dataclasses import dataclass

from PyQt5.QtCore import QObject, pyqtSignal

from Robot import PoseManager, RobotController, ModbusClient


@dataclass
class PoseOperationResult:
    """포즈 작업 결과"""
    success: bool
    message: str


class PoseService(QObject):
    """포즈 저장/이동 서비스"""

    # Qt Signals
    pose_saved = pyqtSignal(str)       # 포즈 저장됨 (이름)
    pose_deleted = pyqtSignal(str)     # 포즈 삭제됨 (이름)
    pose_list_changed = pyqtSignal()   # 포즈 목록 변경됨

    def __init__(self, pose_manager: PoseManager):
        """
        Args:
            pose_manager: PoseManager 인스턴스
        """
        super().__init__()

        self.pose_manager = pose_manager
        self._robot: Optional[ModbusClient] = None
        self._robot_controller: Optional[RobotController] = None

        # 로그 콜백
        self._log_callback: Optional[Callable[[str], None]] = None

    def set_log_callback(self, callback: Callable[[str], None]):
        """로그 콜백 설정"""
        self._log_callback = callback

    def set_robot(self, robot: ModbusClient, robot_controller: RobotController = None):
        """로봇 클라이언트 및 컨트롤러 설정"""
        self._robot = robot
        self._robot_controller = robot_controller

    def _log(self, message: str):
        """로그 출력"""
        if self._log_callback:
            self._log_callback(message)

    @property
    def is_robot_connected(self) -> bool:
        """로봇 연결 상태"""
        return self._robot is not None and self._robot.is_connected

    def get_all_pose_names(self) -> List[str]:
        """모든 저장된 포즈 이름 목록"""
        return self.pose_manager.get_all_names()

    def get_pose_info(self, name: str) -> Optional[dict]:
        """
        포즈 정보 가져오기

        Args:
            name: 포즈 이름

        Returns:
            포즈 정보 딕셔너리 또는 None
        """
        saved_pose = self.pose_manager.get_pose(name)
        if saved_pose:
            pose = saved_pose.pose
            return {
                'name': name,
                'pose_type': saved_pose.pose_type,
                'description': saved_pose.description,
                'x': pose.x,
                'y': pose.y,
                'z': pose.z,
                'rx': pose.rx,
                'ry': pose.ry,
                'rz': pose.rz
            }
        return None

    def read_current_pose(self) -> Optional[Tuple[float, float, float, float, float, float]]:
        """
        로봇의 현재 카메라 포즈 읽기

        Returns:
            (x, y, z, rx, ry, rz) 튜플 또는 None
        """
        if not self.is_robot_connected:
            self._log("로봇에 연결되지 않았습니다.")
            return None

        return self._robot.read_camera_pose()

    def save_current_pose(self, name: str, pose_type: str = "custom",
                          description: str = "") -> PoseOperationResult:
        """
        현재 위치 저장

        Args:
            name: 포즈 이름
            pose_type: 포즈 타입
            description: 설명

        Returns:
            PoseOperationResult
        """
        if not self.is_robot_connected:
            return PoseOperationResult(False, "로봇에 연결되지 않았습니다.")

        # 현재 포즈 읽기
        cam_pose = self._robot.read_camera_pose()
        if cam_pose is None:
            return PoseOperationResult(False, "현재 위치를 읽을 수 없습니다.")

        # 설명 자동 생성
        if not description:
            description = f"X:{cam_pose[0]:.2f}, Y:{cam_pose[1]:.2f}, Z:{cam_pose[2]:.2f}"

        # 저장
        success = self.pose_manager.save_pose_from_tuple(
            name, cam_pose, pose_type, description=description
        )

        if success:
            self._log(f"포즈 저장: {name} ({pose_type})")
            self.pose_saved.emit(name)
            self.pose_list_changed.emit()
            return PoseOperationResult(True, f"'{name}' 저장 완료")
        else:
            return PoseOperationResult(False, "포즈 저장에 실패했습니다.")

    def delete_pose(self, name: str) -> PoseOperationResult:
        """
        저장된 포즈 삭제

        Args:
            name: 삭제할 포즈 이름

        Returns:
            PoseOperationResult
        """
        if self.pose_manager.delete_pose(name):
            self._log(f"포즈 삭제: {name}")
            self.pose_deleted.emit(name)
            self.pose_list_changed.emit()
            return PoseOperationResult(True, f"'{name}' 삭제 완료")
        else:
            return PoseOperationResult(False, "포즈 삭제에 실패했습니다.")

    def move_to_pose(self, name: str) -> PoseOperationResult:
        """
        저장된 포즈로 이동

        Args:
            name: 이동할 포즈 이름

        Returns:
            PoseOperationResult
        """
        if not self._robot_controller:
            return PoseOperationResult(False, "로봇에 연결되지 않았습니다.")

        result = self._robot_controller.move_to_saved_pose(name)
        if result.success:
            self._log(f"이동 명령: {result.message}")
        else:
            self._log(f"이동 실패: {result.message}")

        return PoseOperationResult(result.success, result.message)

    def approach_pose(self, name: str, distance: float = 0.2) -> PoseOperationResult:
        """
        저장된 포즈의 어프로치 위치로 이동

        Args:
            name: 포즈 이름
            distance: 어프로치 거리 (미터)

        Returns:
            PoseOperationResult
        """
        if not self._robot_controller:
            return PoseOperationResult(False, "로봇에 연결되지 않았습니다.")

        result = self._robot_controller.approach_pose(name, approach_distance=distance)
        if result.success:
            self._log(f"어프로치 이동: {result.message}")
        else:
            self._log(f"어프로치 실패: {result.message}")

        return PoseOperationResult(result.success, result.message)

    @staticmethod
    def get_pose_types() -> List[str]:
        """사용 가능한 포즈 타입 목록"""
        return ["ar_tag", "home", "charging_gun", "charging_port", "approach", "custom"]
