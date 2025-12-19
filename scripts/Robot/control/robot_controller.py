#!/usr/bin/env python3
"""
Robot Controller - 로봇 이동 및 제어 명령
"""

import time
from typing import Optional, Tuple, List, Callable
from dataclasses import dataclass

from ..communication import ModbusClient
from ..pose import PoseManager, Pose, PoseType, Register, Command, Response


@dataclass
class MoveResult:
    """이동 결과"""
    success: bool
    message: str
    final_pose: Optional[Tuple[float, float, float, float, float, float]] = None


class RobotController:
    """로봇 제어 클래스"""

    def __init__(self, modbus_client: ModbusClient, pose_manager: Optional[PoseManager] = None):
        """
        Args:
            modbus_client: ModbusClient 인스턴스
            pose_manager: PoseManager 인스턴스 (없으면 새로 생성)
        """
        self.modbus = modbus_client
        self.pose_manager = pose_manager or PoseManager()

        # 콜백
        self._on_move_start: Optional[Callable] = None
        self._on_move_complete: Optional[Callable] = None
        self._on_error: Optional[Callable] = None

    @property
    def is_connected(self) -> bool:
        """연결 상태"""
        return self.modbus.is_connected

    # ==================== 콜백 설정 ====================

    def set_on_move_start(self, callback: Callable):
        """이동 시작 콜백"""
        self._on_move_start = callback

    def set_on_move_complete(self, callback: Callable):
        """이동 완료 콜백"""
        self._on_move_complete = callback

    def set_on_error(self, callback: Callable):
        """에러 콜백"""
        self._on_error = callback

    # ==================== 위치 읽기 ====================

    def get_current_camera_pose(self) -> Optional[Tuple[float, float, float, float, float, float]]:
        """
        현재 카메라 포즈 읽기 (레지스터 158~169)

        Returns:
            (x, y, z, rx, ry, rz) mm/deg 또는 None
        """
        return self.modbus.read_camera_pose()

    def get_current_pose_main(self) -> Optional[Tuple[float, float, float, float, float, float]]:
        """
        현재 pose_main 읽기 (레지스터 301~306)

        Returns:
            (x, y, z, rx, ry, rz) 또는 None
        """
        return self.modbus.read_pose_main()

    def get_current_pose_back(self) -> Optional[Tuple[float, float, float, float, float, float]]:
        """
        현재 pose_back 읽기 (레지스터 307~312)

        Returns:
            (x, y, z, rx, ry, rz) 또는 None
        """
        return self.modbus.read_pose_back()

    # ==================== 위치 저장 ====================

    def save_current_pose(self, name: str, pose_type: str = PoseType.CUSTOM,
                          description: str = "") -> bool:
        """
        현재 카메라 포즈 저장

        Args:
            name: 포즈 이름
            pose_type: 포즈 타입
            description: 설명

        Returns:
            성공 여부
        """
        pose = self.get_current_camera_pose()
        if pose is None:
            if self._on_error:
                self._on_error("현재 위치를 읽을 수 없습니다")
            return False

        return self.pose_manager.save_pose_from_tuple(name, pose, pose_type, description)

    def save_ar_tag_pose(self, name: str, pose_tuple: Tuple[float, float, float, float, float, float],
                         description: str = "") -> bool:
        """
        AR tag 위치 저장

        Args:
            name: 포즈 이름
            pose_tuple: (x, y, z, rx, ry, rz)
            description: 설명

        Returns:
            성공 여부
        """
        return self.pose_manager.save_pose_from_tuple(
            name, pose_tuple, PoseType.AR_TAG, description
        )

    # ==================== 이동 명령 ====================

    def move_to_pose(self, x: float, y: float, z: float,
                     rx: float, ry: float, rz: float,
                     use_back_pose: bool = True) -> MoveResult:
        """
        지정 위치로 이동

        Args:
            x, y, z: 위치 (meter)
            rx, ry, rz: 회전 (degree)
            use_back_pose: True=pose_back 사용, False=pose_main 사용

        Returns:
            MoveResult
        """
        if not self.is_connected:
            return MoveResult(False, "로봇에 연결되지 않았습니다")

        if self._on_move_start:
            self._on_move_start(x, y, z, rx, ry, rz)

        # 위치 데이터 쓰기
        if use_back_pose:
            success = self.modbus.write_pose_main(x, y, z, rx, ry, rz)
        else:
            success = self.modbus.write_pose_back(x, y, z, rx, ry, rz)

        if not success:
            msg = "위치 데이터 쓰기 실패"
            if self._on_error:
                self._on_error(msg)
            return MoveResult(False, msg)

        # 응답 보내기 (로봇이 이동 시작)
        response = Response.USE_POSE_BACK if use_back_pose else Response.USE_POSE_MAIN
        if not self.modbus.write_response(response):
            msg = "응답 쓰기 실패"
            if self._on_error:
                self._on_error(msg)
            return MoveResult(False, msg)

        if self._on_move_complete:
            self._on_move_complete(x, y, z, rx, ry, rz)

        return MoveResult(
            True,
            f"이동 명령 전송: ({x:.3f}, {y:.3f}, {z:.3f}, {rx:.1f}, {ry:.1f}, {rz:.1f})",
            (x, y, z, rx, ry, rz)
        )

    def move_to_pose_tuple(self, pose_tuple: Tuple[float, float, float, float, float, float],
                           use_back_pose: bool = True) -> MoveResult:
        """튜플로 이동"""
        return self.move_to_pose(*pose_tuple, use_back_pose=use_back_pose)

    def move_to_saved_pose(self, name: str, use_back_pose: bool = True) -> MoveResult:
        """
        저장된 위치로 이동

        Args:
            name: 저장된 포즈 이름
            use_back_pose: pose_back 사용 여부

        Returns:
            MoveResult
        """
        pose_tuple = self.pose_manager.get_pose_tuple(name)
        if pose_tuple is None:
            msg = f"저장된 위치 '{name}'을(를) 찾을 수 없습니다"
            if self._on_error:
                self._on_error(msg)
            return MoveResult(False, msg)

        result = self.move_to_pose_tuple(pose_tuple, use_back_pose)
        if result.success:
            result.message = f"'{name}' 위치로 이동: {result.message}"

        return result

    def move_to_ar_tag(self, name: str, offset_z: float = 0.0) -> MoveResult:
        """
        AR tag 위치로 이동 (Z 오프셋 적용 가능)

        Args:
            name: AR tag 포즈 이름
            offset_z: Z 방향 오프셋 (meter, 양수=위로)

        Returns:
            MoveResult
        """
        saved_pose = self.pose_manager.get_pose(name)
        if saved_pose is None:
            msg = f"AR tag '{name}'을(를) 찾을 수 없습니다"
            if self._on_error:
                self._on_error(msg)
            return MoveResult(False, msg)

        pose = saved_pose.pose
        return self.move_to_pose(
            pose.x, pose.y, pose.z + offset_z,
            pose.rx, pose.ry, pose.rz,
            use_back_pose=True
        )

    def move_with_offset(self, name: str, offset_x: float = 0.0,
                         offset_y: float = 0.0, offset_z: float = 0.0) -> MoveResult:
        """
        저장된 위치에 오프셋 적용하여 이동

        Args:
            name: 저장된 포즈 이름
            offset_x, offset_y, offset_z: 오프셋 (meter)

        Returns:
            MoveResult
        """
        saved_pose = self.pose_manager.get_pose(name)
        if saved_pose is None:
            msg = f"저장된 위치 '{name}'을(를) 찾을 수 없습니다"
            if self._on_error:
                self._on_error(msg)
            return MoveResult(False, msg)

        pose = saved_pose.pose
        return self.move_to_pose(
            pose.x + offset_x,
            pose.y + offset_y,
            pose.z + offset_z,
            pose.rx, pose.ry, pose.rz,
            use_back_pose=True
        )

    # ==================== 어프로치 이동 ====================

    def approach_pose(self, name: str, approach_distance: float = 0.2) -> MoveResult:
        """
        저장된 위치의 어프로치 위치로 이동 (Z축 방향으로 뒤로)

        Args:
            name: 저장된 포즈 이름
            approach_distance: 어프로치 거리 (meter)

        Returns:
            MoveResult
        """
        return self.move_with_offset(name, offset_z=-approach_distance)

    def approach_and_move(self, name: str, approach_distance: float = 0.2,
                          use_approach: bool = True) -> MoveResult:
        """
        KAIST 방식: 어프로치 위치 → 타겟 위치로 순차 이동

        pose_back (approach)과 pose_main (target)을 동시에 쓰고
        응답 1을 보내면 로봇이 pose_back → pose_main 순으로 이동

        Args:
            name: 저장된 포즈 이름
            approach_distance: 어프로치 거리 (meter, Z축 방향 뒤로)
            use_approach: True=approach 후 이동, False=직접 이동

        Returns:
            MoveResult
        """
        if not self.is_connected:
            return MoveResult(False, "로봇에 연결되지 않았습니다")

        saved_pose = self.pose_manager.get_pose(name)
        if saved_pose is None:
            msg = f"저장된 위치 '{name}'을(를) 찾을 수 없습니다"
            if self._on_error:
                self._on_error(msg)
            return MoveResult(False, msg)

        pose = saved_pose.pose
        target_pose = (pose.x, pose.y, pose.z, pose.rx, pose.ry, pose.rz)

        if use_approach and approach_distance > 0:
            # Approach pose: 같은 orientation, Z축으로 뒤로
            approach_pose = (
                pose.x, pose.y, pose.z - approach_distance,
                pose.rx, pose.ry, pose.rz
            )

            if self._on_move_start:
                self._on_move_start(*approach_pose)

            # 두 pose 모두 쓰기
            if not self.modbus.write_pose_both(target_pose, approach_pose):
                msg = "위치 데이터 쓰기 실패"
                if self._on_error:
                    self._on_error(msg)
                return MoveResult(False, msg)

            # 응답 1 = pose_back(approach) 먼저 사용
            if not self.modbus.write_response(Response.USE_POSE_BACK):
                msg = "응답 쓰기 실패"
                if self._on_error:
                    self._on_error(msg)
                return MoveResult(False, msg)

            if self._on_move_complete:
                self._on_move_complete(*target_pose)

            return MoveResult(
                True,
                f"'{name}' 어프로치→이동: approach({approach_pose[0]:.3f}, {approach_pose[1]:.3f}, {approach_pose[2]:.3f}) → target({target_pose[0]:.3f}, {target_pose[1]:.3f}, {target_pose[2]:.3f})",
                target_pose
            )
        else:
            # 어프로치 없이 직접 이동
            return self.move_to_pose_tuple(target_pose, use_back_pose=False)

    # ==================== 유틸리티 ====================

    def get_saved_pose_names(self) -> List[str]:
        """저장된 모든 포즈 이름"""
        return self.pose_manager.get_all_names()

    def get_ar_tag_names(self) -> List[str]:
        """AR tag 포즈 이름 목록"""
        ar_poses = self.pose_manager.get_poses_by_type(PoseType.AR_TAG)
        return [p.name for p in ar_poses]

    def delete_saved_pose(self, name: str) -> bool:
        """저장된 포즈 삭제"""
        return self.pose_manager.delete_pose(name)
