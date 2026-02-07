"""
TCP 보정값 계산 모듈
평면 자세 기반 TCP rx, ry, rz 보정
"""
from dataclasses import dataclass
from typing import Tuple, Optional
import numpy as np
from scipy.spatial.transform import Rotation


@dataclass
class TCPCorrection:
    """TCP 보정 결과 (단위: degrees)"""
    delta_rx: float  # degrees
    delta_ry: float  # degrees
    delta_rz: float  # degrees

    # 원본 값 (디버깅용)
    plane_rx: float
    plane_ry: float
    plane_rz: float
    target_rx: float
    target_ry: float
    target_rz: float

    # 최종 절대 TCP 자세 (로봇 이동에 사용)
    final_rx: Optional[float] = None  # degrees
    final_ry: Optional[float] = None  # degrees
    final_rz: Optional[float] = None  # degrees

    # 현재 TCP 정보 (계산 추적용)
    current_tcp_rx: Optional[float] = None
    current_tcp_ry: Optional[float] = None
    current_tcp_rz: Optional[float] = None

    def to_dict(self) -> dict:
        return {
            'delta_rx': self.delta_rx,
            'delta_ry': self.delta_ry,
            'delta_rz': self.delta_rz,
            'plane_rx': self.plane_rx,
            'plane_ry': self.plane_ry,
            'plane_rz': self.plane_rz,
            'target_rx': self.target_rx,
            'target_ry': self.target_ry,
            'target_rz': self.target_rz,
            'final_rx': self.final_rx,
            'final_ry': self.final_ry,
            'final_rz': self.final_rz,
            'current_tcp_rx': self.current_tcp_rx,
            'current_tcp_ry': self.current_tcp_ry,
            'current_tcp_rz': self.current_tcp_rz,
        }


class TCPCorrector:
    """
    평면 자세 기반 TCP 보정값 계산

    평면의 orientation과 목표 orientation의 차이를
    계산하여 보정값 제공

    물리적 의미:
    - 보정값을 현재 TCP에 더하면 TCP가 평면과 수직 정렬됨
    - delta > 0: 해당 축으로 양의 회전 필요
    """

    def __init__(self,
                 target_rx: float = 0.0,
                 target_ry: float = 0.0,
                 target_rz: float = 0.0):
        """
        Args:
            target_rx/ry/rz: 목표 orientation (평면에 수직일 때의 각도)
                             기본값 0,0,0 = 평면 normal과 TCP Z축이 반대 방향
        """
        self.target_rx = target_rx
        self.target_ry = target_ry
        self.target_rz = target_rz

    def _euler_to_rotation_matrix(self, rx: float, ry: float, rz: float) -> np.ndarray:
        """오일러 각도(ZYX)를 회전 행렬로 변환"""
        rot = Rotation.from_euler('ZYX', [rz, ry, rx], degrees=True)
        return rot.as_matrix()

    def _rotation_matrix_to_euler(self, R: np.ndarray) -> Tuple[float, float, float]:
        """회전 행렬을 오일러 각도(ZYX)로 변환"""
        rot = Rotation.from_matrix(R)
        euler = rot.as_euler('ZYX', degrees=True)
        return (euler[2], euler[1], euler[0])  # (rx, ry, rz)

    def compute_correction(self, plane_pose, current_tcp: Optional[Tuple[float, float, float, float, float, float]] = None) -> TCPCorrection:
        """
        TCP 보정값 계산

        Args:
            plane_pose: 검출된 평면 자세 (PlanePose)
            current_tcp: (x, y, z, rx, ry, rz) 현재 TCP 자세 (Optional)
                        제공 시 회전 행렬 합성으로 최종 자세 계산

        Returns:
            TCPCorrection (delta = target - plane)
        """
        delta_rx = self.target_rx - plane_pose.rx
        delta_ry = self.target_ry - plane_pose.ry
        delta_rz = self.target_rz - plane_pose.rz

        result = TCPCorrection(
            delta_rx=delta_rx,
            delta_ry=delta_ry,
            delta_rz=delta_rz,
            plane_rx=plane_pose.rx,
            plane_ry=plane_pose.ry,
            plane_rz=plane_pose.rz,
            target_rx=self.target_rx,
            target_ry=self.target_ry,
            target_rz=self.target_rz,
        )

        # current_tcp 제공 시 회전 행렬 합성
        if current_tcp is not None:
            _, _, _, tcp_rx, tcp_ry, tcp_rz = current_tcp

            # R_final = R_delta @ R_current
            R_current = self._euler_to_rotation_matrix(tcp_rx, tcp_ry, tcp_rz)
            R_delta = self._euler_to_rotation_matrix(delta_rx, delta_ry, delta_rz)
            R_final = R_delta @ R_current

            final_rx, final_ry, final_rz = self._rotation_matrix_to_euler(R_final)

            result.final_rx = final_rx
            result.final_ry = final_ry
            result.final_rz = final_rz
            result.current_tcp_rx = tcp_rx
            result.current_tcp_ry = tcp_ry
            result.current_tcp_rz = tcp_rz

        return result

    def compute_final_tcp(self,
                          current_tcp: Tuple[float, float, float, float, float, float],
                          correction: TCPCorrection
                         ) -> Tuple[float, float, float, float, float, float]:
        """
        회전 행렬 합성으로 최종 TCP 자세 계산

        Args:
            current_tcp: (x, y, z, rx, ry, rz) in mm, degrees
            correction: TCPCorrection

        Returns:
            (x, y, z, final_rx, final_ry, final_rz)
        """
        x, y, z, tcp_rx, tcp_ry, tcp_rz = current_tcp

        # correction에 final_rx가 이미 계산되어 있으면 재사용
        if correction.final_rx is not None:
            return (x, y, z, correction.final_rx, correction.final_ry, correction.final_rz)

        # 없으면 회전 행렬 합성
        R_current = self._euler_to_rotation_matrix(tcp_rx, tcp_ry, tcp_rz)
        R_delta = self._euler_to_rotation_matrix(correction.delta_rx, correction.delta_ry, correction.delta_rz)
        R_final = R_delta @ R_current

        final_rx, final_ry, final_rz = self._rotation_matrix_to_euler(R_final)

        return (x, y, z, final_rx, final_ry, final_rz)

    def apply_correction(self,
                         current_tcp: Tuple[float, float, float, float, float, float],
                         correction: TCPCorrection
                        ) -> Tuple[float, float, float, float, float, float]:
        """
        DEPRECATED: 단순 덧셈 방식 보정 (하위 호환용)
        compute_final_tcp() 사용 권장

        Args:
            current_tcp: (x, y, z, rx, ry, rz) in mm, degrees
            correction: TCPCorrection

        Returns:
            (x, y, z, rx+delta_rx, ry+delta_ry, rz+delta_rz)
        """
        import warnings
        warnings.warn("apply_correction() is deprecated. Use compute_final_tcp().", DeprecationWarning, stacklevel=2)
        return self.apply_correction_simple(current_tcp, correction)

    def apply_correction_simple(self,
                                current_tcp: Tuple[float, float, float, float, float, float],
                                correction: TCPCorrection
                               ) -> Tuple[float, float, float, float, float, float]:
        """
        보정값 적용한 새 TCP 반환 (위치는 유지, 회전만 보정)
        단순 덧셈 방식

        Args:
            current_tcp: (x, y, z, rx, ry, rz) in mm, degrees
            correction: TCPCorrection

        Returns:
            (x, y, z, rx+delta_rx, ry+delta_ry, rz+delta_rz)
        """
        x, y, z, rx, ry, rz = current_tcp
        return (
            x, y, z,
            rx + correction.delta_rx,
            ry + correction.delta_ry,
            rz + correction.delta_rz,
        )
