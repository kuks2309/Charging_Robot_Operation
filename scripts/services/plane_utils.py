"""
평면 유틸리티 함수
Normal/horizontal 벡터에서 Euler 각도 추출 (scipy 기반)
"""
import numpy as np
from typing import Tuple
from scipy.spatial.transform import Rotation
import warnings


def normal_horizontal_to_euler(normal: np.ndarray,
                                horizontal: np.ndarray) -> Tuple[float, float, float]:
    """
    평면의 normal + horizontal 벡터에서 Euler 각도 추출 (scipy 사용)

    좌표계 규약:
    - normal: 평면에 수직, 포트 바깥 방향 (+Z 기준으로 간주)
    - horizontal: 마커1→마커2 방향 (+X 기준으로 간주)
    - vertical: cross(normal, horizontal) = +Y 기준

    Args:
        normal: 평면 법선 벡터 (정규화 불필요, 내부에서 처리)
        horizontal: 수평 벡터 (정규화 불필요)

    Returns:
        (rx, ry, rz) in degrees (ZYX Euler 순서)

    Raises:
        ValueError: 벡터가 평행하거나 영벡터인 경우
    """
    # 1. 정규화
    normal = np.asarray(normal, dtype=np.float64)
    horizontal = np.asarray(horizontal, dtype=np.float64)

    norm_n = np.linalg.norm(normal)
    norm_h = np.linalg.norm(horizontal)

    if norm_n < 1e-10 or norm_h < 1e-10:
        raise ValueError("영벡터가 입력됨")

    z_axis = normal / norm_n

    # 2. 직교 좌표계 구성 (Gram-Schmidt)
    # Y축 = cross(Z, X_initial)
    y_axis = np.cross(z_axis, horizontal)
    norm_y = np.linalg.norm(y_axis)

    if norm_y < 1e-10:
        raise ValueError("normal과 horizontal이 평행함")

    y_axis = y_axis / norm_y

    # X축 = cross(Y, Z) → 직교 보장
    x_axis = np.cross(y_axis, z_axis)
    x_axis = x_axis / np.linalg.norm(x_axis)

    # 3. 회전 행렬 구성 (열 벡터)
    R = np.column_stack([x_axis, y_axis, z_axis])

    # 4. scipy로 Euler 변환 (ZYX extrinsic = 로봇 표준)
    rot = Rotation.from_matrix(R)
    euler = rot.as_euler('ZYX', degrees=True)  # [rz, ry, rx]
    rx, ry, rz = euler[2], euler[1], euler[0]

    # 5. Gimbal lock 경고
    if abs(abs(ry) - 90.0) < 1.0:
        warnings.warn(f"Near gimbal lock: ry={ry:.1f}°, rx/rz may be unstable")

    return (rx, ry, rz)


def ensure_normal_positive_z(normal: np.ndarray) -> np.ndarray:
    """
    Normal 벡터가 +Z 방향(카메라 방향)을 향하도록 보정

    Args:
        normal: 입력 법선 벡터

    Returns:
        +Z 방향으로 보정된 법선 벡터
    """
    normal = np.asarray(normal, dtype=np.float64)
    if normal[2] < 0:
        return -normal
    return normal
