# -*- coding: utf-8 -*-
"""
AR Tag → Base 좌표계 변환 모듈

변환 체인:
    AR Tag (Camera 좌표계)
            ↓ camera_to_vision()
    AR Tag (Vision/TF1 좌표계)
            ↓ vision_to_base()
    AR Tag (Base 좌표계)

좌표계 정의:
    - Camera: OpenCV 카메라 좌표계 (X:우, Y:하, Z:전방)
    - Vision (TF1): 로봇 Tool Frame 1 (X:우, Y:하, Z:전방)
    - Base: 로봇 Base 좌표계 (고정)

변환 관계:
    - Camera → Vision: (1, -1, -1)
    - Vision → Base: 로봇 자세 (Rx, Ry, Rz) 기반 회전 행렬 적용
"""

import numpy as np
import cv2


# =============================================================================
# 함수: euler_to_rotation_matrix(rx, ry, rz)
# =============================================================================
# 목적: 오일러 각 → 회전 행렬 변환
# 입력: rx, ry, rz (degrees)
# 출력: R (3x3 회전 행렬)
# 처리:
#   1. 각도를 라디안으로 변환
#   2. Rx, Ry, Rz 개별 회전 행렬 생성
#   3. R = Rz × Ry × Rx 순서로 곱함
# =============================================================================
def euler_to_rotation_matrix(rx, ry, rz):
    """
    오일러 각 → 회전 행렬 변환

    Args:
        rx, ry, rz: 오일러 각 (degrees)

    Returns:
        R: 3x3 회전 행렬
    """
    # 라디안 변환
    rx_rad = np.radians(rx)
    ry_rad = np.radians(ry)
    rz_rad = np.radians(rz)

    # X축 회전
    Rx = np.array([
        [1, 0, 0],
        [0, np.cos(rx_rad), -np.sin(rx_rad)],
        [0, np.sin(rx_rad), np.cos(rx_rad)]
    ])

    # Y축 회전
    Ry = np.array([
        [np.cos(ry_rad), 0, np.sin(ry_rad)],
        [0, 1, 0],
        [-np.sin(ry_rad), 0, np.cos(ry_rad)]
    ])

    # Z축 회전
    Rz = np.array([
        [np.cos(rz_rad), -np.sin(rz_rad), 0],
        [np.sin(rz_rad), np.cos(rz_rad), 0],
        [0, 0, 1]
    ])

    # R = Rz × Ry × Rx
    R = Rz @ Ry @ Rx

    return R


# =============================================================================
# 함수: rotation_matrix_to_euler(R)
# =============================================================================
# 목적: 회전 행렬 → 오일러 각 변환
# 입력: R (3x3 회전 행렬)
# 출력: (rx, ry, rz) in degrees
# 처리:
#   1. 회전 행렬에서 각 요소 추출
#   2. atan2 함수로 각도 계산
#   3. 라디안 → 도 변환
# =============================================================================
def rotation_matrix_to_euler(R):
    """
    회전 행렬 → 오일러 각 변환 (ZYX 컨벤션)

    Args:
        R: 3x3 회전 행렬

    Returns:
        (rx, ry, rz): 오일러 각 (degrees)
    """
    sy = np.sqrt(R[0, 0]**2 + R[1, 0]**2)

    singular = sy < 1e-6

    if not singular:
        rx = np.arctan2(R[2, 1], R[2, 2])
        ry = np.arctan2(-R[2, 0], sy)
        rz = np.arctan2(R[1, 0], R[0, 0])
    else:
        rx = np.arctan2(-R[1, 2], R[1, 1])
        ry = np.arctan2(-R[2, 0], sy)
        rz = 0

    return np.degrees(rx), np.degrees(ry), np.degrees(rz)


# =============================================================================
# 함수: camera_to_vision(tvec, rvec)
# =============================================================================
# 목적: Camera 좌표계 → Vision(TF1) 좌표계 변환
# 입력: tvec (3,), rvec (3,) - 카메라 좌표계
# 출력: tvec (3,), rvec (3,) - Vision 좌표계
# 처리:
#   1. 위치 변환: (1, -1, -1) 적용
#      vision_x =  camera_x
#      vision_y = -camera_y
#      vision_z = -camera_z
#   2. 회전 변환:
#      R_vision = R_cam_to_vision × R_camera
#      R_cam_to_vision = diag(1, -1, -1)
# =============================================================================
def camera_to_vision(tvec, rvec):
    """
    Camera 좌표계 → Vision(TF1) 좌표계 변환

    Args:
        tvec: 위치 벡터 (3,) - 카메라 좌표계
        rvec: 회전 벡터 (3,) Rodrigues - 카메라 좌표계

    Returns:
        vision_tvec: 위치 벡터 (3,) - Vision 좌표계
        vision_rvec: 회전 벡터 (3,) Rodrigues - Vision 좌표계
    """
    # 변환 행렬 (1, -1, 1) - 카메라 Z+와 Tool Z+가 같은 방향
    R_cam_to_vision = np.array([
        [1, 0, 0],
        [0, -1, 0],
        [0, 0, 1]
    ])

    # 위치 변환
    tvec = np.array(tvec).flatten()
    vision_tvec = R_cam_to_vision @ tvec

    # 회전 변환
    R_camera, _ = cv2.Rodrigues(np.array(rvec).flatten())
    R_vision = R_cam_to_vision @ R_camera
    vision_rvec, _ = cv2.Rodrigues(R_vision)

    return vision_tvec, vision_rvec.flatten()


# =============================================================================
# 함수: vision_to_base(tvec, rvec, robot_pose)
# =============================================================================
# 목적: Vision(TF1) 좌표계 → Base 좌표계 변환
# 입력:
#   - tvec (3,): Vision 좌표계 위치
#   - rvec (3,): Vision 좌표계 회전 (Rodrigues)
#   - robot_pose (6,): 로봇 TCP (X, Y, Z, Rx, Ry, Rz)
# 출력: tvec (3,), rvec (3,) - Base 좌표계
# 처리:
#   1. 로봇 자세에서 회전 행렬 생성: R_base_tool = euler_to_rotation_matrix(Rx, Ry, Rz)
#   2. 위치 변환: P_base = R_base_tool × P_vision + T_tcp
#   3. 회전 변환: R_base = R_base_tool × R_vision
# =============================================================================
def vision_to_base(tvec, rvec, robot_pose):
    """
    Vision(TF1) 좌표계 → Base 좌표계 변환

    Args:
        tvec: 위치 벡터 (3,) - Vision 좌표계 (meters)
        rvec: 회전 벡터 (3,) Rodrigues - Vision 좌표계
        robot_pose: 로봇 TCP (X, Y, Z, Rx, Ry, Rz) - (mm, deg)

    Returns:
        base_tvec: 위치 벡터 (3,) - Base 좌표계 (meters)
        base_rvec: 회전 벡터 (3,) Rodrigues - Base 좌표계
    """
    # 로봇 자세 추출
    tcp_x, tcp_y, tcp_z = robot_pose[0], robot_pose[1], robot_pose[2]
    tcp_rx, tcp_ry, tcp_rz = robot_pose[3], robot_pose[4], robot_pose[5]

    # TCP 위치 (mm → m)
    T_tcp = np.array([tcp_x, tcp_y, tcp_z]) / 1000.0

    # 로봇 자세 회전 행렬
    R_base_tool = euler_to_rotation_matrix(tcp_rx, tcp_ry, tcp_rz)

    # 위치 변환: P_base = R_base_tool × P_vision + T_tcp
    tvec = np.array(tvec).flatten()
    base_tvec = R_base_tool @ tvec + T_tcp

    # 회전 변환: R_base = R_base_tool × R_vision
    R_vision, _ = cv2.Rodrigues(np.array(rvec).flatten())
    R_base = R_base_tool @ R_vision
    base_rvec, _ = cv2.Rodrigues(R_base)

    return base_tvec, base_rvec.flatten()


# =============================================================================
# 함수: ar_to_base(tvec, rvec, robot_pose)
# =============================================================================
# 목적: AR Tag의 Camera 좌표계 pose → Base 좌표계 pose 변환 (메인 함수)
# 입력:
#   - tvec (3,): AR Tag 위치 (카메라 좌표계, meters)
#   - rvec (3,): AR Tag 회전 (Rodrigues 벡터)
#   - robot_pose (6,): 로봇 TCP (X, Y, Z in mm, Rx, Ry, Rz in deg)
# 출력: (X, Y, Z, Rx, Ry, Rz) - Base 좌표계 (mm, deg)
# 처리:
#   1. camera_to_vision() 호출
#   2. vision_to_base() 호출
#   3. 회전 행렬 → 오일러 각 변환
#   4. 단위 변환 (m → mm)
# =============================================================================
def ar_to_base(tvec, rvec, robot_pose):
    """
    AR Tag의 Camera 좌표계 pose → Base 좌표계 pose 변환

    Args:
        tvec: AR Tag 위치 (3,) - 카메라 좌표계 (meters)
        rvec: AR Tag 회전 (3,) Rodrigues - 카메라 좌표계
        robot_pose: 로봇 TCP (X, Y, Z, Rx, Ry, Rz) - (mm, deg)

    Returns:
        (X, Y, Z, Rx, Ry, Rz): Base 좌표계 (mm, deg)
    """
    # 1. Camera → Vision 변환
    vision_tvec, vision_rvec = camera_to_vision(tvec, rvec)

    # 2. Vision → Base 변환
    base_tvec, base_rvec = vision_to_base(vision_tvec, vision_rvec, robot_pose)

    # 3. 회전 행렬 → 오일러 각
    R_base, _ = cv2.Rodrigues(base_rvec)
    rx, ry, rz = rotation_matrix_to_euler(R_base)

    # 4. 위치 단위 변환 (m → mm)
    x = base_tvec[0] * 1000.0
    y = base_tvec[1] * 1000.0
    z = base_tvec[2] * 1000.0

    return (x, y, z, rx, ry, rz)
