"""Toolframe 좌표 변환 유틸리티.

로봇 TCP 좌표를 다른 Toolframe 기준으로 변환.
각 TF는 플랜지(TF4, 오프셋 0)로부터의 TCP 오프셋을 정의.

TF4: 오프셋 없음 (X=0, Y=0, Z=0) — 플랜지 원점
TF5: Hand-Eye Cal 결과 (X=29.23, Y=80.24, Z=-23.40) — 카메라 위치

변환 수학:
  TF는 플랜지 기준 TOOL 좌표의 TCP 오프셋을 정의한다.
  로봇은 TCP 위치를 BASE 프레임으로 보고한다.
  TF가 바뀌면 같은 물리적 로봇 위치가 다른 XYZ로 보고된다.

  TF_A → TF_B 변환:
    1. 플랜지 위치: p_flange = p_TF_A - R_flange @ offset_A
    2. TF_B 위치:   p_TF_B  = p_flange + R_flange @ offset_B

  R_flange: Euler 각도(Rx, Ry, Rz)로부터 계산된 회전 행렬 (ZYX 규약).
  TF4(오프셋=0): p_TF4 = p_flange 직접.

Note: TF5 회전 오프셋(Rx, Ry, Rz)이 있을 경우 TF5_OFFSET의 마지막 3 요소에 추가 가능.
      현재는 병진 오프셋만 정의되어 있음.
"""

import math
import numpy as np


# ---------------------------------------------------------------------------
# TF 오프셋 상수 [x_mm, y_mm, z_mm, rx_deg, ry_deg, rz_deg]
# ---------------------------------------------------------------------------
TF4_OFFSET = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]   # 플랜지 원점 (오프셋 없음)
TF5_OFFSET = [29.23, 80.24, -23.40, 0.0, 0.0, 0.0]  # Hand-Eye Cal (병진만)
# TODO: TF5 회전 오프셋이 캘리브레이션되면 위 rx/ry/rz 값을 업데이트할 것


def euler_to_rotation_matrix(rx: float, ry: float, rz: float) -> np.ndarray:
    """Euler 각도(도)를 3x3 회전 행렬로 변환 (ZYX 규약 = Rz @ Ry @ Rx).

    로봇 컨트롤러와 동일한 ZYX (Roll-Pitch-Yaw) 순서를 사용한다.

    Args:
        rx: X축 회전각 (도)
        ry: Y축 회전각 (도)
        rz: Z축 회전각 (도)

    Returns:
        3x3 numpy 회전 행렬
    """
    rx_r = math.radians(rx)
    ry_r = math.radians(ry)
    rz_r = math.radians(rz)

    # Rx (Roll)
    Rx = np.array([
        [1,             0,              0],
        [0,  math.cos(rx_r), -math.sin(rx_r)],
        [0,  math.sin(rx_r),  math.cos(rx_r)],
    ])
    # Ry (Pitch)
    Ry = np.array([
        [ math.cos(ry_r), 0, math.sin(ry_r)],
        [             0,  1,             0],
        [-math.sin(ry_r), 0, math.cos(ry_r)],
    ])
    # Rz (Yaw)
    Rz = np.array([
        [math.cos(rz_r), -math.sin(rz_r), 0],
        [math.sin(rz_r),  math.cos(rz_r), 0],
        [            0,              0,  1],
    ])

    # ZYX: R = Rz @ Ry @ Rx
    return Rz @ Ry @ Rx


def convert_pose(
    pose: list,
    from_tf_offset: list,
    to_tf_offset: list,
) -> list:
    """TCP 포즈를 한 Toolframe 기준에서 다른 기준으로 변환.

    Args:
        pose: [x, y, z, rx, ry, rz] — 변환할 포즈 (mm, 도)
        from_tf_offset: 출발 TF 오프셋 [x, y, z, rx, ry, rz] (mm, 도)
        to_tf_offset:   도착 TF 오프셋 [x, y, z, rx, ry, rz] (mm, 도)

    Returns:
        변환된 포즈 [x, y, z, rx, ry, rz] (Euler 각도 유지)
    """
    x, y, z, rx, ry, rz = pose

    # 플랜지 기준 회전 행렬 (현재 포즈의 Euler 각도 사용)
    R_flange = euler_to_rotation_matrix(rx, ry, rz)

    p_tcp = np.array([x, y, z])

    # 출발 TF 오프셋 (TOOL 좌표)
    off_from = np.array(from_tf_offset[:3])
    # 도착 TF 오프셋 (TOOL 좌표)
    off_to = np.array(to_tf_offset[:3])

    # 1. 플랜지 위치 계산
    p_flange = p_tcp - R_flange @ off_from

    # 2. 도착 TF 위치 계산
    p_new = p_flange + R_flange @ off_to

    # Euler 각도는 물리적 플랜지 자세가 바뀌지 않으므로 그대로 유지
    # (TF 변경은 보고 TCP 위치만 변경하며, 플랜지 자세(rx,ry,rz)는 동일)
    return [float(p_new[0]), float(p_new[1]), float(p_new[2]), rx, ry, rz]


def tf5_to_tf4(pose: list) -> list:
    """TF5 기준 포즈를 TF4 기준으로 변환.

    Args:
        pose: [x, y, z, rx, ry, rz] (TF5 기준, mm/도)

    Returns:
        [x, y, z, rx, ry, rz] (TF4 기준, mm/도)
    """
    return convert_pose(pose, from_tf_offset=TF5_OFFSET, to_tf_offset=TF4_OFFSET)


def tf4_to_tf5(pose: list) -> list:
    """TF4 기준 포즈를 TF5 기준으로 변환.

    Args:
        pose: [x, y, z, rx, ry, rz] (TF4 기준, mm/도)

    Returns:
        [x, y, z, rx, ry, rz] (TF5 기준, mm/도)
    """
    return convert_pose(pose, from_tf_offset=TF4_OFFSET, to_tf_offset=TF5_OFFSET)


if __name__ == '__main__':
    # 데모: laser_detection_pose.json의 TF5 좌표를 TF4로 변환
    pose_tf5 = [459.67, 608.60, 612.79, 90.0, 0.0, -180.0]

    print("=== TF 변환 데모 ===")
    print(f"TF5 포즈: X={pose_tf5[0]:.2f}  Y={pose_tf5[1]:.2f}  Z={pose_tf5[2]:.2f}"
          f"  Rx={pose_tf5[3]:.2f}  Ry={pose_tf5[4]:.2f}  Rz={pose_tf5[5]:.2f}")

    pose_tf4 = tf5_to_tf4(pose_tf5)
    print(f"TF4 포즈: X={pose_tf4[0]:.2f}  Y={pose_tf4[1]:.2f}  Z={pose_tf4[2]:.2f}"
          f"  Rx={pose_tf4[3]:.2f}  Ry={pose_tf4[4]:.2f}  Rz={pose_tf4[5]:.2f}")

    # 역변환 확인 (TF4 → TF5 → TF4 왕복)
    pose_back = tf4_to_tf5(pose_tf4)
    err = max(abs(a - b) for a, b in zip(pose_tf5, pose_back))
    print(f"왕복 오차 (TF5→TF4→TF5): max={err:.6f}mm (0이면 정확)")

    print()
    print("참고 — TF5 오프셋:", TF5_OFFSET[:3], "(mm, 병진만)")
    print("참고 — TF4 오프셋:", TF4_OFFSET[:3], "(플랜지 원점)")
