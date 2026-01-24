# -*- coding: utf-8 -*-
"""
카메라 캘리브레이션용 위치 생성기

TF1 (Tool Frame 1 - 비전 프레임) 좌표계로 직관적인 패턴 생성
실제 모션 시 Base 좌표계로 변환 필요
"""

from typing import List, Tuple


def generate_planar_positions_vision_tf(xy_step: int, z_step: int) -> List[Tuple[int, int, int]]:
    """
    캘리브레이션 위치 생성 - TF1 (비전 프레임) 좌표계

    좌표계: TF1 (Tool Frame 1 - 카메라 뷰)
    - X: 카메라 좌/우 (이미지 가로)
    - Y: 카메라 상/하 (이미지 세로)
    - Z: 광축 방향 (체스보드와의 거리)

    패턴 구조:
    - Z 고정 상태에서 XY 평면 9개 위치 (중심 시작, 시계방향) 순회
    - 1번(중심)으로 복귀
    - 다음 Z로 이동
    - 반복 (Z 3단계)

    XY 평면 9개 위치 (Z 고정 시, 중심 시작 시계방향):
    ┌─────────────────────────┐
    │ [3]     [2]     [9]     │   ↑ +Y
    │  ↑       ↑       │      │
    │  │       │       ↓      │
    │ [4]←──[1,10]───→[8]     │   Y: 0 (중앙)
    │          │       ↑      │
    │          ↓       │      │
    │ [5]←────[6]────→[7]     │   ↓ -Y
    └─────────────────────────┘
       -X     X:0     +X

    총 위치 수:
    - [0]: 시작 위치 (체스보드 중심 정렬)
    - Z1 (0): 9개 + 복귀 1개 = 10개
    - Z2 (+z_step): 9개 + 복귀 1개 = 10개
    - Z3 (+z_step*2): 9개 + 복귀 1개 = 10개
    - 마지막 원점 복귀: 1개
    = 1 + 30 + 1 = 32개

    Args:
        xy_step: XY 이동 간격 (mm) - 카메라 뷰 기준 좌/우/상/하
        z_step: Z 이동 간격 (mm) - 광축 방향 (거리)

    Returns:
        위치 리스트 [(dx, dy, dz), ...] - TF1 좌표계 기준 상대 오프셋
    """
    # XY 평면 9개 위치 (Z 고정 시, 중심 시작 시계방향)
    # 순서: [1]중심 → [2]상 → [3]좌상 → [4]좌 → [5]좌하 → [6]하 → [7]우하 → [8]우 → [9]우상 → [10]중심복귀
    #
    # ┌─────────────────────────┐
    # │ [3]     [2]     [9]     │   ↑ +Y
    # │ [4]   [1,10]    [8]     │   Y: 0 (중앙)
    # │ [5]     [6]     [7]     │   ↓ -Y
    # └─────────────────────────┘
    #    -X     X:0     +X
    xy_positions = [
        (0, 0),               # [1] 중심
        (0, xy_step),         # [2] 상 (0, +Y)
        (-xy_step, xy_step),  # [3] 좌상 (-X, +Y)
        (-xy_step, 0),        # [4] 좌 (-X, 0)
        (-xy_step, -xy_step), # [5] 좌하 (-X, -Y)
        (0, -xy_step),        # [6] 하 (0, -Y)
        (xy_step, -xy_step),  # [7] 우하 (+X, -Y)
        (xy_step, 0),         # [8] 우 (+X, 0)
        (xy_step, xy_step),   # [9] 우상 (+X, +Y)
    ]

    z_offsets = [0, z_step, z_step * 2]  # Z 3단계 (0 → 100 → 200mm)

    # [0] 시작 위치 (체스보드 중심 정렬 후 기준점)
    positions = [(0, 0, 0)]

    # Z 각 단계에서 XY 평면 9개 위치 순회 후 복귀
    for dz in z_offsets:
        # XY 9개 위치 순회
        for dx, dy in xy_positions:
            positions.append((dx, dy, dz))
        # 해당 Z에서 중심(1번)으로 복귀
        positions.append((0, 0, dz))

    # 마지막: 원점 (0, 0, 0)으로 복귀
    positions.append((0, 0, 0))

    return positions


def generate_planar_positions_base_tf(xy_step: int, z_step: int) -> List[Tuple[int, int, int]]:
    """
    캘리브레이션 위치 생성 - Base 좌표계 (실제 모션용)

    TF1 좌표를 Base 좌표로 변환하여 반환
    카메라 자세: Rx=90°, Ry=0°, Rz=90° (카메라가 아래를 바라봄)

    좌표 변환 (TF1 → Base):
    - TF1 X (좌/우) → Base Y
    - TF1 Y (상/하) → Base X
    - TF1 Z (거리) → Base -Z

    Args:
        xy_step: XY 이동 간격 (mm) - TF1 기준
        z_step: Z 이동 간격 (mm) - TF1 기준

    Returns:
        위치 리스트 [(dx, dy, dz), ...] - Base 좌표계 기준 상대 오프셋
    """
    # TF1 좌표 생성
    vision_positions = generate_planar_positions_vision_tf(xy_step, z_step)

    # TF1 → Base 변환
    base_positions = []
    for tf1_x, tf1_y, tf1_z in vision_positions:
        base_x = tf1_y   # TF1 Y → Base X
        base_y = tf1_x   # TF1 X → Base Y
        base_z = -tf1_z  # TF1 Z → Base -Z
        base_positions.append((base_x, base_y, base_z))

    return base_positions


def generate_base_absolute_positions(
    base_pose: Tuple[float, float, float, float, float, float],
    xy_step: int,
    z_step: int
) -> List[Tuple[float, float, float, float, float, float]]:
    """
    캘리브레이션 위치 생성 - Base 절대 좌표

    기준 좌표(base_pose)에 TF1 오프셋을 적용하여 Base 절대 좌표 생성

    좌표 변환 (TF1 오프셋 → Base 오프셋):
    - TF1 X (좌/우) → Base Y
    - TF1 Y (상/하) → Base Z (위/아래)
    - TF1 Z (거리) → Base -X (로봇 전방)

    Args:
        base_pose: 기준 좌표 (X, Y, Z, Rx, Ry, Rz) - mm, deg
        xy_step: XY 이동 간격 (mm)
        z_step: Z 이동 간격 (mm)

    Returns:
        위치 리스트 [(X, Y, Z, Rx, Ry, Rz), ...] - Base 절대 좌표
    """
    # TF1 오프셋 생성
    tf1_offsets = generate_planar_positions_vision_tf(xy_step, z_step)

    base_x, base_y, base_z = base_pose[0], base_pose[1], base_pose[2]
    base_rx, base_ry, base_rz = base_pose[3], base_pose[4], base_pose[5]

    # Base 절대 좌표로 변환
    positions = []
    for tf1_dx, tf1_dy, tf1_dz in tf1_offsets:
        # TF1 → Base 변환 (Rx=90, Rz=90 자세 기준)
        # TF1 X → Base Y, TF1 Y → Base Z, TF1 Z → Base -X
        abs_x = base_x - tf1_dz  # TF1 Z → Base -X (거리 증가 = X 감소)
        abs_y = base_y + tf1_dx  # TF1 X → Base Y
        abs_z = base_z + tf1_dy  # TF1 Y → Base Z
        positions.append((abs_x, abs_y, abs_z, base_rx, base_ry, base_rz))

    return positions


def format_position_label_base(
    index: int,
    pos: Tuple[float, float, float, float, float, float],
    total_count: int,
    base_pose: Tuple[float, float, float, float, float, float] = None
) -> str:
    """
    Base 절대 좌표를 표시용 문자열로 변환 (상대 + 절대 + 회전)

    Args:
        index: 위치 인덱스 (0부터 시작)
        pos: (X, Y, Z, Rx, Ry, Rz) 튜플 - Base 절대 좌표
        total_count: 전체 위치 수
        base_pose: 기준 좌표 (X, Y, Z, Rx, Ry, Rz) - 상대 좌표 계산용

    Returns:
        포맷된 문자열
    """
    x, y, z, rx, ry, rz = pos
    is_first = (index == 0)
    is_last = (index == total_count - 1)

    # 상대 좌표 계산 (TF1 기준으로 표시)
    # Base → TF1 역변환: Base Y → TF1 X, Base Z → TF1 Y, Base -X → TF1 Z
    if base_pose:
        base_dx = x - base_pose[0]  # Base X 차이
        base_dy = y - base_pose[1]  # Base Y 차이
        base_dz = z - base_pose[2]  # Base Z 차이
        # TF1 좌표로 역변환
        tf1_dx = int(base_dy)   # Base Y → TF1 X
        tf1_dy = int(base_dz)   # Base Z → TF1 Y
        tf1_dz = int(-base_dx)  # Base -X → TF1 Z (부호 반전)
        rel_str = f"X={tf1_dx:+d}, Y={tf1_dy:+d}, Z={tf1_dz:+d}mm"
    else:
        rel_str = ""

    # 회전 정보 (기준과 다를 때만 표시)
    rotation_str = ""
    if base_pose and (rx != base_pose[3] or ry != base_pose[4] or rz != base_pose[5]):
        rotation_str = f" Rx={rx:.0f}°, Ry={ry:.0f}°, Rz={rz:.0f}°"

    if is_first:
        label = f"[{index}] 시작 - X={x:.1f}, Y={y:.1f}, Z={z:.1f}mm{rotation_str}"
    elif is_last:
        label = f"[{index}] 복귀 - X={x:.1f}, Y={y:.1f}, Z={z:.1f}mm{rotation_str}"
    else:
        if rel_str:
            label = f"[{index}] {rel_str}{rotation_str}"
        else:
            label = f"[{index}] X={x:.1f}, Y={y:.1f}, Z={z:.1f}mm{rotation_str}"

    return label


def generate_base_positions_with_rotation(
    base_pose: Tuple[float, float, float, float, float, float],
    xy_step: int,
    z_step: int
) -> List[Tuple[float, float, float, float, float, float]]:
    """
    캘리브레이션 위치 생성 (회전 포함) - Base 절대 좌표

    각 XYZ 위치마다 9개 독립 회전 자세 적용 (복합 회전 금지):
    - Rx만 변화 (Ry=0, Rz=90 고정): 80°, 90°, 100° (3개)
    - Ry만 변화 (Rx=90, Rz=90 고정): -10°, 0°, 10° (3개)
    - Rz만 변화 (Rx=90, Ry=0 고정): 80°, 90°, 100° (3개)
    = 총 9개 회전 자세 (중앙 90/0/90 중복 포함)

    총 위치 수: 32개 XYZ × 9개 회전 = 288개

    Args:
        base_pose: 기준 좌표 (X, Y, Z, Rx, Ry, Rz) - mm, deg
        xy_step: XY 이동 간격 (mm)
        z_step: Z 이동 간격 (mm)

    Returns:
        위치 리스트 [(X, Y, Z, Rx, Ry, Rz), ...] - Base 절대 좌표
    """
    # 기존 XYZ 위치 생성 (32개)
    xyz_positions = generate_base_absolute_positions(base_pose, xy_step, z_step)

    # 독립 회전 자세 (9개 - 복합 회전 금지)
    rotation_poses = []
    # Rx만 변화 (Ry=0, Rz=90 고정)
    for rx in [80, 90, 100]:
        rotation_poses.append((rx, 0, 90))
    # Ry만 변화 (Rx=90, Rz=90 고정)
    for ry in [-10, 0, 10]:
        rotation_poses.append((90, ry, 90))
    # Rz만 변화 (Rx=90, Ry=0 고정)
    for rz in [80, 90, 100]:
        rotation_poses.append((90, 0, rz))

    # 각 XYZ 위치에 대해 9개 회전 자세 적용
    positions = []
    for xyz_pos in xyz_positions:
        x, y, z, _, _, _ = xyz_pos  # XYZ만 사용
        for rx, ry, rz in rotation_poses:
            positions.append((x, y, z, rx, ry, rz))

    return positions


def format_position_label(
    index: int,
    pos: Tuple[int, int, int],
    total_count: int,
    base_pos: Tuple[float, float, float] = None
) -> str:
    """
    위치를 표시용 문자열로 변환 (TF1 좌표계 기준) - 하위 호환용

    Args:
        index: 위치 인덱스 (0부터 시작)
        pos: (dx, dy, dz) 튜플 - TF1 좌표계 상대 오프셋
        total_count: 전체 위치 수
        base_pos: 기준 좌표 (X, Y, Z) - 0번과 마지막에만 표기

    Returns:
        포맷된 문자열
    """
    dx, dy, dz = pos
    is_first = (index == 0)
    is_last = (index == total_count - 1)

    if is_first:
        label = f"[{index}] 시작 (체스보드 중심 정렬)"
        if base_pos:
            bx, by, bz = base_pos
            label += f" - Base: X={bx:.1f}, Y={by:.1f}, Z={bz:.1f}mm"
    elif is_last:
        label = f"[{index}] 복귀 (원점)"
        if base_pos:
            bx, by, bz = base_pos
            label += f" - Base: X={bx:.1f}, Y={by:.1f}, Z={bz:.1f}mm"
    elif dx == 0 and dy == 0:
        # 중심에서 Z 레벨 변경
        label = f"[{index}] 중심 (Z={dz:+d}mm)"
    else:
        label = f"[{index}] X={dx:+d}, Y={dy:+d}, Z={dz:+d}mm"

    return label
