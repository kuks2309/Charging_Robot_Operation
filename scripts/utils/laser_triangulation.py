"""레이저 삼각측량 높이/깊이 역산 유틸리티.

핵심 발견: sensitivity(px/mm)는 Z_tcp가 아니라 Y_px에 의존한다.
  선형 모델: sens(y) = SENS_SLOPE * y_px + SENS_INTERCEPT  (R²=0.98)

카메라 파라미터 (ArduCam, 1920×1080):
  fy  = 5347.3 px
  cy  = 478.2  px

레이저-카메라 기하:
  레이저 각도 α = 12.75° (수평 기준 아래)
  baseline Bx  = -30.0 mm

삼각측량 모델 (v2):
  y_px = y_floor - fy * k * rise / (D0 - k * tread)
  여기서 D0 = z_tcp + delta_z, k = 계단 인덱스 (0-based)

의존성: numpy, scipy (calibrate_sensitivity만)
"""

from __future__ import annotations

import math
from typing import List, Sequence, Tuple

import numpy as np

# ---------------------------------------------------------------------------
# 카메라 / 레이저 상수
# ---------------------------------------------------------------------------
FY_PX: float = 5347.3       # ArduCam focal length (y방향, px)
CY_PX: float = 478.2        # 주점 y좌표 (px)
LASER_ALPHA_DEG: float = 12.75   # 레이저 입사각 (수평 기준 아래, 도)
BX_MM: float = -30.0        # 레이저-카메라 수평 baseline (mm)

# Y_px → sensitivity 선형 모델: sens(y) = SENS_SLOPE * y + SENS_INTERCEPT
# 단위: (px/mm) per px → px/mm
SENS_SLOPE: float = -0.005535
SENS_INTERCEPT: float = 6.7516

# 삼각측량 보정 파라미터 (laser_vertical_triangulation_calib.json v2)
DELTA_Z_MM: float = 1497.0  # z_tcp → 실제 거리 보정 (D0 = z_tcp + delta_z)


# ---------------------------------------------------------------------------
# 내부 헬퍼
# ---------------------------------------------------------------------------

def _sensitivity_at(y_px: float) -> float:
    """Y 픽셀 위치에서의 sensitivity(px/mm)를 선형 모델로 반환.

    Args:
        y_px: 이미지 Y 좌표 (px)

    Returns:
        sensitivity (px/mm)
    """
    return SENS_SLOPE * y_px + SENS_INTERCEPT


# ---------------------------------------------------------------------------
# 공개 함수
# ---------------------------------------------------------------------------

def estimate_height(y_base_px: float, y_measured_px: float) -> float:
    """바닥 기준 Y 좌표와 측정 Y 좌표로부터 높이를 역산한다.

    sensitivity는 두 Y 좌표의 중간값(y_mid)에서 평가한다.

    물리적 의미:
      - y_base_px:    바닥(step 0) 레이저 라인의 Y 좌표
      - y_measured_px: 측정 대상 표면의 레이저 라인 Y 좌표
      - 반환값:       표면이 바닥보다 높은 높이 (mm, 양수 = 바닥보다 높음)

    Args:
        y_base_px:    바닥 기준 Y 픽셀 (px)
        y_measured_px: 측정 표면 Y 픽셀 (px)

    Returns:
        높이 (mm). 양수 = 바닥보다 높음.
    """
    y_mid = (y_base_px + y_measured_px) / 2.0
    sens = _sensitivity_at(y_mid)
    if sens == 0.0:
        raise ValueError(f"sensitivity=0 at y_mid={y_mid:.1f}px — 모델 범위 초과")
    delta_y_px = y_base_px - y_measured_px  # 높을수록 Y_px 감소 (이미지 좌표)
    return delta_y_px / sens


def estimate_depth(
    y_px_sequence: Sequence[float],
    z_tcp_sequence: Sequence[float],
) -> float:
    """Y_px 변화 패턴과 Z_tcp 변화로부터 계단 깊이(tread, mm)를 추정한다.

    삼각측량 모델 (v2):
      y_px = y_floor - fy * tan(α) * Δz / D0
      여기서 D0 = z_tcp + delta_z (카메라-대상 실거리)

    y_px ~ z_tcp 선형 회귀로 기울기(slope = dy/dz)를 추정한 뒤,
      slope = -fy * tan(α) / D0
      → D0 = -fy * tan(α) / slope

    계단 깊이(tread)는 detect_step_transitions로 플랫 구간을 찾은 뒤
    인접 플랫 구간의 Z_tcp 평균 차이로 직접 계산한다.
    플랫 구간이 없으면 z_tcp의 총 범위를 n_steps-1로 나눠 근사한다.

    Args:
        y_px_sequence:  각 스캔 위치의 레이저 Y 픽셀 좌표 시퀀스 (px)
        z_tcp_sequence: 대응 Z_tcp 값 시퀀스 (mm)

    Returns:
        추정 계단 깊이 tread (mm)

    Raises:
        ValueError: 입력 시퀀스 길이가 2 미만이거나 불일치할 때
    """
    y_arr = np.asarray(y_px_sequence, dtype=float)
    z_arr = np.asarray(z_tcp_sequence, dtype=float)
    if y_arr.shape != z_arr.shape or len(y_arr) < 2:
        raise ValueError("y_px_sequence와 z_tcp_sequence는 길이 2 이상의 동일 크기 시퀀스여야 합니다.")

    # 플랫 구간 감지 — 각 계단의 Z 중심 추출
    steps = detect_step_transitions(z_arr.tolist(), y_arr.tolist())
    if len(steps) >= 2:
        # 인접 계단 쌍의 Z_tcp 차이 → 중앙값
        z_means = [s["flat_z_mean"] for s in steps]
        treads = [abs(z_means[i + 1] - z_means[i]) for i in range(len(z_means) - 1)]
        return float(np.median(treads))

    # 플랫 구간 감지 실패 시: 전체 Z 범위 / (점 수 - 1) 근사
    z_range = float(z_arr[-1] - z_arr[0])
    n_intervals = max(len(z_arr) - 1, 1)
    return abs(z_range) / n_intervals


def calibrate_sensitivity(scan_data: dict) -> dict:
    """스캔 데이터로부터 sensitivity 선형 모델 파라미터를 피팅한다.

    scan_data 구조:
      {
        "points": [
          {"y_px": float, "height_mm": float},   # 알려진 높이의 계단 측정값
          ...
        ]
      }

    각 점의 sensitivity = (y_base_px - y_px) / height_mm 를 계산하고
    y_px에 대해 선형 회귀한다.

    Args:
        scan_data: 측정 데이터 딕셔너리. "points" 키 필요.
                   각 포인트: {"y_px": float, "height_mm": float,
                               "y_base_px": float (optional)}

    Returns:
        {
          "slope": float,        # SENS_SLOPE (px/mm per px)
          "intercept": float,    # SENS_INTERCEPT (px/mm)
          "r_squared": float,    # 결정계수
          "n_points": int,
          "sens_values": list,   # 각 점의 sensitivity
          "y_values": list,      # 대응 y_px 값
        }

    Raises:
        ValueError: points가 2개 미만이거나 필수 키 누락 시
        ImportError: scipy 미설치 시 (scipy.stats.linregress 사용)
    """
    try:
        from scipy import stats as sp_stats
    except ImportError as exc:
        raise ImportError("calibrate_sensitivity는 scipy가 필요합니다: pip install scipy") from exc

    points = scan_data.get("points", [])
    if len(points) < 2:
        raise ValueError("calibrate_sensitivity: 최소 2개 이상의 포인트가 필요합니다.")

    y_vals: List[float] = []
    sens_vals: List[float] = []

    for i, pt in enumerate(points):
        if "y_px" not in pt or "height_mm" not in pt:
            raise ValueError(f"points[{i}]: 'y_px'와 'height_mm' 키가 필요합니다.")
        y_px = float(pt["y_px"])
        h_mm = float(pt["height_mm"])
        y_base_px = float(pt.get("y_base_px", CY_PX))  # 기본값: 주점
        if h_mm == 0.0:
            continue  # 높이 0은 sensitivity 계산 불가 (바닥 기준점)
        delta_y = y_base_px - y_px
        sens = delta_y / h_mm  # px/mm
        y_vals.append(y_px)
        sens_vals.append(sens)

    if len(y_vals) < 2:
        raise ValueError("유효한 포인트(height_mm != 0)가 2개 미만입니다.")

    y_arr = np.asarray(y_vals)
    s_arr = np.asarray(sens_vals)

    result = sp_stats.linregress(y_arr, s_arr)
    slope = float(result.slope)
    intercept = float(result.intercept)
    r_squared = float(result.rvalue ** 2)

    return {
        "slope": slope,
        "intercept": intercept,
        "r_squared": r_squared,
        "n_points": len(y_vals),
        "sens_values": sens_vals,
        "y_values": y_vals,
    }


def detect_step_transitions(
    z_arr: Sequence[float],
    y_arr: Sequence[float],
    threshold: float = 3.0,
    min_flat_len: int = 3,
) -> List[dict]:
    """Y_px 시퀀스에서 플랫 구간을 감지하고 계단 전이를 판별한다.

    알고리즘:
      1. 인접 점간 |Δy_px|를 계산한다.
      2. |Δy_px| < threshold 인 구간을 플랫(flat)으로 분류한다.
      3. 연속된 플랫 구간을 그룹핑한다 (최소 min_flat_len 포인트).
      4. 인접 플랫 구간 사이를 전이(transition)로 마킹한다.

    Args:
        z_arr:        Z_tcp 좌표 시퀀스 (mm)
        y_arr:        대응 레이저 Y 픽셀 시퀀스 (px)
        threshold:    플랫 판정 임계값 (px, 기본 3.0)
        min_flat_len: 플랫 구간 최소 포인트 수 (기본 3)

    Returns:
        계단 전이 정보 리스트:
        [
          {
            "step_index": int,           # 0-based 계단 번호
            "flat_z_mean": float,        # 플랫 구간 평균 Z_tcp (mm)
            "flat_y_mean": float,        # 플랫 구간 평균 Y_px (px)
            "flat_y_std": float,         # 플랫 구간 Y_px 표준편차 (px)
            "flat_indices": list[int],   # 플랫 구간 인덱스 목록
            "transition_z": float|None,  # 전이 시작 Z_tcp (mm), 마지막 계단은 None
          },
          ...
        ]
    """
    z_a = np.asarray(z_arr, dtype=float)
    y_a = np.asarray(y_arr, dtype=float)
    n = len(y_a)
    if n < 2:
        return []

    # 인접 차분
    dy = np.abs(np.diff(y_a))

    # 플랫 마스크 (인덱스 i는 점 i와 i+1 사이 차분)
    is_flat = dy < threshold  # shape (n-1,)

    # 플랫 구간 그룹핑: 연속 플랫 포인트 집합 구성
    groups: List[List[int]] = []
    current: List[int] = []

    for i in range(n):
        # 점 i가 플랫 구간에 속하는지: 앞 또는 뒤 차분이 플랫이면 포함
        in_flat = False
        if i < n - 1 and is_flat[i]:
            in_flat = True
        if i > 0 and is_flat[i - 1]:
            in_flat = True

        if in_flat:
            current.append(i)
        else:
            if len(current) >= min_flat_len:
                groups.append(current)
            current = []

    if len(current) >= min_flat_len:
        groups.append(current)

    # 전이 정보 구성
    transitions: List[dict] = []
    for step_idx, group in enumerate(groups):
        y_group = y_a[group]
        z_group = z_a[group]
        flat_y_mean = float(np.mean(y_group))
        flat_y_std = float(np.std(y_group))
        flat_z_mean = float(np.mean(z_group))

        # 다음 그룹과의 경계 Z
        transition_z: float | None = None
        if step_idx < len(groups) - 1:
            next_group = groups[step_idx + 1]
            # 현재 그룹 마지막 인덱스와 다음 그룹 첫 인덱스의 중간 Z
            i_end = group[-1]
            i_next_start = next_group[0]
            transition_z = float((z_a[i_end] + z_a[i_next_start]) / 2.0)

        transitions.append({
            "step_index": step_idx,
            "flat_z_mean": flat_z_mean,
            "flat_y_mean": flat_y_mean,
            "flat_y_std": flat_y_std,
            "flat_indices": group,
            "transition_z": transition_z,
        })

    return transitions


# ---------------------------------------------------------------------------
# 셀프 테스트 (직접 실행 시)
# ---------------------------------------------------------------------------

if __name__ == "__main__":
    print("=== laser_triangulation.py 셀프 테스트 ===\n")

    # 1. estimate_height
    y_base = 800.0
    y_meas = 740.0
    h = estimate_height(y_base, y_meas)
    sens_mid = _sensitivity_at((y_base + y_meas) / 2.0)
    print(f"[estimate_height]")
    print(f"  y_base={y_base}px, y_measured={y_meas}px")
    print(f"  sensitivity(y_mid={( y_base+y_meas)/2:.1f}px) = {sens_mid:.4f} px/mm")
    print(f"  → 높이 = {h:.3f} mm  (기대: ~{(y_base-y_meas)/sens_mid:.3f}mm)\n")

    # 2. estimate_depth / detect_step_transitions — 5단 계단 모사 (tread=10mm)
    # 각 계단: 5포인트 플랫 → 전이(y 급변) 1포인트 → 다음 계단
    tread_true = 10.0
    rise_true = 10.0   # sensitivity * rise_true ≈ Δy_px per step
    sens_approx = 2.68  # ~y=770px 기준 sensitivity
    z_scan: List[float] = []
    y_scan: List[float] = []
    np.random.seed(42)
    for step in range(5):
        z_base = 500.0 + step * tread_true
        y_base_step = 800.0 - step * (sens_approx * rise_true)
        for j in range(5):   # 플랫 구간 5포인트
            z_scan.append(z_base + j * 0.4)
            y_scan.append(y_base_step + float(np.random.normal(0, 0.3)))
        if step < 4:         # 전이 구간 2포인트 (y 급변)
            z_mid = z_base + 5 * 0.4 + 1.0
            y_mid = y_base_step - sens_approx * rise_true * 0.5
            z_scan.append(z_mid)
            y_scan.append(y_mid)
            z_scan.append(z_mid + 1.0)
            y_scan.append(y_mid - sens_approx * rise_true * 0.5)

    steps = detect_step_transitions(z_scan, y_scan, threshold=5.0, min_flat_len=3)
    print(f"[detect_step_transitions]")
    print(f"  감지된 계단 수: {len(steps)}  (기대: 5)")
    for s in steps:
        print(f"  step {s['step_index']}: z_mean={s['flat_z_mean']:.1f}mm, "
              f"y_mean={s['flat_y_mean']:.1f}px, y_std={s['flat_y_std']:.2f}px, "
              f"transition_z={s['transition_z']}")

    tread_est = estimate_depth(y_scan, z_scan)
    print(f"\n[estimate_depth]")
    print(f"  tread_true={tread_true}mm → 추정={tread_est:.3f}mm")

    print("\n=== 모든 테스트 완료 ===")
