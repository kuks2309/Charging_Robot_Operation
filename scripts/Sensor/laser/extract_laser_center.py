"""
레이저 띠 중심선 추출 모듈

붉은색 레이저 띠를 RGB 채널 차이 기반으로 분리한 뒤,
각 열(column)마다 intensity 가중 중심(weighted centroid)을 계산하여
서브픽셀 정밀도의 중심선 좌표를 반환한다.
"""

import cv2
import numpy as np
from pathlib import Path


def extract_red_mask(image: np.ndarray,
                     diff_threshold: int = 30,
                     min_red: int = 80,
                     dilate_size: int = 7) -> np.ndarray:
    """RGB 채널 차이 기반으로 붉은색 레이저 영역을 마스크로 추출한다.

    레이저는 단색광(~650nm)이므로 카메라 센서에서 R 채널이 G, B보다
    항상 높다. 과포화(blooming) 영역에서도 R이 먼저 포화되어
    R ≥ G, R ≥ B 관계가 유지된다.

    판별 조건:
        (R - G) > diff_threshold  AND  (R - B) > diff_threshold  AND  R > min_red

    이후 dilation으로 레이저 경계를 확장하여 line mask 필터의 ROI로 사용한다.

    Parameters
    ----------
    image : np.ndarray
        BGR 이미지
    diff_threshold : int
        R과 G/B 간 최소 차이 (기본 30)
    min_red : int
        R 채널 최소 밝기 (기본 80, 어두운 배경 노이즈 제거)
    dilate_size : int
        dilation 커널 크기 (기본 7). 0이면 dilation 생략.
    """
    b, g, r = cv2.split(image)

    r_int = r.astype(np.int16)
    g_int = g.astype(np.int16)
    b_int = b.astype(np.int16)

    mask = ((r_int - g_int) > diff_threshold) & \
           ((r_int - b_int) > diff_threshold) & \
           (r >= min_red)

    mask = mask.astype(np.uint8) * 255

    # 노이즈 제거
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=1)

    # dilation: 레이저 경계를 확장하여 line mask 필터 적용 영역 확보
    if dilate_size > 0:
        dilate_kernel = cv2.getStructuringElement(
            cv2.MORPH_ELLIPSE, (dilate_size, dilate_size))
        mask = cv2.dilate(mask, dilate_kernel, iterations=1)

    return mask


def _find_best_run(col_mask: np.ndarray) -> tuple[int, int] | None:
    """한 열의 마스크에서 가장 긴 연속 런(connected run)을 찾는다.

    Returns:
        (start_row, end_row) 또는 None (런이 없는 경우)
    """
    runs = []
    in_run = False
    start = 0

    for i, v in enumerate(col_mask):
        if v > 0 and not in_run:
            in_run = True
            start = i
        elif v == 0 and in_run:
            in_run = False
            runs.append((start, i - 1))
    if in_run:
        runs.append((start, len(col_mask) - 1))

    if not runs:
        return None

    # 가장 긴 런 선택
    best = max(runs, key=lambda r: r[1] - r[0])
    return best


def estimate_stripe_width(
    mask: np.ndarray,
    min_stripe_width: int = 3,
    max_stripe_width: int = 200,
) -> float:
    """레드 마스크에서 레이저 스트라이프의 대표 두께(median)를 추정한다.

    각 열에서 가장 긴 연속 런의 폭을 수집한 뒤 median을 반환.

    Returns:
        추정된 스트라이프 두께 (픽셀). 검출 실패 시 0.
    """
    h, w = mask.shape
    widths = []

    for col in range(w):
        run = _find_best_run(mask[:, col])
        if run is None:
            continue
        width = run[1] - run[0] + 1
        if min_stripe_width <= width <= max_stripe_width:
            widths.append(width)

    if len(widths) == 0:
        return 0.0

    return float(np.median(widths))


def _make_zero_sum_mask(center_hw: int) -> np.ndarray:
    """Zero-sum matched filter mask 생성 (레이저 스트라이프 검출용).

    구조: [-flank, +center, -flank]
    - Center: 삼각형 프로파일 [1, 2, ..., center_hw+1, ..., 2, 1]
    - Flank: 삼각형 프로파일, 각 측면 center_hw+1 개
    - 합계 = 0 (DC 성분 제거)

    User design basis: [-1,-2,-1, 1,2,3,2,1, -1,-2,-1] (center_hw=2)

    Args:
        center_hw: center 양의 영역 반폭 (>=1)

    Returns:
        1D zero-sum mask (float64)
    """
    # Center: triangular [1, 2, ..., center_hw+1, ..., 2, 1]
    center_size = 2 * center_hw + 1
    center = np.array(
        [min(i + 1, center_size - i) for i in range(center_size)],
        dtype=np.float64,
    )

    # Flank: triangular [1, 2, ..., peak, ..., 2, 1]
    flank_size = center_hw + 1
    flank = np.array(
        [min(i + 1, flank_size - i) for i in range(flank_size)],
        dtype=np.float64,
    )

    # Scale flanks for zero-sum: center_sum == 2 * flank_sum * scale
    center_sum = center.sum()
    flank_sum = flank.sum()
    scale = center_sum / (2.0 * flank_sum)

    return np.concatenate([-flank * scale, center, -flank * scale])


def extract_laser_center_conv(
    image: np.ndarray,
    min_half_width: int = 2,
    max_half_width: int = 20,
    num_scales: int = 10,
    response_ratio: float = 0.3,
) -> tuple[np.ndarray, np.ndarray, float]:
    """Multi-scale zero-sum convolution mask로 레이저 중심 y좌표를 추출.

    각 열(column)의 Red 채널을 수직 방향으로 multi-scale matched filter와
    correlation하여, 최대 응답 위치를 레이저 중심으로 결정한다.

    Args:
        image: BGR 이미지
        min_half_width: 최소 마스크 center half-width (px)
        max_half_width: 최대 마스크 center half-width (px)
        num_scales: 스캔할 스케일 수
        response_ratio: 최대 응답 대비 임계값 비율 (0~1)

    Returns:
        cols: 유효한 열 인덱스 배열
        centers_y: 각 열의 서브픽셀 중심 y좌표 배열
        est_width: 추정된 레이저 띠 두께 (px)
    """
    # RGB 채널 차이 기반 마스크 (dilation 포함)로 레이저 ROI 추출
    red_mask = extract_red_mask(image)
    red = image[:, :, 2].astype(np.float64)
    red = red * (red_mask > 0).astype(np.float64)  # 레이저 외 영역 제거
    h, w = red.shape

    # Generate scale candidates
    half_widths = np.unique(
        np.linspace(min_half_width, max_half_width, num_scales).astype(int)
    )

    # Compute response for each scale, track best per-pixel
    best_response = np.full((h, w), -np.inf, dtype=np.float64)
    best_scale_idx = np.zeros((h, w), dtype=np.int32)

    for i, hw in enumerate(half_widths):
        mask = _make_zero_sum_mask(int(hw))
        mask /= np.linalg.norm(mask)  # L2 정규화: 스케일 간 공정 비교
        kernel = mask.reshape(-1, 1).astype(np.float64)
        response = cv2.filter2D(red, cv2.CV_64F, kernel)

        better = response > best_response
        best_response[better] = response[better]
        best_scale_idx[better] = i

    # For each column: find y-position of max response
    max_y = np.argmax(best_response, axis=0)        # (w,)
    max_val = best_response[max_y, np.arange(w)]    # (w,)

    # Threshold
    global_max = max_val.max() if w > 0 else 0.0
    if global_max <= 0:
        return np.array([]), np.array([]), 0.0
    threshold = response_ratio * global_max

    cols_list = []
    centers_list = []
    widths_list = []

    for col in range(w):
        if max_val[col] < threshold:
            continue

        y = int(max_y[col])

        # Sub-pixel refinement: parabolic interpolation
        if 1 <= y <= h - 2:
            vm1 = best_response[y - 1, col]
            v0  = best_response[y, col]
            vp1 = best_response[y + 1, col]
            denom = 2.0 * (2.0 * v0 - vm1 - vp1)
            if abs(denom) > 1e-10:
                delta = (vm1 - vp1) / denom
                center_y = y + np.clip(delta, -0.5, 0.5)
            else:
                center_y = float(y)
        else:
            center_y = float(y)

        cols_list.append(col)
        centers_list.append(center_y)

        scale_idx = best_scale_idx[y, col]
        widths_list.append(float(2 * half_widths[scale_idx] + 1))

    est_width = float(np.median(widths_list)) if widths_list else 0.0

    return np.array(cols_list), np.array(centers_list), est_width


def extract_laser_center(
    image: np.ndarray,
    min_stripe_width: int = 3,
    max_stripe_width: int = 80,
    width_tolerance: float = 2.0,
) -> tuple[np.ndarray, np.ndarray, np.ndarray, float]:
    """각 열(column)에서 레이저 띠의 서브픽셀 중심 y좌표를 추출한다.

    1단계: 레드 마스크에서 각 열의 연속 런 분석 → 대표 두께 추정 (median)
    2단계: 대표 두께 ± tolerance 범위 내 열만 필터링
    3단계: 필터링된 런 내에서 weighted centroid 계산

    Args:
        image: BGR 이미지
        min_stripe_width: 유효 레이저 띠 최소 폭 (픽셀)
        max_stripe_width: 유효 레이저 띠 최대 폭 (픽셀)
        width_tolerance: 대표 두께 대비 허용 배수 (median * tolerance = 상한)

    Returns:
        cols: 유효한 열 인덱스 배열
        centers_y: 각 열의 서브픽셀 중심 y좌표 배열
        mask: 붉은색 마스크 이미지
        estimated_width: 추정된 레이저 띠 두께 (픽셀)
    """
    mask = extract_red_mask(image)

    # 1단계: 대표 두께 추정
    est_width = estimate_stripe_width(mask, min_stripe_width, max_stripe_width)

    if est_width < min_stripe_width:
        return np.array([]), np.array([]), mask, 0.0

    # 2단계: 두께 필터링 범위
    width_lo = max(min_stripe_width, est_width / width_tolerance)
    width_hi = min(max_stripe_width, est_width * width_tolerance)

    # 레이저 intensity (Red 채널)
    red_channel = image[:, :, 2].astype(np.float64)
    intensity = red_channel * (mask > 0).astype(np.float64)

    h, w = mask.shape
    cols_list = []
    centers_list = []

    for col in range(w):
        run = _find_best_run(mask[:, col])
        if run is None:
            continue

        run_width = run[1] - run[0] + 1

        # 추정 두께 기준 필터링
        if not (width_lo <= run_width <= width_hi):
            continue

        # 런 영역 내에서만 weighted centroid
        r_start, r_end = run
        run_slice = slice(r_start, r_end + 1)
        run_intensity = intensity[run_slice, col]
        run_rows = np.arange(r_start, r_end + 1, dtype=np.float64)

        total_weight = np.sum(run_intensity)
        if total_weight == 0:
            continue

        center_y = np.sum(run_rows * run_intensity) / total_weight

        cols_list.append(col)
        centers_list.append(center_y)

    cols = np.array(cols_list)
    centers_y = np.array(centers_list)

    return cols, centers_y, mask, est_width


def extract_laser_center_vertical(
    image: np.ndarray,
    min_stripe_width: int = 3,
    max_stripe_width: int = 80,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """각 행(row)에서 레이저 띠의 서브픽셀 중심 x좌표를 추출한다.

    수직 방향 레이저 띠용. 각 행을 스캔하여 x-중심을 계산.

    Args:
        image: BGR 이미지
        min_stripe_width: 유효 레이저 띠 최소 폭 (픽셀)
        max_stripe_width: 유효 레이저 띠 최대 폭 (픽셀)

    Returns:
        rows: 유효한 행 인덱스 배열
        centers_x: 각 행의 서브픽셀 중심 x좌표 배열
        mask: 붉은색 마스크 이미지
    """
    mask = extract_red_mask(image)

    red_channel = image[:, :, 2].astype(np.float64)
    intensity = red_channel * (mask > 0).astype(np.float64)

    h, w = mask.shape
    rows_list = []
    centers_list = []

    for row in range(h):
        row_intensity = intensity[row, :]
        nonzero = np.where(row_intensity > 0)[0]

        if len(nonzero) < min_stripe_width:
            continue

        stripe_width = nonzero[-1] - nonzero[0] + 1
        if stripe_width > max_stripe_width:
            continue

        # 가중 중심 (weighted centroid)
        weights = row_intensity[nonzero]
        center_x = np.sum(nonzero * weights) / np.sum(weights)

        rows_list.append(row)
        centers_list.append(center_x)

    rows = np.array(rows_list)
    centers_x = np.array(centers_list)

    return rows, centers_x, mask


def fit_laser_line(
    cols: np.ndarray,
    centers_y: np.ndarray,
    degree: int = 1,
    mad_scale: float = 3.0,
    max_iterations: int = 3,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Median 기반 반복 outlier 제거 후 다항식 피팅.

    초기 추정에 median을 사용하여 극단 outlier에 강건하다.

    Args:
        cols: x좌표 배열
        centers_y: y좌표 배열
        degree: 다항식 차수 (1=직선, 2=2차)
        mad_scale: MAD(median absolute deviation) 기반 임계값 배수
        max_iterations: 반복 피팅 횟수

    Returns:
        coeffs: 다항식 계수
        inlier_cols: inlier x좌표
        inlier_y: inlier y좌표
    """
    if len(cols) < degree + 1:
        return np.array([]), cols, centers_y

    current_cols = cols.copy()
    current_y = centers_y.copy()

    for _ in range(max_iterations):
        # 1단계: median 기반 초기 필터 (첫 반복) 또는 polyfit (이후)
        if len(current_cols) < degree + 1:
            break

        coeffs = np.polyfit(current_cols, current_y, degree)
        fitted = np.polyval(coeffs, current_cols)
        residuals = np.abs(current_y - fitted)

        # MAD 기반 적응형 임계값
        med_residual = np.median(residuals)
        mad = np.median(np.abs(residuals - med_residual))
        threshold = max(med_residual + mad_scale * max(mad, 1.0), 3.0)

        inlier_mask = residuals < threshold
        new_cols = current_cols[inlier_mask]
        new_y = current_y[inlier_mask]

        # 수렴 확인
        if len(new_cols) == len(current_cols):
            break

        current_cols = new_cols
        current_y = new_y

    # 최종 피팅
    if len(current_cols) >= degree + 1:
        coeffs = np.polyfit(current_cols, current_y, degree)

    return coeffs, current_cols, current_y


def fit_multiple_lines_ransac(
    cols: np.ndarray,
    centers_y: np.ndarray,
    min_inliers: int = 20,
    residual_threshold: float = 2.0,
    max_lines: int = 5,
    ransac_iterations: int = 200,
) -> list[dict]:
    """Laser center points에서 여러 직선을 반복 RANSAC으로 추출.

    점 집합에서 RANSAC을 반복 적용하여 직선을 순차적으로 추출한다.
    각 반복에서 발견된 inlier를 제거한 뒤 다음 직선을 찾는다.

    Args:
        cols: x좌표 배열
        centers_y: y좌표 배열
        min_inliers: 직선으로 인정할 최소 inlier 수
        residual_threshold: RANSAC inlier 판정 임계값 (px)
        max_lines: 최대 직선 수
        ransac_iterations: RANSAC 반복 횟수

    Returns:
        list of dict (x_range 기준 정렬), each with:
            - coeffs: [slope, intercept] (np.polyfit degree=1)
            - inlier_cols: inlier x좌표 배열
            - inlier_y: inlier y좌표 배열
            - angle_deg: 기울기 (도)
            - x_range: (x_min, x_max)
    """
    if len(cols) < min_inliers:
        return []

    rng = np.random.RandomState(42)
    remaining_cols = cols.copy()
    remaining_y = centers_y.copy()
    lines = []

    for _ in range(max_lines):
        if len(remaining_cols) < min_inliers:
            break

        best_inlier_mask = None
        best_inlier_count = 0

        for _ in range(ransac_iterations):
            idx = rng.choice(len(remaining_cols), 2, replace=False)
            x1, y1 = remaining_cols[idx[0]], remaining_y[idx[0]]
            x2, y2 = remaining_cols[idx[1]], remaining_y[idx[1]]

            if abs(x2 - x1) < 1:
                continue

            slope = (y2 - y1) / (x2 - x1)
            intercept = y1 - slope * x1

            fitted = slope * remaining_cols + intercept
            residuals = np.abs(remaining_y - fitted)
            inlier_mask = residuals < residual_threshold
            inlier_count = np.sum(inlier_mask)

            if inlier_count > best_inlier_count:
                best_inlier_count = inlier_count
                best_inlier_mask = inlier_mask

        if best_inlier_mask is None or best_inlier_count < min_inliers:
            break

        inlier_cols = remaining_cols[best_inlier_mask]
        inlier_y = remaining_y[best_inlier_mask]

        coeffs = np.polyfit(inlier_cols, inlier_y, 1)
        angle_deg = float(np.degrees(np.arctan(coeffs[0])))

        lines.append({
            'coeffs': coeffs,
            'inlier_cols': inlier_cols,
            'inlier_y': inlier_y,
            'angle_deg': angle_deg,
            'x_range': (float(inlier_cols.min()), float(inlier_cols.max())),
        })

        remaining_cols = remaining_cols[~best_inlier_mask]
        remaining_y = remaining_y[~best_inlier_mask]

    lines.sort(key=lambda l: l['x_range'][0])
    return lines


def fit_multiple_lines_hough(
    cols: np.ndarray,
    centers_y: np.ndarray,
    min_inliers: int = 20,
    residual_threshold: float = 2.0,
    max_lines: int = 5,
    rho_resolution: float = 1.0,
    theta_resolution_deg: float = 1.0,
    gap_threshold: float = 10.0,
) -> list[dict]:
    """Hough 변환으로 (x, y) 점 좌표에서 여러 직선을 추출.

    각 점 (x, y)에서 accumulator에 직접 투표하여 직선을 검출한다.
    래스터화 없이 sub-pixel 좌표를 그대로 사용하므로 정밀도 손실이 없다.
    검출된 직선은 least-squares refit으로 최종 계수를 구한다.

    알고리즘:
        1. θ grid 생성 (-90° ~ +90°)
        2. 각 점마다 모든 θ에 대해 ρ = x·cosθ + y·sinθ 계산
        3. (ρ, θ) accumulator에 투표
        4. 최대 peak의 (ρ, θ) 직선에 대해 inlier 수집
        5. least-squares refit
        6. inlier 제거 후 반복

    Args:
        cols: x좌표 배열
        centers_y: y좌표 배열
        min_inliers: 직선으로 인정할 최소 inlier 수
        residual_threshold: inlier 판정 거리 임계값 (px)
        max_lines: 최대 직선 수
        rho_resolution: ρ 양자화 해상도 (px)
        theta_resolution_deg: θ 양자화 해상도 (도)

    Returns:
        list of dict (x_range 기준 정렬), each with:
            - coeffs: [slope, intercept] (np.polyfit degree=1)
            - inlier_cols: inlier x좌표 배열
            - inlier_y: inlier y좌표 배열
            - angle_deg: 기울기 (도)
            - x_range: (x_min, x_max)
    """
    if len(cols) < min_inliers:
        return []

    # θ grid: -90° ~ +90° (step = theta_resolution_deg)
    theta_res_rad = np.radians(theta_resolution_deg)
    thetas = np.arange(-np.pi / 2, np.pi / 2, theta_res_rad)
    cos_t = np.cos(thetas)
    sin_t = np.sin(thetas)
    n_theta = len(thetas)

    remaining_cols = cols.astype(np.float64).copy()
    remaining_y = centers_y.astype(np.float64).copy()
    lines = []

    for _ in range(max_lines):
        n_pts = len(remaining_cols)
        if n_pts < min_inliers:
            break

        # ρ 계산: BLAS 행렬곱 (broadcasting 대비 ~2x 빠름)
        # (N, 2) @ (2, T) → (N, T)
        points = np.column_stack([remaining_cols, remaining_y])
        trig = np.vstack([cos_t, sin_t])  # (2, T)
        rhos = points @ trig  # ρ_ij = x_i·cos(θ_j) + y_i·sin(θ_j)

        # bincount로 accumulator 투표 (정수 인덱싱, histogram2d 대비 ~3x 빠름)
        rho_min = rhos.min()
        n_rho = int(np.ceil((rhos.max() - rho_min) / rho_resolution)) + 1
        rho_idx = ((rhos - rho_min) / rho_resolution).astype(np.intp)
        np.clip(rho_idx, 0, n_rho - 1, out=rho_idx)

        theta_idx = np.broadcast_to(
            np.arange(n_theta, dtype=np.intp), rhos.shape
        )
        linear_idx = rho_idx.ravel() * n_theta + theta_idx.ravel()
        accumulator = np.bincount(
            linear_idx, minlength=n_rho * n_theta
        ).reshape(n_rho, n_theta)

        # 최대 peak → (ρ, θ) 추출
        ri, ti = np.unravel_index(np.argmax(accumulator), accumulator.shape)
        peak_rho = rho_min + (ri + 0.5) * rho_resolution  # bin 중심
        peak_theta = thetas[ti]

        # Inlier: 점-직선 거리 < threshold
        # 직선 방정식: x·cos(θ) + y·sin(θ) = ρ
        distances = np.abs(
            remaining_cols * np.cos(peak_theta) +
            remaining_y * np.sin(peak_theta) - peak_rho
        )
        inlier_mask = distances < residual_threshold

        if np.sum(inlier_mask) < min_inliers:
            break

        inlier_cols = remaining_cols[inlier_mask]
        inlier_y = remaining_y[inlier_mask]

        # 갭 기준으로 세그먼트 분리 — 물리적으로 떨어진 구간은 별도 라인
        sort_idx = np.argsort(inlier_cols)
        sorted_cols = inlier_cols[sort_idx]
        sorted_y = inlier_y[sort_idx]
        gap_positions = np.where(np.diff(sorted_cols) > gap_threshold)[0]
        col_segments = np.split(sorted_cols, gap_positions + 1)
        y_segments = np.split(sorted_y, gap_positions + 1)

        for seg_cols, seg_y in zip(col_segments, y_segments):
            if len(seg_cols) < min_inliers:
                continue
            # Least-squares refit → sub-pixel 정밀도 최종 계수
            coeffs = np.polyfit(seg_cols, seg_y, 1)
            angle_deg = float(np.degrees(np.arctan(coeffs[0])))
            lines.append({
                'coeffs': coeffs,
                'inlier_cols': seg_cols,
                'inlier_y': seg_y,
                'angle_deg': angle_deg,
                'x_range': (float(seg_cols.min()), float(seg_cols.max())),
            })

        remaining_cols = remaining_cols[~inlier_mask]
        remaining_y = remaining_y[~inlier_mask]

    lines.sort(key=lambda l: l['x_range'][0])
    return lines


def fit_multiple_lines(
    cols: np.ndarray,
    centers_y: np.ndarray,
    min_inliers: int = 20,
    residual_threshold: float = 2.0,
    max_lines: int = 5,
    **kwargs,
) -> list[dict]:
    """다중 직선 추출 래퍼. Hough 변환 사용.

    RANSAC 비교 시 아래 주석을 전환하면 됨.
    """
    # --- Hough 방식 (현재 활성) ---
    return fit_multiple_lines_hough(
        cols, centers_y,
        min_inliers=min_inliers,
        residual_threshold=residual_threshold,
        max_lines=max_lines,
        **kwargs,
    )
    # --- RANSAC 방식 (비교용) ---
    # return fit_multiple_lines_ransac(
    #     cols, centers_y,
    #     min_inliers=min_inliers,
    #     residual_threshold=residual_threshold,
    #     max_lines=max_lines,
    # )


def detect_aruco_markers(image: np.ndarray) -> list[np.ndarray]:
    """ArUco 마커를 검출하여 코너 리스트를 반환한다.

    Returns:
        corners 리스트. 각 원소는 (4, 2) 배열 (코너 좌표).
        검출 실패 시 빈 리스트.
    """
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_50)
    params = cv2.aruco.DetectorParameters()
    params.minMarkerPerimeterRate = 0.005
    params.adaptiveThreshWinSizeMin = 3
    params.adaptiveThreshWinSizeMax = 53
    params.adaptiveThreshWinSizeStep = 4
    params.adaptiveThreshConstant = 7
    params.minCornerDistanceRate = 0.01
    params.minDistanceToBorder = 1
    params.polygonalApproxAccuracyRate = 0.08
    detector = cv2.aruco.ArucoDetector(aruco_dict, params)

    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY) if len(image.shape) == 3 else image
    corners, ids, _ = detector.detectMarkers(gray)

    if ids is None:
        return []
    return [c.reshape(-1, 2) for c in corners]


def aruco_roi_mask(
    image_shape: tuple[int, int],
    aruco_corners_list: list[np.ndarray],
    margin_ratio: float = 0.5,
) -> np.ndarray:
    """ArUco 마커 주변 ROI 마스크를 생성한다.

    각 마커의 bounding box를 margin_ratio만큼 확장한 영역을 합쳐서
    전체 ROI 마스크를 반환한다.

    Args:
        image_shape: (H, W)
        aruco_corners_list: 각 마커의 (4,2) 코너 좌표 리스트
        margin_ratio: 마커 크기 대비 확장 비율

    Returns:
        roi_mask: (H, W) uint8 마스크 (255=ROI)
    """
    h, w = image_shape[:2]
    roi_mask = np.zeros((h, w), dtype=np.uint8)

    for corners in aruco_corners_list:
        x_min, y_min = corners.min(axis=0)
        x_max, y_max = corners.max(axis=0)
        marker_w = x_max - x_min
        marker_h = y_max - y_min
        mx = marker_w * margin_ratio
        my = marker_h * margin_ratio

        x1 = max(0, int(x_min - mx))
        y1 = max(0, int(y_min - my))
        x2 = min(w, int(x_max + mx))
        y2 = min(h, int(y_max + my))

        roi_mask[y1:y2, x1:x2] = 255

    return roi_mask


def extract_laser_center_with_aruco(
    image: np.ndarray,
    margin_ratio: float = 0.5,
    min_stripe_width: int = 3,
    max_stripe_width: int = 80,
) -> tuple[np.ndarray, np.ndarray, np.ndarray, list[np.ndarray], np.ndarray]:
    """ArUco 마커 ROI 내에서 레이저 띠 중심을 추출한다.

    1. ArUco 마커 검출
    2. 마커 주변 ROI 마스크 생성
    3. ROI 내에서만 붉은색 레이저 중심 추출

    Args:
        image: BGR 이미지
        margin_ratio: 마커 크기 대비 ROI 확장 비율
        min_stripe_width: 유효 레이저 띠 최소 폭
        max_stripe_width: 유효 레이저 띠 최대 폭

    Returns:
        cols: 유효 열 인덱스 (원본 이미지 좌표)
        centers_y: 서브픽셀 중심 y좌표 (원본 이미지 좌표)
        red_mask: 붉은색 마스크 (ROI 적용 후)
        aruco_corners_list: ArUco 코너 리스트
        roi_mask: ROI 마스크
    """
    aruco_corners_list = detect_aruco_markers(image)
    if not aruco_corners_list:
        print("ArUco 마커를 검출하지 못했습니다. 전체 이미지에서 검출합니다.")
        cols, centers_y, mask, _ = extract_laser_center(image, min_stripe_width, max_stripe_width)
        return cols, centers_y, mask, [], np.ones(image.shape[:2], dtype=np.uint8) * 255

    roi_mask = aruco_roi_mask(image.shape[:2], aruco_corners_list, margin_ratio)

    # 붉은색 마스크에 ROI 적용
    red_mask = extract_red_mask(image)
    red_mask = cv2.bitwise_and(red_mask, roi_mask)

    # ROI 마스크 적용된 intensity로 가중 중심 계산
    red_channel = image[:, :, 2].astype(np.float64)
    intensity = red_channel * (red_mask > 0).astype(np.float64)

    h, w = red_mask.shape
    cols_list = []
    centers_list = []

    for col in range(w):
        col_intensity = intensity[:, col]
        nonzero = np.where(col_intensity > 0)[0]

        if len(nonzero) < min_stripe_width:
            continue

        stripe_width = nonzero[-1] - nonzero[0] + 1
        if stripe_width > max_stripe_width:
            continue

        weights = col_intensity[nonzero]
        center_y = np.sum(nonzero * weights) / np.sum(weights)

        cols_list.append(col)
        centers_list.append(center_y)

    cols = np.array(cols_list)
    centers_y = np.array(centers_list)

    return cols, centers_y, red_mask, aruco_corners_list, roi_mask


def visualize(
    image: np.ndarray,
    cols: np.ndarray,
    centers_y: np.ndarray,
    mask: np.ndarray,
    coeffs: np.ndarray | None = None,
    aruco_corners_list: list[np.ndarray] | None = None,
    roi_mask: np.ndarray | None = None,
) -> None:
    """추출 결과를 matplotlib으로 시각화한다."""
    import matplotlib.pyplot as plt
    import matplotlib.patches as patches

    n_plots = 3 if roi_mask is not None else 2
    fig, axes = plt.subplots(1, n_plots, figsize=(6 * n_plots, 5))

    # 원본 + ArUco ROI + 중심점
    rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    axes[0].imshow(rgb)

    # ArUco ROI 사각형 표시
    if aruco_corners_list:
        for corners in aruco_corners_list:
            x_min, y_min = corners.min(axis=0)
            x_max, y_max = corners.max(axis=0)
            marker_w = x_max - x_min
            marker_h = y_max - y_min
            rect = patches.Rectangle(
                (x_min - marker_w * 0.5, y_min - marker_h * 0.5),
                marker_w * 2, marker_h * 2,
                linewidth=2, edgecolor="red", facecolor="none", linestyle="--",
                label="ArUco ROI",
            )
            axes[0].add_patch(rect)

    if len(cols) > 0:
        axes[0].scatter(cols, centers_y, s=1, c="lime", label="center")
    if coeffs is not None and len(coeffs) > 0 and len(cols) > 0:
        x_fit = np.arange(cols.min(), cols.max() + 1)
        y_fit = np.polyval(coeffs, x_fit)
        axes[0].plot(x_fit, y_fit, "y-", linewidth=1, label="fit")
    axes[0].legend(loc="upper right")
    axes[0].set_title("Laser Center Extraction")

    # 마스크 (ROI 적용 후)
    axes[1].imshow(mask, cmap="gray")
    axes[1].set_title("Red Mask (ROI)")

    # ROI 마스크
    if roi_mask is not None:
        axes[2].imshow(roi_mask, cmap="gray")
        axes[2].set_title("ArUco ROI Mask")

    plt.tight_layout()
    plt.show()


def process_image(
    image_path: str | Path,
    visualize_result: bool = True,
    use_aruco: bool = False,
    margin_ratio: float = 0.5,
) -> dict:
    """이미지 파일에서 레이저 중심선을 추출한다.

    Args:
        image_path: 이미지 파일 경로
        visualize_result: 시각화 여부
        use_aruco: ArUco 마커 기반 ROI 사용 여부
        margin_ratio: ArUco ROI 확장 비율

    Returns:
        dict with keys:
            - cols: 유효 열 인덱스
            - centers_y: 서브픽셀 중심 y좌표
            - coeffs: 피팅 다항식 계수
            - angle_deg: 레이저 띠 기울기 (도)
            - aruco_count: 검출된 ArUco 마커 수 (use_aruco=True일 때)
    """
    image = cv2.imread(str(image_path))
    if image is None:
        raise FileNotFoundError(f"이미지를 읽을 수 없습니다: {image_path}")

    aruco_corners_list = []
    roi_mask = None

    if use_aruco:
        cols, centers_y, mask, aruco_corners_list, roi_mask = \
            extract_laser_center_with_aruco(image, margin_ratio)
        print(f"ArUco 마커 {len(aruco_corners_list)}개 검출")
    else:
        cols, centers_y, mask, _ = extract_laser_center(image)

    if len(cols) == 0:
        print("레이저 띠를 검출하지 못했습니다.")
        return {"cols": cols, "centers_y": centers_y, "coeffs": None, "angle_deg": None}

    coeffs, inlier_cols, inlier_y = fit_laser_line(cols, centers_y)

    # 기울기 (도) 계산 — coeffs[0]은 dy/dx
    angle_deg = None
    if len(coeffs) >= 2:
        angle_deg = np.degrees(np.arctan(coeffs[0]))

    print(f"검출 포인트 수: {len(cols)} → inlier: {len(inlier_cols)}")
    print(f"레이저 띠 기울기: {angle_deg:.2f}°" if angle_deg is not None else "기울기 계산 불가")
    if len(inlier_y) > 0:
        print(f"Y좌표 범위: {inlier_y.min():.1f} ~ {inlier_y.max():.1f} px")

    if visualize_result:
        visualize(image, inlier_cols, inlier_y, mask, coeffs, aruco_corners_list, roi_mask)

    return {
        "cols": inlier_cols,
        "centers_y": inlier_y,
        "coeffs": coeffs,
        "angle_deg": angle_deg,
        "aruco_count": len(aruco_corners_list),
    }


if __name__ == "__main__":
    import argparse

    DEFAULT_IMAGE = Path("~/Project/Charging_Robot_Operation/data/aruco/vision_20260214_122957.png").expanduser()

    parser = argparse.ArgumentParser(description="레이저 띠 중심선 추출")
    parser.add_argument(
        "image",
        nargs="?",
        default=str(DEFAULT_IMAGE),
        help="이미지 파일 경로",
    )
    parser.add_argument("--no-vis", action="store_true", help="시각화 비활성화")
    parser.add_argument("--aruco", action="store_true", help="ArUco 마커 기반 ROI 사용")
    parser.add_argument("--margin", type=float, default=0.5, help="ArUco ROI 확장 비율 (기본: 0.5)")
    args = parser.parse_args()

    result = process_image(
        args.image,
        visualize_result=not args.no_vis,
        use_aruco=args.aruco,
        margin_ratio=args.margin,
    )
