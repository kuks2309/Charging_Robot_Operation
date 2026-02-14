"""
레이저 띠 중심선 추출 모듈

붉은색 레이저 띠를 HSV 색공간에서 분리한 뒤,
각 열(column)마다 intensity 가중 중심(weighted centroid)을 계산하여
서브픽셀 정밀도의 중심선 좌표를 반환한다.
"""

import cv2
import numpy as np
from pathlib import Path


def extract_red_mask(image: np.ndarray) -> np.ndarray:
    """HSV 색공간에서 붉은색 레이저 영역을 마스크로 추출한다.

    빨간색은 HSV에서 H=0 부근과 H=170~180 부근 두 구간에 걸쳐 있으므로
    두 범위를 OR 결합한다.
    """
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    # 빨간색 범위 1: H 0~10
    lower1 = np.array([0, 50, 50])
    upper1 = np.array([10, 255, 255])
    mask1 = cv2.inRange(hsv, lower1, upper1)

    # 빨간색 범위 2: H 170~180
    lower2 = np.array([170, 50, 50])
    upper2 = np.array([180, 255, 255])
    mask2 = cv2.inRange(hsv, lower2, upper2)

    mask = cv2.bitwise_or(mask1, mask2)

    # 노이즈 제거
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=1)

    return mask


def extract_laser_center(
    image: np.ndarray,
    min_stripe_width: int = 3,
    max_stripe_width: int = 80,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """각 열(column)에서 레이저 띠의 서브픽셀 중심 y좌표를 추출한다.

    Args:
        image: BGR 이미지
        min_stripe_width: 유효 레이저 띠 최소 폭 (픽셀)
        max_stripe_width: 유효 레이저 띠 최대 폭 (픽셀)

    Returns:
        cols: 유효한 열 인덱스 배열
        centers_y: 각 열의 서브픽셀 중심 y좌표 배열
        mask: 붉은색 마스크 이미지
    """
    mask = extract_red_mask(image)

    # 레이저 intensity로 가중 중심 계산 (Red 채널 사용)
    red_channel = image[:, :, 2].astype(np.float64)
    intensity = red_channel * (mask > 0).astype(np.float64)

    h, w = mask.shape
    row_indices = np.arange(h, dtype=np.float64).reshape(-1, 1)  # (H, 1)

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

        # 가중 중심 (weighted centroid)
        weights = col_intensity[nonzero]
        center_y = np.sum(nonzero * weights) / np.sum(weights)

        cols_list.append(col)
        centers_list.append(center_y)

    cols = np.array(cols_list)
    centers_y = np.array(centers_list)

    return cols, centers_y, mask


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
        cols, centers_y, mask = extract_laser_center(image, min_stripe_width, max_stripe_width)
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
        cols, centers_y, mask = extract_laser_center(image)

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
