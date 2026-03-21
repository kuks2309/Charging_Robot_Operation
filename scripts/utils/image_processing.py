"""
공통 이미지 처리 유틸리티.
cv2 알고리즘 호출을 탭에서 분리하기 위한 래퍼 함수 모음.
"""
import numpy as np
import cv2


def undistort_frame(frame: np.ndarray,
                    camera_matrix: np.ndarray | None,
                    dist_coeffs: np.ndarray | None) -> np.ndarray:
    """프레임 왜곡 보정.

    camera_matrix 또는 dist_coeffs가 None이면 frame의 복사본을 반환.
    유효한 intrinsics가 있으면 cv2.undistort 결과를 반환.
    """
    if camera_matrix is None or dist_coeffs is None:
        return frame.copy()
    return cv2.undistort(frame, camera_matrix, dist_coeffs)


def rvec_to_euler_deg(rvec: np.ndarray) -> tuple[float, float, float]:
    """Rotation vector -> Euler angles (Rx, Ry, Rz) in degrees.

    ZYX convention (roll-pitch-yaw).
    Returns (rx_deg, ry_deg, rz_deg).
    """
    R, _ = cv2.Rodrigues(rvec)
    sy = np.sqrt(R[0, 0] ** 2 + R[1, 0] ** 2)
    singular = sy < 1e-6

    if not singular:
        rx = np.degrees(np.arctan2(R[2, 1], R[2, 2]))
        ry = np.degrees(np.arctan2(-R[2, 0], sy))
        rz = np.degrees(np.arctan2(R[1, 0], R[0, 0]))
    else:
        rx = np.degrees(np.arctan2(-R[1, 2], R[1, 1]))
        ry = np.degrees(np.arctan2(-R[2, 0], sy))
        rz = 0.0

    return float(rx), float(ry), float(rz)


# ──────────────────────────────────────────────────────
# ROI overlay — 공용 (모든 탭에서 사용 가능)
# ──────────────────────────────────────────────────────

def _roi_params(frame, roi_cfg: dict):
    h, w = frame.shape[:2]
    img_cx = w // 2
    if "center_x_px" in roi_cfg:
        cx_abs = int(roi_cfg["center_x_px"])
        offset = abs(cx_abs - img_cx)
    else:
        offset = roi_cfg.get("center_x_offset_px", 0)
    return (
        img_cx,
        offset,
        roi_cfg.get("width_px", 100) // 2,
        int(roi_cfg["center_y_px"]) if "center_y_px" in roi_cfg else h // 2 + int(roi_cfg.get("center_y_offset_px", 0)),
        roi_cfg.get("height_px", 100) // 2,
        tuple(int(c) for c in roi_cfg.get("color", [255, 0, 0])),
        roi_cfg.get("thickness", 2),
    )


def draw_roi_box(frame: np.ndarray,
                 x0: int, y0: int, x1: int, y1: int,
                 color: tuple, thickness: int, *,
                 alpha: float = 0.0,
                 min_thickness: int = 0,
                 label: str | None = None) -> None:
    """좌표 직접 지정 ROI 박스 렌더링. in-place.

    Args:
        alpha: 0.0 이면 단순 경계선. > 0 이면 addWeighted로 반투명 fill 추가.
        min_thickness: 0 이면 무시. > 0 이면 max(thickness, min_thickness) 적용.
        label: None 이면 무시. 문자열이면 박스 좌상단에 putText.
    """
    h, w = frame.shape[:2]
    x0 = max(0, x0); y0 = max(0, y0)
    x1 = min(w, x1); y1 = min(h, y1)
    if x1 <= x0 or y1 <= y0:
        return
    t = max(thickness, min_thickness) if min_thickness > 0 else thickness
    if alpha > 0.0:
        overlay = frame.copy()
        cv2.rectangle(overlay, (x0, y0), (x1, y1), color, -1)
        cv2.addWeighted(overlay, alpha, frame, 1.0 - alpha, 0, frame)
    cv2.rectangle(frame, (x0, y0), (x1, y1), color, t)
    if label:
        lx = x0 + 4
        ly = max(y0 + 20, 20)
        cv2.putText(frame, label, (lx, ly),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, 2)


def draw_roi_single(frame: np.ndarray, roi_cfg: dict) -> None:
    """img_cx + center_x_offset_px (또는 center_x_px 절대좌표) 위치에 단일 ROI 박스를 그린다. in-place."""
    if not roi_cfg:
        return
    img_cx, offset, hw, cy, hh, color, thickness = _roi_params(frame, roi_cfg)
    h, w = frame.shape[:2]
    cx = img_cx + offset
    draw_roi_box(frame, cx - hw, cy - hh, cx + hw, cy + hh, color, thickness)


def draw_roi_symmetric(frame: np.ndarray, roi_cfg: dict) -> None:
    """img_cx ± center_x_offset_px 위치에 좌우 대칭 ROI 박스 2개를 그린다. in-place."""
    if not roi_cfg:
        return
    img_cx, offset, hw, cy, hh, color, thickness = _roi_params(frame, roi_cfg)
    for cx in (img_cx - offset, img_cx + offset):
        draw_roi_box(frame, cx - hw, cy - hh, cx + hw, cy + hh, color, thickness)


def draw_laser_calib_roi(frame: np.ndarray, roi_cfg: dict) -> None:
    """roi_cfg의 mode 필드에 따라 single/symmetric을 선택해 그린다.

    mode: "single"    → draw_roi_single
    mode: "symmetric" → draw_roi_symmetric  (기본값)
    """
    if not roi_cfg:
        return
    if roi_cfg.get("mode", "symmetric") == "single":
        draw_roi_single(frame, roi_cfg)
    else:
        draw_roi_symmetric(frame, roi_cfg)


def compute_calib_roi_rects(frame_w: int, frame_h: int,
                            roi_cfg: dict) -> list[tuple[int, int, int, int]]:
    """calib 포맷 ROI dict -> [(x0, y0, x1, y1), ...] 목록.

    roi_cfg: inner roi dict (outer JSON wrapper 제외).
             키: center_x_px (또는 center_x_offset_px), width_px, center_y_px (또는 center_y_offset_px), height_px, mode
    frame_w, frame_h: 프레임 크기 (이미지 중심 계산에 사용)

    Returns:
        mode="symmetric" (기본) -> 2개 rect: [left, right]
        mode="single"           -> 1개 rect: [center+offset]
        각 rect는 (x0, y0, x1, y1), 프레임 경계 [0,frame_w] x [0,frame_h] 클램핑
    """
    img_cx = frame_w // 2
    if "center_x_px" in roi_cfg:
        cx_abs = int(roi_cfg["center_x_px"])
        offset = abs(cx_abs - img_cx)
    else:
        offset = roi_cfg.get("center_x_offset_px", 0)
    hw     = roi_cfg.get("width_px", 100) // 2
    cy     = int(roi_cfg["center_y_px"]) if "center_y_px" in roi_cfg else frame_h // 2 + int(roi_cfg.get("center_y_offset_px", 0))
    hh     = roi_cfg.get("height_px", 100) // 2
    mode   = roi_cfg.get("mode", "symmetric")

    def _clamp(x0, y0, x1, y1):
        return (max(0, x0), max(0, y0), min(frame_w, x1), min(frame_h, y1))

    if mode == "single":
        cx_center = int(roi_cfg["center_x_px"]) if "center_x_px" in roi_cfg else img_cx + offset
        return [_clamp(cx_center - hw, cy - hh, cx_center + hw, cy + hh)]
    else:  # symmetric
        left_cx  = img_cx - offset
        right_cx = img_cx + offset
        return [
            _clamp(left_cx  - hw, cy - hh, left_cx  + hw, cy + hh),
            _clamp(right_cx - hw, cy - hh, right_cx + hw, cy + hh),
        ]


def compute_roi_rects(frame_w: int, frame_h: int,
                      roi_cfg: dict) -> list[tuple[int, int, int, int]]:
    """통일 ROI 스키마로 좌/우(또는 단일) ROI 좌표 계산.

    스키마 키: center_x_px (또는 center_x_offset_px), width_px, center_y_px (또는 center_y_offset_px), height_px, mode
    mode='symmetric' (기본): 좌/우 2개 반환
    mode='single': 오른쪽(양수 offset 방향) 1개 반환
    height_px <= 0: 전체 프레임 높이 (y0=0, y1=frame_h)
    반환: [(lx0, y0, lx1, y1), (rx0, y0, rx1, y1)]  또는 단일 rect
    """
    img_cx = frame_w // 2
    cx_center = int(roi_cfg['center_x_px']) if 'center_x_px' in roi_cfg else img_cx + int(roi_cfg.get('center_x_offset_px', 0))
    off = abs(cx_center - img_cx)
    hw = int(roi_cfg.get('width_px', 100)) // 2
    cy = int(roi_cfg['center_y_px']) if 'center_y_px' in roi_cfg else frame_h // 2 + int(roi_cfg.get('center_y_offset_px', 0))
    h = int(roi_cfg.get('height_px', 0))
    if h <= 0:
        y0, y1 = 0, frame_h
    else:
        hh = h // 2
        y0 = max(0, cy - hh)
        y1 = min(frame_h, cy + hh)
    mode = roi_cfg.get('mode', 'symmetric')
    if mode == 'single':
        rx0 = max(0, cx_center - hw)
        rx1 = min(frame_w, cx_center + hw)
        return [(rx0, y0, rx1, y1)]
    else:
        lx0 = max(0, img_cx - off - hw)
        lx1 = min(frame_w, img_cx - off + hw)
        rx0 = max(0, img_cx + off - hw)
        rx1 = min(frame_w, img_cx + off + hw)
        return [(lx0, y0, lx1, y1), (rx0, y0, rx1, y1)]


def compute_scan_roi_rects(frame_w: int, frame_h: int,
                           roi_cfg: dict) -> list[tuple[int, int, int, int]]:
    """Deprecated: use compute_roi_rects()."""
    return compute_roi_rects(frame_w, frame_h, roi_cfg)


def draw_laser_fit_line(frame: np.ndarray,
                        coeffs: np.ndarray,
                        inlier_cols: np.ndarray,
                        color: tuple,
                        thickness: int = 2) -> None:
    """레이저 피팅 직선을 inlier_cols 범위에 그린다. in-place.

    Args:
        coeffs: np.polyfit 결과 (최소 2개 원소, 1차: [slope, intercept])
        inlier_cols: 유효 X 픽셀 배열. 빈 배열이면 아무것도 그리지 않음.
        color: BGR 색상 tuple
        thickness: 선 두께 (기본 2)
    """
    if len(coeffs) < 2 or len(inlier_cols) == 0:
        return
    x0, x1 = int(inlier_cols.min()), int(inlier_cols.max())
    y0 = int(round(np.polyval(coeffs, x0)))
    y1 = int(round(np.polyval(coeffs, x1)))
    cv2.line(frame, (x0, y0), (x1, y1), color, thickness)


def draw_horizontal_reference_line(frame: np.ndarray,
                                   y_px: int,
                                   color: tuple,
                                   thickness: int = 2,
                                   *,
                                   alpha: float = 0.0) -> None:
    """이미지 전폭에 걸쳐 수평 기준선을 그린다. in-place.

    Args:
        y_px: 기준선 Y 좌표 (픽셀). 프레임 경계로 클램핑됨.
        color: BGR 색상 tuple
        thickness: 선 두께 (기본 2)
        alpha: 0.0이면 단순 선. > 0이면 addWeighted로 반투명 처리.
    """
    h, w = frame.shape[:2]
    y = max(0, min(h - 1, int(y_px)))
    if alpha > 0.0:
        overlay = frame.copy()
        cv2.line(overlay, (0, y), (w, y), color, thickness)
        cv2.addWeighted(overlay, alpha, frame, 1.0 - alpha, 0, frame)
    else:
        cv2.line(frame, (0, y), (w, y), color, thickness)


def draw_laser_reference_line(frame: np.ndarray,
                              y_px: int,
                              ref_cfg: dict | None) -> None:
    """레이저 정렬 수평 기준선 렌더링. in-place.

    ref_cfg에서 시각 파라미터(color, thickness, alpha, enabled)를 읽고,
    y_px(= roi.center_y_px)를 기준 Y로 draw_horizontal_reference_line 호출.

    Args:
        y_px: 기준선 Y 좌표. 호출자가 roi.center_y_px를 전달.
        ref_cfg: reference_line 설정 dict. None 또는 enabled=False이면 아무것도 그리지 않음.
    """
    if not ref_cfg:
        return
    if not ref_cfg.get("enabled", True):
        return
    color = tuple(int(c) for c in ref_cfg.get("color", [0, 255, 255]))
    thickness = ref_cfg.get("thickness", 2)
    alpha = float(ref_cfg.get("alpha", 0.0))
    draw_horizontal_reference_line(frame, y_px, color, thickness, alpha=alpha)
