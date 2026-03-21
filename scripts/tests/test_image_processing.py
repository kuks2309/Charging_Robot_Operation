"""
단위 테스트: scripts/utils/image_processing.py

테스트 대상:
- undistort_frame: None 파라미터 처리, 유효 파라미터 처리
- rvec_to_euler_deg: 영 벡터 입력, 반환 타입
- draw_roi_box: alpha/min_thickness/label/클램핑/빈 박스 동작
- compute_scan_roi_rects: 표준 입력 픽셀 동치성, 대칭성, 클램핑
"""
import numpy as np
import cv2
import pytest


# conftest.py가 sys.path를 설정하므로 직접 import 가능
from utils.image_processing import undistort_frame, rvec_to_euler_deg
from utils.image_processing import draw_roi_box, compute_roi_rects, draw_laser_fit_line


# ---------------------------------------------------------------------------
# 헬퍼: 테스트용 더미 이미지
# ---------------------------------------------------------------------------

def _make_frame(h=32, w=32):
    """단색 BGR 테스트 이미지를 반환한다."""
    frame = np.zeros((h, w, 3), dtype=np.uint8)
    frame[:] = (50, 100, 150)
    return frame


def _make_camera_matrix(fx=500.0, fy=500.0, cx=320.0, cy=240.0):
    return np.array(
        [[fx, 0.0, cx],
         [0.0, fy, cy],
         [0.0, 0.0, 1.0]],
        dtype=np.float64,
    )


def _make_dist_coeffs():
    return np.zeros((1, 5), dtype=np.float64)


# ---------------------------------------------------------------------------
# undistort_frame
# ---------------------------------------------------------------------------

class TestUndistortFrame:
    """undistort_frame 동작 검증."""

    def test_returns_copy_when_camera_matrix_is_none(self):
        """camera_matrix=None 이면 frame.copy()를 반환해야 한다."""
        frame = _make_frame()
        dist = _make_dist_coeffs()

        result = undistort_frame(frame, camera_matrix=None, dist_coeffs=dist)

        assert result is not frame, "원본 객체를 그대로 반환하면 안 된다"
        np.testing.assert_array_equal(result, frame)

    def test_returns_copy_when_dist_coeffs_is_none(self):
        """dist_coeffs=None 이면 frame.copy()를 반환해야 한다."""
        frame = _make_frame()
        cam = _make_camera_matrix()

        result = undistort_frame(frame, camera_matrix=cam, dist_coeffs=None)

        assert result is not frame
        np.testing.assert_array_equal(result, frame)

    def test_returns_copy_when_both_params_are_none(self):
        """camera_matrix와 dist_coeffs 모두 None 이면 frame.copy()를 반환해야 한다."""
        frame = _make_frame()

        result = undistort_frame(frame, camera_matrix=None, dist_coeffs=None)

        assert result is not frame
        np.testing.assert_array_equal(result, frame)

    def test_applies_undistort_with_valid_params(self):
        """유효한 파라미터가 주어지면 cv2.undistort 결과와 동일해야 한다."""
        frame = _make_frame()
        cam = _make_camera_matrix()
        dist = _make_dist_coeffs()

        result = undistort_frame(frame, camera_matrix=cam, dist_coeffs=dist)
        expected = cv2.undistort(frame, cam, dist)

        np.testing.assert_array_equal(result, expected)

    def test_output_shape_matches_input_shape(self):
        """출력 이미지 shape이 입력과 동일해야 한다."""
        frame = _make_frame(64, 64)
        cam = _make_camera_matrix()
        dist = _make_dist_coeffs()

        result = undistort_frame(frame, camera_matrix=cam, dist_coeffs=dist)

        assert result.shape == frame.shape


# ---------------------------------------------------------------------------
# rvec_to_euler_deg
# ---------------------------------------------------------------------------

class TestRvecToEulerDeg:
    """rvec_to_euler_deg 동작 검증."""

    def test_zero_rvec_returns_zero_euler_angles(self):
        """영 회전벡터([0,0,0])는 (0.0, 0.0, 0.0)을 반환해야 한다."""
        rvec = np.array([0.0, 0.0, 0.0])

        rx, ry, rz = rvec_to_euler_deg(rvec)

        assert rx == pytest.approx(0.0, abs=1e-9)
        assert ry == pytest.approx(0.0, abs=1e-9)
        assert rz == pytest.approx(0.0, abs=1e-9)

    def test_return_type_is_three_floats(self):
        """반환값이 (float, float, float) 타입이어야 한다."""
        rvec = np.array([0.0, 0.0, 0.0])

        result = rvec_to_euler_deg(rvec)

        assert isinstance(result, tuple)
        assert len(result) == 3
        for val in result:
            assert isinstance(val, float)

    def test_90_degree_rotation_around_z(self):
        """Z축 90도 회전 벡터 → Rz ≈ 90도 (±tolerance)를 반환해야 한다."""
        angle_rad = np.pi / 2
        rvec = np.array([0.0, 0.0, angle_rad])

        rx, ry, rz = rvec_to_euler_deg(rvec)

        assert rz == pytest.approx(90.0, abs=1e-6)
        assert rx == pytest.approx(0.0, abs=1e-6)
        assert ry == pytest.approx(0.0, abs=1e-6)

    def test_90_degree_rotation_around_x(self):
        """X축 90도 회전 벡터 → Rx ≈ 90도를 반환해야 한다."""
        angle_rad = np.pi / 2
        rvec = np.array([angle_rad, 0.0, 0.0])

        rx, ry, rz = rvec_to_euler_deg(rvec)

        assert rx == pytest.approx(90.0, abs=1e-6)

    def test_accepts_column_vector_rvec(self):
        """rvec이 (3,1) 형태여도 정상 동작해야 한다."""
        rvec = np.array([[0.0], [0.0], [0.0]])

        rx, ry, rz = rvec_to_euler_deg(rvec)

        assert rx == pytest.approx(0.0, abs=1e-9)
        assert ry == pytest.approx(0.0, abs=1e-9)
        assert rz == pytest.approx(0.0, abs=1e-9)


# ---------------------------------------------------------------------------
# draw_roi_box
# ---------------------------------------------------------------------------

def _make_black_frame(h=100, w=200):
    """검은색(0) BGR 테스트 이미지를 반환한다."""
    return np.zeros((h, w, 3), dtype=np.uint8)


class TestDrawRoiBox:
    """draw_roi_box 동작 검증."""

    def test_draw_roi_box_simple(self):
        """alpha=0일 때 경계선 픽셀만 변해야 한다.

        cv2.rectangle((x0,y0),(x1,y1)) 은 x1,y1 inclusive로 그린다.
        draw_roi_box가 cv2.rectangle(frame, (x0,y0), (x1,y1), ...) 를 직접
        호출하므로 우하단 모서리는 frame[y1, x1] 위치에 찍힌다.
        """
        frame = _make_black_frame(100, 200)
        color = (0, 255, 0)
        thickness = 1

        draw_roi_box(frame, 10, 10, 50, 40, color, thickness)

        # 경계선 위 픽셀은 color와 일치해야 한다 (cv2.rectangle inclusive 끝점)
        assert tuple(frame[10, 10]) == color, "좌상단 모서리"
        assert tuple(frame[10, 50]) == color, "우상단 모서리"
        assert tuple(frame[40, 10]) == color, "좌하단 모서리"
        assert tuple(frame[40, 50]) == color, "우하단 모서리"
        # 박스 내부(중앙)는 여전히 검정이어야 한다
        assert tuple(frame[25, 30]) == (0, 0, 0), "내부는 채워지면 안 된다"

    def test_draw_roi_box_alpha(self):
        """alpha>0일 때 박스 내부 fill 영역이 검정에서 변해야 한다."""
        frame = _make_black_frame(100, 200)
        color = (0, 0, 255)

        draw_roi_box(frame, 10, 10, 80, 80, color, thickness=1, alpha=0.5)

        # fill이 적용된 내부 픽셀은 더 이상 (0,0,0)이 아니어야 한다
        interior_pixel = tuple(frame[40, 40])
        assert interior_pixel != (0, 0, 0), "alpha>0 이면 내부가 채워져야 한다"
        # fill 색상이 color 방향으로 블렌딩됐는지 확인 (B채널이 증가)
        assert interior_pixel[2] > 0, "파란색 채널이 0보다 커야 한다"

    def test_draw_roi_box_min_thickness(self):
        """min_thickness > thickness 이면 min_thickness가 적용되어야 한다."""
        frame_thin = _make_black_frame(60, 120)
        frame_thick = _make_black_frame(60, 120)
        color = (255, 255, 255)

        # thickness=1, min_thickness=0 → 실선 1px
        draw_roi_box(frame_thin, 10, 10, 100, 50, color, thickness=1, min_thickness=0)
        # thickness=1, min_thickness=5 → 실선 5px
        draw_roi_box(frame_thick, 10, 10, 100, 50, color, thickness=1, min_thickness=5)

        # 두꺼운 선은 더 많은 픽셀이 color와 일치해야 한다
        thin_colored = np.sum(np.all(frame_thin == color, axis=2))
        thick_colored = np.sum(np.all(frame_thick == color, axis=2))
        assert thick_colored > thin_colored, (
            "min_thickness=5 이면 더 많은 픽셀이 칠해져야 한다"
        )

    def test_draw_roi_box_clamp(self):
        """프레임 범위를 벗어난 좌표는 클램핑되어 에러 없이 동작해야 한다."""
        frame = _make_black_frame(50, 80)
        color = (128, 64, 32)

        # x1=200, y1=200 → 프레임(80×50) 초과
        draw_roi_box(frame, -10, -10, 200, 200, color, thickness=1)

        # 클램핑 후 유효 영역 안에 경계선이 생겼는지 확인
        assert np.any(np.all(frame == color, axis=2)), (
            "클램핑 후 경계선 픽셀이 존재해야 한다"
        )

    def test_draw_roi_box_empty_noop(self):
        """x1<=x0 이거나 y1<=y0 이면 프레임이 변하지 않아야 한다."""
        frame = _make_black_frame(50, 80)
        original = frame.copy()
        color = (255, 0, 0)

        # x1 <= x0
        draw_roi_box(frame, 30, 10, 20, 40, color, thickness=1)
        np.testing.assert_array_equal(frame, original, err_msg="x1<=x0: 프레임이 변하면 안 된다")

        # y1 <= y0
        draw_roi_box(frame, 10, 30, 40, 20, color, thickness=1)
        np.testing.assert_array_equal(frame, original, err_msg="y1<=y0: 프레임이 변하면 안 된다")

        # 같은 경우 (x1==x0)
        draw_roi_box(frame, 20, 10, 20, 40, color, thickness=1)
        np.testing.assert_array_equal(frame, original, err_msg="x1==x0: 프레임이 변하면 안 된다")


# ---------------------------------------------------------------------------
# compute_scan_roi_rects
# ---------------------------------------------------------------------------

# 태스크 요구사항에 명시된 표준 config
_STD_CFG = {
    "center_x_offset_px": 470,
    "width_px": 280,
    "height_px": 900,
    "center_y_px": 530,  # y_offset + roi_height/2 = 80 + 900/2 = 530
    "mode": "symmetric",
}

# 태스크 요구사항에 명시된 예상 결과
_EXPECTED_LEFT  = (350, 80, 630, 980)
_EXPECTED_RIGHT = (1290, 80, 1570, 980)


class TestComputeScanRoiRects:
    """compute_roi_rects 동작 검증."""

    def test_compute_scan_roi_rects_standard(self):
        """1920×1080, 표준 config → 예상 픽셀 값과 exact match해야 한다."""
        result = compute_roi_rects(1920, 1080, _STD_CFG)

        assert len(result) == 2
        assert result[0] == _EXPECTED_LEFT,  f"좌측 ROI 불일치: {result[0]}"
        assert result[1] == _EXPECTED_RIGHT, f"우측 ROI 불일치: {result[1]}"

    def test_compute_scan_roi_rects_symmetry(self):
        """좌/우 ROI는 이미지 중심(cx)에 대해 x축 대칭이어야 한다."""
        result = compute_roi_rects(1920, 1080, _STD_CFG)

        lx0, _, lx1, _ = result[0]
        rx0, _, rx1, _ = result[1]
        cx = 1920 // 2  # 960

        # 좌 ROI와 우 ROI의 중심이 cx에 대해 대칭
        left_cx  = (lx0 + lx1) / 2
        right_cx = (rx0 + rx1) / 2
        assert left_cx + right_cx == pytest.approx(2 * cx, abs=1), (
            f"좌우 ROI 중심이 대칭이 아님: left_cx={left_cx}, right_cx={right_cx}"
        )

        # ROI 너비도 동일해야 한다
        assert (lx1 - lx0) == (rx1 - rx0), "좌우 ROI 너비가 달라야 한다"

    def test_compute_scan_roi_rects_clamp(self):
        """center_y_px + height_px/2 > frame_h 이면 y1이 frame_h로 클램핑돼야 한다."""
        cfg = {
            "center_x_offset_px": 100,
            "width_px": 60,
            "height_px": 900,   # 500 + 900/2 = 950 > frame_h=800
            "center_y_px": 500,
            "mode": "symmetric",
        }
        frame_h = 800
        result = compute_roi_rects(640, frame_h, cfg)

        _, _, _, ly1 = result[0]
        _, _, _, ry1 = result[1]
        assert ly1 == frame_h, f"좌측 y1={ly1}이 frame_h={frame_h}로 클램핑돼야 한다"
        assert ry1 == frame_h, f"우측 y1={ry1}이 frame_h={frame_h}로 클램핑돼야 한다"

    def test_compute_roi_rects_full_frame_height(self):
        """height_px=0 이면 전체 프레임 높이 사용"""
        cfg = {'center_x_offset_px': 0, 'width_px': 100, 'center_y_px': 540, 'height_px': 0, 'mode': 'symmetric'}
        rects = compute_roi_rects(1920, 1080, cfg)
        assert len(rects) == 2
        assert rects[0][1] == 0    # y0
        assert rects[0][3] == 1080  # y1 = frame_h


# ---------------------------------------------------------------------------
# draw_laser_fit_line
# ---------------------------------------------------------------------------

class TestDrawLaserFitLine:
    """draw_laser_fit_line 동작 검증."""

    def _make_frame(self):
        """검은색(0) 200×400 BGR 테스트 이미지를 반환한다."""
        return np.zeros((200, 400, 3), dtype=np.uint8)

    def test_draw_laser_fit_line_draws_line(self):
        """coeffs=[0.0, 50.0](수평선 y=50), inlier_cols=[100,200,300] 이면
        y=50 행, x=100~300 범위에 픽셀이 찍혀야 한다."""
        frame = self._make_frame()
        color = (0, 255, 0)
        coeffs = [0.0, 50.0]
        inlier_cols = np.array([100, 200, 300])

        draw_laser_fit_line(frame, coeffs, inlier_cols, color)

        # cv2.line은 x0=100, x1=300, y=50 구간을 그린다
        colored_pixels = np.any(frame[50, 100:301] != 0, axis=1)
        assert colored_pixels.any(), "y=50, x=100~300 범위에 픽셀이 찍혀야 한다"

    def test_draw_laser_fit_line_empty_inlier_noop(self):
        """inlier_cols가 빈 배열이면 프레임이 변하지 않아야 한다."""
        frame = self._make_frame()
        original = frame.copy()
        color = (0, 255, 0)
        coeffs = [0.0, 50.0]
        inlier_cols = np.array([])

        draw_laser_fit_line(frame, coeffs, inlier_cols, color)

        np.testing.assert_array_equal(frame, original, err_msg="빈 inlier_cols: 프레임이 변하면 안 된다")

    def test_draw_laser_fit_line_short_coeffs_noop(self):
        """coeffs 길이가 1이면 프레임이 변하지 않아야 한다."""
        frame = self._make_frame()
        original = frame.copy()
        color = (0, 255, 0)
        coeffs = [1.0]
        inlier_cols = np.array([50, 100, 150])

        draw_laser_fit_line(frame, coeffs, inlier_cols, color)

        np.testing.assert_array_equal(frame, original, err_msg="coeffs 길이 1: 프레임이 변하면 안 된다")

    def test_draw_laser_fit_line_pixel_exact(self):
        """coeffs=[0.5, 0.0], inlier_cols=[10, 20] 이면
        (x0=10, y0=5), (x1=20, y1=10) 끝점에 색상 픽셀이 찍혀야 한다."""
        frame = self._make_frame()
        color = (255, 0, 0)
        coeffs = [0.5, 0.0]
        inlier_cols = np.array([10, 20])

        draw_laser_fit_line(frame, coeffs, inlier_cols, color)

        # x0=10 → y0=int(round(0.5*10+0.0))=5
        # x1=20 → y1=int(round(0.5*20+0.0))=10
        assert tuple(frame[5, 10]) == color, "시작 끝점 (x=10, y=5)에 색상 픽셀이 있어야 한다"
        assert tuple(frame[10, 20]) == color, "종료 끝점 (x=20, y=10)에 색상 픽셀이 있어야 한다"

    def test_draw_laser_fit_line_thickness(self):
        """thickness=3으로 호출 시 에러 없이 동작해야 한다."""
        frame = self._make_frame()
        color = (0, 0, 255)
        coeffs = [0.0, 100.0]
        inlier_cols = np.array([50, 150, 250])

        # 예외 없이 실행되면 통과
        draw_laser_fit_line(frame, coeffs, inlier_cols, color, thickness=3)

        # 두꺼운 선이므로 색상 픽셀이 존재해야 한다
        assert np.any(np.all(frame == color, axis=2)), "thickness=3 선이 프레임에 그려져야 한다"
