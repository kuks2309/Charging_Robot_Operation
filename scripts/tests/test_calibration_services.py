"""
단위 테스트: scripts/services/camera_calibration_service.py
           scripts/services/hand_eye_calibration_service.py

테스트 대상:
- CameraCalibrationService.run_calibration: 이미지 수 부족 → success=False
- CameraCalibrationService.run_calibration: 빈 리스트 → success=False + error_message
- HandEyeCalibrationService.solve_pnp_and_store: None 입력 → None 반환
- HandEyeCalibrationService._euler_to_rotation_matrix: [0,0,0] → 단위행렬 근사
"""
import numpy as np
import pytest


# conftest.py가 sys.path와 QApplication을 설정한다
from services.camera_calibration_service import CameraCalibrationService
from services.hand_eye_calibration_service import HandEyeCalibrationService


# ---------------------------------------------------------------------------
# 헬퍼
# ---------------------------------------------------------------------------

def _make_object_points(n_corners=(6, 9), square_size=25.0):
    """체스보드 3D 객체 포인트 하나를 생성한다 (float32)."""
    rows, cols = n_corners
    pts = np.zeros((rows * cols, 3), dtype=np.float32)
    pts[:, :2] = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2) * square_size
    return pts


def _make_image_points(n_corners=(6, 9), base_offset=(100.0, 100.0), noise_scale=0.5):
    """체스보드 2D 이미지 포인트 하나를 생성한다 (Nx1x2 float32)."""
    rows, cols = n_corners
    pts = np.zeros((rows * cols, 1, 2), dtype=np.float32)
    for i, (c, r) in enumerate(
        [(c, r) for c in range(cols) for r in range(rows)]
    ):
        pts[i, 0, 0] = base_offset[0] + c * 30.0 + np.random.uniform(-noise_scale, noise_scale)
        pts[i, 0, 1] = base_offset[1] + r * 30.0 + np.random.uniform(-noise_scale, noise_scale)
    return pts


# ---------------------------------------------------------------------------
# CameraCalibrationService
# ---------------------------------------------------------------------------

class TestCameraCalibrationService:
    """CameraCalibrationService.run_calibration 동작 검증."""

    @pytest.fixture
    def service(self, qapp):
        """QApplication이 있어야 QObject 생성 가능."""
        return CameraCalibrationService()

    def test_returns_failure_when_points_list_is_empty(self, service):
        """이미지 포인트 리스트가 비어 있으면 success=False를 반환해야 한다."""
        result = service.run_calibration(
            points_3d=[],
            points_2d=[],
            image_size=(640, 480),
        )

        assert result["success"] is False

    def test_error_message_is_set_when_points_list_is_empty(self, service):
        """이미지 리스트가 비어 있으면 error_message가 None이 아니어야 한다."""
        result = service.run_calibration(
            points_3d=[],
            points_2d=[],
            image_size=(640, 480),
        )

        assert result["error_message"] is not None
        assert len(result["error_message"]) > 0

    def test_returns_failure_when_only_one_image(self, service):
        """이미지가 1개이면 success=False를 반환해야 한다 (최소 3개 필요)."""
        obj = _make_object_points()
        img = _make_image_points()

        result = service.run_calibration(
            points_3d=[obj],
            points_2d=[img],
            image_size=(640, 480),
        )

        assert result["success"] is False

    def test_returns_failure_when_only_two_images(self, service):
        """이미지가 2개이면 success=False를 반환해야 한다 (최소 3개 필요)."""
        obj = _make_object_points()
        img1 = _make_image_points(base_offset=(100.0, 100.0))
        img2 = _make_image_points(base_offset=(200.0, 150.0))

        result = service.run_calibration(
            points_3d=[obj, obj],
            points_2d=[img1, img2],
            image_size=(640, 480),
        )

        assert result["success"] is False

    def test_result_dict_has_required_keys(self, service):
        """반환 dict에 required 키가 모두 포함되어야 한다."""
        required_keys = {
            "success", "camera_matrix", "dist_coeffs",
            "rms_error", "rvecs", "tvecs", "error_message",
        }
        result = service.run_calibration(
            points_3d=[],
            points_2d=[],
            image_size=(640, 480),
        )

        assert required_keys.issubset(result.keys())

    def test_camera_matrix_is_none_on_failure(self, service):
        """실패 시 camera_matrix가 None이어야 한다."""
        result = service.run_calibration(
            points_3d=[],
            points_2d=[],
            image_size=(640, 480),
        )

        assert result["camera_matrix"] is None


# ---------------------------------------------------------------------------
# HandEyeCalibrationService
# ---------------------------------------------------------------------------

class TestHandEyeCalibrationService:
    """HandEyeCalibrationService 동작 검증."""

    @pytest.fixture
    def service(self, qapp):
        return HandEyeCalibrationService()

    # --- solve_pnp_and_store ---

    def test_solve_pnp_returns_none_when_obj_points_is_none(self, service):
        """obj_points=None 이면 None을 반환해야 한다."""
        cam = np.eye(3, dtype=np.float64)
        result = service.solve_pnp_and_store(
            obj_points=None,
            corners=None,
            camera_matrix=cam,
        )

        assert result is None

    def test_solve_pnp_returns_none_when_corners_is_none(self, service):
        """corners=None 이면 None을 반환해야 한다."""
        obj = np.array([[0, 0, 0], [1, 0, 0], [0, 1, 0], [1, 1, 0]], dtype=np.float32)
        cam = np.eye(3, dtype=np.float64)
        result = service.solve_pnp_and_store(
            obj_points=obj,
            corners=None,
            camera_matrix=cam,
        )

        assert result is None

    def test_solve_pnp_returns_none_when_camera_matrix_is_none(self, service):
        """camera_matrix=None 이면 None을 반환해야 한다."""
        obj = np.array([[0, 0, 0], [1, 0, 0], [0, 1, 0], [1, 1, 0]], dtype=np.float32)
        corners = np.array([[100, 100], [200, 100], [100, 200], [200, 200]], dtype=np.float32)
        result = service.solve_pnp_and_store(
            obj_points=obj,
            corners=corners,
            camera_matrix=None,
        )

        assert result is None

    # --- _euler_to_rotation_matrix ---

    def test_zero_euler_angles_return_identity_matrix(self):
        """(0, 0, 0) 오일러 각도 → 3x3 단위행렬 근사를 반환해야 한다."""
        R = HandEyeCalibrationService._euler_to_rotation_matrix(0.0, 0.0, 0.0)

        np.testing.assert_allclose(R, np.eye(3), atol=1e-12)

    def test_output_is_3x3_matrix(self):
        """반환 행렬이 (3, 3) shape이어야 한다."""
        R = HandEyeCalibrationService._euler_to_rotation_matrix(10.0, 20.0, 30.0)

        assert R.shape == (3, 3)

    def test_output_is_orthogonal_matrix(self):
        """반환 행렬이 직교행렬(R @ R.T ≈ I)이어야 한다."""
        R = HandEyeCalibrationService._euler_to_rotation_matrix(30.0, 45.0, 60.0)

        np.testing.assert_allclose(R @ R.T, np.eye(3), atol=1e-12)

    def test_output_determinant_is_plus_one(self):
        """회전행렬의 행렬식은 +1이어야 한다 (반사행렬 아님)."""
        R = HandEyeCalibrationService._euler_to_rotation_matrix(30.0, 45.0, 60.0)

        assert np.linalg.det(R) == pytest.approx(1.0, abs=1e-12)

    def test_90_degree_rotation_around_x(self):
        """X축 90도 회전 → 알려진 회전행렬과 일치해야 한다."""
        R = HandEyeCalibrationService._euler_to_rotation_matrix(90.0, 0.0, 0.0)
        expected = np.array([
            [1.0,  0.0,  0.0],
            [0.0,  0.0, -1.0],
            [0.0,  1.0,  0.0],
        ])

        np.testing.assert_allclose(R, expected, atol=1e-12)

    def test_90_degree_rotation_around_z(self):
        """Z축 90도 회전 → 알려진 회전행렬과 일치해야 한다."""
        R = HandEyeCalibrationService._euler_to_rotation_matrix(0.0, 0.0, 90.0)
        expected = np.array([
            [ 0.0, -1.0, 0.0],
            [ 1.0,  0.0, 0.0],
            [ 0.0,  0.0, 1.0],
        ])

        np.testing.assert_allclose(R, expected, atol=1e-12)
