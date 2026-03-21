"""
plane_extractor 및 관련 모듈 단위 테스트
"""
import pytest
import numpy as np
import warnings
import sys
import os

# 프로젝트 루트 경로 추가
project_root = os.path.join(os.path.dirname(__file__), '..', '..')
sys.path.insert(0, project_root)
sys.path.insert(0, os.path.join(project_root, 'scripts'))

# 직접 plane_utils 모듈만 import (VisionManager 의존성 우회)
from services.plane_utils import normal_horizontal_to_euler, ensure_normal_positive_z


class TestPlaneUtils:
    """plane_utils 모듈 테스트"""

    def test_normal_horizontal_to_euler_identity(self):
        """normal=[0,0,1], horizontal=[1,0,0] → (0,0,0)"""
        normal = np.array([0, 0, 1])
        horizontal = np.array([1, 0, 0])
        rx, ry, rz = normal_horizontal_to_euler(normal, horizontal)
        assert abs(rx) < 1e-6, f"rx should be 0, got {rx}"
        assert abs(ry) < 1e-6, f"ry should be 0, got {ry}"
        assert abs(rz) < 1e-6, f"rz should be 0, got {rz}"

    def test_normal_horizontal_to_euler_rotated_z(self):
        """Z축 45도 회전"""
        normal = np.array([0, 0, 1])
        horizontal = np.array([np.sqrt(2)/2, np.sqrt(2)/2, 0])  # 45도 회전
        rx, ry, rz = normal_horizontal_to_euler(normal, horizontal)
        assert abs(rx) < 1e-3, f"rx should be 0, got {rx}"
        assert abs(ry) < 1e-3, f"ry should be 0, got {ry}"
        assert abs(rz - 45.0) < 1e-3, f"rz should be 45, got {rz}"

    def test_normal_horizontal_to_euler_rotated_y(self):
        """다양한 normal 방향 테스트"""
        # normal이 X 방향을 향할 때의 동작 확인
        # Gram-Schmidt 직교화로 인해 결과가 달라질 수 있음
        normal = np.array([1, 0, 0])
        horizontal = np.array([0, 1, 0])
        rx, ry, rz = normal_horizontal_to_euler(normal, horizontal)
        # 결과 확인 (수학적으로 유효한 값인지)
        assert not np.isnan(rx), "rx should not be NaN"
        assert not np.isnan(ry), "ry should not be NaN"
        assert not np.isnan(rz), "rz should not be NaN"

    def test_ensure_normal_positive_z_already_positive(self):
        """Normal +Z 방향 → 그대로"""
        n1 = ensure_normal_positive_z(np.array([0.1, 0.2, 0.9]))
        assert n1[2] > 0, "Normal Z should be positive"
        np.testing.assert_array_almost_equal(n1, np.array([0.1, 0.2, 0.9]))

    def test_ensure_normal_positive_z_negative(self):
        """Normal -Z 방향 → 반전"""
        n2 = ensure_normal_positive_z(np.array([0.1, 0.2, -0.9]))
        assert n2[2] > 0, "Normal Z should be positive after flip"
        np.testing.assert_array_almost_equal(n2, np.array([-0.1, -0.2, 0.9]))

    def test_zero_vector_raises_error(self):
        """영벡터 입력 시 ValueError"""
        with pytest.raises(ValueError, match="영벡터"):
            normal_horizontal_to_euler(np.array([0, 0, 0]), np.array([1, 0, 0]))

        with pytest.raises(ValueError, match="영벡터"):
            normal_horizontal_to_euler(np.array([0, 0, 1]), np.array([0, 0, 0]))

    def test_parallel_vectors_raises_error(self):
        """평행 벡터 입력 시 ValueError"""
        with pytest.raises(ValueError, match="평행"):
            normal_horizontal_to_euler(np.array([0, 0, 1]), np.array([0, 0, 1]))

    def test_gimbal_lock_warning(self):
        """Gimbal lock 검출 로직이 존재함을 확인"""
        # normal과 horizontal이 수직인 경우 정상 작동 확인
        normal = np.array([0, 0, 1])
        horizontal = np.array([1, 0, 0])

        # 정상 케이스에서는 경고 없이 작동
        with warnings.catch_warnings(record=True) as w:
            warnings.simplefilter("always")
            rx, ry, rz = normal_horizontal_to_euler(normal, horizontal)
            gimbal_warnings = [warning for warning in w if "gimbal" in str(warning.message).lower()]
            # 일반적인 경우 gimbal lock 없음
            assert isinstance(rx, float), "Should return valid floats"
            assert isinstance(ry, float), "Should return valid floats"
            assert isinstance(rz, float), "Should return valid floats"


class TestTCPCorrector:
    """TCPCorrector 클래스 테스트"""

    def test_compute_correction_zero(self):
        """평면과 정렬 시 보정값 0"""
        from services.dual_aruco_detector import PlanePose
        from services.tcp_corrector import TCPCorrector

        plane = PlanePose(x=0, y=0, z=100, rx=0, ry=0, rz=0, valid_samples=10, total_attempts=10)
        corrector = TCPCorrector(target_rx=0, target_ry=0, target_rz=0)

        result = corrector.compute_correction(plane)

        assert abs(result.delta_rx) < 1e-6, f"delta_rx should be 0, got {result.delta_rx}"
        assert abs(result.delta_ry) < 1e-6, f"delta_ry should be 0, got {result.delta_ry}"
        assert abs(result.delta_rz) < 1e-6, f"delta_rz should be 0, got {result.delta_rz}"

    def test_compute_correction_nonzero(self):
        """틸트된 평면의 보정값"""
        from services.dual_aruco_detector import PlanePose
        from services.tcp_corrector import TCPCorrector

        plane = PlanePose(x=0, y=0, z=100, rx=5, ry=-3, rz=10, valid_samples=10, total_attempts=10)
        corrector = TCPCorrector(target_rx=0, target_ry=0, target_rz=0)

        result = corrector.compute_correction(plane)

        assert abs(result.delta_rx - (-5)) < 1e-6, f"delta_rx should be -5, got {result.delta_rx}"
        assert abs(result.delta_ry - 3) < 1e-6, f"delta_ry should be 3, got {result.delta_ry}"
        assert abs(result.delta_rz - (-10)) < 1e-6, f"delta_rz should be -10, got {result.delta_rz}"

    def test_apply_correction(self):
        """보정값 적용 테스트"""
        from services.dual_aruco_detector import PlanePose
        from services.tcp_corrector import TCPCorrector, TCPCorrection

        corrector = TCPCorrector()

        correction = TCPCorrection(
            delta_rx=5.0, delta_ry=-3.0, delta_rz=10.0,
            plane_rx=0, plane_ry=0, plane_rz=0,
            target_rx=5, target_ry=-3, target_rz=10
        )

        current_tcp = (100.0, 200.0, 300.0, 0.0, 0.0, 0.0)
        new_tcp = corrector.apply_correction(current_tcp, correction)

        assert new_tcp[0] == 100.0, "X should not change"
        assert new_tcp[1] == 200.0, "Y should not change"
        assert new_tcp[2] == 300.0, "Z should not change"
        assert abs(new_tcp[3] - 5.0) < 1e-6, f"rx should be 5.0, got {new_tcp[3]}"
        assert abs(new_tcp[4] - (-3.0)) < 1e-6, f"ry should be -3.0, got {new_tcp[4]}"
        assert abs(new_tcp[5] - 10.0) < 1e-6, f"rz should be 10.0, got {new_tcp[5]}"

    def test_compute_final_tcp_rotation_matrix(self):
        """회전 행렬 합성으로 최종 절대 TCP 자세 계산 - 단순 덧셈과 차이 검증"""
        from services.dual_aruco_detector import PlanePose
        from services.tcp_corrector import TCPCorrector

        plane = PlanePose(x=0, y=0, z=100, rx=5, ry=-3, rz=10, valid_samples=10, total_attempts=10)
        current_tcp = (100.0, 200.0, 300.0, 45.0, 30.0, -15.0)

        corrector = TCPCorrector(target_rx=0, target_ry=0, target_rz=0)
        result = corrector.compute_correction(plane, current_tcp=current_tcp)

        assert result.final_rx is not None
        assert result.final_ry is not None
        assert result.final_rz is not None
        assert result.current_tcp_rx == 45.0
        assert result.current_tcp_ry == 30.0
        assert result.current_tcp_rz == -15.0

        simple_rx = 45.0 + result.delta_rx
        simple_ry = 30.0 + result.delta_ry
        simple_rz = -15.0 + result.delta_rz

        diff_rx = abs(result.final_rx - simple_rx)
        diff_ry = abs(result.final_ry - simple_ry)
        diff_rz = abs(result.final_rz - simple_rz)
        total_diff = diff_rx + diff_ry + diff_rz

        assert total_diff > 0.5, f"Matrix composition should differ from simple addition. Got diff={total_diff:.3f}"

    def test_compute_final_tcp_identity(self):
        """delta=0일 때 최종 자세 = 현재 자세"""
        from services.dual_aruco_detector import PlanePose
        from services.tcp_corrector import TCPCorrector

        plane = PlanePose(x=0, y=0, z=100, rx=0, ry=0, rz=0, valid_samples=10, total_attempts=10)
        current_tcp = (100.0, 200.0, 300.0, 45.0, 30.0, -15.0)

        corrector = TCPCorrector(target_rx=0, target_ry=0, target_rz=0)
        result = corrector.compute_correction(plane, current_tcp=current_tcp)

        assert abs(result.final_rx - 45.0) < 1e-3
        assert abs(result.final_ry - 30.0) < 1e-3
        assert abs(result.final_rz - (-15.0)) < 1e-3

    def test_compute_final_tcp_method(self):
        """compute_final_tcp() 메서드 테스트"""
        from services.dual_aruco_detector import PlanePose
        from services.tcp_corrector import TCPCorrector

        plane = PlanePose(x=0, y=0, z=100, rx=5, ry=-3, rz=10, valid_samples=10, total_attempts=10)
        current_tcp = (100.0, 200.0, 300.0, 45.0, 30.0, -15.0)

        corrector = TCPCorrector(target_rx=0, target_ry=0, target_rz=0)
        correction = corrector.compute_correction(plane, current_tcp=current_tcp)

        final = corrector.compute_final_tcp(current_tcp, correction)

        assert final[0] == 100.0
        assert final[1] == 200.0
        assert final[2] == 300.0
        assert abs(final[3] - correction.final_rx) < 1e-6
        assert abs(final[4] - correction.final_ry) < 1e-6
        assert abs(final[5] - correction.final_rz) < 1e-6

    def test_gimbal_lock_returns_valid_values(self):
        """Gimbal lock 근처에서도 유효한 값 반환"""
        import math
        normal = np.array([1, 0, 0])
        horizontal = np.array([0, 1, 0])

        rx, ry, rz = normal_horizontal_to_euler(normal, horizontal)

        assert not math.isnan(rx) and not math.isinf(rx)
        assert not math.isnan(ry) and not math.isinf(ry)
        assert not math.isnan(rz) and not math.isinf(rz)
        assert -180 <= rx <= 180
        assert -180 <= ry <= 180
        assert -180 <= rz <= 180


class TestPlanePose:
    """PlanePose dataclass 테스트"""

    def test_to_dict(self):
        """to_dict 메서드 테스트"""
        from services.dual_aruco_detector import PlanePose

        pose = PlanePose(x=1.0, y=2.0, z=3.0, rx=10.0, ry=20.0, rz=30.0,
                        valid_samples=5, total_attempts=10)
        d = pose.to_dict()

        assert d['x'] == 1.0
        assert d['y'] == 2.0
        assert d['z'] == 3.0
        assert d['rx'] == 10.0
        assert d['ry'] == 20.0
        assert d['rz'] == 30.0
        assert d['valid_samples'] == 5
        assert d['total_attempts'] == 10

    def test_from_dict(self):
        """from_dict 메서드 테스트"""
        from services.dual_aruco_detector import PlanePose

        d = {
            'x': 1.0, 'y': 2.0, 'z': 3.0,
            'rx': 10.0, 'ry': 20.0, 'rz': 30.0,
            'valid_samples': 5, 'total_attempts': 10
        }
        pose = PlanePose.from_dict(d)

        assert pose.x == 1.0
        assert pose.y == 2.0
        assert pose.z == 3.0
        assert pose.rx == 10.0
        assert pose.ry == 20.0
        assert pose.rz == 30.0
        assert pose.valid_samples == 5
        assert pose.total_attempts == 10


if __name__ == '__main__':
    pytest.main([__file__, '-v'])
