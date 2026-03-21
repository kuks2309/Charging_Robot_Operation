"""normalize_angle 단위 테스트"""
import sys
import os
import math
import pytest

# conftest.py가 scripts/ 디렉토리를 sys.path에 추가한다
from Robot.communication.modbus_client import ModbusClient


class TestNormalizeAngle:
    """ModbusClient.normalize_angle() 테스트"""

    # 정상 범위 값 (변환 없음)
    @pytest.mark.parametrize("angle,expected", [
        (0.0, 0.0),
        (90.0, 90.0),
        (-90.0, -90.0),
        (45.0, 45.0),
        (-45.0, -45.0),
        (179.9, 179.9),
        (-179.9, -179.9),
    ])
    def test_normal_range(self, angle, expected):
        assert abs(ModbusClient.normalize_angle(angle) - expected) < 0.001

    # 경계값
    @pytest.mark.parametrize("angle,expected", [
        (180.0, 180.0),
        (-180.0, 180.0),  # -180 maps to +180 (same orientation)
        (180.1, -179.9),
        (-180.1, 179.9),
    ])
    def test_boundary(self, angle, expected):
        assert abs(ModbusClient.normalize_angle(angle) - expected) < 0.001

    # 래핑 (범위 초과)
    @pytest.mark.parametrize("angle,expected", [
        (185.0, -175.0),
        (-185.0, 175.0),
        (190.0, -170.0),
        (-190.0, 170.0),
        (270.0, -90.0),
        (-270.0, 90.0),
        (360.0, 0.0),
        (-360.0, 0.0),
        (540.0, 180.0),
        (-540.0, 180.0),
        (720.0, 0.0),
    ])
    def test_wrapping(self, angle, expected):
        assert abs(ModbusClient.normalize_angle(angle) - expected) < 0.001

    # 이슈 시나리오 (PRS 165번 중단 원인)
    def test_issue_scenario_positive(self):
        """Rz=175 + 10 = 185 → -175"""
        result = ModbusClient.normalize_angle(175.0 + 10.0)
        assert abs(result - (-175.0)) < 0.001

    def test_issue_scenario_negative(self):
        """Rz=-175 + (-10) = -185 → 175"""
        result = ModbusClient.normalize_angle(-175.0 + (-10.0))
        assert abs(result - 175.0) < 0.001

    # 출력 범위 검증
    def test_output_range(self):
        """모든 출력이 (-180, 180] 범위 내"""
        import random
        random.seed(42)
        for _ in range(10000):
            angle = random.uniform(-1000, 1000)
            result = ModbusClient.normalize_angle(angle)
            assert -180 < result <= 180, f"normalize_angle({angle}) = {result} out of range"

    # ×10 스케일링 후 int16 범위 검증
    def test_scaled_int16_range(self):
        """정규화 후 ×10 값이 int16 범위 내"""
        import random
        random.seed(42)
        for _ in range(10000):
            angle = random.uniform(-1000, 1000)
            result = ModbusClient.normalize_angle(angle)
            scaled = int(round(result * 10))
            assert -32768 <= scaled <= 32767, f"scaled value {scaled} out of int16 range"

    # NaN/Inf 입력 → ValueError
    def test_nan_raises(self):
        with pytest.raises(ValueError, match="유효하지 않은"):
            ModbusClient.normalize_angle(float('nan'))

    def test_inf_raises(self):
        with pytest.raises(ValueError, match="유효하지 않은"):
            ModbusClient.normalize_angle(float('inf'))

    def test_neg_inf_raises(self):
        with pytest.raises(ValueError, match="유효하지 않은"):
            ModbusClient.normalize_angle(float('-inf'))

    # 멱등성 (이중 정규화)
    def test_idempotent(self):
        """정규화를 두 번 적용해도 결과 동일"""
        import random
        random.seed(42)
        for _ in range(1000):
            angle = random.uniform(-1000, 1000)
            once = ModbusClient.normalize_angle(angle)
            twice = ModbusClient.normalize_angle(once)
            assert abs(once - twice) < 0.0001, f"Not idempotent: {angle} -> {once} -> {twice}"
