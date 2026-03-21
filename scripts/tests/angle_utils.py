"""공유 각도 정규화 유틸리티 — ModbusClient.normalize_angle과 동일 로직.

테스트 스크립트의 독립 실행을 위해 ModbusClient import 없이 사용.
로직 변경 시 이 파일과 ModbusClient.normalize_angle 양쪽을 함께 수정할 것.
"""
import math


def normalize_angle(angle: float) -> float:
    """각도를 [-180, 180] 범위로 정규화"""
    if not math.isfinite(angle):
        raise ValueError(f"Invalid angle: {angle}")
    return math.remainder(angle, 360)
