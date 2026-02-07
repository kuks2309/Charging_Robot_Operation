"""
Hand-Eye Calibration Algorithms

OpenCV에서 제공하는 5가지 Hand-Eye 캘리브레이션 알고리즘 정의.
"""

from enum import Enum
import cv2


class Algorithm(Enum):
    """Hand-Eye 캘리브레이션 알고리즘"""

    TSAI = cv2.CALIB_HAND_EYE_TSAI
    """Tsai-Lenz 방법 (1989) - 가장 빠름, 최소 3개 포즈 필요"""

    PARK = cv2.CALIB_HAND_EYE_PARK
    """Park-Martin 방법 (1994) - 회전과 변환을 분리 계산"""

    HORAUD = cv2.CALIB_HAND_EYE_HORAUD
    """Horaud-Dornaika 방법 (1995) - 비선형 최적화 사용"""

    ANDREFF = cv2.CALIB_HAND_EYE_ANDREFF
    """Andreff 방법 (2001) - 선형 시스템 기반"""

    DANIILIDIS = cv2.CALIB_HAND_EYE_DANIILIDIS
    """Daniilidis 방법 (1999) - Dual Quaternion 사용, 로버스트"""

    @classmethod
    def all(cls):
        """모든 알고리즘 반환"""
        return list(cls)

    @classmethod
    def from_string(cls, name: str) -> 'Algorithm':
        """문자열로 알고리즘 선택"""
        name_upper = name.upper()
        for algo in cls:
            if algo.name == name_upper:
                return algo
        raise ValueError(f"Unknown algorithm: {name}. Available: {[a.name for a in cls]}")
