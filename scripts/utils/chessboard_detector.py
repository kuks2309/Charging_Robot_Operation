# -*- coding: utf-8 -*-
"""
체스보드 감지 유틸리티 클래스
"""

from typing import Optional, Tuple
import cv2
import numpy as np
from utils.overlay import draw_center_marker


class ChessboardDetector:
    """
    체스보드 감지 및 코너 추출 클래스

    Usage:
        detector = ChessboardDetector(cols=10, rows=7, square_size=22.0)
        result = detector.detect(frame)
        if result is not None:
            corners, center, angle = result
    """

    # 감지 플래그
    DEFAULT_FLAGS = (
        cv2.CALIB_CB_ADAPTIVE_THRESH +
        cv2.CALIB_CB_NORMALIZE_IMAGE +
        cv2.CALIB_CB_FAST_CHECK
    )

    # 서브픽셀 정밀화 기준
    SUBPIX_CRITERIA = (
        cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER,
        30,
        0.001
    )

    def __init__(
        self,
        cols: int = 10,
        rows: int = 7,
        square_size: float = 22.0
    ):
        """
        Args:
            cols: 체스보드 가로 내부 코너 수
            rows: 체스보드 세로 내부 코너 수
            square_size: 정사각형 크기 (mm)
        """
        self.cols = cols
        self.rows = rows
        self.square_size = square_size

        # 캐시된 결과
        self._last_corners: Optional[np.ndarray] = None
        self._last_center: Optional[Tuple[float, float]] = None
        self._last_angle: Optional[float] = None

    @property
    def board_size(self) -> Tuple[int, int]:
        """체스보드 크기 (cols, rows)"""
        return (self.cols, self.rows)

    def set_board_size(self, cols: int, rows: int):
        """체스보드 크기 설정"""
        self.cols = cols
        self.rows = rows

    def set_square_size(self, size: float):
        """정사각형 크기 설정 (mm)"""
        self.square_size = size

    def detect(
        self,
        frame: np.ndarray,
        refine_subpix: bool = True
    ) -> Optional[Tuple[np.ndarray, Tuple[float, float], float]]:
        """
        체스보드 코너 감지

        Args:
            frame: BGR 형식의 OpenCV 이미지
            refine_subpix: 서브픽셀 정밀화 적용 여부

        Returns:
            (corners, center, angle) 또는 None (감지 실패 시)
            - corners: 코너 좌표 배열 (N, 1, 2)
            - center: 체스보드 중심 (x, y)
            - angle: 회전 각도 (도)
        """
        # 그레이스케일 변환
        if len(frame.shape) == 3:
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        else:
            gray = frame

        # 체스보드 코너 감지
        ret, corners = cv2.findChessboardCorners(
            gray, self.board_size, self.DEFAULT_FLAGS
        )

        if not ret:
            self._last_corners = None
            self._last_center = None
            self._last_angle = None
            return None

        # 서브픽셀 정밀화
        if refine_subpix:
            corners = cv2.cornerSubPix(
                gray, corners, (11, 11), (-1, -1), self.SUBPIX_CRITERIA
            )

        # 중심 및 각도 계산
        center, angle = self._calculate_pose(corners)

        # 캐시 업데이트
        self._last_corners = corners
        self._last_center = center
        self._last_angle = angle

        return corners, center, angle

    def _calculate_pose(
        self,
        corners: np.ndarray
    ) -> Tuple[Tuple[float, float], float]:
        """
        코너로부터 중심 좌표와 회전 각도 계산

        Args:
            corners: 코너 좌표 배열

        Returns:
            (center, angle)
            - center: (x, y) 중심 좌표
            - angle: 회전 각도 (도)
        """
        corners_2d = corners.reshape(-1, 2)

        # 중심 계산
        center_x = np.mean(corners_2d[:, 0])
        center_y = np.mean(corners_2d[:, 1])

        # 각 행의 첫/끝 코너로 각도 계산
        angles = []
        for row_idx in range(self.rows):
            first_corner = corners_2d[row_idx * self.cols]
            last_corner = corners_2d[row_idx * self.cols + (self.cols - 1)]
            dx = last_corner[0] - first_corner[0]
            dy = last_corner[1] - first_corner[1]
            angles.append(np.arctan2(dy, dx))

        avg_angle = np.degrees(np.mean(angles))

        return (center_x, center_y), avg_angle

    def draw_corners(
        self,
        frame: np.ndarray,
        corners: Optional[np.ndarray] = None
    ) -> np.ndarray:
        """
        프레임에 코너 그리기

        Args:
            frame: 원본 프레임
            corners: 코너 좌표 (None이면 마지막 감지 결과 사용)

        Returns:
            코너가 그려진 프레임
        """
        if corners is None:
            corners = self._last_corners

        if corners is None:
            return frame

        return cv2.drawChessboardCorners(
            frame.copy(), self.board_size, corners, True
        )

    def draw_pose(
        self,
        frame: np.ndarray,
        center: Optional[Tuple[float, float]] = None,
        draw_crosshair: bool = True,
        draw_image_center: bool = True
    ) -> np.ndarray:
        """
        프레임에 체스보드 중심 및 이미지 중심 표시

        Args:
            frame: 원본 프레임
            center: 체스보드 중심 (None이면 마지막 감지 결과 사용)
            draw_crosshair: 전체 십자선 그리기
            draw_image_center: 이미지 중심 표시

        Returns:
            표시가 추가된 프레임
        """
        result = frame.copy()
        h, w = result.shape[:2]

        if center is None:
            center = self._last_center

        # 체스보드 중심 표시 (빨간색)
        if center is not None:
            cx, cy = int(center[0]), int(center[1])
            draw_center_marker(result, cx, cy, (0, 0, 255), radius=8, thickness=-1,
                               draw_crosshair_flag=draw_crosshair)

        # 이미지 중심 표시 (파란색)
        if draw_image_center:
            draw_center_marker(result, w // 2, h // 2, (255, 0, 0), radius=5, thickness=-1,
                               draw_crosshair_flag=draw_crosshair)

        return result

    def get_object_points(self) -> np.ndarray:
        """
        캘리브레이션용 3D 객체 점 생성

        Returns:
            (N, 3) 형태의 3D 점 배열
        """
        objp = np.zeros((self.rows * self.cols, 3), np.float32)
        objp[:, :2] = np.mgrid[0:self.cols, 0:self.rows].T.reshape(-1, 2)
        objp *= self.square_size
        return objp

    @property
    def last_result(self) -> Optional[Tuple[np.ndarray, Tuple[float, float], float]]:
        """마지막 감지 결과 반환"""
        if self._last_corners is None:
            return None
        return self._last_corners, self._last_center, self._last_angle

    @property
    def last_corners(self) -> Optional[np.ndarray]:
        """마지막 감지된 코너"""
        return self._last_corners

    @property
    def last_center(self) -> Optional[Tuple[float, float]]:
        """마지막 감지된 중심"""
        return self._last_center

    @property
    def last_angle(self) -> Optional[float]:
        """마지막 감지된 각도"""
        return self._last_angle
