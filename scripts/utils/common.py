# -*- coding: utf-8 -*-
"""
공통 유틸리티 함수 및 데코레이터
"""

import inspect
from datetime import datetime
from functools import wraps
from typing import Optional, Callable
import cv2
import numpy as np

from PyQt5.QtWidgets import QLabel, QMessageBox, QFileDialog, QWidget
from PyQt5.QtGui import QImage, QPixmap
from PyQt5.QtCore import Qt


# ==================== 메시지 상수 ====================

class Messages:
    """공통 메시지 상수"""
    ROBOT_NOT_CONNECTED = "로봇이 연결되지 않았습니다."
    CAMERA_NOT_RUNNING = "카메라가 실행되지 않았습니다."
    CHESSBOARD_NOT_DETECTED = "체스보드를 감지하지 못했습니다.\n카메라 위치를 조정해주세요."
    SNAPSHOT_SAVED = "스냅샷 저장: {}"
    MOVE_FAILED = "이동 실패: {}"


# ==================== 프레임 표시 유틸 ====================

def display_frame_on_label(
    frame: np.ndarray,
    label: QLabel,
    keep_aspect_ratio: bool = True
) -> None:
    """
    OpenCV 프레임을 QLabel에 표시

    Args:
        frame: BGR 형식의 OpenCV 이미지
        label: 표시할 QLabel 위젯
        keep_aspect_ratio: 비율 유지 여부
    """
    if frame is None:
        return

    # BGR -> RGB 변환
    rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    h, w, ch = rgb_frame.shape
    bytes_per_line = ch * w

    # QImage로 변환
    q_image = QImage(rgb_frame.data, w, h, bytes_per_line, QImage.Format_RGB888)

    # QLabel 크기에 맞게 스케일링
    pixmap = QPixmap.fromImage(q_image)

    if keep_aspect_ratio:
        scaled_pixmap = pixmap.scaled(
            label.size(),
            Qt.KeepAspectRatio,
            Qt.SmoothTransformation
        )
    else:
        scaled_pixmap = pixmap.scaled(
            label.size(),
            Qt.IgnoreAspectRatio,
            Qt.SmoothTransformation
        )

    label.setPixmap(scaled_pixmap)


# ==================== 스냅샷 저장 유틸 ====================

def save_snapshot(
    frame: np.ndarray,
    parent: QWidget,
    prefix: str = "snapshot",
    log_callback: Optional[Callable[[str], None]] = None,
    default_dir: Optional[str] = None
) -> Optional[str]:
    """
    프레임을 파일로 저장

    Args:
        frame: 저장할 OpenCV 이미지
        parent: 부모 위젯 (파일 다이얼로그용)
        prefix: 파일명 접두사
        log_callback: 로그 콜백 함수
        default_dir: 기본 저장 디렉토리 (None이면 파일 다이얼로그 기본값)

    Returns:
        저장된 파일 경로 또는 None (취소 시)
    """
    if frame is None:
        QMessageBox.warning(parent, "경고", Messages.CAMERA_NOT_RUNNING)
        return None

    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    default_name = f"{prefix}_{timestamp}.png"

    if default_dir:
        os.makedirs(default_dir, exist_ok=True)
        default_name = os.path.join(default_dir, default_name)

    filepath, _ = QFileDialog.getSaveFileName(
        parent, "스냅샷 저장", default_name, "PNG Files (*.png);;All Files (*)"
    )

    if filepath:
        cv2.imwrite(filepath, frame)
        if log_callback:
            log_callback(Messages.SNAPSHOT_SAVED.format(filepath))
        return filepath

    return None


# ==================== 데코레이터 ====================

def require_robot_connection(method):
    """
    로봇 연결 확인 데코레이터

    사용법:
        @require_robot_connection
        def _on_move(self):
            # self.robot이 연결되어 있어야 실행됨
            ...
    """
    _n_params = len(inspect.signature(method).parameters) - 1  # 'self' 제외
    @wraps(method)
    def wrapper(self, *args, **kwargs):
        if self.robot is None or not self.robot.is_connected:
            QMessageBox.warning(self, "경고", Messages.ROBOT_NOT_CONNECTED)
            return None
        return method(self, *args[:_n_params], **kwargs)
    return wrapper


def require_camera_running(method):
    """
    카메라 실행 확인 데코레이터

    사용법:
        @require_camera_running
        def _on_snapshot(self):
            # self.current_frame이 있어야 실행됨
            ...
    """
    _n_params = len(inspect.signature(method).parameters) - 1  # 'self' 제외
    @wraps(method)
    def wrapper(self, *args, **kwargs):
        if self.current_frame is None:
            QMessageBox.warning(self, "경고", Messages.CAMERA_NOT_RUNNING)
            return None
        return method(self, *args[:_n_params], **kwargs)
    return wrapper
