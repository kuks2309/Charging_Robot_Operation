#!/usr/bin/env python3
"""
DS435 - ArduCam 상대 캘리브레이션 탭
두 카메라의 라이브 피드를 나란히 표시하며 ArUco 마커를 검출/표시
"""

import os
import cv2
import numpy as np
from PyQt5 import uic
from PyQt5.QtWidgets import QWidget
from PyQt5.QtGui import QImage, QPixmap
from PyQt5.QtCore import pyqtSignal

from Sensor.aruco.aruco_detector import ArucoCameraPoseEstimator

# UI 파일 경로
UI_DIR = os.path.join(os.path.dirname(__file__), '..', '..', 'ui')
TAB_STEREO_CALIBRATION_UI = os.path.join(UI_DIR, 'tab_stereo_calibration.ui')

# 고정 표시 크기 (1280x720의 1/2)
DISPLAY_W = 640
DISPLAY_H = 360

# ArUco 검출 쓰로틀링 (매 N프레임마다 검출)
DETECT_EVERY_N = 3


class TabStereoCalibration(QWidget):
    """DS435 - ArduCam 상대(스테레오) 캘리브레이션 탭"""

    log_message = pyqtSignal(str)
    camera_start_requested = pyqtSignal()
    camera_stop_requested = pyqtSignal()
    calib_align_aruco_requested = pyqtSignal()
    calib_align_ds435_requested = pyqtSignal()
    sweep_start_requested = pyqtSignal(float, int)   # (step_mm, count)
    sweep_cancel_requested = pyqtSignal()

    def __init__(self, parent=None):
        super().__init__(parent)
        uic.loadUi(TAB_STEREO_CALIBRATION_UI, self)

        self._ds435_manager = None
        self._arducam_manager = None

        # ArUco 검출기 (VisionManager는 단일 카메라 전용이므로 직접 생성)
        self._aruco_estimator = ArucoCameraPoseEstimator(
            marker_size_meters=0.015,
            dictionary_type=cv2.aruco.DICT_5X5_50,
        )

        # 프레임 카운터 (쓰로틀링용)
        self._ds435_frame_count = 0
        self._arducam_frame_count = 0

        # 마지막 검출 결과 캐시 (비검출 프레임에 재사용)
        self._ds435_last_markers = []
        self._arducam_last_markers = []

        # 캘리브레이션 상태
        self._calib_step = 0
        self.current_frame = None      # ArduCam 최신 프레임
        self.current_ds435_frame = None  # DS435 최신 프레임

        self._connect_signals()

    def _connect_signals(self):
        """시그널 연결"""
        self.btnStartCameras.clicked.connect(self._on_start)
        self.btnStopCameras.clicked.connect(self._on_stop)
        self.btnAlignAruco.clicked.connect(self._on_btn_align_aruco)
        self.btnAlignDS435.clicked.connect(self._on_btn_align_ds435)
        self.btnSweepStart.clicked.connect(self._on_btn_sweep_start)
        self.btnSweepCancel.clicked.connect(self._on_btn_sweep_cancel)
        # 스윕 범위 자동 계산 표시
        self.spinSweepStep.valueChanged.connect(self._update_sweep_range_label)
        self.spinSweepCount.valueChanged.connect(self._update_sweep_range_label)

    def set_camera_managers(self, ds435_manager, arducam_manager):
        """두 카메라 매니저를 설정하고 frame_ready 시그널 연결"""
        self._ds435_manager = ds435_manager
        self._arducam_manager = arducam_manager

        self._ds435_manager.frame_ready.connect(self._on_ds435_frame)
        self._arducam_manager.frame_ready.connect(self._on_arducam_frame)

    def _on_start(self):
        """두 카메라 동시 시작"""
        if self._ds435_manager and not self._ds435_manager.is_running:
            self._ds435_manager.start()
        if self._arducam_manager and not self._arducam_manager.is_running:
            self._arducam_manager.start()
        self.btnStartCameras.setEnabled(False)
        self.btnStopCameras.setEnabled(True)

    def _on_stop(self):
        """두 카메라 동시 정지"""
        if self._ds435_manager and self._ds435_manager.is_running:
            self._ds435_manager.stop()
        if self._arducam_manager and self._arducam_manager.is_running:
            self._arducam_manager.stop()
        self.btnStartCameras.setEnabled(True)
        self.btnStopCameras.setEnabled(False)

    def deactivate(self):
        """탭 비활성화 시 카메라 정지 및 버튼 초기화"""
        self._on_stop()

    def _on_ds435_frame(self, frame):
        """DS435 프레임 수신 → ArUco 검출 + depth 거리 → 좌측 라벨에 표시"""
        if not self.isVisible():
            return
        self.current_ds435_frame = frame
        self._ds435_frame_count += 1
        if self._ds435_frame_count % DETECT_EVERY_N == 0:
            self._ds435_last_markers = self._detect_markers(
                frame, self._ds435_manager)
        distances = self._get_marker_distances(self._ds435_last_markers)
        overlay = self._draw_markers(frame, self._ds435_last_markers, distances)
        self._display_fixed(overlay, self.labelDS435View)

    def _on_arducam_frame(self, frame):
        """ArduCam 프레임 수신 → ArUco 검출 → 우측 라벨에 표시"""
        if not self.isVisible():
            return
        self.current_frame = frame
        self._arducam_frame_count += 1
        if self._arducam_frame_count % DETECT_EVERY_N == 0:
            self._arducam_last_markers = self._detect_markers(
                frame, self._arducam_manager)
        overlay = self._draw_markers(frame, self._arducam_last_markers)
        self._display_fixed(overlay, self.labelArduCamView)

    def _detect_markers(self, frame, camera_manager):
        """ArUco 마커 검출 (intrinsics 가드 포함)"""
        intrinsics = camera_manager.intrinsics
        if intrinsics is None:
            return []
        results = self._aruco_estimator.detect_and_estimate_pose(
            frame, intrinsics)
        return results if results else []

    def _get_marker_distances(self, markers):
        """DS435 depth로 각 마커 중심 거리 측정 (mm)"""
        if not self._ds435_manager or not markers:
            return None
        distances = []
        for m in markers:
            corners = m['corners']
            crn = corners[0] if len(corners.shape) == 3 else corners
            center = np.mean(crn, axis=0).astype(int)
            dist = self._ds435_manager.get_distance_at(
                int(center[0]), int(center[1]), from_color=True)
            distances.append(dist)
        return distances

    @staticmethod
    def _draw_markers(frame, markers, distances=None):
        """검출된 마커를 프레임에 오버레이 (draw_dual_marker_overlay 스타일)"""
        overlay = frame.copy()
        h, w = overlay.shape[:2]

        # 이미지 중심 십자선 (회색)
        img_cx, img_cy = w // 2, h // 2
        cv2.line(overlay, (img_cx, 0), (img_cx, h), (128, 128, 128), 1)
        cv2.line(overlay, (0, img_cy), (w, img_cy), (128, 128, 128), 1)

        if not markers:
            return overlay

        # 마커별 색상: 첫 번째 초록, 두 번째 빨강
        MARKER_COLORS = [(0, 255, 0), (0, 0, 255)]
        centers = []

        for idx, marker in enumerate(markers):
            corners = marker['corners']
            marker_id = marker['id']
            color = MARKER_COLORS[idx % len(MARKER_COLORS)]

            # 코너 라인
            crn = corners[0] if len(corners.shape) == 3 else corners
            for j in range(4):
                pt1 = tuple(crn[j].astype(int))
                pt2 = tuple(crn[(j + 1) % 4].astype(int))
                cv2.line(overlay, pt1, pt2, color, 2)

            # 중심점
            center = np.mean(crn, axis=0).astype(int)
            centers.append(center)
            cv2.circle(overlay, tuple(center), 5, color, -1)

            # ID 텍스트
            cv2.putText(
                overlay, f"ID:{marker_id}",
                (center[0], center[1] - 10),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)

            # 거리 표시 (DS435 depth)
            if distances and idx < len(distances) and distances[idx] is not None:
                dist_mm = distances[idx]
                cv2.putText(
                    overlay, f"{dist_mm:.0f}mm",
                    (center[0] + 10, center[1] + 25),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, 2)

        # 평균 거리 표시
        if distances:
            valid_dists = [d for d in distances if d is not None]
            if valid_dists:
                avg_dist = sum(valid_dists) / len(valid_dists)
                cv2.putText(
                    overlay, f"Avg: {avg_dist:.0f}mm",
                    (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)

        # 2개 마커 검출 시: 중심 연결선 + 중간점 크로스헤어
        if len(centers) >= 2:
            p1 = tuple(centers[0])
            p2 = tuple(centers[1])
            # 연결선 (오렌지)
            cv2.line(overlay, p1, p2, (255, 200, 0), 2)
            # 중간점 크로스헤어 (노랑)
            mid_pt = ((p1[0] + p2[0]) // 2, (p1[1] + p2[1]) // 2)
            cv2.drawMarker(overlay, mid_pt, (0, 255, 255),
                           cv2.MARKER_CROSS, 20, 2)

        return overlay

    @staticmethod
    def _display_fixed(frame, label):
        """프레임을 640x360 고정 크기로 라벨에 표시"""
        if frame is None:
            return
        resized = cv2.resize(
            frame, (DISPLAY_W, DISPLAY_H), interpolation=cv2.INTER_AREA)
        rgb = cv2.cvtColor(resized, cv2.COLOR_BGR2RGB)
        h, w, ch = rgb.shape
        q_image = QImage(rgb.data, w, h, ch * w, QImage.Format_RGB888)
        label.setPixmap(QPixmap.fromImage(q_image))

    # ==================== ArUco 정렬 ====================

    def _update_calib_step(self, step_num, message):
        """캘리브레이션 단계 라벨 업데이트"""
        self._calib_step = step_num
        self.labelCalibStep.setText(message)

    def _on_btn_align_aruco(self):
        """ArUco 정렬 버튼 → 시그널 발행"""
        self._update_calib_step(1, "ArUco 정렬 요청...")
        self.calib_align_aruco_requested.emit()

    def _update_ds435_calib_step(self, step_num, message):
        """DS435 캘리브레이션 단계 라벨 업데이트"""
        self.labelDS435CalibStep.setText(message)

    def _on_btn_align_ds435(self):
        """DS435 ArUco 정렬 버튼 → 시그널 발행"""
        self._update_ds435_calib_step(1, "DS435 정렬 요청...")
        self.calib_align_ds435_requested.emit()

    # ==================== Sweep Calibration ====================

    def _on_btn_sweep_start(self):
        """스윕 시작 버튼"""
        step_mm = self.spinSweepStep.value()
        count = self.spinSweepCount.value()
        self.btnSweepStart.setEnabled(False)
        self.btnSweepCancel.setEnabled(True)
        self.progressSweep.setValue(0)
        self.progressSweep.setMaximum(count * 3)
        self.labelSweepStatus.setText("스윕 시작 요청...")
        self.labelSweepResult.setText("")
        self.sweep_start_requested.emit(step_mm, count)

    def _on_btn_sweep_cancel(self):
        """스윕 중지 버튼"""
        self.sweep_cancel_requested.emit()

    def update_sweep_status(self, message: str):
        """스윕 상태 라벨 업데이트 (서비스에서 호출)"""
        self.labelSweepStatus.setText(message)

    def update_sweep_progress(self, current: int, total: int):
        """스윕 진행률 업데이트"""
        self.progressSweep.setMaximum(total)
        self.progressSweep.setValue(current)

    def reset_sweep_ui(self):
        """스윕 UI 초기화"""
        self.btnSweepStart.setEnabled(True)
        self.btnSweepCancel.setEnabled(False)

    def set_sweep_result(self, text: str):
        """스윕 결과 표시"""
        self.labelSweepResult.setText(text)

    def _update_sweep_range_label(self):
        """스텝 × 횟수 = 총 거리 자동 표시"""
        step = self.spinSweepStep.value()
        count = self.spinSweepCount.value()
        total = step * count
        self.labelSweepRange.setText(f"= {total:.1f}mm")

    def _log(self, msg):
        self.log_message.emit(msg)
