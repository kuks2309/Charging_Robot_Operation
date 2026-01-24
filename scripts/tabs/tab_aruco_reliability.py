#!/usr/bin/env python3
"""
ArUco Tag 신뢰성 검증 탭
ArUco 태그의 위치/자세 검출 신뢰성을 반복 측정하여 통계적으로 분석
"""

import os
import csv
import numpy as np
from datetime import datetime
from PyQt5 import uic
from PyQt5.QtWidgets import QWidget, QFileDialog, QMessageBox, QVBoxLayout
from PyQt5.QtCore import Qt, pyqtSignal, QTimer
from PyQt5.QtGui import QPixmap, QImage

# matplotlib 통합
import matplotlib
matplotlib.use('Qt5Agg')
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure
import matplotlib.pyplot as plt

# 공통 유틸리티
from utils.common import (
    display_frame_on_label,
    require_camera_running,
    Messages,
)

# 좌표 변환 유틸리티
from utils.ar_to_base_tf import camera_to_vision


# UI 파일 경로
UI_DIR = os.path.join(os.path.dirname(__file__), '..', '..', 'ui')
TAB_ARUCO_RELIABILITY_UI = os.path.join(UI_DIR, 'tab_aruco_reliability.ui')


class TabArucoReliability(QWidget):
    """ArUco Tag 신뢰성 검증 탭 클래스"""

    # 시그널 정의
    log_message = pyqtSignal(str)
    camera_start_requested = pyqtSignal()
    camera_stop_requested = pyqtSignal()

    def __init__(self, parent=None):
        super().__init__(parent)

        # UI 로드
        uic.loadUi(TAB_ARUCO_RELIABILITY_UI, self)

        # 카메라/비전 매니저 참조
        self.camera_manager = None
        self.vision_manager = None

        # 현재 프레임
        self.current_frame = None

        # 캡처 상태
        self.is_capturing = False
        self.capture_count = 0
        self.max_captures = 50

        # 수집된 데이터
        self.collected_data = []  # List of dicts: {timestamp, tag_id, detected, tvec, rvec, euler}

        # matplotlib 그래프 설정
        self.figure = Figure(figsize=(6, 3))
        self.canvas = FigureCanvas(self.figure)

        # 그래프 캔버스를 UI에 추가
        layout = QVBoxLayout(self.widgetGraphCanvas)
        layout.addWidget(self.canvas)
        layout.setContentsMargins(0, 0, 0, 0)

        # 캡처 타이머
        self.capture_timer = QTimer()
        self.capture_timer.timeout.connect(self._on_capture_next)

        # 시그널 연결
        self._connect_signals()

        # 초기화
        self._init_ui()

    def _connect_signals(self):
        """내부 시그널-슬롯 연결"""
        # 카메라 버튼
        self.btnStartCamera.clicked.connect(self._on_start_camera)
        self.btnStopCamera.clicked.connect(self._on_stop_camera)

        # 제어 버튼
        self.btnStartCapture.clicked.connect(self._on_start_capture)
        self.btnStopCapture.clicked.connect(self._on_stop_capture)
        self.btnReset.clicked.connect(self._on_reset)

        # 내보내기 버튼
        self.btnExportCSV.clicked.connect(self._on_export_csv)
        self.btnExportGraph.clicked.connect(self._on_export_graph)

    def _init_ui(self):
        """UI 초기화"""
        self.progressBar.setValue(0)
        self.textLog.setPlainText("신뢰성 검증을 시작하려면 '캡처 시작' 버튼을 누르세요.")
        self._update_statistics_ui()

    def set_camera_manager(self, camera_manager):
        """카메라 매니저 설정"""
        self.camera_manager = camera_manager

    def set_vision_manager(self, vision_manager):
        """비전 매니저 설정"""
        self.vision_manager = vision_manager

    def update_frame(self, frame):
        """카메라 프레임 업데이트"""
        if frame is None:
            return

        self.current_frame = frame.copy()

        # ArUco 태그 검출 및 표시
        if self.vision_manager:
            tag_id = self.spinTagID.value()
            _, markers = self.vision_manager.detect_markers(frame)

            # 검출된 마커 그리기
            if markers:
                for marker in markers:
                    if marker['id'] == tag_id:
                        # 마커 윤곽선 그리기
                        corners = marker['corners']
                        # corners shape이 (1, 4, 2)인 경우 (4, 2)로 변환
                        if len(corners.shape) == 3:
                            corners = corners[0]

                        for i in range(4):
                            pt1 = tuple(corners[i].astype(int))
                            pt2 = tuple(corners[(i + 1) % 4].astype(int))
                            import cv2
                            cv2.line(frame, pt1, pt2, (0, 255, 0), 2)

                        # Tag ID 표시
                        center = tuple(corners.mean(axis=0).astype(int))
                        cv2.putText(frame, f"ID:{marker['id']}", center,
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        # 프레임 표시
        display_frame_on_label(frame, self.labelCameraView)

    def _on_start_camera(self):
        """카메라 시작"""
        self.camera_start_requested.emit()
        self._log("카메라 시작 요청")

    def _on_stop_camera(self):
        """카메라 정지"""
        self.camera_stop_requested.emit()
        self._log("카메라 정지 요청")

    def _on_start_capture(self):
        """캡처 시작"""
        if not self.camera_manager or not self.camera_manager.is_running:
            QMessageBox.warning(self, "경고", "카메라가 실행 중이지 않습니다.")
            return

        if not self.vision_manager:
            QMessageBox.warning(self, "경고", "비전 매니저가 설정되지 않았습니다.")
            return

        # 초기화
        self.collected_data = []
        self.capture_count = 0
        self.max_captures = self.spinRepeatCount.value()
        self.is_capturing = True

        # UI 업데이트
        self.btnStartCapture.setEnabled(False)
        self.btnStopCapture.setEnabled(True)
        self.spinTagID.setEnabled(False)
        self.spinRepeatCount.setEnabled(False)
        self.spinStabilizationDelay.setEnabled(False)
        self.progressBar.setValue(0)
        self.progressBar.setMaximum(self.max_captures)
        self.textLog.clear()
        self._log(f"ArUco Tag {self.spinTagID.value()} 신뢰성 검증 시작 ({self.max_captures}회)")

        # 캡처 타이머 시작
        delay_ms = self.spinStabilizationDelay.value()
        self.capture_timer.start(delay_ms)

    def _on_stop_capture(self):
        """캡처 중지"""
        self.is_capturing = False
        self.capture_timer.stop()

        # UI 업데이트
        self.btnStartCapture.setEnabled(True)
        self.btnStopCapture.setEnabled(False)
        self.spinTagID.setEnabled(True)
        self.spinRepeatCount.setEnabled(True)
        self.spinStabilizationDelay.setEnabled(True)

        self._log(f"캡처 중지됨 (총 {self.capture_count}/{self.max_captures}회)")

    def _on_capture_next(self):
        """다음 캡처 수행"""
        if not self.is_capturing or self.capture_count >= self.max_captures:
            self._on_capture_complete()
            return

        # ArUco 태그 검출
        tag_id = self.spinTagID.value()
        timestamp = datetime.now().isoformat()

        if self.current_frame is None:
            self._log(f"[{self.capture_count + 1}/{self.max_captures}] 프레임 없음")
            self.capture_count += 1
            self.progressBar.setValue(self.capture_count)
            return

        # 태그 검출
        _, markers = self.vision_manager.detect_markers(self.current_frame)

        detected = False
        tvec = None
        rvec = None
        euler = None

        if markers:
            for marker in markers:
                if marker['id'] == tag_id:
                    detected = True
                    tvec_cam = marker.get('tvec', None)
                    rvec_cam = marker.get('rvec', None)

                    # Camera 좌표계 → TF1(Vision) 좌표계 변환
                    if tvec_cam is not None and rvec_cam is not None:
                        tvec, rvec = camera_to_vision(tvec_cam, rvec_cam)

                        # Euler 각도 계산 (TF1 좌표계 rvec에서)
                        euler = self._rvec_to_euler(rvec)
                        # 디버깅: Euler 각도 확인
                        self._log(f"  TF1 좌표 - X:{tvec[0]*1000:.1f} Y:{tvec[1]*1000:.1f} Z:{tvec[2]*1000:.1f} mm")
                        self._log(f"  Euler: Rx={euler[0]:.2f}° Ry={euler[1]:.2f}° Rz={euler[2]:.2f}°")
                    else:
                        tvec = None
                        rvec = None
                        euler = None
                        self._log(f"  경고: tvec 또는 rvec이 None입니다")
                    break

        # 데이터 저장
        data_entry = {
            'timestamp': timestamp,
            'tag_id': tag_id,
            'detected': detected,
            'tvec': tvec,
            'rvec': rvec,
            'euler': euler,
        }
        self.collected_data.append(data_entry)

        # 로그 출력
        if detected and tvec is not None:
            # tvec을 flatten하여 (3,) 형태로 변환 (TF1 좌표계, 미터 단위)
            tvec_flat = tvec.flatten()
            x, y, z = tvec_flat[0] * 1000.0, tvec_flat[1] * 1000.0, tvec_flat[2] * 1000.0  # m → mm
            self._log(f"[{self.capture_count + 1}/{self.max_captures}] 검출 성공 (TF1) - X:{x:.1f} Y:{y:.1f} Z:{z:.1f} mm")
        else:
            self._log(f"[{self.capture_count + 1}/{self.max_captures}] 검출 실패")

        # 진행률 업데이트
        self.capture_count += 1
        self.progressBar.setValue(self.capture_count)

    def _on_capture_complete(self):
        """캡처 완료"""
        self.is_capturing = False
        self.capture_timer.stop()

        # UI 복원
        self.btnStartCapture.setEnabled(True)
        self.btnStopCapture.setEnabled(False)
        self.spinTagID.setEnabled(True)
        self.spinRepeatCount.setEnabled(True)
        self.spinStabilizationDelay.setEnabled(True)

        self._log(f"\n캡처 완료: 총 {self.capture_count}회")

        # 통계 계산 및 표시
        self._calculate_and_display_statistics()

        # 그래프 그리기
        self._plot_graphs()

    def _on_reset(self):
        """초기화"""
        self.collected_data = []
        self.capture_count = 0
        self.progressBar.setValue(0)
        self.textLog.clear()
        self._log("초기화 완료")
        self._update_statistics_ui()
        self._clear_graphs()

    def _on_export_csv(self):
        """CSV 내보내기"""
        if not self.collected_data:
            QMessageBox.warning(self, "경고", "저장할 데이터가 없습니다.")
            return

        # 파일 저장 대화상자
        tag_id = self.spinTagID.value()
        default_name = f"aruco_reliability_id{tag_id}_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
        file_path, _ = QFileDialog.getSaveFileName(
            self, "CSV 저장", default_name, "CSV Files (*.csv)"
        )

        if not file_path:
            return

        try:
            with open(file_path, 'w', newline='', encoding='utf-8') as f:
                writer = csv.writer(f)

                # 헤더 (TF1 좌표계, 위치: mm, 회전: deg)
                writer.writerow(['Timestamp', 'Tag_ID', 'Detected', 'X_TF1(mm)', 'Y_TF1(mm)', 'Z_TF1(mm)', 'Rx_TF1(deg)', 'Ry_TF1(deg)', 'Rz_TF1(deg)'])

                # 데이터
                for entry in self.collected_data:
                    row = [entry['timestamp'], entry['tag_id'], entry['detected']]

                    if entry['detected'] and entry['tvec'] is not None:
                        # tvec을 flatten하여 처리 (TF1 좌표계, 미터 → mm 변환)
                        tvec_flat = entry['tvec'].flatten()
                        row.extend([
                            tvec_flat[0] * 1000.0,  # m → mm
                            tvec_flat[1] * 1000.0,  # m → mm
                            tvec_flat[2] * 1000.0,  # m → mm
                        ])
                        if entry['euler'] is not None:
                            row.extend(entry['euler'])
                        else:
                            row.extend([None, None, None])
                    else:
                        row.extend([None, None, None, None, None, None])

                    writer.writerow(row)

            self._log(f"CSV 저장 완료: {file_path}")
            QMessageBox.information(self, "성공", f"CSV 파일이 저장되었습니다.\n{file_path}")

        except Exception as e:
            self._log(f"CSV 저장 실패: {str(e)}")
            QMessageBox.critical(self, "오류", f"CSV 저장 중 오류 발생:\n{str(e)}")

    def _on_export_graph(self):
        """그래프 이미지 저장"""
        if not self.collected_data:
            QMessageBox.warning(self, "경고", "저장할 그래프가 없습니다.")
            return

        # 파일 저장 대화상자
        default_name = f"aruco_reliability_graph_{datetime.now().strftime('%Y%m%d_%H%M%S')}.png"
        file_path, _ = QFileDialog.getSaveFileName(
            self, "그래프 저장", default_name, "PNG Files (*.png);;PDF Files (*.pdf)"
        )

        if not file_path:
            return

        try:
            self.figure.savefig(file_path, dpi=300, bbox_inches='tight')
            self._log(f"그래프 저장 완료: {file_path}")
            QMessageBox.information(self, "성공", f"그래프가 저장되었습니다.\n{file_path}")

        except Exception as e:
            self._log(f"그래프 저장 실패: {str(e)}")
            QMessageBox.critical(self, "오류", f"그래프 저장 중 오류 발생:\n{str(e)}")

    def _calculate_and_display_statistics(self):
        """통계 계산 및 UI 업데이트"""
        if not self.collected_data:
            self._update_statistics_ui()
            return

        # 검출 성공 데이터만 추출
        detected_data = [d for d in self.collected_data if d['detected'] and d['tvec'] is not None]

        if not detected_data:
            self._log("검출 성공한 데이터가 없습니다.")
            self._update_statistics_ui()
            return

        # tvec 데이터 추출 (X, Y, Z) - tvec을 flatten하여 처리 (TF1 좌표계, 미터 단위)
        tvec_array = np.array([d['tvec'].flatten() for d in detected_data])

        # Euler 각도 데이터 추출 (Rx, Ry, Rz)
        euler_array = np.array([d['euler'] for d in detected_data if d['euler'] is not None])

        # 디버깅: Euler 데이터 확인
        self._log(f"\n검출된 데이터: {len(detected_data)}개")
        self._log(f"Euler 각도 데이터: {len(euler_array)}개")
        if len(euler_array) == 0:
            self._log("경고: Euler 각도 데이터가 없습니다!")

        # 통계 계산 (미터 → mm 변환)
        x_mean = np.mean(tvec_array[:, 0]) * 1000.0
        y_mean = np.mean(tvec_array[:, 1]) * 1000.0
        z_mean = np.mean(tvec_array[:, 2]) * 1000.0

        x_std = np.std(tvec_array[:, 0]) * 1000.0
        y_std = np.std(tvec_array[:, 1]) * 1000.0
        z_std = np.std(tvec_array[:, 2]) * 1000.0

        detection_rate = len(detected_data) / len(self.collected_data) * 100

        # Euler 각도 통계
        rx_mean = ry_mean = rz_mean = 0.0
        rx_std = ry_std = rz_std = 0.0

        if len(euler_array) > 0:
            rx_mean = np.mean(euler_array[:, 0])
            ry_mean = np.mean(euler_array[:, 1])
            rz_mean = np.mean(euler_array[:, 2])

            rx_std = np.std(euler_array[:, 0])
            ry_std = np.std(euler_array[:, 1])
            rz_std = np.std(euler_array[:, 2])

        # UI 업데이트
        self._update_statistics_ui(
            x_mean, y_mean, z_mean, x_std, y_std, z_std,
            rx_mean, ry_mean, rz_mean, rx_std, ry_std, rz_std,
            detection_rate
        )

        # 로그 출력
        self._log(f"\n=== 통계 결과 ===")
        self._log(f"검출 성공률: {detection_rate:.1f}% ({len(detected_data)}/{len(self.collected_data)})")
        self._log(f"위치 평균: X={x_mean:.2f}mm, Y={y_mean:.2f}mm, Z={z_mean:.2f}mm")
        self._log(f"위치 표준편차: X={x_std:.2f}mm, Y={y_std:.2f}mm, Z={z_std:.2f}mm")
        self._log(f"회전 평균: Rx={rx_mean:.2f}°, Ry={ry_mean:.2f}°, Rz={rz_mean:.2f}°")
        self._log(f"회전 표준편차: Rx={rx_std:.2f}°, Ry={ry_std:.2f}°, Rz={rz_std:.2f}°")

    def _update_statistics_ui(self, x_mean=None, y_mean=None, z_mean=None,
                               x_std=None, y_std=None, z_std=None,
                               rx_mean=None, ry_mean=None, rz_mean=None,
                               rx_std=None, ry_std=None, rz_std=None,
                               detection_rate=None):
        """통계 UI 업데이트"""
        if x_mean is None:
            # 초기화
            self.labelXMeanValue.setText("-")
            self.labelYMeanValue.setText("-")
            self.labelZMeanValue.setText("-")
            self.labelXStdValue.setText("-")
            self.labelYStdValue.setText("-")
            self.labelZStdValue.setText("-")
            self.labelRxMeanValue.setText("-")
            self.labelRyMeanValue.setText("-")
            self.labelRzMeanValue.setText("-")
            self.labelRxStdValue.setText("-")
            self.labelRyStdValue.setText("-")
            self.labelRzStdValue.setText("-")
            self.labelDetectionRateValue.setText("-")
        else:
            # 값 설정
            self.labelXMeanValue.setText(f"{x_mean:.2f} mm")
            self.labelYMeanValue.setText(f"{y_mean:.2f} mm")
            self.labelZMeanValue.setText(f"{z_mean:.2f} mm")
            self.labelXStdValue.setText(f"{x_std:.2f} mm")
            self.labelYStdValue.setText(f"{y_std:.2f} mm")
            self.labelZStdValue.setText(f"{z_std:.2f} mm")
            self.labelRxMeanValue.setText(f"{rx_mean:.2f}°")
            self.labelRyMeanValue.setText(f"{ry_mean:.2f}°")
            self.labelRzMeanValue.setText(f"{rz_mean:.2f}°")
            self.labelRxStdValue.setText(f"{rx_std:.2f}°")
            self.labelRyStdValue.setText(f"{ry_std:.2f}°")
            self.labelRzStdValue.setText(f"{rz_std:.2f}°")
            self.labelDetectionRateValue.setText(f"{detection_rate:.1f}%")

    def _plot_graphs(self):
        """분포 그래프 그리기"""
        if not self.collected_data:
            self._clear_graphs()
            return

        # 검출 성공 데이터만 추출
        detected_data = [d for d in self.collected_data if d['detected'] and d['tvec'] is not None]

        if not detected_data:
            self._clear_graphs()
            return

        # tvec 데이터 추출 (tvec을 flatten하여 처리, TF1 좌표계, 미터 단위)
        tvec_array = np.array([d['tvec'].flatten() for d in detected_data])
        # mm 단위로 변환
        tvec_array_mm = tvec_array * 1000.0

        # 그래프 초기화
        self.figure.clear()

        # 2x3 서브플롯 생성 (X, Y, Z 각각 scatter + histogram)
        ax1 = self.figure.add_subplot(2, 3, 1)
        ax2 = self.figure.add_subplot(2, 3, 2)
        ax3 = self.figure.add_subplot(2, 3, 3)
        ax4 = self.figure.add_subplot(2, 3, 4)
        ax5 = self.figure.add_subplot(2, 3, 5)
        ax6 = self.figure.add_subplot(2, 3, 6)

        # X 좌표 scatter plot
        ax1.scatter(range(len(tvec_array_mm)), tvec_array_mm[:, 0], alpha=0.5, s=10)
        ax1.axhline(np.mean(tvec_array_mm[:, 0]), color='r', linestyle='--', linewidth=1)
        ax1.set_ylabel('X (mm)')
        ax1.set_title('X Position (TF1)')
        ax1.grid(True, alpha=0.3)

        # Y 좌표 scatter plot
        ax2.scatter(range(len(tvec_array_mm)), tvec_array_mm[:, 1], alpha=0.5, s=10)
        ax2.axhline(np.mean(tvec_array_mm[:, 1]), color='r', linestyle='--', linewidth=1)
        ax2.set_ylabel('Y (mm)')
        ax2.set_title('Y Position (TF1)')
        ax2.grid(True, alpha=0.3)

        # Z 좌표 scatter plot
        ax3.scatter(range(len(tvec_array_mm)), tvec_array_mm[:, 2], alpha=0.5, s=10)
        ax3.axhline(np.mean(tvec_array_mm[:, 2]), color='r', linestyle='--', linewidth=1)
        ax3.set_ylabel('Z (mm)')
        ax3.set_title('Z Position (TF1)')
        ax3.grid(True, alpha=0.3)

        # X 좌표 histogram
        ax4.hist(tvec_array_mm[:, 0], bins=20, alpha=0.7, edgecolor='black')
        ax4.set_xlabel('X (mm)')
        ax4.set_ylabel('Count')
        ax4.grid(True, alpha=0.3)

        # Y 좌표 histogram
        ax5.hist(tvec_array_mm[:, 1], bins=20, alpha=0.7, edgecolor='black')
        ax5.set_xlabel('Y (mm)')
        ax5.set_ylabel('Count')
        ax5.grid(True, alpha=0.3)

        # Z 좌표 histogram
        ax6.hist(tvec_array_mm[:, 2], bins=20, alpha=0.7, edgecolor='black')
        ax6.set_xlabel('Z (mm)')
        ax6.set_ylabel('Count')
        ax6.grid(True, alpha=0.3)

        # 레이아웃 조정
        self.figure.tight_layout()

        # 캔버스 업데이트
        self.canvas.draw()

    def _clear_graphs(self):
        """그래프 초기화"""
        self.figure.clear()
        self.canvas.draw()

    def _rvec_to_euler(self, rvec):
        """Rotation vector를 Euler 각도로 변환 (Rx, Ry, Rz in degrees)"""
        import cv2

        # Rodrigues 변환으로 회전 행렬 얻기
        R, _ = cv2.Rodrigues(rvec)

        # 회전 행렬에서 Euler 각도 추출 (ZYX 순서)
        sy = np.sqrt(R[0, 0] * R[0, 0] + R[1, 0] * R[1, 0])

        singular = sy < 1e-6

        if not singular:
            rx = np.arctan2(R[2, 1], R[2, 2])
            ry = np.arctan2(-R[2, 0], sy)
            rz = np.arctan2(R[1, 0], R[0, 0])
        else:
            rx = np.arctan2(-R[1, 2], R[1, 1])
            ry = np.arctan2(-R[2, 0], sy)
            rz = 0

        # 라디안을 도로 변환
        return [np.degrees(rx), np.degrees(ry), np.degrees(rz)]

    def _log(self, message):
        """로그 출력"""
        self.textLog.appendPlainText(message)
        print(f"[ArUco Reliability] {message}")
