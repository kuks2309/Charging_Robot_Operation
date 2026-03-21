#!/usr/bin/env python3
"""
Laser Scan Service - Z축 레이저 스캔 및 픽셀-거리 관계 분석

로봇 Z축을 스윕하면서 ArduCam 프레임의 좌/우 ROI에서
레이저 중심선 y좌표를 기록하고, z_offset vs y_mean 선형회귀로
slope(px/mm)를 산출한다.

동작 흐름:
  1. 현재 위치 기록 (원점)
  2. 원점에서 레이저 캡처 (step 0)
  3. Z 스윕: -step_mm × num_steps회 (아래 방향)
  4. 원점 복귀
  5. 분석: 좌/우 ROI 선형회귀
  6. 저장: data/laser_scan/scan_{timestamp}.json + .csv
"""

import csv
import cv2
import json
import os
import time
from datetime import datetime

import numpy as np
from PyQt5.QtCore import QObject, QTimer, pyqtSignal
from PyQt5.QtWidgets import QApplication
from scipy.optimize import differential_evolution

from services.laser_detection_service import LaserDetectionService
from utils.image_processing import compute_roi_rects


class LaserScanService(QObject):
    """Z축 레이저 스캔 서비스 (QTimer.singleShot 상태머신)"""

    # Signals
    status_updated = pyqtSignal(str)
    progress_updated = pyqtSignal(int, int)       # (current_step, total_steps)
    step_data_captured = pyqtSignal(int, float, object, object)  # (step_index, z_offset_mm, left_result_or_None, right_result_or_None)
    scan_finished = pyqtSignal(dict)              # analysis results
    scan_error = pyqtSignal(str)                  # error message
    log_message = pyqtSignal(str)                 # log output

    # States
    IDLE = 'IDLE'
    ORIGIN_CAPTURE = 'ORIGIN_CAPTURE'
    Z_SWEEP = 'Z_SWEEP'
    RETURN_TO_ORIGIN = 'RETURN_TO_ORIGIN'
    ANALYZING = 'ANALYZING'
    DONE = 'DONE'
    CANCELLED = 'CANCELLED'
    ERROR = 'ERROR'

    # 삼각측량 캘리브레이션 파일 경로
    _CALIB_FILE = os.path.join(
        os.path.dirname(__file__), '..', '..', 'config',
        'laser', 'vertical', 'laser_vertical_triangulation_calib.json'
    )

    def __init__(self, robot, arducam_manager, roi_config,
                 camera_matrix=None, dist_coeffs=None, parent=None):
        """
        Args:
            robot: ModbusClient (send_base_linear, read_current_pose)
            arducam_manager: ArduCamManager (get_frame)
            roi_config: dict loaded from config/laser/vertical/laser_vertical_scan_roi.json.
                        Expected: roi_config['roi'] with keys
                        center_x_offset_px (or center_x_px), width_px, height_px,
                        center_y_offset_px (or center_y_px), mode ("single" or "symmetric")
            camera_matrix: numpy array for undistortion (optional)
            dist_coeffs: numpy array for undistortion (optional)
        """
        super().__init__(parent)
        self._robot = robot
        self._arducam_manager = arducam_manager
        self._roi_config = roi_config
        self._camera_matrix = camera_matrix
        self._dist_coeffs = dist_coeffs

        self._state = self.IDLE
        self._step_mm = 0.0
        self._total_distance = 0.0
        self._num_steps = 0
        self._current_step = 0
        self._origin_pose = None
        self._scan_data = []
        self._cancel_requested = False

        # 삼각측량 캘리브레이션 로드
        self._triang_calib = self._load_triangulation_calib()

    def _load_triangulation_calib(self) -> dict | None:
        """삼각측량 기하학 캘리브레이션 파일 로드.

        필수 키: h_mm, Bx_mm, alpha_deg
        """
        try:
            with open(self._CALIB_FILE, 'r', encoding='utf-8') as f:
                calib = json.load(f)
            if all(k in calib for k in ('h_mm', 'Bx_mm', 'alpha_deg')):
                return calib
            self._log("[LaserScan] 캘리브레이션 파일에 필수 키 누락 (h_mm, Bx_mm, alpha_deg)")
        except FileNotFoundError:
            pass
        except Exception as exc:
            self._log(f"[LaserScan] 삼각측량 캘리브레이션 로드 실패: {exc}")
        return None

    @property
    def is_running(self) -> bool:
        return self._state not in (self.IDLE, self.DONE, self.CANCELLED, self.ERROR)

    # ==================== Public API ====================

    def start(self, step_mm: float, total_distance: float):
        """레이저 스캔 시작.

        Args:
            step_mm: 각 스텝의 이동 거리 (mm). Must be > 0.
            total_distance: 전체 스캔 거리 (mm). Must be > 0.
        """
        if self.is_running:
            self.scan_error.emit("이미 스캔 중입니다.")
            return

        if step_mm <= 0:
            self.scan_error.emit(f"step_mm은 양수여야 합니다: {step_mm}")
            return
        if total_distance <= 0:
            self.scan_error.emit(f"total_distance는 양수여야 합니다: {total_distance}")
            return

        self._step_mm = step_mm
        self._total_distance = total_distance
        self._num_steps = int(round(total_distance / step_mm))
        self._current_step = 0
        self._scan_data = []
        self._cancel_requested = False

        # Record origin pose
        self._origin_pose = self._robot.read_current_pose()
        if self._origin_pose is None:
            self.scan_error.emit("로봇 위치 읽기 실패: 원점을 기록할 수 없습니다.")
            return

        self._log(
            f"[LaserScan] 시작: step={step_mm}mm, "
            f"total={total_distance}mm, steps={self._num_steps}"
        )
        self._log(
            f"[LaserScan] 원점: X={self._origin_pose[0]:.2f}, "
            f"Y={self._origin_pose[1]:.2f}, Z={self._origin_pose[2]:.2f}"
        )

        self.status_updated.emit("원점 레이저 캡처 중...")
        self._state = self.ORIGIN_CAPTURE
        QTimer.singleShot(100, self._step)

    def cancel(self):
        """스캔 취소 요청 (다음 스텝에서 원점 복귀)"""
        self._cancel_requested = True

    # ==================== State Machine ====================

    def _step(self):
        """상태 머신 메인 디스패처"""
        if self._cancel_requested:
            self._state = self.RETURN_TO_ORIGIN
            self._cancel_requested = False
            self.status_updated.emit("스캔 취소 — 원점 복귀 중...")

        try:
            if self._state == self.ORIGIN_CAPTURE:
                self._do_origin_capture()
            elif self._state == self.Z_SWEEP:
                self._do_sweep_step()
            elif self._state == self.RETURN_TO_ORIGIN:
                self._do_return_to_origin()
            elif self._state == self.ANALYZING:
                self._do_analyze()
        except Exception as exc:
            self._log(f"[LaserScan] 예외 발생: {exc}")
            self._state = self.ERROR
            self.scan_error.emit(f"스캔 오류: {exc}")

    def _do_origin_capture(self):
        """원점(step 0)에서 레이저 데이터 캡처"""
        self._log("[LaserScan] 원점 캡처")
        pose = self._robot.read_current_pose()
        left_result, right_result = self._capture_laser_data()

        entry = {
            'step_index': 0,
            'z_offset_mm': 0.0,
            'robot_pose': [float(v) for v in pose[:6]] if pose is not None else None,
            'left_roi': left_result,
            'right_roi': right_result,
        }
        self._scan_data.append(entry)

        self.progress_updated.emit(0, self._num_steps)
        self.step_data_captured.emit(0, 0.0, left_result, right_result)

        self._state = self.Z_SWEEP
        QTimer.singleShot(100, self._step)

    def _do_sweep_step(self):
        """한 스텝 Z 이동 후 레이저 캡처"""
        if self._current_step >= self._num_steps:
            # 모든 스텝 완료 → 원점 복귀
            self._log("[LaserScan] 스윕 완료 → 원점 복귀")
            self._state = self.RETURN_TO_ORIGIN
            QTimer.singleShot(100, self._step)
            return

        self._current_step += 1
        self.status_updated.emit(f"스캔 중... ({self._current_step}/{self._num_steps})")
        self.progress_updated.emit(self._current_step, self._num_steps)

        # Z축 아래 방향 이동 (-step_mm)
        success, msg = self._robot.send_base_linear(
            'z', -self._step_mm, wait=True,
            process_events_callback=QApplication.processEvents
        )
        if not success:
            self._log(f"[LaserScan] Z 이동 실패: {msg}")
            self.scan_error.emit(f"Z 이동 실패: {msg}")
            self._state = self.RETURN_TO_ORIGIN
            QTimer.singleShot(100, self._step)
            return

        # 위치 안정화 대기
        self._wait_for_position_stable()

        # 레이저 캡처
        left_result, right_result = self._capture_laser_data()
        z_offset = self._step_mm * self._current_step

        pose = self._robot.read_current_pose()
        entry = {
            'step_index': self._current_step,
            'z_offset_mm': float(z_offset),
            'robot_pose': [float(v) for v in pose[:6]] if pose is not None else None,
            'left_roi': left_result,
            'right_roi': right_result,
        }
        self._scan_data.append(entry)

        self._log(
            f"[LaserScan] step {self._current_step}: z_offset={z_offset:.1f}mm "
            f"L={'OK' if left_result else 'NG'} "
            f"R={'OK' if right_result else 'NG'}"
        )

        self.step_data_captured.emit(
            self._current_step, float(z_offset), left_result, right_result
        )

        QTimer.singleShot(100, self._step)

    def _do_return_to_origin(self):
        """실제 현재 위치를 읽어 Z 델타 계산 후 단일 이동으로 원점 복귀"""
        self.status_updated.emit("원점 복귀 중...")
        self._log("[LaserScan] 원점 복귀 시작")

        current_pose = self._robot.read_current_pose()
        if current_pose is None:
            self._log("[LaserScan] 원점 복귀 실패: 현재 위치 읽기 실패")
            self.scan_error.emit("원점 복귀 실패: 현재 위치를 읽을 수 없습니다.")
            self._state = self.ERROR
            return

        delta_z = self._origin_pose[2] - current_pose[2]
        self._log(
            f"[LaserScan] 원점 Z={self._origin_pose[2]:.2f}, "
            f"현재 Z={current_pose[2]:.2f}, delta={delta_z:.2f}"
        )

        if abs(delta_z) > 0.1:
            success, msg = self._robot.send_base_linear(
                'z', delta_z, wait=True,
                process_events_callback=QApplication.processEvents
            )
            if not success:
                self._log(f"[LaserScan] 원점 복귀 이동 실패: {msg} (계속 진행)")
            self._wait_for_position_stable()

        self._log("[LaserScan] 원점 복귀 완료")

        # 취소된 경우 CANCELLED로, 그 외 ANALYZING으로 전이
        if self._state == self.RETURN_TO_ORIGIN and not self._cancel_requested:
            # 정상 완료 후 복귀 → 분석
            self._state = self.ANALYZING
        else:
            # 이미 cancel 처리됨 (RETURN_TO_ORIGIN 진입 시 cancel_requested 클리어됨)
            # 데이터가 있으면 분석, 없으면 CANCELLED
            if self._scan_data:
                self._state = self.ANALYZING
            else:
                self._state = self.CANCELLED
                self.status_updated.emit("스캔 취소됨")
                self.scan_finished.emit({})
                return

        QTimer.singleShot(100, self._step)

    def _do_analyze(self):
        """수집 데이터 선형회귀 분석"""
        self.status_updated.emit("데이터 분석 중...")
        self._log(f"[LaserScan] 분석 시작: {len(self._scan_data)}개 스텝")

        # 좌/우 각각 유효 포인트 추출
        left_z_offsets = []
        left_y_means = []
        right_z_offsets = []
        right_y_means = []

        for entry in self._scan_data:
            z = entry['z_offset_mm']
            left = entry.get('left_roi')
            right = entry.get('right_roi')
            if left is not None:
                left_z_offsets.append(z)
                left_y_means.append(left['y_mean'])
            if right is not None:
                right_z_offsets.append(z)
                right_y_means.append(right['y_mean'])

        left_analysis = self._fit_roi(left_z_offsets, left_y_means, 'left')
        right_analysis = self._fit_roi(right_z_offsets, right_y_means, 'right')

        # 비선형 기하학 모델 기반 각도 추정
        angle_estimate = self._estimate_tilt_angle(
            left_z_offsets, left_y_means,
            right_z_offsets, right_y_means,
        )

        results = {
            'left': left_analysis,
            'right': right_analysis,
            'angle_estimate': angle_estimate,
            'parameters': {
                'step_mm': self._step_mm,
                'total_distance': self._total_distance,
                'num_steps': self._num_steps,
            },
            'raw_data': self._scan_data,
        }

        self._save_results(results)
        self._state = self.DONE
        self.status_updated.emit("스캔 분석 완료!")
        self._log("[LaserScan] 완료")
        self.scan_finished.emit(results)

    def _estimate_tilt_angle(
        self,
        left_z: list, left_y: list,
        right_z: list, right_y: list,
    ) -> dict:
        """비선형 기하학 모델 기반 표면 기울기 각도 추정.

        레이저-카메라 삼각측량 기하학:
          laser hit point = laser_origin + t * laser_dir
          표면 평면과 교차 → 카메라 투영 → y_pixel
          differential_evolution으로 (phi, d0) 피팅.
        """
        if self._triang_calib is None:
            return {'estimated_deg': None, 'error': 'no_calibration'}
        if self._camera_matrix is None:
            return {'estimated_deg': None, 'error': 'no_camera_matrix'}

        h = self._triang_calib['h_mm']
        Bx = self._triang_calib['Bx_mm']
        alpha_rad = np.radians(self._triang_calib['alpha_deg'])
        fy = self._camera_matrix[1, 1]
        cy = self._camera_matrix[1, 2]

        sa, ca = np.sin(alpha_rad), np.cos(alpha_rad)

        def y_model(deltas, d0, phi_rad):
            tp = np.tan(phi_rad)
            t = (d0 - Bx + tp * (h - deltas)) / (ca + sa * tp)
            z_hit = h - t * sa
            x_hit = Bx + t * ca
            return cy - fy * z_hit / x_hit

        def residual_sum(params, z_arr, y_arr):
            d0, phi_deg = params
            phi_rad = np.radians(phi_deg)
            y_pred = y_model(z_arr, d0, phi_rad)
            return np.sum((y_arr - y_pred) ** 2)

        estimates = {}
        for side, z_list, y_list in [('left', left_z, left_y),
                                      ('right', right_z, right_y)]:
            if len(z_list) < 3:
                continue
            z_arr = np.array(z_list, dtype=np.float64)
            y_arr = np.array(y_list, dtype=np.float64)

            result = differential_evolution(
                residual_sum,
                bounds=[(100, 1500), (-45, 45)],  # d0 [mm], phi [deg]
                args=(z_arr, y_arr),
                seed=42, maxiter=500, tol=1e-10,
            )
            d0_fit, phi_fit = result.x
            estimates[side] = float(phi_fit)
            self._log(
                f"[LaserScan] {side}: phi={phi_fit:.2f}°, "
                f"d0={d0_fit:.1f}mm, cost={result.fun:.2f}"
            )

        if not estimates:
            return {'estimated_deg': None, 'error': 'no_valid_data'}

        avg_angle = sum(estimates.values()) / len(estimates)

        self._log(
            f"[LaserScan] 각도 추정: "
            + ", ".join(f"{k}={v:.2f}°" for k, v in estimates.items())
            + f" → 평균={avg_angle:.2f}°"
        )

        return {
            'estimated_deg': round(avg_angle, 2),
            'left_deg': estimates.get('left'),
            'right_deg': estimates.get('right'),
            'model': 'nonlinear_geometric',
            'params': {'h_mm': h, 'Bx_mm': Bx,
                       'alpha_deg': self._triang_calib['alpha_deg']},
        }

    def _fit_roi(self, z_offsets, y_means, label: str) -> dict:
        """단일 ROI에 대한 선형회귀 수행"""
        num_points = len(z_offsets)
        if num_points < 2:
            self._log(f"[LaserScan] {label} 데이터 부족: {num_points}개")
            return {
                'slope_px_per_mm': None,
                'intercept_px': None,
                'r_squared': None,
                'num_points': num_points,
                'error': 'insufficient_data',
            }

        z_arr = np.array(z_offsets, dtype=np.float64)
        y_arr = np.array(y_means, dtype=np.float64)

        coeffs = np.polyfit(z_arr, y_arr, 1)
        slope = float(coeffs[0])
        intercept = float(coeffs[1])

        # R² 계산
        y_pred = np.polyval(coeffs, z_arr)
        ss_res = float(np.sum((y_arr - y_pred) ** 2))
        ss_tot = float(np.sum((y_arr - np.mean(y_arr)) ** 2))
        r_squared = float(1.0 - ss_res / ss_tot) if ss_tot > 1e-12 else 0.0

        self._log(
            f"[LaserScan] {label}: slope={slope:.4f} px/mm, "
            f"intercept={intercept:.2f} px, R²={r_squared:.4f}, "
            f"n={num_points}"
        )

        return {
            'slope_px_per_mm': slope,
            'intercept_px': intercept,
            'r_squared': r_squared,
            'num_points': num_points,
        }

    # ==================== Data Capture ====================

    def _capture_laser_data(self):
        """현재 프레임에서 좌/우 ROI 레이저 데이터 캡처.

        Returns:
            (left_result, right_result): 각각 dict 또는 None
        """
        frame = self._arducam_manager.get_frame()
        if frame is None:
            self._log("[LaserScan] 프레임 없음")
            return None, None

        # 선택적 undistortion
        if self._camera_matrix is not None and self._dist_coeffs is not None:
            frame = cv2.undistort(frame, self._camera_matrix, self._dist_coeffs)

        # ROI 좌표 계산
        roi_cfg = self._roi_config['roi_laser']
        h, w = frame.shape[:2]
        rects = compute_roi_rects(w, h, roi_cfg)

        left_result = self._detect_in_roi(frame, rects[0])
        right_result = self._detect_in_roi(frame, rects[1]) if len(rects) > 1 else None
        return left_result, right_result

    def _detect_in_roi(self, frame: np.ndarray, roi_rect: tuple):
        """ROI 내에서 레이저 중심선 검출 — LaserDetectionService 위임.

        Args:
            frame: 전체 프레임 (BGR)
            roi_rect: (x0, y0, x1, y1) — 원본 이미지 좌표

        Returns:
            dict with {y_mean, y_min, y_max, angle_deg, num_inliers}, or None
        """
        result = LaserDetectionService.detect_in_roi(frame, roi_rect)
        if result is None or len(result['inlier_y']) == 0:
            return None
        iy = result['inlier_y']
        return {
            'y_mean': float(np.mean(iy)),
            'y_min': float(iy.min()),
            'y_max': float(iy.max()),
            'angle_deg': result['angle_deg'],
            'num_inliers': len(iy),
        }

    # ==================== Utilities ====================

    def _wait_for_position_stable(self, timeout: float = 10.0) -> bool:
        """이동 후 Z축 위치 안정화 폴링 (연속 3회 < 0.05mm)"""
        pose = self._robot.read_current_pose()
        if pose is None:
            return False
        last_z = pose[2]
        stable_count = 0
        deadline = time.time() + timeout

        while time.time() < deadline:
            time.sleep(0.2)
            QApplication.processEvents()
            pose_now = self._robot.read_current_pose()
            if pose_now is None:
                continue
            current_z = pose_now[2]
            if abs(current_z - last_z) < 0.05:
                stable_count += 1
                if stable_count >= 3:
                    return True
            else:
                stable_count = 0
            last_z = current_z
        return False

    def _save_results(self, results: dict):
        """결과를 data/laser_scan/에 JSON + CSV로 저장"""
        save_dir = os.path.join(
            os.path.dirname(__file__), '..', '..', 'data', 'laser_scan'
        )
        os.makedirs(save_dir, exist_ok=True)

        ts = datetime.now().strftime('%Y%m%d_%H%M%S')

        # JSON 직렬화용 데이터 준비 (numpy 타입 → Python 기본 타입)
        json_data = {
            'timestamp': datetime.now().isoformat(),
            'origin_pose': (
                [float(v) for v in self._origin_pose[:6]]
                if self._origin_pose is not None else None
            ),
            'parameters': results['parameters'],
            'left': results['left'],
            'right': results['right'],
            'angle_estimate': results.get('angle_estimate'),
            'raw_data': self._serialize_raw_data(results['raw_data']),
        }

        # JSON 저장
        json_path = os.path.join(save_dir, f'scan_{ts}.json')
        with open(json_path, 'w', encoding='utf-8') as f:
            json.dump(json_data, f, indent=2, ensure_ascii=False, default=str)

        # CSV 저장
        csv_path = os.path.join(save_dir, f'scan_{ts}.csv')
        with open(csv_path, 'w', newline='', encoding='utf-8') as f:
            writer = csv.writer(f)
            writer.writerow([
                'step_index', 'z_offset_mm',
                'left_y_mean', 'left_angle_deg',
                'right_y_mean', 'right_angle_deg',
            ])
            for entry in results['raw_data']:
                left = entry.get('left_roi')
                right = entry.get('right_roi')
                writer.writerow([
                    entry['step_index'],
                    f"{entry['z_offset_mm']:.3f}",
                    f"{left['y_mean']:.3f}" if left else '',
                    f"{left['angle_deg']:.4f}" if left else '',
                    f"{right['y_mean']:.3f}" if right else '',
                    f"{right['angle_deg']:.4f}" if right else '',
                ])

        self._log(f"[LaserScan] 저장 완료: {json_path}")
        self._log(f"[LaserScan] CSV 저장: {csv_path}")

    def _serialize_raw_data(self, raw_data: list) -> list:
        """raw_data 내 numpy 타입을 JSON 직렬화 가능한 Python 기본 타입으로 변환"""
        serialized = []
        for entry in raw_data:
            s_entry = {
                'step_index': int(entry['step_index']),
                'z_offset_mm': float(entry['z_offset_mm']),
                'robot_pose': entry['robot_pose'],
                'left_roi': self._serialize_roi_result(entry.get('left_roi')),
                'right_roi': self._serialize_roi_result(entry.get('right_roi')),
            }
            serialized.append(s_entry)
        return serialized

    def _serialize_roi_result(self, result) -> dict | None:
        """ROI 결과 dict의 numpy 타입을 Python 기본 타입으로 변환"""
        if result is None:
            return None
        return {
            'y_mean': float(result['y_mean']),
            'y_min': float(result['y_min']),
            'y_max': float(result['y_max']),
            'angle_deg': float(result['angle_deg']),
            'num_inliers': int(result['num_inliers']),
        }

    def _log(self, msg: str):
        """로그 시그널 발행"""
        self.log_message.emit(msg)
