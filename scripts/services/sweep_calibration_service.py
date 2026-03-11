#!/usr/bin/env python3
"""
Sweep Calibration Service - 로봇 이동과 카메라 픽셀 관계 분석

로봇 X/Y/Z를 스윕하면서 두 카메라(ArduCam + DS435)의
ArUco 마커 픽셀 위치를 기록하고 px/mm 비율을 산출한다.

동작 흐름:
  1. 현재 위치 기록 (원점)
  2. Z 스윕: -step_mm × count회 (차트 방향), 원점 복귀
  3. X 스윕: +step_mm × count회, 원점 복귀
  4. Y 스윕: +step_mm × count회, 원점 복귀
  5. 분석: 선형회귀 + Z축 2차회귀
  6. 저장: data/stereo/sweep_{timestamp}.json + .csv
"""

import json
import os
import time
from datetime import datetime
from typing import Optional, Dict, List

import numpy as np
from PyQt5.QtCore import QObject, QTimer, pyqtSignal
from PyQt5.QtWidgets import QApplication


class SweepCalibrationService(QObject):
    """로봇-픽셀 관계 분석 서비스 (QTimer.singleShot 상태머신)"""

    # Signals
    status_updated = pyqtSignal(str)
    progress_updated = pyqtSignal(int, int)   # (current_step, total_steps)
    sweep_finished = pyqtSignal(dict)
    sweep_error = pyqtSignal(str)
    log_message = pyqtSignal(str)
    # (axis, step_mm, ds435_x, ds435_y, ds435_z_mm, arducam_x, arducam_y)
    data_captured = pyqtSignal(str, float, object, object, object, object, object)

    # States
    IDLE = 'IDLE'
    Z_SWEEP = 'Z_SWEEP'
    Z_RETURN = 'Z_RETURN'
    X_SWEEP = 'X_SWEEP'
    X_RETURN = 'X_RETURN'
    Y_SWEEP = 'Y_SWEEP'
    Y_RETURN = 'Y_RETURN'
    ANALYZING = 'ANALYZING'
    DONE = 'DONE'
    CANCELLED = 'CANCELLED'

    def __init__(self, robot, ds435_manager, arducam_manager,
                 aruco_estimator, parent=None):
        """
        Args:
            robot: ModbusClient (send_base_linear, read_current_pose)
            ds435_manager: CameraManager (get_frame, intrinsics, get_distance_at)
            arducam_manager: ArduCamManager (get_frame, intrinsics)
            aruco_estimator: ArucoCameraPoseEstimator (detect_and_estimate_pose)
        """
        super().__init__(parent)
        self._robot = robot
        self._ds435 = ds435_manager
        self._arducam = arducam_manager
        self._estimator = aruco_estimator

        self._state = self.IDLE
        self._cancel_requested = False

        # Sweep parameters
        self._step_mm = 5.0
        self._count = 10
        self._current_step = 0

        # Origin pose
        self._origin_pose = None

        # Collected data: {axis: [data_points]}
        self._data = {'z': [], 'x': [], 'y': []}

        # Progress tracking
        self._total_steps = 0
        self._completed_steps = 0

    @property
    def is_running(self) -> bool:
        return self._state not in (self.IDLE, self.DONE, self.CANCELLED)

    def start(self, step_mm: float = 5.0, count: int = 10):
        """스윕 캘리브레이션 시작"""
        if self.is_running:
            return

        self._step_mm = step_mm
        self._count = count
        self._cancel_requested = False
        self._data = {'z': [], 'x': [], 'y': []}
        self._completed_steps = 0
        self._total_steps = count * 3  # Z + X + Y

        # Record origin pose
        self._origin_pose = self._robot.read_current_pose()
        if self._origin_pose is None:
            self.sweep_error.emit("로봇 위치 읽기 실패")
            return

        self._log(f"[Sweep] 시작: step={step_mm}mm, count={count}")
        self._log(f"[Sweep] 원점: X={self._origin_pose[0]:.2f}, "
                  f"Y={self._origin_pose[1]:.2f}, Z={self._origin_pose[2]:.2f}")

        # Capture origin data point (Z축만 - X, Y는 복귀 후 각각 캡처)
        self.status_updated.emit("원점 데이터 캡처 중...")
        origin_data = self._capture_data_point()
        if origin_data:
            self._data['z'].append({
                'robot_mm': 0.0,
                **origin_data,
            })
        else:
            self._log("[Sweep] 경고: 원점 데이터 캡처 실패 (마커 미검출)")

        # Start Z sweep
        self._state = self.Z_SWEEP
        self._current_step = 0
        self.status_updated.emit("Z 스윕 시작...")
        QTimer.singleShot(100, self._step)

    def cancel(self):
        """스윕 취소 요청 (다음 스텝에서 원점 복귀)"""
        self._cancel_requested = True
        self.status_updated.emit("취소 중... 원점 복귀 대기")

    # ==================== State Machine ====================

    def _step(self):
        """상태 머신 메인 루프"""
        if self._cancel_requested:
            self.status_updated.emit("취소 중... 원점 복귀")
            self._return_to_origin()
            self._state = self.CANCELLED
            self.status_updated.emit("취소됨 (원점 복귀 완료)")
            self._log("[Sweep] 사용자 취소")
            self.sweep_finished.emit({})  # UI 리셋 트리거
            return

        try:
            if self._state == self.Z_SWEEP:
                self._do_sweep_step('z', -self._step_mm)
            elif self._state == self.Z_RETURN:
                self._do_return('z')
            elif self._state == self.X_SWEEP:
                self._do_sweep_step('x', self._step_mm)
            elif self._state == self.X_RETURN:
                self._do_return('x')
            elif self._state == self.Y_SWEEP:
                self._do_sweep_step('y', self._step_mm)
            elif self._state == self.Y_RETURN:
                self._do_return('y')
            elif self._state == self.ANALYZING:
                self._do_analyze()
        except Exception as e:
            self._log(f"[Sweep] 오류: {e}")
            self._return_to_origin()
            self._state = self.IDLE
            self.sweep_error.emit(f"오류: {e}")

    def _do_sweep_step(self, axis: str, step_mm: float):
        """한 축의 스윕 스텝 실행"""
        if self._current_step >= self._count:
            # This axis complete → return to origin
            next_states = {
                'z': self.Z_RETURN,
                'x': self.X_RETURN,
                'y': self.Y_RETURN,
            }
            self._state = next_states[axis]
            QTimer.singleShot(100, self._step)
            return

        step_idx = self._current_step + 1
        axis_label = axis.upper()
        self.status_updated.emit(
            f"{axis_label} 스윕 [{step_idx}/{self._count}]: "
            f"{step_mm:+.1f}mm 이동 중...")
        self.progress_updated.emit(self._completed_steps, self._total_steps)

        # Move robot
        success, msg = self._robot.send_base_linear(
            axis, step_mm,
            process_events_callback=QApplication.processEvents)
        if not success:
            self._log(f"[Sweep] {axis_label} 이동 실패: {msg}")
            self.sweep_error.emit(f"{axis_label} 이동 실패: {msg}")
            self._return_to_origin()
            self._state = self.IDLE
            return

        # Wait for position stable
        time.sleep(0.2)
        if not self._wait_for_position_stable(axis):
            if self._cancel_requested:
                QTimer.singleShot(0, self._step)
                return
            self._log(f"[Sweep] {axis_label} 위치 안정화 타임아웃 (계속 진행)")

        # Cumulative displacement from origin
        cumulative_mm = step_mm * step_idx

        # Capture data at this position
        data_point = self._capture_data_point()
        if data_point:
            self._data[axis].append({
                'robot_mm': cumulative_mm,
                **data_point,
            })
            self._log(
                f"[Sweep] {axis_label} step {step_idx}: "
                f"{cumulative_mm:+.1f}mm OK")
            # UI 테이블 업데이트 시그널
            ds = data_point.get('ds435')
            ar = data_point.get('arducam')
            self.data_captured.emit(
                axis, cumulative_mm,
                ds['midpoint'][0] if ds else None,
                ds['midpoint'][1] if ds else None,
                ds.get('depth_mm') if ds else None,
                ar['midpoint'][0] if ar else None,
                ar['midpoint'][1] if ar else None,
            )
        else:
            self._log(
                f"[Sweep] {axis_label} step {step_idx}: "
                f"데이터 캡처 실패 (스킵)")

        self._current_step += 1
        self._completed_steps += 1
        self.progress_updated.emit(self._completed_steps, self._total_steps)
        QTimer.singleShot(100, self._step)

    def _do_return(self, axis: str):
        """원점으로 복귀 후 다음 축으로 전이"""
        axis_label = axis.upper()
        self.status_updated.emit(f"{axis_label} 원점 복귀 중...")

        current = self._robot.read_current_pose()
        if current is None:
            self.sweep_error.emit("위치 읽기 실패")
            self._state = self.IDLE
            return

        axis_idx = {'x': 0, 'y': 1, 'z': 2}[axis]
        delta = self._origin_pose[axis_idx] - current[axis_idx]

        if abs(delta) > 0.1:
            success, msg = self._robot.send_base_linear(
                axis, delta,
                process_events_callback=QApplication.processEvents)
            if not success:
                self._log(f"[Sweep] {axis_label} 복귀 실패: {msg} (계속 진행)")
            time.sleep(0.2)
            self._wait_for_position_stable(axis)

        self._log(f"[Sweep] {axis_label} 원점 복귀 완료")

        # Transition to next axis
        next_transitions = {
            'z': (self.X_SWEEP, 'x'),
            'x': (self.Y_SWEEP, 'y'),
            'y': (self.ANALYZING, None),
        }
        next_state, next_axis = next_transitions[axis]
        self._state = next_state
        if next_axis:
            self._current_step = 0
            # 다음 축 시작 전 fresh origin 캡처 (BUG1 fix)
            fresh_origin = self._capture_data_point()
            if fresh_origin:
                self._data[next_axis].append({
                    'robot_mm': 0.0,
                    **fresh_origin,
                })
            else:
                self._log(f"[Sweep] {next_axis.upper()} 원점 캡처 실패")
        QTimer.singleShot(200, self._step)

    def _do_analyze(self):
        """수집 데이터 선형/2차 회귀 분석"""
        self.status_updated.emit("데이터 분석 중...")
        self._log("[Sweep] 분석 시작")

        results = {}
        for axis in ('z', 'x', 'y'):
            points = self._data[axis]
            if len(points) < 2:
                self._log(f"[Sweep] {axis.upper()} 데이터 부족: {len(points)}개")
                continue

            robot_mm = np.array([p['robot_mm'] for p in points])

            arducam_analysis = self._analyze_camera_data(
                points, robot_mm, axis, 'arducam')
            ds435_analysis = self._analyze_camera_data(
                points, robot_mm, axis, 'ds435')

            results[axis] = {
                'arducam': arducam_analysis,
                'ds435': ds435_analysis,
                'step_mm': self._step_mm,
                'count': self._count,
                'data_points': len(points),
            }

            # Log key results
            for cam_key in ('arducam', 'ds435'):
                analysis = results[axis][cam_key]
                if 'px_per_mm_x' in analysis:
                    self._log(
                        f"[Sweep] {axis.upper()}/{cam_key}: "
                        f"px/mm_x={analysis['px_per_mm_x']:.2f}, "
                        f"px/mm_y={analysis['px_per_mm_y']:.2f}, "
                        f"R²_x={analysis.get('r_squared_x', 0):.4f}, "
                        f"R²_y={analysis.get('r_squared_y', 0):.4f}")

        # Save results
        save_path = self._save_results(results)
        self._state = self.DONE
        self.status_updated.emit(f"분석 완료! 저장: {os.path.basename(save_path)}")
        self._log(f"[Sweep] 완료. 저장: {save_path}")
        self.sweep_finished.emit(results)

    # ==================== Data Capture ====================

    def _capture_data_point(self) -> Optional[Dict]:
        """현재 위치에서 양쪽 카메라 데이터 캡처"""
        pose = self._robot.read_current_pose()
        if pose is None:
            return None

        return {
            'robot_pose': [float(v) for v in pose[:6]],
            'arducam': self._capture_camera('arducam'),
            'ds435': self._capture_camera('ds435'),
        }

    def _capture_camera(self, camera_key: str) -> Optional[Dict]:
        """한 카메라의 ArUco 마커 데이터 캡처 (get_frame 직접 사용)"""
        manager = self._arducam if camera_key == 'arducam' else self._ds435

        frame = manager.get_frame()
        if frame is None:
            return None

        intrinsics = manager.intrinsics
        if intrinsics is None:
            return None

        # Detect markers
        markers = self._estimator.detect_and_estimate_pose(frame, intrinsics)
        if not markers or len(markers) < 2:
            return None

        # Sort by ID for consistency, then take first 2
        markers_sorted = sorted(markers, key=lambda m: m['id'])[:2]

        # Extract centers from corners
        centers = []
        marker_sizes = []
        marker_ids = []
        for m in markers_sorted:
            corners = m['corners']
            crn = corners[0] if len(corners.shape) == 3 else corners
            center = np.mean(crn, axis=0)
            centers.append(center)
            # Marker pixel size (diagonal length)
            diag = float(np.linalg.norm(crn[0] - crn[2]))
            marker_sizes.append(diag)
            marker_ids.append(int(m['id']))

        c1, c2 = centers[0], centers[1]
        midpoint = (c1 + c2) / 2.0
        avg_size = sum(marker_sizes) / len(marker_sizes)

        result = {
            'marker1_center': [float(c1[0]), float(c1[1])],
            'marker2_center': [float(c2[0]), float(c2[1])],
            'midpoint': [float(midpoint[0]), float(midpoint[1])],
            'marker_size_px': avg_size,
            'marker_ids': marker_ids,
        }

        # DS435: add depth at midpoint
        if camera_key == 'ds435':
            depth = manager.get_distance_at(
                int(midpoint[0]), int(midpoint[1]), from_color=True)
            result['depth_mm'] = float(depth) if depth is not None else None

        return result

    # ==================== Analysis ====================

    def _analyze_camera_data(self, points: List[Dict], robot_mm: np.ndarray,
                             axis: str, camera_key: str) -> Dict:
        """한 카메라의 수집 데이터에 대해 회귀 분석"""
        # Extract valid midpoints
        mid_x_list = []
        mid_y_list = []
        valid_indices = []

        for i, p in enumerate(points):
            cam_data = p.get(camera_key)
            if cam_data and cam_data.get('midpoint') is not None:
                mid_x_list.append(cam_data['midpoint'][0])
                mid_y_list.append(cam_data['midpoint'][1])
                valid_indices.append(i)

        if len(valid_indices) < 2:
            return {'error': 'insufficient_data', 'valid_points': len(valid_indices)}

        mid_x = np.array(mid_x_list)
        mid_y = np.array(mid_y_list)
        mm_valid = robot_mm[valid_indices]

        # Pixel displacement from origin
        dx_px = mid_x - mid_x[0]
        dy_px = mid_y - mid_y[0]

        result = {
            'valid_points': len(valid_indices),
            'mid_x': [float(v) for v in mid_x],
            'mid_y': [float(v) for v in mid_y],
            'dx_px': [float(v) for v in dx_px],
            'dy_px': [float(v) for v in dy_px],
        }

        if np.std(mm_valid) < 1e-6:
            result['error'] = 'no_variance_in_robot_mm'
            return result

        # 원점 누락 시 상대값 보정 (BUG2 fix)
        mm_relative = mm_valid - mm_valid[0]

        # Linear regression: robot_mm → pixel displacement
        coeff_x = np.polyfit(mm_relative, dx_px, 1)
        result['px_per_mm_x'] = float(coeff_x[0])
        result['linear_fit_x'] = [float(c) for c in coeff_x]

        coeff_y = np.polyfit(mm_relative, dy_px, 1)
        result['px_per_mm_y'] = float(coeff_y[0])
        result['linear_fit_y'] = [float(c) for c in coeff_y]

        # R² values
        result['r_squared_x'] = self._r_squared(mm_relative, dx_px, coeff_x)
        result['r_squared_y'] = self._r_squared(mm_relative, dy_px, coeff_y)

        # Z axis: quadratic fit for nonlinearity (scale changes with distance)
        if axis == 'z' and len(valid_indices) >= 3:
            quad_x = np.polyfit(mm_relative, dx_px, 2)
            quad_y = np.polyfit(mm_relative, dy_px, 2)
            result['quadratic_fit_x'] = [float(c) for c in quad_x]
            result['quadratic_fit_y'] = [float(c) for c in quad_y]
            result['r_squared_quad_x'] = self._r_squared(mm_relative, dx_px, quad_x)
            result['r_squared_quad_y'] = self._r_squared(mm_relative, dy_px, quad_y)

            # Marker size change tracking (BUG3 fix: collect paired (mm, size))
            size_pairs = []
            for idx_v, i in enumerate(valid_indices):
                cam = points[i].get(camera_key, {})
                if cam and cam.get('marker_size_px') is not None:
                    size_pairs.append((mm_relative[idx_v], cam['marker_size_px']))
            if len(size_pairs) >= 2:
                sz_mm = np.array([p[0] for p in size_pairs])
                sz_px = np.array([p[1] for p in size_pairs])
                size_coeff = np.polyfit(sz_mm, sz_px, 1)
                result['size_per_mm'] = float(size_coeff[0])
                result['marker_sizes_px'] = [float(s) for s in sz_px]

        # DS435 depth data
        if camera_key == 'ds435':
            depths = []
            for i in valid_indices:
                cam = points[i].get('ds435', {})
                if cam and cam.get('depth_mm') is not None:
                    depths.append(cam['depth_mm'])
            if depths:
                result['depths_mm'] = depths

        return result

    @staticmethod
    def _r_squared(x: np.ndarray, y: np.ndarray, coeffs: np.ndarray) -> float:
        """R² (결정계수) 계산"""
        pred = np.polyval(coeffs, x)
        ss_res = np.sum((y - pred) ** 2)
        ss_tot = np.sum((y - np.mean(y)) ** 2)
        if ss_tot < 1e-12:
            return 0.0
        return float(1.0 - ss_res / ss_tot)

    # ==================== Utilities ====================

    def _wait_for_position_stable(self, axis: str, timeout: float = 10.0) -> bool:
        """이동 후 위치 안정화 폴링 (연속 3회 < 0.05mm)"""
        axis_idx = {'x': 0, 'y': 1, 'z': 2}[axis]
        stable_count = 0
        deadline = time.time() + timeout

        pose = self._robot.read_current_pose()
        if pose is None:
            return False
        last_val = pose[axis_idx]

        while time.time() < deadline:
            time.sleep(0.2)
            QApplication.processEvents()
            if self._cancel_requested:
                return False
            pose_now = self._robot.read_current_pose()
            if pose_now is None:
                continue
            current_val = pose_now[axis_idx]
            if abs(current_val - last_val) < 0.05:
                stable_count += 1
                if stable_count >= 3:
                    return True
            else:
                stable_count = 0
            last_val = current_val
        return False

    def _return_to_origin(self):
        """모든 축을 원점으로 복귀"""
        if self._origin_pose is None:
            return
        current = self._robot.read_current_pose()
        if current is None:
            return

        for axis, idx in [('x', 0), ('y', 1), ('z', 2)]:
            delta = self._origin_pose[idx] - current[idx]
            if abs(delta) > 0.1:
                self._robot.send_base_linear(
                    axis, delta,
                    process_events_callback=QApplication.processEvents)
                time.sleep(0.2)
                self._wait_for_position_stable(axis)
                # Re-read position after each axis move
                current = self._robot.read_current_pose()
                if current is None:
                    break

    def _save_results(self, results: dict) -> str:
        """결과를 data/stereo/에 JSON + CSV로 저장"""
        data_dir = os.path.join(
            os.path.dirname(__file__), '..', '..', 'data', 'stereo')
        os.makedirs(data_dir, exist_ok=True)

        ts = datetime.now().strftime('%Y%m%d_%H%M%S')

        # 카메라 인트린식 메타데이터 (카메라 교체 시 sweep 무효화 검증용)
        camera_info = {}
        if self._ds435 and self._ds435.intrinsics:
            di = self._ds435.intrinsics
            camera_info['ds435'] = {
                'width': getattr(di, 'width', 0),
                'height': getattr(di, 'height', 0),
                'fy': getattr(di, 'fy', 0),
            }
        if self._arducam and self._arducam.intrinsics:
            ai = self._arducam.intrinsics
            camera_info['arducam'] = {
                'width': getattr(ai, 'width', 0),
                'height': getattr(ai, 'height', 0),
                'fy': getattr(ai, 'fy', 0),
            }

        save_data = {
            'timestamp': datetime.now().isoformat(),
            'origin_pose': [float(v) for v in self._origin_pose[:6]],
            'parameters': {
                'step_mm': self._step_mm,
                'count': self._count,
            },
            'camera_info': camera_info,
            'results': results,
            'raw_data': {
                axis: self._data[axis] for axis in ('z', 'x', 'y')
            },
        }

        # JSON 저장
        json_path = os.path.join(data_dir, f'sweep_{ts}.json')
        with open(json_path, 'w', encoding='utf-8') as f:
            json.dump(save_data, f, indent=2, ensure_ascii=False)

        # CSV 저장 (축, 이동, DS435 X/Y/Z, ArduCam X/Y)
        csv_path = os.path.join(data_dir, f'sweep_{ts}.csv')
        with open(csv_path, 'w', encoding='utf-8') as f:
            f.write('축,이동(mm),DS435 X(px),DS435 Y(px),DS435 Z(mm),'
                    'ArduCam X(px),ArduCam Y(px)\n')
            for axis in ('z', 'x', 'y'):
                for pt in self._data[axis]:
                    ds = pt.get('ds435')
                    ar = pt.get('arducam')
                    ds_x = f"{ds['midpoint'][0]:.1f}" if ds else '-'
                    ds_y = f"{ds['midpoint'][1]:.1f}" if ds else '-'
                    ds_z = f"{ds['depth_mm']:.1f}" if ds and ds.get('depth_mm') else '-'
                    ar_x = f"{ar['midpoint'][0]:.1f}" if ar else '-'
                    ar_y = f"{ar['midpoint'][1]:.1f}" if ar else '-'
                    f.write(f"{axis.upper()},{pt['robot_mm']:+.1f},"
                            f"{ds_x},{ds_y},{ds_z},{ar_x},{ar_y}\n")

        self._log(f"[Sweep] CSV 저장: {csv_path}")
        return json_path

    def _log(self, msg: str):
        """로그 시그널 발행"""
        self.log_message.emit(msg)
