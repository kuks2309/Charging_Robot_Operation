#!/usr/bin/env python3
"""
IPPE Ambiguity 진단 스크립트
- solvePnPGeneric(IPPE_SQUARE)의 2개 해를 비교 분석
- Z축 법선 방향, 오일러 각 차이, tvec 차이 출력
- 5초간 데이터 수집 후 통계 리포트

목적: Rx ±113°, ±144° 불안정의 근본 원인 파악
"""

import os
import sys
import time
import cv2
import yaml
import numpy as np

PROJECT_ROOT = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, PROJECT_ROOT)
CALIB_FILE = os.path.normpath(os.path.join(PROJECT_ROOT, '..', 'config', 'arducam_calibration.yaml'))

ARUCO_DICT_TYPE = cv2.aruco.DICT_5X5_50
MARKER_SIZE = 0.015
DEVICE_INDEX = 6
DURATION = 5.0  # seconds


def load_calibration(filepath):
    with open(filepath, 'r') as f:
        data = yaml.safe_load(f)
    cam = np.array(data['camera_matrix']['data'], dtype=np.float64).reshape(3, 3)
    dist = np.array(data['distortion_coefficients']['data'], dtype=np.float64)
    return cam, dist


def rotation_matrix_to_euler(R):
    sy = np.sqrt(R[0, 0]**2 + R[1, 0]**2)
    if sy > 1e-6:
        x = np.arctan2(R[2, 1], R[2, 2])
        y = np.arctan2(-R[2, 0], sy)
        z = np.arctan2(R[1, 0], R[0, 0])
    else:
        x = np.arctan2(-R[1, 2], R[1, 1])
        y = np.arctan2(-R[2, 0], sy)
        z = 0
    return np.degrees([x, y, z])  # raw [-180, 180]


def main():
    camera_matrix, dist_coeffs = load_calibration(CALIB_FILE)
    print(f"Calibration loaded: fx={camera_matrix[0,0]:.1f}, fy={camera_matrix[1,1]:.1f}")

    aruco_dict = cv2.aruco.getPredefinedDictionary(ARUCO_DICT_TYPE)
    detector = cv2.aruco.ArucoDetector(aruco_dict, cv2.aruco.DetectorParameters())

    obj_points = np.array([
        [-MARKER_SIZE/2,  MARKER_SIZE/2, 0],
        [ MARKER_SIZE/2,  MARKER_SIZE/2, 0],
        [ MARKER_SIZE/2, -MARKER_SIZE/2, 0],
        [-MARKER_SIZE/2, -MARKER_SIZE/2, 0],
    ], dtype=np.float32)

    cap = cv2.VideoCapture(DEVICE_INDEX)
    if not cap.isOpened():
        print(f"Camera open failed (device_index={DEVICE_INDEX})")
        sys.exit(1)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

    # 데이터 저장: marker_id -> list of frame_data
    all_data = {}  # marker_id -> [{'sol0_euler', 'sol1_euler', 'sol0_tvec', 'sol1_tvec', 'sol0_z_axis', 'sol1_z_axis', 'reproj0', 'reproj1'}]

    print(f"\n{'='*70}")
    print(f"IPPE Ambiguity 진단: {DURATION}초 데이터 수집")
    print(f"{'='*70}\n")

    start_time = time.time()
    frame_count = 0

    while time.time() - start_time < DURATION:
        ret, frame = cap.read()
        if not ret:
            continue

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = detector.detectMarkers(gray)

        if ids is None:
            continue

        frame_count += 1

        for i, marker_id in enumerate(ids.flatten()):
            n_solutions, rvecs, tvecs, reproj_errors = cv2.solvePnPGeneric(
                obj_points, corners[i].reshape(-1, 2),
                camera_matrix, dist_coeffs,
                flags=cv2.SOLVEPNP_IPPE_SQUARE
            )

            if n_solutions < 2:
                continue

            # 각 해의 정보 추출
            sols_info = []
            for s in range(2):
                rv = rvecs[s].flatten()
                tv = tvecs[s].flatten()
                R, _ = cv2.Rodrigues(rv)
                euler = rotation_matrix_to_euler(R)
                z_axis = R[:, 2]  # Z축 법선 (마커 표면 법선)
                reproj = reproj_errors[s][0] if reproj_errors is not None else 0
                sols_info.append({
                    'euler': euler,
                    'tvec': tv,
                    'rvec': rv,
                    'z_axis': z_axis,
                    'reproj': reproj,
                    'R': R,
                })

            marker_id = int(marker_id)
            if marker_id not in all_data:
                all_data[marker_id] = []

            all_data[marker_id].append({
                'sol0': sols_info[0],
                'sol1': sols_info[1],
            })

    cap.release()

    elapsed = time.time() - start_time
    print(f"수집 완료: {frame_count} frames / {elapsed:.1f}s\n")

    # =========================================================================
    # 분석 리포트
    # =========================================================================
    for mid in sorted(all_data.keys()):
        frames = all_data[mid]
        n = len(frames)
        print(f"{'='*70}")
        print(f"  마커 ID {mid}: {n} frames 분석")
        print(f"{'='*70}")

        # 1. tvec 차이 (Sol0 vs Sol1)
        tvec_diffs = []
        for f in frames:
            diff = np.linalg.norm(f['sol0']['tvec'] - f['sol1']['tvec']) * 1000  # mm
            tvec_diffs.append(diff)
        tvec_diffs = np.array(tvec_diffs)
        print(f"\n[1] tvec 차이 (Sol0 vs Sol1):")
        print(f"    ||tvec0 - tvec1|| = {tvec_diffs.mean():.3f} ± {tvec_diffs.std():.3f} mm")
        print(f"    range: [{tvec_diffs.min():.3f}, {tvec_diffs.max():.3f}] mm")
        if tvec_diffs.mean() < 1.0:
            print(f"    → tvec이 거의 동일 → 거리 기반 disambiguation 불가능!")

        # 2. Z축 법선 비교
        z_dots = []
        for f in frames:
            dot = np.dot(f['sol0']['z_axis'], f['sol1']['z_axis'])
            z_dots.append(dot)
        z_dots = np.array(z_dots)
        z_angles = np.degrees(np.arccos(np.clip(z_dots, -1, 1)))
        print(f"\n[2] Z축 법선 각도 차이 (Sol0 vs Sol1):")
        print(f"    각도 = {z_angles.mean():.1f} ± {z_angles.std():.1f}°")
        print(f"    range: [{z_angles.min():.1f}, {z_angles.max():.1f}]°")
        print(f"    내적 = {z_dots.mean():.3f} ± {z_dots.std():.3f}")
        if z_dots.mean() < -0.5:
            print(f"    → Z축이 거의 반대 방향 → 전형적인 IPPE 뒤집힘(flip)")

        # 3. 각 해의 Z축이 카메라를 향하는지 확인
        # 카메라 좌표계에서 마커가 카메라 앞에 있으면 tvec_z > 0
        # 마커의 Z축 법선이 카메라를 향하면 z_axis의 z성분 < 0 (카메라 방향으로)
        z_toward_cam_sol0 = []
        z_toward_cam_sol1 = []
        for f in frames:
            # Z축의 z성분이 음수 → 마커 법선이 카메라를 향함
            z_toward_cam_sol0.append(f['sol0']['z_axis'][2] < 0)
            z_toward_cam_sol1.append(f['sol1']['z_axis'][2] < 0)

        pct0 = sum(z_toward_cam_sol0) / n * 100
        pct1 = sum(z_toward_cam_sol1) / n * 100
        print(f"\n[3] Z축이 카메라를 향하는 비율:")
        print(f"    Sol0: {pct0:.1f}% (z_axis.z < 0)")
        print(f"    Sol1: {pct1:.1f}% (z_axis.z < 0)")
        print(f"    → 마커가 카메라를 바라보고 있으면, z_axis.z < 0인 해가 정답")

        # 4. Euler 각 비교
        sol0_eulers = np.array([f['sol0']['euler'] for f in frames])
        sol1_eulers = np.array([f['sol1']['euler'] for f in frames])

        print(f"\n[4] Euler 각 통계 (+/- 그룹 분리):")
        for axis_idx, axis_name in enumerate(['Rx', 'Ry', 'Rz']):
            s0 = sol0_eulers[:, axis_idx]
            s1 = sol1_eulers[:, axis_idx]
            # 전체 통계
            print(f"    {axis_name} Sol0: {s0.mean():7.1f} ± {s0.std():5.1f}° [{s0.min():7.1f}, {s0.max():7.1f}]")
            print(f"    {axis_name} Sol1: {s1.mean():7.1f} ± {s1.std():5.1f}° [{s1.min():7.1f}, {s1.max():7.1f}]")
            # ±180° 경계 감지 시 그룹 분리
            if s0.std() > 30:
                pos = s0[s0 >= 0]
                neg = s0[s0 < 0]
                if len(pos) > 0 and len(neg) > 0:
                    print(f"      → Sol0 (+)그룹: {pos.mean():7.1f} ± {pos.std():5.1f}° (n={len(pos)}, {len(pos)/n*100:.0f}%)")
                    print(f"      → Sol0 (-)그룹: {neg.mean():7.1f} ± {neg.std():5.1f}° (n={len(neg)}, {len(neg)/n*100:.0f}%)")
                    # unwrap 적용 통계
                    unwrapped = s0.copy()
                    unwrapped[unwrapped < 0] += 360
                    print(f"      → Sol0 unwrap:  {unwrapped.mean():7.1f} ± {unwrapped.std():5.1f}°")

        # 5. Reprojection error 비교
        reproj0 = np.array([f['sol0']['reproj'] for f in frames])
        reproj1 = np.array([f['sol1']['reproj'] for f in frames])
        print(f"\n[5] Reprojection Error:")
        print(f"    Sol0: {reproj0.mean():.4f} ± {reproj0.std():.4f}")
        print(f"    Sol1: {reproj1.mean():.4f} ± {reproj1.std():.4f}")
        ratio = reproj1.mean() / reproj0.mean() if reproj0.mean() > 0 else 0
        print(f"    ratio(Sol1/Sol0): {ratio:.2f}")
        if ratio < 2.0:
            print(f"    → reprojection error 차이가 작아 error만으로 구분 어려움")

        # 6. Z축 기반 올바른 해 선택 시 안정성 검증
        # 마커가 카메라를 바라보고 있으므로, z_axis.z < 0인 해 선택
        correct_eulers = []
        for f in frames:
            if f['sol0']['z_axis'][2] < 0:
                correct_eulers.append(f['sol0']['euler'])
            else:
                correct_eulers.append(f['sol1']['euler'])
        correct_eulers = np.array(correct_eulers)

        print(f"\n[6] Z축 법선 기준 올바른 해 선택 시 Euler 각:")
        for axis_idx, axis_name in enumerate(['Rx', 'Ry', 'Rz']):
            vals = correct_eulers[:, axis_idx]
            print(f"    {axis_name}: {vals.mean():7.1f} ± {vals.std():5.1f}° [{vals.min():7.1f}, {vals.max():7.1f}]")

        # Sol0이 선택된 비율
        sol0_selected = sum(1 for f in frames if f['sol0']['z_axis'][2] < 0)
        print(f"    Sol0 선택 비율: {sol0_selected/n*100:.1f}%")
        print()

    # =========================================================================
    # 종합 결론
    # =========================================================================
    print(f"{'='*70}")
    print("  종합 분석 결론")
    print(f"{'='*70}")
    print("""
[원인]
  IPPE (Infinitesimal Planar Pose Estimation)는 평면 마커에서
  2개의 수학적으로 유효한 자세를 반환합니다.
  - 두 해의 tvec(위치)은 거의 동일
  - 두 해의 Z축 법선 방향은 거의 반대 (~180° 차이)
  - Reprojection error도 유사하여 error만으로 구분 불가

  기존 distance-based disambiguation은 tvec이 같으므로 원리적으로
  rotation ambiguity를 해결할 수 없습니다.

[해결책]
  1. Z축 법선 방향 기준: 마커의 Z축이 카메라를 향하는 해 선택
     (z_axis.z < 0 인 해, 카메라 좌표계에서)
  2. 이 방법은 물리적 구속조건 활용:
     "마커는 항상 카메라를 바라보고 있다"
""")


if __name__ == "__main__":
    main()
