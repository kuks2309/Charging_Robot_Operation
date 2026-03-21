#!/usr/bin/env python3
"""
ArUco 검출 신뢰성 검증 테스트

ArduCam으로 50회 연속 캡처하여 ArUco 마커 검출의 일관성을 평가한다.
- 검출 성공률
- tvec / rvec / euler 각도의 평균, 표준편차
- 코너 픽셀 좌표 안정성
- solvePnP (IPPE) 기반 pose 추정 재현성

Usage:
    cd scripts && python -m tests.test_aruco_reliability
    또는
    cd scripts/tests && python test_aruco_reliability.py
"""

import os
import sys
import time
import csv
from datetime import datetime
from collections import defaultdict

import cv2
import numpy as np
import yaml

# sys.path 설정
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
SCRIPTS_DIR = os.path.join(SCRIPT_DIR, "..")
if SCRIPTS_DIR not in sys.path:
    sys.path.insert(0, os.path.abspath(SCRIPTS_DIR))

from utils.camera_utils import detect_arducam_index

# ── 설정 ──────────────────────────────────────────────
NUM_TRIALS = 50
MARKER_SIZE = 0.015  # 15mm
ARUCO_DICT_TYPE = cv2.aruco.DICT_5X5_50
WARMUP_FRAMES = 10  # 카메라 안정화용 버림 프레임 수
CAPTURE_DELAY = 0.1  # 캡처 간 대기 (초)

PROJECT_ROOT = os.path.normpath(os.path.join(SCRIPTS_DIR, ".."))
CALIB_FILE = os.path.join(
    PROJECT_ROOT, "config", "calibration", "arducam", "arducam_calibration.yaml"
)


def load_calibration(filepath):
    """YAML 캘리브레이션 파일 로드 → (camera_matrix, dist_coeffs)"""
    with open(filepath, "r") as f:
        data = yaml.safe_load(f)
    cm = data["camera_matrix"]
    camera_matrix = np.array(cm["data"], dtype=np.float64).reshape(cm["rows"], cm["cols"])
    dc = data["distortion_coefficients"]
    dist_coeffs = np.array(dc["data"], dtype=np.float64).reshape(dc["rows"], dc["cols"])
    return camera_matrix, dist_coeffs


def rotation_matrix_to_euler(R):
    """회전행렬 → ZYX Euler (degrees)"""
    sy = np.sqrt(R[0, 0] ** 2 + R[1, 0] ** 2)
    if sy > 1e-6:
        x = np.arctan2(R[2, 1], R[2, 2])
        y = np.arctan2(-R[2, 0], sy)
        z = np.arctan2(R[1, 0], R[0, 0])
    else:
        x = np.arctan2(-R[1, 2], R[1, 1])
        y = np.arctan2(-R[2, 0], sy)
        z = 0
    return np.degrees([x, y, z])


def print_separator(title=""):
    width = 70
    if title:
        pad = (width - len(title) - 2) // 2
        print(f"\n{'=' * pad} {title} {'=' * pad}")
    else:
        print("=" * width)


def main():
    # ── 캘리브레이션 로드 ──
    if not os.path.exists(CALIB_FILE):
        print(f"[ERROR] 캘리브레이션 파일 없음: {CALIB_FILE}")
        sys.exit(1)
    camera_matrix, dist_coeffs = load_calibration(CALIB_FILE)
    print(f"[OK] 캘리브레이션 로드: {CALIB_FILE}")

    # ── 카메라 열기 ──
    dev_idx = detect_arducam_index()
    if dev_idx is None:
        print("[ERROR] ArduCam 장치를 찾을 수 없습니다")
        sys.exit(1)

    cap = cv2.VideoCapture(dev_idx)
    if not cap.isOpened():
        print(f"[ERROR] 카메라 열기 실패 (device_index={dev_idx})")
        sys.exit(1)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)
    actual_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    actual_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    print(f"[OK] ArduCam 열림: device={dev_idx}, {actual_w}x{actual_h}")

    # 카메라 안정화
    for _ in range(WARMUP_FRAMES):
        cap.read()

    # ── ArUco 설정 ──
    aruco_dict = cv2.aruco.getPredefinedDictionary(ARUCO_DICT_TYPE)
    detector = cv2.aruco.ArucoDetector(aruco_dict, cv2.aruco.DetectorParameters())

    obj_points = np.array(
        [
            [-MARKER_SIZE / 2, MARKER_SIZE / 2, 0],
            [MARKER_SIZE / 2, MARKER_SIZE / 2, 0],
            [MARKER_SIZE / 2, -MARKER_SIZE / 2, 0],
            [-MARKER_SIZE / 2, -MARKER_SIZE / 2, 0],
        ],
        dtype=np.float32,
    )

    # ── 데이터 수집 ──
    # marker_id별 결과 축적
    results = defaultdict(lambda: {
        "tvecs": [],       # (N, 3) mm
        "rvecs": [],       # (N, 3)
        "eulers": [],      # (N, 3) degrees
        "centers_px": [],  # (N, 2) pixel
        "corners_px": [],  # (N, 4, 2) pixel
        "reproj_errs": [], # (N,)
    })

    detect_count = 0  # 1개 이상 검출된 프레임 수
    total_markers_detected = 0

    print_separator("ArUco 신뢰성 테스트 시작")
    print(f"  캡처 횟수: {NUM_TRIALS}")
    print(f"  마커 사전: DICT_5X5_50, 크기: {MARKER_SIZE * 1000:.0f}mm")
    print(f"  캡처 간격: {CAPTURE_DELAY}s")
    print()

    for trial in range(NUM_TRIALS):
        ret, frame = cap.read()
        if not ret:
            print(f"  [{trial + 1:3d}/{NUM_TRIALS}] 프레임 캡처 실패")
            continue

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, rejected = detector.detectMarkers(gray)

        if ids is None or len(ids) == 0:
            print(f"  [{trial + 1:3d}/{NUM_TRIALS}] 검출 실패 (rejected={len(rejected) if rejected else 0})")
            time.sleep(CAPTURE_DELAY)
            continue

        detect_count += 1
        total_markers_detected += len(ids)

        for i, marker_id in enumerate(ids.flatten()):
            mid = int(marker_id)
            corner_2d = corners[i].reshape(-1, 2)

            # solvePnP (IPPE_SQUARE)
            n_sol, rvecs, tvecs, reproj_errors = cv2.solvePnPGeneric(
                obj_points, corner_2d, camera_matrix, dist_coeffs,
                flags=cv2.SOLVEPNP_IPPE_SQUARE,
            )
            if n_sol == 0:
                continue

            # z22 < 0 (마커가 카메라를 향하는 해) 선택
            best_idx = 0
            for s in range(n_sol):
                R_s, _ = cv2.Rodrigues(rvecs[s].flatten())
                if R_s[2, 2] < 0:
                    best_idx = s
                    break

            rv = rvecs[best_idx].flatten()
            tv = tvecs[best_idx].flatten()
            err = reproj_errors[best_idx][0] if reproj_errors is not None else -1

            # LM refinement
            rv_ref, tv_ref = cv2.solvePnPRefineLM(
                obj_points, corner_2d, camera_matrix, dist_coeffs,
                rv.reshape(3, 1), tv.reshape(3, 1),
            )
            rv = rv_ref.flatten()
            tv = tv_ref.flatten()

            R, _ = cv2.Rodrigues(rv)
            euler = rotation_matrix_to_euler(R)

            center = corner_2d.mean(axis=0)

            results[mid]["tvecs"].append(tv * 1000)  # mm 단위
            results[mid]["rvecs"].append(rv)
            results[mid]["eulers"].append(euler)
            results[mid]["centers_px"].append(center)
            results[mid]["corners_px"].append(corner_2d)
            results[mid]["reproj_errs"].append(err)

        detected_ids = ids.flatten().tolist()
        print(f"  [{trial + 1:3d}/{NUM_TRIALS}] IDs={detected_ids} OK")

        time.sleep(CAPTURE_DELAY)

    cap.release()

    # ── 결과 분석 ──
    print_separator("검출 결과 요약")
    print(f"  총 캡처:          {NUM_TRIALS}")
    print(f"  검출 성공 프레임:  {detect_count} / {NUM_TRIALS}  ({detect_count / NUM_TRIALS * 100:.1f}%)")
    print(f"  총 마커 검출 수:   {total_markers_detected}")
    print()

    if not results:
        print("[FAIL] 마커가 한 번도 검출되지 않았습니다.")
        sys.exit(1)

    # CSV 저장 준비
    save_dir = os.path.join(PROJECT_ROOT, "data", "aruco_reliability")
    os.makedirs(save_dir, exist_ok=True)
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")

    for mid in sorted(results.keys()):
        r = results[mid]
        n = len(r["tvecs"])

        tvecs = np.array(r["tvecs"])       # (N, 3) mm
        rvecs = np.array(r["rvecs"])       # (N, 3)
        eulers = np.array(r["eulers"])     # (N, 3) deg
        centers = np.array(r["centers_px"])  # (N, 2) px
        reproj = np.array(r["reproj_errs"])

        print_separator(f"마커 ID {mid}  (검출 {n}/{NUM_TRIALS}회, {n / NUM_TRIALS * 100:.1f}%)")

        # tvec 통계 (mm)
        t_mean = tvecs.mean(axis=0)
        t_std = tvecs.std(axis=0)
        t_range = tvecs.max(axis=0) - tvecs.min(axis=0)
        print(f"\n  [tvec (mm)]")
        print(f"    {'':8s} {'X':>10s} {'Y':>10s} {'Z':>10s}")
        print(f"    {'평균':8s} {t_mean[0]:10.2f} {t_mean[1]:10.2f} {t_mean[2]:10.2f}")
        print(f"    {'표준편차':8s} {t_std[0]:10.3f} {t_std[1]:10.3f} {t_std[2]:10.3f}")
        print(f"    {'범위':8s} {t_range[0]:10.3f} {t_range[1]:10.3f} {t_range[2]:10.3f}")

        # euler 통계 (deg)
        e_mean = eulers.mean(axis=0)
        e_std = eulers.std(axis=0)
        e_range = eulers.max(axis=0) - eulers.min(axis=0)
        print(f"\n  [euler (deg)]")
        print(f"    {'':8s} {'Rx':>10s} {'Ry':>10s} {'Rz':>10s}")
        print(f"    {'평균':8s} {e_mean[0]:10.2f} {e_mean[1]:10.2f} {e_mean[2]:10.2f}")
        print(f"    {'표준편차':8s} {e_std[0]:10.3f} {e_std[1]:10.3f} {e_std[2]:10.3f}")
        print(f"    {'범위':8s} {e_range[0]:10.3f} {e_range[1]:10.3f} {e_range[2]:10.3f}")

        # 코너 픽셀 안정성
        corners_all = np.array(r["corners_px"])  # (N, 4, 2)
        c_std = corners_all.std(axis=0)  # (4, 2)
        print(f"\n  [코너 픽셀 안정성 (std, px)]")
        print(f"    {'코너':8s} {'X':>8s} {'Y':>8s}")
        for ci in range(4):
            print(f"    {'C' + str(ci):8s} {c_std[ci, 0]:8.3f} {c_std[ci, 1]:8.3f}")

        # 중심 픽셀 안정성
        cx_std, cy_std = centers.std(axis=0)
        print(f"\n  [중심 픽셀 (std, px)]")
        print(f"    X std: {cx_std:.3f},  Y std: {cy_std:.3f}")

        # reprojection error
        print(f"\n  [Reprojection Error]")
        print(f"    평균: {reproj.mean():.4f},  최대: {reproj.max():.4f},  최소: {reproj.min():.4f}")

        # 듀얼 마커 거리 (2개 이상 마커가 있을 때)
        other_ids = [k for k in sorted(results.keys()) if k != mid]
        if other_ids:
            for oid in other_ids:
                o_tvecs = np.array(results[oid]["tvecs"])
                # 공통 검출 프레임 수 (min)
                common = min(n, len(o_tvecs))
                if common > 0:
                    dists = np.linalg.norm(tvecs[:common] - o_tvecs[:common], axis=1)
                    print(f"\n  [마커간 거리 ID{mid}-ID{oid} (mm)]")
                    print(f"    평균: {dists.mean():.2f},  std: {dists.std():.3f},  "
                          f"범위: {dists.min():.2f} ~ {dists.max():.2f}")

        # CSV 저장
        csv_path = os.path.join(save_dir, f"reliability_id{mid}_{timestamp}.csv")
        with open(csv_path, "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow([
                "trial", "tvec_x_mm", "tvec_y_mm", "tvec_z_mm",
                "rvec_x", "rvec_y", "rvec_z",
                "euler_rx", "euler_ry", "euler_rz",
                "center_px_x", "center_px_y", "reproj_err",
            ])
            for j in range(n):
                writer.writerow([
                    j + 1,
                    f"{tvecs[j, 0]:.4f}", f"{tvecs[j, 1]:.4f}", f"{tvecs[j, 2]:.4f}",
                    f"{rvecs[j, 0]:.6f}", f"{rvecs[j, 1]:.6f}", f"{rvecs[j, 2]:.6f}",
                    f"{eulers[j, 0]:.4f}", f"{eulers[j, 1]:.4f}", f"{eulers[j, 2]:.4f}",
                    f"{centers[j, 0]:.2f}", f"{centers[j, 1]:.2f}",
                    f"{reproj[j]:.6f}",
                ])
        print(f"\n  CSV 저장: {csv_path}")

    # ── 판정 ──
    print_separator("판정")
    all_pass = True
    for mid in sorted(results.keys()):
        n = len(results[mid]["tvecs"])
        rate = n / NUM_TRIALS * 100
        t_std = np.array(results[mid]["tvecs"]).std(axis=0)
        e_std = np.array(results[mid]["eulers"]).std(axis=0)

        # 판정 기준: 검출률 90% 이상, tvec std < 1mm, euler std < 1deg
        det_ok = rate >= 90.0
        t_ok = np.all(t_std < 1.0)
        e_ok = np.all(e_std < 1.0)
        passed = det_ok and t_ok and e_ok

        status = "PASS" if passed else "FAIL"
        if not passed:
            all_pass = False

        print(f"  ID {mid}: [{status}]  검출={rate:.0f}%  "
              f"tvec_std=({t_std[0]:.3f}, {t_std[1]:.3f}, {t_std[2]:.3f})mm  "
              f"euler_std=({e_std[0]:.3f}, {e_std[1]:.3f}, {e_std[2]:.3f})deg")

    print()
    if all_pass:
        print("  >>> 전체 PASS: ArUco 검출 신뢰성 양호")
    else:
        print("  >>> FAIL 항목 존재: 검출률/안정성 확인 필요")

    print_separator()


if __name__ == "__main__":
    main()
