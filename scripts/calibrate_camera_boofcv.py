#!/usr/bin/env python3
"""
BoofCV를 사용한 카메라 캘리브레이션 스크립트

체스보드 이미지로 카메라 내부 파라미터를 계산합니다.
- OpenCV로 체스보드 코너 감지 (도구)
- BoofCV Brown 모델로 캘리브레이션 (알고리즘)
- 결과를 .npz 형식으로 저장 (기존 프로젝트 호환)

사용법:
    python scripts/calibrate_camera_boofcv.py

출력:
    calibration/camera/calibration_boofcv.npz
"""

import os
import sys
import glob
import numpy as np
import cv2
from pathlib import Path
from dataclasses import dataclass

# PyBoof import
try:
    from pyboof import (
        FactoryFiducialCalibration,
        ConfigGridDimen,
        calibrate_brown
    )
except ImportError:
    print("오류: PyBoof가 설치되지 않았습니다.")
    print("설치: source charging_robot/bin/activate && pip install PyBoof")
    sys.exit(1)


# ===== 데이터 구조 =====
@dataclass
class CalibrationResult:
    """캘리브레이션 결과 데이터 구조체"""
    camera_matrix: np.ndarray
    dist_coeffs: np.ndarray
    rms_error: float
    errors: list
    image_shape: tuple  # (width, height)

    @property
    def width(self):
        return self.image_shape[0]

    @property
    def height(self):
        return self.image_shape[1]

    @property
    def fx(self):
        return self.camera_matrix[0, 0]

    @property
    def fy(self):
        return self.camera_matrix[1, 1]

    @property
    def cx(self):
        return self.camera_matrix[0, 2]

    @property
    def cy(self):
        return self.camera_matrix[1, 2]

    @property
    def k1(self):
        return self.dist_coeffs[0]

    @property
    def k2(self):
        return self.dist_coeffs[1]

    @property
    def p1(self):
        return self.dist_coeffs[2]

    @property
    def p2(self):
        return self.dist_coeffs[3]

    @property
    def k3(self):
        return self.dist_coeffs[4]


# ===== 유틸리티 함수 =====
def ensure_directory(file_path):
    """
    파일 경로의 부모 디렉토리가 존재하는지 확인하고 생성

    Args:
        file_path: Path 객체
    """
    file_path.parent.mkdir(parents=True, exist_ok=True)


# ===== 설정 =====
# 체스보드 설정 (11x8 전체 그리드 = 10x7 내부 코너)
CHESSBOARD_COLS = 10      # 내부 코너 열 수
CHESSBOARD_ROWS = 7       # 내부 코너 행 수
SQUARE_SIZE_MM = 22.0     # 정사각형 크기 (mm)

# 이미지 경로
PROJECT_ROOT = Path(__file__).parent.parent
IMAGE_DIR = PROJECT_ROOT / "calibration" / "camera" / "auto_20260124_155100"
OUTPUT_FILE = PROJECT_ROOT / "calibration" / "camera" / "calibration_boofcv.npz"
OUTPUT_YAML = PROJECT_ROOT / "config" / "ds435_calibration.yaml"


def load_images(image_dir):
    """
    이미지 디렉토리에서 모든 PNG 파일 로드

    Args:
        image_dir: 이미지 디렉토리 경로

    Returns:
        list: (filepath, image) 튜플 리스트
    """
    image_files = sorted(glob.glob(str(image_dir / "*.png")))

    if not image_files:
        print(f"오류: {image_dir}에 이미지가 없습니다.")
        sys.exit(1)

    print(f"이미지 {len(image_files)}개 발견")

    images = []
    for filepath in image_files:
        img = cv2.imread(filepath)
        if img is not None:
            images.append((filepath, img))
        else:
            print(f"경고: {filepath} 로드 실패")

    print(f"이미지 {len(images)}개 로드 완료")
    return images


def detect_chessboards_opencv(images, cols, rows):
    """
    OpenCV로 모든 이미지에서 체스보드 감지

    Args:
        images: (filepath, image) 튜플 리스트
        cols: 내부 코너 열 수
        rows: 내부 코너 행 수

    Returns:
        tuple: (observations, image_shape, success_count)
            observations: PyBoof 형식 [{"pixels": [(idx, x, y), ...]}, ...]
            image_shape: (width, height)
            success_count: 성공한 이미지 수
    """
    observations = []
    success_count = 0
    image_shape = None

    # 서브픽셀 정밀도 기준
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

    print("\n체스보드 감지 시작 (OpenCV)...")

    for idx, (filepath, img) in enumerate(images):
        # 이미지 크기 저장 (첫 번째 이미지에서)
        if image_shape is None:
            h, w = img.shape[:2]
            image_shape = (w, h)  # (width, height)
            print(f"이미지 크기 감지: {w}x{h}")

        # Grayscale 변환
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        # 체스보드 코너 감지
        ret, corners = cv2.findChessboardCorners(gray, (cols, rows), None)

        if ret:
            # 서브픽셀 정밀도로 코너 위치 개선
            corners_refined = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)

            # PyBoof 형식으로 변환: {"pixels": [(index, x, y), ...]}
            pixels = []
            for point_idx, corner in enumerate(corners_refined):
                x, y = corner.ravel()
                pixels.append((point_idx, float(x), float(y)))

            observations.append({"pixels": pixels})
            success_count += 1

            if (idx + 1) % 20 == 0:
                print(f"  진행: {idx + 1}/{len(images)} (성공: {success_count})")
        else:
            # 감지 실패
            filename = os.path.basename(filepath)
            print(f"  [{idx + 1}] 감지 실패: {filename}")

    print(f"\n감지 완료: {success_count}/{len(images)} 성공")

    if success_count < 10:
        print(f"경고: 성공한 이미지가 {success_count}개로 너무 적습니다 (최소 10개 권장)")
        print("캘리브레이션을 계속 진행합니다...")

    return observations, image_shape, success_count


def calibrate_with_boofcv(observations, image_shape, cols, rows, square_size):
    """
    BoofCV Brown 모델로 카메라 캘리브레이션 실행

    Args:
        observations: PyBoof 형식 관측 데이터
        image_shape: (width, height)
        cols: 체스보드 내부 코너 열 수
        rows: 체스보드 내부 코너 행 수
        square_size: 정사각형 크기 (mm)

    Returns:
        tuple: (camera_matrix, dist_coeffs, rms_error, errors) 또는 None
    """
    print("\nBoofCV 캘리브레이션 실행 중...")

    width, height = image_shape

    try:
        # BoofCV ConfigGridDimen은 전체 그리드 크기 필요 (내부 코너 + 1)
        grid_cols = cols + 1  # 10 + 1 = 11
        grid_rows = rows + 1  # 7 + 1 = 8

        # BoofCV 체스보드 감지기 생성 (layout 제공용)
        # ConfigGridDimen 파라미터 순서: (rows, cols)
        config_grid = ConfigGridDimen(grid_rows, grid_cols, square_size)
        detector = FactoryFiducialCalibration.chessboardX(config_grid)

        # BoofCV Brown 캘리브레이션
        intrinsic, errors = calibrate_brown(
            width=width,
            height=height,
            observations=observations,
            detector=detector,
            num_radial=2,      # 방사 왜곡 계수 2개 (k1, k2)
            tangential=True,   # 접선 왜곡 포함 (p1, p2)
            zero_skew=True     # skew 0으로 고정
        )

        # 카메라 매트릭스 생성 (OpenCV 호환)
        fx = intrinsic.fx
        fy = intrinsic.fy
        cx = intrinsic.cx
        cy = intrinsic.cy

        camera_matrix = np.array([
            [fx,  0, cx],
            [ 0, fy, cy],
            [ 0,  0,  1]
        ], dtype=np.float64)

        # 왜곡 계수 (OpenCV 형식: [k1, k2, p1, p2, k3])
        radial = intrinsic.radial  # list
        t1 = intrinsic.t1
        t2 = intrinsic.t2

        if len(radial) >= 2:
            k1, k2 = radial[0], radial[1]
        else:
            k1, k2 = 0.0, 0.0

        dist_coeffs = np.array([k1, k2, t1, t2, 0.0], dtype=np.float64)

        # RMS 오차 계산 (평균 오차)
        if errors:
            mean_errors = [e["mean"] for e in errors]
            rms_error = np.sqrt(np.mean(np.array(mean_errors) ** 2))
        else:
            rms_error = 0.0

        print(f"\n캘리브레이션 성공!")
        print(f"  카메라 매트릭스:")
        print(f"    fx = {fx:.2f}")
        print(f"    fy = {fy:.2f}")
        print(f"    cx = {cx:.2f}")
        print(f"    cy = {cy:.2f}")
        print(f"  왜곡 계수:")
        print(f"    k1 = {k1:.6f}")
        print(f"    k2 = {k2:.6f}")
        print(f"    p1 = {t1:.6f}")
        print(f"    p2 = {t2:.6f}")
        print(f"  RMS 오차: {rms_error:.4f} pixels")

        return CalibrationResult(
            camera_matrix=camera_matrix,
            dist_coeffs=dist_coeffs,
            rms_error=rms_error,
            errors=errors,
            image_shape=image_shape
        )

    except Exception as e:
        print(f"캘리브레이션 실패: {e}")
        import traceback
        traceback.print_exc()
        return None


def save_calibration(output_file, result: CalibrationResult):
    """
    캘리브레이션 결과를 .npz 파일로 저장

    Args:
        output_file: 출력 파일 경로
        result: CalibrationResult 객체
    """
    # 출력 디렉토리 생성
    ensure_directory(output_file)

    # .npz 형식으로 저장 (기존 프로젝트 호환)
    np.savez(
        str(output_file),
        camera_matrix=result.camera_matrix,
        dist_coeffs=result.dist_coeffs,
        rms_error=result.rms_error
    )

    print(f"\n저장 완료: {output_file}")


def save_calibration_yaml(output_file, result: CalibrationResult):
    """
    캘리브레이션 결과를 YAML 파일로 저장 (기존 ds435_calibration.yaml 형식)

    Args:
        output_file: 출력 파일 경로
        result: CalibrationResult 객체
    """
    import yaml

    # 출력 디렉토리 생성
    ensure_directory(output_file)

    # YAML 데이터 구조 (기존 형식과 동일)
    calib_data = {
        'camera_name': 'Intel_RealSense_D435',
        'serial_number': '207522071359',
        'image_width': int(result.width),
        'image_height': int(result.height),
        'camera_matrix': {
            'rows': 3,
            'cols': 3,
            'data': [
                float(result.fx), 0.0, float(result.cx),
                0.0, float(result.fy), float(result.cy),
                0.0, 0.0, 1.0
            ]
        },
        'fx': float(result.fx),
        'fy': float(result.fy),
        'cx': float(result.cx),
        'cy': float(result.cy),
        'distortion_coefficients': {
            'rows': 1,
            'cols': 5,
            'data': [float(result.k1), float(result.k2), float(result.p1), float(result.p2), float(result.k3)]
        },
        'k1': float(result.k1),
        'k2': float(result.k2),
        'p1': float(result.p1),
        'p2': float(result.p2),
        'k3': float(result.k3),
        'reprojection_error': float(result.rms_error)
    }

    # YAML 파일로 저장
    with open(output_file, 'w') as f:
        f.write("# Intel RealSense D435 Camera Calibration Data\n")
        f.write("# Generated by BoofCV calibration\n\n")
        yaml.dump(calib_data, f, default_flow_style=False, sort_keys=False)

    print(f"YAML 저장 완료: {output_file}")


def print_error_analysis(errors):
    """
    이미지별 캘리브레이션 오차 분석 및 출력

    Args:
        errors: BoofCV 오차 리스트 [{"mean": ..., "max_error": ..., ...}, ...]
    """
    if not errors:
        print("오차 정보가 없습니다.")
        return

    print("\n" + "=" * 60)
    print(f"이미지별 캘리브레이션 오차 분석 ({len(errors)}개)")
    print("=" * 60)

    # 통계 계산
    mean_errors = [e["mean"] for e in errors]

    avg_mean = np.mean(mean_errors)
    min_mean = np.min(mean_errors)
    max_mean = np.max(mean_errors)
    min_idx = np.argmin(mean_errors)
    max_idx = np.argmax(mean_errors)

    # 통계 출력
    print(f"\n통계:")
    print(f"  평균 오차: {avg_mean:.4f} pixels")
    print(f"  최소 오차: {min_mean:.4f} pixels (이미지 #{min_idx})")
    print(f"  최대 오차: {max_mean:.4f} pixels (이미지 #{max_idx})")

    # 오차가 높은 이미지 목록 (> 0.5 pixels)
    high_error_threshold = 0.5
    high_errors = [(idx, e) for idx, e in enumerate(errors) if e["mean"] > high_error_threshold]

    if high_errors:
        print(f"\n오차가 높은 이미지 (> {high_error_threshold} pixels):")
        for idx, e in sorted(high_errors, key=lambda x: x[1]["mean"], reverse=True)[:10]:
            print(f"  [#{idx:3d}] 평균: {e['mean']:.4f}px, 최대: {e['max_error']:.4f}px, "
                  f"편향: ({e['bias_x']:.4f}, {e['bias_y']:.4f})")
    else:
        print(f"\n모든 이미지의 오차가 {high_error_threshold} pixels 이하입니다. (우수)")

    # 오차 분포
    bins = [0.0, 0.1, 0.2, 0.5, float('inf')]
    bin_labels = ["0.0~0.1px", "0.1~0.2px", "0.2~0.5px", "> 0.5px"]

    print(f"\n오차 분포:")
    for i in range(len(bins) - 1):
        count = sum(1 for e in mean_errors if bins[i] <= e < bins[i+1])
        percentage = (count / len(mean_errors)) * 100
        print(f"  {bin_labels[i]:12s}: {count:3d}개 ({percentage:5.1f}%)")

    # 이미지별 상세 오차 (전체)
    print(f"\n이미지별 상세 오차:")
    for idx, e in enumerate(errors):
        print(f"  [#{idx:3d}] 평균: {e['mean']:.4f}px, 최대: {e['max_error']:.4f}px, "
              f"편향: ({e['bias_x']:.4f}, {e['bias_y']:.4f})")

    print("=" * 60)


def main():
    """메인 실행 함수"""
    print("=" * 60)
    print("BoofCV 카메라 캘리브레이션")
    print("=" * 60)
    print(f"이미지 디렉토리: {IMAGE_DIR}")
    print(f"체스보드 크기: {CHESSBOARD_COLS}x{CHESSBOARD_ROWS} (내부 코너)")
    print(f"              = {CHESSBOARD_COLS+1}x{CHESSBOARD_ROWS+1} (전체 그리드)")
    print(f"정사각형 크기: {SQUARE_SIZE_MM}mm")
    print(f"출력 파일: {OUTPUT_FILE}")
    print("=" * 60)

    # 1. 이미지 로드
    print("\n1. 이미지 로드...")
    images = load_images(IMAGE_DIR)

    # 2. 체스보드 감지 (OpenCV)
    print("\n2. 체스보드 감지 (OpenCV)...")
    observations, image_shape, success_count = detect_chessboards_opencv(
        images, CHESSBOARD_COLS, CHESSBOARD_ROWS
    )

    if success_count == 0:
        print("\n오류: 체스보드가 감지된 이미지가 없습니다.")
        sys.exit(1)

    # 3. 캘리브레이션 실행 (BoofCV)
    print("\n3. 캘리브레이션 실행 (BoofCV Brown 모델)...")
    result = calibrate_with_boofcv(
        observations, image_shape,
        CHESSBOARD_COLS, CHESSBOARD_ROWS, SQUARE_SIZE_MM
    )

    if result is None:
        print("\n오류: 캘리브레이션 실패")
        sys.exit(1)

    # 4. 결과 저장
    print("\n4. 결과 저장...")
    save_calibration(OUTPUT_FILE, result)
    save_calibration_yaml(OUTPUT_YAML, result)

    # 5. 오차 분석
    print("\n5. 오차 분석...")
    print_error_analysis(result.errors)

    print("\n" + "=" * 60)
    print("캘리브레이션 완료!")
    print(f"GUI에서 로드: {OUTPUT_FILE}")
    print(f"시스템에서 사용: {OUTPUT_YAML}")
    print("=" * 60)


if __name__ == "__main__":
    main()
