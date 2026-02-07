#!/usr/bin/env python3
"""
Hand-Eye Calibration CLI

독립 실행 가능한 Hand-Eye 캘리브레이션 CLI 도구.

Usage:
    python scripts/hand_eye_cli.py calibrate --data-dir calibration/hand_eye_xxx/
    python scripts/hand_eye_cli.py compare --data-dir calibration/hand_eye_xxx/
"""

import argparse
import sys
from pathlib import Path

# 프로젝트 루트를 Python path에 추가
script_dir = Path(__file__).parent
sys.path.insert(0, str(script_dir))

from services.hand_eye_calibration import HandEyeCalibrator, Algorithm
from services.hand_eye_calibration.metrics import compare_algorithms, compare_algorithms_with_error, print_result, compute_reprojection_error
from services.hand_eye_calibration.data_loader import HandEyeDataLoader


def cmd_calibrate(args):
    """단일 알고리즘으로 캘리브레이션 (outlier 자동 제거)"""
    print(f"\n[Hand-Eye Calibration]")
    print(f"Data directory: {args.data_dir}")
    print(f"Algorithm: {args.algorithm}")
    print(f"Outlier removal: {'OFF' if args.no_outlier_removal else 'ON'}")

    # 데이터 통계
    loader = HandEyeDataLoader(args.data_dir)
    stats = loader.get_stats()
    print(f"Total samples: {stats['total_samples']}")
    print(f"Valid samples: {stats['valid_samples']}")

    # 캘리브레이션
    algorithm = Algorithm.from_string(args.algorithm)
    calibrator = HandEyeCalibrator(algorithm=algorithm)
    num_samples = calibrator.load_data(args.data_dir)
    print(f"Loaded {num_samples} pose pairs")

    # Outlier 제거 여부에 따라 다른 메서드 호출
    if args.no_outlier_removal:
        result = calibrator.calibrate()
    else:
        result = calibrator.calibrate_with_outlier_removal(
            algorithm=algorithm,
            sigma_threshold=args.sigma,
            max_iterations=args.max_iter
        )

    print_result(result)

    # 결과 저장
    if args.output:
        calibrator.save_result(args.output)
        print(f"Result saved to: {args.output}")


def cmd_compare(args):
    """모든 알고리즘 비교"""
    print(f"\n[Hand-Eye Calibration - Algorithm Comparison]")
    print(f"Data directory: {args.data_dir}")

    # 데이터 통계
    loader = HandEyeDataLoader(args.data_dir)
    stats = loader.get_stats()
    print(f"Total samples: {stats['total_samples']}")
    print(f"Valid samples: {stats['valid_samples']}\n")

    # 모든 알고리즘으로 캘리브레이션
    calibrator = HandEyeCalibrator()
    num_samples = calibrator.load_data(args.data_dir)
    print(f"Loaded {num_samples} pose pairs\n")

    results = calibrator.calibrate_all_methods()

    # Reprojection Error 포함 비교 테이블 출력
    table, best_algo = compare_algorithms_with_error(
        results, calibrator.robot_poses, calibrator.camera_poses
    )
    print(table)


def main():
    parser = argparse.ArgumentParser(
        description='Hand-Eye Calibration CLI Tool',
        formatter_class=argparse.RawDescriptionHelpFormatter
    )
    subparsers = parser.add_subparsers(dest='command', help='Commands')

    # calibrate 명령
    p_calib = subparsers.add_parser('calibrate', help='Run calibration with single algorithm')
    p_calib.add_argument('--data-dir', '-d', required=True, help='Calibration data directory')
    p_calib.add_argument('--algorithm', '-a', default='PARK',
                         choices=['TSAI', 'PARK', 'HORAUD', 'ANDREFF', 'DANIILIDIS'],
                         help='Calibration algorithm (default: PARK)')
    p_calib.add_argument('--output', '-o', help='Output file path (.npz)')
    p_calib.add_argument('--no-outlier-removal', action='store_true',
                         help='Disable iterative outlier removal')
    p_calib.add_argument('--sigma', type=float, default=2.0,
                         help='Outlier threshold in sigma (default: 2.0)')
    p_calib.add_argument('--max-iter', type=int, default=5,
                         help='Max iterations for outlier removal (default: 5)')
    p_calib.set_defaults(func=cmd_calibrate)

    # compare 명령
    p_compare = subparsers.add_parser('compare', help='Compare all algorithms')
    p_compare.add_argument('--data-dir', '-d', required=True, help='Calibration data directory')
    p_compare.set_defaults(func=cmd_compare)

    args = parser.parse_args()

    if args.command is None:
        parser.print_help()
        return

    args.func(args)


if __name__ == '__main__':
    main()
