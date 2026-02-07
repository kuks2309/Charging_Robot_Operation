"""
Evaluation Metrics for Hand-Eye Calibration

캘리브레이션 결과 평가를 위한 메트릭스.
Reprojection error를 통해 캘리브레이션 정확도 검증.
"""

from typing import Dict, List, Tuple
import numpy as np

from .algorithms import Algorithm
from .models import RobotPose, CameraPose, CalibrationResult, EvaluationMetrics
from .transforms import create_homogeneous_matrix


def compute_reprojection_error(
    result: CalibrationResult,
    robot_poses: List[RobotPose],
    camera_poses: List[CameraPose]
) -> EvaluationMetrics:
    """
    Reprojection Error 계산

    Hand-Eye 캘리브레이션 검증:
    - 각 포즈에서 타겟(체스보드)을 로봇 베이스 좌표로 변환
    - 모든 포즈에서 타겟 위치가 일관되어야 함 (고정 타겟이므로)
    - 위치 분산이 작을수록 캘리브레이션 품질이 좋음

    공식: T_target_in_base = T_gripper2base @ T_cam2gripper @ T_target2cam

    Args:
        result: 캘리브레이션 결과
        robot_poses: 로봇 TCP 포즈 리스트
        camera_poses: 카메라 포즈 리스트

    Returns:
        EvaluationMetrics
    """
    H_cam2gripper = result.to_homogeneous_matrix()

    target_positions_in_base = []

    for robot_pose, camera_pose in zip(robot_poses, camera_poses):
        # T_gripper2base (로봇 TCP → 베이스)
        H_gripper2base = robot_pose.to_homogeneous_matrix()

        # T_target2cam (타겟 → 카메라)
        H_target2cam = camera_pose.to_homogeneous_matrix()

        # T_target_in_base = T_gripper2base @ T_cam2gripper @ T_target2cam
        H_target2base = H_gripper2base @ H_cam2gripper @ H_target2cam

        # 타겟 원점의 베이스 좌표
        target_pos = H_target2base[:3, 3]
        target_positions_in_base.append(target_pos)

    positions = np.array(target_positions_in_base)

    # 이상적으로 모든 위치가 동일해야 함 (타겟은 고정)
    mean_pos = positions.mean(axis=0)
    errors = np.linalg.norm(positions - mean_pos, axis=1)

    return EvaluationMetrics(
        reprojection_error_mean=float(errors.mean()),
        reprojection_error_std=float(errors.std()),
        reprojection_error_max=float(errors.max())
    )


def compare_algorithms_with_error(
    results: Dict[Algorithm, CalibrationResult],
    robot_poses: List[RobotPose],
    camera_poses: List[CameraPose]
) -> Tuple[str, Algorithm]:
    """
    알고리즘 비교 (Reprojection Error 포함)

    Args:
        results: {Algorithm: CalibrationResult}
        robot_poses: 로봇 포즈 리스트
        camera_poses: 카메라 포즈 리스트

    Returns:
        (비교 테이블 문자열, 최적 알고리즘)
    """
    lines = []
    lines.append("=" * 85)
    lines.append("Hand-Eye Calibration Algorithm Comparison (with Reprojection Error)")
    lines.append("=" * 85)
    lines.append(f"{'Algorithm':<12} {'X (mm)':>10} {'Y (mm)':>10} {'Z (mm)':>10} {'Error (mm)':>12} {'Std':>8}")
    lines.append("-" * 85)

    best_algo = None
    best_error = float('inf')

    for algo, result in results.items():
        t = result.t_cam2gripper.flatten()
        metrics = compute_reprojection_error(result, robot_poses, camera_poses)

        lines.append(
            f"{algo.name:<12} {t[0]:>10.2f} {t[1]:>10.2f} {t[2]:>10.2f} "
            f"{metrics.reprojection_error_mean:>12.3f} {metrics.reprojection_error_std:>8.3f}"
        )

        if metrics.reprojection_error_mean < best_error:
            best_error = metrics.reprojection_error_mean
            best_algo = algo

    lines.append("=" * 85)
    lines.append(f"Best Algorithm: {best_algo.name} (Error: {best_error:.3f} mm)")
    lines.append("=" * 85)

    return "\n".join(lines), best_algo


def compare_algorithms(results: Dict[Algorithm, CalibrationResult]) -> str:
    """알고리즘 비교 테이블 (기본 버전)"""
    lines = []
    lines.append("=" * 70)
    lines.append("Hand-Eye Calibration Algorithm Comparison")
    lines.append("=" * 70)
    lines.append(f"{'Algorithm':<12} {'X (mm)':>10} {'Y (mm)':>10} {'Z (mm)':>10} {'Samples':>8}")
    lines.append("-" * 70)

    for algo, result in results.items():
        t = result.t_cam2gripper.flatten()
        lines.append(f"{algo.name:<12} {t[0]:>10.2f} {t[1]:>10.2f} {t[2]:>10.2f} {result.num_samples:>8}")

    lines.append("=" * 70)

    if len(results) > 1:
        translations = np.array([r.t_cam2gripper.flatten() for r in results.values()])
        mean_t = translations.mean(axis=0)
        std_t = translations.std(axis=0)
        lines.append(f"{'Mean':<12} {mean_t[0]:>10.2f} {mean_t[1]:>10.2f} {mean_t[2]:>10.2f}")
        lines.append(f"{'Std':<12} {std_t[0]:>10.2f} {std_t[1]:>10.2f} {std_t[2]:>10.2f}")
        lines.append("=" * 70)

    return "\n".join(lines)


def print_result(result: CalibrationResult):
    """단일 결과 출력"""
    t = result.t_cam2gripper.flatten()
    print(f"\n{'=' * 50}")
    print(f"Hand-Eye Calibration Result ({result.algorithm.name})")
    print(f"{'=' * 50}")
    print(f"Samples used: {result.num_samples}")
    print(f"Translation (mm):")
    print(f"  X: {t[0]:.4f}")
    print(f"  Y: {t[1]:.4f}")
    print(f"  Z: {t[2]:.4f}")
    print(f"\n4x4 Transformation Matrix:")
    print(result.to_homogeneous_matrix())
    print(f"{'=' * 50}\n")
