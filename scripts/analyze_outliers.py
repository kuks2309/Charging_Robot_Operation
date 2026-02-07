#!/usr/bin/env python3
"""Outlier Analysis for Hand-Eye Calibration"""

import sys
from pathlib import Path
import numpy as np

script_dir = Path(__file__).parent
sys.path.insert(0, str(script_dir))

from services.hand_eye_calibration import HandEyeCalibrator, Algorithm
from services.hand_eye_calibration.transforms import create_homogeneous_matrix

DATA_DIR = "calibration/hand_eye_20260131_100241/"

# TF1 값 (로봇 설정에서 확인)
TF1_OFFSET = {
    'X': 33.2,
    'Y': 60.0,  # 이미지에서 부분적으로 보임
    'Z': 35.94,
}

print("=" * 70)
print("Hand-Eye Calibration Outlier Analysis")
print("=" * 70)

# 데이터 로드
calibrator = HandEyeCalibrator(algorithm=Algorithm.PARK)
num_samples = calibrator.load_data(DATA_DIR)
print(f"Loaded {num_samples} pose pairs\n")

# PARK 결과로 캘리브레이션
result = calibrator.calibrate(Algorithm.PARK)
H_cam2gripper = result.to_homogeneous_matrix()

print(f"TF1 (로봇 설정): X={TF1_OFFSET['X']:.2f}, Y={TF1_OFFSET['Y']:.2f}, Z={TF1_OFFSET['Z']:.2f} mm")
t = result.t_cam2gripper.flatten()
print(f"PARK 결과:      X={t[0]:.2f}, Y={t[1]:.2f}, Z={t[2]:.2f} mm")
print(f"차이:           X={t[0]-TF1_OFFSET['X']:.2f}, Y={t[1]-TF1_OFFSET['Y']:.2f}, Z={t[2]-TF1_OFFSET['Z']:.2f} mm\n")

# 각 포즈별 타겟 위치 계산
target_positions = []
for i, (robot_pose, camera_pose) in enumerate(zip(calibrator.robot_poses, calibrator.camera_poses)):
    H_gripper2base = robot_pose.to_homogeneous_matrix()
    H_target2cam = camera_pose.to_homogeneous_matrix()
    H_target2base = H_gripper2base @ H_cam2gripper @ H_target2cam
    target_pos = H_target2base[:3, 3]
    target_positions.append(target_pos)

positions = np.array(target_positions)
mean_pos = positions.mean(axis=0)
errors = np.linalg.norm(positions - mean_pos, axis=1)

# 통계
print("=" * 70)
print("Per-Sample Error Analysis")
print("=" * 70)
print(f"Mean target position: X={mean_pos[0]:.2f}, Y={mean_pos[1]:.2f}, Z={mean_pos[2]:.2f}")
print(f"Error Mean: {errors.mean():.3f} mm, Std: {errors.std():.3f} mm")
print(f"Error Min: {errors.min():.3f} mm, Max: {errors.max():.3f} mm\n")

# Outlier 탐지 (2σ 기준)
threshold = errors.mean() + 2 * errors.std()
outliers = np.where(errors > threshold)[0]

print(f"Outlier threshold (mean + 2σ): {threshold:.3f} mm")
print(f"Outliers found: {len(outliers)} / {len(errors)}\n")

if len(outliers) > 0:
    print("Outlier samples:")
    print(f"{'Index':<8} {'Error (mm)':>12} {'X':>10} {'Y':>10} {'Z':>10}")
    print("-" * 50)
    for idx in outliers[:20]:  # 상위 20개만 표시
        print(f"{idx:<8} {errors[idx]:>12.2f} {positions[idx][0]:>10.2f} {positions[idx][1]:>10.2f} {positions[idx][2]:>10.2f}")

# Outlier 제거 후 재계산
print("\n" + "=" * 70)
print("Recalibration without Outliers")
print("=" * 70)

valid_indices = np.where(errors <= threshold)[0]
print(f"Valid samples: {len(valid_indices)} / {len(errors)}")

# 새 캘리브레이터로 유효한 데이터만 사용
calibrator2 = HandEyeCalibrator(algorithm=Algorithm.PARK)
for idx in valid_indices:
    calibrator2.add_pose_pair(calibrator.robot_poses[idx], calibrator.camera_poses[idx])

result2 = calibrator2.calibrate(Algorithm.PARK)
t2 = result2.t_cam2gripper.flatten()

print(f"\nPARK (outlier 제거 후): X={t2[0]:.2f}, Y={t2[1]:.2f}, Z={t2[2]:.2f} mm")
print(f"TF1 (로봇 설정):        X={TF1_OFFSET['X']:.2f}, Y={TF1_OFFSET['Y']:.2f}, Z={TF1_OFFSET['Z']:.2f} mm")
print(f"차이:                   X={t2[0]-TF1_OFFSET['X']:.2f}, Y={t2[1]-TF1_OFFSET['Y']:.2f}, Z={t2[2]-TF1_OFFSET['Z']:.2f} mm")

# 새 오차 계산
H_cam2gripper2 = result2.to_homogeneous_matrix()
target_positions2 = []
for idx in valid_indices:
    robot_pose = calibrator.robot_poses[idx]
    camera_pose = calibrator.camera_poses[idx]
    H_gripper2base = robot_pose.to_homogeneous_matrix()
    H_target2cam = camera_pose.to_homogeneous_matrix()
    H_target2base = H_gripper2base @ H_cam2gripper2 @ H_target2cam
    target_positions2.append(H_target2base[:3, 3])

positions2 = np.array(target_positions2)
mean_pos2 = positions2.mean(axis=0)
errors2 = np.linalg.norm(positions2 - mean_pos2, axis=1)

print(f"\nNew Error: Mean={errors2.mean():.3f} mm, Std={errors2.std():.3f} mm")
print(f"Improvement: {errors.mean() - errors2.mean():.3f} mm ({(1 - errors2.mean()/errors.mean())*100:.1f}%)")
print("=" * 70)
