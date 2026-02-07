#!/usr/bin/env python3
"""Detailed Error Analysis after Outlier Removal"""

import sys
from pathlib import Path
import numpy as np

script_dir = Path(__file__).parent
sys.path.insert(0, str(script_dir))

from services.hand_eye_calibration import HandEyeCalibrator, Algorithm

DATA_DIR = "calibration/hand_eye_20260131_100241/"

print("=" * 70)
print("Detailed Error Analysis (After Outlier Removal)")
print("=" * 70)

# 1단계: 전체 데이터로 캘리브레이션 및 outlier 탐지
calibrator = HandEyeCalibrator(algorithm=Algorithm.PARK)
num_samples = calibrator.load_data(DATA_DIR)
result = calibrator.calibrate(Algorithm.PARK)
H_cam2gripper = result.to_homogeneous_matrix()

# 각 포즈별 타겟 위치 계산
target_positions = []
for robot_pose, camera_pose in zip(calibrator.robot_poses, calibrator.camera_poses):
    H_gripper2base = robot_pose.to_homogeneous_matrix()
    H_target2cam = camera_pose.to_homogeneous_matrix()
    H_target2base = H_gripper2base @ H_cam2gripper @ H_target2cam
    target_positions.append(H_target2base[:3, 3])

positions = np.array(target_positions)
mean_pos = positions.mean(axis=0)
errors = np.linalg.norm(positions - mean_pos, axis=1)

# Outlier 제거 (2σ 기준)
threshold = errors.mean() + 2 * errors.std()
valid_indices = np.where(errors <= threshold)[0]

print(f"Original samples: {len(errors)}")
print(f"Outliers removed: {len(errors) - len(valid_indices)}")
print(f"Valid samples: {len(valid_indices)}\n")

# 2단계: Outlier 제거 후 재캘리브레이션
calibrator2 = HandEyeCalibrator(algorithm=Algorithm.PARK)
for idx in valid_indices:
    calibrator2.add_pose_pair(calibrator.robot_poses[idx], calibrator.camera_poses[idx])

result2 = calibrator2.calibrate(Algorithm.PARK)
H_cam2gripper2 = result2.to_homogeneous_matrix()
t = result2.t_cam2gripper.flatten()

print(f"Calibration Result (PARK, outlier removed):")
print(f"  X: {t[0]:.2f} mm")
print(f"  Y: {t[1]:.2f} mm")
print(f"  Z: {t[2]:.2f} mm\n")

# 3단계: 새 결과로 오차 재계산
target_positions2 = []
errors_x, errors_y, errors_z = [], [], []

for idx in valid_indices:
    robot_pose = calibrator.robot_poses[idx]
    camera_pose = calibrator.camera_poses[idx]
    H_gripper2base = robot_pose.to_homogeneous_matrix()
    H_target2cam = camera_pose.to_homogeneous_matrix()
    H_target2base = H_gripper2base @ H_cam2gripper2 @ H_target2cam
    target_positions2.append(H_target2base[:3, 3])

positions2 = np.array(target_positions2)
mean_pos2 = positions2.mean(axis=0)

# 축별 오차
errors_x = positions2[:, 0] - mean_pos2[0]
errors_y = positions2[:, 1] - mean_pos2[1]
errors_z = positions2[:, 2] - mean_pos2[2]
errors_total = np.linalg.norm(positions2 - mean_pos2, axis=1)

print("=" * 70)
print("Error Statistics (After Outlier Removal)")
print("=" * 70)
print(f"Target Mean Position: X={mean_pos2[0]:.2f}, Y={mean_pos2[1]:.2f}, Z={mean_pos2[2]:.2f} mm\n")

print(f"{'Axis':<8} {'Mean':>10} {'Std':>10} {'Min':>10} {'Max':>10} {'Range':>10}")
print("-" * 60)
print(f"{'X':<8} {np.mean(errors_x):>10.3f} {np.std(errors_x):>10.3f} {np.min(errors_x):>10.3f} {np.max(errors_x):>10.3f} {np.ptp(errors_x):>10.3f}")
print(f"{'Y':<8} {np.mean(errors_y):>10.3f} {np.std(errors_y):>10.3f} {np.min(errors_y):>10.3f} {np.max(errors_y):>10.3f} {np.ptp(errors_y):>10.3f}")
print(f"{'Z':<8} {np.mean(errors_z):>10.3f} {np.std(errors_z):>10.3f} {np.min(errors_z):>10.3f} {np.max(errors_z):>10.3f} {np.ptp(errors_z):>10.3f}")
print(f"{'Total':<8} {np.mean(errors_total):>10.3f} {np.std(errors_total):>10.3f} {np.min(errors_total):>10.3f} {np.max(errors_total):>10.3f} {np.ptp(errors_total):>10.3f}")

print("\n" + "=" * 70)
print("Error Distribution (Total Error)")
print("=" * 70)

# 오차 분포 (히스토그램 텍스트)
bins = [0, 5, 10, 15, 20, 30, 50, 100]
for i in range(len(bins)-1):
    count = np.sum((errors_total >= bins[i]) & (errors_total < bins[i+1]))
    pct = count / len(errors_total) * 100
    bar = '█' * int(pct / 2)
    print(f"{bins[i]:>3}-{bins[i+1]:<3} mm: {count:>4} ({pct:>5.1f}%) {bar}")

over_50 = np.sum(errors_total >= 50)
print(f"> 50  mm: {over_50:>4} ({over_50/len(errors_total)*100:>5.1f}%)")

print("\n" + "=" * 70)
print("Percentile Analysis")
print("=" * 70)
for p in [50, 75, 90, 95, 99]:
    val = np.percentile(errors_total, p)
    print(f"  {p}th percentile: {val:.2f} mm")

print("\n" + "=" * 70)
print("Practical Interpretation")
print("=" * 70)
p90 = np.percentile(errors_total, 90)
print(f"90% of measurements have error < {p90:.1f} mm")
print(f"Mean error: {np.mean(errors_total):.1f} mm")

if np.mean(errors_total) < 5:
    quality = "EXCELLENT - 정밀 작업 가능"
elif np.mean(errors_total) < 10:
    quality = "GOOD - 일반 피킹 작업 가능"
elif np.mean(errors_total) < 20:
    quality = "FAIR - 대략적 위치 파악 가능"
else:
    quality = "POOR - 추가 캘리브레이션 필요"

print(f"Quality: {quality}")
print("=" * 70)
