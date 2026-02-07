"""
Hand-Eye Calibrator

메인 캘리브레이션 엔진. Eye-in-Hand 구성을 위한 Hand-Eye 캘리브레이션 수행.
"""

from typing import Dict, List, Optional
import numpy as np
import cv2

from .algorithms import Algorithm
from .models import RobotPose, CameraPose, CalibrationResult, EvaluationMetrics
from .data_loader import HandEyeDataLoader


class HandEyeCalibrator:
    """
    Hand-Eye 캘리브레이션 메인 클래스

    Eye-in-Hand 구성: 카메라가 로봇 엔드이펙터에 장착
    결과: Camera → Gripper 변환 행렬 (R_cam2gripper, t_cam2gripper)
    """

    def __init__(self, algorithm: Algorithm = Algorithm.TSAI):
        """
        Args:
            algorithm: 사용할 캘리브레이션 알고리즘 (기본: TSAI)
        """
        self.algorithm = algorithm
        self.robot_poses: List[RobotPose] = []
        self.camera_poses: List[CameraPose] = []
        self.result: Optional[CalibrationResult] = None

    def load_data(self, data_dir: str) -> int:
        """
        CSV + 이미지 데이터 로드

        Args:
            data_dir: 캘리브레이션 데이터 디렉토리

        Returns:
            유효 샘플 수
        """
        loader = HandEyeDataLoader(data_dir)
        self.robot_poses, self.camera_poses = loader.load()
        return len(self.robot_poses)

    def add_pose_pair(self, robot_pose: RobotPose, camera_pose: CameraPose):
        """포즈 쌍 추가"""
        self.robot_poses.append(robot_pose)
        self.camera_poses.append(camera_pose)

    def clear_poses(self):
        """포즈 데이터 초기화"""
        self.robot_poses.clear()
        self.camera_poses.clear()
        self.result = None

    def calibrate(self, algorithm: Optional[Algorithm] = None) -> CalibrationResult:
        """
        Hand-Eye 캘리브레이션 실행

        Args:
            algorithm: 사용할 알고리즘 (None이면 초기 설정 사용)

        Returns:
            CalibrationResult
        """
        if len(self.robot_poses) < 3:
            raise ValueError(f"최소 3개 포즈 쌍 필요. 현재: {len(self.robot_poses)}")

        if algorithm is None:
            algorithm = self.algorithm

        # Gripper → Base 변환 (로봇 TCP 포즈)
        R_gripper2base_list = []
        t_gripper2base_list = []

        for pose in self.robot_poses:
            R = pose.to_rotation_matrix()
            t = pose.to_translation_vector()
            R_gripper2base_list.append(R)
            t_gripper2base_list.append(t)

        # Target → Camera 변환 (카메라에서 본 체스보드)
        R_target2cam_list = []
        t_target2cam_list = []

        for pose in self.camera_poses:
            R = pose.to_rotation_matrix()
            t = pose.to_translation_vector()
            R_target2cam_list.append(R)
            t_target2cam_list.append(t)

        # OpenCV calibrateHandEye 호출
        R_cam2gripper, t_cam2gripper = cv2.calibrateHandEye(
            R_gripper2base_list, t_gripper2base_list,
            R_target2cam_list, t_target2cam_list,
            method=algorithm.value
        )

        self.result = CalibrationResult(
            R_cam2gripper=R_cam2gripper,
            t_cam2gripper=t_cam2gripper,
            algorithm=algorithm,
            num_samples=len(self.robot_poses)
        )

        return self.result

    def calibrate_all_methods(self) -> Dict[Algorithm, CalibrationResult]:
        """
        모든 알고리즘으로 캘리브레이션 실행

        Returns:
            {Algorithm: CalibrationResult} 딕셔너리
        """
        results = {}

        for algorithm in Algorithm.all():
            try:
                result = self.calibrate(algorithm)
                results[algorithm] = result
            except Exception as e:
                print(f"[{algorithm.name}] 실패: {e}")

        return results

    def save_result(self, filepath: str, format: str = 'npz'):
        """
        결과 저장

        Args:
            filepath: 저장 경로
            format: 'npz' 또는 'yaml'
        """
        if self.result is None:
            raise ValueError("저장할 결과가 없습니다. 먼저 calibrate()를 실행하세요.")

        if format == 'npz':
            np.savez(
                filepath,
                hand_eye_matrix=self.result.to_homogeneous_matrix(),
                R_cam2gripper=self.result.R_cam2gripper,
                t_cam2gripper=self.result.t_cam2gripper,
                algorithm=self.result.algorithm.name,
                num_samples=self.result.num_samples
            )
        elif format == 'yaml':
            import yaml
            data = {
                'R_cam2gripper': self.result.R_cam2gripper.tolist(),
                't_cam2gripper': self.result.t_cam2gripper.flatten().tolist(),
                'algorithm': self.result.algorithm.name,
                'num_samples': self.result.num_samples
            }
            with open(filepath, 'w') as f:
                yaml.dump(data, f, default_flow_style=False)
        else:
            raise ValueError(f"지원하지 않는 형식: {format}")

    @classmethod
    def load_result(cls, filepath: str) -> CalibrationResult:
        """
        저장된 결과 로드

        Args:
            filepath: npz 파일 경로

        Returns:
            CalibrationResult
        """
        data = np.load(filepath, allow_pickle=True)

        algorithm_name = str(data['algorithm'])
        algorithm = Algorithm.from_string(algorithm_name)

        return CalibrationResult(
            R_cam2gripper=data['R_cam2gripper'],
            t_cam2gripper=data['t_cam2gripper'],
            algorithm=algorithm,
            num_samples=int(data['num_samples'])
        )

    def get_pose_count(self) -> int:
        """현재 로드된 포즈 쌍 수"""
        return len(self.robot_poses)

    def calibrate_with_outlier_removal(
        self,
        algorithm: Optional[Algorithm] = None,
        sigma_threshold: float = 2.0,
        max_iterations: int = 5
    ) -> CalibrationResult:
        """
        Outlier 제거를 포함한 반복적 캘리브레이션

        1. 전체 데이터로 캘리브레이션
        2. Reprojection error 계산
        3. threshold (mean + sigma*std) 초과 샘플 제거
        4. 수렴할 때까지 반복

        Args:
            algorithm: 사용할 알고리즘 (None이면 초기 설정 사용)
            sigma_threshold: outlier 판별 기준 (default: 2σ)
            max_iterations: 최대 반복 횟수

        Returns:
            CalibrationResult (outlier 제거 후)
        """
        if algorithm is None:
            algorithm = self.algorithm

        # 작업용 복사본
        robot_poses = list(self.robot_poses)
        camera_poses = list(self.camera_poses)

        print(f"\n[Iterative Outlier Removal]")
        print(f"Initial samples: {len(robot_poses)}")
        print(f"Threshold: mean + {sigma_threshold}σ")

        for iteration in range(max_iterations):
            # 현재 데이터로 캘리브레이션
            temp_calibrator = HandEyeCalibrator(algorithm=algorithm)
            for rp, cp in zip(robot_poses, camera_poses):
                temp_calibrator.add_pose_pair(rp, cp)

            result = temp_calibrator.calibrate(algorithm)
            H_cam2gripper = result.to_homogeneous_matrix()

            # Reprojection error 계산
            target_positions = []
            for rp, cp in zip(robot_poses, camera_poses):
                H_gripper2base = rp.to_homogeneous_matrix()
                H_target2cam = cp.to_homogeneous_matrix()
                H_target2base = H_gripper2base @ H_cam2gripper @ H_target2cam
                target_positions.append(H_target2base[:3, 3])

            positions = np.array(target_positions)
            mean_pos = positions.mean(axis=0)
            errors = np.linalg.norm(positions - mean_pos, axis=1)

            threshold = errors.mean() + sigma_threshold * errors.std()
            outlier_mask = errors > threshold
            num_outliers = np.sum(outlier_mask)

            print(f"  Iter {iteration+1}: {len(robot_poses)} samples, "
                  f"error={errors.mean():.2f}±{errors.std():.2f} mm, "
                  f"outliers={num_outliers}")

            # 수렴 확인 (outlier 없음)
            if num_outliers == 0:
                print(f"  Converged at iteration {iteration+1}")
                break

            # Outlier 제거
            valid_indices = np.where(~outlier_mask)[0]
            robot_poses = [robot_poses[i] for i in valid_indices]
            camera_poses = [camera_poses[i] for i in valid_indices]

        # 최종 결과
        print(f"Final samples: {len(robot_poses)}")
        print(f"Final error: {errors.mean():.2f}±{errors.std():.2f} mm\n")

        self.result = result
        return result
