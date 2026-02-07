"""
Hand-Eye Calibration Data Loader

CSV 파일로부터 캘리브레이션 데이터 로드.
"""

from pathlib import Path
from typing import List, Tuple
import pandas as pd
import numpy as np

from .models import RobotPose, CameraPose


class HandEyeDataLoader:
    """
    Hand-Eye 캘리브레이션 데이터 로더

    CSV 형식:
        timestamp, index, image_filename,
        tcp_x, tcp_y, tcp_z, tcp_rx, tcp_ry, tcp_rz,
        chessboard_detected,
        rvec_x, rvec_y, rvec_z, tvec_x, tvec_y, tvec_z
    """

    def __init__(self, data_dir: str):
        """
        Args:
            data_dir: 캘리브레이션 데이터 디렉토리 경로
        """
        self.data_dir = Path(data_dir)
        self.csv_path = self.data_dir / 'hand_eye_data.csv'

    def load(self) -> Tuple[List[RobotPose], List[CameraPose]]:
        """
        CSV 데이터 로드 및 필터링

        Returns:
            (robot_poses, camera_poses): 유효한 포즈 쌍 리스트
        """
        if not self.csv_path.exists():
            raise FileNotFoundError(f"CSV file not found: {self.csv_path}")

        df = pd.read_csv(self.csv_path)
        df_valid = self.filter_valid_samples(df)

        robot_poses = []
        camera_poses = []

        for _, row in df_valid.iterrows():
            # Robot TCP Pose
            robot_pose = RobotPose(
                x=float(row['tcp_x']),
                y=float(row['tcp_y']),
                z=float(row['tcp_z']),
                rx=float(row['tcp_rx']),
                ry=float(row['tcp_ry']),
                rz=float(row['tcp_rz'])
            )
            robot_poses.append(robot_pose)

            # Camera Pose (rvec, tvec)
            rvec = np.array([row['rvec_x'], row['rvec_y'], row['rvec_z']])
            tvec = np.array([row['tvec_x'], row['tvec_y'], row['tvec_z']])
            camera_pose = CameraPose(rvec=rvec, tvec=tvec)
            camera_poses.append(camera_pose)

        return robot_poses, camera_poses

    def filter_valid_samples(self, df: pd.DataFrame) -> pd.DataFrame:
        """
        유효한 샘플만 필터링

        조건:
        - chessboard_detected == True
        - rvec/tvec가 0이 아님

        Args:
            df: 원본 DataFrame

        Returns:
            필터링된 DataFrame
        """
        # chessboard_detected가 True인 샘플만
        if 'chessboard_detected' in df.columns:
            df_valid = df[df['chessboard_detected'] == True].copy()
        else:
            df_valid = df.copy()

        # rvec/tvec가 모두 0인 샘플 제외
        zero_mask = (
            (df_valid['rvec_x'] == 0) &
            (df_valid['rvec_y'] == 0) &
            (df_valid['rvec_z'] == 0) &
            (df_valid['tvec_x'] == 0) &
            (df_valid['tvec_y'] == 0) &
            (df_valid['tvec_z'] == 0)
        )
        df_valid = df_valid[~zero_mask]

        return df_valid

    def get_stats(self) -> dict:
        """데이터 통계 반환"""
        df = pd.read_csv(self.csv_path)
        total = len(df)
        valid = len(self.filter_valid_samples(df))

        return {
            'total_samples': total,
            'valid_samples': valid,
            'invalid_samples': total - valid,
            'data_dir': str(self.data_dir)
        }
