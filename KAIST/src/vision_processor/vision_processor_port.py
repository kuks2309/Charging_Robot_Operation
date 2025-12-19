#!/usr/bin/env python3
# vision_processing.py
# RealSense vision processing with Modbus robot interface

import pyrealsense2 as rs
import numpy as np
import cv2
from ultralytics import YOLO
from scipy.spatial.transform import Rotation as R
import open3d as o3d
import json

from modbus_robot_interface import ModbusRobotInterface
from kalman_filter.kalman_filter import StaticObjectPoseKalmanFilter
from keypoint_detector.aruco import ArucoCameraPoseEstimator
from keypoint_detector.pnp import PnPKeypointAligner
from utils import draw_projected_arrow_cv, Points
from vision_processor.vision_processor_abstract import VisionProcessorAbstract

class VisionProcessorPort(VisionProcessorAbstract):
    """Handles RealSense camera and vision processing."""
    
    def __init__(self, intrinsics):

        marker_len = 0.0182  # marker size in meters
        objp = np.array([
            [-marker_len/2,  marker_len/2, 0],
            [ marker_len/2,  marker_len/2, 0],
            [ marker_len/2, -marker_len/2, 0],
            [-marker_len/2, -marker_len/2, 0]
        ], dtype=np.float32)

        # Initialize Kalman filter
        self.kalman_filter = None
        
        self.pnp_aligner = PnPKeypointAligner(objp,
                                        np.array([[intrinsics.fx, 0, intrinsics.ppx],
                                                [0, intrinsics.fy, intrinsics.ppy],
                                                [0, 0, 1]], dtype=np.float32),
                                        np.array(intrinsics.coeffs[:5], dtype=np.float32))

    def draw_pose_axes(self, image, R_mat, t_vec, K, dist, axis_length=0.05):
        """
        Draws 6D pose axes (X-red, Y-green, Z-blue) on the image.
        """
        # 3D 좌표축 정의 (단위: m)
        axis_points_3d = np.float32([
            [0, 0, 0],                      # origin
            [axis_length, 0, 0],            # X axis
            [0, axis_length, 0],            # Y axis
            [0, 0, axis_length]             # Z axis
        ])

        # ✅ 여기 수정됨: R_mat을 rvec으로 변환 후 사용
        rvec, _ = cv2.Rodrigues(R_mat)
        imgpts, _ = cv2.projectPoints(axis_points_3d, rvec, t_vec, K, dist)
        imgpts = np.int32(imgpts).reshape(-1, 2)

        # 원점 (0,0,0) 위치
        origin = tuple(imgpts[0].ravel())

        # 각 축을 그림
        cv2.line(image, origin, tuple(imgpts[1].ravel()), (0, 0, 255), 3)  # X (빨강)
        cv2.line(image, origin, tuple(imgpts[2].ravel()), (0, 255, 0), 3)  # Y (초록)
        cv2.line(image, origin, tuple(imgpts[3].ravel()), (255, 0, 0), 3)  # Z (파랑)

        # 원점 점 찍기
        cv2.circle(image, origin, 5, (255, 255, 255), -1)

        return image

    def process_frame(self, color_image, intrinsics, marker_poses):
        """Process a single frame for object detection and pose estimation."""
        # if marker_poses is None:
        #     return None

        # pose_info: dictionary (camera coordinate)
        current_pose = None
        aligned_keypoints_2d, pose_info = self.pnp_aligner.align_keypoints_pnp_port(color_image, marker_poses)
        
        if pose_info is not None and "world_pose" in pose_info and pose_info["confidence_score"] > 0.95:
            if self.kalman_filter is None:
                self.kalman_filter = StaticObjectPoseKalmanFilter(
                process_noise_std=1e-6,
                position_noise_std=1e-3,
                rotation_noise_std=1e-3,
                initial_pose=pose_info["world_pose"],
                )
            
            # current_pose: dictionary (world coordinate)
            # current_pose = self.kalman_filter.get_pose(marker_poses)

            # Update Kalman filter
        
            self.kalman_filter.update(
                pose_info['world_pose'], 
                measurement_quality=pose_info["confidence_score"]
                
            )
        
        if self.kalman_filter is not None:
            self.kalman_filter.predict()
            
            current_pose = self.kalman_filter.get_pose(marker_poses)

        # just for visualization, we can ignore
        if current_pose is not None:
            R_cam_obj = current_pose["rotation_object_to_camera"]
            t_cam_obj = current_pose["position_object_to_camera"].reshape(3, 1)
            K = np.array([[intrinsics.fx, 0, intrinsics.ppx],
                        [0, intrinsics.fy, intrinsics.ppy],
                        [0, 0, 1]], dtype=np.float32)
            dist = np.array(intrinsics.coeffs[:5], dtype=np.float32)

            org = (50, 100)   # bottom-left corner of the text (x, y)

            # Font, scale, color, thickness
            font = cv2.FONT_HERSHEY_SIMPLEX
            font_scale = 1
            color = (0, 255, 0)   # BGR (green)
            thickness = 2

            color_image = self.draw_pose_axes(color_image, R_cam_obj, t_cam_obj, K, dist)
            cv2.putText(color_image, str(current_pose["convergence_score"]), org, font, font_scale, color, thickness, cv2.LINE_AA)   
            
            cv2.imshow("Pose Visualization", color_image)
            cv2.waitKey(1)
                   
        return current_pose

