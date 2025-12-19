#!/usr/bin/env python3
# vision_processing.py
# RealSense vision processing with Modbus robot interface

import pyrealsense2 as rs
import numpy as np
import cv2
from ultralytics import YOLO
import open3d as o3d
import json
import time

from kalman_filter.kalman_filter import StaticObjectPoseKalmanFilter
from keypoint_detector.aruco import ArucoCameraPoseEstimator
from keypoint_detector.pnp import PnPKeypointAligner
from keypoint_detector.feature_matcher import preprocess2, preprocess_with_yolo
from utils import Points
from vision_processor.vision_processor_abstract import VisionProcessorAbstract
from utils import draw_projected_arrow_cv
# from keypoint_detector.pidinet import PiDiNet
# from keypoint_detector.config import config_model
import torch
from keypoint_detector.pidi import ContourFinder
from keypoint_detector.gamma_correction import process_bright, process_dimmed, image_agcwd


def process_img(img, visualize=False):
        """
        Process a single OpenCV image (BGR format)
        Args:
            img: OpenCV image (numpy array, BGR format)
        Returns:
            processed: binary edge map ready for circle detection
        """

        # === [1] RGB conversion and normalization ===
        # img = cv2.bilateralFilter(img, 5, 20, 20)
        # clahe = cv2.createCLAHE(clipLimit=5.0, tileGridSize=(4,4))
        # # L_enh = clahe.apply(L)
        # img = clahe.apply(img)
        def adjust_gamma_and_suppress_bgr(img):
            """
            Apply gamma correction and suppress overly bright (white) pixels.
            Input and output are both in BGR format.

            Args:
                image (np.ndarray): Input BGR image.
                gamma (float): Gamma correction factor (<1 brightens, >1 darkens).
                bright_thresh (int): Brightness threshold (0–255) above which pixels are set to black.

            Returns:
                np.ndarray: Processed image (BGR).
            """
            # --- Step 1: Gamma correction ---
                
            YCrCb = cv2.cvtColor(img, cv2.COLOR_BGR2YCrCb)
            Y = YCrCb[:,:,0]
            # Determine whether image is bright or dimmed
            threshold = -1
            exp_in = 25 # Expected global average intensity 
            M,N = img.shape[:2]
            mean_in = np.sum(Y/(M*N)) 
            t = (mean_in - exp_in)/ exp_in
            
            # Process image for gamma correction
            img_output = None
            if t < threshold: # Dimmed Image
                result = process_dimmed(Y)
                YCrCb[:,:,0] = result
                img_output = cv2.cvtColor(YCrCb,cv2.COLOR_YCrCb2BGR)
            elif t > threshold:
                result = process_bright(Y)
                YCrCb[:,:,0] = result
                img_output = cv2.cvtColor(YCrCb,cv2.COLOR_YCrCb2BGR)
            else:
                img_output = img
                
            return img_output



        img = adjust_gamma_and_suppress_bgr(img)    
        
        return img

class VisionProcessor(VisionProcessorAbstract):
    """Handles RealSense camera and vision processing."""
    
    def __init__(self, model_path: str, circle_model_path: str, template_path: str):

        # Load YOLO model
        self.model = YOLO(model_path)
        self.circle_model = YOLO(circle_model_path)
        # Load template points
        with open(template_path, "r") as f:
            template = json.load(f)
            self.template_np = np.array([
                (x["center_x"]/1000, x["center_y"]/1000, x["center_z"]/1000) for x in template
            ])
            self.template_np -= self.template_np.mean(axis=0)
            self.template_np[:, 1] = -self.template_np[:, 1]
            
            # self.template_np[:, 1] -= 0.0015 # += 0.001 # - -> up ; + -> down
            # self.template_np[:, 0] += 0.0015
            
            
        # template = template/1000
        # Initialize Kalman filter
        self.kalman_filter = StaticObjectPoseKalmanFilter(
                process_noise_std=1e-5,
                position_noise_std=1e-3,
                rotation_noise_std=1e-3,
        )
        self.descendants_info = None
        # sam_checkpoint = "mobile_sam.pt"
        # model_type = "vit_t"
        # sam = sam_model_registry[model_type](checkpoint=sam_checkpoint)
        self.contour_finder = ContourFinder() #SamPredictor(sam)

        # pdcs = config_model("carv4")
        # print("pdcs =", pdcs)
        # input("wait")

        # self.contour_finder = PiDiNet(inplane=60, pdcs=pdcs, dil=24, sa=True)
        # checkpoint = torch.load("./table7_pidinet.pth", map_location="cuda")
        # state_dict = {k.replace("module.", ""): v for k, v in checkpoint.items()}


        # self.contour_finder.load_state_dict(state_dict["state_dict"])
        
        # self.contour_finder.eval()

    def process_frame(self, color_image, intrinsics, marker_poses):
        """Process a single frame for object detection and pose estimation."""
        
        if marker_poses is None:
            return None
        
        # Run YOLO detection
        results = self.model(color_image) # cv2.rotate(color_image, cv2.ROTATE_90_CLOCKWISE))
        # cv2.imshow("pidinet", results[-1])
    
        H,W,D = color_image.shape
        if len(results[0].boxes) == 0:
            return None
        
        # Get first detection
        box = results[0].boxes[0]
        x_min, y_min, x_max, y_max = box.xyxy[0]
        x_center = int((x_min + x_max) / 2)
        y_center = int((y_min + y_max) / 2)
        b = Points(x_min=int(x_min), x_max=int(x_max), 
                   y_min=int(y_min), y_max=int(y_max))
        
        # Crop images
        cropped_color = color_image[b.y_min:b.y_max, b.x_min:b.x_max].copy()
        blank_image = np.zeros_like(cropped_color)
        
        # self.contour_finder.set_image(cropped_color)
        
        # mask, _, _ = self.contour_finder.predict()


        c_cropped_color = process_img(cropped_color.copy())
        cv2.imshow("color,", c_cropped_color)
        c_results = self.circle_model(c_cropped_color)
        
        boxes = c_results[0].boxes.xyxy.cpu().numpy()
                
        # edge_map = self.contour_finder.process_img(cropped_color)
            
        # Preprocess
        print("Starting preprocess yolo")
        self.descendants_info, correspondences = preprocess_with_yolo(cropped_color, color_image, self.contour_finder, 
                                                                      c_results[0], self.template_np, offset=(b.x_min, b.y_min),
                                                                      descendants_info=self.descendants_info, expansion_factor=5)
        if self.descendants_info is None:
            return None

        # Draw ellipses
        for centroid in self.descendants_info.get_ellipse_centroids():
            cv2.circle(color_image, centroid, 3, color=(255,0,0), thickness=3)
        # for ellipse in descendants_info.get_ellipses((x_min, y_min)):
        #     cv2.ellipse(color_image, ellipse, color=(255, 0, 0), thickness=2)

        print("Correspondence is", correspondences)
        if len(correspondences) == 0:
            return None

        print("after correspondence")
        # Kalman prediction
        self.kalman_filter.predict()
        
        # Create cropped intrinsics
        full_intrinsics = o3d.camera.PinholeCameraIntrinsic(
            width=W, height=H,
            fx=intrinsics.fx, fy=intrinsics.fy,
            cx=intrinsics.ppx, cy=intrinsics.ppy
        )

        # PnP alignment
        pnp_aligner = PnPKeypointAligner(
            self.template_np, 
            full_intrinsics.intrinsic_matrix
        )
        
        
        kp = self.descendants_info.get_ellipse_centroids()
        
        print("working here?")
        
        _, pose_info = pnp_aligner.align_keypoints_pnp(
            kp, marker_poses, correspondences
        )

        print("working here?2")

        print(correspondences)
        
        if pose_info and pose_info["success"] and pose_info["confidence_score"] >= 0.95: # very important, should be higher like 0.95(it is most important)
            # input("fail")

            # Update Kalman filter
            self.kalman_filter.update(
                pose_info['world_pose'], 
                measurement_quality=pose_info["confidence_score"]
            )
            
            K = np.array([[intrinsics.fx, 0, intrinsics.ppx],
                [0, intrinsics.fy, intrinsics.ppy],
                [0, 0, 1]], dtype=np.float32)
            
            # Draw bounding box
            cv2.rectangle(color_image, (int(x_min), int(y_min)), 
                        (int(x_max), int(y_max)), (0, 255, 0), 2)
            

            
            print(f"BBox Center: (x={x_center}, y={y_center})")
        current_pose = self.kalman_filter.get_pose(marker_poses)
        
        
        
        draw_projected_arrow_cv(color_image, current_pose["rotation_object_to_camera"][:, 0], current_pose["position_object_to_camera"], intrinsics.fx, intrinsics.fy, intrinsics.ppx, intrinsics.ppy, scale=0.1, color=(0,0, 255)) # Red - Normal
        draw_projected_arrow_cv(color_image, current_pose["rotation_object_to_camera"][:, 1], current_pose["position_object_to_camera"], intrinsics.fx, intrinsics.fy, intrinsics.ppx, intrinsics.ppy, scale=0.1, color=(0,0, 255)) # Red - Normal
        draw_projected_arrow_cv(color_image, current_pose["rotation_object_to_camera"][:, 2], current_pose["position_object_to_camera"], intrinsics.fx, intrinsics.fy, intrinsics.ppx, intrinsics.ppy, scale=0.1, color=(0,0, 255)) # Red - Normal
        
        # color_image = self.draw_pose_axes(color_image, current_pose["rotation_object_to_camera"], current_pose["position_object_to_camera"], cropped_intrinsics.intrinsic_matrix, np.array(intrinsics.coeffs), (y_min, x_min))
        org = (50, 100)   # bottom-left corner of the text (x, y)

        # Font, scale, color, thickness
        font = cv2.FONT_HERSHEY_SIMPLEX
        font_scale = 1
        color = (0, 255, 0)   # BGR (green)
        thickness = 2

        cv2.putText(color_image, str(current_pose["convergence_score"]), org, font, font_scale, color, thickness, cv2.LINE_AA)   
        
        cv2.imshow("Pose Visualization", color_image)
        cv2.waitKey(1)
        # offset = current_pose["rotation_object_to_world"]@np.array([0,0,0.5])
        # current_pose["position_object_to_world"] -= offset

        return current_pose