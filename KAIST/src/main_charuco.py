import pyrealsense2 as rs
import numpy as np
import cv2
from ultralytics import YOLO
from keypoint_detector.pidi import ContourFinder
from keypoint_detector.pidinet import PiDiNet
from utils import draw_projected_arrow_cv, project_point, Points
from kalman_filter.kalman_filter import StaticObjectPoseKalmanFilter
from scipy.spatial.transform import Rotation as R
import open3d as o3d
from keypoint_detector.aruco import ArucoCameraPoseEstimator
from keypoint_detector.feature_matcher import preprocess2, preprocess_with_yolo
from pymodbus.client import ModbusTcpClient
import os
from keypoint_detector.pnp import PnPKeypointAligner
import struct
import json

def float_to_registers(f):
    # Ensure we have a proper float
    f = float(f)
    hi, lo = struct.unpack(">HH", struct.pack(">f", f))
    return [int(hi), int(lo)]

def run():
    np.random.seed(42) 

    client = ModbusTcpClient("127.0.0.1", port=5020)
    client.connect()
    
    kalman_filter = StaticObjectPoseKalmanFilter(
        process_noise_std=1e-6,
        position_noise_std=0.02,  # 2cm position uncertainty
        rotation_noise_std=0.1    # ~5.7 degree rotation uncertainty
    )

    with open("point_cloud/detected_circles.json", "r") as f:
        template = json.load(f)
        template_np = np.array([
            (x["center_x"], x["center_y"], x["center_z"]) for x in template
        ])
        template_np = template_np/1000
    charuco_board_config = {
    'grid_size': (8, 6),      # Your grid_width, grid_height  
    'square_size': 0.0298, #0.007958667,    # 50 pixels at 300 DPI = 4.2mm
    'marker_size': 0.02091, #0.005588,   # 35 pixels at 300 DPI = 2.97mm
    }
    # print("size of 50 is 0.0423334 m nad of 70 is 0.0592667 m")

    contour_finder = ContourFinder()

    aruco_estimator = ArucoCameraPoseEstimator()
    # RealSense Camera Setup
    pipeline = rs.pipeline()
    config = rs.config()

    # Enable the depth and color streams
    # config.enable_stream(rs.stream.infrared, 1, 640, 480, rs.format.y8, 15)
    # config.enable_stream(rs.stream.infrared, 2, 640, 480, rs.format.y8, 15)
    # config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 15)
    config.enable_stream(rs.stream.color, 1920, 1080, rs.format.bgr8, 15) # type: ignore

    # Start streaming
    pipeline.start(config)

    # Create alignment object to align depth frames to color frames
    align = rs.align(rs.stream.color)

    # Load the YOLO model
    MODEL_PATH = f'best.pt'  # Update with your trained model path
    MODEL_C_PATH = f'circle_detection_weights.pt' 

    model = YOLO(MODEL_PATH)

    c_model = YOLO(MODEL_C_PATH)
    w_R = None
    try:
        while True:

            current_pose, color_image, intrinsics = get_port_normal_vector(pipeline, align, aruco_estimator, charuco_board_config, model, c_model, kalman_filter, template_np, contour_finder)

            # print(current_pose)
            if current_pose is not None and "rotation_object_to_camera" in current_pose:# and R_p is not None:
                Rk = current_pose["rotation_object_to_camera"]
                centroid = current_pose["position_object_to_camera"]

                offset_centroid = centroid + 0.2 * Rk[:, 2]

                w_R = current_pose["rotation_object_to_world"]
                c_R = current_pose["position_object_to_world"]

                roll, pitch, yaw = R.from_matrix(w_R).as_euler("xyz", degrees=True)
                w_R_text = f"Roll: {roll} - Pitch: {pitch} - Yaw: {yaw}"
                c_R_text = f"x: {c_R[0]} - y: {c_R[1]} - z: {c_R[2]}"
                c_2d = draw_projected_arrow_cv(color_image, Rk[:, 0], centroid, intrinsics.fx, intrinsics.fy, intrinsics.ppx, intrinsics.ppy, color=(255,0, 0)) # Blue - Normal
                draw_projected_arrow_cv(color_image, Rk[:, 1], centroid, intrinsics.fx, intrinsics.fy, intrinsics.ppx, intrinsics.ppy, color=(0,255, 0)) # Green - Normal
                # draw_projected_arrow_cv(color_image, b, R_p[:, 2], centroid, intrinsics.fx, intrinsics.fy, intrinsics.ppx, intrinsics.ppy, color=(0,0, 255)) # Blue - Normal
                draw_projected_arrow_cv(color_image, Rk[:, 2], centroid, intrinsics.fx, intrinsics.fy, intrinsics.ppx, intrinsics.ppy, color=(0,0, 255)) # Red - Normal
                draw_projected_arrow_cv(color_image, -Rk[:, 2], offset_centroid, intrinsics.fx, intrinsics.fy, intrinsics.ppx, intrinsics.ppy, color=(0,0, 255)) # Red - Normal
                draw_projected_arrow_cv(color_image, Rk[:, 1], offset_centroid, intrinsics.fx, intrinsics.fy, intrinsics.ppx, intrinsics.ppy, color=(0,255, 0)) # Red - Normal
                draw_projected_arrow_cv(color_image, Rk[:, 0], offset_centroid, intrinsics.fx, intrinsics.fy, intrinsics.ppx, intrinsics.ppy, color=(255, 0,0)) # Red - Normal
                
                pt1 = offset_centroid - 0.02*Rk[:,0] + 0.02*Rk[:,1]
                pt2 = offset_centroid + 0.02*Rk[:,0] + 0.02*Rk[:,1]
                pt3 = offset_centroid + 0.02*Rk[:,0] - 0.02*Rk[:,1]
                pt4 = offset_centroid - 0.02*Rk[:,0] - 0.02*Rk[:,1]

                pt1 = project_point(pt1, intrinsics.fx, intrinsics.fy, intrinsics.ppx, intrinsics.ppy)
                pt2 = project_point(pt2, intrinsics.fx, intrinsics.fy, intrinsics.ppx, intrinsics.ppy)
                pt3 = project_point(pt3, intrinsics.fx, intrinsics.fy, intrinsics.ppx, intrinsics.ppy)
                pt4 = project_point(pt4, intrinsics.fx, intrinsics.fy, intrinsics.ppx, intrinsics.ppy)

                pts = np.array([pt1, pt2, pt3, pt4])

                cv2.polylines(
                    color_image,
                    [pts],
                    True,
                    color = (100,255,128),
                    thickness=2
                )
                
                cv2.putText(color_image, w_R_text, c_2d, cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
                cv2.putText(color_image, c_R_text, (c_2d[0], c_2d[1] + 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
                # draw_projected_arrow_cv(color_image, b, -marker_poses["board_rotation_in_camera"][:, 2], centroid, intrinsics.fx, intrinsics.fy, intrinsics.ppx, intrinsics.ppy, color=(255,89, 185)) # Blue - Normal
                # draw_projected_arrow_cv(color_image, b, -marker_poses["board_rotation_in_camera"][:, 1], centroid, intrinsics.fx, intrinsics.fy, intrinsics.ppx, intrinsics.ppy, color=(45,138, 128)) # Blue - Normal
                
                # draw_projected_arrow_cv(color_image, b, -marker_poses["camera_rotation"][2], marker_poses["board_position_in_camera"], intrinsics.fx, intrinsics.fy, intrinsics.ppx, intrinsics.ppy, color=(100, 30, 200))

                # color_image = aruco_estimator.visualize_charuco(color_image, intrinsics, marker_poses, charuco_board_config, 
                #                                                 similarity_vectors=np.dot(marker_poses["camera_rotation"][2], R[2]) / (np.linalg.norm(marker_poses["camera_rotation"][2]) * np.linalg.norm(R[2])), 
                                                                # c_centroid=np.linalg.norm(centroid - marker_poses["board_position_in_camera"]), show_info=True)

            
            cv2.imshow("color", color_image)
            # cv2.imwrite("color.jpg", color_image)
            if w_R is not None:
                regs = []
                for val in w_R[0].flatten():
                    regs.extend(float_to_registers(float(val)))  # ensure Python float first
                    
                print("Registers:", regs, [type(x) for x in regs])
                # Now force all to Python int explicitly
                regs = [int(x) for x in regs]
                            
                try:
                    rr = client.write_registers(address=0, values=regs)
                    if rr.isError():
                        print(f"Write failed: {rr}")
                    else:
                        print(f"Successfully sent rotation matrix")
                except Exception as e:
                    print(f"Exception during write: {e}")
                    print(f"Register values: {regs}")
                    print(f"Register types: {[type(x) for x in regs]}")
                
            key = cv2.waitKey(1)
            if key & 0xFF == 27:  # ESC key
                break

    finally:
        pipeline.stop()
        cv2.destroyAllWindows()
        client.close()

def get_port_normal_vector(pipeline, align, aruco_estimator: ArucoCameraPoseEstimator, charuco_board_config, model, c_model, kalman_filter, template_np, contour_finder):
    current_pose = None

    # Wait for a coherent pair of frames: depth and color
    frames = pipeline.wait_for_frames()
    aligned_frames = align.process(frames)

    # Get aligned frames
    # depth_frame = aligned_frames.get_depth_frame()
    color_frame = aligned_frames.get_color_frame()

    # Convert frames to numpy arrays
    color_image = np.asanyarray(color_frame.get_data())
    results = model(color_image)
    intrinsics = color_frame.profile.as_video_stream_profile().intrinsics

    if not color_frame:
        print("No marker depth frame or color frame found")
        return current_pose, color_image, intrinsics

    # Get the intrinsics for the color stream
    full_intrinsics = o3d.camera.PinholeCameraIntrinsic(
        width=color_image.shape[1], height=color_image.shape[0],
        fx=intrinsics.fx, fy=intrinsics.fy,
        cx=intrinsics.ppx, cy=intrinsics.ppy
    )

    marker_poses = aruco_estimator.detect_and_estimate_charuco_pose(
        color_image, intrinsics, charuco_board_config
    )
    # color_image = aruco_estimator.visualize_charuco(color_image, intrinsics, marker_poses, charuco_board_config)


    if marker_poses is None:
        print("No marker poses found")
        return current_pose, color_image, intrinsics
        
    current_pose = kalman_filter.get_pose(marker_poses)
    
    if len(results[0].boxes) > 0:

        kalman_filter.predict()

        box = results[0].boxes[0]
        # Extract bounding box coordinates0.0010000000474974513
        x_min, y_min, x_max, y_max = box.xyxy[0]  # Bounding box (absolute pixel values)

        # Compute center coordinates of the bounding box
        offset = 0
        b = Points(x_min=int(x_min-offset), x_max=int(x_max+offset), y_min=int(y_min-offset), y_max=int(y_max+offset))

        cropped_color_img = color_image[b.y_min:b.y_max, b.x_min:b.x_max].copy()

        c_results = c_model(cropped_color_img)
        edge_map = contour_finder.process_img(cropped_color_img)
        # cv2.imshow('Edges', edge_map)  # OpenCV handles float [0, 1] for display

        descendants_info, correspondences, _, _ = preprocess_with_yolo(edge_map, c_results[0], template_np)
        
        # cv2.imshow("cropped", cropped_color_img)

        if descendants_info is None or len(correspondences) == 0 :
            return current_pose, color_image, intrinsics

        for i, ellipse in enumerate(descendants_info.get_ellipses(offset=(b.x_min, b.y_min))):
            cv2.circle(color_image, ellipse[0], 2, (255, 255, 255), 2)
            cv2.putText(color_image, str(i), ellipse[0], cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

        cropped_intrinsics = o3d.camera.PinholeCameraIntrinsic(
            width=b.w, height=b.h,
            fx=intrinsics.fx, fy=intrinsics.fy,
            cx=intrinsics.ppx - b.x_min, cy=intrinsics.ppy - b.y_min
        )
        print(correspondences)
        pnp_aligner = PnPKeypointAligner(template_np, cropped_intrinsics.intrinsic_matrix)

        kp = descendants_info.get_centroids()
        _, pose_info = pnp_aligner.align_keypoints_pnp(kp, marker_poses, correspondences)#, correspondences=correspondences)
        # visualize_pnp_alignment(pnp_aligner, kp)
        
        
        if pose_info is None or not pose_info["success"]:
            print("No pose info found")
            return current_pose, color_image, intrinsics

        print("updating kalman filter")
        kalman_filter.update(
            pose_info['world_pose'], 
            measurement_quality=0.95#pose_info["reprojection_error"], #pose_quality
        )

    return current_pose, color_image, intrinsics



if __name__ == "__main__":
    run()