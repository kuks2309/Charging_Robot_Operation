import pyrealsense2 as rs
import numpy as np
import cv2
from ultralytics import YOLO
from dataclasses import dataclass
from utils import draw_projected_arrow_cv
from kalman_filter.kalman_filter import StaticObjectPoseKalmanFilter
from scipy.spatial.transform import Rotation as R
import open3d as o3d
from keypoint_detector.aruco import ArucoCameraPoseEstimator
import copy
from pymodbus.client import ModbusTcpClient

from keypoint_detector.pnp import PnPKeypointAligner, visualize_pnp_alignment
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

    charuco_board_config = {
    'grid_size': (8, 6),      # Your grid_width, grid_height  
    'square_size': 0.029,#8, #0.007958667,    # 50 pixels at 300 DPI = 4.2mm
    'marker_size': 0.02,#091, #0.005588,   # 35 pixels at 300 DPI = 2.97mm
    }
    # print("size of 50 is 0.0423334 m nad of 70 is 0.0592667 m")

    aruco_estimator = ArucoCameraPoseEstimator()
    # RealSense Camera Setup
    pipeline = rs.pipeline()
    config = rs.config()

    # Enable the depth and color streams
    # config.enable_stream(rs.stream.infrared, 1, 640, 480, rs.format.y8, 15)
    # config.enable_stream(rs.stream.infrared, 2, 640, 480, rs.format.y8, 15)
    # config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 15)
    config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 15) # type: ignore

    # Start streaming
    pipeline.start(config)

    # Create alignment object to align depth frames to color frames
    align = rs.align(rs.stream.color)

    # Load the YOLO model
    MODEL_PATH = f'best.pt'  # Update with your trained model path
    model = YOLO(MODEL_PATH)
    w_R = None
    try:
        while True:

            current_pose, color_image, intrinsics = get_port_normal_vector(pipeline, align, aruco_estimator, charuco_board_config, model, kalman_filter, template_np)
            print(current_pose)
            # input("wait")
            if current_pose is not None and "rotation_object_to_camera" in current_pose:# and R_p is not None:
                Rk = current_pose["rotation_object_to_camera"]
                centroid = current_pose["position_object_to_camera"]
                w_R = current_pose["rotation_object_to_world"]
                c_R = current_pose["position_object_to_world"]

                roll, pitch, yaw = R.from_matrix(w_R).as_euler("xyz", degrees=True)
                w_R_text = f"Roll: {roll:.2f} - Pitch: {pitch:.2f} - Yaw: {yaw:.2f}"
                c_R_text = f"x: {c_R[0]*1000:.2f} - y: {c_R[1]*1000:.2f} - z: {c_R[2]*1000:.2f}"
                c_2d = draw_projected_arrow_cv(color_image, Rk[:, 0], centroid, intrinsics.fx, intrinsics.fy, intrinsics.ppx, intrinsics.ppy, color=(255,0, 0)) # Blue - Normal
                draw_projected_arrow_cv(color_image, Rk[:, 1], centroid, intrinsics.fx, intrinsics.fy, intrinsics.ppx, intrinsics.ppy, color=(0,255, 0)) # Blue - Normal
                # draw_projected_arrow_cv(color_image, b, R_p[:, 2], centroid, intrinsics.fx, intrinsics.fy, intrinsics.ppx, intrinsics.ppy, color=(0,0, 255)) # Blue - Normal
                draw_projected_arrow_cv(color_image, -Rk[:, 2], centroid, intrinsics.fx, intrinsics.fy, intrinsics.ppx, intrinsics.ppy, color=(4,164, 255)) # Blue - Normal
                cv2.putText(color_image, w_R_text, c_2d, cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
                cv2.putText(color_image, c_R_text, (c_2d[0], c_2d[1] + 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
                # draw_projected_arrow_cv(color_image, b, -marker_poses["board_rotation_in_camera"][:, 2], centroid, intrinsics.fx, intrinsics.fy, intrinsics.ppx, intrinsics.ppy, color=(255,89, 185)) # Blue - Normal
                # draw_projected_arrow_cv(color_image, b, -marker_poses["board_rotation_in_camera"][:, 1], centroid, intrinsics.fx, intrinsics.fy, intrinsics.ppx, intrinsics.ppy, color=(45,138, 128)) # Blue - Normal
                
                # draw_projected_arrow_cv(color_image, b, -marker_poses["camera_rotation"][2], marker_poses["board_position_in_camera"], intrinsics.fx, intrinsics.fy, intrinsics.ppx, intrinsics.ppy, color=(100, 30, 200))

                # color_image = aruco_estimator.visualize_charuco(color_image, intrinsics, marker_poses, charuco_board_config, 
                #                                                 similarity_vectors=np.dot(marker_poses["camera_rotation"][2], R[2]) / (np.linalg.norm(marker_poses["camera_rotation"][2]) * np.linalg.norm(R[2])), 
                                                                # c_centroid=np.linalg.norm(centroid - marker_poses["board_position_in_camera"]), show_info=True)

            cv2.imshow("color", color_image)
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
                
            key = cv2.waitKey(10)
            if key & 0xFF == 27:  # ESC key
                break

    finally:
        pipeline.stop()
        cv2.waitKey(100)   # 잠깐 대기 후
        cv2.destroyAllWindows()
        client.close()

def get_port_normal_vector(pipeline, align, aruco_estimator, charuco_board_config, model, kalman_filter, template_np):
    Rk = None
    w_R = None
    current_pose = None

    # Wait for a coherent pair of frames: depth and color
    frames = pipeline.wait_for_frames()
    aligned_frames = align.process(frames)

    # Get aligned frames
    color_frame = aligned_frames.get_color_frame()

    # Convert frames to numpy arrays
    color_image = np.asanyarray(color_frame.get_data())
    # depth_image = np.asanyarray(depth_frame.get_data())
    results = model(color_image)
    intrinsics = color_frame.profile.as_video_stream_profile().intrinsics

    if not color_frame:
        print("No marker depth frame or color frame found")
        return current_pose, color_image, intrinsics

    marker_poses = aruco_estimator.detect_and_estimate_charuco_pose(
        color_image, intrinsics, charuco_board_config
    )
    
    color_image = aruco_estimator.visualize_charuco(color_image, intrinsics, marker_poses, charuco_board_config)
    current_pose = kalman_filter.get_pose(marker_poses)

    if marker_poses is None:
        print("No marker poses found")
        return current_pose, color_image, intrinsics
    
    kalman_filter.predict()
    
    marker_len = 0.0182
    objp = np.array([
        [-marker_len/2,  marker_len/2, 0],
        [ marker_len/2,  marker_len/2, 0],
        [ marker_len/2, -marker_len/2, 0],
        [-marker_len/2, -marker_len/2, 0]
    ], dtype=np.float32)
    
    marker_offset = {0: (0.0, -0.025, +0.07, 0, 0, 0)}
    
    # --- AprilTag detection & pose estimation ---
    pnp_aligner = PnPKeypointAligner(objp,
                                     np.array([[intrinsics.fx, 0, intrinsics.ppx],
                                               [0, intrinsics.fy, intrinsics.ppy],
                                               [0, 0, 1]], dtype=np.float32),
                                     np.array(intrinsics.coeffs[:5], dtype=np.float32),
                                     offset=np.array([0, -0.025, +0.07, 0, 0, 0]))

    _, pose_info = pnp_aligner.align_keypoints_pnp_marker(color_image, marker_poses, marker_offset)
    
    if pose_info is None or not pose_info["success"]:
        print("No pose info found")
        return current_pose, color_image.copy(), intrinsics

    print("confidence_score: ", pose_info["confidence_score"])
    kalman_filter.update(
        pose_info['world_pose'], 
        measurement_quality=pose_info["confidence_score"],
    )
    current_pose = kalman_filter.get_pose(marker_poses)

    print("Is congerged:", current_pose["is_converged"])
    print("score", current_pose["convergence_score"])
    
    if current_pose is not None:
        # Get metrics
        confidence = pose_info.get("confidence_score", 0.0)
        is_converged = current_pose.get("is_converged", False)
        convergence_score = current_pose.get("convergence_score", 0.0)

        # Define base position
        overlay_origin = (30, 40)

        # Draw background rectangle (for readability)
        cv2.rectangle(
            color_image,
            (overlay_origin[0] - 10, overlay_origin[1] - 25),
            (overlay_origin[0] + 300, overlay_origin[1] + 80),
            (0, 0, 0),
            thickness=-1,
        )

        # Text overlays
        cv2.putText(color_image, f"Confidence: {confidence:.3f}",
                    (overlay_origin[0], overlay_origin[1]),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

        cv2.putText(color_image, f"Convergence Score: {convergence_score:.3f}",
                    (overlay_origin[0], overlay_origin[1] + 25),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

        cv2.putText(color_image, f"Converged: {'YES' if is_converged else 'NO'}",
                    (overlay_origin[0], overlay_origin[1] + 50),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                    (0, 255, 0) if is_converged else (0, 0, 255), 2)
    
    return current_pose, color_image, intrinsics

if __name__ == "__main__":
    run()