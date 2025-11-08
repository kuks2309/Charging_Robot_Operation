#!/usr/bin/env python3
"""
Intel RealSense D435 ArUco Marker Detection Script
This script detects ArUco markers using calibrated camera parameters.
"""

import pyrealsense2 as rs
import numpy as np
import cv2
import cv2.aruco as aruco
import json
import sys


class D435ArUcoDetector:
    def __init__(self, calibration_file='ds435_calibration_data.json',
                 aruco_dict_type=aruco.DICT_4X4_50, marker_size=0.05):
        """
        Initialize D435 ArUco Detector

        Args:
            calibration_file: Path to calibration JSON file
            aruco_dict_type: ArUco dictionary type (default: DICT_4X4_50)
            marker_size: Physical size of marker in meters (default: 0.05m = 5cm)
        """
        # Load calibration data
        self.load_calibration(calibration_file)

        # ArUco detector parameters
        self.aruco_dict = aruco.getPredefinedDictionary(aruco_dict_type)
        self.aruco_params = aruco.DetectorParameters_create()
        self.marker_size = marker_size

        # Create pipeline
        self.pipeline = rs.pipeline()
        self.config = rs.config()

        # Configure stream with calibration resolution
        self.config.enable_stream(
            rs.stream.color,
            self.calib_width,
            self.calib_height,
            rs.format.bgr8,
            30
        )

    def load_calibration(self, calibration_file):
        """Load calibration data from JSON file"""
        try:
            with open(calibration_file, 'r') as f:
                calib_data = json.load(f)

            self.calib_serial = calib_data['seril_number']
            self.calib_width = calib_data['width']
            self.calib_height = calib_data['height']

            # Camera matrix
            self.camera_matrix = np.array([
                [calib_data['fx'], 0, calib_data['cx']],
                [0, calib_data['fy'], calib_data['cy']],
                [0, 0, 1]
            ], dtype=np.float32)

            # Distortion coefficients
            self.dist_coeffs = np.array([
                calib_data['k1'],
                calib_data['k2'],
                calib_data['p1'],
                calib_data['p2'],
                calib_data['k3']
            ], dtype=np.float32)

            print(f"Loaded calibration data:")
            print(f"  Serial Number: {self.calib_serial}")
            print(f"  Resolution: {self.calib_width}x{self.calib_height}")
            print(f"  Reprojection Error: {calib_data['reprojection_error']}")

        except FileNotFoundError:
            print(f"Error: Calibration file '{calibration_file}' not found")
            sys.exit(1)
        except Exception as e:
            print(f"Error loading calibration file: {e}")
            sys.exit(1)

    def start(self):
        """Start the camera pipeline"""
        print("\nStarting RealSense D435 camera...")
        profile = self.pipeline.start(self.config)

        # Get device information
        device = profile.get_device()
        device_serial = device.get_info(rs.camera_info.serial_number)
        print(f"Camera Serial Number: {device_serial}")

        # Check if serial number matches
        if device_serial != self.calib_serial:
            print(f"\nWARNING: Serial number mismatch!")
            print(f"  Calibration file: {self.calib_serial}")
            print(f"  Connected camera: {device_serial}")
            response = input("Continue anyway? (y/n): ")
            if response.lower() != 'y':
                print("Exiting...")
                self.pipeline.stop()
                sys.exit(0)
        else:
            print("Serial number matched! Using calibration data.")

        # Wait for auto-exposure to stabilize
        print("Waiting for camera to stabilize...")
        for _ in range(30):
            self.pipeline.wait_for_frames()
        print("Camera ready!")

    def stop(self):
        """Stop the camera pipeline"""
        self.pipeline.stop()
        print("Camera stopped.")

    def get_frame(self):
        """
        Capture and return RGB frame

        Returns:
            color_image: numpy array of RGB image
        """
        # Wait for frames
        frames = self.pipeline.wait_for_frames()

        # Get color frame
        color_frame = frames.get_color_frame()

        if not color_frame:
            return None

        # Convert to numpy array
        color_image = np.asanyarray(color_frame.get_data())

        return color_image

    def detect_markers(self, image):
        """
        Detect ArUco markers in the image

        Args:
            image: Input image

        Returns:
            corners: Detected marker corners
            ids: Detected marker IDs
            rejected: Rejected candidates
        """
        # Convert to grayscale
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        # Detect markers (using legacy API for older OpenCV versions)
        corners, ids, rejected = aruco.detectMarkers(
            gray,
            self.aruco_dict,
            parameters=self.aruco_params
        )

        return corners, ids, rejected

    def estimate_pose(self, corners):
        """
        Estimate pose of detected markers

        Args:
            corners: Marker corners

        Returns:
            rvecs: Rotation vectors
            tvecs: Translation vectors
        """
        # Estimate pose
        rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(
            corners,
            self.marker_size,
            self.camera_matrix,
            self.dist_coeffs
        )

        return rvecs, tvecs

    def draw_detections(self, image, corners, ids, rvecs=None, tvecs=None):
        """
        Draw detected markers and pose on image

        Args:
            image: Input image
            corners: Marker corners
            ids: Marker IDs
            rvecs: Rotation vectors (optional)
            tvecs: Translation vectors (optional)

        Returns:
            output_image: Image with drawn markers
        """
        output_image = image.copy()

        if ids is not None and len(ids) > 0:
            # Draw detected markers
            aruco.drawDetectedMarkers(output_image, corners, ids)

            # Draw axis for each marker if pose is estimated
            if rvecs is not None and tvecs is not None:
                for i in range(len(ids)):
                    # Draw 3D axis
                    cv2.drawFrameAxes(
                        output_image,
                        self.camera_matrix,
                        self.dist_coeffs,
                        rvecs[i],
                        tvecs[i],
                        self.marker_size * 0.5
                    )

                    # Calculate distance
                    tvec = tvecs[i][0]
                    distance = np.linalg.norm(tvec)

                    # Convert rotation vector to rotation matrix
                    rotation_matrix, _ = cv2.Rodrigues(rvecs[i])

                    # Calculate Euler angles (roll, pitch, yaw) in degrees
                    # Using the rotation matrix to get angles
                    sy = np.sqrt(rotation_matrix[0, 0]**2 + rotation_matrix[1, 0]**2)

                    singular = sy < 1e-6

                    if not singular:
                        roll = np.arctan2(rotation_matrix[2, 1], rotation_matrix[2, 2])
                        pitch = np.arctan2(-rotation_matrix[2, 0], sy)
                        yaw = np.arctan2(rotation_matrix[1, 0], rotation_matrix[0, 0])
                    else:
                        roll = np.arctan2(-rotation_matrix[1, 2], rotation_matrix[1, 1])
                        pitch = np.arctan2(-rotation_matrix[2, 0], sy)
                        yaw = 0

                    # Convert to degrees
                    roll_deg = np.degrees(roll)
                    pitch_deg = np.degrees(pitch)
                    yaw_deg = np.degrees(yaw)

                    # Get corner position for text
                    corner = corners[i][0][0]  # Top-left corner
                    x, y = int(corner[0]), int(corner[1])

                    # Display marker ID and distance
                    text1 = f"ID:{ids[i][0]} D:{distance*100:.1f}cm"
                    cv2.putText(
                        output_image,
                        text1,
                        (x, y - 10),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.5,
                        (0, 255, 0),
                        2
                    )

                    # Display position (X, Y, Z)
                    text2 = f"Pos: X:{tvec[0]*100:.1f} Y:{tvec[1]*100:.1f} Z:{tvec[2]*100:.1f}cm"
                    cv2.putText(
                        output_image,
                        text2,
                        (x, y - 30),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.4,
                        (255, 255, 0),
                        1
                    )

                    # Display orientation (Roll, Pitch, Yaw)
                    text3 = f"Rot: R:{roll_deg:.1f} P:{pitch_deg:.1f} Y:{yaw_deg:.1f}deg"
                    cv2.putText(
                        output_image,
                        text3,
                        (x, y - 50),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.4,
                        (255, 0, 255),
                        1
                    )

        return output_image


def main():
    """Main function for ArUco marker detection"""
    # Create detector instance
    # You can change DICT_4X4_50 to other types like DICT_5X5_100, DICT_6X6_250, etc.
    detector = D435ArUcoDetector(
        calibration_file='ds435_calibration_data.json',
        aruco_dict_type=aruco.DICT_4X4_50,
        marker_size=0.02  # 20cm marker size (adjust to your actual marker size)
    )

    try:
        # Start camera
        detector.start()

        print("\nArUco Marker Detection:")
        print(f"  Dictionary: DICT_4X4_50")
        print(f"  Marker Size: {detector.marker_size * 100}cm")
        print("\nControls:")
        print("  Press 'q' or ESC to quit")

        marker_count = 0

        while True:
            # Get frame
            image = detector.get_frame()

            if image is None:
                continue

            # Detect markers
            corners, ids, rejected = detector.detect_markers(image)

            # Estimate pose if markers detected
            rvecs, tvecs = None, None
            if ids is not None and len(ids) > 0:
                rvecs, tvecs = detector.estimate_pose(corners)
                marker_count = len(ids)
            else:
                marker_count = 0

            # Draw detections
            output_image = detector.draw_detections(image, corners, ids, rvecs, tvecs)

            # Add status text
            status_text = f"Markers Detected: {marker_count}"
            cv2.putText(
                output_image,
                status_text,
                (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 255, 0) if marker_count > 0 else (0, 0, 255),
                2
            )

            # Display
            cv2.imshow('D435 ArUco Marker Detection', output_image)

            # Handle key press
            key = cv2.waitKey(1) & 0xFF

            # Quit
            if key == ord('q') or key == 27:  # 27 is ESC
                break

    except Exception as e:
        print(f"Error: {e}")
        import traceback
        traceback.print_exc()

    finally:
        # Cleanup
        detector.stop()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
