#!/usr/bin/env python3
"""
ArduCam Camera Controller Library
Control ArduCam-40 camera and detect ArUco markers
"""

import numpy as np
import cv2
import cv2.aruco as aruco
import yaml
from typing import Optional, Tuple, Dict, List


class ArduCamController:
    """
    ArduCam Camera Controller with ArUco Detection

    Controls ArduCam-40 camera with calibrated parameters and provides
    ArUco marker detection and pose estimation capabilities.
    """

    def __init__(self,
                 calibration_file: str = 'config/arducam40_calibration.yaml',
                 aruco_dict_type: int = aruco.DICT_4X4_50,
                 marker_size: float = 0.05,
                 camera_index: int = 0):
        """
        Initialize ArduCam Controller

        Args:
            calibration_file: Path to calibration YAML file
            aruco_dict_type: ArUco dictionary type (default: DICT_4X4_50)
            marker_size: Physical size of marker in meters (default: 0.05m = 5cm)
            camera_index: Camera device index (default: 0)
        """
        # Load calibration data
        self.calibration_file = calibration_file
        self.load_calibration()

        # ArUco detector parameters
        self.aruco_dict = aruco.getPredefinedDictionary(aruco_dict_type)
        self.aruco_params = aruco.DetectorParameters_create()
        self.marker_size = marker_size

        # Camera setup
        self.camera_index = camera_index
        self.cap = None
        self._is_running = False

    # ==================== Calibration ====================

    def load_calibration(self):
        """Load calibration data from YAML file"""
        try:
            with open(self.calibration_file, 'r') as f:
                calib_data = yaml.safe_load(f)

            self.camera_name = calib_data.get('camera_name', 'ArduCam')
            self.calib_serial = calib_data.get('serial_number', 'N/A')
            self.calib_width = calib_data['image_width']
            self.calib_height = calib_data['image_height']

            # Camera matrix
            self.camera_matrix = np.array([
                [calib_data['fx'], calib_data.get('skew', 0), calib_data['cx']],
                [0, calib_data['fy'], calib_data['cy']],
                [0, 0, 1]
            ], dtype=np.float32)

            # Distortion coefficients
            self.dist_coeffs = np.array([
                calib_data['k1'],
                calib_data['k2'],
                calib_data.get('p1', 0),
                calib_data.get('p2', 0),
                calib_data.get('k3', 0)
            ], dtype=np.float32)

            print(f"✅ ArduCam calibration loaded:")
            print(f"   Camera: {self.camera_name}")
            print(f"   Resolution: {self.calib_width}x{self.calib_height}")
            if 'reprojection_error' in calib_data:
                print(f"   Reprojection Error: {calib_data['reprojection_error']}")

        except FileNotFoundError:
            print(f"❌ Calibration file '{self.calibration_file}' not found")
            raise
        except Exception as e:
            print(f"❌ Error loading calibration: {e}")
            raise

    # ==================== Camera Control ====================

    def start(self) -> bool:
        """
        Start the ArduCam camera

        Returns:
            bool: True if successful, False otherwise
        """
        if self._is_running:
            print("⚠️  Camera already running")
            return True

        print(f"Starting {self.camera_name} (index: {self.camera_index})...")

        self.cap = cv2.VideoCapture(self.camera_index)

        if not self.cap.isOpened():
            print(f"❌ Cannot open camera {self.camera_index}")
            return False

        # Set resolution to match calibration
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.calib_width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.calib_height)

        # Get actual resolution
        actual_width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        actual_height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

        print(f"📷 Camera Resolution: {actual_width}x{actual_height}")

        # Warn if resolution mismatch
        if (actual_width, actual_height) != (self.calib_width, self.calib_height):
            print(f"⚠️  Resolution mismatch!")
            print(f"   Calibration: {self.calib_width}x{self.calib_height}")
            print(f"   Actual: {actual_width}x{actual_height}")

        # Wait for camera to stabilize
        for _ in range(30):
            self.cap.read()

        self._is_running = True
        print("✅ ArduCam ready!")
        return True

    def stop(self):
        """Stop the camera"""
        if self.cap is not None:
            self.cap.release()
            self._is_running = False
            print("🔌 ArduCam stopped")

    def is_running(self) -> bool:
        """Check if camera is running"""
        return self._is_running

    def get_frame(self) -> Optional[np.ndarray]:
        """
        Capture and return BGR frame

        Returns:
            np.ndarray: BGR image or None if failed
        """
        if not self._is_running or self.cap is None:
            return None

        ret, frame = self.cap.read()

        if not ret:
            return None

        return frame

    # ==================== ArUco Detection ====================

    def detect_markers(self, image: np.ndarray) -> Tuple[List, Optional[np.ndarray], List]:
        """
        Detect ArUco markers in the image

        Args:
            image: Input BGR image

        Returns:
            corners: Detected marker corners
            ids: Detected marker IDs (None if no markers)
            rejected: Rejected candidates
        """
        # Convert to grayscale
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        # Detect markers
        corners, ids, rejected = aruco.detectMarkers(
            gray,
            self.aruco_dict,
            parameters=self.aruco_params
        )

        return corners, ids, rejected

    def estimate_pose(self, corners: List) -> Tuple[np.ndarray, np.ndarray]:
        """
        Estimate pose of detected markers

        Args:
            corners: Marker corners from detect_markers()

        Returns:
            rvecs: Rotation vectors
            tvecs: Translation vectors
        """
        rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(
            corners,
            self.marker_size,
            self.camera_matrix,
            self.dist_coeffs
        )

        return rvecs, tvecs

    def get_marker_pose_data(self, rvecs: np.ndarray, tvecs: np.ndarray,
                            marker_idx: int = 0) -> Dict:
        """
        Extract pose data from rotation and translation vectors

        Args:
            rvecs: Rotation vectors
            tvecs: Translation vectors
            marker_idx: Index of marker to extract data from

        Returns:
            Dictionary with pose data:
            {
                'position': {'x': float, 'y': float, 'z': float},  # in meters
                'rotation': {'roll': float, 'pitch': float, 'yaw': float},  # in degrees
                'distance': float  # in meters
            }
        """
        tvec = tvecs[marker_idx][0]
        rvec = rvecs[marker_idx]

        # Calculate distance
        distance = np.linalg.norm(tvec)

        # Convert rotation vector to rotation matrix
        rotation_matrix, _ = cv2.Rodrigues(rvec)

        # Calculate Euler angles (roll, pitch, yaw) in degrees
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

        return {
            'position': {
                'x': float(tvec[0]),
                'y': float(tvec[1]),
                'z': float(tvec[2])
            },
            'rotation': {
                'roll': float(roll_deg),
                'pitch': float(pitch_deg),
                'yaw': float(yaw_deg)
            },
            'distance': float(distance)
        }

    # ==================== Visualization ====================

    def draw_detections(self, image: np.ndarray, corners: List,
                       ids: Optional[np.ndarray],
                       rvecs: Optional[np.ndarray] = None,
                       tvecs: Optional[np.ndarray] = None,
                       draw_axes: bool = True,
                       draw_info: bool = True) -> np.ndarray:
        """
        Draw detected markers and pose on image

        Args:
            image: Input BGR image
            corners: Marker corners
            ids: Marker IDs
            rvecs: Rotation vectors (optional)
            tvecs: Translation vectors (optional)
            draw_axes: Whether to draw 3D axes
            draw_info: Whether to draw text information

        Returns:
            output_image: Image with drawn markers
        """
        output_image = image.copy()

        if ids is not None and len(ids) > 0:
            # Draw detected markers
            aruco.drawDetectedMarkers(output_image, corners, ids)

            # Draw axis and info for each marker if pose is estimated
            if rvecs is not None and tvecs is not None:
                for i in range(len(ids)):
                    # Draw 3D axis
                    if draw_axes:
                        cv2.drawFrameAxes(
                            output_image,
                            self.camera_matrix,
                            self.dist_coeffs,
                            rvecs[i],
                            tvecs[i],
                            self.marker_size * 0.5
                        )

                    # Draw text information
                    if draw_info:
                        pose_data = self.get_marker_pose_data(rvecs, tvecs, i)

                        # Get corner position for text
                        corner = corners[i][0][0]
                        x, y = int(corner[0]), int(corner[1])

                        # Display marker ID and distance
                        text1 = f"ID:{ids[i][0]} D:{pose_data['distance']*100:.1f}cm"
                        cv2.putText(output_image, text1, (x, y - 15),
                                  cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)

                        # Display position (X, Y, Z)
                        pos = pose_data['position']
                        text2 = f"Pos: X:{pos['x']*100:.1f} Y:{pos['y']*100:.1f} Z:{pos['z']*100:.1f}cm"
                        cv2.putText(output_image, text2, (x, y - 40),
                                  cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 2)

                        # Display orientation (Roll, Pitch, Yaw)
                        rot = pose_data['rotation']
                        text3 = f"Rot: R:{rot['roll']:.1f} P:{rot['pitch']:.1f} Y:{rot['yaw']:.1f}deg"
                        cv2.putText(output_image, text3, (x, y - 65),
                                  cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 255), 2)

        return output_image

    # ==================== Utility Methods ====================

    def get_intrinsics(self) -> Dict:
        """
        Get camera intrinsic parameters

        Returns:
            Dictionary with camera matrix and distortion coefficients
        """
        return {
            'camera_matrix': self.camera_matrix,
            'dist_coeffs': self.dist_coeffs,
            'width': self.calib_width,
            'height': self.calib_height
        }

    # ==================== Context Manager Support ====================

    def __enter__(self):
        """Context manager entry"""
        self.start()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        """Context manager exit"""
        self.stop()

    # ==================== String Representation ====================

    def __str__(self):
        status = "Running" if self._is_running else "Stopped"
        return (f"ArduCamController({self.camera_name}) - {status}\n"
               f"  Resolution: {self.calib_width}x{self.calib_height}\n"
               f"  Camera Index: {self.camera_index}")


# ==================== Example Usage ====================

def example_usage():
    """Example of how to use ArduCamController"""

    # Create controller instance
    arducam = ArduCamController(
        calibration_file='config/arducam40_calibration_data.json',
        aruco_dict_type=aruco.DICT_4X4_50,
        marker_size=0.02,  # 2cm marker
        camera_index=0
    )

    # Start camera
    if not arducam.start():
        print("Failed to start camera")
        return

    try:
        print("Press 'q' to quit")

        while True:
            # Get frame
            frame = arducam.get_frame()

            if frame is None:
                continue

            # Detect markers
            corners, ids, rejected = arducam.detect_markers(frame)

            # Estimate pose if markers detected
            if ids is not None and len(ids) > 0:
                rvecs, tvecs = arducam.estimate_pose(corners)

                # Draw detections
                output = arducam.draw_detections(frame, corners, ids, rvecs, tvecs)

                # Print pose data for first marker
                pose_data = arducam.get_marker_pose_data(rvecs, tvecs, 0)
                print(f"Marker 0: {pose_data}")
            else:
                output = frame

            # Display
            cv2.imshow('ArduCam', output)

            # Check for quit
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    finally:
        arducam.stop()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    example_usage()
