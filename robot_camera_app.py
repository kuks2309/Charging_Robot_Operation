#!/usr/bin/env python3
"""
Robot Camera Control Application
Main UI application for controlling dual camera system and robot TCP
"""

import sys
import os

# Fix Qt plugin conflict between OpenCV and PyQt5
# Delete OpenCV's Qt plugin directory from cv2 module after import
def fix_cv2_qt_conflict():
    """Remove OpenCV's Qt plugins to prevent conflicts with PyQt5"""
    import cv2
    cv2_dir = os.path.dirname(cv2.__file__)
    qt_plugin_dir = os.path.join(cv2_dir, 'qt', 'plugins')

    # Check if cv2 qt plugins directory exists
    if os.path.exists(qt_plugin_dir):
        # Set empty plugin path to disable cv2's Qt
        os.environ['QT_QPA_PLATFORM_PLUGIN_PATH'] = ''

    return cv2

import numpy as np

# Import PyQt5 first to claim the Qt plugin system
from PyQt5 import QtWidgets, QtCore, QtGui, uic
from PyQt5.QtCore import QTimer, pyqtSlot
from PyQt5.QtGui import QImage, QPixmap

# Then import cv2 with conflict fix
cv2 = fix_cv2_qt_conflict()
import pyrealsense2 as rs
import yaml

# Add lib directory to path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'lib'))

from aruco import ArucoCameraPoseEstimator
from robot_controller import RobotTCPController


class D435CameraController:
    """Controller for Intel RealSense D435 Camera"""

    def __init__(self, calibration_file='config/ds435_calibration.yaml'):
        self.calibration_file = calibration_file
        self.pipeline = None
        self.config = None
        self._is_running = False

        # ArUco detector (default to AprilTag 36h11)
        # Will be updated based on UI selection
        self.current_marker_dict = cv2.aruco.DICT_APRILTAG_36h11
        self.aruco_estimator = ArucoCameraPoseEstimator(
            marker_size_meters=0.02,  # 20mm markers
            dictionary_type=self.current_marker_dict
        )

        # Chessboard configuration (10x7 internal corners for 11x8 grid)
        self.chessboard_size = (10, 7)  # Internal corners
        self.square_size = 0.05  # 5cm squares

        # ChArUco board configuration (backup if using ChArUco board)
        self.charuco_board_config = {
            'grid_size': (11, 8),       # 11x8 grid (10x7 checkerboard corners)
            'square_size': 0.05,        # 5cm squares
            'marker_size': 0.035,       # 3.5cm markers
        }

        # Load calibration
        self.load_calibration()

    def load_calibration(self):
        """Load calibration data from YAML file"""
        try:
            with open(self.calibration_file, 'r') as f:
                calib_data = yaml.safe_load(f)

            self.camera_name = calib_data.get('camera_name', 'D435')
            self.calib_serial = calib_data.get('serial_number', '')
            self.calib_width = calib_data['image_width']
            self.calib_height = calib_data['image_height']

            # Store intrinsics for ArUco detection
            self.intrinsics = self._create_intrinsics_from_yaml(calib_data)

            print(f"✅ D435 calibration loaded:")
            print(f"   Camera: {self.camera_name}")
            print(f"   Resolution: {self.calib_width}x{self.calib_height}")

        except FileNotFoundError:
            print(f"❌ Calibration file '{self.calibration_file}' not found")
            raise
        except Exception as e:
            print(f"❌ Error loading calibration: {e}")
            raise

    def _create_intrinsics_from_yaml(self, calib_data):
        """Create intrinsics object from YAML calibration data"""
        class Intrinsics:
            def __init__(self, fx, fy, ppx, ppy, coeffs):
                self.fx = fx
                self.fy = fy
                self.ppx = ppx
                self.ppy = ppy
                self.coeffs = coeffs

        return Intrinsics(
            fx=calib_data['fx'],
            fy=calib_data['fy'],
            ppx=calib_data['cx'],
            ppy=calib_data['cy'],
            coeffs=[calib_data['k1'], calib_data['k2'],
                   calib_data.get('p1', 0), calib_data.get('p2', 0),
                   calib_data.get('k3', 0)]
        )

    def start(self):
        """Start the D435 camera"""
        if self._is_running:
            print("⚠️  D435 already running")
            return True

        print(f"Starting {self.camera_name}...")

        self.pipeline = rs.pipeline()
        self.config = rs.config()

        # Configure stream
        self.config.enable_stream(
            rs.stream.color,
            self.calib_width,
            self.calib_height,
            rs.format.bgr8,
            30
        )

        try:
            # Start pipeline
            profile = self.pipeline.start(self.config)

            # Wait for camera to stabilize
            print("Waiting for camera to stabilize...")
            for _ in range(30):
                self.pipeline.wait_for_frames()

            self._is_running = True
            print("✅ D435 camera ready!")
            return True

        except Exception as e:
            print(f"❌ Failed to start D435: {e}")
            return False

    def update_marker_type(self, marker_dict_type):
        """Update the ArUco/AprilTag marker dictionary type"""
        self.current_marker_dict = marker_dict_type
        self.aruco_estimator = ArucoCameraPoseEstimator(
            marker_size_meters=0.02,  # 20mm markers
            dictionary_type=marker_dict_type
        )

        # Check if AprilTag
        is_apriltag = 'APRILTAG' in str(marker_dict_type)
        marker_type = "AprilTag" if is_apriltag else "ArUco"
        print(f"🔄 Marker type updated to: {marker_type}")

    def stop(self):
        """Stop the camera"""
        if self.pipeline is not None:
            self.pipeline.stop()
            self._is_running = False
            print("🔌 D435 stopped")

    def is_running(self):
        """Check if camera is running"""
        return self._is_running

    def get_frame(self):
        """Get current frame from camera"""
        if not self._is_running or self.pipeline is None:
            return None

        try:
            frames = self.pipeline.wait_for_frames()
            color_frame = frames.get_color_frame()

            if not color_frame:
                return None

            # Convert to numpy array (BGR format)
            color_image = np.asanyarray(color_frame.get_data())
            return color_image

        except Exception as e:
            print(f"Error getting frame: {e}")
            return None

    def detect_aruco(self, image):
        """Detect ArUco markers in image"""
        if image is None:
            return None

        marker_poses = self.aruco_estimator.detect_and_estimate_pose(image, self.intrinsics)
        if marker_poses:
            return self.aruco_estimator.visualize_markers(image, self.intrinsics, marker_poses)
        return image

    def detect_chessboard(self, image):
        """Detect regular chessboard in image"""
        if image is None:
            return None

        board_pose = self.aruco_estimator.detect_and_estimate_chessboard_pose(
            image, self.intrinsics, self.chessboard_size, self.square_size
        )
        if board_pose:
            return self.aruco_estimator.visualize_chessboard(
                image, self.intrinsics, board_pose, self.chessboard_size, self.square_size
            )
        return image

    def detect_charuco(self, image):
        """Detect ChArUco board in image"""
        if image is None:
            return None

        board_pose = self.aruco_estimator.detect_and_estimate_charuco_pose(
            image, self.intrinsics, self.charuco_board_config
        )
        if board_pose:
            return self.aruco_estimator.visualize_charuco(
                image, self.intrinsics, board_pose, self.charuco_board_config
            )
        return image


class RobotCameraApp(QtWidgets.QMainWindow):
    """Main Application Window"""

    def __init__(self):
        super().__init__()

        # Load UI file
        ui_file = os.path.join(os.path.dirname(__file__), 'ui', 'robot_camera_ui.ui')
        uic.loadUi(ui_file, self)

        # Initialize cameras
        self.d435_camera = None

        # Initialize robot controller
        self.robot = None

        # Camera timers
        self.d435_timer = QTimer()
        self.d435_timer.timeout.connect(self.update_d435_frame)

        # Connect signals
        self.connect_signals()

        # Set initial states
        self.d435StatusLabel.setText("Status: Disconnected")
        self.d435StatusLabel.setStyleSheet("color: #e74c3c; padding: 5px;")

        print("🚀 Robot Camera Application initialized")

    def connect_signals(self):
        """Connect UI signals to slots"""
        # D435 Camera controls
        self.startD435Button.clicked.connect(self.start_d435_camera)
        self.stopD435Button.clicked.connect(self.stop_d435_camera)

        # Marker type selection
        self.markerTypeComboBox.currentIndexChanged.connect(self.on_marker_type_changed)

        # Robot controls (placeholders for now)
        self.connectRobotButton.clicked.connect(self.connect_robot)
        self.readTcpButton.clicked.connect(self.read_tcp_position)
        self.useCurrentButton.clicked.connect(self.use_current_position)
        self.moveTcpButton.clicked.connect(self.move_to_target)

        # ArduCam controls (disabled for now)
        self.startArducamButton.setEnabled(False)
        self.stopArducamButton.setEnabled(False)

        # Snapshot
        self.snapshotButton.clicked.connect(self.capture_snapshot)

    @pyqtSlot()
    def start_d435_camera(self):
        """Start D435 camera"""
        try:
            if self.d435_camera is None:
                self.d435_camera = D435CameraController()

                # Apply currently selected marker type
                current_index = self.markerTypeComboBox.currentIndex()
                self.on_marker_type_changed(current_index)

            if self.d435_camera.start():
                self.d435_timer.start(30)  # 30ms = ~33 FPS

                self.startD435Button.setEnabled(False)
                self.stopD435Button.setEnabled(True)
                self.snapshotButton.setEnabled(True)

                self.d435StatusLabel.setText("Status: Connected")
                self.d435StatusLabel.setStyleSheet("color: #2ecc71; padding: 5px;")

                print("✅ D435 camera started successfully")
            else:
                self.show_error("Failed to start D435 camera")

        except Exception as e:
            self.show_error(f"Error starting D435: {str(e)}")

    @pyqtSlot()
    def stop_d435_camera(self):
        """Stop D435 camera"""
        if self.d435_camera is not None:
            self.d435_timer.stop()
            self.d435_camera.stop()

            self.startD435Button.setEnabled(True)
            self.stopD435Button.setEnabled(False)
            self.snapshotButton.setEnabled(False)

            self.d435StatusLabel.setText("Status: Disconnected")
            self.d435StatusLabel.setStyleSheet("color: #e74c3c; padding: 5px;")

            # Clear display
            self.d435CameraLabel.setText("D435 Camera Not Started")

            print("D435 camera stopped")

    @pyqtSlot(int)
    def on_marker_type_changed(self, index):
        """Handle marker type selection change"""
        # Mapping from ComboBox index to dictionary type
        marker_types = {
            0: ('AprilTag 36h11', cv2.aruco.DICT_APRILTAG_36h11),
            1: ('AprilTag 16h5', cv2.aruco.DICT_APRILTAG_16h5),
            2: ('AprilTag 25h9', cv2.aruco.DICT_APRILTAG_25h9),
            3: ('AprilTag 36h10', cv2.aruco.DICT_APRILTAG_36h10),
            4: ('ArUco 6x6', cv2.aruco.DICT_6X6_250),
            5: ('ArUco 4x4', cv2.aruco.DICT_4X4_50),
            6: ('ArUco 5x5', cv2.aruco.DICT_5X5_100),
            7: ('ArUco 7x7', cv2.aruco.DICT_7X7_50),
        }

        if index in marker_types:
            marker_name, marker_dict = marker_types[index]

            # Update D435 camera marker type if initialized
            if self.d435_camera is not None:
                self.d435_camera.update_marker_type(marker_dict)
                print(f"✅ Marker type changed to: {marker_name}")
            else:
                print(f"⚠️  Marker type will be set to {marker_name} when camera starts")

    @pyqtSlot()
    def update_d435_frame(self):
        """Update D435 camera display"""
        if self.d435_camera is None or not self.d435_camera.is_running():
            return

        # Get frame
        frame = self.d435_camera.get_frame()
        if frame is None:
            return

        # Apply detection based on checkboxes
        display_frame = frame.copy()

        # ArUco detection
        if self.d435EnableArucoCheckBox.isChecked():
            display_frame = self.d435_camera.detect_aruco(display_frame)

        # Chessboard detection (regular chessboard)
        if self.d435EnableCharucoCheckBox.isChecked():
            display_frame = self.d435_camera.detect_chessboard(display_frame)

        # Convert to QPixmap and display
        self.display_image(display_frame, self.d435CameraLabel)

    def display_image(self, cv_image, label):
        """Convert OpenCV image to QPixmap and display in label"""
        if cv_image is None:
            return

        # Convert BGR to RGB
        rgb_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)

        # Get image dimensions
        h, w, ch = rgb_image.shape
        bytes_per_line = ch * w

        # Convert to QImage
        qt_image = QImage(rgb_image.data, w, h, bytes_per_line, QImage.Format_RGB888)

        # Scale to fit label while maintaining aspect ratio
        pixmap = QPixmap.fromImage(qt_image)
        scaled_pixmap = pixmap.scaled(
            label.size(),
            QtCore.Qt.KeepAspectRatio,
            QtCore.Qt.SmoothTransformation
        )

        label.setPixmap(scaled_pixmap)

    @pyqtSlot()
    def capture_snapshot(self):
        """Capture snapshot from active cameras"""
        timestamp = QtCore.QDateTime.currentDateTime().toString("yyyyMMdd_HHmmss")
        snapshot_dir = "snapshots"

        # Create directory if it doesn't exist
        os.makedirs(snapshot_dir, exist_ok=True)

        # Capture D435 snapshot
        if self.d435_camera and self.d435_camera.is_running():
            frame = self.d435_camera.get_frame()
            if frame is not None:
                filename = os.path.join(snapshot_dir, f"d435_{timestamp}.jpg")
                cv2.imwrite(filename, frame)
                print(f"📸 D435 snapshot saved: {filename}")

        self.show_info(f"Snapshot saved to {snapshot_dir}/")

    # ==================== Robot Control ====================

    @pyqtSlot()
    def connect_robot(self):
        """Connect to robot"""
        try:
            if self.robot is None:
                # Create robot controller with default settings
                # You can change IP and port here if needed
                self.robot = RobotTCPController(ip="192.168.0.29", port=1502)

            if self.robot.connect():
                self.connectRobotButton.setEnabled(False)
                self.readTcpButton.setEnabled(True)
                self.useCurrentButton.setEnabled(True)
                self.moveTcpButton.setEnabled(True)

                # Enable target input fields
                self.targetXInput.setEnabled(True)
                self.targetYInput.setEnabled(True)
                self.targetZInput.setEnabled(True)
                self.targetRxInput.setEnabled(True)
                self.targetRyInput.setEnabled(True)
                self.targetRzInput.setEnabled(True)

                print("✅ Robot connected successfully")
                self.show_info("Robot connected successfully!")

                # Auto-read initial TCP position
                self.read_tcp_position()
            else:
                self.show_error("Failed to connect to robot")

        except Exception as e:
            self.show_error(f"Error connecting to robot: {str(e)}")

    @pyqtSlot()
    def read_tcp_position(self):
        """Read robot TCP position"""
        if self.robot is None or not self.robot.is_connected():
            self.show_error("Robot not connected")
            return

        try:
            tcp_pos = self.robot.read_tcp_position()

            if tcp_pos is not None:
                # Update current position labels
                self.xValue.setText(f"{tcp_pos['x']:.3f}")
                self.yValue.setText(f"{tcp_pos['y']:.3f}")
                self.zValue.setText(f"{tcp_pos['z']:.3f}")
                self.rollValue.setText(f"{tcp_pos['rx']:.3f}")
                self.pitchValue.setText(f"{tcp_pos['ry']:.3f}")
                self.yawValue.setText(f"{tcp_pos['rz']:.3f}")

                print(f"📍 TCP Position: X={tcp_pos['x']:.3f}, Y={tcp_pos['y']:.3f}, Z={tcp_pos['z']:.3f}")
                print(f"   Orientation: Rx={tcp_pos['rx']:.3f}, Ry={tcp_pos['ry']:.3f}, Rz={tcp_pos['rz']:.3f}")
            else:
                self.show_error("Failed to read TCP position")

        except Exception as e:
            self.show_error(f"Error reading TCP position: {str(e)}")

    @pyqtSlot()
    def use_current_position(self):
        """Use current TCP position as target"""
        if self.robot is None or not self.robot.is_connected():
            self.show_error("Robot not connected")
            return

        try:
            # Read current position first
            tcp_pos = self.robot.read_tcp_position()

            if tcp_pos is not None:
                # Set target inputs to current position
                self.targetXInput.setText(f"{tcp_pos['x']:.3f}")
                self.targetYInput.setText(f"{tcp_pos['y']:.3f}")
                self.targetZInput.setText(f"{tcp_pos['z']:.3f}")
                self.targetRxInput.setText(f"{tcp_pos['rx']:.3f}")
                self.targetRyInput.setText(f"{tcp_pos['ry']:.3f}")
                self.targetRzInput.setText(f"{tcp_pos['rz']:.3f}")

                print("✅ Current position copied to target")
            else:
                self.show_error("Failed to read current position")

        except Exception as e:
            self.show_error(f"Error: {str(e)}")

    @pyqtSlot()
    def move_to_target(self):
        """Move robot to target position"""
        if self.robot is None or not self.robot.is_connected():
            self.show_error("Robot not connected")
            return

        try:
            # Get target values from input fields
            target_x = float(self.targetXInput.text())
            target_y = float(self.targetYInput.text())
            target_z = float(self.targetZInput.text())
            target_rx = float(self.targetRxInput.text())
            target_ry = float(self.targetRyInput.text())
            target_rz = float(self.targetRzInput.text())

            # Confirm with user
            msg = f"Move robot to:\n\n"
            msg += f"Position: X={target_x:.3f}, Y={target_y:.3f}, Z={target_z:.3f}\n"
            msg += f"Orientation: Rx={target_rx:.3f}, Ry={target_ry:.3f}, Rz={target_rz:.3f}\n\n"
            msg += "Are you sure?"

            reply = QtWidgets.QMessageBox.question(
                self, 'Confirm Movement', msg,
                QtWidgets.QMessageBox.Yes | QtWidgets.QMessageBox.No,
                QtWidgets.QMessageBox.No
            )

            if reply == QtWidgets.QMessageBox.Yes:
                # Move robot to target pose
                success = self.robot.move_tcp_to(
                    target_x, target_y, target_z,
                    target_rx, target_ry, target_rz
                )

                if success:
                    print(f"✅ Target position sent to robot")
                    self.show_info("Target position sent successfully!")

                    # Read back the position after a short delay
                    QtCore.QTimer.singleShot(1000, self.read_tcp_position)
                else:
                    self.show_error("Failed to send target position")

        except ValueError:
            self.show_error("Invalid input values. Please enter valid numbers.")
        except Exception as e:
            self.show_error(f"Error moving robot: {str(e)}")

    # ==================== Utility Methods ====================

    def show_error(self, message):
        """Show error message dialog"""
        QtWidgets.QMessageBox.critical(self, "Error", message)

    def show_info(self, message):
        """Show info message dialog"""
        QtWidgets.QMessageBox.information(self, "Information", message)

    def closeEvent(self, event):
        """Handle window close event"""
        # Stop cameras
        if self.d435_camera and self.d435_camera.is_running():
            self.stop_d435_camera()

        # Disconnect robot
        if self.robot and self.robot.is_connected():
            self.robot.disconnect()
            print("🔌 Robot disconnected")

        event.accept()


def main():
    """Main application entry point"""
    app = QtWidgets.QApplication(sys.argv)

    # Set application style
    app.setStyle('Fusion')

    # Create and show main window
    window = RobotCameraApp()
    window.show()

    sys.exit(app.exec_())


if __name__ == '__main__':
    main()
