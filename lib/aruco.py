import cv2
import numpy as np
from cv2 import aruco

class ArucoCameraPoseEstimator:
    """
    Estimate camera pose relative to ArUco markers and ChArUco boards
    """
    
    def __init__(self, marker_size_meters=0.0423334, dictionary_type=aruco.DICT_6X6_250):
        """
        Initialize ArUco pose estimator

        Args:
            marker_size_meters: Physical size of ArUco marker in meters (e.g., 0.05 for 5cm marker)
            dictionary_type: ArUco dictionary type (e.g., aruco.DICT_6X6_250)
        """
        self.marker_size = marker_size_meters
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(dictionary_type)

        # Check OpenCV version and use appropriate API
        opencv_version = tuple(map(int, cv2.__version__.split('.')[:2]))

        # Check if using AprilTag
        is_apriltag = 'APRILTAG' in str(dictionary_type)

        if opencv_version >= (4, 7):
            # OpenCV 4.7.0+ uses ArucoDetector
            self.detector = aruco.ArucoDetector(self.aruco_dict)

            # Optimize detection parameters for better detection
            params = aruco.DetectorParameters()
            # Adaptive thresholding - wider range for varied lighting
            params.adaptiveThreshWinSizeMin = 3
            params.adaptiveThreshWinSizeMax = 23
            params.adaptiveThreshWinSizeStep = 10
            params.adaptiveThreshConstant = 7
            # Corner refinement - use AprilTag method for AprilTag markers
            if is_apriltag:
                params.cornerRefinementMethod = aruco.CORNER_REFINE_APRILTAG
            else:
                params.cornerRefinementMethod = aruco.CORNER_REFINE_SUBPIX
            params.cornerRefinementWinSize = 5
            params.cornerRefinementMaxIterations = 30
            params.cornerRefinementMinAccuracy = 0.1
            # Detection parameters - more permissive for small markers
            params.minMarkerPerimeterRate = 0.01
            params.maxMarkerPerimeterRate = 4.0
            params.polygonalApproxAccuracyRate = 0.05
            params.minCornerDistanceRate = 0.01
            params.minDistanceToBorder = 1
            params.minMarkerDistanceRate = 0.01
            # Perspective removal
            params.perspectiveRemovePixelPerCell = 4
            params.perspectiveRemoveIgnoredMarginPerCell = 0.13
            # Other parameters
            params.minOtsuStdDev = 5.0
            params.markerBorderBits = 1
            self.detector.setDetectorParameters(params)
            self.use_legacy_api = False
        else:
            # OpenCV < 4.7.0 uses legacy API
            self.detector = None
            params = aruco.DetectorParameters_create()
            # Adaptive thresholding - wider range for varied lighting
            params.adaptiveThreshWinSizeMin = 3
            params.adaptiveThreshWinSizeMax = 23
            params.adaptiveThreshWinSizeStep = 10
            params.adaptiveThreshConstant = 7
            # Corner refinement - use AprilTag method for AprilTag markers
            if is_apriltag:
                params.cornerRefinementMethod = aruco.CORNER_REFINE_APRILTAG
            else:
                params.cornerRefinementMethod = aruco.CORNER_REFINE_SUBPIX
            params.cornerRefinementWinSize = 5
            params.cornerRefinementMaxIterations = 30
            params.cornerRefinementMinAccuracy = 0.1
            # Detection parameters - more permissive for small markers
            params.minMarkerPerimeterRate = 0.01
            params.maxMarkerPerimeterRate = 4.0
            params.polygonalApproxAccuracyRate = 0.05
            params.minCornerDistanceRate = 0.01
            params.minDistanceToBorder = 1
            params.minMarkerDistanceRate = 0.01
            # Perspective removal
            params.perspectiveRemovePixelPerCell = 4
            params.perspectiveRemoveIgnoredMarginPerCell = 0.13
            # Other parameters
            params.minOtsuStdDev = 5.0
            params.markerBorderBits = 1
            self.detector_params = params
            self.use_legacy_api = True
        
    def detect_and_estimate_pose(self, image, intrinsics):
        """
        Detect ArUco markers and estimate camera pose relative to each marker
        
        Args:
            image: Input image (grayscale or color)
            intrinsics: Camera intrinsics object with fx, fy, ppx, ppy, coeffs
            
        Returns:
            List of dictionaries with marker info:
            {
                'id': marker_id,
                'corners': marker_corners,
                'rvec': rotation_vector,
                'tvec': translation_vector,
                'rotation_matrix': 3x3_rotation_matrix,
                'camera_position': camera_position_in_marker_frame,
                'camera_rotation': camera_rotation_in_marker_frame
            }
        """
        camera_matrix = np.array([
            [intrinsics.fx, 0, intrinsics.ppx],
            [0, intrinsics.fy, intrinsics.ppy],
            [0, 0, 1]
        ], dtype=np.float32)

        dist_coeffs = np.array(intrinsics.coeffs)
        
        # Convert to grayscale if needed
        if len(image.shape) == 3:
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        else:
            gray = image

        # Detect ArUco markers
        if self.use_legacy_api:
            corners, ids, rejected = aruco.detectMarkers(gray, self.aruco_dict, parameters=self.detector_params)
        else:
            corners, ids, rejected = self.detector.detectMarkers(gray)
        
        marker_poses = []

        if ids is not None:
            # Skip refineDetectedMarkers as it requires a Board object
            # Individual markers work fine without refinement

            # Estimate pose for each detected marker
            rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(
                corners, self.marker_size, camera_matrix, dist_coeffs
            )
            
            for i, marker_id in enumerate(ids.flatten()):
                rvec = rvecs[i][0]
                tvec = tvecs[i][0]
                
                # Convert rotation vector to rotation matrix
                rotation_matrix, _ = cv2.Rodrigues(rvec)
                
                # Camera pose in marker coordinate frame
                camera_rotation_in_marker = rotation_matrix.T
                camera_position_in_marker = -camera_rotation_in_marker @ tvec
                
                marker_info = {
                    'id': int(marker_id),
                    'corners': corners[i],
                    'rvec': rvec,
                    'tvec': tvec,
                    'rotation_matrix': rotation_matrix,
                    'camera_position': camera_position_in_marker,
                    'camera_rotation': camera_rotation_in_marker,
                    'marker_position_in_camera': tvec,
                    'marker_rotation_in_camera': rotation_matrix
                }
                
                marker_poses.append(marker_info)
                
                print(f"Marker {marker_id}:")
                print(f"  Distance: {np.linalg.norm(tvec):.3f}m")
                print(f"  Camera position in marker frame: {camera_position_in_marker}")
                print(f"  Camera rotation angles: {self._rotation_matrix_to_euler(camera_rotation_in_marker)}")
        
        return marker_poses

    def detect_and_estimate_chessboard_pose(self, image, intrinsics, chessboard_size=(10, 7), square_size=0.05):
        """
        Detect regular chessboard and estimate camera pose

        Args:
            image: Input image (grayscale or color)
            intrinsics: Camera intrinsics object
            chessboard_size: (width, height) number of internal corners (e.g., (10, 7) for 11x8 grid)
            square_size: Physical size of each square in meters

        Returns:
            Dictionary with board pose information or None if not detected
        """
        camera_matrix = np.array([
            [intrinsics.fx, 0, intrinsics.ppx],
            [0, intrinsics.fy, intrinsics.ppy],
            [0, 0, 1]
        ], dtype=np.float32)

        dist_coeffs = np.array(intrinsics.coeffs)

        # Convert to grayscale if needed
        if len(image.shape) == 3:
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        else:
            gray = image

        # Find chessboard corners
        flags = cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE + cv2.CALIB_CB_FAST_CHECK
        ret, corners = cv2.findChessboardCorners(gray, chessboard_size, flags)

        if ret:
            # Refine corner locations for sub-pixel accuracy
            criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
            corners_refined = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)

            # Prepare object points (3D points in real world space)
            objp = np.zeros((chessboard_size[0] * chessboard_size[1], 3), np.float32)
            objp[:, :2] = np.mgrid[0:chessboard_size[0], 0:chessboard_size[1]].T.reshape(-1, 2)
            objp *= square_size

            # Estimate pose
            success, rvec, tvec = cv2.solvePnP(objp, corners_refined, camera_matrix, dist_coeffs)

            if success:
                # Convert rotation vector to rotation matrix
                rotation_matrix, _ = cv2.Rodrigues(rvec)
                tvec = tvec.flatten()

                # Camera pose in board coordinate frame
                camera_rotation_in_board = rotation_matrix.T
                camera_position_in_board = -camera_rotation_in_board @ tvec

                board_info = {
                    'type': 'chessboard',
                    'detected_corners': len(corners_refined),
                    'chessboard_size': chessboard_size,
                    'rvec': rvec,
                    'tvec': tvec,
                    'rotation_matrix': rotation_matrix,
                    'camera_position': camera_position_in_board,
                    'camera_rotation': camera_rotation_in_board,
                    'board_position_in_camera': tvec,
                    'board_rotation_in_camera': rotation_matrix,
                    'corners': corners_refined
                }

                distance = np.linalg.norm(tvec)
                euler_angles = self._rotation_matrix_to_euler(camera_rotation_in_board)

                print(f"Chessboard Detected:")
                print(f"  Size: {chessboard_size[0]}x{chessboard_size[1]} corners")
                print(f"  Distance: {distance:.3f}m")
                print(f"  Camera position in board frame: {camera_position_in_board}")
                print(f"  Camera rotation angles: {euler_angles}")

                return board_info
            else:
                print("Could not estimate chessboard pose")
        else:
            print(f"Chessboard corners not found (looking for {chessboard_size[0]}x{chessboard_size[1]})")

        return None

    def detect_and_estimate_charuco_pose(self, image, intrinsics, board_config):
        """
        Detect ChArUco board and estimate camera pose
        
        Args:
            image: Input image (grayscale or color)
            intrinsics: Camera intrinsics object
            board_config: Dictionary with ChArUco board configuration:
                {
                    'grid_size': (width, height),  # e.g., (8, 6)
                    'square_size': square_size_meters,  # e.g., 0.05 for 5cm
                    'marker_size': marker_size_meters,  # e.g., 0.035 for 3.5cm
                }
                
        Returns:
            Dictionary with board pose information or None if not detected
        """
        camera_matrix = np.array([
            [intrinsics.fx, 0, intrinsics.ppx],
            [0, intrinsics.fy, intrinsics.ppy],
            [0, 0, 1]
        ], dtype=np.float32)

        dist_coeffs = np.array(intrinsics.coeffs)
        
        # Create ChArUco board
        board = aruco.CharucoBoard(
            board_config['grid_size'],
            board_config['square_size'], 
            board_config['marker_size'],
            self.aruco_dict
        )
        
        # Convert to grayscale if needed
        if len(image.shape) == 3:
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        else:
            gray = image

        # Step 1: Detect ArUco markers
        if self.use_legacy_api:
            corners, ids, rejected = aruco.detectMarkers(gray, self.aruco_dict, parameters=self.detector_params)
        else:
            corners, ids, rejected = self.detector.detectMarkers(gray)
        
        if ids is not None and len(ids) > 0:
            print(f"Found {len(ids)} ArUco markers in ChArUco board")
                    
            cv2.aruco.refineDetectedMarkers(
                image=gray,
                board=board,
                detectedCorners=corners,
                detectedIds=ids,
                rejectedCorners=rejected,
                cameraMatrix=camera_matrix,
                distCoeffs=dist_coeffs
            )

            # Step 2: Interpolate ChArUco corners using detected ArUco markers
            charuco_retval, charuco_corners, charuco_ids = aruco.interpolateCornersCharuco(
                corners, ids, gray, board
            )
            
            if charuco_retval > 4:  # Need at least 4 corners for pose estimation
                print(f"Interpolated {charuco_retval} ChArUco corners")
                
                # Step 3: Estimate pose using ChArUco corners
                success, rvec, tvec = aruco.estimatePoseCharucoBoard(
                    charuco_corners, charuco_ids, board, camera_matrix, dist_coeffs, None, None
                )
                
                if success:
                    # Convert rotation vector to rotation matrix
                    rotation_matrix, _ = cv2.Rodrigues(rvec)
                    tvec = tvec.flatten()

                    transform_cv = np.eye(4)
                    transform_cv[:3, :3] = rotation_matrix.copy()
                    transform_cv[:3, 3] = tvec.flatten().copy()

                    # Camera pose in board coordinate frame

                    camera_rotation_in_board = rotation_matrix.T
                    camera_position_in_board = -camera_rotation_in_board @ tvec

                    board_info = {
                        'type': 'charuco',
                        'detected_corners': charuco_retval,
                        'aruco_markers_found': len(ids),
                        'rvec': rvec,
                        'tvec': tvec,
                        'rotation_matrix': rotation_matrix,
                        'camera_position': camera_position_in_board,
                        'camera_rotation': camera_rotation_in_board,
                        'board_position_in_camera': tvec,
                        'board_rotation_in_camera': rotation_matrix,
                        'charuco_corners': charuco_corners,
                        'charuco_ids': charuco_ids,
                        'aruco_corners': corners,
                        'aruco_ids': ids
                    }
                    
                    distance = np.linalg.norm(tvec)
                    euler_angles = self._rotation_matrix_to_euler(camera_rotation_in_board)
                    
                    print(f"ChArUco Board Pose:")
                    print(f"  Distance: {distance:.3f}m")
                    print(f"  Camera position in board frame: {camera_position_in_board}")
                    print(f"  Camera rotation angles: {euler_angles}")
                    
                    return board_info
                else:
                    print("Could not estimate ChArUco board pose")
            else:
                print(f"Not enough ChArUco corners detected ({charuco_retval}/4 minimum)")
        else:
            print("No ArUco markers found - cannot detect ChArUco board")
        
        return None

    def visualize_chessboard(self, image, intrinsics, board_pose, chessboard_size=(10, 7), square_size=0.05,
                            similarity_vectors=None, p_centroid=None, c_centroid=None, show_info=True, axis_length=None):
        """
        Visualize detected chessboard with pose information

        Args:
            image: Image to draw on
            intrinsics: Camera intrinsics object
            board_pose: Board pose from detect_and_estimate_chessboard_pose
            chessboard_size: (width, height) number of internal corners
            square_size: Physical size of each square in meters
            show_info: Whether to display pose information
            axis_length: Length of coordinate axes to draw
        """
        if board_pose is None:
            return image

        camera_matrix = np.array([
            [intrinsics.fx, 0, intrinsics.ppx],
            [0, intrinsics.fy, intrinsics.ppy],
            [0, 0, 1]
        ], dtype=np.float32)

        dist_coeffs = np.array(intrinsics.coeffs)

        if axis_length is None:
            axis_length = square_size * 3

        vis_image = image.copy()

        # 1. Draw detected chessboard corners
        cv2.drawChessboardCorners(vis_image, chessboard_size, board_pose['corners'], True)

        # 2. Draw coordinate axes at board origin
        cv2.drawFrameAxes(vis_image, camera_matrix, dist_coeffs,
                         board_pose['rvec'], board_pose['tvec'], axis_length)

        # 3. Draw information text
        if show_info:
            distance = np.linalg.norm(board_pose['tvec'])
            camera_pos = board_pose['camera_position']
            euler_angles = self._rotation_matrix_to_euler(board_pose['camera_rotation'])

            text_lines = [
                f"Chessboard Detected",
                f"Size: {chessboard_size[0]}x{chessboard_size[1]} corners",
                f"Corners detected: {board_pose['detected_corners']}",
                f"Distance: {distance:.3f}m",
                f"Cam Pos: [{camera_pos[0]:.3f}, {camera_pos[1]:.3f}, {camera_pos[2]:.3f}]",
                f"Cam Rot: [{euler_angles[0]:.1f}, {euler_angles[1]:.1f}, {euler_angles[2]:.1f}]deg",
                f"Similarity: {similarity_vectors if similarity_vectors is not None else 'N/A'}",
                f"PC Distance: {p_centroid if p_centroid is not None else 'N/A'}",
                f"C Distance: {c_centroid if c_centroid is not None else 'N/A'}"
            ]

            # Draw text background
            text_height = 20
            max_width = max([cv2.getTextSize(line, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)[0][0]
                           for line in text_lines])

            bg_top_left = (10, 10)
            bg_bottom_right = (max_width + 20, len(text_lines) * text_height + 20)
            cv2.rectangle(vis_image, bg_top_left, bg_bottom_right, (0, 0, 0), -1)
            cv2.rectangle(vis_image, bg_top_left, bg_bottom_right, (255, 255, 255), 2)

            # Draw text
            for i, line in enumerate(text_lines):
                cv2.putText(vis_image, line, (15, 35 + i * text_height),
                          cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

        return vis_image

    def visualize_charuco(self, image, intrinsics, board_pose, board_config, similarity_vectors=None, p_centroid=None, c_centroid=None,
                         show_info=True, axis_length=None):
        """
        Visualize detected ChArUco board with pose information
        
        Args:
            image: Image to draw on
            intrinsics: Camera intrinsics object
            board_pose: Board pose from detect_and_estimate_charuco_pose
            board_config: ChArUco board configuration
            show_info: Whether to display pose information
            axis_length: Length of coordinate axes to draw
        """
        if board_pose is None:
            return image
            
        camera_matrix = np.array([
            [intrinsics.fx, 0, intrinsics.ppx],
            [0, intrinsics.fy, intrinsics.ppy],
            [0, 0, 1]
        ], dtype=np.float32)

        dist_coeffs = np.array(intrinsics.coeffs)
        
        if axis_length is None:
            axis_length = board_config['square_size'] * 3
        
        vis_image = image.copy()
        
        # 1. Draw detected ArUco markers (green)
        if board_pose['aruco_corners'] is not None:
            aruco.drawDetectedMarkers(vis_image, board_pose['aruco_corners'], 
                                    board_pose['aruco_ids'], (0, 255, 0))
        
        # 2. Draw interpolated ChArUco corners (red) - these are more accurate
        if board_pose['charuco_corners'] is not None:
            aruco.drawDetectedCornersCharuco(vis_image, board_pose['charuco_corners'], 
                                           board_pose['charuco_ids'], (0, 0, 255))
        
        # 3. Draw coordinate axes at board origin
        cv2.drawFrameAxes(vis_image, camera_matrix, dist_coeffs, 
                         board_pose['rvec'], board_pose['tvec'], axis_length)
        
        # 4. Draw information text
        if show_info:
            distance = np.linalg.norm(board_pose['tvec'])
            camera_pos = board_pose['camera_position']
            euler_angles = self._rotation_matrix_to_euler(board_pose['camera_rotation'])
            
            text_lines = [
                f"ChArUco Board Detected",
                f"ArUco markers: {board_pose['aruco_markers_found']}",
                f"ChArUco corners: {board_pose['detected_corners']}",
                f"Distance: {distance:.3f}m",
                f"Cam Pos: [{camera_pos[0]:.3f}, {camera_pos[1]:.3f}, {camera_pos[2]:.3f}]",
                f"Cam Rot: [{euler_angles[0]:.1f}, {euler_angles[1]:.1f}, {euler_angles[2]:.1f}]deg",
                f"Similarity: {similarity_vectors if similarity_vectors is not None else 'Nan'}",
                f"PC Distance: {p_centroid if p_centroid is not None else 'Nan'}",
                f"C Distance: {c_centroid if c_centroid is not None else 'Nan'}"
            ]
            
            # Draw text background
            text_height = 20
            max_width = max([cv2.getTextSize(line, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)[0][0] 
                           for line in text_lines])
            
            bg_top_left = (10, 10)
            bg_bottom_right = (max_width + 20, len(text_lines) * text_height + 20)
            cv2.rectangle(vis_image, bg_top_left, bg_bottom_right, (0, 0, 0), -1)
            cv2.rectangle(vis_image, bg_top_left, bg_bottom_right, (255, 255, 255), 2)
            
            # Draw text
            for i, line in enumerate(text_lines):
                cv2.putText(vis_image, line, (15, 35 + i * text_height), 
                          cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        
        return vis_image

    def visualize_markers(self, image, intrinsics, marker_poses, 
                         show_info=True, axis_length=None):
        """
        Comprehensive marker visualization with pose information
        (Original ArUco method - unchanged)
        """
        camera_matrix = np.array([
            [intrinsics.fx, 0, intrinsics.ppx],
            [0, intrinsics.fy, intrinsics.ppy],
            [0, 0, 1]
        ], dtype=np.float32)

        dist_coeffs = np.array(intrinsics.coeffs)
        
        if not marker_poses:
            return image
            
        if axis_length is None:
            axis_length = self.marker_size
        
        vis_image = image.copy()
        
        for i, pose in enumerate(marker_poses):
            marker_id = pose['id']
            corners = pose['corners']
            rvec = pose['rvec']
            tvec = pose['tvec']
            
            # Draw marker boundaries and axes
            aruco.drawDetectedMarkers(vis_image, [corners], np.array([[marker_id]]))
            cv2.drawFrameAxes(vis_image, camera_matrix, dist_coeffs, rvec, tvec, axis_length)
            
            # Draw marker center
            center = np.mean(corners[0], axis=0).astype(int)
            cv2.circle(vis_image, tuple(center), 5, (0, 255, 255), -1)
            
            # Draw information text
            if show_info:
                distance = np.linalg.norm(tvec)
                camera_pos = pose['camera_position']
                euler_angles = self._rotation_matrix_to_euler(pose['camera_rotation'])
                
                text_pos = (center[0] + 20, center[1] - 60 + i * 80)
                text_lines = [
                    f"ID: {marker_id}",
                    f"Dist: {distance:.3f}m",
                    f"Cam Pos: [{camera_pos[0]:.3f}, {camera_pos[1]:.3f}, {camera_pos[2]:.3f}]",
                    f"Cam Rot: [{euler_angles[0]:.1f}, {euler_angles[1]:.1f}, {euler_angles[2]:.1f}]deg"
                ]
                
                self._draw_text_with_background(vis_image, text_lines, text_pos)
        
        return vis_image

    def _draw_text_with_background(self, image, text_lines, pos):
        """Helper function to draw text with background"""
        text_height = 20
        max_text_width = max([cv2.getTextSize(line, cv2.FONT_HERSHEY_SIMPLEX, 0.4, 1)[0][0] 
                            for line in text_lines])
        
        # Background rectangle
        bg_top_left = (pos[0] - 5, pos[1] - 15)
        bg_bottom_right = (pos[0] + max_text_width + 10, 
                         pos[1] + len(text_lines) * text_height + 5)
        cv2.rectangle(image, bg_top_left, bg_bottom_right, (0, 0, 0), -1)
        cv2.rectangle(image, bg_top_left, bg_bottom_right, (255, 255, 255), 1)
        
        # Text lines
        for j, line in enumerate(text_lines):
            line_pos = (pos[0], pos[1] + j * text_height)
            cv2.putText(image, line, line_pos, cv2.FONT_HERSHEY_SIMPLEX, 
                      0.4, (255, 255, 255), 1)

    def _rotation_matrix_to_euler(self, R):
        """Convert rotation matrix to Euler angles (roll, pitch, yaw) in degrees"""
        sy = np.sqrt(R[0,0] * R[0,0] + R[1,0] * R[1,0])
        singular = sy < 1e-6
        
        if not singular:
            x = np.arctan2(R[2,1], R[2,2])  # roll
            y = np.arctan2(-R[2,0], sy)     # pitch  
            z = np.arctan2(R[1,0], R[0,0])  # yaw
        else:
            x = np.arctan2(-R[1,2], R[1,1]) # roll
            y = np.arctan2(-R[2,0], sy)     # pitch
            z = 0                           # yaw
        
        return np.degrees([x, y, z])
    
    def draw_pose_axes(self, image, camera_matrix, dist_coeffs, marker_poses, axis_length=None):
        """
        Draw coordinate axes on detected markers
        """
        if axis_length is None:
            axis_length = self.marker_size
            
        for pose in marker_poses:
            cv2.drawFrameAxes(image, camera_matrix, dist_coeffs, 
                          pose['rvec'], pose['tvec'], axis_length)
            aruco.drawDetectedMarkers(image, [pose['corners']], 
                                    np.array([[pose['id']]]))
    
    def get_world_coordinate_system(self, marker_poses, reference_marker_id=0):
        """
        Establish world coordinate system using a reference marker
        """
        reference_pose = None
        for pose in marker_poses:
            if pose['id'] == reference_marker_id:
                reference_pose = pose
                break
        
        if reference_pose is None:
            print(f"Reference marker {reference_marker_id} not found")
            return None
        
        return {
            'camera_world_position': reference_pose['camera_position'],
            'camera_world_rotation': reference_pose['camera_rotation'],
            'reference_marker_id': reference_marker_id
        }

# Example usage
def example_usage():
    """
    Example of how to use both ArUco and ChArUco detection
    """
    
    # Initialize estimator
    estimator = ArucoCameraPoseEstimator(
        marker_size_meters=0.035,  # 3.5cm markers
        dictionary_type=aruco.DICT_6X6_250
    )
    
    # Mock camera intrinsics (replace with your actual calibration)
    class MockIntrinsics:
        def __init__(self):
            self.fx = 800.0
            self.fy = 800.0
            self.ppx = 320.0
            self.ppy = 240.0
            self.coeffs = [0.1, -0.2, 0, 0, 0]  # k1, k2, p1, p2, k3
    
    intrinsics = MockIntrinsics()
    
    # Example 1: Detect individual ArUco markers
    print("=== DETECTING ARUCO MARKERS ===")
    # image = cv2.imread("your_aruco_image.jpg")
    # marker_poses = estimator.detect_and_estimate_pose(image, intrinsics)
    # vis_image = estimator.visualize_markers(image, intrinsics, marker_poses)
    
    # Example 2: Detect ChArUco board
    print("\n=== DETECTING CHARUCO BOARD ===")
    
    # ChArUco board configuration matching your grid creation
    board_config = {
        'grid_size': (8, 6),           # 8x6 grid
        'square_size': 0.05,           # 5cm squares  
        'marker_size': 0.035,          # 3.5cm markers
    }
    
    # image = cv2.imread("your_charuco_image.jpg") 
    # board_pose = estimator.detect_and_estimate_charuco_pose(image, intrinsics, board_config)
    # vis_image = estimator.visualize_charuco(image, intrinsics, board_pose, board_config)
    
    print("\n💡 USAGE TIPS:")
    print("1. ChArUco boards are much more accurate than individual ArUco markers")
    print("2. ChArUco needs at least 4 corners visible for pose estimation")
    print("3. Individual ArUco markers work with just one marker visible")
    print("4. For your 35 pixel markers at 300 DPI = 0.297cm actual size")
    print("5. Consider using larger markers (413 pixels) for 3.5cm physical size")

if __name__ == "__main__":
    example_usage()