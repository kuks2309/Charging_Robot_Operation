import cv2
import numpy as np
from cv2 import aruco

class ArucoCameraPoseEstimator:
    """
    Estimate camera pose relative to ArUco markers and ChArUco boards
    """
    
    def __init__(self, marker_size_meters=0.015, dictionary_type=aruco.DICT_5X5_50):
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
            params = aruco.DetectorParameters()
            if is_apriltag:
                params.cornerRefinementMethod = aruco.CORNER_REFINE_APRILTAG
            self.detector = aruco.ArucoDetector(self.aruco_dict, params)
            self.use_legacy_api = False
        else:
            # OpenCV < 4.7.0 uses legacy API
            self.detector = None
            params = aruco.DetectorParameters_create()
            if is_apriltag:
                params.cornerRefinementMethod = aruco.CORNER_REFINE_APRILTAG
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

            # Define 3D points for marker corners (marker coordinate frame)
            half_size = self.marker_size / 2.0
            obj_points = np.array([
                [-half_size,  half_size, 0],
                [ half_size,  half_size, 0],
                [ half_size, -half_size, 0],
                [-half_size, -half_size, 0]
            ], dtype=np.float32)

            for i, marker_id in enumerate(ids.flatten()):
                # solvePnPGeneric로 IPPE 2개 해 획득 후 Z축 disambiguation
                n_solutions, rvecs_all, tvecs_all, reproj_errors = cv2.solvePnPGeneric(
                    obj_points,
                    corners[i].reshape(-1, 2),
                    camera_matrix,
                    dist_coeffs,
                    flags=cv2.SOLVEPNP_IPPE_SQUARE
                )

                if n_solutions == 0:
                    continue

                # Z축 법선 기준 disambiguation: z_axis.z < 0 인 해 선택
                rvec = rvecs_all[0].flatten()
                tvec = tvecs_all[0].flatten()
                for s in range(n_solutions):
                    rv = rvecs_all[s].flatten()
                    R_check, _ = cv2.Rodrigues(rv)
                    if R_check[2, 2] < 0:  # z_axis.z < 0
                        rvec = rv
                        tvec = tvecs_all[s].flatten()
                        break

                # LM refinement
                rvec, tvec = cv2.solvePnPRefineLM(
                    obj_points,
                    corners[i].reshape(-1, 2),
                    camera_matrix,
                    dist_coeffs,
                    rvec.reshape(3, 1),
                    tvec.reshape(3, 1)
                )

                rvec = rvec.flatten()
                tvec = tvec.flatten()
                
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
                
        
        return marker_poses

    def detect_marker_centers(self, image, camera_matrix=None, dist_coeffs=None, estimate_pose=False):
        """
        마커 검출 및 중심 좌표 반환 (pose 추정 없이 경량 검출).

        Args:
            image: 입력 이미지 (grayscale 또는 color)
            camera_matrix: 카메라 행렬 (numpy array, optional) - 제공 시 undistortPoints 적용
            dist_coeffs: 왜곡 계수 (numpy array, optional)
            estimate_pose: True이면 solvePnP로 tvec도 반환

        Returns:
            list of dict: [{'id': int, 'corners': ndarray, 'center': (cx, cy), 'tvec': ndarray (optional)}, ...]
        """
        if len(image.shape) == 3:
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        else:
            gray = image

        if self.use_legacy_api:
            corners, ids, _ = aruco.detectMarkers(gray, self.aruco_dict, parameters=self.detector_params)
        else:
            corners, ids, _ = self.detector.detectMarkers(gray)

        # solvePnP용 3D 포인트
        obj_points = None
        if estimate_pose and camera_matrix is not None and dist_coeffs is not None:
            half_size = self.marker_size / 2.0
            obj_points = np.array([
                [-half_size,  half_size, 0],
                [ half_size,  half_size, 0],
                [ half_size, -half_size, 0],
                [-half_size, -half_size, 0]
            ], dtype=np.float32)

        results = []
        if ids is not None:
            for i, mid in enumerate(ids.flatten()):
                crn = corners[i]
                if len(crn.shape) == 3:
                    crn = crn[0]

                if camera_matrix is not None and dist_coeffs is not None:
                    undist = cv2.undistortPoints(
                        crn.reshape(-1, 1, 2).astype(np.float64),
                        camera_matrix, dist_coeffs, P=camera_matrix
                    )
                    center = undist.reshape(-1, 2).mean(axis=0)
                else:
                    center = crn.mean(axis=0)

                entry = {
                    'id': int(mid),
                    'corners': corners[i],
                    'center': (float(center[0]), float(center[1])),
                }

                # pose 추정
                if obj_points is not None:
                    n_sol, rvecs_all, tvecs_all, _ = cv2.solvePnPGeneric(
                        obj_points, crn.reshape(-1, 2),
                        camera_matrix, dist_coeffs,
                        flags=cv2.SOLVEPNP_IPPE_SQUARE
                    )
                    if n_sol > 0:
                        rvec = rvecs_all[0].flatten()
                        tvec = tvecs_all[0].flatten()
                        for s in range(n_sol):
                            R_check, _ = cv2.Rodrigues(rvecs_all[s].flatten())
                            if R_check[2, 2] < 0:
                                rvec = rvecs_all[s].flatten()
                                tvec = tvecs_all[s].flatten()
                                break
                        entry['tvec'] = tvec

                results.append(entry)

        return results

    def detect_and_estimate_pose_dual_disambiguated(self, image, intrinsics, target_ids: tuple, known_distance_m: float):
        """
        Detect two ArUco markers and disambiguate poses using known inter-marker distance.

        Uses solvePnPGeneric to get BOTH IPPE solutions and selects the combination
        that best matches the known distance constraint.

        Args:
            image: Input image
            intrinsics: Camera intrinsics
            target_ids: Tuple of two marker IDs (e.g., (0, 1))
            known_distance_m: Known distance between markers in meters

        Returns:
            Tuple of (marker1_pose, marker2_pose, measured_distance, distance_error, combo_idx)
            Returns (None, None, 0, float('inf'), -1) if detection fails
        """
        print(f"[Disambiguation] Target IDs: {target_ids}, Known distance: {known_distance_m*1000:.2f}mm")

        camera_matrix = np.array([
            [intrinsics.fx, 0, intrinsics.ppx],
            [0, intrinsics.fy, intrinsics.ppy],
            [0, 0, 1]
        ], dtype=np.float32)
        dist_coeffs = np.array(intrinsics.coeffs)

        # Convert to grayscale
        if len(image.shape) == 3:
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        else:
            gray = image

        # Detect markers
        if self.use_legacy_api:
            corners, ids, _ = aruco.detectMarkers(gray, self.aruco_dict, parameters=self.detector_params)
        else:
            corners, ids, _ = self.detector.detectMarkers(gray)

        if ids is None:
            return (None, None, 0, float('inf'), -1)

        ids_flat = ids.flatten()

        # Find target markers
        marker1_idx = np.where(ids_flat == target_ids[0])[0]
        marker2_idx = np.where(ids_flat == target_ids[1])[0]

        if len(marker1_idx) == 0 or len(marker2_idx) == 0:
            return (None, None, 0, float('inf'), -1)

        marker1_idx = marker1_idx[0]
        marker2_idx = marker2_idx[0]

        # Object points for marker
        half_size = self.marker_size / 2.0
        obj_points = np.array([
            [-half_size,  half_size, 0],
            [ half_size,  half_size, 0],
            [ half_size, -half_size, 0],
            [-half_size, -half_size, 0]
        ], dtype=np.float32)

        # Get BOTH IPPE solutions for each marker using solvePnPGeneric
        def get_dual_solutions(corner_idx):
            num_sol, rvecs, tvecs, reproj_errs = cv2.solvePnPGeneric(
                obj_points,
                corners[corner_idx].reshape(-1, 2),
                camera_matrix,
                dist_coeffs,
                flags=cv2.SOLVEPNP_IPPE_SQUARE
            )
            solutions = []
            for i in range(num_sol):
                solutions.append({
                    'rvec': rvecs[i].flatten(),
                    'tvec': tvecs[i].flatten(),
                    'reproj_error': reproj_errs[i][0] if reproj_errs is not None else 0
                })
            return solutions

        m1_solutions = get_dual_solutions(marker1_idx)
        m2_solutions = get_dual_solutions(marker2_idx)

        print(f"[Disambiguation] Marker {target_ids[0]}: {len(m1_solutions)} IPPE solutions, Marker {target_ids[1]}: {len(m2_solutions)} IPPE solutions")

        if len(m1_solutions) < 2 or len(m2_solutions) < 2:
            # Fallback: not enough solutions
            return (None, None, 0, float('inf'), -1)

        # Evaluate all 4 combinations
        best_combo = None
        best_error = float('inf')
        best_distance = 0
        best_combo_idx = -1

        combinations = [
            (0, 0), (0, 1), (1, 0), (1, 1)
        ]

        for combo_idx, (i, j) in enumerate(combinations):
            tvec1 = m1_solutions[i]['tvec']
            tvec2 = m2_solutions[j]['tvec']
            rvec1 = m1_solutions[i]['rvec']
            rvec2 = m2_solutions[j]['rvec']

            distance = np.linalg.norm(tvec1 - tvec2)
            dist_error = abs(distance - known_distance_m)

            # Z축 방향 일치 확인 (같은 평면이면 Z축이 비슷해야 함)
            R1, _ = cv2.Rodrigues(rvec1)
            R2, _ = cv2.Rodrigues(rvec2)
            z1 = R1[:, 2]  # 마커1 Z축
            z2 = R2[:, 2]  # 마커2 Z축
            z_dot = np.dot(z1, z2)  # 내적: 1이면 같은 방향, -1이면 반대
            z_angle = np.degrees(np.arccos(np.clip(z_dot, -1, 1)))

            # Z축 각도가 30° 이상이면 뒤집힌 것으로 판단, 페널티 부여
            if z_angle > 30:
                penalty = 1.0  # 큰 페널티
            else:
                penalty = 0.0

            error = dist_error + penalty

            print(f"[Disambiguation] Combo {combo_idx} ({i},{j}): dist={distance*1000:.2f}mm, z_angle={z_angle:.1f}°, err={dist_error*1000:.2f}mm, penalty={penalty:.1f}")

            if error < best_error:
                best_error = error
                best_distance = distance
                best_combo = (i, j)
                best_combo_idx = combo_idx

        # Apply LM refinement to selected solutions
        i, j = best_combo
        print(f"[Disambiguation] Selected combo {best_combo_idx}, error={best_error*1000:.2f}mm")

        rvec1_refined, tvec1_refined = cv2.solvePnPRefineLM(
            obj_points,
            corners[marker1_idx].reshape(-1, 2),
            camera_matrix,
            dist_coeffs,
            m1_solutions[i]['rvec'].reshape(3, 1),
            m1_solutions[i]['tvec'].reshape(3, 1)
        )

        rvec2_refined, tvec2_refined = cv2.solvePnPRefineLM(
            obj_points,
            corners[marker2_idx].reshape(-1, 2),
            camera_matrix,
            dist_coeffs,
            m2_solutions[j]['rvec'].reshape(3, 1),
            m2_solutions[j]['tvec'].reshape(3, 1)
        )

        # Build marker pose dicts
        def build_pose(rvec, tvec, marker_id, corner):
            rvec = rvec.flatten()
            tvec = tvec.flatten()
            rotation_matrix, _ = cv2.Rodrigues(rvec)
            camera_rotation_in_marker = rotation_matrix.T
            camera_position_in_marker = -camera_rotation_in_marker @ tvec
            return {
                'id': int(marker_id),
                'corners': corner,
                'rvec': rvec,
                'tvec': tvec,
                'rotation_matrix': rotation_matrix,
                'camera_position': camera_position_in_marker,
                'camera_rotation': camera_rotation_in_marker,
                'marker_position_in_camera': tvec,
                'marker_rotation_in_camera': rotation_matrix
            }

        marker1_pose = build_pose(rvec1_refined, tvec1_refined, target_ids[0], corners[marker1_idx])
        marker2_pose = build_pose(rvec2_refined, tvec2_refined, target_ids[1], corners[marker2_idx])

        # Recalculate distance with refined poses
        final_distance = np.linalg.norm(tvec1_refined.flatten() - tvec2_refined.flatten())
        final_error = abs(final_distance - known_distance_m)

        return (marker1_pose, marker2_pose, final_distance, final_error, best_combo_idx)

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

                return board_info

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
                    
                    return board_info

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

    def _draw_text_with_background(self, image, text_lines, pos, alpha=0.5):
        """Helper function to draw text with semi-transparent background"""
        text_height = 20
        max_text_width = max([cv2.getTextSize(line, cv2.FONT_HERSHEY_SIMPLEX, 0.4, 1)[0][0]
                            for line in text_lines])

        # Background rectangle with transparency
        bg_top_left = (pos[0] - 5, pos[1] - 15)
        bg_bottom_right = (pos[0] + max_text_width + 10,
                         pos[1] + len(text_lines) * text_height + 5)

        # Create overlay for semi-transparent background
        overlay = image.copy()
        cv2.rectangle(overlay, bg_top_left, bg_bottom_right, (0, 0, 0), -1)
        cv2.addWeighted(overlay, alpha, image, 1 - alpha, 0, image)

        # Border
        cv2.rectangle(image, bg_top_left, bg_bottom_right, (255, 255, 255), 1)

        # Text lines with outline for better visibility
        for j, line in enumerate(text_lines):
            line_pos = (pos[0], pos[1] + j * text_height)
            # Black outline
            cv2.putText(image, line, line_pos, cv2.FONT_HERSHEY_SIMPLEX,
                      0.4, (0, 0, 0), 2)
            # White text
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