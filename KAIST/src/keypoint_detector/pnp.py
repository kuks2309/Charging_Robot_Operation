import math
import numpy as np
import cv2
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from scipy.spatial.transform import Rotation as R
from scipy.linalg import block_diag
import pupil_apriltags as apriltag

from keypoint_detector.gamma_correction import process_bright, process_dimmed
        
        

def process_img(img, visualize=False):
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    # Preprocessing
    # blur = cv2.GaussianBlur(gray, (5, 5), 0)
    edges = cv2.Canny(gray, 50, 150)
    cv2.imshow("edges", edges)
    # Find contours
    contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    for cnt in contours:
        peri = cv2.arcLength(cnt, True)
        approx = cv2.approxPolyDP(cnt, 0.02 * peri, True)

        # Only rectangles (4 points)
        if len(approx) == 4:
            # Create mask for this rectangle
            mask = np.zeros(img.shape[:2], dtype=np.uint8)
            cv2.drawContours(mask, [approx], -1, (255), -1)

            # Invert ONLY inside the mask
            img[mask == 255] = 255 - img[mask == 255]
        
        # inverted = 255 - img  
        
        cv2.imshow("gamma", img)
        cv2.waitKey(1)
        return img

APRILTAG_DETECTOR = apriltag.Detector(
            families="tag36h11",
            nthreads=4,
            quad_decimate=1.0,
            quad_sigma=0.0,
            refine_edges=True,
            decode_sharpening=0.25,
            debug=False
        )

class PnPKeypointAligner:
    """
    Uses PnP (Perspective-n-Point) to estimate 6DOF pose and align template keypoints
    with detected keypoints in camera images.
    """
    
    def __init__(self, template_keypoints_3d, camera_matrix, dist_coeffs=None, offset=None):
        """
        Initialize PnP-based aligner
        
        Args:
            template_keypoints_3d: Nx3 array of 3D template keypoint positions (world coordinates)
            camera_matrix: 3x3 camera intrinsic matrix
            dist_coeffs: distortion coefficients (can be None)
        """
        self.offset=offset
        self.template_keypoints_3d = np.array(template_keypoints_3d, dtype=np.float32)
        # centroid = np.mean(self.template_keypoints_3d, axis=0)
        # self.template_keypoints_3d -= centroid

        self.camera_matrix = np.array(camera_matrix, dtype=np.float32)
        self.dist_coeffs = dist_coeffs if dist_coeffs is not None else np.zeros(5, dtype=np.float32)
        
        # apriltag detector
        self.detector = APRILTAG_DETECTOR
        
        self.current_pose = None
        self.pose_history = []
        
    def _rotation_offset(self, offset, rvec, tvec, d=None):
                    # marker->camera transform
        Rm, _ = cv2.Rodrigues(rvec)
        T_cam_marker = np.eye(4)
        T_cam_marker[:3, :3] = Rm
        T_cam_marker[:3, 3] = tvec.squeeze()

        T_cam_obj = T_cam_marker

        T_marker_obj = np.eye(4)
        T_marker_obj[:3, :3] = R.from_euler("xyz", offset[3:], degrees=True).as_matrix()
        T_marker_obj[:3, 3] = np.array(offset[:3])
        T_cam_obj = T_cam_marker @ T_marker_obj
            
        return T_cam_obj
    
    def solve_pnp(self, detected_keypoints_2d, correspondences=None, world_info=None, method=cv2.SOLVEPNP_SQPNP):
        """
        Solve PnP to get camera pose relative to object
        
        Args:
            detected_keypoints_2d: Mx2 array of detected 2D keypoints in image
            correspondences: list of (template_idx, detected_idx) pairs, or None for automatic
            method: OpenCV PnP solver method
            
        Returns:
            pose_info: dict containing rotation vector, translation vector, and success flag
        """
        print("Detected_keyponts", detected_keypoints_2d)
        # detected_2d = np.array(detected_keypoints_2d, dtype=np.float32)
        
        print("please work", correspondences)
        
        # if correspondences is None:
        #     # Assume correspondences are in order (first N template points match first N detected)
        #     if len(detected_2d) > len(self.template_keypoints_3d):
        #         print(f"Warning: More detections ({len(detected_2d)}) than template points ({len(self.template_keypoints_3d)})")
            
        #     n_points = min(len(detected_2d), len(self.template_keypoints_3d))
        #     object_points = self.template_keypoints_3d[:n_points]
        #     image_points = detected_2d[:n_points]
        #     used_correspondences = [(i, i) for i in range(n_points)]
        if True:
            # Use provided correspondences
            object_points = []
            image_points = []
            used_correspondences = correspondences

            # print("Detected keypoints 2d", detected_keypoints_2d[detected_idx])
            
            for (template_idx, detected_idx) in correspondences:
                print("detected idx", template_idx, detected_idx)
                if detected_keypoints_2d[detected_idx] is not None: #and template_idx not in [0,1]:
                    object_points.append(self.template_keypoints_3d[template_idx])
                    image_points.append(detected_keypoints_2d[detected_idx])
            
            object_points = np.array(object_points, dtype=np.float32)
            image_points = np.array(image_points, dtype=np.float32)
        
        print("Please work 2")
        
        if len(image_points) < 4:
            print(f"Error: Need at least 4 point correspondences, got {len(object_points)}")
            return None
        
        # Solve PnP
        offset = (0.0,0.003,0,-1, 1,0) # Rx before -1.4 Ry before 0.3 , x was -0;0015
        success, rvec, tvec = cv2.solvePnP(
            object_points, image_points, 
            self.camera_matrix, self.dist_coeffs, 
            flags=method
        )
        
        # Calculate reprojection error
        projected_points, _ = cv2.projectPoints(
            object_points, rvec, tvec, self.camera_matrix, self.dist_coeffs
        )
        projected_points = projected_points.reshape(-1, 2)
        reprojection_error = np.mean(np.linalg.norm(image_points - projected_points, axis=1))
        
        t_cam_obj = self._rotation_offset(offset, rvec, tvec)
        tvec = t_cam_obj[:3, 3]
        
        print("Solved pnp")
        
        if not success:
            print("PnP solution failed")
            return None
        
        # Convert rotation vector to rotation matrix
        # rvec = R.from_matrix(t_cam_obj[:3, :3]).as_matrix()
        
        rotation_matrix = t_cam_obj[:3, :3]

        # if self.offset is not None:
            
        #     T_cam_marker = np.eye(4)
        #     T_cam_marker[:3, :3] = rotation_matrix
        #     T_cam_marker[:3, 3] = tvec.squeeze()

        #     # marker offset (optional)
        #     T_marker_obj = np.eye(4)
        #     T_marker_obj[:3, :3] = R.from_euler("xyz", self.offset[3:], degrees=True).as_matrix()
        #     T_marker_obj[:3, 3] = np.array(self.offset[:3])
        #     T_cam_obj = T_cam_marker @ T_marker_obj
        #     rotation_matrix = T_cam_obj[:3, :3]
        #     tvec = T_cam_obj[:3, 3]
        #     # tvec += rotation_matrix[2, :] @ np.array([0,0,0.071]).T

        rotation_matrix_r = R.from_matrix(rotation_matrix)
        quat_xyzw = rotation_matrix_r.as_quat()  # [x,y,z,w]
        quat_wxyz = np.array([quat_xyzw[3], quat_xyzw[0], quat_xyzw[1], quat_xyzw[2]])  # [w,x,y,z]+

        print("Got quaternion")

        world_pose = None
        if world_info is not None:
            R_w = (R.from_matrix(world_info["camera_rotation"]) * rotation_matrix_r).as_quat()
            R_w = np.array([R_w[3], R_w[0], R_w[1], R_w[2]])
            t_w = world_info["camera_position"].squeeze() + world_info["camera_rotation"] @ tvec.squeeze()
            world_pose = np.concatenate([t_w, R_w])

        print("After world_info")

        pose_info = {
            'success': success,
            'rvec': rvec,
            'tvec': tvec.squeeze(),
            'rotation_matrix': rotation_matrix,
            'rotation_quat': quat_wxyz,
            'pose': np.concatenate([tvec.squeeze(), quat_wxyz]),
            'world_pose': world_pose,
            'correspondences': used_correspondences,
            'n_points': len(object_points),
            'confidence_score': None
        }
        

        
        print("Projected points")
        
    
        
        print("Reprojection Error", reprojection_error)
        pose_info['confidence_score'] = math.exp(-0.0811*reprojection_error) ## Makes it so that 2 pixel error is equivalent to 0.85
        # 0 -> 1; 2 -> 0.85
        self.current_pose = pose_info
        self.pose_history.append(pose_info)
        
        return pose_info
    
    def project_template_to_image(self, pose_info=None, additional_points_3d=None):
        """
        Project template keypoints (and optionally additional 3D points) to image coordinates
        
        Args:
            pose_info: pose information from solve_pnp (uses current if None)
            additional_points_3d: additional 3D points to project
            
        Returns:
            projected_template: 2D image coordinates of template keypoints
            projected_additional: 2D image coordinates of additional points (if provided)
        """
        if pose_info is None:
            pose_info = self.current_pose
            
        if pose_info is None:
            print("No pose information available. Run solve_pnp first.")
            return None, None
        
        # Project template keypoints
        projected_template, _ = cv2.projectPoints(
            self.template_keypoints_3d, 
            pose_info['rvec'], pose_info['tvec'],
            self.camera_matrix, self.dist_coeffs
        )
        projected_template = projected_template.reshape(-1, 2)
        
        projected_additional = None
        if additional_points_3d is not None:
            additional_points_3d = np.array(additional_points_3d, dtype=np.float32)
            projected_additional, _ = cv2.projectPoints(
                additional_points_3d,
                pose_info['rvec'], pose_info['tvec'],
                self.camera_matrix, self.dist_coeffs
            )
            projected_additional = projected_additional.reshape(-1, 2)
        
        return projected_template, projected_additional
    
    def align_keypoints_pnp(self, detected_keypoints_2d, world_info=None, correspondences=None, method=cv2.SOLVEPNP_SQPNP,
                           confidence_threshold=0.7):
        """
        Main function to align keypoints using PnP
        
        Args:
            detected_keypoints_2d: detected 2D keypoints in image
            correspondences: optional correspondences
            confidence_threshold: maximum allowed reprojection error
            
        Returns:
            aligned_keypoints_2d: projected template keypoints in image coordinates
            pose_info: pose estimation information
        """
        # Solve PnP
        pose_info = self.solve_pnp(detected_keypoints_2d, correspondences, world_info, method)
        print("Got the pose info")
        if pose_info is None or not pose_info['success']:
            print("Failed to solve PnP")
            return None, None
        
        # Check reprojection error
        if pose_info['confidence_score'] < confidence_threshold:
            print(f"High reprojection error: {pose_info['confidence_score']:.2f} pixels")
            print("Pose is too unreliable")
            
            return None, None
        # Project all template keypoints to image
        aligned_keypoints_2d, _ = self.project_template_to_image(pose_info)
        
        print("got the aligned_keypoints")
        print(f"PnP alignment successful:")
        print(f"  - Used {pose_info['n_points']} correspondences")
        print(f"  - confidence_score: {pose_info['confidence_score']:.2f} pixels")
        
        return aligned_keypoints_2d, pose_info

    def align_keypoints_pnp_marker(self, color_image, world_info=None):
        # MARKER_OFFSETS = {0: (0.0, -0.037, +0.07, 0, 0, 0)}
        # color_image = process_img(color_image)
        gray = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)
        # gray = cv2.Canny(gray, 50,150)
        cv2.imshow("gray", gray)
        # cv2.waitKey(1)
        detections = self.detector.detect(gray)
        if not detections:
            print("No AprilTag detected")
            return None, None

        T_candidates = []
        reproj_errors = []
        MARKER_OFFSETS = {0: (+0.001, -0.035, +0.07, 0, 0, 0)} 
        for d in detections:
            if d.tag_id not in MARKER_OFFSETS.keys():
                continue
            corners = np.array(d.corners, dtype=np.float32)

            # SolvePnP with square marker
            marker_len = 0.0182  # marker size in meters
            objp = np.array([
                [-marker_len/2,  marker_len/2, 0],
                [ marker_len/2,  marker_len/2, 0],
                [ marker_len/2, -marker_len/2, 0],
                [-marker_len/2, -marker_len/2, 0]
            ], dtype=np.float32)

            print(corners)
            vis_corners = np.array(corners, dtype=np.int32)  # shape (4, 2)

            vis_corners = vis_corners.reshape((-1, 1, 2))          # shape (4,1,2) for cv2

            cv2.polylines(
                color_image, 
                [vis_corners],          # list of point arrays
                isClosed=True,    # True = closed polygon
                color=(0, 255, 0), # green
                thickness=1
            )
            cv2.imshow("corners", color_image)
            cv2.waitKey(1)
            ok, rvec, tvec = cv2.solvePnP(objp, corners, self.camera_matrix, self.dist_coeffs, flags=cv2.SOLVEPNP_IPPE_SQUARE)
            if not ok:
                continue

            # --- Compute reprojection error ---
            projected_points, _ = cv2.projectPoints(objp, rvec, tvec, self.camera_matrix, self.dist_coeffs)
            projected_points = projected_points.squeeze()
            error = np.mean(np.linalg.norm(corners - projected_points, axis=1))  # mean per-point error (in pixels)
            reproj_errors.append(error)

            # marker->camera transform
            Rm, _ = cv2.Rodrigues(rvec)
            T_cam_marker = np.eye(4)
            T_cam_marker[:3, :3] = Rm
            T_cam_marker[:3, 3] = tvec.squeeze()
            # T_cam_obj = T_cam_marker

            if d.tag_id in MARKER_OFFSETS:
                off = MARKER_OFFSETS[d.tag_id]
                T_marker_obj = np.eye(4)
                T_marker_obj[:3, :3] = R.from_euler("xyz", off[3:], degrees=True).as_matrix()
                T_marker_obj[:3, 3] = np.array(off[:3])
                T_cam_obj = T_cam_marker @ T_marker_obj
            else:
                T_cam_obj = T_cam_marker

            T_candidates.append(T_cam_obj)

        if not T_candidates:
            return None, None

        # --- Fuse multiple markers ---
        t_all = np.stack([T[:3, 3] for T in T_candidates])
        t_mean = np.mean(t_all, axis=0)
        quats = np.stack([R.from_matrix(T[:3, :3]).as_quat() for T in T_candidates])
        q_mean = np.mean(quats, axis=0)
        q_mean /= np.linalg.norm(q_mean)
        R_mean = R.from_quat(q_mean).as_matrix()

        T_final = np.eye(4)
        T_final[:3, :3] = R_mean
        T_final[:3, 3] = t_mean

        # Convert back to rvec,tvec
        rvec, _ = cv2.Rodrigues(R_mean)
        tvec = t_mean
        
        # --- world_pose 변환 ---
        world_pose = None
        if world_info is not None:
            R_w = (R.from_matrix(world_info["camera_rotation"]) * R.from_matrix(R_mean)).as_quat()
            R_w = np.array([R_w[3], R_w[0], R_w[1], R_w[2]])  # [w,x,y,z]
            t_w = world_info["camera_position"].squeeze() + world_info["camera_rotation"] @ tvec.squeeze()
            world_pose = np.concatenate([t_w, R_w])

        # --- solve_pnp()와 동일한 포맷으로 통일 ---
        quat_xyzw = R.from_matrix(R_mean).as_quat()
        quat_wxyz = np.array([quat_xyzw[3], quat_xyzw[0], quat_xyzw[1], quat_xyzw[2]])
        mean_reproj_error = float(np.mean(reproj_errors)) if reproj_errors else None
        confidence_score = np.exp(-0.5 * mean_reproj_error)

        pose_info = {
            'success': True,
            'rvec': rvec,
            'tvec': tvec.squeeze(),
            'rotation_matrix': R_mean,
            'rotation_quat': quat_wxyz,
            'pose': np.concatenate([tvec.squeeze(), quat_wxyz]),
            'world_pose': world_pose,
            'correspondences': [],
            'n_points': 4,
            'confidence_score': confidence_score
        }

        self.current_pose = pose_info
        self.pose_history.append(pose_info)

        return None, pose_info  # aligned_keypoints_2d는 필요없으니 None
    
    def align_keypoints_pnp_port(self, color_image, world_info=None):
        # MARKER_OFFSETS = {0: (0.0, -0.037, +0.07, 0, 0, 0)}
        # color_image = process_img(color_image)
        gray = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)
        # gray = cv2.Canny(gray, 50,150)
        cv2.imshow("gray", gray)
        # cv2.waitKey(1)
        detections = self.detector.detect(gray)
        if not detections:
            print("No AprilTag detected")
            return None, None

        T_candidates = []
        reproj_errors = []
        # MARKER_OFFSETS = {10: (0, 0, 0, 0.0, 0, 0)} 
        # MARKER_OFFSETS = {10: (-0.029, +0.063, -0.2, -2.0, 0, 0)}  
        # MARKER_OFFSETS = {10: (-0.029, +0.063, -0.2, 0.0, 0, 0)}  
        # MARKER_OFFSETS = {10: (-0.029, +0.2, 0.0, -17.5, 0, 0)}  # 1 step
        MARKER_OFFSETS = {10: (-0.029, +0.063, 0.0, -2.0, 0, 0)} # 2 step
        for d in detections:
            if d.tag_id not in MARKER_OFFSETS.keys():
                continue
            corners = np.array(d.corners, dtype=np.float32)

            # SolvePnP with square marker
            marker_len = 0.0182  # marker size in meters
            objp = np.array([
                [-marker_len/2,  marker_len/2, 0],
                [ marker_len/2,  marker_len/2, 0],
                [ marker_len/2, -marker_len/2, 0],
                [-marker_len/2, -marker_len/2, 0]
            ], dtype=np.float32)

            print(corners)
            vis_corners = np.array(corners, dtype=np.int32)  # shape (4, 2)

            vis_corners = vis_corners.reshape((-1, 1, 2))          # shape (4,1,2) for cv2

            cv2.polylines(
                color_image, 
                [vis_corners],          # list of point arrays
                isClosed=True,    # True = closed polygon
                color=(0, 255, 0), # green
                thickness=1
            )
            cv2.imshow("corners", color_image)
            cv2.waitKey(1)
            ok, rvec, tvec = cv2.solvePnP(objp, corners, self.camera_matrix, self.dist_coeffs, flags=cv2.SOLVEPNP_IPPE_SQUARE)
            if not ok:
                continue

            # --- Compute reprojection error ---
            projected_points, _ = cv2.projectPoints(objp, rvec, tvec, self.camera_matrix, self.dist_coeffs)
            projected_points = projected_points.squeeze()
            error = np.mean(np.linalg.norm(corners - projected_points, axis=1))  # mean per-point error (in pixels)
            reproj_errors.append(error)

            # marker->camera transform
            Rm, _ = cv2.Rodrigues(rvec)
            T_cam_marker = np.eye(4)
            T_cam_marker[:3, :3] = Rm
            T_cam_marker[:3, 3] = tvec.squeeze()
            # T_cam_obj = T_cam_marker

            if d.tag_id in MARKER_OFFSETS:
                off = MARKER_OFFSETS[d.tag_id]
                T_marker_obj = np.eye(4)
                T_marker_obj[:3, :3] = R.from_euler("xyz", off[3:], degrees=True).as_matrix()
                T_marker_obj[:3, 3] = np.array(off[:3])
                T_cam_obj = T_cam_marker @ T_marker_obj
            else:
                T_cam_obj = T_cam_marker

            T_candidates.append(T_cam_obj)

        if not T_candidates:
            return None, None

        # --- Fuse multiple markers ---
        t_all = np.stack([T[:3, 3] for T in T_candidates])
        t_mean = np.mean(t_all, axis=0)
        quats = np.stack([R.from_matrix(T[:3, :3]).as_quat() for T in T_candidates])
        q_mean = np.mean(quats, axis=0)
        q_mean /= np.linalg.norm(q_mean)
        R_mean = R.from_quat(q_mean).as_matrix()

        T_final = np.eye(4)
        T_final[:3, :3] = R_mean
        T_final[:3, 3] = t_mean

        # Convert back to rvec,tvec
        rvec, _ = cv2.Rodrigues(R_mean)
        tvec = t_mean
        
        # --- world_pose 변환 ---
        world_pose = None
        if world_info is not None:
            R_w = (R.from_matrix(world_info["camera_rotation"]) * R.from_matrix(R_mean)).as_quat()
            R_w = np.array([R_w[3], R_w[0], R_w[1], R_w[2]])  # [w,x,y,z]
            t_w = world_info["camera_position"].squeeze() + world_info["camera_rotation"] @ tvec.squeeze()
            world_pose = np.concatenate([t_w, R_w])

        # --- solve_pnp()와 동일한 포맷으로 통일 ---
        quat_xyzw = R.from_matrix(R_mean).as_quat()
        quat_wxyz = np.array([quat_xyzw[3], quat_xyzw[0], quat_xyzw[1], quat_xyzw[2]])
        mean_reproj_error = float(np.mean(reproj_errors)) if reproj_errors else None
        confidence_score = np.exp(-0.5 * mean_reproj_error)

        pose_info = {
            'success': True,
            'rvec': rvec,
            'tvec': tvec.squeeze(),
            'rotation_matrix': R_mean,
            'rotation_quat': quat_wxyz,
            'pose': np.concatenate([tvec.squeeze(), quat_wxyz]),
            'world_pose': world_pose,
            'correspondences': [],
            'n_points': 4,
            'confidence_score': confidence_score
        }

        self.current_pose = pose_info
        self.pose_history.append(pose_info)

        return None, pose_info  # aligned_keypoints_2d는 필요없으니 None
    
    # def align_keypoints_pnp_port(self, color_image, world_info=None):
    #     # MARKER_OFFSETS = {0: (0.0, -0.037, +0.07, 0, 0, 0)} 
    #     gray = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)
    #     cv2.imshow("gray", gray)
    #     detections = self.detector.detect(gray)
    #     if not detections:
    #         print("No AprilTag detected")
    #         return None, None

    #     T_candidates = []
    #     reproj_errors = []
    #     # MARKER_OFFSETS = { 
    #     #         10: (-0.03, -0.0605, 0.0, +2.5, 0, 0)
    #     #     }
    #     MARKER_OFFSETS = { 
    #             10: (-0.06, -0.0605, 0.0, +2.5, 0, 0)
    #         }

    #     for d in detections:
    #         # just include marker that is in 
    #         if d.tag_id not in MARKER_OFFSETS.keys():
    #             continue

    #         corners = np.array(d.corners, dtype=np.float32)

    #         # SolvePnP with square marker
    #         marker_len = 0.0182  # marker size in meters
    #         objp = np.array([
    #             [-marker_len/2,  marker_len/2, 0],
    #             [ marker_len/2,  marker_len/2, 0],
    #             [ marker_len/2, -marker_len/2, 0],
    #             [-marker_len/2, -marker_len/2, 0]
    #         ], dtype=np.float32)

    #         print(corners)
    #         vis_corners = np.array(corners, dtype=np.int32)  # shape (4, 2)

    #         vis_corners = vis_corners.reshape((-1, 1, 2))          # shape (4,1,2) for cv2

    #         cv2.polylines(
    #             color_image, 
    #             [vis_corners],          # list of point arrays
    #             isClosed=True,    # True = closed polygon
    #             color=(0, 255, 0), # green
    #             thickness=1
    #         )
    #         cv2.imshow("corners", color_image)

    #         ok, rvec, tvec = cv2.solvePnP(objp, corners, self.camera_matrix, self.dist_coeffs, flags=cv2.SOLVEPNP_IPPE_SQUARE)
    #         if not ok:
    #             continue

    #         # --- Compute reprojection error ---
    #         projected_points, _ = cv2.projectPoints(objp, rvec, tvec, self.camera_matrix, self.dist_coeffs)
    #         projected_points = projected_points.squeeze()
    #         error = np.mean(np.linalg.norm(corners - projected_points, axis=1))  # mean per-point error (in pixels)
    #         reproj_errors.append(error)

    #         R_fix = np.diag([-1,-1,1])
    #         R_matrix, _ = cv2.Rodrigues(rvec)
    #         R_matrix_fixed = R_matrix @ R_fix

    #         # marker->camera transform
    #         Rm, _ = cv2.Rodrigues(rvec)
    #         T_cam_marker = np.eye(4)
    #         T_cam_marker[:3, :3] = R_matrix_fixed
    #         T_cam_marker[:3, 3] = tvec.squeeze()
    #         # T_cam_obj = T_cam_marker

    #         # MARKER_OFFSETS = { # -2.5 good
    #         #         5: (+0.006, +0.0605, 0.0, -2.5, 0, 0),
    #         #         10: (0.0, +0.0605, 0.0, -2.5, 0, 0)
    #         #     }
            
    #         MARKER_OFFSETS = { # -2.5 good
    #                 10: (0.0, -0.0605, 0.0, +2.5, 0, 0)
    #             }
            
    #         if d.tag_id in MARKER_OFFSETS:
    #             off = MARKER_OFFSETS[d.tag_id]
    #             T_marker_obj = np.eye(4)
    #             T_marker_obj[:3, :3] = R.from_euler("xyz", off[3:], degrees=True).as_matrix()
    #             T_marker_obj[:3, 3] = np.array(off[:3])
    #             T_cam_obj = T_cam_marker @ T_marker_obj
    #         else:
    #             T_cam_obj = T_cam_marker

    #         T_candidates.append(T_cam_obj)

    #     if not T_candidates:
    #         return None, None

    #     # --- Fuse multiple markers ---
    #     t_all = np.stack([T[:3, 3] for T in T_candidates])
    #     tvec_fixed = np.mean(t_all, axis=0)
    #     quats = np.stack([R.from_matrix(T[:3, :3]).as_quat() for T in T_candidates])
    #     q_mean = np.mean(quats, axis=0)
    #     q_mean /= np.linalg.norm(q_mean)
    #     R_mean_fixed = R.from_quat(q_mean).as_matrix()

    #     # Convert back to rvec,tvec
    #     rvec_fixed, _ = cv2.Rodrigues(R_mean_fixed)
        
    #     # --- world_pose 변환 ---
    #     world_pose = None
    #     if world_info is not None:
    #         R_w = (R.from_matrix(world_info["camera_rotation"]) * R.from_matrix(R_mean_fixed)).as_quat()
    #         R_w = np.array([R_w[3], R_w[0], R_w[1], R_w[2]])  # [w,x,y,z]
    #         t_w = world_info["camera_position"].squeeze() + world_info["camera_rotation"] @ tvec_fixed.squeeze()
    #         world_pose = np.concatenate([t_w, R_w]) # world_pose: this is the input of the kalman filter

    #     # --- solve_pnp()와 동일한 포맷으로 통일 ---
    #     quat_xyzw = R.from_matrix(R_mean_fixed).as_quat()
    #     quat_wxyz = np.array([quat_xyzw[3], quat_xyzw[0], quat_xyzw[1], quat_xyzw[2]])
    #     mean_reproj_error = float(np.mean(reproj_errors)) if reproj_errors else None
    #     confidence_score = np.exp(-0.5 * mean_reproj_error)

    #     pose_info = {
    #         'success': True,
    #         'rvec': rvec_fixed,
    #         'tvec': tvec_fixed.squeeze(),
    #         'rotation_matrix': R_mean_fixed,
    #         'rotation_quat': quat_wxyz,
    #         'pose': np.concatenate([tvec_fixed.squeeze(), quat_wxyz]),
    #         'world_pose': world_pose,
    #         'correspondences': [],
    #         'n_points': 4,
    #         'confidence_score': confidence_score
    #     }

    #     self.current_pose = pose_info
    #     self.pose_history.append(pose_info)

    #     return None, pose_info  # aligned_keypoints_2d는 필요없으니 None
    
    def align_keypoints_pnp_t(self, color_image, world_info=None):
        gray = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)

        # --- 1) Detect markers in full image ---
        detections = self.detector.detect(gray)
        if not detections:
            print("No AprilTag detected")
            return None, None

        T_candidates = []
        reproj_errors = []

        # --- Marker-specific offsets (same as your code) ---
        MARKER_OFFSETS = {
            5: (+0.008, +0.0615, 0.0, 0, 0, 0),
            10: (0.0, +0.0615, 0.0, 0, 0, 0)
        }

        # --- 2) Process EACH marker separately ---
        for d0 in detections:
            # step A: get coarse corners
            c = np.array(d0.corners, dtype=np.float32)

            # build ROI around marker
            pad = 20
            x_min, x_max = int(np.min(c[:,0])), int(np.max(c[:,0]))
            y_min, y_max = int(np.min(c[:,1])), int(np.max(c[:,1]))

            x1 = max(x_min - pad, 0)
            y1 = max(y_min - pad, 0)
            x2 = min(x_max + pad, gray.shape[1])
            y2 = min(y_max + pad, gray.shape[0])

            crop = gray[y1:y2, x1:x2].copy()

            # step B: super-resolution
            sr = cv2.dnn_superres.DnnSuperResImpl_create()
            sr.readModel("ESPCN_x4.pb")
            sr.setModel("espcn", 4)
            scale = 4
            sr_crop = sr.upsample(crop)

            # step C: detect tag again on SR crop
            detections_sr = self.detector.detect(sr_crop)
            if not detections_sr:
                print("Marker", d0.tag_id, "not found in SR crop")
                continue

            d_sr = detections_sr[0]
            c_sr_raw = np.array(d_sr.corners, dtype=np.float32)

            # step D: convert SR-crop coords back to original-image coords
            c_sr = c_sr_raw.copy()
            c_sr /= scale
            c_sr[:,0] += x1
            c_sr[:,1] += y1
            corners = c_sr.astype(np.float32)

            # --- SolvePnP ---
            marker_len = 0.0182
            objp = np.array([
                [-marker_len/2,  marker_len/2, 0],
                [ marker_len/2,  marker_len/2, 0],
                [ marker_len/2, -marker_len/2, 0],
                [-marker_len/2, -marker_len/2, 0]
            ], dtype=np.float32)

            ok, rvec, tvec = cv2.solvePnP(
                objp, corners,
                self.camera_matrix, self.dist_coeffs,
                flags=cv2.SOLVEPNP_IPPE_SQUARE
            )
            if not ok:
                continue

            Rm, _ = cv2.Rodrigues(rvec)

            # --- Compute reprojection error ---
            proj, _ = cv2.projectPoints(objp, rvec, tvec, self.camera_matrix, self.dist_coeffs)
            proj = proj.reshape(-1, 2)
            err = float(np.mean(np.linalg.norm(corners - proj, axis=1)))
            reproj_errors.append(err)

            # --- Convert to 4x4 transform ---
            T_cam_marker = np.eye(4)
            T_cam_marker[:3, :3] = Rm
            T_cam_marker[:3, 3] = tvec.squeeze()

            # --- Apply marker offset if needed ---
            if d0.tag_id in MARKER_OFFSETS:
                off = MARKER_OFFSETS[d0.tag_id]
                T_marker_obj = np.eye(4)
                T_marker_obj[:3, :3] = R.from_euler("xyz", off[3:], degrees=True).as_matrix()
                T_marker_obj[:3, 3] = np.array(off[:3])
                T_cam_obj = T_cam_marker @ T_marker_obj
            else:
                T_cam_obj = T_cam_marker

            T_candidates.append(T_cam_obj)

        # --- No pose candidates? ---
        if not T_candidates:
            return None, None

        # --- 3) Fuse multiple markers (same logic as pnp_marker) ---
        t_all = np.stack([T[:3, 3] for T in T_candidates])
        t_mean = np.mean(t_all, axis=0)

        quats = np.stack([R.from_matrix(T[:3,:3]).as_quat() for T in T_candidates])
        q_mean = quats.mean(axis=0)
        q_mean /= np.linalg.norm(q_mean)

        R_mean = R.from_quat(q_mean).as_matrix()

        # convert to rvec/tvec
        rvec, _ = cv2.Rodrigues(R_mean)
        tvec = t_mean

        # confidence from reprojection error
        mean_err = float(np.mean(reproj_errors))
        confidence_score = np.exp(-0.5 * mean_err)

        # --- World pose (same as your marker code) ---
        world_pose = None
        if world_info is not None:
            R_w = (R.from_matrix(world_info["camera_rotation"]) * R.from_matrix(R_mean)).as_quat()
            R_w = np.array([R_w[3], R_w[0], R_w[1], R_w[2]])
            t_w = world_info["camera_position"].squeeze() + world_info["camera_rotation"] @ tvec.squeeze()
            world_pose = np.concatenate([t_w, R_w])
        
        quat_xyzw = R.from_matrix(R_mean).as_quat()
        quat_wxyz = np.array([quat_xyzw[3], quat_xyzw[0], quat_xyzw[1], quat_xyzw[2]])

        pose_info = {
            "success": True,
            "rvec": rvec,
            "tvec": tvec.squeeze(),
            "rotation_matrix": R_mean,
            "rotation_quat": quat_wxyz,
            "pose": np.concatenate([tvec.squeeze(), quat_wxyz]),
            "world_pose": world_pose,
            "confidence_score": confidence_score,
            "reprojection_error": mean_err,
            "n_points": 4
        }

        self.current_pose = pose_info
        self.pose_history.append(pose_info)

        return None, pose_info



    # def align_keypoints_pnp_marker(self, color_image, intrinsics, world_info=None):
    #     gray = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)
    #     detections = self.detector.detect(gray)

    #     if not detections:
    #         print("No AprilTag detected")
    #         return None, None

    #     T_candidates = []
    #     for d in detections:
    #         corners = np.array(d.corners, dtype=np.float32)

    #         # SolvePnP with square marker
    #         marker_len = 0.019  # marker size in meters
    #         objp = np.array([
    #             [-marker_len/2,  marker_len/2, 0],
    #             [ marker_len/2,  marker_len/2, 0],
    #             [ marker_len/2, -marker_len/2, 0],
    #             [-marker_len/2, -marker_len/2, 0]
    #         ], dtype=np.float32)

    #         K = np.array([[intrinsics.fx, 0, intrinsics.ppx],
    #                       [0, intrinsics.fy, intrinsics.ppy],
    #                       [0, 0, 1]], dtype=np.float32)
    #         dist = np.array(intrinsics.coeffs[:5], dtype=np.float32)

    #         # --- 1단계: 전체 PnP 추정 ---
    #         ok, rvec_full, tvec_full = cv2.solvePnP(
    #             objp, corners, K, dist,
    #             flags=cv2.SOLVEPNP_ITERATIVE
    #         )
    #         R_full, _ = cv2.Rodrigues(rvec_full)
    #         rx_full, ry_full, rz_full = R.from_matrix(R_full).as_euler('xyz', degrees=True)
    #         print("initial_pnp", [rx_full, ry_full, rz_full])
    #         if not ok:
    #             continue

    #         print("working1")

    #         # --- 2단계: ry, rz 고정하고 refine ---
    #         known_ry = 0
    #         known_rz = 0
    #         R_init = R.from_euler('xyz', [rx_full, known_ry, known_rz], degrees=True).as_matrix()
    #         rvec_init, _ = cv2.Rodrigues(R_init)
    #         tvec_init = tvec_full.copy()

    #         ok, rvec_refined, tvec_refined = cv2.solvePnP(
    #             objp, corners, K, dist,
    #             rvec=rvec_init, tvec=tvec_init,
    #             useExtrinsicGuess=True,
    #             flags=cv2.SOLVEPNP_ITERATIVE
    #         )

    #         # --- 결과 ---
    #         R_refined, _ = cv2.Rodrigues(rvec_refined)
    #         rx_refined, _, _ = R.from_matrix(R_refined).as_euler('xyz', degrees=True)
    #         R_final = R.from_euler('xyz', [rx_refined, known_ry, known_rz], degrees=True).as_matrix()

    #         T_cam_marker = np.align_keypoints_pnpeye(4)
    #         T_cam_marker[:3, :3] = R_final
    #         T_cam_marker[:3, 3] = tvec_refined.squeeze()
            
    #         # marker offset (optional)
    #         MARKER_OFFSETS = {0: (0.0, -0.031, 0.0705, 0, 0, 0),
    #                           1: (0.0,  0.00, 0.0, 90, 0, 0)}
    #         if d.tag_id in MARKER_OFFSETS:
    #             off = MARKER_OFFSETS[d.tag_id]
    #             T_marker_obj = np.eye(4)
    #             T_marker_obj[:3, :3] = R.from_euler("xyz", off[3:], degrees=True).as_matrix()
    #             T_marker_obj[:3, 3] = np.array(off[:3])
    #             T_cam_obj = T_cam_marker @ T_marker_obj
    #         else:
    #             T_cam_obj = T_cam_marker

    #         T_candidates.append(T_cam_obj)
            
    #         print("working6")

    #     if not T_candidates:
    #         return None, None

    #     # --- Fuse multiple markers ---
    #     t_all = np.stack([T[:3, 3] for T in T_candidates])
    #     t_mean = np.mean(t_all, axis=0)
    #     quats = np.stack([R.from_matrix(T[:3, :3]).as_quat() for T in T_candidates])
    #     q_mean = np.mean(quats, axis=0)
    #     q_mean /= np.linalg.norm(q_mean)
    #     R_mean = R.from_quat(q_mean).as_matrix()

    #     T_final = np.eye(4)
    #     T_final[:3, :3] = R_mean
    #     T_final[:3, 3] = t_mean

    #     # Convert back to rvec,tvec
    #     rvec, _ = cv2.Rodrigues(R_mean)
    #     tvec = t_mean
        
    #     # --- world_pose 변환 ---
    #     world_pose = None
    #     if world_info is not None:
    #         R_w = (R.from_matrix(world_info["camera_rotation"]) * R.from_matrix(R_mean)).as_quat()
    #         R_w = np.array([R_w[3], R_w[0], R_w[1], R_w[2]])  # [w,x,y,z]
    #         t_w = world_info["camera_position"].squeeze() + world_info["camera_rotation"] @ tvec.squeeze()
    #         world_pose = np.concatenate([t_w, R_w])

    #     # --- solve_pnp()와 동일한 포맷으로 통일 ---
    #     quat_xyzw = R.from_matrix(R_mean).as_quat()
    #     quat_wxyz = np.array([quat_xyzw[3], quat_xyzw[0], quat_xyzw[1], quat_xyzw[2]])

    #     pose_info = {
    #         'success': True,
    #         'rvec': rvec,
    #         'tvec': tvec.squeeze(),
    #         'rotation_matrix': R_mean,
    #         'rotation_quat': quat_wxyz,
    #         'pose': np.concatenate([tvec.squeeze(), quat_wxyz]),
    #         'world_pose': world_pose,
    #         'correspondences': [],
    #         'n_points': 4,
    #         'reprojection_error': 0.0
    #     }
        

    #     self.current_pose = pose_info
    #     self.pose_history.append(pose_info)

    #     return None, pose_info  # aligned_keypoints_2d는 필요없으니 None

    def get_pose_6dof(self, pose_info=None):
        """
        Get 6DOF pose in human-readable format
        
        Returns:
            dict with translation and rotation (roll, pitch, yaw)
        """
        if pose_info is None:
            pose_info = self.current_pose
            
        if pose_info is None:
            return None
        
        # Translation is straightforward
        translation = pose_info['tvec'].flatten()
        
        # Convert rotation matrix to Euler angles (roll, pitch, yaw)
        R = pose_info['rotation_matrix']
        
        # Extract Euler angles from rotation matrix
        sy = np.sqrt(R[0, 0]**2 + R[1, 0]**2)
        singular = sy < 1e-6
        
        if not singular:
            roll = np.arctan2(R[2, 1], R[2, 2])
            pitch = np.arctan2(-R[2, 0], sy)
            yaw = np.arctan2(R[1, 0], R[0, 0])
        else:
            roll = np.arctan2(-R[1, 2], R[1, 1])
            pitch = np.arctan2(-R[2, 0], sy)
            yaw = 0
        
        return {
            'translation_xyz': translation,
            'rotation_rpy': np.array([roll, pitch, yaw]),
            'rotation_rpy_degrees': np.array([roll, pitch, yaw]) * 180 / np.pi
        }

def visualize_pnp_alignment(aligner, detected_keypoints_2d, correspondences=None, 
                           title="PnP Keypoint Alignment", additional_3d_points=None):
    """
    Visualize the PnP keypoint alignment process
    """
    
    # Solve PnP and get aligned keypoints
    aligned_keypoints, pose_info = aligner.align_keypoints_pnp(detected_keypoints_2d, correspondences)
    
    if aligned_keypoints is None:
        print("PnP alignment failed")
        return None
    
    # Project additional 3D points if provided
    projected_additional = None
    if additional_3d_points is not None:
        _, projected_additional = aligner.project_template_to_image(pose_info, additional_3d_points)
    
    # Create visualization
    fig = plt.figure(figsize=(16, 12))
    
    # 2D visualization (main result)
    ax_2d = plt.subplot(2, 2, (1, 2))
    
    # Plot detected keypoints
    ax_2d.scatter(detected_keypoints_2d[:, 0], detected_keypoints_2d[:, 1], 
                  c='red', s=100, marker='o', label='Detected Keypoints', alpha=0.8)
    
    # Plot aligned template keypoints
    ax_2d.scatter(aligned_keypoints[:, 0], aligned_keypoints[:, 1], 
                  c='blue', s=100, marker='s', label='Template (PnP projected)', alpha=0.8)
    
    # Plot additional projected points if any
    if projected_additional is not None:
        ax_2d.scatter(projected_additional[:, 0], projected_additional[:, 1], 
                      c='green', s=80, marker='^', label='Additional 3D Points', alpha=0.8)
    
    # Draw correspondences
    if correspondences is None and len(detected_keypoints_2d) == len(aligned_keypoints):
        correspondences = [(i, i) for i in range(len(detected_keypoints_2d))]
    
    if correspondences:
        for template_idx, detected_idx in correspondences:
            if template_idx < len(aligned_keypoints) and detected_idx < len(detected_keypoints_2d):
                ax_2d.plot([aligned_keypoints[template_idx, 0], detected_keypoints_2d[detected_idx, 0]],
                          [aligned_keypoints[template_idx, 1], detected_keypoints_2d[detected_idx, 1]],
                          'gray', alpha=0.5, linestyle='--', linewidth=1)
    
    # Number the keypoints
    for i, (x, y) in enumerate(detected_keypoints_2d):
        ax_2d.annotate(f'D{i}', (x, y), xytext=(5, 5), textcoords='offset points', 
                       color='red', fontweight='bold')
    
    for i, (x, y) in enumerate(aligned_keypoints):
        ax_2d.annotate(f'T{i}', (x, y), xytext=(5, -15), textcoords='offset points', 
                       color='blue', fontweight='bold')
    
    ax_2d.set_title(f'{title}\nReprojection Error: {pose_info["confidence_score"]:.2f} pixels')
    ax_2d.legend()
    ax_2d.grid(True, alpha=0.3)
    ax_2d.set_xlabel('Image X (pixels)')
    ax_2d.set_ylabel('Image Y (pixels)')
    ax_2d.invert_yaxis()  # Image coordinates convention
    
    # 3D visualization of object pose
    ax_3d = plt.subplot(2, 2, 3, projection='3d')
    
    # Plot template keypoints in world coordinates
    template_3d = aligner.template_keypoints_3d
    ax_3d.scatter(template_3d[:, 0], template_3d[:, 1], template_3d[:, 2], 
                  c='blue', s=100, marker='s', label='Template 3D')
    
    # Plot coordinate frame at estimated pose
    R = pose_info['rotation_matrix']
    t = pose_info['tvec'].flatten()
    
    # Draw coordinate axes (scaled for visibility)
    axis_length = np.max(np.ptp(template_3d, axis=0)) * 0.3
    origin = t
    x_axis = origin + R[:, 0] * axis_length
    y_axis = origin + R[:, 1] * axis_length  
    z_axis = origin + R[:, 2] * axis_length
    
    ax_3d.plot([origin[0], x_axis[0]], [origin[1], x_axis[1]], [origin[2], x_axis[2]], 'r-', linewidth=3, label='X-axis')
    ax_3d.plot([origin[0], y_axis[0]], [origin[1], y_axis[1]], [origin[2], y_axis[2]], 'g-', linewidth=3, label='Y-axis')
    ax_3d.plot([origin[0], z_axis[0]], [origin[1], z_axis[1]], [origin[2], z_axis[2]], 'b-', linewidth=3, label='Z-axis')
    
    # Plot additional 3D points if provided
    if additional_3d_points is not None:
        additional_3d = np.array(additional_3d_points)
        ax_3d.scatter(additional_3d[:, 0], additional_3d[:, 1], additional_3d[:, 2], 
                      c='green', s=80, marker='^', label='Additional 3D Points')
    
    ax_3d.set_title('3D Object Pose')
    ax_3d.legend()
    ax_3d.set_xlabel('X (world)')
    ax_3d.set_ylabel('Y (world)')
    ax_3d.set_zlabel('Z (world)')
    
    # Pose information text
    ax_text = plt.subplot(2, 2, 4)
    ax_text.axis('off')
    
    pose_6dof = aligner.get_pose_6dof(pose_info)
    
    info_text = f"""PnP Pose Estimation Results:

Translation (X, Y, Z):
  {pose_6dof['translation_xyz'][0]:.2f}, {pose_6dof['translation_xyz'][1]:.2f}, {pose_6dof['translation_xyz'][2]:.2f}

Rotation (Roll, Pitch, Yaw):
  {pose_6dof['rotation_rpy_degrees'][0]:.1f}°, {pose_6dof['rotation_rpy_degrees'][1]:.1f}°, {pose_6dof['rotation_rpy_degrees'][2]:.1f}°

Correspondences: {pose_info['n_points']}
Reprojection Error: {pose_info['confidence_score']:.2f} pixels

PnP Method: Iterative
Camera Matrix:
  fx: {aligner.camera_matrix[0,0]:.1f}
  fy: {aligner.camera_matrix[1,1]:.1f}
  cx: {aligner.camera_matrix[0,2]:.1f}
  cy: {aligner.camera_matrix[1,2]:.1f}"""
    
    ax_text.text(0.1, 0.9, info_text, transform=ax_text.transAxes, 
                fontfamily='monospace', fontsize=10, verticalalignment='top')
    
    plt.tight_layout()
    plt.show()
    
    return pose_info

# Example usage
if __name__ == "__main__":
    # Example: 3D template keypoints from your CAD model (in mm or meters)
    template_3d_keypoints = np.array([
        [0, 0, 0],       # Origin corner
        [100, 0, 0],     # X-direction corner
        [0, 50, 0],      # Y-direction corner
        [50, 25, 10],    # Raised center point
        [100, 50, 0],    # Far corner
    ], dtype=np.float32)
    
    # Camera intrinsic parameters (you need to calibrate your stereo camera)
    camera_matrix = np.array([
        [800, 0, 320],   # fx, 0, cx
        [0, 800, 240],   # 0, fy, cy
        [0, 0, 1]        # 0, 0, 1
    ], dtype=np.float32)
    
    # Example detected 2D keypoints in image (pixels)
    detected_2d_keypoints = np.array([
        [150, 200],      # Corresponds to template point 0
        [450, 210],      # Corresponds to template point 1
        [140, 350],      # Corresponds to template point 2
        [300, 280],      # Corresponds to template point 3
        [460, 360],      # Corresponds to template point 4
    ], dtype=np.float32)
    
    # Initialize PnP aligner
    pnp_aligner = PnPKeypointAligner(template_3d_keypoints, camera_matrix)
    
    # Optional: additional 3D points to project (like other features from CAD)
    additional_cad_points = np.array([
        [25, 12.5, 0],   # Additional feature 1
        [75, 37.5, 0],   # Additional feature 2
        [50, 25, 20],    # Higher point
    ])
    
    # Visualize PnP alignment
    pose_result = visualize_pnp_alignment(
        pnp_aligner, 
        detected_2d_keypoints,
        title="PnP-Based Keypoint Alignment",
        additional_3d_points=additional_cad_points
    )
    
    # Get aligned template keypoints for further use
    aligned_keypoints, pose_info = pnp_aligner.align_keypoints_pnp(detected_2d_keypoints)
    
    print(f"\nAligned template keypoints in image coordinates:")
    for i, point in enumerate(aligned_keypoints):
        print(f"  Template point {i}: ({point[0]:.1f}, {point[1]:.1f})")