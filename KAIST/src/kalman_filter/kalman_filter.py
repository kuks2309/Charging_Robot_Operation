import numpy as np
from scipy.spatial.transform import Rotation as R
from scipy.linalg import block_diag
import cv2
from scipy.spatial.transform import Rotation as R, Slerp

class StaticObjectPoseKalmanFilter:
    def __init__(self, initial_pose=None, process_noise_std=1e-4, 
                 position_noise_std=0.01, rotation_noise_std=0.05):
        """
        Kalman Filter for static object pose estimation
        
        State vector: [px, py, pz, qw, qx, qy, qz] (position + quaternion)
        
        Args:
            initial_pose: [x, y, z, qw, qx, qy, qz] or None
            process_noise_std: Process noise for static object
            position_noise_std: Measurement noise std for position (meters)
            rotation_noise_std: Measurement noise std for rotation (radians)
        """
        self.state_dim = 7  # [px, py, pz, qw, qx, qy, qz]
        self.meas_dim = 7   # Same as state for full pose measurement
        
        # State: [px, py, pz, qw, qx, qy, qz]
        if initial_pose is not None:
            self.x = np.array(initial_pose, dtype=float)
        else:
            self.x = np.array([0, 0, 1, 1, 0, 0, 0], dtype=float)  # Default pose
            
        # Normalize quaternion
        self.x[3:7] = self.x[3:7] / np.linalg.norm(self.x[3:7])
        
        # State covariance - start with high uncertainty
        self.P = np.eye(self.state_dim) * 0.1
        
        # Process noise (very small for static object)
        self.Q = np.eye(self.state_dim) * (process_noise_std ** 2)
        
        # Measurement noise
        pos_var = position_noise_std ** 2
        rot_var = rotation_noise_std ** 2
        self.R = block_diag(np.eye(3) * pos_var, np.eye(4) * rot_var)
        
        # State transition (identity for static object)
        self.F = np.eye(self.state_dim)
        
        # Measurement matrix (identity - direct measurement of pose)
        self.H = np.eye(self.meas_dim)
        
        self.initialized = initial_pose is not None

        self.innovation_history = []
        self.max_history = 5
        self.initial_P_trace = np.trace(self.P)  # Store initial uncertainty
        
    def predict(self):
        """Prediction step - minimal change for static object"""
        # State prediction (no change for static object)
        self.x = self.F @ self.x
        
        # Normalize quaternion to maintain unit constraint
        self.x[3:7] = self.x[3:7] / np.linalg.norm(self.x[3:7])
        
        # Covariance prediction
        self.P = self.F @ self.P @ self.F.T + self.Q

    def update(self, measurement, measurement_quality=1.0):
        """
        Update step with pose measurement
        
        Args:
            measurement: [px, py, pz, qw, qx, qy, qz] - measured pose
            measurement_quality: 0-1, affects measurement noise (lower = more noise)
        """
        if measurement is not None:
            z = np.array(measurement, dtype=float)
            z[3:7] = z[3:7] / np.linalg.norm(z[3:7])
            y = z - self.H @ self.x
            if np.dot(self.x[3:7], z[3:7]) < 0:
                z[3:7] = -z[3:7]
                y = z - self.H @ self.x
            
            innovation_magnitude = np.linalg.norm(y)
            self.innovation_history.append(innovation_magnitude)
            if len(self.innovation_history) > self.max_history:
                self.innovation_history.pop(0)
        else:
            return
    
            
        z = np.array(measurement, dtype=float)
        
        # Normalize quaternion in measurement
        z[3:7] = z[3:7] / np.linalg.norm(z[3:7])
        
        # Adapt measurement noise based on quality
        # How current measurement is reliable?
        print("measurement_quality: ", measurement_quality)
        R_adapted = self.R / measurement_quality
        
        # Innovation
        y = z - self.H @ self.x
        
        # Handle quaternion sign ambiguity (q and -q represent same rotation)
        if np.dot(self.x[3:7], z[3:7]) < 0:
            z[3:7] = -z[3:7]
            y = z - self.H @ self.x
            
        # Innovation covariance
        S = self.H @ self.P @ self.H.T + R_adapted
        
        # Kalman gain
        K = self.P @ self.H.T @ np.linalg.inv(S)
        
        # State update
        self.x = self.x + K @ y
        
        # Normalize quaternion after update
        self.x[3:7] = self.x[3:7] / np.linalg.norm(self.x[3:7])
        
        # # ============
        # # --- Position update (standard Kalman) ---
        # self.x[0:3] = self.x[0:3] + (K @ y)[0:3]

        # # --- Rotation update (SLERP instead of linear add) ---
        # q_pred = self.x[3:7]
        # q_meas = z[3:7]
        # alpha = min(0.5, 0.5 * measurement_quality)  # blending factor
        # self.x[3:7] = self.slerp_quaternion(q_pred, q_meas, alpha)
        # # ============
        
        # Covariance update
        I_KH = np.eye(self.state_dim) - K @ self.H
        self.P = I_KH @ self.P
        
        self.initialized = True
        
    def get_convergence_score(self):
        """
        Hybrid convergence metric combining uncertainty reduction and steady-state
        Returns 0.0-1.0, where 1.0 = converged
        """
        # 1. Covariance reduction score
        current_uncertainty = np.trace(self.P)
        converged_uncertainty = 0.01
        
        if current_uncertainty <= converged_uncertainty:
            covariance_score = 1.0
        else:
            # How much has uncertainty reduced from initial?
            reduction_ratio = current_uncertainty / self.initial_P_trace
            covariance_score = 1.0 - reduction_ratio
            covariance_score = np.clip(covariance_score, 0.0, 1.0)
        
        # 2. Innovation steadiness score
        if len(self.innovation_history) < 5:
            innovation_score = 0.0
        else:
            avg_innovation = np.mean(self.innovation_history[-3:])

            innovation_threshold = 0.05
            innovation_score = np.clip(1 - (avg_innovation / innovation_threshold), 0, 1)
        
        # 3. Combined score
        combined = 0.5 * covariance_score + 0.5 * innovation_score
        
        # np.clip(combined, 0.0, 1.0)
        return np.clip(combined, 0.0, 1.0)

    def is_converged(self, threshold=0.85):
        """Check if filter has converged"""
        return self.get_convergence_score() >= threshold

    def get_convergence_status(self):
        """Get detailed convergence information"""
        score = self.get_convergence_score()
        
        if score >= 0.9:
            status = "CONVERGED"
        elif score >= 0.7:
            status = "CONVERGING"
        elif score >= 0.4:
            status = "INITIALIZING"
        else:
            status = "UNCERTAIN"
        
        return {
            'score': score,
            'status': status,
            'uncertainty': np.trace(self.P),
            'is_converged': score >= 0.98
        }

    def get_pose(self, world_pose=None):
        """Get current pose estimate"""
        R_object_to_world = R.from_quat(self.x[[4,5,6,3]]).as_matrix()  # scipy uses [x,y,z,w]
        
        R_ee_obj_to_world = R_object_to_world.copy()
        # # marker
        R_ee_obj_to_world[:, 1] = -R_ee_obj_to_world[:, 1].copy() # 10_14_13_25: camera -> obj
        R_ee_obj_to_world[:, 0] = -R_ee_obj_to_world[:, 0].copy()
        
        t_ee_obj_to_world = self.x[0:3]

        # TODO: Change this back
    
        # standard
        # R_ee_obj_to_world[:, 0] = -R_ee_obj_to_world[:, 0].copy() # 10_14_13_25: camera -> obj
        # R_ee_obj_to_world[:, 2] = -R_ee_obj_to_world[:, 2].copy()
    
        result = {
            # position_object_to_world, rotation_ee_obj_to_world
            'position_object_to_world': t_ee_obj_to_world,#self.x[0:3],
            'quaternion': self.x[3:7],
            'rotation_object_to_world': R_object_to_world,
            'rotation_ee_obj_to_world': R_ee_obj_to_world, 
            'uncertainty': np.sqrt(np.diag(self.P)), # 0.1 > 
            'convergence_score': self.get_convergence_score(),
            'is_converged': self.is_converged(0.98), # actual convergence (marker: 0.99)
            'total_uncertainty': np.trace(self.P)
        }
        
        # Add camera-relative poses if world_pose is provided
        if world_pose is not None:
            # Assuming world_pose contains the camera pose in world coordinates
            if "board_rotation_in_camera" in world_pose and "board_position_in_camera" in world_pose:
                # Standard world-to-camera transformation
                R_world_to_camera = world_pose["board_rotation_in_camera"]
                t_cam_to_world = world_pose["camera_position"].squeeze() # add 13:07
                result["board_rotation_in_camera"] = world_pose["board_rotation_in_camera"]
                result["board_position_in_camera"] = world_pose["board_position_in_camera"]
                result['position_object_to_camera'] = R_world_to_camera @ (self.x[0:3] - t_cam_to_world)
                result['rotation_object_to_camera'] = R_world_to_camera @ R_object_to_world
                # R_world_cam = world_pose["board_rotation_in_camera"]  # This is actually world->camera rotation
                # # R_cam_world = R_world_cam.T  # Camera->world rotation

                # # 2. Transform position
                # t_obj_world = self.x[0:3]  # Object position in world frame
                # t_cam_world = world_pose["camera_position"]  # Camera position in world frame
                # result['position_object_to_camera'] = R_world_cam @ (t_obj_world - t_cam_world)

                # # 3. Transform rotation
                # result['rotation_object_to_camera'] = R_world_cam @ R_object_to_world
            
        return result