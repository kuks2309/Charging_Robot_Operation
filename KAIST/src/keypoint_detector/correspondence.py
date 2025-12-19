import numpy as np
import cv2
from scipy.spatial import ConvexHull
from typing import List, Tuple




def rectify_charging_port(image: np.ndarray, yolo_points: List[Tuple[float, float]], 
                          output_width: int = 400, output_height: int = 400) -> Tuple:
    """
    Rectifies an EV charging port region using oriented bounding box and perspective transform.
    
    Args:
        image: Input image (BGR format)
        keypoints: List of (x, y) keypoint coordinates
        output_width: Width of rectified output
        output_height: Height of rectified output
    
    Returns:
        Tuple of (rectified_image, box_points, transform_matrix) or (None, None, None) if failed
    """
    # Validate input
    keypoints = []#np.zeros((len(yolo_points.boxes), 2))
    output_keypoints = []
    for i, box in enumerate(yolo_points.boxes):
        if box.cls.item() in [0,1]:  # Skip certain classes
            continue
        
        x1, y1, x2, y2 = map(int, box.xyxy[0])
        keypoints.append(np.array([(x2+x1)//2, (y2+y1)/2], dtype=np.int32))
        output_keypoints.append(np.array([x1,y1,x2,y2]))
    output_keypoints = np.stack(output_keypoints, axis=0)
    keypoints = np.stack(keypoints, axis=0)
    
    if len(keypoints) < 4:
        print(f"Error: Need at least 3 keypoints, got {len(keypoints)}")
        return None, None, None
    
    # Convert keypoints to numpy array and filter invalid points
    points = np.array(keypoints, dtype=np.float32)
    
    # Remove any NaN or infinite values
    valid_mask = np.all(np.isfinite(points), axis=1)
    points = points[valid_mask]
    if len(points) < 4:
        print("Error: Not enough valid keypoints after filtering")
        return None, None, None
    
    # Fit minimum area rectangle (oriented bounding box)
    # This finds the smallest rotated rectangle that contains all points
    rect = cv2.minAreaRect(points)
    
    # Get the 4 corner points of the minimum area rectangle
    box = cv2.boxPoints(rect)
    box = np.float32(box)
    
    # Get width and height of the oriented bounding box
    width = int(rect[1][0])
    height = int(rect[1][1])
    
    # Ensure width > height (landscape orientation)
    if width < height:
        width, height = height, width
    
    # Order the box points: top-left, top-right, bottom-right, bottom-left
    box_ordered = order_points(box)
    
    # Define destination points for rectified image
    dst_points = np.array([
        [0, 0],
        [output_width - 1, 0],
        [output_width - 1, output_height - 1],
        [0, output_height - 1]
    ], dtype=np.float32)
    
    # Calculate perspective transform matrix
    M = cv2.getPerspectiveTransform(box_ordered, dst_points)
    print(points.shape, M.shape)
    
    # Apply perspective warp
    # rectified = cv2.warpPerspective(image, M, (output_width, output_height))
    transformed_keypoints = cv2.perspectiveTransform(points.reshape(-1, 1, 2), M).reshape(-1, 2)
    transformed_keypoints -= transformed_keypoints.mean(axis=(0))
    err_dist = 0.3
    points = transformed_keypoints    # shape (N, 2)

    x = points[:, 0]
    y = points[:, 1]

    remove_cond = (
        (np.abs(x) < err_dist * output_width) &
        (y > 0)
    )

    filtered_points = points[~remove_cond]
    output_keypoints = output_keypoints[~remove_cond]
    filtered_points -= filtered_points.mean(axis=0)
    return None, filtered_points, output_keypoints, M


def order_points(pts: np.ndarray) -> np.ndarray:
    """
    Orders points in the order: top-left, top-right, bottom-right, bottom-left.
    
    Args:
        pts: Array of 4 points with shape (4, 2)
    
    Returns:
        Ordered points array
    """
    rect = np.zeros((4, 2), dtype=np.float32)
    
    # Sum of coordinates: top-left has smallest sum, bottom-right has largest
    s = pts.sum(axis=1)
    rect[0] = pts[np.argmin(s)]  # Top-left
    rect[2] = pts[np.argmax(s)]  # Bottom-right
    
    # Difference of coordinates: top-right has smallest diff, bottom-left has largest
    diff = np.diff(pts, axis=1)
    rect[1] = pts[np.argmin(diff)]  # Top-right
    rect[3] = pts[np.argmax(diff)]  # Bottom-left
    
    return rect

def dual_matrix_iterative_refinement(template_points, keypoint_points, max_epochs=10):
    """
    Iteratively refine correspondence by removing poorly-matching template features.
    
    R_assignment: (K, T) - which keypoint matches which template
    R_compressed: (K, K) - square permutation for data transformation
    """
    T = len(template_points)  # Full template size
    K = len(keypoint_points)  # Keypoint size (assume K ≤ T)
    
    # Compute full template features
    template_pairwise = template_points[:, np.newaxis, :] - template_points[np.newaxis, :, :]
    template_dists_full = np.sqrt((template_pairwise**2).sum(axis=2))
    
    # Initialize working data
    keypoint_pairwise = keypoint_points[:, np.newaxis, :] - keypoint_points[np.newaxis, :, :]
    current_kp_dists = np.sqrt((keypoint_pairwise**2).sum(axis=2))  # (K, K)
    
    # Track which template points are still active
    active_templates = np.arange(T)
    current_template_size = T
    
    # Global accumulators
    R_global = np.eye(K)
    S_global = np.eye(K)
    
    for epoch in range(max_epochs):
        print(f"\n=== Epoch {epoch} ===")
        print(f"Keypoints: {K}, Active templates: {current_template_size}")
        
        # Get current active template subset
        template_dists_active = template_dists_full[np.ix_(active_templates, active_templates)]
        template_features = template_dists_active.sum(axis=1)  # (current_template_size,)
        
        # Compute keypoint features
        keypoint_features = current_kp_dists.sum(axis=1)  # (K,)
        
        # STEP 1: Feature matching (R step)
        # Build cost matrix
        feature_diff = np.abs(
            template_features[:, np.newaxis] - 
            keypoint_features[np.newaxis, :]
        )  # (current_template_size, K)
        
        print(f"Feature diff shape: {feature_diff.shape}")
        print(f"Feature diff range: [{feature_diff.min():.3f}, {feature_diff.max():.3f}]")
        
        # Hungarian: match keypoints to templates
        from scipy.optimize import linear_sum_assignment
        
        if current_template_size >= K:
            # Standard case: more templates than keypoints
            template_ind, keypoint_ind = linear_sum_assignment(feature_diff)
            # template_ind: which templates got matched (length K)
            # keypoint_ind: which keypoint matches each template (length K)
            
            print(f"Matched templates: {template_ind}")
            print(f"To keypoints: {keypoint_ind}")
            
            # R_assignment: (K, current_template_size)
            R_assignment = np.zeros((K, current_template_size))
            R_assignment[keypoint_ind, template_ind] = 1
            
            # Identify which templates were matched
            matched_template_mask = np.zeros(current_template_size, dtype=bool)
            matched_template_mask[template_ind] = True
            
            # Update active templates (remove unmatched)
            unmatched_count = (~matched_template_mask).sum()
            if unmatched_count > 0:
                print(f"Removing {unmatched_count} unmatched templates")
                active_templates = active_templates[matched_template_mask]
                current_template_size = len(active_templates)
            
            # R_compressed: reorder keypoints to match template order
            # This is a permutation of keypoints
            R_compressed = np.zeros((K, K))
            R_compressed[np.arange(K), keypoint_ind] = 1  # Permutation
            
        else:
            # More keypoints than templates (shouldn't happen with assumption K ≤ T)
            print("WARNING: More keypoints than active templates")
            keypoint_ind, template_ind = linear_sum_assignment(feature_diff.T)
            R_compressed = np.eye(K)  # No change
        
        print(f"R_assignment shape: {R_assignment.shape}")
        print(f"R_compressed shape: {R_compressed.shape}")
        
        # Apply R_compressed to reorder keypoint data
        current_kp_dists = R_compressed @ current_kp_dists @ R_compressed.T
        
        # STEP 2: Geometric consistency (S step)
        # Find permutation that makes diagonal zero
        perm = np.argmin(np.abs(current_kp_dists), axis=1)  # Find zero column per row
        
        print(f"Diagonal before S: {np.diag(current_kp_dists)[:5]}")
        print(f"Permutation S: {perm}")
        
        # Build S_
        S_ = np.zeros((K, K))
        S_[np.arange(K), perm] = 1
        
        # Apply S_ to columns
        current_kp_dists = current_kp_dists @ S_
        
        print(f"Diagonal after S: {np.diag(current_kp_dists)[:5]}")
        
        # Accumulate transformations
        R_global = R_global @ R_compressed
        S_global = S_global @ S_
        
        # STEP 3: Check convergence
        # Compare current features with active template features
        template_features_current = template_dists_full[np.ix_(active_templates, active_templates)].sum(axis=1)
        keypoint_features_current = current_kp_dists.sum(axis=1)
        
        # Check if sizes match and features align
        if current_template_size == K:
            feature_error = np.linalg.norm(template_features_current - keypoint_features_current)
            RS_diff = np.linalg.norm(R_compressed - S_)
            
            print(f"Feature error: {feature_error:.6f}")
            print(f"||R_compressed - S_||: {RS_diff:.6f}")
            
            if feature_error < 1e-3 and RS_diff < 1e-6:
                print("Converged!")
                break
        
        # Check if no progress
        if current_template_size == K and unmatched_count == 0:
            print("No templates removed, checking convergence...")
            if np.allclose(R_compressed, np.eye(K)) and np.allclose(S_, np.eye(K)):
                print("R and S are identity - converged!")
                break
    
    return {
        'R_global': R_global,
        'S_global': S_global,
        'active_templates': active_templates,
        'final_keypoint_dists': current_kp_dists,
        'correspondence': active_templates  # Which original template each keypoint matched
    }


def compute_pairwise_distances(points):
    """Compute symmetric pairwise distance matrix."""
    diff = points[:, np.newaxis, :] - points[np.newaxis, :, :]
    return np.sqrt((diff**2).sum(axis=2))


def pick_min_iteratively(A, n):
    A = A.copy().astype(float)        # avoid modifying original
    picked = []                       # store (row, col, value)

    for _ in range(n):
        i, j = np.unravel_index(np.argmin(A), A.shape)
        val = A[i, j]
        picked.append(i)

        # block row and column by setting to +inf
        A[i, :] = np.inf
        A[:, j] = np.inf

    return picked

def get_correspondence(yolo_points, template_points, img):
    points = []#np.zeros((len(yolo_points.boxes), 2))
    for i, box in enumerate(yolo_points.boxes):
        if box.cls.item() in [0,1]:  # Skip certain classes
            continue
        
        x1, y1, x2, y2 = map(int, box.xyxy[0])
        points.append(np.array([(x2+x1)//2, (y2+y1)/2], dtype=np.int32))
        
    points = np.stack(points, axis=0)
    for p in points:
        cv2.circle(img, p, 3, (255, 255, 255), 1)
    
    rectified, rectified_points, _, M = rectify_charging_port(
        img, points, 
        output_width=300, 
        output_height=300
    )
    
    # dual_matrix_iterative_refinement(template_points, rectified_points, max_epochs=10)
    # input('wokring?')
    n_epochs = 10
    R = np.eye(len(template_points),len(points))
    
    rectified_points[:, 1] = -rectified_points[:, 1]
    template_pairwise_diffs = template_points[:, np.newaxis, :-1] - template_points[np.newaxis, :, :-1] # [index_i, index_j, (x,y) ]
    keypoints_pairwise_diffs = rectified_points[:, np.newaxis, :] - rectified_points[np.newaxis, :, :]
    keypoint_dists = np.linalg.norm(keypoints_pairwise_diffs, axis=-1)
    
    print(keypoint_dists)
    
    # Get upper triangle indices (fexcluding diagonal)
    # upper_tri_indices = np.triu_indices(len(rectified_points), k=1)
    # upper_tri_indices_template = np.tril_indices(len(template_points), k=1)
    
    # # Extract upper triangle values only
    # kp_x_upper = keypoints_pairwise_diffs[upper_tri_indices][:, 0]
    # kp_y_upper = keypoints_pairwise_diffs[upper_tri_indices][:, 1]

    # template_x_upper = template_pairwise_diffs[upper_tri_indices_template][:, 0]
    # template_y_upper = template_pairwise_diffs[upper_tri_indices_template][:, 1]

    # Or using mean (equivalent for ratio):
    scaling_factor = keypoints_pairwise_diffs.median(axis=(0,1)) / template_pairwise_diffs.median(axis=(0,1))
    print(scaling_factor)

    template_features = template_pairwise_diffs.sum(axis=(1,2))
    keypoint_features = keypoints_pairwise_diffs/np.array([[scaling_factor]])
    
    keypoint_features = keypoint_features.sum(axis=(1,2))
    keypoint_size = len(keypoint_features)
    same_temp_diff = template_features[:, np.newaxis] - template_features[np.newaxis, :]
    same_temp_diff_mean = same_temp_diff.mean(axis=1)
    
    R = np.eye(keypoint_size)
    S = np.eye(keypoint_size)
    for _ in range(n_epochs):
        print(template_features.shape, keypoint_features.shape, R)
        feature_diff = template_features[:, np.newaxis] - keypoint_features[np.newaxis, :] #@ R.T
        feature_diff_mean = feature_diff.mean(axis=1)
        print((feature_diff_mean - same_temp_diff_mean).sum(), template_features)
        input('asofijd')
        indices = pick_min_iteratively(feature_diff, keypoint_size)
                
        R_ = np.zeros((len(indices), keypoint_size)) # n_keypoints, n_template_points -> n,7
        R_[np.arange(keypoint_size), indices] = 1
        R = R @ R_
        non_empty_cols = np.any(R_ != 0, axis=0)

        # 2. Select only those columns
        R_compressed = R_[:, non_empty_cols]

        keypoint_dists = R_compressed @ keypoint_dists
        keypoint_features = R_compressed @ keypoint_features
        perm = np.where(keypoint_dists == 0)[1]   # get zero column for each row

        S_ = np.zeros((keypoint_size,keypoint_size))
        S_[np.arange(keypoint_size), perm] = 1
        S = S @ S_
        keypoint_dists = keypoint_dists @ S_
        keypoint_features = keypoint_features @ S_
        # print(R)
        print(S_)
        print(keypoint_dists)
        print(perm)
        print(feature_diff)
        input("wait")

        
    # Display
    # cv2.imshow('Rectification Result', visualization)
    # cv2.imshow('Rectified Port', rectified)
    # cv2.waitKey(1)
    
    return rectified