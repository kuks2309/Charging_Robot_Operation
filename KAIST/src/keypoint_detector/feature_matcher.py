from collections import defaultdict, deque
import cv2
import numpy as np
import matplotlib.pyplot as plt
from scipy.optimize import linear_sum_assignment
from scipy.spatial.distance import cdist, pdist, squareform

import cv2, numpy as np
from skimage.measure import CircleModel, ransac

from keypoint_detector.correspondence import get_correspondence, rectify_charging_port

# edge 추출
def get_circles(gray):
    edges = cv2.Canny(gray, 50, 150)
    blank_img = np.zeros_like(edges)
    ys, xs = np.nonzero(edges)
    points = np.column_stack((xs, ys))

    # RANSAC 기반 circle fitting
    model_robust, inliers = ransac(points, CircleModel, min_samples=15,
                                residual_threshold=1, max_trials=100)

    xc, yc, r = model_robust.params
    cv2.circle(blank_img, (int(xc), int(yc)), int(r), (255,255,255), 1)
    return blank_img

class ContourDescendant:
    """Class to store information about a contour descendant"""
    def __init__(self, contour_idx, centroid, parent_idx, ellipse, confidence, area):
        self.contour_idx = contour_idx
        self.centroid = centroid
        self.parent_idx = parent_idx
        self.ellipse = ellipse
        self.confidence = confidence
        self.area = area
        
    def get_ellipse_area(self):
        """Calculate the area of the ellipse"""
        a = self.ellipse[1][0] / 2  # Semi-major axis
        b = self.ellipse[1][1] / 2  # Semi-minor axis
        return np.pi * a * b

class ContourInfo:
    """Class to manage contour descendants"""
    def __init__(self, offset=(0,0)):
        self.descendants = []
        self.offset = offset
    
    def add_descendant(self, contour_idx, centroid, parent_idx, ellipse, confidence, area):
        """Add a new descendant"""
        centroid = (centroid[0] + self.offset[0], centroid[1] + self.offset[1])
        e_c, ax, ang = ellipse
        ellipse = ((e_c[0] + self.offset[0], e_c[1] + self.offset[1]), ax, ang)
        descendant = ContourDescendant(contour_idx, centroid, parent_idx, ellipse, confidence, area)
        self.descendants.append(descendant)
        return descendant
    
    def get_areas(self):
        """Get all areas as a list"""
        return [desc.area for desc in self.descendants]

    def get_centroids(self, offset=(0,0)):
        """Get all centroids as a list"""
        return [(desc.centroid[0] + offset[0], desc.centroid[1] + offset[1]) for desc in self.descendants]
    
    def get_ellipse_centroids(self, offset=(0,0)):
        """Get all ellipse centroids"""
        print([x.ellipse[0] for x in self.descendants if x is not None])
        out = []
        for x in self.descendants:
            if x is not None:
                out.append((int(x.ellipse[0][0]+offset[0]), int(x.ellipse[0][1]+offset[1])))
            else:
                out.append(None)
        return out # [(int(x.ellipse[0][0]+offset[0]), int(x.ellipse[0][1]+offset[1])) for x in self.descendants if x is not None]
    
    def get_contour_indices(self):
        """Get all contour indices as a list"""
        return [desc.contour_idx for desc in self.descendants]
    
    def get_parent_indices(self):
        """Get all parent indices as a list"""
        return [desc.parent_idx for desc in self.descendants]
        
    def get_ellipses(self, offset=None):
        """Get all ellipses as a list, with optional centroid offset"""
        if offset is None:
            dx, dy = 0, 0
        else:
            dx, dy = offset

        ellipse_offset = []
        for desc in self.descendants:
            if desc.ellipse[1] is not None:
                (cx, cy), (w, h), angle = desc.ellipse
                new_center = (int(cx + dx), int(cy + dy))
                ellipse_offset.append((new_center, (w, h), angle))

        return ellipse_offset
    
    def remove_all_except_list(self, lst):
        """Remove all the descendants except the list index"""
        self.descendants = [x for i,x in enumerate(self.descendants) if i in lst]
        
    
    def remove_descendant(self, index):
        """Remove a descendant by index"""
        if 0 <= index < len(self.descendants):
            return self.descendants.pop(index)
        return None
    
    def insert_descendant(self, index, descendant):
        """Insert a descendant at a specific index"""
        self.descendants.insert(index, descendant)
    
    def get_valid_indices(self):
        out = []
        for i,d in enumerate(self.descendants):
            if d is not None:
                out.append(i)
        return out
    
    def get_best_confidence_cnt(self):
        """Returns the ContourDescendant with the biggest confidence if it exists"""
        best_confidence = float('-inf')
        best_confidence_idx = -1
        for i, desc in enumerate(self.descendants):
            confidence = desc.confidence
            if confidence > best_confidence:
                best_confidence_idx = i
                best_confidence = confidence
        print("Best confidence", best_confidence, best_confidence_idx)
        return best_confidence_idx, best_confidence
    
    def find_by_centroid(self, target_centroid, tolerance=10):
        """Find descendants with centroids within tolerance of target_centroid"""
        target_centroid = (target_centroid[0] + self.offset[0], target_centroid[1] + self.offset[1])
        matches = []
        for i, desc in enumerate(self.descendants):
            distance = np.linalg.norm(np.array(desc.centroid) - np.array(target_centroid))
            if distance <= tolerance:
                matches.append(i)
        return matches
    
    def _get_pairwise_distances(self, centroids=None):
        
        """Get pairwise distances between centroids"""
        def normalize_points(points):
            centroid = np.mean(points, axis=0)
            points_centered = points - centroid
            scale = np.mean(np.linalg.norm(points_centered, axis=1))
            points_normalized = points_centered / scale
            return points_normalized
        if centroids is None:
            centroids = np.array(self.get_centroids()) # [N, 2]
        
        centroids = normalize_points(centroids)
        if len(centroids) < 4:
            return np.array([])
        # dists = np.linalg.norm(centroids[:, np.newaxis] - centroids, axis=2)
        return squareform(pdist(centroids))
        

    def _simple_centroid_matching(self, template_centroids, max_distance=50):
        """Fallback method: simple nearest neighbor matching by centroid distance
        
        Args:
            template_centroids: Nx2 array of template centroid coordinates
            max_distance: Maximum allowed distance for a valid match
        
        Returns:
            List of tuples (template_idx, image_idx)
        """
        if len(template_centroids) == 0 or len(self.descendants) == 0:
            return []
        
        image_centroids = np.array(self.get_centroids())
        
        # Compute pairwise distances between all template and image centroids
        from scipy.spatial.distance import cdist
        distances = cdist(template_centroids, image_centroids)
        
        # Use Hungarian algorithm on distance matrix
        row_indices, col_indices = linear_sum_assignment(distances)
        
        # Filter by maximum distance threshold
        valid_matches = []
        for template_idx, image_idx in zip(row_indices, col_indices):
            if distances[template_idx, image_idx] <= max_distance:
                valid_matches.append((template_idx, image_idx))
        
        return valid_matches

    def __len__(self):
        return len(self.descendants)
    
    def __getitem__(self, idx):
        """Allow indexing: descendants_info[0]"""
        return self.descendants[idx]


def get_contour_centroid(contour):
    M = cv2.moments(contour)
    if M['m00'] == 0:
        return None
    cx = int(M['m10'] / M['m00'])
    cy = int(M['m01'] / M['m00'])
    return (cx, cy)

def get_ellipse_area(ellipse):
    a = ellipse[1][0] / 2  # Semi-major axis (first element of axes)
    b = ellipse[1][1] / 2  # Semi-minor axis (second element of axes)
    area = np.pi * a * b
    return area

def percentage_on_ellipse(ellipse, cnt):
    (x0, y0), (MA, ma), angle = ellipse
    theta = np.deg2rad(angle)

    # Sample points along the ellipse perimeter
    num_points = 100
    t = np.linspace(0, 2*np.pi, num_points)
    ellipse_pts = np.zeros((num_points, 2))

    for i in range(num_points):
        x = (MA/2) * np.cos(t[i])
        y = (ma/2) * np.sin(t[i])
        # rotate and translate
        x_rot = x * np.cos(theta) - y * np.sin(theta) + x0
        y_rot = x * np.sin(theta) + y * np.cos(theta) + y0
        ellipse_pts[i] = [x_rot, y_rot]

    # Check which ellipse points are inside the contour
    inside_mask = np.array([cv2.pointPolygonTest(cnt, tuple(pt), False) >= 0 for pt in ellipse_pts])

    return np.sum(inside_mask) / num_points

def calculate_solidity(contour):
    """
    Solidity = contour_area / convex_hull_area
    Circles/ellipses have high solidity (close to 1.0)
    """
    area = cv2.contourArea(contour)
    hull = cv2.convexHull(contour)
    hull_area = cv2.contourArea(hull)
    if hull_area == 0:
        return 0
    return area / hull_area

def calculate_extent(contour):
    """
    Extent = contour_area / bounding_rect_area
    Circles have extent around 0.785 (π/4)
    """
    area = cv2.contourArea(contour)
    x, y, w, h = cv2.boundingRect(contour)
    rect_area = w * h
    if rect_area == 0:
        return 0
    return area / rect_area

def is_circle_like(contour, ellipse, min_points=10):
    """
    Comprehensive check if contour is circle/ellipse-like
    Returns (is_valid, confidence_score, metrics_dict)
    """
    if len(contour) < min_points:
        return False, 0.0, {}
    
    # Basic geometric metrics
    area = cv2.contourArea(contour)
    perimeter = cv2.arcLength(contour, True)
    
    if area == 0 or perimeter == 0:
        return False, 0.0, {}
    
    # 1. Circularity: 4π*area/perimeter² (1.0 = perfect circle)
    circularity = 4 * np.pi * area / (perimeter ** 2)
    
    # 2. Solidity: area/convex_hull_area (high for convex shapes)
    solidity = calculate_solidity(contour)
    
    # 3. Extent: area/bounding_rect_area (~0.785 for circles)
    extent = calculate_extent(contour)
    
    # 4. Aspect ratio of bounding rectangle (close to 1.0 for circles)
    x, y, w, h = cv2.boundingRect(contour)
    aspect_ratio = float(w) / h if h != 0 else 0
    if aspect_ratio < 1:
        aspect_ratio = 1 / aspect_ratio
    
    # 5. Ellipse fit quality
    ellipse_area = get_ellipse_area(ellipse)
    area_ratio = min(area, ellipse_area) / max(area, ellipse_area) if max(area, ellipse_area) > 0 else 0
    
    # 6. Ellipse axis ratio (1.0 = circle, higher = elongated)
    ellipse_ratio = ellipse[1][0] / ellipse[1][1] if ellipse[1][1] != 0 else 0
    if ellipse_ratio < 1:
        ellipse_ratio = 1 / ellipse_ratio
    
    # 7. F1 score for ellipse fit
    f1_score = ellipse_f1(contour.squeeze(), ellipse)
    
    # 8. Percentage of contour points on ellipse
    pct_on_ellipse = percentage_on_ellipse(ellipse, contour)
    
    # 9. Convexity defects (circles should have very few/small defects)
    # hull = cv2.convexHull(contour, returnPoints=False)
    # if len(hull) > 3 and len(contour) > 3:
    #     try:
    #         defects = cv2.convexityDefects(contour, hull)
    #         if defects is not None:
    #             # Normalize defect depths by contour size
    #             max_defect = np.max(defects[:, 0, 3]) / 256.0  # depth in pixels
    #             avg_defect = np.mean(defects[:, 0, 3]) / 256.0
    #         else:
    #             max_defect = 0
    #             avg_defect = 0
    #     except:
    #         max_defect = 0
    #         avg_defect = 0
    # else:
    #     max_defect = 0
    #     avg_defect = 0
    
    # Normalize defects (smaller is better for circles)
    # defect_score = 1.0 / (1.0 + avg_defect / 10.0)  # Normalize
    
    metrics = {
        'circularity': circularity,
        'solidity': solidity,
        'extent': extent,
        'aspect_ratio': aspect_ratio,
        'area_ratio': area_ratio,
        'ellipse_ratio': ellipse_ratio,
        'f1_score': f1_score,
        'pct_on_ellipse': pct_on_ellipse,
        # 'defect_score': defect_score,
        'avg_defect': 0,#avg_defect,
        'max_defect': 0,#max_defect,
        'area': area,
    }
    
    # Scoring weights (tune these based on your needs)
    weights = {
        'circularity': 1.0,      # Most important
        'solidity': 1.0,
        'f1_score': 3.0,         # How well ellipse fits
        'pct_on_ellipse': 5.5,
        'area_ratio': 1.0,
        'defect_score': 0.0,
        'ellipse_ratio': 0.0,    # Less weight for aspect ratio
        'aspect_ratio': 0.5,
        'extent': 0.5
    }
    
    # Calculate weighted confidence score
    scores = []
    
    # Circularity: should be high (0.7-1.0 for circles)
    if 0.7 <= circularity <= 1.0:
        scores.append(weights['circularity'] * circularity)
    elif circularity > 0.5:
        scores.append(weights['circularity'] * circularity * 0.5)
    
    # Solidity: should be > 0.85 for circles
    if solidity >= 0.85:
        scores.append(weights['solidity'] * solidity)
    elif solidity >= 0.75:
        scores.append(weights['solidity'] * solidity * 0.7)
    
    # F1 score: should be high
    if f1_score >= 0.6:
        scores.append(weights['f1_score'] * f1_score)
    
    # Percentage on ellipse: should be high
    if pct_on_ellipse >= 0.6:
        scores.append(weights['pct_on_ellipse'] * pct_on_ellipse)
    
    # Area ratio: should be close to 1.0
    if area_ratio >= 0.8:
        scores.append(weights['area_ratio'] * area_ratio)
    
    # Defect score
    # scores.append(weights['defect_score'] * defect_score)
    
    # Ellipse ratio: 1.0 for circles, allow up to ~1.3 for slight ellipses
    if 0.5 <= ellipse_ratio <= 1.5:
        ellipse_score = 1.0 - abs(ellipse_ratio - 1.0) / 1.0
        scores.append(weights['ellipse_ratio'] * ellipse_score)
    
    # Aspect ratio: should be close to 1.0
    if 0.6 <= aspect_ratio <= 1.4:
        aspect_score = 1.0 - abs(aspect_ratio - 1.0) / 1.0
        scores.append(weights['aspect_ratio'] * aspect_score)
    
    # Extent: circles are ~0.785, allow 0.7-0.85
    if 0.5 <= extent <= 0.85:
        scores.append(weights['extent'] * (extent / 0.785))
    
    # Calculate final confidence (0-1 range)
    total_weight = sum(weights.values())
    confidence = sum(scores) / total_weight if total_weight > 0 else 0
    
    # Hard thresholds to reject obvious non-circles
    is_valid = (
        # circularity >= 0.3 and
        # solidity >= 0.3 and
        f1_score >= 0.4 and
        pct_on_ellipse >= 0.3
        # ellipse_ratio <= 1.5
        # avg_defect < 20  # pixels
    )
    return True, confidence, metrics    
    # return is_valid, confidence, metrics


def preprocess_with_yolo(img, full_image, contour_finder, yolo_results, template_np, offset=(0,0),
                         descendants_info=None, expansion_factor=6):
    """
    Preprocess image to detect circles using YOLO bounding boxes with moving average.
    
    Args:
        img: Input BGR image
        contour_finder: Edge detection processor
        yolo_results: YOLO results object
        template_np: Template array with 7 reference points
        descendants_info: Previous ContourInfo for moving average (None for first frame)
        expansion_factor: Pixels to expand YOLO boxes
    
    Returns:
        descendants_info: ContourInfo with circles at fixed indices (0-6)
        correspondences: List of (template_idx, detection_order) for valid detections
    """
    cv2.imshow("edges", img)
    # img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY) if len(img.shape) == 3 else img 
    
    h, w, _ = img.shape
    
    blank_img = np.zeros((h, w, 3)) # full_image.copy() #
    
    # get_correspondence(yolo_results, template_np, blank_img)
    _, rectified_points, normal_points, _ = rectify_charging_port(img, yolo_results, w, h)
    
    r_points = rectified_points.copy()
    r_points[:, 0] -= r_points[:, 0].min(axis=0)
    r_points[:, 1] -= r_points[:, 1].min(axis=0)
    for i, p in enumerate(r_points):
        int_p = (int(p[0]), int(p[1]))
        cv2.circle(blank_img, int_p, 6, (255,255,255), 3)
        cv2.putText(
            blank_img,                   # image (BGR)
            str(i),             # text
            int_p,             # bottom-left corner (x, y)
            cv2.FONT_HERSHEY_SIMPLEX,  # font
            1.0,                  # font scale
            (255, 255, 255),          # color (BGR): green
            2,                    # thickness
            cv2.LINE_AA           # anti-aliased line
        )
    xs = template_np[:, 0]
    ys = template_np[:, 1]

    W  = abs(xs.max() - xs.min())
    H = abs(ys.max() - ys.min())

    # Normalize template
    relative_template = template_np.copy()
    relative_template = relative_template - relative_template.mean(axis=0)
    relative_template[:, 0] = relative_template[:, 0] / W
    relative_template[:, 1] = relative_template[:, 1] / H # y+ is up for the template
    relative_template = relative_template[:, :2]

    # Check YOLO detections
    if yolo_results is None or len(yolo_results.boxes) == 0:
        print("No YOLO detections found!")
        return descendants_info if descendants_info else ContourInfo(), []
    
    print(f"Found {len(yolo_results.boxes)} YOLO detections")
    
    # Initialize or reuse descendants_info
    # if descendants_info is None:
    descendants_info = ContourInfo()
    
    # Match YOLO boxes to template indices
    correspondence_map = {}
    boxes = []
    
    for i, box in enumerate(normal_points):
        
        x1, y1, x2, y2 = box
        c_x = (x1 + x2) / 2
        c_y = (y1 + y2) / 2
        
        p = rectified_points[i]
        centered_c_x = p[0]/w
        centered_c_y = p[1]/h
        
        # Convert to relative coordinates
        # centered_c_x = (c_x - w/2) / w2
        # centered_c_y = (c_y - h/2) / h2
        c = np.array((centered_c_x, centered_c_y))
        
        # Determine possible template indices based on position
        poss_template_idx = list(range(7))
        
        # Filter by horizontal position
        if centered_c_x < 0:
            poss_template_idx = [x for x in poss_template_idx if x not in [1, 4, 6]] 
        else:
            poss_template_idx = [x for x in poss_template_idx if x not in [0, 3, 5]]
        
        # Filter by vertical position
        if centered_c_y >= 0:
            poss_template_idx = [x for x in poss_template_idx if x not in [2, 3, 4, 5, 6]] 
        else:
            poss_template_idx = [x for x in poss_template_idx if x not in [0, 1]] 
        
        # Find closest template point
        poss_points = relative_template[poss_template_idx].copy()
        dists = np.linalg.norm(poss_points - c, ord=2, axis=1)
        dist_i = np.argmin(dists, axis=0)
        chosen_template_idx = poss_template_idx[dist_i]
        print(dists, dist_i, chosen_template_idx, poss_template_idx)
        # input('templates')
        # Keep best match for each template index
        if chosen_template_idx not in correspondence_map:
            correspondence_map[chosen_template_idx] = (len(boxes), dists[dist_i])
            # input("new")
        else:
            # input('not new')
            if dists[dist_i] < correspondence_map[chosen_template_idx][1]: 
                print(dists[dist_i], correspondence_map[chosen_template_idx][1])
                # input('better')
                correspondence_map[chosen_template_idx] = (len(boxes), dists[dist_i])
        
        # Expand box for search region
        x1_exp = max(0, x1 - expansion_factor)
        y1_exp = max(0, y1 - expansion_factor)
        x2_exp = min(w, x2 + expansion_factor)
        y2_exp = min(h, y2 + expansion_factor)
        xyxy2 = (x1_exp, y1_exp, x2_exp, y2_exp)
        
        boxes.append((xyxy2, (int(c_x), int(c_y))))
    
    print("Correspondence map:", correspondence_map)
    
    # Process each matched template index
    img2 = contour_finder.process_img(img, True)
    # cv2.imshow("imgmg", region)
        #     contours, hierarchy = cv2.findContours(region, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        # local_offset = np.array([[[x1, y1]]])
        
        # cv2.drawContours(blank_img, contours, -1, (100,100,100), 2)
    blank_region = np.zeros_like(img2)
        
    for template_idx in range(7):#sorted(correspondence_map.keys()):
        if template_idx not in correspondence_map.keys():
            descendants_info.descendants.append(None)
            
            continue
        box, (c_x, c_y) = boxes[correspondence_map[template_idx][0]]
        x1, y1, x2, y2 = box

        
        # Extract and process region
        cv2.putText(blank_img, str(template_idx), (c_x, c_y), cv2.FONT_HERSHEY_SIMPLEX,
            1.0, (0, 255, 0), 2, cv2.LINE_AA)
        
        region = img2[y1:y2, x1:x2]
        blank_region = np.zeros_like(region)
        h_r, w_r = region.shape[:2]
        print(f"box ({x1}, {y1}, {x2}, {y2}), region shape {region.shape}")

        contours, hierarchy = cv2.findContours(region, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        local_offset = np.array([[[x1, y1]]])
        
        if hierarchy is None or len(hierarchy) == 0:
            continue
        
        hierarchy = hierarchy[0]
        total_offset = (offset[0]+x1, offset[1]+y1) # offset #
        
        h, w = img.shape[:2]

        

        filtered = []
        for c in contours:
            # c is shaped (N,1,2)
            pts = c.reshape(-1, 2)

            # Check if any point lies on border
            on_border = (
                np.any(pts[:,0] <= 0) or
                np.any(pts[:,1] <= 0) or
                np.any(pts[:,0] >= w_r-1) or
                np.any(pts[:,1] >= h_r-1)
            )
            if not on_border and len(pts) > 10:
                # filtered.append(c)
                filtered.append(c)
        if len(filtered) > 0:
        
            # top_parent = np.vstack(filtered)
            top_parent = max(
                [c for i, c in enumerate(filtered) if hierarchy[i][3] == -1],
                key=cv2.contourArea
            )
            contours_info, f_l, s_l = get_descendants2([top_parent], hierarchy, total_offset, return_top=True)
            
        else:
            contours_info = None
        
        # top_parent = max(contours, key=lambda c: cv2.arcLength(c, False))

        # if len(top_parent) > 5:
        # cv2.drawContours(blank_region, top_parent, -1, (255,255,255), -1)
        # cv2.imshow(f"{template_idx}_region", blank_region)
        # ell = cv2.fitEllipse(filtered)
        # ell = ((int(ell[0][0]+x1), int(ell[0][1]+y1)), ell[1], ell[2])
        # cv2.drawContours(blank_region, [top_parent + np.array((x1,y1))], -1, (255,255,0), 2)
        # cv2.ellipse(blank_region, ell, color=(255,255,255),thickness=1)
        
        # if template_idx == 3:
        #     input("wait")
        # if template_idx == 3:
        #     input(f"num on contours in 3 is {len(contours_info)}")
        # # Draw debug visualization
        # for c in f_l:
        #     cnt = c.copy() + total_offset
        #     cv2.drawContours(blank_img, [cnt], -1, (0, 0, 255), 2)
        # if is_removed and contours_info is None:
        #     input("wait")
        if contours_info is not None:
            print("len", len(contours_info))
        if contours_info is not None and len(contours_info) > 0:
            # Get the best detection
            print("before descendants")
            best_desc = contours_info.descendants[0]
            print("after descendants", len(contours_info))

            # cnt_idx = best_desc.contour_idx
            # print("after_cnt_idx")
            # cnt = contours[cnt_idx].copy() + total_offset
            # print("after cnt")
            
            # cv2.drawContours(full_image, [cnt], -1, (255, 0, 0), 2)
            
            ellipses = contours_info.get_ellipses()
            for ellipse in ellipses:
                print("Ellipse here")
                cv2.ellipse(full_image, ellipse, (255,255,255), 2)
                cv2.putText(full_image, str(template_idx), ellipse[0],
                    cv2.FONT_HERSHEY_SIMPLEX, 1.5, (0, 255, 0), 3, cv2.LINE_AA)
            # Update descendants at fixed template index
            descendants_info.descendants.append(contours_info.descendants[0])
            # input(f"got, {template_idx}")
        else:
            print("Failing here?")
            descendants_info.descendants.append(None)
        cv2.imshow(f"Contour", blank_region)

    # if len(descendants_info.descendants) > 0:
    #     print("Waht", len(descendants_info.descendants), descendants_info.descendants)
        
    #     for template_idx in range(7):
    #         print("Descendant:", descendants_info.get_centroid_at_index(template_idx, smoothed=True))
    #         center = descendants_info.get_centroid_at_index(template_idx, smoothed=True)
    #         print("center", center)
    #         if center is not None:
    #             cv2.putText(blank_img, str(template_idx), center,
    #                     cv2.FONT_HERSHEY_SIMPLEX, 1.5, (0, 255, 0), 3, cv2.LINE_AA)
    
    cv2.circle(blank_img, (w//2, h//2), 3, (255,0,0), 3)
    # Build correspondences list (template_idx, detection_order)
    correspondences = [(x,x) for x in descendants_info.get_valid_indices()]#[(idx, idx) for idx in range(len(descendants_info)) if descendants_info.descendants[idx] is not None]
    
    cv2.imshow("YOLO-guided detection", blank_img)
    cv2.waitKey(1)
    
    if len(descendants_info) == 0:
        print("No circles detected in YOLO regions")
        return descendants_info, []
    
    print(f"Total: {len(descendants_info)} circles detected")
    print(f"Valid indices: {descendants_info.get_valid_indices()}")
    print("Correspondences:", correspondences)
    
    return descendants_info, correspondences

# Removed match_and_order_centroids - no longer needed since ordering is based on regions

import cv2
import numpy as np

def find_circular_holes(binary_img):
    """
    binary_img: uint8 image, 0/255
    returns: list of (cx, cy, r) for detected holes
    """

    # 1. OPTIONAL: clean speckle noise
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
    clean = cv2.morphologyEx(binary_img, cv2.MORPH_OPEN, kernel, iterations=1)
    clean = cv2.morphologyEx(clean, cv2.MORPH_CLOSE, kernel, iterations=1)

    # If holes are dark and background is bright, invert
    # clean = 255 - clean

    # 2. Find contours
    contours, _ = cv2.findContours(clean, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)

    circles = []

    for cnt in contours:
        if len(cnt) < 15:
            continue  # too few points to be meaningful

        area = cv2.contourArea(cnt)
        if area < 20:   # discard tiny blobs (but not based on absolute radius)
            continue

        perimeter = cv2.arcLength(cnt, True)
        if perimeter == 0:
            continue

        # --- Shape metrics (scale-invariant) ---
        # Circularity: 1.0 for perfect circle
        circularity = 4 * np.pi * area / (perimeter ** 2)

        # Fit circle and check how well points lie on it
        (cx, cy), radius = cv2.minEnclosingCircle(cnt)
        radius = float(radius)
        if radius <= 0:
            continue

        pts = cnt.reshape(-1, 2).astype(np.float32)
        dists = np.sqrt((pts[:, 0] - cx) ** 2 + (pts[:, 1] - cy) ** 2)
        # deviation from ideal circle, normalized by radius
        dev = np.abs(dists - radius) / radius
        mean_dev = np.mean(dev)

        # --- Thresholds: tune these a bit for your images ---
        # circularity close to 1, mean deviation small
        if circularity > 0.7 and mean_dev < 0.25:
            circles.append((cx, cy, radius))

    return circles


def get_descendants2(contours, hierarchy, offset=(0,0), return_top=False):
    """
    Improved version that specifically detects circle/ellipse-like shapes
    and filters out other geometric shapes
    """
    contours_info = ContourInfo(offset)
    failed_list = []
    success_list = []

    # Pre-calculate areas and filter out obviously wrong contours
    valid_candidates = []
    for idx, (cnt, hier) in enumerate(zip(contours, hierarchy)):
        if len(cnt) < 25:
            failed_list.append(cnt)
            continue
        area = cv2.contourArea(cnt)
        
        valid_candidates.append((idx, cnt, hier, area))
    
    for idx, cnt, hier, area in valid_candidates:
        try:
            ellipse = cv2.fitEllipse(cnt)
        except:
            failed_list.append(cnt)
            print("Failed 3")
            continue
        
        # Use comprehensive circle/ellipse detection
        is_valid, confidence, metrics = is_circle_like(cnt, ellipse)
        
        # Decision criteria: use confidence score + area consistency
        min_confidence = 0.15 # Adjust this threshold (0-1)
        if (is_valid and confidence >= min_confidence):# and 
            # area_deviation < max_area_deviation):
            
            centroid = get_contour_centroid(cnt)
            if centroid is None:
                failed_list.append(cnt)
                continue
                
            same_centroids = contours_info.find_by_centroid(centroid, tolerance=15)

            if len(same_centroids) > 0:
                existing_idx = same_centroids[0]
                existing_desc = contours_info.descendants[existing_idx]
                
                # Get existing contour's metrics
                # existing_cnt = contours[existing_desc.contour_idx]
                existing_confidence = existing_desc.confidence
                # _, existing_confidence, _ = is_circle_like(existing_cnt, existing_ellipse)
                
                # Compare confidence scores
                if confidence > existing_confidence * 1.05:  # 5% better
                    print(f"  -> REPLACED (better confidence: {existing_confidence:.3f} -> {confidence:.3f})")
                    contours_info.remove_descendant(existing_idx)
                    contours_info.add_descendant(idx, centroid, hier[-1], ellipse, confidence, metrics['area'])
                    success_list.append(cnt)
                else:
                    print(f"  -> REJECTED (duplicate, worse confidence)")
                    failed_list.append(cnt)
            else:
                print(f"  -> ACCEPTED")
                success_list.append(cnt)
                contours_info.add_descendant(idx, centroid, hier[-1], ellipse, confidence, metrics['area'])
        else:
            if not is_valid:
                print(f"  -> REJECTED (failed hard thresholds)")
            elif confidence < min_confidence:
                print(f"  -> REJECTED (low confidence: {confidence:.3f} < {min_confidence})")
            failed_list.append(cnt)

    # # Post-processing: Remove spatial outliers
    if return_top and len(contours_info.descendants) > 0:
        best_i, val = contours_info.get_best_confidence_cnt()

        contours_info.remove_all_except_list([best_i])
    elif len(contours_info.descendants) > 3:
        centroids = np.array(contours_info.get_centroids())
        areas = np.array(contours_info.get_areas())
        centroid_mean = np.mean(centroids, axis=0)
        areas_mean = np.mean(areas, axis=0)
        distances = np.linalg.norm(centroids - centroid_mean, axis=1)
        median_dist = np.median(distances)
        
        # Remove contours that are too far from the cluster
        outliers = []
        for i, (dist, a) in enumerate(zip(distances, areas)):
            if dist > median_dist * 1.2 or a > areas_mean * 1.5:  # More than 1.2x median distance
                outliers.append(i)
        
        for i in reversed(outliers):
            removed = contours_info.remove_descendant(i)
            if removed:
                failed_list.append(contours[removed.contour_idx])
                print(f"\nRemoved spatial outlier at index {i} (dist: {distances[i]:.1f}px)")

    print(f"\n=== Summary ===")
    print(f"Total candidates: {len(valid_candidates)}")
    print(f"Accepted: {len(success_list)}")
    print(f"Rejected: {len(failed_list)}")

    return contours_info, failed_list, success_list

def get_descendants(idx, contours, hierarchy, contours_info=None, failed_list=None, success_list=None):
    if contours_info is None:
        contours_info = ContourInfo()

    if failed_list is None:
        failed_list = []
    if success_list is None:
        success_list = []

    child_idx = hierarchy[idx][2]  # first child

    while child_idx != -1:
        cnt = contours[child_idx]
        if len(cnt) > 30:
            ellipse = cv2.fitEllipse(contours[child_idx])
            f1_ellipse = ellipse_f1(cnt.squeeze(), ellipse)
            pct = percentage_on_ellipse(ellipse, cnt)
            ratio_ellipse = ellipse[1][0] / ellipse[1][1] if ellipse[1][1] != 0 else 0
            # area = cv2.contourArea(cnt)
            print(f"Pct: {pct}, Ratio: {ratio_ellipse}, F1: {f1_ellipse}")
            if (pct >= 0.3 and 0.8 <= ratio_ellipse <= 1.2 and f1_ellipse >= 0.4): 
            # if ( f1_ellipse >= 0.5 
                # and circularity >=0.5
                # and convexity >= 0.7):
                centroid = get_contour_centroid(cnt)
                same_centroids = contours_info.find_by_centroid(centroid)

                if len(same_centroids) > 0:
                    # Get the existing descendant for comparison
                    existing_idx = same_centroids[0]
                    existing_desc = contours_info.descendants[existing_idx]
                    
                    p_ellipse = cv2.fitEllipse(contours[existing_desc.contour_idx])
                    p_pct = percentage_on_ellipse(p_ellipse, contours[existing_desc.contour_idx])
                    f1_p_ellipse = ellipse_f1(contours[existing_desc.contour_idx].squeeze(), p_ellipse)
                    area_p_ellipse = get_ellipse_area(p_ellipse)
                    area_ellipse = get_ellipse_area(ellipse)

                    if pct > p_pct:
                        print("Better ellipse found, replacing...", existing_idx, "with", child_idx)
                        # Remove the old descendant
                        contours_info.remove_descendant(existing_idx)
                        # Add the new better one
                        contours_info.add_descendant(child_idx, centroid, hierarchy[child_idx][-1], ellipse)
                        success_list.append(cnt)
                        print("accepted")
                    else:
                        print("rejected")
                        failed_list.append(cnt)
                else:
                    print("accepted")
                    success_list.append(cnt)
                    contours_info.add_descendant(child_idx, centroid, hierarchy[child_idx][-1], ellipse)
            else:
                print("rejected")
                failed_list.append(cnt)        
        else:
            failed_list.append(cnt)

        contours_info, failed_list, success_list = get_descendants(child_idx, contours, hierarchy, contours_info, failed_list, success_list)
        child_idx = hierarchy[child_idx][0]

    return contours_info, failed_list, success_list

def calculate_convexity(contour):
    """Calculate convexity: area/convex_hull_area
    Perfect convex shape = 1.0
    """
    area = cv2.contourArea(contour)
    hull = cv2.convexHull(contour)
    hull_area = cv2.contourArea(hull)
    if hull_area == 0:
        return 0
    return area / hull_area

def preprocess_with_centroids(img, predicted_centroids=None, expansion_factor=1.5):
    """
    Preprocess image to detect circles with optional centroid predictions.
    
    Args:
        img: Input image
        predicted_centroids: List of (x, y) tuples from CV model predictions. 
                           If None, uses original 7-section approach
        expansion_factor: How much to expand the search box around predicted centroids
    
    Returns:
        descendants_info, correspondences, all_hierarchies, all_contours
    """
    image = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY) 
    blank_img = np.zeros_like(img)
    
    image = cv2.GaussianBlur(image, (7, 7), 0)
    image = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8,8)).apply(image)

    edges = cv2.adaptiveThreshold(
        image, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
        cv2.THRESH_BINARY_INV, 15, 3
    )

    h, w = edges.shape
    
    # If centroids are provided, create custom regions around them
    if predicted_centroids is not None and len(predicted_centroids) > 0:
        regions = create_regions_from_centroids(
            predicted_centroids, h, w, expansion_factor
        )
    else:
        # Use original 7-section approach
        regions = get_default_regions(h, w)
    
    descendants_info = ContourInfo()
    valid_contours_mask = np.zeros_like(image)
    all_contours = []
    all_hierarchies = []
    correspondences = []
    
    # Process each region
    for i, (x_start, x_end, y_start, y_end) in enumerate(regions):
        # Extract region
        region = edges[y_start:y_end, x_start:x_end]
        
        # Find contours in this region
        contours, hierarchy = cv2.findContours(
            region, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE
        )
        
        if hierarchy is not None and len(hierarchy) > 0:
            hierarchy = hierarchy[0]
            offset = (x_start, y_start)
            
            # Get descendants with offset
            d_i, failed_list, success_list = get_descendants2(
                contours, hierarchy, offset, return_top=True
            )
            
            print(f"Region {i}: Found {len(d_i)} descendants")
            
            descendants_info.descendants += d_i.descendants
            if len(d_i) > 0:
                correspondences.append((i, len(correspondences)))
            
            # Draw contours
            cv2.drawContours(
                blank_img[y_start:y_end, x_start:x_end], 
                contours, -1, color=(255,255,255), thickness=1
            )
            cv2.drawContours(
                blank_img[y_start:y_end, x_start:x_end], 
                success_list, -1, color=(255,0,0), thickness=2
            )
        
        all_contours += contours
        all_hierarchies.append(hierarchy)
    
    if len(all_hierarchies) > 0:
        all_hierarchies = np.concatenate(all_hierarchies, axis=1)
    
    cv2.imshow("contours_with_centroids.jpg", blank_img)
    
    return descendants_info, correspondences, all_hierarchies, all_contours


def create_regions_from_centroids(centroids, img_h, img_w, expansion_factor):
    """
    Create bounding box regions around predicted centroids.
    
    Args:
        centroids: List of (x, y) tuples
        img_h: Image height
        img_w: Image width
        expansion_factor: How much to expand around centroid
    
    Returns:
        List of (x_start, x_end, y_start, y_end) tuples
    """
    regions = []
    
    # Estimate a reasonable box size based on image dimensions
    base_box_size = min(img_h, img_w) // 8
    expanded_box_size = int(base_box_size * expansion_factor)
    
    for cx, cy in centroids:
        # Create box around centroid
        half_size = expanded_box_size // 2
        
        x_start = max(0, cx - half_size)
        x_end = min(img_w, cx + half_size)
        y_start = max(0, cy - half_size)
        y_end = min(img_h, cy + half_size)
        
        regions.append((x_start, x_end, y_start, y_end))
    
    return regions


def get_default_regions(h, w):
    """
    Get the original 7 default regions.
    
    Returns:
        List of (x_start, x_end, y_start, y_end) tuples
    """
    return [
        # Lower left
        (0, w//2, h//2, h),
        # Lower right
        (w//2, w, h//2, h),
        # Upper left center
        (w//3, 2*w//3, h//4, h//2),
        # Upper left left
        (0, w//2, h//4, h//2),
        # Upper left right
        (w//2, w, h//4, h//2),
        # Upper upper left
        (0, 3*w//5, 0, 5*h//16),
        # Upper upper right
        (2*w//5, w, 0, 5*h//16),
    ]


# Example usage:
def example_usage():
    """
    Example of how to use the function with a CV model
    """
    # Load image
    img = cv2.imread("your_image.jpg")
    
    # Option 1: Use CV model predictions
    # predicted_centroids = your_cv_model.predict(img)  # Returns [(x1,y1), (x2,y2), ...]
    predicted_centroids = [(100, 150), (300, 200), (500, 400)]  # Example predictions
    
    result = preprocess_with_centroids(
        img, 
        predicted_centroids=predicted_centroids,
        expansion_factor=2.0  # Expand search box 2x around predictions
    )
    
    # Option 2: Use original 7-section approach (no model predictions)
    result_original = preprocess_with_centroids(img, predicted_centroids=None)
    
    return result


def preprocess2(img):
    image = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY) 
    blank_img = np.zeros_like(img)
# 
    image = cv2.GaussianBlur(image, (7, 7), 0)
    # image = cv2.bilateralFilter(image, 9, 75, 75)
    image = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8,8)).apply(image)

    edges = cv2.adaptiveThreshold(
        image, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
        cv2.THRESH_BINARY_INV, 15, 3
    )

    h, w = edges.shape
    # cv2.imwrite("edges.jpg", edges)
    # cv2.imshow("edges.jpg", edges)
    contours_ull, hierarchy_ull = cv2.findContours(edges[h//4:h//2, :w//2], cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    contours_ulc, hierarchy_ulc = cv2.findContours(edges[h//4:h//2, w//3:2*w//3], cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    contours_ulr, hierarchy_ulr = cv2.findContours(edges[h//4:h//2, w//2:], cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    
    contours_uul, hierarchy_uul = cv2.findContours(edges[:5*h//16, :3*w//5], cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    contours_uur, hierarchy_uur = cv2.findContours(edges[:5*h//16, 2*w//5:], cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    
    contours_ll, hierarchy_ll = cv2.findContours(edges[h//2:, :w//2], cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    contours_lr, hierarchy_lr = cv2.findContours(edges[h//2:, w//2:], cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    # c_test, _ = cv2.findContours(edges[h//4:h//2, ], cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    # cv2.drawContours(blank_img[:h//2, ], c_test, -1, color=(0,255,0), thickness=2)

    descendants_info = ContourInfo()
    valid_contours_mask = np.zeros_like(image)

    all_contours = []
    all_hierarchies = []
    correspondences = []
    for i, (contours, h, offset) in enumerate(zip(
                                [contours_ll, contours_lr, contours_ulc, contours_ull,contours_ulr, contours_uul, contours_uur ], 
                                [hierarchy_ll, hierarchy_lr, hierarchy_ulc, hierarchy_ull,  hierarchy_ulr, hierarchy_uul, hierarchy_uur], 
                                [(0, h//2,), (w//2, h//2),  (w//3,h//4), (0,h//4),(w//2,h//4), (0,0), (2*w//5, 0)])):
        if h is not None and len(h) > 0:
            hierarchy = h[0]

            # Image center
            d_i, failed_list, success_list = get_descendants2(contours, hierarchy, offset, return_top=True)
            print(len(d_i))
            # input("d_i wait")
            # if i < 2:
            #     if len(d_i) == 0:
            #         return None, None, None, None
                

            descendants_info.descendants += d_i.descendants
            if len(d_i) > 0:
                correspondences.append((i, len(correspondences)))
            # elif i == 2:
            #     if len(descendants_info) < 2:
            #         return None, None, None, None
                
                
            cv2.drawContours(blank_img[offset[1]:, offset[0]:], contours, -1,color=(255,255,255), thickness=1)
            # cv2.drawContours(blank_img[offset[1]:, offset[0]:], failed_list, -1, color=(0,0,255), thickness=1)
            cv2.drawContours(blank_img[offset[1]:, offset[0]:], success_list, -1, color=(255,0,0), thickness=2)

        all_contours += contours
        all_hierarchies.append(h)

    all_hierarchies = np.concatenate(all_hierarchies, axis=1)
    cv2.imshow("contours32.jpg", blank_img)
        
    return descendants_info, correspondences, all_hierarchies, all_contours


def ellipse_precision(points, ellipse):
    center, axes, angle = ellipse
    cx, cy = center
    a, b = axes[0] / 2, axes[1] / 2

    cos_angle = np.cos(np.radians(angle))
    sin_angle = np.sin(np.radians(angle))

    residuals = []
    for x, y in points:
        x_rot = cos_angle * (x - cx) + sin_angle * (y - cy)
        y_rot = -sin_angle * (x - cx) + cos_angle * (y - cy)
        ellipse_value = (x_rot / a) ** 2 + (y_rot / b) ** 2
        residuals.append(abs(ellipse_value - 1.0))
    return np.mean(residuals)

def ellipse_recall(points, ellipse, n_samples=200):
    center, axes, angle = ellipse
    cx, cy = center
    a, b = axes[0] / 2, axes[1] / 2

    t = np.linspace(0, 2*np.pi, n_samples, endpoint=False)
    x = a * np.cos(t)
    y = b * np.sin(t)

    cos_angle = np.cos(np.radians(angle))
    sin_angle = np.sin(np.radians(angle))
    x_world = cx + cos_angle * x - sin_angle * y
    y_world = cy + sin_angle * x + cos_angle * y
    ellipse_points = np.column_stack((x_world, y_world))

    data = np.array(points)
    dists = []
    for ex, ey in ellipse_points:
        dists.append(np.min(np.linalg.norm(data - [ex, ey], axis=1)))
    return np.mean(dists)

def ellipse_f1(points, ellipse, n_samples=200):
    prec_err = ellipse_precision(points, ellipse)
    rec_err = ellipse_recall(points, ellipse, n_samples)

    # Convert errors to scores (higher is better)
    prec_score = 1 / (1 + prec_err)
    rec_score = 1 / (1 + rec_err)

    if prec_score + rec_score == 0:
        return 0
    return 2 * (prec_score * rec_score) / (prec_score + rec_score)