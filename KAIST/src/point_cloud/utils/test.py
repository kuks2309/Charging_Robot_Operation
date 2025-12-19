import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from sklearn.neighbors import NearestNeighbors
import random
from typing import List, Tuple, Optional, Dict, Any
import json
# PLY file reading
try:
    from plyfile import PlyData, PlyElement
    PLY_AVAILABLE = True
except ImportError:
    PLY_AVAILABLE = False
    print("Warning: plyfile not installed. Install with: pip install plyfile")

# Alternative: Open3D for PLY reading
try:
    import open3d as o3d
    OPEN3D_AVAILABLE = True
except ImportError:
    OPEN3D_AVAILABLE = False
    print("Note: open3d not available. Install with: pip install open3d")

class Circle:
    def __init__(self, center_x: float, center_y: float, radius: float, center_z: float = 0):
        self.center = np.array([center_x, center_y, center_z])
        self.center_x = center_x
        self.center_y = center_y
        self.center_z = center_z
        self.radius = radius
    
    def distance_to_point(self, point: np.ndarray) -> float:
        """Calculate distance from point to circle (in the plane)"""
        # Distance from point to circle center (in xy plane)
        center_2d = self.center[:2]
        point_2d = point[:2]
        dist_to_center = np.linalg.norm(point_2d - center_2d)
        # Distance from point to the circle itself
        return abs(dist_to_center - self.radius)

def load_ply_file(filepath: str, method: str = 'auto') -> np.ndarray:
    """
    Load a PLY file and extract point coordinates
    
    Parameters:
    - filepath: path to the .ply file
    - method: 'plyfile', 'open3d', or 'auto' (tries both)
    
    Returns:
    - Nx3 numpy array of points (x, y, z)
    """
    
    if method == 'auto':
        # Try plyfile first, then open3d
        if PLY_AVAILABLE:
            method = 'plyfile'
        elif OPEN3D_AVAILABLE:
            method = 'open3d'
        else:
            raise ImportError("Neither plyfile nor open3d available. Install one with:\n"
                            "pip install plyfile  OR  pip install open3d")
    
    if method == 'plyfile':
        return _load_ply_with_plyfile(filepath)
    elif method == 'open3d':
        return _load_ply_with_open3d(filepath)
    else:
        raise ValueError("Method must be 'plyfile', 'open3d', or 'auto'")

def _load_ply_with_plyfile(filepath: str) -> np.ndarray:
    """Load PLY file using plyfile library"""
    if not PLY_AVAILABLE:
        raise ImportError("plyfile not installed. Install with: pip install plyfile")
    
    print(f"Loading PLY file: {filepath}")
    try:
        plydata = PlyData.read(filepath)
        vertex_data = plydata['vertex']
        
        # Extract coordinates
        x = vertex_data['x']
        y = vertex_data['y'] 
        z = vertex_data['z']
        
        points = np.column_stack([x, y, z])
        print(f"Successfully loaded {len(points)} points using plyfile")
        return points
        
    except Exception as e:
        print(f"Error loading PLY file with plyfile: {e}")
        raise

def _load_ply_with_open3d(filepath: str) -> np.ndarray:
    """Load PLY file using Open3D library"""
    if not OPEN3D_AVAILABLE:
        raise ImportError("open3d not installed. Install with: pip install open3d")
    
    print(f"Loading PLY file: {filepath}")
    try:
        pcd = o3d.io.read_point_cloud(filepath)
        points = np.asarray(pcd.points)
        
        if len(points) == 0:
            raise ValueError("No points found in PLY file")
        
        print(f"Successfully loaded {len(points)} points using Open3D")
        return points
        
    except Exception as e:
        print(f"Error loading PLY file with Open3D: {e}")
        raise

def analyze_point_cloud_stats(points: np.ndarray):
    """Print basic statistics about the point cloud"""
    print("\n" + "="*50)
    print("POINT CLOUD STATISTICS")
    print("="*50)
    print(f"Total points: {len(points):,}")
    print(f"Dimensions: {points.shape}")
    print()
    print("Coordinate ranges:")
    print(f"  X: {points[:, 0].min():.3f} to {points[:, 0].max():.3f}")
    print(f"  Y: {points[:, 1].min():.3f} to {points[:, 1].max():.3f}")
    print(f"  Z: {points[:, 2].min():.3f} to {points[:, 2].max():.3f}")
    print()
    print("Coordinate means:")
    print(f"  X: {points[:, 0].mean():.3f}")
    print(f"  Y: {points[:, 1].mean():.3f}")
    print(f"  Z: {points[:, 2].mean():.3f}")
    print()
    print("Standard deviations:")
    print(f"  X: {points[:, 0].std():.3f}")
    print(f"  Y: {points[:, 1].std():.3f}")
    print(f"  Z: {points[:, 2].std():.3f}")

def visualize_raw_point_cloud(points: np.ndarray, sample_size: int = 30000):
    """Visualize the raw point cloud (subsample for performance)"""
    # Subsample for visualization if too many points
    if len(points) > sample_size:
        indices = np.random.choice(len(points), sample_size, replace=False)
        display_points = points[indices]
        print(f"Displaying {sample_size:,} randomly sampled points for visualization")
    else:
        display_points = points
    
    fig = plt.figure(figsize=(15, 5))
    
    # 3D view
    ax1 = fig.add_subplot(131, projection='3d')
    ax1.scatter(display_points[:, 0], display_points[:, 1], display_points[:, 2], 
               c=display_points[:, 2], cmap='viridis', s=1, alpha=0.6)
    ax1.set_xlabel('X')
    ax1.set_ylabel('Y')
    ax1.set_zlabel('Z')
    ax1.set_title('3D Point Cloud')
    
    # XY view (top down)
    ax2 = fig.add_subplot(132)
    ax2.scatter(display_points[:, 0], display_points[:, 1], 
               c=display_points[:, 2], cmap='viridis', s=1, alpha=0.6)
    ax2.set_xlabel('X')
    ax2.set_ylabel('Y')
    ax2.set_title('Top View (XY)')
    ax2.set_aspect('equal')
    
    # XZ view (side)
    ax3 = fig.add_subplot(133)
    ax3.scatter(display_points[:, 0], display_points[:, 2], 
               c=display_points[:, 1], cmap='viridis', s=1, alpha=0.6)
    ax3.set_xlabel('X')
    ax3.set_ylabel('Z')
    ax3.set_title('Side View (XZ)')
    
    plt.tight_layout()
    # plt.show()
    def __init__(self, center_x: float, center_y: float, radius: float, center_z: float = 0):
        self.center = np.array([center_x, center_y, center_z])
        self.center_x = center_x
        self.center_y = center_y
        self.center_z = center_z
        self.radius = radius
    
    def distance_to_point(self, point: np.ndarray) -> float:
        """Calculate distance from point to circle (in the plane)"""
        # Distance from point to circle center (in xy plane)
        center_2d = self.center[:2]
        point_2d = point[:2]
        dist_to_center = np.linalg.norm(point_2d - center_2d)
        # Distance from point to the circle itself
        return abs(dist_to_center - self.radius)

def fit_circle_3_points(p1: np.ndarray, p2: np.ndarray, p3: np.ndarray) -> Optional[Circle]:
    """
    Fit a circle through 3 points in 2D (using x,y coordinates)
    Returns None if points are collinear
    """
    # Use only x,y coordinates
    p1_2d, p2_2d, p3_2d = p1[:2], p2[:2], p3[:2]
    
    # Check if points are roughly collinear
    v1 = p2_2d - p1_2d
    v2 = p3_2d - p1_2d
    cross_product = np.cross(v1, v2)
    
    if abs(cross_product) < 1e-6:  # Points are collinear
        return None
    
    # Calculate circle center using perpendicular bisectors
    # Midpoints
    mid1 = (p1_2d + p2_2d) / 2
    mid2 = (p2_2d + p3_2d) / 2
    
    # Direction vectors of the lines
    dir1 = p2_2d - p1_2d
    dir2 = p3_2d - p2_2d
    
    # Perpendicular vectors
    perp1 = np.array([-dir1[1], dir1[0]])
    perp2 = np.array([-dir2[1], dir2[0]])
    
    # Solve for intersection of perpendicular bisectors
    # mid1 + t1 * perp1 = mid2 + t2 * perp2
    # mid1 - mid2 = t2 * perp2 - t1 * perp1
    
    A = np.column_stack([-perp1, perp2])
    b = mid2 - mid1
    
    try:
        t = np.linalg.solve(A, b)
        center = mid1 + t[0] * perp1
        radius = np.linalg.norm(center - p1_2d)
        
        # Use average z coordinate for center
        center_z = (p1[2] + p2[2] + p3[2]) / 3
        
        return Circle(center[0], center[1], radius, center_z)
    
    except np.linalg.LinAlgError:
        return None

def ransac_circle_fit(points: np.ndarray, tolerance: float = 0.2, ) -> Tuple[Optional[Circle], np.ndarray]:
    """
    RANSAC algorithm to fit a circle to points
    Returns (best_circle, inlier_points) or (None, empty_array) if no good fit found
    """
    if len(points) < 3:
        return None, np.array([])
    
    points_c = points.copy()
    
    circle_points = []
    next_point_i = random.choice([x for x in range(len(points_c))])
    next_point = points_c[next_point_i]
    while True:
        if len(points_c) == 1:
            return None, np.array([])
        points_c = np.delete(points_c, next_point_i, axis=0)
        dist = np.linalg.norm(points_c - next_point, axis=1)
        closest_point_i = dist.argmin()
        closest_point = points_c[closest_point_i]

        if dist[closest_point_i] < tolerance:
            circle_points.append(closest_point)
            next_point = closest_point
            next_point_i = closest_point_i
        elif len(circle_points) < 10:
            circle_points = []
            next_point_i = random.choice([x for x in range(len(points_c))])
            next_point = points_c[next_point_i]
        else:
            circle_points = np.array(circle_points)
            centroid_circle = circle_points.mean(axis=0)
            radius = np.linalg.norm(circle_points - centroid_circle, axis=1)#.mean()
            mean_radius = radius.mean(keepdims=True)
            std_radius = np.std(radius - mean_radius)/mean_radius
            if std_radius.max() > 0.1:
                circle_points = []
                next_point_i = random.choice([x for x in range(len(points_c))])
                next_point = points_c[next_point_i]
            else:
                radius = mean_radius.squeeze()
                best_circle = Circle(*centroid_circle[:2], radius, 0)
                best_inliers = np.array(circle_points)
                break

    return best_circle, best_inliers


def remove_inlier_points(remaining_points: np.ndarray, inlier_points: np.ndarray, 
                        tolerance: float = 1e-6) -> np.ndarray:
    """Remove inlier points from the remaining points array"""
    if len(inlier_points) == 0:
        return remaining_points
    
    # Use nearest neighbors to find which points to remove
    nn = NearestNeighbors(n_neighbors=1)
    nn.fit(inlier_points)
    
    distances, indices = nn.kneighbors(remaining_points)
    keep_mask = distances.flatten() > tolerance
    
    return remaining_points[keep_mask]

def get_front_surface_points(point_cloud: np.ndarray, z_tolerance: float = 0.01) -> np.ndarray:
    """Extract points closest to camera (highest z values)"""
    min_z = point_cloud[:, 2].min()
    front_mask = point_cloud[:, 2] == min_z
    return point_cloud[front_mask]

def find_closest_point_to_camera(points: np.ndarray) -> Tuple[int, np.ndarray]:
    """Find the point closest to camera (highest z-value)"""
    max_z_idx = np.argmax(points[:, 2])
    return max_z_idx, points[max_z_idx]

def grow_circle_region(seed_point: np.ndarray, remaining_points: np.ndarray, 
                      max_distance: float = 0.5, circle_tolerance: float = 0.02) -> Tuple[Optional[Circle], np.ndarray]:
    """
    Grow a circular region starting from a seed point
    
    Parameters:
    - seed_point: starting point for region growing
    - remaining_points: available points to grow from
    - max_distance: maximum distance to consider points as neighbors
    - circle_tolerance: tolerance for circle fitting
    
    Returns:
    - (fitted_circle, region_points) or (None, empty_array)
    """
    # from sklearn.neighbors import NearestNeighbors
    
    # if len(remaining_points) < 5:
    #     return None, np.array([])
    
    # # Find all points within max_distance of seed
    # nn = NearestNeighbors(radius=max_distance)
    # nn.fit(remaining_points)
    
    # # Get neighbors of seed point
    # neighbors_indices = nn.radius_neighbors([seed_point], return_distance=False)[0]
    
    # if len(neighbors_indices) < 5:
    #     return None, np.array([])
    
    # candidate_points = remaining_points[neighbors_indices]
    
    # Try to fit a circle to these points using RANSAC
    circle, inliers = ransac_circle_fit(
        remaining_points, 
        min_inliers=5, 
        tolerance=circle_tolerance,
        max_iterations=500
    )
    
    if circle is None:
        return None, np.array([])
    
    # # Grow the region: find all points that fit this circle well
    # all_distances = np.array([circle.distance_to_point(p) for p in remaining_points])
    # good_fit_mask = all_distances < circle_tolerance
    
    # if np.sum(good_fit_mask) < 5:
    #     return None, np.array([])
    
    # final_region = remaining_points[good_fit_mask]
    
    # # Refit circle with all good points for better accuracy
    # final_circle = fit_circle_least_squares(final_region)
    # if final_circle is None:
    #     final_circle = circle
    
    return circle, inliers

def fit_circle_least_squares(points: np.ndarray) -> Optional[Circle]:
    """
    Fit circle using least squares method
    Solves: (x-a)² + (y-b)² = r²
    """
    if len(points) < 3:
        return None
    
    # Use only x,y coordinates
    x = points[:, 0]
    y = points[:, 1]
    
    # Set up least squares system
    # (x-a)² + (y-b)² = r²
    # x² - 2ax + a² + y² - 2by + b² = r²
    # x² + y² - 2ax - 2by + (a² + b² - r²) = 0
    # Let A = -2a, B = -2b, C = a² + b² - r²
    # Then: x² + y² + Ax + By + C = 0
    
    A = np.column_stack([x, y, np.ones(len(points))])
    b = -(x*x + y*y)
    
    try:
        # Solve Ax = b
        params, residuals, rank, s = np.linalg.lstsq(A, b, rcond=None)
        
        a = -params[0] / 2
        b_center = -params[1] / 2
        c = params[2]
        
        radius = np.sqrt(a*a + b_center*b_center - c)
        
        if radius > 0:
            center_z = np.mean(points[:, 2])
            return Circle(a, b_center, radius, center_z)
        else:
            return None
            
    except (np.linalg.LinAlgError, ValueError):
        return None

def find_7_circles_region_growing(point_cloud: np.ndarray, z_tolerance: float = 0.01, 
                                 max_distance: float = 0.5, circle_tolerance: float = 0.02) -> List[Dict[str, Any]]:
    """
    Find exactly 7 circles using region growing from closest points
    
    This method:
    1. Finds the closest point to camera
    2. Grows a circular region around it
    3. Removes the region and repeats
    
    Parameters:
    - point_cloud: Nx3 array of points
    - z_tolerance: tolerance for front surface extraction
    - max_distance: initial neighborhood radius for region growing
    - circle_tolerance: tolerance for circle fitting
    """
    print("Extracting front surface points...")
    front_points = get_front_surface_points(point_cloud, z_tolerance)
    print(f"Front surface has {len(front_points)} points")
    
    circles = []
    remaining_points = front_points.copy()
    
    for circle_id in range(7):
        print(f"\nFinding circle {circle_id + 1}/7...")
        print(f"Remaining points: {len(remaining_points)}")
        
        if len(remaining_points) < 10:
            print(f"Warning: Only {len(remaining_points)} points remaining")
            break
        
        # Find closest point to camera in remaining points
        closest_idx, seed_point = find_closest_point_to_camera(remaining_points)
        print(f"Seed point: ({seed_point[0]:.3f}, {seed_point[1]:.3f}, {seed_point[2]:.3f})")
        
        # Grow circular region from this seed
        circle, region_points = grow_circle_region(
            seed_point, 
            remaining_points, 
            max_distance=max_distance,
            circle_tolerance=circle_tolerance
        )
        
        if circle is None or len(region_points) < 5:
            print(f"Could not grow valid circle from seed point")
            # Remove just the seed point and try again
            remaining_points = np.delete(remaining_points, closest_idx, axis=0)
            continue
        
        print(f"Found circle {circle_id + 1}: center=({circle.center_x:.3f}, {circle.center_y:.3f}), "
              f"radius={circle.radius:.3f}, points={len(region_points)}")
        
        circles.append({
            'id': circle_id,
            'center': circle.center,
            'center_x': circle.center_x,
            'center_y': circle.center_y,
            'center_z': circle.center_z,
            'radius': circle.radius,
            'points': region_points,
            'num_points': len(region_points),
            'seed_point': seed_point
        })
        
        # Remove region points from remaining set
        remaining_points = remove_inlier_points(remaining_points, region_points)
    
    print(f"\nSummary: Successfully found {len(circles)}/7 circles using region growing")
    return circles

def find_7_circles(point_cloud: np.ndarray, z_tolerance: float = 0.01, 
                  circle_tolerance: float = 0.2, min_inliers: int = 150, 
                  method: str = 'region_growing') -> List[Dict[str, Any]]:
    """
    Find exactly 7 circles from a point cloud
    
    Parameters:
    - point_cloud: Nx3 array of points (x, y, z)
    - z_tolerance: tolerance for selecting front surface points
    - circle_tolerance: tolerance for circle fitting
    - min_inliers: minimum points required per circle
    
    Returns:
    - List of dictionaries containing circle information
    """
    print("Extracting front surface points...")
    front_points = get_front_surface_points(point_cloud, z_tolerance)
    print(f"Front surface has {len(front_points)} points")
    
    circles = []
    remaining_points = front_points.copy()
    
    for circle_id in range(7):
        print(f"\nFinding circle {circle_id + 1}/7...")
        print(f"Remaining points: {len(remaining_points)}")
        
        if len(remaining_points) < 10:
            print(f"Warning: Only {len(remaining_points)} points remaining, cannot find more circles")
            break
        
        # RANSAC circle fitting
        circle, inliers = ransac_circle_fit(
            remaining_points,
            tolerance=circle_tolerance,
        )
        
        if circle is None:
            print(f"Could not find circle {circle_id + 1}")
            break
        
        print(f"Found circle {circle_id + 1}: center=({circle.center_x:.3f}, {circle.center_y:.3f}), "
              f"radius={circle.radius:.3f}, points={len(inliers)}")
        
        circles.append({
            'id': circle_id,
            'center': circle.center,
            'center_x': circle.center_x,
            'center_y': circle.center_y,
            'center_z': circle.center_z,
            'radius': circle.radius,
            'points': inliers,
            'num_points': len(inliers)
        })
        
        # Remove inlier points from remaining set
        remaining_points = remove_inlier_points(remaining_points, inliers)
    
    print(f"\nSummary: Successfully found {len(circles)}/7 circles")
    
    return circles

def visualize_circles(point_cloud: np.ndarray, circles: List[Dict[str, Any]], 
                     show_3d: bool = True, show_2d: bool = True):
    """Visualize the original point cloud and detected circles"""
    output_circles = []
    for circle in circles:
        print(circle['num_points'], circle['radius'])
        output_circles.append({'center_x': float(circle['center_x']), 'center_y': float(circle['center_y']), 'center_z': float(circle['center_z']), 'radius': float(circle['radius'])})
    with open("detected_circles.json", "w") as f:
        json.dump(output_circles, f, indent=2)
    input("Press Enter to continue...")
    if show_3d:
        # 3D visualization
        fig = plt.figure(figsize=(12, 8))
        ax = fig.add_subplot(121, projection='3d')
        
        # Plot original point cloud
        front_points = get_front_surface_points(point_cloud)
        ax.scatter(front_points[:, 0], front_points[:, 1], front_points[:, 2], 
                  c='lightgray', alpha=0.6, s=1)
        
        # Plot circle points with different colors
        colors = plt.cm.Set3(np.linspace(0, 1, len(circles)))
        for i, circle_info in enumerate(circles):
            points = circle_info['points']
            ax.scatter(points[:, 0], points[:, 1], points[:, 2], 
                      c=[colors[i]], s=20, label=f'Circle {i+1}')
        
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')
        ax.set_title('3D Point Cloud with Detected Circles')
        ax.legend()
    
    if show_2d:
        # 2D top-down view
        ax2 = fig.add_subplot(122) if show_3d else plt.gca()
        
        # Plot all front points
        front_points = get_front_surface_points(point_cloud)
        ax2.scatter(front_points[:, 0], front_points[:, 1], 
                   c='lightgray', alpha=0.6, s=1)
        
        # Plot detected circles
        colors = plt.cm.Set3(np.linspace(0, 1, len(circles)))
        for i, circle_info in enumerate(circles):
            # Plot circle points
            points = circle_info['points']
            ax2.scatter(points[:, 0], points[:, 1], 
                       c=[colors[i]], s=20, label=f'Circle {i+1}')
            
            # Draw circle outline
            theta = np.linspace(0, 2*np.pi, 100)
            circle_x = circle_info['center_x'] + circle_info['radius'] * np.cos(theta)
            circle_y = circle_info['center_y'] + circle_info['radius'] * np.sin(theta)
            ax2.plot(circle_x, circle_y, color=colors[i], linewidth=2, alpha=0.8)
            
            # Mark center
            ax2.plot(circle_info['center_x'], circle_info['center_y'], 
                    'x', color=colors[i], markersize=10, markeredgewidth=3)
        
        ax2.set_xlabel('X')
        ax2.set_ylabel('Y')
        ax2.set_title('2D View - Detected Circles')
        ax2.legend()
        ax2.set_aspect('equal')
        ax2.grid(True, alpha=0.3)
    
    plt.tight_layout()
    plt.savefig("detected_circles.png")
    # plt.show()

def print_circle_summary(circles: List[Dict[str, Any]]):
    """Print a summary of detected circles"""
    print("\n" + "="*50)
    print("CIRCLE DETECTION SUMMARY")
    print("="*50)
    
    for i, circle in enumerate(circles):
        print(f"Circle {i+1}:")
        print(f"  Center: ({circle['center_x']:.3f}, {circle['center_y']:.3f}, {circle['center_z']:.3f})")
        print(f"  Radius: {circle['radius']:.3f}")
        print(f"  Points: {circle['num_points']}")
        print()
    
    print(f"Total circles found: {len(circles)}/7")
    if len(circles) == 7:
        print("✓ Successfully found all 7 circles!")
    else:
        print(f"⚠ Found only {len(circles)} circles. Consider adjusting parameters.")

# Example usage with real PLY file:
if __name__ == "__main__":
    import sys
    
    # Check if PLY file path provided
    if len(sys.argv) > 1:
        ply_filepath = sys.argv[1]
        print("="*60)
        print("LOADING REAL PLY FILE")
        print("="*60)
        
        # try:
            # Load the PLY file
        point_cloud = load_ply_file(ply_filepath)
        
        # Show basic statistics
        analyze_point_cloud_stats(point_cloud)
        
        # Visualize raw point cloud
        print("\nVisualizing raw point cloud...")
        visualize_raw_point_cloud(point_cloud)
        
        # Find circles
        print("\nStarting circle detection...")
        detected_circles = find_7_circles(
            point_cloud,
            z_tolerance=0.2,      # Adjust based on your data
            circle_tolerance=0.5,  # Adjust based on your data  
            min_inliers=15         # Adjust based on point density
        )
        
        # Print results
        print_circle_summary(detected_circles)
        
        # Visualize results
        if detected_circles:
            print("\nVisualizing detected circles...")
            visualize_circles(point_cloud, detected_circles)
        else:
            print("No circles detected. Try adjusting parameters.")
                
        # except Exception as e:
        #     print(f"Error processing PLY file: {e}")
        #     print("Make sure the file exists and is a valid PLY file.")
    
    else:
        # Generate sample data for testing (7 circles)
        print("="*60)
        print("RUNNING WITH SAMPLE DATA")
        print("="*60)
        print("To use your own PLY file, run: python script.py path/to/your/file.ply")
        print()
        print("Generating sample point cloud data...")
        np.random.seed(42)
    
    # # Generate 7 circles with different centers and radii
    # circle_params = [
    #     (0, 0, 1.0),    # center_x, center_y, radius
    #     (3, 0, 0.8),
    #     (-3, 0, 1.2),
    #     (0, 3, 0.9),
    #     (0, -3, 1.1),
    #     (2, 2, 0.7),
    #     (-2, -2, 1.0)
    # ]
    
    # sample_points = []
    # for cx, cy, r in circle_params:
    #     # Generate points on circle with some noise
    #     num_points = np.random.randint(20, 40)
    #     angles = np.random.uniform(0, 2*np.pi, num_points)
        
    #     # Add some noise to radius and position
    #     radius_noise = np.random.normal(0, 0.05, num_points)
    #     x = cx + (r + radius_noise) * np.cos(angles) + np.random.normal(0, 0.02, num_points)
    #     y = cy + (r + radius_noise) * np.sin(angles) + np.random.normal(0, 0.02, num_points)
    #     z = np.random.normal(5.0, 0.01, num_points)  # All points near z=5
        
    #     circle_points = np.column_stack([x, y, z])
    #     sample_points.append(circle_points)
    
    # # Combine all points
    # sample_point_cloud = np.vstack(sample_points)
    # print(f"Generated {len(sample_point_cloud)} sample points")
    
    # # Find circles
    # detected_circles = find_7_circles(sample_point_cloud)
    
    # # Print results
    # print_circle_summary(detected_circles)
    
    # # Visualize results
    # visualize_circles(sample_point_cloud, detected_circles)