import open3d as o3d

# Load point cloud from file (replace with your actual file path)
pcd = o3d.io.read_point_cloud("section_charging_port.ply")

# Check if the point cloud is loaded properly
if pcd.is_empty():
    print("Failed to load point cloud.")
else:
    # Visualize the point cloud
    o3d.visualization.draw_geometries([pcd])
