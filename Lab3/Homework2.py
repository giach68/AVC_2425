import open3d as o3d
import numpy as np

# Parameters
fx = 525.0
fy = 525.0
cx = 319.5
cy = 239.5

def depth_to_point_cloud(depth_image, rgb_image):
    rows, cols = depth_image.shape[:2]  # Get dimensions of the depth image
    point_cloud = []
    colors = []

    for i in range(rows):
        for j in range(cols):
            d = depth_image[i, j]
            if d > 0:  # Filter out invalid depths
                z = d
                x = (j - cx) * d / fx
                y = (i - cy) * d / fy
                point_cloud.append([x, y, z])
                colors.append(rgb_image[i, j])
    
    return np.array(point_cloud), np.array(colors)

def create_mesh_from_point_cloud(points, colors, depth_image):
    rows, cols = depth_image.shape[:2]  # Get dimensions of the depth image
    mesh = o3d.geometry.TriangleMesh()
    mesh.vertices = o3d.utility.Vector3dVector(points)
    mesh.vertex_colors = o3d.utility.Vector3dVector(colors / 255.0)  # Normalize color values to [0, 1]

    # Create triangles based on pixel connectivity
    triangles = []
    for i in range(rows - 1):
        for j in range(cols - 1):
            if depth_image[i, j] > 0 and depth_image[i, j + 1] > 0 and depth_image[i + 1, j] > 0 and depth_image[i + 1, j + 1] > 0:
                # Create two triangles for the quad
                v0 = i * cols + j
                v1 = i * cols + (j + 1)
                v2 = (i + 1) * cols + j
                v3 = (i + 1) * cols + (j + 1)

                triangles.append([v0, v1, v2])
                triangles.append([v2, v1, v3])

    mesh.triangles = o3d.utility.Vector3iVector(np.array(triangles))
    mesh.remove_degenerate_triangles()
    mesh.remove_duplicated_triangles()
    mesh.remove_duplicated_vertices()
    mesh.remove_non_manifold_edges()

    return mesh

# Load the color and depth images

rgb_image_path="football.jpg"
depth_image_path = "image.png"

rgb_image = o3d.io.read_image(rgb_image_path)
depth_image = o3d.io.read_image(depth_image_path)

# Convert depth image to 3D point cloud and retain colors
point_cloud, colors = depth_to_point_cloud(np.asarray(depth_image), np.asarray(rgb_image))

# Create a mesh from the point cloud
mesh = create_mesh_from_point_cloud(point_cloud, colors, np.asarray(depth_image))

# Visualize point cloud and mesh
pcd = o3d.geometry.PointCloud()
pcd.points = o3d.utility.Vector3dVector(point_cloud)
pcd.colors = o3d.utility.Vector3dVector(colors / 255.0)  # Normalize color values to [0, 1]

o3d.visualization.draw_geometries([pcd], "3D Point Cloud")

o3d.visualization.draw_geometries([mesh], "3D Mesh")
