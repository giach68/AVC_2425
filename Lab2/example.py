import open3d as o3d
import copy
import numpy as np



pcd = o3d.io.read_point_cloud("apple_4_4_202.pcd")
print(pcd)
print(np.asarray(pcd.points))
o3d.visualization.draw_geometries([pcd])