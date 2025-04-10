import open3d as o3d

import numpy as np
from copy import deepcopy
import argparse


mesh1 = o3d.io.read_triangle_mesh("part1.obj")
src = mesh1.sample_points_uniformly(1000)
src.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=10, max_nn=30))
mesh2 = o3d.io.read_triangle_mesh("part2.obj")
dst = mesh2.sample_points_uniformly(1000)
dst.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=10, max_nn=30))



o3d.visualization.draw_geometries([src,dst],
                                  point_show_normal=True)

src_fpfh = o3d.pipelines.registration.compute_fpfh_feature(
        src, o3d.geometry.KDTreeSearchParamHybrid(radius=20,
                                             max_nn=100),)
print(src_fpfh)
dst_fpfh = o3d.pipelines.registration.compute_fpfh_feature(
        dst, o3d.geometry.KDTreeSearchParamHybrid(radius=20,
                                             max_nn=100),)
distance_threshold = 1
result = o3d.pipelines.registration.registration_ransac_based_on_feature_matching(
        src, dst, src_fpfh, dst_fpfh, distance_threshold,
        o3d.pipelines.registration.TransformationEstimationPointToPoint(False),
        4, [
            o3d.pipelines.registration.CorrespondenceCheckerBasedOnEdgeLength(
                0.9),
            o3d.pipelines.registration.CorrespondenceCheckerBasedOnDistance(
                distance_threshold)
        ], o3d.pipelines.registration.RANSACConvergenceCriteria(4000000, 500))
