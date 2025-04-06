import open3d as o3d
import numpy as np

if __name__ == '__main__':
    # read the color and the depth image:
    rgb_image = o3d.io.read_image("rgb.jpg")
    depth_image = o3d.io.read_image("depth.png")

    # create an rgbd image object:
    rgbd_image = o3d.geometry.RGBDImage.create_from_color_and_depth(
        rgb_image, depth_image, convert_rgb_to_intensity=False)

    print(rgbd_image)
    # use the rgbd image to create point cloud:
    pcd = o3d.geometry.PointCloud.create_from_rgbd_image(
        rgbd_image,
        o3d.camera.PinholeCameraIntrinsic(
            o3d.camera.PinholeCameraIntrinsicParameters.PrimeSenseDefault))
    # visualize:
    o3d.visualization.draw_geometries([pcd])


    # save the point cloud:
    o3d.io.write_point_cloud("rgbd.ply", pcd)