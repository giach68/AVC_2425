import numpy as np
import cv2 as cv
import open3d as o3d

ply_header = '''ply
format ascii 1.0
element vertex %(vert_num)d
property float x
property float y
property float z
property uchar red
property uchar green
property uchar blue
end_header
'''

def write_ply(fn, verts, colors):
    verts = verts.reshape(-1, 3)
    colors = colors.reshape(-1, 3)
    verts = np.hstack([verts, colors])
    with open(fn, 'wb') as f:
        f.write((ply_header % dict(vert_num=len(verts))).encode('utf-8'))
        np.savetxt(f, verts, fmt='%f %f %f %d %d %d ')


def main():
    print('loading images...')
    imgL = cv.pyrDown(cv.imread('aloeL.jpg'))  # downscale images for faster processing
    imgR = cv.pyrDown(cv.imread('aloeR.jpg'))

    # disparity range is tuned for 'aloe' image pair
    window_size = 3
    min_disp = 16
    num_disp = 112-min_disp
    stereo = cv.StereoSGBM_create(minDisparity = min_disp,
        numDisparities = num_disp,
        blockSize = 16,
        P1 = 8*3*window_size**2,
        P2 = 32*3*window_size**2,
        disp12MaxDiff = 1,
        uniquenessRatio = 10,
        speckleWindowSize = 100,
        speckleRange = 32
    )

    print('computing disparity...')
    disp = stereo.compute(imgL, imgR).astype(np.float32) / 16.0

    print('generating 3d point cloud...',)
    h, w = imgL.shape[:2]
    f = 0.8*w                          # guess for focal length
    Q = np.float32([[1, 0, 0, -0.5*w],
                    [0,-1, 0,  0.5*h], # turn points 180 deg around x-axis,
                    [0, 0, 0,     -f], # so that y-axis looks up
                    [0, 0, 1,      0]])
    points = cv.reprojectImageTo3D(disp, Q)
    colors = cv.cvtColor(imgL, cv.COLOR_BGR2RGB)
    mask = disp > disp.min()
    out_points = points#[mask]
    out_colors = colors#[mask]
    out_fn = 'out.ply'
    write_ply(out_fn, out_points, out_colors)
    print('%s saved' % out_fn)

    cv.imshow('left', imgL)
    cv.imshow('disparity', (disp-min_disp)/num_disp)
    cv.waitKey()
 
    pcd = o3d.geometry.PointCloud()
    verts = out_points.reshape(-1, 3)
    pcd.points = o3d.utility.Vector3dVector(verts)
    o3d.visualization.draw_geometries([pcd])

    print('Done')
    h=imgL.shape[0]
    w=imgL.shape[1]

    mesh = o3d.geometry.TriangleMesh()
    np_vertices = pcd.points
    np_triangles = []
    for j in range(0, h-2):
        for i in range(0, w-2):
            dist = np.linalg.norm(np_vertices[j*w + i] - np_vertices[(j+1)*w + i])
            dist = dist + np.linalg.norm(np_vertices[(j+1)*w + (i+1)] - np_vertices[(j+1)*w + i])
            dist = dist + np.linalg.norm(np_vertices[(j+1)*w + (i+1)] - np_vertices[(j)*w + i])
            if dist < 2:
                tri = [j*w + i, (j+1)*w + i, (j+1)*w + (i+1)]
                np_triangles.append(tri)
            
            dist = np.linalg.norm(np_vertices[j*w + i] - np_vertices[(j)*w + i+1])
            dist = dist + np.linalg.norm(np_vertices[(j)*w + i+1] - np_vertices[(j+1)*w + i+1])
            dist = dist + np.linalg.norm(np_vertices[(j)*w + i] - np_vertices[(j+1)*w + i+1])
            if dist < 2:
                tri = [(j)*w + i+1, (j)*w + (i), (j+1)*w + i+1]
                np_triangles.append(tri)


    mesh.vertices = o3d.utility.Vector3dVector(np_vertices)
    mesh.triangles = o3d.utility.Vector3iVector(np_triangles)

    o3d.visualization.draw([mesh])
    o3d.io.write_triangle_mesh("out.ply",mesh, write_ascii=True)


  



if __name__ == '__main__':
    print(__doc__)
    main()
    cv.destroyAllWindows()
