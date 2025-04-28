import numpy as np
import cv2 as cv

ply_header = '''ply
format ascii 1.0
element vertex %(vert_num)d
property float x
property float y
property float z
property uchar red
property uchar green
property uchar blue
element face %(face_num)d
property list uchar int vertex_index
end_header
'''
       
def write_ply(fn, verts, colors, faces):
    verts = verts.reshape(-1, 3)
    colors = colors.reshape(-1, 3)
    verts_colors = np.hstack([verts, colors])
    
    with open(fn, 'w') as f:
        f.write(ply_header % dict(vert_num=len(verts), face_num=len(faces)))
        np.savetxt(f, verts_colors, fmt='%f %f %f %d %d %d')
        for face in faces:
            f.write('3 %d %d %d\n' % tuple(face))


def main():
    print('loading images...')
    imgL = cv.pyrDown(cv.imread('aloeL.jpg'))  # downscale images for faster processing
    imgR = cv.pyrDown(cv.imread('aloeR.jpg'))

    window_size = 3
    min_disp = 16
    num_disp = 112 - min_disp
    stereo = cv.StereoSGBM_create(
        minDisparity=min_disp,
        numDisparities=num_disp,
        blockSize=16,
        P1=8 * 3 * window_size**2,
        P2=32 * 3 * window_size**2,
        disp12MaxDiff=1,
        uniquenessRatio=10,
        speckleWindowSize=100,
        speckleRange=32
    )

    print('computing disparity...')
    disp = stereo.compute(imgL, imgR).astype(np.float32) / 16.0

    print('generating 3d point cloud...')
    h, w = imgL.shape[:2]
    f = 0.8 * w
    Q = np.float32([[1, 0, 0, -0.5 * w],
                    [0, -1, 0, 0.5 * h],
                    [0, 0, 0, -f],
                    [0, 0, 1, 0]])
    
    points = cv.reprojectImageTo3D(disp, Q)
    colors = cv.cvtColor(imgL, cv.COLOR_BGR2RGB)
    mask = disp > disp.min()

    indices_map = -np.ones((h, w), dtype=int)
    valid_points = points[mask]
    valid_colors = colors[mask]
    
    indices_map[mask] = np.arange(len(valid_points))

    faces = []
    for i in range(h - 1):
        for j in range(w - 1):
            p1 = indices_map[i, j]
            p2 = indices_map[i, j + 1]
            p3 = indices_map[i + 1, j]
            p4 = indices_map[i + 1, j + 1]
            if p1 != -1 and p2 != -1 and p3 != -1:
                faces.append([p1, p2, p3])
            if p2 != -1 and p4 != -1 and p3 != -1:
                faces.append([p2, p4, p3])

    out_fn = 'out.ply'
    write_ply(out_fn, valid_points, valid_colors, faces)
    print('%s saved' % out_fn)

    cv.imshow('left', imgL)
    cv.imshow('disparity', (disp - min_disp) / num_disp)
    cv.waitKey()

    print('Done')



if __name__ == '__main__':
    main()
    cv.destroyAllWindows()