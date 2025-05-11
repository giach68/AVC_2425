import pyvista as pv

mesh = pv.read("gaudi.ply")
print(mesh)
print(mesh.n_open_edges)
print(mesh.is_manifold)


cqual = mesh.compute_cell_quality('min_angle')

cqual.plot(show_edges=True)

curv = mesh.curvature(curv_type='mean')
print(curv)

mesh.plot(scalars=curv,
    clim=[-0.5, 0.5],
    cmap='bwr',
    below_color='blue',
    above_color='red',)