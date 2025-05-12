import pyvista as pv


mesh = pv.read("gaudi_noise.ply")  # Replace with your actual file path

# Apply smoothing
smoothed_mesh = mesh.smooth(n_iter=500)  # Adjust the number of iterations for more/less smoothing

# Visualize the original and smoothed mesh side by side
plotter = pv.Plotter()
plotter.add_mesh(mesh, color="lightblue", label="Original Mesh")
plotter.add_legend()
plotter.show()

plotter = pv.Plotter()
plotter.add_mesh(smoothed_mesh, color="red", label="Smoothed Mesh")

plotter.add_legend()
plotter.show()


smooth_w_taubin = mesh.smooth_taubin(n_iter=50, pass_band=0.5)

pl = pv.Plotter()
pl.add_mesh(smooth_w_taubin, show_edges=True, show_scalar_bar=False)

pl.show()
