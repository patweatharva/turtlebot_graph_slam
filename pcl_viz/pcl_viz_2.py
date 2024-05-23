from pypcd4 import PointCloud
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

directory = '6/'

# Load the PCD files using pypcd4
pc_source = PointCloud.from_path(directory +'2-1source.pcd')
pc_aligned = PointCloud.from_path(directory +'2-1aligned.pcd')
pc_target = PointCloud.from_path(directory +'2-1target.pcd')

# Convert the point cloud data to a NumPy array
points_source = pc_source.numpy(("x", "y", "z"))
points_aligned = pc_aligned.numpy(("x", "y", "z"))
points_target = pc_target.numpy(("x", "y", "z"))

# Separate the x, y, and z coordinates for each point cloud
x_source, y_source, z_source = points_source[:, 0], points_source[:, 1], points_source[:, 2]
x_aligned, y_aligned, z_aligned = points_aligned[:, 0], points_aligned[:, 1], points_aligned[:, 2]
x_target, y_target, z_target = points_target[:, 0], points_target[:, 1], points_target[:, 2]

# Create a figure with two subplots
fig = plt.figure(figsize=(12, 6))

# Subplot for source and target point clouds
ax1 = fig.add_subplot(121, projection='3d')
ax1.scatter(x_source, y_source, z_source, c='r', label='Source', s=2)
ax1.scatter(x_target, y_target, z_target, c='b', label='Target', s=2)
ax1.set_title('Source and Target')
ax1.set_xlabel('X')
ax1.set_ylabel('Y')
ax1.set_zlabel('Z')
ax1.legend()

# Subplot for aligned and target point clouds
ax2 = fig.add_subplot(122, projection='3d')
ax2.scatter(x_aligned, y_aligned, z_aligned, c='g', label='Aligned', s=2)
ax2.scatter(x_target, y_target, z_target, c='b', label='Target', s=2)
ax2.set_title('Aligned and Target')
ax2.set_xlabel('X')
ax2.set_ylabel('Y')
ax2.set_zlabel('Z')
ax2.legend()

# Show the plot
plt.tight_layout()
plt.show()
