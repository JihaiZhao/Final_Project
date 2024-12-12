import numpy as np
np.set_printoptions(precision=4)

import matplotlib.pyplot as plt
import matplotlib as mpl
from scipy.stats import multivariate_normal as mvn

mpl.rcParams['axes.linewidth'] = 3
mpl.rcParams['axes.titlesize'] = 20
mpl.rcParams['axes.labelsize'] = 20
mpl.rcParams['axes.titlepad'] = 8.0
mpl.rcParams['xtick.major.size'] = 6
mpl.rcParams['xtick.major.width'] = 3
mpl.rcParams['xtick.labelsize'] = 20
mpl.rcParams['ytick.major.size'] = 6
mpl.rcParams['ytick.major.width'] = 3
mpl.rcParams['ytick.labelsize'] = 20
mpl.rcParams['lines.markersize'] = 5
mpl.rcParams['legend.fontsize'] = 15

from scipy.stats import multivariate_normal as mvn

# Define the target distribution
mean1 = np.array([0.35, 0.38, 0.38])
cov1 = np.array([
    [0.01, 0.004, 0.004],
    [0.004, 0.01, -0.003],
    [0.004, -0.003, 0.01]
])
w1 = 0.5

mean2 = np.array([0.68, 0.25, 0.25])
cov2 = np.array([
    [0.01, 0.00, 0.00],
    [0.00, 0.01, 0.00],
    [0.00, 0.00, 0.01]
])
w2 = 0.2

mean3 = np.array([0.56, 0.64, 0.7])
cov3 = np.array([
    [0.01, 0.00, 0.00],
    [0.00, 0.01, 0.00],
    [0.00, 0.00, 0.01]
])

w3 = 0.3

def pdf(x):
    return w1 * mvn.pdf(x, mean1, cov1) + \
           w2 * mvn.pdf(x, mean2, cov2) + \
           w3 * mvn.pdf(x, mean3, cov3)
           
           
# import numpy as np
# import matplotlib.pyplot as plt
# from mpl_toolkits.mplot3d import Axes3D
# from scipy.stats import multivariate_normal as mvn

# # 创建图形
# fig = plt.figure(figsize=(12, 8))
# ax = fig.add_subplot(111, projection='3d')

# # 创建更密集的xy平面网格
# x = np.linspace(0, 1, 100)
# y = np.linspace(0, 1, 100)
# X, Y = np.meshgrid(x, y)
# points_2d = np.column_stack((X.flatten(), Y.flatten()))

# # 选择z平面
# z_levels = np.linspace(0, 1, 10)  # 12个z平面

# # 在多个z平面上绘制等高线
# for z_val in z_levels:
#     Z_points = np.column_stack((points_2d, np.full(len(points_2d), z_val)))
#     density = pdf(Z_points)
#     density = density.reshape(X.shape)
    
#     ax.contour(X, Y, density,
#                zdir='z',
#                offset=z_val,
#                levels=15,
#                cmap='viridis',
#                alpha=0.7)

# # 标记三个高斯分布的均值点
# ax.scatter([mean1[0]], [mean1[1]], [mean1[2]], color='red', s=100, label='Mean 1')
# ax.scatter([mean2[0]], [mean2[1]], [mean2[2]], color='blue', s=100, label='Mean 2')
# ax.scatter([mean3[0]], [mean3[1]], [mean3[2]], color='green', s=100, label='Mean 3')

# # 设置图形属性
# ax.set_xlabel('X')
# ax.set_ylabel('Y')
# ax.set_zlabel('Z')
# ax.set_title('3D Gaussian Mixture (Interactive)')
# ax.legend()

# # 设置轴的范围
# ax.set_xlim(0, 1)
# ax.set_ylim(0, 1)
# ax.set_zlim(0, 1)

# plt.show()
# Function to adjust the scale of the contours for a spherical effect
def scale_contours(z_val, max_z, scale_factor=1):
    return np.sqrt(1 - (z_val / max_z)**2)

# Create figure
fig = plt.figure(figsize=(12, 8))
ax = fig.add_subplot(111, projection='3d')

# Create a denser xy plane grid
x = np.linspace(0, 1, 100)
y = np.linspace(0, 1, 100)
X, Y = np.meshgrid(x, y)

# Choose z planes
z_levels = np.linspace(0, 1, 20)  # 20 z planes
max_z = z_levels.max()

# Plot contours on multiple z planes
for z_val in z_levels:
    scale = scale_contours(z_val, max_z)
    scaled_X = X * scale
    scaled_Y = Y * scale
    points_2d = np.column_stack((scaled_X.flatten(), scaled_Y.flatten()))
    Z_points = np.column_stack((points_2d, np.full(len(points_2d), z_val)))
    
    density = pdf(Z_points)
    density = density.reshape(X.shape)
    
    ax.contour(scaled_X, scaled_Y, density,
               zdir='z',
               offset=z_val,
               levels=15,
               cmap='viridis',
               alpha=0.7)

# Mark the means of the three Gaussian distributions
ax.scatter([mean1[0]], [mean1[1]], [mean1[2]], color='red', s=100, label='Mean 1')
ax.scatter([mean2[0]], [mean2[1]], [mean2[2]], color='blue', s=100, label='Mean 2')
ax.scatter([mean3[0]], [mean3[1]], [mean3[2]], color='green', s=100, label='Mean 3')

# Set figure properties
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
ax.set_title('3D Gaussian Mixture with Spherical Effect')
ax.legend()

# Set axis range
ax.set_xlim(0, 1)
ax.set_ylim(0, 1)
ax.set_zlim(0, 1)

plt.show()
