import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# 创建X, Y的网格数据
X = np.linspace(0, 20, 100)
Y = np.linspace(0, 20, 100)
X, Y = np.meshgrid(X, Y)

# 创建一个斜坡函数 Z = X + Y
Z_slope = X

# 创建凸起的山作为障碍物
Z_mountains = np.sin(X)*1.5*np.sin(Y)*1.5 + Z_slope

# 设置画布和3D轴
fig = plt.figure(figsize=(10, 6))
ax = fig.add_subplot(111, projection='3d')

# 画斜坡和障碍物
ax.plot_surface(X, Y, Z_slope, color='c', alpha=0.5, label='Slope')
ax.plot_surface(X, Y, Z_mountains, color='brown', alpha=0.7, label='Mountains')

ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
ax.set_title('3D Slope with Mountains')

plt.show()
