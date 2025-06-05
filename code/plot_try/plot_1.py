import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation

# 创建斜坡
x = np.linspace(0, 10, 100)
y = -x + 10  # 这是一个简单的斜坡方程

# 小球的初始位置
ball_x = [0.5]
ball_y = [-0.5 + 10]

# 更新小球的位置
def update(num, ball_x, ball_y, line):
    ball_x[0] += 0.1
    ball_y[0] -= 0.1  # 假设小球沿斜坡线性滚动
    line.set_data(ball_x, ball_y)
    return line,

fig, ax = plt.subplots()
ax.plot(x, y, '-b')  # 画斜坡
line, = ax.plot(ball_x, ball_y, 'ro')  # 画小球

ani = animation.FuncAnimation(fig, update, frames=100, fargs=[ball_x, ball_y, line], blit=True, repeat=False)

plt.xlim(0, 10)
plt.ylim(0, 10)
plt.gca().set_aspect('equal', adjustable='box')
plt.show()
