import matplotlib.pyplot as plt
import json
import numpy as np

# 读取第一个 JSON 文件
try:
    with open('flow_density.json', 'r') as file:
        data1 = json.load(file)
except FileNotFoundError:
    print("未找到 flow_density.json 文件。")
    raise

# 读取第二个 JSON 文件
try:
    with open('Astar_density.json', 'r') as file:
        data2 = json.load(file)
except FileNotFoundError:
    print("未找到 Astar_density.json 文件。")
    raise

# 读取 CBSH2-RTC.json 文件
try:
    with open('CBSH2_density.json', 'r') as file:
        data3 = json.load(file)
except FileNotFoundError:
    print("未找到 CBSH2-RTC.json 文件。")
    raise

# 读取 LNS2.json 文件
try:
    with open('LNS2_density.json', 'r') as file:
        data4 = json.load(file)
except FileNotFoundError:
    print("未找到 LNS2.json 文件。")
    raise

# 提取数据
max_density1 = data1['max_density']
non_zero_mean_density1 = data1['non_zero_mean_density']

max_density2 = data2['max_density']
non_zero_mean_density2 = data2['non_zero_mean_density']

max_density3 = data3['max_density']
non_zero_mean_density3 = data3['non_zero_mean_density']

max_density4 = data4['max_density']
non_zero_mean_density4 = data4['non_zero_mean_density']

# 找出最大长度
max_length = max(len(max_density1), len(max_density2), len(max_density3), len(max_density4),
                 len(non_zero_mean_density1), len(non_zero_mean_density2), len(non_zero_mean_density3),
                 len(non_zero_mean_density4))

# 补齐数据
max_density1 = np.pad(max_density1, (0, max_length - len(max_density1)), 'constant', constant_values=0)
max_density2 = np.pad(max_density2, (0, max_length - len(max_density2)), 'constant', constant_values=0)
max_density3 = np.pad(max_density3, (0, max_length - len(max_density3)), 'constant', constant_values=0)
max_density4 = np.pad(max_density4, (0, max_length - len(max_density4)), 'constant', constant_values=0)

non_zero_mean_density1 = np.pad(non_zero_mean_density1, (0, max_length - len(non_zero_mean_density1)), 'constant',
                                constant_values=0)
non_zero_mean_density2 = np.pad(non_zero_mean_density2, (0, max_length - len(non_zero_mean_density2)), 'constant',
                                constant_values=0)
non_zero_mean_density3 = np.pad(non_zero_mean_density3, (0, max_length - len(non_zero_mean_density3)), 'constant',
                                constant_values=0)
non_zero_mean_density4 = np.pad(non_zero_mean_density4, (0, max_length - len(non_zero_mean_density4)), 'constant',
                                constant_values=0)

# 创建时间轴
x = np.arange(max_length)

# 调整图形大小，一般 LaTeX 单列宽度约为 3.375 英寸
fig_size = (6, 2.5)

# 绘制 max_density 对比图
plt.figure(figsize=fig_size)
plt.plot(x, max_density1, label='Flow', color='b')
plt.plot(x, max_density2, label='A*', color='c', linestyle='--')
plt.plot(x, max_density3, label='CBSH2-RTC', color='orange', linestyle='-.')
plt.plot(x, max_density4, label='LNS2', color='purple', linestyle=':')
plt.title('Max Density')
plt.xlabel('Time')
plt.ylabel('Value')
plt.grid(True)
plt.legend()
plt.tight_layout()
plt.savefig('max_density_comparison.eps', dpi=300)

# 绘制 non_zero_mean_density 对比图
plt.figure(figsize=fig_size)
plt.plot(x, non_zero_mean_density1, label='Flow', color='r')
plt.plot(x, non_zero_mean_density2, label='A*', color='y', linestyle='--')
plt.plot(x, non_zero_mean_density3, label='CBSH2-RTC', color='pink', linestyle='-.')
plt.plot(x, non_zero_mean_density4, label='LNS2', color='gray', linestyle=':')
plt.title('Mean Density')
plt.xlabel('Time')
plt.ylabel('Value')
plt.grid(True)
plt.legend()
plt.tight_layout()
plt.savefig('mean_density_comparison.eps', dpi=300)

# 显示图形
plt.show()


