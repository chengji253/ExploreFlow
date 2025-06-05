import json
import matplotlib.pyplot as plt

# 读取数据，这里以你提供的数据为例，实际使用时需分别读取四个文件

# 读取 Astar_wait.json 文件数据
with open('Astar_wait.json', 'r') as file:
    data_astar = json.load(file)
num_wait_list_astar = data_astar["num_wait_list"]
num_wait_list_astar[:2] = [0, 0]  # 将前两个元素设置为0

# 读取 CBSH2_wait.json 文件数据
with open('CBSH2_wait.json', 'r') as file:
    data_cbsh2 = json.load(file)
num_wait_list_cbsh2 = data_cbsh2["num_wait_list"]
num_wait_list_cbsh2[:2] = [0, 0]  # 将前两个元素设置为0

# 读取 LNS2_wait.json 文件数据
with open('LNS2_wait.json', 'r') as file:
    data_lns2 = json.load(file)
num_wait_list_lns2 = data_lns2["num_wait_list"]
num_wait_list_lns2[:2] = [0, 0]  # 将前两个元素设置为0

# 读取 flow_wait.json 文件数据
with open('flow_wait.json', 'r') as file:
    data_flow = json.load(file)
num_wait_list_flow = data_flow["num_wait_list"]
num_wait_list_flow[:2] = [0, 0]  # 将前两个元素设置为0


fig_size = (6, 2.3)
plt.figure(figsize=fig_size)
# 绘制对比图
plt.plot(num_wait_list_astar, label='A*')
plt.plot(num_wait_list_cbsh2, label='CBSH2-RTC')
plt.plot(num_wait_list_lns2, label='LNS2')
plt.plot(num_wait_list_flow, label='Flow')

plt.xlabel('Time')
plt.ylabel('Number')
plt.grid(True)
# plt.title('Comparison of Four Algorithms')
plt.legend()
plt.tight_layout()
plt.savefig('wait.eps', dpi=300)
plt.show()