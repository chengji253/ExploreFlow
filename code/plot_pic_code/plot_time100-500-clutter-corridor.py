import numpy as np
import matplotlib as mpl
import matplotlib.pyplot as plt
import json as js
from matplotlib.offsetbox import OffsetImage, AnnotationBbox
import matplotlib.pyplot as plt
import matplotlib.image as mpimg


# 画出到达时间和机器人数量的关系图

class Plot:

    def __init__(self):

        self.data_now = None

        self.num_list = [100, 150, 200, 250, 300, 350, 400, 450, 500]
        self.num_len = len(self.num_list)

        self.FRSP_time = 1*np.ones((self.num_len, 1), dtype=float)

        self.Astar_time = 2*np.ones((self.num_len, 1), dtype=float)

        self.cbsh2_time = 3*np.ones((self.num_len, 1), dtype=float)

        self.lns_time = 4*np.ones((self.num_len, 1), dtype=float)


    def main(self):
        # self.cal_FRSP_time_maze()

        # self.clutter_time_result()
        # self.cal_improvement_ratio()
        # self.plot_time_forest()

        self.maze_time_result()
        self.cal_improvement_ratio_maze()
        self.plot_time_maze()

    def clutter_time_result(self):
        # clutter
        self.FRSP_time =  [85.9, 90.6, 93.2, 98.7, 103.6, 106.4, 111.6, 115.6, 117.1]
        self.Astar_time = [90.1, 95.6, 101.2, 105.7, 110.6, 115.4, 118.8, 123.6, 125.3]
        self.cbsh2_time = [92.8, 96.6, 101.2, 102.7, 109.6, 112.4, 119.6, 122.6, 127.5]
        self.lns_time  =  [91.5, 93.7, 98.2, 100.7, 106.6, 113.4, 115.6, 119.6, 123.5]


    def maze_time_result(self):
        # corridor
        self.FRSP_time_maze =   [95.9, 100.6, 103.2, 107.7, 110.6, 115.4, 118.6, 121.6, 124.1]
        self.Astar_time_maze =  [105.9, 110.6, 118.2, 121.7, 124.6, 127.4, 128.6, 131.6, 132.9]
        self.cbsh2_time_maze  = [107.2, 113.6, 119.3, 125.8, 129.6, 130.4, 132.6, 133.6, 137.1]
        self.lns_time_maze  =   [102.1, 108.6, 112.2, 118.7, 123.6, 125.4, 127.6, 128.6, 129.1]

        self.Astar_time_maze = [x - 4 for x in self.Astar_time_maze]
        self.cbsh2_time_maze = [x - 5 for x in self.cbsh2_time_maze]
        self.lns_time_maze = [x - 2 for x in self.lns_time_maze]


    def cal_improvement_ratio(self):
        Astar_ave = np.mean(self.Astar_time)
        FRSP_ave = np.mean(self.FRSP_time)
        lns_ave = np.mean(self.lns_time)
        cbsh2_ave = np.mean(self.cbsh2_time)

        Astar_improve = (Astar_ave - FRSP_ave)/Astar_ave * 100
        cbsh2_improve = (cbsh2_ave - FRSP_ave)/cbsh2_ave * 100
        lns_improve = (lns_ave - FRSP_ave)/lns_ave * 100

        print("Astar improvement ratio " + str(Astar_improve))
        print("lns improvement ratio " + str(lns_improve))
        print("cbsh2 improvement ratio " + str(cbsh2_improve))

    def cal_improvement_ratio_maze(self):

        Astar_ave = np.mean(self.Astar_time_maze)
        FRSP_ave = np.mean(self.FRSP_time_maze)
        cbsh2_ave = np.mean(self.cbsh2_time_maze)
        lns_ave = np.mean(self.lns_time_maze)

        Astar_improve = (Astar_ave - FRSP_ave)/Astar_ave * 100
        cbsh2_improve = (cbsh2_ave - FRSP_ave)/cbsh2_ave * 100
        lns_improve = (lns_ave - FRSP_ave)/lns_ave * 100

        print("Astar improvement ratio " + str(Astar_improve))
        print("cbsh2 improvement ratio " + str(cbsh2_improve))
        print("lns improvement ratio " + str(lns_improve))

    def plot_time_forest(self):
        # 创建第一个 Axes 对象，并绘制第一条数据集的曲线
        fig, ax1 = plt.subplots(figsize=(3, 2.5))
        # fig, ax1 = plt.subplots()
        # fig, ax1 = plt.subplots()

        # diff 三个相似的橙色
        # '#FF8C00' '#FFA500' '#FFD700'
        # same 三个相似的蓝色
        # '#1E90FF', '#4682B4', '#87CEEB'

        ax1.plot(self.num_list[0:], self.FRSP_time[0:], '^--', label='Flow', linewidth=1, color='#FF8C00')
        ax1.plot(self.num_list[0:], self.Astar_time[0:], 'o--', label='A*', linewidth=1, color='#1E90FF')
        ax1.plot(self.num_list[0:], self.cbsh2_time[0:], 'o--', label='CBSH2-RTC', linewidth=1, color='#4682B4')
        ax1.plot(self.num_list[0:], self.lns_time[0:], 'o--', label='LNS2', linewidth=1, color='#FFA500')
        # ax1.plot(self.num_list, self.run_cost_time[:, 0], '^--', label='Two_mip', linewidth=1, color='#D2691E')

        ax1.set_xlabel('number of robots')
        ax1.set_ylabel('makespan')
        ax1.tick_params('y')
        # ax1.ticklabel_format(style='sci', axis='y', scilimits=(0, 0))

        lines1, labels1 = ax1.get_legend_handles_labels()

        ax1.legend(lines1, labels1, fontsize='x-small', frameon=False, loc='upper left')
        # ax1.legend(fontsize='x-small', frameon=False)

        # x-small [-1, 130]

        ax1.set_xlim([80, 520])
        ax1.set_xticks([i for i in range(100, 510, 100)])
        ax1.set_ylim([80, 130])
        # ax1.set_yticks([i for i in range(800, 1401, 100)])

        # 嵌入图片到右下角
        try:
            # 读取图片
            img = mpimg.imread('map/cl500.png')  # 替换为你的图片路径
            # 创建 OffsetImage 对象，设置缩放比例
            imagebox = OffsetImage(img, zoom=0.15)  # 调整 zoom 参数以控制图片大小
            # 设置图片位置（右下角）
            ab = AnnotationBbox(imagebox, (0.99, 0.01),  # 右下角坐标（相对坐标，0到1）
                                xycoords='axes fraction',  # 使用轴的相对坐标
                                box_alignment=(1, 0),  # 右下对齐
                                frameon=False)  # 无边框
            ax1.add_artist(ab)
        except FileNotFoundError:
            print("嵌入的图片文件未找到，请检查路径！")

        plt.rcParams['pdf.fonttype'] = 42
        plt.rcParams['ps.fonttype'] = 42

        plt.grid(True)
        plt.tight_layout()
        plt.savefig('num-time_clutter-500.eps', bbox_inches='tight', dpi=300)
        plt.show()  # 显示图像


    def plot_time_maze(self):
        # 创建第一个 Axes 对象，并绘制第一条数据集的曲线
        fig, ax1 = plt.subplots(figsize=(3, 2.5))
        # fig, ax1 = plt.subplots()
        # fig, ax1 = plt.subplots()

        # diff 三个相似的橙色
        # '#FF8C00' '#FFA500' '#FFD700'
        # same 三个相似的蓝色
        # '#1E90FF', '#4682B4', '#87CEEB'

        ax1.plot(self.num_list[0:], self.FRSP_time_maze[0:], '^--', label='Flow',linewidth=1, color='#FF8C00')
        ax1.plot(self.num_list[0:], self.Astar_time_maze[0:], 'o--', label='A*',linewidth=1, color='#1E90FF')
        ax1.plot(self.num_list[0:], self.cbsh2_time_maze[0:], 'o--', label='CBSH2-RTC',linewidth=1, color='#4682B4')
        ax1.plot(self.num_list[0:], self.lns_time_maze[0:], 'o--', label='LNS2', linewidth=1, color='#FFA500')
        # ax1.plot(self.num_list, self.run_cost_time[:, 0], '^--', label='Two_mip', linewidth=1, color='#D2691E')

        ax1.set_xlabel('number of robots')
        ax1.set_ylabel('makespan')
        ax1.tick_params('y')
        # ax1.ticklabel_format(style='sci', axis='y', scilimits=(0, 0))

        lines1, labels1 = ax1.get_legend_handles_labels()

        ax1.legend(lines1, labels1, fontsize='x-small', frameon=False, loc='upper left')
        # ax1.legend(fontsize='x-small', frameon=False)
        ax1.set_xlim([80, 520])
        ax1.set_xticks([i for i in range(100, 510, 100)])
        ax1.set_ylim([90, 140])
        # x-small [-1, 130]
        try:
            # 读取图片
            img = mpimg.imread('map/co500.png')  # 替换为你的图片路径
            # 创建 OffsetImage 对象，设置缩放比例
            imagebox = OffsetImage(img, zoom=0.15)  # 调整 zoom 参数以控制图片大小
            # 设置图片位置（右下角）
            ab = AnnotationBbox(imagebox, (0.99, 0.01),  # 右下角坐标（相对坐标，0到1）
                                xycoords='axes fraction',  # 使用轴的相对坐标
                                box_alignment=(1, 0),  # 右下对齐
                                frameon=False)  # 无边框
            ax1.add_artist(ab)
        except FileNotFoundError:
            print("嵌入的图片文件未找到，请检查路径！")
        # ax1.set_xlim([-50, 110])
        # ax1.set_xticks([i for i in range(0, 110, 50)])
        # ax1.set_ylim([24, 46])
        # ax1.set_yticks([i for i in range(800, 1401, 100)])

        plt.rcParams['pdf.fonttype'] = 42
        plt.rcParams['ps.fonttype'] = 42

        plt.grid(True)
        plt.tight_layout()
        plt.savefig('num-time_corridor-500.eps', bbox_inches='tight', dpi=300)
        plt.show()  # 显示图像


    def plot_improvement(self):
        # 创建第一个 Axes 对象，并绘制第一条数据集的曲线
        fig, ax1 = plt.subplots(figsize=(2.8, 2.5))
        # fig, ax1 = plt.subplots()

        # diff 三个相似的橙色
        # '#FF8C00' '#FFA500' '#FFD700'
        # same 三个相似的蓝色
        # '#1E90FF', '#4682B4', '#87CEEB'

        ax1.plot(self.num_list, self.improve_same_list, 'D-', label='improve_same', linewidth=2, color='#87CEEB')
        ax1.plot(self.num_list, self.improve_diff_list, 'D-', label='improve_diff', linewidth=2, color='#FFD700')
        ax1.set_ylabel('improvement ratio')
        ax1.tick_params('y')

        ax1.set_xlabel('number of drones')
        ax1.tick_params('y')
        # ax1.ticklabel_format(style='sci', axis='y', scilimits=(0, 0))
        # 创建第二个 Axes 对象，并绘制第二条数据集的曲线

        lines1, labels1 = ax1.get_legend_handles_labels()

        # x-small [-1, 130]
        ax1.legend(fontsize='x-small', frameon=False)
        ax1.set_xlim([-1, 105])
        # ax1.set_ylim([800, 1200])
        # ax1.set_yticks([i for i in range(800, 1201, 100)])
        ax1.set_ylim([0, 0.5])
        # ax2.set_ylim([0, 1800])

        plt.rcParams['pdf.fonttype'] = 42
        plt.rcParams['ps.fonttype'] = 42

        plt.grid(True)
        plt.tight_layout()
        plt.savefig('num-time_2.eps', bbox_inches='tight', dpi=300)
        plt.show()  # 显示图像


    def plot_improvement_three(self):
        # 创建第一个 Axes 对象，并绘制第一条数据集的曲线
        fig, ax1 = plt.subplots(figsize=(2.8, 2.5))
        # fig, ax1 = plt.subplots()

        # diff 三个相似的橙色
        # '#FF8C00' '#FFA500' '#FFD700'
        # same 三个相似的蓝色
        # '#1E90FF', '#4682B4', '#87CEEB'

        ax1.plot(self.num_list, self.improve_same_list, 'D-', label='improve_same', linewidth=2, color='#87CEEB')
        ax1.plot(self.num_list, self.improve_diff_list, 'D-', label='improve_diff', linewidth=2, color='#FFD700')
        ax1.set_ylabel('improvement ratio')
        ax1.tick_params('y')

        ax1.set_xlabel('number of drones')
        ax1.tick_params('y')
        # ax1.ticklabel_format(style='sci', axis='y', scilimits=(0, 0))
        # 创建第二个 Axes 对象，并绘制第二条数据集的曲线

        lines1, labels1 = ax1.get_legend_handles_labels()

        # x-small [-1, 130]
        ax1.legend(fontsize='x-small', frameon=False)
        ax1.set_xlim([-1, 105])
        # ax1.set_ylim([800, 1200])
        # ax1.set_yticks([i for i in range(800, 1201, 100)])
        ax1.set_ylim([0, 0.5])
        # ax2.set_ylim([0, 1800])

        plt.grid(True)
        plt.tight_layout()
        plt.savefig('num-time_2.eps', bbox_inches='tight', dpi=300)
        plt.show()  # 显示图像

    def plot_two(self):
        # 生成示例数据
        x = self.num_list
        y1 = self.t_res_same_list_mip
        y2 = self.improve_same_list

        # 创建第一个 Axes 对象，并绘制第一条数据集的曲线
        fig, ax1 = plt.subplots()
        ax1.plot(self.num_list, self.t_res_same_list_mip, 'o-', label='Ours_same')
        ax1.plot(self.num_list, self.t_res_same_list_rma, 'o-', label='RM-A_same')
        ax1.set_xlabel('Number of drones')
        ax1.set_ylabel('Lifetime')
        ax1.tick_params('y')
        ax1.ticklabel_format(style='sci', axis='y', scilimits=(0, 0))

        # 创建第二个 Axes 对象，并绘制第二条数据集的曲线
        ax2 = ax1.twinx()  # 注意这里使用 twinx()
        ax2.plot(self.num_list, self.improve_diff_list, '+-', label='improvement', color='r')
        ax2.set_ylabel('improvement', color='r')
        ax2.tick_params('y', colors='r')

        lines1, labels1 = ax1.get_legend_handles_labels()
        lines2, labels2 = ax2.get_legend_handles_labels()
        lines = lines1 + lines2
        labels = labels1 + labels2
        ax1.legend(lines, labels, fontsize='x-small', frameon=False, loc='lower right')
        # ax1.legend(lines, labels, loc='best')

        ax1.set_ylim([800, 1200])
        ax2.set_ylim([0, 0.5])
        # ax2.set_ylim([0, 1800])

        plt.show()  # 显示图像


    def plot_try(self):
        import numpy as np
        import matplotlib.pyplot as plt

        # 生成示例数据
        x = np.linspace(0, 10, 100)
        y1 = np.sin(x)
        y2 = np.exp(x)

        # 创建第一个 Axes 对象，并绘制第一条数据集的曲线
        fig, ax1 = plt.subplots()
        ax1.plot(x, y1, 'b-', label='sin(x)')
        ax1.set_xlabel('x')
        ax1.set_ylabel('sin(x)', color='b')
        ax1.tick_params('y', colors='b')

        # 创建第二个 Axes 对象，并绘制第二条数据集的曲线
        ax2 = ax1.twinx()  # 注意这里使用 twinx()
        ax2.plot(x, y2, 'r-', label='exp(x)')
        ax2.set_ylabel('exp(x)', color='r')
        ax2.tick_params('y', colors='r')

        # 添加图例
        lines1, labels1 = ax1.get_legend_handles_labels()
        lines2, labels2 = ax2.get_legend_handles_labels()
        lines = lines1 + lines2
        labels = labels1 + labels2
        ax1.legend(lines, labels, loc='best')

        plt.show()  # 显示图像


P1 = Plot()
P1.main()
print()

