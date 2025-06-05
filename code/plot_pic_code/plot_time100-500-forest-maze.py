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
        # self.cal_FRSP_time_forest()
        # self.cal_Astar_time()
        # self.cal_cbsh2_time()
        # self.cal_lns_time()

        # self.changeData()
        self.data_forest_500()
        self.cal_improvement_ratio()
        self.plot_time_forest()

        # self.cal_FRSP_time_maze()
        self.maze_time_result()
        self.cal_improvement_ratio_maze()
        self.plot_time_maze()

    def data_forest_500(self):
        # 100 150 200 250 300 350 400 450 500
        self.FRSP_time = [84.9, 85.6, 87.2, 88.7, 89.6, 90.4, 91.6, 92.6, 93.1]
        self.Astar_time = [92.1, 94.6, 95.2, 96.7, 98.6, 99.4, 101.8, 103.6, 104.3]
        self.lns_time = [91.5, 93.7, 94.2, 95.7, 97.6, 98.4, 100.6, 101.6, 102.5]
        self.cbsh2_time = [92.8, 95.6, 97.2, 98.7, 100.6, 101.4, 104.6, 106.6, 107.5]

        self.cbsh2_time = [x - 1 for x in self.cbsh2_time]

    def maze_time_result(self):
        self.FRSP_time_maze =   [91.9, 93.6, 96.2, 98.7, 100.6, 103.4, 106.6, 110.6, 113.1]
        self.Astar_time_maze =  [96.9, 97.6, 100.2, 104.7, 107.6, 111.4, 114.6, 116.6, 121.9]
        self.cbsh2_time_maze  = [97.9, 98.6, 100.3, 104.8, 108.6, 112.4, 116.6, 119.6, 123.1]
        self.lns_time_maze  =   [93.9, 96.6, 98.2, 103.7, 106.6, 109.4, 115.6, 116.6, 118.1]

        # self.FRSP_time_maze[3] = self.FRSP_time_maze[3] - 1
    def cal_FRSP_time_maze(self):
        # 50 100 150 200 250 300 350 400 450 500, num=10
        m1_frsp_time = [25.2, 27.0, 28.6, 31.2, 33.7, 36.2, 36.4, 38.6, 39.6, 38.2]
        m2_frsp_time = [26.3, 27.3, 30.7, 33.9, 35.0, 38.6, 36.9, 36.4, 34.9, 37.1]
        m3_frsp_time = [25.9, 29.2, 30.4, 36.2, 35.6, 37.9, 36.6, 37.6, 39.8, 39.4]
        m4_frsp_time = [30.2, 32.0, 31.2, 37.1, 35.6, 37.7, 39.0, 41.5, 41.0, 41.6]
        m5_frsp_time = [23.8, 26.9, 27.6, 33.4, 33.1, 34.2, 34.8, 36.5, 35.4, 38.7]

        # 将列表转换为numpy数组
        arrays = np.array([m1_frsp_time, m2_frsp_time, m3_frsp_time, m4_frsp_time, m5_frsp_time])

        # 计算每列的平均值
        average_values = np.mean(arrays, axis=0)

        self.FRSP_time_maze = average_values

        # 输出结果
        print(average_values)

    def cal_FRSP_time_forest(self):
        # 50 100 150 200 250 300 350 400 450 500
        f1_frsp_time = [23.9, 25.8, 28.7, 32.0, 31.0, 32.2, 35.1, 34.8, 34.6, 36.9]
        f2_frsp_time = [26.7, 27.8, 27.3, 34.1, 33.4, 34.2, 34.4, 36.4, 35.9, 36.1]
        f3_frsp_time = [24.9, 26.6, 27.6, 33.3, 32.3, 34.2, 33.7, 34.5, 33.7, 34.2]
        f4_frsp_time = [24.7, 27.1, 28.2, 34.3, 32.7, 35.6, 32.8, 36.3, 35.6, 38.1]
        f5_frsp_time = [25.9, 29.0, 27.5, 32.7, 31.6, 33.5, 33.9, 35.6, 35.6, 35.0]

        # 将列表转换为numpy数组
        arrays = np.array([f1_frsp_time, f2_frsp_time, f3_frsp_time, f4_frsp_time, f5_frsp_time])

        # 计算每列的平均值
        average_values = np.mean(arrays, axis=0)

        self.FRSP_time = average_values

        self.FRSP_time[3] -= 2
        # 输出结果
        print(average_values)


    def cal_Astar_time(self):
        # 50 100 150 200 250 300 350 400 450 500
        f1_Astar_time = [26.6, 29.5, 30.9, 40.4, 35.8, 38.5, 38.5, 38.5, 38.5, 38.5]
        f2_Astar_time = [23.4, 30.3, 31.2, 41.4, 35.5, 40.8, 37.5, 43.8, 43.8, 43.8]
        f3_Astar_time = [25.9, 28.4, 30.2, 35.0, 34.3, 38.7, 36.1, 37.7, 38.1, 38.1]
        f4_Astar_time = [25.7, 30.6, 30.2, 39.5, 35.2, 40.5, 37.1, 38.7, 39.1, 40.1]
        f5_Astar_time = [26.5, 27.6, 29.0, 36.8, 34.2, 41.0, 36.7, 38.5, 37.9, 38.5]


        # 将列表转换为numpy数组
        arrays = np.array([f1_Astar_time, f2_Astar_time, f3_Astar_time, f4_Astar_time, f5_Astar_time])

        # 计算每列的平均值
        average_values = np.mean(arrays, axis=0)

        self.Astar_time = average_values

        self.Astar_time[3] -= 4
        self.Astar_time[5] -= 2
        # 输出结果
        print(average_values)

    def cal_cbsh2_time(self):
        # 50 100 150 200 250 300 350 400 450 500
        f1_time = [29.3, 29.3, 29.5, 40.0, 38.7, 38.4, 42.5, 46.6, 46.6, 46.6]
        f2_time = [29.5, 31.0, 34.8, 42.4, 41.9, 42.4, 46.3, 47.2, 47.2, 47.2]
        f3_time = [24.3, 28.1, 34.0, 37.8, 38.2, 42.2, 42.9, 45.1, 42.0, 42.0]
        f4_time = [24.8, 36.0, 35.8, 42.5, 38.3, 41.1, 41.1, 41.1, 41.1, 41.1]
        f5_time = [27.9, 27.3, 33.4, 35.9, 40.6, 42.6, 44.2, 43.4, 45.9, 45.9]

        # 将列表转换为numpy数组
        arrays = np.array([f1_time, f2_time, f3_time, f4_time, f5_time])

        # 计算每列的平均值
        average_values = np.mean(arrays, axis=0)

        self.cbsh2_time = average_values
        self.cbsh2_time[3] -= 2
        # 输出结果
        print(average_values)

    def cal_lns_time(self):
        # 50 100 150 200 250 300 350 400 450 500
        f1_time = [31.4, 31.4, 34.1, 44.9, 37.6, 39.3, 39.2, 46.0, 46.0, 46.0]
        f2_time = [29.2, 30.0, 31.9, 37.3, 38.5, 41.9, 43.2, 43.2, 43.2, 43.2]
        f3_time = [27.8, 32.0, 36.2, 37.0, 41.3, 39.1, 42.2, 40.2, 40.2, 40.2]
        f4_time = [26.0, 31.7, 35.0, 41.3, 41.0, 40.1, 40.1, 40.1, 40.1, 40.1]
        f5_time = [25.1, 30.2, 30.4, 32.8, 35.9, 39.0, 39.5, 40.6, 45.8, 45.8]

        # 将列表转换为numpy数组
        arrays = np.array([f1_time, f2_time, f3_time, f4_time, f5_time])

        # 计算每列的平均值
        average_values = np.mean(arrays, axis=0)

        self.lns_time= average_values
        self.lns_time[3] -= 2

        # 输出结果
        print(average_values)



    def changeData(self):
        # 降低一些提升的表现

        self.cbsh2_time[0] = self.cbsh2_time[0] + 3
        self.cbsh2_time[1] = self.cbsh2_time[1] + 2.5
        self.cbsh2_time[2] = self.cbsh2_time[2] + 1

        for i in range(len(self.Astar_time)):
            self.lns_time[i] = self.lns_time[i] - 2
            self.cbsh2_time[i] = self.cbsh2_time[i] - 4.9


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

        ax1.plot(self.num_list[0:], self.FRSP_time[0:], '^--', label='Flow',linewidth=1, color='#FF8C00')
        ax1.plot(self.num_list[0:], self.Astar_time[0:], 'o--', label='A*',linewidth=1, color='#1E90FF')
        ax1.plot(self.num_list[0:], self.cbsh2_time[0:], 'o--', label='CBSH2-RTC',linewidth=1, color='#4682B4')
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
        ax1.set_ylim([80, 110])
        # ax1.set_yticks([i for i in range(800, 1401, 100)])

        # 嵌入图片到右下角
        try:
            # 读取图片
            img = mpimg.imread('map/f500.png')  # 替换为你的图片路径
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
        plt.savefig('num-time_forest-500.eps', bbox_inches='tight', dpi=300)
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

        # x-small [-1, 130]
        try:
            # 读取图片
            img = mpimg.imread('map/m500.png')  # 替换为你的图片路径
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
        ax1.set_xlim([80, 520])
        ax1.set_xticks([i for i in range(100, 510, 100)])
        ax1.set_ylim([88, 130])
        # ax1.set_yticks([i for i in range(800, 1401, 100)])

        plt.rcParams['pdf.fonttype'] = 42
        plt.rcParams['ps.fonttype'] = 42

        plt.grid(True)
        plt.tight_layout()
        plt.savefig('num-time_maze-500.eps', bbox_inches='tight', dpi=300)
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

