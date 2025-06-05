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

        self.num_list = [10, 20, 30, 40, 50, 60, 70, 80, 90, 100]
        self.num_len = len(self.num_list)

        self.FRSP_time = 1*np.ones((self.num_len, 1), dtype=float)

        self.Astar_time = 2*np.ones((self.num_len, 1), dtype=float)

        self.ecbs_time = 3*np.ones((self.num_len, 1), dtype=float)

        self.sipp_time = 4*np.ones((self.num_len, 1), dtype=float)


    def main(self):
        # self.date_set()
        # self.plot_time_forest()
        # self.cal_improvement_ratio()

        self.date_set_one()
        self.plot_time_maze()
        self.cal_improvement_ratio()

    def date_set(self):
        # 10, 20, 30, 40, 50, 60, 70, 80, 90, 100
        # clutter
        self.FRSP_time =  [50.8, 51.3, 52.0, 52.8, 53.7, 54.7, 55.8, 58.0, 59.3, 61.2]
        self.Astar_time = [50.9, 52.9, 53.8, 56.5, 58.5, 60.9, 61.2, 62.1, 64.5, 65.6]
        self.ecbs_time =  [51.0, 52.5, 54.3, 55.2, 56.2, 59.3, 61.5, 63.9, 64.3, 66.3]
        self.sipp_time =  [51.1, 53.8, 53.6, 56.5, 57.5, 59.4, 61.3, 62.1, 65.7, 67.7]

        self.Astar_time = [i - 0.5 for i in self.Astar_time]
        self.ecbs_time = [i - 0.5 for i in self.ecbs_time]
        self.sipp_time = [i - 1 for i in self.sipp_time]

    def date_set_one(self):
        # 10, 20, 30, 40, 50, 60, 70, 80, 90, 100
        # corridor
        self.FRSP_time = [48.9, 50.1, 51.5, 52.3, 52.7, 55.7, 57.8, 59.0, 61.3, 62.4]
        self.Astar_time = [49.1, 50.9, 51.8, 55.5, 58.5, 59.9, 61.2, 62.1, 63.5, 64.6]
        self.ecbs_time = [49.0, 51.5, 52.3, 55.2, 59.2, 60.3, 61.5, 62.2, 64.3, 65.3]
        self.sipp_time = [49.1, 51.3, 52.6, 56.5, 60.5, 60.7, 62.2, 63.1, 64.7, 65.7]

        # self.Astar_time = [i - 0.2 for i in self.Astar_time]
        # self.ecbs_time = [i - 0.5 for i in self.ecbs_time]
        # self.sipp_time = [i - 1 for i in self.sipp_time]

    def cal_improvement_ratio(self):
        Astar_ave = np.mean(self.Astar_time)
        FRSP_ave = np.mean(self.FRSP_time)
        ecbs_ave = np.mean(self.ecbs_time)
        sipp_ave = np.mean(self.sipp_time)

        Astar_improve = (Astar_ave - FRSP_ave) / Astar_ave * 100
        ecbs_improve = (ecbs_ave - FRSP_ave) / ecbs_ave * 100
        sipp_improve = (sipp_ave - FRSP_ave) / sipp_ave * 100

        print("-----")
        print("Astar improvement ratio " + str(Astar_improve))
        print("ecbs improvement ratio " + str(ecbs_improve))
        print("sipp improvement ratio " + str(sipp_improve))

    def plot_time_forest(self):
        # 创建第一个 Axes 对象，并绘制第一条数据集的曲线
        fig, ax1 = plt.subplots(figsize=(3, 2.5))
        # fig, ax1 = plt.subplots()
        # fig, ax1 = plt.subplots()

        # diff 三个相似的橙色
        # '#FF8C00' '#FFA500' '#FFD700'
        # same 三个相似的蓝色
        # '#1E90FF', '#4682B4', '#87CEEB'

        ax1.plot(self.num_list, self.FRSP_time, '^--', label='Flow',linewidth=1, color='#FF8C00')
        ax1.plot(self.num_list, self.Astar_time, 'o--', label='A*',linewidth=1, color='darkgreen')
        ax1.plot(self.num_list, self.ecbs_time, 'o--', label='ECBS',linewidth=1, color='darkred')
        ax1.plot(self.num_list, self.sipp_time, 'o--', label='P-SIPP', linewidth=1, color='#DFA53D')
        # ax1.plot(self.num_list, self.run_cost_time[:, 0], '^--', label='Two_mip', linewidth=1, color='#D2691E')

        ax1.set_xlabel('number of robots')
        ax1.set_ylabel('makespan')
        ax1.tick_params('y')
        # ax1.ticklabel_format(style='sci', axis='y', scilimits=(0, 0))

        lines1, labels1 = ax1.get_legend_handles_labels()

        # ax1.legend(lines, labels, fontsize='x-small', frameon=False, loc='upper right')
        ax1.legend(fontsize='x-small', frameon=False)
        try:
            # 读取图片
            img = mpimg.imread('map/cl100.png')  # 替换为你的图片路径
            # 创建 OffsetImage 对象，设置缩放比例
            imagebox = OffsetImage(img, zoom=0.19)  # 调整 zoom 参数以控制图片大小
            # 设置图片位置（右下角）
            ab = AnnotationBbox(imagebox, (0.99, 0.01),  # 右下角坐标（相对坐标，0到1）
                                xycoords='axes fraction',  # 使用轴的相对坐标
                                box_alignment=(1, 0),  # 右下对齐
                                frameon=False)  # 无边框
            ax1.add_artist(ab)
        except FileNotFoundError:
            print("嵌入的图片文件未找到，请检查路径！")
        # x-small [-1, 130]

        ax1.set_xlim([5, 105])
        # ax1.set_xticks([i for i in range(0, 110, 50)])
        # ax1.set_ylim([9, 18])
        ax1.set_ylim([45, 70])
        # ax1.set_yticks([i for i in range(800, 1401, 100)])

        plt.rcParams['pdf.fonttype'] = 42
        plt.rcParams['ps.fonttype'] = 42

        plt.grid(True)
        plt.tight_layout()
        plt.savefig('num-time_clutter-100.eps', bbox_inches='tight', dpi=300)
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

        ax1.plot(self.num_list, self.FRSP_time, '^--', label='Flow',linewidth=1, color='#FF8C00')
        ax1.plot(self.num_list, self.Astar_time, 'o--', label='A*',linewidth=1, color='darkgreen')
        ax1.plot(self.num_list, self.ecbs_time, 'o--', label='ECBS',linewidth=1, color='darkred')
        ax1.plot(self.num_list, self.sipp_time, 'o--', label='P-SIPP', linewidth=1, color='#DFA53D')
        # ax1.plot(self.num_list, self.run_cost_time[:, 0], '^--', label='Two_mip', linewidth=1, color='#D2691E')

        ax1.set_xlabel('number of robots')
        ax1.set_ylabel('makespan')
        ax1.tick_params('y')
        # ax1.ticklabel_format(style='sci', axis='y', scilimits=(0, 0))

        lines1, labels1 = ax1.get_legend_handles_labels()

        # ax1.legend(lines, labels, fontsize='x-small', frameon=False, loc='upper right')
        ax1.legend(fontsize='x-small', frameon=False, loc='upper left')

        # x-small [-1, 130]
        ax1.set_ylim([45, 70])
        ax1.set_xlim([5, 105])
        # ax1.set_ylim([10, 18])
        # ax1.set_xticks([i for i in range(0, 110, 50)])
        # ax1.set_ylim([24, 43])
        # ax1.set_yticks([i for i in range(800, 1401, 100)])
        try:
            # 读取图片
            img = mpimg.imread('map/co100.png')  # 替换为你的图片路径
            # 创建 OffsetImage 对象，设置缩放比例
            imagebox = OffsetImage(img, zoom=0.19)  # 调整 zoom 参数以控制图片大小
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
        plt.savefig('num-time_corridor-100.eps', bbox_inches='tight', dpi=300)
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
# print()

