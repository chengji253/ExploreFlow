import numpy as np
import matplotlib as mpl
import matplotlib.pyplot as plt
import json as js
from matplotlib.offsetbox import OffsetImage, AnnotationBbox
import matplotlib.pyplot as plt
import matplotlib.image as mpimg


class Plot:

    def __init__(self):

        self.data_now = None

        self.K = [1, 2, 3, 4, 5, 6, 7, 8, 9, 10]

        self.ave_time_L_5 = [0.97, 1.26, 1.93, 2.02, 2.17, 2.27, 2.39, 2.40, 2.65, 2.71]
        self.T_result_L_5 = [102.8, 97.8, 96.7, 96.2, 95.4, 95.4, 96.6, 96.4, 97.6, 98.9]

        self.ave_time_L_10 = [1.19, 1.56, 1.91, 2.11, 2.22, 2.25, 2.33, 2.32, 2.42, 2.44]
        self.T_result_L_10 = [99.6, 97.0, 94.7, 94.3, 93.7, 93.8, 94.1, 94.2, 94.2, 94.8]

        self.ave_time_L_15 = [1.23, 1.86, 2.05, 2.21, 2.22, 2.31, 2.41, 2.45, 2.61, 2.63]
        self.T_result_L_15 = [98.1, 96.9, 93.1, 93.2, 92.6, 93.1, 92.5, 93.4, 94.7, 93.9]

        self.ave_time_L_20 = [1.41, 1.92, 2.01, 2.14, 2.25, 2.41, 2.36, 2.53, 2.52, 2.58]
        self.T_result_L_20 = [99.6, 96.7, 94.6, 93.6, 93.8, 93.4, 92.8, 93.1, 94.6, 93.0]

        self.ave_time_L_25 = [1.50, 2.04, 2.13, 2.20, 2.28, 2.38, 2.52, 2.57, 2.62, 2.65]
        self.T_result_L_25 = [101.3, 97.1, 96.6, 93.6, 93.1, 93.5, 92.8, 93.1, 93.2, 93.1]

        self.ave_time_L_30 = [1.71, 2.01, 2.16, 2.19, 2.27, 2.34, 2.56, 2.64, 2.68, 2.71]
        self.T_result_L_30 = [105.3, 101.2, 96.2, 94.2, 93.9, 93.4, 92.2, 93.4, 93.0, 93.0]

    def divide_ave_time(self):
        x = 2.25
        self.ave_time_L_5 = [val / x for val in self.ave_time_L_5]
        self.ave_time_L_10 = [val / x for val in self.ave_time_L_10]
        self.ave_time_L_15 = [val / x for val in self.ave_time_L_15]
        self.ave_time_L_20 = [val / x for val in self.ave_time_L_20]
        self.ave_time_L_25 = [val / x for val in self.ave_time_L_25]
        self.ave_time_L_30 = [val / x for val in self.ave_time_L_30]


    def main(self):
        self.divide_ave_time()
        # self.plot_T_result_vs_K()
        # self.plot_ave_time_vs_K()

        self.plot_T_result_vs_K_new()
        self.plot_ave_time_vs_K_new()


    def plot_T_result_vs_K(self):
        fig, ax1 = plt.subplots(figsize=(3, 2.5))
        colors = ['#1f77b4', '#ff7f0e', '#2ca02c', '#d62728', '#9467bd', '#8c564b']  # Distinct colors
        ax1.plot(self.K, self.T_result_L_5, label='L_pre=5', marker='o', markersize=3, color=colors[0])
        ax1.plot(self.K, self.T_result_L_10, label='L_pre=10', marker='s', markersize=3, color=colors[1])
        ax1.plot(self.K, self.T_result_L_15, label='L_pre=15', marker='^', markersize=3, color=colors[2])
        ax1.plot(self.K, self.T_result_L_20, label='L_pre=20', marker='v', markersize=3, color=colors[3])
        ax1.plot(self.K, self.T_result_L_25, label='L_pre=25', marker='D', markersize=3, color=colors[4])
        ax1.plot(self.K, self.T_result_L_30, label='L_pre=30', marker='*', markersize=3, color=colors[5])
        ax1.set_xlabel('K')
        ax1.set_ylabel('time')
        ax1.legend(fontsize=7, frameon=False)
        # ax1.grid(True)
        plt.rcParams['pdf.fonttype'] = 42
        plt.rcParams['ps.fonttype'] = 42

        ax1.set_xlim([0.5, 10.5])
        ax1.set_xticks([i for i in range(1, 11, 1)])
        # ax1.set_ylim([9, 18])
        ax1.set_ylim([90, 106])
        ax1.set_yticks([i for i in range(90, 106, 5)])

        ax1.grid(True, linestyle='--', linewidth=0.5)

        plt.tight_layout()
        plt.savefig('T_K.eps')
        plt.show()
        plt.close()

    def plot_ave_time_vs_K(self):
        fig, ax2 = plt.subplots(figsize=(3, 2.5))
        colors = ['#1f77b4', '#ff7f0e', '#2ca02c', '#d62728', '#9467bd', '#8c564b']  # Distinct colors
        ax2.plot(self.K, self.ave_time_L_5, label='L_pre=5', marker='o', markersize=3, color=colors[0])
        ax2.plot(self.K, self.ave_time_L_10, label='L_pre=10', marker='s', markersize=3, color=colors[1])
        ax2.plot(self.K, self.ave_time_L_15, label='L_pre=15', marker='^', markersize=3, color=colors[2])
        ax2.plot(self.K, self.ave_time_L_20, label='L_pre=20', marker='v', markersize=3, color=colors[3])
        ax2.plot(self.K, self.ave_time_L_25, label='L_pre=25', marker='D', markersize=3, color=colors[4])
        ax2.plot(self.K, self.ave_time_L_30, label='L_pre=30', marker='*', markersize=3, color=colors[5])
        ax2.set_xlabel('K')
        ax2.set_ylabel('computation time (s)')
        ax2.legend(fontsize=7, frameon=False)

        ax2.set_xlim([0.5, 10.5])
        ax2.set_xticks([i for i in range(1, 11, 1)])

        plt.rcParams['pdf.fonttype'] = 42
        plt.rcParams['ps.fonttype'] = 42
        ax2.grid(True)
        plt.tight_layout()
        plt.savefig('ave_time_K.eps')
        plt.show()
        plt.close()


    def plot_T_result_vs_K_new(self):

        # Set global plotting style
        # plt.rcParams.update({
        #     'font.family': 'Times New Roman',
        #     'font.size': 8,
        #     'pdf.fonttype': 42,
        #     'ps.fonttype': 42,
        # })

        fig, ax = plt.subplots(figsize=(3, 2.5))  # Slightly wider
        colors = ['#4E79A7', '#F28E2B', '#E15759', '#76B7B2', '#59A14F', '#EDC948']
        markers = ['o', 's', '^', 'v', 'D', '*']
        labels = ['L_pre=5', 'L_pre=10', 'L_pre=15', 'L_pre=20', 'L_pre=25', 'L_pre=30']
        data_series = [
            self.T_result_L_5, self.T_result_L_10, self.T_result_L_15,
            self.T_result_L_20, self.T_result_L_25, self.T_result_L_30
        ]

        for i, data in enumerate(data_series):
            ax.plot(self.K, data,
                    label=labels[i],
                    marker=markers[i],
                    color=colors[i],
                    markersize=4,
                    linewidth=2.0,
                    markerfacecolor='white',
                    markeredgewidth=2.0)

        ax.set_xlabel('K', fontsize=9)
        ax.set_ylabel('makespan', fontsize=9)
        ax.set_xlim([0.5, 10.5])
        ax.set_xticks(range(1, 11))
        ax.set_ylim([90, 106])
        ax.set_yticks(range(90, 106, 5))
        ax.grid(True, linestyle='--', linewidth=0.5)
        ax.legend(fontsize=8, frameon=False)

        plt.tight_layout()
        plt.savefig('T_K.pdf', bbox_inches='tight', dpi=600)
        plt.show()
        plt.close()

    def plot_ave_time_vs_K_new(self):
        # 设置统一风格参数
        # plt.rcParams.update({
        #     'font.family': 'Times New Roman',
        #     'font.size': 8,
        #     'pdf.fonttype': 42,
        #     'ps.fonttype': 42,
        # })

        fig, ax = plt.subplots(figsize=(3.0, 2.5))
        colors = ['#4E79A7', '#F28E2B', '#E15759', '#76B7B2', '#59A14F', '#EDC948']
        markers = ['o', 's', '^', 'v', 'D', '*']
        labels = ['L_pre=5', 'L_pre=10', 'L_pre=15', 'L_pre=20', 'L_pre=25', 'L_pre=30']
        data_series = [
            self.ave_time_L_5, self.ave_time_L_10, self.ave_time_L_15,
            self.ave_time_L_20, self.ave_time_L_25, self.ave_time_L_30
        ]

        for i, data in enumerate(data_series):
            ax.plot(self.K, data,
                    label=labels[i],
                    marker=markers[i],
                    color=colors[i],
                    markersize=4,
                    linewidth=2.0,
                    markerfacecolor='white',
                    markeredgewidth=2.0)

        ax.set_xlabel('K', fontsize=9)
        ax.set_ylabel('average computation time', fontsize=9)
        ax.set_xlim([0.5, 10.5])
        ax.set_xticks(range(1, 11))
        ax.grid(True, linestyle='--', linewidth=0.5)
        ax.legend(fontsize=8, frameon=False)

        plt.tight_layout()
        plt.savefig('ave_time_K.pdf', bbox_inches='tight', dpi=600)
        plt.show()
        plt.close()

P1 = Plot()
P1.main()