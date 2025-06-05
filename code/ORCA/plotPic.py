import matplotlib
import matplotlib.pyplot as pyplot
from matplotlib.path import Path
import matplotlib.patches as patches
from matplotlib.patches import Polygon
import matplotlib.cm as cmx
import matplotlib.colors as colors
import numpy as np
import matplotlib.pyplot as plt
import random
from scipy.ndimage import gaussian_filter  # 用于平滑热力图
import json


# def get_cmap(n, name='hsv'):
#     '''Returns a function that maps each index in 0, 1, ..., n-1 to a distinct
#     RGB color; the keyword argument name must be a standard mpl colormap name.'''
#     return plt.cm.get_cmap(name, n)

def get_cmap(N):
    '''Returns a function that maps each index in 0, 1, ... N-1 to a distinct RGB color.'''
    color_norm = colors.Normalize(vmin=0, vmax=N - 1)
    scalar_map = cmx.ScalarMappable(norm=color_norm, cmap='hsv')

    def map_index_to_rgb_color(index):
        return scalar_map.to_rgba(index)

    return map_index_to_rgb_color

def visualize_traj_dynamic(agents, obs_info, goals, boundary, name, swarm, time):
    # 使用Agg后端进行非交互式绘图
    matplotlib.use('Agg')
    # 启用抗锯齿功能
    matplotlib.rcParams['path.simplify'] = True
    matplotlib.rcParams['path.simplify_threshold'] = 1.0
    matplotlib.rcParams['figure.dpi'] = 300

    # 创建图形并设置大小
    figure = plt.figure(figsize=(7, 10))
    ax = figure.add_subplot(1, 1, 1)
    ax.grid(True, linestyle='--', alpha=0.7)
    # 使用更具视觉吸引力的颜色映射
    # cmap = get_cmap(len(agents), 'tab20')
    cmap = get_cmap(len(agents))
    # 绘制障碍物

    for vertices in obs_info:
        poly = patches.Polygon(
            vertices,
            facecolor='palegreen',  # 设置填充颜色为绿色
            edgecolor='darkgreen',  # 设置边框颜色为深绿色
            linewidth=2,  # 设置边框宽度
            fill=True,
            alpha=1,
            zorder=2
        )
        ax.add_patch(poly)

    legend_elements = []
    for i in range(0, len(agents)):
        # 如果机器人不存在，则跳过
        if agents[i].agent_exist_ is False:
            continue
        # 绘制机器人
        robot = patches.Circle(
            (agents[i].position_.x_, agents[i].position_.y_),
            radius=agents[i].radius_,
            facecolor=cmap(i),
            edgecolor='black',
            linewidth=0.8,
            ls='solid',
            alpha=0.9,
            zorder=2)
        ax.add_patch(robot)
        # 绘制速度箭头
        # ax.text(agents[i].position_.x_, agents[i].position_.y_, str(i),
        #         fontsize=12, color='black', ha='center', va='center')
        ax.arrow(agents[i].position_.x_, agents[i].position_.y_,
                 agents[i].velocity_.x_, agents[i].velocity_.y_, head_width=0.5, head_length=1, fc=cmap(i), ec=cmap(i), alpha=0.8)
        # 绘制目标点
        if hasattr(goals[i], 'x_') and hasattr(goals[i], 'y_'):
            goal_marker, = ax.plot([goals[i].x_], [goals[i].y_], '*', color=cmap(i), markersize=8, linewidth=3.0)
        # 绘制机器人路径
        path = swarm.des_path_pos[i]
        path = np.array(path)
        path_line, = plt.plot(path[:, 0], path[:, 1], color=cmap(i), linewidth=1.2)

        # # 计算路径的中心点
        # center_idx = len(path) // 2
        # center_idx += random.randint(-3, 3)
        # center_x = path[center_idx, 0]
        # center_y = path[center_idx, 1]
        #
        # # 在中心位置添加数字i
        # plt.text(center_x, center_y, str(i),
        #          color=cmap(i),
        #          ha='center',  # 水平居中
        #          va='center',  # 垂直居中
        #          fontsize=12)  # 字体大小可调整

    # # 添加时间信息
    if time:
        ax.text(1, boundary[1][1] - 2, f'$t={time:.1f} s$', fontsize=20, fontweight='bold')
    # # 设置坐标轴
    # out_size = 2
    ax.set_aspect('equal')
    # boundary = [[-100, 100], [-100, 100]]
    out_size = 0
    ax.set_xlim(boundary[0][0] - out_size, boundary[0][1] + out_size - 1)
    ax.set_ylim(boundary[1][0] - out_size, boundary[1][1] + out_size + 3)

    # 保存图片
    if name:
        plt.savefig(name, dpi=300, bbox_inches='tight')
    # 清除当前图形
    plt.cla()
    plt.close(figure)
    return figure


def visualize_traj_dynamic_hot_map(agents, obs_info, goals, boundary, name, swarm, time):
    # 使用Agg后端进行非交互式绘图
    matplotlib.use('Agg')
    # 启用抗锯齿功能
    matplotlib.rcParams['path.simplify'] = True
    matplotlib.rcParams['path.simplify_threshold'] = 1.0
    matplotlib.rcParams['figure.dpi'] = 300

    # 创建图形并设置固定大小
    figure = plt.figure(figsize=(7, 10), dpi=300)
    ax = figure.add_subplot(1, 1, 1)
    ax.grid(True, linestyle='--', alpha=0.7)

    # 绘制障碍物
    for vertices in obs_info:
        poly = patches.Polygon(
            vertices,
            facecolor='palegreen',
            edgecolor='darkgreen',
            linewidth=2,
            fill=True,
            alpha=1,
            zorder=2
        )
        ax.add_patch(poly)

    # 收集机器人位置
    positions = []
    for i in range(len(agents)):
        if agents[i].agent_exist_:
            positions.append([agents[i].position_.x_, agents[i].position_.y_])
    positions = np.array(positions)

    max_density = 0
    mean_density = 0
    non_zero_mean_density = 0

    # 如果有机器人，绘制热力图
    if len(positions) > 0:
        # 定义热力图的分辨率（bins）
        x_bins = np.linspace(boundary[0][0], boundary[0][1], 100)
        y_bins = np.linspace(boundary[1][0], boundary[1][1], 100)

        # 计算二维直方图（密度）
        heatmap, xedges, yedges = np.histogram2d(
            positions[:, 0], positions[:, 1],
            bins=[x_bins, y_bins],
            density=False  # 使用原始计数
        )

        # 应用高斯平滑
        heatmap = gaussian_filter(heatmap, sigma=1.0)

        # 统计最大值和平均值
        max_density = heatmap.max()
        mean_density = heatmap.mean()
        non_zero_mean_density = heatmap[heatmap > 0].mean() if np.any(heatmap > 0) else 0

        # 设置全局最大密度值（可根据实际数据调整）
        global_max = 1.5  # 基于0-4范围，留余量

        # 归一化到全局最大值
        heatmap = heatmap / global_max

        # 绘制热力图
        im = ax.imshow(
            heatmap.T,
            origin='lower',
            cmap='hot_r',
            interpolation='bilinear',
            extent=[boundary[0][0], boundary[0][1], boundary[1][0], boundary[1][1]],
            alpha=0.6,
            zorder=1,
            vmin=0,
            vmax=1
        )
        # 添加颜色条，固定样式
        plt.colorbar(
            im,
            ax=ax,
            label='Robot Density',
            shrink=0.6,
            aspect=20,
            pad=0.02,
            # ticks=[0, 1, 2]  # 固定刻度
            ticks=[0, 0.2, 0.4, 0.6, 0.8, 1.0]  # 固定刻度
        )

    # 添加时间信息，固定位置
    if time is not None:
        ax.text(
            1, boundary[1][1] - 2,
            f'$t={time:.1f} s$',
            fontsize=20,
            fontweight='bold',
            bbox=dict(facecolor='white', alpha=0.8, edgecolor='none')  # 添加背景
        )

    # 设置坐标轴，固定范围
    ax.set_aspect('equal')
    out_size = 0
    ax.set_xlim(boundary[0][0] - out_size, boundary[0][1] + out_size - 1)
    ax.set_ylim(boundary[1][0] - out_size, boundary[1][1] + out_size + 3)

    # 保存图片，固定参数
    if name:
        plt.savefig(
            name,
            dpi=300,
            bbox_inches='tight',
            pad_inches=0.1,
            format='png'  # 确保PNG格式，适合视频
        )

    # 清除当前图形
    plt.cla()
    plt.close(figure)
    return max_density, mean_density, non_zero_mean_density

def visualize_traj_wait_red_robot(agents, obs_info, goals, boundary, name, swarm, time):
    # 使用Agg后端进行非交互式绘图
    matplotlib.use('Agg')
    # 启用抗锯齿功能
    matplotlib.rcParams['path.simplify'] = True
    matplotlib.rcParams['path.simplify_threshold'] = 1.0
    matplotlib.rcParams['figure.dpi'] = 300

    # 创建图形并设置大小
    figure = plt.figure(figsize=(7, 10))
    ax = figure.add_subplot(1, 1, 1)
    ax.grid(True, linestyle='--', alpha=0.7)

    # 绘制障碍物
    for vertices in obs_info:
        poly = patches.Polygon(
            vertices,
            facecolor='palegreen',  # 设置填充颜色为绿色
            edgecolor='darkgreen',  # 设置边框颜色为深绿色
            linewidth=2,  # 设置边框宽度
            fill=True,
            alpha=1,
            zorder=2
        )
        ax.add_patch(poly)

    legend_elements = []
    num_wait_red = 0
    for i in range(0, len(agents)):
        # 如果机器人不存在，则跳过
        if agents[i].agent_exist_ is False:
            continue

        # 计算机器人速度大小
        speed = np.sqrt(agents[i].velocity_.x_ ** 2 + agents[i].velocity_.y_ ** 2)
        # 根据速度设置颜色：速度<0.5为红色，>=0.5为灰色
        robot_color = 'red' if speed < 0.5 else 'gray'

        if speed < 0.5:
            num_wait_red += 1

        # 绘制机器人
        robot = patches.Circle(
            (agents[i].position_.x_, agents[i].position_.y_),
            radius=agents[i].radius_,
            facecolor=robot_color,
            edgecolor='black',
            linewidth=0.8,
            ls='solid',
            alpha=0.9,
            zorder=2)
        ax.add_patch(robot)

        # 绘制速度箭头
        ax.arrow(agents[i].position_.x_, agents[i].position_.y_,
                 agents[i].velocity_.x_, agents[i].velocity_.y_,
                 head_width=0.5, head_length=1, fc=robot_color, ec=robot_color, alpha=0.8)

        # 绘制目标点
        if hasattr(goals[i], 'x_') and hasattr(goals[i], 'y_'):
            goal_marker, = ax.plot([goals[i].x_], [goals[i].y_], '*', color='gray', markersize=8, linewidth=3.0)

        # 绘制机器人路径
        path = swarm.des_path_pos[i]
        path = np.array(path)
        path_line, = plt.plot(path[:, 0], path[:, 1], color='gray', linewidth=1.2)

    # 添加时间信息
    if time:
        ax.text(1, boundary[1][1] - 2, f'$t={time:.1f} s$', fontsize=20, fontweight='bold')

    # 设置坐标轴
    ax.set_aspect('equal')
    out_size = 0
    ax.set_xlim(boundary[0][0] - out_size, boundary[0][1] + out_size - 1)
    ax.set_ylim(boundary[1][0] - out_size, boundary[1][1] + out_size + 3)

    # 保存图片
    if name:
        plt.savefig(name, dpi=300, bbox_inches='tight')

    # 清除当前图形
    plt.cla()
    plt.close(figure)
    return num_wait_red

def visualize_traj_dynamic_txt(agents, obs_info, goals, name):
    # 使用Agg后端进行非交互式绘图
    matplotlib.use('Agg')
    # 启用抗锯齿功能
    matplotlib.rcParams['path.simplify'] = True
    matplotlib.rcParams['path.simplify_threshold'] = 1.0
    matplotlib.rcParams['figure.dpi'] = 300

    # 创建图形并设置大小
    figure = plt.figure(figsize=(7, 10))
    ax = figure.add_subplot(1, 1, 1)
    ax.grid(True, linestyle='--', alpha=0.7)
    # 使用更具视觉吸引力的颜色映射
    cmap = get_cmap(len(agents))
    # 绘制障碍物

    for vertices in obs_info:
        poly = patches.Polygon(
            vertices,
            facecolor='palegreen',  # 设置填充颜色为绿色
            edgecolor='darkgreen',  # 设置边框颜色为深绿色
            linewidth=2,  # 设置边框宽度
            fill=True,
            alpha=1,
            zorder=2
        )
        ax.add_patch(poly)

    legend_elements = []
    for i in range(0, len(agents)):
        # 如果机器人不存在，则跳过
        # if swarm.robot_exist[i] is False:
        #     continue
        # 绘制机器人
        robot = patches.Circle(
            (agents[i][0], agents[i][1]),
            radius=2,
            facecolor=cmap(i),
            edgecolor='black',
            linewidth=0.8,
            ls='solid',
            alpha=0.9,
            zorder=2)
        ax.add_patch(robot)
        # 绘制速度箭头
        # ax.text(agents[i].position_.x_, agents[i].position_.y_, str(i),
        #         fontsize=12, color='black', ha='center', va='center')
        # ax.arrow(agents[i].position_.x_, agents[i].position_.y_,
        #          agents[i].velocity_.x_*5, agents[i].velocity_.y_*5, head_width=1.5, head_length=2, fc=cmap(i), ec=cmap(i), alpha=0.8)
        # 绘制目标点
        # goal_marker, = ax.plot([goals[i].x_], [goals[i].y_], '*', color=cmap(i), markersize=8, linewidth=3.0)
        # 绘制机器人路径
        # path = swarm.des_path_pos[i]
        # path = np.array(path)
        # path_line, = plt.plot(path[:, 0], path[:, 1], color=cmap(i), linewidth=1.2)

    # # 添加时间信息
    # if time:
    #     ax.text(0, boundary[1][1] + 1, f'$t={time:.1f} s$', fontsize=20, fontweight='bold')
    # # 设置坐标轴
    # out_size = 2
    ax.set_aspect('equal')
    boundary = [[-100, 100], [-100, 100]]
    out_size = 0
    ax.set_xlim(boundary[0][0] - out_size, boundary[0][1] + out_size - 1)
    ax.set_ylim(boundary[1][0] - out_size, boundary[1][1] + out_size + 3)

    # 保存图片
    if name:
        plt.savefig(name, dpi=300, bbox_inches='tight')
    # 清除当前图形
    plt.cla()
    plt.close(figure)
    return figure


def save_density_to_json(max_density_list, mean_density_list, non_zero_mean_density_list, filename):
    density_data = {
        "max_density": max_density_list,
        "mean_density": mean_density_list,
        "non_zero_mean_density": non_zero_mean_density_list
    }

    with open(filename, 'w') as f:
        json.dump(density_data, f, indent=4)


def save_wait_num_json(num_wait_list, filename):
    density_data = {
        "num_wait_list": num_wait_list
    }

    with open(filename, 'w') as f:
        json.dump(density_data, f, indent=4)

def save_T_computation_time(T_r, ave_time, filename):
    data = {
        "T_result": T_r,
        "ave_time": ave_time
    }
    filename = filename + "/r.json"
    with open(filename, 'w') as f:
        json.dump(data, f, indent=4)
