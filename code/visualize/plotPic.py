import matplotlib
import matplotlib.pyplot as pyplot
from matplotlib.path import Path
import matplotlib.patches as patches
from matplotlib.patches import Polygon
import matplotlib.cm as cmx
import matplotlib.colors as colors
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.transforms import Affine2D


def get_cmap(n, name='hsv'):
    '''Returns a function that maps each index in 0, 1, ..., n-1 to a distinct
    RGB color; the keyword argument name must be a standard mpl colormap name.'''
    return plt.cm.get_cmap(name, n)

def visualize_traj_dynamic(ws_model, swarm, boundary, mapInfo, time=None, name=None):
    # 使用Agg后端进行非交互式绘图
    matplotlib.use('Agg')
    # 启用抗锯齿功能
    matplotlib.rcParams['path.simplify'] = True
    matplotlib.rcParams['path.simplify_threshold'] = 1.0
    matplotlib.rcParams['figure.dpi'] = 300
    X = swarm.pos_all
    U = swarm.V_all
    goal = swarm.goal_pos
    # 创建图形并设置大小
    figure = plt.figure(figsize=(7, 10))
    ax = figure.add_subplot(1, 1, 1)
    ax.grid(True, linestyle='--', alpha=0.7)
    # 使用更具视觉吸引力的颜色映射
    cmap = get_cmap(len(X), 'tab20')
    # 绘制障碍物
    for obs_idx in mapInfo.obs_inside_idx:
        x = obs_idx[0]
        y = obs_idx[1]
        wid = obs_idx[2]
        srec = patches.Rectangle(
            (x - 1, y - 1),
            2 * wid, 2 * wid,
            facecolor='palegreen',  # 设置填充颜色为绿色
            edgecolor='darkgreen',  # 设置边框颜色为深绿色
            linewidth=2,  # 设置边框宽度
            fill=True,
            alpha=1,
            zorder=2
        )
        ax.add_patch(srec)
    # for hole in ws_model['circular_obstacles']:
    #     circle = patches.Circle(
    #         (hole[0], hole[1]),  # 圆心坐标
    #         hole[2],  # 半径
    #         facecolor='dimgrey',  # 面颜色
    #         edgecolor='black',  # 边缘颜色
    #         linewidth=1,  # 线宽
    #         fill=True,  # 填充
    #         alpha=0.8)  # 透明度
    #     ax.add_patch(circle)
    # 图例列表
    legend_elements = []
    for i in range(0, len(X)):
        # 如果机器人不存在，则跳过
        if swarm.robot_exist[i] is False:
            continue
        # 绘制机器人
        robot = patches.Circle(
            (X[i][0], X[i][1]),
            radius=ws_model['robot_radius'],
            facecolor=cmap(i),
            edgecolor='black',
            linewidth=0.8,
            ls='solid',
            alpha=0.9,
            zorder=2)
        ax.add_patch(robot)
        # 绘制速度箭头
        ax.arrow(X[i][0], X[i][1], U[i][0]*0.5, U[i][1]*0.5, head_width=0.15, head_length=0.1, fc=cmap(i), ec=cmap(i), alpha=0.8)
        # 绘制目标点
        goal_marker, = ax.plot([goal[i][0]], [goal[i][1]], '*', color=cmap(i), markersize=8, linewidth=3.0)
        # 绘制机器人路径
        path = swarm.des_path_pos[i]
        path = np.array(path)
        path_line, = plt.plot(path[:, 0], path[:, 1], color=cmap(i), linewidth=1.2)

    # 添加时间信息
    if time:
        ax.text(0, boundary[1][1] + 1, f'$t={time:.1f} s$', fontsize=20, fontweight='bold')
    # 设置坐标轴
    out_size = 2
    ax.set_aspect('equal')
    ax.set_xlim(boundary[0][0] - out_size, boundary[0][1] + out_size - 1)
    ax.set_ylim(boundary[1][0] - out_size, boundary[1][1] + out_size + 3)

    # 保存图片
    if name:
        plt.savefig(name, dpi=300, bbox_inches='tight')
    # 清除当前图形
    plt.cla()
    plt.close(figure)
    return figure

def visualize_traj_only_path(ws_model, swarm, boundary, mapInfo, time=None, name=None):
    # 使用Agg后端进行非交互式绘图
    matplotlib.use('Agg')
    # 启用抗锯齿功能
    matplotlib.rcParams['path.simplify'] = True
    matplotlib.rcParams['path.simplify_threshold'] = 1.0
    matplotlib.rcParams['figure.dpi'] = 300
    X = swarm.pos_all
    U = swarm.V_all
    goal = swarm.goal_pos
    # 创建图形并设置大小
    figure = plt.figure(figsize=(7, 10))
    ax = figure.add_subplot(1, 1, 1)
    ax.grid(True, linestyle='--', alpha=0.7)
    # 使用更具视觉吸引力的颜色映射
    cmap = get_cmap(len(X), 'tab20')
    # 绘制障碍物
    for obs_idx in mapInfo.obs_inside_idx:
        x = obs_idx[0]
        y = obs_idx[1]
        wid = obs_idx[2]
        srec = patches.Rectangle(
            (x - 1, y - 1),
            2 * wid, 2 * wid,
            facecolor='palegreen',  # 设置填充颜色为绿色
            edgecolor='darkgreen',  # 设置边框颜色为深绿色
            linewidth=2,  # 设置边框宽度
            fill=True,
            alpha=1,
            zorder=2
        )
        ax.add_patch(srec)
    # 图例列表
    legend_elements = []
    for i in range(0, len(X)):
        # 如果机器人不存在，则跳过
        if swarm.robot_exist[i] is False:
            continue
        # 绘制目标点
        goal_marker, = ax.plot([goal[i][0]], [goal[i][1]], '*', color=cmap(i), markersize=8, linewidth=3.0)
        # 绘制机器人路径
        path = swarm.des_path_pos[i]
        path = np.array(path)
        path_line, = plt.plot(path[:, 0], path[:, 1], color=cmap(i), linewidth=1.2)

    # 添加时间信息
    if time:
        ax.text(0, boundary[1][1] + 1, f'$t={time:.1f} s$', fontsize=20, fontweight='bold')
    # 设置坐标轴
    out_size = 2
    ax.set_aspect('equal')
    ax.set_xlim(boundary[0][0] - out_size, boundary[0][1] + out_size - 1)
    ax.set_ylim(boundary[1][0] - out_size, boundary[1][1] + out_size + 3)

    # 保存图片
    if name:
        plt.savefig(name, dpi=300, bbox_inches='tight')
    # 清除当前图形
    plt.cla()
    plt.close(figure)
    return figure

# def visualize_traj_dynamic(ws_model, swarm, boundary, mapInfo, time=None, name=None):
#     matplotlib.use('Agg')
#     X = swarm.pos_all
#     U = swarm.V_all
#     goal = swarm.goal_pos
#
#     figure = pyplot.figure(figsize=(7, 10))
#     ax = figure.add_subplot(1, 1, 1)
#     cmap = get_cmap(len(X))
#     # plot obstacles
#     for hole in ws_model['circular_obstacles']:
#         srec = matplotlib.patches.Rectangle(
#             (hole[0] - hole[2], hole[1] - hole[2]),
#             2 * hole[2], 2 * hole[2],
#             facecolor='dimgrey',
#             fill=True,
#             alpha=1)
#         # ax.add_patch(srec)
#         circle = patches.Circle(
#             (hole[0], hole[1]),  # 圆心坐标
#             hole[2],  # 半径
#             facecolor='dimgrey',  # 面颜色
#             fill=True,  # 填充
#             alpha=1)  # 透明度
#         ax.add_patch(circle)
#     for i in range(0, len(X)):
#         # -------plot car
#         if swarm.robot_exist[i] is False:
#             continue
#         robot = matplotlib.patches.Circle(
#             (X[i][0], X[i][1]),
#             radius=ws_model['robot_radius'],
#             facecolor=cmap(i),
#             edgecolor='black',
#             linewidth=0.5,
#             ls='solid',
#             alpha=1,
#             zorder=2)
#         ax.add_patch(robot)
#         # ----------plot velocity
#         ax.arrow(X[i][0], X[i][1], U[i][0]*0.5, U[i][1]*0.5, head_width=0.15, head_length=0.1, fc=cmap(i), ec=cmap(i))
#         # ax.text(X[i][0] - 0.1, X[i][1] - 0.1, r'$%s$' % i, fontsize=15, fontweight='bold', zorder=3)
#         ax.plot([goal[i][0]], [goal[i][1]], '*', color=cmap(i), markersize=5, linewidth=3.0)
#
#         # plot path of each robots
#         path = swarm.des_path_pos[i]
#         path = np.array(path)
#         plt.plot(path[:, 0], path[:, 1], color=cmap(i), linewidth=1)

    if time:
        ax.text(0, boundary[1][1] + 1, '$t=%.1f s$' % time, fontsize=20, fontweight='bold')
    # ---set axes ---

    # boundary [[x1,x2], [y1, y2]]
    out_size = 2
    ax.set_aspect('equal')
    ax.set_xlim(boundary[0][0] - out_size, boundary[0][1] + out_size -1)
    ax.set_ylim(boundary[1][0] - out_size, boundary[1][1] + out_size + 3)
    ax.set_xlabel(r'$x (m)$')
    ax.set_ylabel(r'$y (m)$')
    ax.grid(True)
    if name:
        pyplot.savefig(name, dpi=200)
        # pyplot.savefig(name,bbox_inches='tight')
    pyplot.cla()
    pyplot.close(figure)
    return figure


def visualize_traj_dynamic_withoutPath(ws_model, swarm, boundary, mapInfo, time=None, name=None):
    matplotlib.use('Agg')
    X = swarm.pos_all
    U = swarm.V_all
    goal = swarm.goal_pos
    resolution = 10

    figure = pyplot.figure(figsize=(7, 10))
    ax = figure.add_subplot(1, 1, 1)
    cmap = get_cmap(len(X))
    # plot obstacles
    for hole in ws_model['circular_obstacles']:
        srec = matplotlib.patches.Rectangle(
            (hole[0] - hole[2], hole[1] - hole[2]),
            2 * hole[2], 2 * hole[2],
            facecolor='dimgrey',
            fill=True,
            alpha=1)
        ax.add_patch(srec)
        circle = patches.Circle(
            (hole[0], hole[1]),  # 圆心坐标
            hole[2],  # 半径
            facecolor='dimgrey',  # 面颜色
            fill=True,  # 填充
            alpha=1)  # 透明度
        ax.add_patch(circle)
    for i in range(0, len(X)):
        # -------plot car
        if swarm.robot_exist[i] is False:
            continue
        robot = matplotlib.patches.Circle(
            (X[i][0], X[i][1]),
            radius=ws_model['robot_radius'],
            facecolor=cmap(i),
            edgecolor='black',
            linewidth=0.5,
            ls='solid',
            alpha=1,
            zorder=2)
        ax.add_patch(robot)
        # ----------plot velocity
        ax.arrow(X[i][0], X[i][1], U[i][0]*0.5, U[i][1]*0.5, head_width=0.15, head_length=0.1, fc=cmap(i), ec=cmap(i))
        # ax.text(X[i][0] - 0.1, X[i][1] - 0.1, r'$%s$' % i, fontsize=15, fontweight='bold', zorder=3)
        ax.plot([goal[i][0]], [goal[i][1]], '*', color=cmap(i), markersize=5, linewidth=3.0)

        # plot des pos of all drones
        pos = swarm.des_pos[i]
        # pos_n = matplotlib.patches.Rectangle(
        #     (pos[0] - 2*ws_model['robot_radius'], pos[1] - 2*ws_model['robot_radius']),
        #     width=4 * ws_model['robot_radius'],
        #     height=4 * ws_model['robot_radius'],
        #     facecolor='white',
        #     edgecolor='black',
        #     linewidth=0.5,
        #     ls='solid',
        #     alpha=1,
        #     zorder=2)
        # pos_n = matplotlib.patches.Circle(
        #                 (pos[0], pos[1]),
        #                 radius=ws_model['robot_radius'],
        #                 facecolor=cmap(i),
        #                 edgecolor='black',
        #                 linewidth=0.5,
        #                 ls='solid',
        #                 alpha=1,
        #                 zorder=2)
        # ax.add_patch(pos_n)

        # plot path of each robots
        path = swarm.des_path_pos[i]
        path = np.array(path)
        # plt.plot(path[:, 0], path[:, 1], color=cmap(i), linewidth=1)

    # plot pos option
    # for key, node_n in mapInfo.node_all.items():
    #     if 'node_option_pos' in node_n:
    #         pos_list = node_n['node_option_pos']
    #         for pos in pos_list:
    #             pos_option = matplotlib.patches.Circle(
    #                 (pos[0], pos[1]),
    #                 radius=ws_model['robot_radius'],
    #                 facecolor=cmap(0),
    #                 edgecolor='black',
    #                 linewidth=0.5,
    #                 ls='solid',
    #                 alpha=1,
    #                 zorder=2)
    #             ax.add_patch(pos_option)

    if time:
        ax.text(0, 94, '$t=%.1f s$' % time, fontsize=20, fontweight='bold')
    # ---set axes ---

    # boundary [[x1,x2], [y1, y2]]
    out_size = 0.5
    ax.set_aspect('equal')
    ax.set_xlim(boundary[0][0] - out_size, boundary[0][1] + out_size)
    ax.set_ylim(boundary[1][0] - out_size, boundary[1][1] + out_size)
    ax.set_xlabel(r'$x (m)$')
    ax.set_ylabel(r'$y (m)$')
    ax.grid(True)
    if name:
        pyplot.savefig(name, dpi=200)
        # pyplot.savefig(name,bbox_inches='tight')
    pyplot.cla()
    pyplot.close(figure)
    return figure


def visualize_traj_dynamic_1(ws_model, swarm, boundary, mapInfo, time=None, name=None):

    X = swarm.pos_all
    U = swarm.V_all
    goal = swarm.goal_pos
    resolution = 10

    figure = pyplot.figure(figsize=(6, 8))
    ax = figure.add_subplot(1, 1, 1)
    cmap = get_cmap(len(X))
    # plot obstacles
    for hole in ws_model['circular_obstacles']:
        srec = matplotlib.patches.Rectangle(
            (hole[0] - hole[2], hole[1] - hole[2]),
            2 * hole[2], 2 * hole[2],
            facecolor='dimgrey',
            fill=True,
            alpha=1)
        ax.add_patch(srec)
        circle = patches.Circle(
            (hole[0], hole[1]),  # 圆心坐标
            hole[2],  # 半径
            facecolor='dimgrey',  # 面颜色
            fill=True,  # 填充
            alpha=1)  # 透明度
        ax.add_patch(circle)
    # for i in range(0, len(X)):
    #     # -------plot car
    #     if swarm.robot_exist[i] is False:
    #         continue
    #     robot = matplotlib.patches.Circle(
    #         (X[i][0], X[i][1]),
    #         radius=ws_model['robot_radius'],
    #         facecolor=cmap(i),
    #         edgecolor='black',
    #         linewidth=0.5,
    #         ls='solid',
    #         alpha=1,
    #         zorder=2)
    #     ax.add_patch(robot)
    #     # ----------plot velocity
    #     ax.arrow(X[i][0], X[i][1], U[i][0]*0.5, U[i][1]*0.5, head_width=0.15, head_length=0.1, fc=cmap(i), ec=cmap(i))
    #     # ax.text(X[i][0] - 0.1, X[i][1] - 0.1, r'$%s$' % i, fontsize=15, fontweight='bold', zorder=3)
    #     ax.plot([goal[i][0]], [goal[i][1]], '*', color=cmap(i), markersize=5, linewidth=3.0)

    # plot des pos of all drones
    #     pos = swarm.des_pos[i]
    #     pos_n = matplotlib.patches.Circle(
    #                     (pos[0], pos[1]),
    #                     radius=ws_model['robot_radius'],
    #                     facecolor=cmap(i),
    #                     edgecolor='black',
    #                     linewidth=0.5,
    #                     ls='solid',
    #                     alpha=1,
    #                     zorder=2)
    #     ax.add_patch(pos_n)

    # plot pos option
    for key, node_n in mapInfo.node_all.items():
        if 'node_option_pos' in node_n:
            pos_list = node_n['node_option_pos']
            for pos in pos_list:
                pos_option = matplotlib.patches.Circle(
                    (pos[0], pos[1]),
                    radius=ws_model['robot_radius'],
                    facecolor=cmap(0),
                    edgecolor='black',
                    linewidth=0.5,
                    ls='solid',
                    alpha=1,
                    zorder=2)
                ax.add_patch(pos_option)

    if time:
        ax.text(2, 66, '$t=%.1f s$' % time, fontsize=20, fontweight='bold')
    # ---set axes ---

    # boundary [[x1,x2], [y1, y2]]
    out_size = 0.5
    ax.set_aspect('equal')
    ax.set_xlim(boundary[0][0] - out_size, boundary[0][1] + out_size)
    ax.set_ylim(boundary[1][0] - out_size, boundary[1][1] + out_size)
    ax.set_xlabel(r'$x (m)$')
    ax.set_ylabel(r'$y (m)$')
    ax.grid(True)
    if name:
        pyplot.savefig(name, dpi=200)
        # pyplot.savefig(name,bbox_inches='tight')
    pyplot.cla()
    pyplot.close(figure)
    return figure

# def get_cmap(N):
#     '''Returns a function that maps each index in 0, 1, ... N-1 to a distinct RGB color.'''
#     color_norm = colors.Normalize(vmin=0, vmax=N - 1)
#     scalar_map = cmx.ScalarMappable(norm=color_norm, cmap='hsv')
#
#     def map_index_to_rgb_color(index):
#         return scalar_map.to_rgba(index)
#
#     return map_index_to_rgb_color