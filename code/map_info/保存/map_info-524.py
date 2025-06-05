import copy
import time
import math
from matplotlib.patches import Circle
import cv2
import numpy as np
import matplotlib.pyplot as plt
import networkx as nx
from map_info.FF import FlowAlgo
from map_info.bcd import *
from scipy.ndimage import binary_dilation
from pathfinding.dijkstra import Dijkstra
from pathfinding.Astar import Astar
from pathfinding.jps import JPS


class MapInfo:

    def __init__(self, swarm, pic_name):
        self.pic_name = pic_name
        self.swarm = swarm

        self.x_size = None
        self.y_size = None
        self.obstacles = None

        self.num_obs_rect = None
        self.obstacles_rect = None

        self.resolution = 10

        self.number_cells = None

        self.FF_algo = FlowAlgo()

        self.map_all = None
        self.map_01 = None

        self.start_idx = None
        self.end_idx = None

        # cell的num到index
        self.cell_dict = {}

        # 面积area  左边两个顶点 右边两个顶点
        self.cell_info = {}
        self.delete_cell_list = None

        self.dijkstra_result = {}

        self.node_all = {}
        self.edge_all = {}
        # 节点到goal的JPS路径以及经过的node
        self.node_to_goal = {}
        self.jps_path_all = {}
        self.start_node = None
        self.end_node = None

        self.distance = 1
        # 几个格子整合 作为一个位置区域
        self.num_grids = 3
        # 无人机的安全避碰距离
        self.safety_dis = self.swarm.robot_radius*1.2
        # 拓扑关系 节点之间的连接关系 使用邻接矩阵来表示
        self.dijkstra = Dijkstra()
        self.jps = JPS()
        self.Astar = Astar()
        # self.init_main()

    def init_main(self):
        print("Map decomposition and network construction !")
        self.read_map()
        self.extract_info()
        self.construct_cell_structure()
        self.deal_with_resolution()
        self.construct_flow_network_one()
        self.construct_dijkstra()
        self.update_para()

    def init_main_one(self):
        print("Map decomposition and network construction !")
        self.read_map()
        self.extract_info()
        self.construct_cell_structure()
        self.delete_small_cell()
        self.deal_with_resolution()
        self.construct_flow_network_two()
        self.add_edge_for_delete_cell()
        self.construct_dijkstra()
        self.update_para()

    def update_para(self):
        self.swarm.current_node = np.ones((self.swarm.robots_num, 2), dtype=int) * self.start_node

    def deal_with_resolution(self):
        for k, value in self.cell_info.items():
            value['left_up'] = [value['left_up'][0] / self.resolution, value['left_up'][1] / self.resolution]
            value['left_down'] = [value['left_down'][0] / self.resolution, value['left_down'][1] / self.resolution]
            value['right_up'] = [value['right_up'][0] / self.resolution, value['right_up'][1] / self.resolution]
            value['right_down'] = [value['right_down'][0] / self.resolution, value['right_down'][1] / self.resolution]

    def discrete_points_between(self, p1, p2):
        x1, y1 = p1
        x2, y2 = p2
        points = []

        num = max(abs(y2 - y1), abs(x2 - x1))
        dx = (x2 - x1) / num
        dy = (y2 - y1) / num

        for i in range(num + 1):
            x = x1 + i * dx
            y = y1 + i * dy
            points.append((int(x), int(y)))
        return points

    def generate_discrete_points(self, path):
        discrete_points = []
        for i in range(len(path) - 1):
            discrete_points.extend(self.discrete_points_between(path[i], path[i + 1]))
        return discrete_points

    def calculate_distance(self, point1, point2):
        x1, y1 = point1
        x2, y2 = point2
        return math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)

    def calculate_path_length(self, path):
        total_length = 0
        for i in range(len(path) - 1):
            total_length += self.calculate_distance(path[i], path[i + 1])
        return total_length

    def find_node_to_path_dict_test(self):
        # 找到所有node 到终点node的path dis 和 node list 使用JPS搜索
        key = 42
        node = self.node_all[key]
        dict = {}
        node_pos = node['pos']
        start_idx = (int(node_pos[0] * self.resolution), int(node_pos[1] * self.resolution))
        # goal_node = self.swarm.goal_node[0]
        goal_node = self.end_node
        goal_pos = self.node_start_end[goal_node]['pos']
        goal_idx = (int(goal_pos[0] * self.resolution), int(goal_pos[1] * self.resolution))
        path = self.JPS_search(start_idx, goal_idx)
        cell_idx_set = self.find_pass_cells(path)
        node_list = self.find_pass_node(cell_idx_set, key)
        node_list.append(goal_node)
        dis = self.calculate_path_length(path) / self.resolution

        dict['path_node'] = node_list
        dict['distance'] = dis
        self.node_to_goal[key] = dict


    def find_node_to_path_dict(self):
        # 找到所有node 到终点node的path dis 和 node list 使用JPS搜索
        self.node_to_goal = {}
        for key, node in self.node_all.items():
            dict = {}
            node_pos = node['pos']
            start_idx = (int(node_pos[0] * self.resolution), int(node_pos[1] * self.resolution))
            # goal_node = self.swarm.goal_node[0]
            goal_node = self.end_node
            goal_pos = self.node_start_end[goal_node]['pos']
            goal_idx = (int(goal_pos[0] * self.resolution), int(goal_pos[1] * self.resolution))
            path = self.JPS_search(start_idx, goal_idx)
            self.jps_path_all[key] = path
            cell_idx_set = self.find_pass_cells(path)
            node_list = self.find_pass_node(cell_idx_set, key)
            node_list.append(goal_node)
            dis = self.calculate_path_length(path) / self.resolution

            dict['path_node'] = node_list
            dict['distance'] = dis
            self.node_to_goal[key] = dict


    def find_pass_node(self, cell_idx_set, node_idx):
        node_list = []
        for i in range(len(cell_idx_set) - 1):
            cell_down = cell_idx_set[i]
            cell_up = cell_idx_set[i + 1]
            for key, node in self.node_all.items():
                if 'cell_up_set' in node:
                    cell_up_set = node['cell_up_set']
                    cell_down_set = node['cell_down_set']
                    if cell_down in cell_down_set and cell_up in cell_up_set:
                        if node_idx != key:
                            node_list.append(key)
                        break
        return node_list

    def find_pass_cells(self, path):
        # 找到这个无人机经过的所有cell的idx
        discrete_path = self.generate_discrete_points(path)
        cell_idx_list = []
        cell_idx_set = set()
        for p in discrete_path:
            idx = self.map_all[p[0], p[1]] - 1
            if idx <= 0:
                continue
            if len(cell_idx_list) == 0:
                cell_idx_list.append(idx)
                cell_idx_set.add(idx)
            elif len(cell_idx_list) != 0 and idx not in cell_idx_set:
                cell_idx_list.append(idx)
                cell_idx_set.add(idx)
        return cell_idx_list

    def JPS_search(self, start, goal):
        t1 = time.time()
        path = self.jps.method(self.map_01, start, goal, 1)
        t2 = time.time()
        print("jps=" + str(t2 - t1) + "s")
        # self.plot_matrix_map(path)
        return path

    def Astar_search(self, start, goal):
        t1 = time.time()
        path = self.Astar.method(self.dilated_map, start, goal, 1)
        t2 = time.time()
        print("Astar=" + str(t2 - t1) + "s")
        return path
        # self.plot_matrix_map(path)

    def Astar_search_one(self, start, goal):
        t1 = time.time()
        path = self.Astar.method(self.map_01, start, goal, 1)
        t2 = time.time()
        # print("Astar=" + str(t2 - t1) + "s")
        return path

    def shared_edge_to_path(self):
        self.shared_first_edge = {}
        self.shared_second_edge = {}
        first_edge_set = set()
        second_edge_set = set()

        for idx, value in self.path_all.items():
            first_edge = value['first_edge']
            second_edge = value['second_edge']
            first_edge_set.add(first_edge)
            second_edge_set.add(second_edge)

        for edge in first_edge_set:
            path_idx_set = set()
            for path_idx, value in self.path_all.items():
                if edge == value['first_edge']:
                    path_idx_set.add(path_idx)
            self.shared_first_edge[edge] = path_idx_set

        for edge in second_edge_set:
            path_idx_set = set()
            for path_idx, value in self.path_all.items():
                if edge == value['second_edge']:
                    path_idx_set.add(path_idx)
            self.shared_second_edge[edge] = path_idx_set

    def shared_node_to_path(self):
        self.shared_node_all = {}
        adj_node_set = set()
        for idx, value in self.path_all.items():
            # if len(value['path_node']) <= 2:
            #     print()
            #     continue
            adj_node = value['adj_node']
            adj_node_set.add(adj_node)

        for adj_node in adj_node_set:
            path_idx_set = set()
            for path_idx, value in self.path_all.items():
                if adj_node == value['adj_node']:
                    path_idx_set.add(path_idx)
            idx = len(self.shared_node_all)
            self.shared_node_all[idx] = {'adj_node': adj_node, 'path_idx_set': path_idx_set}

    def find_current_path_set_edge(self):
        # 找到以当前无人机所在节点为基础 出去的所有点 到目标点的path的集合
        # 一个新的版本 找到path的一边 二边
        path_all = {}
        for k in range(self.swarm.robots_num):
            s_k = self.swarm.current_node[k]
            g_k = self.swarm.goal_node[k]
            # 当前节点出去的节点集合为
            s_k_out_node = self.adj_node_list(s_k)

            if s_k == self.swarm.goal_node[k]:
                edge_s = str(g_k) + "," + str(g_k)
                dic = {'adj_node': s_k, 'path_node': [s_k], 'path_length': 0,
                       'start_node': s_k, 'end_node': g_k, 'drone_idx': k,
                       'first_edge': edge_s, 'second_edge': edge_s}
                idx = len(path_all)
                path_all[idx] = dic

            for adj_node in s_k_out_node:
                path_node, path_length = self.find_path_mid(s_k, g_k, adj_node)
                # path_node, path_length = self.find_path_mid_JPS(k, s_k, g_k, adj_node)
                path_length = self.revise_path_length(k, s_k, g_k, path_node, path_length)

                if math.isnan(path_length) or math.isinf(path_length):
                    path_length = 999999999

                if len(path_node) >= 3:
                    p1 = path_node[0]
                    p2 = path_node[1]
                    p3 = path_node[2]
                    first_edge = str(p1) + ',' + str(p2)
                    second_edge = str(p2) + ',' + str(p3)
                    dic = {'adj_node': adj_node, 'path_node': path_node, 'path_length': path_length,
                           'start_node': s_k, 'end_node': g_k, 'drone_idx': k,
                           'first_edge': first_edge, 'second_edge': second_edge}
                else:
                    p1 = path_node[0]
                    p2 = path_node[1]
                    first_edge = str(p1) + ',' + str(p2)
                    second_edge = str(p1) + ',' + str(p2)
                    dic = {'adj_node': adj_node, 'path_node': path_node, 'path_length': path_length,
                           'start_node': s_k, 'end_node': g_k, 'drone_idx': k,
                           'first_edge': first_edge, 'second_edge': second_edge}
                idx = len(path_all)
                path_all[idx] = dic
        # print()
        self.path_all = path_all
        # print("path_all_len=" + str(len(self.path_all)))
        return path_all
        # 这些path的下一个点 有哪些path的下一个点是相同的 找到这些相同点的path

    def find_current_path_set_greedy(self):
        # 找到以起点节点为邻居节点出发 到目标点的path的集合
        path_all = {}

        k = 0

        s_k = self.swarm.current_node[k]
        g_k = self.swarm.goal_node[k]
        # 当前节点出去的节点集合为
        s_k_out_node = self.adj_node_list(s_k)

        if s_k == self.swarm.goal_node[k]:
            dic = {'adj_node': s_k, 'path_node': [s_k], 'path_length': 0,
                   'start_node': s_k, 'end_node': g_k, 'drone_idx': k}
            idx = len(path_all)
            path_all[idx] = dic

        for adj_node in s_k_out_node:
            path_node, path_length = self.find_path_mid(s_k, g_k, adj_node)
            # path_length = self.revise_path_length(k, s_k, g_k, path_node, path_length)
            dic = {'adj_node': adj_node, 'path_node': path_node, 'path_length': path_length,
                   'start_node': s_k, 'end_node': g_k, 'drone_idx': k}
            idx = len(path_all)
            path_all[idx] = dic
        # print()
        # self.path_all = path_all
        return path_all

    def find_current_path_set(self):
        # 找到以当前无人机所在节点为基础 出去的所有点 到目标点的path的集合
        path_all = {}
        for k in range(self.swarm.robots_num):
            s_k = self.swarm.current_node[k]
            g_k = self.swarm.goal_node[k]
            # 当前节点出去的节点集合为
            s_k_out_node = self.adj_node_list(s_k)

            if s_k == self.swarm.goal_node[k]:
                dic = {'adj_node': s_k, 'path_node': [s_k], 'path_length': 0,
                       'start_node': s_k, 'end_node': g_k, 'drone_idx': k}
                idx = len(path_all)
                path_all[idx] = dic

            for adj_node in s_k_out_node:
                path_node, path_length = self.find_path_mid(s_k, g_k, adj_node)
                # path_node, path_length = self.find_path_mid_JPS(k, s_k, g_k, adj_node)
                path_length = self.revise_path_length(k, s_k, g_k, path_node, path_length)
                dic = {'adj_node': adj_node, 'path_node': path_node, 'path_length': path_length,
                       'start_node': s_k, 'end_node': g_k, 'drone_idx': k}
                idx = len(path_all)
                path_all[idx] = dic
        # print()
        self.path_all = path_all
        return path_all
        # 这些path的下一个点 有哪些path的下一个点是相同的 找到这些相同点的path

    def node_dis_dij(self, node1, node2):
        node_set = self.dijkstra.adjacency_list[node1]
        for n in node_set:
            if n[0] == node2:
                dis = n[1]
                return dis

    def revise_path_length(self, k, s_k, g_k, path_node, path_length):
        # 修正对于每个飞机的路径长度
        if len(path_node) >= 3:
            second_node = path_node[1]
            second_to_last_node = path_node[-2]
            dis1 = self.node_dis_dij(s_k, second_node)
            dis2 = self.node_dis_dij(second_to_last_node, g_k)

            current_pos = self.swarm.pos_all[k]
            goal_pos = self.swarm.goal_pos[k]

            # current_pos = [x * self.resolution for x in current_pos]
            # goal_pos = [x * self.resolution for x in goal_pos]

            second_pos = self.node_all[second_node]['pos']
            second_last_pos = self.node_all[second_to_last_node]['pos']

            dis1_n = self.swarm.distance(current_pos, second_pos)
            dis2_n = self.swarm.distance(goal_pos, second_last_pos)

            path_length = path_length - dis1 - dis2 + dis1_n + dis2_n
            return path_length
        else:
            return path_length

    def adj_node_list(self, node):
        # 返回和当前节点相邻的所有节点
        s = self.dijkstra.adjacency_list[node]
        node_list = []
        for n in s:
            node_list.append(n[0])
        return node_list

    # def find_path_k(self, number_of_k, s_k_list, g_k_list):
    #     # 输入: 所有的无人机 无人机所在的node 无人机的目标node
    #     # 一共有k无人机
    #     # path_info idx: 1-path node 2-path cost 3-cut edge idx 4-k idx
    #     path_all = {}
    #     for k in range(number_of_k):
    #         # k无人机 起始位置 和 目标位置
    #         s_k = s_k_list[k]
    #         g_k = g_k_list[k]
    #         for e_idx, e_info in self.cut_edges_all.items():
    #             sink_node = e_info['sink']
    #             # path_node, path_length = self.find_path_mid(s_k, g_k, sink_node)
    #             path_node, path_length = self.find_path_mid_JPS(s_k, g_k, sink_node)
    #             dic = {'cut_edge_idx': e_idx, 'path_node': path_node, 'path_length': path_length,
    #                    'start_node': s_k, 'end_node': g_k, 'drone_idx': k}
    #             idx = len(path_all)
    #             path_all[idx] = dic
    #     return path_all

    # def init_cut_edges_all(self):
    #     # 初始化所有的cut边的信息
    #     self.cut_edges_all = {}
    #     cut_num = len(self.min_cut_edges)
    #     for i in range(cut_num):
    #         idx = i
    #         sink = self.min_cut_edges[i].sink
    #         source = self.min_cut_edges[i].source
    #         capacity = self.min_cut_edges[i].capacity
    #         di = {'source': source, 'sink': sink, 'capacity': capacity}
    #         self.cut_edges_all[idx] = di

    def construct_dijkstra(self):
        graph_edges = []
        for edge, info in self.edge_all.items():
            node_start, node_end = map(int, edge.split(','))
            dis = round(info['dis'], 2)
            e = [node_start, node_end, dis]
            graph_edges.append(e)
        self.dijkstra.init_node_edges(graph_edges)
        # self.dijkstra.shortest_path(24, 11)
        # self.dijkstra.print_result()
        # print()

    def find_path_mid_JPS(self, k, start_node, end_node, mid_node):
        # 根据当前出的几个边 直接到goal的path 组成一个集合
        # 开始节点 到中间节点的path 中间节点到目标点的path 两个path组合起来
        # path1, dis1 = self.dijkstra.shortest_path(start_node, mid_node)
        path1 = [start_node, mid_node]
        pos_n = self.swarm.pos_all[k]
        if mid_node != self.end_node:
            pos_m = self.node_all[mid_node]['pos']
        else:
            pos_m = self.swarm.goal_pos[k]
        dis1 = self.list_distance(pos_n, pos_m)

        if mid_node in self.node_to_goal:
            path2 = self.node_to_goal[mid_node]['path_node']
            dis2 = self.node_to_goal[mid_node]['distance']
        else:
            path2 = []
            dis2 = 0
        path1 = list(path1)
        path2 = list(path2)
        path = path1 + path2
        dis = dis1 + dis2
        return path, dis

    def find_path_mid(self, start_node, end_node, mid_node):
        # 根据当前出的几个边 直接到goal的path 组成一个集合
        # 开始节点 到中间节点的path 中间节点到目标点的path 两个path组合起来
        name1 = (start_node, mid_node)
        if name1 in self.dijkstra_result:
            path1 = self.dijkstra_result[name1]['path']
            dis1 = self.dijkstra_result[name1]['dis']
        else:
            path1, dis1 = self.dijkstra.shortest_path(start_node, mid_node)
            dic_n = {'path': path1, 'dis': dis1}
            self.dijkstra_result[name1] = dic_n

        name2 = (mid_node, end_node)
        if name2 in self.dijkstra_result:
            path2 = self.dijkstra_result[name2]['path']
            dis2 = self.dijkstra_result[name2]['dis']
        else:
            path2, dis2 = self.dijkstra.shortest_path(mid_node, end_node)
            dic_n = {'path': path2, 'dis': dis2}
            self.dijkstra_result[name2] = dic_n

        path1 = list(path1)
        path2 = list(path2)
        path1.pop()

        path = path1 + path2
        dis = dis1 + dis2

        return path, dis

    def find_path_node(self, start_node, end_node, min_cut_node, start_out_node):
        # 找到一个path集合用作后续的线性规划的基础
        # 1-起点 终点设置
        # 2-找到 最小割的那些点
        # 3-找到起点出去的几个点 到 最小割的点的 path
        path1, dis1 = self.find_path_mid(start_out_node, end_node, min_cut_node)
        # 4-找到最小割的点 到终点的path
        path2, dis2 = self.dijkstra.shortest_path(start_node, start_out_node)
        # 5-将这些path组合起来得到一个新的path
        path1 = list(path1)
        path1.insert(0, start_node)
        dis = dis1 + dis2
        return path1, dis

    def find_path_set(self):
        # 找到当前对应的所有path set
        path_set = []
        start_n = self.start_idx
        end_n = self.end_idx
        for edge in self.min_cut_edges:
            source = edge.source
            sink = edge.sink
            capa = edge.capacity
            # path_set.append(path)
            path, dis = self.find_path_mid(start_n, end_n, sink)
            path_set.append(path)

    def run_FordFulkerson_algo(self):
        for node in self.node_all:
            self.FF_algo.add_vertex(node)
        for edge, dic in self.edge_all.items():
            node_start, node_end = map(int, edge.split(','))
            capacity = dic['capacity']
            self.FF_algo.add_edge(node_start, node_end, capacity)
        maxflow_r = self.FF_algo.max_flow(self.start_idx, self.end_idx)

        self.min_cut_edges = self.FF_algo.find_min_cut_node_set(self.start_idx)
        print(maxflow_r)
        # self.FF_algo.add_edge()

    def draw_mapInfo(self):
        self.display_separate_cell(self.bcd_out_im, self.bcd_out_cells)
        self.draw_network()

    def judge_whether_pass_obs(self, path_list):
        num = 0
        for p in path_list:
            x, y = p
            if self.map_01[x, y] == 1:
                num += 1
        if num >= 10:
            print("True num=" + str(num))
            return True
        else:
            print("False=" + str(num))
            return False

    def draw_network(self):
        G = nx.Graph()
        for node, data in self.node_all.items():
            G.add_node(node, pos=data['pos'])
        for node, data in self.node_start_end.items():
            G.add_node(node, pos=data['pos'])

        for edge, capacity in self.edge_all.items():
            node_start, node_end = map(int, edge.split(','))
            G.add_edge(node_start, node_end, capacity=capacity)

        pos = nx.get_node_attributes(G, 'pos')
        edge_labels = {(u, v): d['capacity'] for u, v, d in G.edges(data=True)}

        plt.figure(figsize=(8, 6))
        # nx.draw(G, pos, with_labels=True, node_color='skyblue', node_size=100, edge_color='k')
        nx.draw(G, pos, with_labels=True, node_color='skyblue', node_size=30, edge_color='k', width=0.2,
                style='dashed')

        # nx.draw_networkx_edge_labels(G, pos, edge_labels=edge_labels, label_pos=0.4)  # 调整这里的label_pos值

        # Add circles for each node
        # print()
        # self.node_all
        ax = plt.gca()  # Get current axes
        for i in range(len(self.node_all)):
            node_pos_set = self.node_all[i]['node_option_pos']
            for j in range(len(node_pos_set)):
                pos = node_pos_set[j]
                radius = self.swarm.robot_radius
                circle = Circle((pos[0], pos[1]), radius, fill=False, edgecolor='r', linewidth=2)
                ax.add_patch(circle)
        plt.axis('equal')  # Set equal scaling by changing axis limits
        plt.show()

    def delete_small_cell(self):
        # delete cells that the gap between y_up and y_down is to small

        # 删除这些y轴上gap太小的cell 然后连接它们
        # 重新计算对应的边和node
        self.delete_cell_list = set()
        for i in range(len(self.cell_info)):
            cell_n = self.cell_info[i]
            left_up = cell_n['left_up']
            left_down = cell_n['left_down']

            if (left_up[1] - left_down[1]) <= 5:
                self.delete_cell_list.add(i)

    def add_edge_for_delete_cell(self):
        # when some sell is deleted, there need to add some edges
        for cell_n in self.delete_cell_list:
            up_cell_set = self.cell_info[cell_n]['up_node']
            for up_cell in up_cell_set:
                node_list1 = self.find_node_with_up_cell(cell_n)
                node_list2 = self.find_node_with_down_cell(up_cell)
                self.add_edges_by_list(node_list1, node_list2)

    def add_edges_by_list(self, node_list1, node_list2):
        for i in node_list1:
            for j in node_list2:
                ca_min = max(self.node_all[i]['capacity'], self.node_all[j]['capacity'])
                dis = self.list_distance(self.node_all[i]['pos'], self.node_all[j]['pos'])
                edge = str(i) + ',' + str(j)
                # 储存的是 list [capacity, dis]
                edge_capacity = ca_min * (dis / (2*self.safety_dis))

                d_n = {'capacity': edge_capacity, 'dis': dis}
                self.edge_all[edge] = d_n

    def find_node_with_up_cell(self, cell_n):
        node_list = []
        for node, node_info in self.node_all.items():
            if 'cell_up_set' in node_info:
                cell_set = node_info['cell_up_set']
                if cell_n in cell_set:
                    node_list.append(node)
        return node_list

    def find_node_with_down_cell(self, cell_n):
        node_list = []
        for node, node_info in self.node_all.items():
            if 'cell_down_set' in node_info:
                cell_set = node_info['cell_down_set']
                if cell_n in cell_set:
                    node_list.append(node)
        return node_list

    def compute_edge_capacity(self, i, j):
        pos_i = self.node_all[i]['pos']
        pos_j = self.node_all[j]['pos']
        ca_min = min(self.node_all[i]['capacity'], self.node_all[j]['capacity'])
        dis = self.list_distance(self.node_all[i]['pos'], self.node_all[j]['pos'])

        x1, y1 = pos_i
        x2, y2 = pos_j
        vector_x = x2 - x1
        vector_y = y2 - y1
        angle = math.atan2(vector_y, vector_x)
        sin_value = math.sin(angle)

        capacity_edge = int(sin_value*dis/(2*self.safety_dis)*ca_min)
        return capacity_edge

    def construct_flow_network_two(self):
        # 构建节点和边组成的network
        # 找到相邻的边 添加nodes
        for i in range(len(self.cell_info)):

            if i in self.delete_cell_list:
                continue

            cell_n = self.cell_info[i]
            down_node = cell_n['down_node']
            up_node = cell_n['up_node']

            up_edge = [cell_n['left_up'], cell_n['right_up']]
            if bool(up_node):
                for cell in up_node:
                    # 分别添加临边的节点进去
                    up_cell = self.cell_info[cell]
                    up_cell_down_edge = [up_cell['left_down'], up_cell['right_down']]
                    # 向上添加新的node
                    self.add_new_node_one(cell_n, up_edge, up_cell, up_cell_down_edge)

        # 找到节点之间的连接关系 建立边 并为边建立容量
        # print()
        for i in range(len(self.node_all)):
            cell_set_i = self.node_all[i]['cell_set']
            for cell_i in cell_set_i:
                for j in range(len(self.node_all)):
                    if i >= j:
                        continue
                    cell_set_j = self.node_all[j]['cell_set']
                    for cell_j in cell_set_j:
                        if cell_i == cell_j:
                            if self.node_all[i]['pos'][1] == self.node_all[j]['pos'][1]:
                                continue
                            ca_min = max(self.node_all[i]['capacity'], self.node_all[j]['capacity'])
                            dis = self.list_distance(self.node_all[i]['pos'], self.node_all[j]['pos'])
                            edge = str(i) + ',' + str(j)
                            capacity_n = self.compute_edge_capacity(i, j)
                            if capacity_n == 0:
                                capacity_n = 0.1
                            d_n = {'capacity': capacity_n, 'dis': dis}
                            self.edge_all[edge] = d_n

        # 找到哪些节点和 起始的cell 终点的cell有联系 并添加起点和终点的 点与边
        s_cell_idx = 0
        g_cell_idx = self.number_cells - 1

        node_sum = len(self.node_all)
        self.start_idx = node_sum
        self.end_idx = node_sum + 1
        self.start_node = node_sum
        self.end_node = node_sum + 1
        self.node_start_end = {}

        # s_pos = self.cell_info[s_cell_idx]['pos']
        s_pos = self.swarm.start_ave_pos
        dic_s = {'pos': s_pos, 'capacity': 1}
        self.node_start_end[node_sum] = dic_s
        # self.node_all[node_sum] = dic_s

        # g_pos = self.cell_info[g_cell_idx]['pos']
        g_pos = self.swarm.goal_ave_pos
        dic_g = {'pos': g_pos, 'capacity': 1}
        self.node_start_end[node_sum + 1] = dic_g
        # self.node_all[node_sum + 1] = dic_g

        for i in range(len(self.node_all)):
            node_cell_set = self.node_all[i]['cell_set']
            for cell in node_cell_set:
                ca = self.node_all[i]['capacity']
                if cell == s_cell_idx:
                    edge = str(node_sum) + ',' + str(i)
                    dis = self.list_distance(self.node_all[i]['pos'], self.node_start_end[node_sum]['pos'])
                    d_n = {'capacity': ca, 'dis': dis}
                    self.edge_all[edge] = d_n
                if cell == g_cell_idx:
                    edge = str(i) + ',' + str(node_sum + 1)
                    dis = self.list_distance(self.node_all[i]['pos'], self.node_start_end[node_sum + 1]['pos'])
                    d_n = {'capacity': ca, 'dis': dis}
                    self.edge_all[edge] = d_n
        # print()

    def construct_flow_network_one(self):
        # 构建节点和边组成的network
        # 找到相邻的边 添加nodes
        for i in range(len(self.cell_info)):

            # if i in self.delete_cell_list:
            #     continue

            cell_n = self.cell_info[i]
            down_node = cell_n['down_node']
            up_node = cell_n['up_node']

            up_edge = [cell_n['left_up'], cell_n['right_up']]
            if bool(up_node):
                for cell in up_node:
                    # 分别添加临边的节点进去
                    up_cell = self.cell_info[cell]
                    up_cell_down_edge = [up_cell['left_down'], up_cell['right_down']]
                    # 向上添加新的node
                    self.add_new_node_one(cell_n, up_edge, up_cell, up_cell_down_edge)

        # 找到节点之间的连接关系 建立边 并为边建立容量
        # print()
        for i in range(len(self.node_all)):
            cell_set_i = self.node_all[i]['cell_set']
            for cell_i in cell_set_i:
                for j in range(len(self.node_all)):
                    if i >= j:
                        continue
                    cell_set_j = self.node_all[j]['cell_set']
                    for cell_j in cell_set_j:
                        if cell_i == cell_j:
                            if self.node_all[i]['pos'][1] == self.node_all[j]['pos'][1]:
                                continue
                            ca_min = max(self.node_all[i]['capacity'], self.node_all[j]['capacity'])
                            dis = self.list_distance(self.node_all[i]['pos'], self.node_all[j]['pos'])
                            edge = str(i) + ',' + str(j)
                            # 储存的是 list [capacity, dis]
                            capacity_n = self.compute_edge_capacity(i, j)
                            if capacity_n == 0:
                                capacity_n = 0.1
                            d_n = {'capacity': capacity_n, 'dis': dis}
                            self.edge_all[edge] = d_n

        # 找到哪些节点和 起始的cell 终点的cell有联系 并添加起点和终点的 点与边
        s_cell_idx = 0
        g_cell_idx = self.number_cells - 1

        node_sum = len(self.node_all)
        self.start_idx = node_sum
        self.end_idx = node_sum + 1
        self.start_node = node_sum
        self.end_node = node_sum + 1
        self.node_start_end = {}

        # s_pos = self.cell_info[s_cell_idx]['pos']
        s_pos = self.swarm.start_ave_pos
        dic_s = {'pos': s_pos, 'capacity': 1}
        self.node_start_end[node_sum] = dic_s
        # self.node_all[node_sum] = dic_s

        # g_pos = self.cell_info[g_cell_idx]['pos']
        g_pos = self.swarm.goal_ave_pos
        dic_g = {'pos': g_pos, 'capacity': 1}
        self.node_start_end[node_sum + 1] = dic_g
        # self.node_all[node_sum + 1] = dic_g

        for i in range(len(self.node_all)):
            node_cell_set = self.node_all[i]['cell_set']
            for cell in node_cell_set:
                ca = self.node_all[i]['capacity']
                if cell == s_cell_idx:
                    edge = str(node_sum) + ',' + str(i)
                    dis = self.list_distance(self.node_all[i]['pos'], self.node_start_end[node_sum]['pos'])
                    d_n = {'capacity': ca, 'dis': dis}
                    self.edge_all[edge] = d_n
                if cell == g_cell_idx:
                    edge = str(i) + ',' + str(node_sum + 1)
                    dis = self.list_distance(self.node_all[i]['pos'], self.node_start_end[node_sum + 1]['pos'])
                    d_n = {'capacity': ca, 'dis': dis}
                    self.edge_all[edge] = d_n
        # print()

    def construct_flow_network(self):
        # 构建节点和边组成的network
        # 找到相邻的边 添加nodes
        for i in range(len(self.cell_info)):

            if i in self.delete_cell_list:
                continue

            cell_n = self.cell_info[i]
            down_node = cell_n['down_node']
            up_node = cell_n['up_node']

            up_edge = [cell_n['left_up'], cell_n['right_up']]
            if bool(up_node):
                for cell in up_node:
                    # 分别添加临边的节点进去
                    up_cell = self.cell_info[cell]
                    up_cell_down_edge = [up_cell['left_down'], up_cell['right_down']]
                    self.add_new_node(cell_n, up_edge, up_cell, up_cell_down_edge)

        # 找到节点之间的连接关系 建立边 并为边建立容量
        # print()
        for i in range(len(self.node_all)):
            cell_set_i = self.node_all[i]['cell_set']
            for cell_i in cell_set_i:
                for j in range(len(self.node_all)):
                    if i >= j:
                        continue
                    cell_set_j = self.node_all[j]['cell_set']
                    for cell_j in cell_set_j:
                        if cell_i == cell_j:
                            if self.node_all[i]['pos'][1] == self.node_all[j]['pos'][1]:
                                continue
                            ca_min = max(self.node_all[i]['capacity'], self.node_all[j]['capacity'])
                            dis = self.list_distance(self.node_all[i]['pos'], self.node_all[j]['pos'])
                            edge = str(i) + ',' + str(j)
                            # 储存的是 list [capacity, dis]
                            d_n = {'capacity': ca_min, 'dis': dis}
                            self.edge_all[edge] = d_n

        # 找到哪些节点和 起始的cell 终点的cell有联系 并添加起点和终点的 点与边
        s_cell_idx = 0
        g_cell_idx = self.number_cells - 1

        node_sum = len(self.node_all)
        self.start_idx = node_sum
        self.end_idx = node_sum + 1
        self.start_node = node_sum
        self.end_node = node_sum + 1
        self.node_start_end = {}

        # s_pos = self.cell_info[s_cell_idx]['pos']
        s_pos = self.swarm.start_ave_pos
        dic_s = {'pos': s_pos, 'capacity': 1}
        self.node_start_end[node_sum] = dic_s
        # self.node_all[node_sum] = dic_s

        # g_pos = self.cell_info[g_cell_idx]['pos']
        g_pos = self.swarm.goal_ave_pos
        dic_g = {'pos': g_pos, 'capacity': 1}
        self.node_start_end[node_sum + 1] = dic_g
        # self.node_all[node_sum + 1] = dic_g

        for i in range(len(self.node_all)):
            node_cell_set = self.node_all[i]['cell_set']
            for cell in node_cell_set:
                ca = self.node_all[i]['capacity']
                if cell == s_cell_idx:
                    edge = str(node_sum) + ',' + str(i)
                    dis = self.list_distance(self.node_all[i]['pos'], self.node_start_end[node_sum]['pos'])
                    d_n = {'capacity': ca, 'dis': dis}
                    self.edge_all[edge] = d_n
                if cell == g_cell_idx:
                    edge = str(i) + ',' + str(node_sum + 1)
                    dis = self.list_distance(self.node_all[i]['pos'], self.node_start_end[node_sum + 1]['pos'])
                    d_n = {'capacity': ca, 'dis': dis}
                    self.edge_all[edge] = d_n
        # print()

    def compute_path_capacity(self, path_all):

        # delete the longest path
        if len(path_all) >= 3:
            max_key = max(path_all, key=lambda x: path_all[x]['path_length'])
            del path_all[max_key]
            path_all = {new_key: value for new_key, (old_key, value) in enumerate(path_all.items())}

        # add all edge capacity
        capa_all_sum = 0
        for key, p in path_all.items():
            path_node = p['path_node']
            ca_sum = 0
            for i in range(len(path_node) - 1):
                if i == 0:
                    continue
                node_1 = path_node[i]
                node_2 = path_node[i+1]
                edge = str(node_1) + ',' + str(node_2)
                ca = self.edge_all[edge]['capacity']
                ca_sum += ca
            capa_all_sum += ca_sum
            path_all[key]['capacity'] = int(ca_sum)

        ratio_list = []
        for key, p in path_all.items():
            path_capa = p['capacity']
            ratio = path_capa/capa_all_sum
            path_all[key]['ratio'] = ratio
            ratio_list.append(ratio)
        return path_all, ratio_list
    def list_distance(self, l1, l2):
        vector1 = np.array([l1[0], l1[1]])
        vector2 = np.array([l2[0], l2[1]])
        result_vector = vector1 - vector2
        dis = np.linalg.norm(result_vector)
        return dis

    def calculate_discrete_positions(self, p1, p2, b):
        """
        计算并返回线段上的离散位置坐标。
        :return: 一个包含所有离散位置坐标的列表，每个位置也是形如(x, y)的坐标。
        """
        # 计算端点之间的距离
        distance = np.sqrt((p2[0] - p1[0]) ** 2 + (p2[1] - p1[1]) ** 2)

        # 确定最大的离散位置数量
        positions_count = int(distance // b)

        # 计算每个离散位置的坐标
        discrete_positions = []
        for i in range(1, positions_count):
            if i == 0:
                continue
            # 计算当前位置的坐标
            t = i * b / distance
            x = p1[0] + t * (p2[0] - p1[0])
            y = p1[1] + t * (p2[1] - p1[1])
            discrete_positions.append((x, y))

        return discrete_positions

    def add_new_node_one(self, cell_n, cell_n_edge, cell_m, cell_m_edge):
        # 判断后 添加一个新的节点进入
        # 需要附加的信息:相邻的cell 容量 附近的位置
        dis_1 = cell_n_edge[1][0] - cell_n_edge[0][0]
        dis_2 = cell_m_edge[1][0] - cell_m_edge[0][0]

        cell_n_p1 = [cell_n_edge[1][0], cell_n_edge[1][1]]
        cell_n_p2 = [cell_n_edge[0][0], cell_n_edge[0][1]]

        cell_m_p1 = [cell_m_edge[1][0], cell_m_edge[1][1]]
        cell_m_p2 = [cell_m_edge[0][0], cell_m_edge[0][1]]

        # print("cell_n idx=" + str(cell_n['idx']))
        # print(cell_n['pos'])
        #
        # print("cell_m idx=" + str(cell_m['idx']))
        # print(cell_m['pos'])
        # print()

        if dis_1 <= 0 or dis_2 <= 0:
            # pass
            print("add node-error 1")
        if cell_n_edge[0][1] != cell_n_edge[1][1] or cell_m_edge[0][1] != cell_m_edge[1][1]:
            # pass
            print("add node-error 2")
        if dis_2 > dis_1:
            # 以dis_1为中心进行建立节点
            node_pos = [(cell_n_edge[1][0] + cell_n_edge[0][0]) / 2, cell_n_edge[0][1]]
            capacity = int(dis_1 / (2*self.safety_dis))
            node_option_pos = self.calculate_discrete_positions(cell_n_p1, cell_n_p2, 2*self.safety_dis)
            node_option_num = len(node_option_pos)
        else:
            node_pos = [(cell_m_edge[1][0] + cell_m_edge[0][0]) / 2, cell_m_edge[0][1]]
            capacity = int(dis_2 / (2*self.safety_dis))
            node_option_pos = self.calculate_discrete_positions(cell_m_p1, cell_m_p2, 2*self.safety_dis)
            node_option_num = len(node_option_pos)

        partition = 4

        node_option_pos_n = []
        number = 0
        for i in range(node_option_num):
            p = node_option_pos[i]
            node_option_pos_n.append(p)
            number += 1

            if number == partition and i != node_option_num - 1:
                node_option_num_n = len(node_option_pos_n)
                node_pos_n = self.compute_ave_pos(node_option_pos_n)
                # add new node
                idx = len(self.node_all)
                cell_idx_set = set()
                cell_idx_set.add(cell_n['idx'])
                cell_idx_set.add(cell_m['idx'])
                cell_up_set = set()
                cell_down_set = set()
                cell_up_set.add(cell_m['idx'])
                cell_down_set.add(cell_n['idx'])
                c_info = {'pos': node_pos_n, 'capacity': node_option_num_n, 'cell_set': cell_idx_set,
                          'cell_up_set': cell_up_set, 'cell_down_set': cell_down_set,
                          'node_idx': idx, 'node_option_pos': node_option_pos_n, 'node_option_num': node_option_num_n}
                self.node_all[idx] = c_info

                # clear info
                node_option_pos_n = []
                number = 0

            elif i == node_option_num - 1:
                node_option_num_n = len(node_option_pos_n)
                node_pos_n = self.compute_ave_pos(node_option_pos_n)
                # add new node
                idx = len(self.node_all)
                cell_idx_set = set()
                cell_idx_set.add(cell_n['idx'])
                cell_idx_set.add(cell_m['idx'])
                cell_up_set = set()
                cell_down_set = set()
                cell_up_set.add(cell_m['idx'])
                cell_down_set.add(cell_n['idx'])
                c_info = {'pos': node_pos_n, 'capacity': node_option_num_n, 'cell_set': cell_idx_set,
                          'cell_up_set': cell_up_set, 'cell_down_set': cell_down_set,
                          'node_idx': idx, 'node_option_pos': node_option_pos_n, 'node_option_num': node_option_num_n}
                self.node_all[idx] = c_info

                # clear info
                node_option_pos_n = []
                number = 0

    def compute_ave_pos(self, node_option_pos):
        x = 0
        y = 0
        for pos in node_option_pos:
            x += pos[0]
            y += pos[1]
        x = x / len(node_option_pos)
        y = y / len(node_option_pos)

        return (x, y)

    def add_new_node(self, cell_n, cell_n_edge, cell_m, cell_m_edge):
        # 判断后 添加一个新的节点进入
        # 需要附加的信息:相邻的cell 容量 附近的位置
        dis_1 = cell_n_edge[1][0] - cell_n_edge[0][0]
        dis_2 = cell_m_edge[1][0] - cell_m_edge[0][0]

        cell_n_p1 = [cell_n_edge[1][0], cell_n_edge[1][1]]
        cell_n_p2 = [cell_n_edge[0][0], cell_n_edge[0][1]]

        cell_m_p1 = [cell_m_edge[1][0], cell_m_edge[1][1]]
        cell_m_p2 = [cell_m_edge[0][0], cell_m_edge[0][1]]

        if dis_1 <= 0 or dis_2 <= 0:
            print("add node-error 1")
        if cell_n_edge[0][1] != cell_n_edge[1][1] or cell_m_edge[0][1] != cell_m_edge[1][1]:
            print("add node-error 2")
        if dis_2 > dis_1:
            # 以dis_1为中心进行建立节点
            node_pos = [(cell_n_edge[1][0] + cell_n_edge[0][0]) / 2, cell_n_edge[0][1]]
            capacity = int(dis_1 / (2*self.safety_dis))
            node_option_pos = self.calculate_discrete_positions(cell_n_p1, cell_n_p2, 2*self.safety_dis)
            node_option_num = len(node_option_pos)
        else:
            node_pos = [(cell_m_edge[1][0] + cell_m_edge[0][0]) / 2, cell_m_edge[0][1]]
            capacity = int(dis_2 / (2*self.safety_dis))
            node_option_pos = self.calculate_discrete_positions(cell_m_p1, cell_m_p2, 2*self.safety_dis)
            node_option_num = len(node_option_pos)

        idx = len(self.node_all)
        cell_idx_set = set()
        cell_idx_set.add(cell_n['idx'])
        cell_idx_set.add(cell_m['idx'])

        cell_up_set = set()
        cell_down_set = set()
        cell_up_set.add(cell_m['idx'])
        cell_down_set.add(cell_n['idx'])

        c_info = {'pos': node_pos, 'capacity': capacity, 'cell_set': cell_idx_set,
                  'cell_up_set': cell_up_set, 'cell_down_set': cell_down_set,
                  'node_idx': idx, 'node_option_pos': node_option_pos, 'node_option_num': node_option_num}
        self.node_all[idx] = c_info

    def extract_in_out_pos(self):
        # 找到所有的上下边界in out的位置
        # 根据离散的格子数 确定这些边界位置的中心位置 位置的个数 位置的次序
        # up_pos [[]] up_pos_num
        # down_pos [[]] down_pos_num
        # 提取的时候需要忽略那些被obs挡住的 从而更好地提取信息
        for i in range(self.number_cells):
            if i == 0:
                continue
            # 当前的节点信息
            cell_info_n = self.cell_info[i]
            left_up = cell_info_n['left_up']
            left_down = cell_info_n['left_down']
            right_up = cell_info_n['right_up']
            right_down = cell_info_n['right_down']

    def extract_info(self):
        self.x_size = self.map_all.shape[0]
        self.y_size = self.map_all.shape[1]

        self.cell_dict = {index - 1: value for index, value in self.cell_dict.items() if index != 0}
        self.number_cells = self.number_cells - 1

        for i in range(self.number_cells):
            dic_n = {}
            area_i = len(self.cell_dict[i])
            dic_n['area'] = area_i

            # # 先找 x轴的左右边界 然后再找y轴的上下边界
            # x_left, x_right = self.find_left_right_boundary_cell(self.cell_dict[i])
            # left_up, left_down, right_up, right_down = self.find_up_down_boundary(x_left, x_right, self.cell_dict[i])
            # node1 = [x_left, left_up]
            # node2 = [x_left, left_down]
            # node3 = [x_right, right_up]
            # node4 = [x_right, right_down]

            # 先找 y轴的上下边界 然后再找x轴的左右边界
            y_up, y_down = self.find_up_down_boundary_cell(self.cell_dict[i])

            left_up, left_down, right_up, right_down = self.find_left_right_boundary(y_up, y_down, self.cell_dict[i])
            node1 = [left_up, y_up]
            node2 = [left_down, y_down]
            node3 = [right_up, y_up]
            node4 = [right_down, y_down]

            dic_n['left_up'] = node1
            dic_n['left_down'] = node2
            dic_n['right_up'] = node3
            dic_n['right_down'] = node4
            # dic_n['pos'] = [(x_left + x_right) / 2 / self.resolution,
            #                 (left_up + left_down + right_up + right_down) / 4 / self.resolution]
            dic_n['pos'] = [(left_up + left_down + right_up + right_down) / 4 / self.resolution
                , (y_up + y_down) / 2 / self.resolution, ]
            dic_n['idx'] = i

            self.cell_info[i] = dic_n

    def construct_cell_structure(self):
        # 建立cell之间的连接关系
        # 找到上下边界和哪些节点相连接
        for i in range(0, self.number_cells):
            # print("cell=" + str(i))
            cell_n = self.cell_dict[i]
            cell_info_n = self.cell_info[i]

            left_up = cell_info_n['left_up']
            left_down = cell_info_n['left_down']
            right_up = cell_info_n['right_up']
            right_down = cell_info_n['right_down']

            # print(left_up)
            # print(left_down)
            # print(right_up)
            # print(right_down)
            # 遍历下边界 找到有哪些cell相邻
            down_y = left_down[1]
            left_x = left_down[0]
            right_x = right_down[0]
            down_node = set()
            for x in range(left_x, right_x + 1):
                idx = [x, down_y - 1]
                value = self.map_all[idx[0], idx[1]] - 1
                if value != -1 and value != i:
                    down_node.add(value)
            # 遍历上边界 找到哪些cell相邻
            up_y = right_up[1]
            left_x = left_up[0]
            right_x = right_up[0]
            up_node = set()
            for x in range(left_x, right_x + 1):
                idx = [x, up_y + 1]
                value = self.map_all[idx[0], idx[1]] - 1
                if value != -1 and value != i:
                    up_node.add(value)
            self.cell_info[i]['down_node'] = down_node
            self.cell_info[i]['up_node'] = up_node

    def find_up_down_boundary_cell(self, cell_dict):
        # 找到当前左右边界的x值
        y_down = 999999999
        y_up = -100
        for idx in cell_dict:
            if idx[1] <= y_down:
                y_down = idx[1]
            if idx[1] >= y_up:
                y_up = idx[1]
        return y_up, y_down

    def find_left_right_boundary_cell(self, cell_dict):
        # 找到当前左右边界的x值
        x_left = 999999999
        x_right = -100
        for idx in cell_dict:
            if idx[0] <= x_left:
                x_left = idx[0]
            if idx[0] >= x_right:
                x_right = idx[0]
        return x_left, x_right

    def find_left_right_boundary(self, y_up, y_down, cell):
        # 找到上下边界边界的左右y值
        left_up = 999999999
        right_up = -100

        left_down = 99999999
        right_down = -100

        if y_up == y_down:
            for idx in cell:
                if idx[1] == y_up:
                    if idx[0] <= left_up:
                        left_up = idx[0]
                    if idx[0] >= right_up:
                        right_up = idx[0]
            right_down = right_up
            left_down = left_up
            return left_up, left_down, right_up, right_down

        for idx in cell:
            if idx[1] == y_up:
                if idx[0] <= left_up:
                    left_up = idx[0]
                if idx[0] >= right_up:
                    right_up = idx[0]
            elif idx[1] == y_down:
                if idx[0] >= right_down:
                    right_down = idx[0]
                if idx[0] <= left_down:
                    left_down = idx[0]
            else:
                pass

        return left_up, left_down, right_up, right_down

    def find_up_down_boundary(self, x_left, x_right, cell):
        # 找到左右边界的上下y值
        left_up = -1
        left_down = 9999999
        right_up = -1
        right_down = 9999999

        for idx in cell:
            if idx[0] == x_left:
                if idx[1] >= left_up:
                    left_up = idx[1]
                if idx[1] <= left_down:
                    left_down = idx[1]
            elif idx[0] == x_right:
                if idx[1] >= right_up:
                    right_up = idx[1]
                if idx[1] <= right_down:
                    right_down = idx[1]
            else:
                pass

        return left_up, left_down, right_up, right_down

    def find_obs_rectangles(self, map_array):
        nrows, ncols = map_array.shape
        visited = np.zeros_like(map_array, dtype=bool)
        rectangles = []

        def dfs_iterative(x, y, rect):
            stack = [(x, y)]
            while stack:
                x, y = stack.pop()
                if x < 0 or x >= nrows or y < 0 or y >= ncols or visited[x, y] or map_array[x, y] != 0:
                    continue
                visited[x, y] = True
                rect[0] = min(rect[0], x)
                rect[1] = min(rect[1], y)
                rect[2] = max(rect[2], x)
                rect[3] = max(rect[3], y)
                stack.extend([(x + dx, y + dy) for dx, dy in [(0, 1), (1, 0), (0, -1), (-1, 0)]])

        for i in range(nrows):
            for j in range(ncols):
                if map_array[i, j] == 0 and not visited[i, j]:
                    rect = [i, j, i, j]  # x_min, y_min, x_max, y_max
                    dfs_iterative(i, j, rect)
                    # Convert to top-left and bottom-right corners
                    rectangles.append([(rect[0], rect[1]), (rect[2], rect[3])])

        return len(rectangles), rectangles

    def plot_matrix_map(self, path):
        plt.figure()

        # 将为1的元素plot成灰色
        plt.imshow(self.map_01 == 1, cmap='Greys', origin='lower')

        # 将为0的元素plot成白色
        plt.imshow(self.map_01 == 0, cmap='Accent_r', origin='lower', alpha=0.5)

        path = np.array(path)
        plt.plot(path[:, 1], path[:, 0], color='lime', linewidth=2)
        # 显示绘制结果
        plt.show()

    def display_separate_cell(self, separate_map, cells):
        fig_new = plt.figure()

        display_img = np.empty([*separate_map.shape, 3], dtype=np.uint8)
        random_colors = np.random.randint(0, 255, [cells, 3])
        for cell_id in range(1, cells):
            display_img[separate_map == cell_id, :] = random_colors[cell_id, :]
        for idx, cell in self.cell_info.items():
            pos = cell['pos']
            name = 'Ce' + str(idx)
            plt.text(pos[0] * self.resolution, pos[1] * self.resolution, name, ha='center', va='center')

        display_img_rotated = np.rot90(display_img)
        display_img_rotated_flipped = np.flipud(display_img_rotated)
        plt.imshow(display_img_rotated_flipped)
        plt.gca().invert_yaxis()
        # plt.imshow(display_img)

    def read_map(self):
        # Read the original data
        original_map = cv2.imread(self.pic_name)

        # 1's represents free space while 0's represents objects/walls
        if len(original_map.shape) > 2:
            # print("Map image is converted to binary")
            single_channel_map = original_map[:, :, 0]
            _, binary_map = cv2.threshold(single_channel_map, 127, 1, cv2.THRESH_BINARY)

        num_obs, obstacles_rect = self.find_obs_rectangles(binary_map)

        # Call The Boustrophedon Cellular Decomposition function
        bcd_out_im, bcd_out_cells, cell_numbers, cell_boundaries, non_neighboor_cell_numbers = bcd(binary_map)

        self.bcd_out_cells = copy.deepcopy(bcd_out_cells)
        self.bcd_out_im = copy.deepcopy(bcd_out_im)

        self.number_cells = bcd_out_cells
        self.map_all = bcd_out_im

        self.map_01 = copy.deepcopy(bcd_out_im)
        self.map_01[self.map_01 >= 1] = 2
        # 将为0的元素赋值为1
        self.map_01[self.map_01 == 0] = 1
        # 将所有为-1的元素赋值为0
        self.map_01[self.map_01 == 2] = 0
        # self.map_01 = self.map_01[self.map_01 != 0] = 1

        dilated_size = 10

        structure_element = np.ones((dilated_size, dilated_size))

        # 对障碍物地图进行膨胀操作
        self.dilated_map = binary_dilation(self.map_01, structure=structure_element).astype(self.map_01.dtype)

        # self.plot_matrix_map()
        self.num_obs_rect = num_obs
        self.obstacles_rect = obstacles_rect

        for i in range(bcd_out_cells):
            li_n = []
            self.cell_dict[i] = copy.copy(li_n)
        for idx, element in np.ndenumerate(bcd_out_im):
            # print(f"Index: {idx}, Value: {element}")
            li_index = [idx[0], idx[1]]
            self.cell_dict[element].append(copy.copy(li_index))

