import copy
import logging
import time
import math
import yaml
from matplotlib.patches import Circle
import cv2
import numpy as np
import matplotlib.pyplot as plt
import networkx as nx
from scipy.ndimage import binary_dilation
from pathfinding.dijkstra import Dijkstra
from pathfinding.Astar import Astar
from pathfinding.jps import JPS
from .FF import FlowAlgo
from .bcd import bcd


class MapInfo:
    """Class to manage map decomposition, pathfinding, and network construction for robotic swarm navigation."""

    def __init__(self, swarm, pic_name, resolution):
        """Initialize MapInfo with swarm, picture name, and resolution."""
        self.swarm = swarm
        self.pic_name = pic_name
        self.resolution = resolution

        # Map dimensions and obstacles
        self.x_size = None
        self.y_size = None
        self.obstacles = None
        self.num_obs_rect = None
        self.obstacles_rect = None

        # Cell and node data
        self.number_cells = None
        self.cell_dict = {}  # Maps cell number to indices
        self.cell_info = {}  # Stores cell metadata (area, boundaries, etc.)
        self.delete_cell_list = None

        # cell network
        self.node_all = {}  # All nodes in the network
        self.edge_all = {}  # All edges in the network

        # Adj network
        self.node_all_adj = {}  # All nodes in the network
        self.edge_all_adj = {}  # All edges in the network

        self.cellId_to_up_cellNode = {}  # 通过cell 直接映射到下边界的cellNode
        self.cellId_to_down_cellNode = {}  # 通过cell 直接映射到下边界的cellNode

        # 通过cell id 找到它们上下边界的path_node
        self.cellId_to_up_pathNode = {}
        self.cellId_to_down_pathNode = {}

        self.cellNode_to_pathNode = {}  # 通过cellNode 映射到到pathNode

        self.node_to_goal = {}  # Paths from nodes to goal
        self.jps_path_all = {}
        self.start_node = None
        self.end_node = None
        self.start_idx = None
        self.end_idx = None
        self.start_cell = None
        self.goal_cell = None

        self.path_all = None

        self.node_start_end = {}
        self.node_start_end_adj = {}

        # 通过node 映射到 下一个相连的node 有哪些
        self.node_to_neighborNode = {}

        # Map representations
        self.map_all = None  # Full map with cell IDs
        self.map_01 = None  # Binary map (0 for obstacles, 1 for free space)
        self.dilated_map = None  # Dilated binary map for pathfinding

        # Algorithm instances
        self.FF_algo = FlowAlgo()
        self.dijkstra = Dijkstra()
        self.jps = JPS()
        self.Astar = Astar()
        self.dijkstra_result = {}

        # Configuration parameters
        self.distance = 1
        self.num_grids = 3
        self.safety_dis = self.swarm.robot_radius * 2

        self.area_map = None
        self.area_dict = None
        self.area_capacity = None

        self.global_binary_map = None
        self.local_binary_map = None

    def init_main_yaml(self):
        """Initialize map decomposition and network construction from a YAML file."""
        logging.info("Map decomposition and network construction started!")
        self.read_map_yaml()

        self.extract_info()
        self.subdivide_cells(5, 5)
        self.construct_cell_structure()

        self.deal_with_resolution()

        # self.cell_net_construct()
        self.adj_net_construct()
        # self.cellId_info_build()
        # self.draw_separate_cell(self.area_map, len(self.area_dict))
        self.edge_info_build()

        # self.draw_separate_cell(self.bcd_out_im, self.bcd_out_cells)
        # self.draw_separate_cell_area(self.area_map, len(self.area_dict))

        # self.draw_network_adj()
        # self.draw_network_cell()

        self.construct_dijkstra()
        self.update_para()

    def read_map_yaml(self):
        """Read and process a map from a YAML file."""

        # 这里是通过读取yaml文件得到开始list 和 goal list
        # 以及dimensions 以及obstacles
        # 前三个数据是稳定的 但是obs是变化的
        # 这里可以写一个根据无人机当前的位置 更新obs的函数 通过这个函数去得到
        start_list, goal_list, dimensions, obstacles = self.input_map_yaml_file()
        self.global_binary_map = np.ones((dimensions[0] + 1, dimensions[1] + 1), dtype=int)

        self.x_size = dimensions[0]
        self.y_size = dimensions[1]
        # Set boundaries as obstacles
        self.global_binary_map[0, :] = 0
        self.global_binary_map[-1, :] = 0
        self.global_binary_map[:, 0] = 0
        self.global_binary_map[:, -1] = 0

        # Place obstacles
        for obs in obstacles:
            self.global_binary_map[obs[0], obs[1]] = 0
        # self.update_map(dimensions, obstacles)

    def local_obs_perception(self, obstacles):
        # 根据当下无人机的位置 以及 观测的范围
        # 去根据这些去更新obs
        # 得到obs和map后 进一步去

        # obs如果被观测到了 那么就一直存在
        # obs如果没有被观测到 那么就不存在

        # self.swarm.pos_all 存储着所有无人机的位置
        # self.global_binary_map 是全局的地图
        # 输出是 self.local_binary_map
        pass


    def update_map(self,  ):
        # Perform Boustrophedon Cellular Decomposition
        bcd_out_im, bcd_out_cells, _, _, _ = bcd(self.local_binary_map)
        self._process_bcd_output(bcd_out_im, bcd_out_cells)


    def _process_bcd_output(self, bcd_out_im, bcd_out_cells):
        """Common processing for BCD output from image or YAML."""
        self.bcd_out_cells = copy.deepcopy(bcd_out_cells)
        self.bcd_out_im = copy.deepcopy(bcd_out_im)
        self.number_cells = bcd_out_cells
        self.map_all = bcd_out_im

        # Create binary map
        self.map_01 = copy.deepcopy(bcd_out_im)
        self.map_01[self.map_01 >= 1] = 2
        self.map_01[self.map_01 == 0] = 1
        self.map_01[self.map_01 == 2] = 0

        # Dilate map for safety
        structure_element = np.ones((3, 3))
        self.dilated_map = binary_dilation(self.map_01, structure=structure_element).astype(self.map_01.dtype)

        # Initialize cell dictionary
        for i in range(bcd_out_cells):
            self.cell_dict[i] = []
        for idx, element in np.ndenumerate(bcd_out_im):
            self.cell_dict[element].append([idx[0], idx[1]])

    def edge_info_build(self):
        # 根据边的信息 构建通过不同区域的比例
        # 方便后续计算拥堵的时候 直接调用相关信息
        for idx, edge in self.edge_all_adj.items():
            # print("edge=" + str(idx))
            start_node = edge['start']
            end_node = edge['end']
            if start_node == self.start_node:
                start_pos = self.node_start_end_adj[start_node]['pos']
            else:
                start_pos = self.node_all_adj[start_node]['pos']
            if end_node == self.end_node:
                end_pos = self.node_start_end_adj[end_node]['pos']
            else:
                end_pos = self.node_all_adj[end_node]['pos']

            result = self.find_path_area_len(start_pos, end_pos, self.area_map)
            # print(result)
            self.edge_all_adj[idx]['area_info'] = result

    def find_path_area_len(self, start_pos, end_pos, map_n):
        """
        计算从start_pos到end_pos的直线经过的区域和每个区域的长度
        参数:
            start_pos: tuple (row1, col1) 起点坐标
            end_pos: tuple (row2, col2) 终点坐标
            map: numpy.ndarray 二维地图
        返回:
            list of tuples: [(area_value1, length1), (area_value2, length2), ...]
        """
        y1, x1 = start_pos  # row, column
        y2, x2 = end_pos

        # 获取地图的尺寸
        height, width = map_n.shape  # 使用传入的map参数

        # 检查输入坐标是否有效
        if not (0 <= x1 < width and 0 <= y1 < height and
                0 <= x2 < width and 0 <= y2 < height):
            raise ValueError("Coordinates out of map bounds")

        # 计算直线总长度
        total_length = np.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)

        # 如果起点和终点相同
        if start_pos == end_pos:
            return [(map_n[y1, x1], 0.0)]

        # 计算步数（使用较长的维度来确定步数）
        dx = x2 - x1
        dy = y2 - y1
        steps = int(max(abs(dx), abs(dy))) + 1

        # 计算每一步的增量
        x_step = dx / steps
        y_step = dy / steps

        # 存储路径上的点和对应的区域值
        points = []
        for i in range(steps + 1):
            x = int(round(x1 + x_step * i))
            y = int(round(y1 + y_step * i))
            # 确保不超出边界
            x = min(max(x, 0), width - 1)
            y = min(max(y, 0), height - 1)
            points.append((x, y, map_n[y, x]))  # 使用map读取值

        # 计算每个区域的长度
        result = []
        current_value = points[0][2]
        segment_start = 0

        for i in range(1, len(points)):
            if points[i][2] != current_value:
                # 计算前一段的长度
                segment_end = i
                segment_points = points[segment_start:segment_end]
                segment_length = 0
                for j in range(len(segment_points) - 1):
                    x1, y1 = segment_points[j][0], segment_points[j][1]
                    x2, y2 = segment_points[j + 1][0], segment_points[j + 1][1]
                    segment_length += np.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)
                if segment_length > 0 and current_value != 0:
                    result.append((current_value, round(segment_length, 2)))

                # 开始新段
                current_value = points[i][2]
                segment_start = i

        # 处理最后一段
        if segment_start < len(points):
            segment_points = points[segment_start:]
            segment_length = 0
            for j in range(len(segment_points) - 1):
                x1, y1 = segment_points[j][0], segment_points[j][1]
                x2, y2 = segment_points[j + 1][0], segment_points[j + 1][1]
                segment_length += np.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)
            if segment_length > 0 and current_value != 0:
                result.append((current_value, round(segment_length, 2)))

        return result

    def subdivide_cells(self, min_width=3, min_height=3):
        """
        将已有cells细分成更小的块
        horizontal_splits: 横向切分次数
        vertical_splits: 纵向切分次数
        """
        # 创建新的cell字典和矩阵
        new_cell_dict = {}
        new_map = np.zeros_like(self.map_all)
        current_value = 1

        # 遍历每个原始cell
        for cell_value, coordinates in self.cell_dict.items():
            if cell_value == 0 or cell_value == len(self.cell_dict) - 1:
                new_cell_dict[current_value] = coordinates.copy()
                for x, y in coordinates:
                    new_map[x, y] = current_value
                current_value += 1
                continue
            if not coordinates:
                continue

            # 获取当前cell的边界
            coords_array = np.array(coordinates)
            min_x, min_y = coords_array.min(axis=0)
            max_x, max_y = coords_array.max(axis=0)

            # 计算当前cell的实际宽高
            cell_width = max_x - min_x + 1
            cell_height = max_y - min_y + 1

            # 检查宽度和高度条件
            if cell_width < min_width and cell_height < min_height:
                # 宽高都小于最小值，不分割
                new_cell_dict[current_value] = coordinates.copy()
                for x, y in coordinates:
                    new_map[x, y] = current_value
                current_value += 1
                continue
            elif cell_width < min_width and cell_height >= min_height * 3:
                # 宽小于最小值，但高是最小值的3倍以上，只在高上分割
                horizontal_splits = 1
                vertical_splits = max(1, int(cell_height / min_height))
            elif cell_height < min_height and cell_width >= min_width * 3:
                # 高小于最小值，但宽是最小值的3倍以上，只在宽上分割
                horizontal_splits = max(1, int(cell_width / min_width))
                vertical_splits = 1
            else:
                # 正常情况下的分割
                horizontal_splits = max(1, int(cell_width / min_width))
                vertical_splits = max(1, int(cell_height / min_height))

            # 计算每个小块的大小
            width = cell_width / horizontal_splits
            height = cell_height / vertical_splits

            # 进行切分
            for i in range(horizontal_splits):
                for j in range(vertical_splits):
                    # 计算当前小块的边界
                    x_start = int(min_x + i * width)
                    x_end = int(min_x + (i + 1) * width)
                    y_start = int(min_y + j * height)
                    y_end = int(min_y + (j + 1) * height)

                    # 创建新cell
                    new_cell_coords = []
                    for x in range(x_start, x_end):
                        for y in range(y_start, y_end):
                            if [x, y] in coordinates:
                                new_cell_coords.append([x, y])
                                new_map[x, y] = current_value

                    if new_cell_coords:  # 只保存非空的cell
                        new_cell_dict[current_value] = new_cell_coords
                        current_value += 1

        # 更新类的属性
        # self.cell_dict = new_cell_dict
        self.area_map = new_map
        self.area_dict = new_cell_dict

        self.area_capacity = {}
        for idx, area in self.area_dict.items():
            self.area_capacity[idx] = len(area)
        # print()
        # self.draw_separate_cell(new_map, current_value)
        # self.draw_separate_cell(new_map, current_value)

        # 重新生成二值地图
        # self.map_01 = copy.deepcopy(new_map)
        # self.map_01[self.map_01 >= 1] = 0  # Free space
        # self.map_01[self.map_01 == 0] = 1  # Obstacles

        # 更新膨胀地图
        # structure_element = np.ones((3, 3))
        # self.dilated_map = binary_dilation(self.map_01, structure=structure_element).astype(self.map_01.dtype)

    def input_map_yaml_file(self):
        """Parse YAML file for map data."""
        # yaml_file_name = "pic/map_yaml/m1-100.yaml"
        yaml_file_name = self.pic_name
        # yaml_file_name = "pic/map_yaml/m1-500-hard.yaml"
        with open(yaml_file_name, 'r') as file:
            data = yaml.safe_load(file)
            agents = data.get('agents', [])
            start_list = [agent['start'] for agent in agents]
            goal_list = [agent['goal'] for agent in agents]
            map_info = data.get('map', {})
            dimensions = map_info.get('dimensions')
            obstacles = map_info.get('obstacles', [])
            self.obs_inside_idx = map_info.get('obs_inside', [])
            self.obs_vertices = map_info.get('obs_vertices')
            return start_list, goal_list, dimensions, obstacles

    # --- Map Processing Methods ---
    def cellId_info_build(self):
        # 先构建cell node相关信息
        # 通过cell id 直接找到 cell的上下边界 cell node
        self.cellId_to_up_cellNode = {}
        self.cellId_to_down_cellNode = {}
        for idx, node in self.node_all.items():
            cell_up_set = node['cell_up_set']
            cell_down_set = node['cell_down_set']
            cell_set = node['cell_set']
            # 中间的窄节点不考虑
            # 确保 cell_set 长度为2且 cell_up_set 和 cell_down_set 不为空
            if len(cell_set) == 2 and cell_up_set and cell_down_set:
                cell_up_id = list(cell_up_set)[0]
                cell_down_id = list(cell_down_set)[0]
                if cell_down_id in self.cellId_to_up_cellNode:
                    self.cellId_to_up_cellNode[cell_down_id].add(idx)
                else:
                    self.cellId_to_up_cellNode[cell_down_id] = {idx}
                if cell_up_id in self.cellId_to_down_cellNode:
                    self.cellId_to_down_cellNode[cell_up_id].add(idx)
                else:
                    self.cellId_to_down_cellNode[cell_up_id] = {idx}

        self.cellId_to_up_pathNode = {}
        self.cellId_to_down_pathNode = {}
        for idx, node in self.node_all_adj.items():
            cell_up_set = node['cell_up_set']
            cell_down_set = node['cell_down_set']
            # 确保 cell_up_set 和 cell_down_set 不为空
            if cell_up_set and cell_down_set:
                cell_up_id = list(cell_up_set)[0]
                cell_down_id = list(cell_down_set)[0]
                if cell_down_id in self.cellId_to_up_pathNode:
                    self.cellId_to_up_pathNode[cell_down_id].add(idx)
                else:
                    # 使用 {idx} 创建包含单个元素的集合
                    self.cellId_to_up_pathNode[cell_down_id] = {idx}
                if cell_up_id in self.cellId_to_down_pathNode:
                    self.cellId_to_down_pathNode[cell_up_id].add(idx)
                else:
                    # 使用 {idx} 创建包含单个元素的集合
                    self.cellId_to_down_pathNode[cell_up_id] = {idx}

        # 通过 cell node 直接找到path node
        self.cellNode_to_pathNode = {}
        # 找到 cellNode 的 上cell 和 下cell
        # 判断哪些 pathNode 的 上cell 和 下cell 相同 如果相同就认为是一组的
        for idx, node in self.node_start_end.items():
            if node['name'] == 'start':
                cell_node_start = idx
            elif node['name'] == 'goal':
                cell_node_end = idx
        for idx, node in self.node_start_end_adj.items():
            if node['name'] == 'start':
                path_node_start = idx
            if node['name'] == 'goal':
                path_node_end = idx
        self.cellNode_to_pathNode[cell_node_start] = path_node_start
        self.cellNode_to_pathNode[cell_node_end] = path_node_end

        for idx, node in self.node_all.items():
            cell_up_set = node['cell_up_set']
            cell_down_set = node['cell_down_set']
            cell_set = node['cell_set']
            if cell_up_set and cell_down_set:
                cell_up_id = list(cell_up_set)[0]
                cell_down_id = list(cell_down_set)[0]
            path_node_set = set()
            for idx_path, node_path in self.node_all_adj.items():
                cell_up_set_path = node_path['cell_up_set']
                cell_down_set_path = node_path['cell_down_set']
                if cell_up_id in cell_up_set_path and cell_down_id in cell_down_set_path:
                    path_node_set.add(idx_path)
            self.cellNode_to_pathNode[idx] = path_node_set

    def extract_info(self):
        """Extract map dimensions and cell information."""
        self.x_size = self.map_all.shape[0]
        self.y_size = self.map_all.shape[1]

        # Filter out cells spanning entire map height
        keys_to_delete = [i for i in range(self.number_cells) if
                          abs(abs(self.find_up_down_boundary_cell(self.cell_dict[i])[0] -
                                  self.find_up_down_boundary_cell(self.cell_dict[i])[1]) - self.y_size) <= 5]
        self.cell_dict = {k - len(keys_to_delete): v for k, v in self.cell_dict.items() if k not in keys_to_delete}
        self.cell_dict = {i: value for i, value in enumerate([lst for lst in self.cell_dict.values() if lst])}
        self.number_cells = len(self.cell_dict)

        for value, cell in self.cell_dict.items():
            for cell_idx in cell:
                self.map_all[cell_idx[0], cell_idx[1]] = value + 1

        # Extract cell boundaries and properties
        for i in range(self.number_cells):
            dic_n = {'area': len(self.cell_dict[i])}
            y_up, y_down = self.find_up_down_boundary_cell(self.cell_dict[i])
            left_up, left_down, right_up, right_down = self.find_left_right_boundary(y_up, y_down, self.cell_dict[i])
            min_left_x, min_right_x, y_narrowest = self.find_inter_narrowest_edge(self.cell_dict[i])
            map_value = self.find_cell_map_value(self.cell_dict[i])

            dic_n.update({
                'idx': map_value,
                'left_up': [left_up, y_up],
                'left_down': [left_down, y_down],
                'right_up': [right_up, y_up],
                'right_down': [right_down, y_down],
                'pos': [(left_up + left_down + right_up + right_down) / 4 / self.resolution,
                        (y_up + y_down) / 2 / self.resolution]
            })

            narrowest_width = abs(min_right_x - min_left_x)
            up_width = abs(left_up - right_up)
            down_width = abs(left_down - right_down)
            dic_n['inter_exist'] = not (abs(narrowest_width - up_width) <= 2 or abs(narrowest_width - down_width) <= 2)
            if dic_n['inter_exist']:
                dic_n.update({'inter_left': [min_left_x, y_narrowest], 'inter_right': [min_right_x, y_narrowest]})

            self.cell_info[map_value] = dic_n

        self.start_cell = 1
        self.goal_cell = len(self.cell_info)


    def construct_cell_structure(self):
        """Build adjacency relationships between cells."""
        for i in range(1, self.number_cells + 1):
            cell_info_n = self.cell_info[i]
            left_down, right_down = cell_info_n['left_down'], cell_info_n['right_down']
            left_up, right_up = cell_info_n['left_up'], cell_info_n['right_up']

            # Find downward adjacent cells
            down_cell = set()
            for x in range(left_down[0], right_down[0] + 1):
                idx = [x, left_down[1] - 1]
                value = self.map_all[idx[0], idx[1]]
                if value != 0 and value != i:
                    down_cell.add(value)

            # Find upward adjacent cells
            up_cell = set()
            for x in range(left_up[0], right_up[0] + 1):
                idx = [x, right_up[1] + 1]
                value = self.map_all[idx[0], idx[1]]
                if value != 0 and value != i:
                    up_cell.add(value)

            self.cell_info[i].update({'down_cell': down_cell, 'up_cell': up_cell})

    def find_cell_map_value(self, cell_dict):
        value = None
        for idx in cell_dict:
            value = self.map_all[idx[0], idx[1]]
            break
        return value

    def find_up_down_boundary_cell(self, cell_dict):
        # 找到当前左右边界的x值
        y_down = float('inf')
        y_up = -float('inf')
        for idx in cell_dict:
            if idx[1] <= y_down:
                y_down = idx[1]
            if idx[1] >= y_up:
                y_up = idx[1]
        return y_up, y_down

    def find_inter_narrowest_edge(self, cell_dict):
        """
        找到这个cell区域中 最窄的那个横向边 并返回 左右的边界x 坐标
        """
        # 初始化最小宽度为无穷大
        min_width = float('inf')
        # 用于存储所有对应最小宽度的y值
        y_narrowest_list = []
        # 初始化最窄边的左右边界
        min_left_x = None
        min_right_x = None

        # 获取cell区域的上下边界
        y_up, y_down = self.find_up_down_boundary_cell(cell_dict)
        # 遍历每一行
        # print(y_up)
        # print(y_down)
        for y in range(y_down, y_up + 1):
            # 获取当前行的左右边界
            left_x, right_x = self.find_row_boundary(cell_dict, y)
            # 计算当前行的宽度
            width = right_x - left_x

            # 如果当前行的宽度小于最小宽度，则更新最小宽度和最窄边的左右边界
            if width < min_width:
                min_width = width
                min_left_x = left_x
                min_right_x = right_x
                # 清空之前存储的y值，因为找到了更小的宽度
                y_narrowest_list = [y]
            # 如果当前行的宽度等于最小宽度，则将y值添加到列表中
            elif width == min_width:
                y_narrowest_list.append(y)

        # 找到y_narrowest_list中处于最中间的y值
        if y_narrowest_list:
            y_narrowest_list.sort()
            middle_index = len(y_narrowest_list) // 2
            y_narrowest = y_narrowest_list[middle_index]
            min_left_x, min_right_x = self.find_row_boundary(cell_dict, y_narrowest)
        else:
            y_narrowest = None

        return min_left_x, min_right_x, y_narrowest

    def find_row_boundary(self, cell_dict, y):
        """
        用于找到指定行的左右边界
        """
        left_x = float('inf')
        right_x = -float('inf')
        for idx in cell_dict:
            if idx[1] == y:
                if idx[0] < left_x:
                    left_x = idx[0]
                if idx[0] > right_x:
                    right_x = idx[0]
        return left_x, right_x

    def find_left_right_boundary_cell(self, cell_dict):
        """
        找到当前cell区域的左右边界的x值
        """
        # 初始化左右边界为极端值
        left_x = float('inf')
        right_x = -float('inf')
        # 遍历cell区域中的每个点
        for idx in cell_dict:
            # 更新左边界
            if idx[0] < left_x:
                left_x = idx[0]
            # 更新右边界
            if idx[0] > right_x:
                right_x = idx[0]
        return left_x, right_x

    def find_left_right_boundary(self, y_up, y_down, cell):
        """
        找到上下边界对应的左右边界的x值
        """
        # 初始化上下边界对应的左右边界
        left_up = float('inf')
        right_up = -float('inf')
        left_down = float('inf')
        right_down = -float('inf')

        # 如果上下边界相同
        if y_up == y_down:
            for idx in cell:
                if idx[1] == y_up:
                    if idx[0] < left_up:
                        left_up = idx[0]
                    if idx[0] > right_up:
                        right_up = idx[0]
            # 上下边界相同，左右边界也相同
            right_down = right_up
            left_down = left_up
        else:
            for idx in cell:
                if idx[1] == y_up:
                    if idx[0] < left_up:
                        left_up = idx[0]
                    if idx[0] > right_up:
                        right_up = idx[0]
                elif idx[1] == y_down:
                    if idx[0] > right_down:
                        right_down = idx[0]
                    if idx[0] < left_down:
                        left_down = idx[0]

        return left_up, left_down, right_up, right_down

    def find_up_down_boundary(self, x_left, x_right, cell):
        # 找到左右边界的上下y值
        left_up = -float('inf')
        left_down = float('inf')
        right_up = -float('inf')
        right_down = float('inf')

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

    def deal_with_resolution(self):
        """Adjust cell coordinates based on resolution."""
        for cell in self.cell_info.values():
            for key in ['left_up', 'left_down', 'right_up', 'right_down']:
                cell[key] = [x / self.resolution for x in cell[key]]

    def delete_small_cell(self):
        """Mark cells with small y-gap for deletion."""
        self.delete_cell_list = {i for i in range(len(self.cell_info))
                                 if (self.cell_info[i]['left_up'][1] - self.cell_info[i]['left_down'][1]) <= 5}

    # --- Network Construction Methods ---

    def cell_net_construct(self):
        """Construct a simple cell network."""
        # Add nodes at cell boundaries
        inter_node_list = []
        for i in range(len(self.cell_info)):
            cell_n = self.cell_info[i]
            up_cell = cell_n['up_cell']
            up_edge = [cell_n['left_up'], cell_n['right_up']]
            if cell_n['inter_exist']:
                inter_line = [cell_n['inter_left'], cell_n['inter_right']]
                self.add_new_node_only_one(cell_n, inter_line, cell_n, inter_line)
            if up_cell:
                for cell_id in up_cell:
                    up_cell_info = self.cell_info[cell_id]
                    up_cell_down_edge = [up_cell_info['left_down'], up_cell_info['right_down']]
                    self.add_new_node_only_one(cell_n, up_edge, up_cell_info, up_cell_down_edge)

        # Identify intermediate nodes
        for i, node in self.node_all.items():
            if (len(node['cell_up_set']) == 1 and len(node['cell_down_set']) == 1 and
                    node['cell_up_set'] == node['cell_down_set']):
                inter_node_list.append([i, node['cell_up_set']])

        # Add edges between nodes
        for i in range(len(self.node_all)):
            cell_up_set_i = self.node_all[i]['cell_up_set']
            for cell_i in cell_up_set_i:
                if not self.cell_info[cell_i]['inter_exist']:
                    for j in range(len(self.node_all)):
                        if cell_i in self.node_all[j]['cell_down_set']:
                            self.add_new_edge(i, j)
                else:
                    inter_node_all = self.find_cell_inter_node(inter_node_list, cell_i)
                    for inter_node in inter_node_all:
                        if inter_node != i:
                            self.add_new_edge(i, inter_node)

        for i in range(len(self.node_all)):
            cell_down_set_i = self.node_all[i]['cell_down_set']
            for cell_i in cell_down_set_i:
                if self.cell_info[cell_i]['inter_exist']:
                    inter_node_all = self.find_cell_inter_node(inter_node_list, cell_i)
                    for inter_node in inter_node_all:
                        if inter_node != i:
                            self.add_new_edge(inter_node, i)

        # Add start and end nodes
        self._add_start_end_nodes()

    def adj_net_construct(self):
        """Construct adj network."""
        # Add nodes at cell boundaries
        inter_node_list = []
        for i in range(1, len(self.cell_info) + 1):
            cell_n = self.cell_info[i]
            up_cell = cell_n['up_cell']
            up_edge = [cell_n['left_up'], cell_n['right_up']]
            if cell_n['inter_exist']:
                inter_line = [cell_n['inter_left'], cell_n['inter_right']]
                self.add_new_node_one(cell_n, inter_line, cell_n, inter_line)
                # self.add_new_node_only_one(cell_n, inter_line, cell_n, inter_line)

            if up_cell:
                for cell_id in up_cell:
                    up_cell_info = self.cell_info[cell_id]
                    up_cell_down_edge = [up_cell_info['left_down'], up_cell_info['right_down']]
                    self.add_new_node_one(cell_n, up_edge, up_cell_info, up_cell_down_edge)
                    # self.add_new_node_only_one(cell_n, up_edge, up_cell_info, up_cell_down_edge)

        # Identify intermediate nodes
        for i, node in self.node_all_adj.items():
            if (len(node['cell_up_set']) == 1 and len(node['cell_down_set']) == 1 and
                    node['cell_up_set'] == node['cell_down_set']):
                inter_node_list.append([i, node['cell_up_set']])

        # Add edges between nodes
        for i in range(len(self.node_all_adj)):
            cell_up_set_i = self.node_all_adj[i]['cell_up_set']
            for cell_i in cell_up_set_i:
                if not self.cell_info[cell_i]['inter_exist']:
                    for j in range(len(self.node_all_adj)):
                        if cell_i in self.node_all_adj[j]['cell_down_set']:
                            self.add_new_edge(i, j, True)
                            # self.add_new_edge(i, j)
                else:
                    inter_node_all = self.find_cell_inter_node(inter_node_list, cell_i)
                    for inter_node in inter_node_all:
                        if inter_node != i:
                            self.add_new_edge(i, inter_node, True)
                            # self.add_new_edge(i, inter_node)

        for i in range(len(self.node_all_adj)):
            cell_down_set_i = self.node_all_adj[i]['cell_down_set']
            for cell_i in cell_down_set_i:
                if self.cell_info[cell_i]['inter_exist']:
                    inter_node_all = self.find_cell_inter_node(inter_node_list, cell_i)
                    for inter_node in inter_node_all:
                        if inter_node != i:
                            self.add_new_edge(inter_node, i, True)

        # Add start and end nodes
        self._add_start_end_nodes_adj()


    def construct_flow_network_two(self):
        """Construct a flow network with nodes and edges."""
        # Add nodes
        for i in range(len(self.cell_info)):
            if i in self.delete_cell_list:
                continue
            cell_n = self.cell_info[i]
            up_edge = [cell_n['left_up'], cell_n['right_up']]
            for up_cell_id in cell_n.get('up_node', cell_n.get('up_cell', set())):
                up_cell = self.cell_info[up_cell_id]
                up_cell_down_edge = [up_cell['left_down'], up_cell['right_down']]
                self.add_new_node_one(cell_n, up_edge, up_cell, up_cell_down_edge)

        # Add edges
        for i in range(len(self.node_all)):
            for cell_i in self.node_all[i]['cell_up_set']:
                for j in range(len(self.node_all)):
                    if cell_i in self.node_all[j]['cell_down_set']:
                        capacity_n = self.compute_edge_capacity(i, j) or 0.1
                        dis = self.list_distance(self.node_all[i]['pos'], self.node_all[j]['pos'])
                        self.edge_all[f"{i},{j}"] = {'capacity': capacity_n, 'dis': dis}

        # Add start and end nodes
        self._add_start_end_nodes(use_swarm_positions=True)

    def _add_start_end_nodes_adj(self, use_swarm_positions=False):
        """Add start and end nodes to the network."""
        s_cell_idx, g_cell_idx = 1, self.number_cells
        node_sum = len(self.node_all_adj)
        self.start_idx = self.start_node = node_sum
        self.end_idx = self.end_node = node_sum + 1
        self.node_start_end_adj = {}

        s_pos = self.swarm.start_ave_pos if use_swarm_positions else self.cell_info[s_cell_idx]['pos']
        g_pos = self.swarm.goal_ave_pos if use_swarm_positions else self.cell_info[g_cell_idx]['pos']
        self.node_start_end_adj[node_sum] = {'pos': s_pos, 'name': 'start', 'capacity': self.swarm.robots_num if not use_swarm_positions else 1}
        self.node_start_end_adj[node_sum + 1] = {'pos': g_pos, 'name': 'goal', 'capacity': self.swarm.robots_num if not use_swarm_positions else 1}

        for i in range(len(self.node_all_adj)):
            node_cell_set = self.node_all_adj[i].get('cell_set', set())
            ca = self.node_all_adj[i]['capacity']
            if s_cell_idx in node_cell_set:
                dis = self.list_distance(self.node_all_adj[i]['pos'], self.node_start_end_adj[node_sum]['pos'])
                self.edge_all_adj[f"{node_sum},{i}"] = \
                    {'capacity': ca, 'dis': dis, 'start': node_sum, 'end': i}
            if g_cell_idx in node_cell_set:
                dis = self.list_distance(self.node_all_adj[i]['pos'], self.node_start_end_adj[node_sum + 1]['pos'])
                self.edge_all_adj[f"{i},{node_sum + 1}"] = \
                    {'capacity': ca, 'dis': dis, 'start': i, 'end': node_sum + 1}

    def _add_start_end_nodes(self, use_swarm_positions=False):
        """Add start and end nodes to the network."""
        s_cell_idx, g_cell_idx = 0, self.number_cells - 1
        node_sum = len(self.node_all)
        self.start_idx = self.start_node = node_sum
        self.end_idx = self.end_node = node_sum + 1
        self.node_start_end = {}

        s_pos = self.swarm.start_ave_pos if use_swarm_positions else self.cell_info[s_cell_idx]['pos']
        g_pos = self.swarm.goal_ave_pos if use_swarm_positions else self.cell_info[g_cell_idx]['pos']
        self.node_start_end[node_sum] = {'pos': s_pos, 'name': 'start', 'capacity': self.swarm.robots_num if not use_swarm_positions else 1}
        self.node_start_end[node_sum + 1] = {'pos': g_pos, 'name': 'goal', 'capacity': self.swarm.robots_num if not use_swarm_positions else 1}

        for i in range(len(self.node_all)):
            node_cell_set = self.node_all[i].get('cell_set', set())
            # ca = self.node_all[i]['capacity']
            ca = self.swarm.robots_num
            if s_cell_idx in node_cell_set:
                dis = self.list_distance(self.node_all[i]['pos'], self.node_start_end[node_sum]['pos'])
                self.edge_all[f"{node_sum},{i}"] = {'capacity': ca, 'dis': dis}
            if g_cell_idx in node_cell_set:
                dis = self.list_distance(self.node_all[i]['pos'], self.node_start_end[node_sum + 1]['pos'])
                self.edge_all[f"{i},{node_sum + 1}"] = {'capacity': ca, 'dis': dis}

    def add_new_node_only_one(self, cell_n, cell_n_edge, cell_m, cell_m_edge):
        """Add a single node at the midpoint of overlapping edges."""
        over_p1, over_p2 = self.overlap_segment(cell_n_edge[0], cell_n_edge[1], cell_m_edge[0], cell_m_edge[1])
        if over_p1 is None:
            return
        node_option_pos = self.calculate_discrete_positions(over_p1, over_p2, self.safety_dis)
        if len(node_option_pos) != 0:
            self.create_new_node(cell_n, cell_m, node_option_pos)

    def add_new_node_one(self, cell_n, cell_n_edge, cell_m, cell_m_edge):
        """Add nodes along overlapping edges with partitioning."""
        over_p1, over_p2 = self.overlap_segment(cell_n_edge[0], cell_n_edge[1], cell_m_edge[0], cell_m_edge[1])
        if over_p1 is None:
            return
        node_option_pos = self.calculate_discrete_positions(over_p1, over_p2, self.safety_dis)
        partition = 4
        for i in range(0, len(node_option_pos), partition):
            self.create_new_node(cell_n, cell_m, node_option_pos[i:i + partition], True)

    def create_new_node(self, cell_n, cell_m, node_option_pos, use_adj=False):
        """Create a new node with given position options."""
        node_pos = self.compute_ave_pos(node_option_pos)
        if use_adj:
            target_dict = self.node_all_adj
        else:
            target_dict = self.node_all

        idx = len(target_dict)
        cell_set = {cell_n['idx'], cell_m['idx']}
        cell_up_set = {cell_m['idx']}
        cell_down_set = {cell_n['idx']}
        target_dict[idx] = {
            'pos': node_pos, 'capacity': len(node_option_pos), 'cell_set': cell_set,
            'cell_up_set': cell_up_set, 'cell_down_set': cell_down_set,
            'node_idx': idx, 'node_option_pos': node_option_pos, 'node_option_num': len(node_option_pos)
        }

    def add_new_edge(self, i, j, use_adj=False):
        """Add an edge between two nodes with computed capacity."""
        if use_adj:
            target_dict = self.edge_all_adj
            node_all = self.node_all_adj
        else:
            target_dict = self.edge_all
            node_all = self.node_all

        dis = self.list_distance(node_all[i]['pos'], node_all[j]['pos'])
        capacity_n = self.compute_edge_capacity(i, j, use_adj)
        target_dict[f"{i},{j}"] = {'capacity': capacity_n, 'dis': dis, 'start': i, 'end': j}
        if i in self.node_to_neighborNode:
            self.node_to_neighborNode[i].add(j)
        else:
            self.node_to_neighborNode[i] = set()
            self.node_to_neighborNode[i].add(j)

    def construct_dijkstra(self):
        """Construct Dijkstra's adjacency list from edges."""
        graph_edges = []
        for edge, info in self.edge_all_adj.items():
            node_start, node_end = map(int, edge.split(','))
            dis = round(info['dis'], 2)
            graph_edges.append([node_start, node_end, dis])
            # graph_edges.append([node_end, node_start, dis])
        self.dijkstra.init_node_edges(graph_edges)

    # --- Pathfinding Methods ---

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

    def _get_dijkstra_path(self, start, end):
        """Helper to get cached or compute Dijkstra path."""
        name = (start, end)
        if name not in self.dijkstra_result:
            path, dis = self.dijkstra.shortest_path(start, end)
            self.dijkstra_result[name] = {'path': list(path), 'dis': dis}
        return self.dijkstra_result[name]['path'], self.dijkstra_result[name]['dis']

    def find_node_to_path_dict(self):
        """Compute paths from all nodes to goal using JPS."""
        for key, node in self.node_all.items():
            node_pos = [int(p * self.resolution) for p in node['pos']]
            goal_pos = [int(p * self.resolution) for p in self.node_start_end[self.end_node]['pos']]
            path = self.JPS_search(node_pos, goal_pos)
            self.jps_path_all[key] = path
            cell_idx_set = self.find_pass_cells(path)
            node_list = self.find_pass_node(cell_idx_set, key) + [self.end_node]
            dis = self.calculate_path_length(path) / self.resolution
            self.node_to_goal[key] = {'path_node': node_list, 'distance': dis}

    def JPS_search(self, start, goal):
        """Perform JPS pathfinding."""
        return self.jps.method(self.dilated_map, start, goal, 1)

    def Astar_search(self, start, goal):
        """Perform A* pathfinding."""
        t1 = time.time()
        path = self.Astar.method(self.dilated_map, start, goal, 1)
        logging.info(f"Astar time: {time.time() - t1}s")
        return path

    # --- Utility Methods ---

    def list_distance(self, l1, l2):
        """Calculate Euclidean distance between two points."""
        return np.linalg.norm(np.array(l1) - np.array(l2))

    def calculate_discrete_positions(self, p1, p2, step):
        """Generate discrete points between two positions."""
        distance = self.list_distance(p1, p2)
        positions_count = int(distance // step)
        return [tuple(p1[i] + j * step / distance * (p2[i] - p1[i])
                      for i in range(2)) for j in range(1, positions_count)]

    # def compute_edge_capacity(self, i, j, use_adj=False):
    #     """Compute edge capacity based on geometry."""
    #     if use_adj is True:
    #         node_all = self.node_all_adj
    #     else:
    #         node_all = self.node_all
    #
    #     pos_i, pos_j = node_all[i]['pos'], node_all[j]['pos']
    #     ca_min = min(node_all[i]['capacity'], node_all[j]['capacity'])
    #     capacity_edge = ca_min
    #     return capacity_edge

    def compute_edge_capacity(self, i, j, use_adj=False):
        """Compute edge capacity based on geometry."""
        if use_adj is True:
            node_all = self.node_all_adj
        else:
            node_all = self.node_all

        pos_i, pos_j = node_all[i]['pos'], node_all[j]['pos']
        ca_min = min(node_all[i]['capacity'], node_all[j]['capacity'])
        dis = self.list_distance(pos_i, pos_j)
        angle = math.atan2(pos_j[1] - pos_i[1], pos_j[0] - pos_i[0])
        # if math.isnan(angle) or math.isnan(dis) or math.isnan(self.safety_dis):
        #     print(angle)
        #     print(dis)
        #     print("输入值包含 NaN，请检查输入。")
        #     return 1  # 可以根据实际情况返回默认值
        capacity_edge = max(int(math.sin(angle) * dis / (2 * self.safety_dis) * ca_min), 1)
        if math.isnan(capacity_edge):
            print("输入值包含 NaN，请检查输入。")
        return capacity_edge

    def overlap_segment(self, n_p1, n_p2, m_p1, m_p2):
        """Find overlapping segment between two horizontal lines."""
        x1, x2 = sorted([n_p1[0], n_p2[0]])
        x3, x4 = sorted([m_p1[0], m_p2[0]])
        start, end = max(x1, x3), min(x2, x4)
        return ([start, n_p1[1]], [end, n_p1[1]]) if start <= end else (None, None)

    def compute_ave_pos(self, positions):
        """Compute average position from a list of positions."""
        return tuple(np.mean([p[i] for p in positions]) for i in range(2))

    def find_cell_inter_node(self, inter_node_list, cell):
        """Find intermediate node for a given cell."""
        node_li = []
        for node_id, cell_set in inter_node_list:
            if cell in cell_set:
                node_li.append(node_id)
        return node_li

    def update_para(self):
        """Update swarm parameters."""
        self.swarm.current_node = np.ones((self.swarm.robots_num, 2), dtype=int) * self.start_node

    # --- Visualization Methods ---

    def draw_network_adj(self):
        """Draw the network graph."""
        G = nx.Graph()
        for node, data in {**self.node_all_adj, **self.node_start_end_adj}.items():
            G.add_node(node, pos=data['pos'])
        for edge, info in self.edge_all_adj.items():
            node_start, node_end = map(int, edge.split(','))
            G.add_edge(node_start, node_end, capacity=info['capacity'])

        # 创建图形和轴对象
        fig, ax = plt.subplots(figsize=(8, 6))
        # 添加圆形
        for i in range(len(self.node_all_adj)):
            node_pos_set = self.node_all_adj[i]['node_option_pos']
            for j in range(len(node_pos_set)):
                pos = node_pos_set[j]
                radius = self.swarm.robot_radius
                circle = Circle((pos[0], pos[1]), radius, fill=False, edgecolor='r', linewidth=2)
                ax.add_patch(circle)

        pos = nx.get_node_attributes(G, 'pos')
        nx.draw(G, pos, ax=ax, with_labels=True, node_color='skyblue', node_size=30, edge_color='k', width=0.2,
                style='dashed')
        ax.axis('equal')
        plt.show()

    def draw_network_cell(self):
        """Draw the network graph."""
        G = nx.Graph()
        for node, data in {**self.node_all, **self.node_start_end}.items():
            G.add_node(node, pos=data['pos'])
        for edge, info in self.edge_all.items():
            node_start, node_end = map(int, edge.split(','))
            G.add_edge(node_start, node_end, capacity=info['capacity'])

        # 创建图形和轴对象
        fig, ax = plt.subplots(figsize=(8, 6))
        # 添加圆形
        # for i in range(len(self.node_all)):
        #     node_pos_set = self.node_all[i]['node_option_pos']
        #     for j in range(len(node_pos_set)):
        #         pos = node_pos_set[j]
        #         radius = self.swarm.robot_radius
        #         circle = Circle((pos[0], pos[1]), radius, fill=False, edgecolor='r', linewidth=2)
        #         ax.add_patch(circle)

        pos = nx.get_node_attributes(G, 'pos')
        for node in G.nodes():
            if node not in pos:
                print(node)
        nx.draw(G, pos, ax=ax, with_labels=True, node_color='skyblue', node_size=30, edge_color='k', width=0.2,
                style='dashed')
        ax.axis('equal')
        plt.show()

    def draw_separate_cell(self, separate_map, cells_number):
        """Display decomposed cells with nodes."""
        # separate_map = self.bcd_out_im
        # cells = self.bcd_out_cells
        fig, ax = plt.subplots()
        display_img = np.random.randint(0, 255, (*separate_map.shape, 3), dtype=np.uint8)
        for cell_id in range(cells_number + 1):
            display_img[separate_map == cell_id] = np.random.randint(0, 255, 3)
        display_img[separate_map == 0] = [0, 0, 0]

        for idx, cell in self.cell_info.items():
            pos = cell['pos']
            value = cell['idx']
            name = 'Ce' + str(value)
            ax.text(pos[0] * self.resolution, pos[1] * self.resolution, name, ha='center', va='center')

        for cell in self.cell_info.values():
            if cell['inter_exist']:
                ax.plot([cell['inter_left'][0], cell['inter_right'][0]],
                        [cell['inter_left'][1], cell['inter_right'][1]], 'r--')

        # for node in self.node_all.values():
        #     ax.add_patch(Circle(node['pos'], 1, color='r', fill=False))

        ax.imshow(np.flipud(np.rot90(display_img)))
        ax.invert_yaxis()
        plt.show()

    def draw_separate_cell_area(self, separate_map, cells_number):
        """Display decomposed cells with nodes."""
        # separate_map = self.bcd_out_im
        # cells = self.bcd_out_cells
        fig, ax = plt.subplots()
        display_img = np.random.randint(0, 255, (*separate_map.shape, 3), dtype=np.uint8)
        for cell_id in range(cells_number + 1):
            display_img[separate_map == cell_id] = np.random.randint(0, 255, 3)
        display_img[separate_map == 0] = [0, 0, 0]

        for idx, area_n in self.area_dict.items():
            sum_x = 0
            sum_y = 0
            for sub_list in area_n:
                sum_x += sub_list[0]
                sum_y += sub_list[1]
            mean_x = sum_x / len(area_n)
            mean_y = sum_y / len(area_n)
            pos = [mean_x, mean_y]
            value = self.area_map[area_n[0][0], area_n[0][1]]
            name = "r" + str(value)
            ax.text(pos[0] * self.resolution, pos[1] * self.resolution, name, ha='center', va='center')

        for cell in self.cell_info.values():
            if cell['inter_exist']:
                ax.plot([cell['inter_left'][0], cell['inter_right'][0]],
                        [cell['inter_left'][1], cell['inter_right'][1]], 'r--')

        # for node in self.node_all.values():
        #     ax.add_patch(Circle(node['pos'], 1, color='r', fill=False))

        ax.imshow(np.flipud(np.rot90(display_img)))
        ax.invert_yaxis()
        plt.show()

    def draw_mapInfo(self):
        self.draw_separate_cell(self.area_map, len(self.area_dict))
        self.draw_separate_cell_area(self.area_map, len(self.area_dict))
        # self.draw_network_cell()
        self.draw_network_adj()

    def get_path_info(self, path_node, path_length):
        # 通过进来的path node 和 path length
        # 计算出别的信息 并返回一个dict
        pass

    def find_current_path_set_cell_area(self):
        # 以当前无人机所在的cell为基础 cell上边界的所有节点为起点
        # 到目标点的邻居节点的所有path 作为path set之后连接起点和终点 作为最后的path set
        # 统计全新的信息
        # path node, robot_idx, start_node, end_node
        # area 在每个时间段 经过了哪些区域
        # 这里定义 时间范围为 假如为5 那么给出长度为5的区域list [r1, r2, r3, r4, r5]
        path_all = {}
        for i in range(self.swarm.robots_num):
            current_cell = self.swarm.current_cell[i]
            # 当前cell的上边界 node
            if current_cell == self.swarm.goal_cell[i]:
                p1 = self.swarm.goal_node[i]
                p2 = self.swarm.goal_node[i]
                p3 = self.swarm.goal_node[i]
                first_edge = str(p1) + ',' + str(p2)
                second_edge = str(p2) + ',' + str(p3)
                dic = {'path_node': [p1], 'path_length': 1,
                       'start_node': p1, 'end_node': p1, 'robot_idx': i,
                       'first_edge': first_edge, 'second_edge': second_edge}
                idx = len(path_all)
                path_all[idx] = dic
                continue

            node_list = self.find_cell_up_node(current_cell)

            # 假如当前没有的cell没有上邻居节点了 保持上一次的des node不变
            if len(node_list) == 0:
                # logging.info("node list is [] !")
                path_node = copy.copy(self.swarm.des_path_node[i])
                s_i = copy.copy(self.swarm.previous_node[i])
                g_i = copy.copy(self.swarm.goal_node[i])

                if len(path_node) >= 3:
                    p1 = path_node[0]
                    p2 = path_node[1]
                    p3 = path_node[2]
                    first_edge = str(p1) + ',' + str(p2)
                    second_edge = str(p2) + ',' + str(p3)
                    dic = {'path_node': path_node, 'path_length': 1,
                           'start_node': s_i, 'end_node': g_i, 'robot_idx': i,
                           'first_edge': first_edge, 'second_edge': second_edge}
                    idx = len(path_all)
                    path_all[idx] = dic
                elif len(path_node) == 2:
                    p1 = path_node[0]
                    p2 = path_node[1]
                    first_edge = str(p1) + ',' + str(p2)
                    second_edge = str(p1) + ',' + str(p2)
                    dic = {'path_node': path_node, 'path_length': 1,
                           'start_node': s_i, 'end_node': g_i, 'robot_idx': i,
                           'first_edge': first_edge, 'second_edge': second_edge}
                    idx = len(path_all)
                    path_all[idx] = dic

                continue

            goal_neighbor_node_list = self.find_cell_down_node(self.swarm.goal_cell[i])
            for cur_ner_node in node_list:
                for goal_ner_node in goal_neighbor_node_list:
                    s_i = cur_ner_node
                    mid_i = goal_ner_node
                    g_i = self.swarm.goal_node[i]
                    path_node, path_length = self.find_path_mid(s_i, g_i, mid_i)
                    path_length = self.revise_path_length_cell(i, g_i, path_node, path_length)

                    if math.isnan(path_length) or math.isinf(path_length):
                        path_length = float('inf')

                    if len(path_node) >= 3:
                        p1 = path_node[0]
                        p2 = path_node[1]
                        p3 = path_node[2]
                        first_edge = str(p1) + ',' + str(p2)
                        second_edge = str(p2) + ',' + str(p3)
                        dic = {'path_node': path_node, 'path_length': path_length,
                               'start_node': s_i, 'end_node': g_i, 'robot_idx': i,
                               'first_edge': first_edge, 'second_edge': second_edge}
                        idx = len(path_all)
                        path_all[idx] = dic
                    elif len(path_node) == 2:
                        p1 = path_node[0]
                        p2 = path_node[1]
                        first_edge = str(p1) + ',' + str(p2)
                        second_edge = str(p1) + ',' + str(p2)
                        dic = {'path_node': path_node, 'path_length': path_length,
                               'start_node': s_i, 'end_node': g_i, 'robot_idx': i,
                               'first_edge': first_edge, 'second_edge': second_edge}
                        idx = len(path_all)
                        path_all[idx] = dic
                    else:
                        pass

        self.path_all = path_all
        # logging.info("path_all_len=" + str(len(self.path_all)))

    def find_current_path_set_cell_neighbor(self):
        # 以当前无人机所在的cell为基础 cell上边界的所有节点为起点
        # 到目标点的邻居节点的所有path 作为path set之后连接起点和终点 作为最后的path set

        # 一个新的版本 找到path的一边 二边
        path_all = {}
        for i in range(self.swarm.robots_num):
            current_cell = self.swarm.current_cell[i]
            # 当前cell的上边界 node
            if current_cell == self.swarm.goal_cell[i]:
                p1 = self.swarm.goal_node[i]
                p2 = self.swarm.goal_node[i]
                p3 = self.swarm.goal_node[i]
                first_edge = str(p1) + ',' + str(p2)
                second_edge = str(p2) + ',' + str(p3)
                dic = {'path_node': [p1], 'path_length': 1,
                       'start_node': p1, 'end_node': p1, 'robot_idx': i,
                       'first_edge': first_edge, 'second_edge': second_edge}
                idx = len(path_all)
                path_all[idx] = dic
                continue

            node_list = self.find_cell_up_node(current_cell)

            # 假如当前没有的cell没有上邻居节点了 保持上一次的des node不变
            if len(node_list) == 0:
                # logging.info("node list is [] !")
                path_node = copy.copy(self.swarm.des_path_node[i])
                s_i = copy.copy(self.swarm.previous_node[i])
                g_i = copy.copy(self.swarm.goal_node[i])

                if len(path_node) >= 3:
                    p1 = path_node[0]
                    p2 = path_node[1]
                    p3 = path_node[2]
                    first_edge = str(p1) + ',' + str(p2)
                    second_edge = str(p2) + ',' + str(p3)
                    dic = {'path_node': path_node, 'path_length': 1,
                           'start_node': s_i, 'end_node': g_i, 'robot_idx': i,
                           'first_edge': first_edge, 'second_edge': second_edge}
                    idx = len(path_all)
                    path_all[idx] = dic
                elif len(path_node) == 2:
                    p1 = path_node[0]
                    p2 = path_node[1]
                    first_edge = str(p1) + ',' + str(p2)
                    second_edge = str(p1) + ',' + str(p2)
                    dic = {'path_node': path_node, 'path_length': 1,
                           'start_node': s_i, 'end_node': g_i, 'robot_idx': i,
                           'first_edge': first_edge, 'second_edge': second_edge}
                    idx = len(path_all)
                    path_all[idx] = dic

                continue

            goal_neighbor_node_list = self.find_cell_down_node(self.swarm.goal_cell[i])
            for cur_ner_node in node_list:
                for goal_ner_node in goal_neighbor_node_list:
                    s_i = cur_ner_node
                    mid_i = goal_ner_node
                    g_i = self.swarm.goal_node[i]
                    path_node, path_length = self.find_path_mid(s_i, g_i, mid_i)
                    path_length = self.revise_path_length_cell(i, g_i, path_node, path_length)

                    if math.isnan(path_length) or math.isinf(path_length):
                        path_length = float('inf')

                    if len(path_node) >= 3:
                        p1 = path_node[0]
                        p2 = path_node[1]
                        p3 = path_node[2]
                        first_edge = str(p1) + ',' + str(p2)
                        second_edge = str(p2) + ',' + str(p3)
                        dic = {'path_node': path_node, 'path_length': path_length,
                               'start_node': s_i, 'end_node': g_i, 'robot_idx': i,
                               'first_edge': first_edge, 'second_edge': second_edge}
                        idx = len(path_all)
                        path_all[idx] = dic
                    elif len(path_node) == 2:
                        p1 = path_node[0]
                        p2 = path_node[1]
                        first_edge = str(p1) + ',' + str(p2)
                        second_edge = str(p1) + ',' + str(p2)
                        dic = {'path_node': path_node, 'path_length': path_length,
                               'start_node': s_i, 'end_node': g_i, 'robot_idx': i,
                               'first_edge': first_edge, 'second_edge': second_edge}
                        idx = len(path_all)
                        path_all[idx] = dic
                    else:
                        pass

        self.path_all = path_all
        # logging.info("path_all_len=" + str(len(self.path_all)))
        return path_all
        # 这些path的下一个点 有哪些path的下一个点是相同的 找到这些相同点的path

    def find_current_path_set_purning(self):
        # 以当前无人机所在的cell为基础 cell上边界的所有节点为起点
        # 到目标点的邻居节点的所有path 作为path set之后连接起点和终点 作为最后的path set
        # 在前一个版本的基础上 进行剪枝、
        # 计算距离 选择最近的几个 作为有潜力节点

        path_all = {}
        for i in range(self.swarm.robots_num):
            current_cell = self.swarm.current_cell[i]
            # 当前cell的上边界 node
            if current_cell == self.swarm.goal_cell[i]:
                p1 = self.swarm.goal_node[i]
                dic = {'path_node': [p1], 'path_length': 1,
                       'start_node': p1, 'end_node': p1, 'robot_idx': i}
                idx = len(path_all)
                path_all[idx] = dic
                continue

            node_list = self.find_cell_up_node(current_cell)
            nearest_nodes_up, remaining_nodes_up = self.find_potential_node(node_list, i, use_pos_n=True)

            # 假如当前没有的cell没有上邻居节点了 保持上一次的des node不变
            if len(node_list) == 0:
                # logging.info("node list is [] !")
                path_node = copy.copy(self.swarm.des_path_node[i])
                s_i = copy.copy(self.swarm.previous_node[i])
                g_i = copy.copy(self.swarm.goal_node[i])

                if len(path_node) >= 3:
                    dic = {'path_node': path_node, 'path_length': 1,
                           'start_node': s_i, 'end_node': g_i, 'robot_idx': i}
                    idx = len(path_all)
                    path_all[idx] = dic
                elif len(path_node) == 2:
                    dic = {'path_node': path_node, 'path_length': 1,
                           'start_node': s_i, 'end_node': g_i, 'robot_idx': i}
                    idx = len(path_all)
                    path_all[idx] = dic
                continue

            goal_neighbor_node_list = self.find_cell_down_node(self.swarm.goal_cell[i])
            nearest_nodes_down, remaining_nodes_down = \
                self.find_potential_node(goal_neighbor_node_list, i, use_pos_n=False)

            # 上节点中重点节点到目标节点所有下节点连线
            for cur_ner_node in nearest_nodes_up:
                for goal_ner_node in goal_neighbor_node_list:
                    s_i = cur_ner_node
                    mid_i = goal_ner_node
                    g_i = self.swarm.goal_node[i]
                    path_node, path_length = self.find_path_mid(s_i, g_i, mid_i)
                    path_length = self.revise_path_length_cell(i, g_i, path_node, path_length)
                    if math.isnan(path_length) or math.isinf(path_length):
                        path_length = float('inf')

                    if len(path_node) >= 3:
                        dic = {'path_node': path_node, 'path_length': path_length,
                               'start_node': s_i, 'end_node': g_i, 'robot_idx': i}
                        idx = len(path_all)
                        path_all[idx] = dic
                    elif len(path_node) == 2:
                        dic = {'path_node': path_node, 'path_length': path_length,
                               'start_node': s_i, 'end_node': g_i, 'robot_idx': i}
                        idx = len(path_all)
                        path_all[idx] = dic
                    else:
                        pass

            # 上节点中剩余的节点只搜索到目标cell下节点中重要节点的path
            for cur_ner_node in remaining_nodes_up:
                for goal_ner_node in nearest_nodes_down:
                    s_i = cur_ner_node
                    mid_i = goal_ner_node
                    g_i = self.swarm.goal_node[i]
                    path_node, path_length = self.find_path_mid(s_i, g_i, mid_i)
                    path_length = self.revise_path_length_cell(i, g_i, path_node, path_length)
                    if math.isnan(path_length) or math.isinf(path_length):
                        path_length = float('inf')

                    if len(path_node) >= 3:
                        dic = {'path_node': path_node, 'path_length': path_length,
                               'start_node': s_i, 'end_node': g_i, 'robot_idx': i}
                        idx = len(path_all)
                        path_all[idx] = dic
                    elif len(path_node) == 2:
                        dic = {'path_node': path_node, 'path_length': path_length,
                               'start_node': s_i, 'end_node': g_i, 'robot_idx': i}
                        idx = len(path_all)
                        path_all[idx] = dic
                    else:
                        pass

        self.path_all = path_all
        # logging.info("path_all_len=" + str(len(self.path_all)))
        return path_all
        # 这些path的下一个点 有哪些path的下一个点是相同的 找到这些相同点的path

    def deal_with_path(self, path_node, path_length, node_i, node_j, node_k):
        new_path_node = [node_i, node_j] + path_node
        edge_ij = str(node_i) + ',' + str(node_j)
        edge_jk = str(node_j) + ',' + str(node_k)
        path_length += self.edge_all_adj[edge_ij]['dis'] + self.edge_all_adj[edge_jk]['dis']

        return new_path_node, path_length

    def local_path_search(self, robot_idx, path_node):
        path_list = []
        current_cell = self.swarm.current_cell[robot_idx]
        if len(path_node) <= 4:
            return path_list
        last_node = path_node[-2]
        node_list_up = self.find_cell_up_node(current_cell)
        # 当前cell的上边界 node A 和所有与上边界node相邻的node B 进行组合
        # 当前goal path node的倒数第二个node C
        for node_i in node_list_up:
            nei_set_i = self.node_to_neighborNode[node_i]
            for node_j in nei_set_i:
                nei_set_j = self.node_to_neighborNode[node_j]
                for node_k in nei_set_j:
                    s_i = node_k
                    mid_i = last_node
                    g_i = self.swarm.goal_node[robot_idx]
                    # 找到node B 到 node C 之前的路径 使用dijkstra算法
                    path_node, path_length = self.find_path_mid(s_i, g_i, mid_i)
                    path_length = self.revise_path_length_cell(robot_idx, g_i, path_node, path_length)
                    # 连接A-B-C-goal 作为路径加入 备选path set
                    new_path_node, path_length = self.deal_with_path(path_node, path_length, node_i, node_j, node_k)
                    path_list.append([new_path_node, path_length])
        return path_list

    # def local_path_search(self, robot_idx, path_node):
    #     path_list = []
    #     current_cell = self.swarm.current_cell[robot_idx]
    #     if len(path_node) <= 4:
    #         return path_list
    #     last_node = path_node[-2]
    #     node_list_up = self.find_cell_up_node(current_cell)
    #     # 当前cell的上边界 node A 和所有与上边界node相邻的node B 进行组合
    #     # 当前goal path node的倒数第二个node C
    #     for node_i in node_list_up:
    #         nei_set_i = self.node_to_neighborNode[node_i]
    #         for node_j in nei_set_i:
    #             nei_set_j = self.node_to_neighborNode[node_j]
    #             for node_k in nei_set_j:
    #                 s_i = node_k
    #                 mid_i = last_node
    #                 g_i = self.swarm.goal_node[robot_idx]
    #                 # 找到node B 到 node C 之前的路径 使用dijkstra算法
    #                 path_node, path_length = self.find_path_mid(s_i, g_i, mid_i)
    #                 path_length = self.revise_path_length_cell(robot_idx, g_i, path_node, path_length)
    #                 # 连接A-B-C-goal 作为路径加入 备选path set
    #                 new_path_node, path_length = self.deal_with_path(path_node, path_length, node_i, node_j, node_k)
    #                 path_list.append([new_path_node, path_length])
    #     return path_list


    def find_potential_node(self, node_list, robot_idx, use_pos_n):
        if use_pos_n is True:
            pos_n = self.swarm.pos_all[robot_idx]
        else:
            pos_n = self.swarm.goal_pos[robot_idx]

        distance_dict = {}
        for node_n in node_list:
            node_pos = self.node_all_adj[node_n]['pos']
            dis = self.list_distance(node_pos, pos_n)
            distance_dict[node_n] = dis
        sorted_nodes = sorted(distance_dict.items(), key=lambda item: item[1])
        nearest_num = 5
        nearest_nodes = [node for node, _ in sorted_nodes[:nearest_num]]
        remaining_nodes = [node for node, _ in sorted_nodes[nearest_num:]]
        return nearest_nodes, remaining_nodes

    def find_cell_down_node(self, cell_idx):
        node_list = []
        for key, node in self.node_all_adj.items():
            if cell_idx in node['cell_up_set']:
                node_list.append(copy.copy(key))
        return node_list

    def find_cell_up_node(self, cell_idx):
        node_list = []
        for key, node in self.node_all_adj.items():
            if cell_idx in node['cell_down_set']:
                node_list.append(copy.copy(key))
        return node_list

    def revise_path_length_cell(self, i, g_i, path_node, path_length):
        # 修正对于每个飞机的路径长度
        if len(path_node) >= 2:
            # 路径的第一个node
            first_node = path_node[0]
            # 倒数第二个node
            second_to_last_node = path_node[-2]
            # 计算倒数第二个node 到 目标node之间的距离
            dis2 = self.node_dis_dij(second_to_last_node, g_i)

            # 当前的pos 和 目标pos
            current_pos = self.swarm.pos_all[i]
            goal_pos = self.swarm.goal_pos[i]

            # current_pos = [x * self.resolution for x in current_pos]
            # goal_pos = [x * self.resolution for x in goal_pos]
            # 第一个node的位置
            first_pos = self.node_all_adj[first_node]['pos']
            # 倒数第二个node pos
            second_last_pos = self.node_all_adj[second_to_last_node]['pos']

            # 当前pos 和第一个node 之间距离
            dis1_n = self.swarm.distance(current_pos, first_pos)
            # 目标pos 和 倒数第二个node之间的距离
            dis2_n = self.swarm.distance(goal_pos, second_last_pos)

            # 减去node之间距离 修正真实的距离
            path_length = path_length - dis2 + dis1_n + dis2_n
            return path_length
        else:
            return path_length

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

    def find_path_length(self, path_node_list):
        path_len = 0
        capacity_sum = 0
        for i in range(len(path_node_list) - 1):
            node1 = path_node_list[i]
            node2 = path_node_list[i+1]
            edge = str(node1) + "," + str(node2)
            if edge in self.edge_all_adj:
                length = self.edge_all_adj[edge]['dis']
                capa = self.edge_all_adj[edge]['capacity']
                capacity_sum += capa
                path_len += length
        capacity_ave = capacity_sum/(len(path_node_list) + 1)
        return path_len, capacity_ave

    def node_dis_dij(self, node1, node2):
        node_set = self.dijkstra.adjacency_list[node1]
        for n in node_set:
            if n[0] == node2:
                dis = n[1]
                return dis

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

        # 按照边 依次加入对应有这些边的path id
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