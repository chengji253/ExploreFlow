import math
import time
import copy
import networkx as nx
import matplotlib.pyplot as plt
from collections import defaultdict
from collections import Counter
from map_info import map_info
import os
import heapq
from collections import defaultdict
from itertools import product
import random


class Max_Flow_planner:

    def __init__(self, swarm, mapInfo):

        self.swarm = swarm
        self.mapInfo = mapInfo

        self.cell_net = None

        self.adj_net = None

        self.v_net = self.swarm.v_max[0]*0.8

        self.robot_targets = None

        self.paths = None
        self.robots_and_path = None
        self.speed = None  # 统一速度
        self.alpha = None  # 拥堵系数
        self.path_loads = None  # 路径上的机器人数量
        self.path_times = None  # 路径完成时间
        self.assignments = None
        self.robot_positions = None

        self.cell_node_list = None
        self.path_choice_list = []
        self.path_idx_info = {}

        self.beta = 3

        # t1 = time.time()
        # self.cell_net_G()
        # t2 = time.time()
        # # self.run_example_one()
        # t3 = time.time()

        # self.load_balance_example()

        # print("self.cell_net_G time = " + str(t2 - t1))
        # print("t3 - t2=" + str(t3 - t2))

    def run(self):
        # 机器人所在的cell  计算机器人到当前cell的上边界cell node的距离 构建一个小节点
        # 这个小节点就是 机器人所在的节点
        # self.set_robot_start_node()
        self.set_robot_goal_node()
        # # 构建时间扩展网络 G 在这个G上去使用最大流算法 得到每个机器人的path
        # 这个里面是的path是cell node
        self.time_window = 100
        self.cell_net_build()

        t1 = time.time()
        # schedule = self.ford_fulkerson_schedule()
        # self.cell_node_list = self.schedule_to_path(schedule)
        # self.plot_network_with_paths(self.cell_net, schedule)
        # path_set_one = self.build_path_node_list_max_flow()
        path_set_one = []
        t2 = time.time()
        # 根据cell node去 选择 path node
        # p1_set = 在cell node里面的每个path node 都选择上
        # p2_set = 然后当前cell 的上边界 path node 到目标cell 下边界 path node
        path_set_two = self.build_path_node_list_dijkstra()
        t3 = time.time()

        # 将 path node 转换为 path pos
        self.merge_path_set(path_set_one, path_set_two)
        # 输出path pos结果
        t4 = time.time()
        # 使用load balance 算法 在这个里面选择
        des_path_node = self.path_select_main()
        t5 = time.time()
        # 将得到的路径结果输出
        self.swarm.des_path_node = des_path_node
        self.set_center_pos()
        self.position_allocation_greedy_now()
        self.swarm.para_set_planner()
        t6= time.time()

        print("path set one=" + str(t2 - t1))
        print("path set two=" + str(t3 - t2))
        print("merge=" + str(t4 - t3))
        print("path select=" + str(t5 - t4))
        print("greedy=" + str(t6 - t5))

    def set_robot_goal_node(self):
        # 设置机器人的cell net中目标节点
        for idx, node in self.mapInfo.node_start_end.items():
            if node['name'] == 'goal':
                self.robot_targets = idx

    def cell_net_build(self):
        # 将mapInfo中的节点 边信息处理后 转换为networkx
        global id_1_pos, id_2_pos
        self.cell_net = nx.DiGraph()

        for edge_id, edge in self.mapInfo.edge_all.items():
            numbers = edge_id.split(',')
            id_1 = int(numbers[0])
            id_2 = int(numbers[1])

            if id_1 in self.mapInfo.node_all:
                id_1_pos = self.mapInfo.node_all[id_1]['pos']
            if id_2 in self.mapInfo.node_all:
                id_2_pos = self.mapInfo.node_all[id_2]['pos']
            if id_1 in self.mapInfo.node_start_end:
                id_1_pos = self.mapInfo.node_start_end[id_1]['pos']
            if id_2 in self.mapInfo.node_start_end:
                id_2_pos = self.mapInfo.node_start_end[id_2]['pos']

            if id_1 not in self.cell_net.nodes:
                self.cell_net.add_node(id_1, pos=id_1_pos)
            if id_2 not in self.cell_net.nodes:
                self.cell_net.add_node(id_2, pos=id_2_pos)

            capacity = edge['capacity']
            dis = edge['dis']
            time = math.ceil(dis/self.v_net)
            self.cell_net.add_edge(id_1, id_2, travel_time=time, capacity=capacity)

            # 添加节点位置信息

        # self.plot_cell_net()
        # print()
    def schedule_to_path(self, schedule):
        cell_node_list = []
        print("len schedule=" + str(len(schedule)))
        for i in range(self.swarm.robots_num):
            if 0 <= i < len(schedule):
                s_n = schedule[i]
                path_n = []
                for tup in s_n:
                    node_1 = tup[0][0]
                    node_2 = tup[1][0]
                    if len(path_n) == 0:
                        if node_1 != node_2:
                            path_n.append(node_1)
                            path_n.append(node_2)
                        else:
                            path_n.append(node_1)
                    else:
                        if path_n[-1] != node_1:
                            path_n.append(node_1)
                        if path_n[-1] != node_2:
                            path_n.append(node_2)
            else:
                path_n = []
            cell_node_list.append(path_n)
        return cell_node_list

    def plot_cell_net(self):
        if self.cell_net is None:
            print("请先调用 cell_net_G 方法构建网络。")
            return

        # 绘制图形
        pos = {node: data['pos'] for node, data in self.cell_net.nodes(data=True) if 'pos' in data}

        nx.draw_networkx_nodes(self.cell_net, pos)  # 绘制节点
        nx.draw_networkx_edges(self.cell_net, pos)  # 绘制边
        nx.draw_networkx_labels(self.cell_net, pos)  # 绘制节点标签

        # 绘制边的属性（例如 travel_time 和 capacity）
        # edge_labels = {(u, v): f"t=: {d['travel_time']}, c=: {d['capacity']}" for u, v, d in
        #                self.cell_net.edges(data=True)}
        # nx.draw_networkx_edge_labels(self.cell_net, pos, edge_labels=edge_labels)

        plt.title("Cell Network")
        plt.axis('off')  # 关闭坐标轴
        plt.show()

    def set_robot_start_node(self):
        # self.swarm.current_cell
        # 当前cell所在的上边界的cell node
        robot_positions = []
        for i in range(self.swarm.robots_num):
            cell_n = self.swarm.current_cell[i]
            pos_i = self.swarm.pos_all[i]
            up_cell_node_set = self.mapInfo.cellId_to_up_cellNode[cell_n]

            if len(up_cell_node_set) == 0:
                print("len(up_cell_node_set) == 0 error !")
                up_cell_node = None
            elif len(up_cell_node_set) == 1:
                up_cell_node = list[up_cell_node_set][0]
            else:
                up_cell_node = self.choose_cell_node(up_cell_node_set, pos_i)

            cell_pos = self.mapInfo.node_all[up_cell_node]['pos']
            dis_remain = self.mapInfo.list_distance(cell_pos, pos_i)
            # 根据这个计算到节点的剩余时间
            time_remain = math.ceil(dis_remain/self.v_net)
            # 如果剩余的时间小于1 那么就认为在这个node上
            # 不然就认为剩余的时间 还有多少也在这个node上
            if time_remain <= 1:
                robot_positions.append((up_cell_node, 0))
            else:
                robot_positions.append((up_cell_node, time_remain))
        self.robot_positions = robot_positions
        # print(self.robot_positions)
        # self.print_one()

    def print_one(self):
        count_dict = {}
        # 遍历列表中的每个元组
        for tup in self.robot_positions:
            first_element = tup[0]
            # 如果该元素已经在字典中，将其对应的值加 1
            if first_element in count_dict:
                count_dict[first_element] += 1
            # 若不在字典中，初始化为 1
            else:
                count_dict[first_element] = 1

        # 按照键的升序排序，将统计结果存储在列表中
        count_list = [count_dict[key] for key in sorted(count_dict.keys())]
        print(count_dict)

    def choose_cell_node(self, up_cell_node_set, pos_i):
        dis_min = float('inf')
        node_min = None
        for up_node in up_cell_node_set:
            pos_up = self.mapInfo.node_all[up_node]['pos']
            dis_n = self.mapInfo.list_distance(pos_up, pos_i)
            if dis_n <= dis_min:
                node_min = up_node
                dis_min = dis_n
        return node_min

    def build_path_node_list_max_flow(self):
        # 根据得到的cell node list 建立 path node list
        # 建立的思路 在 cell node中每个路径都进去 为set 1 (也许每次考虑前几个 更好？ 而不是全部考虑)
        path_node_all = []
        for i in range(self.swarm.robots_num):
            cell_node_path = self.cell_node_list[i]
            path_node_path = []
            for cell_node_n in cell_node_path:
                path_node_n = self.mapInfo.cellNode_to_pathNode[cell_node_n]
                if isinstance(path_node_n, set):
                    path_node_path.append(path_node_n)
                else:
                    path_node_path.append({path_node_n})

            # 只取前面 5 个 set 进行组合，如果少于 5 个就取全部
            front_num = 5
            num_sets_to_combine = min(front_num, len(path_node_path))
            sets_to_combine = path_node_path[:num_sets_to_combine]

            # 对前面的 set 进行组合
            combinations = product(*sets_to_combine)

            # 处理后面的 set，每个随机选一个元素
            remaining_sets = path_node_path[num_sets_to_combine:]
            result = []
            for combination in combinations:
                current_combination = list(combination)
                for remaining_set in remaining_sets:
                    random_element = random.choice(list(remaining_set))
                    current_combination.append(random_element)
                result.append(current_combination)
            path_node_all.append(result)

        return path_node_all

    def build_path_node_list_dijkstra(self):
        # 当前所在的cell的上path node 到goal cell的下边界node set 2
        path_set_two = self.mapInfo.find_current_path_set_cell_neighbor()
        return path_set_two
    def init_load_balance(self):
        self.alpha = 0.9
        self.path_names = list(self.path_idx_info.keys())
        self.m = len(self.path_names)  # 路径数量
        self.name_to_idx = {name: idx for idx, name in enumerate(self.path_names)}

    def path_select_main(self):
        self.init_load_balance()
        assignment, T_max, T = self.load_balance_path_assignment()
        # 输出结果
        print("机器人路径分配:", assignment)
        # print("每条路径的通行时间 T^j:", [round(t, 2) for t in T])
        print("最大通行时间 T_max:", round(T_max, 2))
        des_path_node = []
        for i in range(len(assignment)):
            path_node = self.path_idx_info[assignment[i]]['path_node']
            des_path_node.append(path_node)
        return des_path_node


    def merge_path_set(self, path_set_one, path_set_two):
        # 找到最大流的path set 和 dijkstra的path set
        # 将它们合并起来考虑
        # 需要构建一个 dict idx 为机器人id
        # 里面值为 机器人的path list
        # 每个path 带有前b条边的信息 我知道第几条边
        # 长度信息 容量信息

        # 构建一个list 和 一个字典
        # 第一个list 机器人有哪些path可以选择 path id 储存到一个list中
        # 字典 键-path idx 值 这个path 的len capa 前几条边的name list
        # 对于边 我们储存边的name string
        # 之后我们通过这个string 那记录每个边的被选择情况
        self.path_choice_list = []
        self.path_idx_info = {}

        # for i in range(len(path_set_one)):
        #     # print('i=' + str(i))
        #     # print(path_set_one[i])
        #     path_choice = []
        #     path_n = path_set_one[i]
        #     for j in range(len(path_n)):
        #         name = "one " + str(i) + "," + str(j)
        #         path_node_n = path_n[j]
        #         path_choice.append(name)
        #         path_len, min_capacity, front_edge = self.find_path_info(path_node_n)
        #         g_i = self.swarm.goal_node[i]
        #         path_length = self.mapInfo.revise_path_length_cell(i, g_i, path_node_n, path_len)
        #         self.path_idx_info[name] = \
        #             {'path_node': path_node_n, 'length': path_length, 'front_edge': front_edge, 'capacity': min_capacity}
        #     self.path_choice_list.append(path_choice)

        self.path_choice_list = [[] for _ in range(self.swarm.robots_num)]
        for idx, path_info in path_set_two.items():
            path_node_n = path_info['path_node']
            path_len = path_info['path_length']
            robot_idx = path_info['robot_idx']
            name = 'two ' + str(robot_idx) + ',' + str(idx)
            _, min_capacity, front_edge = self.find_path_info(path_node_n)
            self.path_idx_info[name] = {'path_node': path_node_n,
                                        'length': path_len, 'front_edge': front_edge, 'capacity': min_capacity}
            self.path_choice_list[robot_idx].append(name)
        # print()

    def find_path_info(self, path_node):
        # 根据传递进来的path node 计算长度 容量 前五边 并储存到 一个字典中
        front_num = self.beta
        front_edge = []
        for i in range(min(front_num, len(path_node) - 1)):
            front_edge.append(f"{path_node[i]},{path_node[i + 1]}")
        while len(front_edge) < front_num:
            front_edge.append(None)
        path_len, min_capacity = self.mapInfo.find_path_length(path_node)
        # print(front_path)
        return path_len, min_capacity, front_edge


    def build_time_expanded_network(self, G, robot_positions, target, t_start, t_end):
        """构建时间扩展网络，边上机器人投影到节点"""
        TEG = nx.DiGraph()

        # 构建时间扩展节点
        for v in G.nodes:
            for t in range(t_start, t_end + 1):
                TEG.add_node((v, t))

        # 构建移动边，使用原始容量
        for u, v in G.edges:
            travel_time = G[u][v]['travel_time']
            capacity = G[u][v]['capacity']
            for t in range(t_start, t_end - travel_time + 1):
                TEG.add_edge((u, t), (v, t + travel_time), capacity=capacity)

        # 添加等待边
        for v in G.nodes:
            for t in range(t_start, t_end):
                TEG.add_edge((v, t), (v, t + 1), capacity=float('inf'))

        # 添加源点和汇点
        S = 'SOURCE'
        T = 'SINK'
        TEG.add_node(S)
        TEG.add_node(T)

        # 统计每个初始位置的机器人数量
        pos_counts = Counter()
        for pos in robot_positions:
            if isinstance(pos, tuple):
                v, t_used = pos
                pos_counts[(v, t_used)] += 1  # 投影到 (v, t_used)
            else:
                pos_counts[(pos, t_start)] += 1  # 投影到 (pos, t_start)

        # 连接机器人初始位置，根据机器人数量设置容量
        for (node, t), count in pos_counts.items():
            TEG.add_edge(S, (node, t), capacity=count)

        # 连接目标
        for t in range(t_start, t_end + 1):
            TEG.add_edge((target, t), T, capacity=len(robot_positions))

        return TEG

    def extract_paths(self, flow_dict, start_node, target_node, TEG):
        """从 flow_dict 中提取所有可能的路径"""
        paths = []
        visited = set()

        def dfs(node, current_path):
            if node[0] == target_node:
                paths.append(current_path[:])
                return
            if node in visited:
                return
            visited.add(node)
            if node in flow_dict:
                for next_node, flow in flow_dict[node].items():
                    if flow > 0 and next_node in TEG[node] and TEG[node][next_node]['capacity'] > 0:
                        current_path.append((node, next_node))
                        dfs(next_node, current_path)
                        current_path.pop()
            visited.remove(node)

        dfs(start_node, [])
        return paths


    def ford_fulkerson_schedule(self):
        robot_positions = self.robot_positions
        G = self.cell_net
        target = self.robot_targets
        time_window = self.time_window
        t_start = 0
        t_end = t_start + time_window
        schedule = defaultdict(list)
        robot_num = len(robot_positions)
        initial_time_window = copy.copy(time_window)

        # Step 1: 计算最大流
        t1 = time.time()
        while True:
            TEG = self.build_time_expanded_network(G, robot_positions, target, t_start, t_end)
            flow_value, flow_dict = nx.maximum_flow(TEG, 'SOURCE', 'SINK')
            print(f"Time window: {time_window}, Flow value: {flow_value}")
            if flow_value >= robot_num:
                print(f"Sufficient flow value ({flow_value}) achieved with time_window = {time_window}")
                break
            time_window += initial_time_window
            t_end = t_start + time_window

        t2 = time.time()
        print(f"Flow value: {flow_value}")

        if flow_value < robot_num:
            print(f"Insufficient flow ({flow_value}) for {robot_num} robots.")
            return schedule

        # Step 2: 统计机器人出发节点
        robot_start_counts = defaultdict(int)
        robot_to_start = {}
        for i, pos in enumerate(robot_positions):
            if isinstance(pos, tuple):
                v, t_used = pos
                start_node = (v, t_used)
            else:
                start_node = (pos, t_start)
            robot_start_counts[start_node] += 1
            robot_to_start[i] = start_node

        # Step 3: 提取路径，支持多流量
        def extract_shortest_paths(flow_dict, source, sink, TEG, robot_num, robot_start_counts):
            paths = []
            remaining_flow = defaultdict(lambda: defaultdict(int))
            for u in flow_dict:
                for v, f in flow_dict[u].items():
                    remaining_flow[u][v] = f

            start_path_flow = defaultdict(int)  # 每个起点的已分配流量
            total_robots_assigned = 0

            while total_robots_assigned < robot_num:
                pq = [(0, [], source, float('inf'))]  # (总时间, 路径, 当前节点, 路径最大流量)
                visited = set()
                parent = {}

                while pq:
                    total_time, current_path, node, path_flow = heapq.heappop(pq)

                    if node == sink:
                        start_node = current_path[1][0] if len(current_path) > 1 else None
                        # 计算路径的实际流量
                        min_flow = path_flow
                        for (u, v) in current_path:
                            min_flow = min(min_flow, remaining_flow[u][v])

                        if min_flow == 0:
                            break

                        # 检查起点流量限制
                        if start_node:
                            available_robots = robot_start_counts[start_node] - start_path_flow[start_node]
                            if available_robots <= 0:
                                break
                            min_flow = min(min_flow, available_robots)

                        if min_flow > 0:
                            paths.append((current_path[:], min_flow, total_time))
                            total_robots_assigned += min_flow
                            if start_node:
                                start_path_flow[start_node] += min_flow
                            # 更新剩余容量
                            for (u, v) in current_path:
                                remaining_flow[u][v] -= min_flow
                            break

                    if node in visited or node not in flow_dict:
                        continue
                    visited.add(node)

                    for next_node, _ in flow_dict[node].items():
                        if next_node not in TEG[node] or remaining_flow[node][next_node] <= 0:
                            continue
                        edge_time = TEG[node][next_node].get('travel_time', 1)
                        new_time = total_time + edge_time
                        new_path = current_path + [(node, next_node)]
                        new_flow = min(path_flow, remaining_flow[node][next_node])
                        heapq.heappush(pq, (new_time, new_path, next_node, new_flow))
                        parent[next_node] = node

                if total_robots_assigned < robot_num and not pq:
                    print(f"Cannot find more paths, stopping at {total_robots_assigned} robots")
                    break

            return paths

        all_paths = extract_shortest_paths(flow_dict, 'SOURCE', 'SINK', TEG, robot_num, robot_start_counts)
        print(f"Total paths extracted: {len(all_paths)}")
        total_flow_extracted = sum(flow for _, flow, _ in all_paths)
        print(f"Total flow extracted: {total_flow_extracted}")

        # if total_flow_extracted < robot_num:
        #     print(f"Error: Only {total_flow_extracted} robots supported, need {robot_num}")
        #     return schedule

        # Step 4: 按起点分组路径
        path_by_start = defaultdict(list)
        for path, flow, total_time in all_paths:
            if len(path) < 2:
                continue
            start_node = path[1][0]
            path_by_start[start_node].append((path, flow, total_time))

        # Step 5: 为每个机器人分配路径
        edge_flow = defaultdict(int)
        assigned_robots = set()
        robot_idx = 0
        for start_node, robot_count in robot_start_counts.items():
            if start_node[0] == target:
                continue

            available_paths = path_by_start[start_node]
            robots_to_assign = robot_count

            for path, flow, total_time in available_paths:
                robots_on_path = min(flow, robots_to_assign)
                path_nodes = [(from_node, to_node) for from_node, to_node in path if
                              from_node != 'SOURCE' and to_node != 'SINK']

                for _ in range(robots_on_path):
                    if robot_idx >= robot_num:
                        break
                    for (from_node, from_t), (to_node, to_t) in path_nodes:
                        edge_flow[((from_node, from_t), (to_node, to_t))] += 1
                        schedule[robot_idx].append(((from_node, from_t), (to_node, to_t)))
                    assigned_robots.add(robot_idx)
                    # print(f"Robot {robot_idx}: start={start_node}, travel_time={total_time}")
                    robot_idx += 1

                robots_to_assign -= robots_on_path
                if robots_to_assign <= 0:
                    break

            if robots_to_assign > 0:
                print(f"Warning: {robots_to_assign} robots at {start_node} not assigned")

        t3 = time.time()
        print(f"Scheduled robots: {len(schedule)} out of {robot_num}")
        print(f"Total flow used (robots): {len(assigned_robots)}")
        print(f"Total edge usage: {sum(edge_flow.values())}")
        print(f"build_time_expanded_network time={t2 - t1}")
        print(f"path_extraction_and_allocation time={t3 - t2}")

        return schedule
    # def ford_fulkerson_schedule(self):
    #     """一次性调度所有机器人到目标"""
    #     robot_positions = self.robot_positions
    #     G = self.cell_net
    #     target = self.robot_targets
    #     time_window = self.time_window
    #
    #     t_start = 0
    #     t_end = t_start + time_window
    #     schedule = defaultdict(list)
    #     robot_num = len(robot_positions)
    #     initial_time_window = copy.copy(time_window)
    #
    #     t1 = time.time()
    #     while True:
    #         TEG = self.build_time_expanded_network(G, robot_positions, target, t_start, t_end)
    #         flow_value, flow_dict = nx.maximum_flow(TEG, 'SOURCE', 'SINK')
    #
    #         print(f"Time window: {time_window}, Flow value: {flow_value}")
    #         if flow_value >= robot_num:
    #             print(f"Sufficient flow value ({flow_value}) achieved with time_window = {time_window}")
    #             break
    #         else:
    #             gap = robot_num - flow_value
    #             print(f"Flow value ({flow_value}) < robot_num ({robot_num}), gap = {gap}")
    #             if flow_value == 0:
    #                 time_window += initial_time_window
    #                 print(f"Flow value is 0, increasing time_window to {time_window}")
    #             else:
    #                 increment = int(time_window * (robot_num - flow_value) / flow_value)
    #                 time_window += max(increment, 1)
    #                 print(f"Adjusting time_window, increment = {increment}, new time_window = {time_window}")
    #         t_end = t_start + time_window
    #
    #     t2 = time.time()
    #     print(f"Flow value: {flow_value}")
    #
    #     if flow_value == 0:
    #         print("No flow possible.")
    #         return schedule
    #
    #     edge_flow = defaultdict(int)
    #     for robot_idx, start_pos in enumerate(robot_positions):
    #         if isinstance(start_pos, tuple):
    #             v, t_used = start_pos
    #             current_node = (v, t_used)  # 投影到 (v, t_used)
    #         else:
    #             current_node = (start_pos, t_start)
    #
    #         if current_node[0] == target:
    #             continue
    #
    #         all_paths = self.extract_paths(flow_dict, current_node, target, TEG)
    #
    #         if not all_paths:
    #             continue
    #
    #         all_paths.sort(key=lambda path: (path[-1][1][1], len(path)))
    #         selected_path = None
    #         for path in all_paths:
    #             can_use_path = True
    #             for (from_node, from_t), (to_node, to_t) in path:
    #                 edge = ((from_node, from_t), (to_node, to_t))
    #                 if edge_flow[edge] >= TEG[(from_node, from_t)][(to_node, to_t)]['capacity']:
    #                     can_use_path = False
    #                     break
    #             if can_use_path:
    #                 selected_path = path
    #                 break
    #         # if selected_path is None:
    #         #     print("robot i=" + str(robot_idx) + "has no path")
    #
    #         if selected_path:
    #             for (from_node, from_t), (to_node, to_t) in selected_path:
    #                 edge_flow[((from_node, from_t), (to_node, to_t))] += 1
    #                 schedule[robot_idx].append(((from_node, from_t), (to_node, to_t)))
    #
    #     t3 = time.time()
    #     print("build_time_expanded_network time=" + str(t2 - t1))
    #     print("extract_paths time=" + str(t3 - t2))
    #
    #     return schedule

    def plot_network_with_paths(self, G, schedule, pos=None):
        """绘制基础网络并在边上显示使用频率，不绘制路径"""
        # 如果未提供位置，则使用自动布局
        if pos is None:
            # pos = nx.spring_layout(G)
            pos = {node: data['pos'] for node, data in self.cell_net.nodes(data=True) if 'pos' in data}

        # 创建绘图
        plt.figure(figsize=(10, 8))

        # 绘制基础网络的节点和边
        nx.draw_networkx_nodes(G, pos, node_size=500, node_color='lightblue')
        nx.draw_networkx_edges(G, pos, edge_color='gray', width=1, arrows=True)
        nx.draw_networkx_labels(G, pos, font_size=12)

        # 统计每条边的使用频率
        edge_usage = defaultdict(int)
        for robot_idx, path in schedule.items():
            for (from_node, from_t), (to_node, to_t) in path:
                edge = (from_node, to_node)
                edge_usage[edge] += 1

        # 添加频率标签
        edge_labels = {}
        for edge, usage in edge_usage.items():
            edge_labels[edge] = str(usage)

        # 在边上显示频率标签
        nx.draw_networkx_edge_labels(G, pos, edge_labels=edge_labels, font_size=10, font_color='black')

        # 添加标题和显示
        plt.title("Base Network with Path Usage Frequencies (No Paths)")
        plt.axis('off')
        plt.show()

    def run_example_one(self):

        # 初始机器人位置
        robot_positions = [20, 20]
        # robot_positions = [20] * 5000
        target = 21
        time_window = 300

        # 运行调度算法
        schedule = self.ford_fulkerson_schedule(self.cell_net, robot_positions, target, time_window)

        # 输出调度计划
        print("\n机器人调度计划：")
        for robot_idx, path in schedule.items():
            print(f"机器人 {robot_idx}:")
            for (from_node, from_t), (to_node, to_t) in path:
                print(f"  from t= {from_t} node= {from_node} to t= {to_t} node= {to_node}")

    def init_path_allocation(self, paths: list[dict], robots_and_path: list[set[int]], speed: float, alpha: float = 0.5):
        self.paths = paths  # 路径信息
        self.robots_and_path = robots_and_path  # 机器人可选路径
        self.speed = speed  # 统一速度
        self.alpha = alpha  # 拥堵系数
        self.path_loads = {p['id']: 0 for p in paths}  # 路径上的机器人数量
        self.path_times = {p['id']: p['length'] / speed for p in paths}  # 路径完成时间
        self.assignments = [-1] * len(robots_and_path)  # 每个机器人的路径分配结果

    def congestion_time(self, path_id: int, k: int) -> float:
        """计算拥堵时间"""
        path = next(p for p in self.paths if p['id'] == path_id)
        width = path['width']
        return self.alpha * (k / width) ** 2

    def update_path_time(self, path_id: int):
        """更新路径完成时间"""
        path = next(p for p in self.paths if p['id'] == path_id)
        base_time = path['length'] / self.speed
        congestion = self.congestion_time(path_id, self.path_loads[path_id])
        self.path_times[path_id] = base_time + congestion

    def allocate_paths(self) -> dict[int, int]:
        """执行路径分配算法"""
        # 按可选路径数量从小到大排序机器人
        robot_order = [(len(self.robots_and_path[i]), i) for i in range(len(self.robots_and_path))]
        robot_order.sort()  # 升序排序

        for _, robot_id in robot_order:
            # 选择当前完成时间最小的路径
            min_time = float('inf')
            best_path = None
            for path_id in self.robots_and_path[robot_id]:
                if self.path_times[path_id] < min_time:
                    min_time = self.path_times[path_id]
                    best_path = path_id

            # 分配机器人到最佳路径
            self.assignments[robot_id] = best_path
            self.path_loads[best_path] += 1
            self.update_path_time(best_path)

        # 返回分配结果
        return dict(enumerate(self.assignments))

    def get_max_time(self) -> float:
        """计算最大完成时间 T_max"""
        return max(self.path_times.values())

    def compute_delay(self, path_name, l_selec):
        """计算路径的拥堵时间 T^j_delay"""
        path_info = self.path_idx_info[path_name]
        front_edges = path_info['front_edge'][:self.beta]  # 只考虑前 beta 个边
        delay = 0
        for k, edge in enumerate(front_edges):
            w_k = self.alpha ** k  # 权重递减
            capacity = path_info.get('capacity', 2)  # 默认容量为 2
            load_ratio = l_selec[edge] / capacity if capacity > 0 else float('inf')
            f = max(0, load_ratio - 1)  # 阈值线性函数
            delay += w_k * f
        return delay

    def load_balance_path_assignment(self):
        """
        执行负载均衡路径分配
        返回:
        - assignment: 每个机器人的路径名称
        - T_max: 最大通行时间
        - T: 每条路径的通行时间
        """
        # 初始化
        assignment = [None] * self.swarm.robots_num  # 机器人 i 分配到的路径名称
        l_selec = defaultdict(int)  # 每条边的选择次数
        T = [self.path_idx_info[name]['length'] / self.v_net for name in self.path_names]  # 初始 T^j

        # 按 |S_i| 递增排序机器人
        robots = sorted(range(self.swarm.robots_num), key=lambda i: len(self.path_choice_list[i]))

        # 贪心分配
        for i in robots:
            S_i = self.path_choice_list[i]  # 机器人 i 的可选路径名称
            # 计算当前每个可选路径的 T^j
            T_options = []
            for path_name in S_i:
                j = self.name_to_idx[path_name]
                delay = self.compute_delay(path_name, l_selec)
                T_j = self.path_idx_info[path_name]['length'] / self.v_net + delay
                if T_j > 0:
                    T_options.append((T_j, path_name))

            # 选择 T^j 最小的路径
            T_min, best_path = min(T_options)
            assignment[i] = best_path

            # 更新边选择次数
            front_edges = self.path_idx_info[best_path]['front_edge'][:self.beta]
            for edge in front_edges:
                l_selec[edge] += 1

            # 更新所有路径的 T^j
            for j, path_name in enumerate(self.path_names):
                delay = self.compute_delay(path_name, l_selec)
                T[j] = self.path_idx_info[path_name]['length'] / self.v_net + delay

        # 计算 T_max
        T_max = max(T)

        return assignment, T_max, T


    def set_center_pos(self):
        des_path_pos = []
        for i in range(self.swarm.robots_num):
            path_node = self.swarm.des_path_node[i]
            pos_list = []
            for node in path_node:
                if node == self.swarm.goal_node[i]:
                    pos = self.swarm.goal_pos[i]
                elif node == self.swarm.start_node[i]:
                    pos = self.mapInfo.node_start_end_adj[node]['pos']
                else:
                    pos = self.mapInfo.node_all_adj[node]['pos']
                    pos = [pos[0], pos[1]]
                pos_list.append(pos)
            des_path_pos.append(pos_list)
        self.swarm.des_path_pos = des_path_pos

    def position_allocation_greedy_now(self):
        # 为进入集群的无人机分配要出去的位置
        # 直接选择最近的 每次迭代直接找贪心的就好 正向传递
        for i in range(self.swarm.robots_num):
            path_node = self.swarm.des_path_node[i]
            pos_n = self.swarm.pos_all[i]

            for idx in range(len(path_node) - 1):
                pos_r = None
                dis_r = 99999999
                node_idx = path_node[idx]
                if idx == 0:
                    pos_compare = copy.copy(pos_n)
                else:
                    pos_compare = copy.copy(self.swarm.des_path_pos[i][idx - 1])
                if 0 <= node_idx < len(self.mapInfo.node_all_adj):
                    node_option = self.mapInfo.node_all_adj[node_idx]['node_option_pos']
                for pos_o in node_option:
                    dis = self.swarm.distance(pos_compare, pos_o)
                    if dis < dis_r:
                        pos_r = copy.copy(pos_o)
                        dis_r = copy.copy(dis)
                self.swarm.des_path_pos[i][idx] = copy.copy(pos_r)



# if __name__ == "__main__":
#     m1 = Max_Flow_planner()
#     m1.run_example()