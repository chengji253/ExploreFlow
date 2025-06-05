import time

import gurobipy as gp
import numpy as np
from gurobipy import GRB
import copy
import math
from collections import Counter
# water flow planner main module


class Flow_area_planner:

    def __init__(self, swarm, mapInfo, predictive_len_limit, time_horizon):
        self.swarm = swarm
        self.mapInfo = mapInfo

        self.number_of_robots = None

        self.number_of_zxp = None

        self.path_all = None

        # 在预测时间范围内 所有path经过的area
        # 构建一个双list
        # 第一个维度是时间 第二个维度是area
        self.area_in_time = None

        # zkp中 哪些是一架飞机 和要为0
        self.zkp_drone_list = None

        # zkp有哪些是属于同一个时间 同一个area
        self.zkp_area_info = None

        # L_pre
        self.predictive_len_limit = predictive_len_limit
        # K_horizon
        self.time_horizon = time_horizon

        self.path_area_dict = {}

        self.area_set_list = [set() for _ in range(self.time_horizon)]
        self.queue_cost_coefficient = [1, 1, 1]
        self.run_cost_coefficient = 0.03

        # area id 直接映射到 path idx
        self.shared_first_area = {}
        self.shared_second_area = {}

        self.area_to_path_info = None
        self.area_num = None

        self.des_path_node = None

    def global_scheduler_run(self):
        # 此时我已经得到了当前的path_all
        # 下一步就是计算出这些path的信息 提供给后续优化
        t0 = time.time()
        # self.mapInfo.find_current_path_set_cell_neighbor()
        self.mapInfo.find_current_path_set_purning()
        self.path_all = self.mapInfo.path_all
        print("len path all=" + str(len(self.path_all)))
        # 找到这些path的前面的area信息
        t1 = time.time()
        self.path_area_info()
        t2 = time.time()
        # print(self.area_set_list)

        self.init_area_info()
        self.compute_num_on_area()

        # des_path_node = self.flow_main()
        des_path_node = self.flow_main_merge()

        t3 = time.time()
        self.des_path_node = des_path_node
        # print()
        print("find_path_set time=" + str(t1 - t0))
        print("path_area_info time=" + str(t2 - t1))
        print("flow_main time=" + str(t3 - t2))

    def local_scheduler_run(self):
        self.position_allocation_load_balance()


    def path_area_info(self):
        # 将所有的area 按照 时间顺序 加入 set中
        for idx, path in self.path_all.items():
            path_node = path['path_node']
            area_info_list = []
            length_sum = 0
            path_node_tuple = tuple(path_node)
            if path_node_tuple in self.path_area_dict:
                area_result = self.path_area_dict[path_node_tuple]
                self.path_all[idx]['area_info_list'] = area_result
                continue

            for i in range(len(path_node) - 1):
                node_1 = path_node[i]
                node_2 = path_node[i+1]
                node_name = str(node_1) + ',' + str(node_2)
                if node_name not in self.mapInfo.edge_all_adj:
                    break

                if 'area_info' not in self.mapInfo.edge_all_adj[node_name]:
                    print("node name=" + str(node_name))

                area_info = self.mapInfo.edge_all_adj[node_name]['area_info']
                for j in range(len(area_info)):
                    length_sum += area_info[j][1]
                    area_info_list.append(area_info[j])
                if length_sum >= self.predictive_len_limit:
                    break
            # area_info_list中存的是在predictive_len_limit中经过的所有area
            # print(area_info_list)
            area_result = self.area_build(area_info_list)
            # print(area_result)
            self.path_all[idx]['area_info_list'] = area_result
            self.path_area_dict[path_node_tuple] = area_result
            self.area_store(area_result)

        self.area_set_list = []
        for i in range(self.time_horizon):
            area_set = set()
            for idx, value in self.path_all.items():
                area_info_list = value['area_info_list']
                area_n = area_info_list[i]
                area_set.add(area_n)
            self.area_set_list.append(area_set)
        # print()
        # 这里构建一个list self.area_to_path_info 其中存的是字典
        # 第一个字典 表示的是第一个时间的所有area
        # 每个字典里面是area到path的set映射
        self.area_to_path_info = [{} for _ in range(self.time_horizon)]
        # 循环整个path_all
        # 构建一个通过 area_id 可以直接映射到 对应哪些path id
        for path_idx, value in self.path_all.items():
            area_info_list = value['area_info_list']
            # 循环所有的时间范围 依次添加进去
            for i in range(self.time_horizon):
                area_n = area_info_list[i]
                if area_n in self.area_to_path_info[i]:
                    self.area_to_path_info[i][area_n].add(path_idx)
                else:
                    self.area_to_path_info[i][area_n] = set()
                    self.area_to_path_info[i][area_n].add(path_idx)

    def query_area(self, path_node):
        area_info_list = []
        length_sum = 0
        path_node_tuple = tuple(path_node)
        if path_node_tuple in self.path_area_dict:
            area_result = self.path_area_dict[path_node_tuple]
            return area_result

        for i in range(len(path_node) - 1):
            node_1 = path_node[i]
            node_2 = path_node[i + 1]
            node_name = str(node_1) + ',' + str(node_2)
            if node_name not in self.mapInfo.edge_all_adj:
                break

            if 'area_info' not in self.mapInfo.edge_all_adj[node_name]:
                print("node name=" + str(node_name))

            area_info = self.mapInfo.edge_all_adj[node_name]['area_info']
            for j in range(len(area_info)):
                length_sum += area_info[j][1]
                area_info_list.append(area_info[j])
            if length_sum >= self.predictive_len_limit:
                break
        area_result = self.area_build(area_info_list)
        self.path_area_dict[path_node_tuple] = area_result
        return area_result

    def area_store(self, area_result):
        for i in range(len(area_result)):
            area_n = area_result[i]
            if area_n is not None:
                self.area_set_list[i].add(area_n)

    def area_build(self, area_info_list: list[tuple[int, int]]) -> list[int]:
        # 初始参数
        time_horizon = self.time_horizon
        size_out = 10  # 初始 size_out
        segments = []
        values = []
        current_end = 0
        # 计算段的起始和结束位置
        for cell_value, length in area_info_list:
            start = current_end
            end = start + length
            segments.append([start, end])
            values.append(cell_value)
            current_end = end
        # 创建初始 grid
        total_length = math.floor(current_end) + 1
        grid = [-1] * total_length
        for i, (start, end) in enumerate(segments):
            grid_start = math.floor(start * size_out)
            grid_end = math.floor(end * size_out)
            grid[grid_start:grid_end] = [values[i]] * (grid_end - grid_start)
        # 重新计算 size_out 以覆盖整个 grid
        size_out = math.ceil(len(grid) / time_horizon)  # 每个时间段的元素数
        result = [None] * time_horizon
        # 计算每个时间段的主要元素
        for i in range(time_horizon):
            start_idx = i * size_out
            end_idx = min((i + 1) * size_out, len(grid))  # 防止越界
            segment = [x for x in grid[start_idx:end_idx] if x != -1]
            if segment:
                result[i] = Counter(segment).most_common(1)[0][0]
            else:
                result[i] = -1
        return result

    # def area_build(self, area_info_list: list[tuple[int, int]]) -> list[int]:
    #     # 计算path对应的area 不同时间段中最长的那个area
    #     # time horizon是划分的 每段的长度
    #     time_horizon = self.time_horizon
    #     size_out = 10
    #     # 初始化结果列表
    #     result = [None] * time_horizon
    #     # 计算起始和结束位置
    #     segments = []
    #     values = []
    #     current_end = 0
    #     for cell_value, length in area_info_list:
    #         start = current_end
    #         end = start + length
    #         segments.append([start, end])
    #         values.append(cell_value)
    #         current_end = end
    #     # 创建填充数组
    #     total_length = math.floor(current_end) + 1
    #     grid = [-1] * total_length
    #     # 填充网格
    #     for i, (start, end) in enumerate(segments):
    #         grid_start = math.floor(start * size_out)
    #         grid_end = math.floor(end * size_out)
    #         grid[grid_start:grid_end] = [values[i]] * (grid_end - grid_start)
    #     # 计算每个时间段的主要元素
    #     for i in range(time_horizon):
    #         start_idx = i * size_out
    #         end_idx = (i + 1) * size_out
    #         segment = [x for x in grid[start_idx:end_idx] if x != -1]
    #         if segment:  # 添加空列表检查
    #             result[i] = Counter(segment).most_common(1)[0][0]
    #         else:
    #             result[i] = -1  # 或其他默认值
    #     # print()
    #     return result

    def init_area_info(self):
        # 初始化不同的path的area信息
        # k是机器人的idx
        self.number_of_k = self.swarm.robots_num

        # 所有路径的信息
        self.path_info = self.mapInfo.path_all

        # 所有边的信息
        self.edge_info = self.mapInfo.edge_all_adj

        # 所有需要决策的二元变量
        self.number_of_zkp = len(self.path_info)

        # 第一个阶段有哪些area
        self.number_of_first_area = None
        self.number_of_second_area = None

        # 第一个阶段中 各个area 占据这些area的zkp
        self.zkp_area_list = None

        # 哪些zkp是属于一架飞机的 方便添加和为1的限制
        self.zkp_drone_list = [[] for _ in range(self.number_of_k)]

        # 路径cost
        self.k_path_cost = []

        for key, value in self.path_info.items():
            path_cost = value['path_length']
            path_cost = round(path_cost, 2)
            if math.isnan(path_cost) or math.isinf(path_cost):
                path_cost = 99999999
            self.k_path_cost.append(path_cost)
            drone_idx = value['robot_idx']
            self.zkp_drone_list[drone_idx].append(key)

        for sublist in self.zkp_drone_list:
            if len(sublist) == 0:
                print("zkp_drone_list 有的是空的")

        self.first_area_idx = []
        self.second_area_idx = []
        self.first_area_capacity = []
        self.second_area_capacity = []

        self.first_area_num = []
        self.second_area_num = []

        self.shared_area = []
        for i in range(self.time_horizon):
            area_idx_list = []
            area_set_n = self.area_set_list[i]
            for area_ in area_set_n:
                area_idx_list.append(area_)
            self.shared_area.append(area_idx_list)


    def flow_main(self):
        # Create a new model
        m = gp.Model("mip_path_allocation")
        m.setParam('TimeLimit', 2)

        # print(f"First area set: {self.area_set_list[0]}")
        # print(f"Second area set: {self.area_set_list[1]}")

        m.params.NonConvex = 2

        zkp_li = list(range(self.number_of_zkp))

        # 机器人数量构建k_li
        k_li = list(range(self.number_of_k))

        # 第一个时间范围中area的数量
        number_of_first_area = len(self.area_set_list[0])
        first_li = list(range(number_of_first_area))

        # 第二个时间范围中area的数量
        number_of_second_area = len(self.area_set_list[1])
        second_li = list(range(number_of_second_area))

        # k 无人机给path上分配的无人机数量 gp.GRB.CONTINUOUS
        z_k_p = m.addVars(zkp_li, vtype=gp.GRB.BINARY, name="z_k_p")

        queue_cost_sum_first = m.addVar(name="queue_cost_sum_first")
        queue_cost_sum_second = m.addVar(name="queue_cost_sum_second")
        # queue cost
        queue_cost_first_area = m.addVars(first_li, name="queue_cost_first_area")
        queue_cost_second_area = m.addVars(second_li, name="queue_cost_second_area")

        first_area_sum = m.addVars(first_li, name="first_area_sum")
        second_area_sum = m.addVars(second_li, name="second_area_sum")

        zkp_sum = m.addVars(k_li, name="zkp_sum")

        run_cost_sum = m.addVar(name="run_cost")

        # 找到z_k_p二维矩阵中 有哪些path是经过边first edge的 将它们加起来
        for shared_i in range(number_of_first_area):
            # 这个list存的是 zkp中有哪些id是属于同一个第一area的
            # zkp_first_area_list 是一个二维list 第一维是area 第二维是具体的path id
            area_idx_n = self.shared_area[0][shared_i]
            path_idx_set = self.area_to_path_info[0][area_idx_n]
            zkp_shared_idx = list(path_idx_set)
            # 这个约束将 所有属于一个area的path id求和 并赋值到first_area_sum这个一维list中
            m.addConstr(first_area_sum[shared_i] == gp.quicksum(z_k_p[idx] for idx in zkp_shared_idx))

        # queue cost of first edge
        # 在得到每个area的path和后 依次计算它们的queue cost
        for shared_i in range(number_of_first_area):
            area_idx_n = self.shared_area[0][shared_i]
            if area_idx_n == -1:
                m.addConstr(queue_cost_first_area[shared_i] == 0)
            else:
                capacity_n = self.mapInfo.area_capacity[area_idx_n]
                if capacity_n <= 0:
                    print("capacity_n <=0 error !")
                if area_idx_n in self.area_num:
                    current_num_area = self.area_num[area_idx_n]
                else:
                    current_num_area = 0
                m.addConstr(queue_cost_first_area[shared_i] ==
                            ((current_num_area + first_area_sum[shared_i] - capacity_n) *
                              (current_num_area + first_area_sum[shared_i] - capacity_n)
                             / (capacity_n * capacity_n)))

        # 找到z_k_p二维矩阵中 有哪些path是经过边second area的 将它们加起来
        for shared_i in range(number_of_second_area):
            area_idx_n = self.shared_area[1][shared_i]
            path_idx_set = self.area_to_path_info[1][area_idx_n]
            zkp_shared_idx = list(path_idx_set)
            m.addConstr(second_area_sum[shared_i] == gp.quicksum(z_k_p[idx] for idx in zkp_shared_idx))

        # queue cost of second edge
        # 在得到每个area的path和后 依次计算它们的queue cost
        for shared_i in range(number_of_second_area):
            area_idx_n = self.shared_area[1][shared_i]
            if area_idx_n == -1:
                m.addConstr(queue_cost_second_area[shared_i] == 0)
            else:
                capacity_n = self.mapInfo.area_capacity[area_idx_n]
                if capacity_n <= 0:
                    print("capacity_n <=0 error !")
                if area_idx_n in self.area_num:
                    current_num_area = self.area_num[area_idx_n]
                else:
                    current_num_area = 0
                m.addConstr(queue_cost_second_area[shared_i] ==
                            ((current_num_area + second_area_sum[shared_i] - capacity_n) *
                             (current_num_area + second_area_sum[shared_i] - capacity_n)
                                    / (capacity_n * capacity_n)))

        # 添加机器人约束 一次只能选择一个path
        for k_i in range(self.number_of_k):
            zkp_list = self.zkp_drone_list[k_i]
            m.addConstr(zkp_sum[k_i] == gp.quicksum(z_k_p[idx] for idx in zkp_list))
        for k_i in range(self.number_of_k):
            zkp_list = self.zkp_drone_list[k_i]
            if len(zkp_list) != 0:
                m.addConstr(zkp_sum[k_i] == 1)

        # 将第一个area的所有queue cost加起来
        m.addConstr(
            queue_cost_sum_first == gp.quicksum(queue_cost_first_area[shared_i] for shared_i in range(number_of_first_area)))

        # 将第二个area的所有queue cost加起来
        m.addConstr(
            queue_cost_sum_second == gp.quicksum(queue_cost_second_area[shared_i] for shared_i in range(number_of_second_area)))

        # m.addConstr(number_of_k == gp.quicksum(x_k_p[idx] for idx in range(number_of_xkp)))

        # 计算所有的path cost
        m.addConstr(run_cost_sum == gp.quicksum(z_k_p[idx] * self.k_path_cost[idx] for idx in range(self.number_of_zkp)))

        obj = m.addVar(name="obj")

        m.addConstr(obj == self.queue_cost_coefficient[0] * queue_cost_sum_first
                    + self.queue_cost_coefficient[1] * queue_cost_sum_second
                    + self.run_cost_coefficient * run_cost_sum)

        m.setParam('OutputFlag', 0)

        m.setObjective(obj, GRB.MINIMIZE)

        m.optimize()

        print(f"Model status: {m.status}")
        if m.status == gp.GRB.INFEASIBLE:
            print("Model is infeasible")
            m.computeIIS()  # 计算不可行约束集
            m.write("infeasible.ilp")  # 输出不可行约束到文件

        all_vars = m.getVars()

        try:
            # 可能会出错的代码段
            # for v in m.getVars():
            #     print('%s %g' % (v.VarName, v.X))
            values = m.getAttr("X", all_vars)
            self.zkp_res = values[0:self.number_of_zkp]

            self.del_path_res()
            des_path_node = [[] for _ in range(self.number_of_k)]
            for path_idx in self.path_sel_res:
                path_node = self.path_info[path_idx]['path_node']
                robot_idx = self.path_info[path_idx]['robot_idx']
                des_path_node[robot_idx] = path_node
        except gp.GurobiError as e:
            # 出错时的处理代码
            print(f"An error occurred: {e}")
            des_path_node = []
        # values = m.getAttr("X", all_vars)
        # names = m.getAttr("VarName", all_vars)
        # print()
        return des_path_node

    def compute_num_on_area(self):
        # 计算在每个area上的机器人数量
        self.area_num = {}
        for i in range(self.swarm.robots_num):
            pos_n = self.swarm.pos_all[i]
            pos_idx = [int(pos_n[0] * self.swarm.resolution), int(pos_n[1] * self.swarm.resolution)]

            size_n = self.mapInfo.map_01.shape
            if 0 <= pos_idx[0] < size_n[0] and 0 <= pos_idx[1] < size_n[1]:
                cell_idx = self.mapInfo.area_map[pos_idx[0], pos_idx[1]]
                if cell_idx in self.area_num:
                    self.area_num[cell_idx] += 1
                else:
                    self.area_num[cell_idx] = 1
    def del_path_res(self):
        # 处理xkp的结果
        self.path_sel_res = []
        for i in range(len(self.zkp_res)):
            if abs(self.zkp_res[i] - 1) <= 0.001:
                self.path_sel_res.append(i)


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

    def position_allocation_load_balance(self):
        # 按照load balance的思想分配
        desNode_to_robotId = {}
        # 找到哪些机器人同属于一个des node
        for i in range(self.swarm.robots_num):
            des_node = self.swarm.des_path_node[i]
            des_n = des_node[0]
            if des_n in desNode_to_robotId:
                desNode_to_robotId[des_n].append(i)
            else:
                desNode_to_robotId[des_n] = []
                desNode_to_robotId[des_n].append(i)
        # 为这些属于一个同一个path node的机器人分配当下的一个path pos
        for desNode, robot_id_list in desNode_to_robotId.items():
            # 找到这个path node中 所有的path pos
            if desNode not in self.mapInfo.node_all_adj:
                continue
            node_option_pos = self.mapInfo.node_all_adj[desNode]['node_option_pos']
            # 计算负载均衡 分配当下path pos
            allocation_r = self.allocation_in_node(robot_id_list, node_option_pos)
            # print()
            # 按照距离最近的原则 分配后续的path pos位置
            for i, pos_first in allocation_r.items():
                path_node = self.swarm.des_path_node[i]
                for idx in range(len(path_node) - 1):
                    if idx == 0:
                        self.swarm.des_path_pos[i][idx] = pos_first
                    else:
                        pos_r = None
                        dis_r = float('inf')
                        node_idx = path_node[idx]
                        pos_compare = copy.copy(self.swarm.des_path_pos[i][idx - 1])
                        node_option = self.mapInfo.node_all_adj[node_idx]['node_option_pos']
                        for pos_o in node_option:
                            dis = self.swarm.distance(pos_compare, pos_o)
                            if dis < dis_r:
                                pos_r = pos_o
                                dis_r = dis
                        self.swarm.des_path_pos[i][idx] = copy.copy(pos_r)


    def allocation_in_node(self, robot_id_list, node_option_pos):
        """
        为每个机器人分配一个出口位置
        参数：
        - robot_id_list: 机器人ID列表
        - node_option_pos: 出口位置列表，例如 [(2.0, 21.0), (3.0, 21.0), ...]
        返回：
        - dict: {robot_id: allocated_pos}，每个机器人分配到的出口位置
        """
        # 初始化结果字典和每个出口的负载计数
        allocation = {}  # 存储每个机器人的分配结果
        load_count = {pos: 0 for pos in node_option_pos}  # 每个出口的当前负载
        # 假设每个出口的容量均分，总机器人数量除以出口数量
        total_robots = len(robot_id_list)
        capacity_per_exit = total_robots / len(node_option_pos) if node_option_pos else 1
        # 负载均衡和距离权重的调节参数
        beta = 0.5  # 可根据需要调整，平衡距离和负载的重要性
        # 为每个机器人分配出口
        for robot_id in robot_id_list:
            # 获取当前机器人的位置
            robot_pos = self.swarm.pos_all[robot_id]  # 假设 pos_all 是一个 (x, y) 元组
            # 计算每个出口的综合成本
            min_cost = float('inf')
            best_pos = None
            for exit_pos in node_option_pos:
                # 计算机器人到出口的欧几里得距离
                distance = self.swarm.distance(robot_pos, exit_pos)
                # 当前出口的负载
                current_load = load_count[exit_pos]
                # 计算综合成本：距离 + 负载均衡因子
                cost = distance + beta * (current_load / capacity_per_exit)
                # 找到成本最低的出口
                if cost < min_cost:
                    min_cost = cost
                    best_pos = exit_pos
            # 分配最佳出口并更新负载
            allocation[robot_id] = best_pos
            load_count[best_pos] += 1
        return allocation

    def local_path_choose(self, i):
        # 局部的决策 考虑更多的分支 距离机器人当前更多的分支
        # 使用局部path set search 搜索当前的一些岔路
        path_node = copy.copy(self.swarm.des_path_node[i])
        path_list_n = self.mapInfo.local_path_search(i, path_node)
        # 对这些岔路进行基本的评估 然后选择最适合自己的path
        path_node_result = self.path_choose(path_list_n)
        return path_node_result

    def path_choose(self, path_list):
        # 在这个path_set中选择一个合适的path node
        self.compute_num_on_area()
        for path_info in path_list:
            path_node = path_info[0]
            path_length = path_info[1]
            area_list = self.query_area(path_node)
            area_num_list = []
            for area in area_list:
                if area in self.area_num:
                    num_n = self.area_num[area]
                else:
                    num_n = 0
                area_num_list.append(num_n)
            # path_info.append(area_num_list)
            if len(area_num_list) >= 2:
                path_length = path_length + area_num_list[0] * 0.5 + area_num_list[1] + 0.2

        if len(path_list) == 0:
            return []
        sorted_path_list = sorted(path_list, key=lambda x: x[1])
        result = sorted_path_list[0][0]
        return result

    def flow_main_merge(self):
        # 现在主要的 算法部分
        # 在原来模型的基础上 进行合并
        # 一边 二边合并起来 一次可以计算多个area
        # Create a new model
        m = gp.Model("mip_path_allocation")
        m.setParam('TimeLimit', 2)

        # print(f"First area set: {self.area_set_list[0]}")
        # print(f"Second area set: {self.area_set_list[1]}")

        m.params.NonConvex = 2

        zkp_li = list(range(self.number_of_zkp))

        # 机器人数量构建k_li
        k_li = list(range(self.number_of_k))

        # k 无人机给path上分配的无人机数量 gp.GRB.CONTINUOUS
        z_k_p = m.addVars(zkp_li, vtype=gp.GRB.BINARY, name="z_k_p")

        zkp_sum = m.addVars(k_li, name="zkp_sum")

        run_cost_sum = m.addVar(name="run_cost")

        # 预测的area的长度
        number_of_predict_area = self.time_horizon  # 动态获取边的数量

        # 初始化存储每个边区域和的列表和队列成本的列表
        area_sum = []  # 每个边的区域和
        queue_cost_area = []  # 每个边的队列成本
        queue_cost_sum = []  # 每个边的总队列成本


        # 为每个预测的area创建变量
        for predict_idx in range(number_of_predict_area):
            num_areas = len(self.shared_area[predict_idx])  # 该边的区域数量
            area_sum.append(m.addVars(num_areas, name=f"area_sum_{predict_idx}"))
            queue_cost_area.append(m.addVars(num_areas, name=f"queue_cost_area_{predict_idx}"))
            queue_cost_sum.append(m.addVar(name=f"queue_cost_sum_{predict_idx}"))

        # 计算每个边的 area_sum 和 queue_cost_area
        for predict_idx in range(number_of_predict_area):
            num_areas = len(self.shared_area[predict_idx])
            # 找到 z_k_p 中经过当前边的路径并加起来
            for shared_i in range(num_areas):
                area_idx_n = self.shared_area[predict_idx][shared_i]
                path_idx_set = self.area_to_path_info[predict_idx][area_idx_n]
                zkp_shared_idx = list(path_idx_set)
                m.addConstr(area_sum[predict_idx][shared_i] == gp.quicksum(z_k_p[idx] for idx in zkp_shared_idx))

            # 计算排队成本
            for shared_i in range(num_areas):
                area_idx_n = self.shared_area[predict_idx][shared_i]
                if area_idx_n == -1:
                    m.addConstr(queue_cost_area[predict_idx][shared_i] == 0)
                else:
                    capacity_n = self.mapInfo.area_capacity[area_idx_n]
                    if capacity_n <= 0:
                        print(f"capacity_n <= 0 error for edge {predict_idx}, area {area_idx_n}!")
                    current_num_area = self.area_num.get(area_idx_n, 0)

                    total_load = current_num_area + area_sum[predict_idx][shared_i]

                    # 新建 overflow 变量：overflow = max(total_load - capacity, 0)
                    overflow = m.addVar(lb=0, name=f"overflow_{predict_idx}_{shared_i}")
                    m.addConstr(overflow >= total_load - capacity_n)
                    m.addConstr(overflow >= 0)

                    # 成本为 overflow^2 / capacity^2
                    norm_factor = capacity_n * capacity_n if capacity_n > 0 else 1  # 避免除以0
                    m.addConstr(queue_cost_area[predict_idx][shared_i] == (overflow * overflow) / norm_factor)

            # 将当前边的所有队列成本加起来
            m.addConstr(queue_cost_sum[predict_idx] == gp.quicksum(queue_cost_area[predict_idx][shared_i]
                                                                for shared_i in range(num_areas)))

        total_queue_cost = gp.quicksum(queue_cost_sum[edge_idx] for edge_idx in range(number_of_predict_area))

        # 添加机器人约束 一次只能选择一个path
        for k_i in range(self.number_of_k):
            zkp_list = self.zkp_drone_list[k_i]
            m.addConstr(zkp_sum[k_i] == gp.quicksum(z_k_p[idx] for idx in zkp_list))
        for k_i in range(self.number_of_k):
            zkp_list = self.zkp_drone_list[k_i]
            if len(zkp_list) != 0:
                m.addConstr(zkp_sum[k_i] == 1)

        # 计算所有的path cost
        m.addConstr(run_cost_sum == gp.quicksum(z_k_p[idx] * self.k_path_cost[idx] for idx in range(self.number_of_zkp)))

        obj = m.addVar(name="obj")

        m.addConstr(obj == total_queue_cost
                    + self.run_cost_coefficient * run_cost_sum)

        m.setParam('OutputFlag', 0)

        m.setObjective(obj, GRB.MINIMIZE)

        m.optimize()

        # print(f"Model status: {m.status}")
        # if m.status == gp.GRB.INFEASIBLE:
        #     print("Model is infeasible")
        #     m.computeIIS()  # 计算不可行约束集
        #     m.write("infeasible.ilp")  # 输出不可行约束到文件

        all_vars = m.getVars()

        try:
            # 可能会出错的代码段
            # for v in m.getVars():
            #     print('%s %g' % (v.VarName, v.X))
            values = m.getAttr("X", all_vars)
            self.zkp_res = values[0:self.number_of_zkp]

            self.del_path_res()
            des_path_node = [[] for _ in range(self.number_of_k)]
            for path_idx in self.path_sel_res:
                path_node = self.path_info[path_idx]['path_node']
                robot_idx = self.path_info[path_idx]['robot_idx']
                des_path_node[robot_idx] = path_node
        except gp.GurobiError as e:
            # 出错时的处理代码
            print(f"An error occurred: {e}")
            des_path_node = []
        # values = m.getAttr("X", all_vars)
        # names = m.getAttr("VarName", all_vars)
        # print()
        return des_path_node


    # def flow_main_merge(self):
    #     # 在原来模型的基础上 进行合并
    #     # 一边 二边合并起来 一次可以计算多个area
    #     # Create a new model
    #     m = gp.Model("mip_path_allocation")
    #     m.setParam('TimeLimit', 2)
    #
    #     # print(f"First area set: {self.area_set_list[0]}")
    #     # print(f"Second area set: {self.area_set_list[1]}")
    #
    #     m.params.NonConvex = 2
    #
    #     zkp_li = list(range(self.number_of_zkp))
    #
    #     # 机器人数量构建k_li
    #     k_li = list(range(self.number_of_k))
    #
    #     # k 无人机给path上分配的无人机数量 gp.GRB.CONTINUOUS
    #     z_k_p = m.addVars(zkp_li, vtype=gp.GRB.BINARY, name="z_k_p")
    #
    #     zkp_sum = m.addVars(k_li, name="zkp_sum")
    #
    #     run_cost_sum = m.addVar(name="run_cost")
    #
    #     # 预测的area的长度
    #     number_of_predict_area = 2  # 动态获取边的数量
    #
    #     # 初始化存储每个边区域和的列表和队列成本的列表
    #     area_sum = []  # 每个边的区域和
    #     queue_cost_area = []  # 每个边的队列成本
    #     queue_cost_sum = []  # 每个边的总队列成本
    #
    #
    #     # 为每个预测的area创建变量
    #     for predict_idx in range(number_of_predict_area):
    #         num_areas = len(self.shared_area[predict_idx])  # 该边的区域数量
    #         area_sum.append(m.addVars(num_areas, name=f"area_sum_{predict_idx}"))
    #         queue_cost_area.append(m.addVars(num_areas, name=f"queue_cost_area_{predict_idx}"))
    #         queue_cost_sum.append(m.addVar(name=f"queue_cost_sum_{predict_idx}"))
    #
    #     # 计算每个边的 area_sum 和 queue_cost_area
    #     for predict_idx in range(number_of_predict_area):
    #         num_areas = len(self.shared_area[predict_idx])
    #         # 找到 z_k_p 中经过当前边的路径并加起来
    #         for shared_i in range(num_areas):
    #             area_idx_n = self.shared_area[predict_idx][shared_i]
    #             path_idx_set = self.area_to_path_info[predict_idx][area_idx_n]
    #             zkp_shared_idx = list(path_idx_set)
    #             m.addConstr(area_sum[predict_idx][shared_i] == gp.quicksum(z_k_p[idx] for idx in zkp_shared_idx))
    #
    #         # 计算排队成本
    #         for shared_i in range(num_areas):
    #             area_idx_n = self.shared_area[predict_idx][shared_i]
    #             if area_idx_n == -1:
    #                 m.addConstr(queue_cost_area[predict_idx][shared_i] == 0)
    #             else:
    #                 capacity_n = self.mapInfo.area_capacity[area_idx_n]
    #                 if capacity_n <= 0:
    #                     print(f"capacity_n <= 0 error for edge {predict_idx}, area {area_idx_n}!")
    #                 # get方法提供了一种优雅的方式来处理键不存在的情况，避免异常 如果不存在就返回 0
    #                 current_num_area = self.area_num.get(area_idx_n, 0)
    #                 m.addConstr(queue_cost_area[predict_idx][shared_i] ==
    #                             ((current_num_area + area_sum[predict_idx][shared_i] - capacity_n) *
    #                              (current_num_area + area_sum[predict_idx][shared_i] - capacity_n)
    #                              / (capacity_n * capacity_n)))
    #
    #         # 将当前边的所有队列成本加起来
    #         m.addConstr(queue_cost_sum[predict_idx] == gp.quicksum(queue_cost_area[predict_idx][shared_i]
    #                                                             for shared_i in range(num_areas)))
    #
    #     total_queue_cost = gp.quicksum(queue_cost_sum[edge_idx] for edge_idx in range(number_of_predict_area))
    #
    #     # 添加机器人约束 一次只能选择一个path
    #     for k_i in range(self.number_of_k):
    #         zkp_list = self.zkp_drone_list[k_i]
    #         m.addConstr(zkp_sum[k_i] == gp.quicksum(z_k_p[idx] for idx in zkp_list))
    #     for k_i in range(self.number_of_k):
    #         zkp_list = self.zkp_drone_list[k_i]
    #         if len(zkp_list) != 0:
    #             m.addConstr(zkp_sum[k_i] == 1)
    #
    #     # 计算所有的path cost
    #     m.addConstr(run_cost_sum == gp.quicksum(z_k_p[idx] * self.k_path_cost[idx] for idx in range(self.number_of_zkp)))
    #
    #     obj = m.addVar(name="obj")
    #
    #     m.addConstr(obj == total_queue_cost
    #                 + self.run_cost_coefficient * run_cost_sum)
    #
    #     m.setParam('OutputFlag', 0)
    #
    #     m.setObjective(obj, GRB.MINIMIZE)
    #
    #     m.optimize()
    #
    #     # print(f"Model status: {m.status}")
    #     # if m.status == gp.GRB.INFEASIBLE:
    #     #     print("Model is infeasible")
    #     #     m.computeIIS()  # 计算不可行约束集
    #     #     m.write("infeasible.ilp")  # 输出不可行约束到文件
    #
    #     all_vars = m.getVars()
    #
    #     try:
    #         # 可能会出错的代码段
    #         # for v in m.getVars():
    #         #     print('%s %g' % (v.VarName, v.X))
    #         values = m.getAttr("X", all_vars)
    #         self.zkp_res = values[0:self.number_of_zkp]
    #
    #         self.del_path_res()
    #         des_path_node = [[] for _ in range(self.number_of_k)]
    #         for path_idx in self.path_sel_res:
    #             path_node = self.path_info[path_idx]['path_node']
    #             robot_idx = self.path_info[path_idx]['robot_idx']
    #             des_path_node[robot_idx] = path_node
    #     except gp.GurobiError as e:
    #         # 出错时的处理代码
    #         print(f"An error occurred: {e}")
    #         des_path_node = []
    #     # values = m.getAttr("X", all_vars)
    #     # names = m.getAttr("VarName", all_vars)
    #     # print()
    #     return des_path_node