import gurobipy as gp
import numpy as np
from gurobipy import GRB
import copy

# water flow planner main module


class Flow_planner:

    def __init__(self, swarm, mapInfo):
        # 包含节点和边的N
        self.swarm = swarm
        self.mapInfo = mapInfo

        # 无人机数量
        self.number_of_k = None
        self.number_of_cut = None
        self.number_of_xkp = None

        self.vel = 1

        self.cut_sink_node = None

        self.path_info = None

        self.xkp_res = None
        self.path_sel_res = None

        # xkp path 的cost
        self.k_path_cost = None
        # xkp中 经过cut边idx
        self.xkp_cut_list = None
        # xkp中 哪些是一架飞机 和要为0
        self.xkp_drone_list = None

        self.cut_edge_idx = None
        self.cut_edge_capacity = None

        self.k_1 = None
        self.k_2 = None
        self.k_3 = None

    def init_para_shared_edge(self):
        self.number_of_k = self.swarm.robots_num

        self.path_info = self.mapInfo.path_all

        self.shared_first_edge = self.mapInfo.shared_first_edge
        self.shared_second_edge = self.mapInfo.shared_second_edge

        self.number_of_xkp = len(self.path_info)

        self.number_of_first_edge = len(self.mapInfo.shared_first_edge)
        self.number_of_second_edge = len(self.mapInfo.shared_second_edge)

        self.xkp_first_edge_list = [[] for _ in range(len(self.mapInfo.shared_first_edge))]
        self.xkp_second_edge_list = [[] for _ in range(len(self.mapInfo.shared_second_edge))]

        self.xkp_drone_list = [[] for _ in range(self.number_of_k)]
        self.k_path_cost = []

        for key, value in self.path_info.items():
            path_cost = value['path_length']
            path_cost = round(path_cost, 2)
            self.k_path_cost.append(path_cost)
            drone_idx = value['robot_idx']
            self.xkp_drone_list[drone_idx].append(key)

        self.first_edge_idx = []
        self.second_edge_idx = []
        self.first_edge_capacity = []
        self.second_edge_capacity = []

        self.first_edge_num = []
        self.second_edge_num = []

        for i, (key, value) in enumerate(self.shared_first_edge.items()):
            path_set = value
            for path in path_set:
                self.xkp_first_edge_list[i].append(path)
            if key in self.mapInfo.edge_all:
                self.first_edge_capacity.append(self.mapInfo.edge_all[key]['capacity'])
            else:
                self.first_edge_capacity.append(1)
            self.first_edge_idx.append(i)

            if key in self.edge_robots_number:
                self.first_edge_num.append(self.edge_robots_number[key])
            else:
                self.first_edge_num.append(0)

        for i, (key, value) in enumerate(self.shared_second_edge.items()):
            path_set = value
            for path in path_set:
                self.xkp_second_edge_list[i].append(path)
            if key in self.mapInfo.edge_all:
                self.second_edge_capacity.append(self.mapInfo.edge_all[key]['capacity'])
            else:
                self.second_edge_capacity.append(1)
            self.second_edge_idx.append(i)
            if key in self.edge_robots_number:
                self.second_edge_num.append(self.edge_robots_number[key])
            else:
                self.second_edge_num.append(0)


    def compute_num_on_edge(self):
        # 计算每个边上的无人机数量
        self.edge_robots_number = {}
        for i in range(self.swarm.robots_num):
            if self.swarm.robot_exist[i] is True:
                node0 = self.swarm.current_edge[i, 0]
                node1 = self.swarm.current_edge[i, 1]
                if node1 != self.swarm.goal_node[i]:
                    edge = str(node0) + ',' + str(node1)
                    if edge in self.edge_robots_number:
                        self.edge_robots_number[edge] += 1
                    else:
                        self.edge_robots_number[edge] = 1

        # for key, item in self.edge_robots_number.items():
        #     print(key + ' ' + str(item))
    def position_allocation_greedy_now(self):
        # 为进入集群的无人机分配要出去的位置
        # 直接选择最近的 每次迭代直接找贪心的就好 正向传递
        for i in range(self.swarm.robots_num):
            path_node = self.swarm.des_path_node[i]
            pos_n = self.swarm.pos_all[i]
            for idx in range(len(path_node) - 1):
                pos_r = None
                dis_r = float('inf')
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

    def position_allocation_greedy_now_each(self, i):
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
            node_option = self.mapInfo.node_all[node_idx]['node_option_pos']
            for pos_o in node_option:
                dis = self.swarm.distance(pos_compare, pos_o)
                if dis < dis_r:
                    pos_r = copy.copy(pos_o)
                    dis_r = copy.copy(dis)
            self.swarm.des_path_pos[i][idx] = copy.copy(pos_r)

    def position_allocation(self):
        # 为进入集群的无人机分配要出去的位置
        # 直接选择最近的 每次迭代直接找贪心的就好 正向传递
        des_path_pos_current = []
        for i in range(self.swarm.robots_num):
            path_node = self.swarm.des_path_node[i]
            pos_n = self.swarm.pos_all[i]

            pos_list = []
            for idx in range(len(path_node) - 1):
                pos_r = None
                dis_r = 99999999
                node_idx = path_node[idx]
                if idx == 0:
                    pos_compare = copy.copy(pos_n)
                else:
                    # pos_compare = copy.copy(self.swarm.des_path_pos[i][idx - 1])
                    pos_compare = copy.copy(pos_list[idx - 1])
                    # pos_compare = copy.copy(des_path_pos_current[i][idx - 1])
                node_option = self.mapInfo.node_all[node_idx]['node_option_pos']
                for pos_o in node_option:
                    dis = self.swarm.distance(pos_compare, pos_o)
                    if dis < dis_r:
                        pos_r = copy.copy(pos_o)
                        dis_r = copy.copy(dis)
                pos_list.append(pos_r)
                # self.swarm.des_path_pos[i][idx] = copy.copy(pos_r)
            des_path_pos_current.append(pos_list)

        # 选择距离goal目标最近的一系列pos
        des_path_pos_goal = []
        for i in range(self.swarm.robots_num):
            path_node = self.swarm.des_path_node[i]
            pos_goal = self.swarm.goal_pos[i]
            pos_list = []

            for idx in range(len(path_node) - 2, -1, -1):
                pos_r = None
                dis_r = 99999999
                node_idx = path_node[idx]

                # pos_compare = copy.copy(self.swarm.des_path_pos[i][idx + 1])
                # pos_compare = copy.copy(des_path_pos_goal[i][idx + 1])
                id_li = len(path_node) - 2 - idx
                if id_li == 0:
                    pos_compare = pos_goal
                else:
                    pos_compare = pos_list[id_li - 1]

                node_option = self.mapInfo.node_all[node_idx]['node_option_pos']
                for pos_o in node_option:
                    dis = self.swarm.distance(pos_compare, pos_o)
                    if dis < dis_r:
                        pos_r = copy.copy(pos_o)
                        dis_r = copy.copy(dis)
                # self.swarm.des_path_pos[i][idx] = copy.copy(pos_r)
                pos_list.append(pos_r)

            pos_list.reverse()
            des_path_pos_goal.append(pos_list)

        for robot_idx in range(self.swarm.robots_num):
            self.compute_new_des_pos(des_path_pos_goal, des_path_pos_current, robot_idx)

        # 得到了当前节点的两个pos选择

        # 当前节点的边NUM情况

        # 根据边num情况和两个pos选择 做出决策

    def compute_new_des_pos(self, des_path_pos_goal, des_path_pos_current, robot_idx):

        if len(des_path_pos_current) != len(des_path_pos_goal):
            print("des pos error !")

        for i in range(len(des_path_pos_current)):
            pos_goal = des_path_pos_goal[robot_idx][i]
            pos_current = des_path_pos_current[robot_idx][i]
            if pos_goal == pos_current:
                pos_r = pos_goal
            else:
                ratio = self.get_ratio_capacity_and_num(robot_idx, i)
                pos_r = self.num_capa_to_new_pos(pos_goal, pos_current, ratio)
            self.swarm.des_path_pos[robot_idx][i] = copy.copy(pos_r)

    def num_capa_to_new_pos(self, goal_pos, current_pos, ratio):
        pos_new = [goal_pos[0], goal_pos[1]]
        pos_add = [ratio*(current_pos[0] - goal_pos[0]), ratio*(current_pos[1] - goal_pos[1])]

        pos_new[0] = pos_new[0] + pos_add[0]
        pos_new[1] = pos_new[1] + pos_add[1]
        pos_new = tuple(pos_new)
        return pos_new


    def get_ratio_capacity_and_num(self, robot_idx, i):
        # 返回当前节点的拥挤程度 根据机器人边的num
        if i == 0:
            node_1 = self.swarm.des_path_node[robot_idx][i]
            node_0 = self.swarm.current_node[robot_idx]
        else:
            node_1 = self.swarm.des_path_node[robot_idx][i]
            node_0 = self.swarm.des_path_node[robot_idx][i - 1]
        edge_n = str(node_0) + ',' + str(node_1)

        if edge_n in self.edge_robots_number:
            num = self.edge_robots_number[edge_n]
        else:
            num = 0

        if edge_n in self.mapInfo.edge_all:
            capacity = self.mapInfo.edge_all[edge_n]['capacity']
        else:
            capacity = 0
        # return robots_num

        if num == 0 or capacity == 0:
            ratio = 0
        else:
            ratio = num/capacity
            print("ratio=" + str(ratio))
            if ratio > 1:
                ratio = 1

        return ratio
    def find_same_des_path_node(self):
        # 找到无人机中des node相同的 并给他们分组
        same_node = {}
        for i in range(self.swarm.robots_num):
            node_n = self.swarm.des_path_node[i][0]
            if node_n in same_node:
                same_node[node_n].add(i)
            else:
                idx_set = set()
                idx_set.add(i)
                same_node[node_n] = idx_set
        # print()

    def path_selection_out_edge(self):
        number_of_k = self.number_of_k

        number_of_first_edge = self.number_of_first_edge
        number_of_second_edge = self.number_of_second_edge

        xkp_first_edge_list = self.xkp_first_edge_list
        xkp_second_edge_list = self.xkp_second_edge_list

        number_of_xkp = self.number_of_xkp
        k_path_cost = self.k_path_cost

        xkp_drone_list = self.xkp_drone_list

        first_edge_capacity = self.first_edge_capacity
        second_edge_capacity = self.second_edge_capacity

        first_edge_current_num = self.first_edge_num
        second_edge_current_num = self.second_edge_num

        k_li = list(range(number_of_k))
        xkp_li = list(range(number_of_xkp))

        first_li = list(range(number_of_first_edge))
        second_li = list(range(number_of_second_edge))

        # Create a new model
        m = gp.Model("mip_path_allocation")
        m.setParam('TimeLimit', 2)
        # m.Params.MIPGap = self.formation_choice_gap

        m.params.NonConvex = 2
        # k类无人机给path上分配的无人机数量 gp.GRB.CONTINUOUS
        x_k_p = m.addVars(xkp_li, vtype=gp.GRB.BINARY, name="x_k_p")
        # x_k_p = m.addVars(p_li, vtype=gp.GRB.CONTINUOUS, name="x_k_p")

        queue_cost_sum_first = m.addVar(name="queue_cost_sum_first")
        queue_cost_sum_second = m.addVar(name="queue_cost_sum_second")
        # queue cost
        queue_cost_first_edge = m.addVars(first_li, name="queue_cost_first_edge")
        queue_cost_second_edge = m.addVars(second_li, name="queue_cost_second_edge")

        first_edge_sum = m.addVars(first_li, name="first_edge_drone_sum")
        second_edge_sum = m.addVars(second_li, name="second_edge_drone_sum")

        xkp_sum = m.addVars(k_li, name="xkp_sum")

        # run_cost = m.addVar(k_li, name="run_cost")

        run_cost_sum = m.addVar(name="run_cost")

        # 找到x_k_p二维矩阵中 有哪些path是经过边first edge的 将它们加起来
        for shared_i in range(number_of_first_edge):
            xkp_shared_idx = xkp_first_edge_list[shared_i]
            m.addConstr(first_edge_sum[shared_i] == gp.quicksum(x_k_p[idx] for idx in xkp_shared_idx))

        # queue cost of first edge
        for shared_i in range(number_of_first_edge):
            m.addConstr(queue_cost_first_edge[shared_i] ==
                        ( (first_edge_current_num[shared_i] + first_edge_sum[shared_i] - first_edge_capacity[shared_i]) *
                          (first_edge_current_num[shared_i] + first_edge_sum[shared_i] - first_edge_capacity[shared_i])
                         / (first_edge_capacity[shared_i] * first_edge_capacity[shared_i]) ) )

        # 找到x_k_p二维矩阵中 有哪些path是经过边second edge的 将它们加起来
        for shared_i in range(number_of_second_edge):
            xkp_shared_idx = xkp_second_edge_list[shared_i]
            m.addConstr(second_edge_sum[shared_i] == gp.quicksum(x_k_p[idx] for idx in xkp_shared_idx))

        # queue cost of second edge
        for shared_i in range(number_of_second_edge):
            m.addConstr(queue_cost_second_edge[shared_i] ==
                        ((second_edge_current_num[shared_i] + second_edge_sum[shared_i] - second_edge_capacity[shared_i]) *
                         (second_edge_current_num[shared_i] + second_edge_sum[shared_i] - second_edge_capacity[shared_i])
                                / (second_edge_capacity[shared_i] * second_edge_capacity[shared_i])))

        for k_i in range(number_of_k):
            xkp_list = xkp_drone_list[k_i]
            m.addConstr(xkp_sum[k_i] == gp.quicksum(x_k_p[idx] for idx in xkp_list))

        for k_i in range(number_of_k):
            m.addConstr(xkp_sum[k_i] == 1)

        m.addConstr(
            queue_cost_sum_first == gp.quicksum(queue_cost_first_edge[shared_i] for shared_i in range(number_of_first_edge)))

        m.addConstr(
            queue_cost_sum_second == gp.quicksum(queue_cost_second_edge[shared_i] for shared_i in range(number_of_second_edge)))

        # m.addConstr(number_of_k == gp.quicksum(x_k_p[idx] for idx in range(number_of_xkp)))

        m.addConstr(run_cost_sum == gp.quicksum(x_k_p[idx] * k_path_cost[idx] for idx in range(number_of_xkp)))

        obj = m.addVar(name="obj")

        m.addConstr(obj == self.k_1*queue_cost_sum_first + self.k_2*queue_cost_sum_second + self.k_3*run_cost_sum)

        m.setParam('OutputFlag', 0)

        m.setObjective(obj, GRB.MINIMIZE)

        m.optimize()

        # for v in m.getVars():
        #     print('%s %g' % (v.VarName, v.X))

        all_vars = m.getVars()

        try:
            # 可能会出错的代码段
            values = m.getAttr("X", all_vars)
            self.xkp_res = values[0:number_of_xkp]

            self.del_path_res()
            des_path_node = []
            for path_idx in self.path_sel_res:
                path_node = self.path_info[path_idx]['path_node']
                des_path_node.append(path_node)
        except gp.GurobiError as e:
            # 出错时的处理代码
            print(f"An error occurred: {e}")
            des_path_node = []
        # values = m.getAttr("X", all_vars)
        # names = m.getAttr("VarName", all_vars)
        # print()
        return des_path_node


    def path_selection_only_run_cost(self):
        number_of_k = self.number_of_k

        number_of_first_edge = self.number_of_first_edge
        number_of_second_edge = self.number_of_second_edge

        xkp_first_edge_list = self.xkp_first_edge_list
        xkp_second_edge_list = self.xkp_second_edge_list

        number_of_xkp = self.number_of_xkp
        k_path_cost = self.k_path_cost

        xkp_drone_list = self.xkp_drone_list

        first_edge_capacity = self.first_edge_capacity
        second_edge_capacity = self.second_edge_capacity

        first_edge_current_num = self.first_edge_num
        second_edge_current_num = self.second_edge_num

        k_li = list(range(number_of_k))
        xkp_li = list(range(number_of_xkp))

        first_li = list(range(number_of_first_edge))
        second_li = list(range(number_of_second_edge))

        # Create a new model
        m = gp.Model("mip_path_allocation")
        m.setParam('TimeLimit', 5)
        # m.Params.MIPGap = self.formation_choice_gap

        m.params.NonConvex = 2
        # k类无人机给path上分配的无人机数量 gp.GRB.CONTINUOUS
        x_k_p = m.addVars(xkp_li, vtype=gp.GRB.BINARY, name="x_k_p")
        # x_k_p = m.addVars(p_li, vtype=gp.GRB.CONTINUOUS, name="x_k_p")

        queue_cost_sum_first = m.addVar(name="queue_cost_sum_first")
        queue_cost_sum_second = m.addVar(name="queue_cost_sum_second")
        # queue cost
        queue_cost_first_edge = m.addVars(first_li, name="queue_cost_first_edge")
        queue_cost_second_edge = m.addVars(second_li, name="queue_cost_second_edge")

        first_edge_sum = m.addVars(first_li, name="first_edge_drone_sum")
        second_edge_sum = m.addVars(second_li, name="second_edge_drone_sum")

        xkp_sum = m.addVars(k_li, name="xkp_sum")

        # run_cost = m.addVar(k_li, name="run_cost")

        run_cost_sum = m.addVar(name="run_cost")

        # 找到x_k_p二维矩阵中 有哪些path是经过边first edge的 将它们加起来
        for shared_i in range(number_of_first_edge):
            xkp_shared_idx = xkp_first_edge_list[shared_i]
            m.addConstr(first_edge_sum[shared_i] == gp.quicksum(x_k_p[idx] for idx in xkp_shared_idx))

        # queue cost of first edge
        for shared_i in range(number_of_first_edge):
            m.addConstr(queue_cost_first_edge[shared_i] ==
                        ((first_edge_current_num[shared_i] + first_edge_sum[shared_i] - first_edge_capacity[shared_i]) *
                         (first_edge_current_num[shared_i] + first_edge_sum[shared_i] - first_edge_capacity[shared_i])
                         / (first_edge_capacity[shared_i] * first_edge_capacity[shared_i])))

        # 找到x_k_p二维矩阵中 有哪些path是经过边second edge的 将它们加起来
        for shared_i in range(number_of_second_edge):
            xkp_shared_idx = xkp_second_edge_list[shared_i]
            m.addConstr(second_edge_sum[shared_i] == gp.quicksum(x_k_p[idx] for idx in xkp_shared_idx))

        # queue cost of second edge
        for shared_i in range(number_of_second_edge):
            m.addConstr(queue_cost_second_edge[shared_i] ==
                        ((second_edge_current_num[shared_i] + second_edge_sum[shared_i] - second_edge_capacity[
                            shared_i]) *
                         (second_edge_current_num[shared_i] + second_edge_sum[shared_i] - second_edge_capacity[
                             shared_i])
                         / (second_edge_capacity[shared_i] * second_edge_capacity[shared_i])))

        for k_i in range(number_of_k):
            xkp_list = xkp_drone_list[k_i]
            m.addConstr(xkp_sum[k_i] == gp.quicksum(x_k_p[idx] for idx in xkp_list))

        for k_i in range(number_of_k):
            m.addConstr(xkp_sum[k_i] == 1)

        m.addConstr(
            queue_cost_sum_first == gp.quicksum(
                queue_cost_first_edge[shared_i] for shared_i in range(number_of_first_edge)))

        m.addConstr(
            queue_cost_sum_second == gp.quicksum(
                queue_cost_second_edge[shared_i] for shared_i in range(number_of_second_edge)))

        # m.addConstr(number_of_k == gp.quicksum(x_k_p[idx] for idx in range(number_of_xkp)))

        m.addConstr(run_cost_sum == gp.quicksum(x_k_p[idx] * k_path_cost[idx] for idx in range(number_of_xkp)))

        obj = m.addVar(name="obj")

        m.addConstr(obj == run_cost_sum)

        m.setParam('OutputFlag', 0)

        m.setObjective(obj, GRB.MINIMIZE)

        m.optimize()

        # for v in m.getVars():
        #     print('%s %g' % (v.VarName, v.X))

        all_vars = m.getVars()

        try:
            # 可能会出错的代码段
            values = m.getAttr("X", all_vars)
            self.xkp_res = values[0:number_of_xkp]

            self.del_path_res()
            des_path_node = []
            for path_idx in self.path_sel_res:
                path_node = self.path_info[path_idx]['path_node']
                des_path_node.append(path_node)
        except gp.GurobiError as e:
            # 出错时的处理代码
            print(f"An error occurred: {e}")
            des_path_node = []

        # print()
        return des_path_node

    def path_selection_only_queue_cost(self):
        number_of_k = self.number_of_k

        number_of_first_edge = self.number_of_first_edge
        number_of_second_edge = self.number_of_second_edge

        xkp_first_edge_list = self.xkp_first_edge_list
        xkp_second_edge_list = self.xkp_second_edge_list

        number_of_xkp = self.number_of_xkp
        k_path_cost = self.k_path_cost

        xkp_drone_list = self.xkp_drone_list

        first_edge_capacity = self.first_edge_capacity
        second_edge_capacity = self.second_edge_capacity

        first_edge_current_num = self.first_edge_num
        second_edge_current_num = self.second_edge_num

        k_li = list(range(number_of_k))
        xkp_li = list(range(number_of_xkp))

        first_li = list(range(number_of_first_edge))
        second_li = list(range(number_of_second_edge))

        # Create a new model
        m = gp.Model("mip_path_allocation")
        m.setParam('TimeLimit', 5)
        # m.Params.MIPGap = self.formation_choice_gap

        m.params.NonConvex = 2
        # k类无人机给path上分配的无人机数量 gp.GRB.CONTINUOUS
        x_k_p = m.addVars(xkp_li, vtype=gp.GRB.BINARY, name="x_k_p")
        # x_k_p = m.addVars(p_li, vtype=gp.GRB.CONTINUOUS, name="x_k_p")

        queue_cost_sum_first = m.addVar(name="queue_cost_sum_first")
        queue_cost_sum_second = m.addVar(name="queue_cost_sum_second")
        # queue cost
        queue_cost_first_edge = m.addVars(first_li, name="queue_cost_first_edge")
        queue_cost_second_edge = m.addVars(second_li, name="queue_cost_second_edge")

        first_edge_sum = m.addVars(first_li, name="first_edge_drone_sum")
        second_edge_sum = m.addVars(second_li, name="second_edge_drone_sum")

        xkp_sum = m.addVars(k_li, name="xkp_sum")

        # run_cost = m.addVar(k_li, name="run_cost")

        run_cost_sum = m.addVar(name="run_cost")

        # 找到x_k_p二维矩阵中 有哪些path是经过边first edge的 将它们加起来
        for shared_i in range(number_of_first_edge):
            xkp_shared_idx = xkp_first_edge_list[shared_i]
            m.addConstr(first_edge_sum[shared_i] == gp.quicksum(x_k_p[idx] for idx in xkp_shared_idx))

        # queue cost of first edge
        for shared_i in range(number_of_first_edge):
            m.addConstr(queue_cost_first_edge[shared_i] ==
                        ((first_edge_current_num[shared_i] + first_edge_sum[shared_i] - first_edge_capacity[shared_i]) *
                         (first_edge_current_num[shared_i] + first_edge_sum[shared_i] - first_edge_capacity[shared_i])
                         / (first_edge_capacity[shared_i] * first_edge_capacity[shared_i])))

        # 找到x_k_p二维矩阵中 有哪些path是经过边second edge的 将它们加起来
        for shared_i in range(number_of_second_edge):
            xkp_shared_idx = xkp_second_edge_list[shared_i]
            m.addConstr(second_edge_sum[shared_i] == gp.quicksum(x_k_p[idx] for idx in xkp_shared_idx))

        # queue cost of second edge
        for shared_i in range(number_of_second_edge):
            m.addConstr(queue_cost_second_edge[shared_i] ==
                        ((second_edge_current_num[shared_i] + second_edge_sum[shared_i] - second_edge_capacity[
                            shared_i]) *
                         (second_edge_current_num[shared_i] + second_edge_sum[shared_i] - second_edge_capacity[
                             shared_i])
                         / (second_edge_capacity[shared_i] * second_edge_capacity[shared_i])))

        for k_i in range(number_of_k):
            xkp_list = xkp_drone_list[k_i]
            m.addConstr(xkp_sum[k_i] == gp.quicksum(x_k_p[idx] for idx in xkp_list))

        for k_i in range(number_of_k):
            m.addConstr(xkp_sum[k_i] == 1)

        m.addConstr(
            queue_cost_sum_first == gp.quicksum(
                queue_cost_first_edge[shared_i] for shared_i in range(number_of_first_edge)))

        m.addConstr(
            queue_cost_sum_second == gp.quicksum(
                queue_cost_second_edge[shared_i] for shared_i in range(number_of_second_edge)))

        # m.addConstr(number_of_k == gp.quicksum(x_k_p[idx] for idx in range(number_of_xkp)))

        m.addConstr(run_cost_sum == gp.quicksum(x_k_p[idx] * k_path_cost[idx] for idx in range(number_of_xkp)))

        obj = m.addVar(name="obj")

        m.addConstr(obj == run_cost_sum*0.01 + queue_cost_sum_first + queue_cost_sum_second * 0.5)

        m.setParam('OutputFlag', 0)

        m.setObjective(obj, GRB.MINIMIZE)

        m.optimize()

        # for v in m.getVars():
        #     print('%s %g' % (v.VarName, v.X))

        all_vars = m.getVars()

        try:
            # 可能会出错的代码段
            values = m.getAttr("X", all_vars)
            self.xkp_res = values[0:number_of_xkp]

            self.del_path_res()
            des_path_node = []
            for path_idx in self.path_sel_res:
                path_node = self.path_info[path_idx]['path_node']
                des_path_node.append(path_node)
        except gp.GurobiError as e:
            # 出错时的处理代码
            print(f"An error occurred: {e}")
            des_path_node = []

        # print()
        return des_path_node

    def del_path_res(self):
        # 处理xkp的结果
        self.path_sel_res = []
        for i in range(len(self.xkp_res)):
            if abs(self.xkp_res[i] - 1) <= 0.001:
                self.path_sel_res.append(i)

