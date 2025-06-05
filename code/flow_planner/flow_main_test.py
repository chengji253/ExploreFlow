import gurobipy as gp
from gurobipy import GRB


# water flow planner main module

class Flow_planner:

    def __init__(self):
        # 包含节点和边的N
        self.network = None
        self.x_e_result = None

    def swarm_allocation(self):
        # 每一个边无人机集群数量的分配
        pass

    def cell_posiiton_allocation(self):
        # 为进入集群的无人机分配要出去的位置
        pass

    def main(self):
        number_of_k = 2
        number_of_path = 3 + 2
        number_of_cut = 3
        k_drones_sum = [50, 50]
        r_min = 2
        v = 1
        # k无人机对应的path idx
        k_path_idx = [[0, 1, 2], [3, 4]]
        # k_path_cost = [15, 10, 15, 10, 15]
        k_path_cost = [10, 9, 10, 9, 10]
        # 经过边e的xkp的index是哪些
        xkp_cut_list = [[0], [1, 3], [2, 4]]
        cut_edge_idx = [0, 1, 2]
        cut_edge_capacity = [10, 1, 10]

        k_li = list(range(number_of_k))
        p_li = list(range(number_of_path))
        cut_li = list(range(number_of_cut))

        # Create a new model
        m = gp.Model("mip_path_allocation")
        # m.setParam('TimeLimit', 2 * 60)
        # m.Params.MIPGap = self.formation_choice_gap

        # k类无人机给path上分配的无人机数量 gp.GRB.CONTINUOUS
        x_k_p = m.addVars(p_li, vtype=gp.GRB.INTEGER, name="x_k_p")
        # x_k_p = m.addVars(p_li, vtype=gp.GRB.CONTINUOUS, name="x_k_p")

        running_cost = m.addVar(name="running_cost")

        path_sum_k = m.addVars(k_li, name="path_sum_k")

        queue_cost = m.addVar(name="queue_cost")
        # queue cost
        queue_cost_cut = m.addVars(cut_li, name="queue_cost_cut", lb=-gp.GRB.INFINITY, ub=gp.GRB.INFINITY)

        cut_edge_drone_sum = m.addVars(cut_li, name="cut_edge_drone_sum")

        # 每k类无人机的path running cost
        for k_i in range(number_of_k):
            k_idx_range = k_path_idx[k_i]
            m.addConstr(path_sum_k[k_i] == gp.quicksum(k_path_cost[idx] * x_k_p[idx] for idx in k_idx_range))

        # running cost of all k drones
        m.addConstr(running_cost == (gp.quicksum(path_sum_k[k_i] for k_i in range(number_of_k))) / v)

        # 找到x_k_p二维矩阵中 有哪些path是经过边e的 将它们加起来
        for cut_i in range(number_of_cut):
            xkp_cut_idx = xkp_cut_list[cut_i]
            m.addConstr(cut_edge_drone_sum[cut_i] == gp.quicksum(x_k_p[idx] for idx in xkp_cut_idx))

        # queue cost of cut edge
        m.addConstr(queue_cost == gp.quicksum(
            ((cut_edge_drone_sum[cut_i]) * r_min / (cut_edge_capacity[cut_i] * v))
            for cut_i in range(number_of_cut)))

        for cut_i in range(number_of_cut):
            m.addConstr(queue_cost_cut[cut_i] ==
                        (cut_edge_drone_sum[cut_i]) * r_min / (cut_edge_capacity[cut_i] * v))

        for k_i in range(number_of_k):
            k_idx_range = k_path_idx[k_i]
            m.addConstr(k_drones_sum[k_i] == gp.quicksum(x_k_p[idx] for idx in k_idx_range))

        obj = m.addVar(name="obj")

        m.addConstr(obj == running_cost + queue_cost)
        # m.addConstr(obj == queue_cost)

        m.setParam('OutputFlag', 0)

        m.setObjective(obj, GRB.MINIMIZE)

        m.optimize()

        for v in m.getVars():
            print('%s %g' % (v.VarName, v.X))

        all_vars = m.getVars()
        values = m.getAttr("X", all_vars)
        # names = m.getAttr("VarName", all_vars)

    def main_one(self):
        number_of_k = 1
        number_of_path = 3
        number_of_cut = 3
        k_drones_sum = [50]
        r_min = 2
        v = 1

        # k无人机对应的path idx
        k_path_idx = [[0, 1, 2]]
        # k_path_cost = [15, 10, 15, 10, 15]
        k_path_cost = [10, 10, 10]

        # 经过边e的xkp的index是哪些
        xkp_cut_list = [[0], [1], [2]]
        cut_edge_idx = [0, 1, 2]
        cut_edge_capacity = [10, 1, 15]

        k_li = list(range(number_of_k))
        p_li = list(range(number_of_path))
        cut_li = list(range(number_of_cut))

        # Create a new model
        m = gp.Model("mip_path_allocation")
        # m.setParam('TimeLimit', 2 * 60)
        # m.Params.MIPGap = self.formation_choice_gap

        m.params.NonConvex = 2
        # k类无人机给path上分配的无人机数量 gp.GRB.CONTINUOUS
        x_k_p = m.addVars(p_li, vtype=gp.GRB.INTEGER, name="x_k_p")
        # x_k_p = m.addVars(p_li, vtype=gp.GRB.CONTINUOUS, name="x_k_p")

        running_cost = m.addVar(name="running_cost")

        path_sum_k = m.addVars(k_li, name="path_sum_k")

        queue_cost = m.addVar(name="queue_cost")
        # queue cost
        queue_cost_cut = m.addVars(cut_li, name="queue_cost_cut", lb=-gp.GRB.INFINITY, ub=gp.GRB.INFINITY)

        cut_edge_drone_sum = m.addVars(cut_li, name="cut_edge_drone_sum")

        # 每k类无人机的path running cost
        for k_i in range(number_of_k):
            k_idx_range = k_path_idx[k_i]
            m.addConstr(path_sum_k[k_i] == gp.quicksum(k_path_cost[idx] * x_k_p[idx] for idx in k_idx_range))

        # running cost of all k drones
        m.addConstr(running_cost == (gp.quicksum(path_sum_k[k_i] for k_i in range(number_of_k))) / v)

        # 找到x_k_p二维矩阵中 有哪些path是经过边e的 将它们加起来
        for cut_i in range(number_of_cut):
            xkp_cut_idx = xkp_cut_list[cut_i]
            m.addConstr(cut_edge_drone_sum[cut_i] == gp.quicksum(x_k_p[idx] for idx in xkp_cut_idx))

        # queue cost of cut edge
        for cut_i in range(number_of_cut):
            m.addConstr(queue_cost_cut[cut_i] ==
                        (cut_edge_drone_sum[cut_i] - cut_edge_capacity[cut_i]) * (
                                    cut_edge_drone_sum[cut_i] - cut_edge_capacity[cut_i])
                        * r_min / (cut_edge_capacity[cut_i] * v))

        m.addConstr(queue_cost == gp.quicksum(queue_cost_cut[cut_i] for cut_i in range(number_of_cut)))

        for k_i in range(number_of_k):
            k_idx_range = k_path_idx[k_i]
            m.addConstr(k_drones_sum[k_i] == gp.quicksum(x_k_p[idx] for idx in k_idx_range))

        obj = m.addVar(name="obj")

        m.addConstr(obj == running_cost + queue_cost)
        # m.addConstr(obj == queue_cost)

        m.setParam('OutputFlag', 0)

        m.setObjective(obj, GRB.MINIMIZE)

        m.optimize()

        for v in m.getVars():
            print('%s %g' % (v.VarName, v.X))

        all_vars = m.getVars()
        values = m.getAttr("X", all_vars)
        # names = m.getAttr("VarName", all_vars)

    def main_3(self):
        number_of_k = 2
        number_of_path = 3 + 2
        number_of_cut = 3
        k_drones_sum = [50, 50]
        number_of_drones = sum(k_drones_sum)
        r_min = 2
        v = 1
        # k无人机对应的path idx
        k_path_idx = [[0, 1, 2], [3, 4]]
        # k_path_cost = [15, 10, 15, 10, 15]
        k_path_cost = [10, 4, 10, 8, 10]
        # 经过边e的xkp的index是哪些
        xkp_cut_list = [[0], [1, 3], [2, 4]]
        cut_edge_idx = [0, 1, 2]
        cut_edge_capacity = [10, 5, 10]

        # 排队权重
        alpha = 2

        k_li = list(range(number_of_k))
        p_li = list(range(number_of_path))
        cut_li = list(range(number_of_cut))

        # Create a new model
        m = gp.Model("mip_path_allocation")
        # m.setParam('TimeLimit', 2 * 60)
        # m.Params.MIPGap = self.formation_choice_gap

        # m.params.NonConvex = 2
        # k类无人机给path上分配的无人机数量 gp.GRB.CONTINUOUS
        x_k_p = m.addVars(p_li, vtype=gp.GRB.INTEGER, name="x_k_p")
        # x_k_p = m.addVars(p_li, vtype=gp.GRB.CONTINUOUS, name="x_k_p")

        running_cost = m.addVar(name="running_cost")

        path_sum_k = m.addVars(k_li, name="path_sum_k")

        queue_cost = m.addVar(name="queue_cost")
        # queue cost
        queue_cost_cut = m.addVars(cut_li, name="queue_cost_cut", lb=-gp.GRB.INFINITY, ub=gp.GRB.INFINITY)

        queue_cost_cut_m = m.addVars(cut_li, name="queue_cost_cut_m")

        cut_edge_drone_sum = m.addVars(cut_li, name="cut_edge_drone_sum")

        # 每k类无人机的path running cost
        for k_i in range(number_of_k):
            k_idx_range = k_path_idx[k_i]
            m.addConstr(path_sum_k[k_i] == gp.quicksum(k_path_cost[idx] * x_k_p[idx] for idx in k_idx_range))

        # running cost of all k drones
        m.addConstr(running_cost == (gp.quicksum(path_sum_k[k_i] for k_i in range(number_of_k))) / v/number_of_drones)

        # 找到x_k_p二维矩阵中 有哪些path是经过边e的 将它们加起来
        for cut_i in range(number_of_cut):
            xkp_cut_idx = xkp_cut_list[cut_i]
            m.addConstr(cut_edge_drone_sum[cut_i] == gp.quicksum(x_k_p[idx] for idx in xkp_cut_idx))

        # queue cost of cut edge
        for cut_i in range(number_of_cut):
            p_idx_list = xkp_cut_list[cut_i]
            ave_running_cost = 0
            for p_idx in p_idx_list:
                ave_running_cost += k_path_cost[p_idx]
            ave_running_cost = ave_running_cost/len(p_idx_list)
            m.addConstr(queue_cost_cut[cut_i] ==
                ((cut_edge_drone_sum[cut_i] - cut_edge_capacity[cut_i]) * r_min / (cut_edge_capacity[cut_i] + ave_running_cost)))

        for cut_i in range(number_of_cut):
            m.addConstr(queue_cost_cut_m[cut_i] == gp.max_(queue_cost_cut[cut_i], 0))

        m.addConstr(queue_cost == gp.quicksum(queue_cost_cut_m[cut_i] for cut_i in range(number_of_cut)))

        for k_i in range(number_of_k):
            k_idx_range = k_path_idx[k_i]
            m.addConstr(k_drones_sum[k_i] == gp.quicksum(x_k_p[idx] for idx in k_idx_range))

        obj = m.addVar(name="obj")

        m.addConstr(obj == running_cost + queue_cost)
        # m.addConstr(obj == queue_cost)

        m.setParam('OutputFlag', 0)

        m.setObjective(obj, GRB.MINIMIZE)

        m.optimize()

        for v in m.getVars():
            print('%s %g' % (v.VarName, v.X))

        all_vars = m.getVars()
        values = m.getAttr("X", all_vars)
        # names = m.getAttr("VarName", all_vars)

    def main_4(self):
        number_of_k = 1
        number_of_path = 3
        number_of_cut = 3
        k_drones_sum = [100]
        number_of_drones = sum(k_drones_sum)
        r_min = 2
        v = 1
        # k无人机对应的path idx
        k_path_idx = [[0, 1, 2]]
        # k_path_cost = [15, 10, 15, 10, 15]
        k_path_cost = [10, 8, 15]

        # 经过边e的xkp的index是哪些
        xkp_cut_list = [[0], [1], [2]]
        cut_edge_idx = [0, 1, 2]
        cut_edge_capacity = [10, 1, 15]

        k_li = list(range(number_of_k))
        p_li = list(range(number_of_path))
        cut_li = list(range(number_of_cut))

        # Create a new model
        m = gp.Model("mip_path_allocation")
        # m.setParam('TimeLimit', 2 * 60)
        # m.Params.MIPGap = self.formation_choice_gap

        # m.params.NonConvex = 2
        # k类无人机给path上分配的无人机数量 gp.GRB.CONTINUOUS
        x_k_p = m.addVars(p_li, vtype=gp.GRB.INTEGER, name="x_k_p")
        # x_k_p = m.addVars(p_li, vtype=gp.GRB.CONTINUOUS, name="x_k_p")

        running_cost = m.addVar(name="running_cost")

        path_sum_cut = m.addVars(cut_li, name="path_sum_k")

        queue_cost = m.addVar(name="queue_cost")
        # queue cost
        queue_cost_cut = m.addVars(cut_li, name="queue_cost_cut", lb=-gp.GRB.INFINITY, ub=gp.GRB.INFINITY)

        queue_cost_cut_m = m.addVars(cut_li, name="queue_cost_cut_m")

        cut_edge_drone_sum = m.addVars(cut_li, name="cut_edge_drone_sum")

        # 每个cut边的running cost
        for cut_i in range(number_of_cut):
            xkp_cut = xkp_cut_list[cut_i]
            p_s = len(xkp_cut)
            m.addConstr(path_sum_cut[cut_i] == gp.quicksum(k_path_cost[idx] * x_k_p[idx] for idx in xkp_cut)/p_s)

        # running cost of all k drones
        m.addConstr(running_cost == (gp.quicksum(path_sum_cut[cut_i] for cut_i in range(number_of_cut))) / v/number_of_drones)

        # 找到x_k_p二维矩阵中 有哪些path是经过边e的 将它们加起来
        for cut_i in range(number_of_cut):
            xkp_cut_idx = xkp_cut_list[cut_i]
            m.addConstr(cut_edge_drone_sum[cut_i] == gp.quicksum(x_k_p[idx] for idx in xkp_cut_idx))

        # queue cost of cut edge
        for cut_i in range(number_of_cut):
            p_idx_list = xkp_cut_list[cut_i]
            ave_running_cost = 0
            for p_idx in p_idx_list:
                ave_running_cost += k_path_cost[p_idx]
            ave_running_cost = ave_running_cost / len(p_idx_list)
            m.addConstr(queue_cost_cut[cut_i] ==
                        ((cut_edge_drone_sum[cut_i] - cut_edge_capacity[cut_i]) * r_min / (
                                    cut_edge_capacity[cut_i] + ave_running_cost)))

        for cut_i in range(number_of_cut):
            m.addConstr(queue_cost_cut_m[cut_i] == gp.max_(queue_cost_cut[cut_i], 0))

        m.addConstr(queue_cost == gp.quicksum(queue_cost_cut_m[cut_i] for cut_i in range(number_of_cut)))

        for k_i in range(number_of_k):
            k_idx_range = k_path_idx[k_i]
            m.addConstr(k_drones_sum[k_i] == gp.quicksum(x_k_p[idx] for idx in k_idx_range))

        obj = m.addVar(name="obj")

        m.addConstr(obj == running_cost + queue_cost)
        # m.addConstr(obj == queue_cost)

        m.setParam('OutputFlag', 0)

        m.setObjective(obj, GRB.MINIMIZE)

        m.optimize()

        for v in m.getVars():
            print('%s %g' % (v.VarName, v.X))

        all_vars = m.getVars()
        values = m.getAttr("X", all_vars)
        # names = m.getAttr("VarName", all_vars)

    def main_norm(self):
        number_of_k = 1
        number_of_path = 3
        number_of_cut = 3
        k_drones_sum = [100]
        v = 1
        # k无人机对应的path idx
        k_path_idx = [[0, 1, 2]]
        # k_path_cost = [15, 10, 15, 10, 15]
        k_path_cost = [10, 8, 15]

        # 经过边e的xkp的index是哪些
        xkp_cut_list = [[0], [1], [2]]
        cut_edge_idx = [0, 1, 2]
        cut_edge_capacity = [10, 2, 15]

        k_li = list(range(number_of_k))
        p_li = list(range(number_of_path))
        cut_li = list(range(number_of_cut))

        # Create a new model
        m = gp.Model("mip_path_allocation")
        # m.setParam('TimeLimit', 2 * 60)
        # m.Params.MIPGap = self.formation_choice_gap

        m.params.NonConvex = 2
        # k类无人机给path上分配的无人机数量 gp.GRB.CONTINUOUS
        x_k_p = m.addVars(p_li, vtype=gp.GRB.INTEGER, name="x_k_p")
        # x_k_p = m.addVars(p_li, vtype=gp.GRB.CONTINUOUS, name="x_k_p")

        queue_cost = m.addVar(name="queue_cost")
        # queue cost
        queue_cost_cut = m.addVars(cut_li, name="queue_cost_cut", lb=-gp.GRB.INFINITY, ub=gp.GRB.INFINITY)

        cut_edge_drone_sum = m.addVars(cut_li, name="cut_edge_drone_sum")

        # 找到x_k_p二维矩阵中 有哪些path是经过边e的 将它们加起来
        for cut_i in range(number_of_cut):
            xkp_cut_idx = xkp_cut_list[cut_i]
            m.addConstr(cut_edge_drone_sum[cut_i] == gp.quicksum(x_k_p[idx] for idx in xkp_cut_idx))

        # queue cost of cut edge
        for cut_i in range(number_of_cut):
            p_idx_list = xkp_cut_list[cut_i]
            ave_running_cost = 0
            for p_idx in p_idx_list:
                ave_running_cost += k_path_cost[p_idx]
            ave_running_cost = ave_running_cost / len(p_idx_list)
            m.addConstr(queue_cost_cut[cut_i] ==
            ((cut_edge_drone_sum[cut_i] - cut_edge_capacity[cut_i]) * (cut_edge_drone_sum[cut_i] - cut_edge_capacity[cut_i])
                         * ave_running_cost / cut_edge_capacity[cut_i] ))

        m.addConstr(queue_cost == gp.quicksum(queue_cost_cut[cut_i] for cut_i in range(number_of_cut)))

        for k_i in range(number_of_k):
            k_idx_range = k_path_idx[k_i]
            m.addConstr(k_drones_sum[k_i] == gp.quicksum(x_k_p[idx] for idx in k_idx_range))

        obj = m.addVar(name="obj")

        # m.addConstr(obj == running_cost + queue_cost)
        m.addConstr(obj == queue_cost)

        m.setParam('OutputFlag', 0)

        m.setObjective(obj, GRB.MINIMIZE)

        m.optimize()

        for v in m.getVars():
            print('%s %g' % (v.VarName, v.X))

        all_vars = m.getVars()
        values = m.getAttr("X", all_vars)
        # names = m.getAttr("VarName", all_vars)


f = Flow_planner()
f.main_norm()