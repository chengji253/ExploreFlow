import random
import time
import os
from ORCA.Vector2 import *
from ORCA.ORCA_Simulator import *
from ORCA.plotPic import *
import os
import time
import json
from map_info import map_info
from flow_planner import flow_main
from flow_planner import max_flow
from flow_planner import flow_area
from visualize import plotPic
import logging
import colorlog
from para import swarm


# 控制是否输出时间和位置
RVO_OUTPUT_TIME_AND_POSITIONS = 1
# 定义常量
RVO_TWO_PI = 2 * math.pi


class Simu_ORCA_flow:

    def __init__(self, str_now):
        # self.pic_name = "pic/map_yaml/m1-10-corridor.yaml"
        # self.pic_name = "pic/map_yaml/m1-500-maze-3.yaml"
        # self.pic_name = "pic/map_yaml/m1-500-forest-n.yaml"
        self.pic_name = "pic/map_yaml/m1-500-hard.yaml"

        self.file_main = "result/test_flow_" + str_now

        self.simulator = ORCA_Simulator()

        self.boundary = None
        self.name = None
        # 机器人需要经过的所有path path 给到之后 需要反向处理一下
        self.path_all = []

        self.reach_dis_goal = None
        self.reach_dis_inter = None

        self.mip_flow_planner = None

        self.max_flow_planner = None

        self.area_flow_planner = None

        self.mapInfo = None

        self.swarm = swarm.Swarm(self.pic_name)

        self.resolution = 1

        self.flow_main_time = []

        self.pic_time_gap = 10

        self.max_density_list = []
        self.mean_density_list = []
        self.non_zero_mean_density = []
        self.num_wait_red_list = []

    def setup_scenario(self):
        # 设定全局时间步长
        self.simulator.setTimeStep(0.1)

        self.boundary = [[0, self.mapInfo.x_size], [0, self.mapInfo.y_size]]
        # self.boundary = [[-210, 210], [-210, 210]]

        self.reach_dis_goal = 0.5
        self.reach_dis_inter = 0.2

        self.name = self.file_main + "/snap" + str(self.simulator.getGlobalTime()) + ".png"

        # 设定后续添加的代理的默认参数
        # neighborDist, maxNeighbors, timeHorizon, timeHorizonObst, radius, maxSpeed,
        # self.simulator.setAgentDefaults(30.0, 20, 20.0, 20.0, 1.5, 5.0)
        self.simulator.setAgentDefaults(3.0, 20, 5.0, 5.0, 0.4, 5.0)
        # self.simulator.setAgentDefaults(30.0, 20, 20.0, 20.0, 0.4, 5.0)


    def init_simulation_area_planner(self, L_predictive, K_horizon):
        # self.title_name = 'data/' + self.add_name + '/para_2'
        self.swarm.init_swarm()
        self.swarm.compute_ave_start_goal_pos()
        self.mapInfo = map_info.MapInfo(self.swarm, self.pic_name, self.resolution)
        self.mapInfo.init_main_yaml()
        # self.pic_name = 'pic/new/maze2.png'
        # self.pic_name = 'pic/new/c1.png'
        # self.pic_name = 'pic/new/c2.png'

        # self.title_name = self.title_name + "flow"
        self.setup_scenario()
        self.set_agents()
        self.set_obstacles()
        # self.set_path_pos()
        self.simulator.init_simulator()

        self.init_state()
        self.update_state()

        self.area_flow_planner = flow_area.Flow_area_planner(self.swarm, self.mapInfo, L_predictive, K_horizon)
        # self.area_flow_planner.predictive_len_limit = L_predictive
        # self.area_flow_planner.time_horizon = K_horizon

        self.swarm.set_start_goal_node(self.mapInfo.start_idx, self.mapInfo.end_idx, 0, self.mapInfo.number_cells - 1)

        # self.mapInfo.draw_mapInfo()
        self.flow_area_planner_run()
        # self.planner_time += t4 - t3
        # self.planner_run_num += 1
        # self.t_r_flow_list.append(t_r_flow)
        # logging.info("mapInfo_time=" + str(t2 - t1) + "s")
        # logging.info("mapCo_time=" + str(t3 - t2) + "s")
        # logging.info("flow_planner_first=" + str(t4 - t3) + "s")

    def init_simulation_flow_planner(self):
        # self.title_name = 'data/' + self.add_name + '/para_2'
        self.swarm.init_swarm()
        self.swarm.compute_ave_start_goal_pos()
        self.pic_name = "pic/map_yaml/m1-500-hard.yaml"
        self.mapInfo = map_info.MapInfo(self.swarm, self.pic_name, self.resolution)
        self.mapInfo.init_main_yaml()
        # self.pic_name = 'pic/new/maze2.png'
        # self.pic_name = 'pic/new/c1.png'
        # self.pic_name = 'pic/new/c2.png'

        # self.title_name = self.title_name + "flow"
        self.setup_scenario()
        self.set_agents()
        self.set_obstacles()
        # self.set_path_pos()
        self.simulator.init_simulator()

        self.init_state()
        self.update_state()

        # self.max_flow_planner = max_flow.Max_Flow_planner(self.swarm, self.mapInfo)
        # self.max_flow_planner.run()

        self.mip_flow_planner = flow_main.Flow_planner(self.swarm, self.mapInfo)
        self.mip_flow_planner.k_1 = 1
        self.mip_flow_planner.k_2 = 0.5
        self.mip_flow_planner.k_3 = 5

        self.swarm.set_start_goal_node(self.mapInfo.start_idx, self.mapInfo.end_idx, 0, self.mapInfo.number_cells - 1)
        # self.mapInfo.draw_mapInfo()

        t_r_flow = self.flow_planner_run()

        # self.planner_time += t4 - t3
        # self.planner_run_num += 1
        # self.t_r_flow_list.append(t_r_flow)
        # logging.info("mapInfo_time=" + str(t2 - t1) + "s")
        # logging.info("mapCo_time=" + str(t3 - t2) + "s")
        # logging.info("flow_planner_first=" + str(t4 - t3) + "s")

    def flow_area_planner_run(self):
        tp1 = time.time()
        self.area_flow_planner.global_scheduler_run()
        des_path_node = self.area_flow_planner.des_path_node

        tp2 = time.time()
        if len(des_path_node) != 0:
            if self.swarm.des_path_node is None:
                self.swarm.des_path_node = des_path_node
            else:
                for i in range(self.swarm.robots_num):
                    des_n = des_path_node[i]
                    if len(des_n) != 0:
                        self.swarm.des_path_node[i] = des_path_node[i]

            # self.area_flow_planner.local_path_choose()
            self.set_center_pos()
            self.area_flow_planner.local_scheduler_run()
            # self.area_flow_planner.position_allocation_load_balance()
            # self.area_flow_planner.position_allocation_greedy_now()
            self.swarm.para_set_planner()
        tp3 = time.time()
        # t_r = [tp2 - tp1, tp3 - tp2]
        # logging.info("area_flow_planner run =" + str(tp2 - tp1) + "s")
        print("area_flow_planner run =" + str(tp2 - tp1) + "s")
        print("local path allocation =" + str(tp3 - tp2) + "s")
        print("flow main=" + str(tp3 - tp1))
        self.flow_main_time.append(tp3 - tp1)
        # logging.info("Path node selection =" + str(tp3 - tp2) + "s")


    def flow_planner_run(self):
        tp1 = time.time()
        # self.mapInfo.find_current_path_set_cell()
        self.mapInfo.find_current_path_set_cell_neighbor()

        self.mapInfo.shared_edge_to_path()
        self.mip_flow_planner.compute_num_on_edge()
        self.mip_flow_planner.init_para_shared_edge()
        tp2 = time.time()

        des_path_node = self.mip_flow_planner.path_selection_out_edge()

        tp3 = time.time()
        if len(des_path_node) != 0:
            self.swarm.des_path_node = des_path_node
            self.set_center_pos()
            self.mip_flow_planner.position_allocation_greedy_now()
            self.swarm.para_set_planner()
        tp4 = time.time()

        t_r = [tp2 - tp1, tp3 - tp2, tp4 - tp3]
        logging.info("Path set search =" + str(tp2 - tp1) + "s")
        logging.info("Path node selection =" + str(tp3 - tp2) + "s")
        logging.info("Position_allocation =" + str(tp4 - tp3) + "s")
        logging.info("flow planner =" + str(tp4 - tp1) + "s")
        return t_r

    def set_center_pos(self):
        des_path_pos = []
        for i in range(self.swarm.robots_num):
            path_node = self.swarm.des_path_node[i]
            pos_list = []
            for node in path_node:
                if node == self.swarm.goal_node[i]:
                    pos = self.swarm.goal_pos[i]
                elif node == self.swarm.start_node[i]:
                    pos = self.mapInfo.node_start_end[node]['pos']
                else:
                    pos = self.mapInfo.node_all_adj[node]['pos']
                    pos = [pos[0], pos[1]]
                pos_list.append(pos)
            des_path_pos.append(pos_list)
        self.swarm.des_path_pos = des_path_pos


    def is_overlapping(self, new_obstacle, existing_obstacles):
        new_x_min = min([p.x_ for p in new_obstacle])
        new_x_max = max([p.x_ for p in new_obstacle])
        new_y_min = min([p.y_ for p in new_obstacle])
        new_y_max = max([p.y_ for p in new_obstacle])

        for obstacle in existing_obstacles:
            x_min = min([p.x_ for p in obstacle])
            x_max = max([p.x_ for p in obstacle])
            y_min = min([p.y_ for p in obstacle])
            y_max = max([p.y_ for p in obstacle])

            if not (new_x_max < x_min or new_x_min > x_max or new_y_max < y_min or new_y_min > y_max):
                return True
        return False

    def set_agents(self):
        goals = []
        # self.simulator.addAgent(Vector2(30.0, 30.0))
        # goals.append(Vector2(30.0, 100.0))
        # self.simulator.goals_info = goals
        #
        # self.path_all = [[Vector2(30.0, 100.0)]]
        for i in range(len(self.swarm.pos_all)):
            pos_n = self.swarm.pos_all[i]
            goal = self.swarm.goal_pos[i]
            agent = Vector2(pos_n[0], pos_n[1])
            goal = Vector2(goal[0], goal[1])
            goals.append(goal)
            self.simulator.addAgent(agent)

        self.simulator.goals_info = goals

    def set_path_pos(self):
        for path_pos in self.swarm.des_path_pos:
            path_list = []
            for pos in path_pos:
                v_n = Vector2(pos[0], pos[1])
                path_list.append(v_n)
            self.path_all.append(path_list)

        # print()
        for sub_list in self.path_all:
            sub_list.reverse()

    def set_obstacles(self):
        for obs_v in self.mapInfo.obs_vertices:
            new_obstacle = []
            for obs in obs_v:
                x = obs[0]
                y = obs[1]
                new_obstacle.append(Vector2(x, y))
            self.simulator.obs_info.append(new_obstacle)
            self.simulator.addObstacle(new_obstacle)

        self.simulator.obs_info_deal()
        self.simulator.processObstacles()

    def set_path_init(self):
        pass

    def update_visualization(self, t):
        # 输出当前全局时间
        # print("time=" + str(simulator.getGlobalTime()))
        time = self.simulator.getGlobalTime()
        self.name = self.file_main + "/snap" + str(t) + ".png"
        visualize_traj_dynamic(self.simulator.agents_, self.simulator.obs_info, self.simulator.goals_info,
                               self.boundary, self.name, self.swarm, time)
        # num_wait_red = visualize_traj_wait_red_robot(self.simulator.agents_, self.simulator.obs_info, self.simulator.goals_info,
        #                        self.boundary, self.name, self.swarm, time)
        # self.num_wait_red_list.append(num_wait_red)
        # max_density, mean_density, non_zero_mean_density \
        #     = visualize_traj_dynamic_hot_map(self.simulator.agents_,
        #         self.simulator.obs_info, self.simulator.goals_info, self.boundary, self.name, self.swarm, time)
        # # max_density, mean_density, non_zero_mean_density = only_hot_map(self.simulator.agents_,
        # #         self.simulator.obs_info, self.simulator.goals_info, self.boundary, self.name, self.swarm, time)
        # self.max_density_list.append(max_density)
        # self.mean_density_list.append(mean_density)
        # self.non_zero_mean_density.append(non_zero_mean_density)


    def set_preferred_velocities(self):
        # 为每个代理设置首选速度
        for i in range(self.simulator.getNumAgents()):
            # 判断是否到了path的最后一个vector 到了就pop掉
            if self.simulator.agents_[i].agent_exist_ is False:
                continue
            position = self.simulator.getAgentPosition(i)
            goal_n = self.swarm.des_path_pos[i][0]
            goal_now = Vector2(goal_n[0], goal_n[1])
            # print("goal now=" + str(goal_now))
            if abs(goal_now - position) <= self.reach_dis_inter:
                self.swarm.update_state_idx(i)
                # if len(self.path_all[i]) > 1:
                #     self.path_all[i].pop()

            goal_vector = goal_now - self.simulator.getAgentPosition(i)
            if absSq(goal_vector) > 1.0:
                goal_vector = normalize(goal_vector)
            self.simulator.setAgentPrefVelocity(i, goal_vector)

    def reached_goal(self):
        # 检查所有智能体是否都到达了它们的目标
        goals = self.simulator.goals_info
        for i in range(self.simulator.getNumAgents()):
            if abs(self.simulator.getAgentPosition(i) - goals[i]) < self.reach_dis_goal:
                self.simulator.agents_[i].agent_exist_ = False

        for i in range(self.simulator.getNumAgents()):
            if self.simulator.agents_[i].agent_exist_ is True:
                # print("agent i=" + str(i) + "is exist")
                return False

        return True

    def update_state(self):
        for i in range(self.simulator.getNumAgents()):
            if self.simulator.agents_[i].agent_exist_ is False:
                continue
            pos_n = [self.simulator.agents_[i].position_.x_, self.simulator.agents_[i].position_.y_]
            # 同时也更新一些pos信息
            self.swarm.pos_all[i] = pos_n
            pos_idx = [int(pos_n[0] * self.resolution), int(pos_n[1] * self.resolution)]

            size_n = self.mapInfo.map_01.shape
            if 0 <= pos_idx[0] < size_n[0] and 0 <= pos_idx[1] < size_n[1]:
                cell_idx = self.mapInfo.map_all[pos_idx[0], pos_idx[1]]
                if cell_idx != -1:
                    self.swarm.current_cell[i] = copy.copy(cell_idx)
                    # print("agent i="+ str(i) +" current cell=" + str(self.swarm.current_cell[i]))
            else:
                return
    def init_state(self):
        if self.swarm.previous_node is None:
            self.swarm.previous_node = [0] * self.swarm.robots_num
        if self.swarm.start_node is None:
            for idx, node in self.mapInfo.node_start_end_adj.items():
                if node['name'] == 'start':
                    self.swarm.start_node = [idx] * self.swarm.robots_num
        if self.swarm.current_cell is None:
            self.swarm.current_cell = [1] * self.swarm.robots_num
        if self.swarm.goal_cell is None:
            self.swarm.goal_cell = [len(self.mapInfo.cell_info)] * self.swarm.robots_num
        if self.swarm.goal_node is None:
            for idx, node in self.mapInfo.node_start_end_adj.items():
                if node['name'] == 'goal':
                    self.swarm.goal_node = [idx] * self.swarm.robots_num

    def mkdir_file(self):
        # 检查文件夹是否存在
        name_n = self.name
        folder_path = os.path.dirname(name_n)  # 获取文件夹的路径
        if os.path.exists(folder_path):  # 如果文件夹存在
            # 删除文件夹中的所有文件
            for filename in os.listdir(folder_path):
                file_path = os.path.join(folder_path, filename)
                if os.path.isfile(file_path) or os.path.islink(file_path):
                    os.remove(file_path)
        else:  # 如果文件夹不存在，则创建文件夹
            os.makedirs(folder_path)

    def main_one(self):
        self.init_simulation_flow_planner()

    def main(self):
        # self.init_simulation_flow_planner()
        self.mkdir_file()
        # 执行并操作模拟
        t = 0
        while not self.reached_goal():
            if t >= 2000:
                break
            t += 1

            if t % self.pic_time_gap == 0:
                print("t=" + str(t))
                self.update_visualization(t)
            if t % 20 == 0:
                self.flow_area_planner_run()
                # self.flow_planner_run()
                ave_time = sum(self.flow_main_time) / len(self.flow_main_time)
                print("ave flow main time=" + str(ave_time))
                # self.area_flow_planner.run()
                # self.max_flow_planner.run()
                # self.flow_planner_run()
                # self.set_path_pos()
            # if t % 10 == 0:
            #     # self.area_flow_planner.position_allocation_greedy_now()
            #     self.area_flow_planner.local_scheduler_run()
            #     self.swarm.para_set_planner()

            self.update_state()
            self.set_preferred_velocities()
            self.simulator.doStep()

            for i in range(self.swarm.robots_num):
                pos_i_vector = self.simulator.agents_[i].position_
                pos_i = [pos_i_vector.x_, pos_i_vector.y_]
                # print("pos now=" + str(pos_i))
                if self.swarm.judge_reach_des(i, pos_i, self.simulator):
                    self.swarm.update_state_idx(i)
            # print()

        t += 1
        self.update_visualization(t)
        ave_time = sum(self.flow_main_time) / len(self.flow_main_time)
        print("ave flow main time=" + str(ave_time))
        T_r = (t-1)/10
        print("T result = " + str(T_r))
        # save_density_to_json(self.max_density_list, self.mean_density_list,
        #                      self.non_zero_mean_density, "flow_density.json")
        # save_wait_num_json(self.num_wait_red_list, "flow_wait.json")
        save_T_computation_time(T_r, ave_time, self.file_main)


if __name__ == "__main__":
    L_predictive = 15
    K_horizon = 3
    name = "f-L=" + str(L_predictive) + "-K=" + str(K_horizon)
    s1 = Simu_ORCA_flow(name)
    s1.init_simulation_area_planner(L_predictive, K_horizon)
    s1.main()
    # L_pre_list = [5, 10, 15, 20, 25, 30]
    # K_horizon_list = [1, 2, 3, 4, 5, 6, 7, 8, 9, 10]
    # for L_predictive in L_pre_list:
    #     for K_horizon in K_horizon_list:
    #         name = "f-L=" + str(L_predictive) + "-K=" + str(K_horizon)
    #         s1 = Simu_ORCA_flow(name)
    #         s1.init_simulation_area_planner(L_predictive, K_horizon)
    #         # s1.main_one()
    #         s1.main()