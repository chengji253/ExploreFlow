import logging
import numpy as np
import copy
import math
import yaml
import os


class Swarm:

    def __init__(self, pic_name):

        self.pic_name = pic_name
        self.robots_num = None
        self.pos_all = None
        self.goal_pos = None

        self.start_ave_pos = None
        self.goal_ave_pos = None
        # 机器人当前所在的node idx
        self.previous_node = None
        self.current_node = None
        self.current_edge = None

        self.goal_cell = None
        self.current_cell = None

        self.goal_node = None
        self.start_node = None
        # 当前机器人需要去的des pos 算法根据这个推进
        self.des_node = None
        self.des_pos = None
        # 机器人需要去的后续一系列的des node pos
        self.des_path_node = None
        self.des_path_pos = None

        self.V_all = None
        self.v_max = None

        # 到达目的地 后续机器人不考虑它的避碰需求了 默认消失了
        self.robot_exist = None
        self.reach_dis = 2
        self.reach_dis_goal = 2

        self.robot_radius = 0.5

        self.v_is_0_count = None

        self.Y_pos_reduce = None
        self.Y_pos_reduce_num = None

        self.V_all_store = []
        self.pos_all_store = []

        self.resolution = 1

        # 位置在这里初始化
        start_list, goal_list, dimensions, obstacles = self.read_yaml_file()
        self.robots_num = len(start_list)
        self.pos_all = start_list
        self.goal_pos = goal_list

    def store_pos_v_data(self):
        self.V_all_store.append(copy.deepcopy(self.V_all))
        self.pos_all_store.append(copy.deepcopy(self.pos_all))

    def para_set_no_planner(self):
        # 设置无人机的path路径
        self.current_node = [24] * self.robots_num
        self.start_node = [24] * self.robots_num
        self.goal_node = [25] * self.robots_num
        self.des_pos = self.goal_pos

        self.des_path_pos = []
        self.des_path_node = []
        for i in range(self.robots_num):
            pos = [self.goal_pos[i]]
            self.des_path_pos.append(pos)
            node = [self.goal_node]
            self.des_path_node.append(node)

    def set_start_goal_node(self, start_idx, goal_idx, start_cell, goal_cell):

        self.start_node = [start_idx] * self.robots_num
        self.goal_node = [goal_idx] * self.robots_num
        self.current_node = [start_idx] * self.robots_num
        self.previous_node = [start_idx] * self.robots_num
        # self.current_cell = [start_cell] * self.robots_num
        # self.goal_cell = [goal_cell] * self.robots_num
        self.Y_pos_reduce_num = [0] * self.robots_num

    def para_set_planner_Other_planner(self):
        des_pos = []
        for i in range(self.robots_num):
            pos = self.des_path_pos[i][0]
            des_pos.append(pos)
        self.des_pos = des_pos

    def para_set_planner(self):
        # 设置无人机的path路径
        # self.current_node = [24]*self.robots_num
        # self.start_node = [24]*self.robots_num
        # self.goal_node = [25]*self.robots_num
        des_pos = []
        des_node = []
        for i in range(self.robots_num):
            pos = self.des_path_pos[i][0]
            des_pos.append(pos)
            node = self.des_path_node[i][0]
            des_node.append(node)
        self.des_pos = des_pos
        self.des_node = des_node

        for i in range(self.robots_num):
            self.des_path_pos[i][-1] = self.goal_pos[i]

    def del_des_path_first(self):
        # 删除第一个node和pos
        for i in range(self.robots_num):
            if len(self.des_path_node[i]) > 1:
                del self.des_path_node[i][0]
                del self.des_path_pos[i][0]

    def update_state_idx_min_dis_Astar(self, idx):
        # print("update_state_idx_min_dis")
        # 找到哪些y轴方向在减小的机器人
        # 检查是否需要更新 des
        # 检查所有的pos 选择距离自己最近的那个
        if self.des_pos is None:
            # no planner--des to goal
            return
        len_n = len(self.des_path_pos[idx])
        if len_n <= 3:
            return
        dis_min = 999999999999
        idx_min = None
        for i in range(len_n):
            now_pos = self.pos_all[idx]
            des_pos = self.des_path_pos[idx][i]
            dis = self.distance(now_pos, des_pos)
            if dis < dis_min:
                dis_min = dis
                idx_min = i
        if idx_min == 0:
            return
        else:
            # print("des_path len=" + str(len_n))
            logging.info("update idx_min=" + str(idx_min))
            self.des_pos[idx] = copy.copy(self.des_path_pos[idx][idx_min])
            del self.des_path_pos[idx][0:idx_min]
            # print("After des_path len=" + str(len(self.des_path_pos[idx][0:idx_min])))

    def update_state_idx_min_dis(self, idx):
        # print("update_state_idx_min_dis")
        # 找到哪些y轴方向在减小的机器人
        # 检查是否需要更新 des
        # 检查所有的pos 选择距离自己最近的那个
        if self.des_node is None:
            # no planner--des to goal
            self.current_node[idx] = self.goal_node[idx]
            return
        len_n = len(self.des_path_node[idx])
        if len_n <= 3:
            return
        dis_min = 999999999999
        idx_min = None
        if len_n >= 20:
            len_use = 15
        else:
            len_use = len_n

        for i in range(len_use):
            now_pos = self.pos_all[idx]
            des_pos = self.des_path_pos[idx][i]
            dis = self.distance(now_pos, des_pos)
            if dis < dis_min:
                dis_min = dis
                idx_min = i
        if idx_min == 0:
            return
        else:
            print("des_path len=" + str(len_n))
            print("update idx_min=" + str(idx_min))
            self.current_node[idx] = copy.copy(self.des_path_node[idx][idx_min - 1])
            self.des_pos[idx] = copy.copy(self.des_path_pos[idx][idx_min])
            self.des_node[idx] = copy.copy(self.des_path_node[idx][idx_min])

            edge_head = copy.copy(self.des_path_node[idx][idx_min - 1])
            edge_tail = copy.copy(self.des_path_node[idx][idx_min])
            self.current_edge[idx, :] = np.array([edge_head, edge_tail])

            del self.des_path_node[idx][0:idx_min]
            del self.des_path_pos[idx][0:idx_min]
            print("After des_path len=" + str(len(self.des_path_node[idx][0:idx_min])))

    def update_state_idx_cell(self, idx):
        # 机器人的state需要更新
        # 检查是否到达当前的des
        # 如果到达更新所有的des current
        if self.des_node is None:
            # no planner--des to goal
            self.current_node[idx] = self.goal_node[idx]
        else:
            self.current_node[idx] = copy.copy(self.des_node[idx])
            if len(self.des_path_node[idx]) > 1:
                edge_head = copy.copy(self.des_path_node[idx][0])
                # print("len 1=" + str(len(self.des_path_node[0])))
                # print("len 2=" + str(len(self.des_path_pos[0])))
                del self.des_path_node[idx][0]
                del self.des_path_pos[idx][0]
                # print("idx=" + str(idx))
                # print("des_node=" + str(self.des_path_node[idx][0]))
                # print("des_pos=" + str(self.des_pos[idx][0]) + ',' + str(self.des_pos[idx][0]))
                self.des_pos[idx] = copy.copy(self.des_path_pos[idx][0])
                self.des_node[idx] = copy.copy(self.des_path_node[idx][0])
                edge_tail = copy.copy(self.des_path_node[idx][0])
                self.current_edge[idx, :] = np.array([edge_head, edge_tail])
                # print("robot " + str(idx) + "=" + str(self.current_edge[idx][0]) + "," + str(self.current_edge[idx][1]))

    def update_state_idx(self, idx):
        # 机器人的state需要更新
        # 检查是否到达当前的des
        # 如果到达更新所有的des current
        if self.des_node is None:
            # no planner--des to goal
            self.current_node[idx] = self.goal_node[idx]
        else:
            self.current_node[idx] = copy.copy(self.des_node[idx])
            self.previous_node[idx] = copy.copy(self.des_node[idx])

            if len(self.des_path_node[idx]) > 1:
                edge_head = copy.copy(self.des_path_node[idx][0])
                # print("len 1=" + str(len(self.des_path_node[0])))
                # print("len 2=" + str(len(self.des_path_pos[0])))
                del self.des_path_node[idx][0]
                del self.des_path_pos[idx][0]
                # print("idx=" + str(idx))
                # print("des_node=" + str(self.des_path_node[idx][0]))
                # print("des_pos=" + str(self.des_pos[idx][0]) + ',' + str(self.des_pos[idx][0]))
                self.des_pos[idx] = copy.copy(self.des_path_pos[idx][0])
                self.des_node[idx] = copy.copy(self.des_path_node[idx][0])
                edge_tail = copy.copy(self.des_path_node[idx][0])
                self.current_edge[idx, :] = np.array([edge_head, edge_tail])
                # print("robot " + str(idx) + "=" + str(self.current_edge[idx][0]) + "," + str(self.current_edge[idx][1]))

    def update_state_idx_Astar(self, idx):
        if len(self.des_path_pos[idx]) >= 2:
            del self.des_path_pos[idx][0]
            self.des_pos[idx] = copy.copy(self.des_path_pos[idx][0])

    def forward_pos_state(self, i, time_step):
        # if self.robot_exist[i] is True:
        self.pos_all[i][0] += self.V_all[i][0] * time_step
        self.pos_all[i][1] += self.V_all[i][1] * time_step

        # 机器人在往回走 可能是被其它机器人 裹挟者 此时需要找到一些别的des

        if self.V_all[i][1] < -1.5:
            self.Y_pos_reduce_num[i] += 1
            if self.Y_pos_reduce_num[i] >= 10:
                self.Y_pos_reduce[i] = True
                # print("robot " + str(i) + "Y pos reduce")
                # print("V=" + str(self.V_all[i][1]))
        else:
            self.Y_pos_reduce_num[i] = 0
            self.Y_pos_reduce[i] = False

    def judge_reach_des(self, i, pos_i, simulator):

        if self.des_pos[i][0] == self.goal_pos[i][0] and self.des_pos[i][1] == self.goal_pos[i][1]:
            # des is goal
            if self.reach(pos_i, self.des_pos[i], self.reach_dis_goal):
                self.robot_exist[i] = False
                self.V_all[i] = [0, 0]

                simulator.agents_[i].agent_exist_ = False
                return True
            else:
                return False
        else:
            # des is not goal
            if self.reach(pos_i, self.des_pos[i], self.reach_dis):
                return True
            else:
                return False

    def judge_robot_all_exist(self):
        return all(x is False for x in self.robot_exist)

    def find_dead_lock(self, V, i, around_no_robots):
        # 找到那个处于死锁位置的机器人
        # 如果没有周围没有机器人 并且自己的速度一直是零 认为可能处于死锁的可能
        # 保留周围是否有obs的判断
        norm_x = np.linalg.norm([V[i][0], V[i][1]])
        if norm_x <= 0.2 and around_no_robots[i] is True and self.robot_exist[i] is True:
            self.v_is_0_count[i] += 1
            if self.v_is_0_count[i] >= 3:
                # print("robot " + str(i) + " is dead lock !")
                return True
        else:
            self.v_is_0_count[i] = 0
            return False

    def deal_with_dead_lock(self, i):
        x_gap = self.des_pos[i][0] - self.pos_all[i][0]
        y_gap = self.des_pos[i][1] - self.pos_all[i][1]

        if abs(x_gap) >= abs(y_gap):
            if y_gap > 0:
                self.V_all[i][1] = 0.5
            else:
                self.V_all[i][1] = -0.5
        elif abs(x_gap) < abs(y_gap):
            if x_gap > 0:
                self.V_all[i][0] = 0.5
            else:
                self.V_all[i][0] = -0.5
        else:
            pass

        if self.v_is_0_count[i] >= 15:
            if abs(x_gap) >= abs(y_gap):
                if y_gap > 0:
                    self.pos_all[i][1] += -0.1
                else:
                    self.pos_all[i][1] += 0.1
            elif abs(x_gap) < abs(y_gap):
                if x_gap > 0:
                    self.pos_all[i][0] += -0.1
                else:
                    self.pos_all[i][0] += 0.1
            else:
                pass


    def compute_ave_start_goal_pos(self):
        self.start_ave_pos = self.compute_ave_pos(self.pos_all)
        self.goal_ave_pos = self.compute_ave_pos(self.goal_pos)
        # print()

    def compute_ave_pos(self, pos_list):
        sum_x = 0
        sum_y = 0
        for pos in pos_list:
            sum_x += pos[0]
            sum_y += pos[1]
        num = len(pos_list)
        return [sum_x / num, sum_y / num]

    def generate_start_pos(self, height_list, width_list):
        pos = []
        for h in height_list:
            for w in width_list:
                pos.append([w, h])
        return pos

    def generate_goal_pos(self, height_list, width_list):
        pos = []
        for h in height_list:
            for w in width_list:
                pos.append([w, h])
        return pos

    def reach(self, p1, p2, bound=0.2):
        if self.distance(p1, p2) < bound:
            return True
        else:
            return False

    def distance(self, pose1, pose2):
        """ compute Euclidean distance for 2D """
        return math.sqrt((pose1[0] - pose2[0]) ** 2 + (pose1[1] - pose2[1]) ** 2)

    def init_swarm(self):
        # self.robots_num = 10
        self.robot_exist = [True] * self.robots_num
        self.v_is_0_count = [0] * self.robots_num
        self.Y_pos_reduce = [False] * self.robots_num
        self.current_edge = np.zeros((self.robots_num, 2), dtype=int)

        # start_h_list = list(range(10, 10 + 5))
        # goal_h_list = list(range(75, 75 + 5))
        # start_h_list = [x - 1 for x in start_h_list]
        # goal_h_list = [x + 1 for x in goal_h_list]
        # width_list = [14, 15]

        # self.pos_all = self.generate_start_pos(start_h_list, width_list)
        # self.goal_pos = self.generate_goal_pos(goal_h_list, width_list)

        self.V_all = [[0, 0] for i in range(len(self.pos_all))]
        self.v_max = [5.0 for i in range(len(self.pos_all))]

    def read_yaml_file(self):
        yaml_file_name = self.pic_name
        start_list = []
        goal_list = []
        current_directory = os.getcwd()
        print(current_directory)
        with open(yaml_file_name, 'r') as file:
            try:
                data = yaml.safe_load(file)
                # 打印读取的数据
                # print("读取的 YAML 数据:")
                # print(data)
                agents = data.get('agents', [])
                for agent in agents:
                    name = agent.get('name')
                    start = agent.get('start')
                    goal = agent.get('goal')
                    start_list.append(start)
                    goal_list.append(goal)
                    # print(f"Agent 名称: {name}")
                    # print(f"起始位置: {start}")
                    # print(f"目标位置: {goal}")
                    # print()
                map_info = data.get('map', {})
                dimensions = map_info.get('dimensions')
                obstacles = map_info.get('obstacles', [])
                # print("地图信息:")
                # print(f"地图尺寸: {dimensions}")
                # print("障碍物位置:")
                # for obstacle in obstacles:
                #     print(obstacle)
                return start_list, goal_list, dimensions, obstacles
            except yaml.YAMLError as e:
                print(f"读取 YAML 文件时出错: {e}")