import copy
import sys
import os
import numpy as np
import time
import json
from RVO_module import RVO
from mapCO import mapCO
from map_info import map_info
from para import swarm
from flow_planner import flow_main
from visualize import plotPic
import logging
import colorlog

# from vis import visualize_traj_dynamic


class Simu_main:

    def __init__(self):

        self.title_name = None
        self.total_time = None
        self.time_step = None
        self.resolution = 10

        self.swarm = swarm.Swarm()
        # self.para_10()
        self.para_160()
        # self.para_200()
        # self.para_40_1()
        # self.para_2()

        self.mapCo = mapCO.mapCo()
        self.mapInfo = map_info.MapInfo(self.swarm)

        self.flow_planner = flow_main.Flow_planner(self.swarm, self.mapInfo)

        self.init_simulation()
        # self.para_map()

    def para_2(self):
        self.title_name = 'data/para_map'
        self.total_time = 30
        self.time_step = 0.01
        self.swarm.init_swarm_2()

    def para_10(self):
        self.title_name = 'data/para_10'
        self.total_time = 30
        self.time_step = 0.01
        self.swarm.init_swarm_10()

    def para_40_1(self):
        self.title_name = 'data/para_40_1'
        self.total_time = 30
        self.time_step = 0.01
        # self.swarm.init_swarm_1()
        self.swarm.init_swarm_40()

    def para_80(self):
        self.title_name = 'data/para_80'
        self.total_time = 40
        self.time_step = 0.01
        # self.swarm.init_swarm_1()
        self.swarm.init_swarm_80()

    def para_160(self):
        self.title_name = 'data/para_160'
        self.total_time = 50
        self.time_step = 0.01
        # self.swarm.init_swarm_1()
        self.swarm.init_swarm_160()

    def para_200(self):
        self.title_name = 'data/para_200'
        self.total_time = 50
        self.time_step = 0.01
        # self.swarm.init_swarm_1()
        self.swarm.init_swarm_200()

    def init_simulation(self):
        t1 = time.time()
        self.mapInfo.init_main()
        t2 = time.time()
        self.mapCo.init_mapCo_one(self.mapInfo)
        t3 = time.time()
        self.swarm.set_start_goal_node(self.mapInfo.start_idx, self.mapInfo.end_idx)
        # self.mapInfo.draw_network()
        self.planner_run()
        # self.no_planner()

        print("mapInfo_time=" + str(t2 - t1) + "s")
        print("mapCo_time=" + str(t3 - t2) + "s")

    def no_planner(self):
        self.swarm.des_pos = self.swarm.goal_pos
        self.swarm.des_path_node = self.swarm.goal_node

    def planner_run(self):
        # self.flow_planner.init_para_cut()
        # des_path_node = self.flow_planner.path_selection_cut_edges()

        self.mapInfo.find_current_path_set()
        self.mapInfo.shared_node_to_path()
        self.flow_planner.init_para_shared_node()
        des_path_node = self.flow_planner.path_selection_out_node()
        # des_path_node = self.flow_planner.path_selection_only_run_cost()

        self.swarm.des_path_node = des_path_node
        self.set_center_pos()
        self.swarm.del_des_path_first()
        self.flow_planner.position_allocation()

        # self.flow_planner.loca_pos_allocation()

        self.swarm.para_set_planner()

        # self.swarm.para_set_no_planner()

    def set_center_pos(self):
        des_path_pos = []
        for i in range(self.swarm.robots_num):
            path_node = self.swarm.des_path_node[i]
            pos_list = []
            for node in path_node:
                pos = self.mapInfo.node_all[node]['pos']
                if node == self.swarm.goal_node[i]:
                    pos = self.swarm.goal_pos[i]
                else:
                    pos = [pos[0], pos[1]]
                pos_list.append(pos)
            des_path_pos.append(pos_list)
        self.swarm.des_path_pos = des_path_pos


    def simulate_main(self):
        # define workspace model
        ws_model = dict()
        ws_model['robot_radius'] = self.swarm.robot_radius
        ws_model['circular_obstacles'] = self.mapCo.obstacles
        # simulation setup
        # total simulation time (s)
        total_time = self.total_time
        # simulation step
        step = self.time_step
        # visualization
        name_n = '/snap%s.png'
        name_n = self.title_name + name_n

        # 检查文件夹是否存在
        folder_path = os.path.dirname(name_n)  # 获取文件夹的路径
        if os.path.exists(folder_path):  # 如果文件夹存在
            # 删除文件夹中的所有文件
            for filename in os.listdir(folder_path):
                file_path = os.path.join(folder_path, filename)
                if os.path.isfile(file_path) or os.path.islink(file_path):
                    os.remove(file_path)
        else:  # 如果文件夹不存在，则创建文件夹
            os.makedirs(folder_path)

        # simulation starts
        t = 0
        while t * step < total_time:
            # compute velocity
            # self.set_des_pos(t)
            self.compute_V_RVO(ws_model)
            # update position
            # if t == 400:
            #     print()
            if t != 0 and t % 100 == 0:
                tp1 = time.time()
                self.planner_run()
                tp2 = time.time()
                print("planner_time=" + str(tp2 - tp1) + "s")

            for i in range(len(self.swarm.pos_all)):
                # 如果机器人陷入deadlock 给平行方向加一个速度
                # if self.swarm.find_dead_lock(self.swarm.V_all, i):
                #     self.swarm.deal_with_dead_lock(i)
                self.swarm.forward_pos_state(i, self.time_step)
                if self.swarm.judge_reach_des(i):
                    self.swarm.update_state_idx(i)
            # ----------------------------------------
            if t % 20 == 0:
                plotPic.visualize_traj_dynamic(ws_model, self.swarm, self.mapCo.boundary, self.mapInfo, time=t * step,
                                            name=name_n % str(t / 10))
                # print("t=" + str(t))
            if self.swarm.judge_robot_all_exist():
                plotPic.visualize_traj_dynamic(ws_model, self.swarm, self.mapCo.boundary, self.mapInfo, time=(t) * step,
                                            name=name_n % str((t) / 10))
                break
            t += 1
            print("t=" + str(t))

        print("time_step=" + str(t * step))

    def set_des_pos(self, t):
        # 改变机器人的临时des
        # if t <= 100:
        #     self.swarm.des_pos = copy.deepcopy(self.swarm.goal_pos)
        #     self.swarm.des_pos[15] = [25, 25]
        # else:
        self.swarm.des_pos = copy.deepcopy(self.swarm.goal_pos)

    def compute_V_RVO(self, ws_model):
        # compute desired vel to goal
        V_des = compute_V_des(self.swarm.pos_all, self.swarm.des_pos, self.swarm.v_max)
        # compute the optimal vel to avoid collision
        V = RVO_update(self.swarm.pos_all, V_des, self.swarm.V_all, ws_model, self.swarm.robot_exist)
        self.swarm.V_all = V


time1 = time.time()
s1 = Simu_main()
# swarm.para_one()

time2 = time.time()
# s1.simulate_main()
time3 = time.time()

init_time = time2 - time1
run_time = time3 - time2
print("init_time=" + str(init_time) + "s")
print("run_time=" + str(run_time) + "s")