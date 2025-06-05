import math
import random
import time
import os
from Vector2 import *
# from ORCA_Simulator import *
from plotPic import *

class pic_one:

    def __init__(self):
        self.pos_all = None
        self.agents = None
        self.time = None
        self.obs_info = []


    def update_visualization(self, simulator):
        # 输出当前全局时间
        print("time=" + str(simulator.getGlobalTime()))

        name = "test" + "/snap" + str(simulator.getGlobalTime()) + ".png"
        visualize_traj_dynamic(simulator.agents_, simulator.obs_info, simulator.goals_info, name)

        for i in range(simulator.getNumAgents()):
            print(f" {simulator.getAgentPosition(i)}", end='')
        print()

    def mkdir_file(self, name_n):
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

    def data_one(self):
        pos_list_new = []
        time_list_new=  []
        for time, pos in self.pos_all.items():
            time_list_new.append(time)
            pos_list_new.append(pos)
        self.agents = pos_list_new
        self.time = time_list_new
        print()

    def obs_info_deal(self):
        obs_new = []
        for obs in self.obs_info:
            vertices = []
            for v in obs:
                o = (v.x_, v.y_)
                vertices.append(o)
            obs_new.append(vertices)

        self.obs_info = obs_new

    def obs_one(self):
        obstacle1 = [Vector2(-10.0, 40.0), Vector2(-40.0, 40.0),
                     Vector2(-40.0, 10.0), Vector2(-10.0, 10.0)]
        obstacle2 = [Vector2(10.0, 40.0), Vector2(10.0, 10.0),
                     Vector2(40.0, 10.0), Vector2(40.0, 40.0)]
        obstacle3 = [Vector2(10.0, -40.0), Vector2(40.0, -40.0),
                     Vector2(40.0, -10.0), Vector2(10.0, -10.0)]
        obstacle4 = [Vector2(-10.0, -40.0), Vector2(-10.0, -10.0),
                     Vector2(-40.0, -10.0), Vector2(-40.0, -40.0)]

        self.obs_info.append(obstacle1)
        self.obs_info.append(obstacle2)
        self.obs_info.append(obstacle3)
        self.obs_info.append(obstacle4)
        self.obs_info_deal()


    def run(self):
        data = {}
        current_time = None
        with open('example/roadmap100.txt', 'r') as file:
            for line in file:
                line = line.strip()
                if line.startswith('time='):
                    current_time = float(line.split('=')[1])
                    data[current_time] = []
                else:
                    positions = line.split(' ')
                    for pos in positions:
                        pos = pos.strip('()')
                        x, y = map(float, pos.split(','))
                        data[current_time].append((x, y))

        print(data)
        self.pos_all = data
        self.data_one()
        self.obs_one()
        name_n = "test" + "/snap" + str(2) + ".png"
        self.mkdir_file(name_n)

        for i in range(len(self.time)):
            if i % 10 == 0:
                goals = []
                name = "test" + "/snap" + str(self.time[i]) + ".png"
                visualize_traj_dynamic_txt(self.agents[i], self.obs_info, goals, name)


p1 = pic_one()
p1.run()