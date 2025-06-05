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


class map_generate:
    def __init__(self):
        self.pic_name = "../pic/map_yaml/m1-500-hard.yaml"
        self.simulator = ORCA_Simulator()
        self.boundary = None
        self.name = None
        self.path_all = []
        self.mapInfo = None
        self.swarm = swarm.Swarm(self.pic_name)
        self.resolution = 1

    def init_map(self):
        self.swarm.init_swarm()
        self.swarm.compute_ave_start_goal_pos()
        self.mapInfo = map_info.MapInfo(self.swarm, self.pic_name, self.resolution)
        self.mapInfo.init_main_yaml()

    def main(self):
        # 生成地图和场景文件
        self.to_file()

    def to_file(self):
        # 获取地图尺寸
        map_array = self.mapInfo.dilated_map
        # map_array = self.mapInfo.map_01
        # map_array = np.rot90(map_array, k=1)  # k=1 表示逆时针旋转90度
        # height, width = rotated_map.shape  # 旋转后 height=101, width=81
        height, width = map_array.shape  # height, width 顺序正确，但文件格式需要先width后height

        # 1. 生成 .map 文件
        map_content = f"""type octile
width {width}
height {height}
map
"""
        # 将ndarray转换为地图格式 (@表示障碍物，.表示空地)
        for i in range(height):
            row = ''
            for j in range(width):
                row += '@' if map_array[i, j] == 1 else '.'
            map_content += row + '\n'

        with open("forest.map", "w") as f:
            f.write(map_content)

        # 2. 生成 .scen 文件
        scen_content = "version 1\n"
        # 获取agent的起始和目标位置
        start_positions = self.swarm.pos_all
        goal_positions = self.swarm.goal_pos

        # 确保起始和目标位置数量匹配
        num_agents = min(len(start_positions), len(goal_positions))

        for i in range(num_agents):
            start_x, start_y = start_positions[i]
            goal_x, goal_y = goal_positions[i]
            # 计算欧几里得距离作为路径长度估计
            distance = np.sqrt((goal_x - start_x) ** 2 + (goal_y - start_y) ** 2)
            # 格式：agent_id map_name width height start_x start_y goal_x goal_y distance
            scen_line = f"{i}\tforest.map\t{width}\t{height}\t{start_y}\t{start_x}\t{goal_y}\t{goal_x}\t{distance:.8f}\n"
            scen_content += scen_line

        with open("forest.scen", "w") as f:
            f.write(scen_content)


# 使用示例
m1 = map_generate()
m1.init_map()
m1.main()
print("Files generated successfully!")