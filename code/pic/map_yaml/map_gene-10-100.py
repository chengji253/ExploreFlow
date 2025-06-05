import yaml
import numpy as np
import matplotlib.pyplot as plt

class map_gene:

    def __init__(self):
        self.dimensions = None
        self.start_positions = None
        self.goal_positions = None
        self.obstacles = []
        self.obs_idx = None
        self.obs_inside = []
        self.file_name = None
        self.robot_number = None
        self.goal_y = None
        self.wid_set = 2
        self.hei_set = 2

    def para_100_forest(self):
        # 视频地图里面obs 保存一下
        self.dimensions = [60, 60]
        self.file_name = "m1-100-forest.yaml"
        self.robot_number = 100
        self.goal_y = 45
        self.generate_positions()
        # self.start_positions = [[10, 1], [11, 1], [12, 1], [13, 1], [14, 1]]
        # self.goal_positions = [[10, 30], [11, 30], [12, 30], [13, 30], [14, 30]]
        self.obs_idx = [[10, 20], [20, 20], [30, 20], [40, 20], [50, 20],
                        [5, 25], [16, 25], [26, 25], [34, 25], [45, 25],
                        [9, 30], [30, 30], [40, 30],
                        [11, 35], [23, 35], [29, 35], [35, 35], [43, 35],
                        [6, 40], [25, 40], [32, 40], [45, 40]
                        ]

    def para_100_maze(self):
        # 视频地图里面obs 保存一下
        self.dimensions = [60, 60]
        self.file_name = "m1-30-maze.yaml"
        self.robot_number = 30
        self.goal_y = 45

        # self.start_positions = [[10, 1], [11, 1], [12, 1], [13, 1], [14, 1]]
        # self.goal_positions = [[10, 30], [11, 30], [12, 30], [13, 30], [14, 30]]
        self.obs_idx = [[12, 20, 2, 10], [20, 20, 2, 15], [5, 38, 10, 2],
                        [30, 20, 2, 8], [26, 30, 10, 2], [38, 20, 15, 2],
                        [47, 28, 7, 2], [40, 28, 2, 10], [5, 24, 2, 10],
                        [22, 40, 8, 2], [50, 32, 2, 7], [35, 36, 2, 4]
                        ]

    def para_100_clutter(self):
        # 视频地图里面obs 保存一下
        self.dimensions = [60, 60]
        self.file_name = "m1-80-clutter.yaml"
        self.robot_number = 80
        self.goal_y = 45

        # self.start_positions = [[10, 1], [11, 1], [12, 1], [13, 1], [14, 1]]
        # self.goal_positions = [[10, 30], [11, 30], [12, 30], [13, 30], [14, 30]]
        self.obs_idx = [[12, 20, 2, 10], [20, 20, 2, 15], [5, 38, 10, 2],
                        [30, 35, 10, 2], [40, 28, 10, 2], [30, 20, 10, 2],
                        [4, 20, 3, 3], [7, 32, 3, 3], [25, 25, 3, 3],
                        [23, 38, 3, 3], [33, 28, 3, 3], [45, 20, 3, 3],
                        [50, 35, 3, 3], [55, 25, 3, 3]
                        ]

    def para_100_corridor(self):
        # 视频地图里面obs 保存一下
        self.dimensions = [60, 60]
        self.file_name = "m1-10-corridor.yaml"
        self.robot_number = 10
        self.goal_y = 45

        # self.start_positions = [[10, 1], [11, 1], [12, 1], [13, 1], [14, 1]]
        # self.goal_positions = [[10, 30], [11, 30], [12, 30], [13, 30], [14, 30]]
        self.obs_idx = [[8, 20, 8, 8], [20, 20, 8, 8], [32, 20, 8, 8], [45, 20, 8, 8],
                        [14, 33, 8, 8], [26, 33, 8, 8], [38, 33, 8, 8]
                        ]

    def para_one(self):
        self.dimensions = [80, 80]
        # self.start_positions = [[10, 1], [11, 1], [12, 1], [13, 1], [14, 1]]
        # self.goal_positions = [[10, 30], [11, 30], [12, 30], [13, 30], [14, 30]]
        # self.obs_idx = [[10, 20], [40, 20], [25, 30]]
        self.obs_idx = [[10, 20], [20, 20], [30, 20], [40, 20], [50, 20], [65, 20], [71, 20],
                        [5, 25], [16, 25], [26, 25], [34, 25], [45, 25], [55, 25],
                        [9, 30], [30, 30], [40, 30], [60, 25],
                        [11, 35], [23, 35], [29, 35], [35, 35], [43, 35], [62, 35], [71, 35],
                        [6, 40], [25, 40], [32, 40], [45, 40], [55, 40]
                        ]

    def para_two(self):
        self.dimensions = [80, 100]
        self.file_name = "m1-100-forest.yaml"
        # self.start_positions = [[10, 1], [11, 1], [12, 1], [13, 1], [14, 1]]
        # self.goal_positions = [[10, 30], [11, 30], [12, 30], [13, 30], [14, 30]]
        # self.obs_idx = [[10, 20], [40, 20], [25, 30]]
        self.obs_idx = [[10, 20], [20, 20], [30, 20], [40, 20], [50, 20], [65, 20], [71, 20],
                        [5, 25], [16, 25], [26, 25], [34, 25], [45, 25], [55, 25],
                        [9, 30], [30, 30], [40, 30], [60, 25],
                        [11, 35], [23, 35], [29, 35], [35, 35], [43, 35], [62, 35], [71, 35],
                        [6, 40], [25, 40], [32, 40], [45, 40], [55, 40],
                        [10, 48], [30, 48], [38, 48], [50, 48], [60, 48],
                        [15, 55], [28, 55], [35, 55], [40, 55], [55, 55], [65, 55], [78, 55],
                        [5, 60], [10, 60], [20, 60], [30, 60], [40, 60], [52, 60], [61, 60],
                        [10, 70], [15, 70], [25, 70], [35, 70], [43, 70], [62, 70], [71, 70],
                        [5, 75], [20, 75], [30, 75], [40, 75], [50, 75], [55, 75]]

    def main(self):
        # self.para_100_forest()
        # self.para_100_maze()
        self.para_100_clutter()
        # self.para_100_corridor()
        # self.para_one()
        # self.para_two()
        self.generate_positions()
        self.generate_obstacles()
        self.generate_vertices()
        self.generate_map()
        self.map_data()

    def map_data(self):

        binary_map = np.ones((self.dimensions[0] + 1, self.dimensions[1] + 1), dtype=int)

        binary_map[0, :] = 0  # 第一行
        binary_map[-1, :] = 0  # 最后一行
        binary_map[:, 0] = 0  # 第一列
        binary_map[:, -1] = 0  # 最后一列

        for obs in self.obstacles:
            binary_map[obs[0], obs[1]] = 0
        self.plot_matrix_map(binary_map)

    def generate_vertices(self):
        # 根据给出的obs forest 直接生成多边形顶点 并输出到文件中
        self.obs_vertices = []
        for obs in self.obs_idx:
            obs_list = []
            x = obs[0]
            y = obs[1]
            if len(obs) == 4:
                width = obs[2]
                height = obs[3]
            else:
                width = self.wid_set
                height = self.hei_set
            obs_list.append([x, y])
            obs_list.append([x+width, y])
            obs_list.append([x+width, y+height])
            obs_list.append([x, y+height])
            self.obs_vertices.append(obs_list)

    def generate_obstacles(self):

        for obs in self.obs_idx:
            x = obs[0]
            y = obs[1]
            if len(obs) == 4:
                width = obs[2]
                height = obs[3]
            else:
                width = self.wid_set
                height = self.hei_set
            for i in range(width):
                for j in range(height):
                    self.obstacles.append([x+i, y+j])
            # for i in range(-1, width + 1):
            #     for j in range(-1, height + 1):
            #         self.obstacles.append([x+i, y+j])
            # self.obstacles.append([x - 1, y])
            # self.obstacles.append([x - 1, y - 1])
            # self.obstacles.append([x, y - 1])
            # self.obstacles.append([x - 1, y + 1])
            # self.obstacles.append([x + 1, y - 1])

            # 矩形左下角的点
            self.obs_inside.append([x, y, 2])

    def generate_positions(self):
        start_positions = []
        goal_positions = []

        num_agents = self.robot_number
        rows = 10  # 每行 10 个
        x_range = (20, 45)
        y_range = (2, 12)
        goal_y = self.goal_y

        for i in range(num_agents):
            x = x_range[0] + (i // rows)  # x 坐标在 10 到 20 之间循环
            y = y_range[0] + (i % rows)  # y 坐标递增
            start_positions.append([x, y])
            goal_positions.append([x, goal_y + y])  # 目标位置的 y 轴固定为 30

        self.start_positions = start_positions
        self.goal_positions = goal_positions
        print("start_positions:", start_positions)
        print("goal_positions:", goal_positions)

    def generate_map(self):
        # 生成 agents 数据
        agents = []
        for i, (start, goal) in enumerate(zip(self.start_positions, self.goal_positions)):
            agent = {
                "name": f"agent{i}",
                "start": start,  # 直接存储为 [x, y]
                "goal": goal  # 直接存储为 [x, y]
            }
            agents.append(agent)
        # 组织成 YAML 格式
        yaml_data = {
            "map": {
                "dimensions": self.dimensions,
                "obstacles": self.obstacles,
                "obs_inside": self.obs_inside,
                "obs_vertices": self.obs_vertices
            },
            "agents": agents
        }
        # 写入 YAML 文件
        with open(self.file_name, "w") as file:
            yaml.dump(yaml_data, file, default_flow_style=None, sort_keys=False)
        print("YAML 文件已生成")

    def plot_matrix_map(self, binary_map):
        # 创建一个自定义的颜色映射
        # 0（障碍物和边界）为黑色，1（背景）为灰色
        binary_map = binary_map.T

        cmap = plt.cm.colors.ListedColormap(['black', '#E0E0E0'])

        # 绘制地图
        fig, ax = plt.subplots()
        ax.imshow(binary_map, cmap=cmap, interpolation='nearest')

        ax.invert_yaxis()
        # 设置坐标轴
        # ax.set_xlabel('X')
        # ax.set_ylabel('Y')
        # ax.set_xticks(np.arange(binary_map.shape[1]))
        # ax.set_yticks(np.arange(binary_map.shape[0]))
        # ax.grid(True, which='both', color='white', linewidth=0.5)  # 添加白色网格线以增强可读性

        # 调整布局并保存
        plt.rcParams['pdf.fonttype'] = 42
        plt.rcParams['ps.fonttype'] = 42
        plt.tight_layout()
        # plt.savefig('map.eps', bbox_inches='tight', dpi=300)
        plt.show()


map1 = map_gene()
map1.main()