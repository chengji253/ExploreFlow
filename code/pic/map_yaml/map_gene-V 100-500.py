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
        self.obs_vertices = []
        self.goal_y = None
        self.wid_set = 2
        self.hei_set = 2

        self.file_name = None
        self.robot_number = None

    def para_100(self):
        # 视频地图里面obs 保存一下
        self.dimensions = [60, 60]
        self.file_name = "m1-30-1.yaml"
        # self.generate_positions()
        self.robot_number = 30
        self.goal_y = 45
        # self.start_positions = [[10, 1], [11, 1], [12, 1], [13, 1], [14, 1]]
        # self.goal_positions = [[10, 30], [11, 30], [12, 30], [13, 30], [14, 30]]
        self.obs_idx = [[10, 20], [20, 20], [30, 20], [40, 20], [50, 20],
                        [5, 25], [16, 25], [26, 25], [34, 25], [45, 25],
                        [9, 30], [30, 30], [40, 30],
                        [11, 35], [23, 35], [29, 35], [35, 35], [43, 35],
                        [6, 40], [25, 40], [32, 40], [45, 40]
                        ]

    def para_one(self):
        self.dimensions = [80, 80]
        # self.start_positions = [[10, 1], [11, 1], [12, 1], [13, 1], [14, 1]]
        # self.goal_positions = [[10, 30], [11, 30], [12, 30], [13, 30], [14, 30]]
        # self.obs_idx = [[10, 20], [40, 20], [25, 30]]
        self.goal_y = 55
        # self.obs_idx = [[10, 20], [20, 20], [30, 20], [40, 20], [50, 20], [65, 20], [71, 20],
        #                 [5, 25], [16, 25], [26, 25], [34, 25], [45, 25], [55, 25],
        #                 [9, 30], [30, 30], [40, 30], [60, 25],
        #                 [11, 35], [23, 35], [29, 35], [35, 35], [43, 35], [62, 35], [71, 35],
        #                 [6, 40], [25, 40], [32, 40], [45, 40], [55, 40]
        #                 ]
        self.obs_idx = [[12, 20, 3, 30], [21, 20, 10, 5], [20, 35, 10, 3], [35, 30, 10, 3],
                        [20, 45, 5, 5], [30, 45, 5, 5], [40, 45, 5, 5],
                        [50, 20, 5, 30], [60, 20, 10, 5], [65, 30, 10, 5],
                        [60, 40, 10, 5], [2, 35, 3, 3], [40, 20, 5, 5], [6, 20, 3, 3]
                        ]

    def para_two(self):
        self.dimensions = [90, 90]
        self.robot_number = 100
        # self.generate_positions()
        # self.start_positions = [[10, 1], [11, 1], [12, 1], [13, 1], [14, 1]]
        # self.goal_positions = [[10, 30], [11, 30], [12, 30], [13, 30], [14, 30]]
        # self.obs_idx = [[10, 20], [40, 20], [25, 30]]
        self.file_name = "m1-100-forest.yaml"
        self.goal_y = 68
        self.obs_idx = [[10, 20], [20, 20], [30, 20], [40, 20], [50, 20], [65, 20], [71, 20],
                        [5, 25], [16, 25], [26, 25], [34, 25], [45, 25], [55, 25],
                        [9, 30], [30, 30], [40, 30], [60, 25],
                        [11, 35], [23, 35], [29, 35], [35, 35], [43, 35], [62, 35], [71, 35],
                        [6, 40], [25, 40], [32, 40], [45, 40], [55, 40],
                        [10, 48], [30, 48], [38, 48], [50, 48], [60, 48],
                        [15, 55], [28, 55], [35, 55], [40, 55], [55, 55], [65, 55],
                        [5, 60], [10, 60], [20, 60], [30, 60], [40, 60], [52, 60], [61, 60]]
        for i in range(len(self.obs_idx)):
            self.obs_idx[i][1] += 5
            self.obs_idx[i][0] += 6


    def para_forest(self):
        self.dimensions = [90, 90]
        self.robot_number = 500
        self.wid_set = 4
        self.hei_set = 4
        # self.generate_positions()
        # self.start_positions = [[10, 1], [11, 1], [12, 1], [13, 1], [14, 1]]
        # self.goal_positions = [[10, 30], [11, 30], [12, 30], [13, 30], [14, 30]]
        # self.obs_idx = [[10, 20], [40, 20], [25, 30]]
        self.file_name = "m1-500-forest-n.yaml"
        self.goal_y = 68
        self.obs_idx = [[10, 20], [20, 20], [30, 20], [40, 20], [50, 20], [65, 20], [75, 20],
                        [5, 28], [16, 28], [26, 28], [34, 28], [45, 28], [55, 28], [70, 28],
                        [11, 35], [23, 35], [35, 35], [43, 35], [62, 35], [71, 35],
                        [3, 42], [22, 42], [40, 42], [55, 42],
                        [10, 48], [30, 48], [50, 48], [70, 48],
                        [0, 55], [10, 60], [20, 55], [30, 60], [40, 55], [52, 60], [65, 55]]
        for i in range(len(self.obs_idx)):
            self.obs_idx[i][1] += 5
            self.obs_idx[i][0] += 6

    def para_maze_1(self):
        self.dimensions = [80, 100]
        self.goal_y = 80
        self.robot_number = 500
        self.file_name = "m1-500-maze-1.yaml"
        self.obs_idx = [[12, 20, 3, 30], [21, 20, 10, 5], [20, 35, 10, 3], [35, 30, 10, 3],
                        [20, 45, 5, 5], [30, 45, 5, 5], [40, 45, 5, 5],
                        [50, 20, 5, 30], [60, 20, 10, 5], [65, 30, 10, 5],
                        [60, 40, 10, 5], [2, 35, 3, 3], [40, 20, 5, 5], [6, 20, 3, 3],
                        [60, 55, 5, 20], [70, 65, 5, 5], [25, 55, 3, 20],
                        [5, 55, 3, 20], [15, 56, 5, 5], [12, 70, 5, 5],
                        [35, 55, 15, 3], [29, 65, 15, 3], [50, 70, 5, 5]
                        ]

    def para_maze_2(self):
        self.dimensions = [80, 100]
        self.goal_y = 80
        self.robot_number = 500
        self.file_name = "m1-500-maze-2.yaml"
        self.obs_idx = [[30, 20, 3, 15], [40, 30, 3, 15], [50, 20, 3, 15],
                        [20, 30, 3, 30], [60, 30, 3, 30], [10, 20, 3, 15],
                        [70, 20, 3, 15], [28, 50, 6, 3], [48, 50, 6, 3],
                        [8, 50, 6, 3], [68, 50, 6, 3],
                        [10, 68, 17, 3], [53, 68, 17, 3], [30, 60, 20, 3],
                        [28, 75, 5, 5], [38, 75, 5, 5], [48, 75, 5, 5],
                        [5, 75, 5, 5], [70, 75, 5, 5],
                        [39, 20, 6, 3], [19, 20, 6, 3], [59, 20, 6, 3]
                        ]

    def para_maze_3(self):
        # 500的 大maze 还需要一个小maze
        self.dimensions = [80, 100]
        self.goal_y = 80
        self.robot_number = 500
        self.file_name = "m1-500-maze-3.yaml"
        self.obs_idx = [[9, 20, 3, 20], [13, 27, 10, 3], [30, 20, 15, 3],
                        [35, 24, 3, 20], [58, 20, 3, 30], [66, 47, 7, 3],
                        [48, 30, 3, 50], [68, 53, 3, 23],
                        [60, 75, 11, 3], [68, 20, 3, 23], [20, 50, 10, 3],
                        [30, 50, 3, 25], [20, 75, 13, 3], [10, 65, 10, 3],
                        [9, 50, 3, 10], [40, 50, 3, 15], [57, 60, 7, 3],
                        [20, 40, 10, 3]
                        ]

    def para_maze_4(self):
        # 500的 大maze 还需要一个小maze
        self.dimensions = [80, 100]
        self.goal_y = 80
        self.robot_number = 500
        self.file_name = "m1-500-maze-4.yaml"
        self.obs_idx = [[9, 20, 3, 20], [13, 27, 10, 3], [30, 20, 15, 3],
                        [35, 23, 3, 20], [58, 20, 3, 30], [66, 47, 7, 3],
                        [48, 30, 3, 50], [40, 62, 4, 4], [68, 53, 3, 23],
                        [60, 75, 11, 3], [68, 20, 3, 23], [20, 50, 10, 3],
                        [30, 50, 3, 25], [20, 75, 13, 3], [10, 65, 10, 3],
                        [57, 63, 5, 3], [38, 50, 4, 4], [9, 50, 3, 10],
                        [20, 40, 10, 3]
                        ]

    def para_corridor_1(self):
        # 500的 大maze 还需要一个小maze
        self.dimensions = [80, 100]
        self.goal_y = 80
        self.robot_number = 500
        self.file_name = "m1-500-corridor.yaml"
        self.obs_idx = [[10, 20, 10, 10], [25, 20, 10, 10], [40, 20, 10, 10], [55, 20, 10, 10],
                        [17, 35, 10, 10], [32, 35, 10, 10], [47, 35, 10, 10], [63, 35, 10, 10],
                        [10, 50, 10, 10], [25, 50, 10, 10], [40, 50, 10, 10], [55, 50, 10, 10],
                        [17, 65, 10, 10], [32, 65, 10, 10], [47, 65, 10, 10], [63, 65, 10, 10]
                        ]

    def para_mix(self):
        # 500的 大maze 还需要一个小maze
        self.dimensions = [80, 100]
        self.goal_y = 80
        self.robot_number = 500
        self.file_name = "m1-500-mix.yaml"
        b_s = 2
        self.obs_idx = [[10, 20, b_s, b_s], [20, 20, b_s, b_s], [30, 20, b_s, b_s], [40, 20, b_s, b_s], [50, 20, b_s, b_s], [65, 20, b_s, b_s], [71, 20, b_s, b_s],
                        [5, 25, b_s, b_s], [16, 25, b_s, b_s], [26, 25, b_s, b_s], [34, 25, b_s, b_s], [45, 25, b_s, b_s], [55, 25, b_s, b_s],
                        [9, 30, b_s, b_s], [30, 30, b_s, b_s], [40, 30, b_s, b_s], [60, 25, b_s, b_s],
                        [11, 35, b_s, b_s], [23, 35, b_s, b_s], [29, 35, b_s, b_s], [35, 35, b_s, b_s], [43, 35, b_s, b_s], [62, 35, b_s, b_s], [71, 35, b_s, b_s],
                        [6, 40, b_s, b_s], [25, 40, b_s, b_s], [32, 40, b_s, b_s], [45, 40, b_s, b_s], [55, 40, b_s, b_s],
                        [10, 45, 2, 15], [18, 50, 5, 2], [28, 45, 2, 8], [25, 60, 10, 2],
                        [35, 45, 10, 2], [40, 55, 2, 10], [46, 55, 8, 2], [60, 45, 10, 2], [65, 53, 2, 10],
                        [75, 45, 2, 2], [72, 53, 3, 3], [5, 65, 10, 2], [52, 65, 8, 2], [52, 45, 2, 2],
                        [20, 70, 5, 5], [8, 75, 5, 5], [30, 70, 5, 5], [45, 70, 5, 5], [73, 65, 2, 11],
                        [63, 73, 9, 2]]

    def main(self):
        # self.para_100()
        # self.para_mix()
        # self.para_corridor_1()
        # self.para_maze_1()
        # self.para_maze_3()
        # self.para_maze_4()
        self.para_forest()

        self.generate_positions()
        self.generate_vertices()
        self.generate_obstacles()
        self.generate_map()
        self.map_data()

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

    def map_data(self):

        binary_map = np.ones((self.dimensions[0] + 1, self.dimensions[1] + 1), dtype=int)

        binary_map[0, :] = 0  # 第一行
        binary_map[-1, :] = 0  # 最后一行
        binary_map[:, 0] = 0  # 第一列
        binary_map[:, -1] = 0  # 最后一列

        for obs in self.obstacles:
            binary_map[obs[0], obs[1]] = 0
        self.plot_matrix_map(binary_map)

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
        # rows = 15  # 每行 10 个
        rows = 15  # 每行 10 个
        # x_range = (20, 65)
        x_range = (25, 45)
        y_range = (3, 12)
        goal_y = self.goal_y

        for i in range(num_agents):

            x = x_range[0] + (i // rows)  # x 坐标在 10 到 20 之间循环
            y = y_range[0] + (i % rows)  # y 坐标递增
            # if y == 12:
            #     start_positions.append([x, y])
            #     goal_positions.append([x, goal_y + y])  # 目标位置的 y 轴固定为 30
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

    # def plot_matrix_map(self, map_01):
    #     plt.figure()
    #     map_01 = map_01.T
    #     # 将为1的元素plot成灰色
    #     plt.imshow(map_01 == 0, cmap='Greys', origin='lower')
    #
    #     # 将为0的元素plot成白色
    #     plt.imshow(map_01 == 1, cmap='Accent_r', origin='lower', alpha=0.5)
    #
    #     # path = np.array(path)
    #     # plt.plot(path[:, 1], path[:, 0], color='lime', linewidth=2)
    #     # 显示绘制结果
    #     plt.show()

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
        plt.savefig('map.eps', bbox_inches='tight', dpi=300)
        plt.show()

map1 = map_gene()
map1.main()