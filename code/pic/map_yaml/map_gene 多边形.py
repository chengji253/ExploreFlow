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

    def para_100(self):
        # 视频地图里面obs 保存一下
        self.dimensions = [80, 80]
        self.generate_positions()
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
        self.dimensions = [80, 100]
        # self.generate_positions()
        # self.start_positions = [[10, 1], [11, 1], [12, 1], [13, 1], [14, 1]]
        # self.goal_positions = [[10, 30], [11, 30], [12, 30], [13, 30], [14, 30]]
        # self.obs_idx = [[10, 20], [40, 20], [25, 30]]
        self.goal_y = 80
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

    def para_maze(self):
        self.dimensions = [80, 100]
        self.goal_y = 80
        # self.obs_idx = [[12, 20, 3, 30], [21, 20, 10, 5], [20, 35, 10, 3], [35, 30, 10, 3],
        #                 [20, 45, 5, 5], [30, 45, 5, 5], [40, 45, 5, 5],
        #                 [50, 20, 5, 30], [60, 20, 10, 5], [65, 30, 10, 5],
        #                 [60, 40, 10, 5], [2, 35, 3, 3], [40, 20, 5, 5], [6, 20, 3, 3],
        #                 [60, 55, 5, 20], [70, 65, 5, 5], [25, 55, 3, 20],
        #                 [5, 55, 3, 20], [15, 56, 5, 5], [12, 70, 5, 5],
        #                 [35, 55, 15, 3], [29, 65, 15, 3], [50, 70, 5, 5]
        #                 ]

    def add_polygon_obstacle(self, vertices):
        """
        添加多边形障碍物
        vertices: List[List[int]] - 多边形的顶点坐标 [[x1,y1], [x2,y2], ...]
        """
        # 保存顶点用于YAML输出
        self.obs_vertices.append(vertices)

        # 计算多边形的边界框
        x_coords = [v[0] for v in vertices]
        y_coords = [v[1] for v in vertices]
        min_x, max_x = min(x_coords), max(x_coords)
        min_y, max_y = min(y_coords), max(y_coords)

        # 使用扫描线算法填充多边形内部
        polygon_obs = []
        for x in range(min_x, max_x + 1):
            for y in range(min_y, max_y + 1):
                if self.is_point_in_polygon(x, y, vertices):
                    polygon_obs.append([x, y])

        # 添加到障碍物列表
        self.obstacles.extend(polygon_obs)
        # 添加一个代表性的内部点（使用第一个顶点）
        self.obs_inside.append([vertices[0][0], vertices[0][1], len(vertices)])

    # def is_point_in_polygon(self, x, y, vertices):
    #     """
    #     判断点是否在多边形内部（射线法）
    #     """
    #     n = len(vertices)
    #     inside = False
    #     p1x, p1y = vertices[0]
    #
    #     for i in range(n + 1):
    #         p2x, p2y = vertices[i % n]
    #         if y > min(p1y, p2y):
    #             if y <= max(p1y, p2y):
    #                 if x <= max(p1x, p2x):
    #                     if p1y != p2y:
    #                         xinters = (y - p1y) * (p2x - p1x) / (p2y - p1y) + p1x
    #                     if p1x == p2x or x <= xinters:
    #                         inside = not inside
    #         p1x, p1y = p2x, p2y
    #
    #     return inside

    def is_point_in_polygon(self, x, y, vertices):
        """
        判断点是否在多边形内部（射线法）
        如果点在边界上，则认为不属于多边形内部
        """
        n = len(vertices)

        # 第一步：检查点是否在顶点上
        for vx, vy in vertices:
            if x == vx and y == vy:
                return False  # 点在顶点上，不属于内部

        # 第二步：检查点是否在边上
        for i in range(n):
            p1x, p1y = vertices[i]
            p2x, p2y = vertices[(i + 1) % n]

            # 如果点在边的 y 范围内
            if min(p1y, p2y) <= y <= max(p1y, p2y) or min(p1y, p2y) >= y >= max(p1y, p2y):
                # 如果边是水平的，且 y 坐标匹配
                if p1y == p2y == y and min(p1x, p2x) <= x <= max(p1x, p2x):
                    return False  # 点在水平边上
                # 如果边不是水平的，检查点是否在边上
                elif p1y != p2y:
                    # 计算点在边上的 x 坐标
                    xinters = (y - p1y) * (p2x - p1x) / (p2y - p1y) + p1x
                    if x == xinters and min(p1x, p2x) <= x <= max(p1x, p2x):
                        return False  # 点在非水平边上

        # 第三步：射线法判断内部
        inside = False
        p1x, p1y = vertices[0]
        for i in range(n + 1):
            p2x, p2y = vertices[i % n]
            if y > min(p1y, p2y):
                if y <= max(p1y, p2y):
                    if x <= max(p1x, p2x):
                        if p1y != p2y:
                            xinters = (y - p1y) * (p2x - p1x) / (p2y - p1y) + p1x
                        if p1x == p2x or x <= xinters:
                            inside = not inside
            p1x, p1y = p2x, p2y

        return inside

    def add_polygon_all(self):
        # 多边形顶点需要逆时针
        polygon_all =[
        [[20, 60], [28, 55], [30, 60], [25, 65]],
        [[25, 21], [30, 25], [25, 30]],
        [[45, 21], [50, 25], [42, 30]],
        [[40, 50], [42, 43], [48, 45], [50, 50], [45, 55]],
        [[11, 35], [20, 35], [15, 45]],
        [[31, 36], [40, 36], [35, 45]],
        [[61, 31], [70, 25], [65, 40]],
        ]
        for polygon in polygon_all:
            # self.obs_vertices.append(r_polygon)
            self.add_polygon_obstacle(polygon)


    # 修改 main 函数以测试多边形
    def main(self):
        self.para_maze()
        self.generate_positions()

        # 添加一些示例多边形障碍物
        self.add_polygon_all()
        # self.generate_vertices()  # 处理矩形障碍物的顶点
        # self.generate_obstacles()  # 生成矩形障碍物
        self.generate_map()
        self.map_data()

    # def main(self):
    #     # self.para_100()
    #     # self.para_one()
    #     self.para_maze()
    #     self.generate_positions()
    #     self.generate_vertices()
    #     self.generate_obstacles()
    #     self.generate_map()
    #     self.map_data()

    def generate_vertices(self):
        # 根据给出的obs forest 直接生成多边形顶点 并输出到文件中
        self.obs_vertices = []
        if self.obs_idx is None:
            return
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

        num_agents = 500
        rows = 15  # 每行 10 个
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
        with open("m1-v.yaml", "w") as file:
            yaml.dump(yaml_data, file, default_flow_style=None, sort_keys=False)
        print("YAML 文件已生成")

    def plot_matrix_map(self, map_01):
        plt.figure()
        map_01 = map_01.T
        # 将为1的元素plot成灰色
        plt.imshow(map_01 == 0, cmap='Greys', origin='lower')

        # 将为0的元素plot成白色
        plt.imshow(map_01 == 1, cmap='Accent_r', origin='lower', alpha=0.5)

        # path = np.array(path)
        # plt.plot(path[:, 1], path[:, 0], color='lime', linewidth=2)
        # 显示绘制结果
        plt.show()


map1 = map_gene()
map1.main()