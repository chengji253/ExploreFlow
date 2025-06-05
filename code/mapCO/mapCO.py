# 生成主要地图 用于避碰 map collision avoidance

class mapCo:

    def __init__(self):
        self.obstacles = None

        self.resolution = None

        self.boundary = None
        self.boundary_obs = None

    def init_mapCo_one(self, mapInfo, resolution):

        self.resolution = resolution

        x_size = mapInfo.x_size / self.resolution
        y_size = mapInfo.y_size / self.resolution

        self.boundary = [[0, x_size], [0, y_size]]

        self.init_obs(mapInfo)

    def init_obs(self, mapInfo):
        # obs [x, y, radius]
        # self.obstacles_1 = self.generate_obs_mapInfo(mapInfo)
        # self.obstacles_2 = self.gene_obs_one()
        # self.obstacles = self.obstacles_1 + self.obstacles_2
        self.obstacles = self.generate_obs_mapInfo(mapInfo)

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

    def gene_obs_one(self):
        obs_idx = [[10, 20], [20, 20], [30, 20], [40, 20], [50, 20],
                        [5, 25], [16, 25], [26, 25], [34, 25], [45, 25],
                        [9, 30], [30, 30], [40, 30],
                        [11, 35], [23, 35], [29, 35], [35, 35], [43, 35],
                        [6, 40], [25, 40], [32, 40], [45, 40],
                        ]
        circles_all = []
        radius = 1.1
        for obs in obs_idx:
            id_x = obs[0] + 1
            id_y = obs[1] + 1
            circles_all.append([id_x, id_y, radius])
        return circles_all

    def generate_obs_mapInfo(self, mapInfo):
        circles_all = []
        map_01 = mapInfo.map_01
        radius = 0.6
        rows, columns = map_01.shape
        for i in range(rows):
            for j in range(columns):
                # print(map_01[i, j])
                # if i == 0 or j == 0 or i == rows - 1 or j == columns - 1:
                #     circles_all.append([i, j, radius])
                if map_01[i, j] == 1:
                    circles_all.append([i, j, radius])
        return circles_all

