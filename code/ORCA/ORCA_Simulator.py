from .KdTree import *
from .Vector2 import *
from .Agent import *
from .Obstacle import *


"""
* @brief 构造一个模拟器实例并为添加的任何新代理设置默认属性。
* @param[in] timeStep 模拟的时间步长。必须为正数。
* @param[in] neighbourDist 新代理在导航中考虑的中心点到其他代理之间的默认最大距离。
                            此数字越大，模拟的运行时间越长。如果数字太低，模拟将不安全。必须为非负数。
* @param[in] maxNeighbors 新代理在导航中考虑的其他代理的默认最大数量。
* 此数字越大，模拟的运行时间越长。如果数字太低，模拟将不安全。
* @param[in] timeHorizo​​n 模拟计算得出的新代理相对于其他代理而言安全的速度的默认最短时间。
    此数字越大，代理对其他代理的存在做出反应的速度越快，但代理在选择速度时拥有的自由度就越小。必须为正数。
* @param[in] timeHorizo​​nObst 模拟计算得出的新代理相对于障碍物而言安全的速度的默认最短时间。
    此数字越大，代理对障碍物的存在做出反应的速度越快，但代理在选择速度时拥有的自由度就越小。必须为正数。
* @param[in] radius 新代理的默认半径。必须为非负数。
* @param[in] maxSpeed 新代理的默认最大速度。必须非负。
"""

class ORCA_Simulator:
    RVO_ERROR = float('inf')

    def __init__(self, timeStep=0.0, neighborDist=0.0, maxNeighbors=0, timeHorizon=0.0,
                 timeHorizonObst=0.0, radius=0.0, maxSpeed=0.0, velocity=None):
        self.agents_ = []
        self.obstacles_ = []

        self.agents_exist_ = []

        self.obs_info = []
        self.goals_info = []

        self.defaultAgent_ = None
        self.kdTree_ = KdTree(self)  # 假设KdTree类已定义

        self.globalTime_ = 0.0
        self.timeStep_ = timeStep

        if velocity is None:
            velocity = Vector2()

        if timeStep != 0.0:
            self.defaultAgent_ = Agent()
            self.defaultAgent_.maxNeighbors_ = maxNeighbors
            self.defaultAgent_.maxSpeed_ = maxSpeed
            self.defaultAgent_.neighborDist_ = neighborDist
            self.defaultAgent_.radius_ = radius
            self.defaultAgent_.timeHorizon_ = timeHorizon
            self.defaultAgent_.timeHorizonObst_ = timeHorizonObst
            self.defaultAgent_.velocity_ = velocity

    def __del__(self):
        if self.defaultAgent_:
            del self.defaultAgent_
        if self.kdTree_:
            del self.kdTree_
        for agent in self.agents_:
            del agent
        for obstacle in self.obstacles_:
            del obstacle

    def init_simulator(self):
        self.agents_exist_ = [True for _ in range(self.getNumAgents())]

    def setTimeStep(self, timeStep):
        self.timeStep_ = timeStep

    def getGlobalTime(self):
        return self.globalTime_

    def getNumAgents(self):
        return len(self.agents_)

    def addAgent(self, position, neighborDist=None, maxNeighbors=None, timeHorizon=None,
                 timeHorizonObst=None, radius=None, maxSpeed=None, velocity=None):
        if self.defaultAgent_:
            if neighborDist is None:
                neighborDist = self.defaultAgent_.neighborDist_
            if maxNeighbors is None:
                maxNeighbors = self.defaultAgent_.maxNeighbors_
            if timeHorizon is None:
                timeHorizon = self.defaultAgent_.timeHorizon_
            if timeHorizonObst is None:
                timeHorizonObst = self.defaultAgent_.timeHorizonObst_
            if radius is None:
                radius = self.defaultAgent_.radius_
            if maxSpeed is None:
                maxSpeed = self.defaultAgent_.maxSpeed_
            if velocity is None:
                velocity = self.defaultAgent_.velocity_

            agent = Agent()
            agent.position_ = position
            agent.velocity_ = velocity
            agent.id_ = len(self.agents_)
            agent.maxNeighbors_ = maxNeighbors
            agent.maxSpeed_ = maxSpeed
            agent.neighborDist_ = neighborDist
            agent.radius_ = radius
            agent.timeHorizon_ = timeHorizon
            agent.timeHorizonObst_ = timeHorizonObst

            # self.agents_[agent.id_] = agent
            self.agents_.append(agent)

            return len(self.agents_) - 1
        return self.RVO_ERROR

    def addObstacle(self, vertices):
        # 检查顶点数量是否大于1
        if len(vertices) > 1:
            # 获取当前障碍物的编号，即添加前obstacles_容器的大小
            obstacleNo = len(self.obstacles_)

            # 遍历所有顶点
            for i in range(len(vertices)):
                # 为当前顶点创建一个新的Obstacle对象
                obstacle = Obstacle()
                # 设置障碍物的位置为当前顶点
                obstacle.point_ = vertices[i]

                # 如果不是第一个顶点，设置当前障碍物的前一个障碍物
                if i != 0:
                    obstacle.previous_ = self.obstacles_[-1]
                    obstacle.previous_.next_ = obstacle
                # 如果是最后一个顶点，设置当前障碍物的下一个障碍物为该障碍物的第一个顶点
                if i == len(vertices) - 1:
                    obstacle.next_ = self.obstacles_[obstacleNo]
                    obstacle.next_.previous_ = obstacle

                # 计算障碍物的方向向量，即下一个顶点减去当前顶点并归一化
                next_index = 0 if i == len(vertices) - 1 else i + 1
                obstacle.direction_ = normalize(vertices[next_index] - vertices[i])

                # 判断障碍物是否为凸多边形
                if len(vertices) == 2:
                    # 如果只有两个顶点，认为是凸的
                    obstacle.isConvex_ = True
                else:
                    # 使用leftOf函数判断是否为凸的
                    prev_index = len(vertices) - 1 if i == 0 else i - 1
                    next_index = 0 if i == len(vertices) - 1 else i + 1
                    obstacle.isConvex_ = leftOf(vertices[prev_index], vertices[i], vertices[next_index]) >= 0
                # 设置障碍物的唯一标识
                obstacle.id_ = len(self.obstacles_)
                # 将障碍物添加到obstacles_容器中
                self.obstacles_.append(obstacle)
            return obstacleNo
        # 如果顶点数量不超过1个，返回错误码
        return self.RVO_ERROR

    def doStep(self):
        self.kdTree_.buildAgentTree()
        for agent in self.agents_:
            # print("id=" + str(agent.id_))
            agent.computeNeighbors(self.kdTree_)
            agent.computeNewVelocity(self.timeStep_)

        # 在这里对打乱的agent 排个序
        self.agents_ = sorted(self.agents_, key=lambda x: x.id_)

        for agent in self.agents_:
            agent.update(self.timeStep_)
        self.globalTime_ += self.timeStep_

    def getAgentAgentNeighbor(self, agentNo, neighborNo):
        return self.agents_[agentNo].agentNeighbors_[neighborNo].second.id_

    def getAgentMaxNeighbors(self, agentNo):
        return self.agents_[agentNo].maxNeighbors_

    def getAgentMaxSpeed(self, agentNo):
        return self.agents_[agentNo].maxSpeed_

    def getAgentNeighborDist(self, agentNo):
        return self.agents_[agentNo].neighborDist_

    def getAgentNumAgentNeighbors(self, agentNo):
        return len(self.agents_[agentNo].agentNeighbors_)

    def getAgentNumObstacleNeighbors(self, agentNo):
        return len(self.agents_[agentNo].obstacleNeighbors_)

    def getAgentNumORCALines(self, agentNo):
        return len(self.agents_[agentNo].orcaLines_)

    def getAgentObstacleNeighbor(self, agentNo, neighborNo):
        return self.agents_[agentNo].obstacleNeighbors_[neighborNo].second.id_

    def getAgentORCALine(self, agentNo, lineNo):
        return self.agents_[agentNo].orcaLines_[lineNo]

    def getAgentPosition(self, agentNo):
        return self.agents_[agentNo].position_

    def getAgentPrefVelocity(self, agentNo):
        return self.agents_[agentNo].prefVelocity_

    def getAgentRadius(self, agentNo):
        return self.agents_[agentNo].radius_

    def getAgentTimeHorizon(self, agentNo):
        return self.agents_[agentNo].timeHorizon_

    def getAgentTimeHorizonObst(self, agentNo):
        return self.agents_[agentNo].timeHorizonObst_

    def getAgentVelocity(self, agentNo):
        return self.agents_[agentNo].velocity_

    def getObstacleVertex(self, vertexNo):
        return self.obstacles_[vertexNo].point_

    def getNextObstacleVertexNo(self, vertexNo):
        return self.obstacles_[vertexNo].next_.id_

    def getPrevObstacleVertexNo(self, vertexNo):
        return self.obstacles_[vertexNo].previous_.id_

    def processObstacles(self):
        self.kdTree_.buildObstacleTree()

    def queryVisibility(self, point1, point2, radius=0.0):
        return self.kdTree_.queryVisibility(point1, point2, radius)

    def setAgentDefaults(self, neighborDist, maxNeighbors, timeHorizon, timeHorizonObst, radius, maxSpeed,
                         velocity=None):
        if velocity is None:
            velocity = Vector2()
        if self.defaultAgent_ is None:
            self.defaultAgent_ = Agent()
        self.defaultAgent_.maxNeighbors_ = maxNeighbors
        self.defaultAgent_.maxSpeed_ = maxSpeed
        self.defaultAgent_.neighborDist_ = neighborDist
        self.defaultAgent_.radius_ = radius
        self.defaultAgent_.timeHorizon_ = timeHorizon
        self.defaultAgent_.timeHorizonObst_ = timeHorizonObst
        self.defaultAgent_.velocity_ = velocity

    def setAgentMaxNeighbors(self, agentNo, maxNeighbors):
        self.agents_[agentNo].maxNeighbors_ = maxNeighbors

    def setAgentMaxSpeed(self, agentNo, maxSpeed):
        self.agents_[agentNo].maxSpeed_ = maxSpeed

    def setAgentNeighborDist(self, agentNo, neighborDist):
        self.agents_[agentNo].neighborDist_ = neighborDist

    def setAgentPosition(self, agentNo, position):
        self.agents_[agentNo].position_ = position

    def setAgentPrefVelocity(self, agentNo, prefVelocity):
        self.agents_[agentNo].prefVelocity_ = prefVelocity

    def setAgentRadius(self, agentNo, radius):
        self.agents_[agentNo].radius_ = radius

    def setAgentTimeHorizon(self, agentNo, timeHorizon):
        self.agents_[agentNo].timeHorizon_ = timeHorizon

    def setAgentTimeHorizonObst(self, agentNo, timeHorizonObst):
        self.agents_[agentNo].timeHorizonObst_ = timeHorizonObst

    def setAgentVelocity(self, agentNo, velocity):
        self.agents_[agentNo].velocity_ = velocity

    def obs_info_deal(self):
        obs_new = []
        for obs in self.obs_info:
            vertices = []
            for v in obs:
                o = (v.x_, v.y_)
                vertices.append(o)
            obs_new.append(vertices)

        self.obs_info = obs_new