import math
import random
import time
import os
from ..Vector2 import *
from ..ORCA_Simulator import *
from ..plotPic import *


# 定义常量
RVO_TWO_PI = 2 * math.pi


def setup_scenario(simulator, goals):
    # 设置随机种子
    random.seed(int(time.time()))

    # 设置模拟的全局时间步长
    simulator.setTimeStep(0.25)

    # 设置后续添加的智能体的默认参数
    simulator.setAgentDefaults(15.0, 10, 5.0, 5.0, 2.0, 5.0)

    # 添加智能体，指定其起始位置，并存储它们在环境另一侧的目标位置
    simulator.addAgent(Vector2(55.0, 55.0))
    goals.append(Vector2(-75.0, -75.0))

    simulator.addAgent(Vector2(55.0, 60.0))
    goals.append(Vector2(-75.0, -75.0))
    # for i in range(5):
    #     for j in range(5):
    #         simulator.addAgent(Vector2(55.0 + i * 10.0, 55.0 + j * 10.0))
    #         goals.append(Vector2(-75.0, -75.0))
    #
    #         simulator.addAgent(Vector2(-55.0 - i * 10.0, 55.0 + j * 10.0))
    #         goals.append(Vector2(75.0, -75.0))
    #
    #         simulator.addAgent(Vector2(55.0 + i * 10.0, -55.0 - j * 10.0))
    #         goals.append(Vector2(-75.0, 75.0))
    #
    #         simulator.addAgent(Vector2(-55.0 - i * 10.0, -55.0 - j * 10.0))
    #         goals.append(Vector2(75.0, 75.0))

    # 添加多边形障碍物，按逆时针顺序指定其顶点
    obstacle1 = [Vector2(-10.0, 40.0), Vector2(-40.0, 40.0),
                 Vector2(-40.0, 10.0), Vector2(-10.0, 10.0)]
    obstacle2 = [Vector2(10.0, 40.0), Vector2(10.0, 10.0),
                 Vector2(40.0, 10.0), Vector2(40.0, 40.0)]
    obstacle3 = [Vector2(10.0, -40.0), Vector2(40.0, -40.0),
                 Vector2(40.0, -10.0), Vector2(10.0, -10.0)]
    obstacle4 = [Vector2(-10.0, -40.0), Vector2(-10.0, -10.0),
                 Vector2(-40.0, -10.0), Vector2(-40.0, -40.0)]

    simulator.obs_info.append(obstacle1)
    simulator.obs_info.append(obstacle2)
    simulator.obs_info.append(obstacle3)
    simulator.obs_info.append(obstacle4)
    simulator.obs_info_deal()
    simulator.goals_info = goals

    simulator.addObstacle(obstacle1)
    simulator.addObstacle(obstacle2)
    simulator.addObstacle(obstacle3)
    simulator.addObstacle(obstacle4)

    # 处理障碍物，以便在模拟中考虑它们
    simulator.processObstacles()


def update_visualization(simulator):
    # 输出当前全局时间
    # print("time=" + str(simulator.getGlobalTime()))
    boundary = [[-100, 100], [-100, 100]]
    name = "test" + "/snap" + str(simulator.getGlobalTime()) + ".png"
    visualize_traj_dynamic(simulator.agents_, simulator.obs_info, simulator.goals_info, boundary, name)

    # for i in range(simulator.getNumAgents()):
    #     print(f" {simulator.getAgentPosition(i)}", end='')
    # print()

def mkdir_file(name_n):
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


def set_preferred_velocities(simulator, path_all):
    for i in range(simulator.getNumAgents()):
        if len(path_all[i]) == 0:
            continue
        goal_vector = path_all[i][-1] - simulator.getAgentPosition(i)
        if goal_vector * goal_vector > 1.0:
            goal_vector = goal_vector / abs(goal_vector)

        simulator.setAgentPrefVelocity(i, goal_vector)

        # 稍微扰动以避免由于完美对称导致的死锁
        angle = random.random() * RVO_TWO_PI
        dist = random.random() * 0.0001
        perturb_vector = Vector2(math.cos(angle), math.sin(angle)) * dist
        simulator.setAgentPrefVelocity(i, simulator.getAgentPrefVelocity(i) + perturb_vector)


def reached_goal(position,  goal):
    if (position - goal) * (position - goal) > 100.0:
        return False
    return True

def check_path(simulator, path_all):
    for i in range(simulator.getNumAgents()):
        goal = path_all[i][-1]
        position = simulator.getAgentPosition(i)
        if reached_goal(position, goal) is True:
            path_all[i].pop()


def reached_goal_all(simulator, goals):
    # 检查所有智能体是否都到达了它们的目标
    for i in range(simulator.getNumAgents()):
        if (simulator.getAgentPosition(i) - goals[i]) * (simulator.getAgentPosition(i) - goals[i]) > 100.0:
            return False
    return True


def main():
    # 存储智能体的目标
    goals = []

    # 创建一个新的模拟器实例
    simulator = RVOSimulator()

    # 设置场景
    setup_scenario(simulator, goals)

    path_all = [[Vector2(75.0, 0.0),
                 Vector2(0.0, -75.0),
                 Vector2(-75.0, -75.0)],
                [Vector2(75.0, 0.0),
                 Vector2(0.0, -75.0),
                 Vector2(-75.0, -75.0)]
                ]

    for sub_list in path_all:
        sub_list.reverse()
    # print(path_all)

    name_n = "test" + "/snap" + str(2) + ".png"
    mkdir_file(name_n)
    # 执行并操作模拟
    t = 0
    while not reached_goal_all(simulator, goals):
        t += 1
        if t % 10 == 0:
            print("t=" + str(t))
            update_visualization(simulator)
        check_path(simulator, path_all)
        set_preferred_velocities(simulator, path_all)
        simulator.doStep()

    del simulator


if __name__ == "__main__":
    main()