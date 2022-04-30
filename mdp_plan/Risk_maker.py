import numpy as np
import utils
import matplotlib.pyplot as plt
from Grid_design import *


def robs_index(f):  # 障碍物识别
    o = []
    for i in range(grid_total):
        if float(f[i]) < -999:
            o.append(i)
    return o


'''flow_x = utils.flow_read_txt("2flow_x0.txt")
flow_y = utils.flow_read_txt("2flow_y0.txt")
obs = robs_index(flow_x)'''


def robs_check(s):  # 检查障碍物 TODO: 跨网格移动时未检测障碍物
    pos_x, pos_y = dim_change1_2(s)

    obs_flag = False

    '''if s in obs:
        obs_flag = True  # 障碍物检查'''

    if pos_x in [12, 13, 14] and pos_y in [15, 16, 17] or \
            pos_x in [7, 8, 9] and pos_y in [20, 21, 22]:
        obs_flag = True  # 障碍物检查

    return obs_flag


def get_robs():
    o = []

    for s in range(grid_total):
        if robs_check(s):
            o.append(s)

    return o


def get_risk(s, o, dr):  # TODO 只取最近的障碍物能量，可以拓展
    min_id = -1
    d_min = float("inf")
    pos_x, pos_y = dim_change1_2(s)

    for i, _ in enumerate(o):
        obs_x, obs_y = dim_change1_2(o[i])
        d = np.hypot(pos_x - obs_x, pos_y - obs_y)
        if d_min >= d:
            d_min = d
            min_id = i

    obs_x, obs_y = dim_change1_2(o[min_id])
    dq = np.hypot(pos_x - obs_x, pos_y - obs_y)

    if dq <= dr:
        if dq <= 0.1:
            dq = 0.1

        risk = 20 * pow((1 - dq / dr), 2)
        return risk
    else:
        return 0


def get_dis(s):  # TODO Heuristic
    target_px, target_py = dim_change1_2(goal)
    px, py = dim_change1_2(s)
    dis = ((target_px - px) ** 2 + (target_py - py) ** 2) ** 0.5

    return 0.001*dis


def get_risk_m(obs):
    # obs = get_robs()
    risk_matrix = np.zeros(grid_total)

    if obs:
        for s in range(grid_total):
            risk_matrix[s] = get_risk(s, obs, 2.5) + get_dis(s)
        print("get risk map !")
    else:
        for s in range(grid_total):
            risk_matrix[s] = get_dis(s)
        print("no collision !")

    return risk_matrix


def draw_heatmap(obs):
    risk_m = get_risk_m(obs)
    risk_2m = np.zeros((grid_line, grid_line))
    for s in range(grid_total):
        pos_x, pos_y = dim_change1_2(s)
        risk_2m[pos_x, pos_y] = risk_m[s]

    risk_2m = np.array(risk_2m).T

    plt.pcolor(risk_2m)
    plt.axis("off")
    plt.axis("equal")


def diy_obs():
    obs = get_robs()
    risk_m = get_risk_m(obs)
    utils.risk_save(risk_m, "risk.txt")
    draw_heatmap(obs)


if __name__ == '__main__':
    pass
