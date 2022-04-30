import numpy as np
import utils
import Flow_generator
from scipy import integrate
from Grid_design import *

q1 = 1
q2 = pow(2, 0.5)
q5 = pow(5, 0.5)

unit = 100  # grid_size

A = ["n", "ne", "neb", "nef",
     "e", "es", "esb", "esf",
     "s", "sw", "swb", "swf",
     "w", "wn", "wnb", "wnf"]  # 行为空间

Flow_generator.double_vortex(1.8)
flow_x = utils.flow_read_txt("flow_x.txt")
flow_y = utils.flow_read_txt("flow_y.txt")
obs = []


def sobs_index(f):  # 障碍物识别
    o = []
    for _ in range(grid_total):
        if float(f[_]) < -99:  # v(obs) = -9999
            o.append(_)
    return o


'''flow_x = utils.flow_read_txt("flow_x6.txt")
flow_y = utils.flow_read_txt("flow_y6.txt")
flow_dx = utils.flow_read_txt("flow_dx.txt")
flow_dy = utils.flow_read_txt("flow_dy.txt")
obs = sobs_index(flow_x)'''

act_speed = 2.0

prob_matrix = np.zeros((grid_total, 16, 16))


def get_time(a):  # 计算动作a执行的时间长
    time = 1

    if a in ["n", "e", "s", "w"]:
        time = 1 * unit / act_speed
    elif a in ["ne", "es", "sw", "wn"]:
        time = q2 * unit / act_speed
    elif a in ["neb", "nef", "esb", "esf", "swb", "swf", "wnb", "wnf"]:
        time = q5 * unit / act_speed

    return time


def get_sig(a, s):
    sig_vfx = abs(0.15 * float(flow_x[s]))  # 流场x方向流速高斯分布标准差
    sig_vfy = abs(0.15 * float(flow_y[s]))  # 流场y方向流速高斯分布标准差

    # sig_vfx = abs(5.0 * float(flow_dx[s]))  # 流场x方向流速高斯分布标准差
    # sig_vfy = abs(5.0 * float(flow_dy[s]))  # 流场y方向流速高斯分布标准差

    if sig_vfx < 0.00001:
        sig_vfx = 0.00001
    if sig_vfy < 0.00001:
        sig_vfy = 0.00001

    dt = get_time(a)

    sif_x = sig_vfx * dt
    sig_y = sig_vfy * dt

    return sif_x, sig_y


def compute_dn(c):  # 计算s到s_next的执行距离
    d = [0, 0]

    if c == "n":
        d = [0.000, 1.000]
    elif c == "ne":
        d = [1.000, 1.000]
    elif c == "neb":
        d = [1.000, 2.000]
    elif c == "nef":
        d = [2.000, 1.000]
    elif c == "e":
        d = [1.000, 0.000]
    elif c == "es":
        d = [1.000, -1.000]
    elif c == "esb":
        d = [2.000, -1.000]
    elif c == "esf":
        d = [1.000, -2.000]
    elif c == "s":
        d = [0.000, -1.000]
    elif c == "sw":
        d = [-1.000, -1.000]
    elif c == "swb":
        d = [-1.000, -2.000]
    elif c == "swf":
        d = [-2.000, -1.000]
    elif c == "w":
        d = [-1.000, 0.000]
    elif c == "wn":
        d = [-1.000, 1.000]
    elif c == "wnb":
        d = [-2.000, 1.000]
    elif c == "wnf":
        d = [-1.000, 2.000]

    d = np.array(d)
    d *= unit

    return d


def compute_dr(s, a):  # 计算auv对流相对速度（推进速度）作用下的移动距离

    act_vector = utils.char_trans(a, act_speed)
    flow_vector = [float(flow_x[s]), float(flow_y[s])]

    auv_vector_x = act_vector[0] - flow_vector[0]
    auv_vector_y = act_vector[1] - flow_vector[1]

    auv_vector = [auv_vector_x, auv_vector_y]
    auv_vector = np.array(auv_vector)
    dr = auv_vector * get_time(a)
    dr = np.array(dr)

    return dr


def compute_df(s, a):  # 计算流场（确定性部分）作用下的移动距离
    vf = [float(flow_x[s]), float(flow_y[s])]
    vf = np.array(vf)
    df = vf * get_time(a)
    xf = df[0]
    yf = df[1]

    return xf, yf


def compute_df_(s, a, s_next):  # 计算从s到s_next，已知推进速度，理论上流场需要作用的距离

    df_ = compute_dn(s_next) - compute_dr(s, a)
    xf_ = df_[0]
    yf_ = df_[1]

    return xf_, yf_


def compute_prob(s, a, s_next):  # 计算状态转移概率
    """
    TODO：1.时间由a决定了，但到达s_next的时间可能不一致;2.状态转移概率中的流速只考虑了当前状态s的流速（与R方式有差异）;3.目前结果与s无关
    """
    sig_x, sig_y = get_sig(a, s)
    xf, yf = compute_df(s, a)
    xf_, yf_ = compute_df_(s, a, s_next)

    def f(x, y):
        func = np.exp(-((pow((x - xf), 2) / (2 * pow(sig_x, 2))) + (pow((y - yf), 2) / (2 * pow(sig_y, 2)))))
        return func

    def bounds_y(*args):
        by = [0, 0]
        by[0] = yf_ - unit / 2
        by[1] = yf_ + unit / 2
        return by

    def bounds_x(*args):
        bx = [0, 0]
        bx[0] = xf_ - unit / 2
        bx[1] = xf_ + unit / 2
        return bx

    result = integrate.nquad(f, [bounds_x, bounds_y])
    prob = (1.0 / (2.0 * np.pi * sig_x * sig_y)) * result[0]

    if prob > 1:
        prob = 1.0
    elif prob < 0.00000000001:
        prob = 0.0

    # print(prob)

    return prob


'''for s in range(grid_total):
    for a, i in zip(A, range(16)):
        for a1, j in zip(A, range(16)):
            if i == j:
                prob_matrix[s, i, j] = 1.0
            else:
                prob_matrix[s, i, j] = 0.0'''


# compute_prob(90, "e", "n")

'''prob_m = utils.prob_read("prob_matrix.npy")

for s in range(grid_total):
    for a, i in zip(A, range(16)):
        for a1, j in zip(A, range(16)):
            if i == j and prob_m[s, i, j] < 0.9:
                print(f"{s}:")
                print(prob_m[s, i, j])
            else:
                pass'''

for s in range(grid_total):
    if s in obs:
        pass
    else:
        for a, i in zip(A, range(16)):
            for a1, j in zip(A, range(16)):
                prob_matrix[s, i, j] = compute_prob(s, a, a1)
                if i == j and prob_matrix[s, i, j] < 0.5:
                    prob_matrix[s, i, j] = 0.99999
    print(f"{s}")

utils.prob_save(prob_matrix, "prob_matrix.npy")
