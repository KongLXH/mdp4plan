import matplotlib.pyplot as plt
import numpy as np
import utils
from Grid_design import *


def double_vortex():
    x, y = np.meshgrid(np.arange(-1, grid_line + 1, 1), np.arange(-1, grid_line + 1, 1))
    u = -np.pi * np.sin(2 * np.pi / (grid_line - 1) * x) * np.cos(2 * np.pi / (grid_line - 1) * y)
    v = np.pi * np.sin(2 * np.pi / (grid_line - 1) * y) * np.cos(2 * np.pi / (grid_line - 1) * x)

    # fig, ax = plt.subplots(figsize=(6, 6))
    m = np.hypot(u, v)
    ax.set_title('wind-driven double gyre model')
    ax.quiver(x, y, u, v, m, pivot='mid', units='inches')


def roms_map(unp, vnp):  # ROMS建图

    def obs_pass(vf):
        for i in range(grid_line):
            for j in range(grid_line):
                if vf[i, j] < -99:
                    vf[i, j] = 0
        return vf

    x, y = np.meshgrid(np.arange(0, grid_line, 1), np.arange(0, grid_line, 1))
    u = utils.flow_read(unp)
    v = utils.flow_read(vnp)
    u = obs_pass(u)
    v = obs_pass(v)
    # fig, ax = plt.subplots(figsize=(6, 6))
    m = np.hypot(u, v)
    ax.set_title('sub_CA_ROMS')
    ax.quiver(x, y, u, v, m, pivot='mid', units='inches')


def make_obstacles_bf(o):
    xs = np.array([])
    ys = np.array([])

    for i, _ in enumerate(o):
        obs_x, obs_y = dim_change1_2(o[i])
        xs = np.append(xs, obs_x)
        ys = np.append(ys, obs_y)

    return xs, ys


def obs_index(f):  # 障碍物识别
    o = []
    for i in range(grid_total):
        if float(f[i]) < -999:  # v(obs) = -9999
            o.append(i)
    return o


if __name__ == '__main__':
    fig = plt.figure(figsize=(6, 6))
    ax = fig.add_subplot(1, 1, 1)

    flow_x = utils.flow_read_txt("2flow_x0.txt")
    obs = obs_index(flow_x)
    obs_xs, obs_ys = make_obstacles_bf(obs)

    # plt.scatter(obs_xs, obs_ys, c="k", s=99, marker="s", label='obstacles')  # 绘制障碍物

    # roms_map("2unp0.npy", "2vnp0.npy")
    double_vortex()

    plt.xlim(-1, grid_line)
    plt.ylim(-1, grid_line)

    # plt.axis("off")

    plt.show()
