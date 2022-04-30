import matplotlib.pyplot as plt
from matplotlib.offsetbox import AnchoredText
import numpy as np
import utils
from Grid_design import *


def double_vortex(k):
    x, y = np.meshgrid(np.arange(-1, grid_line + 1, 1), np.arange(-1, grid_line + 1, 1))
    u = -np.pi * np.sin(2 * np.pi / (grid_line - 1) * x) * np.cos(2 * np.pi / (grid_line - 1) * y)
    v = np.pi * np.sin(2 * np.pi / (grid_line - 1) * y) * np.cos(2 * np.pi / (grid_line - 1) * x)

    m = np.hypot(u, v)
    ax.set_title('wind-driven double gyre model')
    q = ax.quiver(x, y, u, v, m, pivot='mid', units='inches')
    ax.quiverkey(q, X=0.71, Y=1.028, U=3, label='Vf(max) = 1.8 m/s', labelpos='E')
    ax.add_artist(k)


def average_flow(k, a):
    x, y = np.meshgrid(np.arange(0, grid_line, 1), np.arange(0, grid_line, 1))
    u = np.full((grid_line, grid_line), a)
    v = np.full((grid_line, grid_line), a)

    m = np.hypot(u, v)
    ax.set_title('average flow')
    q = ax.quiver(x, y, u, v, m, pivot='mid', units='inches')
    ax.quiverkey(q, X=0.71, Y=1.028, U=0.1, label='Vf(max) = 0.2 m/s', labelpos='E')
    ax.add_artist(k)


def roms_map(unp, vnp, k):  # ROMS建图

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
    q = ax.quiver(x, y, u, v, m, pivot='mid', units='inches')
    ax.quiverkey(q, X=0.71, Y=1.028, U=0.1, label='Vf(max) = 0.2 m/s', labelpos='E')
    ax.add_artist(k)


def make_path(sx, sy, gx, gy, p):
    s = start
    x_list = np.array([sx])
    y_list = np.array([sy])

    for i in range(grid_total):
        s += ds_actions[p[s]]
        xn, yn = dim_change1_2(s)
        x_list = np.append(x_list, xn)
        y_list = np.append(y_list, yn)

        if p[s] == 'nnenebnefeesesbesfsswswbswfwwnwnbwnf':
            x_list = np.append(x_list, gx)
            y_list = np.append(y_list, gy)
            break

    return x_list, y_list


def make_obstacles_bp(p):
    xs = np.array([])
    ys = np.array([])

    for i in range(grid_total):
        if p[i] == "-":
            obs_x, obs_y = dim_change1_2(i)
            xs = np.append(xs, obs_x)
            ys = np.append(ys, obs_y)

    return xs, ys


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


def roms_t():  # TODO 目前时间间隔dt不受动作决策a影响
    start_x, start_y = dim_change1_2(start)
    goal_x, goal_y = dim_change1_2(goal)
    s = start

    path_x = np.array([start_x])
    path_y = np.array([start_y])

    flow_x = utils.flow_read_txt("flow_x0.txt")
    obs = obs_index(flow_x)
    obs_xs, obs_ys = make_obstacles_bf(obs)

    dobs_xs, dobs_ys = [], []
    dx, dy = 3, 8

    for i in range(73):
        plt.cla()

        roms_map(f"unp{i}.npy", f"vnp{i}.npy", at)

        plt.scatter(obs_xs, obs_ys, c="k", s=99, marker="s", label='obstacles')  # 绘制障碍物

        dobs_xs = np.append(dobs_xs, dx)
        dobs_ys = np.append(dobs_ys, dy)
        plt.scatter(dobs_xs, dobs_ys, c="navy", s=99, marker=">", label='dynamic_obstacles')  # 绘制障碍物
        dobs_xs = np.delete(dobs_xs, -1)
        dobs_ys = np.delete(dobs_ys, -1)
        dx += 1

        policy = utils.policy_read(f"policy{i}.txt")
        s += ds_actions[policy[s]]
        xn, yn = dim_change1_2(s)
        path_x = np.append(path_x, xn)
        path_y = np.append(path_y, yn)

        # path_x, path_y = make_path(start_x, start_y, goal_x, goal_y, policy)

        plt.plot(path_x, path_y, c="r", label='decision-path')  # 绘制路线
        plt.scatter(path_x, path_y, c="b", s=30, label='way-points')  # 绘制路径点

        plt.xlim(-1, grid_line)
        plt.ylim(-1, grid_line)
        plt.xlabel('x(km)')
        plt.ylabel('y(km)')
        plt.legend(loc='lower right', framealpha=0.5)
        plt.title('MDP-Planner')

        plt.pause(0.3)

        if goal_x in path_x:
            plt.scatter(start_x, start_y, c="lime", s=30)
            plt.scatter(goal_x, goal_y, c="gold", s=30)
            break


def static_plt():
    start_x, start_y = dim_change1_2(start)
    goal_x, goal_y = dim_change1_2(goal)

    policy = utils.policy_read("policy0.txt")

    obs_xs, obs_ys = make_obstacles_bp(policy)

    plt.scatter(obs_xs, obs_ys, c="k", s=99, marker="s", label='obstacles')  # 绘制障碍物

    path_x, path_y = make_path(start_x, start_y, goal_x, goal_y, policy)

    plt.plot(path_x, path_y, c="r", label='decision-path')  # 绘制路线
    plt.scatter(path_x, path_y, c="b", s=10, label='way-points')  # 绘制路径点

    plt.scatter(start_x, start_y, c="lime", s=30)
    plt.scatter(goal_x, goal_y, c="gold", s=30)

    plt.xlim(-1, grid_line)
    plt.ylim(-1, grid_line)
    plt.xlabel('x(km)')
    plt.ylabel('y(km)')
    plt.legend(loc='lower right', framealpha=0.5)
    plt.title('MDP-Planner(T)')
    plt.show()


if __name__ == '__main__':
    ds_actions = {"n": grid_line, "ne": grid_line + 1, "neb": 2 * grid_line + 1, "nef": grid_line + 2,
                  "e": 1, "es": -grid_line + 1, "esb": -grid_line + 2, "esf": -2 * grid_line + 1,
                  "s": -grid_line, "sw": -grid_line - 1, "swb": -2 * grid_line - 1, "swf": -grid_line - 2,
                  "w": -1, "wn": grid_line - 1, "wnb": grid_line - 2, "wnf": 2 * grid_line - 1}  # 行为对状态的改变

    fig = plt.figure(figsize=(6, 6))
    ax = fig.add_subplot(1, 1, 1)

    at = AnchoredText("kd = 0.5, kf = 1.0, kr = -1.0", prop=dict(size=10), frameon=True, loc='upper left')
    at.patch.set_boxstyle("square, pad=0.")
    at.patch.set(alpha=0.5)

    # average_flow(at, 0.1)
    double_vortex(at)
    # roms_map("un_np.npy", "vn_np.npy", at)
    static_plt()

    # roms_t()
