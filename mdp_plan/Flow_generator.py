import utils
import numpy as np
from Grid_design import *


def average_flow(f):
    flow = [f for _ in range(grid_total)]
    utils.flow_save(flow, "flow.txt")
    print("generate average flow !")


def area_flow(f):
    flow = ['o' for _ in range(grid_total)]
    tunnel = []
    for i in range(5, 13):
        for j in range(6, 12):
            s = dim_change2_1(i, j)
            tunnel.append(s)
    for i in range(grid_total):
        if i in tunnel:
            flow[i] = f
    utils.flow_save(flow, "flow.txt")
    print("generate area flow !")


def double_vortex(a):
    flow_x = np.array([0.0 for _ in range(grid_total)])
    flow_y = np.array([0.0 for _ in range(grid_total)])

    for x in range(grid_line):
        for y in range(grid_line):
            vx = -a * np.sin(np.pi * (2 / (grid_line - 1)) * x) * np.cos(np.pi * (2 / (grid_line - 1)) * y)
            vy = a * np.cos(np.pi * (2 / (grid_line - 1)) * x) * np.sin(np.pi * (2 / (grid_line - 1)) * y)
            s = dim_change2_1(x, y)
            flow_x[s] = vx
            flow_y[s] = vy

    flow_v = np.hypot(flow_x, flow_y)
    flow_av = np.mean(flow_v)

    utils.flow_save(flow_x, "flow_x.txt")
    utils.flow_save(flow_y, "flow_y.txt")
    print(f"generate double_vortex! v(max)={a}m/s, v(aver)={flow_av}m/s")
    return flow_av


if __name__ == '__main__':
    pass
