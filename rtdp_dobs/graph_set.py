from env_set import *
from node_set import Node
import pickle
import matplotlib.pyplot as plt

graph = {}


def build_up_graph(grid, save_path):
    max_vel_1 = max_vel + 1

    # velocity dimension
    vel_list = []
    for i_vel in range(-max_vel_1 + 1, max_vel_1):
        for j_vel in range(-max_vel_1 + 1, max_vel_1):
            vel_list.append([i_vel, j_vel])

    # position dimension
    x_idx, y_idx = np.where(grid == FREE)
    coord = np.stack([x_idx, y_idx], axis=1)
    for p_idx in range(coord.shape[0]):
        pnt = coord[p_idx]
        for vel in vel_list:
            state = Node(pnt[0], pnt[1], vel[0], vel[1])
            state.connect_to_graph(grid)
            graph[state.key] = state

    for pnt in [STA_P]:
        for vel in vel_list:
            state = Node(pnt[0], pnt[1], vel[0], vel[1])
            state.connect_to_graph(grid)
            graph[state.key] = state

    for pnt in [END_P]:
        state = Node(pnt[0], pnt[1], 0, 0)
        state.is_goal = True
        graph[state.key] = state

    output = open(save_path, 'wb')
    pickle.dump(graph, output)
    print("Graph: set over!")


def check_graph(grid):
    plt.figure(figsize=(6, 6))
    # plt.pcolor(grid, edgecolors='k', linewidths=1)
    plt.pcolor(grid, cmap='binary')
    for key in graph.keys():
        for child_idx, child_key in enumerate(graph[key].next_prob_1):  # or next_prob_1
            ux, uy = ACTION_SPACE[child_idx]
            vx, vy = graph[key].vx + ux, graph[key].vy + uy
            child = graph[child_key]
            # check a specific connection
            # plt.title(str(vy) + '_' + str(vx))
            # plt.show()
            if [child.px, child.py] in STA_P:
                print('found')
                continue
            plt.arrow(graph[key].py + 0.5, graph[key].px + 0.5,
                      child.py - graph[key].py, child.px - graph[key].px,
                      color='r', head_width=0.3, head_length=0.1)
            print(key, child_idx)
        # end for
    # end for
    plt.show()


if __name__ == '__main__':

    path = './solution/graph_dp.dat'
    track_map = flow_env

    check_graph(flow_env)
    build_up_graph(track_map, path)
