from env_set import *
from node_set import Node
import pickle
import matplotlib.pyplot as plt
from copy import deepcopy
import numpy as np

graph = {}


def track_the_best_plan(spx=STA_P[0], spy=STA_P[1], svx=0, svy=0):
    start_node = Node(spx, spy, svx, svy)
    start_key = start_node.key
    state = graph[start_key]
    trajectory = [state]
    # for i in range(grid.shape[0]+grid.shape[1]) a safer condition
    while not state.is_goal:
        value_uk = []

        for child_idx in range(len(ACTION_SPACE)):
            # child_key_9 = state.next_prob_9[child_idx]
            child_key = state.next_node[child_idx]
            child_ = graph[child_key]
            value_uk.append(child_.q_value)

        child_key = state.next_node[np.argmin(value_uk)]
        # greedy policy

        state = graph[child_key]
        trajectory.append(state)
        # print(state.px, state.py)
    return trajectory


def visualize_the_best_plan(plan, grid_para):
    def plot_dobs_circle(x, y, r):
        theta = np.linspace(0, 2 * np.pi, 100)
        a = r * np.cos(theta) + x
        b = r * np.sin(theta) + y
        a2 = 2 * r * np.cos(theta) + x
        b2 = 2 * r * np.sin(theta) + y
        plt.plot(a, b, color='r', linestyle='-')
        plt.plot(a2, b2, color='g', linestyle=':')
        plt.scatter(x, y, c='k', marker='o', s=5)

    assert isinstance(plan, list)
    plt.figure(figsize=(6, 6))

    # plt.pcolor(grid_para, edgecolors='k', linewidths=1)
    # plt.pcolor(grid_para, cmap='binary')

    # dobs_xy = [[dobs[i][j] for i in range(len(dobs))] for j in range(len(dobs[0]))]
    # plt.scatter(dobs_xy[0], dobs_xy[1], c="k", s=10, marker="o")

    # for i in range(len(dobs)):
    #     plot_dobs_circle(dobs[i][0], dobs[i][1], 2 * (max_vel ** 2) ** 0.5)

    plan_len = len(plan)
    plan.append(plan[-1])
    # print(len(plan))
    for i in range(plan_len):
        plt.cla()

        plt.xlim(-1, grid_line)
        plt.ylim(-1, grid_line)

        for j in range(len(dobs)):
            plot_dobs_circle(dobs_history[i][j][0], dobs_history[i][j][1], 4 * (max_vel ** 2) ** 0.5)

        for k in range(i + 1):
            plt.scatter(plan[k].py, plan[k].px, c='b', marker='o', s=5)

            # plt.arrow(plan[k].py, plan[k].px, plan[i + 1].py - plan[k].py, plan[i + 1].px - plan[k].px, color='b',
            # head_width=0.3, head_length=0.1)

        plt.pause(0.05)

    plt.show()


def dynamic_programming():
    itr_num = 0
    bellman_error = np.inf
    bellman_error_list = []
    while bellman_error > 0.0001:
        itr_num += 1
        bellman_error = 0.0
        for key in graph.keys():
            state = graph[key]
            if state.is_goal:
                state.q_value = 0
            else:
                value_uk = []
                for child_idx in range(len(ACTION_SPACE)):
                    """
                    child_key_9 = state.next_prob_9[child_idx]
                    child_9 = graph[child_key_9]
                    child_key_1 = state.next_prob_1[child_idx]
                    child_1 = graph[child_key_1]
                    """

                    child_key = state.next_node[child_idx]
                    child_ = graph[child_key]
                    # expected_cost_uk = 0.9 * (1 + child_9.q_value) + 0.1 * (1 + child_1.q_value)
                    expected_cost_uk = 1 + child_.q_value
                    value_uk.append(expected_cost_uk)

                current_value = min(value_uk)
                bellman_error += np.linalg.norm(state.q_value - current_value)
                state.q_value = min(value_uk)
            # end if
        # end for
        bellman_error_list.append(bellman_error)
        print("{}th iteration: {}".format(itr_num, bellman_error))
    # end while

    plt.figure()
    x_axis = range(len(bellman_error_list))
    plt.plot(x_axis, bellman_error_list)
    plt.show()


def RTDP(greedy_prob=0.8):
    itr_num = 0
    bellman_error = np.inf
    bellman_error_list = []
    while bellman_error > 0.0001 and itr_num <= 1e4:
        itr_num += 1
        bellman_error = 0.0
        # graph is a dict of Node (=state), graph[state.key] = state
        # state.key is an unique str representing a state (px,py,vx,vy)

        trajectory = []
        # getting the trajectory from value of Q

        start_p = STA_P
        tmp = Node(start_p[0], start_p[1], 0, 0)
        #        tmp = Node(0, 0, 0, 0)

        state = graph[tmp.key]
        trajectory.append(state)
        # print("Trajectory is:",trajectory)
        #        OPEN = [state]
        #        CLOSED = []

        n = 0
        NUM_INT = 1000  # avoid looping
        while not state.is_goal and n <= NUM_INT:
            # in all reachable states finding out the next state
            n += 1
            neighbor_state_key = []
            neighbor_state_V = []
            # visited_state = []
            for child_idx in range(len(ACTION_SPACE)):
                if ACTION_SPACE[child_idx] == [0, 0]:  # forbidden stay at the same position!
                    continue

                child_key = state.next_node[child_idx]  # only consider the states that are changed
                child_ = graph[child_key]

                if child_ in trajectory:
                    continue  # forbidden re-visiting node!
                else:
                    neighbor_state_key.append(child_key)
                    neighbor_state_V.append(child_.q_value)

                    # trick: avoiding no state to extend! restart!
            if not neighbor_state_key:
                break

            # greedy or random. according to greedy_prob
            if np.random.rand() < greedy_prob:
                min_idx = np.argmin(neighbor_state_V)
                next_state_key = neighbor_state_key[min_idx]

            else:  # randomly select an action but not optimal
                rand_idx = np.random.randint(low=0, high=len(neighbor_state_key))
                next_state_key = neighbor_state_key[rand_idx]

            state = graph[next_state_key]
            trajectory.append(state)

        if n >= NUM_INT:
            #            for st in trajectory:
            #                print(st.key)
            print("It seems that there is a loop during greedy policy!")
            # raise SystemExit(0)

        # Back up the state Q value along the trajectory
        # for (state_0, state_1) in zip(trajectory[0:-1], trajectory[1:]):  # until goal found

        """
        for (state_0, state_1) in zip(trajectory[0:-1], trajectory[1:]):  # until goal found

            expected_cost_uk_star = 1 + state_1.q_value

            current_value = expected_cost_uk_star
            bellman_error += np.linalg.norm(state_0.q_value - current_value)
            state_0.q_value = current_value  # update V
        """

        for state in trajectory:
            if state.is_goal:
                state.q_value = 0
            else:
                value_uk = []
                for child_idx in range(len(ACTION_SPACE)):
                    child_key = state.next_node[child_idx]
                    child_ = graph[child_key]

                    expected_cost_uk = 1 + child_.q_value
                    value_uk.append(expected_cost_uk)

                current_value = min(value_uk)
                bellman_error += np.linalg.norm(state.q_value - current_value)
                state.q_value = min(value_uk)
            # end if
        # end for

        bellman_error_list.append(bellman_error)
        print("{}th iteration: {}".format(itr_num, bellman_error))
    # end while

    plt.figure()
    x_axis = range(len(bellman_error_list))
    plt.plot(x_axis, bellman_error_list)
    plt.show()


def get_reward(s, c, a, cd):
    reward = 0

    def get_time_reward(kh=1):
        time_reward = kh * 1
        return time_reward

    def get_dobs_reward(kr=999, dr=4 * (max_vel ** 2) ** 0.5):
        dobs_reward = 0
        d_min = float("inf")
        min_id = -1

        for i, _ in enumerate(cd):
            cd_x, cd_y = cd[i][0], cd[i][1]
            dis = np.hypot(c.px - cd_x, c.py - cd_y)
            if d_min >= dis:
                d_min = dis
                # min_id = i
        # cd_x, cd_y = cd[min_id][0], cd[min_id][1]
        # dis = np.hypot((c.px - cd_x, c.py - cd_y))
        dis = d_min
        if dis <= dr:

            if dis <= 0:
                dis = 0
                dobs_reward = 999

            dobs_reward = kr * pow((1 - dis / dr), 2)
        return dobs_reward

    reward = get_time_reward() + get_dobs_reward()
    return reward


def greedy_policy(spx, spy, svx, svy, explore, epsilon):
    start_node = Node(spx, spy, svx, svy)
    start_key = start_node.key
    state = graph[start_key]
    trajectory = [state.key]

    n = 0
    NUM_INT = 150
    while not state.is_goal and n <= NUM_INT:
        n += 1
        value_uk = []
        for child_idx in range(len(ACTION_SPACE)):
            child_key = state.next_node[child_idx]
            child_ = graph[child_key]
            value_uk.append(child_.q_value)

        action_idx = np.argmin(value_uk)

        if explore:
            action_idx = explore_action(action_idx, epsilon)
        child_key = state.next_node[action_idx]
        trajectory.append(child_key)
        state = graph[child_key]

        # print('finding feasible path:{},{}'.format(state.px, state.py))
    return trajectory


def explore_action(u_idx, epsilon):
    if np.random.uniform(0, 1) < epsilon:
        return np.random.randint(0, len(ACTION_SPACE))
    else:
        return u_idx


def check_dobs(px, py, d):
    cd = []
    for i, _ in enumerate(dobs):
        dobs_x, dobs_y = dobs[i][0], dobs[i][1]
        dis = np.hypot(px - dobs_x, py - dobs_y)
        if dis <= 8 * (max_vel ** 2) ** 0.5:
            cd.append(d[i])
            # print(f"check dobs{i}")

    return cd


def RT_dynamic_programming(spx=STA_P[0], spy=STA_P[1], svx=0, svy=0, explore=True, epsilon=0.2):
    itr_num = 0
    bellman_error = np.inf
    bellman_error_list = []
    # cdobs = check_dobs(spx, spy, dobs)
    cdobs = dobs

    while bellman_error > 0.0001 and itr_num <= 1e4:
        itr_num += 1
        bellman_error = 0.0
        greedy_plan = greedy_policy(spx, spy, svx, svy, explore, epsilon)

        for key in greedy_plan:

            # if [graph[key].px, graph[key].py] in dobs:

            state = graph[key]
            if state.is_goal:
                state.q_value = 0
            else:
                value_uk = []
                for child_idx in range(len(ACTION_SPACE)):
                    child_key = state.next_node[child_idx]
                    child_ = graph[child_key]

                    # expected_cost_uk = 1 + child_.q_value
                    expected_cost_uk = get_reward(state, child_, child_idx, cd=cdobs) + child_.q_value
                    value_uk.append(expected_cost_uk)

                current_value = min(value_uk)
                bellman_error += np.linalg.norm(state.q_value - current_value)
                state.q_value = min(value_uk)
            # end if
        # end for
        bellman_error_list.append(bellman_error)
        print("{}th iteration: {}".format(itr_num, bellman_error))
        # end while

    """
    plt.figure()
    x_axis = range(len(bellman_error_list))
    plt.plot(x_axis, bellman_error_list)
    plt.show()
    """


def init_Q_value():
    V_map = deepcopy(flow_env).astype(float)
    # end v = 0
    for point in [END_P]:
        V_map[point[0], point[1]] = 0

    # sta v = 0
    for point in [STA_P]:
        V_map[point[0], point[1]] = 0

    # Occupied v
    V_map = V_map * 999

    # [row_total, col_total] = V_map.shape

    # Heuristic
    goal_x = END_P[0]
    goal_y = END_P[1]
    dis = 0
    for px in range(0, grid_line):
        for py in range(0, grid_line):
            if V_map[px, py] == 0:
                # dx = abs(px - goal_x)
                # dy = abs(py - goal_y)

                # Euclidean distance Heuristic
                # dis = (dx ** 2 + dy ** 2) ** 0.5
                dis = np.hypot(px - goal_x, py - goal_y)

                # similar to Diagonal Heuristic
                # dis = math.ceil((dx + dy - 2 * min(dx, dy)) / 4) + math.ceil(min(dx, dy) / 4)
            V_map[px, py] = 1 * dis  # to encourage the racing car to move forward

            # this won't break the admissibility as the obstacle exists

    print("Initial V map!")
    return V_map


def V_to_graph(graph, V_map):
    for key in graph:
        px = graph[key].px
        py = graph[key].py
        vx = graph[key].vx
        vy = graph[key].vy

        # give penalty to those cause accidents except final line
        if (px in [END_P[0]]) and (py in [END_P[1]]):
            continue

        graph[key].q_value = V_map[px][py]

        bound_x_0 = 0
        bound_x_1 = grid_line
        bound_y_0 = 0
        bound_y_1 = grid_line

        # unavoidable outbound

        if abs(vx) == 2:
            if abs(px - bound_x_0) <= 1 or abs(px - bound_x_1) <= 1:
                graph[key].q_value = 999
        elif abs(vx) == 1:
            if abs(px - bound_x_0) <= 0 or abs(px - bound_x_1) <= 0:
                graph[key].q_value = 999

        if abs(vy) == 2:
            if abs(py - bound_y_0) <= 1 or abs(py - bound_y_1) <= 1:
                graph[key].q_value = 999
        elif abs(vy) == 1:
            if abs(py - bound_y_0) <= 0 or abs(py - bound_y_1) <= 0:
                graph[key].q_value = 999

    print("V to graph!")
    return graph


def dobs_update(d):
    d1 = d[0]
    d2 = d[1]

    # d1[0] += 1
    # d1[1] -= 1
    # d2[0] -= 1
    # d2[1] -= 1

    dh = deepcopy(d)
    dobs_history.append(dh)


def static_solve():
    # solve

    # dynamic_programming()
    # RTDP(greedy_prob=0.8)
    RT_dynamic_programming(epsilon=0.2)

    plan = track_the_best_plan()
    visualize_the_best_plan(plan, env_map)


def dynamic_solve():
    # d_solve

    dt_trajectory = {}
    RT_dynamic_programming(epsilon=0.2)
    plan = track_the_best_plan()
    dt_trajectory = [plan[0], plan[1]]
    # dobs_update(dobs)

    while not plan[1].is_goal:
        dobs_update(dobs)

        start_state = plan[1]
        RT_dynamic_programming(start_state.px, start_state.py, start_state.vx, start_state.vy)
        plan = track_the_best_plan(start_state.px, start_state.py, start_state.vx, start_state.vy)

        dt_trajectory.append(plan[1])
        # visualize_the_best_plan(dt_trajectory, env_map)

    dobs_update(dobs)
    visualize_the_best_plan(dt_trajectory, env_map)


if __name__ == '__main__':
    path = './solution/graph_dp.dat'
    env_map = flow_env
    graph = pickle.load(open(path, 'rb'))

    V_map = init_Q_value()  # print(V_map)
    graph = V_to_graph(graph, V_map)

    d = deepcopy(dobs)
    dobs_history = [d]

    # static_solve()
    dynamic_solve()
