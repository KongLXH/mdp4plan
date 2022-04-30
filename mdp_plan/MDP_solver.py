import utils
import Risk_maker
from Grid_design import *
import Flow_generator
import time


def sobs_index(f):  # 障碍物识别
    o = []
    for i in range(grid_total):
        if float(f[i]) < -99:  # v(obs) = -9999
            o.append(i)
    return o


def dobs_update(sx, sy):
    nx = sx + 1
    ny = sy
    return nx, ny


def dobs_index(sdx, sdy, o):
    os = dim_change2_1(sdx, sdy)

    onx, ony = dobs_update(sdx, sdy)
    on = dim_change2_1(onx, ony)

    if (os in o) or (sdx > grid_line - 1) or (sdy > grid_line - 1) or (sdx < 0) or (sdy < 0):
        o = []
    elif (onx > grid_line - 1) or (ony > grid_line - 1) or (onx < 0) or (ony < 0) or (on in o):
        o = [os]
    else:
        o = [os, on]
    return o


def bd_check(s, a):  # 检查边界
    pos_x, pos_y = dim_change1_2(s)

    bd_flag = False

    if (pos_x == 0 and (a in ["wn", "w", "sw", "swf", "wnb", "swb", "wnf"])) or \
            (pos_y == 0 and (a in ["sw", "s", "es", "esf", "swb", "esb", "swf"])) or \
            (pos_y == grid_line - 1 and (a in ["wn", "n", "ne", "wnf", "neb", "wnb", "nef"])) or \
            (pos_x == grid_line - 1 and (a in ["ne", "e", "es", "nef", "esb", "neb", "esf"])) or \
            (pos_x == 1 and (a in ["swf", "wnb"])) or \
            (pos_y == 1 and (a in ["esf", "swb"])) or \
            (pos_x == grid_line - 2 and (a in ["nef", "esb"])) or \
            (pos_y == grid_line - 2 and (a in ["wnf", "neb"])) or \
            s == goal:
        bd_flag = True  # 边界检查

    return bd_flag


def obs_check(s):  # 检查障碍物 TODO: 跨网格移动时未检测障碍物

    obs_flag = False

    if s in obs:
        obs_flag = True  # 障碍物检查

    return obs_flag


def dynamics(s, a):  # 环境动力学
    s_next = s

    wall_flag = bool(bd_check(s, a) or obs_check(s))  # 检测边界与障碍物

    if wall_flag:
        pass
    else:
        ds = ds_actions[a]
        s_next = s + ds

    '''is_end = True if s == goal else False'''  # 终点判断

    return s_next


def compute_pc_reward(s, a):  # 计算路程代价 (kd) TODO 启发式搜索
    pc_reward = 0.0
    kd = 0.5

    if a in ["n", "e", "s", "w"]:
        pc_reward = -1.0 * kd
    elif a in ["ne", "es", "sw", "wn"]:
        pc_reward = -q2 * kd
    elif a in ["neb", "nef", "esb", "esf", "swb", "swf", "wnb", "wnf"]:
        pc_reward = -q5 * kd

    '''if s == goal:
        pc_reward = 0'''

    return pc_reward


def compute_auv_speed(s, a):
    act_speed = 2.0

    act_vector = utils.char_trans(a, act_speed)
    flow_vector = [float(flow_x[s]), float(flow_y[s])]

    auv_vector_x = act_vector[0] - flow_vector[0]
    auv_vector_y = act_vector[1] - flow_vector[1]

    auv_speed = pow((pow(auv_vector_x, 2) + pow(auv_vector_y, 2)), 0.5)

    return auv_speed


def compute_flow_reward(s, a):  # 计算对流代价 (kf)
    flow_reward = 0.0
    kf = 1.0

    sp = dynamics(s, a)
    auv_speed_s = compute_auv_speed(s, a)
    auv_speed_sp = compute_auv_speed(sp, a)

    if a in ["n", "e", "s", "w"]:
        flow_reward = -(0.5 * pow(auv_speed_s, 3) + 0.5 * pow(auv_speed_sp, 3)) * kf
    elif a in ["ne", "es", "sw", "wn"]:
        flow_reward = -(q2 / 2 * pow(auv_speed_s, 3) + q2 / 2 * pow(auv_speed_sp, 3)) * kf
    elif a in ["neb"]:

        # TODO 可以尝试优化重复代码
        """if a[-1] == "b":
            sp1 = dynamics(s, a[0])
            sp2 = dynamics(s, a[0] + a[1])
        else:
            sp1 = dynamics(s, a[0] + a[1])
            sp2 = dynamics(s, a[1])"""

        sp1 = dynamics(s, "n")
        sp2 = dynamics(s, "ne")
        auv_speed_sp1 = compute_auv_speed(sp1, a)
        auv_speed_sp2 = compute_auv_speed(sp2, a)
        flow_reward = -(q5 / 4 * pow(auv_speed_s, 3) + q5 / 4 * pow(auv_speed_sp, 3)
                        + q5 / 4 * pow(auv_speed_sp1, 3) + q5 / 4 * pow(auv_speed_sp2, 3)) * kf
    elif a in ["nef"]:
        sp1 = dynamics(s, "ne")
        sp2 = dynamics(s, "e")
        auv_speed_sp1 = compute_auv_speed(sp1, a)
        auv_speed_sp2 = compute_auv_speed(sp2, a)
        flow_reward = -(q5 / 4 * pow(auv_speed_s, 3) + q5 / 4 * pow(auv_speed_sp, 3)
                        + q5 / 4 * pow(auv_speed_sp1, 3) + q5 / 4 * pow(auv_speed_sp2, 3)) * kf
    elif a in ["esb"]:
        sp1 = dynamics(s, "e")
        sp2 = dynamics(s, "es")
        auv_speed_sp1 = compute_auv_speed(sp1, a)
        auv_speed_sp2 = compute_auv_speed(sp2, a)
        flow_reward = -(q5 / 4 * pow(auv_speed_s, 3) + q5 / 4 * pow(auv_speed_sp, 3)
                        + q5 / 4 * pow(auv_speed_sp1, 3) + q5 / 4 * pow(auv_speed_sp2, 3)) * kf
    elif a in ["esf"]:
        sp1 = dynamics(s, "es")
        sp2 = dynamics(s, "s")
        auv_speed_sp1 = compute_auv_speed(sp1, a)
        auv_speed_sp2 = compute_auv_speed(sp2, a)
        flow_reward = -(q5 / 4 * pow(auv_speed_s, 3) + q5 / 4 * pow(auv_speed_sp, 3)
                        + q5 / 4 * pow(auv_speed_sp1, 3) + q5 / 4 * pow(auv_speed_sp2, 3)) * kf
    elif a in ["swb"]:
        sp1 = dynamics(s, "s")
        sp2 = dynamics(s, "sw")
        auv_speed_sp1 = compute_auv_speed(sp1, a)
        auv_speed_sp2 = compute_auv_speed(sp2, a)
        flow_reward = -(q5 / 4 * pow(auv_speed_s, 3) + q5 / 4 * pow(auv_speed_sp, 3)
                        + q5 / 4 * pow(auv_speed_sp1, 3) + q5 / 4 * pow(auv_speed_sp2, 3)) * kf
    elif a in ["swf"]:
        sp1 = dynamics(s, "sw")
        sp2 = dynamics(s, "w")
        auv_speed_sp1 = compute_auv_speed(sp1, a)
        auv_speed_sp2 = compute_auv_speed(sp2, a)
        flow_reward = -(q5 / 4 * pow(auv_speed_s, 3) + q5 / 4 * pow(auv_speed_sp, 3)
                        + q5 / 4 * pow(auv_speed_sp1, 3) + q5 / 4 * pow(auv_speed_sp2, 3)) * kf
    elif a in ["wnb"]:
        sp1 = dynamics(s, "w")
        sp2 = dynamics(s, "wn")
        auv_speed_sp1 = compute_auv_speed(sp1, a)
        auv_speed_sp2 = compute_auv_speed(sp2, a)
        flow_reward = -(q5 / 4 * pow(auv_speed_s, 3) + q5 / 4 * pow(auv_speed_sp, 3)
                        + q5 / 4 * pow(auv_speed_sp1, 3) + q5 / 4 * pow(auv_speed_sp2, 3)) * kf
    elif a in ["wnf"]:
        sp1 = dynamics(s, "wn")
        sp2 = dynamics(s, "n")
        auv_speed_sp1 = compute_auv_speed(sp1, a)
        auv_speed_sp2 = compute_auv_speed(sp2, a)
        flow_reward = -(q5 / 4 * pow(auv_speed_s, 3) + q5 / 4 * pow(auv_speed_sp, 3)
                        + q5 / 4 * pow(auv_speed_sp1, 3) + q5 / 4 * pow(auv_speed_sp2, 3)) * kf

    '''if s == goal:
        flow_reward = 0'''

    return flow_reward


def compute_risk_reward(s, a):  # 计算危险代价 (kr) # TODO 计算路程风险累计
    # risk_reward = 0.0
    sp = dynamics(s, a)

    kr = - 1.0
    risk_reward = kr * (float(risk_m[sp]))

    '''if a in ["n", "e", "s", "w", "ne", "es", "sw", "wn"]:
        risk_reward = kr * (float(risk_m[s]) + float(risk_m[sp]))
    elif a in ["neb"]:

        # TODO 可以尝试优化重复代码
        """if a[-1] == "b":
            sp1 = dynamics(s, a[0])
            sp2 = dynamics(s, a[0] + a[1])
        else:
            sp1 = dynamics(s, a[0] + a[1])
            sp2 = dynamics(s, a[1])"""

        sp1 = dynamics(s, "n")
        sp2 = dynamics(s, "ne")
        # risk_reward = kr * (float(risk_m[s]) + float(risk_m[sp]) + 1/2*(float(risk_m[sp1]) + float(risk_m[sp2])))
        risk_reward = kr * (float(risk_m[s]) + float(risk_m[sp]) + float(risk_m[sp1]) + float(risk_m[sp2]))

    elif a in ["nef"]:
        sp1 = dynamics(s, "ne")
        sp2 = dynamics(s, "e")
        risk_reward = kr * (float(risk_m[s]) + float(risk_m[sp]) + float(risk_m[sp1]) + float(risk_m[sp2]))

    elif a in ["esb"]:
        sp1 = dynamics(s, "e")
        sp2 = dynamics(s, "es")
        risk_reward = kr * (float(risk_m[s]) + float(risk_m[sp]) + float(risk_m[sp1]) + float(risk_m[sp2]))

    elif a in ["esf"]:
        sp1 = dynamics(s, "es")
        sp2 = dynamics(s, "s")
        risk_reward = kr * (float(risk_m[s]) + float(risk_m[sp]) + float(risk_m[sp1]) + float(risk_m[sp2]))

    elif a in ["swb"]:
        sp1 = dynamics(s, "s")
        sp2 = dynamics(s, "sw")
        risk_reward = kr * (float(risk_m[s]) + float(risk_m[sp]) + float(risk_m[sp1]) + float(risk_m[sp2]))

    elif a in ["swf"]:
        sp1 = dynamics(s, "sw")
        sp2 = dynamics(s, "w")
        risk_reward = kr * (float(risk_m[s]) + float(risk_m[sp]) + float(risk_m[sp1]) + float(risk_m[sp2]))

    elif a in ["wnb"]:
        sp1 = dynamics(s, "w")
        sp2 = dynamics(s, "wn")
        risk_reward = kr * (float(risk_m[s]) + float(risk_m[sp]) + float(risk_m[sp1]) + float(risk_m[sp2]))

    elif a in ["wnf"]:
        sp1 = dynamics(s, "wn")
        sp2 = dynamics(s, "n")
        risk_reward = kr * (float(risk_m[s]) + float(risk_m[sp]) + float(risk_m[sp1]) + float(risk_m[sp2]))'''

    # risk_reward = 0

    return risk_reward


def get_prob(s, a, s1):  # 状态转移概率函数
    # case 2: partially uncertain TODO 需要计算ROMS图的不确定性
    """def get_action(d1, d):
        for a1, ds in ds_actions.items():
            if d1-d == ds:
                return a1

    a1 = get_action(s1, s)

    na = utils.char_index(a)
    na1 = utils.char_index(a1)

    prob = prob_matrix[s, na, na1]

    return prob"""

    # case 1: fully deterministic
    s_next = dynamics(s, a)
    return s1 == s_next


def get_reward(s, a):  # 奖励函数
    if s == dynamics(s, a):
        """碰撞成本"""
        reward = -99999
    else:
        reward = compute_pc_reward(s, a) + compute_flow_reward(s, a) + compute_risk_reward(s, a)

    if s == goal:
        reward = 0

    return reward


def set_value(V, s, v):  # 设置价值字典
    V[s] = v


def get_value(V, s):  # 获取状态价值
    return V[s]


def compute_q(V, s, a):
    """根据给定的MDP，价值函数V，计算状态行为对s,a的价值qsa"""
    q_sa = 0
    fr = []

    # feasible_region
    '''for c in A:
        sp = dynamics(s, c)
        if sp != s:
            fr.append(sp)
        else:
            pass'''

    sp = dynamics(s, a)
    fr.append(sp)

    for s_next in fr:  # TODO P为0时跳过计算步骤
        q_sa += get_prob(s, a, s_next) * get_value(V, s_next)
    q_sa = get_reward(s, a) + gamma * q_sa
    return q_sa


def compute_v_from_max_q(V, s):
    """根据一个状态的下所有可能的行为价值中最大一个来确定当前状态价值"""
    v_s = -float('inf')
    for a in A:
        qsa = compute_q(V, s, a)
        if qsa >= v_s:
            v_s = qsa
    return v_s


def update_V_without_pi(V):
    """在不依赖策略的情况下直接通过后续状态的价值来更新状态价值"""
    V_next = V.copy()
    for s in S:
        set_value(V_next, s, compute_v_from_max_q(V_next, s))
    return V_next


def value_iterate(V, n):
    """价值迭代"""
    for i in range(n):
        V = update_V_without_pi(V)
    return V


def iteration_over(V, V_):
    is_over = True
    for i in range(grid_total):
        if not (obs_check(i)):
            if abs(V_[i] - V[i]) > 0.001:  # TODO 收敛速度计算
                is_over = False
                print("not over ...")
                break
    return is_over


def greedy_policy(V, s):
    max_pi, a_max_pi = -float('inf'), []

    obs_flag = obs_check(s)

    if obs_flag:
        a_max_pi = '-'
    else:
        for a_opt in A:
            """统计后续状态的最大价值以及到达到达该状态的行为（可能不止一个)"""
            s_next = dynamics(s, a_opt)
            pi_s_next = get_value(V, s_next) * get_prob(s, a_opt, s_next) + get_reward(s, a_opt)
            """结合贪婪策略公式理解"""
            if pi_s_next > max_pi:
                max_pi = pi_s_next
                a_max_pi = a_opt
            elif pi_s_next == max_pi:
                a_max_pi += a_opt
    return str(a_max_pi)


def collect_policy(policy, V, n):
    total_p = []
    for i in range(grid_total):
        total_p.append(policy(V, S[i]))
    utils.policy_save(total_p, f"policy{n}.txt")
    print(f"{n}: find policy !")


def time_test(ini_V):
    """时间测试"""
    tic = time.time()
    value_iterate(ini_V, 200)
    toc = time.time()
    print("Time spend {:.3f}s".format(toc - tic))


def no_auto_end_iter(ini_V):
    """无终止条件迭代"""
    V_star = value_iterate(ini_V, 200)
    collect_policy(greedy_policy, V_star, 0)


def auto_end_iter(ini_V):
    """有终止条件迭代"""
    V_star = value_iterate(ini_V, 30)
    print("iteration 1")
    V_star_ = update_V_without_pi(V_star)
    n = 1

    while not (iteration_over(V_star, V_star_)):
        n += 1
        V_star = value_iterate(V_star_, 10)
        print("iteration", n)
        V_star_ = update_V_without_pi(V_star)
    print("iteration over !")

    # 最佳策略采集
    collect_policy(greedy_policy, V_star, 0)


def real_time_DP():
    # TODO RTDP
    pass


def dynamic_obs():  # TODO 动态避障
    flow_x, flow_y = [0 for _ in range(grid_total)], [0 for _ in range(grid_total)]

    for i in range(100):
        V = [0 for _ in range(grid_total)]


def dynamic_flow_obs():
    """时变流场策略采集"""
    for i in range(73):  # TODO 应该设置终止条件

        V = [0 for _ in range(grid_total)]

        flow_x, flow_y, sobs = roms_flow(f"flow_x{i}.txt", f"flow_y{i}.txt")

        # 动态障碍物信息
        dx, dy = 3, 8

        dobs = dobs_index(dx, dy, sobs)
        dx, dy = dobs_update(dx, dy)

        obs = sobs + dobs
        risk_m = Risk_maker.get_risk_m(obs)  # TODO 重置risk

        V_star = value_iterate(V, 30)
        print(f"{i}: iteration 1")
        V_star_ = update_V_without_pi(V_star)
        n = 1

        while not (iteration_over(V_star, V_star_)):
            n += 1
            V_star = value_iterate(V_star_, 10)
            print(f"{i}: iteration", n)
            V_star_ = update_V_without_pi(V_star)

        print(f"{i}: iteration over !")

        collect_policy(greedy_policy, V_star, i)


def sim_flow(fx, fy):
    """模拟流场"""
    Flow_generator.double_vortex(1.8)
    x_flow = utils.flow_read_txt(fx)
    y_flow = utils.flow_read_txt(fy)
    obst = []

    for pos_x in [7, 8, 9]:
        for pos_y in [20, 21, 22]:
            obst.append(dim_change2_1(pos_x, pos_y))

    for pos_x in [12, 13, 14]:
        for pos_y in [15, 16, 17]:
            obst.append(dim_change2_1(pos_x, pos_y))

    return x_flow, y_flow, obst


def roms_flow(fx, fy):
    """roms流场"""
    x_flow = utils.flow_read_txt(fx)
    y_flow = utils.flow_read_txt(fy)
    obst = sobs_index(x_flow)

    return x_flow, y_flow, obst


if __name__ == '__main__':
    S = [i for i in range(grid_total)]  # 状态空间

    # TODO 可尝试修改动作集，使各动作决策用时一致(单位时间间隔内到达指定位置)
    A = ["n", "ne", "neb", "nef",
         "e", "es", "esb", "esf",
         "s", "sw", "swb", "swf",
         "w", "wn", "wnb", "wnf"]  # 行为空间

    ds_actions = {"n": grid_line, "ne": grid_line + 1, "neb": 2 * grid_line + 1, "nef": grid_line + 2,
                  "e": 1, "es": -grid_line + 1, "esb": -grid_line + 2, "esf": -2 * grid_line + 1,
                  "s": -grid_line, "sw": -grid_line - 1, "swb": -2 * grid_line - 1, "swf": -grid_line - 2,
                  "w": -1, "wn": grid_line - 1, "wnb": grid_line - 2, "wnf": 2 * grid_line - 1}  # 行为对状态的改变

    gamma = 1

    # MDP = S, A, P, R, gamma

    # flow_x, flow_y, obs = [0 for _ in range(grid_total)], [0 for _ in range(grid_total)], []
    flow_x, flow_y, obs = sim_flow("flow_x.txt", "flow_y.txt")
    # flow_x, flow_y, obs = roms_flow("flow_nx.txt", "flow_ny.txt")

    # 状态转移概率矩阵
    # prob_matrix = utils.prob_read("roms_31_t6_5s.npy")

    # 风险代价地图
    # risk_m = utils.risk_read_txt("risk.txt")
    risk_m = Risk_maker.get_risk_m(obs)
    # Risk_maker.draw_heatmap(obs)

    # magic num
    q1 = 1
    q2 = pow(2, 0.5)
    q5 = pow(5, 0.5)

    V = [0 for _ in range(grid_total)]  # 重置状态价值

    auto_end_iter(V)
