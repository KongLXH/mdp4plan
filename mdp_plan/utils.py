import os
import numpy as np
from Grid_design import *


def char_index(c):  # 策略转为编号

    c_index = -1

    if c == "n":
        c_index = 0
    elif c == "ne":
        c_index = 1
    elif c == "neb":
        c_index = 2
    elif c == "nef":
        c_index = 3
    elif c == "e":
        c_index = 4
    elif c == "es":
        c_index = 5
    elif c == "esb":
        c_index = 6
    elif c == "esf":
        c_index = 7
    elif c == "s":
        c_index = 8
    elif c == "sw":
        c_index = 9
    elif c == "swb":
        c_index = 10
    elif c == "swf":
        c_index = 11
    elif c == "w":
        c_index = 12
    elif c == "wn":
        c_index = 13
    elif c == "wnb":
        c_index = 14
    elif c == "wnf":
        c_index = 15

    return c_index


def char_trans(c, n):  # 策略转为向量
    policy_vector = [0.00, 0.00]

    if c == "n":
        policy_vector = [0.000, 1.000]
    elif c == "ne":
        policy_vector = [0.705, 0.705]
    elif c == "neb":
        policy_vector = [0.447, 0.894]
    elif c == "nef":
        policy_vector = [0.894, 0.447]
    elif c == "e":
        policy_vector = [1.000, 0.000]
    elif c == "es":
        policy_vector = [0.705, -0.705]
    elif c == "esb":
        policy_vector = [0.894, -0.447]
    elif c == "esf":
        policy_vector = [0.447, -0.894]
    elif c == "s":
        policy_vector = [0.000, -1.000]
    elif c == "sw":
        policy_vector = [-0.705, -0.705]
    elif c == "swb":
        policy_vector = [-0.447, -0.894]
    elif c == "swf":
        policy_vector = [-0.894, -0.447]
    elif c == "w":
        policy_vector = [-1.000, 0.000]
    elif c == "wn":
        policy_vector = [-0.705, 0.705]
    elif c == "wnb":
        policy_vector = [-0.894, 0.447]
    elif c == "wnf":
        policy_vector = [-0.447, 0.894]

    policy_vector[0] *= n
    policy_vector[1] *= n

    return policy_vector


def display_V(V):  # 显示状态价值
    for i in range(grid_total):
        print('{0:>8.3f}'.format(V[i]), end=" ")
        if (i + 1) % grid_line == 0:
            print("")
    print()


def policy_save(content, filename, mode='a'):  # 保存策略

    if os.path.isfile("../mdp_plan/policy/"+filename):
        os.remove("../mdp_plan/policy/"+filename)
        print("remaking policy ...")
    else:
        pass

    policy_dir = '../mdp_plan/policy/'

    file = open(policy_dir + filename, mode)
    for i in range(len(content)):
        file.write(str(content[i]) + '\n')
    file.close()


def policy_read(filename):  # 读取策略

    policy_dir = '../mdp_plan/policy/'

    try:
        file = open(policy_dir + filename, 'r')
    except IOError:
        error = []
        return error
    content = file.readlines()

    for i in range(len(content)):
        content[i] = content[i][:len(content[i]) - 1]

    file.close()
    return content


def flow_save(content, filename, mode='a'):  # 保存流场

    if os.path.isfile("../mdp_plan/flow/" + filename):
        os.remove("../mdp_plan/flow/" + filename)
        print("delete " + filename + " ...")
    else:
        pass

    flow_dir = '../mdp_plan/flow/'

    '''file = open(flow_dir + filename, mode)
    for i in range(len(content)):
        file.write(str(content[i]) + '\n')
    file.close()'''

    np.savetxt(flow_dir + filename, content)


def flow_read(filename):  # 读取流场

    flow_dir = '../mdp_plan/flow/'

    try:
        file = open(flow_dir + filename, 'r')
    except IOError:
        error = []
        print("io_error")
        return error

    '''content = file.readlines()

    for i in range(len(content)):
        content[i] = content[i][:len(content[i]) - 1]'''

    content = np.load(flow_dir + filename)
    print("read " + filename)

    # file.close()
    return content


def flow_read_txt(filename):  # 读取流场

    flow_dir = '../mdp_plan/flow/'

    try:
        file = open(flow_dir + filename, 'r')
    except IOError:
        error = []
        print("io_error")
        return error

    content = file.readlines()

    for i in range(len(content)):
        content[i] = content[i][:len(content[i]) - 1]

    # content = np.load(flow_dir + filename)
    print("read " + filename)

    file.close()
    return content


def prob_save(content, filename, mode='a'):  # 保存概率阵

    if os.path.isfile("../mdp_plan/prob/" + filename):
        os.remove("../mdp_plan/prob/" + filename)
        print("delete " + filename + " ...")
    else:
        pass

    prob_dir = '../mdp_plan/prob/'

    '''file = open(prob_dir + filename, mode)
    for i in range(len(content)):
        file.write(str(content[i]) + '\n')
    file.close()'''

    np.save(prob_dir + filename, content)


def prob_read(filename):  # 读取概率阵

    prob_dir = '../mdp_plan/prob/'

    try:
        file = open(prob_dir + filename, 'r')
    except IOError:
        error = []
        return error

    '''content = file.readlines()

    for i in range(len(content)):
        content[i] = content[i][:len(content[i]) - 1]'''

    content = np.load(prob_dir + filename)
    print("read " + filename)

    # file.close()
    return content


def risk_save(content, filename, mode='a'):  # 保存流场

    if os.path.isfile("../mdp_plan/risk/" + filename):
        os.remove("../mdp_plan/risk/" + filename)
        print("delete " + filename + " ...")
    else:
        pass

    risk_dir = '../mdp_plan/risk/'

    '''file = open(risk_dir + filename, mode)
    for i in range(len(content)):
        file.write(str(content[i]) + '\n')
    file.close()'''

    np.savetxt(risk_dir + filename, content)
    print("save " + filename)


def risk_read_txt(filename):  # 读取流场

    risk_dir = '../mdp_plan/risk/'

    try:
        file = open(risk_dir + filename, 'r')
    except IOError:
        error = []
        print("io_error")
        return error

    content = file.readlines()

    for i in range(len(content)):
        content[i] = content[i][:len(content[i]) - 1]

    # content = np.load(risk_dir + filename)
    print("read " + filename)

    file.close()
    return content
