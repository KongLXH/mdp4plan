import numpy as np

grid_line = 200

STA_P = [0, 0]
END_P = [199, 199]
ACTION_SPACE = [[1, 1], [0, 1], [1, 0], [0, 0], [-1, 0], [0, -1], [1, -1], [-1, 1], [-1, -1]]
action_assert_list = [-1, 0, 1]
FINISH = 3
START = 2
FREE = 0
OCCUPIED = 1
OUTBOUND = -1

flow_env = np.zeros((grid_line, grid_line), dtype=np.int32)
flow_env[STA_P[0], STA_P[1]] = 2
flow_env[END_P[0], END_P[1]] = 3
# print(flow_env)

max_vel = 2

dobs_1 = [15, 33]
dobs_2 = [185, 190]
dobs = [dobs_1, dobs_2]
