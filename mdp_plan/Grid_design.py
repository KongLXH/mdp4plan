def dim_change1_2(s):
    pos_x = s % grid_line
    pos_y = s // grid_line
    return pos_x, pos_y


def dim_change2_1(pos_x, pos_y):
    s = pos_x + pos_y * grid_line
    return s


grid_line = 31  # map_size
grid_total = grid_line * grid_line  # num_grids

start = dim_change2_1(0, 0)  # 起点设置
goal = dim_change2_1(30, 30)  # 目标设置
