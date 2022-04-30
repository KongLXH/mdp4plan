import sys
import pygame
import utils
from Grid_design import *

pygame.init()

ds_actions = {"n": -grid_line, "ne": -(grid_line - 1), "neb": -(2 * grid_line - 1), "nef": -(grid_line - 2),
              "e": 1, "es": grid_line + 1, "esb": grid_line + 2, "esf": 2 * grid_line + 1,
              "s": grid_line, "sw": grid_line - 1, "swb": 2 * grid_line - 1, "swf": grid_line - 2,
              "w": -1, "wn": -(grid_line + 1), "wnb": -(grid_line + 2), "wnf": -(2 * grid_line + 1)}  # 行为对状态的改变

# 窗口设置
height, width = 40 * grid_line, 40 * grid_line
gsize = width / grid_line

# 主屏幕设置
screen = pygame.display.set_mode((width, height))
pygame.display.set_caption('MDP-Planer')
screen.fill('white')

# 分割栅格
rect = [0] * grid_total
for i in range(grid_line):
    for j in range(grid_line):
        rect[j + i * grid_line] = pygame.draw.rect(screen, (255, 255, 255), ((j * gsize, i * gsize), (gsize, gsize)),
                                                   width=1)

# 读取图像
n_img = pygame.image.load("../mdp_plan/img/n.png")
ne_img = pygame.image.load("../mdp_plan/img/ne.png")
neb_img = pygame.image.load("../mdp_plan/img/neb.png")
nef_img = pygame.image.load("../mdp_plan/img/nef.png")
e_img = pygame.image.load("../mdp_plan/img/e.png")
es_img = pygame.image.load("../mdp_plan/img/es.png")
esb_img = pygame.image.load("../mdp_plan/img/esb.png")
esf_img = pygame.image.load("../mdp_plan/img/esf.png")
s_img = pygame.image.load("../mdp_plan/img/s.png")
sw_img = pygame.image.load("../mdp_plan/img/sw.png")
swb_img = pygame.image.load("../mdp_plan/img/swb.png")
swf_img = pygame.image.load("../mdp_plan/img/swf.png")
w_img = pygame.image.load("../mdp_plan/img/w.png")
wn_img = pygame.image.load("../mdp_plan/img/wn.png")
wnb_img = pygame.image.load("../mdp_plan/img/wnb.png")
wnf_img = pygame.image.load("../mdp_plan/img/wnf.png")
b_img = pygame.image.load("../mdp_plan/img/b.png")
c_img = pygame.image.load("../mdp_plan/img/c.jpeg")
p_img = pygame.image.load("../mdp_plan/img/p.png")

# 读取策略
policy = utils.policy_read("policy.txt")
# print("get policy !")

# 图像填充
path_n = n_img.subsurface(rect[0])
path_ne = ne_img.subsurface(rect[0])
path_neb = neb_img.subsurface(rect[0])
path_nef = nef_img.subsurface(rect[0])
path_e = e_img.subsurface(rect[0])
path_es = es_img.subsurface(rect[0])
path_esb = esb_img.subsurface(rect[0])
path_esf = esf_img.subsurface(rect[0])
path_s = s_img.subsurface(rect[0])
path_sw = sw_img.subsurface(rect[0])
path_swb = swb_img.subsurface(rect[0])
path_swf = swf_img.subsurface(rect[0])
path_w = w_img.subsurface(rect[0])
path_wn = wn_img.subsurface(rect[0])
path_wnb = wnb_img.subsurface(rect[0])
path_wnf = wnf_img.subsurface(rect[0])
path_b = b_img.subsurface(rect[0])
path_c = c_img.subsurface(rect[0])
# path_p = p_img.subsurface(rect[0])


def show_policy(p):
    for _ in range(grid_total):
        if p[_] == 'n':
            screen.blit(path_n, rect[_])
        elif p[_] == 'ne':
            screen.blit(path_ne, rect[_])
        elif p[_] == 'neb':
            screen.blit(path_neb, rect[_])
        elif p[_] == 'nef':
            screen.blit(path_nef, rect[_])
        elif p[_] == 'e':
            screen.blit(path_e, rect[_])
        elif p[_] == 'es':
            screen.blit(path_es, rect[_])
        elif p[_] == 'esb':
            screen.blit(path_esb, rect[_])
        elif p[_] == 'esf':
            screen.blit(path_esf, rect[_])
        elif p[_] == 's':
            screen.blit(path_s, rect[_])
        elif p[_] == 'sw':
            screen.blit(path_sw, rect[_])
        elif p[_] == 'swb':
            screen.blit(path_swb, rect[_])
        elif p[_] == 'swf':
            screen.blit(path_swf, rect[_])
        elif p[_] == 'w':
            screen.blit(path_w, rect[_])
        elif p[_] == 'wn':
            screen.blit(path_wn, rect[_])
        elif p[_] == 'wnb':
            screen.blit(path_wnb, rect[_])
        elif p[_] == 'wnf':
            screen.blit(path_wnf, rect[_])
        elif p[_] == 'nnenebnefeesesbesfsswswbswfwwnwnbwnf':
            screen.blit(path_b, rect[_])
        elif p[_] == '-':
            screen.blit(path_c, rect[_])
        else:
            pass


'''def show_path(p):
    s = start
    for _ in range(grid_total):
        screen.blit(path_p, rect[s])
        s += ds_actions[p[s]]
        if p[s] == 'nnenebnefeesesbesfsswswbswfwwnwnbwnf':
            screen.blit(path_p, rect[s])
            break'''


show_policy(policy)
# show_path(policy)

while True:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            pygame.quit()
            sys.exit()

    pygame.display.flip()
