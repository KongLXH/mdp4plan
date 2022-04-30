import matplotlib.patches as mpatches
import matplotlib.pyplot as plt

x_tail = 0
y_tail = 0
x_head = -0.15
y_head = 0.3
dx = x_head - x_tail
dy = y_head - y_tail

fig, axs = plt.subplots(figsize=(0.5, 0.5))

arrow = mpatches.FancyArrowPatch((x_tail, y_tail), (x_head, y_head),
                                 mutation_scale=10, color=[50/255, 118/255, 98/255])
axs.add_patch(arrow)

plt.axis('equal')
plt.axis('off')
plt.tight_layout()

plt.show()