import matplotlib.pyplot as plt
import numpy as np
import matplotlib.patches as mpatches
from matplotlib.collections import PatchCollection

fig, ax = plt.subplots(figsize=(0.5, 0.5))

patches = []

# add an arrow
arrow = mpatches.Arrow(0.0, 0.0, 0.0, 0.1, width=0.06)
patches.append(arrow)

colors = np.linspace(0, 1, len(patches))
collection = PatchCollection(patches, cmap=plt.cm.hsv, alpha=0.3)
collection.set_array(colors)
ax.add_collection(collection)


plt.axis('equal')
plt.axis('off')
plt.tight_layout()

plt.show()