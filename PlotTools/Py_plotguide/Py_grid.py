import matplotlib.pyplot as plt
import numpy as np
import matplotlib.patches as mpatches
from matplotlib.collections import PatchCollection

fig, ax = plt.subplots(figsize=(0.5, 0.5))

patches = []

fancybox = mpatches.FancyBboxPatch([0.05, 0.05], 0.1, 0.1, boxstyle=mpatches.BoxStyle("Round", pad=1))
patches.append(fancybox)

colors = np.linspace(0, 1, len(patches))
collection = PatchCollection(patches, cmap=plt.cm.hsv, alpha=0.5)
collection.set_array(colors)
ax.add_collection(collection)

plt.axis('equal')
plt.axis('off')
plt.tight_layout()

plt.show()
