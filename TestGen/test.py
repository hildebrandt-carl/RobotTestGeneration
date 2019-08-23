import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

data = [1, 2, 3, 100]
indices = ["a", "b", "c", "d"]


fig, (ax1, ax2) = plt.subplots(2,1,sharex=True,figsize=(5,6))

ax1.spines['bottom'].set_visible(False)
ax1.tick_params(axis='x',which='both',bottom=False)
ax2.spines['top'].set_visible(False)

bottom_cut = 80
top_cut = 5

ax2.set_ylim(0, top_cut)
ax1.set_ylim(bottom_cut, 120)

ax1.set_yticks(np.arange(80, 121, 10))
bars1 = ax1.bar(indices, data)
bars2 = ax2.bar(indices, data)

for tick in ax2.get_xticklabels():
    tick.set_rotation(0)

d = .015
kwargs = dict(transform=ax1.transAxes, color='k', clip_on=False)
ax1.plot((-d, +d), (-d, +d), **kwargs)
ax1.plot((1 - d, 1 + d), (-d, +d), **kwargs)
kwargs.update(transform=ax2.transAxes)
ax2.plot((-d, +d), (1 - d, 1 + d), **kwargs)
ax2.plot((1 - d, 1 + d), (1 - d, 1 + d), **kwargs)

for b1, b2 in zip(bars1, bars2):
    posx = b2.get_x() + b2.get_width()/2.
    if b2.get_height() > top_cut:
        ax2.plot((posx-3*d, posx+3*d), (1 - d, 1 + d), color='k', clip_on=False,
                 transform=ax2.get_xaxis_transform())
    if b1.get_height() > bottom_cut:
        ax1.plot((posx-3*d, posx+3*d), (- d, + d), color='k', clip_on=False,
                 transform=ax1.get_xaxis_transform())


plt.show()